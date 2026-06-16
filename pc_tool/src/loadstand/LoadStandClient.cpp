#include "loadstand/LoadStandClient.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <thread>

#ifdef _WIN32
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#endif

namespace drivescope {

LoadStandClient::~LoadStandClient()
{
    close();
}

bool LoadStandClient::open(const std::string& host, int port)
{
#ifndef _WIN32
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        status_ = "TCP client is implemented for Windows in this MVP";
    }
    return false;
#else
    close();

    WSADATA wsa{};
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::lock_guard<std::mutex> lock(dataMutex_);
        status_ = "WSAStartup failed";
        return false;
    }

    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* result = nullptr;
    const std::string portText = std::to_string(port);
    if (getaddrinfo(host.c_str(), portText.c_str(), &hints, &result) != 0 || result == nullptr) {
        std::lock_guard<std::mutex> lock(dataMutex_);
        status_ = "resolve failed: " + host;
        WSACleanup();
        return false;
    }

    SOCKET s = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (s == INVALID_SOCKET) {
        freeaddrinfo(result);
        std::lock_guard<std::mutex> lock(dataMutex_);
        status_ = "socket() failed";
        WSACleanup();
        return false;
    }

    // Non-blocking connect with a ~1.5 s timeout so a dead host doesn't hang
    // the UI thread; then back to blocking for the reader.
    u_long nb = 1;
    ioctlsocket(s, FIONBIO, &nb);
    connect(s, result->ai_addr, static_cast<int>(result->ai_addrlen));
    freeaddrinfo(result);

    fd_set wf;
    FD_ZERO(&wf);
    FD_SET(s, &wf);
    timeval tv{};
    tv.tv_sec = 1;
    tv.tv_usec = 500000;
    if (select(0, nullptr, &wf, nullptr, &tv) <= 0) {
        closesocket(s);
        std::lock_guard<std::mutex> lock(dataMutex_);
        status_ = "connect timeout: " + host + ":" + portText;
        WSACleanup();
        return false;
    }
    nb = 0;
    ioctlsocket(s, FIONBIO, &nb);

    // Short recv timeout so the reader loop can poll stop_ without blocking.
    DWORD rxTimeoutMs = 200;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&rxTimeoutMs),
               sizeof(rxTimeoutMs));

    sock_ = s;
    stop_.store(false);
    opened_.store(true);
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        status_ = "connected " + host + ":" + portText;
        t0ms_ = -1.0;
    }

    // Wake the console and start the DATA stream at 50 Hz.
    rawSend("\r");
    rawSend("stream on 50\r");

    reader_ = std::thread([this] { readerLoop(); });
    return true;
#endif
}

void LoadStandClient::close()
{
#ifdef _WIN32
    const bool was = opened_.exchange(false);
    stop_.store(true);
    if (was) {
        // best-effort: stop the stream before tearing down
        rawSend("stream off\r");
    }
    if (reader_.joinable()) reader_.join();
    {
        std::lock_guard<std::mutex> lock(ioMutex_);
        if (sock_ != INVALID_SOCKET) {
            closesocket(sock_);
            sock_ = INVALID_SOCKET;
            WSACleanup();
        }
    }
#endif
    std::lock_guard<std::mutex> lock(dataMutex_);
    rx_.clear();
    status_ = "closed";
}

std::string LoadStandClient::status()
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    return status_;
}

bool LoadStandClient::rawSend(const std::string& text)
{
#ifndef _WIN32
    (void) text;
    return false;
#else
    std::lock_guard<std::mutex> lock(ioMutex_);
    if (sock_ == INVALID_SOCKET) return false;
    size_t off = 0;
    while (off < text.size()) {
        int n = ::send(sock_, text.data() + off, static_cast<int>(text.size() - off), 0);
        if (n <= 0) return false;
        off += static_cast<size_t>(n);
    }
    return true;
#endif
}

bool LoadStandClient::sendLine(const std::string& line)
{
    if (!opened_.load()) return false;
    return rawSend(line + "\r");
}

void LoadStandClient::readerLoop()
{
#ifdef _WIN32
    char buf[4096];
    while (!stop_.load()) {
        int got = 0;
        {
            std::lock_guard<std::mutex> lock(ioMutex_);
            if (sock_ == INVALID_SOCKET) break;
            got = ::recv(sock_, buf, sizeof(buf), 0);
        }
        if (got > 0) {
            std::lock_guard<std::mutex> lock(dataMutex_);
            rx_.append(buf, buf + got);
            consumeRx();
        } else if (got == 0) {
            std::lock_guard<std::mutex> lock(dataMutex_);
            status_ = "peer closed";
            break;
        } else {
            // timeout (WSAEWOULDBLOCK/WSAETIMEDOUT) -> just poll stop_ again
            const int err = WSAGetLastError();
            if (err != WSAETIMEDOUT && err != WSAEWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }
#endif
}

void LoadStandClient::consumeRx()
{
    // dataMutex_ is held by the caller.
    for (;;) {
        const size_t posR = rx_.find('\r');
        const size_t posN = rx_.find('\n');
        size_t pos = std::string::npos;
        if (posR != std::string::npos && posN != std::string::npos) pos = std::min(posR, posN);
        else pos = (posR != std::string::npos) ? posR : posN;
        if (pos == std::string::npos) break;

        std::string line = rx_.substr(0, pos);
        rx_.erase(0, pos + 1);
        if (line.empty()) continue;

        long ms = 0, set_mA = 0, I_mA = 0, setM_mNm = 0, M_mNm = 0, duty = 0;
        int  dir = 0;
        int n = std::sscanf(line.c_str(), "DATA,%ld,%ld,%ld,%ld,%ld,%ld,%d",
                            &ms, &set_mA, &I_mA, &setM_mNm, &M_mNm, &duty, &dir);
        if (n >= 6) {
            pushSample(ms, set_mA, I_mA, setM_mNm, M_mNm, duty, (n >= 7) ? dir : 0);
        } else {
            buf_.lastText = line;          // status / OK / echo lines
        }
    }
}

void LoadStandClient::pushSample(long ms, long set_mA, long I_mA,
                                 long setM_mNm, long M_mNm, long duty, int dir)
{
    // dataMutex_ is held by the caller.
    if (t0ms_ < 0.0) t0ms_ = static_cast<double>(ms);
    double t = (static_cast<double>(ms) - t0ms_) / 1000.0;
    if (t < 0.0) t = 0.0;              // ignore tick wrap (49.7 days) gracefully

    buf_.t.push_back(t);
    buf_.torque.push_back(static_cast<double>(M_mNm) / 1000.0);
    buf_.setpoint.push_back(static_cast<double>(setM_mNm) / 1000.0);
    buf_.current.push_back(static_cast<double>(I_mA) / 1000.0);
    buf_.duty.push_back(static_cast<double>(duty));
    (void) set_mA;

    buf_.lastTorque   = static_cast<double>(M_mNm) / 1000.0;
    buf_.lastSetpoint = static_cast<double>(setM_mNm) / 1000.0;
    buf_.lastCurrent  = static_cast<double>(I_mA) / 1000.0;
    buf_.lastDuty     = static_cast<double>(duty);
    buf_.lastDir      = dir;
    buf_.samples++;

    if (buf_.t.size() > maxPoints) {
        const size_t drop = buf_.t.size() - maxPoints;
        buf_.t.erase(buf_.t.begin(), buf_.t.begin() + drop);
        buf_.torque.erase(buf_.torque.begin(), buf_.torque.begin() + drop);
        buf_.setpoint.erase(buf_.setpoint.begin(), buf_.setpoint.begin() + drop);
        buf_.current.erase(buf_.current.begin(), buf_.current.begin() + drop);
        buf_.duty.erase(buf_.duty.begin(), buf_.duty.begin() + drop);
    }
}

void LoadStandClient::snapshot(Snapshot& out)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    out = buf_;
}

void LoadStandClient::clearData()
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    buf_ = Snapshot{};
    t0ms_ = -1.0;
}

} // namespace drivescope
