#include "transport/TcpCanTransport.h"

#include "can/CanFrame.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <regex>
#include <sstream>
#include <vector>

#ifdef _WIN32
#include <ws2tcpip.h>
#include <iphlpapi.h>
#pragma comment(lib, "iphlpapi.lib")
#pragma comment(lib, "ws2_32.lib")
#endif

namespace drivescope {

namespace {
double steadyNow()
{
    static const auto start = std::chrono::steady_clock::now();
    using seconds_d = std::chrono::duration<double>;
    return seconds_d(std::chrono::steady_clock::now() - start).count();
}

bool jsonBool(const std::string& line, const char* key, bool fallback)
{
    const std::regex re(std::string("\"") + key + "\"\\s*:\\s*(true|false|0|1)");
    std::smatch m;
    if (!std::regex_search(line, m, re)) return fallback;
    return m[1] == "true" || m[1] == "1";
}

uint32_t jsonUint(const std::string& line, const char* key, uint32_t fallback)
{
    const std::regex re(std::string("\"") + key + "\"\\s*:\\s*(0x[0-9A-Fa-f]+|[0-9]+)");
    std::smatch m;
    if (!std::regex_search(line, m, re)) return fallback;
    return static_cast<uint32_t>(std::stoul(m[1].str(), nullptr, 0));
}

std::string jsonString(const std::string& line, const char* key)
{
    const std::regex re(std::string("\"") + key + "\"\\s*:\\s*\"([^\"]*)\"");
    std::smatch m;
    if (!std::regex_search(line, m, re)) return {};
    return m[1].str();
}
} // namespace

bool TcpCanTransport::open()
{
#ifndef _WIN32
    status_ = "TCP transport is implemented for Windows in this MVP";
    return false;
#else
    close();
    WSADATA wsa{};
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        status_ = "WSAStartup failed";
        return false;
    }

    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* result = nullptr;
    const std::string portText = std::to_string(port);
    if (getaddrinfo(host.c_str(), portText.c_str(), &hints, &result) != 0 || result == nullptr) {
        status_ = "resolve failed";
        WSACleanup();
        return false;
    }

    // Find local NIC whose IPv4 address shares an on-link prefix with the
    // target. We bind the connect-socket to that NIC, which forces egress
    // through it regardless of the kernel's "best route" pick. Without this,
    // a VPN that owns the default route (WireGuard with AllowedIPs=0.0.0.0/0,
    // Tailscale, corporate VPNs) silently steals the connect attempt and the
    // device on the LAN looks unreachable. Falls back to default routing
    // (no bind) when no NIC matches — that's the only useful default left.
    sockaddr_in target{};
    std::memcpy(&target, result->ai_addr, sizeof(target));
    sockaddr_in chosenLocal{};
    bool haveChosen = false;
    {
        ULONG bufSz = 0;
        GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST |
                             GAA_FLAG_SKIP_DNS_SERVER, nullptr, nullptr, &bufSz);
        std::vector<unsigned char> abuf(bufSz);
        auto* aa = reinterpret_cast<IP_ADAPTER_ADDRESSES*>(abuf.data());
        if (GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST |
                                 GAA_FLAG_SKIP_DNS_SERVER, nullptr, aa, &bufSz) == NO_ERROR) {
            ULONG bestMetric = ~0u;
            for (auto* ad = aa; ad; ad = ad->Next) {
                if (ad->OperStatus != IfOperStatusUp) continue;
                if (ad->IfType == IF_TYPE_SOFTWARE_LOOPBACK) continue;
                for (auto* ua = ad->FirstUnicastAddress; ua; ua = ua->Next) {
                    auto* sa = reinterpret_cast<sockaddr_in*>(ua->Address.lpSockaddr);
                    if (sa->sin_family != AF_INET) continue;
                    uint32_t local_be = sa->sin_addr.s_addr;
                    uint32_t target_be = target.sin_addr.s_addr;
                    uint32_t prefix = ua->OnLinkPrefixLength >= 32 ? 32u : ua->OnLinkPrefixLength;
                    if (prefix == 0) continue;
                    uint32_t mask_be = htonl(~((1u << (32 - prefix)) - 1));
                    if ((local_be & mask_be) != (target_be & mask_be)) continue;
                    // Skip 169.254/16 link-local.
                    uint32_t ip_h = ntohl(local_be);
                    if ((ip_h & 0xFFFF0000u) == 0xA9FE0000u) continue;
                    if (ad->Ipv4Metric < bestMetric) {
                        bestMetric = ad->Ipv4Metric;
                        std::memcpy(&chosenLocal, sa, sizeof(chosenLocal));
                        chosenLocal.sin_port = 0; // ephemeral source port
                        haveChosen = true;
                    }
                }
            }
        }
    }

    sock_ = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (sock_ == INVALID_SOCKET) {
        status_ = "socket() failed";
        freeaddrinfo(result);
        close();
        return false;
    }
    if (haveChosen) {
        if (bind(sock_, reinterpret_cast<sockaddr*>(&chosenLocal), sizeof(chosenLocal)) != 0) {
            // Bind failure isn't fatal — fall back to default routing.
            char buf[32] = {0};
            inet_ntop(AF_INET, &chosenLocal.sin_addr, buf, sizeof(buf));
            status_ = std::string("bind ") + buf + " failed; trying default route";
        }
    }

    // Non-blocking connect with a 3 s budget. Was blocking previously, which
    // froze the GUI thread for ~20 s when the host had no route (typical when
    // user clicked Connect on the AP profile while still on office Wi-Fi).
    // Windows' "POSIX WinThreads not responding" dialog popped up around 5 s.
    u_long nonblocking = 1;
    ioctlsocket(sock_, FIONBIO, &nonblocking);

    const int connectRc = connect(sock_, result->ai_addr,
                                  static_cast<int>(result->ai_addrlen));
    if (connectRc != 0) {
        const int err = WSAGetLastError();
        if (err != WSAEWOULDBLOCK && err != WSAEINPROGRESS) {
            char buf[32] = {0};
            if (haveChosen) inet_ntop(AF_INET, &chosenLocal.sin_addr, buf, sizeof(buf));
            status_ = std::string("connect failed (err ") + std::to_string(err) + ")"
                      + (haveChosen ? std::string(" via ") + buf : std::string(" (no on-link NIC)"));
            freeaddrinfo(result);
            close();
            return false;
        }
        // In progress — wait writable for up to 3 s.
        fd_set wfds, efds;
        FD_ZERO(&wfds); FD_ZERO(&efds);
        FD_SET(sock_, &wfds); FD_SET(sock_, &efds);
        timeval tv{};
        tv.tv_sec = 3;
        tv.tv_usec = 0;
        const int sel = select(0, nullptr, &wfds, &efds, &tv);
        if (sel <= 0) {
            char buf[32] = {0};
            if (haveChosen) inet_ntop(AF_INET, &chosenLocal.sin_addr, buf, sizeof(buf));
            status_ = std::string(sel == 0 ? "connect timed out (3s)" : "select failed")
                      + (haveChosen ? std::string(" via ") + buf : std::string(" (no on-link NIC)"));
            freeaddrinfo(result);
            close();
            return false;
        }
        // Confirm via SO_ERROR — select() can pick an exception fd too.
        int soErr = 0;
        int soLen = sizeof(soErr);
        getsockopt(sock_, SOL_SOCKET, SO_ERROR,
                   reinterpret_cast<char*>(&soErr), &soLen);
        if (soErr != 0) {
            status_ = std::string("connect failed (err ") + std::to_string(soErr) + ")";
            freeaddrinfo(result);
            close();
            return false;
        }
    }
    freeaddrinfo(result);

    status_ = "connected " + host + ":" + std::to_string(port);
    return true;
#endif
}

void TcpCanTransport::close()
{
#ifdef _WIN32
    if (sock_ != INVALID_SOCKET) {
        closesocket(sock_);
        sock_ = INVALID_SOCKET;
        WSACleanup();
    }
#endif
    txText_.clear();
    rxText_.clear();
    frames_.clear();
    status_ = "closed";
}

bool TcpCanTransport::isOpen() const
{
#ifdef _WIN32
    return sock_ != INVALID_SOCKET;
#else
    return false;
#endif
}

bool TcpCanTransport::send(const CanFrame& frame)
{
    if (!isOpen()) return false;
    const std::string line = encodeJsonLine(frame);
#ifdef _WIN32
    constexpr size_t kMaxQueuedBytes = 64 * 1024;
    flushTx();
    if (!isOpen()) return false;
    if (txText_.size() + line.size() > kMaxQueuedBytes) {
        status_ = "tx queue full";
        return false;
    }
    txText_ += line;
    flushTx();
    return isOpen();
#else
    return false;
#endif
}

bool TcpCanTransport::poll(CanFrame& frame)
{
    if (!isOpen()) return false;
    readAvailable();
    if (frames_.empty()) return false;
    frame = frames_.front();
    frames_.pop_front();
    return true;
}

void TcpCanTransport::readAvailable()
{
#ifdef _WIN32
    if (sock_ == INVALID_SOCKET) return;
    flushTx();
    if (sock_ == INVALID_SOCKET) return;
    for (;;) {
        char buf[4096];
        const int got = recv(sock_, buf, sizeof(buf), 0);
        if (got > 0) {
            rxText_.append(buf, buf + got);
            if (got < static_cast<int>(sizeof(buf))) break;
            continue;
        }
        if (got == 0) {
            // Peer closed the connection cleanly.
            closesocket(sock_);
            sock_ = INVALID_SOCKET;
            status_ = "peer closed connection";
            break;
        }
        const int err = WSAGetLastError();
        if (err == WSAEWOULDBLOCK || err == WSAEINTR) break;
        closesocket(sock_);
        sock_ = INVALID_SOCKET;
        status_ = "recv error " + std::to_string(err);
        break;
    }
    consumeLines();
#endif
}

void TcpCanTransport::flushTx()
{
#ifdef _WIN32
    while (sock_ != INVALID_SOCKET && !txText_.empty()) {
        const int chunk = static_cast<int>(std::min<size_t>(txText_.size(), 16 * 1024));
        const int sent = ::send(sock_, txText_.data(), chunk, 0);
        if (sent > 0) {
            txText_.erase(0, static_cast<size_t>(sent));
            continue;
        }
        if (sent == 0) {
            break;
        }

        const int err = WSAGetLastError();
        if (err == WSAEWOULDBLOCK || err == WSAEINTR) {
            break;
        }

        closesocket(sock_);
        sock_ = INVALID_SOCKET;
        txText_.clear();
        status_ = "send error " + std::to_string(err);
        break;
    }
#endif
}

void TcpCanTransport::consumeLines()
{
    for (;;) {
        const size_t pos = rxText_.find('\n');
        if (pos == std::string::npos) break;
        std::string line = rxText_.substr(0, pos);
        rxText_.erase(0, pos + 1);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        CanFrame frame;
        if (parseJsonLine(line, frame)) frames_.push_back(frame);
    }
}

bool TcpCanTransport::parseJsonLine(const std::string& line, CanFrame& frame) const
{
    std::vector<uint8_t> data;
    if (!hexToBytes(jsonString(line, "data"), data)) return false;
    frame.timestamp = steadyNow();
    frame.rx = jsonBool(line, "rx", true);
    frame.id = jsonUint(line, "id", 0);
    frame.extended = jsonBool(line, "ext", false);
    frame.fd = jsonBool(line, "fd", data.size() > 8);
    frame.brs = jsonBool(line, "brs", false);
    frame.dlc = lengthToDlc(data.size());
    frame.data = std::move(data);
    return true;
}

std::string TcpCanTransport::encodeJsonLine(const CanFrame& frame) const
{
    std::ostringstream os;
    os << "{\"ts\":" << std::fixed << std::setprecision(6) << frame.timestamp
       << ",\"id\":" << frame.id
       << ",\"rx\":" << (frame.rx ? "true" : "false")
       << ",\"ext\":" << (frame.extended ? "true" : "false")
       << ",\"fd\":" << (frame.fd ? "true" : "false")
       << ",\"brs\":" << (frame.brs ? "true" : "false")
       << ",\"data\":\"" << bytesToHex(frame.data) << "\"}\n";
    return os.str();
}

} // namespace drivescope
