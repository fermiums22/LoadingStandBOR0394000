#include "transport/SlcanTransport.h"

#include <algorithm>
#include <chrono>
#include <thread>

namespace drivescope {

bool SlcanTransport::open()
{
#ifndef _WIN32
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        status_ = "SLCAN COM is implemented for Windows in this MVP";
    }
    return false;
#else
    close();

    std::string path = portName;
    if (path.rfind("\\\\.\\", 0) != 0) path = "\\\\.\\" + path;

    HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING,
                           FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        std::lock_guard<std::mutex> lock(stateMutex_);
        status_ = "open failed: " + portName;
        return false;
    }

    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb)) {
        CloseHandle(h);
        std::lock_guard<std::mutex> lock(stateMutex_);
        status_ = "GetCommState failed";
        return false;
    }
    dcb.BaudRate = static_cast<DWORD>(baud);
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    if (!SetCommState(h, &dcb)) {
        CloseHandle(h);
        std::lock_guard<std::mutex> lock(stateMutex_);
        status_ = "SetCommState failed";
        return false;
    }

    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 10;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 250;
    SetCommTimeouts(h, &timeouts);
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

    handle_ = h;
    stop_.store(false);
    opened_.store(true);

    auto sendCmd = [&](const std::string& cmd) {
        if (!writeStringLocked(cmd + "\r")) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
        return true;
    };

    if (!sendCmd("C") ||
        !sendCmd(nominalCommand) ||
        !sendCmd(dataCommand) ||
        !sendCmd(silentMode ? "M1" : "M0") ||
        !sendCmd("O")) {
        CloseHandle(h);
        handle_ = INVALID_HANDLE_VALUE;
        opened_.store(false);
        std::lock_guard<std::mutex> lock(stateMutex_);
        status_ = "SLCAN setup failed";
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        status_ = "connected " + portName + " " + nominalCommand + " " + dataCommand +
                  (silentMode ? " silent" : " normal");
    }

    reader_ = std::thread([this] { readerLoop(); });
    return true;
#endif
}

void SlcanTransport::close()
{
#ifdef _WIN32
    stop_.store(true);
    opened_.store(false);
    if (reader_.joinable()) reader_.join();
    {
        std::lock_guard<std::mutex> lock(ioMutex_);
        if (handle_ != INVALID_HANDLE_VALUE) {
            DWORD written = 0;
            const char* bye = "C\r";
            WriteFile(handle_, bye, 2, &written, nullptr);
            CloseHandle(handle_);
            handle_ = INVALID_HANDLE_VALUE;
        }
    }
#endif
    std::lock_guard<std::mutex> lock(stateMutex_);
    rxText_.clear();
    frames_.clear();
    status_ = "closed";
}

bool SlcanTransport::isOpen() const
{
    return opened_.load();
}

const std::string& SlcanTransport::status() const
{
    static thread_local std::string snapshot;
    std::lock_guard<std::mutex> lock(stateMutex_);
    snapshot = status_;
    return snapshot;
}

bool SlcanTransport::send(const CanFrame& frame)
{
    if (!opened_.load()) return false;
    return writeStringLocked(parser_.encodeFrame(frame));
}

bool SlcanTransport::poll(CanFrame& frame)
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (frames_.empty()) return false;
    frame = frames_.front();
    frames_.pop_front();
    return true;
}

bool SlcanTransport::writeStringLocked(const std::string& text)
{
#ifndef _WIN32
    (void) text;
    return false;
#else
    std::lock_guard<std::mutex> lock(ioMutex_);
    if (handle_ == INVALID_HANDLE_VALUE) return false;
    DWORD written = 0;
    return WriteFile(handle_, text.data(), static_cast<DWORD>(text.size()), &written, nullptr) &&
           written == text.size();
#endif
}

void SlcanTransport::readerLoop()
{
#ifdef _WIN32
    char buf[4096];
    while (!stop_.load()) {
        DWORD got = 0;
        BOOL ok = FALSE;
        {
            std::lock_guard<std::mutex> lock(ioMutex_);
            if (handle_ == INVALID_HANDLE_VALUE) break;
            ok = ReadFile(handle_, buf, sizeof(buf), &got, nullptr);
        }
        if (!ok) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        if (got == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        std::lock_guard<std::mutex> lock(stateMutex_);
        rxText_.append(buf, buf + got);
        consumeRxBufferLocked();
        if (frames_.size() > 200000) {
            const size_t drop = frames_.size() - 200000;
            frames_.erase(frames_.begin(), frames_.begin() + drop);
        }
    }
#endif
}

void SlcanTransport::consumeRxBufferLocked()
{
    for (;;) {
        const size_t posR = rxText_.find('\r');
        const size_t posN = rxText_.find('\n');
        size_t pos = std::string::npos;
        if (posR != std::string::npos && posN != std::string::npos) pos = std::min(posR, posN);
        else pos = (posR != std::string::npos) ? posR : posN;
        if (pos == std::string::npos) break;

        std::string line = rxText_.substr(0, pos);
        rxText_.erase(0, pos + 1);
        line.erase(std::remove(line.begin(), line.end(), '\a'), line.end());
        if (line.empty()) continue;

        CanFrame frame;
        if (parser_.parseLine(line, frame)) frames_.push_back(frame);
    }
}

} // namespace drivescope
