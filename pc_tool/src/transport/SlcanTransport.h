#pragma once

#include "can/SlcanParser.h"
#include "transport/ICanTransport.h"

#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#endif

namespace drivescope {

class SlcanTransport final : public ICanTransport {
public:
    SlcanTransport() = default;
    ~SlcanTransport() override { close(); }

    std::string portName = "COM3";
    int baud = 8000000;
    std::string nominalCommand = "S6";
    std::string dataCommand = "Y2";
    bool silentMode = false;

    bool open() override;
    void close() override;
    bool send(const CanFrame& frame) override;
    bool poll(CanFrame& frame) override;
    bool isOpen() const override;
    const std::string& status() const override;

private:
    bool writeStringLocked(const std::string& text);
    void readerLoop();
    void consumeRxBufferLocked();

    SlcanParser parser_;
    mutable std::mutex stateMutex_;
    std::string status_ = "closed";
    std::string rxText_;
    std::deque<CanFrame> frames_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> opened_{false};
    std::thread reader_;

#ifdef _WIN32
    HANDLE handle_ = INVALID_HANDLE_VALUE;
    std::mutex ioMutex_;
#endif
};

} // namespace drivescope
