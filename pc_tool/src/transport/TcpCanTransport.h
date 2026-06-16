#pragma once

#include "transport/ICanTransport.h"

#include <deque>
#include <string>

#ifdef _WIN32
#include <winsock2.h>
#endif

namespace drivescope {

class TcpCanTransport final : public ICanTransport {
public:
    std::string host = "192.168.0.102";
    int port = 45333;

    bool open() override;
    void close() override;
    bool send(const CanFrame& frame) override;
    bool poll(CanFrame& frame) override;
    bool isOpen() const override;
    const std::string& status() const override { return status_; }

private:
    void flushTx();
    void readAvailable();
    void consumeLines();
    bool parseJsonLine(const std::string& line, CanFrame& frame) const;
    std::string encodeJsonLine(const CanFrame& frame) const;

    std::string status_ = "closed";
    std::string txText_;
    std::string rxText_;
    std::deque<CanFrame> frames_;

#ifdef _WIN32
    SOCKET sock_ = INVALID_SOCKET;
#endif
};

} // namespace drivescope
