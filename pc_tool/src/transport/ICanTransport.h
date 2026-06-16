#pragma once

#include "can/CanFrame.h"

#include <string>

namespace drivescope {

class ICanTransport {
public:
    virtual ~ICanTransport() = default;

    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool send(const CanFrame& frame) = 0;
    virtual bool poll(CanFrame& frame) = 0;
    virtual bool isOpen() const = 0;
    virtual const std::string& status() const = 0;
};

} // namespace drivescope
