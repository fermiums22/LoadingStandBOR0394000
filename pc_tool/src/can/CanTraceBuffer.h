#pragma once

#include "can/CanFrame.h"

#include <cstddef>
#include <deque>
#include <vector>

namespace drivescope {

class CanTraceBuffer {
public:
    explicit CanTraceBuffer(size_t limit = 10000);

    void push(const CanFrame& frame);
    void clear();
    std::vector<CanFrame> snapshot() const;
    size_t size() const { return frames_.size(); }

private:
    size_t limit_;
    std::deque<CanFrame> frames_;
};

} // namespace drivescope
