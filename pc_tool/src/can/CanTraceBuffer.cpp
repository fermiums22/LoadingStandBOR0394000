#include "can/CanTraceBuffer.h"

namespace drivescope {

CanTraceBuffer::CanTraceBuffer(size_t limit)
    : limit_(limit)
{
}

void CanTraceBuffer::push(const CanFrame& frame)
{
    frames_.push_back(frame);
    while (frames_.size() > limit_) frames_.pop_front();
}

void CanTraceBuffer::clear()
{
    frames_.clear();
}

std::vector<CanFrame> CanTraceBuffer::snapshot() const
{
    return {frames_.begin(), frames_.end()};
}

} // namespace drivescope
