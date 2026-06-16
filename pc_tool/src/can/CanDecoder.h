#pragma once

#include "can/CanFrame.h"

#include <string>
#include <vector>

namespace drivescope {

std::string decodeCanFrame(const CanFrame& frame);

struct TooltipField {
    std::string pos;     // e.g. "bit 8", "byte 0", "bytes 1..4"
    std::string size;    // e.g. "1 bit", "1 byte", "4 bytes (uint32 LE)"
    std::string desc;    // human-readable name (UPPERCASE for flags)
    std::string value;   // current value as string
    bool active;         // highlight (e.g. fault bit set, error flag on)
};

std::vector<TooltipField> buildFrameTooltipTable(const CanFrame& frame);

} // namespace drivescope
