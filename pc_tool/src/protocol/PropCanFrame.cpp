#include "protocol/PropCanFrame.h"

namespace drivescope {

// Copied byte-identically from the former local helpers buildId() /
// getMod/getCmd/getDst/getSrc (CanBootloaderFlasher.cpp) and propCanId()
// (DebugProtocol.cpp). Slice 2a: PropCAN frame-envelope extraction.

uint32_t packPropCanId(const PropCanFrameId& id)
{
    return (uint32_t(id.mod & 0x01u) << 28) |
           ((uint32_t(id.cmd) & 0x0FFFu) << 16) |
           (uint32_t(id.dst) << 8) |
           uint32_t(id.src);
}

uint8_t  propCanMod(uint32_t rawId) { return static_cast<uint8_t>((rawId >> 28) & 0x01u); }
uint16_t propCanCmd(uint32_t rawId) { return static_cast<uint16_t>((rawId >> 16) & 0x0FFFu); }
uint8_t  propCanDst(uint32_t rawId) { return static_cast<uint8_t>((rawId >> 8) & 0xFFu); }
uint8_t  propCanSrc(uint32_t rawId) { return static_cast<uint8_t>(rawId & 0xFFu); }

PropCanFrameId unpackPropCanId(uint32_t rawId)
{
    return PropCanFrameId{ propCanMod(rawId), propCanCmd(rawId), propCanDst(rawId), propCanSrc(rawId) };
}

} // namespace drivescope
