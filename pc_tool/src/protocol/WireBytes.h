#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace drivescope {

// -----------------------------------------------------------------------------
// Little-endian byte primitives for the wire codecs. This is a tiny ALGORITHM
// /primitive module, not a builder framework: just read/write a scalar at a
// byte offset so codec pack/parse code reads like the wire layout instead of a
// wall of shifts and casts. No memcpy (explicit endianness), no classes, no
// fluent API. Bounds are the caller's responsibility (codecs work on fixed
// 8-byte frames).
// -----------------------------------------------------------------------------

inline void putU16Le(std::vector<uint8_t>& d, size_t at, uint16_t v)
{
    d[at]     = static_cast<uint8_t>(v & 0xFFu);
    d[at + 1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
}

inline void putU32Le(std::vector<uint8_t>& d, size_t at, uint32_t v)
{
    d[at]     = static_cast<uint8_t>(v & 0xFFu);
    d[at + 1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
    d[at + 2] = static_cast<uint8_t>((v >> 16) & 0xFFu);
    d[at + 3] = static_cast<uint8_t>((v >> 24) & 0xFFu);
}

inline uint16_t getU16Le(const std::vector<uint8_t>& d, size_t at)
{
    return static_cast<uint16_t>(d[at] | (static_cast<uint16_t>(d[at + 1]) << 8));
}

inline uint32_t getU32Le(const std::vector<uint8_t>& d, size_t at)
{
    return static_cast<uint32_t>(d[at]) |
           (static_cast<uint32_t>(d[at + 1]) << 8) |
           (static_cast<uint32_t>(d[at + 2]) << 16) |
           (static_cast<uint32_t>(d[at + 3]) << 24);
}

} // namespace drivescope
