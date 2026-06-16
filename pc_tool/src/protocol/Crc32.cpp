#include "protocol/Crc32.h"

#include <cstring>

namespace drivescope {

uint32_t captureCrc32Mpeg2Bytes(const uint8_t* data, std::size_t size)
{
    // Byte-stream CRC-32/MPEG-2. Copied byte-identically from the former
    // captureCrc32() in DebugProtocol.cpp (Slice 1: CRC extraction).
    uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= static_cast<uint32_t>(data[i]) << 24;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80000000u) ? ((crc << 1) ^ 0x04C11DB7u) : (crc << 1);
        }
    }
    return crc;
}

namespace {
// One STM32-CRC step over a 32-bit word. Copied byte-identically from the
// former CanBootloaderFlasher::crcStep.
uint32_t stm32CrcStep(uint32_t crc, uint32_t word)
{
    crc ^= word;
    for (uint8_t i = 0; i < 32; ++i) {
        crc = (crc & 0x80000000u) ? (crc << 1) ^ 0x04C11DB7u : (crc << 1);
    }
    return crc;
}
} // namespace

uint32_t stm32WordCrc32(const uint8_t* data, std::size_t size, uint32_t initial)
{
    // Copied byte-identically from the former CanBootloaderFlasher::crcBytes:
    // full 32-bit LE words first, trailing partial word padded with 0xFF.
    uint32_t crc = initial;
    const std::size_t words = size / 4;
    const std::size_t tail = size % 4;
    for (std::size_t i = 0; i < words; ++i) {
        uint32_t value = 0;
        std::memcpy(&value, data + i * 4, 4);
        crc = stm32CrcStep(crc, value);
    }
    if (tail) {
        const uint8_t* last = data + words * 4;
        uint32_t value = 0;
        value |= (tail > 0 ? uint32_t(last[0]) : 0xFFu);
        value |= (tail > 1 ? uint32_t(last[1]) : 0xFFu) << 8;
        value |= (tail > 2 ? uint32_t(last[2]) : 0xFFu) << 16;
        value |= (tail > 3 ? uint32_t(last[3]) : 0xFFu) << 24;
        crc = stm32CrcStep(crc, value);
    }
    return crc;
}

} // namespace drivescope
