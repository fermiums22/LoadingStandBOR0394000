#pragma once

#include <cstddef>
#include <cstdint>

namespace drivescope {

// -----------------------------------------------------------------------------
// CRC32 helpers for the DDV2 protocols. Single responsibility: CRC only.
//
// There are TWO distinct CRC32s in DDV2 — do not confuse them. See
// umbrella protocol/ddv2_protocol.yaml (capture_crc32 vs stm32_word_crc32).
// -----------------------------------------------------------------------------

// CRC-32/MPEG-2, byte-wise. poly 0x04C11DB7, init 0xFFFFFFFF, no reflect, no
// xor-out. check("123456789") == 0x0376E6E7. Used by the CAPTURE protocol only.
// The motor/RK firmware ship the byte-identical loop in their app_capture.c —
// keep them in lockstep.
uint32_t captureCrc32Mpeg2Bytes(const uint8_t* data, std::size_t size);

// STM32 HW-style CRC32. Input is consumed as 32-bit little-endian words; a
// trailing partial word is padded with 0xFF up to 4 bytes. Same poly/init, no
// reflect, no xor-out. check("123456789") == 0xD9020D98. Used by boot/file-
// transfer and unified read. `initial` lets callers chain across chunks (pass
// the previous return value); firmware compares the lower 16 bits for the
// per-MMSG CRC field (mmsg_crc32_low16).
uint32_t stm32WordCrc32(const uint8_t* data, std::size_t size, uint32_t initial = 0xFFFFFFFFu);

} // namespace drivescope
