// Smallest practical host self-test for protocol/Crc32 (Slice 1).
//
// Dependency-free: compile just the CRC module, no GUI/transport deps.
//   g++ -std=c++20 -I ../src crc32_selftest.cpp ../src/protocol/Crc32.cpp -o crc32_selftest
//   ./crc32_selftest
//
// Verifies both golden CRC vectors and the incremental-carry semantics that the
// boot/file-transfer flow relies on (chunked CRC across 4-byte-aligned blocks).
#include "protocol/Crc32.h"

#include <cstdint>
#include <cstdio>

using namespace drivescope;

static int g_failures = 0;

static void expect(const char* name, uint32_t got, uint32_t want)
{
    if (got == want) {
        std::printf("  PASS  %-30s = 0x%08X\n", name, got);
    } else {
        std::printf("  FAIL  %-30s got 0x%08X want 0x%08X\n", name, got, want);
        ++g_failures;
    }
}

int main()
{
    const auto* s = reinterpret_cast<const uint8_t*>("123456789");
    const std::size_t n = 9;

    // Golden vectors (mirror protocol/golden_frames.json crc32_vectors).
    expect("captureCrc32Mpeg2Bytes", captureCrc32Mpeg2Bytes(s, n), 0x0376E6E7u);
    expect("stm32WordCrc32", stm32WordCrc32(s, n), 0xD9020D98u);

    // Incremental carry: two 4-byte (aligned) chunks chained must equal the
    // one-shot CRC — this is exactly how flashFile() carries fileCrc across
    // 16 KiB blocks (each a multiple of 4, so no intermediate tail padding).
    const uint8_t buf[8] = {'1', '2', '3', '4', '5', '6', '7', '8'};
    const uint32_t oneShot = stm32WordCrc32(buf, 8);
    uint32_t chained = stm32WordCrc32(buf, 4);
    chained = stm32WordCrc32(buf + 4, 4, chained);
    expect("stm32 incremental == oneShot", chained, oneShot);

    std::printf("%s\n", g_failures ? "RESULT: FAIL" : "RESULT: PASS");
    return g_failures ? 1 : 0;
}
