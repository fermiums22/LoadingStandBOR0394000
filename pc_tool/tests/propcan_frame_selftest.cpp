// Host self-test for protocol/PropCanFrame — the PropCAN frame envelope
// (29-bit id layout + node addresses + image mode). Slice 2a.
//
// Dependency-free:
//   g++ -std=c++20 -I ../src propcan_frame_selftest.cpp ../src/protocol/PropCanFrame.cpp -o propcan_frame_selftest
//   ./propcan_frame_selftest
#include "protocol/PropCanFrame.h"

#include <cstdint>
#include <cstdio>

using namespace drivescope;

static int g_failures = 0;

// pack + unpack round-trip against a known wire id.
static void roundtrip(const char* name, uint8_t mod, uint16_t cmd, uint8_t dst,
                      uint8_t src, uint32_t expect)
{
    const PropCanFrameId in{mod, cmd, dst, src};
    const uint32_t id = packPropCanId(in);
    const PropCanFrameId out = unpackPropCanId(id);
    const bool ok = (id == expect) &&
                    out.mod == mod && out.cmd == cmd && out.dst == dst && out.src == src &&
                    propCanMod(id) == mod && propCanCmd(id) == cmd &&
                    propCanDst(id) == dst && propCanSrc(id) == src;
    std::printf("  %s  %-14s pack=0x%08X expect=0x%08X\n", ok ? "PASS" : "FAIL", name, id, expect);
    if (!ok) ++g_failures;
}

static void check(const char* name, bool ok)
{
    std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_failures;
}

int main()
{
    // Known frames (mirror umbrella protocol/golden_frames.json).
    roundtrip("STREAM_VALUE", kPropCanModeApp,  0x0E8, kCanAddrBroadcast, kCanAddrMotor, 0x10E8FF02u);
    roundtrip("CAP_REQ",      kPropCanModeApp,  0x0E0, kCanAddrMotor,     kCanAddrPc,    0x10E00210u);
    roundtrip("BOOT_FINISH",  kPropCanModeBoot, 0x0CF, kCanAddrMotor,     kCanAddrPc,    0x00CF0210u);

    // Envelope masking matches the previous code: cmd to 12 bits, mod to 1 bit.
    check("cmd masked to 12 bits",
          packPropCanId({1, 0x1234, 0x02, 0x10}) == packPropCanId({1, 0x234, 0x02, 0x10}));
    check("mod masked to 1 bit", propCanMod(0x10E00210u) == 1 && propCanMod(0x00CF0210u) == 0);

    // Address vocabulary sanity.
    check("addresses", kCanAddrMain == 0x01 && kCanAddrMotor == 0x02 && kCanAddrRk == 0x03 &&
                       kCanAddrPc == 0x10 && kCanAddrBroadcast == 0xFF);

    std::printf("%s\n", g_failures ? "RESULT: FAIL" : "RESULT: PASS");
    return g_failures ? 1 : 0;
}
