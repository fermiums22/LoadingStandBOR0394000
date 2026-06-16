// Host self-test for protocol/WireValue (Slice 2c): generic value decode/format
// + WireValueType string conversion, and the stream size/type encoding that now
// uses the neutral WireValueType.
//
//   g++ -std=c++20 -I../src wire_value_selftest.cpp \
//       ../src/protocol/WireValue.cpp ../src/protocol/StreamCodec.cpp \
//       ../src/protocol/PropCanFrame.cpp -o wire_value_selftest
#include "protocol/WireValue.h"
#include "protocol/StreamCodec.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

using namespace drivescope;

static int g_failures = 0;

static void check(const char* name, bool ok)
{
    std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_failures;
}

static double dec(std::vector<uint8_t> bytes, const char* type, bool& ok)
{
    return decodeValue(bytes, type, ok);
}

int main()
{
    bool ok = false;

    // -- decode LE for every scalar type -----------------------------------
    check("decode u8",  dec({0x2A}, "u8", ok) == 42.0 && ok);
    check("decode i8",  dec({0xFF}, "i8", ok) == -1.0 && ok);
    check("decode u16", dec({0x34, 0x12}, "u16", ok) == 4660.0 && ok);            // 0x1234
    check("decode i16", dec({0x00, 0x80}, "i16", ok) == -32768.0 && ok);
    check("decode u32", dec({0x78, 0x56, 0x34, 0x12}, "u32", ok) == 305419896.0 && ok); // 0x12345678
    check("decode i32", dec({0x00, 0x00, 0x00, 0x80}, "i32", ok) == -2147483648.0 && ok);
    {
        const double v = dec({0x00, 0x00, 0x80, 0x3F}, "float", ok);             // 1.0f LE
        check("decode float", ok && std::fabs(v - 1.0) < 1e-6);
    }
    // wrong size -> not ok
    dec({0x01}, "u32", ok);
    check("decode short buffer -> !ok", !ok);

    // -- format ------------------------------------------------------------
    check("format float", formatValue(1.5, "float", true) == "1.500000");
    check("format int",   formatValue(42.0, "u16", true) == "42");
    check("format n/a",   formatValue(0.0, "u8", false) == "n/a");

    // -- string <-> type index --------------------------------------------
    check("type from string u16", captureSlotTypeFromString("u16") == static_cast<uint8_t>(WireValueType::U16));
    check("type from string int8", captureSlotTypeFromString("int8") == static_cast<uint8_t>(WireValueType::I8));
    check("type from string default f32", captureSlotTypeFromString("???") == static_cast<uint8_t>(WireValueType::F32));
    check("type name roundtrip",
          std::string(captureSlotTypeName(captureSlotTypeFromString("u32"))) == "u32");
    check("type name f32", std::string(captureSlotTypeName(static_cast<uint8_t>(WireValueType::F32))) == "f32");

    // -- stream size/type encoding (neutral WireValueType) -----------------
    check("stream sizeType F32", streamSizeTypeFromCaptureType(WireValueType::F32) == 0x22);
    check("stream sizeType U16", streamSizeTypeFromCaptureType(WireValueType::U16) == 0x01);
    check("stream sizeType I8",  streamSizeTypeFromCaptureType(WireValueType::I8) == 0x10);
    check("stream sizeBytes 1B", streamSizeBytes(0x00) == 1);
    check("stream sizeBytes 2B", streamSizeBytes(0x01) == 2);
    check("stream sizeBytes 4B", streamSizeBytes(0x22) == 4);

    std::printf("%s\n", g_failures ? "RESULT: FAIL" : "RESULT: PASS");
    return g_failures ? 1 : 0;
}
