#include "protocol/WireValue.h"

#include <cstring>
#include <iomanip>
#include <sstream>

namespace drivescope {

// Moved verbatim from DebugProtocol.cpp (Slice 2c). Pure value decode/format.

double decodeValue(const std::vector<uint8_t>& bytes, const std::string& type, bool& ok)
{
    ok = false;
    auto le16 = [&]() -> uint16_t {
        return static_cast<uint16_t>(bytes[0] | (static_cast<uint16_t>(bytes[1]) << 8));
    };
    auto le32 = [&]() -> uint32_t {
        return static_cast<uint32_t>(bytes[0]) |
               (static_cast<uint32_t>(bytes[1]) << 8) |
               (static_cast<uint32_t>(bytes[2]) << 16) |
               (static_cast<uint32_t>(bytes[3]) << 24);
    };

    if ((type == "uint8" || type == "u8") && bytes.size() >= 1) {
        ok = true;
        return bytes[0];
    }
    if ((type == "int8" || type == "i8") && bytes.size() >= 1) {
        ok = true;
        return static_cast<int8_t>(bytes[0]);
    }
    if ((type == "uint16" || type == "u16") && bytes.size() >= 2) {
        ok = true;
        return le16();
    }
    if ((type == "int16" || type == "i16") && bytes.size() >= 2) {
        ok = true;
        return static_cast<int16_t>(le16());
    }
    if ((type == "uint32" || type == "u32") && bytes.size() >= 4) {
        ok = true;
        return le32();
    }
    if ((type == "int32" || type == "i32") && bytes.size() >= 4) {
        ok = true;
        return static_cast<int32_t>(le32());
    }
    if (type == "float" && bytes.size() >= 4) {
        const uint32_t raw = le32();
        float v = 0.0f;
        std::memcpy(&v, &raw, sizeof(v));
        ok = true;
        return v;
    }
    return 0.0;
}

std::string formatValue(double value, const std::string& type, bool ok)
{
    if (!ok) return "n/a";
    std::ostringstream os;
    if (type == "float" || type == "double") {
        os << std::fixed << std::setprecision(6) << value;
    } else {
        os << std::setprecision(0) << std::fixed << value;
    }
    return os.str();
}

uint8_t captureSlotTypeFromString(const std::string& type)
{
    if (type == "uint8"  || type == "u8")  return static_cast<uint8_t>(WireValueType::U8);
    if (type == "int8"   || type == "i8")  return static_cast<uint8_t>(WireValueType::I8);
    if (type == "uint16" || type == "u16") return static_cast<uint8_t>(WireValueType::U16);
    if (type == "int16"  || type == "i16") return static_cast<uint8_t>(WireValueType::I16);
    if (type == "uint32" || type == "u32") return static_cast<uint8_t>(WireValueType::U32);
    if (type == "int32"  || type == "i32") return static_cast<uint8_t>(WireValueType::I32);
    return static_cast<uint8_t>(WireValueType::F32);
}

const char* captureSlotTypeName(uint8_t type)
{
    switch (static_cast<WireValueType>(type)) {
        case WireValueType::U8:  return "u8";
        case WireValueType::I8:  return "i8";
        case WireValueType::U16: return "u16";
        case WireValueType::I16: return "i16";
        case WireValueType::U32: return "u32";
        case WireValueType::I32: return "i32";
        case WireValueType::F32: return "f32";
    }
    return "?";
}

} // namespace drivescope
