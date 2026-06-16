#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace drivescope {

// -----------------------------------------------------------------------------
// Wire value types + generic value decode/format helpers.
//
// WireValueType is the NEUTRAL scalar type shared across protocols, so capture
// slots and stream subscriptions can name a value type without depending on
// each other's codec. The decode/format helpers turn raw little-endian payload
// bytes into a numeric value and a display string. Pure value logic — no frame
// envelope, no transport, no UI.
// -----------------------------------------------------------------------------

enum class WireValueType : uint8_t {
    U8 = 0, I8 = 1,
    U16 = 2, I16 = 3,
    U32 = 4, I32 = 5,
    F32 = 6,
};

// Decode `bytes` (little-endian) as `type` ("uint8"/"u8", "int16"/"i16",
// "float", ...). Sets ok=false and returns 0 when the type/size don't match.
double decodeValue(const std::vector<uint8_t>& bytes, const std::string& type, bool& ok);

// Render a decoded value for display (6 decimals for float/double, else int).
std::string formatValue(double value, const std::string& type, bool ok);

// String <-> WireValueType index (returned as uint8_t for wire/compat use).
uint8_t captureSlotTypeFromString(const std::string& type);
const char* captureSlotTypeName(uint8_t type);

} // namespace drivescope
