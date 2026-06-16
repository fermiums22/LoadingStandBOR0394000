#pragma once

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace drivescope {

struct CanFrame {
    double timestamp = 0.0;
    bool rx = true;
    uint32_t id = 0;
    bool extended = false;
    bool fd = false;
    bool brs = false;
    uint8_t dlc = 0;
    std::vector<uint8_t> data;
};

inline int dlcToLength(uint8_t dlc)
{
    static constexpr int map[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    return map[std::min<uint8_t>(dlc, 15)];
}

inline uint8_t lengthToDlc(size_t len)
{
    if (len <= 8) return static_cast<uint8_t>(len);
    if (len <= 12) return 9;
    if (len <= 16) return 10;
    if (len <= 20) return 11;
    if (len <= 24) return 12;
    if (len <= 32) return 13;
    if (len <= 48) return 14;
    return 15;
}

inline int hexValue(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + c - 'a';
    if (c >= 'A' && c <= 'F') return 10 + c - 'A';
    return -1;
}

inline bool parseHexUint(const std::string& s, size_t pos, size_t count, uint32_t& value)
{
    if (pos + count > s.size()) return false;
    uint32_t out = 0;
    for (size_t i = 0; i < count; ++i) {
        const int v = hexValue(s[pos + i]);
        if (v < 0) return false;
        out = (out << 4) | static_cast<uint32_t>(v);
    }
    value = out;
    return true;
}

inline std::string bytesToHex(const std::vector<uint8_t>& data)
{
    std::ostringstream os;
    os << std::uppercase << std::hex << std::setfill('0');
    for (uint8_t b : data) os << std::setw(2) << static_cast<int>(b);
    return os.str();
}

inline std::string bytesToHexSpaced(const std::vector<uint8_t>& data)
{
    std::ostringstream os;
    os << std::uppercase << std::hex << std::setfill('0');
    for (size_t i = 0; i < data.size(); ++i) {
        if (i != 0) os << ' ';
        os << std::setw(2) << static_cast<int>(data[i]);
    }
    return os.str();
}

inline bool hexToBytes(const std::string& hex, std::vector<uint8_t>& out)
{
    if ((hex.size() % 2) != 0) return false;
    std::vector<uint8_t> bytes;
    bytes.reserve(hex.size() / 2);
    for (size_t i = 0; i < hex.size(); i += 2) {
        const int hi = hexValue(hex[i]);
        const int lo = hexValue(hex[i + 1]);
        if (hi < 0 || lo < 0) return false;
        bytes.push_back(static_cast<uint8_t>((hi << 4) | lo));
    }
    out = std::move(bytes);
    return true;
}

inline std::string idToHex(uint32_t id, bool extended)
{
    std::ostringstream os;
    os << "0x" << std::uppercase << std::hex << std::setfill('0')
       << std::setw(extended ? 8 : 3) << id;
    return os.str();
}

} // namespace drivescope
