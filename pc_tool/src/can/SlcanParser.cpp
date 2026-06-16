#include "can/SlcanParser.h"

#include <iomanip>
#include <sstream>

namespace drivescope {

SlcanParser::SlcanParser()
    : start_(std::chrono::steady_clock::now())
{
}

double SlcanParser::nowSeconds() const
{
    using seconds_d = std::chrono::duration<double>;
    return seconds_d(std::chrono::steady_clock::now() - start_).count();
}

bool SlcanParser::parseLine(const std::string& line, CanFrame& frame)
{
    if (line.empty()) return false;

    const char kind = line[0];
    const bool classicStd = kind == 't';
    const bool classicExt = kind == 'T';
    const bool fdStd = kind == 'd' || kind == 'b';
    const bool fdExt = kind == 'D' || kind == 'B';
    if (!classicStd && !classicExt && !fdStd && !fdExt) return false;

    const bool extended = classicExt || fdExt;
    const bool fd = fdStd || fdExt;
    const bool brs = kind == 'b' || kind == 'B';
    const size_t idChars = extended ? 8 : 3;

    uint32_t id = 0;
    if (!parseHexUint(line, 1, idChars, id)) return false;

    const size_t dlcPos = 1 + idChars;
    if (dlcPos >= line.size()) return false;
    const int dlcValue = hexValue(line[dlcPos]);
    if (dlcValue < 0 || dlcValue > 15) return false;

    const uint8_t dlc = static_cast<uint8_t>(dlcValue);
    const int len = fd ? dlcToLength(dlc) : static_cast<int>(dlc);
    const size_t dataPos = dlcPos + 1;
    if (line.size() < dataPos + static_cast<size_t>(len) * 2) return false;

    std::vector<uint8_t> data;
    data.reserve(static_cast<size_t>(len));
    for (int i = 0; i < len; ++i) {
        uint32_t b = 0;
        if (!parseHexUint(line, dataPos + static_cast<size_t>(i) * 2, 2, b)) return false;
        data.push_back(static_cast<uint8_t>(b));
    }

    frame.timestamp = nowSeconds();
    frame.rx = true;
    frame.id = id;
    frame.extended = extended;
    frame.fd = fd;
    frame.brs = brs;
    frame.dlc = dlc;
    frame.data = std::move(data);
    return true;
}

std::string SlcanParser::encodeFrame(const CanFrame& frame) const
{
    const bool extended = frame.extended;
    const uint8_t dlc = lengthToDlc(frame.data.size());
    const char kind = frame.fd
        ? (frame.brs ? (extended ? 'B' : 'b') : (extended ? 'D' : 'd'))
        : (extended ? 'T' : 't');

    std::ostringstream os;
    os << kind << std::uppercase << std::hex << std::setfill('0');
    os << std::setw(extended ? 8 : 3) << frame.id;
    os << std::setw(1) << static_cast<int>(dlc);
    for (uint8_t b : frame.data) os << std::setw(2) << static_cast<int>(b);
    os << '\r';
    return os.str();
}

} // namespace drivescope
