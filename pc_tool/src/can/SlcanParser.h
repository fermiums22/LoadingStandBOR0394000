#pragma once

#include "can/CanFrame.h"

#include <chrono>
#include <string>

namespace drivescope {

class SlcanParser {
public:
    SlcanParser();

    bool parseLine(const std::string& line, CanFrame& frame);
    std::string encodeFrame(const CanFrame& frame) const;
    double nowSeconds() const;

private:
    std::chrono::steady_clock::time_point start_;
};

} // namespace drivescope
