#pragma once

#include "config/ToolConfig.h"

#include <string>

namespace drivescope {

int runHeadlessCapture(const ToolConfig& config, double durationSec, const std::string& csvPath);

} // namespace drivescope
