#pragma once

#include "can/CanFrame.h"

#include <fstream>
#include <string>

namespace drivescope {

class CanLogger {
public:
    bool start(const std::string& path);
    void stop();
    void log(const CanFrame& frame);
    bool active() const { return file_.is_open(); }
    const std::string& path() const { return path_; }

private:
    std::ofstream file_;
    std::string path_;
};

} // namespace drivescope
