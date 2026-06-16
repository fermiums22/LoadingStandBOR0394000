#pragma once

#include <cstdint>
#include <fstream>
#include <string>

namespace drivescope {

class SignalLogger {
public:
    bool start(const std::string& path);
    void stop();
    void log(double timestamp, uint8_t nodeId, const std::string& name,
             uint32_t address, const std::string& type, const std::string& value);
    bool active() const { return file_.is_open(); }

private:
    std::ofstream file_;
};

} // namespace drivescope
