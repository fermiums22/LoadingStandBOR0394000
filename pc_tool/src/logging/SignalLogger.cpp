#include "logging/SignalLogger.h"

#include <iomanip>

namespace drivescope {

bool SignalLogger::start(const std::string& path)
{
    stop();
    file_.open(path, std::ios::out | std::ios::trunc);
    if (!file_) return false;
    file_ << "timestamp,node_id,name,address,type,value\n";
    return true;
}

void SignalLogger::stop()
{
    if (file_.is_open()) file_.close();
}

void SignalLogger::log(double timestamp, uint8_t nodeId, const std::string& name,
                       uint32_t address, const std::string& type, const std::string& value)
{
    if (!file_) return;
    file_ << std::fixed << std::setprecision(6) << timestamp << ','
          << static_cast<int>(nodeId) << ','
          << '"' << name << '"' << ','
          << "0x" << std::uppercase << std::hex << address << std::dec << ','
          << type << ','
          << value << '\n';
}

} // namespace drivescope
