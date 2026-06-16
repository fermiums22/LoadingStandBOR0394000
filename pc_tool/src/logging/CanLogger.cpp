#include "logging/CanLogger.h"

#include <iomanip>

namespace drivescope {

bool CanLogger::start(const std::string& path)
{
    stop();
    file_.open(path, std::ios::out | std::ios::trunc);
    if (!file_) return false;
    path_ = path;
    file_ << "timestamp,dir,id,ext,fd,brs,dlc,data\n";
    return true;
}

void CanLogger::stop()
{
    if (file_.is_open()) file_.close();
}

void CanLogger::log(const CanFrame& frame)
{
    if (!file_) return;
    file_ << std::fixed << std::setprecision(6) << frame.timestamp << ','
          << (frame.rx ? "RX" : "TX") << ','
          << idToHex(frame.id, frame.extended) << ','
          << (frame.extended ? 1 : 0) << ','
          << (frame.fd ? 1 : 0) << ','
          << (frame.brs ? 1 : 0) << ','
          << static_cast<int>(frame.dlc) << ','
          << bytesToHex(frame.data) << '\n';
}

} // namespace drivescope
