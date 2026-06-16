#pragma once

#include "can/CanFrame.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>

namespace drivescope {

struct CanBootloaderFlashRequest {
    enum class Device { Motor, Rk };

    Device device = Device::Motor;
    std::string filePath;
    bool enterBoot = true;
    bool returnToApp = true;
};

struct CanBootloaderFlashSnapshot {
    bool busy = false;
    float progress = 0.0f;
    std::string status;
    std::string log;
};

class CanBootloaderFlasher {
public:
    using SendFn = std::function<bool(const CanFrame&)>;

    CanBootloaderFlasher() = default;
    ~CanBootloaderFlasher();

    bool start(const CanBootloaderFlashRequest& request, SendFn send);
    void cancel();
    void onFrame(const CanFrame& frame);
    CanBootloaderFlashSnapshot snapshot() const;

private:
    enum : uint8_t {
        kAddrMain = 0x01,
        kAddrMotor = 0x02,
        kAddrRk = 0x03,
        kAddrPc = 0x10,
    };

    enum : uint16_t {
        kAppCmdGoBoot = 0x001,
        kBootCmdGoApp = 0xB01,
        kCmdHeaderFile = 0x0C0,
        kCmdHeaderBlock = 0x0C1,
        kCmdHeaderMmsg = 0x0C2,
        kCmdDataMmsg = 0x0C3,
        kCmdErase = 0x0CE,
        kCmdFinish = 0x0CF,
        kAnsError = 0xAC0,
        kAnsEraseOk = 0xAC1,
        kAnsMmsgOk = 0xAC2,
        kAnsBlockOk = 0xAC3,
        kAnsFinishOk = 0xACF,
    };

    static constexpr size_t kBlockSize = 16 * 1024;
    static constexpr size_t kMmsgMaxDataSize = 1792;
    static constexpr size_t kMsgDataSize = 7;
    static constexpr uint32_t kMotorAppAddress = 0x08008000;
    static constexpr uint32_t kRkAppAddress = 0x08008000;

    void run(CanBootloaderFlashRequest request, SendFn send);
    bool flashFile(const CanBootloaderFlashRequest& request, const SendFn& send);
    bool waitAck(uint16_t cmd, uint32_t timeoutMs);
    bool sendFrame(const SendFn& send, uint8_t mod, uint16_t cmd, uint8_t dst, const uint8_t data[8]);
    bool sendGoBoot(const SendFn& send, uint8_t dst);
    bool sendGoApp(const SendFn& send, uint8_t dst);
    void setStatus(const std::string& status);
    void setProgress(float progress);
    void logLine(const std::string& line);
    void finish(bool ok, const std::string& status);

    static size_t split(size_t size, size_t chunk);

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::map<uint16_t, uint32_t> ackSeen_;
    std::thread worker_;
    std::atomic<bool> cancel_{false};
    bool busy_ = false;
    uint8_t activeDevice_ = 0;
    float progress_ = 0.0f;
    std::string status_;
    std::string log_;
};

} // namespace drivescope
