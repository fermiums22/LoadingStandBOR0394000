#include "protocol/CanBootloaderFlasher.h"

#include "protocol/Crc32.h"
#include "protocol/PropCanFrame.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <vector>

namespace drivescope {

namespace {
// Slice 2a: PropCAN id build/decode helpers moved to protocol/PropCanFrame.
void putU16(uint8_t* dst, uint16_t value) { std::memcpy(dst, &value, sizeof(value)); }
void putU32(uint8_t* dst, uint32_t value) { std::memcpy(dst, &value, sizeof(value)); }
}

CanBootloaderFlasher::~CanBootloaderFlasher()
{
    cancel();
    if (worker_.joinable()) worker_.join();
}

bool CanBootloaderFlasher::start(const CanBootloaderFlashRequest& request, SendFn send)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (busy_) return false;
    }
    if (worker_.joinable()) worker_.join();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        busy_ = true;
        cancel_.store(false);
        progress_ = 0.0f;
        status_ = "starting";
        log_.clear();
        ackSeen_.clear();
        activeDevice_ = request.device == CanBootloaderFlashRequest::Device::Motor ? kAddrMotor : kAddrRk;
    }

    worker_ = std::thread([this, request, send = std::move(send)]() mutable {
        run(request, send);
    });
    return true;
}

void CanBootloaderFlasher::cancel()
{
    cancel_.store(true);
    cv_.notify_all();
}

void CanBootloaderFlasher::onFrame(const CanFrame& frame)
{
    if (!frame.extended) return;
    if (frame.rx == false) return;       // skip our own TX echo if dongle loops it back

    // Be permissive on mod + dst: the device-side prop_can.c builds replies as
    // (selfMode<<28) | (cmd<<16) | (sender_src<<8) | (selfAddr) and we only
    // really need to identify "this came from my target device, and it's an
    // ACK code I care about" — so match by src + cmd. Filtering on mod/dst
    // here would silently drop legitimate ACKs if the bus topology changes.
    const uint8_t src = propCanSrc(frame.id);
    const uint16_t cmd = propCanCmd(frame.id);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!busy_ || src != activeDevice_) return;
        ++ackSeen_[cmd];

        // Log every relevant frame to the flash log so it's visible in the UI
        // when something goes wrong.
        std::ostringstream os;
        os << "RX id=0x" << std::uppercase << std::hex << std::setw(8) << std::setfill('0')
           << frame.id << " cmd=0x" << std::setw(3) << cmd;
        if (cmd == kAnsError) {
            os << " ERROR data:";
            for (uint8_t b : frame.data) {
                os << ' ' << std::setw(2) << std::setfill('0') << int(b);
            }
        } else if (cmd == kAnsEraseOk) {
            os << " ERASE_OK";
        } else if (cmd == kAnsBlockOk) {
            os << " BLOCK_OK";
        } else if (cmd == kAnsMmsgOk) {
            os << " MMSG_OK";
        } else if (cmd == kAnsFinishOk) {
            os << " FINISH_OK";
        }
        log_ += os.str() + "\n";
        constexpr size_t kMaxLog = 16 * 1024;
        if (log_.size() > kMaxLog) {
            log_.erase(0, log_.size() - kMaxLog);
        }
    }
    cv_.notify_all();
}

CanBootloaderFlashSnapshot CanBootloaderFlasher::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    CanBootloaderFlashSnapshot s;
    s.busy = busy_;
    s.progress = progress_;
    s.status = status_;
    s.log = log_;
    return s;
}

void CanBootloaderFlasher::run(CanBootloaderFlashRequest request, SendFn send)
{
    const bool ok = flashFile(request, send);
    if (cancel_.load()) {
        finish(false, "cancelled");
    } else {
        finish(ok, ok ? "done" : "failed");
    }
}

bool CanBootloaderFlasher::flashFile(const CanBootloaderFlashRequest& request, const SendFn& send)
{
    const uint8_t target = request.device == CanBootloaderFlashRequest::Device::Motor ? kAddrMotor : kAddrRk;
    const uint8_t peer = request.device == CanBootloaderFlashRequest::Device::Motor ? kAddrRk : kAddrMotor;
    const uint32_t appAddress = request.device == CanBootloaderFlashRequest::Device::Motor
                              ? kMotorAppAddress
                              : kRkAppAddress;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        activeDevice_ = target;
    }

    if (request.filePath.empty()) {
        logLine("No firmware file selected");
        return false;
    }

    std::ifstream file(request.filePath, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        logLine("Cannot open firmware file: " + request.filePath);
        return false;
    }
    const size_t fileSize = static_cast<size_t>(file.tellg());
    file.seekg(0, std::ios::beg);
    if (fileSize == 0) {
        logLine("Firmware file is empty");
        return false;
    }

    {
        std::ostringstream os;
        os << "File: " << request.filePath << " (" << fileSize << " bytes)";
        logLine(os.str());
    }

    if (request.enterBoot) {
        setStatus("entering bootloader");
        logLine("APP GO_BOOT -> target (silences sibling so it doesn't talk on the bus)");
        // Send GO_BOOT to target AND the peer device — peer goes to boot too
        // so it stops broadcasting and sharing arbitration with us. This
        // mirrors the working main_pcb-side flasher behaviour.
        if (!sendGoBoot(send, target)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        (void)sendGoBoot(send, peer);
        // STM32 reset + bootloader CAN init takes ~700-900 ms in practice;
        // allow a comfortable headroom before the first BOOT-mode command.
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));

        // Drop any stale ACKs accumulated before GO_BOOT (e.g. echoes from
        // the dongle, leftover frames from a previous run).
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ackSeen_.clear();
        }
        logLine("Bootloader should be live now (1500 ms wait elapsed)");
    }

    setStatus("erasing flash");
    uint8_t data[8] = {};
    putU32(&data[0], appAddress);
    putU32(&data[4], static_cast<uint32_t>(fileSize));
    {
        std::ostringstream os;
        os << "TX ERASE addr=0x" << std::hex << std::uppercase << appAddress
           << " size=" << std::dec << fileSize;
        logLine(os.str());
    }
    if (!sendFrame(send, 0, kCmdErase, target, data)) return false;
    if (!waitAck(kAnsEraseOk, 5000)) {
        logLine("ERASE ACK timeout (bootloader didn't reply with 0xAC1 within 5 s)");
        logLine("Hints: confirm dongle is on the same bus + bitrate; confirm device actually rebooted into boot");
        return false;
    }
    logLine("ERASE OK");

    setStatus("sending file header");
    putU32(&data[0], appAddress);
    putU32(&data[4], static_cast<uint32_t>(fileSize));
    if (!sendFrame(send, 0, kCmdHeaderFile, target, data)) return false;

    std::vector<uint8_t> emptyBlock(kBlockSize, 0xFF);
    const uint32_t emptyBlockCrc = stm32WordCrc32(emptyBlock.data(), emptyBlock.size());
    std::vector<uint8_t> block(kBlockSize, 0xFF);
    const size_t blocks = split(fileSize, kBlockSize);

    for (uint16_t bi = 0; bi < blocks; ++bi) {
        if (cancel_.load()) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        const size_t off = size_t(bi) * kBlockSize;
        const size_t chunk = std::min(kBlockSize, fileSize - off);
        std::fill(block.begin(), block.end(), 0xFF);
        file.seekg(static_cast<std::streamoff>(off), std::ios::beg);
        file.read(reinterpret_cast<char*>(block.data()), static_cast<std::streamsize>(chunk));
        if (static_cast<size_t>(file.gcount()) != chunk) {
            logLine("File read error");
            return false;
        }

        const uint32_t blockCrc = stm32WordCrc32(block.data(), chunk);
        bool blockOk = false;
        for (int blockTry = 0; blockTry < 3 && !blockOk && !cancel_.load(); ++blockTry) {
            setStatus("sending block");
            putU32(&data[0], blockCrc);
            putU16(&data[4], bi);
            putU16(&data[6], static_cast<uint16_t>(blocks));
            if (!sendFrame(send, 0, kCmdHeaderBlock, target, data)) continue;

            if (blockCrc == emptyBlockCrc) {
                blockOk = waitAck(kAnsBlockOk, 400);
                continue;
            }

            bool mmsgError = false;
            const size_t totalMmsg = split(chunk, kMmsgMaxDataSize);
            for (uint16_t mi = 0; mi < totalMmsg && !cancel_.load(); ++mi) {
                const size_t mmsgOff = size_t(mi) * kMmsgMaxDataSize;
                const size_t mmsgChunk = std::min(kMmsgMaxDataSize, chunk - mmsgOff);
                const uint32_t mmsgCrc = stm32WordCrc32(block.data() + mmsgOff, mmsgChunk);
                bool mmsgOk = false;
                for (int mmsgTry = 0; mmsgTry < 3 && !mmsgOk && !cancel_.load(); ++mmsgTry) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    std::memset(data, 0, sizeof(data));
                    putU16(&data[0], static_cast<uint16_t>(mmsgChunk));
                    putU16(&data[2], static_cast<uint16_t>(mmsgCrc & 0xFFFFu));
                    putU16(&data[4], static_cast<uint16_t>(totalMmsg));
                    data[6] = 0x0F;
                    data[7] = static_cast<uint8_t>(mi);
                    if (!sendFrame(send, 0, kCmdHeaderMmsg, target, data)) continue;

                    const size_t totalMsg = split(mmsgChunk, kMsgDataSize);
                    for (uint16_t msg = 0; msg < totalMsg && !cancel_.load(); ++msg) {
                        const size_t msgOff = mmsgOff + size_t(msg) * kMsgDataSize;
                        const size_t msgSize = std::min(kMsgDataSize, mmsgChunk - size_t(msg) * kMsgDataSize);
                        std::memset(data, 0xFF, sizeof(data));
                        std::memcpy(&data[0], block.data() + msgOff, msgSize);
                        data[7] = static_cast<uint8_t>(msg);
                        if (!sendFrame(send, 0, kCmdDataMmsg, target, data)) {
                            logLine("Failed to send data frame");
                            return false;
                        }
                    }
                    mmsgOk = waitAck(kAnsMmsgOk, 400);
                }
                if (!mmsgOk) {
                    mmsgError = true;
                    logLine("MMSG ACK timeout");
                    break;
                }
                setProgress(100.0f * float(off + mmsgOff + mmsgChunk) / float(fileSize));
            }
            if (!mmsgError) blockOk = waitAck(kAnsBlockOk, 400);
        }

        if (!blockOk) {
            logLine("BLOCK ACK timeout");
            return false;
        }
    }

    setStatus("finishing");
    uint32_t fileCrc = 0xFFFFFFFFu;
    file.clear();
    file.seekg(0, std::ios::beg);
    std::vector<uint8_t> crcBuf(kBlockSize);
    size_t left = fileSize;
    while (left > 0) {
        const size_t toRead = std::min(kBlockSize, left);
        file.read(reinterpret_cast<char*>(crcBuf.data()), static_cast<std::streamsize>(toRead));
        if (static_cast<size_t>(file.gcount()) != toRead) {
            logLine("File CRC read error");
            return false;
        }
        fileCrc = stm32WordCrc32(crcBuf.data(), toRead, fileCrc);
        left -= toRead;
    }

    putU32(&data[0], fileCrc);
    std::memset(&data[4], 0xFF, 4);
    if (!sendFrame(send, 0, kCmdFinish, target, data)) return false;
    if (!waitAck(kAnsFinishOk, 5000)) {
        logLine("FINISH ACK timeout");
        return false;
    }
    setProgress(100.0f);
    logLine("FINISH OK");

    if (request.returnToApp) {
        logLine("BOOT GO_APP -> target and peer");
        (void)sendGoApp(send, target);
        (void)sendGoApp(send, peer);
    }
    return true;
}

bool CanBootloaderFlasher::waitAck(uint16_t cmd, uint32_t timeoutMs)
{
    std::unique_lock<std::mutex> lock(mutex_);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    return cv_.wait_until(lock, deadline, [&] {
        if (cancel_.load()) return true;
        auto it = ackSeen_.find(kAnsError);
        if (it != ackSeen_.end() && it->second > 0) return true;
        it = ackSeen_.find(cmd);
        return it != ackSeen_.end() && it->second > 0;
    }) && !cancel_.load() && [&] {
        auto err = ackSeen_.find(kAnsError);
        if (err != ackSeen_.end() && err->second > 0) {
            --err->second;
            return false;
        }
        auto ok = ackSeen_.find(cmd);
        if (ok == ackSeen_.end() || ok->second == 0) return false;
        --ok->second;
        return true;
    }();
}

bool CanBootloaderFlasher::sendFrame(const SendFn& send, uint8_t mod, uint16_t cmd, uint8_t dst, const uint8_t data[8])
{
    if (cancel_.load()) return false;
    CanFrame f;
    f.rx = false;
    f.extended = true;
    f.fd = false;
    f.brs = false;
    f.dlc = 8;
    f.id = packPropCanId(PropCanFrameId{mod, cmd, dst, kAddrPc});
    f.data.assign(data, data + 8);
    if (!send(f)) {
        logLine("Transport send failed");
        return false;
    }
    return true;
}

bool CanBootloaderFlasher::sendGoBoot(const SendFn& send, uint8_t dst)
{
    const uint8_t data[8] = {0xCC, 0xCC, 0xCC, 0xCC, 0xDD, 0xDD, 0xDD, 0xDD};
    return sendFrame(send, 1, kAppCmdGoBoot, dst, data);
}

bool CanBootloaderFlasher::sendGoApp(const SendFn& send, uint8_t dst)
{
    uint8_t data[8];
    std::memset(data, 0xFF, sizeof(data));
    return sendFrame(send, 0, kBootCmdGoApp, dst, data);
}

void CanBootloaderFlasher::setStatus(const std::string& status)
{
    std::lock_guard<std::mutex> lock(mutex_);
    status_ = status;
}

void CanBootloaderFlasher::setProgress(float progress)
{
    std::lock_guard<std::mutex> lock(mutex_);
    progress_ = std::clamp(progress, 0.0f, 100.0f);
}

void CanBootloaderFlasher::logLine(const std::string& line)
{
    std::lock_guard<std::mutex> lock(mutex_);
    log_ += line;
    log_ += '\n';
    constexpr size_t kMaxLog = 16 * 1024;
    if (log_.size() > kMaxLog) {
        log_.erase(0, log_.size() - kMaxLog);
    }
}

void CanBootloaderFlasher::finish(bool ok, const std::string& status)
{
    std::lock_guard<std::mutex> lock(mutex_);
    busy_ = false;
    if (ok) progress_ = 100.0f;
    status_ = status;
}

size_t CanBootloaderFlasher::split(size_t size, size_t chunk)
{
    return (size + chunk - 1) / chunk;
}

} // namespace drivescope
