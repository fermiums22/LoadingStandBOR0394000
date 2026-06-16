#include "cli/MotorBlobCli.h"

#include "can/CanFrame.h"
#include "can/SlcanParser.h"
#include "transport/ICanTransport.h"
#include "transport/SlcanTransport.h"
#include "transport/TcpCanTransport.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>

namespace drivescope {

namespace {

/* Mirror the wire constants from MainUi.cpp so the CLI test is independent
 * of any GUI state. Single block, single mmsg, 164 B blob. */
constexpr uint8_t  kAddrPc    = 0x10;
constexpr uint8_t  kAddrMotor = 0x02;

constexpr uint16_t kCmdReadHeaderFile   = 0x0D2;
constexpr uint16_t kCmdReadHeaderBlock  = 0x0D3;
constexpr uint16_t kCmdReadHeaderMmsg   = 0x0D4;
constexpr uint16_t kCmdReadDataMmsg     = 0x0D5;
constexpr uint16_t kCmdReadFileFinish   = 0x0DF;
constexpr uint16_t kAnsReadHeaderFileOk = 0xAD2;
constexpr uint16_t kAnsReadError        = 0xAD8;

constexpr uint16_t kCmdHeaderFile  = 0x0C0;
constexpr uint16_t kCmdHeaderBlock = 0x0C1;
constexpr uint16_t kCmdHeaderMmsg  = 0x0C2;
constexpr uint16_t kCmdDataMmsg    = 0x0C3;
constexpr uint16_t kCmdFileFinish  = 0x0CF;
constexpr uint16_t kAnsFileOk      = 0xACF;
constexpr uint16_t kAnsError       = 0xAC0;

constexpr uint32_t kMotorConfigFlashAddr = 0x080E0000u;
constexpr uint16_t kBlobBytes  = 164u;
constexpr uint16_t kPayloadOff = 16u;     /* first byte after header   */
constexpr uint16_t kPayloadLen = 148u;
constexpr uint16_t kPayloadCrcOff = 12u;  /* inside header             */
constexpr uint16_t kThetaOffsetOff = kPayloadOff + 0u;    /* float */

constexpr uint16_t kMmsgMaxDataSize = 1792u;
constexpr uint8_t  kMsgDataSize     = 7u;

/* PARAM (sub-cmd 0x02) — live RAM state read/write. Distinct from unified
 * READ which reads flash. Used to verify SET actually landed in RAM since
 * unified READ would show the old flash value until SAVE_ALL persists it. */
constexpr uint16_t kCmdConfig    = 0x005;  /* host -> motor sub-cmd dispatch */
constexpr uint16_t kAnsConfig    = 0x0AB;  /* motor -> host sub-cmd dispatch */
constexpr uint8_t  kSubcmdParam  = 0x02;
constexpr uint8_t  kParamActGet  = 0;
constexpr uint8_t  kParamActSet  = 1;
constexpr uint8_t  kParamActSave = 2;      /* SAVE_ALL -> flash */
constexpr uint16_t kParamIdThetaOffset = 0;

/* STM32 hardware-CRC peripheral replay (poly 0x04C11DB7, init 0xFFFFFFFF,
 * no reflection, no XOR-out, LE word pack, 0xFF tail pad). Used for the
 * wire-level integrity check; matches MainUi.cpp::motorStm32HwCrc32 and
 * motor FW propCanGetCrc byte-for-byte. */
uint32_t stm32HwCrc32(const uint8_t* data, std::size_t bytes)
{
    auto feed = [](uint32_t crc, uint32_t word) -> uint32_t {
        crc ^= word;
        for (int b = 0; b < 32; ++b) {
            crc = (crc & 0x80000000u) ? ((crc << 1) ^ 0x04C11DB7u) : (crc << 1);
        }
        return crc;
    };
    uint32_t crc = 0xFFFFFFFFu;
    const std::size_t whole = bytes / 4u;
    for (std::size_t i = 0; i < whole; ++i) {
        const uint8_t* p = data + i * 4u;
        const uint32_t w =
            uint32_t(p[0])         |
            (uint32_t(p[1]) <<  8) |
            (uint32_t(p[2]) << 16) |
            (uint32_t(p[3]) << 24);
        crc = feed(crc, w);
    }
    const std::size_t tail = bytes % 4u;
    if (tail != 0u) {
        const uint8_t* p = data + whole * 4u;
        const uint32_t w =
            uint32_t(p[0]) |
            (uint32_t((tail >= 2) ? p[1] : 0xFFu) <<  8) |
            (uint32_t((tail >= 3) ? p[2] : 0xFFu) << 16) |
            (uint32_t(0xFFu) << 24);
        crc = feed(crc, w);
    }
    return crc;
}

/* CRC32 IEEE 802.3 (reflected, init 0xFFFFFFFF, final XOR 0xFFFFFFFF) --
 * matches motor FW MotorConfigCrc32 in motor_config.c. Used for the
 * on-flash payload integrity check that goes into the blob header. */
uint32_t configPayloadCrc32(const uint8_t* data, std::size_t bytes)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < bytes; ++i) {
        crc ^= uint32_t(data[i]);
        for (int b = 0; b < 8; ++b) {
            const uint32_t lsb_mask = uint32_t(-int32_t(crc & 1u));
            crc = (crc >> 1) ^ (0xEDB88320u & lsb_mask);
        }
    }
    return ~crc;
}

std::unique_ptr<ICanTransport> makeTransport(const ToolConfig& config)
{
    if (config.transport == "slcan") {
        auto t = std::make_unique<SlcanTransport>();
        t->portName = config.slcan.port;
        t->baud = config.slcan.baud;
        t->nominalCommand = config.slcan.nominal;
        t->dataCommand = config.slcan.data;
        t->silentMode = config.slcan.silent;
        return t;
    }
    auto t = std::make_unique<TcpCanTransport>();
    if (config.transport == "wifi_ap") {
        t->host = config.wifiAp.host;
        t->port = config.wifiAp.port;
    } else {
        t->host = config.tcp.host;
        t->port = config.tcp.port;
    }
    return t;
}

CanFrame buildFrame(uint16_t cmd, const uint8_t payload[8])
{
    CanFrame f{};
    f.id = (uint32_t(1) << 28) |          /* mod = APP */
           (uint32_t(cmd) << 16) |
           (uint32_t(kAddrMotor) << 8) |
           uint32_t(kAddrPc);
    f.extended = true;
    f.rx = false;
    f.dlc = 8;
    f.data.assign(payload, payload + 8);
    return f;
}

/* Streamed READ accumulator. Drains the transport poll for up to
 * timeoutSec, ingesting motor's HEADER_FILE_OK / HEADER_BLOCK /
 * HEADER_MMSG / DATA_MMSG frames into buf. On FILE_FINISH validates the
 * file CRC and returns true. On ANS_READ_ERROR or timeout returns
 * false. */
bool collectReadStream(ICanTransport& transport,
                       uint8_t buf[kBlobBytes],
                       uint16_t& outSize,
                       uint32_t& motorCrc,
                       double timeoutSec,
                       std::string& err)
{
    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() +
        std::chrono::milliseconds(static_cast<int>(timeoutSec * 1000.0));

    std::memset(buf, 0, kBlobBytes);
    outSize = 0;
    motorCrc = 0;
    bool gotHeaderFileOk = false;
    uint16_t curMmsgIdx = 0;
    uint16_t curMmsgSize = 0;

    while (clock::now() < deadline) {
        CanFrame frame;
        while (transport.poll(frame)) {
            const uint8_t srcAddr = static_cast<uint8_t>(frame.id & 0xFFu);
            const uint8_t mod = static_cast<uint8_t>((frame.id >> 28) & 0x01u);
            const uint16_t cmd = static_cast<uint16_t>((frame.id >> 16) & 0x0FFFu);
            if (mod != 1u || srcAddr != kAddrMotor || frame.data.size() < 8) continue;

            switch (cmd) {
                case kAnsReadHeaderFileOk:
                    std::memcpy(&outSize, &frame.data[4], sizeof(uint16_t));
                    gotHeaderFileOk = true;
                    break;
                case kCmdReadHeaderBlock:
                    /* No state needed for single-block READ. */
                    break;
                case kCmdReadHeaderMmsg: {
                    uint16_t mmsg_size = 0;
                    std::memcpy(&mmsg_size, &frame.data[0], sizeof(uint16_t));
                    curMmsgIdx = frame.data[7];
                    curMmsgSize = mmsg_size;
                    break;
                }
                case kCmdReadDataMmsg: {
                    if (!gotHeaderFileOk) break;
                    const uint32_t mmsg_off =
                        static_cast<uint32_t>(curMmsgIdx) * kMmsgMaxDataSize;
                    const uint8_t msg_idx = frame.data[7];
                    const uint32_t msg_off = mmsg_off +
                        static_cast<uint32_t>(msg_idx) * kMsgDataSize;
                    const uint32_t bytes_used =
                        static_cast<uint32_t>(msg_idx) * kMsgDataSize;
                    const uint32_t remain = (curMmsgSize > bytes_used)
                        ? (curMmsgSize - bytes_used) : 0u;
                    const uint8_t used = static_cast<uint8_t>(
                        (remain >= kMsgDataSize) ? kMsgDataSize : remain);
                    if (msg_off + used <= kBlobBytes) {
                        std::memcpy(&buf[msg_off], &frame.data[0], used);
                    }
                    break;
                }
                case kCmdReadFileFinish: {
                    if (!gotHeaderFileOk) {
                        err = "FILE_FINISH before HEADER_FILE_OK";
                        return false;
                    }
                    std::memcpy(&motorCrc, &frame.data[0], sizeof(uint32_t));
                    return true;
                }
                case kAnsReadError: {
                    char tmp[32];
                    std::snprintf(tmp, sizeof(tmp), "motor READ_ERROR 0x%02X",
                                  static_cast<unsigned>(frame.data[0]));
                    err = tmp;
                    return false;
                }
                default: break;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    err = "READ stream timeout";
    return false;
}

/* Send unified READ HEADER_FILE for the motor config flash region and
 * collect the streamed response. */
bool unifiedRead(ICanTransport& transport,
                 uint8_t buf[kBlobBytes],
                 double timeoutSec,
                 std::string& err)
{
    uint8_t payload[8] = {};
    const uint32_t addr = kMotorConfigFlashAddr;
    const uint32_t size = kBlobBytes;
    std::memcpy(&payload[0], &addr, sizeof(addr));
    std::memcpy(&payload[4], &size, sizeof(size));
    if (!transport.send(buildFrame(kCmdReadHeaderFile, payload))) {
        err = "transport send (READ_HEADER_FILE) failed";
        return false;
    }

    uint16_t outSize = 0;
    uint32_t motorCrc = 0;
    if (!collectReadStream(transport, buf, outSize, motorCrc, timeoutSec, err)) {
        return false;
    }
    if (outSize != kBlobBytes) {
        char tmp[64];
        std::snprintf(tmp, sizeof(tmp),
                      "expected %u B, motor returned %u", kBlobBytes, outSize);
        err = tmp;
        return false;
    }
    const uint32_t hostCrc = stm32HwCrc32(buf, kBlobBytes);
    if (hostCrc != motorCrc) {
        char tmp[96];
        std::snprintf(tmp, sizeof(tmp),
                      "CRC mismatch: host=0x%08X motor=0x%08X", hostCrc, motorCrc);
        err = tmp;
        return false;
    }
    return true;
}

/* Stream 28 unified WRITE frames with 800 us inter-frame pacing -- mirrors
 * MainUi.cpp::sendMotorBlobSet. */
bool unifiedWrite(ICanTransport& transport,
                  const uint8_t buf[kBlobBytes],
                  double timeoutSec,
                  std::string& err)
{
    const uint32_t addr     = kMotorConfigFlashAddr;
    const uint32_t size     = kBlobBytes;
    const uint32_t fileCrc  = stm32HwCrc32(buf, kBlobBytes);
    const uint32_t blockCrc = fileCrc;
    const uint16_t mmsgCrc16 = static_cast<uint16_t>(fileCrc & 0xFFFFu);

    auto pump = [&](uint16_t cmd, const uint8_t p[8], bool pace) -> bool {
        if (!transport.send(buildFrame(cmd, p))) {
            char tmp[64];
            std::snprintf(tmp, sizeof(tmp), "transport send cmd 0x%03X failed", cmd);
            err = tmp;
            return false;
        }
        if (pace) std::this_thread::sleep_for(std::chrono::microseconds(800));
        return true;
    };

    uint8_t pl[8] = {};

    /* HEADER_FILE */
    std::memcpy(&pl[0], &addr, sizeof(addr));
    std::memcpy(&pl[4], &size, sizeof(size));
    if (!pump(kCmdHeaderFile, pl, true)) return false;

    /* HEADER_BLOCK */
    std::memset(pl, 0, 8);
    std::memcpy(&pl[0], &blockCrc, sizeof(blockCrc));
    pl[6] = 1; pl[7] = 0;
    if (!pump(kCmdHeaderBlock, pl, true)) return false;

    /* HEADER_MMSG */
    std::memset(pl, 0, 8);
    const uint16_t mmsgSize = kBlobBytes;
    const uint16_t mmsgTotal = 1;
    std::memcpy(&pl[0], &mmsgSize, sizeof(uint16_t));
    std::memcpy(&pl[2], &mmsgCrc16, sizeof(uint16_t));
    std::memcpy(&pl[4], &mmsgTotal, sizeof(uint16_t));
    pl[6] = 0x0F;
    pl[7] = 0;
    if (!pump(kCmdHeaderMmsg, pl, true)) return false;

    /* DATA_MMSG ×24 */
    constexpr uint8_t msgTotal = 24;
    for (uint8_t m = 0; m < msgTotal; ++m) {
        const uint16_t srcOff = static_cast<uint16_t>(m) * 7;
        const uint8_t used = (srcOff + 7 <= size)
            ? 7u : static_cast<uint8_t>(size - srcOff);
        std::memset(pl, 0xFF, 7);
        std::memcpy(pl, &buf[srcOff], used);
        pl[7] = m;
        if (!pump(kCmdDataMmsg, pl, m + 1 < msgTotal)) return false;
    }

    /* FILE_FINISH */
    std::memset(pl, 0, 8);
    std::memcpy(&pl[0], &fileCrc, sizeof(fileCrc));
    if (!pump(kCmdFileFinish, pl, false)) return false;

    /* Wait for ANS_GET_FILE_OK (cmd 0xACF) or ANS_ERROR (0xAC0). */
    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() +
        std::chrono::milliseconds(static_cast<int>(timeoutSec * 1000.0));
    while (clock::now() < deadline) {
        CanFrame frame;
        while (transport.poll(frame)) {
            const uint8_t srcAddr = static_cast<uint8_t>(frame.id & 0xFFu);
            const uint8_t mod = static_cast<uint8_t>((frame.id >> 28) & 0x01u);
            const uint16_t cmd = static_cast<uint16_t>((frame.id >> 16) & 0x0FFFu);
            if (mod != 1u || srcAddr != kAddrMotor) continue;
            if (cmd == kAnsFileOk) return true;
            if (cmd == kAnsError) {
                char tmp[64];
                std::snprintf(tmp, sizeof(tmp),
                              "motor ANS_ERROR 0x%02X",
                              frame.data.empty() ? 0u
                                  : static_cast<unsigned>(frame.data[0]));
                err = tmp;
                return false;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    err = "WRITE result timeout (no ANS_GET_FILE_OK or ANS_ERROR)";
    return false;
}

/* PARAM GET — reads a single field from motor's live RAM globals. The
 * unified READ goes through flash so it can't observe a fresh SET until
 * SAVE_ALL has persisted; PARAM GET hits MotorConfigGather and so reflects
 * the live FOC state. Returns the raw u32 value via outBits. */
bool paramGet(ICanTransport& transport, uint16_t id,
              uint32_t& outBits, double timeoutSec, std::string& err)
{
    uint8_t pl[8] = {};
    pl[0] = kSubcmdParam;
    pl[1] = kParamActGet;
    std::memcpy(&pl[2], &id, sizeof(id));
    /* data[4..7] = 0xFFFFFFFF per protocol "no value for GET" convention */
    pl[4] = pl[5] = pl[6] = pl[7] = 0xFFu;
    if (!transport.send(buildFrame(kCmdConfig, pl))) {
        err = "transport send (PARAM GET) failed";
        return false;
    }

    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() +
        std::chrono::milliseconds(static_cast<int>(timeoutSec * 1000.0));
    while (clock::now() < deadline) {
        CanFrame frame;
        while (transport.poll(frame)) {
            const uint8_t srcAddr = static_cast<uint8_t>(frame.id & 0xFFu);
            const uint8_t mod = static_cast<uint8_t>((frame.id >> 28) & 0x01u);
            const uint16_t cmd = static_cast<uint16_t>((frame.id >> 16) & 0x0FFFu);
            if (mod != 1u || srcAddr != kAddrMotor) continue;
            if (cmd != kAnsConfig || frame.data.size() < 8) continue;
            if (frame.data[0] != kSubcmdParam) continue;
            const uint16_t got_id = static_cast<uint16_t>(frame.data[2]) |
                                    (static_cast<uint16_t>(frame.data[3]) << 8);
            if (got_id != id) continue;
            const uint8_t status = frame.data[1];
            if (status != 0) {
                char tmp[64];
                std::snprintf(tmp, sizeof(tmp),
                              "PARAM GET status=%u for id=%u",
                              static_cast<unsigned>(status),
                              static_cast<unsigned>(id));
                err = tmp;
                return false;
            }
            std::memcpy(&outBits, &frame.data[4], sizeof(outBits));
            return true;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    err = "PARAM GET timeout";
    return false;
}

/* PARAM SAVE_ALL — gathers live RAM via MotorConfigGather, recomputes
 * payload CRC32, erases sector 11 and writes the fresh blob. Slow (~1.5s
 * on motor's STM32F405). Use sparingly to avoid flash wear. */
bool paramSaveAll(ICanTransport& transport, double timeoutSec, std::string& err)
{
    uint8_t pl[8] = {};
    pl[0] = kSubcmdParam;
    pl[1] = kParamActSave;
    /* id and value don't matter for SAVE_ALL */
    if (!transport.send(buildFrame(kCmdConfig, pl))) {
        err = "transport send (PARAM SAVE_ALL) failed";
        return false;
    }

    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() +
        std::chrono::milliseconds(static_cast<int>(timeoutSec * 1000.0));
    while (clock::now() < deadline) {
        CanFrame frame;
        while (transport.poll(frame)) {
            const uint8_t srcAddr = static_cast<uint8_t>(frame.id & 0xFFu);
            const uint8_t mod = static_cast<uint8_t>((frame.id >> 28) & 0x01u);
            const uint16_t cmd = static_cast<uint16_t>((frame.id >> 16) & 0x0FFFu);
            if (mod != 1u || srcAddr != kAddrMotor) continue;
            if (cmd != kAnsConfig || frame.data.size() < 8) continue;
            if (frame.data[0] != kSubcmdParam) continue;
            if (frame.data[1] == 0) return true;
            char tmp[64];
            std::snprintf(tmp, sizeof(tmp),
                          "PARAM SAVE_ALL status=%u",
                          static_cast<unsigned>(frame.data[1]));
            err = tmp;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    err = "PARAM SAVE_ALL timeout";
    return false;
}

void printDecoded(const uint8_t buf[kBlobBytes])
{
    uint32_t magic = 0, fwVer = 0, hdrCrc = 0;
    uint16_t payloadSize = 0, reserved = 0;
    std::memcpy(&magic,  &buf[0],  sizeof(magic));
    /* Header layout mirrors MotorConfigHeader_t:
     * magic u32, fw_version u32, payload_size u16, reserved u16,
     * payload_crc u32. */
    std::memcpy(&fwVer,       &buf[4], sizeof(fwVer));
    std::memcpy(&payloadSize, &buf[8], sizeof(payloadSize));
    std::memcpy(&reserved,    &buf[10], sizeof(reserved));
    std::memcpy(&hdrCrc,  &buf[kPayloadCrcOff], sizeof(hdrCrc));

    const uint32_t payloadCrcCheck =
        configPayloadCrc32(&buf[kPayloadOff], kPayloadLen);

    std::printf("  magic              = 0x%08X (expect 0xC0FEEDDA)\n", magic);
    std::printf("  cfg.fw_ver         = 0x%08X\n", fwVer);
    std::printf("  hdr.payload_size   = %u B\n", static_cast<unsigned>(payloadSize));
    std::printf("  hdr.reserved       = 0x%04X\n", static_cast<unsigned>(reserved));
    std::printf("  hdr.payload_crc    = 0x%08X\n", hdrCrc);
    std::printf("  computed payld crc = 0x%08X (%s)\n",
                payloadCrcCheck,
                (payloadCrcCheck == hdrCrc) ? "match" : "MISMATCH");

    float thetaOff = 0.0f;
    std::memcpy(&thetaOff, &buf[kThetaOffsetOff], sizeof(thetaOff));
    std::printf("  theta.offset       = %.6f rad\n", thetaOff);

    for (int s = 0; s < 6; ++s) {
        float v = 0.0f;
        std::memcpy(&v, &buf[kPayloadOff + 4 + s * 4], sizeof(v));
        std::printf("  theta.sector_%d    = %.6f rad\n", s, v);
    }
    const char* piNames[6] = {"kp_id","ki_id","kp_iq","ki_iq","kp_omega","ki_omega"};
    for (int i = 0; i < 6; ++i) {
        float v = 0.0f;
        std::memcpy(&v, &buf[kPayloadOff + 28 + i * 4], sizeof(v));
        std::printf("  pi.%-8s        = %.6f\n", piNames[i], v);
    }
    uint32_t pfc = 0;
    std::memcpy(&pfc, &buf[kPayloadOff + 84], sizeof(pfc));
    std::printf("  pfc.enabled        = %u\n", pfc);
}

} // namespace

int runMotorBlobCli(const ToolConfig& config,
                    const std::string& mode,
                    double newThetaRad,
                    double timeoutSec)
{
    std::unique_ptr<ICanTransport> transport = makeTransport(config);
    if (!transport->open()) {
        std::fprintf(stderr, "[motor-blob] transport open failed: %s\n",
                     transport->status().c_str());
        return 3;
    }
    std::printf("[motor-blob] connected: %s\n", transport->status().c_str());

    /* Drain any pre-existing frames from the bus so they don't get
     * mistaken for our READ stream answers. */
    {
        CanFrame f;
        int drained = 0;
        while (transport->poll(f) && drained < 1000) ++drained;
        if (drained > 0) {
            std::printf("[motor-blob] drained %d stale frame(s)\n", drained);
        }
    }

    uint8_t blob[kBlobBytes] = {};
    std::string err;

    /* ---- READ ---- */
    {
        using clock = std::chrono::steady_clock;
        const auto t0 = clock::now();
        if (!unifiedRead(*transport, blob, timeoutSec, err)) {
            std::fprintf(stderr, "[motor-blob] READ failed: %s\n", err.c_str());
            transport->close();
            return 4;
        }
        const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
            clock::now() - t0).count();
        std::printf("[motor-blob] READ ok (%lld ms)\n", static_cast<long long>(dt));
    }
    printDecoded(blob);

    if (mode == "get") {
        transport->close();
        return 0;
    }

    if (mode != "set-theta") {
        std::fprintf(stderr, "[motor-blob] unknown mode '%s' (expected get|set-theta)\n",
                     mode.c_str());
        transport->close();
        return 2;
    }

    /* ---- SET-THETA round-trip ---- */
    const float oldTheta = [&]() {
        float v = 0.0f;
        std::memcpy(&v, &blob[kThetaOffsetOff], sizeof(v));
        return v;
    }();
    const float newTheta = static_cast<float>(newThetaRad);
    std::printf("[motor-blob] WRITE: theta.offset %.6f -> %.6f rad\n",
                oldTheta, newTheta);

    std::memcpy(&blob[kThetaOffsetOff], &newTheta, sizeof(newTheta));
    const uint32_t newPayloadCrc =
        configPayloadCrc32(&blob[kPayloadOff], kPayloadLen);
    std::memcpy(&blob[kPayloadCrcOff], &newPayloadCrc, sizeof(newPayloadCrc));

    {
        using clock = std::chrono::steady_clock;
        const auto t0 = clock::now();
        if (!unifiedWrite(*transport, blob, timeoutSec, err)) {
            std::fprintf(stderr, "[motor-blob] WRITE failed: %s\n", err.c_str());
            transport->close();
            return 5;
        }
        const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
            clock::now() - t0).count();
        std::printf("[motor-blob] WRITE ok (%lld ms)\n", static_cast<long long>(dt));
    }

    /* ---- VERIFY 1: live RAM via PARAM GET ----
     *
     * Unified READ goes through MOTOR_CONFIG_FLASH_ADDR which is the
     * raw flash sector 11 -- the SET we just sent only updated live RAM
     * (motor FW's MotorConfigScatter), so re-READ would show old values
     * until SAVE_ALL persists. PARAM GET on id=0 (theta.offset) hits
     * the live RAM globals via MotorConfigGather and so reflects the
     * post-SET state immediately. This is the correct semantic check
     * for "did SET actually take effect on the motor?". */
    {
        uint32_t bits = 0;
        if (!paramGet(*transport, kParamIdThetaOffset, bits, timeoutSec, err)) {
            std::fprintf(stderr, "[motor-blob] PARAM GET (RAM verify) failed: %s\n",
                         err.c_str());
            transport->close();
            return 6;
        }
        float got = 0.0f;
        std::memcpy(&got, &bits, sizeof(got));
        const float diff = std::fabs(got - newTheta);
        std::printf("[motor-blob] verify (RAM): theta.offset = %.6f rad (diff %.2e)\n",
                    got, diff);
        if (diff > 1e-6f) {
            std::fprintf(stderr,
                         "[motor-blob] FAIL: WRITE did not land in RAM (want %.6f, got %.6f)\n",
                         newTheta, got);
            transport->close();
            return 7;
        }
    }

    /* Restore RAM to the original theta value so the test leaves no
     * lingering offset on the motor. Same WRITE protocol path; if the
     * first WRITE worked this one will too. We do NOT chain SAVE_ALL --
     * flash sector 11 still has the original value so a power cycle would
     * restore it anyway; avoiding the flash erase keeps the test cheap
     * and the operator's flash-write budget healthy. */
    std::memcpy(&blob[kThetaOffsetOff], &oldTheta, sizeof(oldTheta));
    const uint32_t restoreCrc =
        configPayloadCrc32(&blob[kPayloadOff], kPayloadLen);
    std::memcpy(&blob[kPayloadCrcOff], &restoreCrc, sizeof(restoreCrc));
    if (!unifiedWrite(*transport, blob, timeoutSec, err)) {
        std::fprintf(stderr, "[motor-blob] RESTORE WRITE failed: %s\n", err.c_str());
        transport->close();
        return 8;
    }
    {
        uint32_t bits = 0;
        if (paramGet(*transport, kParamIdThetaOffset, bits, timeoutSec, err)) {
            float got = 0.0f;
            std::memcpy(&got, &bits, sizeof(got));
            std::printf("[motor-blob] restored: theta.offset = %.6f rad\n", got);
        }
    }

    std::printf("[motor-blob] PASS: round-trip OK (RAM updated by WRITE)\n");
    std::printf("[motor-blob] note: re-READ via unified protocol shows flash, not RAM.\n");
    std::printf("[motor-blob] note: chain PARAM SAVE_ALL to persist to sector 11.\n");
    transport->close();
    return 0;
}

} // namespace drivescope
