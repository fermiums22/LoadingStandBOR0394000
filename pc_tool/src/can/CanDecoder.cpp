#include "can/CanDecoder.h"
#include "protocol/DebugProtocol.h"

#include <cstring>
#include <iomanip>
#include <initializer_list>
#include <sstream>
#include <utility>

namespace drivescope {

namespace {
constexpr uint16_t kMotorCmdConfig = 0x005;
constexpr uint16_t kMotorAnsConfig = 0x0AB;
constexpr uint8_t kMotorCfgThetaOffset = 0x00;
constexpr uint8_t kMotorCfgThetaSector = 0x01;
constexpr uint8_t kMotorCfgParam       = 0x02;

const char* motorParamActionName(uint8_t action)
{
    switch (action) {
        case 0:  return "GET";
        case 1:  return "SET";
        case 2:  return "SAVE_ALL";
        case 3:  return "LOAD_ALL";
        case 4:  return "RESET_DEFAULTS";
        case 5:  return "GET_COUNT";
        case 6:  return "GET_META";
        case 7:  return "GET_DEFAULT";
        case 8:  return "GET_MIN";
        case 9:  return "GET_MAX";
        case 10: return "GET_NAME_CHUNK";
        default: return "?";
    }
}

const char* motorParamIdName(uint16_t id)
{
    switch (id) {
        case 0:  return "theta.offset";
        case 1:  return "theta.sector_0";
        case 2:  return "theta.sector_1";
        case 3:  return "theta.sector_2";
        case 4:  return "theta.sector_3";
        case 5:  return "theta.sector_4";
        case 6:  return "theta.sector_5";
        case 10: return "pi.kp_id";
        case 11: return "pi.ki_id";
        case 12: return "pi.kp_iq";
        case 13: return "pi.ki_iq";
        case 14: return "pi.kp_omega";
        case 15: return "pi.ki_omega";
        case 20: return "profiler.rs";
        case 21: return "profiler.ld";
        case 22: return "profiler.lq";
        case 23: return "profiler.lambda";
        case 24: return "profiler.j";
        case 25: return "profiler.b";
        case 26: return "profiler.valid";
        case 27: return "profiler.uptime";
        case 30: return "diag.brk_live";
        case 31: return "diag.brk_low_cnt";
        case 32: return "diag.brk_at_flt";
        case 33: return "diag.brk_flt_cnt";
        case 34: return "diag.wdog_hits";
        case 35: return "diag.wdog_persist_ms";
        case 36: return "diag.wdog_spurious_brk";
        case 37: return "cfg.fw_version";
        case 38: return "cfg.crc";
        case 50: return "pfc.enabled";
        default: return "?";
    }
}

const char* motorParamGroupName(uint8_t g)
{
    switch (g) {
        case 0: return "THETA";
        case 1: return "PI";
        case 2: return "PROFILER";
        case 3: return "LIMITS";
        case 4: return "DIAG";
        case 5: return "HW";       // pfc.enabled
        case 6: return "CFG";      // cfg.fw_version / cfg.crc
        default: return "?";
    }
}

const char* motorParamTypeName(uint8_t t)
{
    switch (t) {
        case 0: return "f32";
        case 1: return "u32";
        default: return "?";
    }
}

// CFG sub-cmd 0x03 BLOB action codes — see app_motor.h AppMotorBlobAction_e.
// Total blob = 164 bytes streamed in 33 chunks of 5 bytes each.
const char* motorBlobActionName(uint8_t action)
{
    switch (action) {
        case 0: return "GET";
        case 1: return "GET_RESP_CHUNK";
        case 2: return "SET_CHUNK";
        case 3: return "SET_RESULT";
        default: return "?";
    }
}

// Top-level CFG sub-cmd byte (data[0]) for cmd 0x005 / 0x0AB.
const char* motorCfgSubcmdName(uint8_t sub)
{
    switch (sub) {
        case 0x00: return "THETA_OFFSET";
        case 0x01: return "THETA_SECTOR";
        case 0x02: return "PARAM";
        case 0x03: return "BLOB";
        default:   return "?";
    }
}

// WiFi mode byte used by MAIN_CMD_SET_WIFI_MODE (0x100, host → MAIN) and
// MAIN_ANS_WIFI_STATUS (0xB00, MAIN → broadcast).
const char* mainWifiModeName(uint8_t mode)
{
    switch (mode) {
        case 0: return "OFF";
        case 1: return "STA";
        case 2: return "AP";
        default: return "?";
    }
}

const char* nodeName(uint8_t id)
{
    switch (id) {
        case 0x01: return "MAIN";
        case 0x02: return "MOTOR";
        case 0x03: return "RK";
        case 0x04: return "ESP32";
        case 0x10: return "PC";
        case 0xFF: return "BROADCAST";
        default: return "?";
    }
}

std::string nodeLabel(uint8_t id)
{
    const char* name = nodeName(id);
    std::ostringstream os;
    os << name;
    if (name[0] == '?') {
        os << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
           << static_cast<int>(id) << std::dec << std::setfill(' ');
    }
    return os.str();
}

const char* modeName(uint8_t mod)
{
    return mod == 0 ? "BOOT" : "APP";
}

std::string fileTransferName(uint16_t cmd)
{
    switch (cmd) {
        case 0x0C0: return "FT header_file";
        case 0x0C1: return "FT block_crc";
        case 0x0C2: return "FT mmsg_header";
        case 0x0C3: return "FT mmsg_data";
        case 0x0CE: return "FT erase";
        case 0x0CF: return "FT finish";
        case 0xAC0: return "FT/BOOT error";
        case 0xAC1: return "FT erase_ok";
        case 0xAC2: return "FT mmsg_ok";
        case 0xAC3: return "FT block_ok";
        case 0xACF: return "FT finish_ok";
        /* Unified file-transfer READ protocol (motor app, 2026-05-13).
         * Mirror of FT write-side: host sends 0xD2 READ_HEADER_FILE,
         * MCU acks 0xAD2 then streams 0xD3 block / 0xD4 mmsg / 0xD5
         * data / 0xDF finish back. Lives in app-mode (mod=1) so does
         * NOT collide with bootloader FT in mod=0. See
         * docs/can-unified-file-transfer.md. */
        case 0x0D2: return "RD header_file";
        case 0x0D3: return "RD block_crc";
        case 0x0D4: return "RD mmsg_header";
        case 0x0D5: return "RD mmsg_data";
        case 0x0DF: return "RD finish";
        case 0xAD2: return "RD header_ok";
        case 0xAD3: return "RD block_ok";
        case 0xAD4: return "RD mmsg_ok";
        case 0xAD8: return "RD error";
        case 0xADF: return "RD finish_ok";
        default: return {};
    }
}

std::string bootCmdName(uint16_t cmd)
{
    switch (cmd) {
        case 0xB00: return "BOOT go_to_boot";
        case 0xB01: return "BOOT go_to_app";
        case 0xFFF: return "BOOT ping_req";
        case 0x0FF: return "BOOT ping_rsp";
        default: return fileTransferName(cmd);
    }
}

std::string motorCmdName(uint16_t cmd, bool srcIsMotor)
{
    switch (cmd) {
        case 0x000: return "MTR reset";
        case 0x001: return "MTR go_boot";
        case 0x002: return srcIsMotor ? "MTR ctrl/status" : "MTR ctrl";
        case 0x003: return "MTR emergency_stop";
        case 0x004: return "MTR pfc_control";
        case 0x005: return "MTR cfg";
        case 0x008: return "MTR tone";
        case 0x0E2: return "MTR stream_add_req";
        case 0x0E3: return "MTR stream_add_rsp";
        case 0x0E4: return "MTR stream_remove";
        case 0x0E5: return "MTR stream_clear";
        case 0x0E6: return "MTR stream_status_req";
        case 0x0E7: return "MTR stream_status_rsp";
        case 0x0E8: return "MTR stream_value";
        case 0x0E9: return "MTR stream_heartbeat";
        case 0x0A0: return "MTR mode/error";
        case 0x0A1: return "MTR power_W";
        case 0x0A2: return "MTR Uq_V";
        case 0x0A3: return "MTR rpm";
        case 0x0A4: return "MTR Udc_V";
        case 0x0A5: return "MTR temp";
        case 0x0A6: return "MTR fault";
        case 0x0A7: return "MTR CAN diag";
        case 0x0A8: return "MTR sys diag";
        case 0x0A9: return "MTR cur diag";
        case 0x0AA: return "MTR pfc_state";
        case 0x0AB: return "MTR cfg_rsp";
        case 0x0AC: return "MTR profiler";
        case 0x0AD: return "MTR theta_cal";
        case 0x0AE: return "MTR sched_diag";
        case 0x0AF: return "MTR sched_diag2";
        case 0x0FF: return "MTR ping_rsp";
        case 0xFFF: return "MTR ping_req";
        default: return fileTransferName(cmd);
    }
}

// Commands directed at MAIN (mainPCB / SSD202D) and answers it broadcasts.
// Used when src or dst is MAIN and not handled by motor/RK matchers above.
std::string mainCmdName(uint16_t cmd, bool srcIsMain)
{
    switch (cmd) {
        case 0x010: return "MAIN diag_mode_set";
        case 0x011: return "MAIN diag_heartbeat";
        case 0x012: return "MAIN motor_proxy";
        case 0x013: return "MAIN rk_proxy";
        case 0x014: return "MAIN pfc_proxy";
        case 0x100: return "MAIN set_wifi_mode";
        case 0xB00: return "MAIN ans_wifi_status";
        case 0xB10: return "MAIN ans_diag_status";
        case 0xB11: return "MAIN ans_diag_telemetry";
        case 0x0FF: return "MAIN ping_rsp";
        case 0xFFF: return "MAIN ping_req";
        default: (void)srcIsMain; return fileTransferName(cmd);
    }
}

std::string rkCmdName(uint16_t cmd, bool srcIsRk)
{
    switch (cmd) {
        case 0x000: return "RK reset";
        case 0x001: return "RK go_boot";
        case 0x002: return srcIsRk ? "RK motor_mode" : "RK led";
        case 0x003: return srcIsRk ? "RK emergency_stop" : "RK bracket_servo";
        case 0x004: return "RK motorlock_servo";
        case 0x005: return "RK motor_ctrl";
        case 0x0A0: return "RK switch";
        case 0x0A1: return "RK angles";
        case 0x0FF: return "RK ping_rsp";
        case 0xFFF: return "RK ping_req";
        default: return fileTransferName(cmd);
    }
}

bool hasBytes(const std::vector<uint8_t>& data, size_t offset, size_t count)
{
    return offset <= data.size() && count <= data.size() - offset;
}

float readFloat(const std::vector<uint8_t>& data, size_t offset)
{
    if (!hasBytes(data, offset, 4)) return 0.0f;
    float v = 0.0f;
    std::memcpy(&v, data.data() + offset, sizeof(v));
    return v;
}

uint16_t readU16(const std::vector<uint8_t>& data, size_t offset)
{
    if (!hasBytes(data, offset, 2)) return 0;
    return static_cast<uint16_t>(data[offset]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[offset + 1]) << 8);
}

uint32_t readU32(const std::vector<uint8_t>& data, size_t offset)
{
    if (!hasBytes(data, offset, 4)) return 0;
    return static_cast<uint32_t>(data[offset]) |
           (static_cast<uint32_t>(data[offset + 1]) << 8) |
           (static_cast<uint32_t>(data[offset + 2]) << 16) |
           (static_cast<uint32_t>(data[offset + 3]) << 24);
}

std::string versionText(const std::vector<uint8_t>& data, size_t offset)
{
    if (!hasBytes(data, offset, 4)) return {};
    // Motor / RK firmware stores the version as a uint32_t literal
    // (e.g. VER_APPLICATION = 0x00020008 in motor's main.c) and ships
    // its raw memory image over CAN via memcpy(buf, &ver, 4). On a
    // little-endian core that puts byte[0] = LSB = build, byte[3] =
    // MSB = major. So to render the dotted-quad in human "M.m.p.b"
    // order we read the bytes from offset+3 down to offset+0.
    std::ostringstream os;
    os << static_cast<int>(data[offset + 3]) << '.'
       << static_cast<int>(data[offset + 2]) << '.'
       << static_cast<int>(data[offset + 1]) << '.'
       << static_cast<int>(data[offset + 0]);
    return os.str();
}

void appendHex32(std::ostringstream& os, uint32_t value)
{
    os << "0x" << std::uppercase << std::hex << std::setw(8) << std::setfill('0')
       << value << std::dec << std::setfill(' ');
}

void appendFileTransferPayload(std::ostringstream& os, uint16_t cmd, const std::vector<uint8_t>& data)
{
    if (!hasBytes(data, 0, 8)) return;
    switch (cmd) {
        case 0x0C0:
            os << " addr=";
            appendHex32(os, readU32(data, 0));
            os << " size=" << readU32(data, 4);
            break;
        case 0x0C1:
            os << " crc=";
            appendHex32(os, readU32(data, 0));
            os << " block=" << readU16(data, 4) << "/" << readU16(data, 6);
            break;
        case 0x0C2:
            os << " mmsg_size=" << readU16(data, 0)
               << " crc16=0x" << std::uppercase << std::hex << std::setw(4) << std::setfill('0')
               << readU16(data, 2) << std::dec << std::setfill(' ')
               << " total=" << readU16(data, 4)
               << " idx=" << static_cast<int>(data[7]);
            break;
        case 0x0C3:
            os << " idx=" << static_cast<int>(data[7]);
            break;
        case 0x0CE:
            os << " erase_addr=";
            appendHex32(os, readU32(data, 0));
            os << " len=" << readU32(data, 4);
            break;
        case 0x0CF:
            os << " crc=";
            appendHex32(os, readU32(data, 0));
            break;
        /* Unified READ protocol: same payload shapes as WRITE side. */
        case 0x0D2:    // host → motor: addr + size
        case 0xAD2:    // motor → host: addr + capped_size echo
            os << " addr=";
            appendHex32(os, readU32(data, 0));
            os << " size=" << readU32(data, 4);
            break;
        case 0x0D3:    // motor → host: HEADER_BLOCK (same as 0xC1)
            os << " crc=";
            appendHex32(os, readU32(data, 0));
            os << " block=" << readU16(data, 4) << "/" << readU16(data, 6);
            break;
        case 0x0D4:    // motor → host: HEADER_MMSG (same as 0xC2)
            os << " mmsg_size=" << readU16(data, 0)
               << " crc16=0x" << std::uppercase << std::hex << std::setw(4) << std::setfill('0')
               << readU16(data, 2) << std::dec << std::setfill(' ')
               << " total=" << readU16(data, 4)
               << " idx=" << static_cast<int>(data[7]);
            break;
        case 0x0D5:    // motor → host: DATA_MMSG (same as 0xC3)
            os << " idx=" << static_cast<int>(data[7]);
            break;
        case 0x0DF:    // motor → host: FILE_FINISH (file CRC)
            os << " crc=";
            appendHex32(os, readU32(data, 0));
            break;
        case 0xAD3:    // host → motor: BLOCK_OK ack (reserved for v2)
        case 0xAD4:    // host → motor: MMSG_OK ack (reserved for v2)
            os << " idx=" << static_cast<int>(data[1]);
            break;
        case 0xAD8:    // either direction: ERROR
            os << " err=0x" << std::uppercase << std::hex
               << static_cast<int>(data[0]) << std::dec;
            break;
        default:
            break;
    }
}

// Names match motor firmware struct APP_FAULT_T (events.h:109), all uppercase
// to read like the documentation's flag list.
const char* const kMotorFaultNames[32] = {
    "I2T", "OVERHEAT_MOTOR", "OVERHEAT_DRV", "CAN_SILENCE", "OVERVOLT",
    "OC_U", "OC_V", "OC_W", "ESTOP",
    "CAN_BUS_OFF", "CAN_PASSIVE", "CAN_ERR_WARN", "CAN_LEC",
    "CAN_RX_OVR", "CAN_TX_TO", "CAN_TX_FULL", "CAN_RESTART",
    "ADC_STUCK", "PHASE_SUM_IMBAL", "VDC_UV", "HALL_INVALID",
    "OMEGA_NAN", "THETA_NAN", "OFFSET_CAL_TO",
    "HARDFAULT", "MEMFAULT", "BUSFAULT", "USAGEFAULT", "NMI",
    "TIM1_BREAK", "RESET_UNEXPECTED", "INIT_DEGRADED"
};

std::string decodeMotorFaultBits(uint32_t mask)
{
    if (mask == 0) return std::string("none");
    std::string out;
    for (int b = 0; b < 32; ++b) {
        if (mask & (1u << b)) {
            if (!out.empty()) out += ',';
            out += kMotorFaultNames[b];
        }
    }
    return out;
}

const char* motorThetaActionName(uint8_t action)
{
    switch (action) {
        case 0: return "GET";
        case 1: return "SET";
        case 2: return "SET_SAVE";
        case 3: return "CAL";
        case 4: return "CAL_SAVE";
        case 5: return "SAVE";
        case 6: return "CLEAR";
        default: return "?";
    }
}

const char* motorThetaStatusName(uint8_t status)
{
    switch (status) {
        case 0: return "OK";
        case 1: return "BAD_ARG";
        case 2: return "BUSY";
        case 3: return "NOT_READY";
        case 4: return "STORAGE_EMPTY";
        case 5: return "STORAGE_FAIL";
        case 6: return "FAULT";
        case 7: return "RUNNING";
        default: return "?";
    }
}

// Render one-line text payload for cmd 0x005 (request) / 0x0AB (response)
// sub-cmds. Used by both the "to motor" and "from motor" branches below.
void appendMotorCfgPayload(std::ostringstream& os, const CanFrame& frame, bool srcIsMotor)
{
    const auto& d = frame.data;
    if (!hasBytes(d, 0, 8)) return;
    const uint8_t sub = d[0];
    switch (sub) {
        case kMotorCfgThetaOffset:
            if (srcIsMotor) {
                // Response: data[1]=status, data[2..5]=theta f32, data[6]=flags, data[7]=driverMode
                os << " theta status=" << motorThetaStatusName(d[1])
                   << " offset=" << readFloat(d, 2) << "rad"
                   << " flags=0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
                   << static_cast<int>(d[6]) << std::dec << std::setfill(' ')
                   << " mode=" << static_cast<int>(d[7]);
            } else {
                // Request: data[1..4]=value f32, data[5]=action, data[6..7]=guard
                os << " theta action=" << motorThetaActionName(d[5])
                   << " value=" << readFloat(d, 1)
                   << " guard=" << static_cast<int>(d[6]) << "/" << static_cast<int>(d[7]);
            }
            break;
        case kMotorCfgThetaSector:
            // Legacy sub-cmd kept for compatibility; both directions use the
            // sector index at data[1].
            os << " theta_sector idx=" << static_cast<int>(d[1]);
            if (srcIsMotor) os << " value=" << readFloat(d, 2);
            break;
        case kMotorCfgParam: {
            // Request:  data[1]=action, data[2..3]=id u16 LE, data[4..7]=value
            // Response: data[1]=status, data[2..3]=id u16 LE echo, data[4..7]=value
            const uint16_t id = readU16(d, 2);
            if (srcIsMotor) {
                os << " PARAM status=" << motorThetaStatusName(d[1])
                   << " id=" << id << "[" << motorParamIdName(id) << "]"
                   << " v=" << readFloat(d, 4)
                   << "/0x" << std::uppercase << std::hex << std::setw(8) << std::setfill('0')
                   << readU32(d, 4) << std::dec << std::setfill(' ');
            } else {
                const uint8_t action = d[1];
                os << " PARAM " << motorParamActionName(action)
                   << " id=" << id << "[" << motorParamIdName(id) << "]";
                if (action == 1 /* SET */) {
                    os << " v=" << readFloat(d, 4)
                       << "/0x" << std::uppercase << std::hex << std::setw(8) << std::setfill('0')
                       << readU32(d, 4) << std::dec << std::setfill(' ');
                } else if (action == 10 /* GET_NAME_CHUNK */) {
                    os << " chunk=" << readU16(d, 6);
                }
            }
            break;
        }
        case 0x03: {
            // BLOB (whole motor-config staged transfer, 164 B / 33 chunks).
            // Request: data[1]=action, data[2]=chunk_idx, data[3..7]=5B chunk.
            // Response: same shape, plus terminal SET_RESULT carries computed
            // CRC32 in data[4..7] and status in data[3].
            const uint8_t action = d[1];
            const uint8_t chunkIdx = d[2];
            os << " BLOB " << motorBlobActionName(action)
               << " chunk=" << static_cast<int>(chunkIdx) << "/"
               << "32";
            if (action == 3 /* SET_RESULT */) {
                os << " status=" << motorThetaStatusName(d[3])
                   << " crc=0x" << std::uppercase << std::hex << std::setw(8) << std::setfill('0')
                   << readU32(d, 4) << std::dec << std::setfill(' ');
            }
            break;
        }
        default:
            os << " subcmd=" << motorCfgSubcmdName(sub)
               << "(0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
               << static_cast<int>(sub) << std::dec << std::setfill(' ') << ")";
            break;
    }
}

void appendMotorPayload(std::ostringstream& os, uint16_t cmd, const CanFrame& frame, bool srcIsMotor)
{
    const auto& d = frame.data;
    // --- Host → motor ---
    if (!srcIsMotor) {
        switch (cmd) {
            case 0x000: // RESET
                // Magic bytes from the bootloader-style boot/app handshake.
                if (hasBytes(d, 0, 8)) {
                    os << " magic=0x" << std::uppercase << std::hex << std::setw(8) << std::setfill('0')
                       << readU32(d, 0) << "/0x" << std::setw(8) << readU32(d, 4)
                       << std::dec << std::setfill(' ');
                }
                return;
            case 0x001: // GO_TO_BOOT — same magic-handshake pattern
                if (hasBytes(d, 0, 8)) {
                    os << " magic=0x" << std::uppercase << std::hex << std::setw(8) << std::setfill('0')
                       << readU32(d, 0) << "/0x" << std::setw(8) << readU32(d, 4)
                       << std::dec << std::setfill(' ');
                }
                return;
            case 0x002: // CTRL: data[0]=driverMode, data[1..4]=value f32
                if (hasBytes(d, 1, 4)) {
                    os << " mode=" << static_cast<int>(d[0]) << " set=" << readFloat(d, 1) << "%";
                }
                return;
            case 0x003: // EMERGENCY_STOP: payload is all 0xFF guard.
                return;
            case 0x004: // PFC_CONTROL: data[0]=0/1 enable
                if (hasBytes(d, 0, 1)) os << " enable=" << static_cast<int>(d[0]);
                return;
            case kMotorCmdConfig:
                appendMotorCfgPayload(os, frame, /*srcIsMotor=*/false);
                return;
            case 0x008: // TONE: data[0..1]=freq_hz, data[2..3]=dur_ms, data[4]=amp%
                if (hasBytes(d, 0, 5)) {
                    os << " freq=" << readU16(d, 0) << "Hz"
                       << " dur=" << readU16(d, 2) << "ms"
                       << " amp=" << static_cast<int>(d[4]) << "%";
                }
                return;
            case 0x0FF: // PING request: payload all 0xFF
                return;
            default:
                return;
        }
    }
    // --- Motor → host ---

    switch (cmd) {
        case 0x0A0:
            if (hasBytes(d, 0, 1)) os << " mode=" << static_cast<int>(d[0]);
            break;
        case 0x0A1:
            if (hasBytes(d, 1, 4)) os << " mode=" << static_cast<int>(d[0]) << " P=" << readFloat(d, 1) << "W";
            break;
        case 0x0A2:
            if (hasBytes(d, 1, 4)) os << " mode=" << static_cast<int>(d[0]) << " Uq=" << readFloat(d, 1) << "V";
            break;
        case 0x0A3:
            if (hasBytes(d, 1, 4)) os << " mode=" << static_cast<int>(d[0]) << " rpm=" << readFloat(d, 1);
            break;
        case 0x0A4:
            if (hasBytes(d, 1, 4)) os << " mode=" << static_cast<int>(d[0]) << " Udc=" << readFloat(d, 1) << "V";
            break;
        case 0x0A5:
            if (hasBytes(d, 0, 8)) os << " vt=" << readFloat(d, 0) << " motor=" << readFloat(d, 4);
            break;
        case 0x0A6:
            if (hasBytes(d, 0, 8)) {
                const uint32_t mask = readU32(d, 1);
                os << " " << decodeMotorFaultBits(mask)
                   << " (rec=" << static_cast<int>(d[5])
                   << " tec=" << static_cast<int>(d[6]) << ")";
            }
            break;
        case 0x0A7:
            if (hasBytes(d, 0, 8)) {
                os << " mode=" << static_cast<int>(d[0])
                   << " lec=" << static_cast<int>(d[1])
                   << " restart=" << readU16(d, 2)
                   << " rx_ovr=" << readU16(d, 4)
                   << " tx_drop=" << readU16(d, 6);
            }
            break;
        case 0x0A8:
            if (hasBytes(d, 0, 8)) {
                os << " mode=" << static_cast<int>(d[0])
                   << " reset=0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
                   << static_cast<int>(d[1]) << std::dec << std::setfill(' ')
                   << " exc=" << static_cast<int>(d[2])
                   << " tim1=" << static_cast<int>(d[3])
                   << " oc=[" << static_cast<int>(d[4]) << ','
                   << static_cast<int>(d[5]) << ','
                   << static_cast<int>(d[6]) << "]"
                   << " i2t=" << static_cast<int>(d[7]);
            }
            break;
        case 0x0A9:
            if (hasBytes(d, 0, 8)) {
                const uint8_t status = d[0];
                const float filt = readFloat(d, 1);
                const float step = 0.2f;
                const float u_off = static_cast<float>(static_cast<int8_t>(d[5])) * step;
                const float v_off = static_cast<float>(static_cast<int8_t>(d[6])) * step;
                const float w_off = static_cast<float>(static_cast<int8_t>(d[7])) * step;
                os << " mode=" << static_cast<int>(status & 0x0F)
                   << " cal=" << ((status & 0x10) ? 1 : 0)
                   << " run=" << ((status & 0x20) ? 1 : 0)
                   << " align=" << ((status & 0x40) ? 1 : 0)
                   << " sum_filt=" << filt << "A"
                   << " off=[U=" << u_off << ",V=" << v_off << ",W=" << w_off << "]A";
            }
            break;
        case 0x0AA:
            if (hasBytes(d, 0, 2)) {
                os << " mode=" << static_cast<int>(d[0])
                   << " pfc=" << static_cast<int>(d[1]);
            }
            break;
        case kMotorAnsConfig:
            appendMotorCfgPayload(os, frame, /*srcIsMotor=*/true);
            break;
        case 0x0FF: {
            const std::string boot = versionText(d, 0);
            const std::string app = versionText(d, 4);
            if (!boot.empty() && !app.empty()) os << " boot=" << boot << " app=" << app;
            break;
        }
        default:
            break;
    }
}

void appendRkPayload(std::ostringstream& os, uint16_t cmd, const CanFrame& frame, bool srcIsRk)
{
    const auto& d = frame.data;
    if (!srcIsRk) {
        switch (cmd) {
            case 0x002:
                if (hasBytes(d, 0, 1)) os << " led=" << static_cast<int>(d[0]);
                break;
            case 0x003:
            case 0x004:
                if (hasBytes(d, 0, 5)) os << " set=" << readFloat(d, 0) << "% pwm=" << static_cast<int>(d[4]);
                break;
            case 0x005:
                if (hasBytes(d, 0, 1)) os << " subcmd=" << static_cast<int>(d[0]);
                break;
            default:
                break;
        }
        return;
    }

    switch (cmd) {
        case 0x0A0:
            if (hasBytes(d, 0, 8)) {
                os << " enc=" << static_cast<int>(d[0])
                   << " flags=0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
                   << static_cast<int>(d[1]) << std::dec << std::setfill(' ')
                   << " rfid=" << static_cast<int>(d[2])
                   << " rfid_ver=" << static_cast<int>(d[3]) << '.'
                   << static_cast<int>(d[4]) << '.'
                   << static_cast<int>(d[5])
                   << " sensors=" << static_cast<int>(d[6])
                   << " led=" << static_cast<int>(d[7]);
            }
            break;
        case 0x0A1:
            if (hasBytes(d, 0, 8)) os << " bracket=" << readFloat(d, 0) << " motor=" << readFloat(d, 4);
            break;
        case 0x0FF: {
            const std::string boot = versionText(d, 0);
            const std::string app = versionText(d, 4);
            if (!boot.empty() && !app.empty()) os << " boot=" << boot << " app=" << app;
            break;
        }
        default:
            break;
    }
}
} // namespace

std::string decodeCanFrame(const CanFrame& frame)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(2);

    if (!frame.extended) return "STD";

    const uint8_t src = static_cast<uint8_t>(frame.id & 0xFF);
    const uint8_t dst = static_cast<uint8_t>((frame.id >> 8) & 0xFF);
    const uint16_t cmd = static_cast<uint16_t>((frame.id >> 16) & 0x0FFF);
    const uint8_t mod = static_cast<uint8_t>((frame.id >> 28) & 0x01);

    auto srcLetter = [](uint8_t n) -> char {
        switch (n) {
            case 0x01: return 'A';
            case 0x02: return 'M';
            case 0x03: return 'R';
            case 0x04: return 'E';
            case 0x10: return 'P';
            case 0xFF: return '*';
            default:   return '?';
        }
    };
    os << srcLetter(src) << ' ';

    const bool readMemReq = (mod == 1 && cmd == kCanCmdReadMemReq && src == kCanAddrPc);
    const bool readMemRsp = (mod == 1 && cmd == kCanCmdReadMemRsp && dst == kCanAddrPc);
    const bool capReq = (mod == 1 && cmd == kCanCmdCapReq && src == kCanAddrPc);
    const bool capRsp = (mod == 1 && cmd == kCanCmdCapRsp && dst == kCanAddrPc);

    auto capSubName = [](uint8_t s) -> const char* {
        switch (s) {
            case kCapSubReset:      return "reset";
            case kCapSubSetSlot:    return "set_slot";
            case kCapSubSetConfig:  return "set_config";
            case kCapSubSetTrigger: return "set_trigger";
            case kCapSubArm:        return "arm";
            case kCapSubStop:       return "stop";
            case kCapSubStatusReq:  return "status";
            case kCapSubReadChunk:  return "read_chunk";
            case kCapSubCrc:        return "crc_req";
            case kCapAnsAck:        return "ack";
            case kCapAnsStatus:     return "status_rsp";
            case kCapAnsCrc:        return "crc_rsp";
            case kCapAnsData:       return "data";
            case kCapAnsDataEnd:    return "data_end";
            case kCapAnsError:      return "error";
            default:                return "?";
        }
    };

    std::string name;
    if (readMemReq) {
        name = "READ_MEM req";
    } else if (readMemRsp) {
        name = "READ_MEM rsp";
    } else if (capReq) {
        name = std::string("CAP req:") + capSubName(frame.data.empty() ? 0 : frame.data[0]);
    } else if (capRsp) {
        name = std::string("CAP rsp:") + capSubName(frame.data.empty() ? 0 : frame.data[0]);
    } else if (mod == 0) {
        name = bootCmdName(cmd);
    } else if (src == 0x02 || dst == 0x02) {
        name = motorCmdName(cmd, src == 0x02);
    }
    if (name.empty() && mod == 1 && (src == 0x03 || dst == 0x03)) {
        name = rkCmdName(cmd, src == 0x03);
    }
    if (name.empty() && mod == 1 && (src == 0x01 || dst == 0x01)) {
        name = mainCmdName(cmd, src == 0x01);
    }
    if (name.empty()) {
        std::ostringstream cmdHex;
        cmdHex << "cmd=0x" << std::uppercase << std::hex << std::setw(3) << std::setfill('0') << cmd;
        name = cmdHex.str();
    }
    os << name;

    std::ostringstream payloadOs;
    payloadOs << std::fixed << std::setprecision(2);
    if (readMemReq) {
        if (hasBytes(frame.data, 0, 8)) {
            const uint32_t addr = readU32(frame.data, 2);
            payloadOs << " dst=" << nodeLabel(dst)
                      << " seq=" << static_cast<int>(frame.data[1])
                      << " addr=0x" << std::uppercase << std::hex << std::setw(8) << std::setfill('0') << addr
                      << std::dec << std::setfill(' ') << " +" << static_cast<int>(frame.data[6]);
        }
    } else if (readMemRsp) {
        if (hasBytes(frame.data, 0, 8)) {
            payloadOs << " src=" << nodeLabel(src)
                      << " seq=" << static_cast<int>(frame.data[1])
                      << " status=" << static_cast<int>(frame.data[2]);
            if (frame.data[2] == 0 && frame.data[3] > 0) {
                payloadOs << " v=0x" << std::uppercase << std::hex << std::setfill('0');
                for (int i = 3 + frame.data[3]; i >= 4; --i) {
                    if (i < static_cast<int>(frame.data.size())) {
                        payloadOs << std::setw(2) << static_cast<int>(frame.data[static_cast<size_t>(i)]);
                    }
                }
                payloadOs << std::dec << std::setfill(' ');
            }
        }
    } else if (capReq && hasBytes(frame.data, 0, 8)) {
        const uint8_t sub = frame.data[0];
        payloadOs << " dst=" << nodeLabel(dst);
        if (sub == kCapSubSetSlot) {
            payloadOs << " slot=" << static_cast<int>(frame.data[1])
                      << " addr=0x" << std::uppercase << std::hex << std::setw(8) << std::setfill('0')
                      << readU32(frame.data, 2) << std::dec << std::setfill(' ')
                      << " size=" << static_cast<int>(frame.data[6])
                      << " type=" << static_cast<int>(frame.data[7]);
        } else if (sub == kCapSubSetConfig) {
            payloadOs << " slots=" << static_cast<int>(frame.data[1])
                      << " period_us=" << readU16(frame.data, 2)
                      << " pre/post=" << static_cast<int>(frame.data[4]) << "/"
                      << static_cast<int>(frame.data[5])
                      << " buf_kb=" << static_cast<int>(frame.data[6]);
        } else if (sub == kCapSubSetTrigger) {
            payloadOs << " mode=" << static_cast<int>(frame.data[1])
                      << " slot=" << static_cast<int>(frame.data[2])
                      << " thr=" << static_cast<int32_t>(readU32(frame.data, 3));
        } else if (sub == kCapSubReadChunk) {
            payloadOs << " off=" << readU16(frame.data, 1)
                      << " count=" << readU16(frame.data, 3);
        }
    } else if (capRsp && hasBytes(frame.data, 0, 8)) {
        const uint8_t sub = frame.data[0];
        payloadOs << " src=" << nodeLabel(src);
        if (sub == kCapAnsAck || sub == kCapAnsError) {
            payloadOs << " orig=0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(frame.data[1]) << std::dec << std::setfill(' ')
                      << " status=" << static_cast<int>(frame.data[2]);
        } else if (sub == kCapAnsStatus) {
            const uint8_t st = frame.data[1];
            const char* stName = (st == 0 ? "IDLE" :
                                  st == 1 ? "ARMED" :
                                  st == 2 ? "TRIG" :
                                  st == 3 ? "DONE" :
                                  st == 4 ? "ERR" : "?");
            payloadOs << " state=" << stName
                      << " slots=" << static_cast<int>(frame.data[2])
                      << " samples=" << readU16(frame.data, 3)
                      << " trig_idx=" << readU16(frame.data, 5);
        } else if (sub == kCapAnsData || sub == kCapAnsDataEnd) {
            payloadOs << " off=" << readU16(frame.data, 1);
            if (sub == kCapAnsDataEnd) payloadOs << " end(n=" << static_cast<int>(frame.data[7]) << ")";
        } else if (sub == kCapAnsCrc) {
            // [0x96, crc32_le32, totalBytes_le16, 0xFF]
            payloadOs << " crc=0x" << std::uppercase << std::hex << std::setw(8) << std::setfill('0')
                      << readU32(frame.data, 1) << std::dec << std::setfill(' ')
                      << " bytes=" << readU16(frame.data, 5);
        }
    } else if (mod == 1) {
        if (src == 0x02 || dst == 0x02) appendMotorPayload(payloadOs, cmd, frame, src == 0x02);
        if (src == 0x03 || dst == 0x03) appendRkPayload(payloadOs, cmd, frame, src == 0x03);
        // mainPCB-specific terse payload — emitted only when neither motor
        // nor RK matched, so we don't double-print on broadcast frames.
        if (src == 0x01 || dst == 0x01) {
            const auto& d = frame.data;
            switch (cmd) {
                case 0x010:
                    if (hasBytes(d, 0, 8)) {
                        payloadOs << " mode=" << static_cast<int>(d[0])
                                  << " magic=" << (char)d[4] << (char)d[5] << (char)d[6] << (char)d[7];
                    }
                    break;
                case 0x011:
                    if (hasBytes(d, 0, 1)) payloadOs << " seq=" << static_cast<int>(d[0]);
                    break;
                case 0x012:
                    if (hasBytes(d, 0, 5)) {
                        const char* act = "?";
                        switch (d[0]) {
                            case 0: act = "STOP"; break;
                            case 1: act = "START"; break;
                            case 2: act = "PARK"; break;
                            case 3: act = "RESET_ERR"; break;
                            case 4: act = "SET_SPEED"; break;
                        }
                        payloadOs << " action=" << act
                                  << " speed=" << readFloat(d, 1) << "%";
                    }
                    break;
                case 0x013:
                    if (hasBytes(d, 0, 1)) {
                        const char* sub = "?";
                        switch (d[0]) {
                            case 0: sub = "ESTOP_DIS"; break;
                            case 1: sub = "ESTOP_EN"; break;
                            case 2: sub = "PARK_DIS"; break;
                            case 3: sub = "PARK_EN"; break;
                        }
                        payloadOs << " subcmd=" << sub;
                    }
                    break;
                case 0x014:
                    if (hasBytes(d, 0, 2)) {
                        payloadOs << " enable=" << static_cast<int>(d[0])
                                  << " override=" << static_cast<int>(d[1]);
                    }
                    break;
                case 0xB10:
                    if (hasBytes(d, 0, 8)) {
                        uint16_t wdg = readU16(d, 4);
                        payloadOs << " flags=0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
                                  << static_cast<int>(d[0]) << std::dec << std::setfill(' ')
                                  << " fsm=" << static_cast<int>(d[1])
                                  << " hb_seq=" << static_cast<int>(d[2])
                                  << " wdg=" << wdg << "ms";
                    }
                    break;
                case 0xB11:
                    if (hasBytes(d, 0, 8)) {
                        uint16_t vdc_dV = readU16(d, 6);
                        payloadOs << " mode=" << static_cast<int>(d[0])
                                  << " sf=0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
                                  << static_cast<int>(d[1]) << std::dec << std::setfill(' ')
                                  << " rpm=" << readFloat(d, 2)
                                  << " Vdc=" << (static_cast<float>(vdc_dV) * 0.1f) << "V";
                    }
                    break;
                case 0x100:
                    // MAIN_CMD_SET_WIFI_MODE (host → MAIN): data[0]=0/1/2 OFF/STA/AP
                    if (hasBytes(d, 0, 1)) {
                        payloadOs << " mode=" << mainWifiModeName(d[0])
                                  << "(" << static_cast<int>(d[0]) << ")";
                    }
                    break;
                case 0xB00:
                    // MAIN_ANS_WIFI_STATUS (MAIN → broadcast): mode + IPv4 + ok
                    if (hasBytes(d, 0, 6)) {
                        payloadOs << " mode=" << mainWifiModeName(d[0])
                                  << " ip=" << static_cast<int>(d[1]) << '.'
                                  << static_cast<int>(d[2]) << '.'
                                  << static_cast<int>(d[3]) << '.'
                                  << static_cast<int>(d[4])
                                  << " ok=" << static_cast<int>(d[5]);
                    }
                    break;
                default: break;
            }
        }
        appendFileTransferPayload(payloadOs, cmd, frame.data);
    } else {
        if (cmd == 0x0FF) {
            const std::string boot = versionText(frame.data, 0);
            const std::string app = versionText(frame.data, 4);
            if (!boot.empty() && !app.empty()) payloadOs << " boot=" << boot << " app=" << app;
        }
        appendFileTransferPayload(payloadOs, cmd, frame.data);
    }
    const std::string payload = payloadOs.str();
    if (!payload.empty()) os << ":" << payload;
    return os.str();
}

namespace {

std::string fmtFloat(float v) {
    std::ostringstream o; o << std::fixed << std::setprecision(3) << v; return o.str();
}
std::string fmtHex32(uint32_t v) {
    std::ostringstream o; o << "0x" << std::uppercase << std::hex << std::setw(8) << std::setfill('0') << v; return o.str();
}
std::string fmtHex8(uint8_t v) {
    std::ostringstream o; o << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(v); return o.str();
}

const char* motorWorkModeName(uint8_t mode)
{
    switch (mode) {
        case 0: return "STOP";
        case 1: return "IDLE / CLEAR_FAULTS";
        case 2: return "SPEED";
        case 7: return "MOMENT / VOLTAGE_PERCENT";
        default: return "?";
    }
}

const char* diagFsmName(uint8_t state)
{
    switch (state) {
        case 0: return "WARNING";
        case 1: return "STOP";
        case 2: return "PAUSE";
        case 3: return "RUN";
        case 4: return "PARKING";
        case 5: return "PARK";
        default: return "?";
    }
}

const char* motorProxyActionName(uint8_t action)
{
    switch (action) {
        case 0: return "STOP";
        case 1: return "START";
        case 2: return "PARK";
        case 3: return "RESET_ERR / CLEAR_FAULTS";
        case 4: return "SET_SPEED";
        default: return "?";
    }
}

const char* rkSubcmdName(uint8_t subcmd)
{
    switch (subcmd) {
        case 0: return "ESTOP_DIS";
        case 1: return "ESTOP_EN";
        case 2: return "PARKING_DIS";
        case 3: return "PARKING_EN";
        default: return "?";
    }
}

std::string boolText(bool v)
{
    return v ? "1 (true)" : "0 (false)";
}

std::string byteWithName(uint8_t v, const char* name)
{
    std::ostringstream o;
    o << static_cast<int>(v) << " (" << name << ")";
    return o.str();
}

void appendPackedFlagRows(std::vector<TooltipField>& out,
                          uint8_t value,
                          std::initializer_list<std::pair<uint8_t, const char*>> bits)
{
    for (const auto& bit : bits) {
        const bool on = (value & static_cast<uint8_t>(1u << bit.first)) != 0;
        out.push_back({"bit " + std::to_string(bit.first), "1 bit", bit.second, boolText(on), on});
    }
}

void appendMotorCtrlRows(std::vector<TooltipField>& out, const CanFrame& f)
{
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "driverMode / work_mode",
                   byteWithName(f.data[0], motorWorkModeName(f.data[0])), false});
    out.push_back({"bytes 1..4", "4 bytes (float LE)", "speed_or_value_percent",
                   fmtFloat(readFloat(f.data, 1)), false});
    out.push_back({"byte 5", "1 byte", "reserved guard", fmtHex8(f.data[5]), f.data[5] != 0xFF});
    out.push_back({"byte 6", "1 byte", "reserved guard", fmtHex8(f.data[6]), f.data[6] != 0xFF});
    out.push_back({"byte 7", "1 byte", "reserved guard", fmtHex8(f.data[7]), f.data[7] != 0xFF});
}

void appendMotorThetaConfigRows(std::vector<TooltipField>& out, const CanFrame& f)
{
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "subcmd",
                   f.data[0] == kMotorCfgThetaOffset ? "THETA_OFFSET" : "UNKNOWN",
                   f.data[0] != kMotorCfgThetaOffset});
    out.push_back({"bytes 1..4", "4 bytes (float LE)", "value (rad or align A)",
                   fmtFloat(readFloat(f.data, 1)), false});
    out.push_back({"byte 5", "1 byte", "action",
                   byteWithName(f.data[5], motorThetaActionName(f.data[5])), false});
    out.push_back({"byte 6", "1 byte", "guard 0xA5",
                   fmtHex8(f.data[6]), f.data[5] != 0 && f.data[6] != 0xA5});
    out.push_back({"byte 7", "1 byte", "guard 0x5A",
                   fmtHex8(f.data[7]), f.data[5] != 0 && f.data[7] != 0x5A});
}

void appendMotorThetaConfigRspRows(std::vector<TooltipField>& out, const CanFrame& f)
{
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "subcmd",
                   f.data[0] == kMotorCfgThetaOffset ? "THETA_OFFSET" : "UNKNOWN",
                   f.data[0] != kMotorCfgThetaOffset});
    out.push_back({"byte 1", "1 byte", "status",
                   byteWithName(f.data[1], motorThetaStatusName(f.data[1])), f.data[1] != 0 && f.data[1] != 4});
    out.push_back({"bytes 2..5", "4 bytes (float LE)", "theta_actual_el_offset (rad)",
                   fmtFloat(readFloat(f.data, 2)), false});
    out.push_back({"byte 6", "1 byte (flags)", "flags", fmtHex8(f.data[6]), f.data[6] != 0});
    appendPackedFlagRows(out, f.data[6], {
        {0, "saved value valid"},
        {1, "loaded from backup"},
        {2, "calibration active"},
        {3, "save after calibration requested"},
    });
    out.push_back({"byte 7", "1 byte", "driverMode",
                   byteWithName(f.data[7], motorWorkModeName(f.data[7])), false});
}

// CFG sub-cmd 0x02 PARAM request: data[1]=action, data[2..3]=u16 id LE,
// data[4..7]=value (f32 or u32, semantics per action). Special case for
// GET_NAME_CHUNK: data[6..7] = chunk index (u16 LE) instead of value.
void appendMotorParamConfigRows(std::vector<TooltipField>& out, const CanFrame& f)
{
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "subcmd", "PARAM", false});

    const uint8_t  action = f.data[1];
    const uint16_t id     = static_cast<uint16_t>(f.data[2]) |
                            (static_cast<uint16_t>(f.data[3]) << 8);

    out.push_back({"byte 1", "1 byte", "action",
                   byteWithName(action, motorParamActionName(action)), false});
    {
        std::ostringstream os;
        os << id << " [" << motorParamIdName(id) << "]";
        out.push_back({"bytes 2..3", "2 bytes (u16 LE)", "param_id", os.str(),
                       std::string(motorParamIdName(id)) == "?"});
    }

    if (action == 10 /* GET_NAME_CHUNK */) {
        const uint16_t chunk = static_cast<uint16_t>(f.data[6]) |
                               (static_cast<uint16_t>(f.data[7]) << 8);
        std::ostringstream os; os << chunk;
        out.push_back({"bytes 4..5", "2 bytes", "padding (0xFFFF)",
                       (f.data[4] == 0xFF && f.data[5] == 0xFF) ? "0xFFFF" : "unexpected",
                       !(f.data[4] == 0xFF && f.data[5] == 0xFF)});
        out.push_back({"bytes 6..7", "2 bytes (u16 LE)", "chunk_idx", os.str(), false});
    } else if (action == 1 /* SET */) {
        out.push_back({"bytes 4..7", "4 bytes (f32 LE / u32 LE)",
                       "value (interpret per param type)",
                       fmtFloat(readFloat(f.data, 4)) + " / 0x" +
                           fmtHex32(readU32(f.data, 4)),
                       false});
    } else {
        out.push_back({"bytes 4..7", "4 bytes", "value (ignored for this action)",
                       fmtHex32(readU32(f.data, 4)), false});
    }
}

// CFG sub-cmd 0x02 PARAM response: data[1]=status, data[2..3]=u16 id echo,
// data[4..7]=value (semantics per action -- caller must remember which
// action they sent since action is NOT echoed in response).
void appendMotorParamConfigRspRows(std::vector<TooltipField>& out, const CanFrame& f)
{
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "subcmd", "PARAM", false});
    out.push_back({"byte 1", "1 byte", "status",
                   byteWithName(f.data[1], motorThetaStatusName(f.data[1])),
                   f.data[1] != 0});
    const uint16_t id = static_cast<uint16_t>(f.data[2]) |
                        (static_cast<uint16_t>(f.data[3]) << 8);
    {
        std::ostringstream os;
        os << id << " [" << motorParamIdName(id) << "]";
        out.push_back({"bytes 2..3", "2 bytes (u16 LE)", "param_id echo", os.str(), false});
    }

    const uint32_t v_u32 = readU32(f.data, 4);
    const float    v_f32 = readFloat(f.data, 4);

    // For GET_META, byte0=group, byte1=type, byte2=flags packed in u32.
    // We can't tell from response alone if this was GET_META or any other
    // GET action, so show both interpretations.
    std::ostringstream os;
    os << fmtFloat(v_f32) << " (as f32)  |  0x"
       << std::uppercase << std::hex << std::setw(8) << std::setfill('0') << v_u32
       << std::dec << std::setfill(' ') << " (as u32)";
    out.push_back({"bytes 4..7", "4 bytes", "value", os.str(), false});

    // Decode the u32 as packed META in case this was GET_META.
    out.push_back({"  if GET_META", "byte 0", "group",
                   byteWithName(static_cast<uint8_t>(v_u32 & 0xFFu),
                                motorParamGroupName(static_cast<uint8_t>(v_u32 & 0xFFu))),
                   false});
    out.push_back({"  if GET_META", "byte 1", "type",
                   byteWithName(static_cast<uint8_t>((v_u32 >> 8) & 0xFFu),
                                motorParamTypeName(static_cast<uint8_t>((v_u32 >> 8) & 0xFFu))),
                   false});
    out.push_back({"  if GET_META", "byte 2", "flags",
                   fmtHex8(static_cast<uint8_t>((v_u32 >> 16) & 0xFFu)),
                   false});
}

// CFG sub-cmd 0x03 BLOB rows. Same wire shape both directions:
//   data[0]=0x03, data[1]=action (AppMotorBlobAction_e), data[2]=chunk_idx
//   data[3..7] = 5 bytes of blob payload (or status/CRC for SET_RESULT).
// One special case: action=SET_RESULT (3) re-purposes the payload as
// status@data[3] and computed CRC32@data[4..7].
void appendMotorBlobConfigRows(std::vector<TooltipField>& out, const CanFrame& f)
{
    if (!hasBytes(f.data, 0, 8)) return;
    const uint8_t action = f.data[1];
    const uint8_t chunkIdx = f.data[2];
    out.push_back({"byte 0", "1 byte", "subcmd", "BLOB (0x03)", false});
    out.push_back({"byte 1", "1 byte", "action",
                   byteWithName(action, motorBlobActionName(action)),
                   std::string(motorBlobActionName(action)) == "?"});
    out.push_back({"byte 2", "1 byte", "chunk_idx (0..32)",
                   std::to_string(static_cast<int>(chunkIdx)), chunkIdx > 32});
    if (action == 3 /* SET_RESULT */) {
        out.push_back({"byte 3", "1 byte", "status",
                       byteWithName(f.data[3], motorThetaStatusName(f.data[3])),
                       f.data[3] != 0});
        out.push_back({"bytes 4..7", "4 bytes (uint32 LE)", "computed_crc32",
                       fmtHex32(readU32(f.data, 4)), false});
    } else {
        std::ostringstream o;
        o << std::uppercase << std::hex << std::setfill('0');
        for (int i = 3; i < 8; ++i) {
            if (i > 3) o << ' ';
            o << std::setw(2) << static_cast<int>(f.data[i]);
        }
        out.push_back({"bytes 3..7", "5 bytes", "blob payload chunk",
                       o.str(), false});
    }
}

// Sub-cmd dispatcher for cmd 0x005 (request) / 0x0AB (response).
void appendMotorConfigRows(std::vector<TooltipField>& out, const CanFrame& f)
{
    if (!hasBytes(f.data, 0, 1)) return;
    switch (f.data[0]) {
        case kMotorCfgThetaOffset: appendMotorThetaConfigRows(out, f); break;
        case kMotorCfgThetaSector:
            out.push_back({"byte 0", "1 byte", "subcmd", "THETA_SECTOR", false});
            break;
        case kMotorCfgParam:       appendMotorParamConfigRows(out, f); break;
        case 0x03:                 appendMotorBlobConfigRows(out, f); break;
        default:
            out.push_back({"byte 0", "1 byte", "subcmd (UNKNOWN)",
                           fmtHex8(f.data[0]), true});
            break;
    }
}

void appendMotorConfigRspRows(std::vector<TooltipField>& out, const CanFrame& f)
{
    if (!hasBytes(f.data, 0, 1)) return;
    switch (f.data[0]) {
        case kMotorCfgThetaOffset: appendMotorThetaConfigRspRows(out, f); break;
        case kMotorCfgThetaSector:
            out.push_back({"byte 0", "1 byte", "subcmd", "THETA_SECTOR rsp", false});
            break;
        case kMotorCfgParam:       appendMotorParamConfigRspRows(out, f); break;
        case 0x03:                 appendMotorBlobConfigRows(out, f); break;
        default:
            out.push_back({"byte 0", "1 byte", "subcmd (UNKNOWN)",
                           fmtHex8(f.data[0]), true});
            break;
    }
}

void appendMotorPfcControlRows(std::vector<TooltipField>& out, const CanFrame& f)
{
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "enable", boolText(f.data[0] != 0), f.data[0] != 0});
    out.push_back({"bytes 1..7", "7 bytes", "reserved guard",
                   (f.data[1] == 0xFF && f.data[2] == 0xFF && f.data[3] == 0xFF &&
                    f.data[4] == 0xFF && f.data[5] == 0xFF && f.data[6] == 0xFF &&
                    f.data[7] == 0xFF) ? "all 0xFF" : "unexpected value",
                   !(f.data[1] == 0xFF && f.data[2] == 0xFF && f.data[3] == 0xFF &&
                     f.data[4] == 0xFF && f.data[5] == 0xFF && f.data[6] == 0xFF &&
                     f.data[7] == 0xFF)});
}

void appendMotorPfcStateRows(std::vector<TooltipField>& out, const CanFrame& f)
{
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "driverMode",
                   byteWithName(f.data[0], motorWorkModeName(f.data[0])), false});
    out.push_back({"byte 1", "1 byte", "pfc_enabled", boolText(f.data[1] != 0), f.data[1] != 0});
    out.push_back({"bytes 2..7", "6 bytes", "reserved", "0", false});
}

void appendMotorFaultRows(std::vector<TooltipField>& out, const CanFrame& f)
{
    if (!hasBytes(f.data, 0, 8)) return;
    const uint32_t mask = readU32(f.data, 1);
    out.push_back({"byte 0", "1 byte", "mode", std::to_string(static_cast<int>(f.data[0])), false});
    out.push_back({"bytes 1..4", "4 bytes (uint32 LE)", "fault_mask", fmtHex32(mask), mask != 0u});
    // 32 individual fault bits: position 1..4 = bits 0..31 of the LE uint32.
    for (int b = 0; b < 32; ++b) {
        const bool on = (mask & (1u << b)) != 0u;
        std::string pos = "bit " + std::to_string(b);
        out.push_back({pos, "1 bit", kMotorFaultNames[b], on ? "1" : "0", on});
    }
    out.push_back({"byte 5", "1 byte", "rec (RX error counter)", std::to_string(static_cast<int>(f.data[5])), f.data[5] != 0});
    out.push_back({"byte 6", "1 byte", "tec (TX error counter)", std::to_string(static_cast<int>(f.data[6])), f.data[6] != 0});
    out.push_back({"byte 7", "1 byte", "can_flags", fmtHex8(f.data[7]), f.data[7] != 0});
    // can_flags bit field — motor CanDiag_s CAN_DIAG_FLAG_* (can.h:274-281).
    appendPackedFlagRows(out, f.data[7], {
        {0, "BOFF (bus-off)"},
        {1, "EPVF (error-passive)"},
        {2, "EWGF (error-warning)"},
        {3, "RX_OVERRUN"},
        {4, "TX_TIMEOUT"},
        {5, "TX_FIFO_FULL"},
        {6, "RESTART happened"},
        {7, "ERR_CB_SEEN (HAL ErrorCallback fired)"},
    });
}

void appendMotorTempRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"bytes 0..3", "4 bytes (float)", "vt (driver temp, C)", fmtFloat(readFloat(f.data, 0)), false});
    out.push_back({"bytes 4..7", "4 bytes (float)", "motor (motor temp, C)", fmtFloat(readFloat(f.data, 4)), false});
}

void appendMotorScalarRows(std::vector<TooltipField>& out, const CanFrame& f, const char* nameAfterMode)
{
    if (!hasBytes(f.data, 0, 5)) return;
    out.push_back({"byte 0", "1 byte", "mode", std::to_string(static_cast<int>(f.data[0])), false});
    out.push_back({"bytes 1..4", "4 bytes (float)", nameAfterMode, fmtFloat(readFloat(f.data, 1)), false});
}

void appendMotorCanDiagRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "mode", std::to_string(static_cast<int>(f.data[0])), false});
    out.push_back({"byte 1", "1 byte", "lec (last error code)", std::to_string(static_cast<int>(f.data[1])), f.data[1] != 0});
    out.push_back({"bytes 2..3", "2 bytes (uint16 LE)", "restart count", std::to_string(readU16(f.data, 2)), false});
    out.push_back({"bytes 4..5", "2 bytes (uint16 LE)", "rx_ovr (RX overrun)", std::to_string(readU16(f.data, 4)), readU16(f.data, 4) != 0});
    out.push_back({"bytes 6..7", "2 bytes (uint16 LE)", "tx_drop", std::to_string(readU16(f.data, 6)), readU16(f.data, 6) != 0});
}

void appendMotorSysDiagRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "mode", std::to_string(static_cast<int>(f.data[0])), false});
    out.push_back({"byte 1", "1 byte", "reset cause flags (RCC_CSR)", fmtHex8(f.data[1]), f.data[1] != 0});
    // reset-cause bit field — appResetCauseDecoded (motor main.c:301-308).
    appendPackedFlagRows(out, f.data[1], {
        {0, "LPWR (low-power reset)"},
        {1, "WWDG (window watchdog)"},
        {2, "IWDG (independent watchdog)"},
        {3, "SFT (software reset)"},
        {4, "POR (power-on reset)"},
        {5, "PIN (NRST pin reset)"},
        {6, "BOR (brown-out reset)"},
    });
    const char* excName = (f.data[2] == 1) ? "HardFault" : (f.data[2] == 2) ? "MemManage"
                        : (f.data[2] == 3) ? "BusFault"  : (f.data[2] == 4) ? "UsageFault"
                        : (f.data[2] == 5) ? "NMI" : "none";
    out.push_back({"byte 2", "1 byte", "exc (survived exception)", byteWithName(f.data[2], excName), f.data[2] != 0});
    out.push_back({"byte 3", "1 byte", "tim1 break count", std::to_string(static_cast<int>(f.data[3])), f.data[3] != 0});
    out.push_back({"byte 4", "1 byte", "OC_U trip count", std::to_string(static_cast<int>(f.data[4])), f.data[4] != 0});
    out.push_back({"byte 5", "1 byte", "OC_V trip count", std::to_string(static_cast<int>(f.data[5])), f.data[5] != 0});
    out.push_back({"byte 6", "1 byte", "OC_W trip count", std::to_string(static_cast<int>(f.data[6])), f.data[6] != 0});
    out.push_back({"byte 7", "1 byte", "i2t accumulator", std::to_string(static_cast<int>(f.data[7])), f.data[7] != 0});
}

void appendMotorCurDiagRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    const uint8_t status = f.data[0];
    char status_text[64];
    std::snprintf(status_text, sizeof(status_text),
                  "mode=%d cal=%d run=%d align=%d",
                  status & 0x0F,
                  (status & 0x10) ? 1 : 0,
                  (status & 0x20) ? 1 : 0,
                  (status & 0x40) ? 1 : 0);
    out.push_back({"byte 0", "1 byte (packed)", "status flags", status_text, false});
    out.push_back({"bytes 1..4", "4 bytes (float LE)", "i_phase_sum_offset_filt (A)", fmtFloat(readFloat(f.data, 1)), readFloat(f.data, 1) > 3.25f});
    const float step = 0.2f;
    const float u_off = static_cast<float>(static_cast<int8_t>(f.data[5])) * step;
    const float v_off = static_cast<float>(static_cast<int8_t>(f.data[6])) * step;
    const float w_off = static_cast<float>(static_cast<int8_t>(f.data[7])) * step;
    out.push_back({"byte 5", "1 byte (int8 * 0.2A)", "i_phase_u_offset (A)", fmtFloat(u_off), false});
    out.push_back({"byte 6", "1 byte (int8 * 0.2A)", "i_phase_v_offset (A)", fmtFloat(v_off), false});
    out.push_back({"byte 7", "1 byte (int8 * 0.2A)", "i_phase_w_offset (A)", fmtFloat(w_off), false});
}

// --- Motor profiler / theta-cal / scheduler-diag telemetry (0x0AC..0x0AF) ---
const char* profilerStateName(uint8_t s) {
    switch (s) { case 0: return "IDLE"; case 1: return "RUNNING"; case 2: return "DONE";
                 case 3: return "ERROR"; case 4: return "ABORTED"; default: return "?"; }
}
const char* profilerTestName(uint8_t t) {
    switch (t) { case 0: return "NONE"; case 1: return "NOLOAD_PRECHECK";
                 case 2: return "BLOCKED_RS_L_PI"; default: return "?"; }
}
const char* profilerErrName(uint8_t e) {
    switch (e) { case 0: return "NONE"; case 1: return "BUSY"; case 2: return "NOT_READY";
                 case 3: return "FAULT"; case 4: return "BAD_ARG"; case 5: return "MOVED";
                 case 6: return "SIGNAL"; case 7: return "RANGE"; default: return "?"; }
}
const char* profilerResultName(uint8_t r) {
    switch (r) { case 0: return "Rs"; case 1: return "Ld"; case 2: return "Lq"; case 3: return "Kp_id";
                 case 4: return "Ki_id"; case 5: return "Kp_iq"; case 6: return "Ki_iq"; default: return "?"; }
}

// 0x0AC MTR_ANS_PROFILER: state/test/progress/(error+result)/value.
void appendMotorProfilerRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "state", byteWithName(f.data[0], profilerStateName(f.data[0])), f.data[0] == 3});
    out.push_back({"byte 1", "1 byte", "active_test", byteWithName(f.data[1], profilerTestName(f.data[1])), false});
    out.push_back({"byte 2", "1 byte", "progress %", std::to_string((int)f.data[2]), false});
    const uint8_t err = f.data[3] & 0x0F;
    const uint8_t res = (f.data[3] >> 4) & 0x0F;
    out.push_back({"byte 3 lo", "nibble", "error", byteWithName(err, profilerErrName(err)), err != 0});
    out.push_back({"byte 3 hi", "nibble", "result_sel", byteWithName(res, profilerResultName(res)), false});
    out.push_back({"bytes 4..7", "4 bytes (float LE)", "value (selected result)", fmtFloat(readFloat(f.data, 4)), false});
}

// 0x0AD MTR_ANS_THETA_CAL: theta-calibration live status / completion.
void appendMotorThetaCalRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    const char* ph = (f.data[1] == 0) ? "IDLE" : (f.data[1] == 1) ? "RUNNING"
                   : (f.data[1] == 2) ? "DONE_OK" : (f.data[1] == 3) ? "DONE_FAIL" : "?";
    out.push_back({"byte 0", "1 byte", "driverMode", byteWithName(f.data[0], motorWorkModeName(f.data[0])), false});
    out.push_back({"byte 1", "1 byte", "cal phase", byteWithName(f.data[1], ph), f.data[1] == 3});
    out.push_back({"byte 2", "1 byte", "progress %", std::to_string((int)f.data[2]), false});
    out.push_back({"byte 3", "1 byte", "flags", fmtHex8(f.data[3]), f.data[3] != 0});
    appendPackedFlagRows(out, f.data[3], {
        {0, "sector S1 LUT valid"}, {1, "sector S2 LUT valid"}, {2, "sector S3 LUT valid"},
        {3, "sector S4 LUT valid"}, {4, "sector S5 LUT valid"}, {5, "sector S6 LUT valid"},
        {6, "GLOBAL_OK (offset known)"}, {7, "SAVE_REQ (save to flash)"},
    });
    out.push_back({"bytes 4..7", "4 bytes (float LE)", "theta_offset (rad)", fmtFloat(readFloat(f.data, 4)), false});
}

// 0x0AE MTR_ANS_SCHED_DIAG: motor_can_stream lane-2/3 scheduler diagnostics.
void appendMotorSchedDiagRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "capture_active", boolText(f.data[0] != 0), f.data[0] != 0});
    out.push_back({"byte 1", "1 byte", "normal_depth (queue)", std::to_string((int)f.data[1]), false});
    out.push_back({"byte 2", "1 byte", "capture_depth (queue)", std::to_string((int)f.data[2]), false});
    out.push_back({"bytes 3..4", "2 bytes (uint16 LE)", "normal_drop_count", std::to_string(readU16(f.data, 3)), readU16(f.data, 3) != 0});
    out.push_back({"bytes 5..6", "2 bytes (uint16 LE)", "capture_drop_count", std::to_string(readU16(f.data, 5)), readU16(f.data, 5) != 0});
    const uint8_t nf = f.data[7] & 0x0F;
    const uint8_t cf = (f.data[7] >> 4) & 0x0F;
    out.push_back({"byte 7 lo", "nibble", "normal_send_fail", std::to_string((int)nf), nf != 0});
    out.push_back({"byte 7 hi", "nibble", "capture_send_fail", std::to_string((int)cf), cf != 0});
}

// 0x0AF MTR_ANS_SCHED_DIAG2: lane-1 reliable-path counters (giveup must be 0).
void appendMotorSchedDiag2Rows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "reserved", fmtHex8(f.data[0]), false});
    out.push_back({"bytes 1..2", "2 bytes (uint16 LE)", "reliable_backpressure_count", std::to_string(readU16(f.data, 1)), readU16(f.data, 1) != 0});
    out.push_back({"bytes 3..4", "2 bytes (uint16 LE)", "reliable_sync_giveup_count", std::to_string(readU16(f.data, 3)), readU16(f.data, 3) != 0});
    out.push_back({"bytes 5..6", "2 bytes (uint16 LE)", "reliable_max_retry_seen", std::to_string(readU16(f.data, 5)), false});
    out.push_back({"byte 7", "1 byte", "reserved", fmtHex8(f.data[7]), false});
}

void appendRkSwitchRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "encoder count", std::to_string(static_cast<int>(f.data[0])), false});
    out.push_back({"byte 1", "1 byte", "flags", fmtHex8(f.data[1]), f.data[1] != 0});
    out.push_back({"byte 2", "1 byte", "RFID id",   std::to_string(static_cast<int>(f.data[2])), false});
    char ver[16];
    std::snprintf(ver, sizeof(ver), "%u.%u.%u", f.data[3], f.data[4], f.data[5]);
    out.push_back({"bytes 3..5", "3 bytes", "RFID version (maj.min.patch)", ver, false});
    out.push_back({"byte 6", "1 byte", "sensors bitmask", fmtHex8(f.data[6]), false});
    out.push_back({"byte 7", "1 byte", "led state", std::to_string(static_cast<int>(f.data[7])), false});
}

void appendRkAnglesRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"bytes 0..3", "4 bytes (float)", "bracket angle (deg)", fmtFloat(readFloat(f.data, 0)), false});
    out.push_back({"bytes 4..7", "4 bytes (float)", "motor angle (deg)", fmtFloat(readFloat(f.data, 4)), false});
}

void appendRkMotorCtrlRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "subcmd", byteWithName(f.data[0], rkSubcmdName(f.data[0])), false});
    out.push_back({"bytes 1..7", "7 bytes", "reserved guard",
                   (f.data[1] == 0xFF && f.data[2] == 0xFF && f.data[3] == 0xFF &&
                    f.data[4] == 0xFF && f.data[5] == 0xFF && f.data[6] == 0xFF &&
                    f.data[7] == 0xFF) ? "all 0xFF" : "unexpected value",
                   !(f.data[1] == 0xFF && f.data[2] == 0xFF && f.data[3] == 0xFF &&
                     f.data[4] == 0xFF && f.data[5] == 0xFF && f.data[6] == 0xFF &&
                     f.data[7] == 0xFF)});
}

void appendMainDiagModeRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    const bool enter = f.data[0] != 0;
    char magic[5] = {static_cast<char>(f.data[4]), static_cast<char>(f.data[5]),
                     static_cast<char>(f.data[6]), static_cast<char>(f.data[7]), 0};
    out.push_back({"byte 0", "1 byte", "mode", enter ? "1 (enter diag)" : "0 (exit diag)", enter});
    out.push_back({"bytes 1..3", "3 bytes", "reserved guard",
                   (f.data[1] == 0xFF && f.data[2] == 0xFF && f.data[3] == 0xFF) ? "all 0xFF" : "unexpected value",
                   !(f.data[1] == 0xFF && f.data[2] == 0xFF && f.data[3] == 0xFF)});
    out.push_back({"bytes 4..7", "4 bytes ASCII", "magic", magic,
                   !(f.data[4] == 'D' && f.data[5] == 'D' && f.data[6] == 'V' && f.data[7] == '2')});
}

void appendMainHeartbeatRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "heartbeat seq", std::to_string(static_cast<int>(f.data[0])), false});
    out.push_back({"bytes 1..7", "7 bytes", "reserved", "ignored", false});
}

void appendMainMotorProxyRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "action", byteWithName(f.data[0], motorProxyActionName(f.data[0])), false});
    out.push_back({"bytes 1..4", "4 bytes (float LE)", "speed_pct", fmtFloat(readFloat(f.data, 1)), false});
    out.push_back({"bytes 5..7", "3 bytes", "reserved guard",
                   (f.data[5] == 0xFF && f.data[6] == 0xFF && f.data[7] == 0xFF) ? "all 0xFF" : "unexpected value",
                   !(f.data[5] == 0xFF && f.data[6] == 0xFF && f.data[7] == 0xFF)});
}

void appendMainRkProxyRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "subcmd", byteWithName(f.data[0], rkSubcmdName(f.data[0])), false});
    out.push_back({"bytes 1..7", "7 bytes", "reserved guard",
                   (f.data[1] == 0xFF && f.data[2] == 0xFF && f.data[3] == 0xFF &&
                    f.data[4] == 0xFF && f.data[5] == 0xFF && f.data[6] == 0xFF &&
                    f.data[7] == 0xFF) ? "all 0xFF" : "unexpected value",
                   !(f.data[1] == 0xFF && f.data[2] == 0xFF && f.data[3] == 0xFF &&
                     f.data[4] == 0xFF && f.data[5] == 0xFF && f.data[6] == 0xFF &&
                     f.data[7] == 0xFF)});
}

void appendMainPfcProxyRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "enable", boolText(f.data[0] != 0), f.data[0] != 0});
    out.push_back({"byte 1", "1 byte", "override_safety", boolText(f.data[1] != 0), f.data[1] != 0});
    out.push_back({"bytes 2..7", "6 bytes", "reserved guard",
                   (f.data[2] == 0xFF && f.data[3] == 0xFF && f.data[4] == 0xFF &&
                    f.data[5] == 0xFF && f.data[6] == 0xFF && f.data[7] == 0xFF) ? "all 0xFF" : "unexpected value",
                   !(f.data[2] == 0xFF && f.data[3] == 0xFF && f.data[4] == 0xFF &&
                     f.data[5] == 0xFF && f.data[6] == 0xFF && f.data[7] == 0xFF)});
}

void appendMainDiagStatusRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    const uint8_t flags = f.data[0];
    out.push_back({"byte 0", "1 byte (packed)", "flags", fmtHex8(flags), flags != 0});
    appendPackedFlagRows(out, flags, {
        {0, "diag_active"},
        {1, "motor_running (FSM RUN/PARKING)"},
        {2, "rk_armed"},
        {3, "pfc_enabled"},
    });
    out.push_back({"byte 1", "1 byte", "fsm_state", byteWithName(f.data[1], diagFsmName(f.data[1])), false});
    out.push_back({"byte 2", "1 byte", "hb_seq_echo", std::to_string(static_cast<int>(f.data[2])), false});
    out.push_back({"byte 3", "1 byte", "reserved", fmtHex8(f.data[3]), f.data[3] != 0});
    out.push_back({"bytes 4..5", "2 bytes (uint16 LE)", "wdg_remaining_ms", std::to_string(readU16(f.data, 4)), false});
    out.push_back({"bytes 6..7", "2 bytes", "reserved", fmtHex8(f.data[6]) + " " + fmtHex8(f.data[7]), (f.data[6] | f.data[7]) != 0});
}

void appendMainDiagTelemetryRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    const uint8_t status = f.data[1];
    out.push_back({"byte 0", "1 byte", "work_mode", byteWithName(f.data[0], motorWorkModeName(f.data[0])), false});
    out.push_back({"byte 1", "1 byte (packed)", "status_flags", fmtHex8(status), status != 0});
    appendPackedFlagRows(out, status, {
        {0, "motor_online"},
        {1, "rk_online"},
        {2, "motor_in_bootloader"},
        {3, "rk_in_bootloader"},
    });
    out.push_back({"bytes 2..5", "4 bytes (float LE)", "rpm", fmtFloat(readFloat(f.data, 2)), false});
    out.push_back({"bytes 6..7", "2 bytes (uint16 LE, 0.1V)", "vdc", fmtFloat(static_cast<float>(readU16(f.data, 6)) * 0.1f), false});
}

// MAIN_CMD_SET_WIFI_MODE (0x100, host → MAIN): payload[0] = WifiMode enum.
void appendMainWifiModeSetRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "mode",
                   byteWithName(f.data[0], mainWifiModeName(f.data[0])),
                   std::string(mainWifiModeName(f.data[0])) == "?"});
    out.push_back({"bytes 1..7", "7 bytes", "reserved", "ignored", false});
}

// MAIN_ANS_WIFI_STATUS (0xB00, MAIN → broadcast):
//   [0]    mode_now (0/1/2 = OFF/STA/AP)
//   [1..4] ip address bytes (a.b.c.d)
//   [5]    ok flag (1 = transition succeeded)
//   [6..7] reserved
void appendMainWifiStatusRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "mode_now",
                   byteWithName(f.data[0], mainWifiModeName(f.data[0])),
                   std::string(mainWifiModeName(f.data[0])) == "?"});
    char ip[20];
    std::snprintf(ip, sizeof(ip), "%u.%u.%u.%u",
                  f.data[1], f.data[2], f.data[3], f.data[4]);
    out.push_back({"bytes 1..4", "4 bytes", "ipv4 address", ip, false});
    out.push_back({"byte 5", "1 byte", "ok (transition ok)",
                   boolText(f.data[5] != 0), f.data[5] == 0});
    out.push_back({"bytes 6..7", "2 bytes", "reserved",
                   fmtHex8(f.data[6]) + " " + fmtHex8(f.data[7]), false});
}

// MTR_CMD_TONE (0x008, host → motor): single-shot stator-vibration buzzer.
//   [0..1] freq_hz (u16 LE),  [2..3] duration_ms (u16 LE),  [4] amp%
//   [5..7] guard 0xFF
void appendMotorToneRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    const uint16_t freq = readU16(f.data, 0);
    const uint16_t dur  = readU16(f.data, 2);
    out.push_back({"bytes 0..1", "2 bytes (u16 LE)", "freq_hz",
                   std::to_string(freq) + " Hz", freq < 30 || freq > 1500});
    out.push_back({"bytes 2..3", "2 bytes (u16 LE)", "duration_ms",
                   std::to_string(dur) + " ms", dur == 0 || dur > 2000});
    out.push_back({"byte 4", "1 byte", "amplitude_pct",
                   std::to_string(static_cast<int>(f.data[4])) + " %",
                   f.data[4] > 25});
    const bool guard_ok = (f.data[5] == 0xFF && f.data[6] == 0xFF && f.data[7] == 0xFF);
    out.push_back({"bytes 5..7", "3 bytes", "guard 0xFF",
                   guard_ok ? "all 0xFF" : "unexpected value", !guard_ok});
}

// MTR_CMD_EMERGENCY_STOP (0x003, host → motor): no payload, all 0xFF.
void appendMotorEStopRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    const uint32_t lo = readU32(f.data, 0);
    const uint32_t hi = readU32(f.data, 4);
    const bool guard_ok = (lo == 0xFFFFFFFFu) && (hi == 0xFFFFFFFFu);
    out.push_back({"bytes 0..7", "8 bytes", "guard 0xFF (no payload)",
                   fmtHex32(lo) + " " + fmtHex32(hi), !guard_ok});
}

// MTR_CMD_RESET (0x000) and MTR_CMD_GO_TO_BOOT (0x001) carry a magic
// 64-bit handshake (paired bootloader/app payloads). Decoder only knows
// the shape; the firmware checks for specific values.
void appendMotorMagicRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"bytes 0..3", "4 bytes (uint32 LE)", "magic_lo",
                   fmtHex32(readU32(f.data, 0)), false});
    out.push_back({"bytes 4..7", "4 bytes (uint32 LE)", "magic_hi",
                   fmtHex32(readU32(f.data, 4)), false});
}

// MTR_CMD_PING (0xFFF, host → motor): all 0xFF.
void appendMotorPingReqRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    const uint32_t lo = readU32(f.data, 0);
    const uint32_t hi = readU32(f.data, 4);
    const bool guard_ok = (lo == 0xFFFFFFFFu) && (hi == 0xFFFFFFFFu);
    out.push_back({"bytes 0..7", "8 bytes", "ping (no payload, all 0xFF)",
                   fmtHex32(lo) + " " + fmtHex32(hi), !guard_ok});
}

// PING response 0x0FF in app-mode: bootVer @ 0..3 + appVer @ 4..7 (M.m.p.b).
void appendAppPingRspRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"bytes 0..3", "4 bytes (M.m.p.b)", "boot version",
                   versionText(f.data, 0), false});
    out.push_back({"bytes 4..7", "4 bytes (M.m.p.b)", "app version",
                   versionText(f.data, 4), false});
}

// RK_ANS_EMERGENCY_STOP (0x003, src=RK, dst=BROADCAST): all 0xFF; signals
// that the operator hit one of the e-stop buttons on the rotary knob.
void appendRkEmergencyAnsRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    const uint32_t lo = readU32(f.data, 0);
    const uint32_t hi = readU32(f.data, 4);
    const bool guard_ok = (lo == 0xFFFFFFFFu) && (hi == 0xFFFFFFFFu);
    out.push_back({"bytes 0..7", "8 bytes", "estop broadcast (no payload)",
                   fmtHex32(lo) + " " + fmtHex32(hi), !guard_ok});
}

// RK_CMD_BRACKET_SERVO / RK_CMD_MOTOR_LOCK_SERVO (host → RK):
//   [0..3] set_pos f32 (0..100 %, clamp on RK)
//   [4]    pwm_enable (0/1)
//   [5..7] guard 0xFF
void appendRkServoCtrlRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    const float pos = readFloat(f.data, 0);
    out.push_back({"bytes 0..3", "4 bytes (float LE)", "set_pos (%)",
                   fmtFloat(pos), pos < 0.0f || pos > 100.0f});
    out.push_back({"byte 4", "1 byte", "pwm_enable",
                   boolText(f.data[4] != 0), false});
    const bool guard_ok = (f.data[5] == 0xFF && f.data[6] == 0xFF && f.data[7] == 0xFF);
    out.push_back({"bytes 5..7", "3 bytes", "guard 0xFF",
                   guard_ok ? "all 0xFF" : "unexpected value", !guard_ok});
}

// RK_CMD_LED (host → RK): brightness byte + guard 0xFF.
void appendRkLedRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    out.push_back({"byte 0", "1 byte", "led value",
                   std::to_string(static_cast<int>(f.data[0])), false});
    const bool guard_ok = (f.data[1] == 0xFF && f.data[2] == 0xFF && f.data[3] == 0xFF &&
                           f.data[4] == 0xFF && f.data[5] == 0xFF && f.data[6] == 0xFF &&
                           f.data[7] == 0xFF);
    out.push_back({"bytes 1..7", "7 bytes", "guard 0xFF",
                   guard_ok ? "all 0xFF" : "unexpected value", !guard_ok});
}

// -- Triggered Capture (0x0E0 req / 0x0E1 rsp) -----------------------------
// payload[0] = sub-cmd; remaining bytes per-sub. Used for both request and
// response — we dispatch on the sub-cmd byte itself, not on cmd id.

const char* capStateName(uint8_t s) {
    switch (s) {
        case 0: return "IDLE";
        case 1: return "ARMED";
        case 2: return "TRIGGERED";
        case 3: return "DONE";
        case 4: return "ERROR";
        default: return "?";
    }
}

const char* capTriggerName(uint8_t t) {
    switch (t) {
        case 0: return "IMMEDIATE";
        case 1: return "GREATER";
        case 2: return "LESS";
        case 3: return "RISING";
        case 4: return "FALLING";
        case 5: return "CHANGED";
        default: return "?";
    }
}

const char* capSlotTypeName(uint8_t t) {
    switch (t) {
        case 0: return "U8";
        case 1: return "I8";
        case 2: return "U16";
        case 3: return "I16";
        case 4: return "U32";
        case 5: return "I32";
        case 6: return "F32";
        default: return "?";
    }
}

const char* capSubNameVerbose(uint8_t s) {
    switch (s) {
        case kCapSubReset:      return "RESET";
        case kCapSubSetSlot:    return "SET_SLOT";
        case kCapSubSetConfig:  return "SET_CONFIG";
        case kCapSubSetTrigger: return "SET_TRIGGER";
        case kCapSubArm:        return "ARM";
        case kCapSubStop:       return "STOP";
        case kCapSubStatusReq:  return "STATUS_REQ";
        case kCapSubReadChunk:  return "READ_CHUNK";
        case kCapSubCrc:        return "CRC_REQ";
        case kCapAnsAck:        return "ACK";
        case kCapAnsStatus:     return "STATUS_RSP";
        case kCapAnsCrc:        return "CRC_RSP";
        case kCapAnsData:       return "DATA";
        case kCapAnsDataEnd:    return "DATA_END";
        case kCapAnsError:      return "ERROR";
        default:                return "?";
    }
}

const char* capErrCodeName(uint8_t e) {
    switch (e) {
        case 0: return "OK";
        case 1: return "BAD_SUB";
        case 2: return "BAD_STATE";
        case 3: return "BAD_SLOT";
        case 4: return "BAD_CONFIG";
        case 5: return "BAD_TRIGGER";
        case 6: return "BUF_TOO_SMALL";
        case 7: return "NOT_IMPLEMENTED";
        default: return "?";
    }
}

void appendCapReqRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    const uint8_t sub = f.data[0];
    out.push_back({"byte 0", "1 byte", "sub-cmd",
                   byteWithName(sub, capSubNameVerbose(sub)), false});
    switch (sub) {
        case kCapSubSetSlot:
            out.push_back({"byte 1", "1 byte", "slot_idx (0..5)",
                           std::to_string(static_cast<int>(f.data[1])), f.data[1] >= 6});
            out.push_back({"bytes 2..5", "4 bytes (uint32 LE)", "address",
                           fmtHex32(readU32(f.data, 2)), false});
            out.push_back({"byte 6", "1 byte", "size (1/2/4)",
                           std::to_string(static_cast<int>(f.data[6])),
                           f.data[6] != 1 && f.data[6] != 2 && f.data[6] != 4});
            out.push_back({"byte 7", "1 byte", "slot type",
                           byteWithName(f.data[7], capSlotTypeName(f.data[7])),
                           false});
            break;
        case kCapSubSetConfig:
            out.push_back({"byte 1", "1 byte", "slot_count (1..6)",
                           std::to_string(static_cast<int>(f.data[1])),
                           f.data[1] < 1 || f.data[1] > 6});
            out.push_back({"bytes 2..3", "2 bytes (uint16 LE)", "period_us",
                           std::to_string(readU16(f.data, 2)), false});
            out.push_back({"byte 4", "1 byte", "pre_pct",
                           std::to_string(static_cast<int>(f.data[4])), false});
            out.push_back({"byte 5", "1 byte", "post_pct",
                           std::to_string(static_cast<int>(f.data[5])), false});
            out.push_back({"byte 6", "1 byte", "buffer_kb",
                           std::to_string(static_cast<int>(f.data[6])), false});
            out.push_back({"byte 7", "1 byte", "reserved",
                           fmtHex8(f.data[7]), false});
            break;
        case kCapSubSetTrigger:
            out.push_back({"byte 1", "1 byte", "trigger mode",
                           byteWithName(f.data[1], capTriggerName(f.data[1])),
                           false});
            out.push_back({"byte 2", "1 byte", "trigger slot idx",
                           std::to_string(static_cast<int>(f.data[2])), false});
            out.push_back({"bytes 3..6", "4 bytes (int32 LE)", "threshold",
                           std::to_string(static_cast<int32_t>(readU32(f.data, 3))),
                           false});
            out.push_back({"byte 7", "1 byte", "reserved",
                           fmtHex8(f.data[7]), false});
            break;
        case kCapSubReadChunk:
            out.push_back({"bytes 1..2", "2 bytes (uint16 LE)", "byte_offset",
                           std::to_string(readU16(f.data, 1)), false});
            out.push_back({"bytes 3..4", "2 bytes (uint16 LE)", "byte_count",
                           std::to_string(readU16(f.data, 3)), false});
            out.push_back({"bytes 5..7", "3 bytes", "reserved",
                           fmtHex8(f.data[5]) + " " + fmtHex8(f.data[6]) + " " +
                           fmtHex8(f.data[7]), false});
            break;
        case kCapSubReset:
        case kCapSubArm:
        case kCapSubStop:
        case kCapSubStatusReq:
        case kCapSubCrc:
            // No further fields; the sub-cmd byte alone fully specifies the
            // request. Remaining 7 bytes are filler (typically 0).
            out.push_back({"bytes 1..7", "7 bytes", "filler", "ignored", false});
            break;
        default:
            out.push_back({"bytes 1..7", "7 bytes", "payload (UNKNOWN)",
                           fmtHex32(readU32(f.data, 1)) + " ...", true});
            break;
    }
}

void appendCapRspRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (!hasBytes(f.data, 0, 8)) return;
    const uint8_t sub = f.data[0];
    out.push_back({"byte 0", "1 byte", "sub-cmd",
                   byteWithName(sub, capSubNameVerbose(sub)), false});
    switch (sub) {
        case kCapAnsAck:
        case kCapAnsError:
            out.push_back({"byte 1", "1 byte", "original sub-cmd",
                           byteWithName(f.data[1], capSubNameVerbose(f.data[1])),
                           false});
            out.push_back({"byte 2", "1 byte", "status",
                           byteWithName(f.data[2], capErrCodeName(f.data[2])),
                           f.data[2] != 0});
            out.push_back({"bytes 3..7", "5 bytes", "reserved",
                           fmtHex32(readU32(f.data, 3)) + " ...", false});
            break;
        case kCapAnsStatus:
            out.push_back({"byte 1", "1 byte", "state",
                           byteWithName(f.data[1], capStateName(f.data[1])), false});
            out.push_back({"byte 2", "1 byte", "slot_count",
                           std::to_string(static_cast<int>(f.data[2])), false});
            out.push_back({"bytes 3..4", "2 bytes (uint16 LE)", "samples_captured",
                           std::to_string(readU16(f.data, 3)), false});
            out.push_back({"bytes 5..6", "2 bytes (uint16 LE)", "trigger_sample_idx",
                           std::to_string(readU16(f.data, 5)), false});
            out.push_back({"byte 7", "1 byte (packed)", "err_code+full",
                           fmtHex8(f.data[7]), f.data[7] != 0});
            appendPackedFlagRows(out, f.data[7], {
                {7, "buffer_full"},
            });
            break;
        case kCapAnsCrc:
            // [0x96, crc32_le32, totalBytes_le16, 0xFF]
            out.push_back({"bytes 1..4", "4 bytes (uint32 LE)", "crc32",
                           fmtHex32(readU32(f.data, 1)), false});
            out.push_back({"bytes 5..6", "2 bytes (uint16 LE)", "totalBytes echo",
                           std::to_string(readU16(f.data, 5)), false});
            out.push_back({"byte 7", "1 byte", "guard 0xFF",
                           fmtHex8(f.data[7]), f.data[7] != 0xFF});
            break;
        case kCapAnsData:
        case kCapAnsDataEnd: {
            out.push_back({"bytes 1..2", "2 bytes (uint16 LE)", "byte_offset",
                           std::to_string(readU16(f.data, 1)), false});
            std::ostringstream o;
            o << std::uppercase << std::hex << std::setfill('0');
            const int payloadEnd = (sub == kCapAnsDataEnd) ? 7 : 8;
            for (int i = 3; i < payloadEnd; ++i) {
                if (i > 3) o << ' ';
                o << std::setw(2) << static_cast<int>(f.data[i]);
            }
            out.push_back({"bytes 3.." + std::to_string(payloadEnd - 1),
                           std::to_string(payloadEnd - 3) + " bytes",
                           sub == kCapAnsDataEnd ? "payload (last)" : "payload (5B)",
                           o.str(), false});
            if (sub == kCapAnsDataEnd) {
                out.push_back({"byte 7", "1 byte", "validBytes hint (1..4)",
                               std::to_string(static_cast<int>(f.data[7])),
                               f.data[7] == 0 || f.data[7] > 5});
            }
            break;
        }
        default:
            out.push_back({"bytes 1..7", "7 bytes", "payload (UNKNOWN)",
                           fmtHex32(readU32(f.data, 1)) + " ...", true});
            break;
    }
}

void appendReadMemReqRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (f.data.size() < 8) return;
    out.push_back({"byte 0", "1 byte", "cmd (0x03 = READ_MEM)", fmtHex8(f.data[0]), false});
    out.push_back({"byte 1", "1 byte", "seq", std::to_string(static_cast<int>(f.data[1])), false});
    out.push_back({"bytes 2..5", "4 bytes (uint32 LE)", "address", fmtHex32(readU32(f.data, 2)), false});
    out.push_back({"byte 6", "1 byte", "size to read", std::to_string(static_cast<int>(f.data[6])), false});
}

void appendReadMemRspRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (f.data.size() < 8) return;
    out.push_back({"byte 0", "1 byte", "cmd (0x83 = READ_MEM rsp)", fmtHex8(f.data[0]), false});
    out.push_back({"byte 1", "1 byte", "seq", std::to_string(static_cast<int>(f.data[1])), false});
    out.push_back({"byte 2", "1 byte", "status (0=ok)", std::to_string(static_cast<int>(f.data[2])), f.data[2] != 0});
    out.push_back({"byte 3", "1 byte", "size returned", std::to_string(static_cast<int>(f.data[3])), false});
    out.push_back({"bytes 4..7", "4 bytes (LE value)", "value", fmtHex32(readU32(f.data, 4)), false});
}

// === Bootloader / file-transfer (mod=BOOT) =====================
// Sources of truth on the device side:
//   include/can_bootloader.h (cmd codes)
//   src/devices/can_bus/prop_can_protocol.cpp (payload layout)
//   MOTOR/.../prop_can.h, prop_can.c (PropCanErrCode_e)
// Same protocol is reused for motor and RK; this is keyed off the
// 12-bit cmd in mod==0 and the dst byte tells you which target.

const char* bootErrCodeName(uint8_t code)
{
    switch (code) {
        case 0:    return "OK";
        case 1:    return "CRC_MMSG";
        case 2:    return "CRC_BLOCK";
        case 3:    return "CRC_FILE";
        case 4:    return "ADDR";
        case 5:    return "CLEAR_FLASH";
        case 6:    return "WRITE_FLASH";
        case 7:    return "ADDR_SIZE";
        case 8:    return "SUB_INDEX_MMSG";
        case 9:    return "START_BAD_ZERO_ADDR";
        case 10:   return "START_BAD_CRC";
        case 11:   return "HOST_ID_BAD";
        case 0xFF: return "UNKNOWN_COMMAND";
        default:   return "?";
    }
}

void appendBootHeaderFileRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (f.data.size() < 8) return;
    out.push_back({"bytes 0..3", "4 bytes (uint32 LE)", "mcu_rom_addr (flash base)", fmtHex32(readU32(f.data, 0)), false});
    out.push_back({"bytes 4..7", "4 bytes (uint32 LE)", "file_size (bytes)", std::to_string(readU32(f.data, 4)), false});
}

void appendBootBlockCrcRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (f.data.size() < 8) return;
    out.push_back({"bytes 0..3", "4 bytes (uint32 LE)", "block CRC32 (STM32 0x04C11DB7)", fmtHex32(readU32(f.data, 0)), false});
    out.push_back({"bytes 4..5", "2 bytes (uint16 LE)", "block_idx", std::to_string(readU16(f.data, 4)), false});
    out.push_back({"bytes 6..7", "2 bytes (uint16 LE)", "block_total", std::to_string(readU16(f.data, 6)), false});
}

void appendBootMmsgHeaderRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (f.data.size() < 8) return;
    out.push_back({"bytes 0..1", "2 bytes (uint16 LE)", "mmsg_size (payload bytes)", std::to_string(readU16(f.data, 0)), false});
    out.push_back({"bytes 2..3", "2 bytes (uint16 LE)", "mmsg_crc (low half of CRC32)", fmtHex8(f.data[3]) + " " + fmtHex8(f.data[2]), false});
    out.push_back({"bytes 4..5", "2 bytes (uint16 LE)", "mmsg_total (frames in this MMSG)", std::to_string(readU16(f.data, 4)), false});
    out.push_back({"byte 6", "1 byte", "guard (always 0xF)", fmtHex8(f.data[6]), f.data[6] != 0x0F});
    out.push_back({"byte 7", "1 byte", "mmsg_idx (this MMSG within block)", std::to_string(static_cast<int>(f.data[7])), false});
}

void appendBootMmsgDataRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (f.data.size() < 8) return;
    std::ostringstream payload;
    payload << std::uppercase << std::hex << std::setfill('0');
    for (int i = 0; i < 7; ++i) {
        if (i) payload << ' ';
        payload << std::setw(2) << static_cast<int>(f.data[i]);
    }
    out.push_back({"bytes 0..6", "7 bytes", "payload", payload.str(), false});
    out.push_back({"byte 7", "1 byte", "msg_idx (this 7-byte chunk in MMSG)", std::to_string(static_cast<int>(f.data[7])), false});
}

void appendBootEraseRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (f.data.size() < 8) return;
    out.push_back({"bytes 0..3", "4 bytes (uint32 LE)", "erase_addr", fmtHex32(readU32(f.data, 0)), false});
    out.push_back({"bytes 4..7", "4 bytes (uint32 LE)", "erase_len (bytes)", std::to_string(readU32(f.data, 4)), false});
}

void appendBootFinishRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (f.data.size() < 8) return;
    out.push_back({"bytes 0..3", "4 bytes (uint32 LE)", "file CRC32", fmtHex32(readU32(f.data, 0)), false});
    out.push_back({"bytes 4..7", "4 bytes", "0xFFFFFFFF guard", fmtHex32(readU32(f.data, 4)), readU32(f.data, 4) != 0xFFFFFFFFu});
}

void appendBootErrorRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (f.data.size() < 8) return;
    const uint8_t code = f.data[0];
    out.push_back({"byte 0", "1 byte", std::string("error code (") + bootErrCodeName(code) + ")",
                   std::to_string(static_cast<int>(code)), code != 0});
    out.push_back({"bytes 1..7", "7 bytes", "context (filler 0xFF)",
                   fmtHex8(f.data[1]) + " ...", false});
}

void appendBootEraseOkRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (f.data.size() < 8) return;
    out.push_back({"bytes 0..7", "8 bytes", "ERASE_OK (no payload, all 0xFF guard)",
                   fmtHex32(readU32(f.data, 0)) + " " + fmtHex32(readU32(f.data, 4)), false});
}

void appendBootMmsgOkRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (f.data.size() < 8) return;
    out.push_back({"byte 0", "1 byte", "mmsg_idx (echo)", std::to_string(static_cast<int>(f.data[0])), false});
    out.push_back({"bytes 1..7", "7 bytes", "guard 0xFF", fmtHex8(f.data[7]), false});
}

void appendBootBlockOkRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (f.data.size() < 8) return;
    out.push_back({"bytes 0..1", "2 bytes (uint16 LE)", "block_idx (echo)", std::to_string(readU16(f.data, 0)), false});
    out.push_back({"bytes 2..7", "6 bytes", "guard 0xFF", fmtHex8(f.data[7]), false});
}

void appendBootFinishOkRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (f.data.size() < 8) return;
    out.push_back({"bytes 0..3", "4 bytes (uint32 LE)", "file CRC32 (echo)", fmtHex32(readU32(f.data, 0)), false});
    out.push_back({"bytes 4..7", "4 bytes", "guard 0xFFFFFFFF", fmtHex32(readU32(f.data, 4)), false});
}

void appendBootGoToBootRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (f.data.size() < 8) return;
    // Magic bytes from CanBootloader::goToBoot — 0xCC,0xCC,0xCC,0xCC,0xDD,0xDD,0xDD,0xDD.
    const uint32_t magic_lo = readU32(f.data, 0);
    const uint32_t magic_hi = readU32(f.data, 4);
    const bool magic_ok = (magic_lo == 0xCCCCCCCCu) && (magic_hi == 0xDDDDDDDDu);
    out.push_back({"bytes 0..3", "4 bytes", "magic 0xCCCCCCCC", fmtHex32(magic_lo), !magic_ok});
    out.push_back({"bytes 4..7", "4 bytes", "magic 0xDDDDDDDD", fmtHex32(magic_hi), !magic_ok});
}

void appendBootGoToAppRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (f.data.size() < 8) return;
    const uint32_t lo = readU32(f.data, 0);
    const uint32_t hi = readU32(f.data, 4);
    const bool guard_ok = (lo == 0xFFFFFFFFu) && (hi == 0xFFFFFFFFu);
    out.push_back({"bytes 0..7", "8 bytes", "guard 0xFF (no payload)",
                   fmtHex32(lo) + " " + fmtHex32(hi), !guard_ok});
}

void appendBootPingReqRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (f.data.size() < 8) return;
    out.push_back({"bytes 0..7", "8 bytes", "ping (no payload, all 0xFF)",
                   fmtHex32(readU32(f.data, 0)) + " " + fmtHex32(readU32(f.data, 4)), false});
}

void appendBootPingRspRows(std::vector<TooltipField>& out, const CanFrame& f) {
    if (f.data.size() < 8) return;
    // Bootloader ping response carries a version 4-tuple in bytes [0..3].
    out.push_back({"bytes 0..3", "4 bytes (M.m.p.b)", "fw version (boot or app, depending on currently-running image)",
                   versionText(f.data, 0), false});
    out.push_back({"bytes 4..7", "4 bytes", "context / guard 0xFF",
                   fmtHex32(readU32(f.data, 4)), false});
}

bool appendBootloaderRows(std::vector<TooltipField>& out, uint16_t cmd, const CanFrame& f)
{
    switch (cmd) {
        case 0x0C0: appendBootHeaderFileRows(out, f);  return true;
        case 0x0C1: appendBootBlockCrcRows(out, f);    return true;
        case 0x0C2: appendBootMmsgHeaderRows(out, f);  return true;
        case 0x0C3: appendBootMmsgDataRows(out, f);    return true;
        case 0x0CE: appendBootEraseRows(out, f);       return true;
        case 0x0CF: appendBootFinishRows(out, f);      return true;
        case 0xAC0: appendBootErrorRows(out, f);       return true;
        case 0xAC1: appendBootEraseOkRows(out, f);     return true;
        case 0xAC2: appendBootMmsgOkRows(out, f);      return true;
        case 0xAC3: appendBootBlockOkRows(out, f);     return true;
        case 0xACF: appendBootFinishOkRows(out, f);    return true;
        case 0xB00: appendBootGoToBootRows(out, f);    return true;
        case 0xB01: appendBootGoToAppRows(out, f);     return true;
        case 0xFFF: appendBootPingReqRows(out, f);     return true;
        case 0x0FF: appendBootPingRspRows(out, f);     return true;
        default:    return false;
    }
}

/* Unified READ protocol rows (motor app, mod==1). Layouts of HEADER_BLOCK
 * / HEADER_MMSG / DATA_MMSG / FILE_FINISH are byte-identical to the
 * bootloader file-transfer side, so we reuse the bootloader row
 * appenders. HEADER_FILE (0xD2 request and 0xAD2 ack) layout is also
 * identical to bootloader's 0xC0. New: ERROR (0xAD8) with err code in
 * byte 0. */
bool appendUnifiedReadRows(std::vector<TooltipField>& out, uint16_t cmd, const CanFrame& f)
{
    switch (cmd) {
        case 0x0D2: appendBootHeaderFileRows(out, f); return true;   // host → motor: addr + size
        case 0xAD2: appendBootHeaderFileRows(out, f); return true;   // motor → host: addr + capped_size echo
        case 0x0D3: appendBootBlockCrcRows(out, f);   return true;   // motor → host: block_crc + idx/total
        case 0x0D4: appendBootMmsgHeaderRows(out, f); return true;   // motor → host: mmsg header
        case 0x0D5: appendBootMmsgDataRows(out, f);   return true;   // motor → host: 7B payload + msg_idx
        case 0x0DF: appendBootFinishRows(out, f);     return true;   // motor → host: file CRC
        case 0xAD8: appendBootErrorRows(out, f);      return true;   // either direction: ERROR
        default:    return false;
    }
}

// STREAM_* subscription protocol (0x0E2..0x0E9). Motor-side per-sample push
// streaming that replaced slow polled READ_MEM for live plotting. Wire layouts
// mirror DebugProtocol.h Stream* structs / makeStream* builders. Lives in
// app-mode (mod=1), motor target (src/dst 0x02; STREAM_VALUE dst=BROADCAST).
const char* streamAddStatusName(uint8_t s)
{
    switch (s) {
        case 0: return "OK";
        case 1: return "TABLE_FULL";
        case 2: return "BAD_ADDR";
        case 3: return "BAD_TYPE";
        case 4: return "DUPLICATE";
        default: return "?";
    }
}

// size_type byte: bits 0-1 size code (0=1B,1=2B,2=4B), bits 4-7 type (0=u,1=i,2=f).
std::string streamSizeTypeText(uint8_t st)
{
    const uint8_t sizeCode = st & 0x03;
    const uint8_t typeCode = (st >> 4) & 0x0F;
    const char* tc = (typeCode == 0) ? "u" : (typeCode == 1) ? "i" : (typeCode == 2) ? "f" : "?";
    const int bits = (sizeCode == 0) ? 8 : (sizeCode == 1) ? 16 : (sizeCode == 2) ? 32 : 0;
    std::ostringstream o;
    o << tc << bits << " (" << fmtHex8(st) << ")";
    return o.str();
}

void appendStreamRows(std::vector<TooltipField>& out, uint16_t cmd, const CanFrame& f)
{
    if (!hasBytes(f.data, 0, 8)) return;
    const auto& d = f.data;
    switch (cmd) {
        case kCanCmdStreamAddReq:        // PC -> Motor
            out.push_back({"byte 0", "1 byte", "op", byteWithName(d[0], "ADD (0x01)"), false});
            out.push_back({"byte 1", "1 byte", "seq", std::to_string((int)d[1]), false});
            out.push_back({"bytes 2..5", "4 bytes (uint32 LE)", "address", fmtHex32(readU32(d, 2)), false});
            out.push_back({"byte 6", "1 byte", "size_type", streamSizeTypeText(d[6]), false});
            out.push_back({"byte 7", "1 byte", "period_hint (0=motor decides)", std::to_string((int)d[7]), false});
            break;
        case kCanCmdStreamAddRsp:        // Motor -> PC
            out.push_back({"byte 0", "1 byte", "status", byteWithName(d[0], streamAddStatusName(d[0])), d[0] != 0});
            out.push_back({"byte 1", "1 byte", "seq", std::to_string((int)d[1]), false});
            out.push_back({"byte 2", "1 byte", "slot_id", d[2] == 0xFF ? "none (0xFF)" : std::to_string((int)d[2]), false});
            out.push_back({"byte 3", "1 byte", "used (subs in use)", std::to_string((int)d[3]), false});
            out.push_back({"byte 4", "1 byte", "total (table size)", std::to_string((int)d[4]), false});
            out.push_back({"byte 5", "1 byte", "actual_period_ticks", std::to_string((int)d[5]), false});
            out.push_back({"bytes 6..7", "2 bytes (uint16 LE)", "normal_dropped", std::to_string(readU16(d, 6)), readU16(d, 6) != 0});
            break;
        case kCanCmdStreamRemove:        // PC -> Motor (silent)
            out.push_back({"byte 0", "1 byte", "op", byteWithName(d[0], "REMOVE (0x02)"), false});
            out.push_back({"byte 1", "1 byte", "seq", std::to_string((int)d[1]), false});
            out.push_back({"byte 2", "1 byte", "slot_id (0xFF=by addr)", std::to_string((int)d[2]), false});
            out.push_back({"bytes 3..6", "4 bytes (uint32 LE)", "address", fmtHex32(readU32(d, 3)), false});
            out.push_back({"byte 7", "1 byte", "reserved guard", fmtHex8(d[7]), d[7] != 0xFF});
            break;
        case kCanCmdStreamClear:         // PC -> Motor (silent)
            out.push_back({"byte 0", "1 byte", "op", byteWithName(d[0], "CLEAR (0x03)"), false});
            out.push_back({"bytes 1..7", "7 bytes", "reserved guard", "0xFF", false});
            break;
        case kCanCmdStreamStatusReq:     // PC -> Motor
            out.push_back({"byte 0", "1 byte", "op", byteWithName(d[0], "STATUS_REQ (0x04)"), false});
            out.push_back({"bytes 1..7", "7 bytes", "reserved guard", "0xFF", false});
            break;
        case kCanCmdStreamStatusRsp:     // Motor -> PC
            out.push_back({"byte 0", "1 byte", "used (subs in use)", std::to_string((int)d[0]), false});
            out.push_back({"byte 1", "1 byte", "total (table size)", std::to_string((int)d[1]), false});
            out.push_back({"byte 2", "1 byte", "capture_active", boolText(d[2] != 0), d[2] != 0});
            out.push_back({"byte 3", "1 byte", "period_ticks_now", std::to_string((int)d[3]), false});
            out.push_back({"bytes 4..5", "2 bytes (uint16 LE)", "normal_dropped", std::to_string(readU16(d, 4)), readU16(d, 4) != 0});
            out.push_back({"bytes 6..7", "2 bytes (uint16 LE)", "capture_dropped", std::to_string(readU16(d, 6)), readU16(d, 6) != 0});
            break;
        case kCanCmdStreamValue:         // Motor -> BROADCAST
            out.push_back({"byte 0", "1 byte", "slot_id", std::to_string((int)d[0]), false});
            out.push_back({"byte 1", "1 byte", "size_type", streamSizeTypeText(d[1]), false});
            out.push_back({"bytes 2..5", "4 bytes", "value (raw; decode per size_type)", fmtHex32(readU32(d, 2)), false});
            out.push_back({"byte 6", "1 byte", "tick_lsb", std::to_string((int)d[6]), false});
            out.push_back({"byte 7", "1 byte", "flags", fmtHex8(d[7]), d[7] != 0});
            break;
        case kCanCmdStreamHeartbeat:     // PC -> Motor (silent)
            out.push_back({"byte 0", "1 byte", "op", byteWithName(d[0], "HEARTBEAT (0x05)"), false});
            out.push_back({"bytes 1..7", "7 bytes", "reserved guard", "0xFF", false});
            break;
        default: break;
    }
}

} // namespace

std::vector<TooltipField> buildFrameTooltipTable(const CanFrame& frame)
{
    std::vector<TooltipField> out;
    if (!frame.extended) return out;

    const uint8_t src = static_cast<uint8_t>(frame.id & 0xFF);
    const uint8_t dst = static_cast<uint8_t>((frame.id >> 8) & 0xFF);
    const uint8_t mod = static_cast<uint8_t>((frame.id >> 28) & 0x01);
    const uint16_t cmd = static_cast<uint16_t>((frame.id >> 16) & 0x0FFF);

    if (mod == 1 && cmd == kCanCmdReadMemReq && src == kCanAddrPc) {
        appendReadMemReqRows(out, frame);
        return out;
    }
    if (mod == 1 && cmd == kCanCmdReadMemRsp && dst == kCanAddrPc) {
        appendReadMemRspRows(out, frame);
        return out;
    }

    // Triggered Capture lives in app-mode on cmd 0x0E0 (PC → MCU) and 0x0E1
    // (MCU → PC). Sub-cmd in payload[0] dispatches per-sub.
    if (mod == 1 && cmd == kCanCmdCapReq && src == kCanAddrPc) {
        appendCapReqRows(out, frame);
        return out;
    }
    if (mod == 1 && cmd == kCanCmdCapRsp && dst == kCanAddrPc) {
        appendCapRspRows(out, frame);
        return out;
    }

    // Bootloader / file-transfer protocol lives in mod==BOOT (the M-bit
    // distinguishes the same 0x0C0..0x0CF / 0xAC0..0xACF command space
    // from the app-mode telemetry that overlaps it). Same handler for
    // motor and RK targets — dst byte tells you which one.
    if (mod == 0 && appendBootloaderRows(out, cmd, frame)) {
        return out;
    }

    const bool motor = (src == 0x02 || dst == 0x02);
    const bool rk    = (src == 0x03 || dst == 0x03);
    const bool main  = (src == 0x01 || dst == 0x01);

    if (motor) {
        switch (cmd) {
            case 0x000: appendMotorMagicRows(out, frame);                         break;
            case 0x001: appendMotorMagicRows(out, frame);                         break;
            case 0x002: appendMotorCtrlRows(out, frame);                          break;
            case 0x003: appendMotorEStopRows(out, frame);                         break;
            case 0x004: appendMotorPfcControlRows(out, frame);                    break;
            case 0x005: appendMotorConfigRows(out, frame);                        break;
            case 0x008: appendMotorToneRows(out, frame);                          break;
            case 0x0A1: appendMotorScalarRows(out, frame, "P (power, W)");        break;
            case 0x0A2: appendMotorScalarRows(out, frame, "Uq (q-axis voltage, V)"); break;
            case 0x0A3: appendMotorScalarRows(out, frame, "rpm");                 break;
            case 0x0A4: appendMotorScalarRows(out, frame, "Udc (DC bus, V)");     break;
            case 0x0A5: appendMotorTempRows(out, frame);                          break;
            case 0x0A6: appendMotorFaultRows(out, frame);                         break;
            case 0x0A7: appendMotorCanDiagRows(out, frame);                       break;
            case 0x0A8: appendMotorSysDiagRows(out, frame);                       break;
            case 0x0A9: appendMotorCurDiagRows(out, frame);                       break;
            case 0x0AA: appendMotorPfcStateRows(out, frame);                      break;
            case 0x0AB: appendMotorConfigRspRows(out, frame);                     break;
            case 0x0AC: appendMotorProfilerRows(out, frame);                      break;
            case 0x0AD: appendMotorThetaCalRows(out, frame);                      break;
            case 0x0AE: appendMotorSchedDiagRows(out, frame);                     break;
            case 0x0AF: appendMotorSchedDiag2Rows(out, frame);                    break;
            case 0x0FF: appendAppPingRspRows(out, frame);                         break;
            case 0xFFF: appendMotorPingReqRows(out, frame);                       break;
            /* Unified READ protocol (app-mode) — same wire layouts as
             * bootloader FT (mod=0 0xCx) but lives in mod=1. See
             * appendUnifiedReadRows for details. */
            case 0x0D2: case 0x0D3: case 0x0D4: case 0x0D5: case 0x0DF:
            case 0xAD2: case 0xAD3: case 0xAD4: case 0xAD8: case 0xADF:
                appendUnifiedReadRows(out, cmd, frame); break;
            /* Unified WRITE protocol (Phase 3, 2026-05-13). Same wire
             * codes as bootloader file-transfer (0xC0..0xCF / 0xAC0..
             * 0xACF) but lives in mod==1 (app-mode). Motor app routes
             * HEADER_FILE addr==0x080E0000 into config-staging mode --
             * bytes go to bufBlock, MotorConfigScatter on FILE_FINISH,
             * no flash erase/write while the FOC ISR is running. */
            case 0x0C0: case 0x0C1: case 0x0C2: case 0x0C3:
            case 0x0CE: case 0x0CF:
            case 0xAC0: case 0xAC1: case 0xAC2: case 0xAC3: case 0xACF:
                appendBootloaderRows(out, cmd, frame); break;
            /* STREAM_* subscription protocol (0x0E2..0x0E9, app-mode). Motor
             * per-sample push streaming for live plotting. */
            case 0x0E2: case 0x0E3: case 0x0E4: case 0x0E5:
            case 0x0E6: case 0x0E7: case 0x0E8: case 0x0E9:
                appendStreamRows(out, cmd, frame); break;
            default: break;
        }
    } else if (rk) {
        // For RK cmds 0x002..0x004 the meaning depends on direction.
        // src==RK answers vs host requests: pick the helper accordingly.
        const bool srcIsRk = (src == 0x03);
        switch (cmd) {
            case 0x002:
                // src==RK: APP_RK_ANS_MOTOR_CONTROL (status / 8 B payload).
                // dst==RK: APP_RK_CMD_LED (data[0]=brightness + guard).
                if (!srcIsRk) appendRkLedRows(out, frame);
                break;
            case 0x003:
                if (srcIsRk) appendRkEmergencyAnsRows(out, frame);
                else         appendRkServoCtrlRows(out, frame);   // bracket servo
                break;
            case 0x004:
                if (!srcIsRk) appendRkServoCtrlRows(out, frame);   // motor-lock servo
                break;
            case 0x005: appendRkMotorCtrlRows(out, frame); break;
            case 0x0A0: appendRkSwitchRows(out, frame);  break;
            case 0x0A1: appendRkAnglesRows(out, frame);  break;
            case 0x0FF: appendAppPingRspRows(out, frame); break;
            case 0xFFF: appendMotorPingReqRows(out, frame); break;
            default: break;
        }
    } else if (main) {
        switch (cmd) {
            case 0x010: appendMainDiagModeRows(out, frame);       break;
            case 0x011: appendMainHeartbeatRows(out, frame);      break;
            case 0x012: appendMainMotorProxyRows(out, frame);     break;
            case 0x013: appendMainRkProxyRows(out, frame);        break;
            case 0x014: appendMainPfcProxyRows(out, frame);       break;
            case 0x100: appendMainWifiModeSetRows(out, frame);    break;
            case 0xB00: appendMainWifiStatusRows(out, frame);     break;
            case 0xB10: appendMainDiagStatusRows(out, frame);     break;
            case 0xB11: appendMainDiagTelemetryRows(out, frame);  break;
            default: break;
        }
    }
    return out;
}

} // namespace drivescope
