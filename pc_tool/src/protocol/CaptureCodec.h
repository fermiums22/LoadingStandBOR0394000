#pragma once

#include "can/CanFrame.h"
#include "protocol/PropCanFrame.h"
#include "protocol/WireValue.h"   // WireValueType (neutral scalar type)

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace drivescope {

// -----------------------------------------------------------------------------
// Triggered Capture wire codec. Pure pack/decode of the CAP_REQ (0x0E0) /
// CAP_RSP (0x0E1) protocol: subcommand in payload[0]. No UI, no session/retry
// (the capture FSM lives in MainUi/services), no transport, no logging, no
// mutable state. CRC uses protocol/Crc32 (capture_crc32 = MPEG-2 byte-wise).
// Canonical spec: umbrella protocol/ddv2_protocol.yaml (capture).
// -----------------------------------------------------------------------------

inline constexpr uint16_t kCanCmdCapReq = 0x0E0;  // PC -> MCU
inline constexpr uint16_t kCanCmdCapRsp = 0x0E1;  // MCU -> PC

// Subcommands carried in payload[0] of a CAP_REQ frame.
inline constexpr uint8_t kCapSubReset       = 0x10;
inline constexpr uint8_t kCapSubSetSlot     = 0x11;
inline constexpr uint8_t kCapSubSetConfig   = 0x12;
inline constexpr uint8_t kCapSubSetTrigger  = 0x13;
inline constexpr uint8_t kCapSubArm         = 0x14;
inline constexpr uint8_t kCapSubStop        = 0x15;
inline constexpr uint8_t kCapSubStatusReq   = 0x16;
inline constexpr uint8_t kCapSubReadChunk   = 0x17;
inline constexpr uint8_t kCapSubCrc         = 0x18;

// Subcommands carried in payload[0] of a CAP_RSP frame.
inline constexpr uint8_t kCapAnsAck       = 0x90;  // [0x90, original_sub, status, ...]
inline constexpr uint8_t kCapAnsStatus    = 0x95;  // see CaptureStatus
inline constexpr uint8_t kCapAnsCrc       = 0x96;  // [0x96, crc32_le32, totalBytes_le16, 0xFF]
inline constexpr uint8_t kCapAnsData      = 0x97;  // [0x97, byteOffset_le16, payload×5]
inline constexpr uint8_t kCapAnsDataEnd   = 0x98;  // last frame of a CAP_READ_CHUNK stream
inline constexpr uint8_t kCapAnsError     = 0x9F;  // [0x9F, original_sub, errcode, ...]

enum class CaptureState : uint8_t {
    Idle = 0,
    Armed = 1,
    Triggered = 2,
    Done = 3,
    Error = 4,
};

enum class CaptureTrigger : uint8_t {
    Immediate = 0,  // arm -> fill buffer -> done; pre/post ignored
    Greater = 1,
    Less = 2,
    Rising = 3,
    Falling = 4,
    Changed = 5,
};

// Capture slot type is the neutral wire value type (kept as an alias so
// existing CaptureSlotType::X call sites stay unchanged).
using CaptureSlotType = WireValueType;

enum class CaptureError : uint8_t {
    None = 0,
    BadSub = 1,        // unknown subcommand
    BadState = 2,      // cmd not allowed in current state
    BadSlot = 3,       // slot index / size / type out of range
    BadConfig = 4,     // period / pre+post / buffer_kb invalid
    BadTrigger = 5,    // trigger slot out of configured slots
    BufferTooSmall = 6,
    NotImplemented = 7,
};

inline constexpr uint8_t kCapMaxSlots = 6;
inline constexpr uint8_t kCapDataPayloadBytes = 5;

struct CaptureSlotConfig {
    uint32_t address = 0;
    uint8_t size = 4;            // 1, 2 or 4
    CaptureSlotType type = CaptureSlotType::F32;
};

struct CaptureConfig {
    uint8_t nodeId = kCanAddrMotor;
    uint8_t slotCount = 0;       // 1..kCapMaxSlots
    uint16_t periodUs = 250;
    uint8_t prePct = 20;         // 0..100, must satisfy prePct + postPct <= 100
    uint8_t postPct = 80;
    uint8_t bufferKb = 16;       // requested; MCU caps to its own ceiling
    CaptureTrigger trigger = CaptureTrigger::Immediate;
    uint8_t triggerSlot = 0;
    int32_t triggerThreshold = 0;
    std::array<CaptureSlotConfig, kCapMaxSlots> slots{};
};

struct CaptureStatus {
    uint8_t nodeId = 0;
    CaptureState state = CaptureState::Idle;
    uint8_t slotCount = 0;
    uint16_t samplesCaptured = 0;     // valid samples currently in buffer
    uint16_t triggerSampleIdx = 0;    // index of trigger sample, valid in DONE
    uint8_t errorCode = 0;
    bool bufferFull = false;
};

struct CaptureAck {
    uint8_t nodeId = 0;
    uint8_t originalSub = 0;
    uint8_t status = 0;          // 0 = OK, otherwise CaptureError code
    bool isError = false;        // true when the frame was 0x9F not 0x90
};

struct CaptureChunkRequest {
    uint8_t nodeId = kCanAddrMotor;
    uint16_t byteOffset = 0;
    uint16_t byteCount = 0;
};

struct CaptureDataFrame {
    uint8_t nodeId = 0;
    uint16_t byteOffset = 0;
    bool last = false;           // true on CAP_DATA_END (0x98)
    std::array<uint8_t, kCapDataPayloadBytes> payload{};
    uint8_t validBytes = 0;      // <= kCapDataPayloadBytes
};

struct CaptureCrcReply {
    uint8_t  nodeId = 0;
    uint32_t crc = 0;            // CRC32 of linearised valid-window buffer
    uint16_t totalBytes = 0;     // echoed back so we can sanity-check coverage
};

// CRC32 used by the capture protocol. Software byte-stream CRC (MPEG-2 form).
// The motor and RK firmware ship the identical routine in their app_capture.c.
uint32_t captureCrc32(const uint8_t* data, size_t len);

namespace capture {

CanFrame makeReset(uint8_t nodeId, double timestamp);
CanFrame makeSetSlot(uint8_t nodeId, uint8_t slotIdx, const CaptureSlotConfig& slot, double timestamp);
CanFrame makeSetConfig(uint8_t nodeId, const CaptureConfig& cfg, double timestamp);
CanFrame makeSetTrigger(uint8_t nodeId, const CaptureConfig& cfg, double timestamp);
CanFrame makeArm(uint8_t nodeId, double timestamp);
CanFrame makeStop(uint8_t nodeId, double timestamp);
CanFrame makeStatusReq(uint8_t nodeId, double timestamp);
CanFrame makeReadChunk(const CaptureChunkRequest& req, double timestamp);
CanFrame makeCrcReq(uint8_t nodeId, double timestamp);

std::optional<CaptureStatus> parseStatus(const CanFrame& frame);
std::optional<CaptureAck> parseAck(const CanFrame& frame);
std::optional<CaptureDataFrame> parseData(const CanFrame& frame);
std::optional<CaptureCrcReply> parseCrc(const CanFrame& frame);

} // namespace capture
} // namespace drivescope
