#pragma once

#include "can/CanFrame.h"
#include "protocol/PropCanFrame.h"
#include "protocol/WireValue.h"   // WireValueType (neutral; no dependency on CaptureCodec)

#include <array>
#include <cstdint>
#include <optional>

namespace drivescope {

// -----------------------------------------------------------------------------
// STREAM_* subscription wire codec (Phase B/C). Pure pack/decode. PC subscribes
// to motor variables; motor pushes STREAM_VALUE (broadcast, dst=0xFF) frames.
// No UI, no session/retry/TTL bookkeeping (that lives in MainUi/services), no
// transport, no logging, no mutable state.
// Canonical spec: umbrella protocol/ddv2_protocol.yaml (stream).
// -----------------------------------------------------------------------------

inline constexpr uint16_t kCanCmdStreamAddReq    = 0x0E2;
inline constexpr uint16_t kCanCmdStreamAddRsp    = 0x0E3;
inline constexpr uint16_t kCanCmdStreamRemove    = 0x0E4;
inline constexpr uint16_t kCanCmdStreamClear     = 0x0E5;
inline constexpr uint16_t kCanCmdStreamStatusReq = 0x0E6;
inline constexpr uint16_t kCanCmdStreamStatusRsp = 0x0E7;
inline constexpr uint16_t kCanCmdStreamValue     = 0x0E8;
inline constexpr uint16_t kCanCmdStreamHeartbeat = 0x0E9;

inline constexpr uint8_t kStreamOpAdd       = 0x01;
inline constexpr uint8_t kStreamOpRemove    = 0x02;
inline constexpr uint8_t kStreamOpClear     = 0x03;
inline constexpr uint8_t kStreamOpStatusReq = 0x04;
inline constexpr uint8_t kStreamOpHeartbeat = 0x05;

inline constexpr uint8_t kStreamAddStatusOk         = 0;
inline constexpr uint8_t kStreamAddStatusTableFull  = 1;
inline constexpr uint8_t kStreamAddStatusBadAddr    = 2;
inline constexpr uint8_t kStreamAddStatusBadType    = 3;
inline constexpr uint8_t kStreamAddStatusDuplicate  = 4;

inline constexpr uint8_t kStreamInvalidSlot         = 0xFF;
inline constexpr uint8_t kStreamSubTableSize        = 12;

// size_type byte encoding on the wire:
//   bits 0-1: size code  (00 = 1B, 01 = 2B, 10 = 4B)
//   bits 4-7: type code  (0 = u, 1 = i, 2 = f)
uint8_t streamSizeTypeFromCaptureType(WireValueType type);
uint8_t streamSizeBytes(uint8_t sizeType);

struct StreamSubscribeRequest {
    uint8_t  nodeId = kCanAddrMotor;
    uint8_t  seq = 0;
    uint32_t address = 0;
    uint8_t  sizeType = 0;   // packed bits as above
    uint8_t  periodHint = 0; // 0 = motor decides
};

struct StreamSubscribeResponse {
    uint8_t  nodeId = 0;
    uint8_t  status = 0;     // see kStreamAddStatus*
    uint8_t  seq = 0;
    uint8_t  slotId = kStreamInvalidSlot;
    uint8_t  used = 0;
    uint8_t  total = 0;
    uint8_t  actualPeriodTicks = 0;
    uint16_t normalDroppedAtMoment = 0;
};

struct StreamStatusReply {
    uint8_t  nodeId = 0;
    uint8_t  used = 0;
    uint8_t  total = 0;
    bool     captureActive = false;
    uint8_t  periodTicksNow = 0;
    uint16_t normalDropped = 0;
    uint16_t captureDropped = 0;
};

struct StreamValueFrame {
    uint8_t  nodeId = 0;
    uint8_t  slotId = 0;
    uint8_t  sizeType = 0;
    std::array<uint8_t, 4> valueBytes{};
    uint8_t  tickLsb = 0;
    uint8_t  flags = 0;
};

namespace stream {

CanFrame makeAdd(const StreamSubscribeRequest& req, double timestamp);
CanFrame makeRemove(uint8_t nodeId, uint8_t seq, uint8_t slotId, double timestamp);
CanFrame makeRemoveByAddr(uint8_t nodeId, uint8_t seq, uint32_t address, double timestamp);
CanFrame makeClear(uint8_t nodeId, double timestamp);
CanFrame makeStatusReq(uint8_t nodeId, double timestamp);
CanFrame makeHeartbeat(uint8_t nodeId, double timestamp);

std::optional<StreamSubscribeResponse> parseAddRsp(const CanFrame& frame);
std::optional<StreamStatusReply> parseStatusRsp(const CanFrame& frame);
std::optional<StreamValueFrame> parseValue(const CanFrame& frame);

} // namespace stream
} // namespace drivescope
