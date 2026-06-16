#pragma once

#include "can/CanFrame.h"
#include "protocol/PropCanFrame.h"
#include "protocol/ReadMemCodec.h"
#include "protocol/CaptureCodec.h"
#include "protocol/StreamCodec.h"
#include "protocol/WireValue.h"   // re-export value helpers (decodeValue, etc.)

#include <cstdint>
#include <optional>

namespace drivescope {

// -----------------------------------------------------------------------------
// DebugProtocol is a thin compatibility FACADE over the per-family wire codecs
// (ReadMemCodec / CaptureCodec / StreamCodec). It preserves the historical
// method surface so existing callers (MainUi, HeadlessCapture) are unaffected;
// each method simply delegates to the matching codec free function.
//
// All wire layout, command-id constants, enums and DTOs live in the codec
// headers above and are re-exported through these includes — so anyone who
// already includes DebugProtocol.h keeps seeing them unchanged.
// -----------------------------------------------------------------------------
class DebugProtocol {
public:
    CanFrame makeReadMem(const DebugReadRequest& req, double timestamp) const;
    std::optional<DebugReadResponse> parseReadMemResponse(const CanFrame& frame) const;

    // -- Capture protocol builders -----------------------------------------
    CanFrame makeCapReset(uint8_t nodeId, double timestamp) const;
    CanFrame makeCapSetSlot(uint8_t nodeId, uint8_t slotIdx,
                            const CaptureSlotConfig& slot, double timestamp) const;
    CanFrame makeCapSetConfig(uint8_t nodeId, const CaptureConfig& cfg, double timestamp) const;
    CanFrame makeCapSetTrigger(uint8_t nodeId, const CaptureConfig& cfg, double timestamp) const;
    CanFrame makeCapArm(uint8_t nodeId, double timestamp) const;
    CanFrame makeCapStop(uint8_t nodeId, double timestamp) const;
    CanFrame makeCapStatusReq(uint8_t nodeId, double timestamp) const;
    CanFrame makeCapReadChunk(const CaptureChunkRequest& req, double timestamp) const;
    CanFrame makeCapCrcReq(uint8_t nodeId, double timestamp) const;

    // -- Capture protocol parsers ------------------------------------------
    std::optional<CaptureStatus> parseCapStatus(const CanFrame& frame) const;
    std::optional<CaptureAck> parseCapAck(const CanFrame& frame) const;
    std::optional<CaptureDataFrame> parseCapData(const CanFrame& frame) const;
    std::optional<CaptureCrcReply> parseCapCrc(const CanFrame& frame) const;

    // -- STREAM_* protocol (Phase C) ---------------------------------------
    CanFrame makeStreamAdd(const StreamSubscribeRequest& req, double timestamp) const;
    CanFrame makeStreamRemove(uint8_t nodeId, uint8_t seq, uint8_t slotId, double timestamp) const;
    CanFrame makeStreamRemoveByAddr(uint8_t nodeId, uint8_t seq, uint32_t address, double timestamp) const;
    CanFrame makeStreamClear(uint8_t nodeId, double timestamp) const;
    CanFrame makeStreamStatusReq(uint8_t nodeId, double timestamp) const;
    CanFrame makeStreamHeartbeat(uint8_t nodeId, double timestamp) const;

    std::optional<StreamSubscribeResponse> parseStreamAddRsp(const CanFrame& frame) const;
    std::optional<StreamStatusReply> parseStreamStatusRsp(const CanFrame& frame) const;
    std::optional<StreamValueFrame> parseStreamValue(const CanFrame& frame) const;
};

// Generic value helpers (decodeValue / formatValue / captureSlotType*) now live
// in protocol/WireValue.h, re-exported via the include above so existing
// callers are unaffected.

} // namespace drivescope
