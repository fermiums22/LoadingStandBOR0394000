#include "protocol/DebugProtocol.h"

namespace drivescope {

// -----------------------------------------------------------------------------
// Facade: every method delegates to the matching per-family codec. No wire
// layout here — see ReadMemCodec / CaptureCodec / StreamCodec.
// -----------------------------------------------------------------------------

CanFrame DebugProtocol::makeReadMem(const DebugReadRequest& req, double timestamp) const
{
    return readmem::makeRequest(req, timestamp);
}
std::optional<DebugReadResponse> DebugProtocol::parseReadMemResponse(const CanFrame& frame) const
{
    return readmem::parseResponse(frame);
}

CanFrame DebugProtocol::makeCapReset(uint8_t nodeId, double timestamp) const
{
    return capture::makeReset(nodeId, timestamp);
}
CanFrame DebugProtocol::makeCapSetSlot(uint8_t nodeId, uint8_t slotIdx,
                                       const CaptureSlotConfig& slot, double timestamp) const
{
    return capture::makeSetSlot(nodeId, slotIdx, slot, timestamp);
}
CanFrame DebugProtocol::makeCapSetConfig(uint8_t nodeId, const CaptureConfig& cfg, double timestamp) const
{
    return capture::makeSetConfig(nodeId, cfg, timestamp);
}
CanFrame DebugProtocol::makeCapSetTrigger(uint8_t nodeId, const CaptureConfig& cfg, double timestamp) const
{
    return capture::makeSetTrigger(nodeId, cfg, timestamp);
}
CanFrame DebugProtocol::makeCapArm(uint8_t nodeId, double timestamp) const
{
    return capture::makeArm(nodeId, timestamp);
}
CanFrame DebugProtocol::makeCapStop(uint8_t nodeId, double timestamp) const
{
    return capture::makeStop(nodeId, timestamp);
}
CanFrame DebugProtocol::makeCapStatusReq(uint8_t nodeId, double timestamp) const
{
    return capture::makeStatusReq(nodeId, timestamp);
}
CanFrame DebugProtocol::makeCapReadChunk(const CaptureChunkRequest& req, double timestamp) const
{
    return capture::makeReadChunk(req, timestamp);
}
CanFrame DebugProtocol::makeCapCrcReq(uint8_t nodeId, double timestamp) const
{
    return capture::makeCrcReq(nodeId, timestamp);
}
std::optional<CaptureStatus> DebugProtocol::parseCapStatus(const CanFrame& frame) const
{
    return capture::parseStatus(frame);
}
std::optional<CaptureAck> DebugProtocol::parseCapAck(const CanFrame& frame) const
{
    return capture::parseAck(frame);
}
std::optional<CaptureDataFrame> DebugProtocol::parseCapData(const CanFrame& frame) const
{
    return capture::parseData(frame);
}
std::optional<CaptureCrcReply> DebugProtocol::parseCapCrc(const CanFrame& frame) const
{
    return capture::parseCrc(frame);
}

CanFrame DebugProtocol::makeStreamAdd(const StreamSubscribeRequest& req, double timestamp) const
{
    return stream::makeAdd(req, timestamp);
}
CanFrame DebugProtocol::makeStreamRemove(uint8_t nodeId, uint8_t seq, uint8_t slotId, double timestamp) const
{
    return stream::makeRemove(nodeId, seq, slotId, timestamp);
}
CanFrame DebugProtocol::makeStreamRemoveByAddr(uint8_t nodeId, uint8_t seq, uint32_t address, double timestamp) const
{
    return stream::makeRemoveByAddr(nodeId, seq, address, timestamp);
}
CanFrame DebugProtocol::makeStreamClear(uint8_t nodeId, double timestamp) const
{
    return stream::makeClear(nodeId, timestamp);
}
CanFrame DebugProtocol::makeStreamStatusReq(uint8_t nodeId, double timestamp) const
{
    return stream::makeStatusReq(nodeId, timestamp);
}
CanFrame DebugProtocol::makeStreamHeartbeat(uint8_t nodeId, double timestamp) const
{
    return stream::makeHeartbeat(nodeId, timestamp);
}
std::optional<StreamSubscribeResponse> DebugProtocol::parseStreamAddRsp(const CanFrame& frame) const
{
    return stream::parseAddRsp(frame);
}
std::optional<StreamStatusReply> DebugProtocol::parseStreamStatusRsp(const CanFrame& frame) const
{
    return stream::parseStatusRsp(frame);
}
std::optional<StreamValueFrame> DebugProtocol::parseStreamValue(const CanFrame& frame) const
{
    return stream::parseValue(frame);
}

} // namespace drivescope
