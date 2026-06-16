#include "protocol/StreamCodec.h"

#include "protocol/PropCanFrame.h"
#include "protocol/WireBytes.h"

namespace drivescope {

uint8_t streamSizeTypeFromCaptureType(WireValueType type)
{
    // size code: 00=1B, 01=2B, 10=4B; type code: 0=u, 1=i, 2=f.
    switch (type) {
        case WireValueType::U8:  return 0x00; // size=0, type=0
        case WireValueType::I8:  return 0x10; // size=0, type=1
        case WireValueType::U16: return 0x01; // size=1, type=0
        case WireValueType::I16: return 0x11; // size=1, type=1
        case WireValueType::U32: return 0x02; // size=2, type=0
        case WireValueType::I32: return 0x12; // size=2, type=1
        case WireValueType::F32: return 0x22; // size=2, type=2
    }
    return 0x02; // sane default = u32
}

uint8_t streamSizeBytes(uint8_t sizeType)
{
    const uint8_t code = sizeType & 0x03u;
    if (code == 0u) return 1u;
    if (code == 1u) return 2u;
    if (code == 2u) return 4u;
    return 0u;
}

namespace stream {

namespace {
CanFrame makeStreamFrame(uint16_t cmd, uint8_t nodeId, double timestamp)
{
    CanFrame frame;
    frame.timestamp = timestamp;
    frame.rx = false;
    frame.id = packPropCanId(PropCanFrameId{kPropCanModeApp, cmd, nodeId, kCanAddrPc});
    frame.extended = true;
    frame.fd = false;
    frame.brs = false;
    frame.dlc = 8;
    frame.data.assign(8, 0);
    return frame;
}

uint8_t srcOf(const CanFrame& frame)
{
    return propCanSrc(frame.id);
}
} // namespace

CanFrame makeAdd(const StreamSubscribeRequest& req, double timestamp)
{
    CanFrame f = makeStreamFrame(kCanCmdStreamAddReq, req.nodeId, timestamp);
    auto& d = f.data;
    d[0] = kStreamOpAdd;        // [0]   op
    d[1] = req.seq;             // [1]   seq
    putU32Le(d, 2, req.address); // [2..5] address
    d[6] = req.sizeType;        // [6]   size_type
    d[7] = req.periodHint;      // [7]   period hint
    return f;
}

CanFrame makeRemove(uint8_t nodeId, uint8_t seq, uint8_t slotId, double timestamp)
{
    CanFrame f = makeStreamFrame(kCanCmdStreamRemove, nodeId, timestamp);
    auto& d = f.data;
    d[0] = kStreamOpRemove; // [0] op
    d[1] = seq;             // [1] seq
    d[2] = slotId;          // [2] slot id
    d[7] = 0xFF;
    return f;
}

CanFrame makeRemoveByAddr(uint8_t nodeId, uint8_t seq, uint32_t address, double timestamp)
{
    CanFrame f = makeStreamFrame(kCanCmdStreamRemove, nodeId, timestamp);
    auto& d = f.data;
    d[0] = kStreamOpRemove;        // [0]   op
    d[1] = seq;                    // [1]   seq
    d[2] = kStreamInvalidSlot;     // [2]   slot id = remove-by-addr marker
    putU32Le(d, 3, address);       // [3..6] address
    d[7] = 0xFF;
    return f;
}

CanFrame makeClear(uint8_t nodeId, double timestamp)
{
    CanFrame f = makeStreamFrame(kCanCmdStreamClear, nodeId, timestamp);
    f.data[0] = kStreamOpClear;
    for (size_t i = 1; i < 8; ++i) f.data[i] = 0xFF;
    return f;
}

CanFrame makeStatusReq(uint8_t nodeId, double timestamp)
{
    CanFrame f = makeStreamFrame(kCanCmdStreamStatusReq, nodeId, timestamp);
    f.data[0] = kStreamOpStatusReq;
    for (size_t i = 1; i < 8; ++i) f.data[i] = 0xFF;
    return f;
}

CanFrame makeHeartbeat(uint8_t nodeId, double timestamp)
{
    CanFrame f = makeStreamFrame(kCanCmdStreamHeartbeat, nodeId, timestamp);
    f.data[0] = kStreamOpHeartbeat;
    for (size_t i = 1; i < 8; ++i) f.data[i] = 0xFF;
    return f;
}

std::optional<StreamSubscribeResponse> parseAddRsp(const CanFrame& frame)
{
    if (!frame.extended || frame.data.size() < 8) return std::nullopt;
    const uint8_t mod = propCanMod(frame.id);
    const uint16_t cmd = propCanCmd(frame.id);
    const uint8_t dst = propCanDst(frame.id);
    if (mod != 1u || cmd != kCanCmdStreamAddRsp || dst != kCanAddrPc) {
        return std::nullopt;
    }
    const auto& d = frame.data;
    StreamSubscribeResponse r;
    r.nodeId = srcOf(frame);
    r.status = d[0];                       // [0]   status
    r.seq = d[1];                          // [1]   seq
    r.slotId = d[2];                       // [2]   slot id
    r.used = d[3];                         // [3]   used
    r.total = d[4];                        // [4]   total
    r.actualPeriodTicks = d[5];            // [5]   period ticks
    r.normalDroppedAtMoment = getU16Le(d, 6); // [6..7] normal dropped
    return r;
}

std::optional<StreamStatusReply> parseStatusRsp(const CanFrame& frame)
{
    if (!frame.extended || frame.data.size() < 8) return std::nullopt;
    const uint8_t mod = propCanMod(frame.id);
    const uint16_t cmd = propCanCmd(frame.id);
    const uint8_t dst = propCanDst(frame.id);
    if (mod != 1u || cmd != kCanCmdStreamStatusRsp || dst != kCanAddrPc) {
        return std::nullopt;
    }
    const auto& d = frame.data;
    StreamStatusReply r;
    r.nodeId = srcOf(frame);
    r.used = d[0];                       // [0]   used
    r.total = d[1];                      // [1]   total
    r.captureActive = d[2] != 0u;        // [2]   capture active
    r.periodTicksNow = d[3];             // [3]   period ticks
    r.normalDropped = getU16Le(d, 4);    // [4..5] normal dropped
    r.captureDropped = getU16Le(d, 6);   // [6..7] capture dropped
    return r;
}

std::optional<StreamValueFrame> parseValue(const CanFrame& frame)
{
    if (!frame.extended || frame.data.size() < 8) return std::nullopt;
    const uint8_t mod = propCanMod(frame.id);
    const uint16_t cmd = propCanCmd(frame.id);
    // STREAM_VALUE is broadcast (dst=0xFF) — pc_tool listens regardless of dst.
    if (mod != 1u || cmd != kCanCmdStreamValue) {
        return std::nullopt;
    }
    const auto& d = frame.data;
    StreamValueFrame v;
    v.nodeId = srcOf(frame);
    v.slotId = d[0];                  // [0]   slot id
    v.sizeType = d[1];                // [1]   size_type
    v.valueBytes[0] = d[2];           // [2..5] raw value bytes (interpret per sizeType)
    v.valueBytes[1] = d[3];
    v.valueBytes[2] = d[4];
    v.valueBytes[3] = d[5];
    v.tickLsb = d[6];                 // [6]   tick lsb
    v.flags = d[7];                   // [7]   flags
    return v;
}

} // namespace stream
} // namespace drivescope
