#include "protocol/CaptureCodec.h"

#include "protocol/Crc32.h"
#include "protocol/PropCanFrame.h"
#include "protocol/WireBytes.h"

namespace drivescope {

uint32_t captureCrc32(const uint8_t* data, size_t len)
{
    // Capture CRC = MPEG-2 byte-wise (protocol/Crc32). Thin forwarder so call
    // sites stay byte-for-byte identical.
    return captureCrc32Mpeg2Bytes(data, len);
}

namespace capture {

namespace {
CanFrame makeCapFrame(uint8_t nodeId, double timestamp)
{
    CanFrame frame;
    frame.timestamp = timestamp;
    frame.rx = false;
    frame.id = packPropCanId(PropCanFrameId{kPropCanModeApp, kCanCmdCapReq, nodeId, kCanAddrPc});
    frame.extended = true;
    frame.fd = false;
    frame.brs = false;
    frame.dlc = 8;
    frame.data.assign(8, 0);
    return frame;
}

bool isCapRsp(const CanFrame& frame)
{
    if (!frame.extended || frame.data.size() < 8) return false;
    const uint8_t mod = propCanMod(frame.id);
    const uint16_t cmd = propCanCmd(frame.id);
    const uint8_t dst = propCanDst(frame.id);
    return mod == 1u && cmd == kCanCmdCapRsp && dst == kCanAddrPc;
}

uint8_t srcOf(const CanFrame& frame)
{
    return propCanSrc(frame.id);
}
} // namespace

CanFrame makeReset(uint8_t nodeId, double timestamp)
{
    CanFrame f = makeCapFrame(nodeId, timestamp);
    f.data[0] = kCapSubReset;
    return f;
}

CanFrame makeSetSlot(uint8_t nodeId, uint8_t slotIdx,
                     const CaptureSlotConfig& slot, double timestamp)
{
    CanFrame f = makeCapFrame(nodeId, timestamp);
    auto& d = f.data;
    d[0] = kCapSubSetSlot;                  // [0]   sub
    d[1] = slotIdx;                         // [1]   slot index
    putU32Le(d, 2, slot.address);           // [2..5] address
    d[6] = slot.size;                       // [6]   size (1|2|4)
    d[7] = static_cast<uint8_t>(slot.type); // [7]   type
    return f;
}

CanFrame makeSetConfig(uint8_t nodeId, const CaptureConfig& cfg, double timestamp)
{
    CanFrame f = makeCapFrame(nodeId, timestamp);
    auto& d = f.data;
    d[0] = kCapSubSetConfig;     // [0]   sub
    d[1] = cfg.slotCount;        // [1]   slot count
    putU16Le(d, 2, cfg.periodUs); // [2..3] period us
    d[4] = cfg.prePct;           // [4]   pre %
    d[5] = cfg.postPct;          // [5]   post %
    d[6] = cfg.bufferKb;         // [6]   buffer KiB
    d[7] = 0;                    // [7]   reserved
    return f;
}

CanFrame makeSetTrigger(uint8_t nodeId, const CaptureConfig& cfg, double timestamp)
{
    CanFrame f = makeCapFrame(nodeId, timestamp);
    auto& d = f.data;
    d[0] = kCapSubSetTrigger;                       // [0]   sub
    d[1] = static_cast<uint8_t>(cfg.trigger);       // [1]   trigger mode
    d[2] = cfg.triggerSlot;                         // [2]   trigger slot
    putU32Le(d, 3, static_cast<uint32_t>(cfg.triggerThreshold)); // [3..6] threshold i32
    d[7] = 0;                                       // [7]   reserved
    return f;
}

CanFrame makeArm(uint8_t nodeId, double timestamp)
{
    CanFrame f = makeCapFrame(nodeId, timestamp);
    f.data[0] = kCapSubArm;
    return f;
}

CanFrame makeStop(uint8_t nodeId, double timestamp)
{
    CanFrame f = makeCapFrame(nodeId, timestamp);
    f.data[0] = kCapSubStop;
    return f;
}

CanFrame makeStatusReq(uint8_t nodeId, double timestamp)
{
    CanFrame f = makeCapFrame(nodeId, timestamp);
    f.data[0] = kCapSubStatusReq;
    return f;
}

CanFrame makeReadChunk(const CaptureChunkRequest& req, double timestamp)
{
    CanFrame f = makeCapFrame(req.nodeId, timestamp);
    auto& d = f.data;
    d[0] = kCapSubReadChunk;       // [0]   sub
    putU16Le(d, 1, req.byteOffset); // [1..2] byte offset
    putU16Le(d, 3, req.byteCount);  // [3..4] byte count (0 = all)
    return f;
}

CanFrame makeCrcReq(uint8_t nodeId, double timestamp)
{
    CanFrame f = makeCapFrame(nodeId, timestamp);
    f.data[0] = kCapSubCrc;
    return f;
}

std::optional<CaptureStatus> parseStatus(const CanFrame& frame)
{
    if (!isCapRsp(frame)) return std::nullopt;
    if (frame.data[0] != kCapAnsStatus) return std::nullopt;
    const auto& d = frame.data;
    CaptureStatus s;
    s.nodeId = srcOf(frame);
    s.state = static_cast<CaptureState>(d[1]);
    s.slotCount = d[2];
    s.samplesCaptured = getU16Le(d, 3);
    s.triggerSampleIdx = getU16Le(d, 5);
    s.errorCode = static_cast<uint8_t>(d[7] & 0x7Fu);
    s.bufferFull = (d[7] & 0x80u) != 0u;
    return s;
}

std::optional<CaptureAck> parseAck(const CanFrame& frame)
{
    if (!isCapRsp(frame)) return std::nullopt;
    if (frame.data[0] != kCapAnsAck && frame.data[0] != kCapAnsError) return std::nullopt;
    CaptureAck a;
    a.nodeId = srcOf(frame);
    a.isError = (frame.data[0] == kCapAnsError);
    a.originalSub = frame.data[1];
    a.status = frame.data[2];
    return a;
}

std::optional<CaptureCrcReply> parseCrc(const CanFrame& frame)
{
    if (!isCapRsp(frame)) return std::nullopt;
    if (frame.data[0] != kCapAnsCrc) return std::nullopt;
    const auto& d = frame.data;
    CaptureCrcReply r;
    r.nodeId = srcOf(frame);
    r.crc = getU32Le(d, 1);        // [1..4] crc32
    r.totalBytes = getU16Le(d, 5); // [5..6] total bytes
    return r;
}

std::optional<CaptureDataFrame> parseData(const CanFrame& frame)
{
    if (!isCapRsp(frame)) return std::nullopt;
    if (frame.data[0] != kCapAnsData && frame.data[0] != kCapAnsDataEnd) return std::nullopt;
    const auto& d = frame.data;
    CaptureDataFrame out;
    out.nodeId = srcOf(frame);
    out.last = (d[0] == kCapAnsDataEnd);
    out.byteOffset = getU16Le(d, 1);  // [1..2] byte offset
    // payload bytes 3..7 (5 bytes). For the END frame, the validBytes count is
    // packed into byte 7; if absent, assume all 5 valid.
    for (size_t i = 0; i < kCapDataPayloadBytes; ++i) {
        out.payload[i] = d[3 + i];
    }
    if (out.last) {
        const uint8_t v = d[7];
        out.validBytes = (v <= kCapDataPayloadBytes) ? v : kCapDataPayloadBytes;
    } else {
        out.validBytes = kCapDataPayloadBytes;
    }
    return out;
}

} // namespace capture
} // namespace drivescope
