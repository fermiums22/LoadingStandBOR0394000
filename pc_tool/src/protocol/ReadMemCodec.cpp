#include "protocol/ReadMemCodec.h"

#include "protocol/PropCanFrame.h"
#include "protocol/WireBytes.h"

#include <algorithm>

namespace drivescope {
namespace readmem {

CanFrame makeRequest(const DebugReadRequest& req, double timestamp)
{
    CanFrame frame;
    frame.timestamp = timestamp;
    frame.rx = false;
    frame.id = packPropCanId(PropCanFrameId{kPropCanModeApp, kCanCmdReadMemReq, req.nodeId, kCanAddrPc});
    frame.extended = true;
    frame.fd = false;
    frame.brs = false;
    frame.dlc = 8;
    frame.data.resize(8, 0);
    auto& d = frame.data;
    d[0] = 0x03;          // READ_MEM request guard
    d[1] = req.seq;
    putU32Le(d, 2, req.address);
    d[6] = req.size;
    d[7] = 0;
    return frame;
}

std::optional<DebugReadResponse> parseResponse(const CanFrame& frame)
{
    if (!frame.extended || frame.data.size() < 8) {
        return std::nullopt;
    }
    const uint8_t mod = propCanMod(frame.id);
    const uint16_t cmd = propCanCmd(frame.id);
    const uint8_t dst = propCanDst(frame.id);
    const uint8_t src = propCanSrc(frame.id);
    if (mod != 1u || cmd != kCanCmdReadMemRsp || dst != kCanAddrPc) {
        return std::nullopt;
    }
    if (frame.data[0] != 0x83) return std::nullopt;

    DebugReadResponse rsp;
    rsp.nodeId = src;
    rsp.seq = frame.data[1];
    rsp.status = frame.data[2];
    rsp.size = frame.data[3];
    const size_t copy = std::min<size_t>(rsp.size, 4);
    rsp.value.assign(frame.data.begin() + 4, frame.data.begin() + 4 + copy);
    return rsp;
}

} // namespace readmem
} // namespace drivescope
