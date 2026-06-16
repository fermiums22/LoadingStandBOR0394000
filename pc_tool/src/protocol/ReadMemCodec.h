#pragma once

#include "can/CanFrame.h"
#include "protocol/PropCanFrame.h"

#include <cstdint>
#include <optional>
#include <vector>

namespace drivescope {

// -----------------------------------------------------------------------------
// READ_MEM wire codec. Pure pack/decode of the legacy single-word debug read
// (cmd 0x0D0 request / 0x0D1 response, guard bytes 0x03 / 0x83). No UI, no
// session/retry, no transport, no logging, no mutable state. Uses PropCanFrame
// for the id envelope. Canonical spec: umbrella protocol/ddv2_protocol.yaml.
// -----------------------------------------------------------------------------

inline constexpr uint16_t kCanCmdReadMemReq = 0x0D0;
inline constexpr uint16_t kCanCmdReadMemRsp = 0x0D1;

struct DebugReadRequest {
    uint8_t nodeId = 2;
    uint8_t seq = 0;
    uint32_t address = 0;
    uint8_t size = 4;
};

struct DebugReadResponse {
    uint8_t nodeId = 0;
    uint8_t seq = 0;
    uint8_t status = 0;
    uint8_t size = 0;
    std::vector<uint8_t> value;
};

namespace readmem {

CanFrame makeRequest(const DebugReadRequest& req, double timestamp);
std::optional<DebugReadResponse> parseResponse(const CanFrame& frame);

} // namespace readmem
} // namespace drivescope
