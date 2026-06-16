#pragma once

#include <cstdint>

namespace drivescope {

// -----------------------------------------------------------------------------
// PropCAN frame envelope.
//
// This is the shared envelope every PropCAN frame carries: the 29-bit extended
// -id layout plus the node-address and image-mode vocabulary. It is a DOMAIN
// boundary (the frame envelope), not a one-function helper.
//
//   id = (mod << 28) | ((cmd & 0xFFF) << 16) | (dst << 8) | src
//
// Envelope ONLY — deliberately NOT here: per-protocol command ids and payload
// layouts (those belong to each protocol's codec), CRC, retry/session state,
// UI labels, sniffer presentation, YAML parsing.
//
// Canonical spec: umbrella protocol/ddv2_protocol.yaml (propcan_id / addresses /
// modes).
// -----------------------------------------------------------------------------

// Image mode — id bit 28.
inline constexpr uint8_t kPropCanModeBoot = 0;
inline constexpr uint8_t kPropCanModeApp  = 1;

// Node addresses (dst / src). ESP32 is a logical CAN node reached via the MAIN
// proxy; kept here because decoders still reference it.
inline constexpr uint8_t kCanAddrMain      = 0x01;
inline constexpr uint8_t kCanAddrMotor     = 0x02;
inline constexpr uint8_t kCanAddrRk        = 0x03;
inline constexpr uint8_t kCanAddrEsp32     = 0x04;
inline constexpr uint8_t kCanAddrPc        = 0x10;
inline constexpr uint8_t kCanAddrBroadcast = 0xFF;

struct PropCanFrameId {
    uint8_t  mod;
    uint16_t cmd;
    uint8_t  dst;
    uint8_t  src;
};

uint32_t       packPropCanId(const PropCanFrameId& id);
PropCanFrameId unpackPropCanId(uint32_t rawId);

uint8_t  propCanMod(uint32_t rawId);
uint16_t propCanCmd(uint32_t rawId);
uint8_t  propCanDst(uint32_t rawId);
uint8_t  propCanSrc(uint32_t rawId);

} // namespace drivescope
