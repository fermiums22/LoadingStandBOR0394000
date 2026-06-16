#pragma once

#include "can/CanFrame.h"

#include <fstream>
#include <mutex>
#include <string>

namespace drivescope {

// -----------------------------------------------------------------------------
// Passive CAN frame sniffer.
//
// Appends one JSONL line per frame that ALREADY passes through pc_tool — TX via
// MainUi::sendFrameThreadSafe, RX via MainUi::handleFrame. It is strictly
// observational:
//   * default OFF — a sink is opened only if the env var DRIVESCOPE_SNIFF_JSONL
//     points at a writable file path;
//   * never changes send/receive behavior;
//   * IO errors are swallowed (logging must not break the tool);
//   * thread-safe (TX and RX may run on different threads).
//
// It does NOT decode protocol itself: the caller passes a pre-decoded text
// string (from CanDecoder::decodeCanFrame), so wire layout stays in exactly one
// place and is never duplicated in the transport/sniffer layer.
// -----------------------------------------------------------------------------
class CanSniffer {
public:
    enum class Dir { Tx, Rx };

    // Lazily probes $DRIVESCOPE_SNIFF_JSONL on first call and opens the sink.
    // Returns whether a sink is open, so callers can skip the decode cost
    // entirely when the sniffer is off (zero overhead in the default case).
    bool isActive();

    // Append one frame event. `transport` is a short label ("tcp"/"slcan"/...).
    // `decoded` may be empty; unknown/malformed frames are logged without crash.
    void record(Dir dir, const char* transport, const CanFrame& frame,
                const std::string& decoded);

private:
    void ensureInit();          // one-time env probe + open

    std::mutex    mutex_;
    std::ofstream out_;
    bool          tried_  = false;
    bool          active_ = false;
};

} // namespace drivescope
