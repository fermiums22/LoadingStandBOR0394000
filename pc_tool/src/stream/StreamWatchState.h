#pragma once

#include <string>

namespace drivescope {

// -----------------------------------------------------------------------------
// StreamWatch — recovery state machine for ONE plotted variable's STREAM
// subscription. Pure logic: no transport, no UI, no protocol bytes. The owner
// (MainUi) feeds it events (we sent an ADD / got an ADD_RSP / got a VALUE) and
// calls tick() periodically; tick() returns the ACTION the owner should take.
//
// Why this exists: a live plot freezes when the pc_tool->app_dd->CAN->motor
// command path breaks one-directionally (motor keeps sending telemetry but
// stops receiving). The old code just re-sent STREAM_ADD every ~2s forever and
// silently degraded. This FSM makes the lifecycle explicit, backs off instead
// of spamming, asks the motor to STATUS/CLEAR before re-ADD on a stale slot,
// and escalates to a visible "no motor response" state the UI can surface.
//
// States:
//   Idle                  not subscribed; tick will request a subscription
//   AddPending            STREAM_ADD_REQ sent, waiting for ADD_RSP or first VALUE
//   Subscribed            receiving STREAM_VALUE
//   Stale                 was Subscribed, no VALUE for valueStaleSec
//   Resyncing             backing off between re-subscribe attempts
//   FailedNoMotorResponse N consecutive ADDs unanswered -> path is dead; stop
//                         spamming, surface it, slow-probe for recovery
// -----------------------------------------------------------------------------
enum class StreamWatchState {
    Idle,
    AddPending,
    Subscribed,
    Stale,
    Resyncing,
    FailedNoMotorResponse,
};

struct StreamWatchConfig {
    double valueStaleSec       = 1.0;  // Subscribed + no VALUE this long -> Stale
    double addTimeoutSec       = 0.5;  // ADD_REQ with no RSP/VALUE this long -> retry
    double resyncBackoffSec    = 2.0;  // base spacing between resync ADDs
    double resyncBackoffMaxSec = 8.0;  // backoff cap (also the failed-probe period)
    int    maxUnansweredAdds   = 4;    // consecutive unanswered ADDs -> Failed
};

// What the owner should do as a result of tick(). The owner maps these to the
// actual StreamCodec frames; the FSM never builds bytes itself.
enum class StreamWatchAction {
    None,                  // nothing to do this tick
    SendAdd,               // (re)send STREAM_ADD_REQ
    SendStatusThenAdd,     // first STREAM_STATUS_REQ / CLEAR the stale slot, then ADD
};

class StreamWatch {
public:
    explicit StreamWatch(StreamWatchConfig cfg = {}) : cfg_(cfg) {}

    // --- events fed by the owner -------------------------------------------
    void onAddSent(double now);            // we transmitted STREAM_ADD_REQ
    void onAddRsp(double now, bool accepted); // STREAM_ADD_RSP for this slot
    void onValue(double now);              // STREAM_VALUE for this slot
    void clear();                          // removed/unsubscribed -> Idle

    // --- periodic driver ----------------------------------------------------
    StreamWatchAction tick(double now);

    // --- introspection (for UI / valueText) --------------------------------
    StreamWatchState state() const { return state_; }
    const char* stateName() const;
    int unansweredAdds() const { return unansweredAdds_; }
    std::string statusText(double now) const;

private:
    StreamWatchConfig cfg_;
    StreamWatchState  state_ = StreamWatchState::Idle;
    double addSentAt_   = -1.0;
    double lastValueAt_ = -1.0;
    double lastResyncAt_ = -1.0;
    int    unansweredAdds_ = 0;
    bool   lastAddRejected_ = false;  // motor answered ADD with an error code
};

} // namespace drivescope
