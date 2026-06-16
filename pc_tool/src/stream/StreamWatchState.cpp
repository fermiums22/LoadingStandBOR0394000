#include "stream/StreamWatchState.h"

#include <algorithm>
#include <cstdio>

namespace drivescope {

void StreamWatch::onAddSent(double now)
{
    addSentAt_ = now;
    if (state_ == StreamWatchState::Idle || state_ == StreamWatchState::Stale ||
        state_ == StreamWatchState::Resyncing) {
        state_ = StreamWatchState::AddPending;
    }
}

void StreamWatch::onAddRsp(double now, bool accepted)
{
    // The motor ANSWERED, so the path is alive in both directions — clear the
    // no-response counter regardless of accept/reject.
    unansweredAdds_ = 0;
    lastAddRejected_ = !accepted;
    if (accepted) {
        state_ = StreamWatchState::Subscribed;
        lastValueAt_ = now;   // grace period before stale
    } else {
        // TABLE_FULL / BAD_ADDR / BAD_SIZE etc.: motor is reachable but refused.
        // Back off and retry rather than hammer; not a dead-path failure.
        state_ = StreamWatchState::Resyncing;
        lastResyncAt_ = now;
    }
}

void StreamWatch::onValue(double now)
{
    lastValueAt_ = now;
    unansweredAdds_ = 0;
    state_ = StreamWatchState::Subscribed;
}

void StreamWatch::clear()
{
    state_ = StreamWatchState::Idle;
    addSentAt_ = lastValueAt_ = lastResyncAt_ = -1.0;
    unansweredAdds_ = 0;
    lastAddRejected_ = false;
}

StreamWatchAction StreamWatch::tick(double now)
{
    switch (state_) {
        case StreamWatchState::Idle:
            // Arm the subscription.
            state_ = StreamWatchState::AddPending;
            addSentAt_ = now;
            return StreamWatchAction::SendAdd;

        case StreamWatchState::AddPending:
            if (addSentAt_ < 0.0 || (now - addSentAt_) < cfg_.addTimeoutSec) {
                return StreamWatchAction::None;   // still waiting for RSP/VALUE
            }
            // ADD went unanswered.
            ++unansweredAdds_;
            if (unansweredAdds_ >= cfg_.maxUnansweredAdds) {
                state_ = StreamWatchState::FailedNoMotorResponse;
                lastResyncAt_ = now;
                return StreamWatchAction::None;
            }
            state_ = StreamWatchState::Resyncing;
            lastResyncAt_ = now;
            return StreamWatchAction::None;

        case StreamWatchState::Subscribed:
            if (lastValueAt_ >= 0.0 && (now - lastValueAt_) >= cfg_.valueStaleSec) {
                state_ = StreamWatchState::Stale;
            }
            return StreamWatchAction::None;

        case StreamWatchState::Stale:
            // First recovery step: ask the motor to STATUS/CLEAR the slot, then
            // re-ADD — a stale subscription on the motor must be torn down so the
            // re-ADD isn't rejected as a duplicate.
            state_ = StreamWatchState::AddPending;
            addSentAt_ = now;
            lastResyncAt_ = now;
            return StreamWatchAction::SendStatusThenAdd;

        case StreamWatchState::Resyncing: {
            // Backoff grows with the number of unanswered attempts, capped.
            const double backoff = std::min(cfg_.resyncBackoffSec * unansweredAdds_,
                                            cfg_.resyncBackoffMaxSec);
            if (lastResyncAt_ < 0.0 || (now - lastResyncAt_) < backoff) {
                return StreamWatchAction::None;
            }
            state_ = StreamWatchState::AddPending;
            addSentAt_ = now;
            lastResyncAt_ = now;
            return StreamWatchAction::SendAdd;
        }

        case StreamWatchState::FailedNoMotorResponse:
            // Stop spamming. Slow-probe at the backoff cap; stay Failed until an
            // ADD_RSP or VALUE actually arrives (handled in onAddRsp/onValue).
            if (lastResyncAt_ < 0.0 ||
                (now - lastResyncAt_) >= cfg_.resyncBackoffMaxSec) {
                addSentAt_ = now;
                lastResyncAt_ = now;
                return StreamWatchAction::SendAdd;
            }
            return StreamWatchAction::None;
    }
    return StreamWatchAction::None;
}

const char* StreamWatch::stateName() const
{
    switch (state_) {
        case StreamWatchState::Idle:                  return "idle";
        case StreamWatchState::AddPending:            return "add_pending";
        case StreamWatchState::Subscribed:            return "subscribed";
        case StreamWatchState::Stale:                 return "stale";
        case StreamWatchState::Resyncing:             return "resyncing";
        case StreamWatchState::FailedNoMotorResponse: return "failed_no_motor_response";
    }
    return "?";
}

std::string StreamWatch::statusText(double now) const
{
    char buf[96];
    switch (state_) {
        case StreamWatchState::Idle:
            return "idle";
        case StreamWatchState::AddPending:
            return "subscribing…";
        case StreamWatchState::Subscribed:
            return "subscribed";
        case StreamWatchState::Stale:
            std::snprintf(buf, sizeof(buf), "stale (no data %.1fs)",
                          lastValueAt_ >= 0.0 ? now - lastValueAt_ : 0.0);
            return buf;
        case StreamWatchState::Resyncing:
            std::snprintf(buf, sizeof(buf), "resyncing (%d unanswered)", unansweredAdds_);
            return buf;
        case StreamWatchState::FailedNoMotorResponse:
            return "NO MOTOR RESPONSE — command path dead";
    }
    return "?";
}

} // namespace drivescope
