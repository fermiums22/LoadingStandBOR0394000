// ThetaCalibration.cpp -- Test 1 calibration state machine. See header.

#include "calibration/ThetaCalibration.h"

namespace drivescope {
namespace calib {

namespace {

// Human reason for a theta CFG reject status. Mirrors the messages the old
// MainUi frame parser built inline (kept byte-identical so the operator sees
// the same banner text).
const char* thetaRejectReason(uint8_t st)
{
    switch (static_cast<ThetaStatus>(st)) {
        case ThetaStatus::BadArg:
            return "BAD_ARG -- align current out of range or guard bytes wrong";
        case ThetaStatus::Busy:
            return "BUSY -- motor is running, aligning or already calibrating";
        case ThetaStatus::NotReady:
            return "NOT_READY -- analog init not complete; wait and retry";
        case ThetaStatus::StorageFail:
            return "STORAGE_FAIL -- flash/RTC write error";
        case ThetaStatus::Fault:
            return "FAULT -- a STOP_MASK fault is live; clear faults first";
        default:
            return "rejected";
    }
}

// Which CFG statuses are genuine command rejections (1,2,3,5,6). OK(0),
// STORAGE_EMPTY(4) and RUNNING(7) are not failures.
bool thetaStatusIsReject(uint8_t st)
{
    switch (static_cast<ThetaStatus>(st)) {
        case ThetaStatus::BadArg:
        case ThetaStatus::Busy:
        case ThetaStatus::NotReady:
        case ThetaStatus::StorageFail:
        case ThetaStatus::Fault:
            return true;
        default:
            return false;
    }
}

} // namespace

void ThetaCalibration::releaseGuard()
{
    if (guard_ && guard_->ownedBy(Test::Theta)) guard_->release();
}

bool ThetaCalibration::start(const ThetaParams& params, double now)
{
    if (!cb_.sendConfig) return false;
    // One run at a time: refuse if a different test owns the slot.
    if (guard_ && !guard_->tryAcquire(Test::Theta)) {
        failMsg_ = "Another calibration test is already running.";
        return false;
    }

    failMsg_.clear();
    const uint8_t action = params.saveAfterCal ? kActionCalSave : kActionCal;

    // sendConfig returning true only means the CAN frame was queued -- the
    // motor may still answer BUSY / NOT_READY. We mark "start pending" and wait
    // for the RUNNING phase telemetry (or a reject) to resolve it.
    if (cb_.sendConfig(action, params.alignCurrentA)) {
        run_.reset(Test::Theta);
        run_.enter(RunPhase::Starting, now);
        startedAt_ = now;
        prevPhase_ = 0;       // arm a fresh edge
        pending_   = true;
        if (cb_.setStatus)
            cb_.setStatus("CFG theta calibration requested -- awaiting motor ack");
        return true;
    }

    releaseGuard();
    run_.fail(Error::MotorRejected, now);
    pending_ = false;
    failMsg_ = "Could not send the calibration command (transport not ready).";
    return false;
}

void ThetaCalibration::abort(double now)
{
    pending_ = false;
    // Explicit terminal state + message so the pill can't hang on Running and
    // the operator sees that the run was aborted (not silently dropped).
    run_.enter(RunPhase::Aborted, now);
    run_.verdict     = Verdict::Fail;   // aborted run is a non-pass terminal
    run_.error       = Error::None;     // operator action, not a motor fault
    run_.errorReason = "aborted by operator";
    failMsg_ = "Theta calibration aborted by operator.";
    releaseGuard();
}

void ThetaCalibration::finalizeDone(bool ok, double now)
{
    pending_ = false;
    if (ok) {
        // FW reported a successful fit. (Fuller pass-criteria validation --
        // all six sectors valid + coherence above threshold -- is a later
        // slice; behaviour here matches the previous UI.)
        run_.verdict = Verdict::Pass;
        run_.enter(RunPhase::Done, now);
        failMsg_.clear();
        if (cb_.readResult) cb_.readResult();
        if (cb_.setStatus)
            cb_.setStatus("theta calibration done -- re-reading BLOB");
    } else {
        // The global fit did not converge (rotor likely moved during the
        // SAMPLE phase).
        run_.fail(Error::Fault, now);
        failMsg_ =
            "Theta calibration FAILED: the firmware could not fit a coherent "
            "offset (rotor likely moved during the measurement). Make sure "
            "the shaft is free and unloaded, then retry.";
        if (cb_.setStatus) cb_.setStatus("theta calibration failed");
    }
    releaseGuard();
}

void ThetaCalibration::tick(double now)
{
    if (!pending_) return;
    if (now - startedAt_ <= kStartTimeoutSec) return;

    // Belt-and-braces timeout: the explicit reject (onCfgReject) usually fires
    // first, but if the motor stays silent this resolves the stuck pending.
    pending_ = false;
    if (failMsg_.empty()) {
        failMsg_ =
            "Calibration did not start: the motor never reported the RUNNING "
            "phase. It is probably BUSY or NOT_READY -- check diag mode, FSM "
            "idle and the fault mask above.";
    }
    run_.fail(Error::Timeout, now);
    releaseGuard();
}

void ThetaCalibration::onCalTelemetry(uint8_t phase, double now)
{
    // A RUNNING phase is the motor's real confirmation that the command was
    // accepted -- this is what clears the pending flag. But once a run has
    // reached a terminal state (Aborted / Done / Failed) a stray RUNNING frame
    // from the motor's still-draining telemetry must NOT revive it: an aborted
    // run stays aborted until the operator starts a new one.
    if (phase == static_cast<uint8_t>(ThetaCalPhase::Running) &&
        !run_.isTerminal()) {
        pending_ = false;
        if (run_.phase != RunPhase::Running) run_.enter(RunPhase::Running, now);
    }

    // Finalize on a DONE phase. Mid-sweep BLOB GETs return stale flash values;
    // only after the firmware finalises the fit are the calibrated values live.
    //
    // Main path: the RUNNING -> DONE edge. Fallback: a DONE that arrives while
    // OUR run is still Starting/Running but the RUNNING phase was never observed
    // (a very short sweep or a dropped telemetry frame) -- finalize anyway so
    // the run cannot get stuck. A run that already reached a terminal state
    // (e.g. a late DONE after Abort) is never re-finalized.
    const bool doneOk       = phase == static_cast<uint8_t>(ThetaCalPhase::DoneOk);
    const bool doneFail     = phase == static_cast<uint8_t>(ThetaCalPhase::DoneFail);
    const bool sawRunEdge   = prevPhase_ == static_cast<uint8_t>(ThetaCalPhase::Running);
    if ((doneOk || doneFail) && !run_.isTerminal() &&
        (sawRunEdge || run_.isBusy())) {
        finalizeDone(doneOk, now);
    }

    prevPhase_ = phase;
}

void ThetaCalibration::onCfgReject(uint8_t thetaStatus, double now)
{
    // Only our in-flight command matters; ignore stray GET-response statuses.
    if (!run_.isBusy()) return;
    if (!thetaStatusIsReject(thetaStatus)) return;

    failMsg_ = std::string("Theta command rejected: ") +
               thetaRejectReason(thetaStatus) + ".";
    pending_ = false;
    run_.fail(thetaStatusToError(thetaStatus), now);
    releaseGuard();
}

} // namespace calib
} // namespace drivescope
