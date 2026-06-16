// MachineProfilerCalibration.cpp -- Test 2 profiler state machine. See header.

#include "calibration/MachineProfilerCalibration.h"

namespace drivescope {
namespace calib {

void MachineProfilerCalibration::releaseGuard()
{
    if (guard_ && guard_->ownedBy(Test::Machine)) guard_->release();
}

bool MachineProfilerCalibration::start(Procedure proc, double now)
{
    if (!cb_.sendCommand || !cb_.sendLimits) return false;
    // One run at a time: refuse if a different test owns the slot.
    if (guard_ && !guard_->tryAcquire(Test::Machine)) {
        failMsg_ = "Another calibration test is already running.";
        return false;
    }

    if (cb_.clearResults) cb_.clearResults();
    doneFetchIssued_ = false;
    failMsg_.clear();
    progT_.clear();
    progY_.clear();

    const bool isKz = (proc == Procedure::BlockedIdentify);
    const uint8_t arg = isKz ? kProfilerProcBlockedIdentify
                             : kProfilerProcNoloadPrecheck;

    // Limits first, then START. sendCommand returning true only means the CAN
    // frames were queued; the motor's state telemetry resolves the run.
    const bool okk = cb_.sendLimits() &&
                     cb_.sendCommand(kProfilerActionStart, arg, 0);
    if (okk) {
        run_.reset(Test::Machine);
        run_.enter(RunPhase::Running, now);   // FW telemetry refines this
        runStart_  = now;
        lastState_ = static_cast<uint8_t>(ProfilerState::Idle);
        if (cb_.setStatus)
            cb_.setStatus(isKz ? "profiler KZ identification started"
                               : "profiler XX pre-check started");
        return true;
    }

    releaseGuard();
    run_.fail(Error::MotorRejected, now);
    failMsg_ = isKz ? "profiler KZ identification failed to start"
                    : "profiler XX pre-check failed to start";
    if (cb_.setStatus) cb_.setStatus(failMsg_);
    return false;
}

void MachineProfilerCalibration::abort(double now)
{
    doneFetchIssued_ = false;
    if (cb_.sendCommand) cb_.sendCommand(kProfilerActionAbort, 0, 0);
    // Explicit terminal state + message so the pill can't hang on Running and
    // the operator sees the run was aborted.
    run_.enter(RunPhase::Aborted, now);
    run_.verdict     = Verdict::Fail;
    run_.error       = Error::None;       // operator action, not a motor fault
    run_.errorReason = "aborted by operator";
    failMsg_ = "Machine identification aborted by operator.";
    releaseGuard();
    if (cb_.setStatus) cb_.setStatus("profiler ABORT sent");
}

void MachineProfilerCalibration::update(bool canSend,
                                        const MachineTelemetry& tel, double now)
{
    const uint8_t st = tel.state;

    // Operator abort is sticky: ignore all telemetry (including a stray RUNNING
    // from still-draining frames) until the next start(). Keep the edge tracker
    // coherent so a later real run detects the IDLE/DONE -> RUNNING edge.
    if (run_.phase == RunPhase::Aborted) {
        lastState_ = st;
        return;
    }

    // --- map firmware state -> run phase / verdict -----------------------
    // Done's verdict is recomputed live: state reaches DONE before the Rs
    // result frame arrives, so the pass gate flips once result0Valid is true.
    switch (static_cast<ProfilerState>(st)) {
        case ProfilerState::Running:
            if (run_.phase != RunPhase::Running) run_.enter(RunPhase::Running, now);
            run_.verdict = Verdict::Unknown;
            break;
        case ProfilerState::Done:
            if (run_.phase != RunPhase::Done) run_.enter(RunPhase::Done, now);
            run_.verdict = tel.result0Valid ? Verdict::Pass : Verdict::Unknown;
            break;
        case ProfilerState::Error:
        case ProfilerState::Aborted:
            if (run_.phase != RunPhase::Failed) {
                run_.fail(profilerErrorToError(tel.error), now);
                failMsg_ = std::string("Machine identification failed: ") +
                           profilerErrorName(tel.error) + ".";
            }
            break;
        case ProfilerState::Idle:
            // Keep the last meaningful phase (don't downgrade a finished run).
            break;
    }

    // --- status polling while running ------------------------------------
    if (canSend && st == static_cast<uint8_t>(ProfilerState::Running) &&
        now >= nextStatusAt_) {
        cb_.sendCommand(kProfilerActionGetStatus, 0, 0);
        nextStatusAt_ = now + kStatusPollSec;
    }

    // --- DONE -> fetch results + config blob, exactly once ---------------
    if (st == static_cast<uint8_t>(ProfilerState::Done)) {
        if (canSend && !doneFetchIssued_) {
            if (cb_.requestResults)       cb_.requestResults();
            if (cb_.requestConfigRefresh) cb_.requestConfigRefresh();
            doneFetchIssued_ = true;
            if (cb_.setStatus)
                cb_.setStatus("profiler done -- results + config refresh requested");
        }
    } else {
        doneFetchIssued_ = false;
    }

    // --- live progress mini-graph ----------------------------------------
    // IDLE/DONE -> RUNNING edge resets the history; while RUNNING, append a
    // throttled (t, progress) sample.
    if (st == static_cast<uint8_t>(ProfilerState::Running) &&
        lastState_ != static_cast<uint8_t>(ProfilerState::Running)) {
        progT_.clear();
        progY_.clear();
        runStart_ = now;
    }
    if (st == static_cast<uint8_t>(ProfilerState::Running)) {
        const float t = static_cast<float>(now - runStart_);
        if (progT_.empty() || t - progT_.back() > kProgressSampleSec) {
            progT_.push_back(t);
            progY_.push_back(static_cast<float>(tel.progress));
        }
    }
    lastState_ = st;
}

} // namespace calib
} // namespace drivescope
