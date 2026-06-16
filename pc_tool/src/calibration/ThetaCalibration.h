// ThetaCalibration.h -- Test 1 (position-sensor / electrical-angle) calibration
// SERVICE. Second slice of docs/calibration-autotests-tz.md: move the real
// theta state machine OUT of the UI.
//
// This object owns everything the Calibration tab used to carry in loose
// MainUi members (calibThetaStartPending_ / calibThetaStartedAt_ /
// calibThetaPrevPhase_ / calibThetaFailMsg_) plus the edge/timeout/reject logic
// that was split between CalibrationTab.cpp and MainUi.cpp's frame parser:
//
//   - start-pending -> RUNNING -> DONE edge detection (MTR_ANS_THETA_CAL phase);
//   - start timeout (no RUNNING ack);
//   - firmware reject (theta CFG status) -> calibration error + reason;
//   - pass/fail verdict;
//   - acquire / release of the shared calib::RunGuard.
//
// It is pure domain code: no ImGui, no transport, no threads. Everything it
// cannot do itself it asks for through injected callbacks (Callbacks below):
// sending the CAN command, re-reading the result blob, and pushing a one-line
// status note back to the UI. The host clock is passed in as `now` (seconds)
// so the service never reaches for ImGui::GetTime itself.

#pragma once

#include "calibration/CalibrationModel.h"
#include "calibration/CalibrationRunGuard.h"

#include <functional>
#include <string>

namespace drivescope {
namespace calib {

class ThetaCalibration {
public:
    // Theta CFG command actions (motor FW config subcmd 0x00). Naming the bare
    // 3/4 that the old UI passed to sendMotorThetaConfig; not a protocol change.
    static constexpr uint8_t kActionCal     = 3;  // calibrate (RAM only)
    static constexpr uint8_t kActionCalSave = 4;  // calibrate + persist to flash

    // No RUNNING ack within this window after Start => the command was almost
    // certainly rejected (BUSY / NOT_READY). Matches the old 2 s UI timeout.
    static constexpr double kStartTimeoutSec = 2.0;

    struct Callbacks {
        // Send the theta CFG command. Returns false if the frame could not be
        // queued (no transport). Wraps MainUi::sendMotorThetaConfig.
        std::function<bool(uint8_t action, float alignCurrentA)> sendConfig;
        // Re-read the calibration result once, on the RUNNING->DONE_OK edge.
        // Wraps MainUi::readAllThetaCalibration. May be null.
        std::function<void()> readResult;
        // Push a short status note to the shared UI status line. May be null.
        std::function<void(const std::string&)> setStatus;
    };

    void setCallbacks(Callbacks cb) { cb_ = std::move(cb); }
    void bindGuard(RunGuard* guard) { guard_ = guard; }
    bool isWired() const { return static_cast<bool>(cb_.sendConfig); }

    // --- operator actions (called from drawCalibTheta) -------------------
    // Start a run. Returns true if the command was queued. Acquires the
    // shared run guard; fails fast if another test owns it.
    bool start(const ThetaParams& params, double now);
    // Operator abort. Resets run state and releases the guard. The caller is
    // responsible for the actual STOP/torque-off frame (a transport action).
    void abort(double now);

    // --- per-frame tick (called from drawCalibration) --------------------
    // Resolves the start-pending timeout. Cheap; safe to call every frame.
    void tick(double now);

    // --- telemetry events (called from MainUi frame parser) --------------
    // MTR_ANS_THETA_CAL phase byte (0 idle,1 running,2 done_ok,3 done_fail).
    void onCalTelemetry(uint8_t phase, double now);
    // Theta CFG (subcmd 0x00) response status. Treated as a reject only while a
    // run is in flight and only for failure statuses; benign GET replies (OK /
    // RUNNING / STORAGE_EMPTY) are ignored.
    void onCfgReject(uint8_t thetaStatus, double now);

    // --- queries for the thin UI -----------------------------------------
    RunPhase phase() const   { return run_.phase; }
    Verdict  verdict() const { return run_.verdict; }
    Error    error() const   { return run_.error; }
    bool startPending() const { return pending_; }
    bool running() const      { return run_.phase == RunPhase::Running; }
    const std::string& failMessage() const { return failMsg_; }
    void dismissFailMessage() { failMsg_.clear(); }

private:
    void releaseGuard();   // release only if we own the slot
    // Finalize a run on a DONE phase (verdict, result read-back, guard release).
    // Shared by the RUNNING->DONE edge and the no-RUNNING-seen fallback.
    void finalizeDone(bool ok, double now);

    Callbacks cb_;
    RunGuard* guard_ = nullptr;
    RunState  run_{};
    std::string failMsg_;
    uint8_t prevPhase_ = 0;   // last MTR_ANS_THETA_CAL phase, for edge detection
    double  startedAt_ = 0.0; // host clock of the Start press (timeout base)
    bool    pending_   = false;
};

} // namespace calib
} // namespace drivescope
