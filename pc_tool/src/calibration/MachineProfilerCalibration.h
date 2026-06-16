// MachineProfilerCalibration.h -- Test 2 (machine parameters / motor
// identification) calibration SERVICE. Third slice of
// docs/calibration-autotests-tz.md: move the real profiler state machine OUT of
// drawCalibMachine().
//
// This object owns what the Calibration tab used to drive inline in the draw
// loop: the START/ABORT command issue, the GET_STATUS polling cadence, the
// "DONE -> fetch results + config blob once" edge, the live progress history,
// and the pass/fail verdict. It acquires / releases the shared calib::RunGuard
// so only one calibration test runs at a time.
//
// Pure domain code: no ImGui, no transport, no threads. The motor profiler
// telemetry is fed in each frame as a small MachineTelemetry snapshot (the UI
// builds it from its MotorProfilerView); everything the service cannot do
// itself it asks for through injected Callbacks. The host clock is passed in as
// `now` (seconds) so the service never touches ImGui::GetTime.

#pragma once

#include "calibration/CalibrationModel.h"
#include "calibration/CalibrationRunGuard.h"

#include <functional>
#include <string>
#include <vector>

namespace drivescope {
namespace calib {

// Profiler CFG command actions (motor FW config subcmd 0x04). Naming the bare
// ints the old UI passed to sendMotorProfilerCommand; not a protocol change.
inline constexpr uint8_t kProfilerActionGetStatus = 0;
inline constexpr uint8_t kProfilerActionStart     = 2;
inline constexpr uint8_t kProfilerActionAbort     = 3;
// Procedure selectors (the `arg` of a START).
inline constexpr uint8_t kProfilerProcNoloadPrecheck = 1;  // XX: free shaft
inline constexpr uint8_t kProfilerProcBlockedIdentify = 2;  // KZ: locked rotor

// Snapshot of the motor profiler telemetry the UI feeds in each frame. Built
// from MainUi::MotorProfilerView; keeps the service free of any UI type.
struct MachineTelemetry {
    uint8_t state        = 0;   // calib::ProfilerState
    uint8_t progress     = 0;   // 0..100
    uint8_t error        = 0;   // calib::ProfilerError
    uint8_t activeTest   = 0;   // calib::ProfilerTest
    bool    result0Valid = false;   // Rs result present -> the pass gate
};

class MachineProfilerCalibration {
public:
    static constexpr double kStatusPollSec     = 0.25;  // GET_STATUS cadence
    static constexpr double kProgressSampleSec = 0.10;  // mini-graph throttle

    enum class Procedure { NoloadPrecheck, BlockedIdentify };   // XX / KZ

    struct Callbacks {
        std::function<bool()> sendLimits;        // MainUi::sendMotorProfilerLimits
        std::function<bool(uint8_t action, uint8_t arg, uint32_t value)>
                              sendCommand;       // MainUi::sendMotorProfilerCommand
        std::function<void()> requestResults;    // MainUi::requestMotorProfilerResults
        std::function<void()> requestConfigRefresh;  // MainUi::sendMotorBlobGet
        std::function<void()> clearResults;      // wipe MotorProfilerView result flags
        std::function<void(const std::string&)> setStatus;  // shared UI status line
    };

    void setCallbacks(Callbacks cb) { cb_ = std::move(cb); }
    void bindGuard(RunGuard* guard) { guard_ = guard; }
    bool isWired() const { return static_cast<bool>(cb_.sendCommand); }

    // --- operator actions (called from drawCalibMachine) -----------------
    // Start one procedure. Acquires the run guard; fails fast if another test
    // owns it. Returns true if limits + START were queued.
    bool start(Procedure proc, double now);
    // Operator abort: sends ABORT, sets a clear terminal state, releases guard.
    void abort(double now);

    // --- per-frame service (called from drawCalibMachine) ----------------
    // Reads the telemetry snapshot and drives status polling, the done-fetch
    // edge, the progress history and the verdict. canSend gates outgoing polls.
    void update(bool canSend, const MachineTelemetry& tel, double now);

    // --- queries for the thin UI -----------------------------------------
    RunPhase phase() const   { return run_.phase; }
    Verdict  verdict() const { return run_.verdict; }
    Error    error() const   { return run_.error; }
    bool running() const     { return run_.phase == RunPhase::Running; }
    const std::string& failMessage() const { return failMsg_; }
    void dismissFailMessage() { failMsg_.clear(); }
    // Live progress mini-graph series (seconds since run start, progress %).
    const std::vector<float>& progressTimes()  const { return progT_; }
    const std::vector<float>& progressValues() const { return progY_; }

private:
    void releaseGuard();   // release only if we own the slot

    Callbacks cb_;
    RunGuard* guard_ = nullptr;
    RunState  run_{};
    std::string failMsg_;

    bool    doneFetchIssued_ = false;  // results+blob fetched for this DONE
    double  nextStatusAt_    = 0.0;    // next GET_STATUS poll
    uint8_t lastState_       = 0;      // for the IDLE/DONE -> RUNNING edge
    double  runStart_        = 0.0;    // host clock the live graph started
    std::vector<float> progT_;
    std::vector<float> progY_;
};

} // namespace calib
} // namespace drivescope
