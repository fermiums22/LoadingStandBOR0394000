// CalibrationModel.h -- pure domain models for the four motor calibration
// autotests (docs/calibration-autotests-tz.md, "First Implementation Slice").
//
// Layering (TZ "Code Organization"):
//   - src/protocol/*       : pure wire codecs (no session state).
//   - src/calibration/*    : THIS layer. Test state, Params/Result/Phase/Error
//                            structs, validation + metric types. No UI, no
//                            threads, no transport, no ImGui, no retry policy.
//   - ui/CalibrationTab.cpp: renders this state and calls services.
//
// This header is the readable type vocabulary the UI and future per-test
// services share. It deliberately holds NO execution logic yet -- start/poll/
// read/apply land in later slices test by test. The firmware-reported enums
// below MIRROR the motor FW status bytes (their integer values are part of the
// CAN contract); changing the numbers here without changing firmware is a bug,
// hence the static_asserts in CalibrationRunGuard.cpp.

#pragma once

#include <cstdint>

namespace drivescope {
namespace calib {

// ===========================================================================
// Which test. Order matches the Calibration tab left rail (kCalibSectionLabels).
// ===========================================================================
enum class Test : uint8_t {
    Theta       = 0,  // Test 1: position-sensor / electrical-angle calibration
    Machine     = 1,  // Test 2: machine params / motor identification (profiler)
    CurrentStep = 2,  // Test 3: current PI loop step response
    SpeedLoop   = 3,  // Test 4: speed loop / inertia / coast-down
};

inline const char* testName(Test t)
{
    switch (t) {
        case Test::Theta:       return "Theta calibration";
        case Test::Machine:     return "Machine params";
        case Test::CurrentStep: return "Current loop step";
        case Test::SpeedLoop:   return "Speed loop / inertia";
    }
    return "?";
}

// ===========================================================================
// Generic run lifecycle, shared by all four tests. This is the PC-tool-side
// state machine skeleton; the firmware-reported phase enums (further down)
// feed it but are not the same thing -- e.g. ThetaCalPhase::Running drives a
// transition into RunPhase::Running, ThetaCalPhase::DoneOk into ReadingResult.
// ===========================================================================
enum class RunPhase : uint8_t {
    Idle,            // nothing scheduled
    CheckingGuards,  // verifying preconditions (§"Before starting any test")
    Starting,        // command sent, awaiting motor's RUNNING ack
    Running,         // motor executing the test
    ReadingResult,   // RUNNING->DONE edge seen, pulling config/blob/capture once
    Computing,       // deriving metrics / validating result
    Done,            // result ready, awaiting operator Apply/Save
    Failed,          // see RunState::error for the reason
    Aborted,         // operator or safety-path abort
};

inline const char* runPhaseName(RunPhase p)
{
    switch (p) {
        case RunPhase::Idle:           return "idle";
        case RunPhase::CheckingGuards: return "checking";
        case RunPhase::Starting:       return "starting";
        case RunPhase::Running:        return "running";
        case RunPhase::ReadingResult:  return "reading result";
        case RunPhase::Computing:      return "computing";
        case RunPhase::Done:           return "done";
        case RunPhase::Failed:         return "failed";
        case RunPhase::Aborted:        return "aborted";
    }
    return "?";
}

inline bool runPhaseIsBusy(RunPhase p)
{
    return p == RunPhase::CheckingGuards || p == RunPhase::Starting ||
           p == RunPhase::Running        || p == RunPhase::ReadingResult ||
           p == RunPhase::Computing;
}

inline bool runPhaseIsTerminal(RunPhase p)
{
    return p == RunPhase::Done || p == RunPhase::Failed || p == RunPhase::Aborted;
}

// Pass/fail verdict, decided after RunPhase::Computing against the per-test
// pass criteria (TZ "Pass criteria" / "Fail criteria").
enum class Verdict : uint8_t {
    Unknown,  // not run this session / still in progress
    Pass,
    Fail,
};

// Why a run failed or aborted. Maps the TZ "Fail criteria" common to all four
// tests onto a single readable enum; the firmware-specific reject codes
// (ThetaStatus / ProfilerError) are translated into one of these.
enum class Error : uint8_t {
    None,
    PreconditionFailed,  // a §"Before starting" gate was not met
    MotorBusy,           // motor rejected: already running / wrong mode
    MotorRejected,       // motor rejected: BAD_ARG / NOT_READY
    Timeout,             // never reached the expected phase in time
    Fault,               // a critical fault appeared mid-run
    IncompleteData,      // e.g. missing sectors, missing profiler outputs
    OutOfRange,          // a measured value failed its sanity band
    CaptureFailed,       // Triggered Capture did not return usable data
    FeedbackLost,        // speed/current feedback invalid during the run
    Unstable,            // response diverged / unsafe overshoot
};

inline const char* errorName(Error e)
{
    switch (e) {
        case Error::None:               return "none";
        case Error::PreconditionFailed: return "precondition not met";
        case Error::MotorBusy:          return "motor busy";
        case Error::MotorRejected:      return "motor rejected command";
        case Error::Timeout:            return "timeout";
        case Error::Fault:              return "motor fault";
        case Error::IncompleteData:     return "incomplete data";
        case Error::OutOfRange:         return "value out of range";
        case Error::CaptureFailed:      return "capture failed";
        case Error::FeedbackLost:       return "feedback lost";
        case Error::Unstable:           return "unstable response";
    }
    return "?";
}

// ===========================================================================
// Firmware-reported status mirrors. The INTEGER VALUES below are the CAN wire
// contract (motor FW reports these bytes in MTR_ANS_THETA_CAL / MTR_ANS_CONFIG
// profiler telemetry). They replace the file-local switch tables previously
// duplicated in CalibrationTab.cpp (calibThetaStatusName / calibProfiler*Name).
// ===========================================================================

// MTR_ANS_THETA_CAL phase byte.
enum class ThetaCalPhase : uint8_t {
    Idle     = 0,
    Running  = 1,
    DoneOk   = 2,
    DoneFail = 3,
};

inline const char* thetaCalPhaseName(uint8_t p)
{
    switch (static_cast<ThetaCalPhase>(p)) {
        case ThetaCalPhase::Idle:     return "idle";
        case ThetaCalPhase::Running:  return "RUNNING";
        case ThetaCalPhase::DoneOk:   return "done (ok)";
        case ThetaCalPhase::DoneFail: return "done (FAIL)";
    }
    return "?";
}

// Theta CFG response status byte (subcmd 0x00).
enum class ThetaStatus : uint8_t {
    Ok           = 0,
    BadArg       = 1,
    Busy         = 2,
    NotReady     = 3,
    StorageEmpty = 4,
    StorageFail  = 5,
    Fault        = 6,
    Running      = 7,
};

inline const char* thetaStatusName(uint8_t s)
{
    switch (static_cast<ThetaStatus>(s)) {
        case ThetaStatus::Ok:           return "OK";
        case ThetaStatus::BadArg:       return "BAD_ARG";
        case ThetaStatus::Busy:         return "BUSY";
        case ThetaStatus::NotReady:     return "NOT_READY";
        case ThetaStatus::StorageEmpty: return "STORAGE_EMPTY";
        case ThetaStatus::StorageFail:  return "STORAGE_FAIL";
        case ThetaStatus::Fault:        return "FAULT";
        case ThetaStatus::Running:      return "RUNNING";
    }
    return "?";
}

// Motor profiler state machine (CFG subcmd 0x04 telemetry).
enum class ProfilerState : uint8_t {
    Idle    = 0,
    Running = 1,
    Done    = 2,
    Error   = 3,
    Aborted = 4,
};

inline const char* profilerStateName(uint8_t s)
{
    switch (static_cast<ProfilerState>(s)) {
        case ProfilerState::Idle:    return "IDLE";
        case ProfilerState::Running: return "RUNNING";
        case ProfilerState::Done:    return "DONE";
        case ProfilerState::Error:   return "ERROR";
        case ProfilerState::Aborted: return "ABORTED";
    }
    return "?";
}

enum class ProfilerTest : uint8_t {
    None = 0,
    Xx   = 1,   // resistance / open-circuit sub-test
    Kz   = 2,   // locked-rotor sub-test
};

inline const char* profilerTestName(uint8_t t)
{
    switch (static_cast<ProfilerTest>(t)) {
        case ProfilerTest::None: return "NONE";
        case ProfilerTest::Xx:   return "XX";
        case ProfilerTest::Kz:   return "KZ";
    }
    return "?";
}

enum class ProfilerError : uint8_t {
    None     = 0,
    Busy     = 1,
    NotReady = 2,
    Fault    = 3,
    BadArg   = 4,
    Moved    = 5,
    Signal   = 6,
    Range    = 7,
};

inline const char* profilerErrorName(uint8_t e)
{
    switch (static_cast<ProfilerError>(e)) {
        case ProfilerError::None:     return "NONE";
        case ProfilerError::Busy:     return "BUSY";
        case ProfilerError::NotReady: return "NOT_READY";
        case ProfilerError::Fault:    return "FAULT";
        case ProfilerError::BadArg:   return "BAD_ARG";
        case ProfilerError::Moved:    return "MOVED";
        case ProfilerError::Signal:   return "SIGNAL";
        case ProfilerError::Range:    return "RANGE";
    }
    return "?";
}

// Translate a firmware reject status into our generic Error. Centralised so
// every test reports rejects consistently (TZ "update the run state with a
// short error reason").
inline Error thetaStatusToError(uint8_t s)
{
    switch (static_cast<ThetaStatus>(s)) {
        case ThetaStatus::Ok:
        case ThetaStatus::Running:      return Error::None;
        case ThetaStatus::Busy:         return Error::MotorBusy;
        case ThetaStatus::NotReady:     return Error::MotorRejected;
        case ThetaStatus::BadArg:       return Error::MotorRejected;
        case ThetaStatus::Fault:        return Error::Fault;
        case ThetaStatus::StorageEmpty:
        case ThetaStatus::StorageFail:  return Error::IncompleteData;
    }
    return Error::MotorRejected;
}

inline Error profilerErrorToError(uint8_t e)
{
    switch (static_cast<ProfilerError>(e)) {
        case ProfilerError::None:     return Error::None;
        case ProfilerError::Busy:     return Error::MotorBusy;
        case ProfilerError::NotReady: return Error::MotorRejected;
        case ProfilerError::BadArg:   return Error::MotorRejected;
        case ProfilerError::Fault:    return Error::Fault;
        case ProfilerError::Moved:    return Error::PreconditionFailed;  // rotor not locked
        case ProfilerError::Signal:   return Error::FeedbackLost;
        case ProfilerError::Range:    return Error::OutOfRange;
    }
    return Error::MotorRejected;
}

// ===========================================================================
// Gate / sanity thresholds. Single source of truth for the calibration-domain
// pass bands (kept here, not scattered in UI switch bodies). Datasheet nominal
// values for Rs/Ld/Lq still live in ui/CalibMeta.h; only the tolerance/quality
// gates that the calibration verdict depends on belong here.
// ===========================================================================
inline constexpr int   kThetaSectorCount       = 6;     // 60-deg Hall sectors
inline constexpr float kThetaSectorSkewTolDeg  = 15.0f; // advisory per-sector skew band
inline constexpr float kMachineRsTolPct        = 10.0f; // Rs PASS/WARN band
inline constexpr float kMachineLTolPct         = 15.0f; // Ld/Lq PASS/WARN band

// ===========================================================================
// Test 1 -- Theta / position-sensor calibration.
// ===========================================================================
struct ThetaParams {
    float alignCurrentA = 0.30f;        // alignment-sweep current vector magnitude
    bool  saveAfterCal  = false;        // persist to flash after a good run
    bool  shaftFreeConfirmed = false;   // operator confirm: shaft free to rotate

    static constexpr float kAlignCurrentMinA = 0.05f;
    static constexpr float kAlignCurrentMaxA = 1.00f;
};

struct ThetaResult {
    bool  reachedDone = false;
    float offsetRad   = 0.0f;                       // global electrical-angle offset
    float sectorLutRad[kThetaSectorCount] = {0,0,0,0,0,0};
    bool  sectorValid [kThetaSectorCount] = {false,false,false,false,false,false};
    float coherence   = 0.0f;                        // FW-reported quality (0..1)
    bool  coherenceValid = false;

    int validSectorCount() const
    {
        int n = 0;
        for (bool v : sectorValid) if (v) ++n;
        return n;
    }
    bool allSectorsValid() const { return validSectorCount() == kThetaSectorCount; }
};

// ===========================================================================
// Test 2 -- Machine parameters / motor identification (profiler).
// ===========================================================================
struct MachineParams {
    float testCurrentA   = 0.50f;
    float maxVoltagePct  = 5.00f;
    float pulseMs        = 20.0f;
    float settleMs       = 20.0f;
    int   repeatCount    = 4;
    float bandwidthHz    = 300.0f;
    bool  rotorLockedConfirmed = false; // operator confirm: rotor mechanically locked
};

struct MachineResult {
    bool  valid    = false;
    float rsOhm    = 0.0f;   bool rsValid     = false;
    float ldH      = 0.0f;   bool ldValid     = false;
    float lqH      = 0.0f;   bool lqValid     = false;
    float lambdaWb = 0.0f;   bool lambdaValid = false;
};

// ===========================================================================
// Test 3 -- Current loop step test. Structure-only this slice; the step-metric
// MATH (rise/overshoot/settle/...) is intentionally deferred (TZ rule 6).
// ===========================================================================
enum class CurrentStepMode : uint8_t {
    Iq = 0,   // primary target
    Id = 1,   // same structure, added later
};

inline const char* currentStepModeName(CurrentStepMode m)
{
    return m == CurrentStepMode::Iq ? "Iq step" : "Id step";
}

struct CurrentStepParams {
    CurrentStepMode mode = CurrentStepMode::Iq;
    float fromA      = 0.0f;
    float toA        = 0.3f;
    float stepMs     = 500.0f;   // hold time before/after the step
    float captureUs  = 200.0f;   // capture sample period
    bool  areaClearConfirmed = false;  // operator confirm: safe to produce torque
    bool  thetaValidRequired = true;   // Test 3 needs a valid theta calibration
};

// Step-response metrics, common to Test 3 (current) and Test 4 (speed step).
// Fields are the TZ Test 3 list; values are filled by a future metric pass.
struct StepMetrics {
    bool  valid        = false;
    float baseline     = 0.0f;   // pre-step steady value
    float target       = 0.0f;   // post-step steady value
    float riseMs       = 0.0f;   // 10%..90% rise time
    float overshootPct = 0.0f;   // peak overshoot beyond target, %
    float settleMs     = 0.0f;   // time to stay within the settle band
    float ssErrPct     = 0.0f;   // steady-state error, %
    float noiseRms     = 0.0f;   // post-settle residual RMS
};

// ===========================================================================
// Test 4 -- Speed loop / inertia / coast-down.
// ===========================================================================
enum class SpeedScenario : uint8_t {
    SpeedStep = 0,   // speed PI step response
    CoastDown = 1,   // decay / friction / inertia fit
};

inline const char* speedScenarioName(SpeedScenario s)
{
    return s == SpeedScenario::SpeedStep ? "speed step" : "coast-down";
}

struct SpeedLoopParams {
    SpeedScenario scenario = SpeedScenario::SpeedStep;
    float startRpm    = 300.0f;
    float captureUs   = 500.0f;
    bool  safeSpinConfirmed = false;   // operator confirm: safe to spin the motor
};

struct SpeedLoopResult {
    bool  valid       = false;
    // Speed-step branch reuses StepMetrics (rise/overshoot/settle/...).
    StepMetrics step;
    float speedRipple = 0.0f;   bool rippleValid = false;
    // Coast-down branch: decay fit -> friction/inertia (math deferred).
    float decayTau    = 0.0f;   bool decayValid  = false;
    float inertiaJ    = 0.0f;   bool inertiaValid = false;
    float frictionB   = 0.0f;   bool frictionValid = false;
};

// ===========================================================================
// RunState -- the readable per-run status the UI renders and the (future)
// per-test service drives. One instance per test; the active-run guard
// (CalibrationRunGuard) ensures only one is non-idle at a time.
// ===========================================================================
struct RunState {
    Test     test       = Test::Theta;
    RunPhase phase      = RunPhase::Idle;
    Verdict  verdict    = Verdict::Unknown;
    Error    error      = Error::None;
    const char* errorReason = "";   // short human reason; points at errorName() or a literal
    double   startedAt      = 0.0;  // host clock (e.g. ImGui::GetTime) the run began
    double   phaseEnteredAt = 0.0;  // host clock the current phase was entered
    uint8_t  progressPct    = 0;    // 0..100 when the motor reports it

    bool isBusy() const     { return runPhaseIsBusy(phase); }
    bool isTerminal() const { return runPhaseIsTerminal(phase); }

    void reset(Test t)
    {
        *this = RunState{};
        test = t;
    }

    void fail(Error e, double now)
    {
        phase          = RunPhase::Failed;
        verdict        = Verdict::Fail;
        error          = e;
        errorReason    = errorName(e);
        phaseEnteredAt = now;
    }

    void enter(RunPhase p, double now)
    {
        phase          = p;
        phaseEnteredAt = now;
    }
};

} // namespace calib
} // namespace drivescope
