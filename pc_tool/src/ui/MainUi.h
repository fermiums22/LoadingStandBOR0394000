#pragma once

#include "can/CanTraceBuffer.h"
#include "can/SlcanParser.h"
#include "elf/ElfSymbolLoader.h"
#include "logging/CanLogger.h"
#include "logging/SignalLogger.h"
#include "sniffer/CanSniffer.h"
#include "stream/StreamWatchState.h"
#include "protocol/CanBootloaderFlasher.h"
#include "protocol/DebugProtocol.h"
#include "transport/SlcanTransport.h"
#include "transport/TcpCanTransport.h"
#include "loadstand/LoadStandClient.h"
#include "ui/CalibMeta.h"
#include "calibration/CalibrationModel.h"
#include "calibration/CalibrationRunGuard.h"
#include "calibration/ThetaCalibration.h"
#include "calibration/MachineProfilerCalibration.h"

#include <atomic>
#include <array>
#include <deque>
#include <map>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace drivescope {

struct ToolConfig;
struct WatchConfig;

struct DeployProgressState {
    std::atomic<int> percent{0};
    std::atomic<bool> active{false};
    mutable std::mutex mutex;
    std::string text;
};

enum class FilterMode : int { Include = 0, Exclude = 1 };
enum class FilterField : int { Id = 0, Substr = 1, Endpoint = 2, Direction = 3 };

struct CanFilterRule {
    bool enabled = true;
    FilterMode mode = FilterMode::Include;
    FilterField field = FilterField::Id;
    uint32_t idValue = 0;
    bool extended = false;
    std::array<char, 96> textBuf{};
    int dirValue = 0;
};

class MainUi {
public:
    MainUi() = default;
    ~MainUi();
    void applyConfig(const ToolConfig& config);
    void draw();
    void setNativeWindow(void* glfwWindow) { nativeWindow_ = glfwWindow; }
    void setScenarioPath(const std::string& path);
    bool saveScenarioNow(const char* reason = nullptr);

private:
    void* nativeWindow_ = nullptr;
    void drawTitleBar(float height);
    enum class TransportKind { Slcan, Tcp, WifiAp, Manual };

    struct WatchVar : VariableSymbol {
        bool enabled = true;
        bool plot = false;
        // Whether this watch is included in the next Triggered Capture
        // shot. Capture is independent of `plot`/`enabled`: a variable
        // can be captured-only (no live polling), captured-and-plotted, etc.
        bool capture = false;
        double plotScale = 1.0;
        float pollHz = 10.0f;
        double nextPollTime = 0.0;
        uint8_t seq = 0;
        double lastTimestamp = 0.0;
        double value = 0.0;
        bool valueOk = false;
        std::string valueText = "n/a";
        std::vector<double> xs;
        std::vector<double> ys;
        std::string source;
        // Phase C STREAM_* subscription state. When streamSubscribed=true,
        // pollWatchVariables() does NOT send READ_MEM for this watch —
        // values arrive via STREAM_VALUE frames from the motor at the
        // adaptive period (~166 Hz for 12 vars, ~333 Hz for 6 vars).
        uint8_t streamSlotId = 0xFF;     // motor-assigned slot; 0xFF = none
        bool    streamSubscribed = false;
        bool    streamPending = false;   // ADD_REQ sent, awaiting ADD_RSP
        bool    streamQuotaLimited = false;
        double  streamSentAt = 0.0;
        double  streamLastValueAt = 0.0;
        // Recovery state machine: backoff + failed_no_motor_response instead of
        // a blind 2 s re-ADD. valueStaleSec 2.5 (tolerate a brief gap), addTimeout
        // 0.6 s, resync backoff 2->8 s, 4 unanswered ADDs -> failed.
        StreamWatch sw{StreamWatchConfig{2.5, 0.6, 2.0, 8.0, 4}};
    };

    struct SymbolSource {
        std::string name;
        std::array<char, 260> elfBuf{};
        std::array<char, 260> symBuf{};
        std::array<char, 64> filterBuf{};
        std::vector<VariableSymbol> symbols;
        std::string status;
        uint8_t defaultNodeId = 2;
    };


    void updateIo();
    void drawConnection();
    void drawCanMonitor();
    void drawVariables();
    void drawPlots();
    void drawRemoteControl();
    void drawDeviceUpdates();
    void drawLoadStand();     // loading-stand brake torque over the Pi TCP bridge
    void drawMotorConfig();   // Phase 3: motor param-table tab (Theta/PI/Profiler/DIAG)
    void drawMotorTab();
    void drawRkTab();

    // ---- Calibration tab (docs/calibration-tab-redesign.md) ------------
    // A dedicated tab that re-homes the scattered theta + profiler UI into
    // a left-rail layout with a compact config strip and four test panels.
    void drawCalibration();
    void drawCalibrationRail();
    void drawCalibConfigStrip();
    void drawCalibTheta();        // Test 1 -- migrated from drawMotorTab()
    void drawCalibMachine();      // Test 2 -- migrated from drawMotorProfilerPanel()
    void drawCalibPiTuning();     // Test 3 -- new
    void drawCalibInertia();      // Test 4 -- new
    void drawCalibConfigFull();   // compact dense editable config grid

    // Shared Calibration helpers (spec §5).
    struct CalibGate {
        bool     canSend   = false;
        bool     inDiag    = false;
        bool     motorIdle = false;
        uint32_t faultMask = 0;
    };
    bool calibPreconditions(CalibGate& g);
    // INAV-style range slider + numeric entry + Set; returns true if the
    // staged value changed this frame. liveValue is the current motor value.
    bool calibRangeField(const CalibParamMeta& m, float liveValue,
                         CalibFieldSync& sync, bool showSetButton);
    void calibSyncChip(const CalibFieldSync& sync);
    void calibResultTable(const CalibResultRow* rows, int n);
    void calibResultPlot(const char* title, const float* t,
                         const float* y1, const float* y2, int n,
                         double triggerT);
    void beginTestPanel(const char* title, const char* blurb, CalibPill pill);
    void endTestPanel();
    bool calibPreconditionRow(const char* label, bool ok);
    // Fixed-height single-line status slot. Shows the last calibStatus_ note
    // in place so incoming telemetry never reflows the panel below it.
    void calibStatusLine();
    // Read a blob-resident f32/u32 config field by param-table id.
    float calibBlobValueF32(uint16_t id);
    // Drive the per-field sync chips off a fresh motorBlob_ snapshot.
    void calibRefreshSyncFromBlob();

    int         calibSection_ = 0;           // selected left-rail section
                                             // (0 = Test 1 Theta, the default
                                             //  landing section)
    std::string calibStatus_;                // last action note (fixed slot)
    std::unordered_map<uint16_t, CalibFieldSync> calibSync_;  // per-field sync

    // Per-test result pills (index matches kCalibSectionLabels order minus the
    // trailing Config row -- see CalibrationTab.cpp).
    CalibPill calibPillTheta_   = CalibPill::None;
    CalibPill calibPillMachine_ = CalibPill::None;
    CalibPill calibPillPi_      = CalibPill::None;
    CalibPill calibPillInertia_ = CalibPill::None;

    // Single-active-run rule (calibration-autotests-tz.md: "Only one
    // calibration run may be active at a time"). Domain layer, no UI deps.
    // Wired in here so future per-test slices acquire/release it on their
    // Start/Done/Abort paths instead of re-implementing the invariant with
    // per-test *StartPending_ booleans. Not yet enforced by the existing
    // panels (that migration is the next slice) -- adding it changes no
    // current behaviour.
    calib::RunGuard calibRunGuard_;

    // Test 1 (theta) state machine. Owns the start-pending / RUNNING / DONE-edge
    // / timeout / reject / verdict logic that used to live as loose MainUi
    // members (calibThetaStartPending_ / *StartedAt_ / *PrevPhase_ / *FailMsg_)
    // split across CalibrationTab.cpp and the frame parser. drawCalibTheta()
    // now only renders its state and calls start()/abort(); the frame parser
    // feeds it onCalTelemetry()/onCfgReject(). Wired in wireThetaCalibration().
    calib::ThetaCalibration thetaCal_;
    void wireThetaCalibration();
    // Derive the rail/panel result pill from thetaCal_ + live telemetry.
    CalibPill thetaPill() const;

    // Test 2 (machine params / profiler) state machine. Owns the START/ABORT
    // issue, GET_STATUS polling, the DONE -> fetch-results+blob edge, the live
    // progress history and the verdict that used to run inline in
    // drawCalibMachine(). The panel now only renders + calls start()/abort()/
    // update(). Wired in wireMachineProfilerCalibration().
    calib::MachineProfilerCalibration machineProf_;
    void wireMachineProfilerCalibration();
    CalibPill machinePill() const;

    // Test 1 operator confirm (shaft free to rotate).
    bool calibThetaShaftFree_ = false;
    // Test 2 operator confirm (rotor mechanically locked, required for KZ).
    bool calibRotorLocked_ = false;
    // Test 3 operator confirm (safe to energise / shaft area clear).
    bool calibPiAreaClear_ = false;
    int  calibPiLoopSel_   = 0;     // 0 = Iq current step, 1 = speed step
    float calibPiStepFrom_ = 0.0f;
    float calibPiStepTo_   = 0.3f;
    float calibPiStepMs_   = 500.0f;
    float calibPiCaptureUs_ = 200.0f;
    // Test 4 operator confirm (safe to spin the motor).
    bool  calibInertiaSafeSpin_ = false;
    float calibInertiaStartRpm_ = 300.0f;
    float calibInertiaCaptureUs_ = 500.0f;

    // ---- Test 3 / Test 4 live step-response capture ---------------------
    // A self-contained capture run for the Calibration tab. It does NOT
    // touch the Plots-tab capture_ session, but it uses the same watches_
    // variable table for slot selection. Tests auto-mark their variables
    // from symbols_motor.json, build a GET /capture query, and decode the
    // binary body into per-slot float series. Used by Test 3 (PI step) and
    // Test 4 (coast-down) -- both are step/transient acquisitions with the
    // same shape.
    enum class CalibCapPhase {
        Idle,        // nothing running
        Stepping,    // step command issued, waiting for the worker
        Fetching,    // worker thread running fetchCaptureViaHttp
        Done,        // result decoded, plot/metrics ready
        Failed,      // command rejected or capture failed
    };
    struct CalibStepMetrics {
        bool   valid       = false;
        float  baseline    = 0.0f;   // pre-step steady value
        float  target      = 0.0f;   // post-step steady value
        float  peak        = 0.0f;   // max excursion after the step
        float  riseMs      = 0.0f;   // 10%..90% rise time
        float  overshootPct = 0.0f;  // peak overshoot beyond target, %
        float  settleMs    = 0.0f;   // time to stay within +/-2% band
        float  ssErrPct    = 0.0f;   // steady-state error, %
        float  noiseRms    = 0.0f;   // RMS of the post-settle residual
    };
    struct CalibCapRun {
        CalibCapPhase phase = CalibCapPhase::Idle;
        std::string   status;             // human status line
        std::string   error;              // populated on Failed
        double        startedAt = 0.0;    // ImGui time the run began
        double        stepAt    = 0.0;    // ImGui time the step was commanded
        bool          isSpeedLoop = false;// true = speed step, false = Iq
        bool          synthSetpoint = false;
        bool          abortRequested = false;
        float         commandFrom = 0.0f;
        float         commandTo = 0.0f;
        float         stepDelayMs = 0.0f;
        float         pulseWidthMs = 0.0f;   // 0 = held step, >0 = finite pulse
        // Decoded series. t[] in milliseconds relative to capture start.
        std::vector<float> t;
        std::vector<float> setpoint;      // commanded value (slot 0)
        std::vector<float> measured;      // measured value  (slot 1)
        std::vector<float> amplitude;     // |current vector| or measured copy
        float              periodUs = 200.0f;
        uint16_t           triggerIdx = 0;
        // Frequency spectrum of `measured` (filled on Done).
        std::vector<float> fftFreq;
        std::vector<float> fftMag;
        CalibStepMetrics   metrics;
    };
    CalibCapRun calibStep_;     // Test 3 step response
    CalibCapRun calibCoast_;    // Test 4 coast-down

    // Test 2 live mini-graph + run-state machine now live in machineProf_
    // (calib::MachineProfilerCalibration); the progress history is exposed via
    // its progressTimes()/progressValues() spans.

    // The HTTP capture worker state for the Calibration tab lives further
    // down, after the CaptureHttpResult struct is defined (search
    // "calibCapThread_").

    // Calibration tests use the same watch table as the Variables/Plots UI.
    // Required variables are resolved from symbols_motor.json and then
    // marked for capture/plot there; no separate hardcoded slot list exists.
    bool calibEnsureMotorCaptureVars(const std::vector<const char*>& names,
                                     std::vector<size_t>& watchIdx,
                                     std::string& error);
    std::string calibBuildCaptureQueryFromWatches(const std::vector<size_t>& watchIdx,
                                                  int durationMs, uint16_t periodUs,
                                                  uint8_t prePct, uint8_t postPct) const;
    // Spawn the calib capture worker. Returns false if a worker is busy or
    // the transport cannot yield an HTTP host. On success calibCapDone_
    // flips true when the worker finishes.
    bool calibLaunchCapture(CalibCapRun& run, const std::string& query);
    // Drain a finished calib capture in the GUI tick: decode the binary
    // body into run.setpoint/measured/amplitude, compute FFT + metrics.
    void calibDrainCapture();
    // Compute step-response metrics from a decoded run.
    void calibComputeStepMetrics(CalibCapRun& run);
    // Shared step-response renderer (Test 3 graphs 1/2 + metric bars).
    void drawCalibStepResults(CalibCapRun& run);
    // Test 4 coast-down decay renderer (decay curve + exponential fit).
    void drawCalibCoastResults(CalibCapRun& run);

    void drawTabStrip();   // custom icon-tab bar; sets activeTab_ + draws separator
    void drawLogging();

    // Shared per-tab header strip: a single Take control / Release control
    // button plus a compact diag-mode indicator. Rendered at the top of every
    // tab body (see draw()) so device handover is one consistent control with
    // one source of truth (diagModeOn_ / lastDiagStatus_), never duplicated or
    // buried inside a single tab.
    void drawDeviceControlBar();

    // ---- Diagnostic mode (motor/RK control over CAN) ------------------
    // Send a frame via the active transport with the given mainPCB cmd.
    // Returns false if no transport is open.
    bool sendMainCmd(uint16_t cmd, const uint8_t payload[8]);
    void diagModeRequest(bool enter);
    void sendMotorProxy(uint8_t action, float speed_pct);
    void sendRkProxy(uint8_t subcmd);
    void sendPfcProxy(bool enable, bool overrideSafety);
    bool sendMotorThetaConfig(uint8_t action, float value, uint8_t bin = 0xFFu);
    bool sendMotorSectorSet(uint8_t bin, float value, bool save);
    // Direct MTR_CMD_CTRL driverMode-3 (CURRENT) Iq step command. Builds
    // and sends an 8-byte control frame straight over the active
    // transport (works in SLCAN / Wi-Fi STA / AP alike). The motor FW
    // hard-clamps iq_amps to a safe limit; pass 0 to hold zero torque,
    // and use sendMotorProxy STOP afterwards to leave current mode.
    bool sendMotorCurrentStep(float iq_amps);
    void readAllThetaCalibration();
    void maybePollMotorTheta(double now);

    /* Whole-config-blob protocol (replaces per-field theta GET/SET 2026-05-13).
     * Atomic: pc_tool gets the whole 164-byte config blob in one streamed
     * response and ships it back the same way; FW verifies CRC32 before
     * Scatter to RAM. No more per-sector race. */
    bool sendMotorBlobGet();
    bool sendMotorBlobSet(const uint8_t blob_bytes[164]);
    /* Returns true if the cache holds a fully-received, magic-verified
     * blob; out_buf (>=164 bytes) is filled with a snapshot. */
    bool motorBlobSnapshot(uint8_t out_buf[164]) const;
    /* Stage a single theta-slot edit into the cached blob + send to motor.
     *   slot 0    = global theta_offset
     *   slot 1..6 = theta_sector_lut[slot-1]
     * Recomputes header.payload_crc32 over the modified payload before
     * the SET stream goes out so the FW commit-verify passes byte-for-byte.
     * Returns false if no valid blob was cached yet (user must Read first). */
    bool applyThetaSlotEdit(int slot, float value_rad);
    /* Generic blob-backed Set: writes value_bits at the byte offset
     * derived from param-table id, recomputes payload CRC32, kicks off
     * a 33-chunk BLOB SET. Returns false if id is not blob-resident,
     * blob cache is empty, or the SET tx fails. Used by the Motor
     * Config tab so Set buttons on PI gains / profiler / pfc edit the
     * same shared snapshot as the theta calibration tab. */
    bool applyMotorConfigBlobSet(uint16_t id, uint32_t value_bits);

    // Motor tone-mode (driverMode 8 / cmd 0x008): sends a single note frame.
    // The motor firmware buzzes the stator at `freq_hz` for `duration_ms`,
    // amp clamped to 0..25 % on the firmware side. Frame layout:
    //   data[0..1] = freq_hz (u16 LE)
    //   data[2..3] = duration_ms (u16 LE)
    //   data[4]    = amp_pct (u8, 0..100; firmware clamps to 25)
    //   data[5..7] = 0xFF
    bool sendMotorTone(uint16_t freq_hz, uint16_t duration_ms, uint8_t amp_pct);
    // Kick a multi-note melody: queues notes, pumpMelody() advances per-frame.
    void startMelody(int melodyIdx);
    void stopMelody();
    void pumpMelody(double now);
    // Pumped from updateIo while diag mode is active so the WDG on mainPCB
    // doesn't time us out.
    void maybeSendDiagHeartbeat();

    void ensureDefaultSources();
    void addWatch(const VariableSymbol& sym, bool plot, const std::string& sourceName);
    void togglePlot(const VariableSymbol& sym, const std::string& sourceName, uint8_t nodeId);
    void handleFrame(const CanFrame& frame);
    void pollWatchVariables(double now);

    // Triggered Capture session machinery. See drawCaptureControl() for
    // the UI panel and pollCaptureSession() for the state machine that
    // walks RESET -> SET_SLOT*N -> SET_CONFIG -> SET_TRIGGER -> ARM ->
    // (poll STATUS until DONE) -> READ_CHUNK -> ingest CAP_DATA.
    enum class CaptureSessionPhase {
        Idle,
        Reset,
        SetSlot,
        SetConfig,
        SetTrigger,
        Arm,
        WaitDone,
        ReadChunk,
        Drain,
        VerifyCrc,    // all bytes received → asked MCU for CRC32, awaiting reply
        Apply,
        CooldownDone,
    };
    struct CaptureSession {
        CaptureSessionPhase phase = CaptureSessionPhase::Idle;
        uint8_t  nodeId = 2;
        uint16_t periodUs = 250;
        uint8_t  prePct = 0;
        uint8_t  postPct = 100;
        uint8_t  bufferKb = 32;
        uint8_t  triggerMode = 0;   // immediate — UI no longer exposes this
        uint8_t  triggerSlot = 0;
        int32_t  triggerThreshold = 0;
        // User-facing knob: total capture window in milliseconds. The state
        // machine derives periodUs and bufferKb from this on Trigger click,
        // bounded by per-node hardware limits in computeCaptureBounds().
        int      durationMs = 200;

        std::vector<size_t> watchIdxBySlot; // watches_ index per slot
        uint8_t slotsConfigured = 0;        // counter while sending SET_SLOT
        uint8_t lastSubSent = 0;            // for ack-matching, debug
        uint16_t samplesCaptured = 0;
        uint16_t triggerSampleIdx = 0;
        uint16_t bytesPerSample = 0;
        uint16_t bytesExpected = 0;
        std::vector<uint8_t> rawBytes;      // CAP_DATA accumulator
        std::vector<bool>    byteSeen;
        int retriesLeft = 0;                // gap re-requests budgeted per session
        double phaseDeadline = 0.0;        // when to retry/timeout
        double nextStatusAt = 0.0;          // next CAP_STATUS poll
        double startedAt = 0.0;
        double doneStatusAt = 0.0;          // wall time when DONE was first seen
        double cooldownUntil = 0.0;
        double lastDataAt = 0.0;            // wall time of last CAP_DATA frame
        double nextStallProbeAt = 0.0;      // earliest time to re-issue READ_CHUNK on stall
        uint16_t lastResumeOffset = 0;      // last offset we asked the MCU to resume from
        // READ_CHUNK download is paced in small windows instead of asking
        // the MCU to dump the whole capture buffer in one burst. This keeps
        // CAN load bounded and leaves airtime for live READ_MEM graph updates.
        bool     streamInFlight = false;
        uint16_t streamOffset = 0;
        uint16_t streamEndOffset = 0;
        uint16_t nextChunkOffset = 0;
        double   streamRequestAt = 0.0;
        double   nextChunkAt = 0.0;
        // Integrity verification (CAP_CRC). Pc-tool computes CRC32 over
        // rawBytes after every byte is seen; if it matches the MCU's
        // reply we're guaranteed end-to-end byte fidelity. On mismatch
        // we wipe rawBytes/byteSeen and re-pull the whole window —
        // bounded by crcAttemptsLeft to keep a corrupt session from
        // looping forever.
        uint8_t  crcAttemptsLeft = 0;
        double   crcRequestSentAt = 0.0;
        double   crcDeadline = 0.0;
        uint32_t lastCrcLocal = 0;
        uint32_t lastCrcRemote = 0;
        std::string status = "idle";
        std::string lastError;
        // Plot annotation: ranges (startRel, endRel) of past capture
        // windows. Each completed Triggered Capture pushes one entry.
        // Drawn as paired vertical lines on the plot's X axis. Persisted
        // in the scenario JSON and in CSV snapshot headers so save/load
        // and replay show the same markers as the live session did.
        struct CaptureWindow { double startRel; double endRel; };
        std::vector<CaptureWindow> windowsRel;
    };
    CaptureSession capture_;

    bool startCaptureSession();
    /* Internal: run the capture state machine for ONE specific node.
     * Picks the watches whose nodeId matches `targetNode`. Used both as
     * the leaf of startCaptureSession (single-node case) and as the
     * continuation when CooldownDone of one node chains to the next
     * pending node in pendingCaptureNodes_. */
    bool startNodeCapture(uint8_t targetNode);

    /* HTTP-driven Triggered Capture (TCP / Wi-Fi AP transports only).
     *
     * On the SLCAN/dongle path pc_tool drives the full CAP_REQ state
     * machine itself; the dongle is a direct CAN bus participant and
     * sees the same airtime as the MCUs. Over Wi-Fi/TCP we instead
     * tunnel a single HTTP GET /capture?... over SSH to the device,
     * and let app_dd run the entire CAN session locally and return the
     * raw byte buffer in one HTTPS body. That removes ~50× the
     * round-trip latency and stops the chunked READ_CHUNK loop from
     * sharing airtime with live READ_MEM polls.
     *
     * The worker thread is owned per-capture: when transport == Tcp or
     * WifiAp and the capture session is started, startNodeCapture spawns
     * `captureHttpThread_`, fills in captureHttpResult_ on completion
     * under `captureHttpMutex_`, and sets captureHttpDone_=true.
     * pollCaptureSession picks the result up on the next GUI tick and
     * either calls applyCapturedSamples() or fails the session.
     *
     * Restrictions:
     *   - Single in-flight worker per session. The Idle/CooldownDone
     *     gate in startCaptureSession already enforces serialisation,
     *     so we don't need an extra mutex around launching.
     *   - The worker MUST NOT touch watches_, GUI state, or call into
     *     ImGui -- it only marshals raw byte buffers and metadata
     *     through CaptureHttpResult. */
    struct CaptureHttpResult {
        std::vector<uint8_t> bytes;
        uint16_t samples = 0;
        uint16_t trigger_idx = 0;
        uint16_t period_us = 0;
        uint8_t  slot_count = 0;
        uint32_t bytes_per_sample = 0;
        uint32_t crc32 = 0;
        uint32_t elapsed_ms = 0;
        uint32_t fill_ms = 0;     // stage 1: MCU ring-buffer fill (from app_dd)
        uint32_t read_ms = 0;     // stage 2: motor->SoM CAN read (from app_dd)
        uint32_t stage3_ms = 0;   // stage 3: SoM->pc_tool HTTP fetch (measured here)
        int      http_status = 0;
        std::string error_stage;     // populated on failure
        std::string error_message;   // populated on failure
        bool     ok = false;         // true iff the body parsed AND headers parsed AND status 200
    };
    bool fetchCaptureViaHttp(const std::string& host,
                             const std::string& query_string,
                             CaptureHttpResult& out);
    /* Spawn the HTTP worker for the current capture session. The session
     * fields (watchIdxBySlot, nodeId, periodUs, durationMs etc.) MUST be
     * populated by the caller before calling this; the worker only reads
     * a snapshot. Returns false if no host can be derived from the
     * active transport, or if a worker is already running. */
    bool launchCaptureHttpWorker();
    /* Drain captureHttpDone_ in the GUI tick. Returns true if the worker
     * delivered a result this tick (success OR failure -- caller handles
     * the phase transition); false if still running or no worker armed. */
    bool drainCaptureHttpResult(double now);
    /* True iff the active transport routes us through app_dd's
     * /capture HTTP endpoint instead of the direct-CAN CAP_REQ state
     * machine. Centralised so the trigger / cooldown / chain logic can
     * branch on a single predicate. */
    bool captureUsesHttpPath() const;
    /* Build the GET /capture query string for the current session
     * (nodeId, durationMs, periodUs, prePct/postPct, bufferKb, and one
     * &slotN= entry per watchIdxBySlot). The full URL the worker
     * fetches is "http://127.0.0.2:10011/capture?<query>". */
    std::string buildCaptureHttpQuery() const;

    // HTTP capture worker state. Set by GUI tick (Idle), consumed by
    // pollCaptureSession when captureHttpDone_ flips true.
    std::thread             captureHttpThread_;
    std::atomic<bool>       captureHttpRunning_{false};
    std::atomic<bool>       captureHttpDone_{false};
    mutable std::mutex      captureHttpMutex_;
    CaptureHttpResult       captureHttpResult_;
    // Live byte-counter the worker increments mid-stream so the toolbar
    // status text can show "fetching capture: X.X KB / Y.Y KB". Now fed
    // by polling app_dd /capture/progress every ~250 ms; bytes_received
    // and total_bytes come straight from the device-side atomics so the
    // counter is honest about how much has actually moved over CAN, not
    // how much HTTP body has been read.
    std::atomic<uint64_t>   captureHttpBytesRx_{0};
    std::atomic<uint64_t>   captureHttpBytesExpected_{0};
    // Polled progress fields fed by /capture/progress on each tick.
    // captureHttpPhaseCode_ matches CapClient::Progress::Phase enum
    // (0=idle 1=setup 2=armed 3=sampling 4=transferring 5=done 6=error).
    // captureHttpPhaseName_ is the short string ("sampling", "transferring"
    // etc.) -- atomic pointer to a static string literal so the GUI tick
    // can read it cheaply without locking.
    std::atomic<uint8_t>    captureHttpPhaseCode_{0};
    std::atomic<uint16_t>   captureHttpSamples_{0};
    std::atomic<uint32_t>   captureHttpElapsedMs_{0};
    // Wall-clock at which the worker started. Used in pollCaptureSession
    // to print elapsed time alongside the byte counter.
    double                  captureHttpStartedAt_ = 0.0;

    /* HTTP-driven motor-config blob get/set (TCP / Wi-Fi-AP transports).
     * On SLCAN the existing unified READ/WRITE CAN frames work fine
     * (pc-tool is on the bus). Over Wi-Fi every CAN frame round-trips
     * through the TCP-CAN bridge — for a 164-byte blob that's 28 frames
     * read + 28 frames write, plus per-frame RTT — so we route the
     * whole operation through app_dd's /motor/config/{get,set} which
     * runs the file-transfer state machine locally over CAN and returns
     * the result in one JSON response. Symmetric with the /capture path.
     *
     * Lifecycle:
     *   sendMotorBlobGet / sendMotorBlobSet checks motorBlobUsesHttpPath
     *   and either spawns a worker thread (HTTP path) or sends CAN
     *   frames directly (SLCAN path). drainMotorBlobHttpResult, called
     *   from the GUI tick, joins the worker once done, splats the bytes
     *   into motorBlob_, and runs the same post-receive caches the
     *   CAN-arrival path runs (lastMotorTelemetry_.theta_*, motorConfig-
     *   Cache, etc.). One worker at a time; collisions become
     *   "operation already in flight" status. */
    enum class MotorBlobHttpKind : uint8_t {
        Get    = 0,
        Set    = 1,
        Save   = 2,
        Reset  = 3,   // /motor/config/reset: RESET->SAVE->read-back, returns blob
    };
    struct MotorBlobHttpResult {
        bool       ok = false;
        MotorBlobHttpKind kind = MotorBlobHttpKind::Get;
        uint16_t   bytes = 0;
        uint32_t   crc = 0;
        uint32_t   elapsed_ms = 0;
        std::vector<uint8_t> blob;     // 164 B on GET, empty on SET/SAVE
        std::string error_stage;
        std::string error_message;
    };
    // endpoint/tmoSec/kind let this same reader serve both /motor/config/get
    // (default) and /motor/config/reset (which also returns the 164-B blob).
    bool fetchMotorBlobGetHttp(const std::string& host,
                               MotorBlobHttpResult& out,
                               const char* endpoint = "/motor/config/get",
                               int tmoSec = 8,
                               MotorBlobHttpKind kind = MotorBlobHttpKind::Get);
    bool fetchMotorBlobSetHttp(const std::string& host,
                               const uint8_t blob[164],
                               uint32_t crc,
                               MotorBlobHttpResult& out);
    bool fetchMotorBlobSaveHttp(const std::string& host,
                                MotorBlobHttpResult& out);
    bool launchMotorBlobGetHttp();
    bool launchMotorBlobSetHttp(const uint8_t blob[164]);
    bool launchMotorBlobSaveHttp();
    bool launchMotorBlobResetHttp();
    bool drainMotorBlobHttpResult(double now);
    bool motorBlobUsesHttpPath() const;
    // Post-receive: fan a freshly assembled motorBlob_ into all
    // dependent caches (telemetry mirror, motorConfigCache, valid flag).
    // Shared between the CAN-arrival path and the HTTP-drain path so
    // they produce byte-identical downstream state.
    void applyMotorBlobReceived(uint16_t bytes, uint32_t crc,
                                const char* source);

    std::thread             motorBlobHttpThread_;
    std::atomic<bool>       motorBlobHttpRunning_{false};
    std::atomic<bool>       motorBlobHttpDone_{false};
    mutable std::mutex      motorBlobHttpMutex_;
    MotorBlobHttpResult     motorBlobHttpResult_;
    double                  motorBlobHttpStartedAt_ = 0.0;
    /* Two-step state for "SAVE then auto-verify": SAVE_ALL alone tells
     * the operator that the motor accepted the persist command, but the
     * only proof that flash actually changed is a follow-up /motor/
     * config/get. After a successful SAVE we set this flag, queue the
     * GET on the same worker channel, and on completion show the
     * "saved to flash, verified — field X = Y" message instead of the
     * ambiguous "SAVE_ALL ok" alone. Reset to false the moment we kick
     * the verify GET; the GET-handling branch reads it before clearing. */
    bool                    motorBlobPendingSaveVerify_ = false;
    uint32_t                motorBlobSaveAckCrc_ = 0u;  // motor's wire CRC echo

    // HTTP capture worker dedicated to the Calibration tab (separate from
    // captureHttp* which serves the Plots-tab session). Lives here, after
    // the CaptureHttpResult struct, so a calib capture and a Plots capture
    // can never collide.
    std::thread             calibCapThread_;
    std::atomic<bool>       calibCapRunning_{false};
    std::atomic<bool>       calibCapDone_{false};
    std::mutex              calibCapMutex_;
    CaptureHttpResult       calibCapResult_;
    CalibCapRun*            calibCapTarget_ = nullptr;  // which run is fetching

    /* Pending-node FIFO for multi-MCU sequential capture. User marks
     * variables (or plots them) across motor / RK / app_dd / ESP32; the
     * Trigger click queues every distinct node, captures the first
     * node, and on CooldownDone of that session pops the next node and
     * re-enters the state machine. Each node's data lands in the plot
     * via the existing applyCapturedSamples path. Empty when all
     * sessions have completed (or only one node was involved). */
    std::deque<uint8_t> pendingCaptureNodes_;
    void pollCaptureSession(double now);
    void handleCapFrame(const CanFrame& frame);
    void resetCaptureSession(const char* reason);
    void applyCapturedSamples();
    bool requestCaptureChunk(uint16_t byteOffset, uint16_t byteCount,
                             double now, const char* statusPrefix);
    // drawCaptureControl removed: trigger UI now lives inline in the plot
    // toolbar (Trigger icon + Cap, ms field). No multi-row panel anymore.
    bool isCapturePollSuppressed(const WatchVar& w) const;
    double estimateCaptureCooldownSec() const;
    bool loadSymbolsInto(SymbolSource& source);
    bool exportElfAndLoadInto(SymbolSource& source);
    bool resolveWatchConfig(const WatchConfig& watch, VariableSymbol& symbol, std::string& source) const;
    SymbolSource& ensureSourceForPath(const std::string& symbolsPath);
    ICanTransport* selectedTransport();
    bool isConnected() const;
    bool sendFrameThreadSafe(const CanFrame& frame);
    void connectSelectedTransport();
    void disconnectTransport();
    bool selectDiscoveredTcpTarget(bool isAp, bool markDirty);
    void scanDiscoveryAsync();   // broadcasts DDV2_DISCOVER, fills discovered_
    void scanWlanAsync();        // Windows WLAN, fills wlanScan_ for AP tab
    void connectWlanAsync(const std::string& ssid, const std::string& password,
                          bool autoOpenTcp = true);

    // Latest ANS_WIFI_STATUS (CAN id 0x1B00FF01) decoded from the bus. Shown
    // under the Remote Control buttons so the operator sees a visible ack
    // instead of having to flip to CAN Monitor + decode by hand.
    struct WifiStatusReply {
        bool   valid = false;
        uint8_t mode = 0;        // 0=OFF, 1=STA, 2=AP
        std::string ip;          // dotted-quad
        bool   ok = false;
        double receivedAt = 0.0; // ImGui::GetTime() seconds
    };
    WifiStatusReply lastWifiStatus_;

    // Per-node operating mode tracker. Updated from every CAN frame whose
    // src is a known node (motor/RK/main/ESP32): the M-bit in the
    // extended ID tells us whether the node currently runs the app or
    // the bootloader. Used by:
    //   - Remote Control to draw a green "APP" / orange "BOOT" indicator
    //     and the last reported firmware version per node.
    //   - pollWatchVariables() to *suppress* READ_MEM polls while the
    //     target is in BOOT (the bootloader has no READ_MEM handler, so
    //     polls would just rack up timeouts and throw errors during a
    //     flash). The suppression auto-clears once we see an APP-mode
    //     frame from that node, or after `kNodeModeStaleSec` of silence.
    struct NodeMode {
        bool   valid = false;
        uint8_t mod = 0xFFu;        // 0 = BOOT, 1 = APP, 0xFF = unknown
        double  modSeenAt = 0.0;    // wall-clock of last frame from this node
        bool    versionValid = false;
        std::string bootVersion;    // dotted-quad string, from ping_rsp [0..3]
        std::string appVersion;     // dotted-quad string, from ping_rsp [4..7]
        double  versionAt = 0.0;
        double  pingSentAt = 0.0;   // wall-clock of last ping we sent
        // Auto-ping scheduling: after Go-to-Boot/Go-to-App we want to
        // know the new mode without waiting for the operator to click
        // Ping. autoPingDueAt is the wall-clock at which updateIo will
        // emit a ping in the requested mode. Set/cleared by
        // sendNodeGoToBoot / sendNodeGoToApp / updateIo.
        double  autoPingDueAt = 0.0;
        bool    autoPingBootMode = false;
    };
    static constexpr size_t kNodeIdMax = 8;
    std::array<NodeMode, kNodeIdMax> nodeMode_{};
    void updateNodeModeFromFrame(const CanFrame& frame);
    bool nodeIsInBoot(uint8_t nodeId) const;
    bool nodeIsInApp(uint8_t nodeId) const;
    bool sendNodePing(uint8_t nodeId, bool bootMode);
    bool sendNodeGoToBoot(uint8_t nodeId);
    bool sendNodeGoToApp(uint8_t nodeId);
    void drawNodeStatePanel(uint8_t nodeId, const char* label);

    // Telemetry sniffed from mainPCB. Populated by handleFrame() whenever an
    // ANS_DIAG_STATUS / ANS_DIAG_TELEMETRY frame arrives, or when motor's
    // own ANS_FAULT/ANS_RPM/ANS_PFC_STATE are seen on the wire.
    struct DiagStatusReply {
        bool   valid = false;
        bool   diag_active = false;
        bool   motor_running = false;
        bool   rk_armed = false;
        bool   pfc_enabled = false;
        uint8_t fsm_state = 0;
        uint8_t hb_seq_echo = 0;
        uint16_t wdg_remaining_ms = 0;
        double receivedAt = 0.0;
    };
    DiagStatusReply lastDiagStatus_;

    struct MotorTelemetry {
        bool   valid = false;
        bool   motor_online = false;
        bool   rk_online = false;
        bool   motor_in_boot = false;
        bool   rk_in_boot = false;
        uint8_t mode = 0;
        float  rpm = 0.0f;
        float  voltage_dc = 0.0f;       // from MAIN_ANS_DIAG_TELEMETRY (deci-volts)
        float  voltage_q = 0.0f;        // from motor MTR_ANS_UQ_V
        float  power_W = 0.0f;          // from motor MTR_ANS_POWER_W
        float  motor_temp = 0.0f;
        float  vt_temp = 0.0f;
        uint32_t fault_mask = 0;
        bool   theta_valid = false;
        float  theta_offset_rad = 0.0f;
        uint8_t theta_status = 0;
        uint8_t theta_flags = 0;
        uint8_t theta_driver_mode = 0;
        double theta_receivedAt = 0.0;
        // Per-Hall-sector LUT (firmware >= 2.1.6). Populated bin-by-bin
        // through GET_SECTOR queries; valid flags reset when host requests
        // a re-read so stale values don't get displayed across sessions.
        bool   theta_sector_valid [6] = {false, false, false, false, false, false};
        float  theta_sector_lut_rad [6] = {0,0,0,0,0,0};
        double theta_sector_receivedAt [6] = {0,0,0,0,0,0};
        double receivedAt = 0.0;

        // -- Theta-calibration live telemetry (MTR_ANS_THETA_CAL 0x0AD).
        // Pushed by the motor every round-robin cycle while -- and after --
        // a calibration sweep runs. cal_phase: 0 IDLE,1 RUNNING,2 DONE_OK,
        // 3 DONE_FAIL. cal_sector_valid bit S-1 marks Hall sector S's LUT
        // bin as accumulated enough samples (drives the live polar fill).
        bool    theta_cal_valid       = false;
        uint8_t cal_phase             = 0;
        uint8_t cal_progress_pct      = 0;
        uint8_t cal_sector_valid      = 0;   // bits 0..5 = Hall sectors 1..6
        bool    cal_global_ok         = false;
        float   cal_theta_offset_rad  = 0.0f;
        double  cal_receivedAt        = 0.0;
    };
    MotorTelemetry lastMotorTelemetry_;

    struct MotorProfilerView {
        bool    valid = false;
        uint8_t cfg_status = 0;
        uint8_t state = 0;
        uint8_t active_test = 0;
        uint8_t progress = 0;
        uint8_t error = 0;
        uint8_t result_sel = 0;
        float   results[7] = {0,0,0,0,0,0,0};
        bool    result_valid[7] = {false,false,false,false,false,false,false};
        double  receivedAt = 0.0;
    };
    MotorProfilerView motorProfiler_;

    // PC-tool side diag state.
    bool diagModeOn_ = false;       // user toggle; we send DIAG_MODE_SET on edge
    float motorSpeedSlider_ = 30.0f;  // 0..100 percent
    bool motorSpeedSendPending_ = false;
    double nextMotorSpeedSendAt_ = 0.0;
    bool motorProxyDesiredRun_ = false;
    double nextMotorProxyKeepaliveAt_ = 0.0;
    float motorThetaAlignCurrent_ = 0.30f;
    float motorThetaManualRad_ = 6.2f;
    bool motorThetaManualDirty_ = false;
    bool motorThetaSaveAfterCal_ = false;
    bool motorThetaAutoPoll_ = true;
    double nextMotorThetaPollAt_ = 0.0;
    std::string motorThetaTxStatus_;
    /* Pending sub-command queue. Each entry is (action_or_subcmd, bin):
     *   - For subcmd 0x00 (THETA_OFFSET): first = action enum (kMotorThetaGet,
     *     kMotorThetaGetSector, kMotorThetaSet, ...). bin matters only for
     *     kMotorThetaGetSector; otherwise 0xFF.
     *   - For subcmd 0x01 (THETA_SECTOR): first = kMotorCfgThetaSector (0x01),
     *     bin = the sector index 0..5 we wrote.
     * The motor sends one ANS_CONFIG per request in send order; response
     * handler pops the front to know how to route. */
    std::deque<std::pair<uint8_t, uint8_t>> motorThetaPending_;

    /* Whole-blob receive cache.
     *
     * Migrated 2026-05-13 to the unified READ protocol: handleFrame
     * accumulates DATA_MMSG (0x0D5) bytes into motorBlob_, validates
     * STM32-HW-CRC32 on FILE_FINISH (0x0DF), and flips motorBlobValid_
     * true on match. WRITE side (BLOB SET_CHUNK on subcmd 0x03) still
     * uses the old per-chunk path -- Phase 3 of the unified migration.
     *
     * motorReadActiveMmsg_ tracks the in-flight READ_HEADER_MMSG's
     * idx + byte size so DATA_MMSG frames know where to splat their
     * 7-byte payloads inside motorBlob_. */
    uint8_t  motorBlob_[164] = {};
    uint64_t motorBlobReceivedMask_ = 0u;
    bool     motorBlobValid_ = false;
    double   motorBlobReceivedAt_ = 0.0;
    uint8_t  motorBlobLastSetResult_ = 0xFFu;   /* 0xFF = no SET attempted yet */
    uint32_t motorBlobLastSetCrcEcho_ = 0u;
    double   motorBlobLastSetResultAt_ = 0.0;

    /* After a user SET, motorBlob_ is the source of truth: pc_tool wrote
     * the new bytes to motor RAM, and we want subsequent auto-poll BLOB
     * GET *responses* to NOT clobber the local cache with the flash bytes
     * (which still hold the pre-Save value). The unified READ on motor
     * side currently reads raw flash sector 11, so without this guard
     * every Set visibly reverts after one auto-poll tick. Cleared on:
     *   - successful PARAM SAVE_ALL (flash now matches RAM),
     *   - explicit "Read All" button (user asked for fresh flash view),
     *   - timeout (5 minutes -- avoids stale lockout if user forgets to Save). */
    double   motorBlobAuthoritativeUntil_ = 0.0;

    /* Unified READ in-flight tracker. Set by READ_HEADER_FILE_OK
     * (expected_size), advanced by READ_HEADER_MMSG (current mmsg
     * idx + size in bytes), consumed by READ_DATA_MMSG (computes byte
     * offset = mmsg_idx*MMSG_MAX + msg_idx*MSG_DATA), validated on
     * READ_FILE_FINISH (host-side CRC32 over assembled motorBlob_
     * vs motor-reported fileCrc). */
    uint16_t motorReadExpectedSize_ = 0;
    uint16_t motorReadCurMmsgIdx_   = 0;
    uint16_t motorReadCurMmsgSize_  = 0;
    bool     motorReadInFlight_     = false;

    /* Inline edit buffers for the LUT table. Slot 0 = global offset,
     * slots 1..6 = sector 1..6. Initialized to current measured value
     * each time a fresh response arrives, but the user's in-progress
     * edits aren't stomped (motorThetaSlotDirty_ marks "user is editing,
     * don't auto-update from incoming responses"). */
    float motorThetaSlotEdit_  [7] = {0,0,0,0,0,0,0};

    // --- Motor Config tab (param-table viewer) ----------------------
    // Cache populated by handleFrame() when MTR_ANS_CONFIG (0x0AB)
    // sub-cmd 0x02 (PARAM) responses arrive. The motor reports value
    // in 4 raw bytes -- we keep them as u32 and reinterpret per the
    // hardcoded param type when rendering.
    struct MotorConfigParam {
        uint16_t    id;
        const char* name;
        uint8_t     group;     // 1=PI, 2=PROFILER, 4=DIAG
        uint8_t     type;      // 0=f32, 1=u32
        bool        editable;  // false for read-only DIAG params
    };
    struct MotorConfigEntry {
        uint32_t rawValue = 0;
        bool     hasValue = false;
        double   lastSeenTs = 0.0;
        uint8_t  lastStatus = 0;
    };
    void drawMotorConfigParamRow(const MotorConfigParam& def, MotorConfigEntry& e);
    /* Global "Set": splice EVERY editable, blob-resident field from the
     * edit buffers onto the last-read baseline blob, recompute the
     * payload CRC32, and send the whole 164-byte blob once (so the FW
     * verifies CRC over the entire config). Replaces the per-row Set.
     * Requires a prior Get (motorBlobValid_). Definition in MainUi.cpp. */
    bool applyMotorConfigBlobSetAll();
    bool requestMotorConfigLiveRead();
    std::map<uint16_t, MotorConfigEntry> motorConfigCache_;
    std::map<uint16_t, std::string> motorConfigEditBuffer_;  // per-row text buffer
    // Short label of the blob op currently in flight ("Reading config",
    // "Writing config to RAM", "Saving to flash") — shown next to the
    // spinner in the Motor Config tab progress line.
    std::string motorConfigOpLabel_;
    // Legacy guard flag from the old one-click Set+Save flow. Kept only so
    // old async failure paths can clear it harmlessly; the UI no longer sets
    // it. Motor Config Set is RAM/live only, Save All is the explicit
    // RAM->flash persist step.
    bool motorConfigSetThenSave_ = false;

    // Motor auto-ID / PI tune panel (CFG subcmd 0x04).
    float motorProfilerTestCurrentA_ = 0.5f;
    float motorProfilerMaxVoltagePct_ = 5.0f;
    float motorProfilerPulseMs_ = 20.0f;
    float motorProfilerSettleMs_ = 20.0f;
    int   motorProfilerRepeatCount_ = 4;
    float motorProfilerBandwidthHz_ = 300.0f;
    std::string motorProfilerStatus_;
    bool   motorProfilerDoneFetchIssued_ = false;
    double motorProfilerNextStatusAt_ = 0.0;

    // Background enumeration: walk the known param list, send one GET
    // every ~25 ms so we don't congest the bus. Loops back at end.
    //
    // After unification onto motorBlob_, motorConfigEnumIdx_ is kept only
    // for the legacy "Read All" status text; motorConfigDiagIdx_ drives
    // the DIAG round-robin (ids 30..36), and motorConfigNextBlobGetAt_
    // schedules the periodic BLOB GET that feeds both tabs.
    int    motorConfigEnumIdx_ = 0;
    int    motorConfigDiagIdx_ = 0;
    double motorConfigNextGetAt_ = 0.0;
    double motorConfigNextBlobGetAt_ = 0.0;
    bool   motorConfigAutoPoll_ = true;
    double motorConfigLastReadAllStartAt_ = 0.0;
    std::string motorConfigStatus_;

    // Send helpers / handlers
    bool sendMotorConfigParam(uint8_t action, uint16_t id, uint32_t value_bits);
    bool sendMotorProfilerCommand(uint8_t action, uint8_t arg, uint32_t value_bits);
    bool sendMotorProfilerLimits();
    void requestMotorProfilerResults();
    void drawMotorProfilerPanel(bool canSend);
    void handleMotorConfigResponse(uint16_t id, uint8_t status, uint32_t value_bits);
    /* After a completed BLOB GET, splat every blob-resident config field
     * into motorConfigCache_ under its param-table id. Single source of
     * truth: the same staged buffer powers the calibration tab and the
     * Motor Config tab, so there is no longer a parallel PARAM-poll
     * pathway for persistent fields. DIAG ids stay PARAM-polled. */
    void populateMotorConfigCacheFromBlob();
    /* STM32 hardware CRC peripheral default replay. Used to verify the
     * unified READ protocol's FILE_FINISH file CRC against the
     * locally-assembled motorBlob_. Matches motor FW propCanGetCrc
     * and app_dd's simple_crc32_stm32hw byte-for-byte. Definition in
     * MainUi.cpp. */
    static uint32_t motorStm32HwCrc32(const uint8_t* data, std::size_t bytes);
    bool  motorThetaSlotDirty_ [7] = {false,false,false,false,false,false,false};
    bool pfcSafetyLock_ = true;
    uint8_t hbSeq_ = 0;
    double nextHeartbeatAt_ = 0.0;

    // Tone / melody playback state. -1 = idle, otherwise index into the
    // hard-coded library in MainUi.cpp (Mario / Tetris / Imperial / Beep).
    int   activeMelodyIdx_     = -1;
    int   pendingMelodyIdx_    = -1;
    size_t melodyNoteIdx_      = 0;
    double nextMelodyNoteAt_   = 0.0;
    int   melodyAmpPct_        = 20;       // 0..25, firmware caps at 25
    int   melodyTransposeOct_  = 0;        // -2..+2 octaves
    float melodyDurationScale_ = 1.0f;     // 0.5x..2.0x tempo control
    int   melodyInternoteGapMs_ = 35;

    // Throttle Remote Control mode-switch clicks so the operator can't queue
    // up multiple ForceMode() calls before the SOM has finished the previous
    // one. ForceMode takes ~3 s for AP and ~12 s for STA; spamming would
    // trample on in-flight TrySTA/StartAP and confuse the bus.
    double remoteCmdCooldownUntil_ = 0.0;

    // Deferred TCP disconnect after a route-breaking SET_WIFI_MODE. Closing
    // the socket immediately after send() can race with the kernel TX queue —
    // FIN/RST goes out before the JSON line is fully on the wire and bridge
    // never sees it. We arm this and let updateIo() do the actual close once
    // ~500 ms have passed, by which time the Wi-Fi stack has the frame.
    double disconnectScheduledAt_ = 0.0;

    // Tab visibility / focus state for the dockable tab windows. Each tab is
    // its own ImGui::Begin in the dockspace and can be torn out into a native
    // OS window. The tab strip in the title-bar area now just toggles these
    // bools and pushes focus to the corresponding window.
    static constexpr int kNumTabs = 9;   // incl. Calibration + Load Stand tabs
    bool tabOpen_[kNumTabs]   = {true, true, true, true, true, true, true, true, true};
    bool tabFocusReq_[kNumTabs] = {false, false, false, false, false, false, false, false, false};
    bool firstDockLayout_ = true;
    int focusConnectionStartupFrames_ = 12;
    int activeTab_ = 0; // kept only because old tab strip logic referenced it
    void recordBusBits(const CanFrame& f);
    void drawStatusBar(float height);
    int currentNominalBitrate() const;
    void markScenarioDirty(const char* reason = nullptr);
    void maybeSaveScenario();
    std::string scenarioPathFor(const std::string& path) const;
    void resetTransportRuntimeState();

    TransportKind transportKind_ = TransportKind::Slcan;
    SlcanTransport slcan_;
    TcpCanTransport tcp_;

    // Loading-stand brake (STM32) telemetry over the Pi serial->TCP bridge.
    // Connection/control UI lives on the "Load Stand" tab; the torque samples
    // are fed into these buffers (shared time base) and drawn on the MAIN
    // Plots graph so the existing ruler / autoscale / signal logger apply.
    LoadStandClient loadStand_;
    char  lsHost_[64]     = "192.168.0.104";
    int   lsPort_         = 5555;
    float lsSetNm_        = 2.0f;
    char  lsCmd_[64]      = "";
    std::vector<double> standXs_, standTorque_, standSet_, standCur_;
    size_t standLastSamples_   = 0;
    bool   standPlotTorque_    = true;
    bool   standPlotSetpoint_  = false;
    bool   standPlotCurrent_   = false;
    void   pumpLoadStand();      // append new stand samples each frame
    ICanTransport* activeTransport_ = nullptr;
    std::mutex transportMutex_;

    CanTraceBuffer trace_{10000};
    CanLogger canLogger_;
    SignalLogger signalLogger_;
    CanSniffer sniffer_;   // passive, default-off (env DRIVESCOPE_SNIFF_JSONL)
    DebugProtocol debug_;
    CanBootloaderFlasher bootFlasher_;
    SlcanParser clock_;
    ElfSymbolLoader symbolLoader_;

    std::vector<SymbolSource> sources_;
    std::vector<WatchVar> watches_;
    std::unordered_map<uint32_t, size_t> pending_;
    std::unordered_map<uint32_t, double> pendingSentAt_;
    uint8_t nextSeq_ = 1;
    size_t pollCursor_ = 0;
    // Phase C STREAM_* state. STREAM is currently motor-only. Firmware has
    // 12 slots, but live plotting over Wi-Fi is capped to 6 active variables.
    // RK and ESP32 keep using READ_MEM polling via the legacy path.
    // motorStreamSlotToWatch_[slot] == index into watches_, or -1 if free.
    static constexpr size_t kMaxLiveStreamWatches = 6;
    std::array<int, 12> motorStreamSlotToWatch_ {{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}};
    double lastStreamHeartbeatAt_ = 0.0;
    uint8_t streamAddSeqNext_ = 0;
    // STREAM_VALUE → watch lookup needs a (seq, nodeId) for the add_rsp
    // matching. We track outstanding ADD requests by seq → watch index.
    std::unordered_map<uint8_t, size_t> streamAddPending_;

    void streamTick(double now);
    void sendStreamAddForWatch(size_t watchIdx, double now);
    void sendStreamRemoveForWatch(size_t watchIdx, double now);
    void sendStreamStatusReq(double now);
    void handleStreamAddRsp(const StreamSubscribeResponse& rsp, double now);
    void handleStreamValue(const StreamValueFrame& v, double now);
    void handleStreamStatusRsp(const StreamStatusReply& r, double now);
    void sendStreamHeartbeat(double now);
    size_t activeOrPendingStreamWatchCount() const;
    // Last STREAM_STATUS_RSP from the motor — proof its stream engine answers;
    // a fresh one means the command path is alive again.
    StreamStatusReply lastStreamStatus_{};
    double            lastStreamStatusAt_ = 0.0;
    bool streamSupportedForNode(uint8_t nodeId) const {
        return nodeId == kCanAddrMotor;  // Phase B/C: motor only
    }

    std::array<char, 64> comPort_{"COM3"};
    std::array<char, 32> slcanNominal_{"S6"};
    std::array<char, 32> slcanData_{"Y2"};
    int slcanBaud_ = 8000000;
    bool slcanSilent_ = false;

    std::array<char, 128> tcpHost_{"192.168.0.103"};
    int tcpPort_ = 45333;
    int tcpBusBitrate_ = 500000;
    std::array<char, 64> wifiApSsid_{"Kitchen_Machine_<UUID>"};
    std::array<char, 128> wifiApHost_{"192.168.1.0"};
    int wifiApPort_ = 45333;
    int wifiApBusBitrate_ = 500000;

    std::array<char, 260> motorFirmwarePath_{"../INVERTER/APP_MOTOR.bin"};
    std::array<char, 260> rkFirmwarePath_{"../RK/APP_RK.bin"};
    std::array<char, 260> esp32FirmwareDir_{"../SSD202D/app/firmware/esp32"};
    std::array<char, 260> mainAppScriptPath_{};
    std::array<char, 260> mainResourcesScriptPath_{};
    std::array<char, 260> recoveryAppDir_{};
    std::array<char, 260> recoveryOtaPath_{};
    bool recoveryAppDirInitialized_ = false;
    std::future<std::string> mainDeployFuture_;
    std::shared_ptr<DeployProgressState> mainDeployProgress_;
    bool mainDeployRunning_ = false;
    bool preserveTransportDuringDeploy_ = false;
    bool disconnectAfterMainDeploy_ = false;
    std::string mainDeployStatus_;

    // One-button service-update connect. Replaces the old Scan / Join AP /
    // Service AP triplet — the future drives the whole sequence:
    //   1. If a CAN transport is open, send SET_WIFI_MODE=AP so the device
    //      brings up its Kitchen_Machine_* hotspot.
    //   2. Scan and WlanConnect to the strongest Kitchen_Machine_* SSID
    //      with the baked-in bork2025 password.
    //   3. SSH to the device and killall app_dd / httpd / otaunpack /
    //      xor_file so nothing holds /app open or starves the CPU during
    //      the SFTP upload that the operator will run next.
    // After the future succeeds, updateIo() leaves DriveScope disconnected:
    // service mode killed app_dd, so there is no live CAN bridge until the
    // operator presses Run App, resets, or starts the device manually.
    void startServiceConnect();
    std::future<std::string> serviceConnectFuture_;
    bool serviceConnectRunning_ = false;
    std::string serviceConnectStatus_;
    // Worker → UI status text. Shared so the worker thread can update it
    // mid-flight without touching MainUi members directly.
    std::shared_ptr<std::string>     serviceConnectStageText_;
    std::shared_ptr<std::mutex>      serviceConnectStageMutex_;

    // UDP-discovered devices populating the WiFi AP dropdown.
    // Each entry comes from the device's discovery_service.cpp reply.
    struct DiscoveredDevice {
        std::string name;     // SSID-style name, e.g. "Kitchen_Machine_4BB7"
        std::string ip;       // current STA or AP IP
        std::string mode;     // "STA" / "AP" / "OFF"
        int         tcpPort = 45333;
    };
    std::vector<DiscoveredDevice> discovered_;
    int  discoveredSelected_ = -1;
    std::string discoverStatus_;  // "scanning..." / "found N devices" / etc.
    double discoverNextScanAt_ = 0.0;
    bool discoverScanning_ = false;

    // Windows WLAN scan state for the AP tab. The AP tab can NOT use UDP
    // discovery to find devices because the laptop is not yet on the device's
    // hotspot — there's no IP path. Instead we enumerate visible Wi-Fi SSIDs
    // matching "Kitchen_Machine_*", let the user pick one, then WlanConnect
    // with the baked-in password. After association, jump straight to TCP
    // 192.168.1.1:45333 (the device-side AP IP is hardcoded by hostapd).
    struct WlanEntry {
        std::string ssid;
        int  signal_pct = 0;
        bool secured = true;
        bool connected = false;
    };
    std::vector<WlanEntry> wlanScan_;
    int  wlanSelected_ = -1;
    std::string wlanStatus_;        // "scanning..." / "found N" / "no Wi-Fi adapters" / etc.
    double wlanNextScanAt_ = 0.0;
    bool wlanScanning_ = false;
    // Tracks an in-flight WlanConnect; when the OS reports the matching SSID
    // associated, we set wifiApHost_/Port_ and auto-press Connect.
    bool   wlanConnecting_ = false;
    double wlanConnectStartAt_ = 0.0;
    std::string wlanConnectTarget_;
    bool   wlanAutoOpenTcpPending_ = false;

    std::array<char, 64> plotFilter_{""};

    bool monitorPaused_ = false;
    bool monitorGroupByID_ = false;

    std::vector<CanFilterRule> filterRules_;
    bool monitorShowFilters_ = true;

    struct CellSelKey {
        bool valid = false;
        bool grouped = false;
        double ts = 0.0;
        uint32_t id = 0;
        bool ext = false;
        int col = -1;
    };
    CellSelKey monitorSelKey_;
    std::string monitorSelText_;
    bool monitorTextView_ = false;
    std::unordered_set<std::string> legendHidden_;
    bool     monitorPinnedValid_ = false;  // a frame is shown in the right-side decode panel
    CanFrame monitorPinnedFrame_;
    float    varsSplitX_ = 720.0f;
    float    monitorSplitX_ = -1.0f;  // <0 => initialise on first draw to ~70% of width
    float    plotSplitX_ = 300.0f;

    std::deque<std::pair<double, uint32_t>> rateBucket_;
    std::deque<std::pair<double, uint32_t>> transportBucket_;
    size_t totalFrames_ = 0;
    float canLoadEma_ = 0.0f;
    float netKbpsEma_ = 0.0f;
    float fpsEma_ = 0.0f;

    float uiScale_ = 1.0f;
    double plotYMin_ = 0.0;
    double plotYMax_ = 1.0;
    bool   plotYManual_ = false;
    std::array<char, 260> snapshotPath_{"drivescope_snapshot.csv"};
    std::array<char, 260> plotSnapshotPath_{"drivescope_plot.csv"};
    std::array<char, 260> varsConfigPath_{"drivescope_vars.json"};
    std::string snapshotStatus_;
    std::string plotSnapshotStatus_;
    std::string varsConfigStatus_;
    std::string lastFileMsg_;
    bool replayMode_ = false;

    std::string scenarioPath_;
    std::string scenarioStatus_;
    std::string scenarioDirtyReason_;
    bool scenarioDirty_ = false;
    double scenarioSaveDue_ = 0.0;

    bool plotPaused_ = false;
    bool plotAutoscale_ = true;
    bool plotCrosshair_ = true;
    bool plotRulerMode_ = false;
    struct PlotRuler {
        double x1 = 0.0;
        double y1 = 0.0;
        double x2 = 0.0;
        double y2 = 0.0;
    };
    std::vector<PlotRuler> plotRulers_;
    bool plotRulerDrawing_ = false;
    PlotRuler plotRulerDraft_{};
    bool plotFollowLive_ = true;
    // When the active transport closes (disconnect or transport switch) we
    // freeze the X-axis "now" so the user can still see the last data instead
    // of watching it scroll off into empty future. plotFrozenAt_ holds the
    // relative time at which the freeze started; the next reconnect clears
    // the flag and time resumes from real wall-clock again.
    bool   plotFrozen_   = false;
    double plotFrozenAt_ = 0.0;
    bool pendingFitAll_ = false;
    bool plotXManual_ = false;
    double plotXMin_ = 0.0;
    double plotXMax_ = 0.0;
    double plotTimeOrigin_ = 0.0;
    float plotWindowSec_ = 30.0f;
    float plotHistorySec_ = 3600.0f;

    std::array<char, 260> canLogPath_{"drivescope_can.csv"};
    std::array<char, 260> signalLogPath_{"drivescope_signals.csv"};
};

} // namespace drivescope
