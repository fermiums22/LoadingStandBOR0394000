// CalibrationTab.cpp -- the DriveScope Calibration tab.
//
// Implements docs/calibration-tab-redesign.md: a dedicated dockable tab with
// a left rail of sub-sections, a compact live motor-config strip, and four
// test panels (Theta / Machine params / PI tuning / Inertia). The FOC theta
// calibration UI (previously buried in Remote Control's drawMotorTab) and the
// motor profiler panel (previously a CollapsingHeader in drawMotorConfig) are
// re-homed here. No new transport: every write goes through the existing
// motorBlob_ / applyMotorConfigBlobSet / applyThetaSlotEdit path; the profiler
// reuses sendMotorProfilerCommand / sendMotorProfilerLimits.
//
// All drawCalib* functions are members of MainUi -- the codebase already
// splits MainUi across translation units; this is one more.

#include "ui/MainUi.h"
#include "ui/CalibFft.h"

#include "imgui.h"
#include "imgui_internal.h"
#include "implot.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace drivescope {

namespace {

// --------------------------------------------------------------------------
// Local re-implementations of the small name/offset helpers that live as
// file-static functions in MainUi.cpp (a separate TU cannot see them). Kept
// byte-identical to the originals.
// --------------------------------------------------------------------------
const char* calibFsmStateName(uint8_t s)
{
    switch (s) {
        case 0: return "WARNING"; case 1: return "STOP"; case 2: return "PAUSE";
        case 3: return "RUN";     case 4: return "PARKING"; case 5: return "PARK";
        default: return "?";
    }
}

const char* calibThetaStatusName(uint8_t s)
{
    switch (s) {
        case 0: return "OK";        case 1: return "BAD_ARG";  case 2: return "BUSY";
        case 3: return "NOT_READY"; case 4: return "STORAGE_EMPTY";
        case 5: return "STORAGE_FAIL"; case 6: return "FAULT"; case 7: return "RUNNING";
        default: return "?";
    }
}

const char* calibProfilerStateName(uint8_t s)
{
    switch (s) {
        case 0: return "IDLE"; case 1: return "RUNNING"; case 2: return "DONE";
        case 3: return "ERROR"; case 4: return "ABORTED"; default: return "?";
    }
}

const char* calibProfilerTestName(uint8_t t)
{
    switch (t) { case 0: return "NONE"; case 1: return "XX"; case 2: return "KZ";
        default: return "?"; }
}

const char* calibProfilerErrorName(uint8_t e)
{
    switch (e) {
        case 0: return "NONE"; case 1: return "BUSY"; case 2: return "NOT_READY";
        case 3: return "FAULT"; case 4: return "BAD_ARG"; case 5: return "MOVED";
        case 6: return "SIGNAL"; case 7: return "RANGE"; default: return "?";
    }
}

// Param-table id -> motorBlob_ byte offset. Mirrors motorConfigBlobOffsetForId
// in MainUi.cpp (constants: payload starts at 16, payload-crc at 12).
int calibBlobOffsetForId(uint16_t id)
{
    constexpr int P = 16;          // kMotorCfgBlobPayloadOffset
    switch (id) {
        case 0:  return P + 0;
        case 1:  return P + 4 + 0 * 4;
        case 2:  return P + 4 + 1 * 4;
        case 3:  return P + 4 + 2 * 4;
        case 4:  return P + 4 + 3 * 4;
        case 5:  return P + 4 + 4 * 4;
        case 6:  return P + 4 + 5 * 4;
        case 10: return P + 28;
        case 11: return P + 32;
        case 12: return P + 36;
        case 13: return P + 40;
        case 14: return P + 44;
        case 15: return P + 48;
        case 20: return P + 52;
        case 21: return P + 56;
        case 22: return P + 60;
        case 23: return P + 64;
        case 24: return P + 68;
        case 25: return P + 72;
        case 26: return P + 76;
        case 27: return P + 80;
        case 37: return 4;         // cfg.fw_ver (header)
        case 38: return 12;        // cfg.crc (header, kMotorBlobOffPayloadCrc)
        case 50: return P + 84;    // pfc.enabled
        default: return -1;
    }
}

ImVec4 calibPillColor(CalibPill p)
{
    switch (p) {
        case CalibPill::None:    return ImVec4(0.55f, 0.55f, 0.58f, 1.0f);
        case CalibPill::Running: return ImVec4(0.95f, 0.75f, 0.25f, 1.0f);
        case CalibPill::Pass:    return ImVec4(0.45f, 0.85f, 0.45f, 1.0f);
        case CalibPill::Fail:    return ImVec4(0.95f, 0.40f, 0.40f, 1.0f);
        case CalibPill::Stale:   return ImVec4(0.80f, 0.70f, 0.40f, 1.0f);
    }
    return ImVec4(0.55f, 0.55f, 0.58f, 1.0f);
}

const char* calibPillLabel(CalibPill p)
{
    switch (p) {
        case CalibPill::None:    return "--";
        case CalibPill::Running: return "RUN";
        case CalibPill::Pass:    return "PASS";
        case CalibPill::Fail:    return "FAIL";
        case CalibPill::Stale:   return "STALE";
    }
    return "--";
}

// Draw a small rounded pill at the current cursor.
void drawPill(CalibPill p)
{
    const ImVec4 c = calibPillColor(p);
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(c.x, c.y, c.z, 0.30f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(c.x, c.y, c.z, 0.30f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(c.x, c.y, c.z, 0.30f));
    ImGui::PushStyleColor(ImGuiCol_Text, c);
    ImGui::SmallButton(calibPillLabel(p));
    ImGui::PopStyleColor(4);
}

float calibPlotHeight(float width, float minH = 300.0f, float maxH = 430.0f)
{
    return std::clamp(width * 0.72f, minH, maxH);
}

void calibFitPlotOnDataChange(const char* stableId, int dataVersion)
{
    ImGuiStorage* storage = ImGui::GetStateStorage();
    const ImGuiID id = ImGui::GetID(stableId);
    const int prev = storage->GetInt(id, -1);
    if (prev != dataVersion) {
        ImPlot::SetNextAxesToFit();
        storage->SetInt(id, dataVersion);
    }
}

struct CalibPlotInputScope {
    ImPlotInputMap saved;
    CalibPlotInputScope() : saved(ImPlot::GetInputMap())
    {
        ImPlotInputMap fresh;
        ImPlot::GetInputMap() = fresh;
    }
    ~CalibPlotInputScope()
    {
        ImPlot::GetInputMap() = saved;
    }
};

} // namespace

// --------------------------------------------------------------------------
// Calibration sections dispatch table (spec §2.2). The label list is at
// namespace scope; the member-function pointers are resolved inside the
// MainUi methods below (a namespace-scope table cannot name private members).
// Each section carries a short subtitle so the navigation rail reads
// unambiguously as "pick a test" rather than a cramped list.
// --------------------------------------------------------------------------
struct CalibSectionLabel {
    const char* title;
    const char* subtitle;
};
static const CalibSectionLabel kCalibSectionLabels[] = {
    { "Test 1",            "Theta calibration"          },
    { "Test 2",            "Machine params (Rs/Ld/Lq)"  },
    { "Test 3",            "PI tuning + step response"   },
    { "Test 4",            "Inertia (J) + friction"      },
    { "Config (full)",     "raw config editor"          },
};
static constexpr int kCalibSectionCount =
    sizeof(kCalibSectionLabels) / sizeof(kCalibSectionLabels[0]);

// ==========================================================================
// Shared gating + helpers (spec §2.3, §5).
// ==========================================================================
bool MainUi::calibPreconditions(CalibGate& g)
{
    g.canSend = activeTransport_ && activeTransport_->isOpen();
    g.inDiag  = lastDiagStatus_.valid ? lastDiagStatus_.diag_active : diagModeOn_;
    const uint8_t fsm = lastDiagStatus_.fsm_state;
    const bool fsmBusy = (fsm == 3 /* RUN */) || (fsm == 4 /* PARKING */);
    const bool profBusy = (motorProfiler_.state == 1 /* RUNNING */);
    g.motorIdle = !fsmBusy && !profBusy;
    g.faultMask = lastMotorTelemetry_.fault_mask;
    return g.canSend;
}

float MainUi::calibBlobValueF32(uint16_t id)
{
    if (!motorBlobValid_) return 0.0f;
    const int off = calibBlobOffsetForId(id);
    if (off < 0 || off + 4 > static_cast<int>(sizeof(motorBlob_))) return 0.0f;
    float v = 0.0f;
    std::memcpy(&v, &motorBlob_[off], sizeof(v));
    return v;
}

void MainUi::calibSyncChip(const CalibFieldSync& sync)
{
    ImVec4 col;  const char* label;
    switch (sync.state) {
        case CalibSyncState::Synced: col = ImVec4(0.85f,0.85f,0.85f,1.0f); label = "ok";     break;
        case CalibSyncState::Edited: col = ImVec4(0.40f,0.78f,0.95f,1.0f); label = "edited"; break;
        case CalibSyncState::Sent:   col = ImVec4(0.95f,0.75f,0.25f,1.0f); label = "sent";   break;
        case CalibSyncState::Acked:  col = ImVec4(0.45f,0.85f,0.45f,1.0f); label = "ACK";    break;
        case CalibSyncState::Error:  col = ImVec4(0.95f,0.40f,0.40f,1.0f); label = "err";    break;
        default: col = ImVec4(0.85f,0.85f,0.85f,1.0f); label = "ok"; break;
    }
    ImGui::TextColored(col, "%s", label);
}

// Drive per-field sync chips: a fresh motorBlob_ value matching the sent value
// promotes Sent -> Acked; the Acked green flash decays to Synced after ~1 s;
// a Sent that never matched within 700 ms becomes Error (timeout).
void MainUi::calibRefreshSyncFromBlob()
{
    const double now = ImGui::GetTime();
    for (auto& kv : calibSync_) {
        CalibFieldSync& s = kv.second;
        if (s.state == CalibSyncState::Sent) {
            const float live = calibBlobValueF32(kv.first);
            if (std::fabs(live - s.sentValue) <= std::fabs(s.sentValue) * 1e-3f + 1e-7f) {
                s.state   = CalibSyncState::Acked;
                s.ackedAt = now;
            } else if (now - s.sentAt > 0.70) {
                s.state = CalibSyncState::Error;   // SET deadline elapsed
            }
        } else if (s.state == CalibSyncState::Acked) {
            if (now - s.ackedAt > 1.0) s.state = CalibSyncState::Synced;
        }
    }
}

// Spec §5.2 -- INAV-style range slider + numeric entry + Set.
// Returns true if the staged value changed this frame.
bool MainUi::calibRangeField(const CalibParamMeta& m, float liveValue,
                             CalibFieldSync& sync, bool showSetButton)
{
    ImGui::PushID(static_cast<int>(m.id));
    if (!sync.hasStaged) { sync.staged = liveValue; sync.hasStaged = true; }

    bool changed = false;

    // label (150 px)
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(m.label);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", m.blurb);
    ImGui::SameLine(158.0f);

    // slider (220 px). Integer/bool params get stepped sliders.
    ImGui::SetNextItemWidth(220.0f);
    if (m.isInteger) {
        int iv = static_cast<int>(std::lround(sync.staged));
        const int lo = static_cast<int>(m.uiMin), hi = static_cast<int>(m.uiMax);
        if (ImGui::SliderInt("##sld", &iv, lo, hi)) {
            sync.staged = static_cast<float>(std::clamp(iv, lo, hi));
            changed = true;
        }
    } else {
        // Pick a format with enough precision for tiny gains.
        const char* fmt = (m.uiMax <= 1.0f) ? "%.6f" : "%.3f";
        if (ImGui::SliderFloat("##sld", &sync.staged, m.uiMin, m.uiMax, fmt)) {
            sync.staged = std::clamp(sync.staged, m.uiMin, m.uiMax);
            changed = true;
        }
    }
    ImGui::SameLine();

    // numeric entry (90 px) -- accepts exact values, clamps to range.
    ImGui::SetNextItemWidth(90.0f);
    float entry = sync.staged;
    if (ImGui::InputFloat("##ent", &entry, 0.0f, 0.0f,
                          (m.uiMax <= 1.0f) ? "%.6f" : "%.4f",
                          ImGuiInputTextFlags_EnterReturnsTrue)) {
        if (entry < m.uiMin) entry = m.uiMin;
        if (entry > m.uiMax) entry = m.uiMax;
        if (entry != sync.staged) { sync.staged = entry; changed = true; }
    }
    ImGui::SameLine();

    if (changed && sync.state != CalibSyncState::Sent) {
        sync.state = (std::fabs(sync.staged - liveValue) > 1e-9f)
                     ? CalibSyncState::Edited : CalibSyncState::Synced;
    }

    // Set button (writes RAM via blob path).
    bool setClicked = false;
    if (showSetButton) {
        if (ImGui::Button("Set", ImVec2(46.0f, 0.0f))) setClicked = true;
        ImGui::SameLine();
    }

    // spec / default (grey)
    char specBuf[32];
    std::snprintf(specBuf, sizeof(specBuf), "spec %.4g", m.uiDefault);
    ImGui::TextDisabled("%s", specBuf);
    ImGui::SameLine();

    // live value (dimmed) + sync chip
    ImGui::TextDisabled("(live %.4g)", liveValue);
    ImGui::SameLine();
    calibSyncChip(sync);

    // INAV-style descriptive line under the row.
    ImGui::Indent(8.0f);
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.60f, 0.60f, 0.62f, 1.0f));
    ImGui::TextWrapped("%s", m.blurb);
    ImGui::PopStyleColor();
    ImGui::Unindent(8.0f);

    if (setClicked) {
        uint32_t bits = 0;
        std::memcpy(&bits, &sync.staged, sizeof(bits));
        if (applyMotorConfigBlobSet(m.id, bits)) {
            sync.state     = CalibSyncState::Sent;
            sync.sentValue = sync.staged;
            sync.sentAt    = ImGui::GetTime();
            char note[80];
            std::snprintf(note, sizeof(note), "CFG BLOB SET id=%u -> %.4g",
                          m.id, sync.staged);
            calibStatus_ = note;
        } else {
            sync.state = CalibSyncState::Error;
            calibStatus_ = "BLOB SET failed -- press Read All first";
        }
    }

    ImGui::PopID();
    return changed;
}

void MainUi::calibResultTable(const CalibResultRow* rows, int n)
{
    if (ImGui::BeginTable("##calib-results", 4,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                          ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("result");
        ImGui::TableSetupColumn("measured");
        ImGui::TableSetupColumn("spec");
        ImGui::TableSetupColumn("verdict", ImGuiTableColumnFlags_WidthFixed, 70.0f);
        ImGui::TableHeadersRow();
        for (int i = 0; i < n; ++i) {
            const CalibResultRow& r = rows[i];
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextUnformatted(r.label);
            ImGui::TableNextColumn();
            if (r.hasValue) ImGui::Text("%.5g %s", r.value, r.unit);
            else            ImGui::TextDisabled("--");
            ImGui::TableNextColumn();
            if (r.hasSpec)
                ImGui::Text("%.4g %s +/-%.0f%%", r.specNominal, r.unit, r.specTolPct);
            else
                ImGui::TextDisabled("--");
            ImGui::TableNextColumn();
            if (!r.hasSpec) {
                ImGui::TextDisabled("--");
            } else if (!r.hasValue) {
                ImGui::TextDisabled("--");
            } else {
                const double band = std::fabs(r.specNominal) * r.specTolPct / 100.0;
                const bool pass = std::fabs(r.value - r.specNominal) <= band;
                if (pass) ImGui::TextColored(ImVec4(0.45f,0.85f,0.45f,1.0f), "PASS");
                else      ImGui::TextColored(ImVec4(0.95f,0.40f,0.40f,1.0f), "FAIL");
            }
        }
        ImGui::EndTable();
    }
}

void MainUi::calibResultPlot(const char* title, const float* t,
                             const float* y1, const float* y2, int n,
                             double triggerT)
{
    if (n <= 1 || t == nullptr) {
        ImGui::TextDisabled("(%s: no samples yet)", title);
        return;
    }
    if (ImPlot::BeginPlot(title, ImVec2(-1, 220.0f))) {
        ImPlot::SetupAxes("time", "value", ImPlotAxisFlags_AutoFit,
                          ImPlotAxisFlags_AutoFit);
        if (y1) ImPlot::PlotLine("series 1", t, y1, n);
        if (y2) ImPlot::PlotLine("series 2", t, y2, n);
        if (triggerT >= 0.0) {
            double xs[1] = { triggerT };
            // Triggered-Capture marker colour, matches Plots tab.
            ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.30f,0.85f,0.95f,0.70f));
            ImPlot::PlotInfLines("trigger", xs, 1);
            ImPlot::PopStyleColor();
        }
        ImPlot::EndPlot();
    }
}

// Spec §5.4 -- test-panel scaffold.
void MainUi::beginTestPanel(const char* title, const char* blurb, CalibPill pill)
{
    ImGui::PushFont(nullptr);
    ImGui::TextUnformatted(title);
    ImGui::SameLine();
    ImGui::TextDisabled("result:");
    ImGui::SameLine();
    drawPill(pill);
    ImGui::PopFont();
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.72f, 0.72f, 0.74f, 1.0f));
    ImGui::TextWrapped("%s", blurb);
    ImGui::PopStyleColor();
    ImGui::Separator();
}

void MainUi::endTestPanel()
{
    ImGui::Spacing();
}

bool MainUi::calibPreconditionRow(const char* label, bool ok)
{
    if (ok) ImGui::TextColored(ImVec4(0.45f,0.85f,0.45f,1.0f), "[x]");
    else    ImGui::TextColored(ImVec4(0.95f,0.40f,0.40f,1.0f), "[ ]");
    ImGui::SameLine();
    ImGui::TextUnformatted(label);
    return ok;
}

// Fixed-height status slot. A bordered, fixed-size box that shows the last
// action note. Its geometry never depends on live data, so nothing below it
// reflows when telemetry arrives -- this replaces the old growing "Last
// frames" tail that caused the per-frame jitter.
void MainUi::calibStatusLine()
{
    const float h = ImGui::GetFrameHeight();
    ImGui::BeginChild("##calib-status", ImVec2(-FLT_MIN, h),
                      ImGuiChildFlags_Border);
    ImGui::AlignTextToFramePadding();
    if (calibStatus_.empty()) {
        ImGui::TextDisabled("status: (idle)");
    } else {
        ImGui::TextDisabled("status:");
        ImGui::SameLine();
        ImGui::TextUnformatted(calibStatus_.c_str());
    }
    ImGui::EndChild();
}

// Bind the theta service to its MainUi-owned dependencies. Called once from
// applyConfig(). The service is pure domain code; everything it cannot do
// itself (send the CAN command, re-read the blob, push a status note) it
// reaches through these callbacks.
void MainUi::wireThetaCalibration()
{
    thetaCal_.bindGuard(&calibRunGuard_);
    calib::ThetaCalibration::Callbacks cb;
    cb.sendConfig = [this](uint8_t action, float alignCurrentA) {
        return sendMotorThetaConfig(action, alignCurrentA);
    };
    cb.readResult = [this]() { readAllThetaCalibration(); };
    cb.setStatus  = [this](const std::string& s) { calibStatus_ = s; };
    thetaCal_.setCallbacks(std::move(cb));
}

// Map the theta service state + live telemetry onto a rail/panel result pill.
// Cached into calibPillTheta_ each frame so the rail code stays unchanged.
CalibPill MainUi::thetaPill() const
{
    const bool telemetryRunning = lastMotorTelemetry_.theta_cal_valid &&
                                  lastMotorTelemetry_.cal_phase == 1u;
    // Aborted is a non-pass terminal: show FAIL so the pill never hangs on
    // Running after an operator abort.
    if (thetaCal_.phase() == calib::RunPhase::Failed ||
        thetaCal_.phase() == calib::RunPhase::Aborted)  return CalibPill::Fail;
    if (telemetryRunning || thetaCal_.startPending())   return CalibPill::Running;
    if (thetaCal_.verdict() == calib::Verdict::Pass)    return CalibPill::Pass;
    return CalibPill::None;
}

// Bind the machine-profiler service to its MainUi-owned dependencies. Called
// once from applyConfig().
void MainUi::wireMachineProfilerCalibration()
{
    machineProf_.bindGuard(&calibRunGuard_);
    calib::MachineProfilerCalibration::Callbacks cb;
    cb.sendLimits     = [this]() { return sendMotorProfilerLimits(); };
    cb.sendCommand    = [this](uint8_t action, uint8_t arg, uint32_t value) {
        return sendMotorProfilerCommand(action, arg, value);
    };
    cb.requestResults = [this]() { requestMotorProfilerResults(); };
    cb.requestConfigRefresh = [this]() { sendMotorBlobGet(); };
    cb.clearResults   = [this]() {
        std::fill(std::begin(motorProfiler_.result_valid),
                  std::end(motorProfiler_.result_valid), false);
    };
    cb.setStatus      = [this](const std::string& s) { calibStatus_ = s; };
    machineProf_.setCallbacks(std::move(cb));
}

// Map the machine-profiler service state onto a rail/panel result pill. While
// a finished run is still fetching its Rs result the pill reads Running (it
// flips to Pass once the result arrives), matching the old inline behaviour.
CalibPill MainUi::machinePill() const
{
    if (machineProf_.phase() == calib::RunPhase::Failed ||
        machineProf_.phase() == calib::RunPhase::Aborted) return CalibPill::Fail;
    if (machineProf_.verdict() == calib::Verdict::Pass)   return CalibPill::Pass;
    if (machineProf_.phase() == calib::RunPhase::Running ||
        machineProf_.phase() == calib::RunPhase::Done)    return CalibPill::Running;
    return CalibPill::None;
}

// ==========================================================================
// Tab entry point + left rail.
// ==========================================================================
void MainUi::drawCalibration()
{
    CalibGate g;
    calibPreconditions(g);
    calibRefreshSyncFromBlob();

    // Advance the theta state machine (resolves the start-pending timeout even
    // when the panel is not the visible section) and refresh its result pill
    // before the rail draws it. The pill is a cached view of thetaCal_.
    thetaCal_.tick(ImGui::GetTime());
    calibPillTheta_ = thetaPill();
    // Machine pill mirrors machineProf_ (refreshed by drawCalibMachine's
    // per-frame update()); read it here so the rail stays in sync.
    calibPillMachine_ = machinePill();

    // Header line.
    ImGui::TextUnformatted("Calibration");
    ImGui::SameLine();
    {
        uint32_t fwBits = 0;
        if (motorBlobValid_) std::memcpy(&fwBits, &motorBlob_[4], sizeof(fwBits));
        if (motorBlobValid_) {
            ImGui::TextDisabled("[motor v%u.%u.%u]", (fwBits >> 16) & 0xFFu,
                                (fwBits >> 8) & 0xFFu, fwBits & 0xFFu);
        } else {
            ImGui::TextDisabled("[motor: not read]");
        }
    }
    // A disconnected transport never blocks navigation -- only the action
    // buttons inside a panel are disabled (spec fix: static / offline rail).
    if (!g.canSend) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.25f, 1.0f),
                           "(offline -- connect to run tests)");
    }

    // Compact config strip, pinned above the rail/content split.
    drawCalibConfigStrip();
    ImGui::Separator();

    // Rail + content split. The rail is pure local UI state -- it renders and
    // switches sections regardless of connection or any pending refresh.
    //
    // Both children force ImGuiWindowFlags_AlwaysVerticalScrollbar so the
    // scrollbar gutter is permanently reserved. Without it, when the content
    // height sits near the child height, ImGui's auto scrollbar toggles on/off
    // every frame: that changes the inner width, which reflows the content,
    // which changes the height -- a feedback loop that makes the whole panel
    // visibly "breathe". A permanently-present gutter keeps the inner width
    // constant and breaks the loop.
    const float railW = 232.0f;
    ImGui::BeginChild("##calib-rail", ImVec2(railW, 0),
                      ImGuiChildFlags_Border,
                      ImGuiWindowFlags_AlwaysVerticalScrollbar);
    drawCalibrationRail();
    ImGui::EndChild();
    ImGui::SameLine();
    ImGui::BeginChild("##calib-content", ImVec2(0, 0),
                      ImGuiChildFlags_Border,
                      ImGuiWindowFlags_AlwaysVerticalScrollbar);
    switch (calibSection_) {
        case 0:  drawCalibTheta();      break;
        case 1:  drawCalibMachine();    break;
        case 2:  drawCalibPiTuning();   break;
        case 3:  drawCalibInertia();    break;
        case 4:  drawCalibConfigFull(); break;
        default: drawCalibTheta();      break;
    }
    ImGui::EndChild();
}

// Draw one navigation entry in the rail: a large, well-spaced hit target with
// a title, a subtitle, an obvious selected highlight and an optional result
// pill. Returns true if the entry was clicked this frame.
static bool drawCalibRailEntry(int index, const CalibSectionLabel& lbl,
                               bool selected, const CalibPill* pill)
{
    ImGui::PushID(index);

    const ImGuiStyle& style = ImGui::GetStyle();
    const float lineH    = ImGui::GetTextLineHeight();
    const float padY     = 8.0f;
    const float entryH   = padY * 2.0f + lineH * 2.0f + 4.0f;
    const float width    = ImGui::GetContentRegionAvail().x;

    // The whole entry is one Selectable so it is an unambiguous click target.
    const ImVec2 cursor = ImGui::GetCursorScreenPos();
    bool clicked = ImGui::Selectable("##nav", selected,
                                     ImGuiSelectableFlags_None,
                                     ImVec2(width, entryH));

    // Accent bar on the selected entry so the highlight is obvious.
    ImDrawList* dl = ImGui::GetWindowDrawList();
    if (selected) {
        dl->AddRectFilled(cursor,
                          ImVec2(cursor.x + 4.0f, cursor.y + entryH),
                          ImGui::GetColorU32(ImVec4(0.40f, 0.78f, 0.95f, 1.0f)),
                          2.0f);
    }

    // Title + subtitle text, drawn over the Selectable.
    const ImVec2 textPos(cursor.x + 12.0f, cursor.y + padY);
    const ImU32 titleCol = ImGui::GetColorU32(
        selected ? ImVec4(1.0f, 1.0f, 1.0f, 1.0f)
                 : ImVec4(0.88f, 0.88f, 0.90f, 1.0f));
    const ImU32 subCol = ImGui::GetColorU32(ImVec4(0.62f, 0.62f, 0.65f, 1.0f));
    dl->AddText(textPos, titleCol, lbl.title);
    dl->AddText(ImVec2(textPos.x, textPos.y + lineH + 4.0f), subCol,
                lbl.subtitle);

    // Result pill, right-aligned and vertically centred.
    if (pill != nullptr) {
        const char* pl = calibPillLabel(*pill);
        const ImVec4 pc = calibPillColor(*pill);
        const ImVec2 pillSize = ImGui::CalcTextSize(pl);
        const float  px = cursor.x + width - pillSize.x - 22.0f;
        const float  py = cursor.y + (entryH - pillSize.y) * 0.5f;
        dl->AddRectFilled(ImVec2(px - 6.0f, py - 3.0f),
                          ImVec2(px + pillSize.x + 6.0f, py + pillSize.y + 3.0f),
                          ImGui::GetColorU32(ImVec4(pc.x, pc.y, pc.z, 0.28f)),
                          4.0f);
        dl->AddText(ImVec2(px, py), ImGui::GetColorU32(pc), pl);
    }

    (void)style;
    ImGui::PopID();
    return clicked;
}

void MainUi::drawCalibrationRail()
{
    // Per-section result pills; nullptr for Config (full).
    const CalibPill* pills[kCalibSectionCount] = {
        &calibPillTheta_, &calibPillMachine_, &calibPillPi_,
        &calibPillInertia_, nullptr,
    };

    ImGui::TextDisabled("CALIBRATION STEPS");
    ImGui::Spacing();

    for (int i = 0; i < kCalibSectionCount; ++i) {
        // Generous separation before "Config (full)" -- it is not a test.
        if (i == 4) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();
            ImGui::TextDisabled("CONFIGURATION");
            ImGui::Spacing();
        }
        const bool selected = (calibSection_ == i);
        if (drawCalibRailEntry(i, kCalibSectionLabels[i], selected, pills[i])) {
            // Pure local UI state -- no connection / refresh gating.
            calibSection_ = i;
        }
        ImGui::Spacing();
    }
}

// ==========================================================================
// Compact motor-config strip (spec §3.2).
// ==========================================================================
void MainUi::drawCalibConfigStrip()
{
    if (!ImGui::CollapsingHeader("Motor configuration",
                                 ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    CalibGate g; calibPreconditions(g);

    // Each field gets its OWN line -- no ImGui::SameLine(). A SameLine after
    // variable-width text (%.4g of Rs/lam, %.2f of PI gains) makes whatever
    // follows it jump horizontally every time the live value refreshes; that
    // is the "it still jitters when something updates" the user sees. Four
    // fixed lines: constant height, zero horizontal coupling.
    if (!motorBlobValid_) {
        ImGui::TextDisabled("config not read yet -- press Read All");
        ImGui::TextDisabled(" ");
        ImGui::TextDisabled(" ");
        ImGui::TextDisabled(" ");
    } else {
        const float thetaOff = calibBlobValueF32(0);
        int sectorsCal = 0;
        for (int s = 0; s < 6; ++s)
            if (lastMotorTelemetry_.theta_sector_valid[s]) ++sectorsCal;
        ImGui::Text("THETA    offset %+.5f rad   sectors %d/6",
                    thetaOff, sectorsCal);
        ImGui::Text("PI       Id %.2f/%.2f   Iq %.2f/%.2f   w %.4f/%.2g",
                    calibBlobValueF32(10), calibBlobValueF32(11),
                    calibBlobValueF32(12), calibBlobValueF32(13),
                    calibBlobValueF32(14), calibBlobValueF32(15));
        ImGui::Text("MACHINE  Rs %.4g   Ld %.3f mH   Lq %.3f mH   lam %.4g",
                    calibBlobValueF32(20), calibBlobValueF32(21) * 1000.0f,
                    calibBlobValueF32(22) * 1000.0f, calibBlobValueF32(23));
        const bool pfc = calibBlobValueF32(50) != 0.0f;
        ImGui::TextColored(pfc ? ImVec4(0.45f,0.85f,0.45f,1.0f)
                               : ImVec4(0.85f,0.85f,0.85f,1.0f),
                           "PFC      [%s]", pfc ? "ON" : "off");
    }

    // Action buttons.
    if (ImGui::Button("Read All##strip")) {
        motorBlobAuthoritativeUntil_ = 0.0;
        sendMotorBlobGet();
        for (auto& kv : calibSync_) kv.second.hasStaged = false;
        calibStatus_ = "Read All -- BLOB GET requested";
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!g.motorIdle);
    if (ImGui::Button("Save to flash##strip")) {
        sendMotorConfigParam(/*SAVE_ALL*/ 2, 0, 0);
        motorBlobAuthoritativeUntil_ = 0.0;
        calibStatus_ = "PARAM SAVE_ALL sent (flash erase ~1.5 s)";
    }
    ImGui::EndDisabled();
    if (!g.motorIdle && ImGui::IsItemHovered()) {
        ImGui::SetTooltip("motor must be idle, flash erase blocks the bus ~1.5 s");
    }
    ImGui::SameLine();
    if (ImGui::Button("Revert RAM##strip")) {
        sendMotorConfigParam(/*LOAD_ALL*/ 3, 0, 0);
        calibStatus_ = "PARAM LOAD_ALL sent (RAM <- flash)";
    }

    // Unsaved-edit count.
    int edited = 0;
    for (const auto& kv : calibSync_)
        if (kv.second.state == CalibSyncState::Edited ||
            kv.second.state == CalibSyncState::Sent) ++edited;
    ImGui::SameLine(0, 24);
    if (edited > 0)
        ImGui::TextColored(ImVec4(0.40f,0.78f,0.95f,1.0f),
                           "%d field(s) edited -- not on motor", edited);
    else
        ImGui::TextDisabled("config in sync");
}

// ==========================================================================
// Test 1 -- Theta calibration (migrated from drawMotorTab()).
// ==========================================================================
void MainUi::drawCalibTheta()
{
    CalibGate g; calibPreconditions(g);

    // (A) header. "Running" is driven by the MTR_ANS_THETA_CAL telemetry
    // phase (RUNNING=1), the motor's authoritative live state. The run-state
    // machine (start-pending / DONE edge / timeout / reject / verdict) lives in
    // thetaCal_; this panel only reads its state and the live telemetry below.
    // calibPillTheta_ is refreshed from thetaCal_ in drawCalibration().
    const bool running = lastMotorTelemetry_.theta_cal_valid &&
                         lastMotorTelemetry_.cal_phase == 1u;
    beginTestPanel("Test 1 - Theta (electrical-angle) calibration",
                   "Calibrates the offset between the Hall sensor reference "
                   "and the motor's electrical zero so field-oriented control "
                   "commutates correctly. Runs a controlled current vector "
                   "that pulls the rotor to a known angle, then records the "
                   "measured offset -- globally and per 60 deg Hall sector.",
                   calibPillTheta_);

    // (B) preconditions
    ImGui::TextDisabled("Preconditions");
    bool ok = true;
    ok &= calibPreconditionRow("transport open", g.canSend);
    bool diagOk = calibPreconditionRow("diag mode active", g.inDiag);
    if (!g.inDiag) {
        ImGui::SameLine();
        ImGui::TextDisabled("(use Take control in the header bar)");
    }
    ok &= diagOk;
    ok &= calibPreconditionRow("FSM idle", g.motorIdle);
    ok &= calibPreconditionRow("no PWM fault (mask & 0x1FF == 0)",
                               (g.faultMask & 0x1FFu) == 0);
    ImGui::Checkbox("shaft is free to rotate (operator confirm)",
                    &calibThetaShaftFree_);
    ok &= calibThetaShaftFree_;
    ImGui::Separator();

    // (C) parameters
    ImGui::TextDisabled("Parameters");
    ImGui::SetNextItemWidth(220.0f);
    ImGui::SliderFloat("Align current (A)", &motorThetaAlignCurrent_,
                       0.05f, 1.00f, "%.2f");
    motorThetaAlignCurrent_ = std::clamp(motorThetaAlignCurrent_, 0.05f, 1.0f);
    ImGui::SameLine();
    ImGui::TextDisabled("pull strength -- more = firmer + hotter");
    ImGui::Checkbox("Save result to flash after calibration",
                    &motorThetaSaveAfterCal_);
    ImGui::Separator();

    // (D) run strip. The Start/Abort handlers are thin: they build the
    // ThetaParams and hand off to thetaCal_, which owns the start-pending,
    // RUNNING/DONE-edge, timeout, reject and verdict logic. No state machine
    // lives in this panel anymore.
    ImGui::TextDisabled("Run");
    // Disabled if preconditions fail, our own run is in flight, telemetry shows
    // a sweep already running, OR a DIFFERENT calibration test owns the single
    // run guard (calibration-autotests-tz.md: one run at a time).
    const bool guardBlocked = calibRunGuard_.blockedFor(calib::Test::Theta);
    ImGui::BeginDisabled(!ok || thetaCal_.startPending() || running ||
                         guardBlocked);
    if (ImGui::Button("Start calibration", ImVec2(160, 0))) {
        calib::ThetaParams p;
        p.alignCurrentA      = motorThetaAlignCurrent_;
        p.saveAfterCal       = motorThetaSaveAfterCal_;
        p.shaftFreeConfirmed = calibThetaShaftFree_;
        thetaCal_.start(p, ImGui::GetTime());
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Abort", ImVec2(90, 0))) {
        // STOP/torque-off is a transport action and stays in the UI; the
        // service just resets run state and releases the guard.
        sendMotorProxy(0 /* STOP */, 0.0f);
        thetaCal_.abort(ImGui::GetTime());
        calibStatus_ = "STOP sent (abort theta calibration)";
    }
    ImGui::SameLine();
    if (ImGui::Button("Read calibration", ImVec2(150, 0))) {
        readAllThetaCalibration();
        calibStatus_ = "theta read-all (BLOB GET)";
    }
    if (guardBlocked) {
        ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.25f, 1.0f),
                           "another calibration test is running -- finish it first");
    }

    // Persistent failure banner. The message is owned by thetaCal_ (set on a
    // DONE_FAIL phase, a rejected command, or a start timeout) and cleared
    // when a fresh calibration starts or a DONE_OK arrives.
    if (!thetaCal_.failMessage().empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.97f, 0.45f, 0.45f, 1.0f));
        ImGui::TextWrapped("[FAIL] %s", thetaCal_.failMessage().c_str());
        ImGui::PopStyleColor();
        ImGui::SameLine();
        if (ImGui::SmallButton("dismiss##thetafail")) thetaCal_.dismissFailMessage();
    }

    // Status line + progress bar. Progress is now a REAL percent from the
    // MTR_ANS_THETA_CAL telemetry (FW reports 0..100 across the ~6 s
    // sweep), not an indeterminate animation. Both slots are always
    // drawn so an incoming status never reflows the results table.
    bool thetaRunning = running;
    if (lastMotorTelemetry_.theta_cal_valid) {
        const char* phName[] = { "idle", "RUNNING", "done (ok)", "done (FAIL)" };
        const uint8_t ph = lastMotorTelemetry_.cal_phase;
        ImGui::Text("calibration phase: %s",
                    (ph < 4u) ? phName[ph] : "?");
    } else if (lastMotorTelemetry_.theta_valid) {
        ImGui::Text("theta status: %s",
                    calibThetaStatusName(lastMotorTelemetry_.theta_status));
    } else {
        ImGui::TextDisabled("theta status: no data yet -- press Read calibration");
    }
    if (thetaCal_.startPending()) {
        const float f = (float)(0.5 + 0.5 * std::sin(ImGui::GetTime() * 4.0));
        ImGui::ProgressBar(f, ImVec2(-FLT_MIN, 0), "starting...");
    } else if (thetaRunning) {
        const float pct = std::clamp(
            lastMotorTelemetry_.cal_progress_pct / 100.0f, 0.0f, 1.0f);
        char lbl[32];
        std::snprintf(lbl, sizeof(lbl), "calibrating %u%%",
                      (unsigned)lastMotorTelemetry_.cal_progress_pct);
        ImGui::ProgressBar(pct, ImVec2(-FLT_MIN, 0), lbl);
    } else {
        ImGui::ProgressBar(0.0f, ImVec2(-FLT_MIN, 0), "idle");
    }
    calibStatusLine();
    ImGui::Separator();

    // (E) results -- the 7-row LUT table.
    ImGui::TextDisabled("Results -- theta LUT");
    if (ImGui::BeginTable("##calib-theta-lut", 6,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                          ImGuiTableFlags_SizingFixedFit)) {
        ImGui::TableSetupColumn("slot",      ImGuiTableColumnFlags_WidthFixed, 120.0f);
        ImGui::TableSetupColumn("rad",       ImGuiTableColumnFlags_WidthFixed, 110.0f);
        ImGui::TableSetupColumn("deg",       ImGuiTableColumnFlags_WidthFixed,  85.0f);
        ImGui::TableSetupColumn("d-deg",     ImGuiTableColumnFlags_WidthFixed,  85.0f);
        ImGui::TableSetupColumn("edit rad",  ImGuiTableColumnFlags_WidthFixed, 130.0f);
        ImGui::TableSetupColumn("Set",       ImGuiTableColumnFlags_WidthFixed,  60.0f);
        ImGui::TableHeadersRow();

        for (int slot = 0; slot <= 6; ++slot) {
            ImGui::TableNextRow();
            ImGui::PushID(slot);
            ImGui::TableSetColumnIndex(0);
            if (slot == 0) ImGui::TextUnformatted("global offset");
            else           ImGui::Text("sector %d", slot);

            bool valid;  float r;
            if (slot == 0) {
                valid = lastMotorTelemetry_.theta_valid;
                r = valid ? lastMotorTelemetry_.theta_offset_rad : 0.0f;
            } else {
                valid = lastMotorTelemetry_.theta_sector_valid[slot - 1];
                r = valid ? lastMotorTelemetry_.theta_sector_lut_rad[slot - 1]
                          : 0.0f;
            }
            const float d = r * 57.2957795f;

            ImGui::TableSetColumnIndex(1);
            if (valid) ImGui::Text("%+.5f", r); else ImGui::TextDisabled("--");
            ImGui::TableSetColumnIndex(2);
            if (valid) ImGui::Text("%+8.3f", d); else ImGui::TextDisabled("--");
            ImGui::TableSetColumnIndex(3);
            if (slot == 0) {
                ImGui::TextDisabled("n/a");
            } else if (valid) {
                float defD = ((float)(slot - 1) + 0.5f) * 60.0f;
                float delta = d - defD;
                while (delta >  180.0f) delta -= 360.0f;
                while (delta < -180.0f) delta += 360.0f;
                // Advisory +/-15 deg band.
                const bool inBand = std::fabs(delta) <= 15.0f;
                ImGui::TextColored(inBand ? ImVec4(0.45f,0.85f,0.45f,1.0f)
                                          : ImVec4(0.95f,0.75f,0.25f,1.0f),
                                   "%+8.3f", delta);
            } else {
                ImGui::TextDisabled("--");
            }
            ImGui::TableSetColumnIndex(4);
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::InputFloat("##edit", &motorThetaSlotEdit_[slot],
                                  0.001f, 0.01f, "%.5f")) {
                motorThetaSlotDirty_[slot] = true;
                if (slot == 0) {
                    lastMotorTelemetry_.theta_valid = true;
                    lastMotorTelemetry_.theta_offset_rad = motorThetaSlotEdit_[slot];
                    if (lastMotorTelemetry_.theta_cal_valid)
                        lastMotorTelemetry_.cal_theta_offset_rad = motorThetaSlotEdit_[slot];
                } else {
                    lastMotorTelemetry_.theta_sector_valid[slot - 1] = true;
                    lastMotorTelemetry_.theta_sector_lut_rad[slot - 1] =
                        motorThetaSlotEdit_[slot];
                    lastMotorTelemetry_.theta_sector_receivedAt[slot - 1] =
                        ImGui::GetTime();
                }
            }
            ImGui::TableSetColumnIndex(5);
            ImGui::BeginDisabled(!g.canSend);
            if (ImGui::Button("Set##slot", ImVec2(-FLT_MIN, 0))) {
                if (applyThetaSlotEdit(slot, motorThetaSlotEdit_[slot])) {
                    motorThetaSlotDirty_[slot] = false;
                    if (slot == 0) {
                        lastMotorTelemetry_.theta_valid = true;
                        lastMotorTelemetry_.theta_offset_rad = motorThetaSlotEdit_[slot];
                        if (lastMotorTelemetry_.theta_cal_valid)
                            lastMotorTelemetry_.cal_theta_offset_rad = motorThetaSlotEdit_[slot];
                    } else {
                        lastMotorTelemetry_.theta_sector_valid[slot - 1] = true;
                        lastMotorTelemetry_.theta_sector_lut_rad[slot - 1] =
                            motorThetaSlotEdit_[slot];
                        lastMotorTelemetry_.theta_sector_receivedAt[slot - 1] =
                            ImGui::GetTime();
                    }
                    char note[64];
                    std::snprintf(note, sizeof(note),
                                  "theta slot %d SET %.5f rad", slot,
                                  motorThetaSlotEdit_[slot]);
                    calibStatus_ = note;
                }
            }
            ImGui::EndDisabled();
            ImGui::PopID();
        }
        ImGui::EndTable();
    }

    // ----------------------------------------------------------------
    // Live polar / clock diagram (spec Test 1). Draws, on a unit circle:
    //   - the 6 nominal 60-deg Hall-sector boundary spokes (faint),
    //   - a labelled POINT at each sector's nominal 60-deg centre (grey),
    //   - a labelled POINT at each sector's MEASURED electrical angle
    //     (green if within +/-15 deg of nominal, amber if skewed),
    //   - the live rotor electrical angle as a moving needle while the
    //     calibration sweep is running.
    // The measured points update as each sector's LUT entry arrives, so
    // the operator watches the diagram fill in live, then sees the final
    // per-sector result on completion.
    // ----------------------------------------------------------------
    if (ImPlot::BeginPlot("Theta polar (Hall sectors)", ImVec2(280, 280),
                          ImPlotFlags_Equal | ImPlotFlags_NoLegend |
                          ImPlotFlags_NoMenus)) {
        ImPlot::SetupAxesLimits(-1.25, 1.25, -1.25, 1.25, ImPlotCond_Always);
        ImPlot::SetupAxis(ImAxis_X1, nullptr, ImPlotAxisFlags_NoDecorations);
        ImPlot::SetupAxis(ImAxis_Y1, nullptr, ImPlotAxisFlags_NoDecorations);

        // Unit circle.
        float cx[37], cy[37];
        for (int i = 0; i <= 36; ++i) {
            const float a = (float)i * 10.0f * 3.14159265f / 180.0f;
            cx[i] = std::cos(a); cy[i] = std::sin(a);
        }
        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.45f,0.45f,0.50f,0.9f));
        ImPlot::PlotLine("##circle", cx, cy, 37);
        ImPlot::PopStyleColor();

        // Faint sector boundary spokes every 60 deg.
        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.35f,0.35f,0.40f,0.7f));
        for (int s = 0; s < 6; ++s) {
            const float a = (float)s * 60.0f * 3.14159265f / 180.0f;
            float bx[2] = { 0.0f, std::cos(a) };
            float by[2] = { 0.0f, std::sin(a) };
            char id[16]; std::snprintf(id, sizeof(id), "##b%d", s);
            ImPlot::PlotLine(id, bx, by, 2);
        }
        ImPlot::PopStyleColor();

        // Nominal sector-centre points (uniform 60 deg model).
        float nomX[6], nomY[6];
        for (int s = 0; s < 6; ++s) {
            const float a = ((float)s + 0.5f) * 60.0f * 3.14159265f / 180.0f;
            nomX[s] = 0.78f * std::cos(a);
            nomY[s] = 0.78f * std::sin(a);
        }
        ImPlot::PushStyleColor(ImPlotCol_MarkerOutline,
                               ImVec4(0.55f,0.55f,0.60f,1.0f));
        ImPlot::PushStyleColor(ImPlotCol_MarkerFill,
                               ImVec4(0.55f,0.55f,0.60f,0.4f));
        ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 5.0f);
        ImPlot::PlotScatter("nominal", nomX, nomY, 6);
        ImPlot::PopStyleColor(2);

        // Live sweep progress: while the calibration is running the motor
        // does NOT yet know each sector's measured angle (that is fitted
        // at the end), but MTR_ANS_THETA_CAL's per-sector valid bitmap
        // tells us which Hall sectors have already accumulated enough
        // samples. Light those nominal-centre markers blue so the
        // operator watches the diagram fill in sector-by-sector live.
        if (thetaRunning && lastMotorTelemetry_.theta_cal_valid) {
            float liveX[6], liveY[6];
            int nLive = 0;
            for (int s = 0; s < 6; ++s) {
                if (lastMotorTelemetry_.cal_sector_valid & (1u << s)) {
                    liveX[nLive] = nomX[s];
                    liveY[nLive] = nomY[s];
                    ++nLive;
                }
            }
            if (nLive > 0) {
                ImPlot::PushStyleColor(ImPlotCol_MarkerOutline,
                                       ImVec4(0.40f,0.78f,0.95f,1.0f));
                ImPlot::PushStyleColor(ImPlotCol_MarkerFill,
                                       ImVec4(0.40f,0.78f,0.95f,0.7f));
                ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 8.0f);
                ImPlot::PlotScatter("sector measured (live)", liveX, liveY, nLive);
                ImPlot::PopStyleColor(2);
            }
        }

        // Measured sector points -- one Scatter per pass/skew colour so the
        // operator can tell a good sector from a skewed one at a glance.
        float okX[6], okY[6], badX[6], badY[6];
        int nOk = 0, nBad = 0;
        for (int s = 0; s < 6; ++s) {
            if (!lastMotorTelemetry_.theta_sector_valid[s]) continue;
            const float ang = lastMotorTelemetry_.theta_sector_lut_rad[s];
            const float px = std::cos(ang), py = std::sin(ang);
            // Skew vs the nominal 60-deg centre.
            float deltaDeg = ang * 57.2957795f -
                             ((float)s + 0.5f) * 60.0f;
            while (deltaDeg >  180.0f) deltaDeg -= 360.0f;
            while (deltaDeg < -180.0f) deltaDeg += 360.0f;
            if (std::fabs(deltaDeg) <= 15.0f) {
                okX[nOk] = px; okY[nOk] = py; ++nOk;
            } else {
                badX[nBad] = px; badY[nBad] = py; ++nBad;
            }
        }
        if (nOk > 0) {
            ImPlot::PushStyleColor(ImPlotCol_MarkerOutline,
                                   ImVec4(0.45f,0.85f,0.45f,1.0f));
            ImPlot::PushStyleColor(ImPlotCol_MarkerFill,
                                   ImVec4(0.45f,0.85f,0.45f,0.8f));
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Diamond, 7.0f);
            ImPlot::PlotScatter("measured (in band)", okX, okY, nOk);
            ImPlot::PopStyleColor(2);
        }
        if (nBad > 0) {
            ImPlot::PushStyleColor(ImPlotCol_MarkerOutline,
                                   ImVec4(0.95f,0.65f,0.25f,1.0f));
            ImPlot::PushStyleColor(ImPlotCol_MarkerFill,
                                   ImVec4(0.95f,0.65f,0.25f,0.8f));
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Diamond, 7.0f);
            ImPlot::PlotScatter("measured (skewed)", badX, badY, nBad);
            ImPlot::PopStyleColor(2);
        }

        // Global theta-offset needle. Prefer the live MTR_ANS_THETA_CAL
        // telemetry value (refreshes every round-robin cycle) over the
        // BLOB-sourced theta_offset_rad, which is only as fresh as the
        // last BLOB GET. Falls back to the BLOB value when no cal
        // telemetry has been seen this session.
        const bool haveNeedle = lastMotorTelemetry_.theta_cal_valid ||
                                lastMotorTelemetry_.theta_valid;
        if (haveNeedle) {
            const float ang = lastMotorTelemetry_.theta_cal_valid
                                  ? lastMotorTelemetry_.cal_theta_offset_rad
                                  : lastMotorTelemetry_.theta_offset_rad;
            float nx[2] = { 0.0f, std::cos(ang) };
            float ny[2] = { 0.0f, std::sin(ang) };
            ImPlot::PushStyleColor(ImPlotCol_Line,
                                   thetaRunning
                                       ? ImVec4(0.40f,0.78f,0.95f,1.0f)
                                       : ImVec4(0.40f,0.78f,0.95f,0.55f));
            ImPlot::PlotLine("theta offset", nx, ny, 2);
            ImPlot::PopStyleColor();
        }
        ImPlot::EndPlot();
    }
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::TextDisabled("Grey = nominal 60 deg sector centres.");
    ImGui::TextDisabled("Green diamond = measured sector within +/-15 deg.");
    ImGui::TextDisabled("Amber diamond = measured sector skewed.");
    ImGui::TextDisabled("Blue needle = live rotor angle during the sweep.");
    {
        int got = 0;
        for (int s = 0; s < 6; ++s)
            if (lastMotorTelemetry_.theta_sector_valid[s]) ++got;
        ImGui::Spacing();
        ImGui::Text("Sectors measured: %d / 6", got);
    }
    ImGui::Spacing();
    ImGui::BeginDisabled(!g.motorIdle);
    if (ImGui::Button("Save All to flash", ImVec2(160, 0))) {
        sendMotorConfigParam(/*SAVE_ALL*/ 2, 0, 0);
        motorBlobAuthoritativeUntil_ = 0.0;
        calibStatus_ = "PARAM SAVE_ALL sent";
    }
    ImGui::EndDisabled();
    ImGui::EndGroup();

    endTestPanel();
}

// ==========================================================================
// Test 2 -- Machine parameter calibration (profiler).
// ==========================================================================
void MainUi::drawCalibMachine()
{
    CalibGate g; calibPreconditions(g);

    // Feed the run-state machine this frame's profiler telemetry first, then
    // read its derived pill. The service owns polling / done-fetch / verdict /
    // progress history; this panel only renders and issues operator actions.
    {
        calib::MachineTelemetry tel;
        tel.state        = motorProfiler_.state;
        tel.progress     = motorProfiler_.progress;
        tel.error        = motorProfiler_.error;
        tel.activeTest   = motorProfiler_.active_test;
        tel.result0Valid = motorProfiler_.result_valid[0];
        machineProf_.update(g.canSend, tel, ImGui::GetTime());
    }
    calibPillMachine_ = machinePill();

    beginTestPanel("Test 2 - Machine parameter calibration",
                   "Auto-identifies the motor's electrical parameters -- "
                   "stator resistance Rs and inductances Ld/Lq -- by injecting "
                   "controlled test signals with the rotor locked, then derives "
                   "recommended current-loop PI gains. Two procedures: a "
                   "no-load pre-check (XX, free shaft) and a locked-rotor "
                   "measurement (KZ).",
                   calibPillMachine_);

    // (B) preconditions
    ImGui::TextDisabled("Preconditions");
    calibPreconditionRow("transport open", g.canSend);
    bool diagOk = calibPreconditionRow("diag mode active", g.inDiag);
    if (!g.inDiag) {
        ImGui::SameLine();
        ImGui::TextDisabled("(use Take control in the header bar)");
    }
    calibPreconditionRow("FSM idle / profiler not running", g.motorIdle);
    calibPreconditionRow("no PWM fault", (g.faultMask & 0x1FFu) == 0);
    ImGui::Checkbox("rotor mechanically locked (required for KZ)",
                    &calibRotorLocked_);
    const bool baseOk = g.canSend && diagOk && g.motorIdle &&
                        ((g.faultMask & 0x1FFu) == 0);
    ImGui::Separator();

    // (C) parameters -- profiler limits.
    ImGui::TextDisabled("Parameters");
    ImGui::SetNextItemWidth(200.0f);
    ImGui::SliderFloat("Test current (A)", &motorProfilerTestCurrentA_,
                       0.05f, 3.0f, "%.2f");
    ImGui::SetNextItemWidth(200.0f);
    ImGui::SliderFloat("Max voltage (%)", &motorProfilerMaxVoltagePct_,
                       1.0f, 25.0f, "%.1f");
    ImGui::SetNextItemWidth(200.0f);
    ImGui::SliderFloat("PI bandwidth (Hz)", &motorProfilerBandwidthHz_,
                       50.0f, 2000.0f, "%.0f");
    ImGui::SetNextItemWidth(200.0f);
    ImGui::SliderFloat("Pulse length (ms)", &motorProfilerPulseMs_,
                       1.0f, 200.0f, "%.0f");
    ImGui::SetNextItemWidth(200.0f);
    ImGui::SliderFloat("Settle time (ms)", &motorProfilerSettleMs_,
                       1.0f, 200.0f, "%.0f");
    ImGui::SetNextItemWidth(200.0f);
    ImGui::SliderInt("Repeat count", &motorProfilerRepeatCount_, 1, 16);
    motorProfilerRepeatCount_ = std::clamp(motorProfilerRepeatCount_, 1, 16);
    ImGui::Separator();

    // (D) run strip -- two procedures. The handlers are thin: they hand off to
    // machineProf_, which clears results, sends limits + START, and owns the
    // polling / done-fetch / verdict that used to live here. A run is also
    // blocked if a DIFFERENT calibration test owns the single run guard.
    const bool guardBlocked = calibRunGuard_.blockedFor(calib::Test::Machine);
    ImGui::TextDisabled("Run");
    ImGui::BeginDisabled(!baseOk || guardBlocked);
    if (ImGui::Button("Run XX pre-check", ImVec2(160, 0))) {
        machineProf_.start(
            calib::MachineProfilerCalibration::Procedure::NoloadPrecheck,
            ImGui::GetTime());
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!baseOk || !calibRotorLocked_ || guardBlocked);
    if (ImGui::Button("Run KZ identification", ImVec2(190, 0))) {
        machineProf_.start(
            calib::MachineProfilerCalibration::Procedure::BlockedIdentify,
            ImGui::GetTime());
    }
    ImGui::EndDisabled();
    if (!calibRotorLocked_ && ImGui::IsItemHovered())
        ImGui::SetTooltip("confirm the rotor is mechanically locked first");
    ImGui::SameLine();
    if (ImGui::Button("Abort##mach", ImVec2(90, 0))) {
        machineProf_.abort(ImGui::GetTime());
    }
    if (guardBlocked) {
        ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.25f, 1.0f),
                           "another calibration test is running -- finish it first");
    }

    ImGui::Text("state %s  test %s  error %s",
                calibProfilerStateName(motorProfiler_.state),
                calibProfilerTestName(motorProfiler_.active_test),
                calibProfilerErrorName(motorProfiler_.error));
    ImGui::ProgressBar(static_cast<float>(motorProfiler_.progress) / 100.0f,
                       ImVec2(-FLT_MIN, 0));

    // Failure banner owned by the service (FW error/abort or operator abort).
    if (!machineProf_.failMessage().empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.97f, 0.45f, 0.45f, 1.0f));
        ImGui::TextWrapped("[FAIL] %s", machineProf_.failMessage().c_str());
        ImGui::PopStyleColor();
        ImGui::SameLine();
        if (ImGui::SmallButton("dismiss##machfail"))
            machineProf_.dismissFailMessage();
    }

    calibStatusLine();
    ImGui::Separator();

    // Live progress mini-graph -- the test's live state at a glance.
    if (ImPlot::BeginPlot("Identification progress", ImVec2(-1, 150.0f),
                          ImPlotFlags_NoLegend | ImPlotFlags_NoMenus)) {
        ImPlot::SetupAxes("time (s)", "progress %",
                          ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_None);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, 105.0, ImPlotCond_Always);
        const std::vector<float>& progT = machineProf_.progressTimes();
        const std::vector<float>& progY = machineProf_.progressValues();
        if (progT.size() > 1) {
            ImPlot::PushStyleColor(ImPlotCol_Line,
                                   (motorProfiler_.state == 1)
                                       ? ImVec4(0.95f,0.75f,0.25f,1.0f)
                                       : ImVec4(0.45f,0.85f,0.45f,1.0f));
            ImPlot::PlotLine("progress", progT.data(), progY.data(),
                             static_cast<int>(progT.size()));
            ImPlot::PopStyleColor();
        } else {
            ImPlot::Annotation(0.5, 0.5, ImVec4(0,0,0,0), ImVec2(0,0), false,
                               "live progress appears while a run is active");
        }
        ImPlot::EndPlot();
    }
    ImGui::Separator();

    // (E) results -- Rs/Ld/Lq with spec bands.
    ImGui::TextDisabled("Results");
    CalibResultRow rows[7];
    rows[0] = { "Rs", motorProfiler_.result_valid[0],
                motorProfiler_.results[0], "Ohm", true,
                kSpecRsOhm, kSpecRsTolPct };
    rows[1] = { "Ld", motorProfiler_.result_valid[1],
                motorProfiler_.results[1] * 1000.0, "mH", true,
                kSpecLdH * 1000.0, kSpecLTolPct };
    rows[2] = { "Lq", motorProfiler_.result_valid[2],
                motorProfiler_.results[2] * 1000.0, "mH", true,
                kSpecLqH * 1000.0, kSpecLTolPct };
    rows[3] = { "Kp Id", motorProfiler_.result_valid[3],
                motorProfiler_.results[3], "", false, 0, 0 };
    rows[4] = { "Ki Id", motorProfiler_.result_valid[4],
                motorProfiler_.results[4], "", false, 0, 0 };
    rows[5] = { "Kp Iq", motorProfiler_.result_valid[5],
                motorProfiler_.results[5], "", false, 0, 0 };
    rows[6] = { "Ki Iq", motorProfiler_.result_valid[6],
                motorProfiler_.results[6], "", false, 0, 0 };
    calibResultTable(rows, 7);

    // Final result bar chart -- Rs (Ohm) and Ld/Lq (mH) measured vs spec.
    if (motorProfiler_.result_valid[0] || motorProfiler_.result_valid[1] ||
        motorProfiler_.result_valid[2]) {
        if (ImPlot::BeginPlot("Measured vs spec", ImVec2(-1, 170.0f),
                              ImPlotFlags_NoMenus)) {
            static const char* kLabels[3] = { "Rs (Ohm)", "Ld (mH)", "Lq (mH)" };
            static const double kPos[3] = { 0.0, 1.0, 2.0 };
            const double meas[3] = {
                motorProfiler_.result_valid[0] ? motorProfiler_.results[0] : 0.0,
                motorProfiler_.result_valid[1] ? motorProfiler_.results[1]*1000.0 : 0.0,
                motorProfiler_.result_valid[2] ? motorProfiler_.results[2]*1000.0 : 0.0,
            };
            const double spec[3] = {
                kSpecRsOhm, kSpecLdH * 1000.0, kSpecLqH * 1000.0 };
            ImPlot::SetupAxes("parameter", "value",
                              ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            ImPlot::SetupAxisTicks(ImAxis_X1, kPos, 3, kLabels);
            ImPlot::PushStyleColor(ImPlotCol_Fill, ImVec4(0.55f,0.55f,0.60f,0.7f));
            ImPlot::PlotBars("spec", kPos, spec, 3, 0.30);
            ImPlot::PopStyleColor();
            ImPlot::PushStyleColor(ImPlotCol_Fill, ImVec4(0.40f,0.78f,0.95f,0.85f));
            double posMeas[3] = { 0.32, 1.32, 2.32 };
            ImPlot::PlotBars("measured", posMeas, meas, 3, 0.30);
            ImPlot::PopStyleColor();
            ImPlot::EndPlot();
        }
    }

    ImGui::Spacing();
    ImGui::BeginDisabled(!g.canSend || !motorProfiler_.result_valid[3]);
    if (ImGui::Button("Apply PI gains -> Test 3", ImVec2(190, 0))) {
        // Copy derived current-loop gains into the live pi.* ids (RAM only).
        auto applyF = [&](uint16_t id, float v) {
            uint32_t b; std::memcpy(&b, &v, 4);
            applyMotorConfigBlobSet(id, b);
        };
        applyF(10, motorProfiler_.results[3]);   // pi.kp_id
        applyF(11, motorProfiler_.results[4]);   // pi.ki_id
        applyF(12, motorProfiler_.results[5]);   // pi.kp_iq
        applyF(13, motorProfiler_.results[6]);   // pi.ki_iq
        for (auto& kv : calibSync_) kv.second.hasStaged = false;
        calibStatus_ = "derived PI gains applied to RAM (ids 10..13)";
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!g.canSend || !g.motorIdle);
    if (ImGui::Button("Save to flash##mach", ImVec2(140, 0))) {
        sendMotorConfigParam(/*SAVE_ALL*/ 2, 0, 0);
        motorBlobAuthoritativeUntil_ = 0.0;
        calibStatus_ = "PARAM SAVE_ALL sent";
    }
    ImGui::EndDisabled();

    endTestPanel();
}

// ==========================================================================
// Test 3 -- PI tuning + step response.
// ==========================================================================
void MainUi::drawCalibPiTuning()
{
    CalibGate g; calibPreconditions(g);

    beginTestPanel("Test 3 - PI regulator tuning + step response",
                   "Tunes the current-loop and speed-loop PI gains. Each gain "
                   "is editable live (RAM only) so you can A/B sets without "
                   "re-flashing. The step-response acquisition reuses the "
                   "Triggered Capture pipeline -- see the note in the Run zone.",
                   calibPillPi_);

    // (B) preconditions
    ImGui::TextDisabled("Preconditions");
    calibPreconditionRow("transport open", g.canSend);
    bool diagOk = calibPreconditionRow("diag mode active", g.inDiag);
    if (!g.inDiag) {
        ImGui::SameLine();
        ImGui::TextDisabled("(use Take control in the header bar)");
    }
    calibPreconditionRow("FSM idle", g.motorIdle);
    calibPreconditionRow("no PWM fault", (g.faultMask & 0x1FFu) == 0);
    ImGui::Checkbox("safe to energise / shaft area clear",
                    &calibPiAreaClear_);
    (void)diagOk;
    ImGui::Separator();

    // (C) parameters -- the 6 PI gain fields, via the range-field component.
    ImGui::TextDisabled("Current loop (Id / Iq)");
    if (ImGui::SmallButton("Compute from Test 2")) {
        // Roadmap formulas: Kp = bandwidth * L, Ki = bandwidth * Rs / 10000.
        const float bw = motorProfilerBandwidthHz_;
        const float rs = calibBlobValueF32(20);
        const float ld = calibBlobValueF32(21);
        const float lq = calibBlobValueF32(22);
        auto stage = [&](uint16_t id, float v) {
            CalibFieldSync& s = calibSync_[id];
            s.staged = v; s.hasStaged = true;
            s.state = CalibSyncState::Edited;
        };
        if (rs > 0.0f && ld > 0.0f && lq > 0.0f) {
            stage(10, bw * ld);                 // Id Kp
            stage(11, bw * rs / 10000.0f);      // Id Ki
            stage(12, bw * lq);                 // Iq Kp
            stage(13, bw * rs / 10000.0f);      // Iq Ki
            calibStatus_ = "PI gains computed from Test 2 (staged, not sent)";
        } else {
            calibStatus_ = "Compute from Test 2: run Test 2 first";
        }
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Kp = bandwidth * L, Ki = bandwidth * Rs / 10000");

    for (uint16_t id : {uint16_t(10), uint16_t(11), uint16_t(12), uint16_t(13)}) {
        const CalibParamMeta* m = nullptr;
        for (int i = 0; i < kCalibParamMetaCount; ++i)
            if (kCalibParamMeta[i].id == id) { m = &kCalibParamMeta[i]; break; }
        if (!m) continue;
        calibRangeField(*m, calibBlobValueF32(id), calibSync_[id], true);
    }

    ImGui::Spacing();
    ImGui::TextDisabled("Speed loop");
    for (uint16_t id : {uint16_t(14), uint16_t(15)}) {
        const CalibParamMeta* m = nullptr;
        for (int i = 0; i < kCalibParamMetaCount; ++i)
            if (kCalibParamMeta[i].id == id) { m = &kCalibParamMeta[i]; break; }
        if (!m) continue;
        calibRangeField(*m, calibBlobValueF32(id), calibSync_[id], true);
    }

    ImGui::Spacing();
    ImGui::TextDisabled("Step test config");
    // Step-loop selector. The Iq current step (driverMode 3, CURRENT) is
    // now reachable in the motor FW and is the DIRECT excitation for the
    // inner current PI -- prefer it. The speed step is kept as a fallback
    // for cases where a current step is undesirable (e.g. rotor must not
    // produce torque). calibPiLoopSel_: 0 = Iq current step, 1 = speed step.
    ImGui::RadioButton("Iq current step (direct)", &calibPiLoopSel_, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Speed step (fallback)", &calibPiLoopSel_, 1);
    const bool iqStep = (calibPiLoopSel_ == 0);
    if (iqStep) {
        ImGui::TextWrapped(
            "Iq current step: the motor enters CURRENT mode (driverMode 3) "
            "and the torque-axis set-point is stepped directly. This is the "
            "cleanest excitation for the inner current PI. The motor FW "
            "hard-clamps Iq to the continuous-rated current, so the step "
            "values below cannot overdrive the windings.");
        ImGui::SetNextItemWidth(120.0f);
        ImGui::InputFloat("Step from (A)", &calibPiStepFrom_, 0.0f, 0.0f, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120.0f);
        ImGui::InputFloat("Step to (A)", &calibPiStepTo_, 0.0f, 0.0f, "%.2f");
        // Soft host-side hint clamp (FW clamps authoritatively anyway).
        calibPiStepFrom_ = std::clamp(calibPiStepFrom_, -4.0f, 4.0f);
        calibPiStepTo_   = std::clamp(calibPiStepTo_,   -4.0f, 4.0f);
    } else {
        ImGui::TextWrapped(
            "Speed step (fallback): a SPEED set-point step. The inner "
            "current loop is excited indirectly via the speed PI. Use the "
            "Iq current step above for a direct current-loop measurement.");
        ImGui::SetNextItemWidth(120.0f);
        ImGui::InputFloat("Step from (rpm)", &calibPiStepFrom_, 0.0f, 0.0f, "%.0f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120.0f);
        ImGui::InputFloat("Step to (rpm)", &calibPiStepTo_, 0.0f, 0.0f, "%.0f");
    }
    ImGui::SetNextItemWidth(200.0f);
    ImGui::SliderFloat("Step duration (ms)", &calibPiStepMs_, 50.0f, 2000.0f, "%.0f");
    ImGui::SetNextItemWidth(200.0f);
    ImGui::SliderFloat("Capture rate (us)", &calibPiCaptureUs_, 100.0f, 1000.0f, "%.0f");
    ImGui::Separator();

    // (D) run strip
    ImGui::TextDisabled("Run");
    ImGui::BeginDisabled(!g.canSend);
    if (ImGui::Button("Apply gains", ImVec2(120, 0))) {
        // Push every staged current/speed gain to RAM.
        for (uint16_t id : {uint16_t(10), uint16_t(11), uint16_t(12),
                            uint16_t(13), uint16_t(14), uint16_t(15)}) {
            CalibFieldSync& s = calibSync_[id];
            if (s.state == CalibSyncState::Edited) {
                uint32_t b; std::memcpy(&b, &s.staged, 4);
                if (applyMotorConfigBlobSet(id, b)) {
                    s.state = CalibSyncState::Sent;
                    s.sentValue = s.staged;
                    s.sentAt = ImGui::GetTime();
                }
            }
        }
        calibStatus_ = "PI gains applied to RAM";
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    // ------------------------------------------------------------------
    // Step test: a real, working acquisition.
    //   1. Command the motor to the "from" speed (sendMotorProxy START).
    //   2. Launch the /capture HTTP worker -- it samples i_SDQ.d/q +
    //      omega on the device for `duration_ms`, with pre-trigger so the
    //      pre-step baseline is recorded.
    //   3. ~120 ms into the capture window, command the speed step to the
    //      "to" value (sendMotorProxy SET_SPEED). The step lands inside
    //      the capture window; pre_pct keeps the baseline.
    //   4. drainCalibCapture decodes the body, runs the FFT + metrics.
    // The /capture endpoint requires the TCP or Wi-Fi-AP transport.
    const bool httpOk = (transportKind_ == TransportKind::Tcp ||
                         transportKind_ == TransportKind::WifiAp);
    const bool stepWorkerBusy =
        calibCapRunning_.load(std::memory_order_acquire) &&
        calibCapTarget_ == &calibStep_;
    const bool stepBusy = (calibStep_.phase == CalibCapPhase::Stepping ||
                           calibStep_.phase == CalibCapPhase::Fetching ||
                           stepWorkerBusy);
    std::vector<size_t> piWatchIdx;
    std::string piCaptureError;
    const std::vector<const char*> piRequired = iqStep
        ? std::vector<const char*>{
              "i_SDQ_desired.q_axis", "i_SDQ.d_axis", "i_SDQ.q_axis" }
        : std::vector<const char*>{
              "omega_desired_mech_rpm", "i_SDQ.d_axis", "i_SDQ.q_axis" };
    const bool piCaptureVarsOk =
        calibEnsureMotorCaptureVars(piRequired, piWatchIdx, piCaptureError);
    if (!piCaptureVarsOk) {
        ImGui::TextColored(ImVec4(0.95f,0.75f,0.25f,1.0f),
                           "Capture variables: %s", piCaptureError.c_str());
    } else {
        ImGui::TextDisabled("Capture variables auto-selected from Variables: %s, %s, %s",
                            watches_[piWatchIdx[0]].name.c_str(),
                            watches_[piWatchIdx[1]].name.c_str(),
                            watches_[piWatchIdx[2]].name.c_str());
        const uint32_t samplesFit = static_cast<uint32_t>(
            (32u * 1024u) / static_cast<uint32_t>(
                std::max<size_t>(1, piWatchIdx.size()) * 4u));
        uint32_t previewPeriodUs = static_cast<uint32_t>(calibPiCaptureUs_);
        if (samplesFit > 0) {
            const uint32_t needUs = static_cast<uint32_t>(
                std::ceil((static_cast<double>(calibPiStepMs_) * 1000.0) /
                          static_cast<double>(samplesFit)));
            previewPeriodUs = std::max(previewPeriodUs, needUs);
        }
        if (previewPeriodUs > static_cast<uint32_t>(calibPiCaptureUs_) + 1u) {
            ImGui::TextColored(ImVec4(0.95f,0.75f,0.25f,1.0f),
                               "Capture period auto-raised to %u us so %.0f ms fits in 32 KB",
                               previewPeriodUs, calibPiStepMs_);
        }
    }
    ImGui::BeginDisabled(!g.canSend || !httpOk || !calibPiAreaClear_ ||
                         !piCaptureVarsOk ||
                         stepBusy);
    if (ImGui::Button("Run step test", ImVec2(140, 0))) {
        calibStep_ = CalibCapRun{};
        calibStep_.phase       = CalibCapPhase::Stepping;
        calibStep_.startedAt   = ImGui::GetTime();
        const int capDurationMs = static_cast<int>(calibPiStepMs_);
        const size_t capSlots = std::max<size_t>(1, piWatchIdx.size());
        const uint32_t samplesFit = static_cast<uint32_t>(
            (32u * 1024u) / static_cast<uint32_t>(capSlots * 4u));
        uint16_t effectivePeriodUs = static_cast<uint16_t>(calibPiCaptureUs_);
        if (samplesFit > 0) {
            const uint32_t needUs = static_cast<uint32_t>(
                std::ceil((static_cast<double>(capDurationMs) * 1000.0) /
                          static_cast<double>(samplesFit)));
            effectivePeriodUs = static_cast<uint16_t>(
                std::clamp<uint32_t>(
                    std::max<uint32_t>(static_cast<uint32_t>(effectivePeriodUs), needUs),
                    100u, 65535u));
        }
        calibStep_.periodUs    = static_cast<float>(effectivePeriodUs);
        calibStep_.isSpeedLoop = !iqStep;
        calibStep_.synthSetpoint = true;
        calibStep_.commandFrom = calibPiStepFrom_;
        calibStep_.commandTo = calibPiStepTo_;
        calibStep_.stepDelayMs = 120.0f;
        // 1. Pre-position the motor at the "from" set-point.
        //    For Iq, the device-side /motor/current-step-capture endpoint
        //    applies the current pulse after the capture is armed, so there
        //    is no host/SSH timing race. Speed fallback still uses the old
        //    host-commanded step.
        if (!iqStep)
            sendMotorProxy(1 /* START */, calibPiStepFrom_);
        std::string q = calibBuildCaptureQueryFromWatches(
            piWatchIdx, capDurationMs, effectivePeriodUs, 25, 75);
        if (iqStep) {
            char prefix[160];
            const int delayMs = 120;
            const int widthMs = std::max(20, static_cast<int>(calibPiStepMs_) - delayMs - 20);
            calibStep_.pulseWidthMs = static_cast<float>(widthMs);
            std::snprintf(prefix, sizeof(prefix),
                          "/motor/current-step-capture?amp=%.6g&delay_ms=%d&width_ms=%d&",
                          calibPiStepTo_, delayMs, widthMs);
            q = std::string(prefix) + q;
        } else {
            calibStep_.pulseWidthMs = 0.0f;
        }
        if (calibLaunchCapture(calibStep_, q)) {
            calibStep_.phase  = CalibCapPhase::Fetching;
            calibStep_.status = iqStep ? "device-side Iq pulse + capture..."
                                       : "stepping + capturing...";
            calibPillPi_ = CalibPill::Running;
            calibStatus_ = iqStep ? "step test: app_dd will pulse Iq inside capture"
                                  : "step test: motor -> from-speed, capture armed";
        } else {
            calibStep_.phase = CalibCapPhase::Failed;
            calibStatus_ = calibStep_.error;
            calibPillPi_ = CalibPill::Fail;
        }
    }
    ImGui::EndDisabled();
    if (!httpOk && ImGui::IsItemHovered())
        ImGui::SetTooltip("Step capture uses app_dd's /capture HTTP endpoint "
                          "-- connect via TCP or Wi-Fi AP (SLCAN has no "
                          "app_dd to talk to).");
    else if (!calibPiAreaClear_ && ImGui::IsItemHovered())
        ImGui::SetTooltip("confirm the shaft area is clear first");
    ImGui::SameLine();
    ImGui::BeginDisabled(!g.canSend);
    if (ImGui::Button("Abort##pi", ImVec2(90, 0))) {
        // STOP via the proxy idles the motor for both step modes (the FW
        // leaves CURRENT mode on a driverMode-0 frame). Belt-and-braces:
        // for an Iq step also push a direct mode-0 frame so the inverter
        // is disarmed even if the proxy path is unavailable.
        sendMotorProxy(0 /* STOP */, 0.0f);
        if (calibStep_.isSpeedLoop == false)
            sendMotorCurrentStep(0.0f);
        if (stepWorkerBusy) {
            calibStep_.abortRequested = true;
            calibStep_.phase  = CalibCapPhase::Fetching;
            calibStep_.error  = "aborted by operator";
            calibStep_.status = "aborting -- waiting for capture worker";
        } else if (calibStep_.phase == CalibCapPhase::Stepping ||
                   calibStep_.phase == CalibCapPhase::Fetching) {
            calibStep_.abortRequested = true;
            calibStep_.phase  = CalibCapPhase::Failed;
            calibStep_.error  = "aborted by operator";
            calibStep_.status = "aborted";
        }
        calibStatus_ = "STOP sent (abort step test)";
    }
    ImGui::EndDisabled();

    // Step-test state machine driver. Once the capture worker is running,
    // command the step ~120 ms in so it lands inside the window.
    const double nowT = ImGui::GetTime();
    if (calibStep_.isSpeedLoop &&
        calibStep_.phase == CalibCapPhase::Fetching &&
        calibStep_.stepAt == 0.0 &&
        nowT - calibStep_.startedAt > 0.12) {
        sendMotorProxy(4 /* SET_SPEED */, calibPiStepTo_);
        calibStatus_ = "step test: speed step commanded";
        calibStep_.stepAt = nowT;
        calibStep_.stepDelayMs =
            static_cast<float>((nowT - calibStep_.startedAt) * 1000.0);
        calibStep_.status = "step commanded -- waiting for capture body";
    }
    // Drain a finished worker (decodes body, FFT, metrics).
    const CalibCapPhase phaseBeforeDrain = calibStep_.phase;
    calibDrainCapture();
    if (calibStep_.phase == CalibCapPhase::Done)
        calibPillPi_ = CalibPill::Pass;
    else if (calibStep_.phase == CalibCapPhase::Failed)
        calibPillPi_ = CalibPill::Fail;
    // On the edge into Done/Failed, idle the motor so it does not keep
    // holding the "to" current/speed after the capture window closes.
    if (phaseBeforeDrain == CalibCapPhase::Fetching &&
        (calibStep_.phase == CalibCapPhase::Done ||
         calibStep_.phase == CalibCapPhase::Failed)) {
        sendMotorProxy(0 /* STOP */, 0.0f);
        if (calibStep_.isSpeedLoop == false)
            sendMotorCurrentStep(0.0f);
    }

    // Progress / status line.
    if (stepBusy) {
        const double elapsed = std::max(0.0, nowT - calibStep_.startedAt);
        const double expected = std::max(1.0, calibStep_.periodUs *
                                              (32.0 * 1024.0 / (3.0 * 4.0)) /
                                              1000000.0 + 2.5);
        const float f = static_cast<float>(std::clamp(elapsed / expected,
                                                      0.02, 0.98));
        const uint64_t rx = captureHttpBytesRx_.load(std::memory_order_relaxed);
        const uint64_t exp = captureHttpBytesExpected_.load(std::memory_order_relaxed);
        char label[180];
        if (exp > 0) {
            std::snprintf(label, sizeof(label), "%s  %.1fs  %llu/%llu B",
                          calibStep_.status.c_str(), elapsed,
                          static_cast<unsigned long long>(rx),
                          static_cast<unsigned long long>(exp));
        } else {
            std::snprintf(label, sizeof(label), "%s  %.1fs",
                          calibStep_.status.c_str(), elapsed);
        }
        ImGui::ProgressBar(f, ImVec2(-FLT_MIN, 0), label);
    } else if (calibStep_.phase == CalibCapPhase::Done) {
        ImGui::ProgressBar(1.0f, ImVec2(-FLT_MIN, 0), "step capture done");
    } else if (calibStep_.phase == CalibCapPhase::Failed) {
        ImGui::ProgressBar(0.0f, ImVec2(-FLT_MIN, 0),
                           calibStep_.error.empty() ? "failed"
                                                    : calibStep_.error.c_str());
    } else {
        ImGui::ProgressBar(0.0f, ImVec2(-FLT_MIN, 0), "idle");
    }
    calibStatusLine();
    ImGui::Separator();

    // (E) results -- step response, FFT spectrum, metric bar chart.
    drawCalibStepResults(calibStep_);

    ImGui::BeginDisabled(!g.canSend || !g.motorIdle);
    if (ImGui::Button("Save to flash##pi", ImVec2(140, 0))) {
        sendMotorConfigParam(/*SAVE_ALL*/ 2, 0, 0);
        motorBlobAuthoritativeUntil_ = 0.0;
        calibStatus_ = "PARAM SAVE_ALL sent";
    }
    ImGui::EndDisabled();

    endTestPanel();
}

// --------------------------------------------------------------------------
// Shared step-response result renderer (Test 3 graphs 1/2 + metric bars).
// --------------------------------------------------------------------------
void MainUi::drawCalibStepResults(CalibCapRun& run)
{
    CalibPlotInputScope plotInput;

    ImGui::TextDisabled("Results -- step response");

    const bool haveData = (run.phase == CalibCapPhase::Done &&
                           run.t.size() > 1);

    auto drawTimePlot = [&]() {
        ImGui::TextDisabled("|I| current-vector amplitude and commanded step");
        const float w = ImGui::GetContentRegionAvail().x;
        calibFitPlotOnDataChange("##pi_step_fit", haveData ? (int)run.t.size() : 0);
        if (ImPlot::BeginPlot("Step response (time domain)",
                              ImVec2(-1, calibPlotHeight(w)))) {
            ImPlot::SetupAxes("time (ms)", "current / command",
                              ImPlotAxisFlags_None, ImPlotAxisFlags_None);
            if (haveData) {
                size_t peakIdx = 0;
                float peakVal = run.amplitude.empty() ? 0.0f : run.amplitude[0];
                for (size_t i = 1; i < run.amplitude.size(); ++i) {
                    if (run.amplitude[i] > peakVal) {
                        peakVal = run.amplitude[i];
                        peakIdx = i;
                    }
                }
                ImPlot::PushStyleColor(ImPlotCol_Line,
                                       ImVec4(0.40f,0.78f,0.95f,1.0f));
                ImPlot::PlotLine("|I| current vector (A)", run.t.data(),
                                 run.amplitude.data(),
                                 static_cast<int>(run.amplitude.size()));
                ImPlot::PopStyleColor();
                ImPlot::PushStyleColor(ImPlotCol_Line,
                                       ImVec4(0.85f,0.70f,0.40f,0.9f));
                ImPlot::PlotLine("set-point", run.t.data(),
                                 run.setpoint.data(),
                                 static_cast<int>(run.setpoint.size()));
                ImPlot::PopStyleColor();
                if (run.triggerIdx > 0 && run.triggerIdx < run.t.size()) {
                    double xs[1] = { run.t[run.triggerIdx] };
                    ImPlot::PushStyleColor(ImPlotCol_Line,
                                           ImVec4(0.30f,0.85f,0.95f,0.70f));
                    ImPlot::PlotInfLines("step instant", xs, 1);
                    ImPlot::PopStyleColor();
                }
                if (peakIdx < run.t.size()) {
                    const double px[1] = { run.t[peakIdx] };
                    const double py[1] = { peakVal };
                    ImPlot::PushStyleColor(ImPlotCol_MarkerOutline,
                                           ImVec4(1.0f,0.85f,0.35f,1.0f));
                    ImPlot::PushStyleColor(ImPlotCol_MarkerFill,
                                           ImVec4(1.0f,0.85f,0.35f,0.85f));
                    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 6.0f);
                    ImPlot::PlotScatter("peak |I|", px, py, 1);
                    ImPlot::PopStyleColor(2);
                    ImPlot::Annotation(px[0], py[0], ImVec4(0,0,0,0),
                                       ImVec2(8, -18), true,
                                       "peak %.3f A", peakVal);
                }
                if (ImPlot::IsPlotHovered() && !run.t.empty()) {
                    const ImPlotPoint mp = ImPlot::GetPlotMousePos();
                    size_t nearest = 0;
                    double best = std::fabs(static_cast<double>(run.t[0]) - mp.x);
                    for (size_t i = 1; i < run.t.size(); ++i) {
                        const double d = std::fabs(static_cast<double>(run.t[i]) - mp.x);
                        if (d < best) { best = d; nearest = i; }
                    }
                    const float amp = nearest < run.amplitude.size()
                        ? run.amplitude[nearest] : 0.0f;
                    const float cmd = nearest < run.setpoint.size()
                        ? run.setpoint[nearest] : 0.0f;
                    ImGui::BeginTooltip();
                    ImGui::Text("t %.2f ms", run.t[nearest]);
                    ImGui::Text("|I| %.4f A", amp);
                    ImGui::Text("set-point %.4f A", cmd);
                    ImGui::Separator();
                    ImGui::Text("peak %.4f A at %.2f ms",
                                peakVal, peakIdx < run.t.size() ? run.t[peakIdx] : 0.0f);
                    ImGui::EndTooltip();
                }
            } else {
                ImPlot::SetupAxesLimits(0.0, 1.0, 0.0, 1.0, ImPlotCond_Once);
                ImPlot::Annotation(0.5, 0.5, ImVec4(0,0,0,0), ImVec2(0,0), false,
                                   "run a step test to populate");
            }
            ImPlot::EndPlot();
        }
    };

    auto drawFftPlot = [&]() {
        ImGui::TextDisabled("FFT of |I| after the step, DC bin hidden");
        const bool haveFft = haveData && run.fftFreq.size() > 1;
        const float w = ImGui::GetContentRegionAvail().x;
        calibFitPlotOnDataChange("##pi_fft_fit", haveFft ? (int)run.fftFreq.size() : 0);
        if (ImPlot::BeginPlot("Frequency spectrum (FFT of |I|)",
                              ImVec2(-1, calibPlotHeight(w)))) {
            ImPlot::SetupAxes("frequency (Hz)", "magnitude",
                              ImPlotAxisFlags_None, ImPlotAxisFlags_None);
            if (haveFft) {
                ImPlot::PushStyleColor(ImPlotCol_Line,
                                       ImVec4(0.55f,0.85f,0.55f,1.0f));
                ImPlot::PlotLine("spectrum",
                                 run.fftFreq.data() + 1, run.fftMag.data() + 1,
                                 static_cast<int>(run.fftFreq.size() - 1));
                ImPlot::PopStyleColor();
            } else {
                ImPlot::SetupAxesLimits(0.0, 1.0, 0.0, 1.0, ImPlotCond_Once);
                ImPlot::Annotation(0.5, 0.5, ImVec4(0,0,0,0), ImVec2(0,0), false,
                                   "FFT populates from the captured signal");
            }
            ImPlot::EndPlot();
        }
    };

    const float availW = ImGui::GetContentRegionAvail().x;
    if (availW > 760.0f &&
        ImGui::BeginTable("##pi_result_plot_grid", 2,
                          ImGuiTableFlags_SizingStretchSame |
                          ImGuiTableFlags_PadOuterX)) {
        ImGui::TableNextColumn();
        drawTimePlot();
        ImGui::TableNextColumn();
        drawFftPlot();
        ImGui::EndTable();
    } else {
        drawTimePlot();
        drawFftPlot();
    }

    // -- Metric bar chart + verdict --------------------------------------
    const CalibStepMetrics& m = run.metrics;
    ImGui::TextDisabled("Numeric PI quality metrics from the captured step");
    calibFitPlotOnDataChange("##pi_metrics_fit", m.valid ? 1 : 0);
    if (ImPlot::BeginPlot("PI tuning metrics", ImVec2(-1, 230.0f),
                          ImPlotFlags_NoLegend)) {
        // Four normalised bars: overshoot %, rise ms, settle ms, ss-err %.
        // They live on a shared 0..N axis with text tick labels.
        static const char* kLabels[4] = {
            "overshoot %", "rise ms", "settle ms", "ss-err %" };
        static const double kPos[4] = { 0.0, 1.0, 2.0, 3.0 };
        const double vals[4] = {
            m.valid ? m.overshootPct : 0.0,
            m.valid ? m.riseMs       : 0.0,
            m.valid ? m.settleMs     : 0.0,
            m.valid ? m.ssErrPct     : 0.0,
        };
        ImPlot::SetupAxes("metric", "value",
                          ImPlotAxisFlags_None, ImPlotAxisFlags_None);
        ImPlot::SetupAxisTicks(ImAxis_X1, kPos, 4, kLabels);
        ImPlot::PushStyleColor(ImPlotCol_Fill, ImVec4(0.40f,0.78f,0.95f,0.8f));
        ImPlot::PlotBars("metrics", kPos, vals, 4, 0.55);
        ImPlot::PopStyleColor();
        ImPlot::EndPlot();
    }

    if (m.valid) {
        ImGui::Text("baseline %.3f  ->  target %.3f   peak %.3f",
                    m.baseline, m.target, m.peak);
        ImGui::Text("rise %.2f ms | overshoot %.1f %% | settling %.2f ms | "
                    "ss-err %.2f %% | noise RMS %.4f",
                    m.riseMs, m.overshootPct, m.settleMs, m.ssErrPct,
                    m.noiseRms);
        // Advisory verdict.
        const bool good = (m.overshootPct < 20.0f) && (m.settleMs < 50.0f);
        if (good)
            ImGui::TextColored(ImVec4(0.45f,0.85f,0.45f,1.0f),
                               "verdict PASS -- step response within band");
        else if (m.overshootPct >= 20.0f)
            ImGui::TextColored(ImVec4(0.95f,0.75f,0.25f,1.0f),
                               "verdict WARN -- overshoot high: lower Kp or "
                               "raise Ki");
        else
            ImGui::TextColored(ImVec4(0.95f,0.75f,0.25f,1.0f),
                               "verdict WARN -- slow settling: raise Kp");
    } else {
        ImGui::TextDisabled("rise/overshoot/settling/ss-error populate once a "
                            "step capture is acquired.");
    }
}

// ==========================================================================
// Test 4 -- Inertia (J) + friction + future.
// ==========================================================================
void MainUi::drawCalibInertia()
{
    CalibGate g; calibPreconditions(g);

    beginTestPanel("Test 4 - Inertia (J) calibration + friction",
                   "Determines the rotor + load moment of inertia J and "
                   "viscous friction B -- needed to tune the outer speed loop. "
                   "Tier A: manual entry of J/B/lambda + a host-side coast-down "
                   "estimate. Tier B: a firmware identification routine "
                   "(motor FW roadmap Phase 6+, not yet available).",
                   calibPillInertia_);

    // (B) preconditions
    ImGui::TextDisabled("Preconditions");
    calibPreconditionRow("transport open", g.canSend);
    calibPreconditionRow("diag mode active", g.inDiag);
    calibPreconditionRow("FSM idle", g.motorIdle);
    calibPreconditionRow("no PWM fault", (g.faultMask & 0x1FFu) == 0);
    ImGui::Checkbox("safe to spin the motor", &calibInertiaSafeSpin_);
    ImGui::Separator();

    // (C) parameters
    ImGui::TextDisabled("Parameters");
    ImGui::SetNextItemWidth(200.0f);
    ImGui::SliderFloat("Coast-down start speed (rpm)", &calibInertiaStartRpm_,
                       100.0f, 800.0f, "%.0f");
    ImGui::SetNextItemWidth(200.0f);
    ImGui::SliderFloat("Capture rate (us)", &calibInertiaCaptureUs_,
                       200.0f, 1000.0f, "%.0f");
    ImGui::Spacing();
    for (uint16_t id : {uint16_t(24), uint16_t(25), uint16_t(23)}) {
        const CalibParamMeta* m = nullptr;
        for (int i = 0; i < kCalibParamMetaCount; ++i)
            if (kCalibParamMeta[i].id == id) { m = &kCalibParamMeta[i]; break; }
        if (!m) continue;
        calibRangeField(*m, calibBlobValueF32(id), calibSync_[id], true);
    }
    ImGui::SameLine();
    // lambda_pm helper -- derive from steady-state V_q / omega.
    ImGui::BeginDisabled(!g.canSend);
    if (ImGui::Button("Estimate lambda_pm")) {
        // lambda = (V_q - Rs*I_q) / omega_elec. With I_q ~ 0 in speed mode
        // and I_d = 0: lambda ~= V_q / omega_elec. We use the live telemetry.
        const float vq    = lastMotorTelemetry_.voltage_q;
        const float rpm   = lastMotorTelemetry_.rpm;
        const float omega = rpm * 2.0f * 3.14159265f / 60.0f;
        if (std::fabs(omega) > 1.0f) {
            const float lambda = vq / omega;
            uint32_t b; std::memcpy(&b, &lambda, 4);
            if (applyMotorConfigBlobSet(23, b)) {
                calibSync_[23].hasStaged = false;
                char note[80];
                std::snprintf(note, sizeof(note),
                              "lambda_pm estimated %.5f V*s/rad (RAM)", lambda);
                calibStatus_ = note;
            }
        } else {
            calibStatus_ = "lambda_pm: spin the motor first (omega ~ 0)";
        }
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("lambda = (V_q - Rs*I_q) / omega_elec; spin the motor "
                          "in speed mode with I_d = 0 first");
    ImGui::Separator();

    // (D) run strip -- Tier A coast-down is a real, working acquisition.
    ImGui::TextDisabled("Run");
    // Coast-down: spin the motor up, command STOP, and capture omega decay
    // via the same /capture HTTP pipeline Test 3 uses. The fit happens
    // host-side in calibDrainCapture / the renderer below. Needs no new
    // firmware -- it reuses the speed command + the capture endpoint.
    const bool httpOk = (transportKind_ == TransportKind::Tcp ||
                         transportKind_ == TransportKind::WifiAp);
    const bool coastWorkerBusy =
        calibCapRunning_.load(std::memory_order_acquire) &&
        calibCapTarget_ == &calibCoast_;
    const bool coastBusy = (calibCoast_.phase == CalibCapPhase::Stepping ||
                            calibCoast_.phase == CalibCapPhase::Fetching ||
                            coastWorkerBusy);
    std::vector<size_t> coastWatchIdx;
    std::string coastCaptureError;
    const bool coastCaptureVarsOk = calibEnsureMotorCaptureVars(
        { "omega_desired_mech_rpm", "omega_actual_mech_rpm" },
        coastWatchIdx, coastCaptureError);
    if (!coastCaptureVarsOk) {
        ImGui::TextColored(ImVec4(0.95f,0.75f,0.25f,1.0f),
                           "Capture variables: %s", coastCaptureError.c_str());
    } else {
        ImGui::TextDisabled("Capture variables auto-selected from Variables: %s, %s",
                            watches_[coastWatchIdx[0]].name.c_str(),
                            watches_[coastWatchIdx[1]].name.c_str());
    }
    ImGui::BeginDisabled(!g.canSend || !httpOk || !calibInertiaSafeSpin_ ||
                         !coastCaptureVarsOk ||
                         coastBusy);
    if (ImGui::Button("Run coast-down", ImVec2(150, 0))) {
        calibCoast_ = CalibCapRun{};
        calibCoast_.phase       = CalibCapPhase::Stepping;
        calibCoast_.startedAt   = ImGui::GetTime();
        calibCoast_.periodUs    = calibInertiaCaptureUs_;
        calibCoast_.isSpeedLoop = true;
        // Spin the motor up to the start speed.
        sendMotorProxy(1 /* START */, calibInertiaStartRpm_);
        // Long enough to see the full decay; pre-trigger 15 %.
        const int durMs = std::max(800, static_cast<int>(calibInertiaStartRpm_) * 4);
        const std::string q = calibBuildCaptureQueryFromWatches(
            coastWatchIdx, durMs,
            static_cast<uint16_t>(calibInertiaCaptureUs_), 15, 85);
        if (calibLaunchCapture(calibCoast_, q)) {
            calibCoast_.phase  = CalibCapPhase::Fetching;
            calibCoast_.status = "spinning up + capturing...";
            calibPillInertia_ = CalibPill::Running;
            calibStatus_ = "coast-down: motor spin-up, capture armed";
        } else {
            calibCoast_.phase = CalibCapPhase::Failed;
            calibStatus_ = calibCoast_.error;
            calibPillInertia_ = CalibPill::Fail;
        }
    }
    ImGui::EndDisabled();
    if (!httpOk && ImGui::IsItemHovered())
        ImGui::SetTooltip("Coast-down capture uses app_dd's /capture HTTP "
                          "endpoint -- connect via TCP or Wi-Fi AP.");
    ImGui::SameLine();
    if (ImGui::Button("Abort##coast", ImVec2(90, 0))) {
        sendMotorProxy(0 /* STOP */, 0.0f);
        if (coastWorkerBusy) {
            calibCoast_.abortRequested = true;
            calibCoast_.phase  = CalibCapPhase::Fetching;
            calibCoast_.error  = "aborted by operator";
            calibCoast_.status = "aborting -- waiting for capture worker";
        } else if (coastBusy) {
            calibCoast_.abortRequested = true;
            calibCoast_.phase  = CalibCapPhase::Failed;
            calibCoast_.error  = "aborted by operator";
            calibCoast_.status = "aborted";
        }
        calibStatus_ = "STOP sent (abort coast-down)";
    }
    ImGui::SameLine();
    // Tier B -- firmware-pending.
    ImGui::BeginDisabled(true);   // TODO motor FW Phase 6: PROF_TEST_* inertia
    ImGui::Button("Run J/B identification", ImVec2(190, 0));
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("needs a motor FW profiler test for inertia "
                          "(roadmap Phase 6) -- not yet in app_motor.h");
    ImGui::SameLine();
    ImGui::TextDisabled("(Tier B -- motor FW Phase 6+)");

    // Coast-down state machine: command STOP ~250 ms into the capture so
    // the pre-trigger window holds the steady spin-up baseline.
    const double nowT = ImGui::GetTime();
    if (calibCoast_.phase == CalibCapPhase::Fetching &&
        calibCoast_.stepAt == 0.0 &&
        nowT - calibCoast_.startedAt > 0.25) {
        sendMotorProxy(0 /* STOP */, 0.0f);
        calibCoast_.stepAt = nowT;
        calibCoast_.status = "STOP commanded -- capturing decay";
        calibStatus_ = "coast-down: STOP commanded";
    }
    calibDrainCapture();
    if (calibCoast_.phase == CalibCapPhase::Done)
        calibPillInertia_ = CalibPill::Pass;
    else if (calibCoast_.phase == CalibCapPhase::Failed)
        calibPillInertia_ = CalibPill::Fail;

    if (coastBusy) {
        const float f = (float)(0.5 + 0.5 * std::sin(nowT * 4.0));
        ImGui::ProgressBar(f, ImVec2(-FLT_MIN, 0), calibCoast_.status.c_str());
    } else if (calibCoast_.phase == CalibCapPhase::Done) {
        ImGui::ProgressBar(1.0f, ImVec2(-FLT_MIN, 0), "coast-down done");
    } else if (calibCoast_.phase == CalibCapPhase::Failed) {
        ImGui::ProgressBar(0.0f, ImVec2(-FLT_MIN, 0),
                           calibCoast_.error.empty() ? "failed"
                                                     : calibCoast_.error.c_str());
    } else {
        ImGui::ProgressBar(0.0f, ImVec2(-FLT_MIN, 0), "idle");
    }
    calibStatusLine();
    ImGui::Separator();

    // (E) results -- coast-down decay plot + exponential fit -> J, B.
    drawCalibCoastResults(calibCoast_);

    ImGui::BeginDisabled(!g.canSend || !g.motorIdle);
    if (ImGui::Button("Save to flash##inertia", ImVec2(140, 0))) {
        sendMotorConfigParam(/*SAVE_ALL*/ 2, 0, 0);
        motorBlobAuthoritativeUntil_ = 0.0;
        calibStatus_ = "PARAM SAVE_ALL sent";
    }
    ImGui::EndDisabled();

    endTestPanel();
}

// --------------------------------------------------------------------------
// Test 4 coast-down result renderer: decay curve + exponential fit.
//
// A free coast-down obeys J*dw/dt = -B*w, so w(t) = w0 * exp(-(B/J) t).
// We fit the time constant tau = J/B from the log-linear slope of the
// decay; that yields the RATIO B/J directly. The absolute split into J and
// B needs one more known quantity (drive torque or a second test) which
// the motor FW does not yet expose -- so the fit reports tau and B/J, and
// derives J from the friction-B field if the operator has entered one.
// --------------------------------------------------------------------------
void MainUi::drawCalibCoastResults(CalibCapRun& run)
{
    CalibPlotInputScope plotInput;

    ImGui::TextDisabled("Results -- coast-down decay");
    const bool haveData = (run.phase == CalibCapPhase::Done &&
                           run.measured.size() > 4);

    // Fit an exponential to the post-step decay.
    float tau = 0.0f, r2 = 0.0f, w0 = 0.0f;
    bool fitOk = false;
    if (haveData) {
        // Find the decay region: from the step instant to the end.
        size_t s0 = run.triggerIdx;
        if (s0 == 0 || s0 >= run.measured.size()) {
            // Fallback: first sample where omega starts dropping hard.
            float peak = 0.0f; size_t peakI = 0;
            for (size_t i = 0; i < run.measured.size(); ++i)
                if (run.measured[i] > peak) { peak = run.measured[i]; peakI = i; }
            s0 = peakI;
        }
        w0 = run.measured[s0];
        // Linear regression of ln(w) vs t over the region where w is a
        // meaningful fraction of w0 (avoid the noisy near-zero tail).
        double sx = 0, sy = 0, sxx = 0, sxy = 0; int nfit = 0;
        const float wMin = std::max(1.0f, w0 * 0.10f);
        for (size_t i = s0; i < run.measured.size(); ++i) {
            const float w = run.measured[i];
            if (w < wMin) break;
            const double x = run.t[i] - run.t[s0];
            const double y = std::log(static_cast<double>(w));
            sx += x; sy += y; sxx += x * x; sxy += x * y; ++nfit;
        }
        if (nfit >= 4) {
            const double denom = nfit * sxx - sx * sx;
            if (std::fabs(denom) > 1e-9) {
                const double slope = (nfit * sxy - sx * sy) / denom;
                const double inter = (sy - slope * sx) / nfit;
                if (slope < 0.0) {
                    tau = static_cast<float>(-1.0 / slope);  // ms
                    // R^2 of the log-linear fit.
                    double ssTot = 0, ssRes = 0;
                    const double ymean = sy / nfit;
                    int k = 0;
                    for (size_t i = s0; i < run.measured.size() && k < nfit;
                         ++i, ++k) {
                        const float w = run.measured[i];
                        if (w < wMin) break;
                        const double x = run.t[i] - run.t[s0];
                        const double y = std::log(static_cast<double>(w));
                        const double yhat = inter + slope * x;
                        ssTot += (y - ymean) * (y - ymean);
                        ssRes += (y - yhat) * (y - yhat);
                    }
                    r2 = (ssTot > 1e-12)
                         ? static_cast<float>(1.0 - ssRes / ssTot) : 0.0f;
                    fitOk = (tau > 0.0f);
                }
            }
        }
    }

    ImGui::TextDisabled("Measured speed decay and exponential fit");
    calibFitPlotOnDataChange("##coast_decay_fit", haveData ? (int)run.measured.size() : 0);
    const float plotW = ImGui::GetContentRegionAvail().x;
    if (ImPlot::BeginPlot("Coast-down decay",
                          ImVec2(-1, calibPlotHeight(plotW, 320.0f, 460.0f)))) {
        ImPlot::SetupAxes("time (ms)", "speed (rpm)",
                          ImPlotAxisFlags_None, ImPlotAxisFlags_None);
        if (haveData) {
            ImPlot::PushStyleColor(ImPlotCol_Line,
                                   ImVec4(0.40f,0.78f,0.95f,1.0f));
            ImPlot::PlotLine("omega measured (rpm)", run.t.data(),
                             run.measured.data(),
                             static_cast<int>(run.measured.size()));
            ImPlot::PopStyleColor();
            if (fitOk) {
                // Overlay the fitted exponential.
                std::vector<float> fy(run.t.size());
                const float t0 = (run.triggerIdx < run.t.size())
                                 ? run.t[run.triggerIdx] : run.t[0];
                for (size_t i = 0; i < run.t.size(); ++i) {
                    const float dt = run.t[i] - t0;
                    fy[i] = (dt >= 0.0f)
                            ? w0 * std::exp(-dt / tau)
                            : run.measured[i];
                }
                ImPlot::PushStyleColor(ImPlotCol_Line,
                                       ImVec4(0.95f,0.65f,0.25f,0.9f));
                ImPlot::PlotLine("fitted exp", run.t.data(), fy.data(),
                                 static_cast<int>(fy.size()));
                ImPlot::PopStyleColor();
            }
        } else {
            ImPlot::SetupAxesLimits(0.0, 1.0, 0.0, 1.0, ImPlotCond_Once);
            ImPlot::Annotation(0.5, 0.5, ImVec4(0,0,0,0), ImVec2(0,0), false,
                               "run a coast-down to populate");
        }
        ImPlot::EndPlot();
    }

    if (fitOk) {
        ImGui::Text("decay time constant tau = J/B = %.1f ms   (fit R2 %.3f)",
                    tau, r2);
        const float bOverJ = 1000.0f / tau;   // 1/s
        ImGui::Text("ratio B/J = %.4f s^-1", bOverJ);
        const float bVal = calibBlobValueF32(25);
        if (bVal > 1e-9f) {
            ImGui::Text("with friction B = %.5f (id 25): "
                        "estimated J = %.6f kg*m^2", bVal, bVal / bOverJ);
        } else {
            ImGui::TextDisabled("Enter friction B (id 25) above to split the "
                                "ratio into an absolute J estimate; the "
                                "coast-down alone gives only B/J.");
        }
    } else if (haveData) {
        ImGui::TextColored(ImVec4(0.95f,0.75f,0.25f,1.0f),
                           "decay fit failed -- not enough clean decay "
                           "samples (raise start speed / capture longer)");
    } else {
        ImGui::TextDisabled("fitted tau, B/J and J populate once a coast-down "
                            "capture is acquired.");
    }
}

// ==========================================================================
// Config (full) -- the compact dense editable grid (spec §3.3).
// ==========================================================================
void MainUi::drawCalibConfigFull()
{
    CalibGate g; calibPreconditions(g);

    ImGui::TextUnformatted("Full motor configuration");
    ImGui::SameLine();
    ImGui::TextDisabled("(everyday editor -- RAM via blob, Save is explicit)");
    ImGui::Separator();

    if (!motorBlobValid_) {
        ImGui::TextColored(ImVec4(0.95f,0.75f,0.25f,1.0f),
                           "config not read yet -- press Read All below");
    }

    // PI current/speed loop group.
    if (ImGui::CollapsingHeader("PI current / speed loop",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        for (uint16_t id : {uint16_t(10), uint16_t(11), uint16_t(12),
                            uint16_t(13), uint16_t(14), uint16_t(15)}) {
            const CalibParamMeta* m = nullptr;
            for (int i = 0; i < kCalibParamMetaCount; ++i)
                if (kCalibParamMeta[i].id == id) { m = &kCalibParamMeta[i]; break; }
            if (m) calibRangeField(*m, calibBlobValueF32(id),
                                   calibSync_[id], true);
        }
    }

    // Machine params group.
    if (ImGui::CollapsingHeader("Machine params (measured by Test 2)",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        for (uint16_t id : {uint16_t(20), uint16_t(21), uint16_t(22),
                            uint16_t(23), uint16_t(24), uint16_t(25)}) {
            const CalibParamMeta* m = nullptr;
            for (int i = 0; i < kCalibParamMetaCount; ++i)
                if (kCalibParamMeta[i].id == id) { m = &kCalibParamMeta[i]; break; }
            if (m) calibRangeField(*m, calibBlobValueF32(id),
                                   calibSync_[id], true);
        }
    }

    // HW group (collapsed by default).
    if (ImGui::CollapsingHeader("HW feature defaults")) {
        const CalibParamMeta* m = nullptr;
        for (int i = 0; i < kCalibParamMetaCount; ++i)
            if (kCalibParamMeta[i].id == 50) { m = &kCalibParamMeta[i]; break; }
        if (m) calibRangeField(*m, calibBlobValueF32(50), calibSync_[50], true);
    }

    // Theta group (read-only here -- edit via Test 1).
    if (ImGui::CollapsingHeader("Theta LUT (read-only -- edit in Test 1)")) {
        if (ImGui::BeginTable("##cfgfull-theta", 2,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("slot");
            ImGui::TableSetupColumn("value (rad)");
            ImGui::TableHeadersRow();
            const char* names[7] = { "theta.offset", "theta.sector_0",
                "theta.sector_1", "theta.sector_2", "theta.sector_3",
                "theta.sector_4", "theta.sector_5" };
            for (int i = 0; i < 7; ++i) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn(); ImGui::TextUnformatted(names[i]);
                ImGui::TableNextColumn();
                ImGui::Text("%+.5f", calibBlobValueF32(static_cast<uint16_t>(i)));
            }
            ImGui::EndTable();
        }
        if (ImGui::Button("-> Test 1 (Theta)")) calibSection_ = 0;
    }

    // DIAG counters (read-only).
    if (ImGui::CollapsingHeader("DIAG counters (read-only)")) {
        if (ImGui::BeginTable("##cfgfull-diag", 2,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("id / name");
            ImGui::TableSetupColumn("value");
            ImGui::TableHeadersRow();
            struct D { uint16_t id; const char* name; };
            static const D kDiag[] = {
                {30,"diag.brk_live"}, {31,"diag.brk_low_cnt"},
                {32,"diag.brk_at_flt"}, {33,"diag.brk_flt_cnt"},
                {34,"diag.wdog_hits"}, {35,"diag.wdog_ms"},
                {36,"diag.wdog_spr_brk"},
            };
            for (const auto& d : kDiag) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn(); ImGui::Text("%u %s", d.id, d.name);
                ImGui::TableNextColumn();
                auto it = motorConfigCache_.find(d.id);
                if (it != motorConfigCache_.end() && it->second.hasValue)
                    ImGui::Text("0x%08X (%u)", it->second.rawValue,
                                it->second.rawValue);
                else
                    ImGui::TextDisabled("--");
            }
            ImGui::EndTable();
        }
    }

    ImGui::Separator();
    if (ImGui::Button("Read All")) {
        motorBlobAuthoritativeUntil_ = 0.0;
        sendMotorBlobGet();
        for (auto& kv : calibSync_) kv.second.hasStaged = false;
        calibStatus_ = "Read All -- BLOB GET requested";
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!g.motorIdle);
    if (ImGui::Button("Save to flash")) {
        sendMotorConfigParam(/*SAVE_ALL*/ 2, 0, 0);
        motorBlobAuthoritativeUntil_ = 0.0;
        calibStatus_ = "PARAM SAVE_ALL sent";
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Revert RAM")) {
        sendMotorConfigParam(/*LOAD_ALL*/ 3, 0, 0);
        calibStatus_ = "PARAM LOAD_ALL sent";
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset defaults")) {
        sendMotorConfigParam(/*RESET_DEFAULTS*/ 4, 0, 0);
        calibStatus_ = "PARAM RESET_DEFAULTS sent (RAM only)";
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear staged edits")) {
        for (auto& kv : calibSync_) {
            kv.second.hasStaged = false;
            kv.second.state = CalibSyncState::Synced;
        }
        calibStatus_ = "staged edits cleared";
    }
    calibStatusLine();
}

// ==========================================================================
// Calibration-tab capture pipeline (Test 3 / Test 4).
//
// This reuses the verified /capture HTTP endpoint (fetchCaptureViaHttp), but
// the variable list is the shared watches_ table from Variables/Plots. Tests
// auto-enable their required motor variables there and refuse to run when the
// hardware capture slot limit would be exceeded.
// ==========================================================================
bool MainUi::calibEnsureMotorCaptureVars(const std::vector<const char*>& names,
                                         std::vector<size_t>& watchIdx,
                                         std::string& error)
{
    watchIdx.clear();
    error.clear();

    const SymbolSource* motor = nullptr;
    for (const SymbolSource& s : sources_) {
        if (s.name == "motor") { motor = &s; break; }
    }
    if (!motor) {
        error = "motor symbols are not loaded";
        return false;
    }

    auto findSym = [&](const char* name) -> const VariableSymbol* {
        for (const VariableSymbol& v : motor->symbols)
            if (v.name == name) return &v;
        return nullptr;
    };
    auto findWatch = [&](const VariableSymbol& sym) -> size_t {
        for (size_t i = 0; i < watches_.size(); ++i) {
            if (watches_[i].nodeId == motor->defaultNodeId &&
                watches_[i].address == sym.address) {
                return i;
            }
        }
        return static_cast<size_t>(-1);
    };

    bool changed = false;
    for (const char* name : names) {
        const VariableSymbol* sym = findSym(name);
        if (!sym) {
            error = std::string("required motor variable missing in symbols json: ") + name;
            return false;
        }
        size_t wi = findWatch(*sym);
        if (wi == static_cast<size_t>(-1)) {
            WatchVar w;
            static_cast<VariableSymbol&>(w) = *sym;
            w.nodeId = motor->defaultNodeId;
            w.pollHz = 10.0f;
            w.enabled = true;
            w.capture = true;
            w.plot = true;
            w.source = motor->name;
            watches_.push_back(std::move(w));
            wi = watches_.size() - 1;
            changed = true;
        } else {
            WatchVar& w = watches_[wi];
            if (!w.enabled || !w.capture || !w.plot) {
                w.enabled = true;
                w.capture = true;
                w.plot = true;
                changed = true;
            }
            if (w.source.empty()) {
                w.source = motor->name;
                changed = true;
            }
        }
        watchIdx.push_back(wi);
    }

    size_t motorCaptureCount = 0;
    for (const WatchVar& w : watches_) {
        if (w.nodeId == motor->defaultNodeId && w.capture) ++motorCaptureCount;
    }
    if (motorCaptureCount > kCapMaxSlots) {
        error = "motor capture set has " + std::to_string(motorCaptureCount) +
                " variables, hardware limit is 6; unmark/remove extra variables in Variables/Plots";
        return false;
    }

    if (changed) markScenarioDirty("calibration capture variables");
    return true;
}

std::string MainUi::calibBuildCaptureQueryFromWatches(const std::vector<size_t>& watchIdx,
                                                      int durationMs, uint16_t periodUs,
                                                      uint8_t prePct, uint8_t postPct) const
{
    std::string q;
    char buf[80];
    std::snprintf(buf, sizeof(buf), "node=2&duration_ms=%d&period_us=%u",
                  durationMs, static_cast<unsigned>(periodUs));
    q += buf;
    std::snprintf(buf, sizeof(buf), "&buffer_kb=32&pre_pct=%u&post_pct=%u",
                  static_cast<unsigned>(prePct), static_cast<unsigned>(postPct));
    q += buf;
    for (size_t slot = 0; slot < watchIdx.size() && slot < kCapMaxSlots; ++slot) {
        const WatchVar& w = watches_[watchIdx[slot]];
        // Match buildCaptureHttpQuery: slot syntax is 0xADDR:TYPE (no
        // explicit size — the short type token implies width). TYPE set
        // = u8/i8/u16/i16/u32/i32/f32 (see CapClient::parseSlotSpec).
        const char* tyShort = "f32";
        const std::string& ty = w.type;
        if      (ty == "uint8"  || ty == "u8")  tyShort = "u8";
        else if (ty == "int8"   || ty == "i8")  tyShort = "i8";
        else if (ty == "uint16" || ty == "u16") tyShort = "u16";
        else if (ty == "int16"  || ty == "i16") tyShort = "i16";
        else if (ty == "uint32" || ty == "u32") tyShort = "u32";
        else if (ty == "int32"  || ty == "i32") tyShort = "i32";
        std::snprintf(buf, sizeof(buf), "&slot%zu=0x%X:%s", slot,
                      static_cast<unsigned>(w.address), tyShort);
        q += buf;
    }
    return q;
}

bool MainUi::calibLaunchCapture(CalibCapRun& run, const std::string& query)
{
    if (calibCapRunning_.load(std::memory_order_acquire)) {
        run.error = "a calibration capture is already running";
        run.phase = CalibCapPhase::Failed;
        return false;
    }
    if (calibCapThread_.joinable()) calibCapThread_.join();

    // Resolve the HTTP host -- /capture only exists on the TCP / Wi-Fi-AP
    // path (the SSH tunnel into app_dd). SLCAN has no app_dd to talk to.
    std::string host;
    if (transportKind_ == TransportKind::Tcp) {
        host = tcpHost_.data();
    } else if (transportKind_ == TransportKind::WifiAp) {
        host = wifiApHost_.data();
    } else {
        run.error = "step capture needs the TCP or Wi-Fi-AP transport "
                    "(the /capture endpoint lives in app_dd)";
        run.phase = CalibCapPhase::Failed;
        return false;
    }

    {
        std::lock_guard<std::mutex> lk(calibCapMutex_);
        calibCapResult_ = CaptureHttpResult{};
    }
    calibCapTarget_ = &run;
    captureHttpBytesRx_.store(0, std::memory_order_relaxed);
    captureHttpBytesExpected_.store(0, std::memory_order_relaxed);
    captureHttpStartedAt_ = clock_.nowSeconds();
    calibCapDone_.store(false, std::memory_order_release);
    calibCapRunning_.store(true, std::memory_order_release);

    calibCapThread_ = std::thread([this, host, query]() {
        CaptureHttpResult r;
        (void) fetchCaptureViaHttp(host, query, r);
        {
            std::lock_guard<std::mutex> lk(calibCapMutex_);
            calibCapResult_ = std::move(r);
        }
        calibCapDone_.store(true, std::memory_order_release);
        calibCapRunning_.store(false, std::memory_order_release);
    });
    return true;
}

void MainUi::calibComputeStepMetrics(CalibCapRun& run)
{
    CalibStepMetrics m;
    const std::vector<float>& y = run.amplitude;
    const std::vector<float>& t = run.t;
    const size_t n = std::min(y.size(), t.size());
    if (n < 8) { run.metrics = m; return; }

    // Step instant: triggerIdx if inside range, else the largest single-step
    // jump in the setpoint series (fallback when no pre-trigger is honoured).
    size_t stepIdx = run.triggerIdx;
    if (stepIdx == 0 || stepIdx >= n) {
        float biggest = 0.0f;
        for (size_t i = 1; i < std::min(run.setpoint.size(), n); ++i) {
            const float d = std::fabs(run.setpoint[i] - run.setpoint[i - 1]);
            if (d > biggest) { biggest = d; stepIdx = i; }
        }
        if (stepIdx == 0 || stepIdx >= n) stepIdx = n / 4;
    }

    // Baseline = mean of the pre-step samples; target = mean of the last
    // quarter of the post-step samples (assumed settled).
    double base = 0.0; size_t baseN = 0;
    for (size_t i = 0; i < stepIdx; ++i) { base += y[i]; ++baseN; }
    m.baseline = baseN ? static_cast<float>(base / baseN) : y[0];

    const size_t tailStart = stepIdx + (n - stepIdx) * 3 / 4;
    double tgt = 0.0; size_t tgtN = 0;
    for (size_t i = tailStart; i < n; ++i) { tgt += y[i]; ++tgtN; }
    m.target = tgtN ? static_cast<float>(tgt / tgtN) : y[n - 1];

    const float span = m.target - m.baseline;
    const float aspan = std::fabs(span);

    // Peak excursion after the step.
    m.peak = y[stepIdx];
    for (size_t i = stepIdx; i < n; ++i) {
        if (span >= 0.0f) { if (y[i] > m.peak) m.peak = y[i]; }
        else              { if (y[i] < m.peak) m.peak = y[i]; }
    }

    if (aspan > 1e-6f) {
        // Overshoot beyond the target, as a percent of the step.
        const float over = (span >= 0.0f) ? (m.peak - m.target)
                                          : (m.target - m.peak);
        m.overshootPct = std::max(0.0f, over) / aspan * 100.0f;

        // Rise time: 10% -> 90% crossing of the step.
        const float lo = m.baseline + span * 0.1f;
        const float hi = m.baseline + span * 0.9f;
        float tLo = -1.0f, tHi = -1.0f;
        for (size_t i = stepIdx; i < n; ++i) {
            const bool passLo = (span >= 0.0f) ? (y[i] >= lo) : (y[i] <= lo);
            const bool passHi = (span >= 0.0f) ? (y[i] >= hi) : (y[i] <= hi);
            if (tLo < 0.0f && passLo) tLo = t[i];
            if (tHi < 0.0f && passHi) { tHi = t[i]; break; }
        }
        if (tLo >= 0.0f && tHi >= tLo) m.riseMs = tHi - tLo;

        // Settling time: last instant the signal leaves a +/-2% band, time
        // measured from the step instant.
        const float band = aspan * 0.02f;
        float lastOut = t[stepIdx];
        for (size_t i = stepIdx; i < n; ++i) {
            if (std::fabs(y[i] - m.target) > band) lastOut = t[i];
        }
        m.settleMs = lastOut - t[stepIdx];

        // Steady-state error vs the commanded target (slot 0 tail).
        float cmdTgt = m.target;
        if (run.setpoint.size() >= n && tgtN) {
            double c = 0.0;
            for (size_t i = tailStart; i < n; ++i) c += run.setpoint[i];
            cmdTgt = static_cast<float>(c / tgtN);
        }
        m.ssErrPct = std::fabs(cmdTgt - m.target) / aspan * 100.0f;

        // Noise RMS over the settled tail.
        double acc = 0.0; size_t cnt = 0;
        for (size_t i = tailStart; i < n; ++i) {
            const float e = y[i] - m.target;
            acc += static_cast<double>(e) * e; ++cnt;
        }
        m.noiseRms = cnt ? static_cast<float>(std::sqrt(acc / cnt)) : 0.0f;
        m.valid = true;
    }
    run.metrics = m;
}

void MainUi::calibDrainCapture()
{
    if (!calibCapDone_.load(std::memory_order_acquire)) return;
    if (calibCapThread_.joinable()) calibCapThread_.join();
    calibCapDone_.store(false, std::memory_order_release);

    CaptureHttpResult r;
    {
        std::lock_guard<std::mutex> lk(calibCapMutex_);
        r = std::move(calibCapResult_);
        calibCapResult_ = CaptureHttpResult{};
    }
    CalibCapRun* run = calibCapTarget_;
    calibCapTarget_ = nullptr;
    if (run == nullptr) return;

    if (run->abortRequested) {
        run->phase  = CalibCapPhase::Failed;
        run->error  = run->error.empty() ? "aborted by operator" : run->error;
        run->status = "aborted";
        calibStatus_ = run->error;
        return;
    }

    if (!r.ok) {
        run->phase  = CalibCapPhase::Failed;
        run->error  = "capture failed (" + r.error_stage + "): " +
                      r.error_message;
        run->status = "error";
        calibStatus_ = run->error;
        return;
    }

    // Decode the binary body. The device packs every slot at 4 bytes per
    // sample regardless of declared size (see applyCapturedSamples), so a
    // sample stride is slot_count * 4. Slot 0 = setpoint, slot 1 = measured.
    const uint16_t samples = r.samples;
    const uint8_t  slots   = r.slot_count;
    const size_t   stride  = static_cast<size_t>(slots) * 4u;
    run->t.clear();
    run->setpoint.clear();
    run->measured.clear();
    run->amplitude.clear();
    run->periodUs   = (r.period_us > 0) ? r.period_us : run->periodUs;
    run->triggerIdx = r.trigger_idx;

    const float dtMs = run->periodUs / 1000.0f;
    auto readF32 = [&](size_t sample, uint8_t slot) -> float {
        const size_t off = sample * stride + static_cast<size_t>(slot) * 4u;
        float v = 0.0f;
        if (off + 4 <= r.bytes.size()) std::memcpy(&v, &r.bytes[off], 4);
        return v;
    };
    for (uint16_t i = 0; i < samples; ++i) {
        run->t.push_back(static_cast<float>(i) * dtMs);
        const float sp = (slots >= 1) ? readF32(i, 0) : 0.0f;
        const float me = (slots >= 2) ? readF32(i, 1) : sp;
        run->setpoint.push_back(sp);
        run->measured.push_back(me);
        if (slots >= 3) {
            // Three-slot capture: slot 1 = id, slot 2 = iq -> amplitude.
            const float idv = me;
            const float iqv = readF32(i, 2);
            run->amplitude.push_back(std::sqrt(idv * idv + iqv * iqv));
        } else {
            run->amplitude.push_back(me);
        }
    }
    if (run->synthSetpoint && !run->t.empty()) {
        const uint16_t stepSample = static_cast<uint16_t>(
            std::clamp(run->stepDelayMs / dtMs, 0.0f,
                       static_cast<float>(run->t.size() - 1)));
        run->triggerIdx = stepSample;
        for (size_t i = 0; i < run->setpoint.size(); ++i) {
            const float tMs = run->t[i];
            const bool afterStart = (i >= stepSample);
            const bool beforeEnd = (run->pulseWidthMs <= 0.0f) ||
                                   (tMs < run->stepDelayMs + run->pulseWidthMs);
            run->setpoint[i] = (afterStart && beforeEnd)
                ? run->commandTo : run->commandFrom;
        }
    }

    // FFT of the measured signal so the spectrum graph has content.
    if (run->measured.size() >= 8) {
        const double fs = 1.0e6 / static_cast<double>(run->periodUs);
        calibFftMagnitude(run->measured, fs, run->fftFreq, run->fftMag);
    }
    calibComputeStepMetrics(*run);

    run->phase  = CalibCapPhase::Done;
    run->status = "capture complete -- " + std::to_string(samples) +
                  " samples";
    calibStatus_ = run->status;
}

} // namespace drivescope
