#include "ui/MainUi.h"
#include "loadstand/LoadStandClient.h"

#include "imgui.h"

#include <cstdio>

namespace drivescope {

// Pull new loading-stand samples into the main-plot buffers each frame. The
// torque is plotted on the MAIN "Plots" graph (see drawPlots), so the existing
// ruler, autoscale and signal logger all apply -- no separate graph. Called
// from MainUi::draw() every frame, regardless of the active tab.
void MainUi::pumpLoadStand()
{
    if (!loadStand_.isOpen()) {
        standLastSamples_ = 0;
        return;
    }
    LoadStandClient::Snapshot s;
    loadStand_.snapshot(s);
    if (s.samples == standLastSamples_) return;   // nothing new since last frame
    standLastSamples_ = s.samples;
    if (plotPaused_) return;

    const double tRel = clock_.nowSeconds() - plotTimeOrigin_;
    standXs_.push_back(tRel);
    standTorque_.push_back(s.lastTorque);
    standSet_.push_back(s.lastSetpoint);
    standCur_.push_back(s.lastCurrent);

    // Trim to the same history window the watches use.
    const double keepAfter = tRel - static_cast<double>(plotHistorySec_);
    size_t first = 0;
    while (first < standXs_.size() && standXs_[first] < keepAfter) ++first;
    if (first > 0) {
        standXs_.erase(standXs_.begin(), standXs_.begin() + static_cast<std::ptrdiff_t>(first));
        standTorque_.erase(standTorque_.begin(), standTorque_.begin() + static_cast<std::ptrdiff_t>(first));
        standSet_.erase(standSet_.begin(), standSet_.begin() + static_cast<std::ptrdiff_t>(first));
        standCur_.erase(standCur_.begin(), standCur_.begin() + static_cast<std::ptrdiff_t>(first));
    }

    // Feed the signal logger so "save" includes the stand torque.
    if (signalLogger_.active()) {
        char buf[24];
        std::snprintf(buf, sizeof buf, "%.3f", s.lastTorque);
        signalLogger_.log(clock_.nowSeconds(), 0, "stand.M_Nm", 0, "Nm", buf);
    }
}

// "Load Stand" tab: connection + control only. The torque curve itself shows up
// on the main Plots graph (toggles below pick which stand series are drawn).
void MainUi::drawLoadStand()
{
    LoadStandClient& ls = loadStand_;

    ImGui::TextUnformatted("Loading-stand brake (STM32) via Pi serial->TCP bridge");
    ImGui::TextDisabled("Torque is plotted on the main 'Plots' tab (ruler + save apply there).");
    ImGui::Separator();

    // ---- connection -------------------------------------------------------
    ImGui::SetNextItemWidth(170);
    ImGui::InputText("host", lsHost_, sizeof(lsHost_));
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90);
    ImGui::InputInt("port", &lsPort_);
    ImGui::SameLine();
    if (!ls.isOpen()) {
        if (ImGui::Button("Connect")) ls.open(lsHost_, lsPort_);
    } else {
        if (ImGui::Button("Disconnect")) ls.close();
    }
    ImGui::SameLine();
    ImGui::TextColored(ls.isOpen() ? ImVec4(0.4f, 1.0f, 0.4f, 1.0f)
                                   : ImVec4(0.9f, 0.7f, 0.3f, 1.0f),
                       "%s", ls.status().c_str());

    LoadStandClient::Snapshot snap;
    ls.snapshot(snap);

    // ---- control ----------------------------------------------------------
    ImGui::BeginDisabled(!ls.isOpen());
    ImGui::SetNextItemWidth(180);
    ImGui::SliderFloat("setpoint Nm", &lsSetNm_, 0.0f, 25.0f, "%.1f");
    ImGui::SameLine();
    if (ImGui::Button("Set torque")) {
        char cmd[40];
        std::snprintf(cmd, sizeof cmd, "set_m %.2f", lsSetNm_);
        ls.sendLine(cmd);
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop (vt off)")) ls.sendLine("vt off");
    ImGui::SameLine();
    if (ImGui::Button("Re-zero (cal)")) ls.sendLine("cal");

    ImGui::SetNextItemWidth(240);
    const bool enter = ImGui::InputText("cmd", lsCmd_, sizeof(lsCmd_),
                                        ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SameLine();
    if ((ImGui::Button("Send") || enter) && lsCmd_[0] != '\0') {
        ls.sendLine(lsCmd_);
        lsCmd_[0] = '\0';
    }
    ImGui::EndDisabled();

    // ---- live readout -----------------------------------------------------
    const char* dirName = (snap.lastDir > 0) ? "POS (driving +)"
                        : (snap.lastDir < 0) ? "NEG (driving -)"
                                             : "ARMED (no load)";
    ImGui::Text("M = %+.3f Nm   set = %.2f Nm   I = %.3f A   duty = %.0f%%   dir = %s   (%zu pts)",
                snap.lastTorque, snap.lastSetpoint, snap.lastCurrent,
                snap.lastDuty, dirName, snap.samples);
    if (!snap.lastText.empty())
        ImGui::TextDisabled("last: %s", snap.lastText.c_str());

    // ---- which stand series to draw on the main plot ----------------------
    ImGui::Separator();
    ImGui::TextUnformatted("Show on main plot:");
    ImGui::SameLine(); ImGui::Checkbox("torque",   &standPlotTorque_);
    ImGui::SameLine(); ImGui::Checkbox("setpoint", &standPlotSetpoint_);
    ImGui::SameLine(); ImGui::Checkbox("current",  &standPlotCurrent_);
}

} // namespace drivescope
