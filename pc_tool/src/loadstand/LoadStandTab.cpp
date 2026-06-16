#include "ui/MainUi.h"
#include "loadstand/LoadStandClient.h"

#include "imgui.h"
#include "implot.h"

#include <cstdio>

namespace drivescope {

// "Load Stand" dockable tab: connect to the loading-stand STM32 over the Pi TCP
// bridge and plot its torque alongside the combine's own plots. Defined here
// (not in the MainUi.cpp monolith) but is a MainUi member, so it sees privates.
void MainUi::drawLoadStand()
{
    LoadStandClient& ls = loadStand_;

    ImGui::TextUnformatted("Loading-stand brake (STM32) via Pi serial->TCP bridge");
    ImGui::Separator();

    // ---- connection -------------------------------------------------------
    ImGui::SetNextItemWidth(170);
    ImGui::InputText("host", lsHost_, sizeof(lsHost_));
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90);
    ImGui::InputInt("port", &lsPort_);
    ImGui::SameLine();
    if (!ls.isOpen()) {
        if (ImGui::Button("Connect")) {
            ls.clearData();
            ls.open(lsHost_, lsPort_);
        }
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

    // ---- plot options -----------------------------------------------------
    ImGui::Checkbox("setpoint", &lsShowSetpoint_);
    ImGui::SameLine();
    ImGui::Checkbox("current (A)", &lsShowCurrent_);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(180);
    ImGui::SliderFloat("window s", &lsWindowSec_, 2.0f, 120.0f, "%.0f");

    // ---- torque plot ------------------------------------------------------
    const double nowT = snap.t.empty() ? 0.0 : snap.t.back();
    if (ImPlot::BeginPlot("##loadstand_torque", ImVec2(-1, -1),
                          ImPlotFlags_NoMenus | ImPlotFlags_NoMouseText)) {
        ImPlot::SetupAxes("device time, s", "torque, Nm", 0, ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxisLimits(ImAxis_X1, nowT - static_cast<double>(lsWindowSec_),
                                nowT, ImGuiCond_Always);
        ImPlot::SetupLegend(ImPlotLocation_NorthWest, 0);

        const int n = static_cast<int>(snap.t.size());
        if (n > 0) {
            ImPlot::SetNextLineStyle(ImVec4(0.20f, 0.80f, 1.00f, 1.0f));
            ImPlot::PlotLine("torque", snap.t.data(), snap.torque.data(), n);
            if (lsShowSetpoint_) {
                ImPlot::SetNextLineStyle(ImVec4(1.00f, 0.60f, 0.10f, 1.0f));
                ImPlot::PlotLine("setpoint", snap.t.data(), snap.setpoint.data(), n);
            }
            if (lsShowCurrent_) {
                ImPlot::SetNextLineStyle(ImVec4(0.55f, 1.00f, 0.40f, 1.0f));
                ImPlot::PlotLine("current (A)", snap.t.data(), snap.current.data(), n);
            }
        } else {
            ImPlot::PlotDummy("torque");
        }
        ImPlot::EndPlot();
    }
}

} // namespace drivescope
