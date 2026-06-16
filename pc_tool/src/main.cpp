#include "cli/HeadlessCapture.h"
#include "cli/MotorBlobCli.h"
#include "config/ToolConfig.h"
#include "ui/MainUi.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#ifdef _WIN32
#define GLFW_EXPOSE_NATIVE_WIN32
#endif
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>
#ifdef _WIN32
#include <GLFW/glfw3native.h>
#include <windows.h>
#include <windowsx.h>
#endif

namespace {

#ifdef _WIN32
WNDPROC gDriveScopePrevWndProc = nullptr;

LRESULT CALLBACK driveScopeWindowProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    if (msg == WM_NCHITTEST && !IsZoomed(hwnd)) {
        const LRESULT base =
            gDriveScopePrevWndProc != nullptr
                ? CallWindowProcW(gDriveScopePrevWndProc, hwnd, msg, wParam, lParam)
                : DefWindowProcW(hwnd, msg, wParam, lParam);
        if (base != HTCLIENT) return base;

        RECT rect{};
        GetWindowRect(hwnd, &rect);
        const POINT pt{GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam)};
        const int border = std::max(6, GetSystemMetrics(SM_CXSIZEFRAME));
        const bool left   = pt.x >= rect.left   && pt.x < rect.left + border;
        const bool right  = pt.x <= rect.right  && pt.x > rect.right - border;
        const bool top    = pt.y >= rect.top    && pt.y < rect.top + border;
        const bool bottom = pt.y <= rect.bottom && pt.y > rect.bottom - border;

        if (top && left)     return HTTOPLEFT;
        if (top && right)    return HTTOPRIGHT;
        if (bottom && left)  return HTBOTTOMLEFT;
        if (bottom && right) return HTBOTTOMRIGHT;
        if (left)            return HTLEFT;
        if (right)           return HTRIGHT;
        if (top)             return HTTOP;
        if (bottom)          return HTBOTTOM;
        return HTCLIENT;
    }

    return gDriveScopePrevWndProc != nullptr
        ? CallWindowProcW(gDriveScopePrevWndProc, hwnd, msg, wParam, lParam)
        : DefWindowProcW(hwnd, msg, wParam, lParam);
}

void installUndecoratedResize(GLFWwindow* window)
{
    HWND hwnd = glfwGetWin32Window(window);
    if (hwnd == nullptr) return;
    gDriveScopePrevWndProc = reinterpret_cast<WNDPROC>(
        SetWindowLongPtrW(hwnd, GWLP_WNDPROC,
                          reinterpret_cast<LONG_PTR>(driveScopeWindowProc)));
}
#endif

void applyDriveScopeStyle()
{
    // Quiet VS Code-like workbench palette: readable first, decorative second.
    ImGuiStyle& s = ImGui::GetStyle();
    s.WindowRounding    = 0.0f;
    s.ChildRounding     = 5.0f;
    s.FrameRounding     = 4.0f;
    s.PopupRounding     = 5.0f;
    s.GrabRounding      = 4.0f;
    s.TabRounding       = 0.0f;
    s.ScrollbarRounding = 6.0f;
    s.WindowPadding     = ImVec2(12, 10);
    s.FramePadding      = ImVec2(9, 5);
    s.ItemSpacing       = ImVec2(8, 7);
    s.ItemInnerSpacing  = ImVec2(7, 5);
    s.IndentSpacing     = 17.0f;
    s.ScrollbarSize     = 12.0f;
    s.GrabMinSize       = 12.0f;
    s.WindowBorderSize  = 1.0f;
    s.FrameBorderSize   = 0.0f;
    s.PopupBorderSize   = 1.0f;
    s.TabBorderSize     = 0.0f;
    s.CellPadding       = ImVec2(8, 5);
    s.WindowTitleAlign  = ImVec2(0.0f, 0.5f);
    s.DisabledAlpha     = 0.60f;

    ImVec4* c = s.Colors;
    // Surfaces
    c[ImGuiCol_WindowBg]             = ImVec4(0.118f, 0.118f, 0.118f, 1.00f); // #1E1E1E
    c[ImGuiCol_ChildBg]              = ImVec4(0.150f, 0.150f, 0.154f, 1.00f); // #262627
    c[ImGuiCol_PopupBg]              = ImVec4(0.172f, 0.172f, 0.180f, 0.99f);
    c[ImGuiCol_MenuBarBg]            = ImVec4(0.160f, 0.160f, 0.166f, 1.00f);
    // Text
    c[ImGuiCol_Text]                 = ImVec4(0.855f, 0.855f, 0.855f, 1.00f); // #DADADA
    c[ImGuiCol_TextDisabled]         = ImVec4(0.600f, 0.600f, 0.615f, 1.00f);
    c[ImGuiCol_TextSelectedBg]       = ImVec4(0.125f, 0.305f, 0.465f, 0.85f);
    // Borders
    c[ImGuiCol_Border]               = ImVec4(0.235f, 0.235f, 0.245f, 1.00f);
    c[ImGuiCol_BorderShadow]         = ImVec4(0.000f, 0.000f, 0.000f, 0.35f);
    c[ImGuiCol_Separator]            = ImVec4(0.230f, 0.230f, 0.240f, 1.00f);
    c[ImGuiCol_SeparatorHovered]     = ImVec4(0.290f, 0.430f, 0.550f, 0.85f);
    c[ImGuiCol_SeparatorActive]      = ImVec4(0.260f, 0.520f, 0.735f, 1.00f);
    // Inputs / frames
    c[ImGuiCol_FrameBg]              = ImVec4(0.220f, 0.220f, 0.230f, 1.00f);
    c[ImGuiCol_FrameBgHovered]       = ImVec4(0.270f, 0.270f, 0.285f, 1.00f);
    c[ImGuiCol_FrameBgActive]        = ImVec4(0.245f, 0.330f, 0.405f, 1.00f);
    // Title
    c[ImGuiCol_TitleBg]              = ImVec4(0.135f, 0.135f, 0.142f, 1.00f);
    c[ImGuiCol_TitleBgActive]        = ImVec4(0.145f, 0.145f, 0.152f, 1.00f);
    c[ImGuiCol_TitleBgCollapsed]     = ImVec4(0.135f, 0.135f, 0.142f, 1.00f);
    // Scrollbar
    c[ImGuiCol_ScrollbarBg]          = ImVec4(0.118f, 0.118f, 0.118f, 1.00f);
    c[ImGuiCol_ScrollbarGrab]        = ImVec4(0.310f, 0.310f, 0.318f, 1.00f);
    c[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.396f, 0.396f, 0.404f, 1.00f);
    c[ImGuiCol_ScrollbarGrabActive]  = ImVec4(0.490f, 0.490f, 0.502f, 1.00f);
    // Controls (accent blue)
    c[ImGuiCol_CheckMark]            = ImVec4(0.345f, 0.690f, 0.960f, 1.00f);
    c[ImGuiCol_SliderGrab]           = ImVec4(0.345f, 0.690f, 0.960f, 1.00f);
    c[ImGuiCol_SliderGrabActive]     = ImVec4(0.460f, 0.760f, 1.000f, 1.00f);
    // Buttons
    c[ImGuiCol_Button]               = ImVec4(0.245f, 0.245f, 0.255f, 1.00f);
    c[ImGuiCol_ButtonHovered]        = ImVec4(0.305f, 0.305f, 0.320f, 1.00f);
    c[ImGuiCol_ButtonActive]         = ImVec4(0.245f, 0.390f, 0.520f, 1.00f);
    // List items / TreeNode / CollapsingHeader / Selectable
    c[ImGuiCol_Header]               = ImVec4(0.210f, 0.235f, 0.260f, 1.00f);
    c[ImGuiCol_HeaderHovered]        = ImVec4(0.250f, 0.285f, 0.315f, 1.00f);
    c[ImGuiCol_HeaderActive]         = ImVec4(0.285f, 0.335f, 0.385f, 1.00f);
    // Resize grip
    c[ImGuiCol_ResizeGrip]           = ImVec4(0.310f, 0.310f, 0.318f, 0.40f);
    c[ImGuiCol_ResizeGripHovered]    = ImVec4(0.000f, 0.478f, 0.800f, 0.80f);
    c[ImGuiCol_ResizeGripActive]     = ImVec4(0.000f, 0.478f, 0.800f, 1.00f);
    // Tabs (square, VSCode style: active matches editor bg)
    c[ImGuiCol_Tab]                  = ImVec4(0.160f, 0.160f, 0.168f, 1.00f);
    c[ImGuiCol_TabHovered]           = ImVec4(0.230f, 0.250f, 0.270f, 1.00f);
    c[ImGuiCol_TabActive]            = ImVec4(0.118f, 0.118f, 0.118f, 1.00f);
    c[ImGuiCol_TabUnfocused]         = ImVec4(0.160f, 0.160f, 0.168f, 1.00f);
    c[ImGuiCol_TabUnfocusedActive]   = ImVec4(0.145f, 0.145f, 0.152f, 1.00f);
    // Tables
    c[ImGuiCol_TableHeaderBg]        = ImVec4(0.185f, 0.185f, 0.195f, 1.00f);
    c[ImGuiCol_TableBorderStrong]    = ImVec4(0.285f, 0.285f, 0.295f, 1.00f);
    c[ImGuiCol_TableBorderLight]     = ImVec4(0.220f, 0.220f, 0.230f, 1.00f);
    c[ImGuiCol_TableRowBg]           = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    c[ImGuiCol_TableRowBgAlt]        = ImVec4(1.0f, 1.0f, 1.0f, 0.032f);
    // Plot legacy
    c[ImGuiCol_PlotLines]            = ImVec4(0.000f, 0.478f, 0.800f, 1.00f);
    c[ImGuiCol_PlotLinesHovered]     = ImVec4(0.090f, 0.620f, 0.969f, 1.00f);
    c[ImGuiCol_PlotHistogram]        = ImVec4(0.470f, 0.745f, 0.255f, 1.00f);
    c[ImGuiCol_PlotHistogramHovered] = ImVec4(0.560f, 0.835f, 0.345f, 1.00f);
}

void applyDriveScopePlotStyle()
{
    ImPlotStyle& p = ImPlot::GetStyle();
    p.LineWeight = 1.8f;
    p.MarkerSize = 4.0f;
    p.MarkerWeight = 1.0f;
    p.FillAlpha = 0.25f;
    p.PlotPadding = ImVec2(10, 8);
    p.LabelPadding = ImVec2(5, 5);
    p.LegendPadding = ImVec2(8, 6);
    p.LegendInnerPadding = ImVec2(6, 4);
    p.LegendSpacing = ImVec2(6, 3);
    p.MousePosPadding = ImVec2(8, 6);

    ImVec4* c = p.Colors;
    c[ImPlotCol_PlotBg]       = ImVec4(0.105f, 0.105f, 0.110f, 1.00f);
    c[ImPlotCol_PlotBorder]   = ImVec4(0.300f, 0.300f, 0.315f, 1.00f);
    c[ImPlotCol_LegendBg]     = ImVec4(0.135f, 0.135f, 0.145f, 0.92f);
    c[ImPlotCol_LegendBorder] = ImVec4(0.255f, 0.255f, 0.270f, 1.00f);
    c[ImPlotCol_LegendText]   = ImVec4(0.870f, 0.870f, 0.880f, 1.00f);
    c[ImPlotCol_TitleText]    = ImVec4(0.870f, 0.870f, 0.880f, 1.00f);
    c[ImPlotCol_InlayText]    = ImVec4(0.720f, 0.720f, 0.740f, 1.00f);
    c[ImPlotCol_AxisText]     = ImVec4(0.720f, 0.720f, 0.740f, 1.00f);
    c[ImPlotCol_AxisGrid]     = ImVec4(0.350f, 0.350f, 0.370f, 0.33f);
    c[ImPlotCol_AxisTick]     = ImVec4(0.560f, 0.560f, 0.585f, 0.55f);
    c[ImPlotCol_AxisBgHovered]= ImVec4(0.245f, 0.330f, 0.405f, 0.45f);
    c[ImPlotCol_AxisBgActive] = ImVec4(0.245f, 0.390f, 0.520f, 0.55f);
    c[ImPlotCol_Selection]    = ImVec4(0.345f, 0.690f, 0.960f, 0.28f);
    c[ImPlotCol_Crosshairs]   = ImVec4(0.850f, 0.850f, 0.875f, 0.55f);
    p.Colormap = ImPlotColormap_Deep;
}

void loadDriveScopeFont(ImGuiIO& io)
{
    namespace fs = std::filesystem;
    const char* uiFonts[] = {
        "C:/Windows/Fonts/SegUIVar.ttf",
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/Calibri.ttf",
        "C:/Windows/Fonts/Tahoma.ttf",
    };
    for (const char* path : uiFonts) {
        if (fs::exists(path)) {
            ImFontConfig cfg;
            cfg.OversampleH = 2;
            cfg.OversampleV = 2;
            cfg.PixelSnapH = false;
            cfg.RasterizerMultiply = 1.08f;
            io.Fonts->AddFontFromFileTTF(path, 16.5f, &cfg, io.Fonts->GetGlyphRangesCyrillic());
            break;
        }
    }
    const char* monoFonts[] = {
        "C:/Windows/Fonts/CascadiaMono.ttf",
        "C:/Windows/Fonts/CascadiaCode.ttf",
        "C:/Windows/Fonts/consola.ttf",
    };
    for (const char* path : monoFonts) {
        if (fs::exists(path)) {
            ImFontConfig cfg;
            cfg.OversampleH = 2;
            cfg.OversampleV = 1;
            cfg.PixelSnapH = false;
            cfg.RasterizerMultiply = 1.04f;
            io.Fonts->AddFontFromFileTTF(path, 15.0f, &cfg, io.Fonts->GetGlyphRangesCyrillic());
            break;
        }
    }
}

std::vector<unsigned char> makeLogoIcon(int size)
{
    std::vector<unsigned char> px(static_cast<size_t>(size) * size * 4, 0);
    const float cx = size * 0.5f;
    const float cy = size * 0.5f;
    const float radius = size * 0.46f;
    const float ringInner = radius - size * 0.07f;
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            const float dx = x - cx;
            const float dy = y - cy;
            const float dist = std::sqrt(dx * dx + dy * dy);
            unsigned char* p = &px[(static_cast<size_t>(y) * size + x) * 4];

            if (dist > radius + 1.0f) continue;

            // Disc fill (#007ACC) with anti-aliased edge
            const float discAlpha = std::max(0.0f, std::min(1.0f, radius - dist + 0.5f));
            unsigned char r = 0x12, g = 0x32, b = 0x4A; // dark inset for ring inside
            unsigned char a = static_cast<unsigned char>(discAlpha * 255.0f);

            if (dist > ringInner) {
                // Outer ring in accent blue
                r = 0x00; g = 0x7A; b = 0xCC;
            }

            // Sine wave across the disc, only inside the inner area
            const float xn = (x - cx) / (size * 0.5f);
            if (dist <= ringInner) {
                const float wy = cy + std::sin(xn * 3.14159f * 2.0f) * size * 0.18f;
                const float waveDist = std::abs(static_cast<float>(y) - wy);
                if (waveDist < 1.6f && std::abs(dx) < ringInner * 0.95f) {
                    const float wAlpha = std::max(0.0f, std::min(1.0f, 1.6f - waveDist));
                    const unsigned char wv = static_cast<unsigned char>(wAlpha * 255.0f);
                    r = (unsigned char)std::min(255, r + wv);
                    g = (unsigned char)std::min(255, g + wv);
                    b = (unsigned char)std::min(255, b + wv);
                }
                // Faint horizontal centerline
                if (std::abs(static_cast<float>(y) - cy) < 0.7f && std::abs(dx) < ringInner * 0.85f) {
                    r = std::max<unsigned char>(r, 60);
                    g = std::max<unsigned char>(g, 110);
                    b = std::max<unsigned char>(b, 150);
                }
            }

            p[0] = r;
            p[1] = g;
            p[2] = b;
            p[3] = a;
        }
    }
    return px;
}

void printUsage()
{
    std::puts(
        "DriveScope\n"
        "  drivescope.exe [--config path]\n"
        "  drivescope.exe --capture path [--duration sec] [--out file.csv]\n"
        "  drivescope.exe --motor-blob <get|set-theta> --config path\n"
        "                 [--theta-rad N] [--timeout-ms N]\n"
        "\n"
        "Examples:\n"
        "  drivescope.exe --config tools/pc-tool/config/drivescope_wifi_debug.json\n"
        "  drivescope.exe --capture tools/pc-tool/config/drivescope_wifi_debug.json --duration 10 --out capture.csv\n"
        "  drivescope.exe --motor-blob get --config dist/config/drivescope_tcp_protocol_probe.json\n"
        "  drivescope.exe --motor-blob set-theta --theta-rad 6.20015 --config dist/config/drivescope_tcp_protocol_probe.json\n");
}

#ifdef _WIN32
/* DriveScope.exe is linked as /SUBSYSTEM:WINDOWS (GUI) so launching it
 * from Explorer doesn't pop a console. The CRT doesn't auto-bind
 * stdin/stdout/stderr to anything in GUI mode, so std::printf goes
 * nowhere by default. CLI invocations need explicit rewiring.
 *
 * Two cases to handle:
 *   (a) Launched from PowerShell/cmd interactively: the parent has a
 *       console and our std handles point at it. AttachConsole +
 *       freopen("CONOUT$") restores the natural printf flow.
 *   (b) Launched with redirected I/O (Start-Process -RedirectStandardOutput,
 *       Bash piped input, GitHub Actions): GetStdHandle returns the
 *       inherited pipe/file handle. We wrap those handles back into the
 *       CRT FILE* slots via _open_osfhandle so std::printf reaches them.
 *
 * Both cases coexist: AttachConsole succeeds case (a); the GetStdHandle
 * rewire works for both. Either way std::printf works after this returns. */
#include <io.h>
#include <fcntl.h>

void attachParentConsoleForCli()
{
    const HANDLE originalStdout = GetStdHandle(STD_OUTPUT_HANDLE);
    const HANDLE originalStderr = GetStdHandle(STD_ERROR_HANDLE);
    const HANDLE originalStdin  = GetStdHandle(STD_INPUT_HANDLE);

    auto isRedirected = [](HANDLE h) -> bool {
        if (h == INVALID_HANDLE_VALUE || h == NULL) return false;
        const DWORD type = GetFileType(h);
        return type == FILE_TYPE_PIPE || type == FILE_TYPE_DISK;
    };

    const bool stdoutRedirected = isRedirected(originalStdout);
    const bool stderrRedirected = isRedirected(originalStderr);
    const bool stdinRedirected  = isRedirected(originalStdin);

    const BOOL haveConsole =
        (!stdoutRedirected || !stderrRedirected || !stdinRedirected)
            ? AttachConsole(ATTACH_PARENT_PROCESS)
            : FALSE;

    auto rewire = [&](HANDLE original,
                      DWORD which,
                      const char* conPath,
                      FILE* crtFile,
                      const char* mode,
                      bool redirected) {
        HANDLE h = redirected ? original : GetStdHandle(which);
        if (h != INVALID_HANDLE_VALUE && h != NULL) {
            const int fd = _open_osfhandle(reinterpret_cast<intptr_t>(h),
                                           (mode[0] == 'r') ? _O_RDONLY : 0);
            if (fd >= 0) {
                FILE* f = _fdopen(fd, mode);
                if (f != nullptr) {
                    *crtFile = *f;
                    std::setvbuf(crtFile, nullptr, _IONBF, 0);
                    return;
                }
            }
        }
        if (haveConsole) {
            FILE* dummy = nullptr;
            freopen_s(&dummy, conPath, mode, crtFile);
            std::setvbuf(crtFile, nullptr, _IONBF, 0);
        }
    };

    rewire(originalStdout, STD_OUTPUT_HANDLE, "CONOUT$", stdout, "w", stdoutRedirected);
    rewire(originalStderr, STD_ERROR_HANDLE,  "CONOUT$", stderr, "w", stderrRedirected);
    rewire(originalStdin,  STD_INPUT_HANDLE,  "CONIN$",  stdin,  "r", stdinRedirected);
    /* Squelch any prior accumulated state from C++ iostreams. */
    std::ios::sync_with_stdio(true);
}
#else
inline void attachParentConsoleForCli() {}
#endif

std::filesystem::path executableDir(const char* argv0)
{
    namespace fs = std::filesystem;
#ifdef _WIN32
    wchar_t buf[MAX_PATH] = {};
    const DWORD n = GetModuleFileNameW(nullptr, buf, MAX_PATH);
    if (n > 0 && n < MAX_PATH) return fs::path(buf).parent_path();
#endif
    if (argv0 != nullptr && argv0[0] != '\0') {
        return fs::absolute(fs::path(argv0)).parent_path();
    }
    return fs::current_path();
}

std::filesystem::path defaultScenarioPath(const char* argv0)
{
    return executableDir(argv0) / "drivescope_scenario.json";
}
} // namespace

int main(int argc, char** argv)
{
    std::string guiConfigPath;
    std::string captureConfigPath;
    std::string captureOut = "drivescope_capture.csv";
    double captureDuration = 10.0;
    /* CLI motor-blob driver -- exercises the unified file-transfer
     * protocol from the same code base as the GUI; the same constants
     * and CRC routines as MainUi::sendMotorBlobSet are used so a CLI
     * pass guarantees the GUI tab will work end-to-end. */
    std::string motorBlobMode;
    double motorBlobTheta = 0.0;
    double motorBlobTimeoutSec = 2.0;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage();
            return 0;
        }
        if (arg == "--config" && i + 1 < argc) {
            guiConfigPath = argv[++i];
            continue;
        }
        if (arg == "--capture" && i + 1 < argc) {
            captureConfigPath = argv[++i];
            continue;
        }
        if (arg == "--duration" && i + 1 < argc) {
            captureDuration = std::atof(argv[++i]);
            continue;
        }
        if (arg == "--out" && i + 1 < argc) {
            captureOut = argv[++i];
            continue;
        }
        if (arg == "--motor-blob" && i + 1 < argc) {
            motorBlobMode = argv[++i];
            continue;
        }
        if (arg == "--theta-rad" && i + 1 < argc) {
            motorBlobTheta = std::atof(argv[++i]);
            continue;
        }
        if (arg == "--timeout-ms" && i + 1 < argc) {
            motorBlobTimeoutSec = std::atof(argv[++i]) / 1000.0;
            continue;
        }
        std::fprintf(stderr, "unknown or incomplete argument: %s\n", arg.c_str());
        printUsage();
        return 2;
    }

    if (!motorBlobMode.empty()) {
        attachParentConsoleForCli();
        if (guiConfigPath.empty()) {
            std::fprintf(stderr,
                         "--motor-blob requires --config <path-to-transport-config.json>\n");
            return 2;
        }
        drivescope::ToolConfig config;
        std::string error;
        if (!drivescope::loadToolConfig(guiConfigPath, config, error)) {
            std::fprintf(stderr, "config error: %s\n", error.c_str());
            return 2;
        }
        return drivescope::runMotorBlobCli(config, motorBlobMode,
                                           motorBlobTheta, motorBlobTimeoutSec);
    }

    if (!captureConfigPath.empty()) {
        attachParentConsoleForCli();
        drivescope::ToolConfig config;
        std::string error;
        if (!drivescope::loadToolConfig(captureConfigPath, config, error)) {
            std::fprintf(stderr, "config error: %s\n", error.c_str());
            return 2;
        }
        return drivescope::runHeadlessCapture(config, captureDuration, captureOut);
    }

    const std::filesystem::path scenarioPath =
        guiConfigPath.empty() ? defaultScenarioPath(argc > 0 ? argv[0] : nullptr)
                              : std::filesystem::path(guiConfigPath);

    drivescope::ToolConfig guiConfig;
    bool hasGuiConfig = false;
    if (!guiConfigPath.empty()) {
        std::string error;
        if (!drivescope::loadToolConfig(guiConfigPath, guiConfig, error)) {
            std::fprintf(stderr, "config error: %s\n", error.c_str());
            return 2;
        }
        hasGuiConfig = true;
    } else if (std::filesystem::exists(scenarioPath)) {
        std::string error;
        if (drivescope::loadToolConfig(scenarioPath.string(), guiConfig, error)) {
            hasGuiConfig = true;
        } else {
            std::fprintf(stderr, "scenario ignored: %s\n", error.c_str());
        }
    } else {
        const std::filesystem::path seedPath =
            scenarioPath.parent_path() / "config" / "drivescope_wifi_debug.json";
        if (std::filesystem::exists(seedPath)) {
            std::string error;
            if (drivescope::loadToolConfig(seedPath.string(), guiConfig, error)) {
                guiConfig.autoconnect = false;
                hasGuiConfig = true;
            } else {
                std::fprintf(stderr, "seed config ignored: %s\n", error.c_str());
            }
        }
    }

    if (!glfwInit()) {
        std::fprintf(stderr, "glfwInit failed\n");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
    GLFWwindow* window = glfwCreateWindow(1360, 820, "DriveScope", nullptr, nullptr);
    if (window == nullptr) {
        std::fprintf(stderr, "glfwCreateWindow failed\n");
        glfwTerminate();
        return 1;
    }
    glfwSetWindowSizeLimits(window, 760, 480, GLFW_DONT_CARE, GLFW_DONT_CARE);
#ifdef _WIN32
    installUndecoratedResize(window);
#endif
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    {
        std::vector<unsigned char> iconPixels32 = makeLogoIcon(32);
        std::vector<unsigned char> iconPixels48 = makeLogoIcon(48);
        GLFWimage icons[2];
        icons[0].width = 32;
        icons[0].height = 32;
        icons[0].pixels = iconPixels32.data();
        icons[1].width = 48;
        icons[1].height = 48;
        icons[1].pixels = iconPixels48.data();
        glfwSetWindowIcon(window, 2, icons);
    }

    // Center window on the primary monitor's work area.
    {
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();
        if (monitor != nullptr) {
            int mx = 0, my = 0, mw = 0, mh = 0;
            glfwGetMonitorWorkarea(monitor, &mx, &my, &mw, &mh);
            int ww = 0, wh = 0;
            glfwGetWindowSize(window, &ww, &wh);
            if (ww > mw) ww = mw - 80;
            if (wh > mh) wh = mh - 80;
            glfwSetWindowSize(window, ww, wh);
            const int px = mx + std::max(0, (mw - ww) / 2);
            const int py = my + std::max(0, (mh - wh) / 2);
            glfwSetWindowPos(window, px, py);
        }
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    // Docking branch features: tabs become dockable ImGui windows (DockingEnable),
    // and dragging one out of the main window spawns a real OS-level platform
    // window (ViewportsEnable) — needed for "see plots, motor, CAN at once".
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    // Don't auto-merge popped-out viewports back into the host whenever they
    // happen to overlap — once the operator pulled CAN Monitor out, it stays
    // out until they drag it back themselves.
    io.ConfigViewportsNoAutoMerge = true;
    ImGui::StyleColorsDark();
    applyDriveScopeStyle();
    applyDriveScopePlotStyle();
    loadDriveScopeFont(io);
    // Multi-viewport requires opaque, non-rounded window backgrounds otherwise
    // popped-out OS windows render with transparent corners over the desktop.
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        ImGuiStyle& style = ImGui::GetStyle();
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    drivescope::MainUi ui;
    ui.setNativeWindow(window);
    ui.setScenarioPath(scenarioPath.string());
    if (hasGuiConfig) ui.applyConfig(guiConfig);
    if (!std::filesystem::exists(scenarioPath)) {
        ui.saveScenarioNow("created");
    }
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ui.draw();

        ImGui::Render();
        int displayW = 0;
        int displayH = 0;
        glfwGetFramebufferSize(window, &displayW, &displayH);
        glViewport(0, 0, displayW, displayH);
        glClearColor(0.08f, 0.08f, 0.09f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Render any popped-out viewports (each one becomes a real OS window).
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
            GLFWwindow* backupCtx = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backupCtx);
        }

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
