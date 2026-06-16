#include "ui/MainUi.h"

#include "wlan/WindowsWlan.h"

#include "can/CanDecoder.h"
#include "config/ToolConfig.h"
#include "imgui.h"
#include "imgui_internal.h"   // ImGui::DockBuilder* APIs (multi-viewport docking)
#include "implot.h"
#include "implot_internal.h"

#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3.h>
#ifdef _WIN32
#include <GLFW/glfw3native.h>
#include <windows.h>
#include <commdlg.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>
#pragma comment(lib, "iphlpapi.lib")
#pragma comment(lib, "ws2_32.lib")
#endif

#include <future>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <fstream>
#include <cstring>
#include <map>
#include <mutex>
#include <sstream>
#include <thread>

namespace drivescope {

namespace {
// Forward declaration of vector-icon enum + drawer so MainUi::draw() can name
// them when wiring up the dockspace tab icons. The actual definitions live
// further down with the rest of the toolbar/tab-icon helpers.
enum class ToolIcon {
    Pause,
    Play,
    Clear,
    Autoscale,
    ResetY,
    Crosshair,
    Ruler,
    Follow,
    Fit,
    Save,
    Open,
    Trigger,          // lightning bolt -- triggered-capture shot
    // Tab icons (left of each tab label in the top tab strip).
    TabConnection,    // plug
    TabMonitor,       // stacked log lines
    TabVariables,     // brackets with dots
    TabPlots,         // sine-wave squiggle
    TabRemote,        // antenna with signal arcs
    TabCalibration,   // crosshair target / dial -- calibration tab
    TabUpdates,       // upload arrow into a chip
};
void drawToolIcon(ImDrawList* dl, ToolIcon icon, ImVec2 min, ImVec2 max, ImU32 color);

constexpr float kDefaultWatchPeriodSec = 0.1f;
constexpr float kMinWatchPeriodSec = 0.02f; // 50 Hz max for READ_MEM polling.
constexpr float kMaxWatchPeriodSec = 10.0f;
constexpr float kDefaultWatchHz = 1.0f / kDefaultWatchPeriodSec;
constexpr float kMinWatchHz = 1.0f / kMaxWatchPeriodSec;
constexpr float kMaxWatchHz = 1.0f / kMinWatchPeriodSec;
constexpr size_t kMaxPendingReadMem = 4;
constexpr size_t kMaxReadMemSendsPerPoll = 2;
constexpr double kReadMemTimeoutSec = 0.75;
constexpr double kReadMemRetrySec = 0.02;
/* Historical note below is obsolete; current code uses 1024-byte windows.
 * Bytes per READ_CHUNK request. This is used ONLY on the SLCAN
 * direct-CAN path: pc_tool talks straight to motor over the USB-CAN
 * dongle, no Wi-Fi in the loop, so streaming the whole buffer in one
 * READ_CHUNK is fine (motor caps internally to totalBytes; per-frame
 * pacing on the motor's TX FIFO handles bus arbitration).
 *
 * On the TCP/Wi-Fi path pc_tool does NOT use this loop -- it calls
 * the app_dd `/capture` HTTP endpoint which runs the entire capture
 * session locally over CAN and returns the result as one TCP body.
 * See docs/can-unified-file-transfer.md "РђСЂС…РёС‚РµРєС‚СѓСЂР° С‚СЂР°РЅСЃРїРѕСЂС‚Р°".
 *
 * 0xFFFE = max u16 - 1 (avoids 0xFFFF wraparound corner cases). */
/* Current bench limit: full 8 KiB one-shot READ_CHUNK can lose a CAP_DATA
 * frame in the SSD202D userspace bridge/TCP path. 1024-byte windows have
 * passed repeated full-buffer CRC tests and keep the UI out of fake timeout
 * recovery loops. */
constexpr uint16_t kCaptureReadChunkBytes = 1024;
/* Inter-chunk pacing: time host waits after DATA_END before firing the
 * next READ_CHUNK. Conservative 16 ms gap matches motor's
 * AppCaptureService drain budget -- one GUI tick of pollCaptureSession
 * to re-arm the request and dispatch via the deferred path. Aggressive
 * 2 ms gap + inline RX-callback dispatch was tried 2026-05-13 to push
 * throughput from ~20 % to ~40 % bus utilisation; on the live rig it
 * over-ran the motor's 20-deep prop_can SW TX FIFO when telemetry
 * broadcasts (Udc, fault, temp) shared the bus, causing stall-resume
 * loops at ~70-80 % of every 32 KB capture. 16 ms gap = ~19 KB/s, a
 * 32 KB capture finishes in ~1.7 s. Bump only after rate-limiting the
 * motor side (token bucket) so this can't overflow.  */
constexpr double kCaptureChunkGapSec = 0.016;
constexpr double kCaptureChunkStallSec = 0.8;
constexpr uint16_t kMotorCmdCtrl   = 0x002;
constexpr uint16_t kMotorCmdConfig = 0x005;
constexpr uint16_t kMotorAnsConfig = 0x0AB;
constexpr uint16_t kMotorAnsProfiler = 0x0AC;
constexpr uint16_t kMotorCmdTone   = 0x008;
constexpr uint8_t  kMotorCfgProfiler = 0x04;

/* Unified file-transfer READ protocol (2026-05-13). Motor FW codes
 * 0xD2..0xDF for the request + stream, 0xAD2..0xADF for the ACKs /
 * status. See docs/can-unified-file-transfer.md and motor
 * Core/Inc/prop_can.h::PROP_CAN_CMD_READ_HEADER_FILE. Single-shot,
 * single-block, max 16 KiB per call. */
constexpr uint16_t kFtCmdReadHeaderFile  = 0x0D2;
constexpr uint16_t kFtCmdReadHeaderBlock = 0x0D3;
constexpr uint16_t kFtCmdReadHeaderMmsg  = 0x0D4;
constexpr uint16_t kFtCmdReadDataMmsg    = 0x0D5;
constexpr uint16_t kFtCmdReadFileFinish  = 0x0DF;
constexpr uint16_t kFtAnsReadHeaderFileOk = 0xAD2;
constexpr uint16_t kFtAnsReadBlockOk      = 0xAD3;
constexpr uint16_t kFtAnsReadMmsgOk       = 0xAD4;
constexpr uint16_t kFtAnsReadError        = 0xAD8;
constexpr uint16_t kFtAnsReadFileOk       = 0xADF;

/* Unified file-transfer WRITE protocol (Phase 3, 2026-05-13).
 * Reuses the bootloader's wire codes 0xC0..0xCF (host в†’ MCU); in
 * app-mode (mod=1) on motor those route into prop_can.c's existing
 * propCanCmdProcess which has been retargeted: when HEADER_FILE.addr
 * equals MOTOR_CONFIG_FLASH_ADDR (0x080E0000), the bytes stay in the
 * staging bufBlock instead of being flashed; on FILE_FINISH the wire
 * CRC32 is verified against bufBlock and MotorConfigScatter applies
 * the payload to live RAM. Subsequent "Save to flash" still goes
 * through param-table SAVE_ALL, which is the only flash-write path. */
constexpr uint16_t kFtCmdHeaderFile  = 0x0C0;
constexpr uint16_t kFtCmdHeaderBlock = 0x0C1;
constexpr uint16_t kFtCmdHeaderMmsg  = 0x0C2;
constexpr uint16_t kFtCmdDataMmsg    = 0x0C3;
constexpr uint16_t kFtCmdEraseFlash  = 0x0CE;
constexpr uint16_t kFtCmdFileFinish  = 0x0CF;
constexpr uint16_t kFtAnsError       = 0xAC0;
constexpr uint16_t kFtAnsEraseOk     = 0xAC1;
constexpr uint16_t kFtAnsMmsgOk      = 0xAC2;
constexpr uint16_t kFtAnsBlockOk     = 0xAC3;
constexpr uint16_t kFtAnsFileOk      = 0xACF;
constexpr uint16_t kFtMmsgMaxDataSize     = 1792u;   /* 256 msgs * 7 B */
constexpr uint8_t  kFtMsgDataSize         = 7u;
/* motor_config flash sector 11 base address вЂ” same as
 * motor_config.h::MOTOR_CONFIG_FLASH_ADDR. Used as the canonical
 * read source for the config blob. */
constexpr uint32_t kMotorConfigFlashAddr = 0x080E0000u;

// Per the motor firmware, tone is hard-clamped to:
//   freq    [30, 1500]  Hz
//   duration[1, 2000]   ms
//   amp     [0, 25]     percent
// Anything outside is silently saturated, but we already feed valid values.

struct ToneNote {
    uint16_t freq_hz;       // 0 = REST
    uint16_t duration_ms;
};
struct Melody {
    const char*     name;
    const ToneNote* notes;
    size_t          count;
    int             default_internote_ms;
};

// ---- Note frequency table (equal temperament, A4 = 440 Hz) -----------------
// Only the notes we actually use; values rounded to the nearest Hz which is
// well below the 30..1500 Hz firmware clamp resolution.
namespace ToneFreq {
constexpr uint16_t REST = 0;
constexpr uint16_t C2 = 65,   D2 = 73,   E2 = 82,   F2 = 87,   G2 = 98,   A2 = 110,  B2 = 123;
constexpr uint16_t C3 = 131,  D3 = 147,  E3 = 165,  F3 = 175,  G3 = 196,  A3 = 220,  B3 = 247;
constexpr uint16_t C4 = 262,  CS4 = 277, D4 = 294,  DS4 = 311, E4 = 330,  F4 = 349,  FS4 = 370,
                   G4 = 392,  GS4 = 415, A4 = 440,  AS4 = 466, B4 = 494;
constexpr uint16_t C5 = 523,  CS5 = 554, D5 = 587,  DS5 = 622, E5 = 659,  F5 = 698,  FS5 = 740,
                   G5 = 784,  GS5 = 831, A5 = 880,  AS5 = 932, B5 = 988;
constexpr uint16_t C6 = 1047, D6 = 1175, E6 = 1319;
}  // namespace ToneFreq

// ---- Melody data ----------------------------------------------------------
namespace TF = ToneFreq;
namespace {

constexpr ToneNote kMelodyMario[] = {
    {TF::E5,120},{TF::E5,120},{TF::REST,120},{TF::E5,120},
    {TF::REST,120},{TF::C5,120},{TF::E5,120},{TF::REST,120},
    {TF::G5,240},{TF::REST,240},
    {TF::G4,240},{TF::REST,240},
    {TF::C5,200},{TF::REST,100},{TF::G4,200},{TF::REST,200},
    {TF::E4,200},{TF::REST,100},
    {TF::A4,180},{TF::B4,180},{TF::AS4,100},{TF::A4,200},
    {TF::G4,150},{TF::E5,150},{TF::G5,150},{TF::A5,200},
    {TF::F5,150},{TF::G5,100},{TF::REST,100},
    {TF::E5,200},{TF::C5,150},{TF::D5,150},{TF::B4,240},
};

// Tetris (Korobeiniki) opening вЂ” instantly recognisable, tempo ~150 BPM.
constexpr ToneNote kMelodyTetris[] = {
    {TF::E5,200},{TF::B4,100},{TF::C5,100},{TF::D5,200},{TF::C5,100},{TF::B4,100},
    {TF::A4,200},{TF::A4,100},{TF::C5,100},{TF::E5,200},{TF::D5,100},{TF::C5,100},
    {TF::B4,300},{TF::C5,100},{TF::D5,200},{TF::E5,200},
    {TF::C5,200},{TF::A4,200},{TF::A4,200},{TF::REST,100},
    {TF::D5,200},{TF::F5,100},{TF::A5,200},{TF::G5,100},{TF::F5,100},
    {TF::E5,300},{TF::C5,100},{TF::E5,200},{TF::D5,100},{TF::C5,100},
    {TF::B4,200},{TF::B4,100},{TF::C5,100},{TF::D5,200},{TF::E5,200},
    {TF::C5,200},{TF::A4,200},{TF::A4,200},
};

// Imperial March (Star Wars) вЂ” ~120 BPM, signature dotted-quarter rhythm.
constexpr ToneNote kMelodyImperial[] = {
    {TF::A4,400},{TF::A4,400},{TF::A4,400},
    {TF::F4,300},{TF::C5,100},
    {TF::A4,400},{TF::F4,300},{TF::C5,100},{TF::A4,650},{TF::REST,150},
    {TF::E5,400},{TF::E5,400},{TF::E5,400},
    {TF::F5,300},{TF::C5,100},
    {TF::GS4,400},{TF::F4,300},{TF::C5,100},{TF::A4,650},
};

// Doot ("look at me I'm a motor") -- a friendly 3-tone power-up.
constexpr ToneNote kMelodyPowerUp[] = {
    {TF::C5,100},{TF::E5,100},{TF::G5,100},{TF::C6,200},
    {TF::G5,100},{TF::C6,300},
};

// Single-note 1 sec beep at 440 Hz -- factory smoke test "yes it works".
constexpr ToneNote kMelodyBeep[] = {
    {TF::A4,1000},
};

// Happy Birthday (transposed to C-major, simple version).
constexpr ToneNote kMelodyHappyBday[] = {
    {TF::C4,200},{TF::C4,200},{TF::D4,400},{TF::C4,400},{TF::F4,400},{TF::E4,800},
    {TF::C4,200},{TF::C4,200},{TF::D4,400},{TF::C4,400},{TF::G4,400},{TF::F4,800},
    {TF::C4,200},{TF::C4,200},{TF::C5,400},{TF::A4,400},{TF::F4,400},{TF::E4,400},{TF::D4,800},
    {TF::AS4,200},{TF::AS4,200},{TF::A4,400},{TF::F4,400},{TF::G4,400},{TF::F4,800},
};

constexpr Melody kMelodies[] = {
    {"Super Mario Bros",  kMelodyMario,     sizeof(kMelodyMario)/sizeof(ToneNote),     35},
    {"Tetris (Korobeiniki)", kMelodyTetris, sizeof(kMelodyTetris)/sizeof(ToneNote),    25},
    {"Imperial March",    kMelodyImperial,  sizeof(kMelodyImperial)/sizeof(ToneNote),  35},
    {"Power Up",          kMelodyPowerUp,   sizeof(kMelodyPowerUp)/sizeof(ToneNote),    20},
    {"Happy Birthday",    kMelodyHappyBday, sizeof(kMelodyHappyBday)/sizeof(ToneNote), 30},
    {"Beep (1 s @ 440 Hz)", kMelodyBeep,    sizeof(kMelodyBeep)/sizeof(ToneNote),       0},
};
constexpr int kMelodyCount = (int)(sizeof(kMelodies)/sizeof(Melody));

}  // namespace

constexpr uint8_t kMotorCfgThetaOffset = 0x00;
constexpr uint8_t kMotorCfgThetaSector = 0x01;  /* FW dispatcher returns BAD_ARG; kept to avoid touching callers. */
constexpr uint8_t kMotorCfgParam       = 0x02;
constexpr uint8_t kMotorCfgBlob        = 0x03;
/* Old per-field theta actions return BAD_ARG on the FW side since
 * 2026-05-13 -- whole-blob GET/SET replaces them. Constants kept for
 * source-compat with the (now-dead) sendMotorThetaConfig / Sector / poll
 * code paths; new flows must call sendMotorBlobGet / sendMotorBlobSet. */
constexpr uint8_t kMotorThetaGet        = 0;
constexpr uint8_t kMotorThetaSet        = 1;
constexpr uint8_t kMotorThetaSetSave    = 2;
constexpr uint8_t kMotorThetaCal        = 3;
constexpr uint8_t kMotorThetaCalSave    = 4;
constexpr uint8_t kMotorThetaSave       = 5;
constexpr uint8_t kMotorThetaClear      = 6;
constexpr uint8_t kMotorThetaGetSector  = 7;
constexpr uint8_t kMotorThetaGuard0     = 0xA5;
constexpr uint8_t kMotorThetaGuard1     = 0x5A;

/* Whole-config-blob transfer. Constants mirror app_motor.h. */
constexpr uint8_t  kMotorBlobActGet               = 0;
constexpr uint8_t  kMotorBlobActGetResponseChunk  = 1;
constexpr uint8_t  kMotorBlobActSetChunk          = 2;
constexpr uint8_t  kMotorBlobActSetResult         = 3;
constexpr uint8_t  kMotorCfgBlobChunkBytes        = 5u;
constexpr uint16_t kMotorCfgBlobTotalBytes        = 164u;     /* hdr 16 + payload 148 */
constexpr uint8_t  kMotorCfgBlobChunkCount        = 33u;
constexpr uint16_t kMotorCfgBlobPayloadOffset     = 16u;      /* first byte of payload inside blob */
constexpr uint16_t kMotorCfgBlobPayloadBytes      = 148u;
constexpr uint16_t kMotorBlobOffThetaOffset       = kMotorCfgBlobPayloadOffset + 0u;   /* float */
constexpr uint16_t kMotorBlobOffThetaSectorLut    = kMotorCfgBlobPayloadOffset + 4u;   /* 6 floats */
constexpr uint16_t kMotorBlobOffPiKpId            = kMotorCfgBlobPayloadOffset + 28u;
constexpr uint16_t kMotorBlobOffPiKiId            = kMotorCfgBlobPayloadOffset + 32u;
constexpr uint16_t kMotorBlobOffPiKpIq            = kMotorCfgBlobPayloadOffset + 36u;
constexpr uint16_t kMotorBlobOffPiKiIq            = kMotorCfgBlobPayloadOffset + 40u;
constexpr uint16_t kMotorBlobOffPiKpOmega         = kMotorCfgBlobPayloadOffset + 44u;
constexpr uint16_t kMotorBlobOffPiKiOmega         = kMotorCfgBlobPayloadOffset + 48u;
constexpr uint16_t kMotorBlobOffPayloadCrc        = 12u;      /* inside header */
constexpr uint32_t kMotorBlobMagic                = 0xC0FEEDDAu;
constexpr uint8_t kMotorThetaFlagSaved      = 1u << 0;
constexpr uint8_t kMotorThetaFlagLoaded     = 1u << 1;
constexpr uint8_t kMotorThetaFlagCalActive  = 1u << 2;
constexpr uint8_t kMotorThetaFlagSaveReq    = 1u << 3;
constexpr uint8_t kMotorThetaFlagSectorCal  = 1u << 4;

/* Param-table id -> staged motorBlob_ byte offset. Lives at file scope
 * so both applyMotorConfigBlobSet() (~line 9k, well above the Motor
 * Config tab block) and populateMotorConfigCacheFromBlob() (~line 11k)
 * can see the same definition; anonymous namespaces in different
 * translation-unit spans don't merge, so the previous attempt to keep
 * this next to MotorConfigParamDef + forward-declare it here produced
 * an ambiguous overload. Kept as static (internal linkage) to mirror
 * what the anonymous namespace was doing.
 *
 * Returns -1 for ids that are not blob-resident (DIAG counters). */
static int motorConfigBlobOffsetForId(uint16_t id)
{
    switch (id) {
        case 0:  return kMotorBlobOffThetaOffset;             /* theta.offset      */
        case 1:  return kMotorBlobOffThetaSectorLut + 0 * 4;  /* theta.sector_0    */
        case 2:  return kMotorBlobOffThetaSectorLut + 1 * 4;
        case 3:  return kMotorBlobOffThetaSectorLut + 2 * 4;
        case 4:  return kMotorBlobOffThetaSectorLut + 3 * 4;
        case 5:  return kMotorBlobOffThetaSectorLut + 4 * 4;
        case 6:  return kMotorBlobOffThetaSectorLut + 5 * 4;
        case 10: return kMotorBlobOffPiKpId;                  /* PI gains          */
        case 11: return kMotorBlobOffPiKiId;
        case 12: return kMotorBlobOffPiKpIq;
        case 13: return kMotorBlobOffPiKiIq;
        case 14: return kMotorBlobOffPiKpOmega;
        case 15: return kMotorBlobOffPiKiOmega;
        case 20: return kMotorCfgBlobPayloadOffset + 52;      /* profiler.rs       */
        case 21: return kMotorCfgBlobPayloadOffset + 56;      /* profiler.ld       */
        case 22: return kMotorCfgBlobPayloadOffset + 60;      /* profiler.lq       */
        case 23: return kMotorCfgBlobPayloadOffset + 64;      /* profiler.lambda   */
        case 24: return kMotorCfgBlobPayloadOffset + 68;      /* profiler.j        */
        case 25: return kMotorCfgBlobPayloadOffset + 72;      /* profiler.b        */
        case 26: return kMotorCfgBlobPayloadOffset + 76;      /* profiler.valid    */
        case 27: return kMotorCfgBlobPayloadOffset + 80;      /* profiler.uptime   */
        case 37: return 4;                                    /* cfg.fw_ver (hdr)  */
        case 38: return kMotorBlobOffPayloadCrc;              /* cfg.crc    (hdr)  */
        case 50: return kMotorCfgBlobPayloadOffset + 84;      /* pfc.enabled       */
        default: return -1;
    }
}

struct PlotScaleOption {
    const char* label;
    double value;
};

constexpr PlotScaleOption kPlotScaleOptions[] = {
    { "/1000000", 0.000001 },
    { "/100000",  0.00001 },
    { "/10000",   0.0001 },
    { "/1000",    0.001 },
    { "/100", 0.01 },
    { "/10",  0.1 },
    { "/5",   0.2 },
    { "/2",   0.5 },
    { "x1",   1.0 },
    { "x2",   2.0 },
    { "x5",   5.0 },
    { "x10",  10.0 },
    { "x100", 100.0 },
    { "x1000", 1000.0 },
    { "x10000", 10000.0 },
    { "x100000", 100000.0 },
    { "x1000000", 1000000.0 },
};

double normalizePlotScale(double value)
{
    return std::isfinite(value) && value > 0.0 ? value : 1.0;
}

bool samePlotScale(double lhs, double rhs)
{
    return std::abs(lhs - rhs) < 1e-12;
}

std::string plotScaleLabel(double value)
{
    value = normalizePlotScale(value);
    for (const PlotScaleOption& option : kPlotScaleOptions) {
        if (samePlotScale(value, option.value)) return option.label;
    }

    char buf[32] = {};
    if (value < 1.0) {
        std::snprintf(buf, sizeof(buf), "/%.6g", 1.0 / value);
    } else {
        std::snprintf(buf, sizeof(buf), "x%.6g", value);
    }
    return buf;
}

std::string plotSeriesLabel(const std::string& name, double scale)
{
    if (samePlotScale(normalizePlotScale(scale), 1.0)) return name;
    return name + " (" + plotScaleLabel(scale) + ")";
}

float clampWatchPeriodSec(float sec)
{
    if (!std::isfinite(sec)) return kDefaultWatchPeriodSec;
    return std::clamp(sec, kMinWatchPeriodSec, kMaxWatchPeriodSec);
}

float clampWatchHz(float hz)
{
    if (!std::isfinite(hz)) return kDefaultWatchHz;
    return std::clamp(hz, kMinWatchHz, kMaxWatchHz);
}

float watchHzFromPeriodSec(float sec)
{
    return 1.0f / clampWatchPeriodSec(sec);
}

float periodSecFromWatchHz(float hz)
{
    return 1.0f / clampWatchHz(hz);
}

struct ByteGap {
    size_t start = 0;
    size_t end = 0;
    bool found = false;
};

ByteGap findMissingBytes(const std::vector<bool>& seen, size_t begin, size_t end)
{
    ByteGap gap;
    if (begin > seen.size()) begin = seen.size();
    if (end > seen.size()) end = seen.size();
    for (size_t i = begin; i < end; ++i) {
        if (!seen[i]) {
            gap.start = i;
            gap.end = i + 1u;
            while (gap.end < end && !seen[gap.end]) {
                ++gap.end;
            }
            gap.found = true;
            break;
        }
    }
    return gap;
}

// Per-node Triggered-Capture limits. Numbers must mirror what the MCU
// firmware allocates (motor: 32 KB CCMRAM ring + 100 us min sampler period;
// RK: 8 KB SRAM ring + 500 us TIM6 piggy-back; ESP32: 16 KB DRAM ring +
// 100 us esp_timer floor). If any of those change in firmware, update here
// in lockstep -- the UI uses these to advertise valid duration ranges.
struct CaptureBounds {
    uint32_t bufferBytes = 32u * 1024u;
    uint16_t periodMinUs = 100;
    uint16_t periodMaxUs = 1000;
    int      minMs = 1;
    int      maxMs = 1000;
    uint16_t periodUs = 250;   // chosen to match a given durationMs
    uint8_t  bufferKb = 32;
    uint32_t bufSamples = 0;
};

CaptureBounds computeCaptureBounds(uint8_t nodeId, uint8_t slotCount, int requestedMs)
{
    CaptureBounds b;
    if (nodeId == kCanAddrRk) {
        b.bufferBytes = 8u * 1024u;
        b.periodMinUs = 500;
    } else if (nodeId == kCanAddrEsp32) {
        b.bufferBytes = 16u * 1024u;
        b.periodMinUs = 100;
    } else {
        // motor / unknown -> motor profile
        b.bufferBytes = 32u * 1024u;
        b.periodMinUs = 100;
    }
    b.periodMaxUs = 1000;

    const uint8_t slots = slotCount == 0 ? 1 : slotCount;
    const uint32_t bytesPerSample = static_cast<uint32_t>(slots) * 4u;
    b.bufSamples = b.bufferBytes / bytesPerSample;
    if (b.bufSamples == 0) b.bufSamples = 1;

    const double minMs = std::ceil(static_cast<double>(b.bufSamples) *
                                   static_cast<double>(b.periodMinUs) / 1000.0);
    const double maxMs = std::floor(static_cast<double>(b.bufSamples) *
                                    static_cast<double>(b.periodMaxUs) / 1000.0);
    b.minMs = std::max(1, static_cast<int>(minMs));
    b.maxMs = std::max(b.minMs, static_cast<int>(maxMs));

    int targetMs = std::clamp(requestedMs, b.minMs, b.maxMs);
    const double periodUs = std::round(static_cast<double>(targetMs) * 1000.0 /
                                       static_cast<double>(b.bufSamples));
    b.periodUs = static_cast<uint16_t>(std::clamp<double>(periodUs,
                                                          b.periodMinUs, b.periodMaxUs));
    b.bufferKb = static_cast<uint8_t>(std::min<uint32_t>(b.bufferBytes / 1024u, 64u));
    return b;
}

uint8_t sizeForType(const std::string& type)
{
    if (type == "uint8" || type == "int8") return 1;
    if (type == "uint16" || type == "int16") return 2;
    if (type == "double") return 8;
    return 4;
}

bool parseOptionalIdFilter(const char* text, uint32_t& id)
{
    if (text == nullptr || text[0] == '\0') return false;
    char* end = nullptr;
    const unsigned long value = std::strtoul(text, &end, 0);
    if (end == text) return false;
    while (*end == ' ' || *end == '\t') ++end;
    if (*end != '\0') return false;
    id = static_cast<uint32_t>(value);
    return true;
}

uint32_t pendingKey(uint8_t nodeId, uint8_t seq)
{
    return (static_cast<uint32_t>(nodeId) << 8) | seq;
}

std::string quoteArg(const std::string& value)
{
    std::string out = "\"";
    for (char c : value) {
        if (c == '"') out += "\\\"";
        else out += c;
    }
    out += "\"";
    return out;
}

std::string shellSingleQuote(const std::string& value)
{
    std::string out = "'";
    for (char c : value) {
        if (c == '\'') out += "'\\''";
        else out += c;
    }
    out += "'";
    return out;
}

bool endsWithInsensitive(std::string value, std::string suffix)
{
    auto lower = [](std::string& s) {
        for (char& c : s) {
            if (c >= 'A' && c <= 'Z') c = static_cast<char>(c + 32);
        }
    };
    lower(value);
    lower(suffix);
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

using ScriptEnv = std::vector<std::pair<std::string, std::string>>;

std::string shellEnvPrefix(const ScriptEnv& env)
{
    std::string out;
    for (const auto& kv : env) {
        if (kv.first.empty()) continue;
        out += kv.first;
        out += "=";
        out += shellSingleQuote(kv.second);
        out += " ";
    }
    return out;
}

std::string commandForScript(const std::string& scriptPath, const ScriptEnv& env = {})
{
    namespace fs = std::filesystem;
    if (scriptPath.rfind("\\\\wsl.localhost\\", 0) == 0 ||
        scriptPath.rfind("\\\\wsl$\\", 0) == 0) {
        const std::string prefix = scriptPath.rfind("\\\\wsl.localhost\\", 0) == 0
                                 ? "\\\\wsl.localhost\\"
                                 : "\\\\wsl$\\";
        std::string rest = scriptPath.substr(prefix.size());
        const size_t slash = rest.find('\\');
        if (slash != std::string::npos) {
            const std::string distro = rest.substr(0, slash);
            std::string linuxPath = "/" + rest.substr(slash + 1);
            std::replace(linuxPath.begin(), linuxPath.end(), '\\', '/');
            fs::path lp(linuxPath);
            const std::string dir = lp.parent_path().generic_string();
            const std::string file = lp.filename().generic_string();
            return "wsl.exe -d " + quoteArg(distro) + " bash -lc " +
                   quoteArg("cd " + shellSingleQuote(dir) + " && " +
                            shellEnvPrefix(env) + "./" + shellSingleQuote(file));
        }
    }
    if (!env.empty() && endsWithInsensitive(scriptPath, ".sh")) {
        return "bash -lc " + quoteArg(shellEnvPrefix(env) + "bash " + shellSingleQuote(scriptPath));
    }
    if (endsWithInsensitive(scriptPath, ".sh")) {
        return "bash " + quoteArg(scriptPath);
    }
    return quoteArg(scriptPath);
}

std::string jsonEscape(const std::string& value)
{
    std::string out;
    out.reserve(value.size() + 8);
    for (char c : value) {
        if (c == '"' || c == '\\') {
            out += '\\';
            out += c;
        } else if (c == '\n') {
            out += "\\n";
        } else if (c == '\r') {
            out += "\\r";
        } else if (c == '\t') {
            out += "\\t";
        } else {
            out += c;
        }
    }
    return out;
}

/* Locate the directory that holds the running DriveScope executable.
 * GetModuleFileNameW with HMODULE=nullptr returns the .exe path even
 * when the binary was launched from elsewhere via a shortcut. Falls
 * back to the cwd as a last resort. Mirrored from main.cpp's
 * executableDir but kept local so the symbol-loader doesn't depend on
 * the entrypoint. */
std::filesystem::path runningExeDir()
{
    namespace fs = std::filesystem;
#ifdef _WIN32
    wchar_t buf[MAX_PATH] = {};
    const DWORD n = GetModuleFileNameW(nullptr, buf, MAX_PATH);
    if (n > 0 && n < MAX_PATH) return fs::path(buf).parent_path();
#endif
    std::error_code ec;
    return fs::current_path(ec);
}

/* Walk up from `start`, looking for a path that has `<dir>/<rel>` and
 * return that resolved path. Used to find `tools/pc-tool/dist/elf_export.exe`
 * relative to the running .exe regardless of whether the user runs from
 * `dist/` (release) or `build/pc-tool/` (dev build) or anywhere under
 * the umbrella root. */
std::filesystem::path findUpward(const std::filesystem::path& start,
                                 const char* rel)
{
    namespace fs = std::filesystem;
    fs::path cur = start;
    std::error_code ec;
    for (int hops = 0; hops < 8 && !cur.empty(); ++hops) {
        fs::path candidate = cur / rel;
        if (fs::exists(candidate, ec)) return candidate;
        const fs::path parent = cur.parent_path();
        if (parent == cur) break;
        cur = parent;
    }
    return {};
}

std::string findElfExporter()
{
    namespace fs = std::filesystem;
    /* Resolve relative to the executable first (this works whether the
     * binary lives in dist/ or build/pc-tool/), then fall back to cwd
     * candidates for legacy invocations. */
    const fs::path exeDir = runningExeDir();
    const char* upward[] = {
        "tools/elf_export.py",
        "tools/pc-tool/../elf_export.py",   /* unlikely but harmless */
    };
    for (const char* rel : upward) {
        const fs::path p = findUpward(exeDir, rel);
        if (!p.empty()) return p.string();
    }
    const char* cwdRel[] = {
        "tools/elf_export.py",
        "../tools/elf_export.py",
        "../../tools/elf_export.py",
        "../../../tools/elf_export.py",
    };
    for (const char* candidate : cwdRel) {
        if (fs::exists(candidate)) return candidate;
    }
    return "tools/elf_export.py";
}

std::string findElfPython()
{
    namespace fs = std::filesystem;
    /* The portable PyInstaller-bundled exporter is the preferred path
     * because it doesn't depend on Python being on PATH. Search the
     * running exe's directory + walk up the tree to find it -- handles
     * both `dist/elf_export.exe` (release) and
     * `<umbrella>/tools/pc-tool/dist/elf_export.exe` (dev build run from
     * build/pc-tool/). */
    const fs::path exeDir = runningExeDir();

    {
        const fs::path next = exeDir / "elf_export.exe";
        std::error_code ec;
        if (fs::exists(next, ec)) return next.string();
    }
    /* Walk up looking for the bundled exporter at the canonical relative
     * location it lives at after `cmake --build`. */
    {
        const fs::path p = findUpward(exeDir, "tools/pc-tool/dist/elf_export.exe");
        if (!p.empty()) return p.string();
    }
    /* Try project-local venv pythons next, also relative to exe. */
    const char* upward[] = {
        "tools/pc-tool/.venv/Scripts/python.exe",
        "tools/pc-tool/.venv/bin/python.exe",
        ".venv/Scripts/python.exe",
        ".venv/bin/python.exe",
    };
    for (const char* rel : upward) {
        const fs::path p = findUpward(exeDir, rel);
        if (!p.empty()) return p.string();
    }
    /* Legacy cwd-relative candidates (preserves behaviour for tests/CI
     * that cd into the source tree before launching). */
    const char* cwdRel[] = {
        "elf_export.exe",
        "tools/pc-tool/dist/elf_export.exe",
        "tools/pc-tool/elf_export.exe",
        "build/pc-tool/.venv/bin/python.exe",
        "build/pc-tool/.venv/Scripts/python.exe",
        "../.venv/bin/python.exe",
        "../.venv/Scripts/python.exe",
        ".venv/bin/python.exe",
        ".venv/Scripts/python.exe",
    };
    for (const char* candidate : cwdRel) {
        if (fs::exists(candidate)) return candidate;
    }
    return "python";
}

bool isStandaloneExporter(const std::string& path)
{
    const std::string lower = [&]{
        std::string s = path;
        for (char& c : s) if (c >= 'A' && c <= 'Z') c = static_cast<char>(c + 32);
        return s;
    }();
    return lower.find("elf_export.exe") != std::string::npos;
}

#ifdef _WIN32
bool nativeFileDialog(bool save, const char* filter, std::string& path)
{
    namespace fs = std::filesystem;
    wchar_t buf[1024] = {0};

    // Normalize away `..` segments and resolve to an absolute path. Without
    // this, paths like `D:/.../build/pc-tool/../SSD202D/OTA/SStarOta.bin.gz`
    // (an absolute path containing `..`) confuse GetOpenFileNameW -- it
    // either silently bails before showing the dialog or opens it behind
    // the main window with no preselection. lexically_normal() collapses
    // `..` so we always feed in a clean path.
    fs::path initial;
    if (!path.empty()) {
        std::error_code ec;
        initial = fs::absolute(fs::path(path), ec).lexically_normal();
        if (ec) initial = fs::path(path).lexically_normal();
    }

    // Pre-set the suggested filename + initial directory separately. If the
    // file's parent doesn't exist, drop it to let Windows pick a sensible
    // default (Documents) instead of failing.
    std::wstring initialDirW;
    if (!initial.empty()) {
        std::wstring ws = initial.wstring();
        // Convert forward slashes -- GetOpenFileNameW only likes backslashes.
        for (auto& c : ws) if (c == L'/') c = L'\\';
        if (ws.size() < 1023) std::wcscpy(buf, ws.c_str());

        std::error_code ec;
        fs::path parent = initial.parent_path();
        if (!parent.empty() && fs::is_directory(parent, ec)) {
            initialDirW = parent.wstring();
            for (auto& c : initialDirW) if (c == L'/') c = L'\\';
        }
    }

    OPENFILENAMEW ofn = {};
    ofn.lStructSize = sizeof(ofn);
    // Prefer the focused/foreground window so the dialog opens in front.
    // GetActiveWindow returns the calling thread's active window, which can
    // be NULL when the click came from an ImGui-docked viewport -- then the
    // file dialog appears as a topmost orphan, sometimes hidden behind the
    // main window. Fall back through GetForegroundWindow в†’ NULL.
    HWND owner = GetActiveWindow();
    if (!owner) owner = GetForegroundWindow();
    ofn.hwndOwner = owner;

    // Filter is multistring "Description\0*.ext\0\0".
    static thread_local std::wstring filterW;
    filterW.clear();
    if (filter) {
        for (const char* p = filter; ; ) {
            size_t n = std::strlen(p);
            filterW.append(p, p + n);
            filterW.push_back(L'\0');
            p += n + 1;
            if (*p == '\0') break;
        }
    } else {
        filterW = L"All files\0*.*\0";
        filterW.push_back(L'\0');
    }
    filterW.push_back(L'\0');
    ofn.lpstrFilter = filterW.c_str();
    ofn.lpstrFile = buf;
    ofn.nMaxFile = 1023;
    if (!initialDirW.empty()) ofn.lpstrInitialDir = initialDirW.c_str();
    ofn.Flags = OFN_EXPLORER | OFN_NOCHANGEDIR;
    if (save) {
        ofn.Flags |= OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST;
        if (!GetSaveFileNameW(&ofn)) return false;
    } else {
        ofn.Flags |= OFN_FILEMUSTEXIST;
        if (!GetOpenFileNameW(&ofn)) return false;
    }
    int needed = WideCharToMultiByte(CP_UTF8, 0, buf, -1, nullptr, 0, nullptr, nullptr);
    if (needed <= 0) return false;
    std::string utf8(static_cast<size_t>(needed) - 1, '\0');
    WideCharToMultiByte(CP_UTF8, 0, buf, -1, utf8.data(), needed, nullptr, nullptr);
    path = std::move(utf8);
    return true;
}
#else
bool nativeFileDialog(bool, const char*, std::string&) { return false; }
#endif

std::string exeDir()
{
#ifdef _WIN32
    wchar_t buf[MAX_PATH];
    DWORD n = GetModuleFileNameW(nullptr, buf, MAX_PATH);
    if (n == 0 || n >= MAX_PATH) return {};
    namespace fs = std::filesystem;
    return fs::path(buf).parent_path().string();
#else
    return {};
#endif
}

std::string defaultRecoveryAppDir()
{
    namespace fs = std::filesystem;
    const std::string ed = exeDir();
    const std::string candidates[] = {
        ed + "/../SSD202D/app",
        ed + "/../../SSD202D/app",
        "//wsl.localhost/Ubuntu_DDV2/home/fermiums/linux_ssd202d_dd/sdk/app_ssd202d_ddv2/app",
    };
    for (const std::string& c : candidates) {
        if (!c.empty() && fs::is_directory(fs::path(c))) return fs::path(c).string();
    }
    return ed.empty() ? std::string("../SSD202D/app") : (ed + "/../SSD202D/app");
}

std::string defaultRecoveryOtaPath()
{
    namespace fs = std::filesystem;
    const std::string ed = exeDir();
    const std::string candidates[] = {
        ed + "/../SSD202D/OTA/SStarOta.bin.gz",
        ed + "/../../SSD202D/OTA/SStarOta.bin.gz",
    };
    for (const std::string& c : candidates) {
        if (!c.empty() && fs::is_regular_file(fs::path(c))) return fs::path(c).string();
    }
    return ed.empty() ? std::string("../SSD202D/OTA/SStarOta.bin.gz")
                      : (ed + "/../SSD202D/OTA/SStarOta.bin.gz");
}

std::string defaultEsp32FirmwareDir()
{
    namespace fs = std::filesystem;
    const std::string ed = exeDir();
    const std::string candidates[] = {
        ed + "/../SSD202D/app/firmware/esp32",
        ed + "/../../SSD202D/app/firmware/esp32",
        ed + "/../ESP32/bin",
        ed + "/../../ESP32/bin",
        "//wsl.localhost/Ubuntu_DDV2/home/fermiums/linux_ssd202d_dd/sdk/app_ssd202d_ddv2/app/firmware/esp32",
        "D:/w_spase/DDV2/mainPCB/app_esp32_dd/.pio/build/esp32dev",
    };
    for (const std::string& c : candidates) {
        if (!c.empty() && fs::is_directory(fs::path(c))) return fs::path(c).string();
    }
    return ed.empty() ? std::string("../SSD202D/app/firmware/esp32")
                      : (ed + "/../SSD202D/app/firmware/esp32");
}

std::string resolveDataPath(const std::string& path)
{
    namespace fs = std::filesystem;
    if (path.empty()) return path;
    if (fs::exists(path)) return path;
    const std::string base = fs::path(path).filename().string();
    const std::string ed = exeDir();
    const std::string candidates[] = {
        path,
        ed + "/" + path,
        std::string("../") + path,
        std::string("../../") + path,
        std::string("../../../") + path,
        std::string("build/pc-tool/") + base,
        std::string("../build/pc-tool/") + base,
        std::string("../../build/pc-tool/") + base,
        ed + "/" + base,
        ed + "/symbols/" + base,
        ed + "/../build/pc-tool/" + base,
        ed + "/../" + path,
        ed + "/../../" + path,
        ed + "/../../../" + path,
        "D:/w_spase/DDV2/MOTOR/app_stm32f4_motor/Debug/" + base,
        "D:/w_spase/DDV2/RK/app_stm32l4_rk/Debug/" + base,
        "D:/w_spase/DDV2/mainPCB/app_esp32_dd/.pio/build/esp32dev",
        "//wsl.localhost/Ubuntu_DDV2/home/fermiums/linux_ssd202d_dd/sdk/app_ssd202d_ddv2/app/firmware/" + base,
        "//wsl.localhost/Ubuntu_DDV2/home/fermiums/linux_ssd202d_dd/sdk/app_ssd202d_ddv2/app/firmware/motor/" + base,
        "//wsl.localhost/Ubuntu_DDV2/home/fermiums/linux_ssd202d_dd/sdk/app_ssd202d_ddv2/app/firmware/rk/" + base,
    };
    for (const std::string& c : candidates) {
        if (!c.empty() && fs::exists(c)) return c;
    }
    return path;
}

std::string deriveSourceName(const std::string& path)
{
    namespace fs = std::filesystem;
    std::string base = fs::path(path).filename().string();
    if (base.rfind("symbols_", 0) == 0) base.erase(0, std::string("symbols_").size());
    const std::string ext = ".json";
    if (base.size() > ext.size() && base.compare(base.size() - ext.size(), ext.size(), ext) == 0) {
        base.erase(base.size() - ext.size());
    }
    if (base.empty()) base = "source";
    return base;
}

template <size_t N>
void setTextBuffer(std::array<char, N>& target, const std::string& value)
{
    target.fill('\0');
    std::snprintf(target.data(), target.size(), "%s", value.c_str());
}

std::string recoveryCgiScript()
{
    return R"(#!/bin/sh
echo "Content-type: text/plain"
echo ""

cmd="${QUERY_STRING#cmd=}"

    case "$cmd" in
install)
    echo "[recovery] stopping app_dd"
    killall -TERM app_dd 2>/dev/null || true
    sleep 1
    killall -9 app_dd 2>/dev/null || true

    if [ ! -s /tmp/drivescope_app.tar.gz ]; then
        echo "[recovery] missing /tmp/drivescope_app.tar.gz"
        exit 1
    fi

    echo "[recovery] replacing /app"
    mkdir -p /app
    for p in /app/* /app/.[!.]*; do
        [ -e "$p" ] && rm -rf "$p"
    done

    tar -xzf /tmp/drivescope_app.tar.gz -C /
    chmod 755 /app/app_dd /app/otaunpack /app/xor_file 2>/dev/null || true
    chmod 755 /app/httpd/cgi-bin/*.sh 2>/dev/null || true
    rm -f /tmp/drivescope_app.tar.gz
    sync
    (sleep 1
     for p in $(ps | awk '/[h]ttpd -p 8080/{print $1}'); do
         kill "$p" 2>/dev/null || true
     done
     sleep 1
     busybox httpd -p 8080 -h /app/httpd
    ) >/dev/null 2>&1 &
    echo "[recovery] ok"
    ;;
linux_ota)
    echo "[recovery] starting Linux OTA"
    mkdir -p /app/firmware

    ota_file="/app/firmware/SStarOta.bin.gz"
    if [ ! -s "$ota_file" ]; then
        echo "[recovery] missing $ota_file"
        exit 1
    fi

    otaunpack="/tmp/drivescope_otaunpack"
    if [ ! -x "$otaunpack" ]; then
        otaunpack="/app/otaunpack"
    fi
    if [ ! -x "$otaunpack" ]; then
        echo "[recovery] missing otaunpack"
        exit 1
    fi

    "$otaunpack" -x "$ota_file"
    rc=$?
    sync
    echo "[recovery] Linux OTA finished rc=$rc"
    exit $rc
    ;;
reboot)
    echo "[recovery] rebooting"
    sync
    (sleep 1; reboot) >/dev/null 2>&1 &
    ;;
ping)
    echo "[recovery] ok"
    ;;
*)
    echo "[recovery] unknown command"
    exit 2
    ;;
esac
)";
}

bool writeTextFile(const std::filesystem::path& path, const std::string& text)
{
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f << text;
    return static_cast<bool>(f);
}

struct CommandResult {
    int rc = -1;
    std::string output;
};

void setDeployProgress(const std::shared_ptr<DeployProgressState>& progress,
                       int percent, const std::string& text)
{
    if (!progress) return;
    progress->percent.store(std::clamp(percent, 0, 100), std::memory_order_relaxed);
    progress->active.store(true, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(progress->mutex);
    progress->text = text;
}

bool deployProgressSnapshot(const std::shared_ptr<DeployProgressState>& progress,
                            int& percent, std::string& text)
{
    if (!progress || !progress->active.load(std::memory_order_relaxed)) return false;
    percent = progress->percent.load(std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(progress->mutex);
    text = progress->text;
    return true;
}

std::string formatBytes(uintmax_t bytes)
{
    char buf[64];
    const double b = static_cast<double>(bytes);
    if (bytes >= 1024ull * 1024ull) {
        std::snprintf(buf, sizeof(buf), "%.1f MB", b / (1024.0 * 1024.0));
    } else if (bytes >= 1024ull) {
        std::snprintf(buf, sizeof(buf), "%.1f KB", b / 1024.0);
    } else {
        std::snprintf(buf, sizeof(buf), "%llu B",
                      static_cast<unsigned long long>(bytes));
    }
    return buf;
}

int lastPercentInOutput(const std::string& text)
{
    int last = -1;
    for (size_t i = 0; i < text.size(); ++i) {
        if (text[i] != '%') continue;
        size_t start = i;
        while (start > 0 && std::isdigit(static_cast<unsigned char>(text[start - 1]))) {
            --start;
        }
        if (start == i) continue;
        const int pct = std::atoi(text.substr(start, i - start).c_str());
        if (pct >= 0 && pct <= 100) last = pct;
    }
    return last;
}

std::string tailText(const std::string& text, size_t maxLen = 1800)
{
    if (text.size() <= maxLen) return text;
    return std::string("...") + text.substr(text.size() - maxLen);
}

CommandResult runCommandHidden(const std::string& command,
                               const std::function<void(const std::string&)>& onOutput = {},
                               uint32_t timeoutMs = 0,
                               uint32_t stallTimeoutMs = 0)
{
#ifdef _WIN32
    SECURITY_ATTRIBUTES sa{};
    sa.nLength = sizeof(sa);
    sa.bInheritHandle = TRUE;

    HANDLE readPipe = nullptr;
    HANDLE writePipe = nullptr;
    if (!CreatePipe(&readPipe, &writePipe, &sa, 0)) {
        return {-1, "CreatePipe failed: " + std::to_string(GetLastError())};
    }
    SetHandleInformation(readPipe, HANDLE_FLAG_INHERIT, 0);

    STARTUPINFOA si{};
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
    si.hStdOutput = writePipe;
    si.hStdError = writePipe;

    PROCESS_INFORMATION pi{};
    std::vector<char> cmd(command.begin(), command.end());
    cmd.push_back('\0');

    const BOOL ok = CreateProcessA(nullptr, cmd.data(), nullptr, nullptr, TRUE,
                                   CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi);
    CloseHandle(writePipe);
    if (!ok) {
        const DWORD err = GetLastError();
        CloseHandle(readPipe);
        return {-1, "CreateProcess failed: " + std::to_string(err)};
    }

    std::string out;
    char buf[4096];
    const ULONGLONG startedAt = GetTickCount64();
    ULONGLONG lastOutputAt = startedAt;
    bool killed = false;
    std::string killReason;
    for (;;) {
        DWORD avail = 0;
        if (PeekNamedPipe(readPipe, nullptr, 0, nullptr, &avail, nullptr) && avail > 0) {
            DWORD got = 0;
            const DWORD want = std::min<DWORD>(avail, sizeof(buf));
            if (ReadFile(readPipe, buf, want, &got, nullptr) && got > 0) {
                std::string chunk(buf, buf + got);
                out += chunk;
                lastOutputAt = GetTickCount64();
                if (onOutput) onOutput(chunk);
            }
            continue;
        }

        const DWORD wait = WaitForSingleObject(pi.hProcess, 50);
        if (wait == WAIT_OBJECT_0) {
            while (PeekNamedPipe(readPipe, nullptr, 0, nullptr, &avail, nullptr) && avail > 0) {
                DWORD got = 0;
                const DWORD want = std::min<DWORD>(avail, sizeof(buf));
                if (!ReadFile(readPipe, buf, want, &got, nullptr) || got == 0) break;
                std::string chunk(buf, buf + got);
                out += chunk;
                if (onOutput) onOutput(chunk);
            }
            break;
        }

        const ULONGLONG now = GetTickCount64();
        if (timeoutMs > 0 && now - startedAt > timeoutMs) {
            killed = true;
            killReason = "command timeout after " + std::to_string(timeoutMs / 1000) + "s";
        } else if (stallTimeoutMs > 0 && now - lastOutputAt > stallTimeoutMs) {
            killed = true;
            killReason = "command stalled for " + std::to_string(stallTimeoutMs / 1000) + "s";
        }
        if (killed) {
            TerminateProcess(pi.hProcess, WAIT_TIMEOUT);
            WaitForSingleObject(pi.hProcess, 2000);
            out += "\n" + killReason + "\n";
            break;
        }
    }
    CloseHandle(readPipe);

    DWORD exitCode = 1;
    GetExitCodeProcess(pi.hProcess, &exitCode);
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    return {killed ? static_cast<int>(WAIT_TIMEOUT) : static_cast<int>(exitCode), out};
#else
    const int rc = std::system(command.c_str());
    return {rc, {}};
#endif
}

std::string runRecoveryCurlCommand(const std::string& command, const char* step)
{
    const CommandResult result = runCommandHidden(command);
    if (result.rc == 0) return {};
    std::string msg = std::string(step) + " failed rc=" + std::to_string(result.rc);
    if (!result.output.empty()) msg += ": " + tailText(result.output);
    return msg;
}

std::string firstExistingPath(const std::vector<std::string>& candidates)
{
    namespace fs = std::filesystem;
    for (const std::string& candidate : candidates) {
        if (candidate.empty()) continue;
        if (candidate.find('/') == std::string::npos &&
            candidate.find('\\') == std::string::npos) {
            return candidate;
        }
        std::error_code ec;
        if (fs::is_regular_file(fs::path(candidate), ec)) return candidate;
    }
    return {};
}

bool commandOutputContains(const std::string& command, const std::string& needle)
{
    CommandResult result = runCommandHidden(command, {}, 10000, 5000);
    if (result.rc != 0) return false;
    std::string out = result.output;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out.find(needle) != std::string::npos;
}

std::string recoveryCurlExe()
{
    const std::string ed = exeDir();
    std::vector<std::string> candidates = {
        ed + "/curl.exe",
        ed + "/ssh/curl.exe",
        "C:/msys64/mingw64/bin/curl.exe",
        "C:/msys64/usr/bin/curl.exe",
        "C:/Program Files/Git/mingw64/bin/curl.exe",
        "C:/Program Files/Git/usr/bin/curl.exe",
        "curl.exe",
    };
    for (const std::string& candidate : candidates) {
        std::string path = firstExistingPath({candidate});
        if (path.empty()) continue;
        if (commandOutputContains(quoteArg(path) + " --version", "sftp")) {
            return path;
        }
    }
    return "curl.exe";
}

std::string recoveryCurlBase()
{
    return quoteArg(recoveryCurlExe()) +
           " --fail --silent --show-error --connect-timeout 8 --insecure --user " +
           quoteArg("root:bork2025");
}

std::string recoveryHttpCurlBase()
{
    return quoteArg(recoveryCurlExe()) +
           " --fail --silent --show-error --connect-timeout 8";
}

std::string recoveryPlinkExe()
{
    const std::string ed = exeDir();
    return firstExistingPath({
        ed + "/plink.exe",
        ed + "/ssh/plink.exe",
        "C:/Program Files/PuTTY/plink.exe",
        "C:/Program Files (x86)/PuTTY/plink.exe",
        "plink.exe",
    });
}

std::string recoveryPscpExe()
{
    const std::string ed = exeDir();
    return firstExistingPath({
        ed + "/pscp.exe",
        ed + "/ssh/pscp.exe",
        "C:/Program Files/PuTTY/pscp.exe",
        "C:/Program Files (x86)/PuTTY/pscp.exe",
        "pscp.exe",
    });
}

// Bitvise sftpc.exe: empirically ~2Г— faster and dramatically more reliable
// than PuTTY's pscp on the recovery AP (Bitvise's SFTP implementation has
// better pipelining and flow control over flaky Wi-Fi). Recovery/AP payload
// uploads use this path exclusively.
std::string recoveryBitviseSftpc()
{
    const std::string ed = exeDir();
    return firstExistingPath({
        ed + "/sftpc.exe",
        ed + "/ssh/sftpc.exe",
        "C:/Program Files/Bitvise SSH Client/sftpc.exe",
        "C:/Program Files (x86)/Bitvise SSH Client/sftpc.exe",
    });
}

// Bitvise sexec.exe: preferred remote command executor. We pass the live
// fingerprint discovered by the shared host-key probe.
std::string recoveryBitviseSexec()
{
    const std::string ed = exeDir();
    return firstExistingPath({
        ed + "/sexec.exe",
        ed + "/ssh/sexec.exe",
        "C:/Program Files/Bitvise SSH Client/sexec.exe",
        "C:/Program Files (x86)/Bitvise SSH Client/sexec.exe",
    });
}

// Forward decl: recoveryLog() is defined further down with the rest of
// the recovery-log helpers but used by runRecoveryRemoteSh() below.
void recoveryLog(const std::string& line);

// The recovery device exposes its sshd at one of two IPs depending on how
// the user got there:
//   * 192.168.1.0  -- button-held recovery boot (init-script hostapd)
//   * 192.168.1.1  -- Service AP from a running app_dd (its own hostapd
//                    config sets the gateway to .1)
// We probe TCP :22 with a 2-second timeout, prefer .1 (Service-AP path
// is the common case from DriveScope's "Service AP" button), fall back
// to .0 (button-held recovery), then cache the result for 30 s so we
// don't waste 4 s probing in front of every plink/sexec call inside one
// install/OTA operation.
// Holds the cached recovery-host IP between calls. Static-locals can't be
// reset from outside, so we lift them to namespace scope and expose an
// invalidator that callers fire whenever the device IP is expected to flip
// (Service AP press, Reset press -- any state change in the
// device's hostapd).
std::string g_recoveryHostCached;
ULONGLONG   g_recoveryHostCachedAt = 0;

// Forward declaration so invalidateRecoveryHostCache can also clear the
// fingerprint cache (defined further down with the rest of the host-key
// helpers). Keeping them coupled means a Service AP press invalidates
// both the host IP probe and any stale fingerprint from the previous IP.
void invalidateHostKeyFpCache(const char* reason);

constexpr const char* kRecoveryKnownHostKey =
    "SHA256:l63iZ9rjClz4nzwCIogn8oW3CBpQEHnCMzMbp+hDdzU";

void invalidateRecoveryHostCache(const char* reason = nullptr)
{
    g_recoveryHostCached.clear();
    g_recoveryHostCachedAt = 0;
    invalidateHostKeyFpCache(reason);
    if (reason) recoveryLog(std::string("[host-probe] cache invalidated (") +
                            reason + ")");
}

std::string resolveRecoveryHost()
{
    if (!g_recoveryHostCached.empty() &&
        (GetTickCount64() - g_recoveryHostCachedAt) < 30000) {
        return g_recoveryHostCached;
    }

    // Winsock may not be initialised yet (the SLCAN/TCP transports init it
    // lazily on connect). WSAStartup is reference-counted and safe to call
    // multiple times -- without this, socket() below returns INVALID_SOCKET,
    // both probes silently fail, and we always fall through to the default
    // 192.168.1.0. That's exactly the symptom: "command stalled for 12s"
    // when the device is actually on 192.168.1.1 (Service AP via app_dd).
    static bool wsaInit = false;
    if (!wsaInit) {
        WSADATA wsa{};
        WSAStartup(MAKEWORD(2, 2), &wsa);
        wsaInit = true;
    }

    auto tryProbe = [](const char* ip) -> bool {
        SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (s == INVALID_SOCKET) {
            recoveryLog(std::string("[host-probe] socket() failed for ") + ip +
                        " err=" + std::to_string(WSAGetLastError()));
            return false;
        }
        u_long nb = 1;
        ioctlsocket(s, FIONBIO, &nb);
        sockaddr_in sa{};
        sa.sin_family = AF_INET;
        sa.sin_port   = htons(22);
        inet_pton(AF_INET, ip, &sa.sin_addr);
        connect(s, reinterpret_cast<const sockaddr*>(&sa), sizeof(sa));
        fd_set wf; FD_ZERO(&wf); FD_SET(s, &wf);
        timeval tv{2, 0};
        const int rs = select(0, nullptr, &wf, nullptr, &tv);
        bool ok = false;
        if (rs > 0) {
            int err = 0; int errlen = sizeof(err);
            if (getsockopt(s, SOL_SOCKET, SO_ERROR,
                           reinterpret_cast<char*>(&err), &errlen) == 0) {
                ok = (err == 0);
            }
        }
        closesocket(s);
        return ok;
    };

    const char* ip = nullptr;
    if (tryProbe("192.168.1.1")) ip = "192.168.1.1";
    else if (tryProbe("192.168.1.0")) ip = "192.168.1.0";

    if (ip) {
        recoveryLog(std::string("[host-probe] selected ") + ip);
        g_recoveryHostCached   = ip;
        g_recoveryHostCachedAt = GetTickCount64();
    } else {
        // Both probes failed -- return the historical default so the
        // subsequent SSH/SFTP call surfaces a normal "connection refused"
        // error instead of an empty string here. Don't cache the failure.
        recoveryLog("[host-probe] neither 192.168.1.0 nor 192.168.1.1 "
                    "answers TCP :22; defaulting to 192.168.1.0");
        return "192.168.1.0";
    }
    return g_recoveryHostCached;
}

std::string gracefulStopAppDdSh(bool includeHttpd)
{
    std::string cmd =
        "killall -TERM app_dd otaunpack xor_file 2>/dev/null || true; "
        "for i in 1 2 3 4 5 6 7 8 9 10; do "
        "  pidof app_dd >/dev/null 2>&1 || break; "
        "  sleep 0.2; "
        "done; "
        "pidof app_dd >/dev/null 2>&1 && killall -9 app_dd 2>/dev/null || true; "
        "killall -9 otaunpack xor_file 2>/dev/null || true; ";
    if (includeHttpd) {
        cmd +=
            "killall -TERM httpd 2>/dev/null || true; "
            "sleep 0.3; "
            "killall -9 httpd 2>/dev/null || true; ";
    }
    return cmd;
}

// Windows OpenSSH ssh-keyscan/ssh-keygen paths. These have shipped with
// Windows 10 1809+ and Windows 11 by default; on older boxes they're
// missing and we fall back to Bitvise's accept-new behaviour.
std::string winOpenSshDir()
{
#ifdef _WIN32
    char buf[MAX_PATH];
    UINT n = GetSystemDirectoryA(buf, MAX_PATH);
    if (n == 0 || n >= MAX_PATH) return "C:/Windows/System32/OpenSSH";
    std::string sys(buf, buf + n);
    for (auto& c : sys) if (c == '\\') c = '/';
    return sys + "/OpenSSH";
#else
    return {};
#endif
}

// Probe the device's actual SSH host fingerprint via ssh-keyscan + ssh-keygen.
// Cached per host for the session. Returns "SHA256:xxxx" or empty string when
// probing isn't possible (no Windows OpenSSH installed, host unreachable, etc).
//
// Why: Bitvise's `-hostKeyFp=any` and PuTTY's pinned-key fallback both fail
// when the device gets re-flashed and the host key changes -- sexec then
// surfaces "ERROR: The received host key has been rejected. Reason: User
// input is disabled.", and the `Upload /app` flow can't even kill app_dd to
// start the upload. Probing the live key sidesteps both problems: the very
// fingerprint the device is presenting is the one we hand to sexec/plink.
std::string g_hostKeyFpCache_host;
std::string g_hostKeyFpCache_fp;
ULONGLONG   g_hostKeyFpCache_at = 0;

void invalidateHostKeyFpCache(const char* reason = nullptr)
{
    g_hostKeyFpCache_host.clear();
    g_hostKeyFpCache_fp.clear();
    g_hostKeyFpCache_at = 0;
    if (reason) recoveryLog(std::string("[host-fp] cache invalidated (") +
                            reason + ")");
}

// Attempt 1: ssh-keyscan + ssh-keygen via Windows OpenSSH. Returns SHA256:
// fingerprint or empty string on failure.
std::string probeHostFpViaOpenSsh(const std::string& host)
{
#ifdef _WIN32
    namespace fs = std::filesystem;
    const std::string dir = winOpenSshDir();
    const std::string keyscan = dir + "/ssh-keyscan.exe";
    const std::string keygen  = dir + "/ssh-keygen.exe";
    std::error_code ec;
    if (!fs::is_regular_file(fs::path(keyscan), ec) ||
        !fs::is_regular_file(fs::path(keygen),  ec)) {
        return {};
    }

    char tmpDir[MAX_PATH];
    DWORD tn = GetTempPathA(MAX_PATH, tmpDir);
    const std::string tempBase = (tn == 0 || tn >= MAX_PATH)
        ? std::string{} : std::string(tmpDir);
    const std::string knownPath = tempBase + "drivescope_known_" +
                                  std::to_string(GetCurrentProcessId()) + ".txt";

    const std::string scanCmd =
        "cmd.exe /c " + quoteArg(quoteArg(keyscan) +
        " -T 5 -t ssh-ed25519,rsa,ecdsa-sha2-nistp256 " + quoteArg(host) +
        " > " + quoteArg(knownPath) + " 2>nul");
    runCommandHidden(scanCmd, {}, 12000, 8000);

    if (!fs::is_regular_file(fs::path(knownPath), ec) ||
        fs::file_size(fs::path(knownPath), ec) == 0) {
        recoveryLog("[host-fp] ssh-keyscan empty for " + host);
        DeleteFileA(knownPath.c_str());
        return {};
    }

    const std::string keygenCmd = quoteArg(keygen) + " -lf " + quoteArg(knownPath);
    const CommandResult kr = runCommandHidden(keygenCmd, {}, 5000, 4000);
    DeleteFileA(knownPath.c_str());
    if (kr.rc != 0 || kr.output.empty()) return {};

    const size_t pos = kr.output.find("SHA256:");
    if (pos == std::string::npos) return {};
    const size_t end = kr.output.find_first_of(" \t\r\n", pos);
    std::string fp = kr.output.substr(pos,
        end == std::string::npos ? std::string::npos : end - pos);
    return fp.size() >= 8 ? fp : std::string{};
#else
    (void)host;
    return {};
#endif
}

// Attempt 2: PuTTY plink without -batch, stdin closed. plink prints the
// host fingerprint to stdout/stderr on first connect ("The server's
// ssh-ed25519 key fingerprint is: SHA256:xxx"), then aborts because it
// can't read a y/n answer from a closed stdin. We harvest the fingerprint
// from the captured output.
//
// Why we need this fallback: Windows OpenSSH (path 1) ships only on
// Win10 1809+, and many DriveScope users run older Windows or have it
// disabled. PuTTY ships in dist/ssh/ alongside DriveScope.exe, so plink
// is essentially always present.
std::string probeHostFpViaPlink(const std::string& host)
{
#ifdef _WIN32
    const std::string plink = recoveryPlinkExe();
    if (plink.empty()) return {};

    // -ssh: force SSH-2.
    // -P 22: explicit port (some Win shells corrupt default).
    // -no-antispoof, -ssh-connection: not used here; defaults are fine.
    // We DO NOT pass -batch -- that suppresses the very fingerprint print
    // we want. We DO NOT pass -hostkey -- that bypasses the verification
    // path entirely and plink connects without printing the fingerprint.
    //
    // `cmd.exe /c "<plink> < NUL"` redirects stdin from NUL so plink
    // reads EOF when it asks "Store key in cache? (y/n)" and aborts.
    // The fingerprint is already in stderr by then. We merge stderr into
    // stdout via 2>&1 so runCommandHidden captures both.
    const std::string cmd = "cmd.exe /c " + quoteArg(
        quoteArg(plink) +
        " -ssh -P 22 -pw bork2025 root@" + host +
        " exit < NUL 2>&1");

    const CommandResult r = runCommandHidden(cmd, {}, 12000, 10000);

    const size_t pos = r.output.find("SHA256:");
    if (pos == std::string::npos) {
        recoveryLog("[host-fp] plink probe found no SHA256 in output (" +
                    std::to_string(r.output.size()) + " bytes captured, rc=" +
                    std::to_string(r.rc) + ")");
        return {};
    }
    const size_t end = r.output.find_first_of(" \t\r\n\"", pos);
    std::string fp = r.output.substr(pos,
        end == std::string::npos ? std::string::npos : end - pos);
    return fp.size() >= 8 ? fp : std::string{};
#else
    (void)host;
    return {};
#endif
}

std::string sha256FpForBitvise(std::string fp)
{
    constexpr const char* prefix = "SHA256:";
    if (fp.rfind(prefix, 0) == 0) {
        fp.erase(0, std::strlen(prefix));
    }
    return fp;
}

// Attempt 3: ask Bitvise itself to connect without a host-key override. In
// unattended mode it rejects the unknown key, but it still prints the exact
// "SHA-256 fingerprint: ..." line first. This is the most reliable probe for
// devices whose host key is already cached in PuTTY or whose OpenSSH
// ssh-keyscan handshake does not return a key.
std::string probeHostFpViaBitviseSftpc(const std::string& host)
{
#ifdef _WIN32
    const std::string sftpc = recoveryBitviseSftpc();
    if (sftpc.empty()) return {};

    const std::string cmd =
        quoteArg(sftpc) +
        " " + quoteArg(std::string("root@") + host) +
        " -pw=bork2025 -unat=y -progress=none -cmd=exit";
    const CommandResult r = runCommandHidden(cmd, {}, 15000, 12000);

    const std::string marker = "SHA-256 fingerprint:";
    const size_t pos = r.output.find(marker);
    if (pos == std::string::npos) {
        recoveryLog("[host-fp] Bitvise probe found no SHA-256 fingerprint (" +
                    std::to_string(r.output.size()) + " bytes captured, rc=" +
                    std::to_string(r.rc) + ")");
        return {};
    }
    size_t start = pos + marker.size();
    while (start < r.output.size() &&
           std::isspace(static_cast<unsigned char>(r.output[start]))) {
        ++start;
    }
    size_t end = start;
    while (end < r.output.size() &&
           !std::isspace(static_cast<unsigned char>(r.output[end])) &&
           r.output[end] != '.') {
        ++end;
    }
    const std::string raw = r.output.substr(start, end - start);
    if (raw.size() < 8) return {};
    return "SHA256:" + raw;
#else
    (void)host;
    return {};
#endif
}

std::string discoveredHostKeyFp(const std::string& host)
{
#ifdef _WIN32
    if (host.empty()) return {};
    if (host == g_hostKeyFpCache_host && !g_hostKeyFpCache_fp.empty() &&
        (GetTickCount64() - g_hostKeyFpCache_at) < 60000) {
        return g_hostKeyFpCache_fp;
    }

    std::string fp = probeHostFpViaBitviseSftpc(host);
    const char* via = "bitvise-sftpc";
    if (fp.empty()) {
        fp = probeHostFpViaOpenSsh(host);
        via = "ssh-keyscan";
    }
    if (fp.empty()) {
        fp = probeHostFpViaPlink(host);
        via = "plink";
    }
    if (fp.empty()) {
        recoveryLog("[host-fp] no probe succeeded for " + host +
                    " -- Bitvise/plink calls will fail unless the host"
                    " key is already pinned in the registry");
        return {};
    }

    g_hostKeyFpCache_host = host;
    g_hostKeyFpCache_fp   = fp;
    g_hostKeyFpCache_at   = GetTickCount64();
    recoveryLog(std::string("[host-fp] ") + host + " -> " + fp +
                " (via " + via + ")");
    return fp;
#else
    (void)host;
    return {};
#endif
}

// Run a remote shell command on the recovery device. Strategy:
//   1. Probe the device's live host fingerprint, preferring Bitvise's own
//      key-exchange output. This matches the SFTP upload path and handles
//      freshly generated keys on new SOM/Linux images.
//   2. With a fingerprint in hand, prefer Bitvise sexec. This keeps Connect,
//      /app deploy, OTA, and Wi-Fi firmware flash on the same host-key logic.
//   3. Fall back to plink only if Bitvise is missing.
//
// The output and exit code are returned via CommandResult.
CommandResult runRecoveryRemoteSh(const std::string& remoteCmd,
                                  uint32_t timeoutMs,
                                  uint32_t stallMs,
                                  const char* tag,
                                  const std::function<void(const std::string&)>& onOutput = {})
{
    const std::string kHostStr = resolveRecoveryHost();
    const char* kHost = kHostStr.c_str();
    constexpr const char* kPassword = "bork2025";

    const std::string discoveredFp = discoveredHostKeyFp(kHostStr);

    std::string cmd;
    bool usingSexec = false;
    const std::string sexec = recoveryBitviseSexec();
    const std::string plink = recoveryPlinkExe();

    if (!discoveredFp.empty() && !sexec.empty()) {
        usingSexec = true;
        cmd = quoteArg(sexec) +
              " " + quoteArg(std::string("root@") + kHost) +
              " " + quoteArg(std::string("-pw=") + kPassword) +
              " -unat=y -exitZero -hostKeyFp=" + sha256FpForBitvise(discoveredFp) +
              " " + quoteArg(std::string("-cmd=") + remoteCmd);
    } else if (!discoveredFp.empty() && !plink.empty()) {
        cmd = quoteArg(plink) +
              " -batch -ssh -hostkey " + quoteArg(discoveredFp) +
              " -pw " + quoteArg(kPassword) +
              " root@" + kHost + " " + quoteArg(remoteCmd);
    } else if (!plink.empty()) {
        // Last-resort pinned key. If the device's recovery image was
        // re-flashed with a different key, plink will refuse -- install
        // Windows OpenSSH (Win10 1809+) or just leave PuTTY in dist/ssh/
        // so the no-batch fingerprint probe above can run.
        cmd = quoteArg(plink) +
              " -batch -ssh -hostkey " + quoteArg(kRecoveryKnownHostKey) +
              " -pw " + quoteArg(kPassword) +
              " root@" + kHost + " " + quoteArg(remoteCmd);
    } else {
        CommandResult r;
        r.rc = -1;
        r.output = "no SSH client found and host fingerprint not probable. "
                   "Install PuTTY (plink.exe) or Windows OpenSSH "
                   "(C:\\Windows\\System32\\OpenSSH).";
        return r;
    }

    const ULONGLONG start = GetTickCount64();
    if (tag) recoveryLog(std::string("[") + (usingSexec ? "sexec " : "plink ") +
                         tag + "] starting");
    CommandResult r = runCommandHidden(cmd, onOutput, timeoutMs, stallMs);
    if (tag) {
        char buf[200];
        std::snprintf(buf, sizeof(buf), "[%s %s] done rc=%d time=%.1fs",
                      usingSexec ? "sexec" : "plink",
                      tag, r.rc, (GetTickCount64() - start) / 1000.0);
        recoveryLog(buf);
        if (!r.output.empty()) {
            recoveryLog(std::string("[") + (usingSexec ? "sexec " : "plink ") +
                        tag + "] tail: " + tailText(r.output, 300));
        }
    }
    return r;
}

std::string recoverySshCommand(const std::string& remoteCommand,
                               const char* step)
{
    constexpr const char* kHost = "192.168.1.0";
    constexpr const char* kPassword = "bork2025";
    constexpr const char* kKnownHostKey = "SHA256:l63iZ9rjClz4nzwCIogn8oW3CBpQEHnCMzMbp+hDdzU";

    const std::string plink = recoveryPlinkExe();
    if (plink.empty()) return std::string(step) + " failed: plink.exe not found";

    const std::string cmd = quoteArg(plink) +
        " -batch -ssh -hostkey " + quoteArg(kKnownHostKey) +
        " -pw " + quoteArg(kPassword) +
        " root@" + kHost + " " + quoteArg(remoteCommand);
    const CommandResult result = runCommandHidden(cmd, {}, 30000, 15000);
    if (result.rc == 0) return {};
    std::string msg = std::string(step) + " failed rc=" + std::to_string(result.rc);
    if (!result.output.empty()) msg += ": " + tailText(result.output);
    return msg;
}

std::string recoveryUploadFile(const std::filesystem::path& localPath,
                               const std::string& remotePath,
                               const char* step,
                               const std::shared_ptr<DeployProgressState>& progress,
                               int basePercent,
                               int spanPercent,
                               const std::string& progressLabel)
{
    constexpr const char* kHost = "192.168.1.0";
    constexpr const char* kPassword = "bork2025";
    constexpr const char* kKnownHostKey = "SHA256:l63iZ9rjClz4nzwCIogn8oW3CBpQEHnCMzMbp+hDdzU";

    const std::string pscp = recoveryPscpExe();
    if (pscp.empty()) {
        std::string cmd = recoveryCurlBase() +
              " --ftp-create-dirs -T " + quoteArg(localPath.string()) +
              " " + quoteArg(std::string("sftp://") + kHost + "/" + remotePath);
        return runRecoveryCurlCommand(cmd, step);
    }

    const std::string remote = std::string("root@") + kHost + ":" + remotePath;
    const std::string cmd = quoteArg(pscp) +
        " -batch -scp -hostkey " + quoteArg(kKnownHostKey) +
        " -pw " + quoteArg(kPassword) +
        " " + quoteArg(localPath.string()) + " " + quoteArg(remote);

    std::string progressBuffer;
    const uintmax_t localSize = std::filesystem::file_size(localPath);
    const uint32_t timeoutMs = static_cast<uint32_t>(
        std::clamp<uintmax_t>(localSize / 1024u * 250u, 120000u, 900000u));
    const CommandResult result = runCommandHidden(cmd, [&](const std::string& chunk) {
        progressBuffer += chunk;
        if (progressBuffer.size() > 4096) {
            progressBuffer.erase(0, progressBuffer.size() - 4096);
        }
        const int pct = lastPercentInOutput(progressBuffer);
        if (pct >= 0) {
            setDeployProgress(progress,
                              basePercent + (spanPercent * pct) / 100,
                              progressLabel + " " + std::to_string(pct) + "%");
        }
    }, timeoutMs, 45000);
    if (result.rc == 0) return {};

    (void)recoverySshCommand(
        "for p in $(ps | awk '/[s]cp -t/{print $1}'); do kill \"$p\" 2>/dev/null || true; done; "
        "rm -f " + shellSingleQuote(remotePath),
        "cleanup stalled upload");

    std::string msg = std::string(step) + " failed rc=" + std::to_string(result.rc);
    if (!result.output.empty()) msg += ": " + tailText(result.output);
    return msg;
}

// Recovery debug log: %TEMP%\drivescope_recovery.log. Every recovery
// operation appends a timestamped line so the user can attach the file
// when reporting an issue. Opened append-mode each call -- slow but
// crash-safe (if DriveScope dies mid-upload, the log is on disk).
std::string recoveryLogPath()
{
#ifdef _WIN32
    char buf[MAX_PATH];
    DWORD n = GetTempPathA(MAX_PATH, buf);
    if (n == 0 || n >= MAX_PATH) return "drivescope_recovery.log";
    return std::string(buf) + "drivescope_recovery.log";
#else
    return "/tmp/drivescope_recovery.log";
#endif
}

void recoveryLog(const std::string& line)
{
    const std::string path = recoveryLogPath();
    std::ofstream f(path, std::ios::app);
    if (!f) return;
    using clock = std::chrono::system_clock;
    const auto now = clock::now();
    const auto t = clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch()).count() % 1000;
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char ts[32];
    std::snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d.%03d",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec, static_cast<int>(ms));
    f << ts << "  " << line << "\n";
}

// Spawn plink, redirect its stdin from `localFile`, capture stdout/stderr.
// We feed the file to plink ourselves chunk-by-chunk so progress reflects
// real bytes-on-the-wire instead of pscp's block-buffered "X%" text (which
// hangs at 4% on Win32 anonymous pipes). The remote command on the other
// end consumes our stream (e.g. `tar -xzf - -C /` or `cat > /path`).
struct PlinkPipeResult {
    int rc = -1;
    std::string output;     // combined stdout/stderr from plink/remote
    std::string failStage;  // empty on success
};

PlinkPipeResult runPlinkPipeFile(const std::string& remoteCommand,
                                 const std::filesystem::path& localFile,
                                 const std::shared_ptr<DeployProgressState>& progress,
                                 int basePercent, int spanPercent,
                                 const std::string& progressLabel,
                                 uint32_t remoteTailTimeoutMs = 180000)
{
    PlinkPipeResult result;
#ifdef _WIN32
    constexpr const char* kHost = "192.168.1.0";
    constexpr const char* kPassword = "bork2025";

    const std::string plink = recoveryPlinkExe();
    if (plink.empty()) {
        result.output = "plink.exe not found";
        result.failStage = "spawn";
        return result;
    }

    HANDLE hFile = CreateFileA(localFile.string().c_str(), GENERIC_READ, FILE_SHARE_READ,
                               nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (hFile == INVALID_HANDLE_VALUE) {
        result.output = "cannot open local file: " + localFile.string();
        result.failStage = "open";
        return result;
    }
    LARGE_INTEGER fileSize{};
    if (!GetFileSizeEx(hFile, &fileSize)) fileSize.QuadPart = 0;

    SECURITY_ATTRIBUTES sa{};
    sa.nLength = sizeof(sa);
    sa.bInheritHandle = TRUE;

    HANDLE stdinR = nullptr, stdinW = nullptr;
    if (!CreatePipe(&stdinR, &stdinW, &sa, 256 * 1024)) {
        CloseHandle(hFile);
        result.output = "CreatePipe(stdin) failed: " + std::to_string(GetLastError());
        result.failStage = "spawn";
        return result;
    }
    SetHandleInformation(stdinW, HANDLE_FLAG_INHERIT, 0);

    HANDLE stdoutR = nullptr, stdoutW = nullptr;
    if (!CreatePipe(&stdoutR, &stdoutW, &sa, 0)) {
        CloseHandle(hFile);
        CloseHandle(stdinR); CloseHandle(stdinW);
        result.output = "CreatePipe(stdout) failed: " + std::to_string(GetLastError());
        result.failStage = "spawn";
        return result;
    }
    SetHandleInformation(stdoutR, HANDLE_FLAG_INHERIT, 0);

    STARTUPINFOA si{};
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdInput = stdinR;
    si.hStdOutput = stdoutW;
    si.hStdError = stdoutW;

    // Pin the recovery-image host key. Plink in -batch refuses unknown
    // keys, and we have no console to confirm at (CREATE_NO_WINDOW). All
    // devices using the same recovery firmware share this fingerprint;
    // if it ever rotates the user gets a clean "host key mismatch" error
    // and we update this constant rather than silently downgrading trust.
    constexpr const char* kKnownHostKey =
        "SHA256:l63iZ9rjClz4nzwCIogn8oW3CBpQEHnCMzMbp+hDdzU";
    std::string cmdStr = quoteArg(plink) +
        " -batch -ssh -hostkey " + quoteArg(kKnownHostKey) +
        " -pw " + quoteArg(kPassword) +
        " root@" + kHost + " " + quoteArg(remoteCommand);
    std::vector<char> cmdBuf(cmdStr.begin(), cmdStr.end());
    cmdBuf.push_back('\0');

    PROCESS_INFORMATION pi{};
    const BOOL spawnedOk = CreateProcessA(nullptr, cmdBuf.data(), nullptr, nullptr, TRUE,
                                          CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi);
    CloseHandle(stdinR);
    CloseHandle(stdoutW);
    if (!spawnedOk) {
        const DWORD err = GetLastError();
        CloseHandle(hFile);
        CloseHandle(stdinW);
        CloseHandle(stdoutR);
        result.output = "CreateProcess(plink) failed err=" + std::to_string(err);
        result.failStage = "spawn";
        return result;
    }

    std::mutex outMutex;
    std::string out;
    std::atomic<bool> readerStop{false};

    std::thread reader([&]() {
        char buf[4096];
        for (;;) {
            DWORD avail = 0;
            const BOOL ok = PeekNamedPipe(stdoutR, nullptr, 0, nullptr, &avail, nullptr);
            if (!ok) {
                // pipe closed (process exit + write end gone)
                break;
            }
            if (avail > 0) {
                DWORD got = 0;
                const DWORD want = std::min<DWORD>(avail, sizeof(buf));
                if (!ReadFile(stdoutR, buf, want, &got, nullptr) || got == 0) break;
                std::lock_guard<std::mutex> lk(outMutex);
                out.append(buf, got);
                continue;
            }
            if (readerStop.load(std::memory_order_relaxed)) {
                Sleep(20);
                // give a couple more cycles for any final tail
                DWORD a2 = 0;
                if (!PeekNamedPipe(stdoutR, nullptr, 0, nullptr, &a2, nullptr) || a2 == 0) {
                    break;
                }
                continue;
            }
            Sleep(50);
        }
    });

    // Stream local file в†’ plink stdin pipe, byte-accurate progress.
    constexpr DWORD kChunk = 64 * 1024;
    std::vector<char> fileBuf(kChunk);
    LONGLONG totalSent = 0;
    bool streamErr = false;
    std::string streamErrMsg;

    for (;;) {
        DWORD got = 0;
        if (!ReadFile(hFile, fileBuf.data(), kChunk, &got, nullptr)) {
            streamErr = true;
            streamErrMsg = "local read failed: " + std::to_string(GetLastError());
            break;
        }
        if (got == 0) break;  // EOF

        DWORD written = 0;
        const char* p = fileBuf.data();
        DWORD remaining = got;
        while (remaining > 0) {
            DWORD chunk = 0;
            if (!WriteFile(stdinW, p, remaining, &chunk, nullptr) || chunk == 0) {
                const DWORD werr = GetLastError();
                streamErr = true;
                streamErrMsg = "stdin write failed err=" + std::to_string(werr);
                break;
            }
            p += chunk;
            remaining -= chunk;
            written += chunk;
        }
        if (streamErr) break;
        totalSent += written;

        if (fileSize.QuadPart > 0) {
            const int pct = static_cast<int>(
                std::clamp<long long>((totalSent * 100) / fileSize.QuadPart, 0, 100));
            const int outPct = basePercent + (spanPercent * pct) / 100;
            // Status bar truncates at 42 chars and the UPDATE prefix already
            // shows the percent, so the suffix only carries label + bytes.
            // Format: "<label> 1.8/20.6 MB" => fits comfortably.
            const double sentMb  = static_cast<double>(totalSent) / (1024.0 * 1024.0);
            const double totalMb = static_cast<double>(fileSize.QuadPart) / (1024.0 * 1024.0);
            char buf[128];
            std::snprintf(buf, sizeof(buf), "%s %.1f/%.1f MB",
                          progressLabel.c_str(), sentMb, totalMb);
            setDeployProgress(progress, outPct, buf);
        }
    }
    CloseHandle(hFile);
    // Closing stdinW signals EOF to the remote process so its consumer
    // (tar / cat) finishes and the SSH session terminates.
    CloseHandle(stdinW);

    setDeployProgress(progress, basePercent + spanPercent,
                      progressLabel + ": installing on device");

    DWORD waitRc = WaitForSingleObject(pi.hProcess, remoteTailTimeoutMs);
    bool killedTimeout = false;
    if (waitRc == WAIT_TIMEOUT) {
        killedTimeout = true;
        TerminateProcess(pi.hProcess, 1);
        WaitForSingleObject(pi.hProcess, 2000);
    }

    DWORD exitCode = 1;
    GetExitCodeProcess(pi.hProcess, &exitCode);

    readerStop.store(true);
    reader.join();
    CloseHandle(stdoutR);
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);

    {
        std::lock_guard<std::mutex> lk(outMutex);
        result.output = out;
    }

    if (streamErr) {
        result.rc = -1;
        result.failStage = "stream";
        if (!streamErrMsg.empty()) {
            result.output += (result.output.empty() ? "" : "\n");
            result.output += streamErrMsg;
        }
        return result;
    }
    if (killedTimeout) {
        result.rc = -2;
        result.failStage = "wait";
        result.output += (result.output.empty() ? "" : "\n");
        result.output += "remote command timeout after " +
                         std::to_string(remoteTailTimeoutMs / 1000) + "s";
        return result;
    }
    result.rc = static_cast<int>(exitCode);
    if (result.rc != 0) result.failStage = "remote";
    return result;
#else
    (void)remoteCommand; (void)localFile; (void)progress;
    (void)basePercent; (void)spanPercent; (void)progressLabel; (void)remoteTailTimeoutMs;
    result.output = "plink streaming is Windows-only";
    result.failStage = "platform";
    return result;
#endif
}

// Forward declaration so uploadFileToRecovery can reference uploadFileViaPscp
// (which is defined further down).
std::string uploadFileViaPscp(const std::filesystem::path& src,
                              const std::string& remotePath,
                              const std::shared_ptr<DeployProgressState>& progress,
                              int basePct, int spanPct,
                              const std::string& label,
                              uint32_t timeoutMs);

// Upload a single local file using Bitvise sftpc.exe with -progress=percent.
// sftpc emits "0\n", "1\n", ... "100\n" on stdout one per percentage point;
// we read line-by-line and forward to setDeployProgress, giving real-time
// byte-accurate progress. Empirically ~2Г— faster than pscp on flaky Wi-Fi.
std::string uploadFileViaBitvise(const std::filesystem::path& src,
                                 const std::string& remotePath,
                                 const std::shared_ptr<DeployProgressState>& progress,
                                 int basePct, int spanPct,
                                 const std::string& label,
                                 uint32_t timeoutMs)
{
    const std::string sftpc = recoveryBitviseSftpc();
    if (sftpc.empty()) return "Bitvise sftpc.exe not found";

    const std::string kHostStr = resolveRecoveryHost();
    const char* kHost = kHostStr.c_str();
    constexpr const char* kPassword = "bork2025";

    // Bitvise sftpc requires a SPECIFIC host fingerprint -- there is no
    // documented "accept any" mode in Bitvise's CLI (the legacy
    // `-hostKeyFp=any` was a no-op that silently fell back to the
    // registry, and on a clean machine surfaces "User input is disabled"
    // or dumps sftpc's help with rc=2). discoveredHostKeyFp() tries
    // OpenSSH, PuTTY, and finally Bitvise's own "reject but print the
    // presented fingerprint" path.
    const std::string discoveredFp = discoveredHostKeyFp(kHostStr);
    if (discoveredFp.empty()) {
        return "Bitvise sftpc could not probe the device host fingerprint";
    }
    const std::string fpFlag =
        std::string(" -hostKeyFp=") + sha256FpForBitvise(discoveredFp);

    // `put -o` = overwrite if exists. Without -o, sftpc errors out with
    // "Open request has failed with SFTP error Failure" when /tmp/<file>
    // is left over from a previous attempt -- and the user just sees the
    // upload "stuck at starting".
    std::string remoteForward = src.string();
    for (auto& c : remoteForward) if (c == '\\') c = '/';
    // Per `help put`: USAGE: put local-path [remote-path] [-o]. Options
    // come after the paths.
    const std::string putCmd = std::string("put \"") + remoteForward +
                               "\" \"" + remotePath + "\" -o";
    const std::string cmdStr = quoteArg(sftpc) +
        " " + quoteArg(std::string("root@") + kHost) +
        " " + quoteArg(std::string("-pw=") + kPassword) +
        " -unat=y -progress=percent" + fpFlag +
        " " + quoteArg(std::string("-cmd=") + putCmd);

    const uintmax_t fileSize = std::filesystem::file_size(src);
    recoveryLog("[bitvise] start " + label + " src=" + src.string() +
                " dst=" + remotePath +
                " size=" + formatBytes(fileSize));
    const ULONGLONG startedAt = GetTickCount64();

    // Reuse runCommandHidden -- it already gives us real-time stdout
    // chunks (PeekNamedPipe + ReadFile loop).
    // We translate sftpc's "<percent>\n" lines into deploy-progress updates.
    std::string lineBuf;
    int lastPct = -1;
    auto onChunk = [&](const std::string& chunk) {
        lineBuf += chunk;
        for (;;) {
            const size_t nl = lineBuf.find_first_of("\r\n");
            if (nl == std::string::npos) break;
            std::string line = lineBuf.substr(0, nl);
            lineBuf.erase(0, nl + 1);
            // sftpc with -progress=percent prints just "NN" (sometimes
            // with leading space) on its own line. Anything else is its
            // banner / error and we ignore it for progress purposes.
            std::string trimmed;
            for (char c : line) if (c != ' ' && c != '\t') trimmed += c;
            if (trimmed.empty()) continue;
            bool allDigits = true;
            for (char c : trimmed) if (!std::isdigit(static_cast<unsigned char>(c))) { allDigits = false; break; }
            if (!allDigits) continue;
            const int pct = std::clamp(std::atoi(trimmed.c_str()), 0, 100);
            if (pct == lastPct) continue;
            lastPct = pct;
            const int outPct = basePct + (spanPct * pct) / 100;
            const double sentMb  = (static_cast<double>(fileSize) * pct / 100.0) / (1024.0 * 1024.0);
            const double totalMb = static_cast<double>(fileSize) / (1024.0 * 1024.0);
            char buf[128];
            std::snprintf(buf, sizeof(buf), "%s %.1f/%.1f MB",
                          label.c_str(), sentMb, totalMb);
            setDeployProgress(progress, outPct, buf);
        }
    };

    // Do not kill Bitvise on short no-output stalls: on the recovery AP the
    // radio can pause for tens of seconds and then resume cleanly. Keep only
    // a generous total timeout so the UI cannot hang forever.
    const uint32_t patientTimeoutMs = std::max<uint32_t>(timeoutMs, 600000);
    const CommandResult r = runCommandHidden(cmdStr, onChunk,
                                             patientTimeoutMs, 0);

    const ULONGLONG elapsedMs = GetTickCount64() - startedAt;
    const double rate = (elapsedMs > 0)
        ? (static_cast<double>(fileSize) / 1048576.0) / (elapsedMs / 1000.0) : 0.0;
    char rateBuf[200];
    std::snprintf(rateBuf, sizeof(rateBuf),
                  "[bitvise] done rc=%d time=%.1fs rate=%.2f MB/s",
                  r.rc, elapsedMs / 1000.0, rate);
    recoveryLog(rateBuf);

    if (r.rc != 0) {
        std::string msg = "sftpc exit rc=" + std::to_string(r.rc);
        // Annotate the most common failure modes so the user knows what
        // actually happened without parsing Bitvise's chatty diagnostics.
        if (r.output.find("Windows error 10054") != std::string::npos ||
            r.output.find("forcibly closed") != std::string::npos ||
            r.output.find("FlowSocketWriter") != std::string::npos) {
            msg += " (TCP connection reset by device -- Wi-Fi flap or device busy)";
        } else if (r.output.find("Connection timed out") != std::string::npos) {
            msg += " (Wi-Fi connection timed out -- check signal / re-join AP)";
        } else if (r.output.find("Authentication") != std::string::npos) {
            msg += " (auth failed -- check device password)";
        } else if (r.output.find("error Failure") != std::string::npos &&
                   r.output.find("already exists") != std::string::npos) {
            msg += " (remote file already exists -- should not happen with -o, file a bug)";
        }
        if (!r.output.empty()) msg += ": " + tailText(r.output, 300);
        return msg;
    }
    char buf[128];
    const double totalMb = static_cast<double>(fileSize) / (1024.0 * 1024.0);
    std::snprintf(buf, sizeof(buf), "%s %.1f/%.1f MB", label.c_str(), totalMb, totalMb);
    setDeployProgress(progress, basePct + spanPct, buf);
    return {};
}

// Front-door uploader: Bitvise sftpc.exe only. On the recovery AP it has
// better flow control than pscp, and falling back to pscp after a transient
// AP flap just turns a recoverable upload into a hard disconnect.
std::string uploadFileToRecovery(const std::filesystem::path& src,
                                 const std::string& remotePath,
                                 const std::shared_ptr<DeployProgressState>& progress,
                                 int basePct, int spanPct,
                                 const std::string& label,
                                 uint32_t timeoutMs)
{
    if (recoveryBitviseSftpc().empty()) {
        return "Bitvise sftpc.exe not found. Install Bitvise SSH Client or keep "
               "dist/ssh/sftpc.exe next to DriveScope.exe";
    }

    std::string lastErr;
    constexpr int kAttempts = 4;
    for (int attempt = 1; attempt <= kAttempts; ++attempt) {
        if (attempt > 1) {
            const int retryPct = basePct + std::max(0, spanPct) / 10;
            setDeployProgress(progress, retryPct,
                              label + " Bitvise retry " + std::to_string(attempt));
            recoveryLog("[upload] retrying Bitvise sftpc attempt " +
                        std::to_string(attempt) + "/" + std::to_string(kAttempts));
            std::this_thread::sleep_for(std::chrono::milliseconds(1200 * attempt));
            invalidateRecoveryHostCache("Bitvise upload retry");
        }

        std::string err = uploadFileViaBitvise(src, remotePath, progress,
                                               basePct, spanPct, label, timeoutMs);
        if (err.empty()) return {};
        lastErr = std::move(err);
        recoveryLog("[upload] Bitvise sftpc attempt " +
                    std::to_string(attempt) + "/" + std::to_string(kAttempts) +
                    " failed: " + tailText(lastErr, 240));
    }

    return "Bitvise upload failed after " + std::to_string(kAttempts) +
           " attempts: " + lastErr;
}

// Upload a single local file to a remote path on the recovery device via
// pscp -sftp. Time-based progress synthesised from a 1.2 MB/s estimate;
// stdout/stderr в†’ NUL so pscp's writes can never block on a Win32 pipe.
// Returns empty string on success, error description otherwise.
std::string uploadFileViaPscp(const std::filesystem::path& src,
                              const std::string& remotePath,
                              const std::shared_ptr<DeployProgressState>& progress,
                              int basePct, int spanPct,
                              const std::string& label,
                              uint32_t timeoutMs)
{
    const std::string pscp = recoveryPscpExe();
    if (pscp.empty()) return "pscp.exe not found";

    const std::string kHostStr = resolveRecoveryHost();
    const char* kHost = kHostStr.c_str();
    constexpr const char* kPassword = "bork2025";
    // Prefer the live device fingerprint when probable (Windows OpenSSH
    // present); fall back to the historical pinned recovery-image key for
    // boxes without ssh-keyscan.
    const std::string discoveredFp = discoveredHostKeyFp(kHostStr);
    const std::string kKnownHostKey = !discoveredFp.empty()
        ? discoveredFp
        : std::string("SHA256:l63iZ9rjClz4nzwCIogn8oW3CBpQEHnCMzMbp+hDdzU");

    const std::string remote = std::string("root@") + kHost + ":" + remotePath;
    const std::string cmdStr = quoteArg(pscp) +
        " -batch -sftp -hostkey " + quoteArg(kKnownHostKey) +
        " -pw " + quoteArg(kPassword) +
        " -q " +
        " " + quoteArg(src.string()) + " " + quoteArg(remote);

    HANDLE hNul = CreateFileA("NUL", GENERIC_WRITE,
                              FILE_SHARE_READ | FILE_SHARE_WRITE,
                              nullptr, OPEN_EXISTING, 0, nullptr);
    HANDLE hNulInherit = nullptr;
    if (hNul != INVALID_HANDLE_VALUE) {
        DuplicateHandle(GetCurrentProcess(), hNul, GetCurrentProcess(),
                        &hNulInherit, 0, TRUE, DUPLICATE_SAME_ACCESS);
        CloseHandle(hNul);
    }

    STARTUPINFOA si{};
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdInput = nullptr;
    si.hStdOutput = hNulInherit;
    si.hStdError = hNulInherit;

    PROCESS_INFORMATION pi{};
    std::vector<char> cmdBuf(cmdStr.begin(), cmdStr.end());
    cmdBuf.push_back('\0');
    const uintmax_t fileSize = std::filesystem::file_size(src);
    recoveryLog("[pscp] start " + label + " src=" + src.string() +
                " dst=" + remotePath +
                " size=" + formatBytes(fileSize));
    const BOOL spawned = CreateProcessA(nullptr, cmdBuf.data(), nullptr, nullptr,
                                        TRUE, CREATE_NO_WINDOW, nullptr, nullptr,
                                        &si, &pi);
    if (hNulInherit) CloseHandle(hNulInherit);
    if (!spawned) {
        const DWORD err = GetLastError();
        recoveryLog("[pscp] CreateProcess failed err=" + std::to_string(err));
        return "CreateProcess(pscp) failed err=" + std::to_string(err);
    }

    const ULONGLONG startedAt = GetTickCount64();
    const double estimatedBps = 1.2 * 1024.0 * 1024.0;
    for (;;) {
        const DWORD wait = WaitForSingleObject(pi.hProcess, 250);
        const ULONGLONG nowMs = GetTickCount64() - startedAt;
        if (wait == WAIT_OBJECT_0) break;
        if (nowMs > timeoutMs) {
            TerminateProcess(pi.hProcess, 1);
            WaitForSingleObject(pi.hProcess, 2000);
            CloseHandle(pi.hThread);
            CloseHandle(pi.hProcess);
            return "pscp timed out after " + std::to_string(timeoutMs / 1000) + "s";
        }
        const double sentBytes =
            std::min<double>(estimatedBps * (nowMs / 1000.0),
                             0.95 * static_cast<double>(fileSize));
        const int pct = static_cast<int>(
            std::clamp<double>((sentBytes * 100.0) / static_cast<double>(fileSize),
                               0.0, 95.0));
        const int outPct = basePct + (spanPct * pct) / 100;
        const double sentMb  = sentBytes / (1024.0 * 1024.0);
        const double totalMb = static_cast<double>(fileSize) / (1024.0 * 1024.0);
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s %.1f/%.1f MB",
                      label.c_str(), sentMb, totalMb);
        setDeployProgress(progress, outPct, buf);
    }

    DWORD exitCode = 1;
    GetExitCodeProcess(pi.hProcess, &exitCode);
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);

    const ULONGLONG elapsedMs = GetTickCount64() - startedAt;
    const double rate = (elapsedMs > 0)
        ? (static_cast<double>(fileSize) / 1048576.0) / (elapsedMs / 1000.0) : 0.0;
    char rateBuf[160];
    std::snprintf(rateBuf, sizeof(rateBuf),
                  "[pscp] done rc=%lu time=%.1fs rate=%.2f MB/s",
                  static_cast<unsigned long>(exitCode), elapsedMs / 1000.0, rate);
    recoveryLog(rateBuf);

    if (exitCode != 0) {
        return "pscp exit rc=" + std::to_string(exitCode) +
               " -- likely network drop on recovery AP";
    }
    char buf[128];
    const double totalMb = static_cast<double>(fileSize) / (1024.0 * 1024.0);
    std::snprintf(buf, sizeof(buf), "%s %.1f/%.1f MB", label.c_str(), totalMb, totalMb);
    setDeployProgress(progress, basePct + spanPct, buf);
    return {};
}

std::string runRecoveryInstall(const std::string& appDirText,
                               const std::shared_ptr<DeployProgressState>& progress)
{
    namespace fs = std::filesystem;
    const ULONGLONG totalStart = GetTickCount64();
    invalidateRecoveryHostCache("/app sync start");
    recoveryLog("============= /app sync START appDir=" + appDirText + " =============");

    setDeployProgress(progress, 2, "checking app folder");

    fs::path appDir = fs::path(appDirText);
    if (!fs::is_directory(appDir)) {
        recoveryLog("[fail] app dir missing: " + appDir.string());
        return "recovery /app sync: app folder not found: " + appDir.string();
    }
    if (appDir.filename().string() != "app") {
        recoveryLog("[fail] folder not named 'app': " + appDir.filename().string());
        return "recovery /app sync: selected folder must be named app";
    }

    std::error_code ec;
    fs::path tmpDir = fs::temp_directory_path(ec) /
                      ("drivescope-recovery-" +
                       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    if (ec || !fs::create_directories(tmpDir, ec)) {
        recoveryLog("[fail] cannot create temp folder: " + tmpDir.string());
        return "recovery /app sync: cannot create temp folder";
    }
    auto cleanup = [&]() { fs::remove_all(tmpDir, ec); };

    const fs::path tarPath = tmpDir / "drivescope_app.tar.gz";

    setDeployProgress(progress, 6, "packing app folder");
    const ULONGLONG tarStart = GetTickCount64();
    {
        std::string cmd = std::string("tar.exe -C ") + quoteArg(appDir.parent_path().string()) +
                          " -czf " + quoteArg(tarPath.string()) + " " +
                          quoteArg(appDir.filename().string());
        const CommandResult r = runCommandHidden(cmd, {}, 600000, 60000);
        if (r.rc != 0) {
            recoveryLog("[fail] tar rc=" + std::to_string(r.rc) + " " + tailText(r.output));
            cleanup();
            return "recovery /app sync: pack app folder failed rc=" +
                   std::to_string(r.rc) +
                   (r.output.empty() ? std::string{} : ": " + tailText(r.output));
        }
    }

    const uintmax_t archiveSize = fs::file_size(tarPath, ec);
    {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "[tar] done time=%.1fs size=%s",
                      (GetTickCount64() - tarStart) / 1000.0,
                      formatBytes(archiveSize).c_str());
        recoveryLog(buf);
    }
    setDeployProgress(progress, 14, "app archive ready: " + formatBytes(archiveSize));

    // Strategy (the simple, brick-proof one the user asked for):
    //
    //   STEP 1 -- upload tar.gz to /tmp on the device. /app is untouched.
    //            If the network dies, /app is still intact в†’ no brick,
    //            user just retries.
    //   STEP 2 -- kill every process that holds /app open (httpd, app_dd,
    //            otaunpack, xor_file).
    //   STEP 3 -- wipe /app and extract the local /tmp tar.gz into /app.
    //            This step doesn't touch the network so it can't stall
    //            mid-extract from a Wi-Fi blip.
    //   STEP 4 -- clean up /tmp, chmod, sync, confirm.
    //
    // The two-SSH-call split is intentional: STEP 1 uses `cat` as the
    // stdin sink (the simplest possible reader -- starts draining stdin
    // immediately, no pre-cat work, so plink's WriteFile never blocks).
    // STEP 2-4 has no stdin involved в†’ can take as long as it needs.
    //
    // Tmpfs sanity: tar.gz is ~21 MB, /tmp has 50 MB free. We delete it
    // right after extraction.

    // ---- STEP 0: stop app_dd and friends BEFORE the SFTP upload ----
    // When we land here from "Service AP" (operator switched the device to
    // AP mode via CAN cmd 0x100 but app_dd is still running), app_dd is
    // pegging the SOM CPU rendering UI / decoding fonts / serving its
    // command server / running CAN bridge. SFTP transfer over the same
    // device's sshd then crawls or stalls (UI shows "16% /app upload
    // starting" not progressing). Killing app_dd first releases the CPU
    // and frees up /app/* file handles. This is also safe in pure recovery
    // mode (button-held boot) -- there's no app_dd to kill, killall just
    // returns 0. We do NOT touch hostapd/dnsmasq/sshd, so the AP and our
    // SSH session stay up.
    setDeployProgress(progress, 15, "stopping app_dd before upload");
    {
        const CommandResult kr = runRecoveryRemoteSh(
            gracefulStopAppDdSh(/*includeHttpd=*/true) +
            "sleep 1; "
            "echo PRE_KILL_OK",
            20000, 12000, "pre-kill-app_dd");
        if (kr.output.find("PRE_KILL_OK") == std::string::npos) {
            cleanup();
            return std::string("recovery /app sync: could not reach device sshd "
                   "to stop app_dd. ") + tailText(kr.output, 300);
        }
    }

    // ---- STEP 1: upload tar.gz to /tmp/drivescope_app.tar.gz via SFTP ----
    // Single attempt -- with app_dd dead and the WLAN scan paused, the
    // device's CPU is free and the upload should land first try.
    setDeployProgress(progress, 18, "/app upload starting");
    {
        const std::string err = uploadFileToRecovery(
            tarPath, "/tmp/drivescope_app.tar.gz", progress,
            /*basePct=*/18, /*spanPct=*/68,
            "/app upload", /*timeoutMs=*/240000);
        if (!err.empty()) {
            recoveryLog("[fail] upload: " + err);
            cleanup();
            return "recovery /app sync: upload failed: " + err +
                   "  (/app is still intact -- device is not bricked)";
        }

        // Verify the uploaded size matches what we sent -- a partial SFTP
        // success is rare but possible on a brutal network drop.
        const CommandResult sr = runRecoveryRemoteSh(
            "wc -c < /tmp/drivescope_app.tar.gz",
            15000, 8000, "size-verify");
        if (sr.rc == 0) {
            const uintmax_t got = std::strtoull(sr.output.c_str(), nullptr, 10);
            if (got != archiveSize) {
                cleanup();
                return "recovery /app sync: uploaded size mismatch (" +
                       formatBytes(got) + " on device vs " +
                       formatBytes(archiveSize) + " local) -- retry";
            }
        }
    }
    cleanup();  // local tar.gz no longer needed; the device has its copy

    // ---- STEP 2-4: stop services, wipe /app, extract from /tmp, clean up ----
    setDeployProgress(progress, 88, "stopping services, replacing /app");
    const ULONGLONG installStart = GetTickCount64();
    {
        // Verify the uploaded archive then kill processes, wipe /app, and
        // extract. Verifying first means a corrupted upload (-z test fails)
        // never wipes a working /app. /app/httpd is killed twice -- once
        // before the wipe, once after, in case any watchdog respawns it.
        const std::string installCmd =
            // verify archive integrity (gzip header + tar OK) before any
            // destructive operation. If this fails /app stays intact.
            std::string("gzip -t /tmp/drivescope_app.tar.gz || { echo BAD_ARCHIVE; exit 1; }; ") +
            // kill everything that holds /app open
            gracefulStopAppDdSh(/*includeHttpd=*/true) +
            "sleep 1; "
            "killall -9 httpd 2>/dev/null; "  // in case a watchdog respawned it
            // wipe /app contents
            "rm -rf /app/* /app/.[!.]* 2>/dev/null; "
            "mkdir -p /app && "
            // extract from local tmpfs file -- no network involved here
            "tar -xzf /tmp/drivescope_app.tar.gz -C / || { echo EXTRACT_FAIL; exit 1; }; "
            // clean up the staged archive
            "rm -f /tmp/drivescope_app.tar.gz; "
            // finalize permissions and confirm
            "chmod 755 /app/app_dd /app/otaunpack /app/xor_file 2>/dev/null; "
            "chmod 755 /app/httpd/cgi-bin/*.sh 2>/dev/null; "
            "sync && echo INSTALL_OK";

        recoveryLog("[install] starting remote (gzip -t, killall, rm, tar -xzf, chmod, sync)");
        const CommandResult ir = runRecoveryRemoteSh(installCmd, 90000, 60000, "install");
        (void)installStart;  // log entry already records time in runRecoveryRemoteSh

        if (ir.output.find("BAD_ARCHIVE") != std::string::npos) {
            return std::string("recovery /app sync: uploaded archive is corrupt -- /app left untouched");
        }
        if (ir.rc != 0) {
            std::string msg = "recovery /app sync: install on device failed rc=" +
                              std::to_string(ir.rc);
            if (ir.output.find("EXTRACT_FAIL") != std::string::npos) {
                msg += " -- tar extract failed; tar.gz still in /tmp/drivescope_app.tar.gz";
            }
            if (!ir.output.empty()) msg += ": " + tailText(ir.output);
            return msg;
        }
        if (ir.output.find("INSTALL_OK") == std::string::npos) {
            return std::string("recovery /app sync: install did not confirm: ") +
                   (ir.output.empty() ? std::string("no remote output") : tailText(ir.output));
        }
    }

    {
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "============= /app sync OK total=%.1fs =============",
                      (GetTickCount64() - totalStart) / 1000.0);
        recoveryLog(buf);
    }
    setDeployProgress(progress, 100, "app update complete");
    return "recovery /app sync: ok, disconnected; press Run App when ready";
}

std::string runRecoveryLinuxOta(const std::string& otaPathText, const std::string& appDirText,
                                const std::shared_ptr<DeployProgressState>& progress)
{
    namespace fs = std::filesystem;
    const ULONGLONG totalStart = GetTickCount64();
    invalidateRecoveryHostCache("Linux OTA start");
    recoveryLog("============= Linux OTA START ota=" + otaPathText +
                " appDir=" + appDirText + " =============");

    setDeployProgress(progress, 2, "checking OTA package");

    fs::path otaPath = fs::path(otaPathText);
    if (!fs::is_regular_file(otaPath)) {
        recoveryLog("[fail] OTA package not found: " + otaPath.string());
        return "recovery Linux OTA: package not found: " + otaPath.string();
    }
    if (otaPath.filename().string() != "SStarOta.bin.gz") {
        return "recovery Linux OTA: selected file must be SStarOta.bin.gz";
    }

    fs::path appDir = fs::path(appDirText);
    fs::path localOtaunpack = appDir / "otaunpack";
    const bool hasLocalOtaunpack = fs::is_regular_file(localOtaunpack);

    const uintmax_t otaSize = fs::file_size(otaPath);
    setDeployProgress(progress, 6, "OTA package ready: " + formatBytes(otaSize));

    // STEP 0: stop app_dd before any SFTP transfer. Same rationale as in
    // runRecoveryInstall -- keeping app_dd alive while uploading via the
    // device's sshd starves the SOM CPU and SFTP crawls.
    setDeployProgress(progress, 5, "stopping app_dd before OTA");
    {
        const CommandResult kr = runRecoveryRemoteSh(
            gracefulStopAppDdSh(/*includeHttpd=*/true) +
            "sleep 1; "
            "echo PRE_KILL_OK",
            20000, 12000, "ota-pre-kill");
        if (kr.output.find("PRE_KILL_OK") == std::string::npos) {
            return std::string("recovery Linux OTA: could not reach device sshd "
                   "to stop app_dd. ") + tailText(kr.output, 300);
        }
    }

    // Settle delay + SSH preflight: when OTA runs immediately after a /app
    // sync, the device has just been hammered with rm -rf + tar -xzf + sync
    // + chmod, plus we killed busybox httpd. The kernel/dropbear can be busy
    // enough on the weak SOM CPU that a fresh SFTP connection during this
    // window gets RST'd mid-transfer (Windows error 10054 -- "remote forcibly
    // closed connection"). Three seconds of breathing room reliably avoids
    // it. If OTA is launched standalone (no /app sync just before), this is
    // a tiny one-time wait. The preflight after also confirms sshd is up
    // before we start the heavier SFTP transfer.
    setDeployProgress(progress, 7, "device settling after install");
    recoveryLog("[ota] settling 3 s before SFTP");
    Sleep(3000);
    {
        const CommandResult ping = runRecoveryRemoteSh(
            "echo PING_OK", 10000, 6000, "ota-preflight");
        if (ping.rc != 0 || ping.output.find("PING_OK") == std::string::npos) {
            return std::string("recovery Linux OTA: device SSH not responsive -- "
                   "press Reset device and retry. ") + tailText(ping.output, 300);
        }
    }

    // ---- STEP 1: upload local otaunpack to /app/otaunpack via Bitvise SFTP ----
    // Recovery firmware images sometimes ship without /app/otaunpack; we
    // always push the local one first so OTA never fails on a missing tool.
    if (hasLocalOtaunpack) {
        setDeployProgress(progress, 8, "uploading otaunpack");
        const std::string err = uploadFileToRecovery(
            localOtaunpack, "/app/otaunpack", progress,
            /*basePct=*/8, /*spanPct=*/10, "otaunpack",
            /*timeoutMs=*/120000);
        if (!err.empty()) {
            recoveryLog("[fail] otaunpack upload: " + err);
            return "recovery Linux OTA: otaunpack upload failed: " + err;
        }
        const CommandResult cr = runRecoveryRemoteSh(
            "chmod 755 /app/otaunpack && sync && echo CHMOD_OK",
            15000, 8000, "chmod-otaunpack");
        if (cr.rc != 0 || cr.output.find("CHMOD_OK") == std::string::npos) {
            return "recovery Linux OTA: chmod /app/otaunpack failed: " +
                   tailText(cr.output);
        }
    }

    // ---- STEP 2: upload OTA payload to /tmp/SStarOta.bin.gz via Bitvise SFTP ----
    // We stage in /tmp (tmpfs, RAM) so the device-side extract reads from
    // RAM and the upload itself never touches /app -- if the network drops
    // mid-upload the device is still bootable.
    setDeployProgress(progress, 20, "OTA payload upload starting");
    {
        const std::string err = uploadFileToRecovery(
            otaPath, "/tmp/SStarOta.bin.gz", progress,
            /*basePct=*/20, /*spanPct=*/55, "OTA upload",
            /*timeoutMs=*/300000);
        if (!err.empty()) {
            recoveryLog("[fail] OTA upload: " + err);
            return "recovery Linux OTA: upload failed: " + err +
                   "  (device is still in recovery mode -- retry)";
        }
    }

    // ---- STEP 3: verify gzip + size, then run otaunpack on the device ----
    setDeployProgress(progress, 78, "device unpacking OTA");
    {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%llu",
                      static_cast<unsigned long long>(otaSize));
        const std::string installCmd =
            std::string("test -s /tmp/SStarOta.bin.gz || { echo NO_PAYLOAD; exit 1; }; "
            "got=$(wc -c < /tmp/SStarOta.bin.gz); "
            "[ \"$got\" = \"") + buf + "\" ] || { echo SIZE_MISMATCH got=$got; exit 1; }; "
            "gzip -t /tmp/SStarOta.bin.gz || { echo BAD_ARCHIVE; exit 1; }; "
            "test -x /app/otaunpack || { echo NO_OTAUNPACK; exit 1; }; "
            "/app/otaunpack -x /tmp/SStarOta.bin.gz; rc=$?; "
            "rm -f /tmp/SStarOta.bin.gz; "
            "sync; echo OTA_RC=$rc";
        const CommandResult ir = runRecoveryRemoteSh(installCmd, 600000, 300000, "ota-install");

        if (ir.output.find("NO_PAYLOAD") != std::string::npos) {
            return "recovery Linux OTA: payload disappeared from /tmp before unpack";
        }
        if (ir.output.find("SIZE_MISMATCH") != std::string::npos) {
            return "recovery Linux OTA: uploaded size mismatch -- retry: " +
                   tailText(ir.output);
        }
        if (ir.output.find("BAD_ARCHIVE") != std::string::npos) {
            return "recovery Linux OTA: uploaded archive is corrupt (gzip -t failed) -- retry";
        }
        if (ir.output.find("NO_OTAUNPACK") != std::string::npos) {
            return "recovery Linux OTA: device has no /app/otaunpack -- Upload /app first";
        }
        const size_t markerPos = ir.output.find("OTA_RC=");
        if (markerPos == std::string::npos) {
            return "recovery Linux OTA: did not confirm: " + tailText(ir.output);
        }
        const int otaRc = std::atoi(ir.output.c_str() + markerPos + 7);
        if (otaRc != 0) {
            return "recovery Linux OTA: otaunpack rc=" + std::to_string(otaRc) +
                   " -- " + tailText(ir.output);
        }
    }

    {
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "============= Linux OTA OK total=%.1fs =============",
                      (GetTickCount64() - totalStart) / 1000.0);
        recoveryLog(buf);
    }
    setDeployProgress(progress, 100, "Linux OTA complete");
    return hasLocalOtaunpack
         ? "recovery Linux OTA: ok, press Run App or Reset device"
         : "recovery Linux OTA: ok using device otaunpack, press Run App or Reset device";
}

std::string runRecoveryLaunchApp()
{
    invalidateRecoveryHostCache("Run App start");
    recoveryLog("[run-app] launching app_dd without pc_tool auto-connect");

    const std::string remoteCmd =
        "test -x /app/app_dd || { echo MISSING_APP_DD; exit 1; }; "
        // httpd/recovery UI can hold /app/httpd files and the LCD stack; stop
        // it, but do not touch hostapd/dnsmasq/sshd or we lose AP/SSH.
        "killall -TERM httpd 2>/dev/null || true; "
        "sleep 0.3; "
        "killall -9 httpd 2>/dev/null || true; "
        "if pidof app_dd >/dev/null 2>&1; then "
        "  echo APP_ALREADY_RUNNING; "
        "  pidof app_dd | head -1 | awk '{print \"PID=\"$1}'; "
        "  exit 0; "
        "fi; "
        "rm -f /tmp/app_dd.log; "
        "(cd /app && setsid nohup /app/app_dd > /tmp/app_dd.log 2>&1 < /dev/null &); "
        "sleep 0.8; "
        "if pidof app_dd >/dev/null 2>&1; then "
        "  echo APP_STARTED; "
        "  pidof app_dd | head -1 | awk '{print \"PID=\"$1}'; "
        "  exit 0; "
        "fi; "
        "echo APP_START_NOT_CONFIRMED; "
        "tail -n 30 /tmp/app_dd.log 2>/dev/null";

    const CommandResult result = runRecoveryRemoteSh(remoteCmd, 20000, 12000, "run-app");

    if (result.output.find("MISSING_APP_DD") != std::string::npos) {
        return "recovery run app: /app/app_dd not on device -- Upload /app first";
    }
    if (result.output.find("APP_STARTED") != std::string::npos) {
        return "recovery run app: app_dd started; wait a few seconds, then Connect manually";
    }
    if (result.output.find("APP_ALREADY_RUNNING") != std::string::npos) {
        return "recovery run app: app_dd already running; Connect manually";
    }
    if (result.output.find("APP_START_NOT_CONFIRMED") != std::string::npos) {
        return std::string("recovery run app: launch sent but app_dd pid not confirmed -- ") +
               tailText(result.output, 500);
    }
    if (result.rc != 0) {
        std::string msg = "recovery run app: SSH failed rc=" + std::to_string(result.rc);
        if (!result.output.empty()) msg += ": " + tailText(result.output);
        return msg;
    }
    return "recovery run app: no confirmation: " + tailText(result.output);
}

std::string runRecoveryReset()
{
    invalidateRecoveryHostCache("Reset device start");
    // The reboot command drops the SSH session immediately, so the SSH
    // wrapper may exit non-zero with "Server unexpectedly closed network
    // connection" -- that's fine as long as we sent the command.
    const CommandResult result = runRecoveryRemoteSh(
        "sync; reboot", 15000, 8000, "reset");

    // plink returns 0 normally; if the device reboots before plink exits
    // it returns ~1 with a "Server unexpectedly closed network connection"
    // tail in stderr -- accept that as "command sent".
    std::string lowerOutput = result.output;
    std::transform(lowerOutput.begin(), lowerOutput.end(), lowerOutput.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    const bool serverClosed =
        lowerOutput.find("unexpectedly closed") != std::string::npos ||
        lowerOutput.find("connection reset")    != std::string::npos ||
        lowerOutput.find("connection abort")    != std::string::npos ||
        lowerOutput.find("connection lost")     != std::string::npos ||
        lowerOutput.find("connection closed")   != std::string::npos ||
        lowerOutput.find("session has terminated") != std::string::npos ||
        lowerOutput.find("server disconnected") != std::string::npos;
    if (result.rc != 0 && !serverClosed) {
        std::string msg = "recovery reset: SSH reboot failed rc=" +
                          std::to_string(result.rc);
        if (!result.output.empty()) msg += ": " + tailText(result.output);
        return msg;
    }
    return "recovery reset: SSH reboot command sent";
}

enum class SomFlashKind { Esp32, Motor, Rk };

const char* somFlashKindName(SomFlashKind kind)
{
    switch (kind) {
        case SomFlashKind::Esp32: return "esp32";
        case SomFlashKind::Motor: return "motor";
        case SomFlashKind::Rk:    return "rk";
    }
    return "unknown";
}

std::string somFlashKindLabel(SomFlashKind kind)
{
    switch (kind) {
        case SomFlashKind::Esp32: return "ESP32";
        case SomFlashKind::Motor: return "Motor";
        case SomFlashKind::Rk:    return "RK";
    }
    return "Unknown";
}

uint32_t uploadTimeoutFor(const std::filesystem::path& path)
{
    namespace fs = std::filesystem;
    std::error_code ec;
    const uintmax_t fileSize = fs::file_size(path, ec);
    if (ec || fileSize == 0) return 120000;
    return static_cast<uint32_t>(
        std::clamp<uintmax_t>((fileSize / 1024u) * 250u, 120000u, 300000u));
}

std::string runSomPeripheralFlash(SomFlashKind kind,
                                  const std::string& sourceText,
                                  const std::shared_ptr<DeployProgressState>& progress)
{
#ifdef _WIN32
    namespace fs = std::filesystem;
    const std::string kindName = somFlashKindName(kind);
    const std::string kindLabel = somFlashKindLabel(kind);
    invalidateRecoveryHostCache((std::string("wifi flash ") + kindName + " start").c_str());
    recoveryLog("============= Wi-Fi peripheral flash START kind=" + kindName +
                " src=" + sourceText + " =============");

    setDeployProgress(progress, 2, "checking " + kindLabel + " files");

    const std::string remoteDir = "/app/firmware/" + kindName;
    {
        const CommandResult prep = runRecoveryRemoteSh(
            "mkdir -p " + shellSingleQuote(remoteDir) + " && echo PREP_OK",
            15000, 8000, "peripheral-flash-prepare");
        if (prep.rc != 0 || prep.output.find("PREP_OK") == std::string::npos) {
            return "wifi flash " + kindName +
                   ": could not prepare " + remoteDir + " over SSH. " +
                   tailText(prep.output, 300);
        }
    }

    if (kind == SomFlashKind::Esp32) {
        fs::path dir(resolveDataPath(sourceText.empty() ? defaultEsp32FirmwareDir() : sourceText));
        if (!fs::is_directory(dir)) {
            return "wifi flash esp32: firmware folder not found: " + dir.string();
        }

        const char* required[] = {
            "bootloader.bin",
            "ota_data_initial.bin",
            "partitions.bin",
            "firmware.bin",
        };
        constexpr size_t requiredCount = sizeof(required) / sizeof(required[0]);
        for (const char* f : required) {
            if (!fs::is_regular_file(dir / f)) {
                return std::string("wifi flash esp32: missing ") + f +
                       " in " + dir.string();
            }
        }

        for (size_t i = 0; i < requiredCount; ++i) {
            const fs::path local = dir / required[i];
            const int base = 8 + static_cast<int>((40 * i) / requiredCount);
            const int next = 8 + static_cast<int>((40 * (i + 1)) / requiredCount);
            const std::string err = uploadFileToRecovery(
                local, remoteDir + "/" + required[i], progress,
                base, next - base, std::string("esp32 ") + required[i],
                uploadTimeoutFor(local));
            if (!err.empty()) {
                return "wifi flash esp32: upload failed: " + err;
            }
        }
    } else {
        fs::path local(resolveDataPath(sourceText));
        if (!fs::is_regular_file(local)) {
            return "wifi flash " + kindName + ": firmware file not found: " +
                   local.string();
        }
        const std::string expected =
            (kind == SomFlashKind::Motor) ? "APP_MOTOR.bin" : "APP_RK.bin";
        const std::string err = uploadFileToRecovery(
            local, remoteDir + "/" + expected, progress,
            8, 40, kindName + " bin", uploadTimeoutFor(local));
        if (!err.empty()) {
            return "wifi flash " + kindName + ": upload failed: " + err;
        }
    }

    setDeployProgress(progress, 50, "triggering /flash/" + kindName);
    const std::string endpoint = "http://127.0.0.2:10011/flash/" + kindName;
    const uint32_t flashTimeoutMs = 600000;
    const std::string remoteCmd =
        "if command -v wget >/dev/null 2>&1; then exec wget -T 590 -qO- " +
        endpoint + "; fi; "
        "if command -v curl >/dev/null 2>&1; then exec curl --connect-timeout 20 "
        "--max-time 590 -sN " + endpoint + "; fi; "
        "echo 'no wget or curl on device' >&2; exit 97";

    std::string lineBuf;
    std::string terminal;
    int lastPct = -1;
    auto handleLine = [&](std::string line) {
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
            line.pop_back();
        }
        if (line.empty()) return;
        recoveryLog("[wifi-flash " + kindName + "] " + line);
        if (line.rfind("start ", 0) == 0) {
            setDeployProgress(progress, 52, "flash started");
            return;
        }
        if (line.rfind("progress ", 0) == 0) {
            const int pct = std::clamp(std::atoi(line.c_str() + 9), 0, 100);
            if (pct != lastPct) {
                lastPct = pct;
                setDeployProgress(progress, 50 + pct / 2,
                                  kindLabel + " flash " + std::to_string(pct) + "%");
            }
            return;
        }
        if (line.rfind("done", 0) == 0 ||
            line.rfind("error", 0) == 0 ||
            line.rfind("timeout", 0) == 0) {
            terminal = line;
        }
    };
    auto onOutput = [&](const std::string& chunk) {
        lineBuf += chunk;
        for (;;) {
            const size_t nl = lineBuf.find_first_of("\r\n");
            if (nl == std::string::npos) break;
            std::string line = lineBuf.substr(0, nl);
            lineBuf.erase(0, nl + 1);
            handleLine(line);
        }
    };

    const CommandResult flash = runRecoveryRemoteSh(
        remoteCmd, flashTimeoutMs, 0,
        (std::string("flash-") + kindName).c_str(), onOutput);
    if (!lineBuf.empty()) {
        handleLine(lineBuf);
        lineBuf.clear();
    }

    if (terminal == "done") {
        setDeployProgress(progress, 100, kindLabel + " flash done");
        recoveryLog("============= Wi-Fi peripheral flash OK kind=" + kindName +
                    " =============");
        return "wifi flash " + kindName + ": done";
    }
    if (!terminal.empty()) {
        return "wifi flash " + kindName + ": " + terminal +
               ". See " + recoveryLogPath();
    }
    if (flash.rc != 0) {
        return "wifi flash " + kindName + ": command failed rc=" +
               std::to_string(flash.rc) +
               " -- is app_dd running with COMMAND_SERVER=1? " +
               tailText(flash.output, 400);
    }
    return "wifi flash " + kindName +
           ": no terminal status from /flash endpoint -- is app_dd running "
           "with COMMAND_SERVER=1? " + tailText(flash.output, 400);
#else
    (void)kind; (void)sourceText; (void)progress;
    return "wifi peripheral flash is available on Windows builds.";
#endif
}
} // namespace

void MainUi::setScenarioPath(const std::string& path)
{
    scenarioPath_ = path;
    if (!scenarioPath_.empty()) {
        setTextBuffer(varsConfigPath_, scenarioPath_);
        scenarioStatus_ = "scenario: " + scenarioPath_;
    }
}

std::string MainUi::scenarioPathFor(const std::string& path) const
{
    namespace fs = std::filesystem;
    if (path.empty() || scenarioPath_.empty()) return path;

    fs::path scenarioDir = fs::absolute(fs::path(scenarioPath_)).parent_path();
    fs::path candidate(path);
    if (!candidate.is_absolute()) {
        const std::string resolved = resolveDataPath(path);
        if (fs::exists(resolved)) candidate = fs::absolute(resolved);
    }

    if (candidate.is_absolute()) {
        std::error_code ec;
        fs::path rel = fs::relative(candidate, scenarioDir, ec);
        if (!ec && !rel.empty()) {
            const std::string generic = rel.generic_string();
            if (generic.rfind("..", 0) != 0) return generic;
        }
    }

    return fs::path(path).generic_string();
}

void MainUi::markScenarioDirty(const char* reason)
{
    if (scenarioPath_.empty()) return;
    scenarioDirty_ = true;
    if (reason != nullptr && reason[0] != '\0') scenarioDirtyReason_ = reason;
    scenarioSaveDue_ = clock_.nowSeconds() + 0.35;
}

void MainUi::maybeSaveScenario()
{
    if (!scenarioDirty_) return;
    if (clock_.nowSeconds() < scenarioSaveDue_) return;
    saveScenarioNow(scenarioDirtyReason_.empty() ? "autosave" : scenarioDirtyReason_.c_str());
}

bool MainUi::saveScenarioNow(const char* reason)
{
    namespace fs = std::filesystem;
    if (scenarioPath_.empty()) return false;

    ensureDefaultSources();
    if (!recoveryAppDirInitialized_) {
        setTextBuffer(recoveryAppDir_, defaultRecoveryAppDir());
        setTextBuffer(recoveryOtaPath_, defaultRecoveryOtaPath());
        setTextBuffer(esp32FirmwareDir_, defaultEsp32FirmwareDir());
        recoveryAppDirInitialized_ = true;
    }

    const fs::path outPath(scenarioPath_);
    std::error_code ec;
    fs::create_directories(fs::absolute(outPath).parent_path(), ec);

    std::ofstream f(scenarioPath_, std::ios::binary);
    if (!f) {
        scenarioStatus_ = "scenario save failed: " + scenarioPath_;
        return false;
    }

    const std::string transport =
        transportKind_ == TransportKind::Slcan  ? "slcan"   :
        transportKind_ == TransportKind::WifiAp ? "wifi_ap" :
        transportKind_ == TransportKind::Manual ? "manual"  :
                                                  "tcp";
    f << "{\n"
      << "  \"version\": 1,\n"
      << "  \"description\": \"DriveScope portable launch scenario. Keep this file next to DriveScope.exe.\",\n"
      << "  \"transport\": \"" << transport << "\",\n"
      << "  \"autoconnect\": false,\n"
      << "  \"tcp\": {\n"
      << "    \"enabled\": true,\n"
      << "    \"host\": \"" << jsonEscape(tcpHost_.data()) << "\",\n"
      << "    \"port\": " << tcpPort_ << ",\n"
      << "    \"bus_bitrate\": " << tcpBusBitrate_ << "\n"
      << "  },\n"
      << "  \"wifi_ap\": {\n"
      << "    \"enabled\": true,\n"
      << "    \"ssid\": \"" << jsonEscape(wifiApSsid_.data()) << "\",\n"
      << "    \"host\": \"" << jsonEscape(wifiApHost_.data()) << "\",\n"
      << "    \"port\": " << wifiApPort_ << ",\n"
      << "    \"bus_bitrate\": " << wifiApBusBitrate_ << "\n"
      << "  },\n"
      << "  \"slcan\": {\n"
      << "    \"enabled\": true,\n"
      << "    \"port\": \"" << jsonEscape(comPort_.data()) << "\",\n"
      << "    \"baud\": " << slcanBaud_ << ",\n"
      << "    \"nominal\": \"" << jsonEscape(slcanNominal_.data()) << "\",\n"
      << "    \"data\": \"" << jsonEscape(slcanData_.data()) << "\",\n"
      << "    \"silent\": " << (slcanSilent_ ? "true" : "false") << "\n"
      << "  },\n"
      << "  \"updates\": {\n"
      << "    \"motor_firmware\": \"" << jsonEscape(scenarioPathFor(motorFirmwarePath_.data())) << "\",\n"
      << "    \"rk_firmware\": \"" << jsonEscape(scenarioPathFor(rkFirmwarePath_.data())) << "\",\n"
      << "    \"esp32_firmware_dir\": \"" << jsonEscape(scenarioPathFor(esp32FirmwareDir_.data())) << "\",\n"
      << "    \"main_app_script\": \"" << jsonEscape(scenarioPathFor(mainAppScriptPath_.data())) << "\",\n"
      << "    \"main_resources_script\": \"" << jsonEscape(scenarioPathFor(mainResourcesScriptPath_.data())) << "\",\n"
      << "    \"recovery_app_dir\": \"" << jsonEscape(scenarioPathFor(recoveryAppDir_.data())) << "\",\n"
      << "    \"recovery_ota\": \"" << jsonEscape(scenarioPathFor(recoveryOtaPath_.data())) << "\"\n"
      << "  },\n"
      << "  \"symbols\": [\n";

    for (size_t i = 0; i < sources_.size(); ++i) {
        const SymbolSource& s = sources_[i];
        f << "    \"" << jsonEscape(scenarioPathFor(s.symBuf.data())) << "\""
          << (i + 1 < sources_.size() ? ",\n" : "\n");
    }
    f << "  ],\n"
      << "  \"watches\": [\n";

    for (size_t i = 0; i < watches_.size(); ++i) {
        const WatchVar& w = watches_[i];
        f << "    {\"name\":\"" << jsonEscape(w.name) << "\","
          << "\"address\":\"" << idToHex(w.address, true) << "\","
          << "\"type\":\"" << jsonEscape(w.type) << "\","
          << "\"size\":" << static_cast<int>(w.size) << ","
          << "\"node_id\":" << static_cast<int>(w.nodeId) << ","
          << "\"period_sec\":" << periodSecFromWatchHz(w.pollHz) << ","
          << "\"enabled\":" << (w.enabled ? "true" : "false") << ","
          << "\"plot\":" << (w.plot ? "true" : "false") << ","
          << "\"capture\":" << (w.capture ? "true" : "false") << ","
          << "\"plot_scale\":" << normalizePlotScale(w.plotScale) << ","
          << "\"source\":\"" << jsonEscape(w.source) << "\"}"
          << (i + 1 < watches_.size() ? ",\n" : "\n");
    }
    f << "  ],\n"
      << "  \"capture\": {"
      << "\"node_id\":" << static_cast<int>(capture_.nodeId)
      << ",\"period_us\":" << capture_.periodUs
      << ",\"pre_pct\":" << static_cast<int>(capture_.prePct)
      << ",\"post_pct\":" << static_cast<int>(capture_.postPct)
      << ",\"buffer_kb\":" << static_cast<int>(capture_.bufferKb)
      << ",\"trigger_mode\":" << static_cast<int>(capture_.triggerMode)
      << ",\"trigger_slot\":" << static_cast<int>(capture_.triggerSlot)
      << ",\"trigger_threshold\":" << capture_.triggerThreshold
      << ",\"duration_ms\":" << capture_.durationMs
      << ",\"windows\":[";
    for (size_t i = 0; i < capture_.windowsRel.size(); ++i) {
        const auto& cw = capture_.windowsRel[i];
        f << (i > 0 ? "," : "")
          << "{\"start\":" << cw.startRel << ",\"end\":" << cw.endRel << "}";
    }
    f << "]}\n}\n";

    scenarioDirty_ = false;
    scenarioDirtyReason_.clear();
    scenarioStatus_ = "scenario saved";
    if (reason != nullptr && reason[0] != '\0') scenarioStatus_ += std::string(" (") + reason + ")";
    lastFileMsg_ = "scenario: " + scenarioPath_;
    return true;
}

MainUi::~MainUi()
{
    // The HTTP capture worker is a one-shot std::thread that calls into
    // runCommandHidden (which can sit in WaitForSingleObject for up to
    // ~3 minutes if a TCP fetch is stuck). On clean shutdown we want it
    // joined so the process doesn't surface "thread is destroyed without
    // joining" via std::terminate. The worker is a pure data fetch so
    // detaching would also be safe (it touches only its own atomics +
    // captureHttpResult_), but join() keeps shutdown deterministic.
    if (captureHttpThread_.joinable()) {
        captureHttpThread_.join();
    }
}

void MainUi::applyConfig(const ToolConfig& config)
{
    if      (config.transport == "slcan")   transportKind_ = TransportKind::Slcan;
    else if (config.transport == "wifi_ap") transportKind_ = TransportKind::WifiAp;
    else if (config.transport == "manual")  transportKind_ = TransportKind::Manual;
    else                                    transportKind_ = TransportKind::Tcp;

    setTextBuffer(comPort_, config.slcan.port);
    setTextBuffer(slcanNominal_, config.slcan.nominal);
    setTextBuffer(slcanData_, config.slcan.data);
    slcanBaud_ = config.slcan.baud;
    slcanSilent_ = config.slcan.silent;

    setTextBuffer(tcpHost_, config.tcp.host);
    tcpPort_ = config.tcp.port;
    tcpBusBitrate_ = config.tcp.busBitrate;
    setTextBuffer(wifiApSsid_, config.wifiAp.ssid);
    setTextBuffer(wifiApHost_, config.wifiAp.host);
    wifiApPort_ = config.wifiAp.port;
    wifiApBusBitrate_ = config.wifiAp.busBitrate;

    if (config.updates.present) {
        setTextBuffer(motorFirmwarePath_, config.updates.motorFirmware);
        setTextBuffer(rkFirmwarePath_, config.updates.rkFirmware);
        if (!config.updates.esp32FirmwareDir.empty()) {
            setTextBuffer(esp32FirmwareDir_, config.updates.esp32FirmwareDir);
        }
        setTextBuffer(mainAppScriptPath_, config.updates.mainAppScript);
        setTextBuffer(mainResourcesScriptPath_, config.updates.mainResourcesScript);
        setTextBuffer(recoveryAppDir_, config.updates.recoveryAppDir);
        setTextBuffer(recoveryOtaPath_, config.updates.recoveryOta);
        recoveryAppDirInitialized_ = true;
    }

    // Bind the calibration services to their MainUi-owned dependencies once.
    wireThetaCalibration();
    wireMachineProfilerCalibration();

    ensureDefaultSources();
    for (const std::string& path : config.symbolFiles) {
        SymbolSource& src = ensureSourceForPath(path);
        setTextBuffer(src.symBuf, path);
        loadSymbolsInto(src);
    }

    watches_.clear();
    pending_.clear();
    pendingSentAt_.clear();
    pollCursor_ = 0;
    for (const WatchConfig& watch : config.watches) {
        VariableSymbol sym;
        std::string sourceName;
        if (!resolveWatchConfig(watch, sym, sourceName)) continue;
        WatchVar w;
        static_cast<VariableSymbol&>(w) = sym;
        w.enabled = watch.enabled;
        w.plot = watch.plot;
        w.capture = watch.capture;
        w.plotScale = normalizePlotScale(watch.plotScale);
        w.pollHz = clampWatchHz(watch.hz);
        w.source = sourceName;
        watches_.push_back(std::move(w));
    }

    capture_.nodeId = config.capture.nodeId;
    capture_.periodUs = config.capture.periodUs;
    capture_.prePct = config.capture.prePct;
    capture_.postPct = config.capture.postPct;
    capture_.bufferKb = config.capture.bufferKb;
    capture_.triggerMode = config.capture.triggerMode;
    capture_.triggerSlot = config.capture.triggerSlot;
    capture_.triggerThreshold = config.capture.triggerThreshold;
    capture_.durationMs = config.capture.durationMs;
    /* Capture-window markers are session-local: they encode plot-relative
     * timestamps anchored to the in-memory plotTimeOrigin_ at the moment
     * the capture finished. After app restart (or any scenario reload)
     * plotTimeOrigin_ has shifted, so old markers would land at random X
     * coordinates in the new axis -- the "phantom markers" the operator
     * reported. Always start a fresh session with no markers; the scenario
     * loader still parses the JSON field for forward-compat but discards
     * the values. */
    capture_.windowsRel.clear();

    if (config.autoconnect &&
        transportKind_ != TransportKind::Tcp &&
        transportKind_ != TransportKind::WifiAp) {
        connectSelectedTransport();
        saveScenarioNow("connect");
    } else if (config.autoconnect) {
        discoverNextScanAt_ = 0.0;
        wlanNextScanAt_ = 0.0;
        if (transportKind_ == TransportKind::Tcp) {
            discoverStatus_ = "waiting for discovery before TCP connect";
        } else if (transportKind_ == TransportKind::WifiAp) {
            wlanStatus_ = "waiting for Wi-Fi scan before AP connect";
        }
    }
}

void MainUi::draw()
{
    updateIo();

    {
        ImGuiIO& io = ImGui::GetIO();
        if (io.KeyCtrl) {
            if (ImGui::IsKeyPressed(ImGuiKey_Equal, false) ||
                ImGui::IsKeyPressed(ImGuiKey_KeypadAdd, false)) {
                uiScale_ = std::min(2.0f, uiScale_ + 0.1f);
            }
            if (ImGui::IsKeyPressed(ImGuiKey_Minus, false) ||
                ImGui::IsKeyPressed(ImGuiKey_KeypadSubtract, false)) {
                uiScale_ = std::max(0.6f, uiScale_ - 0.1f);
            }
            if (ImGui::IsKeyPressed(ImGuiKey_0, false)) {
                uiScale_ = 1.0f;
            }
        }
        io.FontGlobalScale = uiScale_;
    }

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->Pos);
    ImGui::SetNextWindowSize(vp->Size);
    ImGui::SetNextWindowViewport(vp->ID);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    const ImGuiWindowFlags rootFlags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
        ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
        // The root host itself must NOT accept docking; the DockSpace inside
        // it is what receives docked tab windows.
        ImGuiWindowFlags_NoDocking;
    ImGui::Begin("##root", nullptr, rootFlags);
    ImGui::PopStyleVar(2);

    drawTitleBar(40.0f);

    const float statusH = 28.0f;
    const float contentH = ImGui::GetContentRegionAvail().y - statusH;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::BeginChild("##content", ImVec2(0, contentH),
                      ImGuiChildFlags_None);

    // Dockspace fills the entire content region. The dockspace's own tab bar
    // (drawn by ImGui) shows the tabs and supports drag-out into a native
    // OS window. Reopening a closed tab is done from the "View" popup in the
    // title bar.
    const ImGuiID dockspaceId = ImGui::GetID("DriveScopeDock");
    ImGui::DockSpace(dockspaceId, ImVec2(0.0f, 0.0f),
                     ImGuiDockNodeFlags_PassthruCentralNode);

    // Tab definition table -- name string is the ImGui window key (used by
    // dockspace + .ini layout) and the icon is overlay-drawn on the tab
    // strip after rendering. Leading spaces in the name reserve room for
    // the icon glyph at the left of the tab text.
    struct DockTabDef {
        const char* name;     // window name (must contain leading spaces!)
        const char* label;    // user-visible label without padding
        ToolIcon    icon;
        void (MainUi::*body)();
    };
    static const DockTabDef kDockTabs[kNumTabs] = {
        { "    Connection",     "Connection",     ToolIcon::TabConnection, &MainUi::drawConnection    },
        { "    CAN Monitor",    "CAN Monitor",    ToolIcon::TabMonitor,    &MainUi::drawCanMonitor    },
        { "    Variables",      "Variables",      ToolIcon::TabVariables,  &MainUi::drawVariables     },
        { "    Plots",          "Plots",          ToolIcon::TabPlots,      &MainUi::drawPlots         },
        { "    Remote Control", "Remote Control", ToolIcon::TabRemote,     &MainUi::drawRemoteControl },
        { "    Calibration",    "Calibration",    ToolIcon::TabCalibration,&MainUi::drawCalibration   },
        { "    Motor Config",   "Motor Config",   ToolIcon::TabUpdates,    &MainUi::drawMotorConfig   },  // penultimate
        { "    Updates",        "Updates",        ToolIcon::TabUpdates,    &MainUi::drawDeviceUpdates },
        { "    Load Stand",     "Load Stand",     ToolIcon::TabPlots,      &MainUi::drawLoadStand     },
    };

    if (firstDockLayout_) {
        // Build the initial layout once: all tabs stacked into a single
        // dock node (so they look like a tab bar). User can drag any one
        // out from there. ImGui persists subsequent layouts in imgui.ini.
        ImGuiDockNode* node = ImGui::DockBuilderGetNode(dockspaceId);
        const bool isFresh = (node == nullptr || node->IsLeafNode() == false ||
                              node->TabBar == nullptr);
        if (isFresh) {
            ImGui::DockBuilderRemoveNode(dockspaceId);
            ImGui::DockBuilderAddNode(dockspaceId,
                static_cast<ImGuiDockNodeFlags>(ImGuiDockNodeFlags_DockSpace) |
                ImGuiDockNodeFlags_PassthruCentralNode);
            ImGui::DockBuilderSetNodeSize(dockspaceId, ImGui::GetMainViewport()->Size);
            for (int i = 0; i < kNumTabs; ++i) {
                ImGui::DockBuilderDockWindow(kDockTabs[i].name, dockspaceId);
            }
            ImGui::DockBuilderFinish(dockspaceId);
        }
        firstDockLayout_ = false;
    }

    ImGui::EndChild();
    ImGui::PopStyleVar();

    drawStatusBar(statusH);

    ImGui::End();

    // ---- Tab windows (each one dockable + tear-off-able) -----------------
    // Closing a window flips tabOpen_[i] to false; the View popup in the
    // title bar can reopen it.
    if (focusConnectionStartupFrames_ > 0) {
        tabOpen_[0] = true;
        activeTab_ = 0;
    }
    for (int i = 0; i < kNumTabs; ++i) {
        if (!tabOpen_[i]) continue;
        if (tabFocusReq_[i]) {
            ImGui::SetNextWindowFocus();
            tabFocusReq_[i] = false;
        }
        if (ImGui::Begin(kDockTabs[i].name, &tabOpen_[i], ImGuiWindowFlags_NoCollapse)) {
            // Shared device take/release-control strip on every tab -- one
            // common control, one source of truth (see drawDeviceControlBar).
            drawDeviceControlBar();
            (this->*kDockTabs[i].body)();
        }
        ImGui::End();
    }
    if (focusConnectionStartupFrames_ > 0) {
        ImGuiWindow* connectionWindow = ImGui::FindWindowByName(kDockTabs[0].name);
        if (connectionWindow != nullptr) {
            if (ImGuiDockNode* node = connectionWindow->DockNode) {
                node->SelectedTabId = connectionWindow->TabId;
                node->VisibleWindow = connectionWindow;
                if (node->TabBar != nullptr) {
                    node->TabBar->SelectedTabId = connectionWindow->TabId;
                    node->TabBar->NextSelectedTabId = connectionWindow->TabId;
                }
            }
            ImGui::FocusWindow(connectionWindow);
            if (connectionWindow->DockTabIsVisible) {
                focusConnectionStartupFrames_ = 0;
            } else {
                --focusConnectionStartupFrames_;
            }
        } else {
            --focusConnectionStartupFrames_;
        }
    }

    // ---- Overlay vector icons on each tab ---------------------------------
    // The native dockspace tab bar is text-only; we re-add the original
    // ToolIcon glyphs by asking each docked window for its own tab-item rect
    // (set by ImGui every frame when the window is shown as a tab). Icons
    // are drawn into the per-viewport foreground draw list so they survive
    // pop-out into a separate native window.
    {
        const ImU32 iconActive = IM_COL32(0xE6, 0xE6, 0xE6, 0xFF);
        const ImU32 iconIdle   = IM_COL32(0xA0, 0xA0, 0xA8, 0xFF);
        for (int i = 0; i < kNumTabs; ++i) {
            ImGuiWindow* w = ImGui::FindWindowByName(kDockTabs[i].name);
            if (w == nullptr) continue;
            // Skip windows that aren't currently visible as a tab. DockIsActive
            // means "docked and the dock node is being rendered"; combined with
            // a non-empty DockTabItemRect this guarantees we have a real tab
            // strip slot to draw into. Untouched DockTabItemRect from prior
            // frames may persist with stale (0,0) coords -- width filter kills
            // those.
            if (!w->DockIsActive) continue;
            const ImRect& r = w->DockTabItemRect;
            const float tabW = r.Max.x - r.Min.x;
            const float tabH = r.Max.y - r.Min.y;
            if (tabW <= 4.0f || tabH <= 4.0f) continue;
            ImGuiViewport* vp = w->Viewport ? w->Viewport : ImGui::GetMainViewport();
            ImDrawList* dl = ImGui::GetForegroundDrawList(vp);
            const float iconSize = 14.0f;
            const float iy = (r.Min.y + r.Max.y - iconSize) * 0.5f;
            const ImVec2 iMin(r.Min.x + 8.0f, iy);
            const ImVec2 iMax(iMin.x + iconSize, iy + iconSize);
            const bool isSelected = w->DockTabIsVisible;
            drawToolIcon(dl, kDockTabs[i].icon, iMin, iMax,
                         isSelected ? iconActive : iconIdle);
        }
    }

    maybeSaveScenario();
}

void MainUi::drawTitleBar(float height)
{
#ifdef _WIN32
    GLFWwindow* w = static_cast<GLFWwindow*>(nativeWindow_);
#endif

    const ImVec2 winPos = ImGui::GetWindowPos();
    const float winW = ImGui::GetWindowSize().x;
    ImDrawList* dl = ImGui::GetWindowDrawList();

    const ImU32 barCol = IM_COL32(0x24, 0x24, 0x27, 0xFF);
    const ImU32 underline = IM_COL32(0x12, 0x12, 0x14, 0xFF);
    dl->AddRectFilled(winPos, ImVec2(winPos.x + winW, winPos.y + height), barCol);
    dl->AddLine(ImVec2(winPos.x, winPos.y + height - 0.5f),
                ImVec2(winPos.x + winW, winPos.y + height - 0.5f), underline, 1.0f);

    const float btnW = 46.0f;
    const float viewBtnW = 86.0f;
    const float btnsX = winPos.x + winW - btnW * 3.0f;
    const float viewBtnX = btnsX - viewBtnW - 6.0f;

    ImGui::SetCursorScreenPos(winPos);
    // Drag area stops short of the View menu so clicking the menu doesn't
    // start a window drag.
    ImGui::InvisibleButton("##titlebar-drag", ImVec2(viewBtnX - winPos.x, height));
    const bool dragHovered = ImGui::IsItemHovered();
    const bool dragClicked = ImGui::IsItemActivated();
    const bool dragDoubleClicked = ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0);

#ifdef _WIN32
    if (dragDoubleClicked && w) {
        if (glfwGetWindowAttrib(w, GLFW_MAXIMIZED)) glfwRestoreWindow(w);
        else glfwMaximizeWindow(w);
    } else if (dragClicked && !dragDoubleClicked && w) {
        HWND hwnd = glfwGetWin32Window(w);
        if (hwnd) {
            ReleaseCapture();
            // SendMessage(WM_NCLBUTTONDOWN, HTCAPTION) enters Windows' modal
            // window-move loop and only returns when the user releases the
            // mouse -- but during that loop the OS posts WM_NCLBUTTONUP
            // (non-client), NOT WM_LBUTTONUP, so GLFW/ImGui never see the
            // release. Without intervention, ImGui still thinks LMB is
            // pressed on the next frame; the next *real* click is parsed as
            // "press-while-already-down" (no IsItemActivated), and the user
            // has to click 2-3 times before the title bar reacts again.
            // Inject a synthetic release into ImGui's input queue right
            // after the modal loop ends so the very next click works
            // cleanly on the first attempt.
            SendMessageW(hwnd, WM_NCLBUTTONDOWN, HTCAPTION, 0);
            ImGui::GetIO().AddMouseButtonEvent(ImGuiMouseButton_Left, false);
        }
    }
#else
    (void)dragHovered; (void)dragClicked; (void)dragDoubleClicked;
#endif

    // Logo: blue ring with sine wave
    const float lcx = winPos.x + 18.0f;
    const float lcy = winPos.y + height * 0.5f;
    const float lr  = 11.0f;
    dl->AddCircleFilled(ImVec2(lcx, lcy), lr, IM_COL32(0x00, 0x7A, 0xCC, 0xFF));
    dl->AddCircle(ImVec2(lcx, lcy), lr, IM_COL32(0x12, 0x32, 0x4A, 0xFF), 0, 1.5f);
    {
        const int N = 28;
        ImVec2 prev;
        for (int i = 0; i <= N; ++i) {
            const float xn = -1.0f + 2.0f * (static_cast<float>(i) / N);
            const float wx = lcx + xn * (lr * 0.85f);
            const float wy = lcy + std::sin(xn * 3.14159f * 2.0f) * (lr * 0.45f);
            ImVec2 cur(wx, wy);
            if (i > 0) dl->AddLine(prev, cur, IM_COL32(0xFF, 0xFF, 0xFF, 0xE0), 1.6f);
            prev = cur;
        }
        dl->AddLine(ImVec2(lcx - lr * 0.85f, lcy),
                    ImVec2(lcx + lr * 0.85f, lcy),
                    IM_COL32(0xFF, 0xFF, 0xFF, 0x40), 1.0f);
    }

    // App name + subtitle
    const float textY = winPos.y + (height - ImGui::GetFontSize()) * 0.5f;
    dl->AddText(ImVec2(winPos.x + 36.0f, textY),
                IM_COL32(0xE6, 0xE6, 0xE6, 0xFF), "DriveScope");
    const ImVec2 nameSize = ImGui::CalcTextSize("DriveScope");
    char subtitleBuf[160];
    std::snprintf(subtitleBuf, sizeof(subtitleBuf),
                  "CAN/CAN-FD scope for STM32 + SOM   Ctrl+/- zoom %.0f%%",
                  static_cast<double>(uiScale_) * 100.0);
    dl->AddText(ImVec2(winPos.x + 36.0f + nameSize.x + 10.0f, textY),
                IM_COL32(0x9A, 0x9A, 0xA0, 0xFF), subtitleBuf);

    // Window control buttons (min, max, close) -- rendered as geometric shapes for crisp alignment.
    enum class TitleIcon { Min, Max, Restore, Close };
    auto titleBtn = [&](TitleIcon icon, float x, ImU32 hoverCol, bool danger) -> bool {
        const ImVec2 p0(x, winPos.y);
        const ImVec2 p1(x + btnW, winPos.y + height);
        const bool hov = ImGui::IsMouseHoveringRect(p0, p1);
        const bool clicked = hov && ImGui::IsMouseReleased(0);
        if (hov) dl->AddRectFilled(p0, p1, hoverCol);

        const float cx = (p0.x + p1.x) * 0.5f;
        const float cy = (p0.y + p1.y) * 0.5f;
        const float r  = 5.0f;
        const ImU32 fg = (danger && hov) ? IM_COL32(0xFF, 0xFF, 0xFF, 0xFF)
                                          : IM_COL32(0xCF, 0xCF, 0xCF, 0xFF);
        const float t = 1.4f;

        switch (icon) {
            case TitleIcon::Min:
                dl->AddLine(ImVec2(cx - r, cy + 0.5f), ImVec2(cx + r, cy + 0.5f), fg, t);
                break;
            case TitleIcon::Max:
                dl->AddRect(ImVec2(cx - r, cy - r), ImVec2(cx + r, cy + r), fg, 0.0f, 0, t);
                break;
            case TitleIcon::Restore:
                dl->AddRect(ImVec2(cx - r + 2, cy - r),     ImVec2(cx + r,     cy + r - 2), fg, 0.0f, 0, t);
                dl->AddRectFilled(ImVec2(cx - r, cy - r + 2), ImVec2(cx + r - 2, cy + r),
                                  barCol);
                dl->AddRect(ImVec2(cx - r, cy - r + 2),     ImVec2(cx + r - 2, cy + r),     fg, 0.0f, 0, t);
                break;
            case TitleIcon::Close:
                dl->AddLine(ImVec2(cx - r, cy - r), ImVec2(cx + r, cy + r), fg, t);
                dl->AddLine(ImVec2(cx - r, cy + r), ImVec2(cx + r, cy - r), fg, t);
                break;
        }
        return clicked;
    };

#ifdef _WIN32
    const bool maximized = w ? (glfwGetWindowAttrib(w, GLFW_MAXIMIZED) != 0) : false;
#else
    const bool maximized = false;
#endif
    const ImU32 hoverNeutral = IM_COL32(0xFF, 0xFF, 0xFF, 0x18);
    const ImU32 hoverClose   = IM_COL32(0xE8, 0x11, 0x23, 0xFF);

    if (titleBtn(TitleIcon::Min,                                btnsX,               hoverNeutral, false)) {
#ifdef _WIN32
        if (w) glfwIconifyWindow(w);
#endif
    }
    if (titleBtn(maximized ? TitleIcon::Restore : TitleIcon::Max,
                                                                btnsX + btnW,        hoverNeutral, false)) {
#ifdef _WIN32
        if (w) {
            if (maximized) glfwRestoreWindow(w);
            else           glfwMaximizeWindow(w);
        }
#endif
    }
    if (titleBtn(TitleIcon::Close,                              btnsX + btnW * 2,    hoverClose,   true)) {
#ifdef _WIN32
        if (w) glfwSetWindowShouldClose(w, GLFW_TRUE);
#endif
    }

    // ---- View menu (dockable-tab visibility) -----------------------------
    // Sits just left of Min/Max/Close. Lets the user reopen a closed tab or
    // reset the dock layout. The tabs themselves are docked windows whose
    // tab bar is rendered by ImGui's dockspace, so we don't need our own.
    ImGui::SetCursorScreenPos(ImVec2(viewBtnX, winPos.y));
    ImGui::InvisibleButton("##title-view", ImVec2(viewBtnW, height));
    const bool viewHov = ImGui::IsItemHovered();
    const bool viewClicked = ImGui::IsItemClicked();
    if (viewHov) {
        dl->AddRectFilled(ImVec2(viewBtnX, winPos.y),
                          ImVec2(viewBtnX + viewBtnW, winPos.y + height),
                          hoverNeutral);
    }
    const ImU32 viewFg = IM_COL32(0xCF, 0xCF, 0xCF, 0xFF);
    dl->AddText(ImVec2(viewBtnX + 14.0f, textY), viewFg, "View");
    {
        const float caretX = viewBtnX + viewBtnW - 16.0f;
        const float caretY = winPos.y + height * 0.5f;
        dl->AddTriangleFilled(ImVec2(caretX - 4, caretY - 2),
                              ImVec2(caretX + 4, caretY - 2),
                              ImVec2(caretX,     caretY + 3), viewFg);
    }
    if (viewClicked) ImGui::OpenPopup("##view-popup");
    if (ImGui::BeginPopup("##view-popup")) {
        static const char* const kTabNames[kNumTabs] = {
            "Connection", "CAN Monitor", "Variables", "Plots", "Remote Control",
            "Calibration", "Motor Config", "Updates", "Load Stand",
        };
        for (int i = 0; i < kNumTabs; ++i) {
            if (ImGui::MenuItem(kTabNames[i], nullptr, tabOpen_[i])) {
                tabOpen_[i] = !tabOpen_[i];
                if (tabOpen_[i]) tabFocusReq_[i] = true;
            }
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Show all tabs")) {
            for (int i = 0; i < kNumTabs; ++i) tabOpen_[i] = true;
        }
        if (ImGui::MenuItem("Reset window layout")) {
            for (int i = 0; i < kNumTabs; ++i) tabOpen_[i] = true;
            firstDockLayout_ = true;
        }
        ImGui::EndPopup();
    }
}

void MainUi::drawStatusBar(float height)
{
    const double tNow = clock_.nowSeconds();
    while (!rateBucket_.empty() && (tNow - rateBucket_.front().first) > 1.0) {
        rateBucket_.pop_front();
    }
    while (!transportBucket_.empty() && (tNow - transportBucket_.front().first) > 1.0) {
        transportBucket_.pop_front();
    }
    uint64_t bitsLastSec = 0;
    for (const auto& kv : rateBucket_) bitsLastSec += kv.second;
    uint64_t transportBytesLastSec = 0;
    for (const auto& kv : transportBucket_) transportBytesLastSec += kv.second;
    const size_t fps = rateBucket_.size();
    const int rate = currentNominalBitrate();
    const double rawLoad = (rate > 0)
        ? std::min(999.0, static_cast<double>(bitsLastSec) * 100.0 / static_cast<double>(rate))
        : 0.0;
    const float rawKbps = static_cast<float>(transportBytesLastSec) / 1024.0f;

    // EMA smoothing so the displayed numbers don't flicker every frame.
    constexpr float alpha = 0.10f;
    canLoadEma_ = canLoadEma_ * (1.0f - alpha) + static_cast<float>(rawLoad) * alpha;
    netKbpsEma_ = netKbpsEma_ * (1.0f - alpha) + rawKbps * alpha;
    fpsEma_     = fpsEma_     * (1.0f - alpha) + static_cast<float>(fps) * alpha;
    const double load = canLoadEma_;

    const bool connected = isConnected();
    const ImU32 barCol      = connected ? IM_COL32(0x00, 0x7A, 0xCC, 0xFF)
                                        : IM_COL32(0x60, 0x60, 0x60, 0xFF);
    const ImU32 textCol     = IM_COL32(0xFF, 0xFF, 0xFF, 0xFF);
    const ImU32 dimTextCol  = IM_COL32(0xE0, 0xE0, 0xE0, 0xD0);

    const ImVec2 winPos = ImGui::GetWindowPos();
    const ImVec2 winSize = ImGui::GetWindowSize();
    const ImVec2 p0(winPos.x, winPos.y + winSize.y - height);
    const ImVec2 p1(winPos.x + winSize.x, winPos.y + winSize.y);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, p1, barCol);

    const float cy = p0.y + height * 0.5f;
    const float textY = p0.y + (height - ImGui::GetFontSize()) * 0.5f;

    // Connection indicator dot.
    dl->AddCircleFilled(ImVec2(p0.x + 14.0f, cy), 4.5f,
                        connected ? IM_COL32(0x9C, 0xFF, 0x9C, 0xFF)
                                  : IM_COL32(0xFF, 0xC8, 0xC8, 0xFF));

    const float quickBtnX = p0.x + 26.0f;
    const float quickBtnY = p0.y + 3.0f;
    const float quickBtnW = connected ? 96.0f : 82.0f;
    ImGui::SetCursorScreenPos(ImVec2(quickBtnX, quickBtnY));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10.0f, 2.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 3.0f);
    ImGui::PushStyleColor(ImGuiCol_Button,        connected ? ImVec4(0.45f, 0.20f, 0.20f, 0.88f)
                                                            : ImVec4(0.10f, 0.34f, 0.54f, 0.92f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, connected ? ImVec4(0.58f, 0.25f, 0.25f, 1.00f)
                                                            : ImVec4(0.12f, 0.43f, 0.68f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  connected ? ImVec4(0.70f, 0.30f, 0.30f, 1.00f)
                                                            : ImVec4(0.08f, 0.50f, 0.80f, 1.00f));
    if (ImGui::Button(connected ? "Disconnect##tray-connect" : "Connect##tray-connect",
                      ImVec2(quickBtnW, height - 6.0f))) {
        if (connected) disconnectTransport();
        else           connectSelectedTransport();
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Quick %s using the current Connection tab settings.",
                          connected ? "disconnect" : "connect");
    }
    ImGui::PopStyleColor(3);
    ImGui::PopStyleVar(2);

    auto rateLabel = [](int r) -> std::string {
        if (r >= 1000000) return std::to_string(r / 1000000) + " Mbit/s";
        if (r >= 1000)    return std::to_string(r / 1000) + " kbit/s";
        return std::to_string(r) + " bit/s";
    };

    // Pick monospace font (loaded second by loadDriveScopeFont) for stable digit columns.
    ImFont* monoFont = ImGui::GetFont();
    {
        const ImVector<ImFont*>& fonts = ImGui::GetIO().Fonts->Fonts;
        if (fonts.Size > 1 && fonts[1] != nullptr) monoFont = fonts[1];
    }
    const float monoSize = monoFont->FontSize;

    auto drawProgressBar = [&](float x, float w, double pct, const char* label) {
        const float pbH = height - 8.0f;
        const ImVec2 a(x, p0.y + 4.0f);
        const ImVec2 b(x + w, a.y + pbH);
        dl->AddRectFilled(a, b, IM_COL32(0x18, 0x18, 0x1A, 0xFF), 3.0f);
        const float fillW = w * static_cast<float>(std::min(1.0, pct / 100.0));
        const ImU32 fillCol = (pct < 60) ? IM_COL32(0x4A, 0xC9, 0x6B, 0xFF) :
                              (pct < 90) ? IM_COL32(0xE6, 0xC0, 0x4A, 0xFF) :
                                            IM_COL32(0xE0, 0x4A, 0x4A, 0xFF);
        dl->AddRectFilled(a, ImVec2(a.x + fillW, b.y), fillCol, 3.0f);
        dl->AddRect(a, b, IM_COL32(0x33, 0x33, 0x33, 0xFF), 3.0f, 0, 1.0f);
        const ImVec2 ts = monoFont->CalcTextSizeA(monoSize, FLT_MAX, 0.0f, label);
        dl->AddText(monoFont, monoSize,
                    ImVec2(a.x + (w - ts.x) * 0.5f, p0.y + (height - ts.y) * 0.5f),
                    IM_COL32(0xFF, 0xFF, 0xFF, 0xFF), label);
    };
    auto drawWorkProgressBar = [&](float x, float w, double pct, const char* label) {
        const float pbH = height - 8.0f;
        const ImVec2 a(x, p0.y + 4.0f);
        const ImVec2 b(x + w, a.y + pbH);
        dl->AddRectFilled(a, b, IM_COL32(0x18, 0x18, 0x1A, 0xFF), 3.0f);
        const float fillW = w * static_cast<float>(std::min(1.0, pct / 100.0));
        dl->AddRectFilled(a, ImVec2(a.x + fillW, b.y), IM_COL32(0x2C, 0x8D, 0xD8, 0xFF), 3.0f);
        dl->AddRect(a, b, IM_COL32(0x77, 0xB7, 0xE8, 0xA0), 3.0f, 0, 1.0f);
        const ImVec2 ts = monoFont->CalcTextSizeA(monoSize, FLT_MAX, 0.0f, label);
        dl->AddText(monoFont, monoSize,
                    ImVec2(a.x + std::max(6.0f, (w - ts.x) * 0.5f),
                           p0.y + (height - ts.y) * 0.5f),
                    IM_COL32(0xFF, 0xFF, 0xFF, 0xFF), label);
    };

    // === Left side: connection summary ===
    const char* transport =
        transportKind_ == TransportKind::Slcan  ? "SLCAN"  :
        transportKind_ == TransportKind::WifiAp ? "AP"     :
        transportKind_ == TransportKind::Manual ? "Manual" :
                                                  "STA";
    std::string portInfo;
    if (transportKind_ == TransportKind::Slcan) {
        portInfo = std::string(comPort_.data()) + " " + slcanNominal_.data() + "/" + slcanData_.data();
    } else if (transportKind_ == TransportKind::WifiAp) {
        portInfo = std::string(wifiApHost_.data()) + ":" + std::to_string(wifiApPort_);
    } else {
        portInfo = std::string(tcpHost_.data()) + ":" + std::to_string(tcpPort_);
    }
    char headerBuf[256];
    std::snprintf(headerBuf, sizeof(headerBuf), "%s  %s  %s  %s",
                  connected ? "[connected]" : "[disconnected]",
                  transport, portInfo.c_str(), rateLabel(rate).c_str());
    const float headerX = quickBtnX + quickBtnW + 12.0f;
    dl->AddText(ImVec2(headerX, textY), textCol, headerBuf);
    const float headerW = ImGui::CalcTextSize(headerBuf).x;

    // Last opened/saved file (after the connection block, dimmer).
    if (!lastFileMsg_.empty()) {
        const float fileX = headerX + headerW + 18.0f;
        dl->AddText(ImVec2(fileX, textY), dimTextCol, lastFileMsg_.c_str());
    }

    // === Transport throughput bar (left of CAN load) ===
    const float kbps = netKbpsEma_;
    const float netCapacityKBps = (transportKind_ == TransportKind::Slcan)
        ? std::max(1.0f, static_cast<float>(slcanBaud_) / 10240.0f)
        : 1024.0f;
    const double netPct = std::min(100.0, 100.0 * static_cast<double>(kbps) / netCapacityKBps);
    char netLabel[64];
    std::snprintf(netLabel, sizeof(netLabel), "%s %6.1f kB/s",
                  (transportKind_ == TransportKind::Slcan)  ? "UART"   :
                  (transportKind_ == TransportKind::WifiAp) ? "AP"     :
                  (transportKind_ == TransportKind::Manual) ? "Manual" : "STA", kbps);

    // === CAN bus load bar (right side) ===
    char canLabel[64];
    std::snprintf(canLabel, sizeof(canLabel), "CAN %5.1f%%", load);

    char tailBuf[128];
    std::snprintf(tailBuf, sizeof(tailBuf), "%4.0f fps   total %7zu",
                  fpsEma_, totalFrames_);
    const ImVec2 tailSize = monoFont->CalcTextSizeA(monoSize, FLT_MAX, 0.0f, tailBuf);
    const float tailW = tailSize.x;

    // Right-anchored layout:  [tailBuf] [CAN bar] [Net bar]
    const float pbW = 140.0f;
    const float gap = 14.0f;
    const float rightX = p1.x - 14.0f;
    const float tailX  = rightX - tailW;
    const float canBarX = tailX - gap - pbW;
    const float netBarX = canBarX - gap - pbW;

    int deployPct = 0;
    std::string deployText;
    if (deployProgressSnapshot(mainDeployProgress_, deployPct, deployText)) {
        // Bar width = 2*pbW + gap = 294 px в‰€ 42 mono chars at the status-bar
        // font. The "UPDATE 100%  " prefix takes 13 chars, leaving ~29 chars
        // for the user-supplied text. Truncate harder so the label never
        // overflows into the right-side fps/total counters.
        constexpr size_t kMaxDeployText = 29;
        if (deployText.size() > kMaxDeployText) {
            deployText = deployText.substr(0, kMaxDeployText - 3) + "...";
        }
        char deployLabel[96];
        std::snprintf(deployLabel, sizeof(deployLabel), "UPDATE %3d%%  %s",
                      deployPct, deployText.c_str());
        drawWorkProgressBar(netBarX, pbW * 2.0f + gap, deployPct, deployLabel);
    } else {
        drawProgressBar(netBarX, pbW, netPct, netLabel);
        drawProgressBar(canBarX, pbW, load, canLabel);
    }
    dl->AddText(monoFont, monoSize,
                ImVec2(tailX, p0.y + (height - monoSize) * 0.5f),
                dimTextCol, tailBuf);
}

ICanTransport* MainUi::selectedTransport()
{
    return transportKind_ == TransportKind::Slcan
        ? static_cast<ICanTransport*>(&slcan_)
        : static_cast<ICanTransport*>(&tcp_);
}

bool MainUi::isConnected() const
{
    return activeTransport_ != nullptr && activeTransport_->isOpen();
}

bool MainUi::sendFrameThreadSafe(const CanFrame& frame)
{
    bool ok;
    {
        std::lock_guard<std::mutex> lock(transportMutex_);
        ok = activeTransport_ != nullptr &&
             activeTransport_->isOpen() &&
             activeTransport_->send(frame);
    }
    // Passive sniffer tap (TX), OUTSIDE transportMutex_ so JSONL flush/decode
    // never holds the transport lock. Record only on a successful send; decode
    // only when a sink is open, so the default (off) path costs nothing and
    // behavior is unchanged.
    if (ok && sniffer_.isActive()) {
        const char* tl = transportKind_ == TransportKind::Slcan  ? "slcan"
                       : transportKind_ == TransportKind::WifiAp ? "wifi_ap"
                       : transportKind_ == TransportKind::Manual ? "manual" : "tcp";
        sniffer_.record(CanSniffer::Dir::Tx, tl, frame, decodeCanFrame(frame));
    }
    return ok;
}

void MainUi::resetTransportRuntimeState()
{
    pending_.clear();
    pendingSentAt_.clear();
    nextSeq_ = 1;
    pollCursor_ = 0;

    streamAddPending_.clear();
    motorStreamSlotToWatch_.fill(-1);
    lastStreamHeartbeatAt_ = 0.0;
    for (WatchVar& w : watches_) {
        w.streamSlotId = 0xFF;
        w.streamSubscribed = false;
        w.streamPending = false;
        w.streamQuotaLimited = false;
        w.streamSentAt = 0.0;
        w.streamLastValueAt = 0.0;
    }
}

void MainUi::disconnectTransport()
{
    std::lock_guard<std::mutex> lock(transportMutex_);
    if (activeTransport_ != nullptr) activeTransport_->close();
    activeTransport_ = nullptr;
    resetTransportRuntimeState();
}

bool MainUi::selectDiscoveredTcpTarget(bool isAp, bool markDirty)
{
    int chosen = -1;
    auto matchesMode = [&](const DiscoveredDevice& d) {
        const bool apIp = d.ip.rfind("192.168.1.", 0) == 0;
        return isAp ? apIp : !apIp;
    };

    if (discoveredSelected_ >= 0 &&
        discoveredSelected_ < static_cast<int>(discovered_.size()) &&
        matchesMode(discovered_[discoveredSelected_])) {
        chosen = discoveredSelected_;
    } else {
        for (size_t i = 0; i < discovered_.size(); ++i) {
            if (matchesMode(discovered_[i])) {
                chosen = static_cast<int>(i);
                break;
            }
        }
    }

    if (chosen < 0) return false;

    discoveredSelected_ = chosen;
    const auto& d = discovered_[chosen];
    if (isAp) {
        std::snprintf(wifiApSsid_.data(), wifiApSsid_.size(), "%s", d.name.c_str());
        std::snprintf(wifiApHost_.data(), wifiApHost_.size(), "%s", d.ip.c_str());
        wifiApPort_ = d.tcpPort > 0 ? d.tcpPort : 45333;
    } else {
        std::snprintf(tcpHost_.data(), tcpHost_.size(), "%s", d.ip.c_str());
        tcpPort_ = d.tcpPort > 0 ? d.tcpPort : 45333;
    }
    if (markDirty) markScenarioDirty("connection changed");
    return true;
}

void MainUi::connectSelectedTransport()
{
    disconnectTransport();
    if (transportKind_ == TransportKind::Slcan) {
        slcan_.portName = comPort_.data();
        slcan_.baud = slcanBaud_;
        slcan_.nominalCommand = slcanNominal_.data();
        slcan_.dataCommand = slcanData_.data();
        slcan_.silentMode = slcanSilent_;
        activeTransport_ = selectedTransport();
        activeTransport_->open();
        return;
    }

    if (transportKind_ == TransportKind::WifiAp) {
#ifdef _WIN32
        // The single Connect button now handles both stages of the AP path:
        // (1) WlanConnect to the selected Kitchen_Machine_* SSID (skipped
        //     when the laptop is already on it), (2) open TCP to 192.168.1.1.
        // Refuses to do anything when there's nothing to connect to -- that's
        // the "do nothing if Wi-Fi not found" the user asked for.
        if (wlanScan_.empty() ||
            wlanSelected_ < 0 ||
            wlanSelected_ >= static_cast<int>(wlanScan_.size())) {
            wlanStatus_ = "no Kitchen_Machine_* hotspot visible / selected";
            return;
        }
        const std::string ssid = wlanScan_[wlanSelected_].ssid;
        const std::string current = drivescope::wlan::currentSsid();
        if (current != ssid) {
            // Kick async WlanConnect; updateIo() polls currentSsid() and on
            // success opens TCP automatically (wlanAutoOpenTcpPending_).
            connectWlanAsync(ssid, /*password=*/"bork2025");
            return;
        }
#endif
        // Already on the device's hotspot -- just open TCP.
        std::snprintf(wifiApHost_.data(), wifiApHost_.size(), "%s", "192.168.1.1");
        wifiApPort_ = 45333;
        tcp_.host = wifiApHost_.data();
        tcp_.port = wifiApPort_;
        activeTransport_ = selectedTransport();
        activeTransport_->open();
        return;
    }

    if (transportKind_ == TransportKind::Tcp) {
        if (!selectDiscoveredTcpTarget(/*isAp=*/false, /*markDirty=*/true)) {
            discoverStatus_ = "no STA device discovered - TCP connect skipped";
            return;
        }
    }

    // STA uses the discovered target above; Manual uses whatever host/port is in the form.
    tcp_.host = tcpHost_.data();
    tcp_.port = tcpPort_;
    activeTransport_ = selectedTransport();
    activeTransport_->open();
}

#ifdef _WIN32
namespace {

// Send "DDV2_DISCOVER" to every local subnet broadcast address + 255.255.255.255
// from a single UDP socket, then collect replies for `wait_ms` milliseconds.
// Returns the parsed (name, ip, mode, tcp_port) tuples. The "РјР°РєСЃРёРјР°Р»СЊРЅРѕ
// РґСѓР±РѕРІС‹Р№" approach: don't trust default routing -- broadcast on every IPv4
// interface we own. Quietly skips any interface we can't bind to.
struct RawReply { std::string ip; std::string body; };

// Per-interface UDP probe. We open ONE socket per local IPv4 unicast address
// and `bind()` it to that NIC's IP -- that pins the egress to this exact
// interface, sidestepping a VPN that may have grabbed the default route at
// metric 0 (commonly seen with WireGuard / Tailscale). For each socket we
// send DDV2_DISCOVER to (a) the interface's directed subnet broadcast and
// (b) the limited broadcast 255.255.255.255, then drain replies. This is the
// "РјР°РєСЃРёРјР°Р»СЊРЅРѕ РґСѓР±РѕРІС‹Р№" approach: no reliance on OS routing decisions, just
// an explicit broadcast on every cable/radio we own.
static std::vector<RawReply> doDiscoveryScan(int wait_ms)
{
    std::vector<RawReply> out;

    // Enumerate adapters once.
    ULONG bufSz = 0;
    GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST |
                         GAA_FLAG_SKIP_DNS_SERVER, nullptr, nullptr, &bufSz);
    std::vector<unsigned char> abuf(bufSz);
    auto* aa = reinterpret_cast<IP_ADAPTER_ADDRESSES*>(abuf.data());
    if (GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST |
                             GAA_FLAG_SKIP_DNS_SERVER, nullptr, aa, &bufSz) != NO_ERROR) {
        return out;
    }

    struct ProbeSock {
        SOCKET s;
        uint32_t local_ip_be;
    };
    std::vector<ProbeSock> probes;

    for (auto* ad = aa; ad; ad = ad->Next) {
        if (ad->OperStatus != IfOperStatusUp) continue;
        if (ad->IfType == IF_TYPE_SOFTWARE_LOOPBACK) continue;
        for (auto* ua = ad->FirstUnicastAddress; ua; ua = ua->Next) {
            auto* sa = reinterpret_cast<sockaddr_in*>(ua->Address.lpSockaddr);
            if (sa->sin_family != AF_INET) continue;

            uint32_t ip_be   = sa->sin_addr.s_addr;
            uint32_t prefix  = ua->OnLinkPrefixLength >= 32 ? 32u : ua->OnLinkPrefixLength;
            uint32_t mask_be = prefix == 0 ? 0u : htonl(~((1u << (32 - prefix)) - 1));
            uint32_t bcast_be = ip_be | ~mask_be;

            // Skip 169.254/16 link-local -- wastes time, never our device.
            uint32_t ip_h = ntohl(ip_be);
            if ((ip_h & 0xFFFF0000u) == 0xA9FE0000u) continue;

            SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (s == INVALID_SOCKET) continue;

            BOOL yes = TRUE;
            setsockopt(s, SOL_SOCKET, SO_BROADCAST,
                       reinterpret_cast<const char*>(&yes), sizeof(yes));
            setsockopt(s, SOL_SOCKET, SO_REUSEADDR,
                       reinterpret_cast<const char*>(&yes), sizeof(yes));

            // Bind to *this* NIC's IP -- pins outbound to this interface.
            sockaddr_in la{};
            la.sin_family      = AF_INET;
            la.sin_addr.s_addr = ip_be;
            la.sin_port        = 0;
            if (bind(s, reinterpret_cast<sockaddr*>(&la), sizeof(la)) == SOCKET_ERROR) {
                closesocket(s);
                continue;
            }

            DWORD rcvto = 100;
            setsockopt(s, SOL_SOCKET, SO_RCVTIMEO,
                       reinterpret_cast<const char*>(&rcvto), sizeof(rcvto));

            auto sendTo = [&](uint32_t addr_be) {
                sockaddr_in dst{};
                dst.sin_family      = AF_INET;
                dst.sin_addr.s_addr = addr_be;
                dst.sin_port        = htons(17557);
                const char* msg     = "DDV2_DISCOVER";
                sendto(s, msg, 13, 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
            };

            // Three shots: subnet-directed bcast, limited bcast, and the
            // gateway IP (covers the AP case where bcast is sometimes filtered
            // by hostapd's bridge logic but unicast to .1 always works).
            if (bcast_be != ip_be && bcast_be != 0xFFFFFFFFu) sendTo(bcast_be);
            sendTo(htonl(INADDR_BROADCAST));
            // Gateway-as-unicast guess: assume .1 of the same /24 -- cheap and
            // covers the common AP gateway 192.168.1.1 even when its broadcast
            // is dropped.
            uint32_t guess_h = (ntohl(ip_be) & 0xFFFFFF00u) | 1u;
            sendTo(htonl(guess_h));

            // Some office APs/routers suppress client broadcast while normal
            // unicast TCP still works. In that case a device at e.g.
            // 192.168.0.246 is reachable manually but never answers the
            // broadcast probe above. For small on-link IPv4 networks, sweep
            // the subnet with unicast DDV2_DISCOVER packets. Keep it bounded
            // so a broad VPN /16 adapter cannot turn auto-scan into a packet
            // storm.
            if (prefix >= 24u && prefix <= 30u) {
                const uint32_t local_h = ntohl(ip_be);
                const uint32_t mask_h = ntohl(mask_be);
                const uint32_t network_h = local_h & mask_h;
                const uint32_t bcast_h = ntohl(bcast_be);
                const uint32_t first_h = network_h + 1u;
                const uint32_t last_h = bcast_h > 0u ? (bcast_h - 1u) : bcast_h;
                uint32_t sent = 0;
                for (uint32_t host_h = first_h;
                     host_h <= last_h && sent < 512u;
                     ++host_h) {
                    if (host_h == local_h) continue;
                    sendTo(htonl(host_h));
                    ++sent;
                }
            }

            probes.push_back({s, ip_be});
        }
    }

    // Drain replies from all probe sockets up to wait_ms total.
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
    while (std::chrono::steady_clock::now() < deadline && !probes.empty()) {
        // Use select() so we don't busy-spin per socket.
        fd_set rfds;
        FD_ZERO(&rfds);
        SOCKET maxs = 0;
        for (auto& p : probes) {
            FD_SET(p.s, &rfds);
            if (p.s > maxs) maxs = p.s;
        }
        timeval tv{0, 100 * 1000}; // 100 ms slice
        int rv = select(static_cast<int>(maxs + 1), &rfds, nullptr, nullptr, &tv);
        if (rv <= 0) continue;

        for (auto& p : probes) {
            if (!FD_ISSET(p.s, &rfds)) continue;
            char rx[512];
            sockaddr_in src{};
            int slen = sizeof(src);
            int n = recvfrom(p.s, rx, sizeof(rx) - 1, 0,
                             reinterpret_cast<sockaddr*>(&src), &slen);
            if (n <= 0) continue;
            rx[n] = 0;
            if (std::strstr(rx, "discovery_version") == nullptr) continue;
            char ipStr[INET_ADDRSTRLEN] = {0};
            inet_ntop(AF_INET, &src.sin_addr, ipStr, sizeof(ipStr));
            out.push_back({ipStr, rx});
        }
    }

    for (auto& p : probes) closesocket(p.s);
    return out;
}

// Tiny single-key JSON string extractor for ASCII-only fields. Good enough for
// our flat one-line discovery payload -- pulling in nlohmann/json just for this
// would be overkill.
static std::string extractJsonStr(const std::string& body, const char* key)
{
    std::string pat = std::string("\"") + key + "\":\"";
    auto p = body.find(pat);
    if (p == std::string::npos) return {};
    p += pat.size();
    auto e = body.find('"', p);
    if (e == std::string::npos) return {};
    return body.substr(p, e - p);
}

static int extractJsonInt(const std::string& body, const char* key, int dflt)
{
    std::string pat = std::string("\"") + key + "\":";
    auto p = body.find(pat);
    if (p == std::string::npos) return dflt;
    p += pat.size();
    int val = 0;
    int sign = 1;
    if (p < body.size() && body[p] == '-') { sign = -1; ++p; }
    bool any = false;
    while (p < body.size() && body[p] >= '0' && body[p] <= '9') {
        val = val * 10 + (body[p] - '0');
        ++p;
        any = true;
    }
    return any ? sign * val : dflt;
}

} // namespace
#endif // _WIN32

void MainUi::scanDiscoveryAsync()
{
#ifdef _WIN32
    if (discoverScanning_) return;
    discoverScanning_ = true;
    discoverStatus_ = "scanning...";
    // Run on a background thread so the UI stays responsive during the wait.
    std::thread([this]() {
        // WSAStartup is idempotent; the SLCAN/TCP transports already do it
        // elsewhere, but we redo it to be standalone.
        WSADATA wsa{};
        WSAStartup(MAKEWORD(2, 2), &wsa);
        auto raw = doDiscoveryScan(/*wait_ms=*/1500);

        std::vector<DiscoveredDevice> parsed;
        for (const auto& r : raw) {
            DiscoveredDevice d;
            d.name = extractJsonStr(r.body, "name");
            d.ip   = extractJsonStr(r.body, "ip");
            d.mode = extractJsonStr(r.body, "mode");
            d.tcpPort = extractJsonInt(r.body, "tcp_port", 45333);
            // Prefer the IP the device claimed; fall back to source IP if it
            // shipped an empty/0 IP for some reason.
            if (d.ip.empty()) d.ip = r.ip;
            parsed.push_back(std::move(d));
        }

        // Drop duplicates on (name, ip).
        std::sort(parsed.begin(), parsed.end(), [](const auto& a, const auto& b) {
            if (a.name != b.name) return a.name < b.name;
            return a.ip < b.ip;
        });
        parsed.erase(std::unique(parsed.begin(), parsed.end(),
                                 [](const auto& a, const auto& b) {
                                     return a.name == b.name && a.ip == b.ip;
                                 }),
                     parsed.end());

        discovered_ = std::move(parsed);
        if (discovered_.empty()) {
            discoverStatus_ = "no devices found - check broadcast / try AP";
        } else {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "found %zu device(s)", discovered_.size());
            discoverStatus_ = buf;
        }
        discoverScanning_ = false;
    }).detach();
#else
    discoverStatus_ = "discovery only on Windows for now";
#endif
}

void MainUi::updateIo()
{
    const bool holdApTransport =
        mainDeployRunning_ && preserveTransportDuringDeploy_;

    if (!holdApTransport && activeTransport_ != nullptr && activeTransport_->isOpen()) {
        for (int i = 0; i < 1000; ++i) {
            CanFrame frame;
            bool got = false;
            {
                std::lock_guard<std::mutex> lock(transportMutex_);
                if (activeTransport_ != nullptr && activeTransport_->isOpen()) {
                    got = activeTransport_->poll(frame);
                }
            }
            if (!got) break;
            handleFrame(frame);
        }
    }

    const bool canFlashBusy = bootFlasher_.snapshot().busy || holdApTransport;
    if (!canFlashBusy) {
        const double pollNow = clock_.nowSeconds();
        // Phase C 2026-05-27: subscription-based graphs. For motor watches
        // streamTick sends STREAM_ADD on enable / STREAM_REMOVE on disable
        // and emits STREAM_HEARTBEAT 1 Hz; STREAM_VALUE frames land via
        // handleFrameRx → handleStreamValue. pollWatchVariables() now skips
        // motor watches that are streamSubscribed=true (no READ_MEM polling
        // on subscribed slots). RK / ESP32 keep polling for now.
        streamTick(pollNow);
        pollWatchVariables(pollNow);
        // Drain HTTP worker BEFORE maybePollMotorTheta so a fresh result
        // lands into motorBlob_ before the next auto-poll tick decides
        // whether to fire another GET.
        drainMotorBlobHttpResult(pollNow);
        maybePollMotorTheta(pollNow);
        // Drive the Triggered Capture state machine: send STATUS_REQ
        // pings while waiting for DONE, send READ_CHUNK on DONE, time
        // out stalled phases. Most frames come back via handleCapFrame.
        pollCaptureSession(pollNow);

        // Auto-ping after Go-to-Boot / Go-to-App so the Remote Control
        // indicator flips without the operator having to click Ping.
        // The 300 ms delay set in sendNodeGoTo{Boot,App} gives the MCU
        // time to NVIC_SystemReset() and bring the bootloader / app CAN
        // stack up.
        for (size_t nid = 1; nid < kNodeIdMax; ++nid) {
            NodeMode& nm = nodeMode_[nid];
            if (nm.autoPingDueAt == 0.0) continue;
            if (pollNow < nm.autoPingDueAt) continue;
            nm.autoPingDueAt = 0.0;
            sendNodePing(static_cast<uint8_t>(nid), nm.autoPingBootMode);
        }
    }

    // Pump diag-mode heartbeats while we asked for control. Cadence 500 ms,
    // mainPCB WDG window is 2 s, so we have plenty of headroom for a dropped
    // frame.
    if (!canFlashBusy) {
        maybeSendDiagHeartbeat();
    }

    // Tone / melody sequencer: each note is a separate APP_MOTOR_CMD_TONE frame
    // (firmware auto-reverts to driverMode 0 after duration_ms expires) so we
    // just have to dispatch the next note when the previous one's duration has
    // elapsed. Tempo, volume, and transpose come from the user-controllable
    // sliders on the Motor tab. Idle when activeMelodyIdx_ == -1.
    pumpMelody(ImGui::GetTime());

    // Run the deferred-disconnect armed by Remote Control after a route-
    // breaking SET_WIFI_MODE command. The 500 ms delay gives our TCP send
    // buffer time to drain to the SOM before we close.
    if (disconnectScheduledAt_ > 0.0 && ImGui::GetTime() >= disconnectScheduledAt_) {
        disconnectScheduledAt_ = 0.0;
        disconnectTransport();
    }

    // Pump service-connect future. While running, mirror the worker's
    // stage text into serviceConnectStatus_ so the UI button shows live
    // progress. On completion, do NOT open a CAN bridge: service mode
    // means app_dd was just killed, so its TCP CAN bridge on :45333 is
    // dead. Showing "[connected] AP 500 kbit/s" in the status bar would
    // be a lie -- the operator can't actually do CAN work in service
    // mode. The operator brings it back with Run App, Reset device, or a
    // manual AP start after the update, so we leave DriveScope disconnected.
    //
    // We DO close any Wi-Fi-based transport that was open before the
    // service connect (STA TCP / AP TCP / Manual TCP), because the
    // route through it is dead now. SLCAN is independent of device
    // Wi-Fi state and stays open so the operator can still send CAN
    // commands (e.g. flip the device back into STA after they're done).
    if (serviceConnectRunning_) {
        if (serviceConnectStageText_ && serviceConnectStageMutex_) {
            std::lock_guard<std::mutex> lk(*serviceConnectStageMutex_);
            if (!serviceConnectStageText_->empty() &&
                *serviceConnectStageText_ != serviceConnectStatus_) {
                serviceConnectStatus_ = *serviceConnectStageText_;
            }
        }
        if (serviceConnectFuture_.valid() &&
            serviceConnectFuture_.wait_for(std::chrono::milliseconds(0)) ==
                std::future_status::ready) {
            const std::string result = serviceConnectFuture_.get();
            serviceConnectRunning_ = false;
            serviceConnectStatus_ = result;
            const bool success = result.find("ready") != std::string::npos;
            if (success && transportKind_ != TransportKind::Slcan) {
                // Drop any TCP-based transport so the status bar doesn't
                // pretend we still have a working CAN bridge.
                if (activeTransport_ != nullptr) {
                    activeTransport_->close();
                    activeTransport_ = nullptr;
                }
            }
        }
    }

#ifdef _WIN32
    // Poll WLAN association after WlanConnect -- when the target SSID becomes
    // current we fill in AP host:port and (optionally) auto-press Connect so
    // the customer flow is one click in pc_tool, no manual host typing.
    if (wlanConnecting_) {
        const std::string current = drivescope::wlan::currentSsid();
        if (!wlanConnectTarget_.empty() && current == wlanConnectTarget_) {
            wlanConnecting_ = false;
            std::snprintf(wifiApSsid_.data(), wifiApSsid_.size(),
                          "%s", wlanConnectTarget_.c_str());
            std::snprintf(wifiApHost_.data(), wifiApHost_.size(), "%s",
                          "192.168.1.1");
            wifiApPort_ = 45333;
            wlanStatus_ = wlanAutoOpenTcpPending_
                         ? "Wi-Fi connected - opening CAN bridge..."
                         : "Wi-Fi connected";
            markScenarioDirty("wlan connected");
            if (wlanAutoOpenTcpPending_) {
                wlanAutoOpenTcpPending_ = false;
                transportKind_ = TransportKind::WifiAp;
                connectSelectedTransport();
            }
        } else if (ImGui::GetTime() - wlanConnectStartAt_ > 25.0) {
            wlanConnecting_ = false;
            wlanAutoOpenTcpPending_ = false;
            wlanStatus_ = "Wi-Fi connect timed out - wrong password or AP gone";
        }
    }
#endif
}

void MainUi::scanWlanAsync()
{
#ifdef _WIN32
    if (wlanScanning_) return;
    wlanScanning_ = true;
    if (wlanStatus_.empty() || wlanStatus_.rfind("scanning", 0) == 0) {
        wlanStatus_ = "scanning Wi-Fi...";
    }
    std::thread([this]() {
        auto r = drivescope::wlan::scan("Kitchen_Machine_");
        if (!r.ok) {
            wlanStatus_ = std::string("Wi-Fi scan failed: ") + r.error;
            wlanScan_.clear();
            wlanScanning_ = false;
            return;
        }
        std::vector<WlanEntry> next;
        next.reserve(r.entries.size());
        for (auto& e : r.entries) {
            WlanEntry w;
            w.ssid       = std::move(e.ssid);
            w.signal_pct = e.signal_pct;
            w.secured    = e.secured;
            w.connected  = e.connected;
            next.push_back(std::move(w));
        }
        wlanScan_ = std::move(next);
        if (!wlanConnecting_) {
            // While a connect is in flight we keep the connect-status text;
            // scan results just refresh in the background.
            if (wlanScan_.empty()) {
                wlanStatus_ = "no Kitchen_Machine_* SSIDs visible - power-cycle device into AP?";
            } else {
                char buf[64];
                std::snprintf(buf, sizeof(buf),
                              "found %zu Kitchen_Machine_* SSID(s)", wlanScan_.size());
                wlanStatus_ = buf;
            }
        }
        wlanScanning_ = false;
    }).detach();
#else
    wlanStatus_ = "WLAN auto-connect only available on Windows";
#endif
}

void MainUi::connectWlanAsync(const std::string& ssid, const std::string& password,
                              bool autoOpenTcp)
{
#ifdef _WIN32
    wlanConnectTarget_ = ssid;
    wlanConnectStartAt_ = ImGui::GetTime();
    wlanConnecting_ = true;
    wlanAutoOpenTcpPending_ = autoOpenTcp;
    wlanStatus_ = std::string("connecting to ") + ssid + " ...";
    std::thread([this, ssid, password]() {
        std::string err;
        if (!drivescope::wlan::connect(ssid, password, err)) {
            wlanConnecting_ = false;
            wlanAutoOpenTcpPending_ = false;
            wlanStatus_ = std::string("WlanConnect failed: ") + err;
        }
        // On success the OS handles the handshake. Polling in updateIo()
        // detects when currentSsid() == ssid and triggers TCP connect.
    }).detach();
#else
    (void)ssid; (void)password;
    (void)autoOpenTcp;
    wlanStatus_ = "WLAN auto-connect only available on Windows";
#endif
}

void MainUi::handleFrame(const CanFrame& frame_in)
{
    // Re-stamp every incoming frame with our unified clock_ so plot/log/trace
    // share one time origin. Without this, TcpCanTransport::steadyNow() uses
    // a function-local static that initialises on FIRST TCP open -- which
    // floats relative to clock_ (whose origin is pc_tool launch). Plots end
    // up drawn at -<delta> seconds where delta = "time pc_tool was running
    // before the user clicked Connect". Symptom: trace shows fresh data, but
    // the line on Plots sits 10-60 s in the past relative to the X axis "now".
    CanFrame frame = frame_in;
    frame.timestamp = clock_.nowSeconds();

    // Passive sniffer tap (RX). Same cheap-when-off guard as the TX side.
    if (sniffer_.isActive()) {
        const char* tl = transportKind_ == TransportKind::Slcan  ? "slcan"
                       : transportKind_ == TransportKind::WifiAp ? "wifi_ap"
                       : transportKind_ == TransportKind::Manual ? "manual" : "tcp";
        sniffer_.record(CanSniffer::Dir::Rx, tl, frame, decodeCanFrame(frame));
    }

    if (!monitorPaused_) trace_.push(frame);
    canLogger_.log(frame);
    recordBusBits(frame);
    bootFlasher_.onFrame(frame);

    // Per-node mode tracker: every CAN frame whose src is a known node
    // updates "is this node currently running app or boot?" and (for
    // ping responses) the firmware-version cache. Used by the Remote
    // Control panel and the READ_MEM poll suppressor.
    updateNodeModeFromFrame(frame);

    // Sniff ANS_WIFI_STATUS (id 0x1B00FF01) for the Remote Control tab.
    // Payload: [mode, ip0, ip1, ip2, ip3, ok, 0, 0]. See app_dd
    // handle_app_main_frame case 0x100.
    if (frame.id == 0x1B00FF01u && frame.data.size() >= 6) {
        lastWifiStatus_.valid = true;
        lastWifiStatus_.mode  = frame.data[0];
        char buf[24];
        std::snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
                      (unsigned)frame.data[1], (unsigned)frame.data[2],
                      (unsigned)frame.data[3], (unsigned)frame.data[4]);
        lastWifiStatus_.ip = buf;
        lastWifiStatus_.ok = frame.data[5] != 0;
        lastWifiStatus_.receivedAt = ImGui::GetTime();
    }

    // ---- mainPCB diag-mode status sniffing ----
    // ID layout: mod=APP(1)<<28 | cmd<<16 | dst=BRODCAST(0xFF)<<8 | src=MAIN(0x01)
    constexpr uint32_t kIdMainAnsDiagStatus    = (1u<<28) | (0xB10u<<16) | (0xFFu<<8) | 0x01u;  // 0x1B10FF01
    constexpr uint32_t kIdMainAnsDiagTelemetry = (1u<<28) | (0xB11u<<16) | (0xFFu<<8) | 0x01u;  // 0x1B11FF01

    if (frame.id == kIdMainAnsDiagStatus && frame.data.size() >= 8) {
        const uint8_t flags = frame.data[0];
        lastDiagStatus_.valid          = true;
        lastDiagStatus_.diag_active    = (flags & 0x01u) != 0;
        lastDiagStatus_.motor_running  = (flags & 0x02u) != 0;
        lastDiagStatus_.rk_armed       = (flags & 0x04u) != 0;
        lastDiagStatus_.pfc_enabled    = (flags & 0x08u) != 0;
        lastDiagStatus_.fsm_state      = frame.data[1];
        lastDiagStatus_.hb_seq_echo    = frame.data[2];
        uint16_t wdg = 0;
        std::memcpy(&wdg, &frame.data[4], sizeof(uint16_t));
        lastDiagStatus_.wdg_remaining_ms = wdg;
        lastDiagStatus_.receivedAt = ImGui::GetTime();
        diagModeOn_ = lastDiagStatus_.diag_active;
    }

    if (frame.id == kIdMainAnsDiagTelemetry && frame.data.size() >= 8) {
        lastMotorTelemetry_.valid = true;
        lastMotorTelemetry_.mode  = frame.data[0];
        const uint8_t sf = frame.data[1];
        lastMotorTelemetry_.motor_online  = (sf & 0x01u) != 0;
        lastMotorTelemetry_.rk_online     = (sf & 0x02u) != 0;
        lastMotorTelemetry_.motor_in_boot = (sf & 0x04u) != 0;
        lastMotorTelemetry_.rk_in_boot    = (sf & 0x08u) != 0;
        float rpm = 0.0f;
        std::memcpy(&rpm, &frame.data[2], sizeof(float));
        lastMotorTelemetry_.rpm = rpm;
        uint16_t vdc_dV = 0;
        std::memcpy(&vdc_dV, &frame.data[6], sizeof(uint16_t));
        lastMotorTelemetry_.voltage_dc = static_cast<float>(vdc_dV) * 0.1f;
        lastMotorTelemetry_.receivedAt = ImGui::GetTime();
    }

    // ---- Direct motor frames (when on the same CAN bus / SLCAN) ----
    // src=MOTOR(0x02), dst=MAIN(0x01), mod=APP(1).
    const uint8_t frameMod = static_cast<uint8_t>((frame.id >> 28) & 0x01u);
    const uint8_t frameSrc = static_cast<uint8_t>(frame.id & 0xFFu);
    const bool motorFrame = (frameMod == 1u && frameSrc == kCanAddrMotor);
    if (motorFrame && frame.data.size() >= 8) {
        const uint16_t cmd = static_cast<uint16_t>((frame.id >> 16) & 0x0FFFu);
        const auto getF = [&](int off) {
            float v = 0.0f; std::memcpy(&v, &frame.data[off], sizeof(float)); return v;
        };
        switch (cmd) {
            case 0x0A1: lastMotorTelemetry_.power_W   = getF(1); lastMotorTelemetry_.mode = frame.data[0]; break;
            case 0x0A2: lastMotorTelemetry_.voltage_q = getF(1); break;
            case 0x0A3: lastMotorTelemetry_.rpm       = getF(1); break;
            case 0x0A4: lastMotorTelemetry_.voltage_dc= getF(1); break;
            case 0x0A5: lastMotorTelemetry_.vt_temp   = getF(0);
                        lastMotorTelemetry_.motor_temp= getF(4); break;
            case 0x0A6: { uint32_t m=0; std::memcpy(&m, &frame.data[1], 4); lastMotorTelemetry_.fault_mask = m; lastMotorTelemetry_.mode = frame.data[0]; break; }
            case 0x0AA: lastDiagStatus_.pfc_enabled = (frame.data[1] != 0); break;
            case kMotorAnsProfiler: {
                motorProfiler_.valid = true;
                motorProfiler_.state = frame.data[0];
                motorProfiler_.active_test = frame.data[1];
                motorProfiler_.progress = frame.data[2];
                motorProfiler_.error = static_cast<uint8_t>(frame.data[3] & 0x0Fu);
                motorProfiler_.result_sel = static_cast<uint8_t>((frame.data[3] >> 4) & 0x0Fu);
                if (motorProfiler_.result_sel < 7u) {
                    float v = 0.0f;
                    std::memcpy(&v, &frame.data[4], sizeof(v));
                    motorProfiler_.results[motorProfiler_.result_sel] = v;
                    motorProfiler_.result_valid[motorProfiler_.result_sel] = true;
                }
                motorProfiler_.receivedAt = ImGui::GetTime();
                break;
            }
            case 0x0AD: {
                // MTR_ANS_THETA_CAL -- theta-calibration live telemetry.
                // Wire layout (motor FW events_can.c case MOTOR_INFO_THETA):
                //   [0] driverMode  [1] cal phase  [2] progress %
                //   [3] flags: bits0..5 per-sector LUT valid, bit6 global ok,
                //              bit7 save-to-flash requested
                //   [4..7] global theta offset (float32 LE, rad)
                lastMotorTelemetry_.mode = frame.data[0];
                const uint8_t phase = frame.data[1];
                const uint8_t flags = frame.data[3];
                lastMotorTelemetry_.theta_cal_valid      = true;
                lastMotorTelemetry_.cal_phase            = phase;
                lastMotorTelemetry_.cal_progress_pct     = frame.data[2];
                lastMotorTelemetry_.cal_sector_valid     = static_cast<uint8_t>(flags & 0x3Fu);
                lastMotorTelemetry_.cal_global_ok        = (flags & 0x40u) != 0u;
                lastMotorTelemetry_.cal_theta_offset_rad = getF(4);
                lastMotorTelemetry_.cal_receivedAt       = ImGui::GetTime();

                // Theta run-state machine owns the RUNNING ack, the RUNNING ->
                // DONE edge (re-read BLOB once on DONE_OK, fail reason on
                // DONE_FAIL), the verdict and the run-guard release. See
                // calib::ThetaCalibration.
                thetaCal_.onCalTelemetry(phase, ImGui::GetTime());
                break;
            }
            case kMotorAnsConfig:
                if (frame.data[0] == 0x02 /* PARAM */) {
                    /* Motor Config tab cache. data[1]=status,
                     * data[2..3]=u16 id LE, data[4..7]=value raw u32 LE. */
                    if (frame.data.size() >= 8) {
                        const uint16_t pid = static_cast<uint16_t>(frame.data[2]) |
                                             (static_cast<uint16_t>(frame.data[3]) << 8);
                        uint32_t val = 0;
                        std::memcpy(&val, &frame.data[4], 4);
                        handleMotorConfigResponse(pid, frame.data[1], val);
                    }
                } else if (frame.data[0] == kMotorCfgProfiler) {
                    motorProfiler_.valid = true;
                    motorProfiler_.cfg_status = frame.data[1];
                    motorProfiler_.state = frame.data[2];
                    const bool looksLikeStatus =
                        frame.data[7] == 0u &&
                        frame.data[4] <= 100u &&
                        frame.data[5] <= 7u &&
                        frame.data[6] < 7u;
                    if (looksLikeStatus) {
                        motorProfiler_.active_test = frame.data[3];
                        motorProfiler_.progress = frame.data[4];
                        motorProfiler_.error = frame.data[5];
                        motorProfiler_.result_sel = frame.data[6];
                    } else {
                        const uint8_t sel = frame.data[3];
                        if (sel < 7u) {
                            float v = 0.0f;
                            std::memcpy(&v, &frame.data[4], sizeof(v));
                            motorProfiler_.results[sel] = v;
                            motorProfiler_.result_valid[sel] = true;
                            motorProfiler_.result_sel = sel;
                        }
                    }
                    motorProfiler_.receivedAt = ImGui::GetTime();
                } else if (frame.data[0] == kMotorCfgBlob) {
                    /* Whole-blob transfer response. data[1] = action,
                     * data[2] = chunk_idx, data[3..7] = payload bytes.
                     * Stream assembles into motorBlob_; on completeness +
                     * magic verify, motorBlobValid_ flips true and theta
                     * mirrors populate from the staged buffer. */
                    const uint8_t blobAction = frame.data[1];
                    if (blobAction == kMotorBlobActGetResponseChunk) {
                        const uint8_t cidx = frame.data[2];
                        if (cidx < kMotorCfgBlobChunkCount) {
                            const uint32_t off = uint32_t(cidx) * kMotorCfgBlobChunkBytes;
                            const uint8_t  remain = (cidx == kMotorCfgBlobChunkCount - 1u)
                                ? uint8_t(kMotorCfgBlobTotalBytes - off)
                                : kMotorCfgBlobChunkBytes;
                            for (uint8_t b = 0; b < remain && (off + b) < sizeof(motorBlob_); ++b) {
                                motorBlob_[off + b] = frame.data[3u + b];
                            }
                            motorBlobReceivedMask_ |= (uint64_t(1u) << cidx);
                            const uint64_t fullMask =
                                (uint64_t(1u) << kMotorCfgBlobChunkCount) - 1u;
                            if (motorBlobReceivedMask_ == fullMask) {
                                /* All chunks present. Verify magic word in
                                 * header offset 0..3 (LE uint32). */
                                uint32_t magic = 0;
                                std::memcpy(&magic, &motorBlob_[0], sizeof(magic));
                                if (magic == kMotorBlobMagic) {
                                    motorBlobValid_ = true;
                                    motorBlobReceivedAt_ = ImGui::GetTime();
                                    /* Mirror theta fields into existing
                                     * lastMotorTelemetry_ slots so the
                                     * existing UI keeps rendering without
                                     * a separate code path. */
                                    float thetaOff = 0.0f;
                                    std::memcpy(&thetaOff,
                                                &motorBlob_[kMotorBlobOffThetaOffset],
                                                sizeof(thetaOff));
                                    lastMotorTelemetry_.theta_valid = true;
                                    lastMotorTelemetry_.theta_offset_rad = thetaOff;
                                    lastMotorTelemetry_.theta_receivedAt = ImGui::GetTime();
                                    if (!motorThetaManualDirty_) {
                                        motorThetaManualRad_ = thetaOff;
                                    }
                                    if (!motorThetaSlotDirty_[0]) {
                                        motorThetaSlotEdit_[0] = thetaOff;
                                    }
                                    for (uint8_t s = 0; s < 6u; ++s) {
                                        float v = 0.0f;
                                        std::memcpy(&v,
                                                    &motorBlob_[kMotorBlobOffThetaSectorLut + s * 4u],
                                                    sizeof(v));
                                        lastMotorTelemetry_.theta_sector_valid[s] = true;
                                        lastMotorTelemetry_.theta_sector_lut_rad[s] = v;
                                        lastMotorTelemetry_.theta_sector_receivedAt[s] = ImGui::GetTime();
                                        if (!motorThetaSlotDirty_[s + 1u]) {
                                            motorThetaSlotEdit_[s + 1u] = v;
                                        }
                                    }
                                    motorThetaTxStatus_ = "blob GET ok (164 B, CRC verified by FW)";
                                    /* Unified config view: the Motor
                                     * Config tab reads from the same
                                     * motorConfigCache_ map that used to
                                     * be filled by per-param PARAM GETs.
                                     * Populate it from the staged blob
                                     * so a single GET feeds both tabs. */
                                    populateMotorConfigCacheFromBlob();
                                } else {
                                    /* Header magic mismatch -- something
                                     * fishy. Drop the buffer and let the
                                     * next poll retry. */
                                    motorBlobReceivedMask_ = 0u;
                                    motorThetaTxStatus_ = "blob GET: bad magic, retrying";
                                }
                            }
                        }
                    } else if (blobAction == kMotorBlobActSetResult) {
                        motorBlobLastSetResult_ = frame.data[3];
                        std::memcpy(&motorBlobLastSetCrcEcho_, &frame.data[4],
                                    sizeof(motorBlobLastSetCrcEcho_));
                        motorBlobLastSetResultAt_ = ImGui::GetTime();
                        char buf[96] = {};
                        std::snprintf(buf, sizeof(buf),
                                      "blob SET result: status=%u crc=0x%08X",
                                      (unsigned) motorBlobLastSetResult_,
                                      (unsigned) motorBlobLastSetCrcEcho_);
                        motorThetaTxStatus_ = buf;
                    }
                    /* SET_CHUNK ACK frames (action=2) are informational
                     * only -- we don't track per-chunk state on the host
                     * since the FW will reject the COMMIT if any chunk
                     * was missed. Ignore. */
                } else if (frame.data[0] == kMotorCfgThetaOffset ||
                    frame.data[0] == kMotorCfgThetaSector) {
                    const uint8_t respSubcmd = frame.data[0];
                    const float angle  = getF(2);
                    const uint8_t st   = frame.data[1];
                    const uint8_t flg  = frame.data[6];
                    const uint8_t dm   = frame.data[7];
                    const double now   = ImGui::GetTime();

                    // Status/flags fields reflect the motor's CURRENT
                    // global state regardless of which value [2..5] held,
                    // so always update those.
                    lastMotorTelemetry_.theta_status = st;
                    lastMotorTelemetry_.theta_flags = flg;
                    lastMotorTelemetry_.theta_driver_mode = dm;

                    // A rejected CAL/CAL_SAVE command (global-offset subcmd)
                    // is surfaced by the theta state machine, which builds the
                    // banner text, fails the run and releases the guard. It
                    // ignores benign GET-response statuses and statuses seen
                    // while no run is in flight.
                    if (respSubcmd == kMotorCfgThetaOffset) {
                        thetaCal_.onCfgReject(st, ImGui::GetTime());
                    }

                    // Route the angle into the right slot. Sector subcmd
                    // responses always go to LUT[bin]. Global subcmd
                    // responses go to LUT[bin] iff the pending front was
                    // a GET_SECTOR for that bin; otherwise to global.
                    bool routedToSector = false;
                    if (!motorThetaPending_.empty()) {
                        const auto pend = motorThetaPending_.front();
                        const bool sectorResp = (respSubcmd == kMotorCfgThetaSector) &&
                                                (pend.first == kMotorCfgThetaSector);
                        const bool getSectorResp = (respSubcmd == kMotorCfgThetaOffset) &&
                                                   (pend.first == kMotorThetaGetSector);
                        if ((sectorResp || getSectorResp) && pend.second < 6u) {
                            const uint8_t bin = pend.second;
                            motorThetaPending_.pop_front();
                            lastMotorTelemetry_.theta_sector_valid[bin] = true;
                            lastMotorTelemetry_.theta_sector_lut_rad[bin] = angle;
                            lastMotorTelemetry_.theta_sector_receivedAt[bin] = now;
                            // If the user wasn't mid-edit on this slot, sync
                            // the edit buffer to the new value.
                            if (!motorThetaSlotDirty_[bin + 1u]) {
                                motorThetaSlotEdit_[bin + 1u] = angle;
                            }
                            routedToSector = true;
                            char buf[96] = {};
                            std::snprintf(buf, sizeof(buf),
                                          "sector %u response status=%u angle=%.5f rad",
                                          (unsigned)(bin + 1u), (unsigned)st, angle);
                            motorThetaTxStatus_ = buf;
                        } else if (respSubcmd == kMotorCfgThetaOffset) {
                            // Pending front matches the global response;
                            // pop it.
                            motorThetaPending_.pop_front();
                        }
                    }
                    if (!routedToSector && respSubcmd == kMotorCfgThetaOffset) {
                        // Default: response is for the global offset (auto-poll
                        // GET, manual GET, SET ack, CAL ack, CLEAR ack).
                        lastMotorTelemetry_.theta_valid = true;
                        lastMotorTelemetry_.theta_offset_rad = angle;
                        lastMotorTelemetry_.theta_receivedAt = now;
                        if (!motorThetaManualDirty_) {
                            motorThetaManualRad_ = angle;
                        }
                        if (!motorThetaSlotDirty_[0]) {
                            motorThetaSlotEdit_[0] = angle;
                        }
                        char buf[96] = {};
                        std::snprintf(buf, sizeof(buf), "global response status=%u angle=%.5f rad",
                                      (unsigned)st, angle);
                        motorThetaTxStatus_ = buf;
                    }
                }
                break;
            /* ========== Unified READ protocol (motor app, 2026-05-13) =====
             * Same code namespace as the legacy BLOB sub-cmd 0x03 above,
             * but separated wire cmds:
             *   0xAD2 ANS_READ_HEADER_FILE_OK -> reset bytes, capture
             *         expected_size echo.
             *   0xD3  HEADER_BLOCK             -> no action needed for
             *         single-block reads (we read only fileCrc on
             *         FILE_FINISH).
             *   0xD4  HEADER_MMSG              -> capture cur_mmsg
             *         idx + size so DATA_MMSG knows where to splat.
             *   0xD5  DATA_MMSG                -> splat 7 B into
             *         motorBlob_ at (mmsg_idx*1792 + msg_idx*7).
             *   0xDF  FILE_FINISH              -> validate host CRC32 ==
             *         motor fileCrc; on match -> populate caches.
             *   0xAD8 READ_ERROR               -> status string only. */
            case kFtAnsReadHeaderFileOk:
                std::memcpy(&motorReadExpectedSize_, &frame.data[4],
                            sizeof(uint16_t));
                std::memset(motorBlob_, 0, sizeof(motorBlob_));
                motorReadInFlight_  = true;
                motorReadCurMmsgIdx_  = 0;
                motorReadCurMmsgSize_ = 0;
                break;
            case kFtCmdReadHeaderBlock:
                /* No per-block state needed in single-block flow. */
                break;
            case kFtCmdReadHeaderMmsg: {
                uint16_t mmsg_size = 0;
                std::memcpy(&mmsg_size, &frame.data[0], sizeof(uint16_t));
                motorReadCurMmsgIdx_  = frame.data[7];
                motorReadCurMmsgSize_ = mmsg_size;
                break;
            }
            case kFtCmdReadDataMmsg: {
                if (!motorReadInFlight_) break;
                const uint32_t mmsg_off =
                    static_cast<uint32_t>(motorReadCurMmsgIdx_) * kFtMmsgMaxDataSize;
                const uint8_t  msg_idx = frame.data[7];
                const uint32_t msg_off = mmsg_off +
                    static_cast<uint32_t>(msg_idx) * kFtMsgDataSize;
                /* Last msg in the mmsg may carry fewer than 7 valid bytes. */
                const uint32_t bytes_used_in_mmsg =
                    static_cast<uint32_t>(msg_idx) * kFtMsgDataSize;
                const uint32_t mmsg_remain =
                    (motorReadCurMmsgSize_ > bytes_used_in_mmsg)
                    ? (motorReadCurMmsgSize_ - bytes_used_in_mmsg) : 0u;
                const uint8_t msg_used =
                    static_cast<uint8_t>((mmsg_remain >= kFtMsgDataSize)
                                         ? kFtMsgDataSize : mmsg_remain);
                if (msg_off + msg_used <= sizeof(motorBlob_)) {
                    std::memcpy(&motorBlob_[msg_off], &frame.data[0], msg_used);
                }
                break;
            }
            case kFtCmdReadFileFinish: {
                if (!motorReadInFlight_) break;
                motorReadInFlight_ = false;
                uint32_t motor_crc = 0;
                std::memcpy(&motor_crc, &frame.data[0], sizeof(uint32_t));
                const uint32_t host_crc =
                    motorStm32HwCrc32(motorBlob_, motorReadExpectedSize_);
                if (motor_crc == host_crc) {
                    motorBlobValid_ = true;
                    motorBlobReceivedAt_ = ImGui::GetTime();
                    /* Mirror header fields + theta into telemetry caches
                     * (same code path as the legacy BLOB receive). */
                    float thetaOff = 0.0f;
                    std::memcpy(&thetaOff,
                                &motorBlob_[kMotorBlobOffThetaOffset],
                                sizeof(thetaOff));
                    lastMotorTelemetry_.theta_valid = true;
                    lastMotorTelemetry_.theta_offset_rad = thetaOff;
                    lastMotorTelemetry_.theta_receivedAt = ImGui::GetTime();
                    if (!motorThetaManualDirty_) motorThetaManualRad_ = thetaOff;
                    if (!motorThetaSlotDirty_[0]) motorThetaSlotEdit_[0] = thetaOff;
                    for (uint8_t s = 0; s < 6u; ++s) {
                        float v = 0.0f;
                        std::memcpy(&v,
                                    &motorBlob_[kMotorBlobOffThetaSectorLut + s * 4u],
                                    sizeof(v));
                        lastMotorTelemetry_.theta_sector_valid[s] = true;
                        lastMotorTelemetry_.theta_sector_lut_rad[s] = v;
                        lastMotorTelemetry_.theta_sector_receivedAt[s] = ImGui::GetTime();
                        if (!motorThetaSlotDirty_[s + 1u]) {
                            motorThetaSlotEdit_[s + 1u] = v;
                        }
                    }
                    populateMotorConfigCacheFromBlob();
                    char buf[96] = {};
                    std::snprintf(buf, sizeof(buf),
                                  "unified READ ok (%u B, CRC 0x%08X)",
                                  static_cast<unsigned>(motorReadExpectedSize_),
                                  static_cast<unsigned>(motor_crc));
                    motorThetaTxStatus_ = buf;
                } else {
                    char buf[160] = {};
                    std::snprintf(buf, sizeof(buf),
                                  "unified READ CRC mismatch: motor=0x%08X host=0x%08X",
                                  static_cast<unsigned>(motor_crc),
                                  static_cast<unsigned>(host_crc));
                    motorThetaTxStatus_ = buf;
                }
                break;
            }
            case kFtAnsReadError:
                motorReadInFlight_ = false;
                {
                    char buf[64] = {};
                    std::snprintf(buf, sizeof(buf), "unified READ err=0x%02X",
                                  static_cast<unsigned>(frame.data[0]));
                    motorThetaTxStatus_ = buf;
                }
                break;
            /* Unified WRITE (Phase 3, 2026-05-13) -- response codes
             * 0xAC0..0xACF. Motor sends ANS_GET_FILE_OK after
             * FILE_FINISH if the wire CRC32 matches bufBlock and
             * MotorConfigScatter applied successfully; ANS_ERROR
             * carries err code (e.g. EBUSY when motor running). The
             * per-block / per-mmsg OKs are informational and only
             * tracked for status text. */
            case kFtAnsFileOk:
                motorBlobLastSetResult_ = 0;   /* 0 = OK (matches legacy semantics) */
                motorBlobLastSetResultAt_ = ImGui::GetTime();
                motorThetaTxStatus_ = "unified WRITE applied (config scattered to RAM)";
                /* Mirror into Motor Config tab status so the operator sees
                 * the "applied" confirmation regardless of which tab is open. */
                motorConfigStatus_ = "BLOB SET applied (motor scattered to RAM)";
                break;
            case kFtAnsError: {
                motorBlobLastSetResult_ = frame.data[0];
                motorBlobLastSetResultAt_ = ImGui::GetTime();
                char errbuf[80] = {};
                std::snprintf(errbuf, sizeof(errbuf),
                              "unified WRITE err=0x%02X (likely motor busy / CRC fail)",
                              static_cast<unsigned>(frame.data[0]));
                motorThetaTxStatus_ = errbuf;
                motorConfigStatus_  = errbuf;
                break;
            }
            case kFtAnsBlockOk:
            case kFtAnsMmsgOk:
            case kFtAnsEraseOk:
                /* Informational only; not needed for the single-block
                 * config write flow. Falls through silently. */
                break;
            default: break;
        }
        lastMotorTelemetry_.valid = true;
        lastMotorTelemetry_.receivedAt = ImGui::GetTime();
    }

    // Triggered Capture frames go to the capture state machine. They share
    // the PropCAN ID with READ_MEM but have a distinct cmd code, so we
    // dispatch first and bail to avoid the READ_MEM parser misinterpreting
    // a CAP response.
    {
        const uint8_t mod = static_cast<uint8_t>((frame.id >> 28) & 0x01u);
        const uint16_t cmd = static_cast<uint16_t>((frame.id >> 16) & 0x0FFFu);
        const uint8_t  dst = static_cast<uint8_t>((frame.id >> 8) & 0xFFu);
        if (mod == 1u && cmd == kCanCmdCapRsp && dst == kCanAddrPc) {
            handleCapFrame(frame);
            return;
        }
    }

    // Phase C 2026-05-27: STREAM_* frames from motor. STREAM_ADD_RSP and
    // STREAM_VALUE both carry data we need to plumb back into the watch.
    // Dispatch before the READ_MEM parser so the cmd_id check doesn't
    // misclassify them.
    if (auto srsp = debug_.parseStreamAddRsp(frame)) {
        handleStreamAddRsp(*srsp, frame.timestamp);
        return;
    }
    if (auto sval = debug_.parseStreamValue(frame)) {
        handleStreamValue(*sval, frame.timestamp);
        return;
    }
    if (auto sstat = debug_.parseStreamStatusRsp(frame)) {
        handleStreamStatusRsp(*sstat, frame.timestamp);
        return;
    }

    const auto rsp = debug_.parseReadMemResponse(frame);
    if (!rsp) return;

    const auto it = pending_.find(pendingKey(rsp->nodeId, rsp->seq));
    if (it == pending_.end() || it->second >= watches_.size()) return;

    WatchVar& w = watches_[it->second];
    pendingSentAt_.erase(it->first);
    pending_.erase(it);
    w.lastTimestamp = frame.timestamp;

    bool ok = false;
    if (rsp->status == 0) {
        w.value = decodeValue(rsp->value, w.type, ok);
    }
    w.valueOk = ok;
    if (rsp->status != 0) {
        w.valueText = "read err " + std::to_string(static_cast<unsigned>(rsp->status));
    } else {
        w.valueText = formatValue(w.value, w.type, ok);
    }

    if (ok && w.plot && !plotPaused_) {
        const double tRel = frame.timestamp - plotTimeOrigin_;
        w.xs.push_back(tRel);
        w.ys.push_back(w.value);
        const double keepAfter = tRel - static_cast<double>(plotHistorySec_);
        size_t first = 0;
        while (first < w.xs.size() && w.xs[first] < keepAfter) ++first;
        if (first > 0) {
            w.xs.erase(w.xs.begin(), w.xs.begin() + static_cast<std::ptrdiff_t>(first));
            w.ys.erase(w.ys.begin(), w.ys.begin() + static_cast<std::ptrdiff_t>(first));
        }
        signalLogger_.log(frame.timestamp, w.nodeId, w.name, w.address, w.type, w.valueText);
    }
}

// === Per-node operating-mode tracker ====================================
// Sources: CanBootloader (mod=BOOT cmds 0xB00/0xB01/0xFFF/0x0FF) and the
// app-mode ping (mod=APP cmd 0x0FF). The tracker is intentionally lazy:
// it only believes what it sees on the bus. There is no active probing
// loop -- sendNodePing() lets the user explicitly force a fresh ping
// from the Remote Control panel.

namespace {
constexpr double kNodeModeStaleSec = 8.0;
}

std::string nodeFwVersionString(const std::vector<uint8_t>& d, size_t off)
{
    if (off + 4 > d.size()) return {};
    // Same convention as CanDecoder::versionText -- the firmware stores
    // the version as a uint32_t and memcpy's its raw LE bytes into the
    // CAN payload, so byte[0] is the LSB. Render M.m.p.b by reading
    // from off+3 down to off+0.
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
                  static_cast<unsigned>(d[off + 3]),
                  static_cast<unsigned>(d[off + 2]),
                  static_cast<unsigned>(d[off + 1]),
                  static_cast<unsigned>(d[off + 0]));
    return buf;
}

void MainUi::updateNodeModeFromFrame(const CanFrame& frame)
{
    if (!frame.extended) return;
    if (frame.data.size() < 8) return;
    const uint8_t src = static_cast<uint8_t>(frame.id & 0xFFu);
    const uint8_t mod = static_cast<uint8_t>((frame.id >> 28) & 0x01u);
    const uint16_t cmd = static_cast<uint16_t>((frame.id >> 16) & 0x0FFFu);
    if (src == 0u || src >= kNodeIdMax) return;
    if (src == kCanAddrPc) return;  // PC is us -- don't track
    NodeMode& nm = nodeMode_[src];
    nm.valid = true;
    nm.mod = mod;
    nm.modSeenAt = clock_.nowSeconds();
    // Ping reply carries [bootVer, appVer] regardless of which mode is
    // currently running -- same payload format on motor/RK app + boot.
    if (cmd == 0x0FFu) {
        nm.bootVersion = nodeFwVersionString(frame.data, 0);
        nm.appVersion  = nodeFwVersionString(frame.data, 4);
        nm.versionValid = true;
        nm.versionAt = nm.modSeenAt;
    }
}

bool MainUi::nodeIsInBoot(uint8_t nodeId) const
{
    if (nodeId >= kNodeIdMax) return false;
    const NodeMode& nm = nodeMode_[nodeId];
    if (!nm.valid) return false;
    return nm.mod == 0u;
}

bool MainUi::nodeIsInApp(uint8_t nodeId) const
{
    if (nodeId >= kNodeIdMax) return false;
    const NodeMode& nm = nodeMode_[nodeId];
    if (!nm.valid) return false;
    return nm.mod == 1u && (clock_.nowSeconds() - nm.modSeenAt) < kNodeModeStaleSec;
}

bool MainUi::sendNodePing(uint8_t nodeId, bool bootMode)
{
    if (!isConnected()) return false;
    CanFrame f{};
    f.extended = true;
    // mod | cmd | dst | src
    const uint8_t mod = bootMode ? 0u : 1u;
    const uint16_t cmd = bootMode ? 0xFFFu : 0x0FFu;
    f.id = (static_cast<uint32_t>(mod) << 28) |
           (static_cast<uint32_t>(cmd & 0x0FFFu) << 16) |
           (static_cast<uint32_t>(nodeId) << 8) |
           kCanAddrPc;
    f.data.assign(8, 0xFFu);
    f.timestamp = clock_.nowSeconds();
    if (!sendFrameThreadSafe(f)) return false;
    if (nodeId < kNodeIdMax) nodeMode_[nodeId].pingSentAt = f.timestamp;
    return true;
}

bool MainUi::sendNodeGoToBoot(uint8_t nodeId)
{
    if (!isConnected()) return false;
    // mod=APP, cmd=0x001 (motor/RK GO_BOOT in app mode), magic 0xCCГ—4 0xDDГ—4.
    CanFrame f{};
    f.extended = true;
    f.id = (1u << 28) | (0x001u << 16) | (static_cast<uint32_t>(nodeId) << 8) | kCanAddrPc;
    f.data = {0xCC, 0xCC, 0xCC, 0xCC, 0xDD, 0xDD, 0xDD, 0xDD};
    f.timestamp = clock_.nowSeconds();
    if (!sendFrameThreadSafe(f)) return false;
    // Schedule a follow-up ping in BOOT mode so the indicator flips
    // without the operator having to click Ping. 300 ms gives the MCU
    // time to reset and bring the bootloader's CAN stack up.
    if (nodeId < kNodeIdMax) {
        NodeMode& nm = nodeMode_[nodeId];
        nm.valid = true;
        nm.mod = 0u;
        nm.modSeenAt = f.timestamp;
        nm.autoPingDueAt = f.timestamp + 0.3;
        nm.autoPingBootMode = true;
    }
    return true;
}

bool MainUi::sendNodeGoToApp(uint8_t nodeId)
{
    if (!isConnected()) return false;
    // mod=BOOT, cmd=0xB01 BOOT_CMD_GO_TO_APP, all 0xFF guard.
    CanFrame f{};
    f.extended = true;
    f.id = (0u << 28) | (0xB01u << 16) | (static_cast<uint32_t>(nodeId) << 8) | kCanAddrPc;
    f.data.assign(8, 0xFFu);
    f.timestamp = clock_.nowSeconds();
    if (!sendFrameThreadSafe(f)) return false;
    // Same auto-ping pattern, but in APP mode this time.
    if (nodeId < kNodeIdMax) {
        nodeMode_[nodeId].autoPingDueAt = f.timestamp + 0.3;
        nodeMode_[nodeId].autoPingBootMode = false;
    }
    return true;
}

// ============================================================================
// Phase C (2026-05-27) — STREAM_* subscription orchestration. Motor-only for
// now (12 slots); RK / ESP32 keep using legacy READ_MEM polling.
//
// Lifecycle per motor watch:
//   enabled false→true:  sendStreamAddForWatch → ADD_REQ on bus.
//   ADD_RSP arrives:     handleStreamAddRsp marks streamSubscribed=true and
//                        records slotId → watch idx in motorStreamSlotToWatch_.
//   STREAM_VALUE rx:     handleStreamValue decodes 4 LE bytes per sizeType,
//                        appends to xs/ys (if plot), updates valueText.
//   enabled true→false:  sendStreamRemoveForWatch (silent).
//
// While any motor watch is subscribed we send STREAM_HEARTBEAT at 1 Hz so
// the motor's 5-second TTL doesn't drop the subscription on idle pc_tool.
// ============================================================================

void MainUi::sendStreamAddForWatch(size_t watchIdx, double now)
{
    if (watchIdx >= watches_.size()) return;
    WatchVar& w = watches_[watchIdx];
    if (!streamSupportedForNode(w.nodeId)) return;
    if (activeTransport_ == nullptr || !activeTransport_->isOpen()) return;
    if (activeOrPendingStreamWatchCount() >= kMaxLiveStreamWatches) {
        w.streamQuotaLimited = true;
        w.streamPending = false;
        w.streamSubscribed = false;
        w.streamSlotId = 0xFF;
        w.valueOk = false;
        w.valueText = "stream limit 6";
        w.nextPollTime = now + 0.5;
        return;
    }
    w.streamQuotaLimited = false;

    StreamSubscribeRequest req;
    req.nodeId     = w.nodeId;
    req.seq        = streamAddSeqNext_++;
    req.address    = w.address;
    req.sizeType   = streamSizeTypeFromCaptureType(
                        static_cast<CaptureSlotType>(captureSlotTypeFromString(w.type)));
    req.periodHint = 0;  // motor picks the adaptive period

    CanFrame tx = debug_.makeStreamAdd(req, now);
    if (!sendFrameThreadSafe(tx)) return;

    w.streamPending = true;
    w.streamSentAt = now;
    w.sw.onAddSent(now);
    streamAddPending_[req.seq] = watchIdx;
    if (activeTransport_ == static_cast<ICanTransport*>(&slcan_)) {
        if (!monitorPaused_) trace_.push(tx);
        canLogger_.log(tx);
    }
    recordBusBits(tx);
}

void MainUi::sendStreamRemoveForWatch(size_t watchIdx, double now)
{
    if (watchIdx >= watches_.size()) return;
    WatchVar& w = watches_[watchIdx];
    if (!streamSupportedForNode(w.nodeId)) return;
    if (activeTransport_ == nullptr || !activeTransport_->isOpen()) return;
    if (w.streamSlotId == 0xFF) {
        w.streamSubscribed = false;
        w.streamPending = false;
        w.streamQuotaLimited = false;
        return;
    }
    CanFrame tx = debug_.makeStreamRemove(w.nodeId,
                                          streamAddSeqNext_++,
                                          w.streamSlotId, now);
    (void) sendFrameThreadSafe(tx);  // silent — motor doesn't ACK REMOVE
    if (activeTransport_ == static_cast<ICanTransport*>(&slcan_)) {
        if (!monitorPaused_) trace_.push(tx);
        canLogger_.log(tx);
    }
    recordBusBits(tx);

    if (w.streamSlotId < motorStreamSlotToWatch_.size()) {
        motorStreamSlotToWatch_[w.streamSlotId] = -1;
    }
    w.streamSlotId     = 0xFF;
    w.streamSubscribed = false;
    w.streamPending    = false;
    w.streamQuotaLimited = false;
}

void MainUi::handleStreamAddRsp(const StreamSubscribeResponse& rsp, double now)
{
    auto it = streamAddPending_.find(rsp.seq);
    if (it == streamAddPending_.end()) return;
    const size_t watchIdx = it->second;
    streamAddPending_.erase(it);
    if (watchIdx >= watches_.size()) return;
    WatchVar& w = watches_[watchIdx];
    w.streamPending = false;

    const bool accepted = (rsp.status == kStreamAddStatusOk ||
                           rsp.status == kStreamAddStatusDuplicate);
    w.sw.onAddRsp(now, accepted);   // motor answered -> path alive either way
    if (accepted) {
        w.streamSlotId      = rsp.slotId;
        w.streamSubscribed  = true;
        w.streamQuotaLimited = false;
        w.streamLastValueAt = now;
        if (rsp.slotId < motorStreamSlotToWatch_.size()) {
            motorStreamSlotToWatch_[rsp.slotId] = static_cast<int>(watchIdx);
        }
    } else {
        // BAD_ADDR / BAD_SIZE / TABLE_FULL — give up on subscription,
        // fall through to READ_MEM polling for this watch (no retry storm).
        w.streamSlotId     = 0xFF;
        w.streamSubscribed = false;
        if (rsp.status == kStreamAddStatusTableFull) {
            w.streamQuotaLimited = true;
            w.valueOk = false;
            w.valueText = "stream limit 6";
            w.nextPollTime = now + 0.5;
        } else {
            w.streamQuotaLimited = false;
        }
    }
}

void MainUi::handleStreamValue(const StreamValueFrame& v, double now)
{
    if (v.slotId >= motorStreamSlotToWatch_.size()) return;
    const int idx = motorStreamSlotToWatch_[v.slotId];
    if (idx < 0 || static_cast<size_t>(idx) >= watches_.size()) return;
    WatchVar& w = watches_[static_cast<size_t>(idx)];
    if (!w.streamSubscribed) return;

    // Trim to declared-size bytes so decodeValue('uint8') doesn't see 4
    // bytes worth of zero-pad as a tiny u32.
    const uint8_t sz = streamSizeBytes(v.sizeType);
    const size_t n = (sz == 0u || sz > 4u) ? 4u : sz;
    std::vector<uint8_t> bytes(v.valueBytes.begin(),
                               v.valueBytes.begin() + n);
    bool ok = false;
    const double val = decodeValue(bytes, w.type, ok);
    w.value          = val;
    w.valueOk        = ok;
    w.lastTimestamp  = now;
    w.streamLastValueAt = now;
    w.sw.onValue(now);
    w.valueText      = formatValue(val, w.type, ok);

    if (w.plot && !plotPaused_) {
        const double tRel = now - plotTimeOrigin_;
        w.xs.push_back(tRel);
        w.ys.push_back(val);
        const double cutoff = tRel - plotHistorySec_;
        while (!w.xs.empty() && w.xs.front() < cutoff) {
            w.xs.erase(w.xs.begin());
            w.ys.erase(w.ys.begin());
        }
    }
}

void MainUi::sendStreamHeartbeat(double now)
{
    if (activeTransport_ == nullptr || !activeTransport_->isOpen()) return;
    CanFrame tx = debug_.makeStreamHeartbeat(kCanAddrMotor, now);
    (void) sendFrameThreadSafe(tx);
    if (activeTransport_ == static_cast<ICanTransport*>(&slcan_)) {
        if (!monitorPaused_) trace_.push(tx);
        canLogger_.log(tx);
    }
    recordBusBits(tx);
    lastStreamHeartbeatAt_ = now;
}

void MainUi::sendStreamStatusReq(double now)
{
    if (activeTransport_ == nullptr || !activeTransport_->isOpen()) return;
    CanFrame tx = debug_.makeStreamStatusReq(kCanAddrMotor, now);
    (void) sendFrameThreadSafe(tx);
    if (activeTransport_ == static_cast<ICanTransport*>(&slcan_)) {
        if (!monitorPaused_) trace_.push(tx);
        canLogger_.log(tx);
    }
    recordBusBits(tx);
}

void MainUi::handleStreamStatusRsp(const StreamStatusReply& r, double now)
{
    // Consume STREAM_STATUS_RSP (previously dropped): it is direct proof the
    // motor's stream engine is answering. Keep the latest for the UI/diagnosis;
    // any watch stuck in failed_no_motor_response recovers on its next VALUE.
    lastStreamStatus_   = r;
    lastStreamStatusAt_ = now;
}

size_t MainUi::activeOrPendingStreamWatchCount() const
{
    size_t count = 0;
    for (const WatchVar& w : watches_) {
        if (!streamSupportedForNode(w.nodeId)) continue;
        if (w.streamSubscribed || w.streamPending) ++count;
    }
    return count;
}

void MainUi::streamTick(double now)
{
    if (activeTransport_ == nullptr || !activeTransport_->isOpen()) return;

    bool anySubscribed = false;
    for (size_t i = 0; i < watches_.size(); ++i) {
        WatchVar& w = watches_[i];
        if (!streamSupportedForNode(w.nodeId)) continue;

        // Motor in BOOT: its subscription is gone (bootloader has no STREAM_*).
        // Reset our slot mirror AND the recovery FSM; we re-subscribe on
        // app-mode return next tick.
        if (nodeIsInBoot(w.nodeId)) {
            if (w.streamSlotId < motorStreamSlotToWatch_.size()) {
                motorStreamSlotToWatch_[w.streamSlotId] = -1;
            }
            w.streamSubscribed = false;
            w.streamPending = false;
            w.streamSlotId = 0xFF;
            w.streamQuotaLimited = false;
            w.sw.clear();
            continue;
        }

        if (w.enabled) {
            if (w.streamQuotaLimited &&
                activeOrPendingStreamWatchCount() < kMaxLiveStreamWatches) {
                w.streamQuotaLimited = false;
                w.sw.clear();
            }
            if (w.streamQuotaLimited) {
                w.valueOk = false;
                w.valueText = "stream limit 6";
                w.nextPollTime = now + 0.5;
                continue;
            }
            // The StreamWatch FSM owns the (re)subscribe policy: exponential
            // backoff instead of a blind 2 s re-ADD, a STATUS+teardown before
            // re-adding a stale slot, and a terminal failed_no_motor_response
            // state that STOPS the flood and surfaces in the UI. Liveness
            // (onValue / onAddRsp) is fed from the RX handlers; tick() only
            // decides what to send now.
            switch (w.sw.tick(now)) {
                case StreamWatchAction::SendAdd:
                    sendStreamAddForWatch(i, now);
                    break;
                case StreamWatchAction::SendStatusThenAdd:
                    // Stale slot: probe the motor (STATUS_REQ) and tear the slot
                    // down so the re-ADD isn't rejected as a duplicate, then ADD.
                    sendStreamStatusReq(now);
                    if (w.streamSlotId != 0xFF) sendStreamRemoveForWatch(i, now);
                    sendStreamAddForWatch(i, now);
                    break;
                case StreamWatchAction::None:
                    break;
            }
            if (w.sw.state() != StreamWatchState::Subscribed) {
                w.valueOk   = false;
                w.valueText = w.sw.statusText(now);   // "resyncing"/"NO MOTOR RESPONSE…"
            }
        } else {
            if (w.streamSubscribed) sendStreamRemoveForWatch(i, now);
            if (w.sw.state() != StreamWatchState::Idle) w.sw.clear();
            w.streamQuotaLimited = false;
        }
        if (w.streamSubscribed) anySubscribed = true;
    }

    if (anySubscribed && (now - lastStreamHeartbeatAt_) >= 1.0) {
        sendStreamHeartbeat(now);
    }
}

void MainUi::pollWatchVariables(double now)
{
    if (activeTransport_ == nullptr || !activeTransport_->isOpen()) return;

    for (auto it = pendingSentAt_.begin(); it != pendingSentAt_.end();) {
        if (now - it->second <= kReadMemTimeoutSec) {
            ++it;
            continue;
        }
        const auto pendingIt = pending_.find(it->first);
        if (pendingIt != pending_.end() && pendingIt->second < watches_.size()) {
            watches_[pendingIt->second].valueText = "timeout";
        }
        pending_.erase(it->first);
        it = pendingSentAt_.erase(it);
    }

    size_t sentThisPoll = 0;
    const size_t watchCount = watches_.size();
    if (watchCount == 0) return;
    pollCursor_ %= watchCount;
    for (size_t scanned = 0; scanned < watchCount; ++scanned) {
        if (sentThisPoll >= kMaxReadMemSendsPerPoll ||
            pending_.size() >= kMaxPendingReadMem) {
            break;
        }

        const size_t i = (pollCursor_ + scanned) % watchCount;
        WatchVar& w = watches_[i];
        if (!w.enabled || now < w.nextPollTime) continue;
        // Suppress live polling for variables involved in the active
        // capture session -- their xs/ys are about to be filled by
        // CAP_DATA frames and we don't want READ_MEM jittered samples
        // mixing in.
        if (isCapturePollSuppressed(w)) {
            w.nextPollTime = now + 0.2;
            continue;
        }
        // Suppress polling while the target node is in BOOT mode. The
        // bootloader has no READ_MEM handler, so polls would just rack
        // up "timeout" entries for every variable on that node and
        // pollute the bus with unanswered traffic during a flash.
        // Auto-clears as soon as we see an APP-mode frame from the node
        // (or after kNodeModeStaleSec of silence).
        if (nodeIsInBoot(w.nodeId)) {
            w.valueText = "node in BOOT";
            w.valueOk = false;
            w.nextPollTime = now + 0.5;
            continue;
        }
        // Phase C: watch is being driven by STREAM_VALUE pushes — do NOT
        // also poll with READ_MEM. streamTick() handles ADD/REMOVE/HEARTBEAT.
        if (w.streamSubscribed) {
            w.nextPollTime = now + 1.0;  // far enough that pollWatchVariables ignores it
            continue;
        }
        if (w.streamQuotaLimited) {
            w.valueOk = false;
            w.valueText = "stream limit 6";
            w.nextPollTime = now + 0.5;
            continue;
        }
        w.pollHz = clampWatchHz(w.pollHz);

        const bool alreadyPending = std::any_of(
            pending_.begin(),
            pending_.end(),
            [i](const auto& entry) { return entry.second == i; });
        if (alreadyPending) continue;

        if (pending_.size() >= kMaxPendingReadMem) {
            w.nextPollTime = now + kReadMemRetrySec;
            break;
        }

        const uint8_t size = sizeForType(w.type);
        if (size > 4) {
            w.valueText = "double skipped";
            w.nextPollTime = now + 1.0 / w.pollHz;
            continue;
        }

        DebugReadRequest req;
        req.nodeId = w.nodeId;
        req.seq = nextSeq_++;
        req.address = w.address;
        req.size = size;
        w.seq = req.seq;

        CanFrame tx = debug_.makeReadMem(req, now);
        if (sendFrameThreadSafe(tx)) {
            const uint32_t key = pendingKey(req.nodeId, req.seq);
            pending_[key] = i;
            pendingSentAt_[key] = now;
            w.nextPollTime = now + 1.0 / w.pollHz;
            ++sentThisPoll;
            if (activeTransport_ == static_cast<ICanTransport*>(&slcan_)) {
                if (!monitorPaused_) trace_.push(tx);
                canLogger_.log(tx);
            }
            recordBusBits(tx);
        } else {
            w.valueText = "tx failed";
            w.nextPollTime = now + 0.1;
            if (!activeTransport_->isOpen()) break;
        }
    }
    pollCursor_ = (pollCursor_ + std::max<size_t>(sentThisPoll, 1)) % watchCount;
}

void MainUi::recordBusBits(const CanFrame& f)
{
    const double t = clock_.nowSeconds();
    int bits = f.extended ? 67 : 47;       // header + ack + EOF + IFS, std/ext
    bits += static_cast<int>(f.data.size()) * 8;
    if (f.fd) bits += 24;
    bits = (bits * 117) / 100;
    rateBucket_.emplace_back(t, static_cast<uint32_t>(bits));
    while (!rateBucket_.empty() && (t - rateBucket_.front().first) > 1.0) {
        rateBucket_.pop_front();
    }

    // Approximate transport bytes/sec: TCP JSON line ~ 80 chars; SLCAN ~ 30 chars.
    const uint32_t transportBytes =
        (transportKind_ == TransportKind::Slcan) ? 30u : 80u;
    transportBucket_.emplace_back(t, transportBytes);
    while (!transportBucket_.empty() && (t - transportBucket_.front().first) > 1.0) {
        transportBucket_.pop_front();
    }

    ++totalFrames_;
}

int MainUi::currentNominalBitrate() const
{
    auto rateForSlcan = [](const char* cmd) -> int {
        if (cmd == nullptr || cmd[0] == '\0') return 0;
        const char c = cmd[0];
        const int idx = (cmd[1] >= '0' && cmd[1] <= '9') ? (cmd[1] - '0') : -1;
        if ((c == 'S' || c == 's') && idx >= 0 && idx <= 8) {
            static const int rates[] = {10000, 20000, 50000, 100000, 125000,
                                        250000, 500000, 800000, 1000000};
            return rates[idx];
        }
        if ((c == 'Y' || c == 'y') && idx >= 0 && idx <= 7) {
            static const int rates[] = {125000, 250000, 500000, 1000000,
                                        2000000, 5000000, 8000000, 10000000};
            return rates[idx];
        }
        return 0;
    };
    if (transportKind_ == TransportKind::Slcan) return rateForSlcan(slcanNominal_.data());
    if (transportKind_ == TransportKind::WifiAp) return wifiApBusBitrate_;
    return tcpBusBitrate_;
}

bool MainUi::loadSymbolsInto(SymbolSource& source)
{
    std::string error;
    const std::string resolved = resolveDataPath(source.symBuf.data());

    if (symbolLoader_.loadSymbolsJson(resolved, source.symbols, error)) {
        // symbols.json doesn't specify a per-symbol node, so align all loaded
        // symbols with the source's default node -- otherwise watched/plotted
        // keys (built from src.defaultNodeId) won't match the addWatch nodeId.
        for (VariableSymbol& s : source.symbols) {
            s.nodeId = source.defaultNodeId;
        }
        source.status = "loaded " + std::to_string(source.symbols.size()) + " symbols";
        if (resolved != std::string(source.symBuf.data())) {
            source.status += " from " + resolved;
        }
        return true;
    }
    source.status = error + " (tried " + resolved + ")";
    source.symbols.clear();
    return false;
}

#ifdef _WIN32
/* Run a command line silently (no console flash, captured exit code).
 * std::system invokes cmd.exe via shell which pops a console window on
 * every call from a GUI-subsystem binary. CreateProcessW with
 * CREATE_NO_WINDOW + DETACHED_PROCESS avoids the flash; the parent
 * blocks on WaitForSingleObject so the call remains synchronous. */
static int runHidden(const std::string& commandUtf8)
{
    std::wstring cmdW;
    cmdW.reserve(commandUtf8.size());
    const int wlen = MultiByteToWideChar(CP_UTF8, 0, commandUtf8.c_str(),
                                         static_cast<int>(commandUtf8.size()),
                                         nullptr, 0);
    if (wlen > 0) {
        cmdW.resize(static_cast<size_t>(wlen));
        MultiByteToWideChar(CP_UTF8, 0, commandUtf8.c_str(),
                            static_cast<int>(commandUtf8.size()),
                            cmdW.data(), wlen);
    }
    /* CreateProcessW needs a mutable buffer for lpCommandLine. */
    std::vector<wchar_t> mutableCmd(cmdW.begin(), cmdW.end());
    mutableCmd.push_back(L'\0');

    STARTUPINFOW si{};
    si.cb = sizeof(si);
    PROCESS_INFORMATION pi{};
    const DWORD flags = CREATE_NO_WINDOW;
    const BOOL ok = CreateProcessW(nullptr, mutableCmd.data(), nullptr, nullptr,
                                   FALSE, flags, nullptr, nullptr, &si, &pi);
    if (!ok) return -1;
    WaitForSingleObject(pi.hProcess, INFINITE);
    DWORD rc = 0;
    GetExitCodeProcess(pi.hProcess, &rc);
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);
    return static_cast<int>(rc);
}
#endif

bool MainUi::exportElfAndLoadInto(SymbolSource& source)
{
    const std::string script = findElfExporter();
    const std::string python = findElfPython();
    std::string command;
    if (isStandaloneExporter(python)) {
        command = quoteArg(python) + " " +
                  quoteArg(source.elfBuf.data()) + " " + quoteArg(source.symBuf.data());
    } else {
        command = quoteArg(python) + " " + quoteArg(script) + " " +
                  quoteArg(source.elfBuf.data()) + " " + quoteArg(source.symBuf.data());
    }
    source.status = "exporting ELF...";
#ifdef _WIN32
    const int rc = runHidden(command);
#else
    const int rc = std::system(command.c_str());
#endif
    if (rc != 0) {
        /* When `python` is bare (no path), the runtime fell back to
         * PATH lookup and almost certainly didn't find python.exe on
         * the user's machine -- bundled elf_export.exe is the supported
         * path. Surface a hint pointing at the missing artefact so the
         * user knows to rebuild pc_tool (CMake refresh_symbols.cmake)
         * or copy elf_export.exe next to drivescope.exe. */
        std::string hint;
        if (python == "python") {
            hint = " вЂ” no elf_export.exe found near the .exe or under "
                   "tools/pc-tool/dist/, and python is not on PATH";
        }
        source.status = "ELF export failed, rc=" + std::to_string(rc) +
                        " (tried " + python + ")" + hint;
        return false;
    }
    return loadSymbolsInto(source);
}

void MainUi::ensureDefaultSources()
{
    if (!sources_.empty()) return;
    /* Default symbols paths are `symbols/symbols_*.json` relative to the
     * running .exe. CMake POST_BUILD lays them out at
     * `<build>/symbols/symbols_*.json`, the release bundle at
     * `DriveScope/symbols/symbols_*.json` -- one and the same scenario
     * works in both, which is what makes the DriveScope folder portable
     * to USB. ELF paths default to the canonical umbrella-relative
     * locations so a fresh checkout's Export+Load button works without
     * editing anything; the user can swap to any path via the Variables
     * tab's Browse... button. */
    SymbolSource motor;
    motor.name = "motor";
    setTextBuffer(motor.elfBuf, std::string("MOTOR/app_stm32f4_motor/Debug/app_stm32f4_motor.elf"));
    setTextBuffer(motor.symBuf, std::string("symbols/symbols_motor.json"));
    motor.defaultNodeId = 2;
    loadSymbolsInto(motor);
    sources_.push_back(std::move(motor));

    SymbolSource rk;
    rk.name = "rk";
    setTextBuffer(rk.elfBuf, std::string("RK/app_stm32l4_rk/Debug/app_stm32l4_rk.elf"));
    setTextBuffer(rk.symBuf, std::string("symbols/symbols_rk.json"));
    rk.defaultNodeId = 3;
    loadSymbolsInto(rk);
    sources_.push_back(std::move(rk));

    // mainPCB / SSD202D Linux SOM. Node 1 (MAIN) supports the same READ_MEM
    // request/response protocol as motor/RK through app_dd's local responder.
    SymbolSource dd;
    dd.name = "dd";
    setTextBuffer(dd.elfBuf,
                  std::string("//wsl.localhost/Ubuntu_DDV2/home/fermiums/linux_ssd202d_dd/sdk/app_ssd202d_ddv2/build/app_dd"));
    setTextBuffer(dd.symBuf, std::string("symbols/symbols_dd.json"));
    dd.defaultNodeId = 1;
    loadSymbolsInto(dd);
    sources_.push_back(std::move(dd));

    // ESP32 on the mainPCB. Connected to SSD202D over UART, NOT directly on CAN.
    // app_dd currently returns READ_MEM status=2 for node 4 until the UART proxy
    // and ESP32-side memory responder are implemented.
    SymbolSource esp32;
    esp32.name = "esp32";
    setTextBuffer(esp32.elfBuf,
                  std::string("mainPCB/app_esp32_dd/.pio/build/esp32dev/firmware.elf"));
    setTextBuffer(esp32.symBuf, std::string("symbols/symbols_esp32.json"));
    esp32.defaultNodeId = 4;
    loadSymbolsInto(esp32);
    sources_.push_back(std::move(esp32));
}

MainUi::SymbolSource& MainUi::ensureSourceForPath(const std::string& symbolsPath)
{
    ensureDefaultSources();
    namespace fs = std::filesystem;
    const std::string base = fs::path(symbolsPath).filename().string();
    for (SymbolSource& s : sources_) {
        if (std::string(s.symBuf.data()) == symbolsPath) return s;
    }
    for (SymbolSource& s : sources_) {
        if (!s.name.empty() && base.find(s.name) != std::string::npos &&
            std::string(s.symBuf.data()).empty()) {
            return s;
        }
    }
    for (SymbolSource& s : sources_) {
        if (!s.name.empty() && base.find(s.name) != std::string::npos) {
            return s;
        }
    }
    SymbolSource ns;
    ns.name = deriveSourceName(symbolsPath);
    sources_.push_back(std::move(ns));
    return sources_.back();
}

bool MainUi::resolveWatchConfig(const WatchConfig& watch, VariableSymbol& symbol, std::string& sourceName) const
{
    // Always prefer the live symbols table over the saved address. The
    // scenario file caches the address from a previous build, but rebuilding
    // any peripheral can shift BSS layout -- saved values then point at zeroed
    // RAM and reads come back as 0. Looking up by name re-anchors to whatever
    // the freshly-refreshed symbols_*.json says is current.
    const std::string& wanted = watch.symbolName.empty() ? watch.name : watch.symbolName;
    if (!wanted.empty()) {
        for (const SymbolSource& src : sources_) {
            for (const VariableSymbol& candidate : src.symbols) {
                if (candidate.name == wanted) {
                    symbol = candidate;
                    symbol.nodeId = watch.nodeId;
                    if (!watch.name.empty()) symbol.name = watch.name;
                    sourceName = src.name;
                    return true;
                }
            }
        }
    }

    // Symbol not found (variable was renamed / source not loaded). Fall back
    // to the saved address so the watch still works on a best-effort basis.
    if (!watch.hasAddress) return false;

    symbol.name = watch.name.empty() ? watch.symbolName : watch.name;
    symbol.address = watch.address;
    symbol.type = watch.type;
    symbol.size = watch.size;
    symbol.nodeId = watch.nodeId;
    sourceName = "config";
    return symbol.address != 0 && !symbol.name.empty();
}

void MainUi::drawConnection()
{
    auto setTransport = [&](TransportKind newKind) {
        if (newKind == transportKind_) return;
        if (activeTransport_ != nullptr) {
            activeTransport_->close();
            activeTransport_ = nullptr;
        }
        transportKind_ = newKind;

        // Wipe per-connection state so the new transport starts clean. Without
        // this, lingering READ_MEM seqs from the old transport poison the new
        // one (replies get matched to old watches). resetTransportRuntimeState()
        // also drops STREAM slots so the next TCP/Manual connect re-subscribes.
        resetTransportRuntimeState();
        // Stale rate/load EMAs would make the status bar lie until the next
        // bucket window rolls over, so clear those with the transport switch.
        rateBucket_.clear();
        transportBucket_.clear();
        canLoadEma_  = 0.0f;
        netKbpsEma_  = 0.0f;
        // Clear pending POST values on watch list so we don't show old "ok"
        // states next to silent variables.
        for (auto& w : watches_) {
            w.valueOk = false;
            w.valueText = "n/a";
        }

        markScenarioDirty("transport changed");
    };

    auto transportRadio = [&](const char* label, TransportKind kind) {
        if (ImGui::RadioButton(label, transportKind_ == kind)) setTransport(kind);
    };

    auto drawSlcanForm = [&]() {
        if (ImGui::BeginTable("##slcan-form", 2,
                              ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_PadOuterX,
                              ImVec2(430.0f, 0.0f))) {
            ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed, 128.0f);
            ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthFixed, 286.0f);
            auto row = [&](const char* label) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::TextDisabled("%s", label);
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-1);
            };

            row("COM port");
            if (ImGui::InputText("##com-port", comPort_.data(), comPort_.size())) {
                markScenarioDirty("connection changed");
            }

            row("Serial baud");
            const int bauds[] = {115200, 230400, 460800, 500000, 921600,
                                 1000000, 2000000, 3000000, 4000000, 8000000};
            const char* labels[] = {"115200", "230400", "460800", "500000", "921600",
                                    "1000000", "2000000", "3000000", "4000000", "8000000"};
            int idx = static_cast<int>(IM_ARRAYSIZE(bauds)) - 1;
            for (int i = 0; i < IM_ARRAYSIZE(bauds); ++i) {
                if (bauds[i] == slcanBaud_) { idx = i; break; }
            }
            if (ImGui::Combo("##baud", &idx, labels, IM_ARRAYSIZE(labels))) {
                slcanBaud_ = bauds[idx];
                markScenarioDirty("connection changed");
            }
            ImGui::TextDisabled("USB-CDC may ignore this value.");

            row("Nominal cmd");
            if (ImGui::InputText("##nominal-cmd", slcanNominal_.data(), slcanNominal_.size())) {
                markScenarioDirty("connection changed");
            }

            row("FD data cmd");
            if (ImGui::InputText("##data-cmd", slcanData_.data(), slcanData_.size())) {
                markScenarioDirty("connection changed");
            }

            row("Listen mode");
            if (ImGui::Checkbox("Silent only", &slcanSilent_)) {
                markScenarioDirty("connection changed");
            }

            ImGui::EndTable();
        }
    };

    // STA and AP transports both connect TCP to a Kitchen Machine that
    // announces itself via UDP discovery. Same form for both -- only the
    // expected IP subnet differs (STA: office network, AP: 192.168.1.x while
    // laptop is joined to the device's own hotspot). One dropdown, no manual
    // IP/port/bitrate. Auto-rescan on a 4-second timer while the panel is
    // visible -- no Scan button.
    auto drawDiscoveryForm = [&](const char* tableId, bool isAp) {
        const bool connected = isConnected();
        // Auto-rescan tick -- only while DISCONNECTED. Once a transport is open
        // there is nothing useful to discover; pc_tool was burning CPU + UDP
        // broadcasts every 4 s "for nothing", and the dropdown text "Status:
        // scanning..." was confusing right next to a green "Connected" badge.
        const double now = ImGui::GetTime();
        // Also gate UDP-broadcast discovery on no-deploy: 4-byte broadcast
        // packets aren't huge but every Wi-Fi side-effect during the upload
        // matters when the SSH stream is sensitive to channel hiccups.
        if (!connected && !mainDeployRunning_ &&
            !discoverScanning_ && now >= discoverNextScanAt_) {
            discoverNextScanAt_ = now + 4.0;
            scanDiscoveryAsync();
        }

        // Wider value column so "Kitchen_Machine_XXXX - 192.168.0.103 (STA)"
        // fits inside the combo box without the arrow appearing to "drift" mid-text.
        if (ImGui::BeginTable(tableId, 2,
                              ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_PadOuterX,
                              ImVec2(720.0f, 0.0f))) {
            ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed, 128.0f);
            ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthFixed, 580.0f);
            auto row = [&](const char* label) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::TextDisabled("%s", label);
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-1);
            };

            // Filter out devices not matching our expected subnet for the
            // current transport: STA = anything that is NOT 192.168.1.x; AP =
            // only 192.168.1.x.
            std::vector<int> visibleIdx;
            visibleIdx.reserve(discovered_.size());
            for (size_t i = 0; i < discovered_.size(); ++i) {
                const auto& d = discovered_[i];
                bool isApIp = d.ip.rfind("192.168.1.", 0) == 0;
                if (isAp && isApIp) visibleIdx.push_back((int)i);
                else if (!isAp && !isApIp) visibleIdx.push_back((int)i);
            }

            row("Device");
            std::vector<std::string> labels;
            labels.reserve(visibleIdx.size());
            for (int i : visibleIdx) {
                const auto& d = discovered_[i];
                std::string l = d.name.empty() ? std::string("(unnamed)") : d.name;
                l += "   ";
                l += d.ip;
                if (!d.mode.empty()) { l += " ("; l += d.mode; l += ")"; }
                labels.push_back(std::move(l));
            }
            std::vector<const char*> labelPtrs;
            labelPtrs.reserve(labels.size());
            for (const auto& s : labels) labelPtrs.push_back(s.c_str());

            if (visibleIdx.empty()) {
                ImGui::TextDisabled(discoverScanning_
                                        ? "(scanning network for Kitchen_Machine_* devices...)"
                                        : "(no devices yet - auto-scan in progress)");
            } else {
                bool selectedVisible = false;
                for (int i : visibleIdx) {
                    if (i == discoveredSelected_) {
                        selectedVisible = true;
                        break;
                    }
                }
                if (!selectedVisible) {
                    selectDiscoveredTcpTarget(isAp, /*markDirty=*/false);
                }

                int localSel = 0;
                for (size_t k = 0; k < visibleIdx.size(); ++k) {
                    if (visibleIdx[k] == discoveredSelected_) { localSel = (int)k; break; }
                }
                if (connected) ImGui::BeginDisabled();
                if (ImGui::Combo("##discovery-device", &localSel,
                                 labelPtrs.data(),
                                 static_cast<int>(labelPtrs.size()))) {
                    discoveredSelected_ = visibleIdx[localSel];
                    const auto& d = discovered_[discoveredSelected_];
                    if (isAp) {
                        std::snprintf(wifiApSsid_.data(), wifiApSsid_.size(), "%s", d.name.c_str());
                        std::snprintf(wifiApHost_.data(), wifiApHost_.size(), "%s", d.ip.c_str());
                        wifiApPort_ = d.tcpPort > 0 ? d.tcpPort : 45333;
                    } else {
                        std::snprintf(tcpHost_.data(), tcpHost_.size(), "%s", d.ip.c_str());
                        tcpPort_ = d.tcpPort > 0 ? d.tcpPort : 45333;
                    }
                    markScenarioDirty("connection changed");
                }
                if (connected) ImGui::EndDisabled();
            }

            ImGui::EndTable();
        }

        // Status line -- when CONNECTED, just confirm what we're talking to and
        // stop pretending we're still scanning. When disconnected, show whatever
        // discoverStatus_ says ("found N", "no devices found...", etc.).
        if (connected) {
            const char* host = isAp ? wifiApHost_.data() : tcpHost_.data();
            const int   port = isAp ? wifiApPort_ : tcpPort_;
            ImGui::TextColored(ImVec4(0.55f, 0.85f, 0.55f, 1.0f),
                               "Connected to %s:%d", host, port);
        } else if (!discoverStatus_.empty()) {
            ImGui::TextDisabled("Status: %s", discoverStatus_.c_str());
        }
        if (!connected && !isAp &&
            discoveredSelected_ >= 0 &&
            discoveredSelected_ < static_cast<int>(discovered_.size())) {
            const auto& d = discovered_[discoveredSelected_];
            const bool apIp = d.ip.rfind("192.168.1.", 0) == 0;
            if (!apIp) {
                ImGui::TextDisabled("Target from discovery: %s:%d",
                                    d.ip.c_str(),
                                    d.tcpPort > 0 ? d.tcpPort : 45333);
            }
        }

        if (isAp) {
            ImGui::TextDisabled("AP mode - laptop must be joined to the device's");
            ImGui::TextDisabled("Kitchen_Machine_* hotspot (password: bork2025).");
        } else {
            ImGui::TextDisabled("STA mode - device on office Wi-Fi. UDP :17557 broadcast,");
            ImGui::TextDisabled("filtered to Kitchen_Machine_* announcements.");
        }
    };

    auto drawTcpForm = [&]() { drawDiscoveryForm("##sta-form", /*isAp=*/false); };

    // AP form: enumerates *visible* Wi-Fi SSIDs starting with "Kitchen_Machine_"
    // via the Windows WLAN API. Picking + Connect triggers WlanConnect with the
    // baked-in WPA2-Personal password and, on success, auto-opens the CAN-TCP
    // bridge at 192.168.1.1:45333. UDP discovery is *not* useful here -- the
    // laptop is not on the device's subnet until after WlanConnect.
    auto drawWifiApForm = [&]() {
        const bool connected = isConnected();
        const double now = ImGui::GetTime();
        // Skip Wi-Fi scan when already connected -- no point burning WlanScan
        // requests at 4 s cadence after the bridge is up.
        // Also pause scans while a recovery upload is running -- see the
        // matching note in drawDeviceUpdates(). WlanScan briefly takes
        // the radio off-channel and rips the SSH stream apart.
        if (!connected && !mainDeployRunning_ &&
            !wlanScanning_ && !wlanConnecting_ && now >= wlanNextScanAt_) {
            wlanNextScanAt_ = now + 4.0;
            scanWlanAsync();
        }

        if (ImGui::BeginTable("##ap-form", 2,
                              ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_PadOuterX,
                              ImVec2(720.0f, 0.0f))) {
            ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed, 128.0f);
            ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthFixed, 580.0f);
            auto row = [&](const char* label) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::TextDisabled("%s", label);
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-1);
            };

            row("Hotspot");
            std::vector<std::string> labels;
            labels.reserve(wlanScan_.size());
            for (const auto& w : wlanScan_) {
                char buf[160];
                std::snprintf(buf, sizeof(buf), "%s%s   (signal %d%%)",
                              w.ssid.c_str(),
                              w.connected ? " - connected" : "",
                              w.signal_pct);
                labels.push_back(buf);
            }
            std::vector<const char*> labelPtrs;
            labelPtrs.reserve(labels.size());
            for (const auto& s : labels) labelPtrs.push_back(s.c_str());

            if (wlanScan_.empty()) {
                ImGui::TextDisabled(wlanScanning_
                                        ? "(scanning Wi-Fi for Kitchen_Machine_*...)"
                                        : "(no Kitchen_Machine_* SSIDs visible)");
            } else {
                int sel = wlanSelected_;
                if (sel < 0 || sel >= (int)wlanScan_.size()) {
                    // Default to the already-connected entry, if any.
                    for (size_t i = 0; i < wlanScan_.size(); ++i) {
                        if (wlanScan_[i].connected) { sel = (int)i; break; }
                    }
                    if (sel < 0) sel = 0;
                }
                if (connected) ImGui::BeginDisabled();
                if (ImGui::Combo("##wlan-ssid", &sel,
                                 labelPtrs.data(),
                                 static_cast<int>(labelPtrs.size()))) {
                    wlanSelected_ = sel;
                }
                if (wlanSelected_ < 0 && sel >= 0) wlanSelected_ = sel;
                if (connected) ImGui::EndDisabled();
            }

            ImGui::EndTable();
        }

        if (connected) {
            ImGui::TextColored(ImVec4(0.55f, 0.85f, 0.55f, 1.0f),
                               "Connected to %s:%d", wifiApHost_.data(), wifiApPort_);
        } else if (!wlanStatus_.empty()) {
            ImGui::TextDisabled("Status: %s", wlanStatus_.c_str());
        }

        ImGui::TextDisabled("AP mode - pc_tool will join the device's hotspot");
        ImGui::TextDisabled("(WPA2/AES, password baked in: bork2025) and open TCP");
        ImGui::TextDisabled("CAN at 192.168.1.1:45333 automatically.");
    };

    // Manual mode: type host:port. No discovery, no filter -- sometimes the
    // device is on a network where neither STA-discovery nor AP-mode reaches
    // (e.g. a managed switch with broadcast off + unknown IP), and the
    // engineer reads it from the device label or DHCP lease.
    auto drawManualForm = [&]() {
        if (ImGui::BeginTable("##manual-form", 2,
                              ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_PadOuterX,
                              ImVec2(430.0f, 0.0f))) {
            ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed, 128.0f);
            ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthFixed, 286.0f);
            auto row = [&](const char* label) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::TextDisabled("%s", label);
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-1);
            };

            row("Host");
            if (ImGui::InputText("##manual-host", tcpHost_.data(), tcpHost_.size())) {
                markScenarioDirty("connection changed");
            }

            row("TCP port");
            if (ImGui::InputInt("##manual-port", &tcpPort_)) {
                markScenarioDirty("connection changed");
            }

            ImGui::EndTable();
        }
        ImGui::TextDisabled("Use this when the device is on an exotic network");
        ImGui::TextDisabled("and you've read its IP off the engineer screen.");
    };

    auto drawSettings = [&]() {
        ImGui::TextDisabled("Transport");
        transportRadio("SLCAN COM", TransportKind::Slcan);
        ImGui::SameLine(0.0f, 14.0f);
        transportRadio("Wifi STA", TransportKind::Tcp);
        ImGui::SameLine(0.0f, 14.0f);
        transportRadio("Wifi AP", TransportKind::WifiAp);
        ImGui::SameLine(0.0f, 14.0f);
        transportRadio("Wifi Manual", TransportKind::Manual);
        ImGui::SameLine(0.0f, 24.0f);
        const bool connected = isConnected();
        // Gate Connect when there's nothing to connect to: in WifiAp mode no
        // visible/selected hotspot; in STA mode no UDP-discovered Kitchen_Machine_*
        // on the office subnet. Without these, connectSelectedTransport would
        // burn the 3 s TCP timeout against the placeholder default IP and feel
        // like a hang. Manual + SLCAN always allowed (operator picked the host).
        int visibleStaCount = 0;
        for (const auto& d : discovered_) {
            if (d.ip.rfind("192.168.1.", 0) != 0) ++visibleStaCount;
        }
        const bool apHasTarget = transportKind_ != TransportKind::WifiAp ||
                                 (!wlanScan_.empty() &&
                                  wlanSelected_ >= 0 &&
                                  wlanSelected_ < (int)wlanScan_.size());
        const bool staHasTarget = transportKind_ != TransportKind::Tcp ||
                                  visibleStaCount > 0;
        const bool busy = wlanConnecting_;
        const bool buttonDisabled = !connected && (busy || !apHasTarget || !staHasTarget);
        const char* connectLabel =
            connected ? "Disconnect"
            : busy    ? "Connecting..."
                      : "Connect";
        if (buttonDisabled) ImGui::BeginDisabled();
        if (ImGui::Button(connectLabel, ImVec2(128.0f, 28.0f))) {
            if (connected) disconnectTransport();
            else           connectSelectedTransport();
        }
        if (buttonDisabled) ImGui::EndDisabled();
        if (ImGui::IsItemHovered()) {
            if (!apHasTarget) {
                ImGui::SetTooltip("Pick a Kitchen_Machine_* hotspot from the dropdown first.");
            } else if (!staHasTarget) {
                ImGui::SetTooltip("Wait for a Kitchen_Machine_* device to be discovered on the network first.");
            } else if (busy) {
                ImGui::SetTooltip("Joining Wi-Fi hotspot, then opening TCP CAN bridge...");
            } else {
                ImGui::SetTooltip("%s using the selected transport settings.",
                                  connected ? "Disconnect" : "Connect");
            }
        }

        // Surface transport status next to the Connect button. Without this a
        // failed open() ("connect failed (err 10060) (no on-link NIC)" when
        // the laptop isn't joined to the device's Wi-Fi, VPN-stolen route,
        // port mismatch, etc) is invisible: the button stays "Connect" and
        // the click looks like it did nothing.
        if (activeTransport_ != nullptr) {
            ImGui::SameLine(0.0f, 12.0f);
            const std::string& s = activeTransport_->status();
            if (!s.empty()) {
                ImVec4 col = (s.find("failed") != std::string::npos ||
                              s.find("closed") != std::string::npos ||
                              s.find("error")  != std::string::npos)
                             ? ImVec4(0.95f, 0.55f, 0.15f, 1.0f)
                             : ImVec4(0.55f, 0.85f, 0.55f, 1.0f);
                ImGui::TextColored(col, "%s", s.c_str());
            }
        }

#ifdef _WIN32
        // Pre-flight: in Manual mode on an AP IP (192.168.1.x), if the laptop
        // is not currently joined to a Kitchen_Machine_* hotspot, no on-link
        // NIC will match and TCP open will fail. Tell the user before the
        // click, with a hint to use the Wifi AP tab for one-click join.
        if (!connected && transportKind_ == TransportKind::Manual &&
            std::strncmp(tcpHost_.data(), "192.168.1.", 10) == 0)
        {
            const std::string cur = drivescope::wlan::currentSsid();
            if (cur.rfind("Kitchen_Machine_", 0) != 0) {
                ImGui::TextColored(ImVec4(0.95f, 0.55f, 0.15f, 1.0f),
                    "Note: 192.168.1.x = device AP. Join Kitchen_Machine_*"
                    " (Wi-Fi tray) first, or use the Wifi AP tab to auto-join.");
            }
        }
#endif

        ImGui::Dummy(ImVec2(0.0f, 8.0f));
        if      (transportKind_ == TransportKind::Slcan)  drawSlcanForm();
        else if (transportKind_ == TransportKind::WifiAp) drawWifiApForm();
        else if (transportKind_ == TransportKind::Manual) drawManualForm();
        else                                              drawTcpForm();
    };

    auto statusRow = [](const char* key, const char* value, bool muted = false) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("%s", key);
        ImGui::TableNextColumn();
        if (muted) ImGui::TextDisabled("%s", value);
        else       ImGui::TextWrapped("%s", value);
    };

    auto drawStatus = [&]() {
        const ICanTransport* shown = activeTransport_ != nullptr ? activeTransport_ : selectedTransport();
        const bool connected = activeTransport_ != nullptr && activeTransport_->isOpen();
        ImGui::TextDisabled("Status");
        const ImVec4 dot = connected ? ImVec4(0.45f, 0.90f, 0.35f, 1.0f)
                                     : ImVec4(0.90f, 0.35f, 0.35f, 1.0f);
        ImGui::ColorButton("##conn-dot", dot,
                           ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoDragDrop,
                           ImVec2(11.0f, 11.0f));
        ImGui::SameLine();
        ImGui::TextUnformatted(connected ? "Connected" : "Disconnected");

        ImGui::Dummy(ImVec2(0.0f, 8.0f));
        if (ImGui::BeginTable("##connection-status", 2,
                              ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_PadOuterX,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TableSetupColumn("key", ImGuiTableColumnFlags_WidthFixed, 96.0f);
            ImGui::TableSetupColumn("value", 0, 1.0f);
            statusRow("Transport", shown->status().c_str());
            char framesBuf[48];
            std::snprintf(framesBuf, sizeof(framesBuf), "%zu", trace_.size());
            statusRow("Frames", framesBuf);
            if (!scenarioPath_.empty()) statusRow("Scenario", scenarioPath_.c_str());
            if (!scenarioStatus_.empty()) statusRow("Scenario IO", scenarioStatus_.c_str(), true);
            if (!lastFileMsg_.empty()) statusRow("File", lastFileMsg_.c_str(), true);
            ImGui::EndTable();
        }
    };

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(14.0f, 12.0f));
    ImGui::BeginChild("##connection-body", ImVec2(0.0f, 0.0f),
                      ImGuiChildFlags_AlwaysUseWindowPadding);

    const float availW = ImGui::GetContentRegionAvail().x;
    const bool wide = availW >= 880.0f;
    if (wide) {
        const float leftW = std::clamp(availW * 0.42f, 430.0f, 540.0f);
        ImGui::BeginChild("##connection-left", ImVec2(leftW, 0.0f), ImGuiChildFlags_None);
        drawSettings();
        ImGui::EndChild();
        ImGui::SameLine(0.0f, 26.0f);
        ImGui::BeginChild("##connection-right", ImVec2(0.0f, 0.0f), ImGuiChildFlags_None);
        drawStatus();
        ImGui::EndChild();
    } else {
        drawSettings();
        ImGui::Separator();
        drawStatus();
    }

    ImGui::EndChild();
    ImGui::PopStyleVar();
}

namespace {

void inputClipboardMenu(char* buf, size_t bufSize)
{
    if (ImGui::BeginPopupContextItem()) {
        const bool empty = (buf[0] == '\0');
        if (ImGui::MenuItem("Cut", nullptr, false, !empty)) {
            ImGui::SetClipboardText(buf);
            buf[0] = '\0';
        }
        if (ImGui::MenuItem("Copy", nullptr, false, !empty)) {
            ImGui::SetClipboardText(buf);
        }
        if (ImGui::MenuItem("Paste")) {
            const char* clip = ImGui::GetClipboardText();
            if (clip) {
                std::strncpy(buf, clip, bufSize - 1);
                buf[bufSize - 1] = '\0';
            }
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Clear", nullptr, false, !empty)) {
            buf[0] = '\0';
        }
        ImGui::EndPopup();
    }
}

struct AggRow {
    uint32_t id = 0;
    bool extended = false;
    bool fd = false;
    bool brs = false;
    bool rx = false;
    uint8_t dlc = 0;
    std::vector<uint8_t> data;
    std::string parsed;
    double lastTs = 0.0;
    double periodMs = 0.0;
    size_t count = 0;
    std::vector<uint8_t> firstBytes;
    std::vector<uint8_t> everChanged;
};

struct FrameTags {
    std::string mode;
    std::string src;
    std::string dst;
    std::string flow;
};

inline FrameTags extractFrameTags(const CanFrame& f)
{
    FrameTags t;
    auto label = [](uint8_t n) -> std::string {
        switch (n) {
            case 0x01: return "MAIN";
            case 0x02: return "MOTOR";
            case 0x03: return "RK";
            case 0x04: return "ESP32";
            case 0x10: return "PC";
            case 0xFF: return "BROADCAST";
            default: return "?";
        }
    };
    if (f.extended) {
        const uint8_t srcN = static_cast<uint8_t>(f.id & 0xFF);
        const uint8_t dstN = static_cast<uint8_t>((f.id >> 8) & 0xFF);
        const uint8_t mod = static_cast<uint8_t>((f.id >> 28) & 0x01);
        t.mode = mod ? "APP" : "BOOT";
        t.src = label(srcN);
        t.dst = label(dstN);
        t.flow = t.src + "->" + t.dst;
    } else {
        t.mode = "STD";
    }
    return t;
}

std::string buildRowCsv(const CanFrame& f, const std::string& parsed)
{
    std::ostringstream os;
    os << std::fixed;
    os.precision(6);
    os << f.timestamp << ',' << (f.rx ? "RX" : "TX") << ','
       << idToHex(f.id, f.extended) << ','
       << (f.extended ? 1 : 0) << ',' << (f.fd ? 1 : 0) << ',' << (f.brs ? 1 : 0)
       << ',' << static_cast<int>(f.dlc) << ',' << bytesToHex(f.data) << ','
       << '"';
    for (char c : parsed) {
        if (c == '"') os << "\"\"";
        else os << c;
    }
    os << '"';
    return os.str();
}

bool containsCi(const std::string& haystack, const char* needle)
{
    if (needle == nullptr || *needle == '\0') return true;
    auto tolower_ascii = [](char c) {
        if (c >= 'A' && c <= 'Z') return static_cast<char>(c + ('a' - 'A'));
        return c;
    };
    const size_t hl = haystack.size();
    size_t nl = 0;
    while (needle[nl]) ++nl;
    if (nl == 0) return true;
    if (nl > hl) return false;
    for (size_t i = 0; i + nl <= hl; ++i) {
        size_t j = 0;
        for (; j < nl; ++j) {
            if (tolower_ascii(haystack[i + j]) != tolower_ascii(needle[j])) break;
        }
        if (j == nl) return true;
    }
    return false;
}

struct FilterRequest {
    bool present = false;
    bool blank = false;
    bool exclude = false;
    drivescope::FilterField field = drivescope::FilterField::Id;
    uint32_t idValue = 0;
    bool extended = false;
    std::string text;
};

void renderDecodeTableBody(const CanFrame& f, bool selectableValues)
{
    const std::vector<TooltipField> fields = buildFrameTooltipTable(f);
    const std::string parsed = decodeCanFrame(f);

    auto pushCellStyle = []() {
        ImGui::PushStyleColor(ImGuiCol_FrameBg,        ImVec4(0,0,0,0));
        ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(1,1,1,0.04f));
        ImGui::PushStyleColor(ImGuiCol_FrameBgActive,  ImVec4(0.035f,0.278f,0.443f,1.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(2, 1));
        ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
    };
    auto popCellStyle = []() {
        ImGui::PopStyleVar(2);
        ImGui::PopStyleColor(3);
    };

    auto selText = [&](const char* id, const std::string& text) {
        static thread_local char buf[2048];
        const size_t n = std::min(text.size(), sizeof(buf) - 1);
        std::memcpy(buf, text.data(), n);
        buf[n] = '\0';
        if (selectableValues) {
            pushCellStyle();
            ImGui::SetNextItemWidth(-1);
            ImGui::InputText(id, buf, sizeof(buf), ImGuiInputTextFlags_ReadOnly);
            popCellStyle();
        } else {
            ImGui::TextUnformatted(text.c_str());
        }
    };

    // Parsed summary header (one-liner). In pinned-panel mode it's a read-only
    // multiline input so user can drag-select / Ctrl+C.
    if (selectableValues) {
        static thread_local char pbuf[1024];
        const size_t pn = std::min(parsed.size(), sizeof(pbuf) - 1);
        std::memcpy(pbuf, parsed.data(), pn);
        pbuf[pn] = '\0';
        pushCellStyle();
        ImGui::SetNextItemWidth(-1);
        ImGui::InputText("##parsed", pbuf, sizeof(pbuf), ImGuiInputTextFlags_ReadOnly);
        popCellStyle();
    } else {
        ImGui::PushTextWrapPos(620.0f);
        ImGui::TextUnformatted(parsed.c_str());
        ImGui::PopTextWrapPos();
    }
    ImGui::Separator();

    // Frame metadata table -- shown first so user can grab timestamp / id / DLC etc.
    const ImGuiTableFlags tflags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                   ImGuiTableFlags_SizingStretchProp;
    if (ImGui::BeginTable("##meta", 2, tflags)) {
        ImGui::TableSetupColumn("field", 0, 1.0f);
        ImGui::TableSetupColumn("value", 0, 2.4f);
        ImGui::TableHeadersRow();
        char tbuf[64];
        std::snprintf(tbuf, sizeof(tbuf), "%.6f s", f.timestamp);
        const std::pair<const char*, std::string> rows[] = {
            {"timestamp", tbuf},
            {"direction", f.rx ? "RX" : "TX"},
            {"id",        idToHex(f.id, f.extended)},
            {"extended",  f.extended ? "1" : "0"},
            {"FD",        f.fd ? "1" : "0"},
            {"BRS",       f.brs ? "1" : "0"},
            {"DLC",       std::to_string(static_cast<int>(f.dlc))},
            {"raw data",  bytesToHexSpaced(f.data)},
        };
        int idx = 0;
        for (const auto& kv : rows) {
            ImGui::TableNextRow();
            ImGui::PushID(idx++);
            ImGui::TableNextColumn(); selText("##k", kv.first);
            ImGui::TableNextColumn(); selText("##v", kv.second);
            ImGui::PopID();
        }
        ImGui::EndTable();
    }

    if (fields.empty()) {
        ImGui::Spacing();
        ImGui::TextDisabled("(no per-field decoder for this frame type)");
        return;
    }

    ImGui::Spacing();
    if (ImGui::BeginTable("##decode", 4, tflags)) {
        ImGui::TableSetupColumn("position", 0, 0.9f);
        ImGui::TableSetupColumn("size",     0, 1.1f);
        ImGui::TableSetupColumn("description", 0, 2.0f);
        ImGui::TableSetupColumn("value",    0, 0.9f);
        ImGui::TableHeadersRow();

        const ImU32 hi = IM_COL32(0x6E, 0x53, 0x2A, 0xA0);
        int rowIdx = 0;
        for (const TooltipField& tf : fields) {
            ImGui::TableNextRow();
            if (tf.active) ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, hi);
            ImGui::PushID(rowIdx++);
            ImGui::TableNextColumn(); selText("##p", tf.pos);
            ImGui::TableNextColumn(); selText("##s", tf.size);
            ImGui::TableNextColumn(); selText("##d", tf.desc);
            ImGui::TableNextColumn(); selText("##v", tf.value);
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
}

void renderTooltipDecodeTable(const CanFrame& f)
{
    ImGui::SetNextWindowSize(ImVec2(660.0f, 0.0f), ImGuiCond_Always);
    ImGui::BeginTooltip();
    renderDecodeTableBody(f, false);
    ImGui::EndTooltip();
}

FilterRequest renderRowMenuBody(const CanFrame& f, const std::string& parsed, const FrameTags& tags)
{
    FilterRequest req;
    {
        if (ImGui::MenuItem("Copy ID")) {
            ImGui::SetClipboardText(idToHex(f.id, f.extended).c_str());
        }
        if (ImGui::MenuItem("Copy data hex")) {
            ImGui::SetClipboardText(bytesToHexSpaced(f.data).c_str());
        }
        if (ImGui::MenuItem("Copy parsed")) {
            ImGui::SetClipboardText(parsed.c_str());
        }
        if (ImGui::MenuItem("Copy row as CSV")) {
            ImGui::SetClipboardText(buildRowCsv(f, parsed).c_str());
        }
        ImGui::Separator();
        const std::string idStr = idToHex(f.id, f.extended);
        if (ImGui::BeginMenu("Filter")) {
            std::string label;
            label = "Include ID " + idStr;
            if (ImGui::MenuItem(label.c_str())) {
                req.present = true; req.exclude = false;
                req.field = drivescope::FilterField::Id;
                req.idValue = f.id; req.extended = f.extended;
            }
            label = "Exclude ID " + idStr;
            if (ImGui::MenuItem(label.c_str())) {
                req.present = true; req.exclude = true;
                req.field = drivescope::FilterField::Id;
                req.idValue = f.id; req.extended = f.extended;
            }
            ImGui::Separator();
            if (!tags.flow.empty() && tags.flow != "REQ" && tags.flow != "RSP") {
                label = "Include flow " + tags.flow;
                if (ImGui::MenuItem(label.c_str())) {
                    req.present = true; req.exclude = false;
                    req.field = drivescope::FilterField::Endpoint;
                    req.text = tags.flow;
                }
                label = "Exclude flow " + tags.flow;
                if (ImGui::MenuItem(label.c_str())) {
                    req.present = true; req.exclude = true;
                    req.field = drivescope::FilterField::Endpoint;
                    req.text = tags.flow;
                }
            }
            if (!tags.src.empty() && tags.src != "?") {
                label = "Include any from " + tags.src;
                if (ImGui::MenuItem(label.c_str())) {
                    req.present = true; req.exclude = false;
                    req.field = drivescope::FilterField::Endpoint;
                    req.text = tags.src;
                }
            }
            if (!tags.dst.empty() && tags.dst != "?") {
                label = "Include any to " + tags.dst;
                if (ImGui::MenuItem(label.c_str())) {
                    req.present = true; req.exclude = false;
                    req.field = drivescope::FilterField::Endpoint;
                    req.text = tags.dst;
                }
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Add blank rule (edit later)")) {
                req.present = true; req.blank = true;
            }
            ImGui::EndMenu();
        }
    }
    return req;
}

} // namespace

namespace {

bool ruleMatches(const CanFilterRule& r, const CanFrame& f, const FrameTags& tags, const std::string& parsed)
{
    switch (r.field) {
        case FilterField::Id:
            return f.id == r.idValue && f.extended == r.extended;
        case FilterField::Substr:
            return r.textBuf[0] != '\0' && containsCi(parsed, r.textBuf.data());
        case FilterField::Endpoint: {
            const char* needle = r.textBuf.data();
            if (needle[0] == '\0') return true;
            const std::string n(needle);
            const size_t arrowR = n.find("->");
            const size_t arrowB = n.find("<->");
            if (arrowB != std::string::npos) {
                const std::string a = n.substr(0, arrowB);
                const std::string b = n.substr(arrowB + 3);
                return (tags.src == a && tags.dst == b) || (tags.src == b && tags.dst == a);
            }
            if (arrowR != std::string::npos) {
                const std::string a = n.substr(0, arrowR);
                const std::string b = n.substr(arrowR + 2);
                return tags.src == a && tags.dst == b;
            }
            return tags.src == n || tags.dst == n;
        }
        case FilterField::Direction:
            return (r.dirValue == 0) ? f.rx : !f.rx;
    }
    return false;
}

bool framePassesRules(const CanFrame& f, const FrameTags& tags, const std::string& parsed,
                      const std::vector<CanFilterRule>& rules)
{
    bool hasInclude = false;
    bool anyInclude = false;
    for (const CanFilterRule& r : rules) {
        if (!r.enabled) continue;
        const bool matches = ruleMatches(r, f, tags, parsed);
        if (r.mode == FilterMode::Exclude && matches) return false;
        if (r.mode == FilterMode::Include) {
            hasInclude = true;
            if (matches) anyInclude = true;
        }
    }
    if (hasInclude && !anyInclude) return false;
    return true;
}

ImVec4 ageColor(double age, const ImVec4& base)
{
    float alpha = 1.0f;
    if (age > 0.5) {
        alpha = static_cast<float>(1.0 - (age - 0.5) / 4.5);
        if (alpha < 0.52f) alpha = 0.52f;
        if (alpha > 1.0f)  alpha = 1.0f;
    }
    return ImVec4(base.x, base.y, base.z, base.w * alpha);
}

ImFont* monoUiFont()
{
    const ImVector<ImFont*>& fonts = ImGui::GetIO().Fonts->Fonts;
    if (fonts.Size > 1 && fonts[1] != nullptr) return fonts[1];
    return ImGui::GetFont();
}

void toolbarDivider()
{
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
}

// ToolIcon enum lives at the top of the file (forward-declared for
// MainUi::draw() use). Definition not duplicated here.

ImVec2 iconPoint(const ImVec2& min, const ImVec2& max, float x, float y)
{
    return ImVec2(min.x + (max.x - min.x) * x,
                  min.y + (max.y - min.y) * y);
}

void drawArrow(ImDrawList* dl, ImVec2 from, ImVec2 to, ImU32 color, float thickness)
{
    const float dx = to.x - from.x;
    const float dy = to.y - from.y;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len <= 0.001f) return;
    const float ux = dx / len;
    const float uy = dy / len;
    const float px = -uy;
    const float py = ux;
    const float head = 4.6f;
    dl->AddLine(from, to, color, thickness);
    dl->AddTriangleFilled(
        to,
        ImVec2(to.x - ux * head + px * head * 0.62f, to.y - uy * head + py * head * 0.62f),
        ImVec2(to.x - ux * head - px * head * 0.62f, to.y - uy * head - py * head * 0.62f),
        color);
}

void drawToolIcon(ImDrawList* dl, ToolIcon icon, ImVec2 min, ImVec2 max, ImU32 color)
{
    const float t = 1.7f;
    auto p = [&](float x, float y) { return iconPoint(min, max, x, y); };

    switch (icon) {
        case ToolIcon::Pause:
            dl->AddRectFilled(p(0.36f, 0.25f), p(0.45f, 0.75f), color, 1.0f);
            dl->AddRectFilled(p(0.55f, 0.25f), p(0.64f, 0.75f), color, 1.0f);
            break;
        case ToolIcon::Play:
            dl->AddTriangleFilled(p(0.38f, 0.24f), p(0.38f, 0.76f), p(0.72f, 0.50f), color);
            break;
        case ToolIcon::Clear:
            dl->AddLine(p(0.32f, 0.34f), p(0.68f, 0.34f), color, t);
            dl->AddLine(p(0.43f, 0.25f), p(0.57f, 0.25f), color, t);
            dl->AddLine(p(0.38f, 0.42f), p(0.42f, 0.76f), color, t);
            dl->AddLine(p(0.62f, 0.42f), p(0.58f, 0.76f), color, t);
            dl->AddLine(p(0.43f, 0.76f), p(0.57f, 0.76f), color, t);
            break;
        case ToolIcon::Autoscale:
            drawArrow(dl, p(0.50f, 0.52f), p(0.50f, 0.20f), color, t);
            drawArrow(dl, p(0.50f, 0.48f), p(0.50f, 0.80f), color, t);
            dl->AddLine(p(0.28f, 0.62f), p(0.42f, 0.42f), color, t);
            dl->AddLine(p(0.42f, 0.42f), p(0.58f, 0.58f), color, t);
            dl->AddLine(p(0.58f, 0.58f), p(0.72f, 0.34f), color, t);
            break;
        case ToolIcon::ResetY:
            dl->AddLine(p(0.34f, 0.76f), p(0.34f, 0.24f), color, t);
            drawArrow(dl, p(0.34f, 0.52f), p(0.34f, 0.20f), color, t);
            dl->PathArcTo(p(0.58f, 0.52f), (max.y - min.y) * 0.18f, 0.35f, 4.85f, 20);
            dl->PathStroke(color, 0, t);
            drawArrow(dl, p(0.50f, 0.36f), p(0.66f, 0.34f), color, t);
            break;
        case ToolIcon::Crosshair:
            dl->AddCircle(p(0.50f, 0.50f), (max.y - min.y) * 0.22f, color, 32, t);
            dl->AddLine(p(0.50f, 0.18f), p(0.50f, 0.34f), color, t);
            dl->AddLine(p(0.50f, 0.66f), p(0.50f, 0.82f), color, t);
            dl->AddLine(p(0.18f, 0.50f), p(0.34f, 0.50f), color, t);
            dl->AddLine(p(0.66f, 0.50f), p(0.82f, 0.50f), color, t);
            break;
        case ToolIcon::Ruler:
            dl->AddLine(p(0.24f, 0.72f), p(0.76f, 0.28f), color, t);
            dl->AddCircleFilled(p(0.24f, 0.72f), (max.y - min.y) * 0.055f, color, 10);
            dl->AddCircleFilled(p(0.76f, 0.28f), (max.y - min.y) * 0.055f, color, 10);
            dl->AddLine(p(0.20f, 0.58f), p(0.20f, 0.82f), color, t);
            dl->AddLine(p(0.16f, 0.78f), p(0.28f, 0.78f), color, t);
            dl->AddLine(p(0.80f, 0.18f), p(0.80f, 0.42f), color, t);
            dl->AddLine(p(0.72f, 0.22f), p(0.84f, 0.22f), color, t);
            break;
        case ToolIcon::Follow:
            dl->AddLine(p(0.24f, 0.30f), p(0.44f, 0.50f), color, t);
            dl->AddLine(p(0.24f, 0.70f), p(0.44f, 0.50f), color, t);
            dl->AddLine(p(0.46f, 0.30f), p(0.66f, 0.50f), color, t);
            dl->AddLine(p(0.46f, 0.70f), p(0.66f, 0.50f), color, t);
            dl->AddLine(p(0.76f, 0.24f), p(0.76f, 0.76f), color, t);
            break;
        case ToolIcon::Fit:
            dl->AddLine(p(0.28f, 0.30f), p(0.28f, 0.46f), color, t);
            dl->AddLine(p(0.28f, 0.30f), p(0.44f, 0.30f), color, t);
            dl->AddLine(p(0.72f, 0.30f), p(0.56f, 0.30f), color, t);
            dl->AddLine(p(0.72f, 0.30f), p(0.72f, 0.46f), color, t);
            dl->AddLine(p(0.28f, 0.70f), p(0.44f, 0.70f), color, t);
            dl->AddLine(p(0.28f, 0.70f), p(0.28f, 0.54f), color, t);
            dl->AddLine(p(0.72f, 0.70f), p(0.56f, 0.70f), color, t);
            dl->AddLine(p(0.72f, 0.70f), p(0.72f, 0.54f), color, t);
            break;
        case ToolIcon::Save:
            dl->AddRect(p(0.28f, 0.22f), p(0.72f, 0.78f), color, 1.0f, 0, t);
            dl->AddRectFilled(p(0.36f, 0.25f), p(0.58f, 0.40f), color, 1.0f);
            dl->AddRect(p(0.38f, 0.56f), p(0.64f, 0.78f), color, 1.0f, 0, t);
            break;
        case ToolIcon::Open:
            dl->AddLine(p(0.24f, 0.36f), p(0.42f, 0.36f), color, t);
            dl->AddLine(p(0.42f, 0.36f), p(0.48f, 0.45f), color, t);
            dl->AddLine(p(0.48f, 0.45f), p(0.76f, 0.45f), color, t);
            dl->AddLine(p(0.76f, 0.45f), p(0.68f, 0.76f), color, t);
            dl->AddLine(p(0.68f, 0.76f), p(0.24f, 0.76f), color, t);
            dl->AddLine(p(0.24f, 0.76f), p(0.24f, 0.36f), color, t);
            break;
        case ToolIcon::Trigger: {
            // Lightning bolt as a 3-segment thick zigzag. AddConvexPolyFilled
            // refused the bolt silhouette (it isn't convex -- leftover triangles
            // from the fill produced visible artifacts). Stroked Z-shape is
            // visually consistent with the other line-icons in this row.
            const float bolt = t * 1.4f;
            dl->AddLine(p(0.60f, 0.20f), p(0.34f, 0.52f), color, bolt);
            dl->AddLine(p(0.34f, 0.52f), p(0.60f, 0.50f), color, bolt);
            dl->AddLine(p(0.60f, 0.50f), p(0.36f, 0.82f), color, bolt);
            break;
        }
        case ToolIcon::TabConnection:
            // Plug: body + cable + two prongs.
            dl->AddRectFilled(p(0.40f, 0.30f), p(0.66f, 0.62f), color, 1.5f);
            dl->AddLine(p(0.46f, 0.30f), p(0.46f, 0.20f), color, t);
            dl->AddLine(p(0.60f, 0.30f), p(0.60f, 0.20f), color, t);
            dl->AddLine(p(0.53f, 0.62f), p(0.53f, 0.78f), color, t);
            break;
        case ToolIcon::TabMonitor:
            // Log lines: 4 horizontal bars.
            dl->AddRectFilled(p(0.24f, 0.26f), p(0.76f, 0.34f), color, 1.0f);
            dl->AddRectFilled(p(0.24f, 0.40f), p(0.66f, 0.48f), color, 1.0f);
            dl->AddRectFilled(p(0.24f, 0.54f), p(0.72f, 0.62f), color, 1.0f);
            dl->AddRectFilled(p(0.24f, 0.68f), p(0.58f, 0.76f), color, 1.0f);
            break;
        case ToolIcon::TabVariables:
            // Curly braces with two dots between: { . . }
            dl->AddBezierQuadratic(p(0.34f, 0.22f), p(0.24f, 0.22f), p(0.26f, 0.50f), color, t, 12);
            dl->AddBezierQuadratic(p(0.26f, 0.50f), p(0.24f, 0.78f), p(0.34f, 0.78f), color, t, 12);
            dl->AddBezierQuadratic(p(0.66f, 0.22f), p(0.76f, 0.22f), p(0.74f, 0.50f), color, t, 12);
            dl->AddBezierQuadratic(p(0.74f, 0.50f), p(0.76f, 0.78f), p(0.66f, 0.78f), color, t, 12);
            dl->AddCircleFilled(p(0.43f, 0.50f), (max.y - min.y) * 0.045f, color, 8);
            dl->AddCircleFilled(p(0.57f, 0.50f), (max.y - min.y) * 0.045f, color, 8);
            break;
        case ToolIcon::TabPlots:
            // Waveform squiggle.
            dl->AddBezierCubic(p(0.20f, 0.70f), p(0.32f, 0.20f),
                               p(0.46f, 0.20f), p(0.50f, 0.50f),
                               color, t, 16);
            dl->AddBezierCubic(p(0.50f, 0.50f), p(0.54f, 0.80f),
                               p(0.68f, 0.80f), p(0.80f, 0.30f),
                               color, t, 16);
            break;
        case ToolIcon::TabRemote:
            // Antenna + two signal arcs.
            dl->AddLine(p(0.50f, 0.36f), p(0.50f, 0.78f), color, t);
            dl->AddLine(p(0.40f, 0.78f), p(0.60f, 0.78f), color, t);
            dl->AddCircleFilled(p(0.50f, 0.30f), (max.y - min.y) * 0.06f, color, 12);
            dl->PathArcTo(p(0.50f, 0.30f), (max.y - min.y) * 0.18f, 3.49f, 5.93f, 18);
            dl->PathStroke(color, 0, t);
            dl->PathArcTo(p(0.50f, 0.30f), (max.y - min.y) * 0.30f, 3.49f, 5.93f, 22);
            dl->PathStroke(color, 0, t);
            break;
        case ToolIcon::TabCalibration: {
            // Dial / target -- a circle with crosshair ticks (calibration).
            const float r = (max.y - min.y) * 0.26f;
            dl->AddCircle(p(0.50f, 0.52f), r, color, 20, t);
            dl->AddLine(p(0.50f, 0.22f), p(0.50f, 0.34f), color, t);
            dl->AddLine(p(0.50f, 0.70f), p(0.50f, 0.82f), color, t);
            dl->AddLine(p(0.20f, 0.52f), p(0.32f, 0.52f), color, t);
            dl->AddLine(p(0.68f, 0.52f), p(0.80f, 0.52f), color, t);
            dl->AddCircleFilled(p(0.50f, 0.52f), (max.y - min.y) * 0.05f, color, 10);
            break;
        }
        case ToolIcon::TabUpdates:
            // Upload arrow into a small flash chip.
            dl->AddRect(p(0.24f, 0.48f), p(0.76f, 0.78f), color, 1.5f, 0, t);
            dl->AddLine(p(0.34f, 0.78f), p(0.34f, 0.86f), color, t);
            dl->AddLine(p(0.50f, 0.78f), p(0.50f, 0.86f), color, t);
            dl->AddLine(p(0.66f, 0.78f), p(0.66f, 0.86f), color, t);
            drawArrow(dl, p(0.50f, 0.22f), p(0.50f, 0.54f), color, t);
            dl->AddLine(p(0.36f, 0.36f), p(0.50f, 0.22f), color, t);
            dl->AddLine(p(0.64f, 0.36f), p(0.50f, 0.22f), color, t);
            break;
    }
}

// Custom tab: InvisibleButton sized to fit ToolIcon + label. Draws a tab-
// shaped background (rounded top corners only), the icon left, the text right,
// and a brighter underline when active. Returns true on click.
bool customTab(const char* id, const char* label, ToolIcon icon, bool active)
{
    constexpr float iconBox = 18.0f;
    constexpr float padX    = 12.0f;
    constexpr float padY    = 8.0f;
    constexpr float gap     = 8.0f;

    const ImVec2 textSize = ImGui::CalcTextSize(label);
    const ImVec2 size(padX + iconBox + gap + textSize.x + padX,
                      std::max(iconBox, textSize.y) + 2.0f * padY);

    const bool clicked = ImGui::InvisibleButton(id, size);
    const bool hovered = ImGui::IsItemHovered();
    const bool held    = ImGui::IsItemActive();
    const ImVec2 min = ImGui::GetItemRectMin();
    const ImVec2 max = ImGui::GetItemRectMax();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    // Background -- same blue palette as the iconButton in Plots toolbar.
    ImVec4 bg = ImVec4(0.12f, 0.13f, 0.15f, 1.00f);
    if (active)  bg = ImVec4(0.05f, 0.33f, 0.56f, 1.00f);
    if (hovered) bg = ImVec4(0.19f, 0.35f, 0.47f, 1.00f);
    if (held)    bg = ImVec4(0.075f, 0.42f, 0.69f, 1.00f);
    dl->AddRectFilled(min, max,
                      ImGui::ColorConvertFloat4ToU32(bg),
                      6.0f, ImDrawFlags_RoundCornersTop);
    dl->AddRect(min, max,
                IM_COL32(75, 75, 80, active ? 220 : 140),
                6.0f, ImDrawFlags_RoundCornersTop, 1.0f);

    // Active underline (Material-style) for extra "this one is selected" cue.
    if (active) {
        dl->AddRectFilled(ImVec2(min.x + 6.0f, max.y - 3.0f),
                          ImVec2(max.x - 6.0f, max.y - 1.0f),
                          IM_COL32(120, 200, 255, 230));
    }

    // Icon (square box, vertically centered).
    const float iconY = min.y + (size.y - iconBox) * 0.5f;
    const ImVec2 iconMin(min.x + padX, iconY);
    const ImVec2 iconMax(iconMin.x + iconBox, iconY + iconBox);
    const ImU32 fg = active ? IM_COL32(245, 250, 255, 255)
                            : IM_COL32(220, 225, 232, 245);
    drawToolIcon(dl, icon, iconMin, iconMax, fg);

    // Label, vertically centered next to the icon.
    const ImVec2 textPos(iconMax.x + gap,
                         min.y + (size.y - textSize.y) * 0.5f);
    dl->AddText(textPos, fg, label);

    return clicked;
}

bool iconButton(const char* id, ToolIcon icon, const char* tooltip, bool active = false, ImVec2 size = ImVec2(30.0f, 24.0f))
{
    const bool clicked = ImGui::InvisibleButton(id, size);
    const bool hovered = ImGui::IsItemHovered();
    const bool held = ImGui::IsItemActive();
    const ImVec2 min = ImGui::GetItemRectMin();
    const ImVec2 max = ImGui::GetItemRectMax();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    ImVec4 bg = ImGui::GetStyleColorVec4(ImGuiCol_Button);
    if (active)      bg = ImVec4(0.050f, 0.330f, 0.560f, 1.00f);
    if (hovered)     bg = ImVec4(0.190f, 0.350f, 0.470f, 1.00f);
    if (held)        bg = ImVec4(0.075f, 0.420f, 0.690f, 1.00f);
    const float rounding = ImGui::GetStyle().FrameRounding;
    dl->AddRectFilled(min, max, ImGui::ColorConvertFloat4ToU32(bg), rounding);
    dl->AddRect(min, max, IM_COL32(75, 75, 80, active ? 210 : 130), rounding);
    drawToolIcon(dl, icon, min, max,
                 active ? IM_COL32(245, 250, 255, 255) : IM_COL32(215, 218, 222, 245));

    if (tooltip != nullptr && tooltip[0] != '\0' && hovered) {
        ImGui::SetTooltip("%s", tooltip);
    }
    return clicked;
}

std::string countLabel(const std::string& name, int active, size_t total, const char* activeWord)
{
    std::ostringstream os;
    os << (name.empty() ? "source" : name) << "  "
       << active << ' ' << activeWord << " of " << total;
    return os.str();
}

} // namespace

// Draw a marker on the monitor table's scrollbar showing where the pinned
// (analyzed) row sits, so it's locatable even when scrolled out of view.
// Call immediately after ImGui::EndTable() (uses the table's last-item rect).
static void drawMonitorPinMarker(int pinnedRow, int total)
{
    if (pinnedRow < 0 || total <= 0) return;
    const ImVec2 rmin = ImGui::GetItemRectMin();
    const ImVec2 rmax = ImGui::GetItemRectMax();
    const float headerH = ImGui::GetFrameHeight();
    const float top = rmin.y + headerH;
    const float h = rmax.y - top;
    if (h <= 0.0f) return;
    const float frac = (total > 1) ? static_cast<float>(pinnedRow) / static_cast<float>(total - 1) : 0.0f;
    const float y = top + frac * h;
    const float sbw = ImGui::GetStyle().ScrollbarSize;
    const ImU32 col = IM_COL32(0, 122, 204, 240);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(ImVec2(rmax.x - sbw, y - 2.0f), ImVec2(rmax.x, y + 2.0f), col, 0.0f);
    dl->AddTriangleFilled(ImVec2(rmax.x - sbw - 7.0f, y),
                          ImVec2(rmax.x - sbw, y - 5.0f),
                          ImVec2(rmax.x - sbw, y + 5.0f), col);
}

void MainUi::drawCanMonitor()
{
    if (monitorSelKey_.valid && !monitorSelText_.empty() &&
        ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_C, false)) {
        ImGui::SetClipboardText(monitorSelText_.c_str());
    }
    ImGui::SeparatorText("Traffic");
    ImGui::Checkbox("Pause", &monitorPaused_);
    ImGui::SameLine();
    if (ImGui::Button("Clear")) trace_.clear();
    ImGui::SameLine();
    ImGui::Checkbox("Group by ID", &monitorGroupByID_);
    ImGui::SameLine();
    ImGui::Checkbox("Filters", &monitorShowFilters_);
    toolbarDivider();
    if (!canLogger_.active()) {
        if (ImGui::SmallButton("Start log")) canLogger_.start(snapshotPath_.data());
    } else {
        if (ImGui::SmallButton("Stop log")) canLogger_.stop();
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Save CSV")) {
        std::string chosen = snapshotPath_.data();
        if (nativeFileDialog(true, "CSV files\0*.csv\0All files\0*.*\0", chosen)) {
            setTextBuffer(snapshotPath_, chosen);
        }
        const auto frames = trace_.snapshot();
        std::ofstream f(snapshotPath_.data(), std::ios::binary);
        if (f) {
            f << "timestamp,dir,id,ext,fd,brs,dlc,data,parsed\n";
            for (const CanFrame& fr : frames) {
                f << buildRowCsv(fr, decodeCanFrame(fr)) << "\n";
            }
            snapshotStatus_ = "saved " + std::to_string(frames.size()) + " frames";
            lastFileMsg_ = "saved: " + std::string(snapshotPath_.data());
        } else {
            snapshotStatus_ = "save failed";
        }
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Load CSV")) {
        std::string chosen = snapshotPath_.data();
        if (nativeFileDialog(false, "CSV files\0*.csv\0All files\0*.*\0", chosen)) {
            setTextBuffer(snapshotPath_, chosen);
        }
        std::ifstream f(snapshotPath_.data(), std::ios::binary);
        if (!f) {
            snapshotStatus_ = "open failed";
        } else {
            if (activeTransport_ != nullptr) { activeTransport_->close(); activeTransport_ = nullptr; }
            trace_.clear();
            replayMode_ = true;
            std::string line;
            std::getline(f, line); // header
            size_t loaded = 0;
            while (std::getline(f, line)) {
                std::stringstream ss(line);
                std::string tok;
                CanFrame fr;
                std::vector<std::string> cols;
                while (std::getline(ss, tok, ',')) cols.push_back(tok);
                if (cols.size() < 8) continue;
                try {
                    fr.timestamp = std::stod(cols[0]);
                    fr.rx = (cols[1] == "RX");
                    fr.id = static_cast<uint32_t>(std::stoul(cols[2], nullptr, 0));
                    fr.extended = std::stoi(cols[3]) != 0;
                    fr.fd = std::stoi(cols[4]) != 0;
                    fr.brs = std::stoi(cols[5]) != 0;
                    fr.dlc = static_cast<uint8_t>(std::stoi(cols[6]));
                    hexToBytes(cols[7], fr.data);
                    trace_.push(fr);
                    ++loaded;
                } catch (...) { /* skip bad row */ }
            }
            snapshotStatus_ = "loaded " + std::to_string(loaded) + " frames (replay)";
            lastFileMsg_ = "loaded: " + std::string(snapshotPath_.data());
        }
    }

    toolbarDivider();
    ImGui::Checkbox("Text view", &monitorTextView_);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Switch to plain-text view. Drag mouse to select across multiple rows,\n"
                          "then Ctrl+C to copy. Ctrl+A selects all.");
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "Right-click any row -> Filter to add include/exclude rules.\n"
            "Group by ID: one row per unique ID, dims as last frame ages.\n"
            "Text view: native drag-select across the whole table.");
    }

    if (monitorShowFilters_) {
        if (ImGui::BeginChild("##rules", ImVec2(0, 145), ImGuiChildFlags_Border)) {
            if (ImGui::SmallButton("+ Rule")) filterRules_.emplace_back();
            ImGui::SameLine();
            if (ImGui::SmallButton("Clear all rules")) filterRules_.clear();
            ImGui::SameLine();
            ImGui::TextDisabled("right-click a row in the monitor to add filters quickly");

            int delIdx = -1;
            int dupIdx = -1;
            int addAtIdx = -1;
            const ImGuiTableFlags rflags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                           ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp;
            if (ImGui::BeginTable("##frules", 6, rflags, ImVec2(0, 0))) {
                ImGui::TableSetupColumn("on",     0, 0.25f);
                ImGui::TableSetupColumn("mode",   0, 0.6f);
                ImGui::TableSetupColumn("field",  0, 0.7f);
                ImGui::TableSetupColumn("value",  0, 2.4f);
                ImGui::TableSetupColumn("delete", 0, 0.3f);
                ImGui::TableHeadersRow();
                for (size_t i = 0; i < filterRules_.size(); ++i) {
                    CanFilterRule& r = filterRules_[i];
                    ImGui::PushID(static_cast<int>(i));
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn(); ImGui::Checkbox("##en", &r.enabled);
                    ImGui::TableNextColumn();
                    {
                        const char* modes[] = {"Include", "Exclude"};
                        int m = static_cast<int>(r.mode);
                        ImGui::SetNextItemWidth(-1);
                        if (ImGui::Combo("##m", &m, modes, 2)) r.mode = static_cast<FilterMode>(m);
                    }
                    ImGui::TableNextColumn();
                    {
                        const char* fields[] = {"Id", "Substr", "Endpoint", "Direction"};
                        int fld = static_cast<int>(r.field);
                        ImGui::SetNextItemWidth(-1);
                        if (ImGui::Combo("##f", &fld, fields, 4)) r.field = static_cast<FilterField>(fld);
                    }
                    ImGui::TableNextColumn();
                    switch (r.field) {
                        case FilterField::Id: {
                            char tmp[32];
                            std::snprintf(tmp, sizeof(tmp), "0x%X", r.idValue);
                            ImGui::SetNextItemWidth(-72);
                            if (ImGui::InputText("##id", tmp, sizeof(tmp))) {
                                r.idValue = static_cast<uint32_t>(std::strtoul(tmp, nullptr, 0));
                            }
                            inputClipboardMenu(tmp, sizeof(tmp));
                            ImGui::SameLine();
                            ImGui::Checkbox("ext", &r.extended);
                            break;
                        }
                        case FilterField::Direction: {
                            const char* dirs[] = {"RX", "TX"};
                            ImGui::SetNextItemWidth(-1);
                            ImGui::Combo("##dir", &r.dirValue, dirs, 2);
                            break;
                        }
                        default:
                            ImGui::SetNextItemWidth(-1);
                            ImGui::InputTextWithHint("##t",
                                r.field == FilterField::Endpoint ? "MAIN, MOTOR, RK or MAIN->MOTOR"
                                                                 : "substring of parsed text",
                                r.textBuf.data(), r.textBuf.size());
                            inputClipboardMenu(r.textBuf.data(), r.textBuf.size());
                            break;
                    }
                    ImGui::TableNextColumn();
                    if (ImGui::SmallButton("X")) delIdx = static_cast<int>(i);
                    if (ImGui::BeginPopupContextItem("##rmrule")) {
                        if (ImGui::MenuItem("Add new rule above")) addAtIdx = static_cast<int>(i);
                        if (ImGui::MenuItem("Add new rule below")) addAtIdx = static_cast<int>(i) + 1;
                        if (ImGui::MenuItem("Duplicate rule"))     dupIdx   = static_cast<int>(i);
                        ImGui::Separator();
                        if (ImGui::MenuItem("Remove rule"))         delIdx   = static_cast<int>(i);
                        ImGui::EndPopup();
                    }
                    ImGui::PopID();
                }
                ImGui::EndTable();
            }
            if (delIdx >= 0 && delIdx < static_cast<int>(filterRules_.size())) {
                filterRules_.erase(filterRules_.begin() + delIdx);
            } else if (dupIdx >= 0 && dupIdx < static_cast<int>(filterRules_.size())) {
                filterRules_.insert(filterRules_.begin() + dupIdx + 1, filterRules_[dupIdx]);
            } else if (addAtIdx >= 0 && addAtIdx <= static_cast<int>(filterRules_.size())) {
                filterRules_.insert(filterRules_.begin() + addAtIdx, CanFilterRule{});
            }
        }
        ImGui::EndChild();
    }

    auto pushNewRule = [&](const FilterRequest& req) {
        if (!req.present) return;
        CanFilterRule r;
        r.enabled = true;
        if (!req.blank) {
            r.mode = req.exclude ? FilterMode::Exclude : FilterMode::Include;
            r.field = req.field;
            r.idValue = req.idValue;
            r.extended = req.extended;
            if (!req.text.empty()) {
                std::snprintf(r.textBuf.data(), r.textBuf.size(), "%s", req.text.c_str());
            }
        }
        filterRules_.push_back(r);
        monitorShowFilters_ = true;
    };

    const auto frames = trace_.snapshot();
    auto passesQuick = [](const CanFrame&, const std::string&) { return true; };

    const ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable |
                                  ImGuiTableFlags_Hideable | ImGuiTableFlags_ScrollY |
                                  ImGuiTableFlags_SizingStretchProp;

    const double tNow = clock_.nowSeconds();
    const ImVec4 colByteChanged(1.00f, 0.86f, 0.36f, 1.0f);
    const ImVec4 colByteStatic (0.65f, 0.66f, 0.70f, 1.0f);

    // Horizontal split: frames table on the left, persistent decode panel on the right.
    // Click a row in the table to load it into the right-side decode view.
    const float monAvailW = ImGui::GetContentRegionAvail().x;
    const float monMinLeft = 360.0f;
    const float monMinRight = 280.0f;
    if (monitorSplitX_ < 0.0f) monitorSplitX_ = monAvailW * 0.68f;
    if (monitorSplitX_ < monMinLeft) monitorSplitX_ = monMinLeft;
    if (monitorSplitX_ > monAvailW - monMinRight) monitorSplitX_ = std::max(monMinLeft, monAvailW - monMinRight);
    ImGui::BeginChild("##mon-left", ImVec2(monitorSplitX_, 0), ImGuiChildFlags_None,
                      ImGuiWindowFlags_HorizontalScrollbar);

    if (monitorGroupByID_) {
        std::unordered_map<uint64_t, AggRow> groups;
        groups.reserve(frames.size());
        for (const CanFrame& f : frames) {
            const std::string parsed = decodeCanFrame(f);
            if (!passesQuick(f, parsed)) continue;
            const FrameTags tags = extractFrameTags(f);
            if (!framePassesRules(f, tags, parsed, filterRules_)) continue;

            const uint64_t key = (static_cast<uint64_t>(f.id) << 1) | (f.extended ? 1ULL : 0ULL);
            AggRow& a = groups[key];
            if (a.count == 0) {
                a.firstBytes = f.data;
                a.everChanged.assign(f.data.size(), 0);
            } else {
                const double dt = f.timestamp - a.lastTs;
                if (dt >= 0.0) {
                    if (a.periodMs <= 0.0) a.periodMs = dt * 1000.0;
                    else a.periodMs = 0.85 * a.periodMs + 0.15 * dt * 1000.0;
                }
                if (a.firstBytes.size() < f.data.size()) a.firstBytes.resize(f.data.size(), 0);
                if (a.everChanged.size() < f.data.size()) a.everChanged.resize(f.data.size(), 1);
                for (size_t i = 0; i < f.data.size(); ++i) {
                    if (i < a.firstBytes.size() && f.data[i] != a.firstBytes[i]) {
                        a.everChanged[i] = 1;
                    }
                }
            }
            a.id = f.id;
            a.extended = f.extended;
            a.fd = f.fd;
            a.brs = f.brs;
            a.rx = f.rx;
            a.dlc = f.dlc;
            a.data = f.data;
            a.parsed = parsed;
            a.lastTs = f.timestamp;
            a.count++;
        }

        std::vector<const AggRow*> rows;
        rows.reserve(groups.size());
        for (auto& kv : groups) rows.push_back(&kv.second);
        std::sort(rows.begin(), rows.end(), [](const AggRow* a, const AggRow* b) {
            if (a->id != b->id) return a->id < b->id;
            return a->extended < b->extended;
        });

        FilterRequest pendingReq;
        ImGui::PushFont(monoUiFont());
        if (ImGui::BeginTable("can-grouped", 9, flags, ImVec2(0, -1))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("count",     0, 0.6f);
            ImGui::TableSetupColumn("period ms", 0, 0.7f);
            ImGui::TableSetupColumn("dir",       0, 0.4f);
            ImGui::TableSetupColumn("id",        0, 0.8f);
            ImGui::TableSetupColumn("EXT",       ImGuiTableColumnFlags_DefaultHide, 0.4f);
            ImGui::TableSetupColumn("FD",        ImGuiTableColumnFlags_DefaultHide, 0.3f);
            ImGui::TableSetupColumn("DLC",       0, 0.4f);
            ImGui::TableSetupColumn("DATA",      0, 2.0f);
            ImGui::TableSetupColumn("PARSED",    0, 3.0f);
            ImGui::TableHeadersRow();

            int pinnedRow = -1;
            if (monitorPinnedValid_) {
                for (size_t r = 0; r < rows.size(); ++r) {
                    if (rows[r]->id == monitorPinnedFrame_.id &&
                        rows[r]->extended == monitorPinnedFrame_.extended &&
                        rows[r]->rx == monitorPinnedFrame_.rx) {
                        pinnedRow = static_cast<int>(r);
                        break;
                    }
                }
            }

            ImGuiListClipper clipper;
            clipper.Begin(static_cast<int>(rows.size()));
            while (clipper.Step()) {
                for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row) {
                    const AggRow& a = *rows[row];
                    const double age = std::max(0.0, tNow - a.lastTs);
                    const ImVec4 baseText = ImGui::GetStyleColorVec4(ImGuiCol_Text);
                    const ImVec4 dimText  = ageColor(age, baseText);
                    const ImVec4 dimByte  = ageColor(age, ImVec4(1.0f,1.0f,1.0f,1.0f));

                    ImGui::TableNextRow();
                    if (monitorPinnedValid_ &&
                        a.id == monitorPinnedFrame_.id &&
                        a.extended == monitorPinnedFrame_.extended &&
                        a.rx == monitorPinnedFrame_.rx) {
                        ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, IM_COL32(0, 122, 204, 110));
                    }
                    ImGui::PushID(row);

                    CanFrame fakeFrame;
                    fakeFrame.timestamp = a.lastTs;
                    fakeFrame.rx = a.rx;
                    fakeFrame.id = a.id;
                    fakeFrame.extended = a.extended;
                    fakeFrame.fd = a.fd;
                    fakeFrame.brs = a.brs;
                    fakeFrame.dlc = a.dlc;
                    fakeFrame.data = a.data;
                    const FrameTags tags = extractFrameTags(fakeFrame);
                    bool rowClicked = false;

                    auto cellSel = [&](const std::string& text, int col) {
                        ImGui::TableNextColumn();
                        char idbuf[16];
                        std::snprintf(idbuf, sizeof(idbuf), "##c%d", col);
                        static thread_local char buf[2048];
                        const size_t n = std::min(text.size(), sizeof(buf) - 1);
                        std::memcpy(buf, text.data(), n);
                        buf[n] = '\0';
                        ImGui::PushStyleColor(ImGuiCol_FrameBg,        ImVec4(0,0,0,0));
                        ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(1,1,1,0.04f));
                        ImGui::PushStyleColor(ImGuiCol_FrameBgActive,  ImVec4(0.035f,0.278f,0.443f,1.0f));
                        ImGui::PushStyleColor(ImGuiCol_Text, dimText);
                        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(2, 1));
                        ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
                        ImGui::SetNextItemWidth(-1);
                        ImGui::InputText(idbuf, buf, sizeof(buf), ImGuiInputTextFlags_ReadOnly);
                        ImGui::PopStyleVar(2);
                        ImGui::PopStyleColor(4);
                        if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(0)) {
                            rowClicked = true;
                        }
                        ImGui::OpenPopupOnItemClick("##rowctx", ImGuiPopupFlags_MouseButtonRight);
                    };

                    {
                        char b[32]; std::snprintf(b, sizeof(b), "%zu", a.count); cellSel(b, 0);
                        std::snprintf(b, sizeof(b), "%.2f", a.periodMs); cellSel(b, 1);
                        cellSel(a.rx ? "RX" : "TX", 2);
                        cellSel(idToHex(a.id, a.extended), 3);
                        cellSel(a.extended ? "1" : "0", 4);
                        cellSel(a.fd ? "1" : "0", 5);
                        std::snprintf(b, sizeof(b), "%d", static_cast<int>(a.dlc)); cellSel(b, 6);
                    }

                    cellSel(bytesToHexSpaced(a.data), 7);
                    cellSel(a.parsed, 8);
                    (void)dimByte; (void)colByteChanged; (void)colByteStatic;

                    if (rowClicked) {
                        monitorPinnedFrame_ = fakeFrame;
                        monitorPinnedValid_ = true;
                    }

                    ImGui::SetNextWindowPos(ImGui::GetMousePos(), ImGuiCond_Appearing);
                    if (ImGui::BeginPopup("##rowctx")) {
                        FilterRequest req = renderRowMenuBody(fakeFrame, a.parsed, tags);
                        if (req.present) pendingReq = req;
                        ImGui::EndPopup();
                    }

                    ImGui::PopID();
                }
            }
            clipper.End();
            ImGui::EndTable();
            drawMonitorPinMarker(pinnedRow, static_cast<int>(rows.size()));
        }
        ImGui::PopFont();
        pushNewRule(pendingReq);
    }
    else if (monitorTextView_) {
        std::vector<size_t> visible;
        visible.reserve(frames.size());
        std::vector<std::string> parsedCache(frames.size());
        std::vector<FrameTags> tagsCache(frames.size());
        for (size_t i = 0; i < frames.size(); ++i) {
            parsedCache[i] = decodeCanFrame(frames[i]);
            tagsCache[i] = extractFrameTags(frames[i]);
            if (passesQuick(frames[i], parsedCache[i]) &&
                framePassesRules(frames[i], tagsCache[i], parsedCache[i], filterRules_)) {
                visible.push_back(i);
            }
        }

        static thread_local std::string text;
        text.clear();
        text.reserve(visible.size() * 96 + 64);
        text += "timestamp,dir,id,ext,fd,brs,dlc,data,parsed\n";
        for (size_t i : visible) {
            text += buildRowCsv(frames[i], parsedCache[i]);
            text += '\n';
        }
        static thread_local std::vector<char> buf;
        const size_t need = text.size() + 1;
        if (buf.size() < need) buf.resize(need);
        std::memcpy(buf.data(), text.data(), text.size());
        buf[text.size()] = '\0';
        ImGui::PushFont(monoUiFont());
        ImGui::PushStyleColor(ImGuiCol_FrameBg, ImGui::GetStyleColorVec4(ImGuiCol_ChildBg));
        ImGui::InputTextMultiline("##textview", buf.data(), buf.size(), ImVec2(-1, -1),
                                  ImGuiInputTextFlags_ReadOnly);
        ImGui::PopStyleColor();
        ImGui::PopFont();
    }
    else {

    ImGui::PushFont(monoUiFont());
    if (ImGui::BeginTable("can-table", 9, flags, ImVec2(0, -1))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("timestamp", 0, 0.9f);
        ImGui::TableSetupColumn("dir", 0, 0.3f);
        ImGui::TableSetupColumn("id", 0, 0.7f);
        ImGui::TableSetupColumn("EXT", ImGuiTableColumnFlags_DefaultHide, 0.3f);
        ImGui::TableSetupColumn("FD",  ImGuiTableColumnFlags_DefaultHide, 0.3f);
        ImGui::TableSetupColumn("BRS", ImGuiTableColumnFlags_DefaultHide, 0.3f);
        ImGui::TableSetupColumn("DLC", 0, 0.4f);
        ImGui::TableSetupColumn("DATA", 0, 2.0f);
        ImGui::TableSetupColumn("PARSED", 0, 3.0f);
        ImGui::TableHeadersRow();

        std::vector<size_t> visible;
        visible.reserve(frames.size());
        std::vector<std::string> parsedCache(frames.size());
        std::vector<FrameTags> tagsCache(frames.size());
        for (size_t i = 0; i < frames.size(); ++i) {
            parsedCache[i] = decodeCanFrame(frames[i]);
            tagsCache[i] = extractFrameTags(frames[i]);
            if (passesQuick(frames[i], parsedCache[i]) &&
                framePassesRules(frames[i], tagsCache[i], parsedCache[i], filterRules_)) {
                visible.push_back(i);
            }
        }

        // Index (within visible[]) of the row pinned in the decode panel — used
        // both for the row highlight (above) and the scrollbar marker (below).
        int pinnedRow = -1;
        if (monitorPinnedValid_) {
            for (size_t r = 0; r < visible.size(); ++r) {
                const CanFrame& vf = frames[visible[r]];
                if (vf.timestamp == monitorPinnedFrame_.timestamp && vf.id == monitorPinnedFrame_.id) {
                    pinnedRow = static_cast<int>(r);
                    break;
                }
            }
        }

        FilterRequest pendingReq;
        ImGuiListClipper clipper;
        clipper.Begin(static_cast<int>(visible.size()));
        while (clipper.Step()) {
            for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row) {
                const size_t idx = visible[row];
                const CanFrame& f = frames[idx];
                const std::string& parsed = parsedCache[idx];
                const FrameTags& tags = tagsCache[idx];
                ImGui::TableNextRow();
                // Highlight the row currently pinned in the right-hand decode panel,
                // so it stays findable while scrolling.
                if (monitorPinnedValid_ &&
                    f.timestamp == monitorPinnedFrame_.timestamp &&
                    f.id == monitorPinnedFrame_.id) {
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, IM_COL32(0, 122, 204, 110));
                }
                ImGui::PushID(row);

                bool rowClicked = false;
                auto cellSel = [&](const std::string& text, int col) {
                    ImGui::TableNextColumn();
                    char idbuf[16];
                    std::snprintf(idbuf, sizeof(idbuf), "##c%d", col);
                    static thread_local char buf[2048];
                    const size_t n = std::min(text.size(), sizeof(buf) - 1);
                    std::memcpy(buf, text.data(), n);
                    buf[n] = '\0';
                    ImGui::PushStyleColor(ImGuiCol_FrameBg,        ImVec4(0,0,0,0));
                    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(1,1,1,0.04f));
                    ImGui::PushStyleColor(ImGuiCol_FrameBgActive,  ImVec4(0.035f,0.278f,0.443f,1.0f));
                    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(2, 1));
                    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
                    ImGui::SetNextItemWidth(-1);
                    ImGui::InputText(idbuf, buf, sizeof(buf), ImGuiInputTextFlags_ReadOnly);
                    ImGui::PopStyleVar(2);
                    ImGui::PopStyleColor(3);
                    if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(0)) {
                        rowClicked = true;
                    }
                    ImGui::OpenPopupOnItemClick("##rowctx", ImGuiPopupFlags_MouseButtonRight);
                };

                char b[32]; std::snprintf(b, sizeof(b), "%.6f", f.timestamp); cellSel(b, 0);
                cellSel(f.rx ? "RX" : "TX", 1);
                cellSel(idToHex(f.id, f.extended), 2);
                cellSel(f.extended ? "1" : "0", 3);
                cellSel(f.fd ? "1" : "0", 4);
                cellSel(f.brs ? "1" : "0", 5);
                std::snprintf(b, sizeof(b), "%d", static_cast<int>(f.dlc)); cellSel(b, 6);
                cellSel(bytesToHexSpaced(f.data), 7);
                cellSel(parsed, 8);

                if (rowClicked) {
                    monitorPinnedFrame_ = f;
                    monitorPinnedValid_ = true;
                }

                ImGui::SetNextWindowPos(ImGui::GetMousePos(), ImGuiCond_Appearing);
                if (ImGui::BeginPopup("##rowctx")) {
                    FilterRequest req = renderRowMenuBody(f, parsed, tags);
                    if (req.present) pendingReq = req;
                    ImGui::EndPopup();
                }

                ImGui::PopID();
            }
        }
        clipper.End();
        const float maxScroll = ImGui::GetScrollMaxY();
        if (!monitorPaused_ && ImGui::GetScrollY() >= maxScroll - 50.0f) {
            ImGui::SetScrollHereY(1.0f);
        }
        ImGui::EndTable();
        drawMonitorPinMarker(pinnedRow, static_cast<int>(visible.size()));
        pushNewRule(pendingReq);
    }
    ImGui::PopFont();

    } // end of else (non-grouped, non-textview)

    ImGui::EndChild();   // ##mon-left

    // Splitter
    ImGui::SameLine(0.0f, 0.0f);
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0,0,0,0));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.45f, 0.55f, 0.75f, 0.40f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.45f, 0.65f, 0.95f, 0.80f));
    const float monSplitH = ImGui::GetContentRegionAvail().y;
    ImGui::Button("##mon-splitter", ImVec2(6.0f, monSplitH));
    ImGui::PopStyleColor(3);
    if (ImGui::IsItemActive()) {
        monitorSplitX_ += ImGui::GetIO().MouseDelta.x;
        if (monitorSplitX_ < monMinLeft) monitorSplitX_ = monMinLeft;
        if (monitorSplitX_ > monAvailW - monMinRight) monitorSplitX_ = monAvailW - monMinRight;
    }
    if (ImGui::IsItemHovered() || ImGui::IsItemActive()) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    }
    ImGui::SameLine(0.0f, 0.0f);

    // Right-side decode panel
    ImGui::BeginChild("##mon-right", ImVec2(0, 0), ImGuiChildFlags_Border,
                      ImGuiWindowFlags_AlwaysVerticalScrollbar);
    if (monitorPinnedValid_) {
        renderDecodeTableBody(monitorPinnedFrame_, true);
    } else {
        ImGui::TextDisabled("Click any row in the monitor to inspect its fields here.");
        ImGui::Spacing();
        ImGui::TextDisabled("Selected row stays pinned until you click another one.");
        ImGui::TextDisabled("Drag in value cells to select text, Ctrl+C to copy.");
    }
    ImGui::EndChild();
}

void MainUi::addWatch(const VariableSymbol& sym, bool plot, const std::string& sourceName)
{
    // Match by (address, nodeId) only. Display name can differ (config files may
    // override w.name to something like "motor.iq_meas" while symbol stays "iq_meas").
    for (WatchVar& w : watches_) {
        if (w.nodeId == sym.nodeId && w.address == sym.address) {
            w.enabled = true;
            w.plot = w.plot || plot;
            w.nextPollTime = 0.0;
            if (w.source.empty()) w.source = sourceName;
            markScenarioDirty("watch changed");
            return;
        }
    }
    WatchVar w;
    static_cast<VariableSymbol&>(w) = sym;
    w.pollHz = watchHzFromPeriodSec(kDefaultWatchPeriodSec);
    w.plot = plot;
    w.source = sourceName;
    watches_.push_back(std::move(w));
    markScenarioDirty("watch added");
}

void MainUi::togglePlot(const VariableSymbol& sym, const std::string& sourceName, uint8_t nodeId)
{
    // Single click in symbol list / Plots tree: add if absent, remove if present.
    // Match by address+nodeId only (works for both freshly added and config-loaded).
    for (auto it = watches_.begin(); it != watches_.end(); ++it) {
        if (it->address == sym.address && it->nodeId == nodeId) {
            watches_.erase(it);
            pending_.clear();
            pendingSentAt_.clear();
            nextSeq_ = 1;
            pollCursor_ = 0;
            markScenarioDirty("watch removed");
            return;
        }
    }
    VariableSymbol effective = sym;
    effective.nodeId = nodeId;
    addWatch(effective, true, sourceName);
}

void MainUi::drawVariables()
{
    ensureDefaultSources();

    ImGui::BeginChild("##vars-scroll", ImVec2(0, 0), ImGuiChildFlags_None,
                      ImGuiWindowFlags_AlwaysVerticalScrollbar);

    ImGui::SeparatorText("Projects and watches");
    if (ImGui::Button("Add project")) {
        SymbolSource ns;
        ns.name = "source" + std::to_string(sources_.size() + 1);
        sources_.push_back(std::move(ns));
        markScenarioDirty("project added");
    }
    toolbarDivider();
    if (ImGui::Button("Save scenario")) {
        saveScenarioNow("manual");
    }
    ImGui::SameLine();
    ImGui::TextDisabled("%s%s",
                        scenarioPath_.empty() ? "(scenario path is not set)" : scenarioPath_.c_str(),
                        scenarioDirty_ ? "  [modified]" : "");
    if (!scenarioStatus_.empty()) {
        ImGui::SameLine();
        ImGui::TextDisabled("| %s", scenarioStatus_.c_str());
    }

    if (false) {
    ImGui::SameLine();
    ImGui::TextUnformatted("|");
    ImGui::SameLine();
    if (ImGui::Button("Save vars")) {
        std::string chosen = varsConfigPath_.data();
        if (nativeFileDialog(true, "JSON files\0*.json\0All files\0*.*\0", chosen)) {
            setTextBuffer(varsConfigPath_, chosen);
        }
        std::ofstream f(varsConfigPath_.data(), std::ios::binary);
        if (!f) {
            varsConfigStatus_ = "save failed";
        } else {
            auto esc = [](const std::string& s) {
                std::string out;
                out.reserve(s.size() + 2);
                for (char c : s) {
                    if (c == '"' || c == '\\') { out += '\\'; out += c; }
                    else if (c == '\n') out += "\\n";
                    else out += c;
                }
                return out;
            };
            f << "{\n  \"sources\": [\n";
            for (size_t i = 0; i < sources_.size(); ++i) {
                const SymbolSource& s = sources_[i];
                f << "    {\"name\":\"" << esc(s.name) << "\","
                  << "\"elf\":\"" << esc(s.elfBuf.data()) << "\","
                  << "\"symbols\":\"" << esc(s.symBuf.data()) << "\","
                  << "\"node\":" << static_cast<int>(s.defaultNodeId) << "}"
                  << (i + 1 < sources_.size() ? ",\n" : "\n");
            }
            f << "  ],\n  \"watches\": [\n";
            for (size_t i = 0; i < watches_.size(); ++i) {
                const WatchVar& w = watches_[i];
                f << "    {\"name\":\"" << esc(w.name) << "\","
                  << "\"address\":" << w.address << ","
                  << "\"type\":\"" << esc(w.type) << "\","
                  << "\"size\":" << static_cast<int>(w.size) << ","
                  << "\"node_id\":" << static_cast<int>(w.nodeId) << ","
                  << "\"period_sec\":" << periodSecFromWatchHz(w.pollHz) << ","
                  << "\"enabled\":" << (w.enabled ? "true" : "false") << ","
                  << "\"plot\":" << (w.plot ? "true" : "false") << ","
                  << "\"plot_scale\":" << normalizePlotScale(w.plotScale) << ","
                  << "\"source\":\"" << esc(w.source) << "\"}"
                  << (i + 1 < watches_.size() ? ",\n" : "\n");
            }
            f << "  ]\n}\n";
            varsConfigStatus_ = "saved " + std::to_string(sources_.size()) +
                                " sources, " + std::to_string(watches_.size()) + " watches";
            lastFileMsg_ = "saved: " + std::string(varsConfigPath_.data());
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Load vars")) {
        std::string chosen = varsConfigPath_.data();
        if (nativeFileDialog(false, "JSON files\0*.json\0All files\0*.*\0", chosen)) {
            setTextBuffer(varsConfigPath_, chosen);
        }
        std::ifstream f(varsConfigPath_.data(), std::ios::binary);
        if (!f) {
            varsConfigStatus_ = "open failed";
        } else {
            std::string text((std::istreambuf_iterator<char>(f)),
                             std::istreambuf_iterator<char>());
            sources_.clear();
            watches_.clear();
            pending_.clear();
            pendingSentAt_.clear();
            nextSeq_ = 1;
            pollCursor_ = 0;
            // Tiny JSON sniff: extract objects in "sources" then in "watches".
            auto blockOf = [&](const std::string& key) -> std::string {
                const std::string needle = "\"" + key + "\"";
                size_t k = text.find(needle);
                if (k == std::string::npos) return {};
                size_t b = text.find('[', k);
                if (b == std::string::npos) return {};
                int depth = 0;
                for (size_t i = b; i < text.size(); ++i) {
                    if (text[i] == '[') ++depth;
                    else if (text[i] == ']') { if (--depth == 0) return text.substr(b + 1, i - b - 1); }
                }
                return {};
            };
            auto eachObj = [](const std::string& blk) {
                std::vector<std::string> out;
                int depth = 0; size_t s = 0;
                for (size_t i = 0; i < blk.size(); ++i) {
                    if (blk[i] == '{') { if (depth == 0) s = i; ++depth; }
                    else if (blk[i] == '}') { if (--depth == 0) out.push_back(blk.substr(s, i - s + 1)); }
                }
                return out;
            };
            auto strField = [](const std::string& obj, const std::string& key) -> std::string {
                const std::string n = "\"" + key + "\"";
                size_t k = obj.find(n);
                if (k == std::string::npos) return {};
                k = obj.find(':', k); if (k == std::string::npos) return {};
                k = obj.find('"', k); if (k == std::string::npos) return {};
                size_t e = obj.find('"', k + 1); if (e == std::string::npos) return {};
                return obj.substr(k + 1, e - k - 1);
            };
            auto numField = [](const std::string& obj, const std::string& key, double def = 0.0) -> double {
                const std::string n = "\"" + key + "\"";
                size_t k = obj.find(n);
                if (k == std::string::npos) return def;
                k = obj.find(':', k); if (k == std::string::npos) return def;
                ++k;
                while (k < obj.size() && (obj[k] == ' ' || obj[k] == '\t')) ++k;
                try { return std::stod(obj.substr(k)); } catch (...) { return def; }
            };
            auto boolField = [](const std::string& obj, const std::string& key, bool def) -> bool {
                const std::string n = "\"" + key + "\"";
                size_t k = obj.find(n);
                if (k == std::string::npos) return def;
                return obj.find("true", k) != std::string::npos &&
                       obj.find("true", k) < obj.find('}', k);
            };
            for (const std::string& s : eachObj(blockOf("sources"))) {
                SymbolSource src;
                src.name = strField(s, "name");
                setTextBuffer(src.elfBuf, strField(s, "elf"));
                setTextBuffer(src.symBuf, strField(s, "symbols"));
                src.defaultNodeId = static_cast<uint8_t>(numField(s, "node", 2.0));
                loadSymbolsInto(src);
                sources_.push_back(std::move(src));
            }
            for (const std::string& s : eachObj(blockOf("watches"))) {
                WatchVar w;
                w.name = strField(s, "name");
                w.address = static_cast<uint32_t>(numField(s, "address"));
                w.type = strField(s, "type");
                if (w.type.empty()) w.type = "uint32";
                w.size = static_cast<uint8_t>(numField(s, "size", 4.0));
                w.nodeId = static_cast<uint8_t>(numField(s, "node_id", 2.0));
                const double periodSec = numField(s, "period_sec", -1.0);
                if (periodSec > 0.0) {
                    w.pollHz = watchHzFromPeriodSec(static_cast<float>(periodSec));
                } else {
                    w.pollHz = clampWatchHz(static_cast<float>(numField(s, "hz", kDefaultWatchHz)));
                }
                w.enabled = boolField(s, "enabled", true);
                w.plot = boolField(s, "plot", false);
                w.plotScale = normalizePlotScale(numField(s, "plot_scale", 1.0));
                w.source = strField(s, "source");
                watches_.push_back(std::move(w));
            }
            varsConfigStatus_ = "loaded " + std::to_string(sources_.size()) +
                                " sources, " + std::to_string(watches_.size()) + " watches";
            lastFileMsg_ = "loaded: " + std::string(varsConfigPath_.data());
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(4 default: motor + rk + dd + esp32%s)",
                        varsConfigStatus_.empty() ? "" : (" - " + varsConfigStatus_).c_str());

    }

    // Horizontal split with a draggable splitter. Left = sources/symbols (gets
    // a horizontal scrollbar when narrow so long names like hcan1.Init.SyncJumpWidth
    // don't wrap). Right = Watch list. Drag the thin bar between them to resize.
    const float availW = ImGui::GetContentRegionAvail().x;
    const float minLeft = 220.0f;
    const float minRight = 240.0f;
    if (varsSplitX_ < minLeft) varsSplitX_ = minLeft;
    if (varsSplitX_ > availW - minRight) varsSplitX_ = std::max(minLeft, availW - minRight);
    ImGui::BeginChild("##vars-left", ImVec2(varsSplitX_, 0), ImGuiChildFlags_Border,
                      ImGuiWindowFlags_AlwaysVerticalScrollbar |
                      ImGuiWindowFlags_HorizontalScrollbar);

    auto keyOf = [](uint32_t addr, uint8_t node) -> uint64_t {
        return (static_cast<uint64_t>(addr) << 8) | node;
    };
    std::unordered_set<uint64_t> watchedKeys;
    std::unordered_set<uint64_t> plottedKeys;
    for (const WatchVar& w : watches_) {
        if (w.enabled) watchedKeys.insert(keyOf(w.address, w.nodeId));
        if (w.plot)    plottedKeys.insert(keyOf(w.address, w.nodeId));
    }

    int removeIdx = -1;
    for (size_t si = 0; si < sources_.size(); ++si) {
        SymbolSource& src = sources_[si];
        ImGui::PushID(static_cast<int>(si));
        int srcWatched = 0;
        for (const auto& s : src.symbols) {
            if (watchedKeys.count(keyOf(s.address, src.defaultNodeId))) ++srcWatched;
        }
        const std::string headerLabel =
            countLabel(src.name.empty() ? std::string("source") : src.name,
                       srcWatched, src.symbols.size(), "watched") +
            ", node " + std::to_string(static_cast<int>(src.defaultNodeId));
        if (ImGui::CollapsingHeader(headerLabel.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
            char nameBuf[64];
            std::snprintf(nameBuf, sizeof(nameBuf), "%s", src.name.c_str());
            ImGui::SetNextItemWidth(180);
            if (ImGui::InputText("project name", nameBuf, sizeof(nameBuf))) {
                src.name = nameBuf;
                markScenarioDirty("project changed");
            }
            ImGui::SameLine();
            int defNode = src.defaultNodeId;
            ImGui::SetNextItemWidth(70);
            if (ImGui::InputInt("default node", &defNode, 1, 1)) {
                if (defNode < 0) defNode = 0;
                if (defNode > 127) defNode = 127;
                src.defaultNodeId = static_cast<uint8_t>(defNode);
                for (VariableSymbol& s : src.symbols) s.nodeId = src.defaultNodeId;
                markScenarioDirty("project node changed");
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("remove project")) {
                removeIdx = static_cast<int>(si);
                markScenarioDirty("project removed");
            }

            /* ELF path row: text field + Browse + Export+Load.
             * Browse opens an OS-native file picker pre-pointed at the
             * current ELF directory. Convenient when the user just
             * rebuilt firmware in a fresh repo checkout and the cached
             * path is stale вЂ” no need to retype the full path. */
            ImGui::SetNextItemWidth(-260);
            if (ImGui::InputText("ELF", src.elfBuf.data(), src.elfBuf.size())) {
                markScenarioDirty("project changed");
            }
            ImGui::SameLine();
            if (ImGui::Button("Browse...##elf")) {
                std::string chosen = src.elfBuf.data();
                if (nativeFileDialog(/*save=*/false,
                                     "ELF\0*.elf\0app_dd ELF\0app_dd*\0All files\0*.*\0",
                                     chosen)) {
                    setTextBuffer(src.elfBuf, chosen);
                    markScenarioDirty("ELF path changed");
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Export+Load##e")) {
                exportElfAndLoadInto(src);
                markScenarioDirty("symbols exported");
            }

            /* symbols.json path row: text field + Browse + Load.
             * The Load button reuses a pre-generated JSON without
             * invoking elf_export, so DriveScope ships fully portable
             * for end users: copy the dist/ folder to a USB drive,
             * launch DriveScope.exe on any Windows box, watch the
             * bundled symbols/*.json get picked up automatically.
             * Browse lets the user swap to a different JSON without
             * editing the path manually. */
            ImGui::SetNextItemWidth(-260);
            if (ImGui::InputText("symbols.json", src.symBuf.data(), src.symBuf.size())) {
                markScenarioDirty("project changed");
            }
            ImGui::SameLine();
            if (ImGui::Button("Browse...##sym")) {
                std::string chosen = src.symBuf.data();
                if (nativeFileDialog(/*save=*/false,
                                     "Symbols JSON\0*.json\0All files\0*.*\0",
                                     chosen)) {
                    setTextBuffer(src.symBuf, chosen);
                    markScenarioDirty("symbols path changed");
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Load##s")) {
                loadSymbolsInto(src);
                markScenarioDirty("symbols loaded");
            }

            ImGui::TextDisabled("Status: %s", src.status.empty() ? "(not loaded)" : src.status.c_str());

            ImGui::SetNextItemWidth(-1);
            ImGui::InputTextWithHint("##sflt", "filter symbols by name",
                                     src.filterBuf.data(), src.filterBuf.size());
            const char* needle = src.filterBuf.data();

            std::map<std::string, std::vector<size_t>> byFile;
            for (size_t i = 0; i < src.symbols.size(); ++i) {
                if (needle[0] != '\0' && !containsCi(src.symbols[i].name, needle)) continue;
                const std::string& f = src.symbols[i].file;
                byFile[f.empty() ? std::string("(unknown)") : f].push_back(i);
            }

            ImGui::PushFont(monoUiFont());
            ImGui::BeginChild("##symtree", ImVec2(0, 300), ImGuiChildFlags_Border);
            for (auto& kv : byFile) {
                const std::string& fname = kv.first;
                const std::vector<size_t>& idxs = kv.second;
                int fileWatched = 0;
                for (size_t i : idxs) {
                    if (watchedKeys.count(keyOf(src.symbols[i].address, src.defaultNodeId))) {
                        ++fileWatched;
                    }
                }
                const std::string label = fname + "  " +
                    std::to_string(fileWatched) + " / " +
                    std::to_string(idxs.size()) + "###varfile-" + src.name + "-" + fname;
                if (ImGui::TreeNodeEx(label.c_str(), ImGuiTreeNodeFlags_SpanAvailWidth)) {
                    const ImGuiTableFlags stab =
                        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                        ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable |
                        ImGuiTableFlags_Hideable | ImGuiTableFlags_SizingStretchProp;
                    if (ImGui::BeginTable("symbols", 6, stab, ImVec2(0, 0))) {
                        ImGui::TableSetupColumn("name", 0, 2.4f);
                        ImGui::TableSetupColumn("address", 0, 1.0f);
                        ImGui::TableSetupColumn("type", 0, 0.6f);
                        ImGui::TableSetupColumn("size", 0, 0.4f);
                        ImGui::TableSetupColumn("node_id", 0, 0.6f);
                        ImGui::TableSetupColumn("actions", 0, 1.0f);
                        ImGui::TableHeadersRow();

                        ImGuiListClipper clip;
                        clip.Begin(static_cast<int>(idxs.size()));
                        while (clip.Step()) {
                            for (int row = clip.DisplayStart; row < clip.DisplayEnd; ++row) {
                                const size_t i = idxs[row];
                                VariableSymbol& s = src.symbols[i];
                                const uint64_t k = keyOf(s.address, src.defaultNodeId);
                                const bool isPlotted = plottedKeys.count(k) != 0;
                                const bool isWatched = watchedKeys.count(k) != 0;
                                ImGui::PushID(static_cast<int>(i));
                                ImGui::TableNextRow();
                                if (isPlotted) {
                                    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0,
                                        IM_COL32(0x24, 0x37, 0x48, 0x95));
                                } else if (isWatched) {
                                    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0,
                                        IM_COL32(0x2A, 0x36, 0x28, 0x90));
                                }
                                if (isPlotted)      ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.70f, 0.84f, 0.96f, 1.0f));
                                else if (isWatched) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.76f, 0.88f, 0.66f, 1.0f));
                                ImGui::TableNextColumn();
                                if (isPlotted)      ImGui::TextUnformatted("plot");
                                else if (isWatched) ImGui::TextUnformatted("watch");
                                else                ImGui::TextUnformatted("");
                                ImGui::SameLine();
                                ImGui::TextUnformatted(s.name.c_str());
                                ImGui::TableNextColumn(); ImGui::TextUnformatted(idToHex(s.address, true).c_str());
                                ImGui::TableNextColumn(); ImGui::TextUnformatted(s.type.c_str());
                                ImGui::TableNextColumn(); ImGui::Text("%u", s.size);
                                ImGui::TableNextColumn();
                                ImGui::SetNextItemWidth(-1);
                                ImGui::InputScalar("##node", ImGuiDataType_U8, &s.nodeId);
                                ImGui::TableNextColumn();
                                {
                                    const uint8_t effNode = (s.nodeId == 0) ? src.defaultNodeId : s.nodeId;
                                    const bool inList = isWatched || isPlotted;
                                    const std::string btn = std::string(inList ? "Remove" : "Add") + "##t";
                                    if (ImGui::SmallButton(btn.c_str())) {
                                        VariableSymbol effective = s; effective.nodeId = effNode;
                                        togglePlot(effective, src.name, effNode);
                                    }
                                }
                                if (isPlotted || isWatched) ImGui::PopStyleColor();
                                ImGui::PopID();
                            }
                        }
                        clip.End();
                        ImGui::EndTable();
                    }
                    ImGui::TreePop();
                }
            }
            ImGui::EndChild();
            ImGui::PopFont();
        }
        ImGui::PopID();
    }
    if (removeIdx >= 0 && removeIdx < static_cast<int>(sources_.size())) {
        sources_.erase(sources_.begin() + removeIdx);
        markScenarioDirty("project removed");
    }

    ImGui::EndChild();   // ##vars-left

    ImGui::SameLine(0.0f, 0.0f);
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0,0,0,0));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.45f, 0.55f, 0.75f, 0.40f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.45f, 0.65f, 0.95f, 0.80f));
    const float splitterH = ImGui::GetContentRegionAvail().y;
    ImGui::Button("##vars-splitter", ImVec2(6.0f, splitterH));
    ImGui::PopStyleColor(3);
    if (ImGui::IsItemActive()) {
        varsSplitX_ += ImGui::GetIO().MouseDelta.x;
        if (varsSplitX_ < minLeft) varsSplitX_ = minLeft;
        if (varsSplitX_ > availW - minRight) varsSplitX_ = availW - minRight;
    }
    if (ImGui::IsItemHovered() || ImGui::IsItemActive()) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    }
    ImGui::SameLine(0.0f, 0.0f);

    ImGui::BeginChild("##vars-right", ImVec2(0, 0), ImGuiChildFlags_Border,
                      ImGuiWindowFlags_AlwaysVerticalScrollbar |
                      ImGuiWindowFlags_HorizontalScrollbar);

    ImGui::SeparatorText("Watch");
    int watchRemove = -1;
    ImGui::PushFont(monoUiFont());
    if (ImGui::BeginTable("watch", 10, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
                                          ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable |
                                          ImGuiTableFlags_SizingStretchProp,
                          ImVec2(0, -1))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("on",     ImGuiTableColumnFlags_WidthFixed, 42.0f);
        ImGui::TableSetupColumn("plot",   ImGuiTableColumnFlags_WidthFixed, 46.0f);
        ImGui::TableSetupColumn("src",    0, 0.75f);
        ImGui::TableSetupColumn("name",   0, 1.90f);
        ImGui::TableSetupColumn("addr",   0, 0.95f);
        ImGui::TableSetupColumn("type",   0, 0.75f);
        ImGui::TableSetupColumn("node",   ImGuiTableColumnFlags_WidthFixed, 50.0f);
        ImGui::TableSetupColumn("period", ImGuiTableColumnFlags_WidthFixed, 132.0f);
        ImGui::TableSetupColumn("value",  0, 1.10f);
        ImGui::TableSetupColumn("del",    ImGuiTableColumnFlags_WidthFixed, 40.0f);
        ImGui::TableHeadersRow();
        for (size_t i = 0; i < watches_.size(); ++i) {
            ImGui::PushID(static_cast<int>(i));
            WatchVar& w = watches_[i];
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            if (ImGui::Checkbox("##en", &w.enabled)) {
                markScenarioDirty("watch changed");
            }
            ImGui::TableNextColumn();
            if (ImGui::Checkbox("##plot", &w.plot)) {
                markScenarioDirty("watch changed");
            }
            ImGui::TableNextColumn(); ImGui::TextUnformatted(w.source.c_str());
            ImGui::TableNextColumn(); ImGui::TextUnformatted(w.name.c_str());
            ImGui::TableNextColumn(); ImGui::TextUnformatted(idToHex(w.address, true).c_str());
            ImGui::TableNextColumn(); ImGui::TextUnformatted(w.type.c_str());
            ImGui::TableNextColumn(); ImGui::Text("%u", w.nodeId);
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(130.0f);
            float periodSec = periodSecFromWatchHz(w.pollHz);
            if (ImGui::SliderFloat("##period", &periodSec, kMinWatchPeriodSec, kMaxWatchPeriodSec,
                                   "%.2f s", ImGuiSliderFlags_Logarithmic)) {
                w.pollHz = watchHzFromPeriodSec(periodSec);
                w.nextPollTime = 0.0;
                markScenarioDirty("watch changed");
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("READ_MEM polling period for this variable");
            }
            ImGui::TableNextColumn(); ImGui::TextUnformatted(w.valueText.c_str());
            ImGui::TableNextColumn();
            if (ImGui::SmallButton("X##rm")) watchRemove = static_cast<int>(i);
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    ImGui::PopFont();
    if (watchRemove >= 0 && watchRemove < static_cast<int>(watches_.size())) {
        watches_.erase(watches_.begin() + watchRemove);
        pending_.clear();
        pendingSentAt_.clear();
        nextSeq_ = 1;
        pollCursor_ = 0;
        markScenarioDirty("watch removed");
    }
    ImGui::EndChild();   // ##vars-right
    ImGui::EndChild();   // ##vars-scroll
}

// Shared device-control header strip. Drawn at the top of every tab body so
// the operator can take or release device control from wherever they are.
// State is the single diagModeOn_ / lastDiagStatus_ pair -- the same one the
// Remote Control tab and the motor logic already use, so there is exactly one
// source of truth and the indicator stays consistent across all tabs.
void MainUi::drawDeviceControlBar()
{
    const bool canSend = activeTransport_ && activeTransport_->isOpen();
    const bool inDiag  = lastDiagStatus_.valid ? lastDiagStatus_.diag_active
                                               : diagModeOn_;

    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("Device:");
    ImGui::SameLine();
    if (inDiag)
        ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.45f, 1.0f), "PC-controlled (diag)");
    else
        ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.25f, 1.0f), "device-owned");

    ImGui::SameLine(0.0f, 16.0f);
    ImGui::BeginDisabled(!canSend);
    if (!inDiag) {
        if (ImGui::Button("Take control"))
            diagModeRequest(true);
    } else {
        if (ImGui::Button("Release control"))
            diagModeRequest(false);
    }
    ImGui::EndDisabled();
    if (!canSend && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Connect a transport (Connection tab) to take control");
    }

    ImGui::Separator();
}

void MainUi::drawTabStrip()
{
    // Visibility toggle bar (was selector). With docking enabled each tab is
    // its own ImGui window inside the dockspace, so this strip's job is now:
    //   - Show which tab windows are currently open (highlighted)
    //   - Click an open one  -> bring it to focus / front in its dock node
    //   - Click a closed one -> reopen it (it'll re-dock to its last spot)
    // Right-click any button for "Show only this" / "Reset layout" actions.
    struct TabDef { const char* label; ToolIcon icon; };
    static const TabDef tabs[] = {
        { "Connection",     ToolIcon::TabConnection },
        { "CAN Monitor",    ToolIcon::TabMonitor    },
        { "Variables",      ToolIcon::TabVariables  },
        { "Plots",          ToolIcon::TabPlots      },
        { "Remote Control", ToolIcon::TabRemote      },
        { "Calibration",    ToolIcon::TabCalibration },
        { "Motor Config",   ToolIcon::TabUpdates     },   // penultimate, reuses icon
        { "Updates",        ToolIcon::TabUpdates     },
        { "Load Stand",     ToolIcon::TabPlots       },
    };
    static_assert(IM_ARRAYSIZE(tabs) == kNumTabs, "tab strip / state mismatch");

    float stripBottomY = ImGui::GetCursorScreenPos().y;
    for (int i = 0; i < kNumTabs; ++i) {
        if (i > 0) ImGui::SameLine(0.0f, 4.0f);
        char id[24];
        std::snprintf(id, sizeof(id), "##tab%d", i);
        if (customTab(id, tabs[i].label, tabs[i].icon, tabOpen_[i])) {
            if (!tabOpen_[i]) {
                tabOpen_[i] = true;
            }
            tabFocusReq_[i] = true;
            activeTab_ = i;
        }
        if (ImGui::IsItemHovered() && !tabOpen_[i]) {
            ImGui::SetTooltip("Click to reopen %s", tabs[i].label);
        } else if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Click to focus %s\nDrag the window's title bar out to detach it into a separate OS window",
                              tabs[i].label);
        }
        if (ImGui::BeginPopupContextItem(id)) {
            if (ImGui::MenuItem("Show only this")) {
                for (int j = 0; j < kNumTabs; ++j) tabOpen_[j] = (j == i);
                tabFocusReq_[i] = true;
            }
            if (ImGui::MenuItem("Show all tabs")) {
                for (int j = 0; j < kNumTabs; ++j) tabOpen_[j] = true;
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Reset window layout")) {
                firstDockLayout_ = true;
                for (int j = 0; j < kNumTabs; ++j) tabOpen_[j] = true;
            }
            ImGui::EndPopup();
        }
        if (i == 0) stripBottomY = ImGui::GetItemRectMax().y;
    }

    // Hairline under the tab strip -- anchors the active tab to the panel.
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float winX = ImGui::GetWindowPos().x;
    const float winW = ImGui::GetWindowSize().x;
    dl->AddLine(ImVec2(winX, stripBottomY),
                ImVec2(winX + winW, stripBottomY),
                IM_COL32(75, 75, 80, 200), 1.0f);
    ImGui::Dummy(ImVec2(0.0f, 8.0f));
}

void MainUi::drawDeviceUpdates()
{
    if (mainDeployRunning_ && mainDeployFuture_.valid() &&
        mainDeployFuture_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        mainDeployStatus_ = mainDeployFuture_.get();
        mainDeployRunning_ = false;
        preserveTransportDuringDeploy_ = false;
        const bool disconnectAfterDeploy = disconnectAfterMainDeploy_;
        disconnectAfterMainDeploy_ = false;
        if (mainDeployProgress_) {
            mainDeployProgress_->active.store(false, std::memory_order_relaxed);
        }

        if (disconnectAfterDeploy) {
            disconnectTransport();
            if (mainDeployStatus_.find("disconnected") == std::string::npos) {
                mainDeployStatus_ += " -- disconnected";
            }
        }
    }

    const bool connected = isConnected();
    const CanBootloaderFlashSnapshot flash = bootFlasher_.snapshot();
    auto browsePath = [](std::array<char, 260>& buf, const char* filter) {
        std::string path = buf.data();
        if (nativeFileDialog(false, filter, path)) {
            setTextBuffer(buf, path);
            return true;
        }
        return false;
    };
    auto startFlash = [&](CanBootloaderFlashRequest::Device device, const char* path) {
        CanBootloaderFlashRequest req;
        req.device = device;
        req.filePath = resolveDataPath(path);
        req.enterBoot = true;
        req.returnToApp = true;
        bootFlasher_.start(req, [this](const CanFrame& frame) {
            return sendFrameThreadSafe(frame);
        });
    };
    // (launchScript helper removed -- mainPCB deploy section that called
    //  it has been retired in favour of Emergency AP recovery below.)

#ifdef _WIN32
    const std::string currentSsid = drivescope::wlan::currentSsid();
    const bool onKitchenAp = currentSsid.rfind("Kitchen_Machine_", 0) == 0;
#else
    const bool onKitchenAp = false;
#endif

    ImGui::TextUnformatted("CAN bootloader");
    ImGui::Separator();
    if (!connected) {
        ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.35f, 1.0f),
                           "Connect the USB-CAN dongle first.");
    } else if (transportKind_ != TransportKind::Slcan) {
        ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.25f, 1.0f),
                           "Direct CAN flash is for the USB-CAN dongle. Use Wi-Fi flash below in AP mode.");
    }

    if (ImGui::BeginTable("##updates-can", 3,
                          ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_PadOuterX,
                          ImVec2(0.0f, 0.0f))) {
        ImGui::TableSetupColumn("device", ImGuiTableColumnFlags_WidthFixed, 86.0f);
        ImGui::TableSetupColumn("path", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("action", ImGuiTableColumnFlags_WidthFixed, 150.0f);

        auto firmwareRow = [&](const char* label,
                               std::array<char, 260>& path,
                               CanBootloaderFlashRequest::Device device,
                               const char* buttonId) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(label);
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(-1);
            if (ImGui::InputText((std::string("##") + buttonId + "-path").c_str(), path.data(), path.size())) {
                markScenarioDirty("update path changed");
            }
            ImGui::TableNextColumn();
            if (ImGui::Button((std::string("...##") + buttonId).c_str(), ImVec2(32.0f, 0.0f))) {
                if (browsePath(path, "Firmware (*.bin)\0*.bin\0All files\0*.*\0")) {
                    markScenarioDirty("update path changed");
                }
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Browse firmware .bin");
            ImGui::SameLine();
            const bool disabled = !connected || transportKind_ != TransportKind::Slcan || flash.busy;
            if (disabled) ImGui::BeginDisabled();
            if (ImGui::Button((std::string("Flash##") + buttonId).c_str(), ImVec2(90.0f, 0.0f))) {
                startFlash(device, path.data());
            }
            if (disabled) ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                ImGui::SetTooltip("Direct bootloader flash over USB-CAN/SLCAN. Works without mainPCB if the dongle is on the target CAN bus.");
            }
        };

        firmwareRow("Motor", motorFirmwarePath_, CanBootloaderFlashRequest::Device::Motor, "motor-fw");
        firmwareRow("RK", rkFirmwarePath_, CanBootloaderFlashRequest::Device::Rk, "rk-fw");
        ImGui::EndTable();
    }

    if (flash.busy) {
        if (ImGui::Button("Cancel CAN flash", ImVec2(150.0f, 0.0f))) bootFlasher_.cancel();
    }
    ImGui::ProgressBar(flash.progress / 100.0f, ImVec2(-1.0f, 0.0f), flash.status.c_str());
    if (!flash.log.empty()) {
        ImGui::BeginChild("##flash-log", ImVec2(0.0f, 150.0f), ImGuiChildFlags_Border);
        ImGui::TextUnformatted(flash.log.c_str());
        if (flash.busy) ImGui::SetScrollHereY(1.0f);
        ImGui::EndChild();
    }

    ImGui::Dummy(ImVec2(0.0f, 14.0f));
    ImGui::TextUnformatted("Wi-Fi flash (running app_dd)");
    ImGui::Separator();
#ifdef _WIN32
    ImGui::TextWrapped("Uploads firmware to /app/firmware/<target> over SSH, then asks the running app_dd command server to execute /flash/<target>. Use this in AP mode.");
    const bool wifiFlashDisabled = mainDeployRunning_ || flash.busy || !onKitchenAp;
    auto wifiFlashDisabledTooltip = [&]() {
        if (!ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) return;
        if (!onKitchenAp) {
            ImGui::SetTooltip("Join/connect a Kitchen_Machine_* AP with app_dd running first.");
        } else if (mainDeployRunning_) {
            ImGui::SetTooltip("Wait for the current update to finish.");
        } else if (flash.busy) {
            ImGui::SetTooltip("Wait for the direct CAN flash to finish.");
        }
    };
    auto startWifiFlash = [&](SomFlashKind kind, const std::string& source) {
        const std::string kindName = somFlashKindName(kind);
        mainDeployStatus_ = "wifi flash " + kindName + ": running";
        mainDeployProgress_ = std::make_shared<DeployProgressState>();
        setDeployProgress(mainDeployProgress_, 0, "starting " + kindName + " flash");
        preserveTransportDuringDeploy_ = true;
        mainDeployRunning_ = true;
        mainDeployFuture_ = std::async(std::launch::async,
                                       [kind, source, progress = mainDeployProgress_]() {
            return runSomPeripheralFlash(kind, source, progress);
        });
    };

    if (ImGui::BeginTable("##updates-wifi-flash", 3,
                          ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_PadOuterX,
                          ImVec2(0.0f, 0.0f))) {
        ImGui::TableSetupColumn("device", ImGuiTableColumnFlags_WidthFixed, 86.0f);
        ImGui::TableSetupColumn("path", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("action", ImGuiTableColumnFlags_WidthFixed, 166.0f);

        auto wifiFileRow = [&](const char* label,
                               std::array<char, 260>& path,
                               SomFlashKind kind,
                               const char* buttonId) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(label);
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(-1);
            if (ImGui::InputText((std::string("##wifi-") + buttonId + "-path").c_str(),
                                 path.data(), path.size())) {
                markScenarioDirty("update path changed");
            }
            ImGui::TableNextColumn();
            if (ImGui::Button((std::string("...##wifi-") + buttonId).c_str(), ImVec2(32.0f, 0.0f))) {
                if (browsePath(path, "Firmware (*.bin)\0*.bin\0All files\0*.*\0")) {
                    markScenarioDirty("update path changed");
                }
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Browse firmware .bin");
            ImGui::SameLine();
            if (wifiFlashDisabled) ImGui::BeginDisabled();
            if (ImGui::Button((std::string("Upload+Flash##wifi-") + buttonId).c_str(), ImVec2(120.0f, 0.0f))) {
                startWifiFlash(kind, path.data());
            }
            if (wifiFlashDisabled) ImGui::EndDisabled();
            wifiFlashDisabledTooltip();
        };

        wifiFileRow("Motor", motorFirmwarePath_, SomFlashKind::Motor, "motor-fw");
        wifiFileRow("RK", rkFirmwarePath_, SomFlashKind::Rk, "rk-fw");

        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("ESP32");
        ImGui::TableNextColumn();
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputText("##wifi-esp32-dir", esp32FirmwareDir_.data(), esp32FirmwareDir_.size())) {
            markScenarioDirty("update path changed");
        }
        ImGui::TableNextColumn();
        if (ImGui::Button("...##wifi-esp32-dir-pick", ImVec2(32.0f, 0.0f))) {
            std::string path = std::filesystem::path(esp32FirmwareDir_.data()).append("firmware.bin").string();
            if (nativeFileDialog(false, "ESP32 firmware.bin\0firmware.bin\0Firmware (*.bin)\0*.bin\0All files\0*.*\0", path)) {
                setTextBuffer(esp32FirmwareDir_, std::filesystem::path(path).parent_path().string());
                markScenarioDirty("update path changed");
            }
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Pick firmware.bin inside the ESP32 bundle folder");
        ImGui::SameLine();
        if (wifiFlashDisabled) ImGui::BeginDisabled();
        if (ImGui::Button("Upload+Flash##wifi-esp32-fw", ImVec2(120.0f, 0.0f))) {
            startWifiFlash(SomFlashKind::Esp32, esp32FirmwareDir_.data());
        }
        if (wifiFlashDisabled) ImGui::EndDisabled();
        wifiFlashDisabledTooltip();

        ImGui::EndTable();
    }
#else
    ImGui::TextDisabled("Wi-Fi firmware flash is available on Windows builds.");
#endif

    // The old "mainPCB deploy" block (free-form script Run buttons for
    // app_dd and all /app files) has been removed: production updates of
    // /app go through Emergency AP recovery below, which is the single
    // documented path. The mainAppScriptPath_/mainResourcesScriptPath_
    // members are still loaded/saved from scenario for backwards-compat
    // with older config files, just no longer surfaced in the UI.

    ImGui::Dummy(ImVec2(0.0f, 14.0f));
    ImGui::TextUnformatted("Emergency AP recovery");
    ImGui::Separator();
#ifdef _WIN32
    if (!recoveryAppDirInitialized_) {
        setTextBuffer(recoveryAppDir_, defaultRecoveryAppDir());
        setTextBuffer(recoveryOtaPath_, defaultRecoveryOtaPath());
        setTextBuffer(esp32FirmwareDir_, defaultEsp32FirmwareDir());
        recoveryAppDirInitialized_ = true;
    }

    const double now = ImGui::GetTime();
    // Don't trigger WLAN scans while a deploy is in flight: WlanGetAvailableNetworkList
    // does an active scan that briefly takes the radio off-channel, which has been
    // observed to break the SSH/SFTP upload mid-stream ("Software caused connection
    // abort" at 1-19 MB depending on luck). The same upload run from a plain shell
    // outside DriveScope completes cleanly, which points squarely at our background
    // scanner. Pause it while uploads/installs run; resume normally afterward.
    if (!mainDeployRunning_ &&
        !wlanScanning_ && !wlanConnecting_ && now >= wlanNextScanAt_) {
        wlanNextScanAt_ = now + 4.0;
        scanWlanAsync();
    }

    const bool recoveryActionDisabled = mainDeployRunning_ || !onKitchenAp;
    auto recoveryDisabledTooltip = [&]() {
        if (!ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) return;
        if (!onKitchenAp) {
            ImGui::SetTooltip("Join a Kitchen_Machine_* hotspot first.");
        } else if (mainDeployRunning_) {
            ImGui::SetTooltip("Wait for the current update to finish.");
        }
    };

    ImGui::TextWrapped("One Connect below: device into AP mode (via CAN if open), "
                       "join its Kitchen_Machine_* hotspot, stop app_dd / httpd so "
                       "Upload /app has no fight. Then Upload, optional Run OTA, Run App.");

    // -----------------------------------------------------------------
    // Connect (service-mode) entry. SSID dropdown lets the operator
    // override the auto-pick (strongest signal); Connect button kicks
    // startServiceConnect() which handles the whole flow.
    // -----------------------------------------------------------------
    if (ImGui::BeginTable("##service-connect-row", 3,
                          ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_PadOuterX,
                          ImVec2(0.0f, 0.0f))) {
        ImGui::TableSetupColumn("label",  ImGuiTableColumnFlags_WidthFixed, 120.0f);
        ImGui::TableSetupColumn("ssid",   ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("button", ImGuiTableColumnFlags_WidthFixed, 130.0f);

        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("Hotspot");
        ImGui::TableNextColumn();
        ImGui::SetNextItemWidth(-1);

        std::vector<std::string> labels;
        labels.reserve(wlanScan_.size());
        for (const auto& w : wlanScan_) {
            char buf[160];
            std::snprintf(buf, sizeof(buf), "%s%s   (signal %d%%)",
                          w.ssid.c_str(),
                          w.connected ? " - connected" : "",
                          w.signal_pct);
            labels.push_back(buf);
        }
        std::vector<const char*> labelPtrs;
        labelPtrs.reserve(labels.size());
        for (const auto& s : labels) labelPtrs.push_back(s.c_str());

        if (wlanScan_.empty()) {
            ImGui::TextDisabled("%s", wlanScanning_
                                      ? "(scanning Wi-Fi for Kitchen_Machine_*...)"
                                      : "(no Kitchen_Machine_* SSIDs visible yet)");
        } else {
            int sel = wlanSelected_;
            if (sel < 0 || sel >= static_cast<int>(wlanScan_.size())) {
                int bestSignal = -1;
                for (size_t i = 0; i < wlanScan_.size(); ++i) {
                    if (wlanScan_[i].connected) { sel = static_cast<int>(i); break; }
                    if (static_cast<int>(wlanScan_[i].signal_pct) > bestSignal) {
                        bestSignal = wlanScan_[i].signal_pct;
                        sel = static_cast<int>(i);
                    }
                }
                if (sel < 0) sel = 0;
            }
            if (ImGui::Combo("##service-wlan-ssid", &sel,
                             labelPtrs.data(), static_cast<int>(labelPtrs.size()))) {
                wlanSelected_ = sel;
            }
            if (wlanSelected_ < 0 && sel >= 0) wlanSelected_ = sel;
        }

        ImGui::TableNextColumn();
        const bool busy = mainDeployRunning_ || serviceConnectRunning_;
        if (busy) ImGui::BeginDisabled();
        const char* label = serviceConnectRunning_ ? "Connecting..." : "Connect";
        if (ImGui::Button(label, ImVec2(120.0f, 28.0f))) {
            startServiceConnect();
        }
        if (busy) ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            ImGui::SetTooltip("Connect to service mode for an /app update.");
        }

        ImGui::EndTable();
    }

    if (!serviceConnectStatus_.empty()) {
        ImGui::TextWrapped("%s", serviceConnectStatus_.c_str());
    }

    ImGui::Dummy(ImVec2(0.0f, 6.0f));

    if (ImGui::BeginTable("##updates-recovery", 3,
                          ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_PadOuterX,
                          ImVec2(0.0f, 0.0f))) {
        ImGui::TableSetupColumn("what", ImGuiTableColumnFlags_WidthFixed, 120.0f);
        ImGui::TableSetupColumn("target", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("action", ImGuiTableColumnFlags_WidthFixed, 280.0f);

        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("App folder");
        ImGui::TableNextColumn();
        ImGui::SetNextItemWidth(-1);
        // Recovery input fields are session-local -- see the picker note
        // below. Editing here does not modify scenario.json.
        ImGui::InputText("##recovery-app-dir", recoveryAppDir_.data(), recoveryAppDir_.size());
        ImGui::TableNextColumn();
        // Browse button stays clickable regardless of AP connection -- the
        // user must be able to pre-pick the path before joining the AP.
        if (ImGui::Button("...##recovery-app-dir-pick", ImVec2(32.0f, 0.0f))) {
            std::string path = std::filesystem::path(recoveryAppDir_.data()).append("app_dd").string();
            if (nativeFileDialog(false, "app_dd\0app_dd\0All files\0*.*\0", path)) {
                // In-memory only -- DO NOT markScenarioDirty. The shipped
                // scenario must keep its relative-to-release-folder paths
                // (`../SSD202D/app`); a tester running from build/pc-tool/
                // who picks an absolute D:\BORK\DDV2\... path would otherwise
                // poison the canonical scenario file with a path that's
                // wrong for the actual release bundle.
                setTextBuffer(recoveryAppDir_, std::filesystem::path(path).parent_path().string());
            }
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Pick the app_dd binary inside the SSD202D/app folder (this session only -- does NOT modify scenario)");
        ImGui::SameLine();
        if (recoveryActionDisabled) ImGui::BeginDisabled();
        if (ImGui::Button("Upload /app##recovery-sync", ImVec2(118.0f, 0.0f))) {
            const std::string appDir = recoveryAppDir_.data();
            mainDeployStatus_ = "recovery /app sync: running";
            mainDeployProgress_ = std::make_shared<DeployProgressState>();
            setDeployProgress(mainDeployProgress_, 0, "starting app update");
            preserveTransportDuringDeploy_ = true;
            disconnectAfterMainDeploy_ = true;
            mainDeployRunning_ = true;
            mainDeployFuture_ = std::async(std::launch::async,
                                           [appDir, progress = mainDeployProgress_]() {
                return runRecoveryInstall(appDir, progress);
            });
        }
        if (recoveryActionDisabled) ImGui::EndDisabled();
        recoveryDisabledTooltip();

        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Linux OTA");
        ImGui::TableNextColumn();
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputText("##recovery-ota-path", recoveryOtaPath_.data(), recoveryOtaPath_.size())) {
            markScenarioDirty("update path changed");
        }
        ImGui::TableNextColumn();
        // Same as the app picker: always clickable, even off the AP.
        if (ImGui::Button("...##recovery-ota-path-pick", ImVec2(32.0f, 0.0f))) {
            std::string path = recoveryOtaPath_.data();
            if (nativeFileDialog(false, "OTA package (*.gz)\0*.gz\0All files\0*.*\0", path)) {
                // In-memory only -- see the matching note on the App folder
                // picker. Manual file picks from the Recovery section are
                // session-local and do NOT persist into scenario.json.
                setTextBuffer(recoveryOtaPath_, path);
            }
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Pick SStarOta.bin.gz (this session only -- does NOT modify scenario)");
        ImGui::SameLine();
        if (recoveryActionDisabled) ImGui::BeginDisabled();
        if (ImGui::Button("Run OTA##recovery-ota", ImVec2(98.0f, 0.0f))) {
            const std::string otaPath = recoveryOtaPath_.data();
            const std::string appDir = recoveryAppDir_.data();
            mainDeployStatus_ = "recovery Linux OTA: running";
            mainDeployProgress_ = std::make_shared<DeployProgressState>();
            setDeployProgress(mainDeployProgress_, 0, "starting Linux OTA");
            preserveTransportDuringDeploy_ = true;
            disconnectAfterMainDeploy_ = true;
            mainDeployRunning_ = true;
            mainDeployFuture_ = std::async(std::launch::async,
                                           [otaPath, appDir, progress = mainDeployProgress_]() {
                return runRecoveryLinuxOta(otaPath, appDir, progress);
            });
        }
        if (recoveryActionDisabled) ImGui::EndDisabled();
        recoveryDisabledTooltip();

        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Device");
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(onKitchenAp ? currentSsid.c_str() : "not on Kitchen_Machine_*");
        ImGui::TableNextColumn();
        if (recoveryActionDisabled) ImGui::BeginDisabled();
        if (ImGui::Button("Run App##recovery-runapp", ImVec2(86.0f, 0.0f))) {
            mainDeployStatus_ = "recovery run app: running";
            mainDeployProgress_ = std::make_shared<DeployProgressState>();
            setDeployProgress(mainDeployProgress_, 20, "starting app_dd over SSH");
            preserveTransportDuringDeploy_ = true;
            disconnectAfterMainDeploy_ = true;
            mainDeployRunning_ = true;
            mainDeployFuture_ = std::async(std::launch::async, [progress = mainDeployProgress_]() {
                setDeployProgress(progress, 60, "launching /app/app_dd");
                const std::string result = runRecoveryLaunchApp();
                setDeployProgress(progress, 100,
                    result.find("app_dd started") != std::string::npos ||
                    result.find("already running") != std::string::npos
                        ? "app_dd launch sent"
                        : "run app: see status");
                return result;
            });
        }
        if (recoveryActionDisabled) ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            ImGui::SetTooltip("Start /app/app_dd over Bitvise/SSH, like running it manually in a terminal.\n"
                              "pc_tool will disconnect and will NOT auto-connect; wait a few seconds, then Connect manually.");
        } else {
            recoveryDisabledTooltip();
        }
        ImGui::SameLine();
        if (recoveryActionDisabled) ImGui::BeginDisabled();
        if (ImGui::Button("Reset device##recovery-reset", ImVec2(118.0f, 0.0f))) {
            mainDeployStatus_ = "recovery reset: running";
            mainDeployProgress_ = std::make_shared<DeployProgressState>();
            setDeployProgress(mainDeployProgress_, 15, "sending SSH reboot");
            preserveTransportDuringDeploy_ = false;
            disconnectAfterMainDeploy_ = true;
            mainDeployRunning_ = true;
            mainDeployFuture_ = std::async(std::launch::async, [progress = mainDeployProgress_]() {
                setDeployProgress(progress, 40, "sync; reboot over SSH");
                const std::string result = runRecoveryReset();
                setDeployProgress(progress, 100, "reset command sent");
                return result;
            });
        }
        if (recoveryActionDisabled) ImGui::EndDisabled();
        recoveryDisabledTooltip();

        ImGui::EndTable();
    }
    if (!wlanStatus_.empty()) {
        ImGui::TextDisabled("Wi-Fi: %s", wlanStatus_.c_str());
    }
#else
    ImGui::TextDisabled("Emergency AP recovery is available on Windows builds.");
#endif

    if (!mainDeployStatus_.empty()) {
        ImGui::TextWrapped("%s", mainDeployStatus_.c_str());
    }
}

// One-button service-mode connect. Replaces the old Scan / Join AP /
// Service AP triplet -- picks the right path automatically:
//
//   * Already on Kitchen_Machine_*: skip Wi-Fi steps, jump straight to
//     killing app_dd / httpd over SSH.
//   * Open CAN transport (any kind), not yet on AP: send CAN cmd 0x100
//     SET_WIFI_MODE=AP so the device brings up its hotspot, then wait,
//     then scan + WlanConnect, then SSH kill.
//   * No CAN, not on AP: just scan + WlanConnect (assumes the operator
//     held the unit's button to boot it into recovery hostapd already).
//
// All long-running steps run on a worker thread. The UI thread reads
// the live status via `serviceConnectStageText_`, and updateIo() picks
// up the future when it's ready and opens the Wi-Fi AP CAN bridge.
void MainUi::startServiceConnect()
{
#ifdef _WIN32
    if (serviceConnectRunning_) return;
    serviceConnectRunning_ = true;
    serviceConnectStatus_ = "starting...";

    // Snapshot transport state so the worker doesn't touch UI members.
    const bool canOpen = activeTransport_ != nullptr && activeTransport_->isOpen();

    // Honour the operator's SSID pick from the Hotspot dropdown. If they
    // haven't picked one (or the scan list is stale/empty), the worker
    // falls back to the strongest live Kitchen_Machine_* it finds.
    std::string preferredSsid;
    if (wlanSelected_ >= 0 &&
        wlanSelected_ < static_cast<int>(wlanScan_.size())) {
        preferredSsid = wlanScan_[wlanSelected_].ssid;
    }

    serviceConnectStageText_  = std::make_shared<std::string>("starting...");
    serviceConnectStageMutex_ = std::make_shared<std::mutex>();
    auto stageText = serviceConnectStageText_;
    auto stageMtx  = serviceConnectStageMutex_;
    auto setStage = [stageText, stageMtx](const std::string& s) {
        std::lock_guard<std::mutex> lk(*stageMtx);
        *stageText = s;
    };

    // Capture a thread-safe CAN-AP send. sendFrameThreadSafe locks the
    // transport mutex, so this is safe to call from the worker thread.
    auto sendCanApMode = [this]() -> bool {
        CanFrame f{};
        f.id = (uint32_t(1) << 28) | (uint32_t(0x100) << 16) |
               (uint32_t(kCanAddrMain) << 8) | uint32_t(kCanAddrPc);
        f.extended = true;
        f.fd = false;
        f.brs = false;
        f.rx = false;
        f.data.assign(8, 0);
        f.data[0] = 2;  // SET_WIFI_MODE.mode = AP
        f.dlc = 8;
        return sendFrameThreadSafe(f);
    };

    serviceConnectFuture_ = std::async(std::launch::async,
        [canOpen, preferredSsid, sendCanApMode, setStage]() -> std::string {

        std::string cur = drivescope::wlan::currentSsid();
        bool onAp = cur.rfind("Kitchen_Machine_", 0) == 0;

        if (!onAp) {
            // Step 1: ask the device to bring up its hotspot, if we have any
            // way to talk to it. With nothing connected we assume the device
            // is already in recovery boot mode (button-held) and just try
            // to find its SSID.
            if (canOpen) {
                setStage("requesting device AP mode via CAN...");
                invalidateRecoveryHostCache("service connect: AP request");
                if (!sendCanApMode()) {
                    return "service connect: CAN frame send failed (transport closed?)";
                }
                // ForceMode on the device takes ~3-5 s for AP. Give 12 s
                // headroom so hostapd / dnsmasq are definitely up before
                // we start the WLAN scan dance.
                Sleep(12000);
            } else {
                setStage("no CAN transport open -- assuming device already in AP/recovery");
            }

            // Step 2: scan + WlanConnect. Up to ~32 s of patience for the
            // hotspot to show up in the OS-level scan results.
            setStage("scanning Wi-Fi for Kitchen_Machine_*...");
            std::string targetSsid;
            for (int i = 0; i < 8 && targetSsid.empty(); ++i) {
                auto r = drivescope::wlan::scan("Kitchen_Machine_");
                if (r.ok) {
                    // Prefer the operator's pick from the dropdown; fall
                    // back to the strongest live signal if it's gone or
                    // wasn't set.
                    if (!preferredSsid.empty()) {
                        for (const auto& e : r.entries) {
                            if (e.ssid == preferredSsid) {
                                targetSsid = e.ssid;
                                break;
                            }
                        }
                    }
                    if (targetSsid.empty()) {
                        int bestSignal = -1;
                        for (const auto& e : r.entries) {
                            if (e.signal_pct > bestSignal) {
                                bestSignal = e.signal_pct;
                                targetSsid = e.ssid;
                            }
                        }
                    }
                }
                if (!targetSsid.empty()) break;
                Sleep(4000);
            }
            if (targetSsid.empty()) {
                return "service connect: no Kitchen_Machine_* SSID visible after 30 s. "
                       "Confirm the laptop has Wi-Fi enabled and the device hostapd is up.";
            }

            setStage("joining " + targetSsid + "...");
            std::string err;
            if (!drivescope::wlan::connect(targetSsid, "bork2025", err)) {
                return "service connect: WlanConnect failed: " + err;
            }

            // Wait for OS to report association.
            for (int j = 0; j < 50; ++j) {  // 25 s
                Sleep(500);
                if (drivescope::wlan::currentSsid() == targetSsid) {
                    onAp = true;
                    cur = targetSsid;
                    break;
                }
            }
            if (!onAp) {
                return "service connect: WlanConnect issued but OS hasn't "
                       "reported association after 25 s -- check password, "
                       "or try Wi-Fi tray.";
            }
        }

        // Step 3: stop everything that would fight an SFTP upload -- app_dd
        // (CPU-heavy UI/CAN bridge), httpd (holds /app/httpd open),
        // otaunpack/xor_file (also touch /app). The killall is run via
        // the same SSH wrapper that Upload /app uses, so any host-key
        // probing / fingerprint pinning is shared.
        setStage("stopping app_dd / httpd on device over SSH...");
        invalidateRecoveryHostCache("service connect: pre-kill");
        const CommandResult kr = runRecoveryRemoteSh(
            gracefulStopAppDdSh(/*includeHttpd=*/true) +
            "sleep 1; "
            "echo PRE_KILL_OK",
            20000, 12000, "service-connect-kill");
        if (kr.output.find("PRE_KILL_OK") == std::string::npos) {
            return std::string("service connect: cannot reach device sshd. ") +
                   tailText(kr.output, 300);
        }

        return "service mode ready -- press Upload /app, then Run App";
    });
#else
    serviceConnectStatus_ = "Service-mode connect is available on Windows builds.";
#endif
}

void MainUi::drawRemoteControl()
{
    // Send raw CAN command frames to the device over the active transport.
    // Active transport must be open. SLCAN dongle is the most reliable channel
    // because it doesn't depend on the SOM's IP route -- Wi-Fi flips kill TCP
    // mid-roundtrip and the response (0xB00) never arrives back through the
    // bridge. Over TCP the request still reaches main_pcb (and ForceMode runs)
    // but we lose the ack; over SLCAN both directions survive any flip.
    const bool canSend = activeTransport_ != nullptr && activeTransport_->isOpen();

    // 3 s cooldown so a multi-click doesn't pile up ForceMode() calls before
    // hostapd / wpa_supplicant has finished the previous mode change.
    constexpr double kCooldownSec = 3.0;
    const double now = ImGui::GetTime();
    const bool onCooldown = now < remoteCmdCooldownUntil_;

    auto sendWifiModeCmd = [&](uint8_t mode) {
        if (!canSend || onCooldown) return;
        CanFrame f{};
        // mod=APP(1)<<28 | cmd(0x100)<<16 | dst=MAIN(0x01)<<8 | src=PC(0x10)
        f.id        = (uint32_t(1) << 28) | (uint32_t(0x100) << 16) |
                      (uint32_t(kCanAddrMain) << 8) | uint32_t(kCanAddrPc);
        f.extended  = true;
        f.fd        = false;
        f.brs       = false;
        f.rx        = false;
        f.data.assign(8, 0);
        f.data[0]   = mode;
        f.dlc       = 8;
        sendFrameThreadSafe(f);
        remoteCmdCooldownUntil_ = ImGui::GetTime() + kCooldownSec;

        // If we're riding a Wi-Fi TCP transport and the requested mode change
        // would tear down the route we're on, disconnect proactively so the
        // operator sees an honest "Disconnected" state instead of a "Connected"
        // badge sitting on a dead socket waiting for a TCP RST. SLCAN is
        // independent of Wi-Fi and stays open across any flip.
        if (transportKind_ == TransportKind::Slcan) return;
        bool routeBreaks = false;
        if (transportKind_ == TransportKind::Tcp) {
            // STA-side TCP (192.168.0.x): dies when device leaves STA -> any
            // mode that isn't STA (1) breaks us.
            routeBreaks = (mode != 1);
        } else if (transportKind_ == TransportKind::WifiAp) {
            // AP-side TCP (192.168.1.x): dies when device leaves AP.
            routeBreaks = (mode != 2);
        } else { // Manual
            // Best-effort by subnet of the host the operator typed.
            const std::string h = tcpHost_.data();
            if (h.rfind("192.168.1.", 0) == 0) routeBreaks = (mode != 2);
            else                                routeBreaks = (mode != 1);
        }
        if (routeBreaks) {
            // Defer 500 ms so the bytes actually leave the NIC before close().
            // Without the delay the kernel sometimes RSTs a half-flushed
            // socket and the SOM never receives the frame -> ForceMode
            // doesn't run, mode doesn't change.
            disconnectScheduledAt_ = ImGui::GetTime() + 0.5;
        }
    };

    ImGui::Dummy(ImVec2(0.0f, 6.0f));
    ImGui::TextDisabled("Device Wi-Fi mode (CAN cmd 0x100, ANS_WIFI_STATUS 0xB00 in CAN Monitor)");
    ImGui::Dummy(ImVec2(0.0f, 4.0f));

    const bool buttonsDisabled = !canSend || onCooldown;
    if (buttonsDisabled) ImGui::BeginDisabled();
    if (ImGui::Button("Device -> STA",  ImVec2(140, 30))) sendWifiModeCmd(1);
    ImGui::SameLine();
    if (ImGui::Button("Device -> AP",   ImVec2(140, 30))) sendWifiModeCmd(2);
    ImGui::SameLine();
    if (ImGui::Button("Device -> OFF",  ImVec2(140, 30))) sendWifiModeCmd(0);
    if (buttonsDisabled) ImGui::EndDisabled();

    if (onCooldown && canSend) {
        ImGui::SameLine();
        ImGui::TextDisabled("cooldown %.1fs", remoteCmdCooldownUntil_ - now);
    }

    if (!canSend) {
        ImGui::Dummy(ImVec2(0.0f, 6.0f));
        ImGui::TextColored(ImVec4(0.95f, 0.55f, 0.15f, 1.0f),
                           "Connect a transport (Connection tab) first - these buttons");
        ImGui::TextColored(ImVec4(0.95f, 0.55f, 0.15f, 1.0f),
                           "ride the active CAN transport to reach main_pcb.");
    } else {
        ImGui::Dummy(ImVec2(0.0f, 8.0f));
        ImGui::TextDisabled("Last reply (ANS_WIFI_STATUS):");
        if (lastWifiStatus_.valid) {
            const char* modeName = (lastWifiStatus_.mode == 1) ? "STA"
                                 : (lastWifiStatus_.mode == 2) ? "AP"
                                 : "OFF";
            const double age = ImGui::GetTime() - lastWifiStatus_.receivedAt;
            const ImVec4 col = lastWifiStatus_.ok
                                ? ImVec4(0.55f, 0.85f, 0.55f, 1.0f)
                                : ImVec4(0.95f, 0.55f, 0.15f, 1.0f);
            ImGui::TextColored(col,
                "  mode=%s   ip=%s   ok=%d   (%.1fs ago)",
                modeName,
                lastWifiStatus_.ip.empty() ? "(empty)" : lastWifiStatus_.ip.c_str(),
                lastWifiStatus_.ok ? 1 : 0,
                age);
        } else {
            ImGui::TextDisabled("  (none yet - click a button above; ~3s for AP, ~12s for STA)");
        }
    }

    ImGui::Dummy(ImVec2(0.0f, 16.0f));
    ImGui::Separator();
    ImGui::Dummy(ImVec2(0.0f, 8.0f));
    ImGui::TextDisabled("Node state (BOOT vs APP) -- sniffed from CAN frames");
    ImGui::Dummy(ImVec2(0.0f, 4.0f));
    drawNodeStatePanel(kCanAddrMotor, "Motor (STM32F4)");
    drawNodeStatePanel(kCanAddrRk,    "RK (STM32L4)");

    ImGui::Dummy(ImVec2(0.0f, 16.0f));
    ImGui::Separator();
    ImGui::Dummy(ImVec2(0.0f, 10.0f));
    drawMotorTab();

    ImGui::Dummy(ImVec2(0.0f, 16.0f));
    ImGui::Separator();
    ImGui::Dummy(ImVec2(0.0f, 10.0f));
    drawRkTab();
}

void MainUi::drawNodeStatePanel(uint8_t nodeId, const char* label)
{
    if (nodeId >= kNodeIdMax) return;
    const NodeMode& nm = nodeMode_[nodeId];
    const double now = clock_.nowSeconds();
    const double age = nm.valid ? (now - nm.modSeenAt) : 1e9;
    const bool fresh = nm.valid && age < kNodeModeStaleSec;
    const bool inBoot = fresh && nm.mod == 0u;
    const bool inApp  = fresh && nm.mod == 1u;

    ImGui::PushID(static_cast<int>(nodeId));

    // Coloured state pill.
    ImVec4 col;
    const char* state;
    if (inApp)         { col = ImVec4(0.45f, 0.85f, 0.45f, 1.0f); state = "APP"; }
    else if (inBoot)   { col = ImVec4(0.95f, 0.65f, 0.20f, 1.0f); state = "BOOT"; }
    else if (nm.valid) { col = ImVec4(0.65f, 0.65f, 0.65f, 1.0f); state = "STALE"; }
    else               { col = ImVec4(0.50f, 0.50f, 0.50f, 1.0f); state = "?"; }

    ImGui::Text("%-20s", label);
    ImGui::SameLine();
    ImGui::TextColored(col, "[%s]", state);
    ImGui::SameLine();
    if (nm.valid) {
        ImGui::TextDisabled("(%.1fs ago)", age);
    } else {
        ImGui::TextDisabled("(no frames seen -- try Ping below)");
    }

    // Versions (from a recent ping).
    if (nm.versionValid) {
        ImGui::Text("    boot fw: %s    app fw: %s",
                    nm.bootVersion.c_str(), nm.appVersion.c_str());
    } else {
        ImGui::TextDisabled("    versions: unknown -- click Ping to query");
    }

    // Action buttons.
    const bool canSend = isConnected();
    if (!canSend) ImGui::BeginDisabled();
    if (ImGui::Button("Ping (app)", ImVec2(110, 24))) sendNodePing(nodeId, false);
    ImGui::SameLine();
    if (ImGui::Button("Ping (boot)", ImVec2(110, 24))) sendNodePing(nodeId, true);
    ImGui::SameLine();
    // Go-to-Boot only sensible from APP -- show pressed-out look otherwise.
    {
        const bool gtbDisabled = !inApp;
        if (gtbDisabled) ImGui::BeginDisabled();
        if (ImGui::Button("Go to BOOT", ImVec2(120, 24))) sendNodeGoToBoot(nodeId);
        if (gtbDisabled) ImGui::EndDisabled();
    }
    ImGui::SameLine();
    {
        const bool gtaDisabled = !inBoot;
        if (gtaDisabled) ImGui::BeginDisabled();
        if (ImGui::Button("Go to APP", ImVec2(110, 24))) sendNodeGoToApp(nodeId);
        if (gtaDisabled) ImGui::EndDisabled();
    }
    if (!canSend) ImGui::EndDisabled();

    ImGui::Dummy(ImVec2(0.0f, 6.0f));
    ImGui::PopID();
}

void MainUi::drawPlots()
{
    ensureDefaultSources();

    const float splitW = 7.0f;
    const float plotAvailW = ImGui::GetContentRegionAvail().x;
    const float plotMinLeft = 210.0f;
    const float plotMinRight = 360.0f;
    const float plotMaxLeft = std::max(plotMinLeft, plotAvailW - plotMinRight - splitW);
    plotSplitX_ = std::clamp(plotSplitX_, plotMinLeft, plotMaxLeft);

    ImGui::BeginChild("##plot-left", ImVec2(plotSplitX_, 0), ImGuiChildFlags_Border,
                      ImGuiWindowFlags_AlwaysVerticalScrollbar |
                      ImGuiWindowFlags_HorizontalScrollbar);

    enum class TreeOp { None, ExpandAll, CollapseAll };
    TreeOp treeOp = TreeOp::None;
    if (ImGui::SmallButton("Expand"))   treeOp = TreeOp::ExpandAll;
    ImGui::SameLine();
    if (ImGui::SmallButton("Collapse")) treeOp = TreeOp::CollapseAll;

    ImGui::SetNextItemWidth(-1);
    ImGui::InputTextWithHint("##pflt", "filter symbols by name",
                             plotFilter_.data(), plotFilter_.size());
    const char* pneedle = plotFilter_.data();

    auto keyOfPlot = [](uint32_t addr, uint8_t node) -> uint64_t {
        return (static_cast<uint64_t>(addr) << 8) | node;
    };
    std::unordered_set<uint64_t> plottedKeysL;
    for (const WatchVar& w : watches_) {
        if (w.plot) plottedKeysL.insert(keyOfPlot(w.address, w.nodeId));
    }

    ImGui::PushFont(monoUiFont());
    for (size_t si = 0; si < sources_.size(); ++si) {
        SymbolSource& src = sources_[si];
        ImGui::PushID(static_cast<int>(si));

        int sourcePlotted = 0;
        for (const auto& s : src.symbols) {
            if (plottedKeysL.count(keyOfPlot(s.address, src.defaultNodeId))) ++sourcePlotted;
        }
        const std::string sourceLabel =
            countLabel(src.name, sourcePlotted, src.symbols.size(), "plotted") +
            "###src-" + src.name;

        if (treeOp == TreeOp::ExpandAll)   ImGui::SetNextItemOpen(true);
        else if (treeOp == TreeOp::CollapseAll) ImGui::SetNextItemOpen(false);
        else                                ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);

        const bool sourceOpen = ImGui::TreeNodeEx(sourceLabel.c_str(),
                                                  ImGuiTreeNodeFlags_Framed |
                                                  ImGuiTreeNodeFlags_SpanAvailWidth);
        if (sourceOpen) {
            std::map<std::string, std::vector<size_t>> byFile;
            for (size_t i = 0; i < src.symbols.size(); ++i) {
                if (pneedle[0] != '\0' && !containsCi(src.symbols[i].name, pneedle)) continue;
                const std::string& f = src.symbols[i].file;
                byFile[f.empty() ? std::string("(unknown)") : f].push_back(i);
            }

            for (auto& kv : byFile) {
                const std::string& fname = kv.first;
                const std::vector<size_t>& idxs = kv.second;
                int filePlotted = 0;
                for (size_t i : idxs) {
                    if (plottedKeysL.count(keyOfPlot(src.symbols[i].address, src.defaultNodeId))) {
                        ++filePlotted;
                    }
                }
                const std::string fileLabel = fname + "  " +
                    std::to_string(filePlotted) + " / " +
                    std::to_string(idxs.size()) + "###file-" + src.name + "-" + fname;

                if (treeOp == TreeOp::ExpandAll)        ImGui::SetNextItemOpen(true);
                else if (treeOp == TreeOp::CollapseAll) ImGui::SetNextItemOpen(false);

                ImGui::PushID(fname.c_str());
                if (ImGui::TreeNodeEx(fileLabel.c_str(), ImGuiTreeNodeFlags_SpanAvailWidth)) {
                    for (size_t i : idxs) {
                        const VariableSymbol& s = src.symbols[i];
                        const bool plotted = plottedKeysL.count(keyOfPlot(s.address, src.defaultNodeId)) != 0;
                        ImGui::PushID(static_cast<int>(i));
                        if (plotted) {
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.70f, 0.84f, 0.96f, 1.0f));
                        }
                        const std::string label = s.name;
                        if (ImGui::Selectable(label.c_str(), plotted,
                                              ImGuiSelectableFlags_DontClosePopups)) {
                            togglePlot(s, src.name, src.defaultNodeId);
                        }
                        if (plotted) ImGui::PopStyleColor();
                        if (ImGui::IsItemHovered()) {
                            ImGui::SetTooltip(
                                "%s\nfile=%s\naddr=%s type=%s node=%u\nclick to toggle plot",
                                s.name.c_str(),
                                s.file.empty() ? "(unknown)" : s.file.c_str(),
                                idToHex(s.address, true).c_str(),
                                s.type.c_str(),
                                static_cast<unsigned>(src.defaultNodeId));
                        }
                        ImGui::PopID();
                    }
                    ImGui::TreePop();
                }
                ImGui::PopID();
            }
            ImGui::TreePop();
        }
        ImGui::PopID();
    }
    ImGui::PopFont();
    ImGui::EndChild();

    ImGui::SameLine(0.0f, 0.0f);
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.45f, 0.55f, 0.75f, 0.40f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.45f, 0.65f, 0.95f, 0.80f));
    ImGui::Button("##plot-splitter", ImVec2(splitW, ImGui::GetContentRegionAvail().y));
    ImGui::PopStyleColor(3);
    if (ImGui::IsItemActive()) {
        plotSplitX_ += ImGui::GetIO().MouseDelta.x;
        plotSplitX_ = std::clamp(plotSplitX_, plotMinLeft, plotMaxLeft);
    }
    if (ImGui::IsItemHovered() || ImGui::IsItemActive()) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    }
    ImGui::SameLine(0.0f, 0.0f);
    ImGui::BeginChild("##plot-right", ImVec2(0, 0), ImGuiChildFlags_None,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(5.0f, 5.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(7.0f, 3.0f));
    if (iconButton("##plot-pause",
                   plotPaused_ ? ToolIcon::Play : ToolIcon::Pause,
                   plotPaused_ ? "Resume plotting" : "Pause plotting",
                   plotPaused_)) {
        plotPaused_ = !plotPaused_;
    }
    ImGui::SameLine();
    if (iconButton("##plot-clear", ToolIcon::Clear, "Clear plotted samples and restart X axis")) {
        for (WatchVar& w : watches_) {
            w.xs.clear();
            w.ys.clear();
        }
        // Capture-window markers carry plot-relative timestamps that are
        // anchored to the OLD plotTimeOrigin_. Once we move the origin
        // forward they'd land at random positions on the new axis -- the
        // "phantom markers" the operator saw. Drop them along with the
        // samples they brackets.
        capture_.windowsRel.clear();
        plotRulers_.clear();
        plotRulerDrawing_ = false;
        // Restart X axis from zero: from now on, frame timestamps are stored as
        // (frame.timestamp - plotTimeOrigin_), so xs and the X axis tick labels
        // reset relative to the moment of Clear.
        plotTimeOrigin_ = clock_.nowSeconds();
        plotXManual_ = false;
        if (!plotPaused_) plotFollowLive_ = true;
    }
    ImGui::SameLine();
    if (iconButton("##plot-autoscale", ToolIcon::Autoscale,
                   "Autoscale Y while following live data", plotAutoscale_)) {
        plotAutoscale_ = !plotAutoscale_;
    }
    if (plotAutoscale_) plotYManual_ = false;
    ImGui::SameLine();
    if (iconButton("##plot-reset-y", ToolIcon::ResetY,
                   "Reset Y axis. Wheel = Y zoom; Ctrl+wheel = X zoom/live window; Shift+wheel = X+Y zoom and exits Follow live.")) {
        plotYManual_ = false;
        plotAutoscale_ = true;
    }
    ImGui::SameLine();
    if (iconButton("##plot-crosshair", ToolIcon::Crosshair,
                   "Crosshair: vertical cursor with values under the mouse",
                   plotCrosshair_)) {
        plotCrosshair_ = !plotCrosshair_;
    }
    ImGui::SameLine();
    if (iconButton("##plot-ruler", ToolIcon::Ruler,
                   "Ruler: LMB draws measurement lines instead of panning. Wheel and axis zoom still work.",
                   plotRulerMode_)) {
        plotRulerMode_ = !plotRulerMode_;
        plotRulerDrawing_ = false;
        if (plotRulerMode_) {
            plotFollowLive_ = false;
        }
    }
    ImGui::SameLine();
    if (iconButton("##plot-clear-rulers", ToolIcon::Clear,
                   "Clear ruler lines", !plotRulers_.empty() || plotRulerDrawing_)) {
        plotRulers_.clear();
        plotRulerDrawing_ = false;
    }
    ImGui::SameLine();
    if (iconButton("##plot-follow", ToolIcon::Follow,
                   "Follow live: X axis slides with realtime. Manual X zoom/pan turns it off.",
                   plotFollowLive_, ImVec2(34.0f, 24.0f))) {
        plotFollowLive_ = !plotFollowLive_;
        if (plotFollowLive_) plotXManual_ = false;
    }
    ImGui::SameLine();
    if (iconButton("##plot-fit", ToolIcon::Fit, "Fit X+Y to all collected data once")) {
        pendingFitAll_ = true;
    }
    toolbarDivider();
    ImGui::SetNextItemWidth(88);
    ImGui::DragFloat("Win, s", &plotWindowSec_, 1.0f, 1.0f, 600.0f, "%.0f");
    {
        // Mouse-wheel-on-hover quick adjust. DragFloat only changes on
        // click+drag; nudging the value to e.g. 31 s by clicking is finicky.
        // With this, hovering the field and rolling the wheel steps the
        // window size by В±5% per notch (with shift = В±20%, ctrl = В±1%).
        const ImGuiIO& io2 = ImGui::GetIO();
        if (ImGui::IsItemHovered() && io2.MouseWheel != 0.0f) {
            const float pct = io2.KeyShift ? 0.20f
                            : io2.KeyCtrl  ? 0.01f
                                           : 0.05f;
            const float factor = io2.MouseWheel > 0.0f ? (1.0f + pct) : (1.0f / (1.0f + pct));
            plotWindowSec_ = std::clamp(plotWindowSec_ * factor, 1.0f, 600.0f);
            markScenarioDirty("plot window resized via field-wheel");
            ImGui::GetIO().MouseWheel = 0.0f;
        }
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Visible live window in seconds.\nWheel on field: В±5%% (Shift В±20%%, Ctrl В±1%%).");
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(88);
    ImGui::DragFloat("Hist, s", &plotHistorySec_, 10.0f, 30.0f, 86400.0f, "%.0f");
    {
        const ImGuiIO& io2 = ImGui::GetIO();
        if (ImGui::IsItemHovered() && io2.MouseWheel != 0.0f) {
            const float pct = io2.KeyShift ? 0.20f
                            : io2.KeyCtrl  ? 0.01f
                                           : 0.05f;
            const float factor = io2.MouseWheel > 0.0f ? (1.0f + pct) : (1.0f / (1.0f + pct));
            plotHistorySec_ = std::clamp(plotHistorySec_ * factor, 30.0f, 86400.0f);
            markScenarioDirty("plot history resized via field-wheel");
            ImGui::GetIO().MouseWheel = 0.0f;
        }
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("How many seconds of recent data to keep in memory (default 1 hour).\nWheel on field: В±5%% (Shift В±20%%, Ctrl В±1%%).");
    }

    toolbarDivider();
    if (iconButton("##plot-save", ToolIcon::Save, "Save plotted samples to CSV")) {
        std::string chosen = plotSnapshotPath_.data();
        if (nativeFileDialog(true, "CSV files\0*.csv\0All files\0*.*\0", chosen)) {
            setTextBuffer(plotSnapshotPath_, chosen);
        }
        std::ofstream f(plotSnapshotPath_.data(), std::ios::binary);
        if (!f) {
            plotSnapshotStatus_ = "save failed";
        } else {
            // Capture-window markers ride on top of the snapshot as `#`
            // comment lines. Loader recognises and strips them; standard
            // CSV viewers (Excel, pandas, etc) treat them as comments or
            // skipped non-numeric rows.
            for (const auto& cw : capture_.windowsRel) {
                f << "# capture_window: " << cw.startRel << ',' << cw.endRel << '\n';
            }
            f << "timestamp,series,value\n";
            size_t total = 0;
            for (const WatchVar& w : watches_) {
                if (!w.plot) continue;
                for (size_t i = 0; i < w.xs.size() && i < w.ys.size(); ++i) {
                    f << w.xs[i] << ',' << w.name << ',' << w.ys[i] << '\n';
                    ++total;
                }
            }
            plotSnapshotStatus_ = "saved " + std::to_string(total) + " samples";
            lastFileMsg_ = "saved: " + std::string(plotSnapshotPath_.data());
        }
    }
    ImGui::SameLine();
    if (iconButton("##plot-open", ToolIcon::Open, "Open plotted samples from CSV")) {
        std::string chosen = plotSnapshotPath_.data();
        if (nativeFileDialog(false, "CSV files\0*.csv\0All files\0*.*\0", chosen)) {
            setTextBuffer(plotSnapshotPath_, chosen);
        }
        std::ifstream f(plotSnapshotPath_.data(), std::ios::binary);
        if (!f) {
            plotSnapshotStatus_ = "open failed";
        } else {
            for (WatchVar& w : watches_) { w.xs.clear(); w.ys.clear(); }
            capture_.windowsRel.clear();
            replayMode_ = true;
            std::string line;
            // Header section: zero or more `# capture_window: start,end`
            // lines, then the literal "timestamp,series,value" header row.
            // The old format had no comment lines; that still loads cleanly.
            while (std::getline(f, line)) {
                if (line.empty()) continue;
                if (line[0] != '#') break;            // first non-comment is CSV header
                const std::string tag = "# capture_window:";
                if (line.compare(0, tag.size(), tag) == 0) {
                    std::string rest = line.substr(tag.size());
                    while (!rest.empty() && rest.front() == ' ') rest.erase(rest.begin());
                    const size_t comma = rest.find(',');
                    if (comma == std::string::npos) continue;
                    try {
                        const double s = std::stod(rest.substr(0, comma));
                        const double e = std::stod(rest.substr(comma + 1));
                        if (e >= s) capture_.windowsRel.push_back({s, e});
                    } catch (...) { /* skip malformed marker */ }
                }
            }
            // `line` now holds the CSV header row. Discard and proceed.
            size_t total = 0;
            while (std::getline(f, line)) {
                std::stringstream ss(line);
                std::string ts, name, val;
                if (!std::getline(ss, ts, ',')) continue;
                if (!std::getline(ss, name, ',')) continue;
                if (!std::getline(ss, val, ',')) continue;
                try {
                    const double t = std::stod(ts);
                    const double v = std::stod(val);
                    bool found = false;
                    for (WatchVar& w : watches_) {
                        if (w.name == name) {
                            w.xs.push_back(t);
                            w.ys.push_back(v);
                            w.plot = true;
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        WatchVar nw;
                        nw.name = name;
                        nw.plot = true;
                        nw.enabled = false;
                        nw.xs.push_back(t);
                        nw.ys.push_back(v);
                        watches_.push_back(std::move(nw));
                    }
                    ++total;
                } catch (...) { /* skip */ }
            }
            plotSnapshotStatus_ = "loaded " + std::to_string(total) + " samples (replay)";
            lastFileMsg_ = "loaded: " + std::string(plotSnapshotPath_.data());
            plotPaused_ = true;
        }
    }
    // -- Triggered Capture controls inline in the same toolbar row --
    // Style decision: replaced the old multi-row panel (period combo,
    // pre/post percent, buffer KB, trigger mode, threshold, big green
    // Trigger button) with one icon button + one duration field. Period
    // and buffer are derived from the duration via computeCaptureBounds()
    // at click time; trigger mode is fixed at "immediate" so the snap
    // happens the moment the user clicks. Capture set = whatever is on
    // the plot, narrowed to the first node and capped at kCapMaxSlots.
    toolbarDivider();
    {
        // Predict capture set (same priority order as startCaptureSession:
        // explicitly cap-marked first, then fall back to plotted). This
        // determines the slot count that bounds the duration field.
        uint8_t previewNode = 0;
        uint8_t previewSlots = 0;
        auto scan = [&](bool wantCap) {
            if (previewSlots > 0) return;
            for (const WatchVar& w : watches_) {
                const bool include = wantCap ? w.capture : w.plot;
                if (!include) continue;
                if (previewNode == 0) previewNode = w.nodeId;
                if (w.nodeId != previewNode) continue;
                if (previewSlots >= kCapMaxSlots) break;
                ++previewSlots;
            }
        };
        scan(true);
        scan(false);
        if (previewNode == 0) previewNode = capture_.nodeId;

        const CaptureBounds bounds = computeCaptureBounds(
            previewNode,
            previewSlots,
            capture_.durationMs);
        // Pin durationMs into valid range whenever slot count or node changes
        // (e.g. user toggled a plot series, switched targets).
        if (capture_.durationMs < bounds.minMs || capture_.durationMs > bounds.maxMs) {
            capture_.durationMs = std::clamp(capture_.durationMs, bounds.minMs, bounds.maxMs);
        }

        ImGui::SetNextItemWidth(86);
        int dur = capture_.durationMs;
        if (ImGui::DragInt("Cap, ms", &dur, 1.0f, bounds.minMs, bounds.maxMs)) {
            capture_.durationMs = std::clamp(dur, bounds.minMs, bounds.maxMs);
            markScenarioDirty("capture duration");
        }
        {
            const ImGuiIO& io2 = ImGui::GetIO();
            if (ImGui::IsItemHovered() && io2.MouseWheel != 0.0f) {
                const float pct = io2.KeyShift ? 0.20f
                                : io2.KeyCtrl  ? 0.01f
                                               : 0.05f;
                const float factor = io2.MouseWheel > 0.0f
                                         ? (1.0f + pct)
                                         : (1.0f / (1.0f + pct));
                int next = static_cast<int>(std::round(
                    static_cast<double>(capture_.durationMs) * factor));
                next = std::clamp(next, bounds.minMs, bounds.maxMs);
                if (next == capture_.durationMs) {
                    // Smallest possible step on tiny values.
                    next += io2.MouseWheel > 0.0f ? +1 : -1;
                    next = std::clamp(next, bounds.minMs, bounds.maxMs);
                }
                capture_.durationMs = next;
                markScenarioDirty("capture duration via field-wheel");
                ImGui::GetIO().MouseWheel = 0.0f;
            }
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(
                "Triggered Capture window in ms.\n"
                "Range %d..%d ms (auto: %u-byte ring / %u variable%s = %u samples).\n"
                "Period at click: ~%u us. Wheel: 5%% (Shift 20%%, Ctrl 1%%).",
                bounds.minMs, bounds.maxMs,
                static_cast<unsigned>(bounds.bufferBytes),
                static_cast<unsigned>(previewSlots == 0 ? 1 : previewSlots),
                previewSlots == 0 ? "" : "s",
                static_cast<unsigned>(bounds.bufSamples),
                static_cast<unsigned>(bounds.periodUs));
        }

        ImGui::SameLine();
        const bool busy = (capture_.phase != CaptureSessionPhase::Idle &&
                           capture_.phase != CaptureSessionPhase::CooldownDone);
        const double now = clock_.nowSeconds();
        const bool inCooldown = (capture_.phase == CaptureSessionPhase::CooldownDone &&
                                 now < capture_.cooldownUntil);
        const bool canTrigger = isConnected() && previewSlots > 0 && !busy && !inCooldown;

        char triggerTip[320];
        if (!isConnected()) {
            std::snprintf(triggerTip, sizeof(triggerTip),
                          "Triggered Capture: connect to a CAN transport first.");
        } else if (previewSlots == 0) {
            std::snprintf(triggerTip, sizeof(triggerTip),
                          "Triggered Capture: plot at least one variable, then click.");
        } else if (busy) {
            std::snprintf(triggerTip, sizeof(triggerTip),
                          "Triggered Capture: busy (%s).", capture_.status.c_str());
        } else if (inCooldown) {
            const double left = capture_.cooldownUntil - now;
            std::snprintf(triggerTip, sizeof(triggerTip),
                          "Triggered Capture: cooldown %.1fs (%s).",
                          left, capture_.status.c_str());
        } else {
            std::snprintf(triggerTip, sizeof(triggerTip),
                          "Trigger: snap %d ms of %u variable%s on this plot (period ~%u us, immediate).",
                          capture_.durationMs,
                          static_cast<unsigned>(previewSlots),
                          previewSlots == 1 ? "" : "s",
                          static_cast<unsigned>(bounds.periodUs));
        }

        if (iconButton("##plot-trigger", ToolIcon::Trigger, triggerTip,
                       busy || inCooldown)) {
            if (canTrigger) startCaptureSession();
        }

        // Compact status text: greyed when idle, yellow while busy, white
        // during cooldown. Same row, no extra panel below the toolbar.
        ImGui::SameLine();
        if (busy) {
            ImGui::TextColored(ImVec4(0.95f, 0.85f, 0.30f, 1.0f),
                               "%s", capture_.status.c_str());
        } else if (inCooldown) {
            const double left = capture_.cooldownUntil - now;
            ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.85f, 1.0f),
                               "cooldown %.1fs", left);
        } else if (!capture_.lastError.empty()) {
            ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.40f, 1.0f),
                               "%s", capture_.lastError.c_str());
        }
    }
    ImGui::PopStyleVar(2);
    ImGui::Spacing();

    struct PlottedRef {
        const WatchVar* w;
        ImVec4 color;
    };
    std::vector<PlottedRef> plotted;
    plotted.reserve(6);

    {
        // Mouse semantics -- kept deliberately minimal:
        //   LMB drag -> pan X+Y (manualPlotMove below disengages Live first).
        //   RMB drag -> box-select; Y axis is locked during the drag so the
        //               resulting zoom is X-only.
        //   wheel    -> handled by our own code (window resize in Live, Y/X
        //               zoom otherwise). We don't use ImPlot's built-in
        //               scroll zoom.
        //
        // To kill ImPlot's stock context menu, mouse-coord overlay, and
        // double-click Fit, we use ImPlotFlags_NoMenus + NoMouseText on
        // BeginPlot below AND assign Fit/SelectCancel to extra mouse
        // buttons (X1/X2). Don't set these to ImGuiMouseButton_COUNT (5):
        // ImPlot indexes IO.MouseDown[InputMap.X] directly, and MouseDown
        // is bool[5] -- index 5 is out of bounds and reads garbage,
        // spuriously firing the popup-on-hover bug.
        constexpr ImGuiMouseButton kSafeUnused1 = static_cast<ImGuiMouseButton>(3); // X1
        constexpr ImGuiMouseButton kSafeUnused2 = static_cast<ImGuiMouseButton>(4); // X2
        ImPlotInputMap& im = ImPlot::GetInputMap();
        im.Pan          = plotRulerMode_ ? kSafeUnused1 : ImGuiMouseButton_Left;
        im.PanMod       = ImGuiMod_None;
        im.Select       = ImGuiMouseButton_Right;
        im.SelectMod    = ImGuiMod_None;
        im.SelectCancel = kSafeUnused1;
        im.Fit          = kSafeUnused2;
        im.Menu         = kSafeUnused1;  // even with NoMenus flag, keep this in [0..4]
        // Disable ImPlot's stock wheel zoom. We apply the operator-requested
        // wheel mapping below after plotting: wheel=Y, Ctrl=X, Shift=X+Y.
        im.ZoomMod      = ImGuiMod_Ctrl | ImGuiMod_Shift | ImGuiMod_Alt | ImGuiMod_Super;
        im.ZoomRate     = 0.0f;
    }

    // NoMenus    = no axis-context popup on right-click in axis area.
    // NoMouseText = no floating "(x, y)" coordinates display in plot corner
    //              (operator finds it noisy on hover, especially when
    //              disconnected).
    // BoxSelect stays enabled -- that's our RMB-drag X-zoom.
    constexpr ImPlotFlags kPlotFlags = ImPlotFlags_NoMenus | ImPlotFlags_NoMouseText;
    if (ImPlot::BeginPlot("Signals", ImVec2(-1, -1), kPlotFlags)) {
        // Freeze the plot's "now" at the moment we lose the transport so
        // existing curves stay on screen instead of scrolling off into empty
        // future. Reconnect resumes wall-clock-based now and the gap from
        // freeze->reconnect is visible as a blank interval -- that's the
        // signal to the operator that data was lost during the disconnect.
        const double rawNow = clock_.nowSeconds() - plotTimeOrigin_;
        if (!isConnected()) {
            if (!plotFrozen_) { plotFrozen_ = true; plotFrozenAt_ = rawNow; }
        } else {
            plotFrozen_ = false;
        }
        const double now = plotFrozen_ ? plotFrozenAt_ : rawNow;
        const ImGuiIO& io = ImGui::GetIO();
        // `useY_AutoFit` no longer depends on plotYManual_ -- that flag is now
        // a one-shot "push my plotYMin_/Max_ into ImPlot once" trigger, not a
        // sticky state. Sticky "don't autofit anymore" lives in plotAutoscale_,
        // which the wheel-zoom-Y handler clears.
        const bool useY_AutoFit = plotAutoscale_ && plotFollowLive_;
        // Lock Y during RMB drag so the resulting box-zoom only changes X.
        // ImPlot's default selection rect zooms BOTH axes; locking Y makes
        // the post-release zoom a no-op on Y while X gets the new range.
        const bool rmbHeld = ImGui::IsMouseDown(ImGuiMouseButton_Right);
        ImPlotAxisFlags yFlags = useY_AutoFit ? ImPlotAxisFlags_AutoFit : 0;
        if (rmbHeld) yFlags |= ImPlotAxisFlags_Lock;
        ImPlot::SetupAxes("time, hh:mm:ss", "value", 0, yFlags);
        ImPlot::SetupAxisFormat(ImAxis_X1,
            +[](double v, char* buf, int n, void*) -> int {
                const bool neg = v < 0;
                double t = neg ? -v : v;
                const int hours = static_cast<int>(t / 3600.0);
                const int mins = static_cast<int>(t / 60.0) - hours * 60;
                const double secs = t - hours * 3600.0 - mins * 60.0;
                return std::snprintf(buf, static_cast<size_t>(n),
                                     "%s%02d:%02d:%06.3f",
                                     neg ? "-" : "", hours, mins, secs);
            },
            nullptr);
        // ImPlot's built-in legend click toggles ImPlotItem::Show internally;
        // we mirror it into legendHidden_ each frame so crosshair tooltip stays in sync.
        ImPlot::SetupLegend(ImPlotLocation_NorthWest, 0);
        // X-axis state machine:
        //   Live ON  -> override every frame to slide [now-window, now].
        //   Live OFF + plotXManual_ (one-shot) -> push plotXMin_/Max_ into
        //     ImPlot ONCE; then clear the flag so subsequent LMB pan / RMB
        //     box-zoom can mutate the axis freely without us clobbering it.
        //   Otherwise -> seed once on plot creation; ImPlot owns it after.
        if (plotFollowLive_ && !plotPaused_) {
            ImPlot::SetupAxisLimits(ImAxis_X1, now - plotWindowSec_, now, ImGuiCond_Always);
            plotXManual_ = false;
        } else if (plotXManual_) {
            ImPlot::SetupAxisLimits(ImAxis_X1, plotXMin_, plotXMax_, ImGuiCond_Always);
            plotXManual_ = false;
        } else {
            ImPlot::SetupAxisLimits(ImAxis_X1, now - plotWindowSec_, now, ImGuiCond_Once);
        }
        // Y-axis: same one-shot pattern as X. Sticky "don't autofit" already
        // captured by plotAutoscale_, so we don't need plotYManual_ to be
        // sticky; clearing it after the override lets LMB pan touch Y too.
        if (plotYManual_) {
            ImPlot::SetupAxisLimits(ImAxis_Y1, plotYMin_, plotYMax_, ImGuiCond_Always);
            plotYManual_ = false;
        }
        if (pendingFitAll_) {
            double xMin = std::numeric_limits<double>::max();
            double xMax = -std::numeric_limits<double>::max();
            double yMin = std::numeric_limits<double>::max();
            double yMax = -std::numeric_limits<double>::max();
            bool any = false;
            for (const WatchVar& w : watches_) {
                if (!w.plot) continue;
                // Variables that were hidden via the legend toggle are not
                // visually on the plot -- they shouldn't pull the fit range
                // outwards. Was a bug: invisible curves still expanded
                // X/Y fits by their time/value extents.
                if (legendHidden_.count(w.name) > 0) continue;
                const size_t pointCount = std::min(w.xs.size(), w.ys.size());
                for (size_t i = 0; i < pointCount; ++i) {
                    xMin = std::min(xMin, w.xs[i]);
                    xMax = std::max(xMax, w.xs[i]);
                    const double y = w.ys[i] * w.plotScale;
                    yMin = std::min(yMin, y);
                    yMax = std::max(yMax, y);
                    any = true;
                }
            }
            if (any) {
                const double xPad = std::max(0.5, (xMax - xMin) * 0.02);
                const double yPad = std::max(1e-3, (yMax - yMin) * 0.05);
                ImPlot::SetupAxisLimits(ImAxis_X1, xMin - xPad, xMax + xPad, ImPlotCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, yMin - yPad, yMax + yPad, ImPlotCond_Always);
                plotXMin_ = xMin - xPad;
                plotXMax_ = xMax + xPad;
                plotYMin_ = yMin - yPad;
                plotYMax_ = yMax + yPad;
                // Pin the fitted range so a Live override doesn't slide the
                // X axis right after Fit completes.
                plotFollowLive_ = false;
                plotXManual_ = true;
                plotYManual_ = true;
                plotAutoscale_ = false;
            }
            pendingFitAll_ = false;
        }

        int colorIdx = 0;
        std::vector<size_t> renderable;
        renderable.reserve(6);
        for (size_t wi = 0; wi < watches_.size(); ++wi) {
            const WatchVar& w = watches_[wi];
            if (!w.plot) continue;
            if (renderable.size() >= 6) break;
            renderable.push_back(wi);
        }
        size_t legendRemoveIndex = std::numeric_limits<size_t>::max();
        for (size_t wi : renderable) {
            WatchVar& w = watches_[wi];
            const std::string plotLabel = plotSeriesLabel(w.name, w.plotScale);
            const ImVec4 col = ImPlot::GetColormapColor(colorIdx++);
            ImPlot::SetNextLineStyle(col);
            const size_t pointCount = std::min(w.xs.size(), w.ys.size());
            std::vector<double> scaledYs;
            const double* ys = w.ys.data();
            if (pointCount > 0 && !samePlotScale(w.plotScale, 1.0)) {
                scaledYs.resize(pointCount);
                for (size_t i = 0; i < pointCount; ++i) {
                    scaledYs[i] = w.ys[i] * w.plotScale;
                }
                ys = scaledYs.data();
            }
            if (pointCount == 0) {
                ImPlot::PlotDummy(plotLabel.c_str());
            } else if (pointCount == 1) {
                ImPlot::PlotScatter(plotLabel.c_str(), w.xs.data(), ys, 1);
            } else {
                ImPlot::PlotLine(plotLabel.c_str(), w.xs.data(), ys,
                                 static_cast<int>(pointCount));
            }
            // Mirror ImPlot's internal Show into our legendHidden_ so the crosshair
            // tooltip skips entries that the user clicked off in the legend.
            ImPlotPlot* plotCtx = ImPlot::GetCurrentPlot();
            ImPlotItem* item = nullptr;
            if (plotCtx) {
                const ImGuiID id = plotCtx->Items.GetItemID(plotLabel.c_str());
                item = plotCtx->Items.GetItem(id);
                const bool hidden = (item && !item->Show);
                if (hidden) legendHidden_.insert(w.name);
                else        legendHidden_.erase(w.name);
            }
            if (ImPlot::BeginLegendPopup(plotLabel.c_str(), ImGuiMouseButton_Right)) {
                const bool visible = item == nullptr || item->Show;
                if (ImGui::MenuItem(visible ? "Hide" : "Show", nullptr, false, item != nullptr)) {
                    item->Show = !item->Show;
                    if (item->Show) legendHidden_.erase(w.name);
                    else            legendHidden_.insert(w.name);
                }
                if (ImGui::MenuItem("Remove from plot")) {
                    legendRemoveIndex = wi;
                }
                // Toggle Triggered Capture inclusion. The flag is persisted
                // per-watch in the scenario; the actual capture happens
                // when the user clicks Trigger in the capture panel below.
                if (ImGui::MenuItem(w.capture ? "Unmark from capture set"
                                              : "Mark for capture",
                                    nullptr, w.capture)) {
                    w.capture = !w.capture;
                    markScenarioDirty("watch capture toggled");
                }
                if (ImGui::BeginMenu("Set scale")) {
                    for (const PlotScaleOption& option : kPlotScaleOptions) {
                        const bool selected = samePlotScale(w.plotScale, option.value);
                        if (ImGui::MenuItem(option.label, nullptr, selected)) {
                            w.plotScale = option.value;
                            markScenarioDirty("watch scale changed");
                        }
                    }
                    ImGui::EndMenu();
                }
                ImPlot::EndLegendPopup();
            }
            if (!w.xs.empty() && legendHidden_.count(w.name) == 0) {
                plotted.push_back({&w, col});
            }
        }

        // Capture-window markers live on the X axis only. Keep them label-free:
        // small ticks mark start/end without drawing through the signal area.
        if (!capture_.windowsRel.empty()) {
            const double earliestKept = (clock_.nowSeconds() - plotTimeOrigin_) -
                                        static_cast<double>(plotHistorySec_);
            const ImPlotRect lim = ImPlot::GetPlotLimits();
            const ImVec2 plotPos = ImPlot::GetPlotPos();
            const ImVec2 plotSize = ImPlot::GetPlotSize();
            const float axisY = plotPos.y + plotSize.y;
            ImDrawList* dl = ImPlot::GetPlotDrawList();
            const ImU32 startCol = IM_COL32(75, 235, 100, 210);
            const ImU32 endCol   = IM_COL32(240, 105,  70, 210);
            for (const auto& w : capture_.windowsRel) {
                if (w.endRel < earliestKept) continue;
                auto marker = [&](double x, ImU32 col) {
                    if (x < lim.X.Min || x > lim.X.Max) return;
                    const float px = ImPlot::PlotToPixels(ImPlotPoint(x, lim.Y.Min)).x;
                    dl->AddLine(ImVec2(px, axisY + 1.0f),
                                ImVec2(px, axisY + 9.0f),
                                col, 1.0f);
                    dl->AddTriangleFilled(ImVec2(px, axisY + 1.0f),
                                          ImVec2(px - 3.0f, axisY + 5.0f),
                                          ImVec2(px + 3.0f, axisY + 5.0f),
                                          col);
                };
                marker(w.startRel, startCol);
                marker(w.endRel, endCol);
            }
        }

        auto fmt_dt = [](double dt, char* buf, size_t n) {
            const char* sign = dt < 0.0 ? "-" : "";
            double t = std::fabs(dt);
            if (t >= 1.0) {
                std::snprintf(buf, n, "%s%.3fs", sign, t);
            } else if (t >= 0.001) {
                std::snprintf(buf, n, "%s%.3fms", sign, t * 1000.0);
            } else {
                std::snprintf(buf, n, "%s%.0fus", sign, t * 1000000.0);
            }
        };
        auto draw_ruler = [&](const PlotRuler& r, bool draft) {
            const ImPlotRect lim = ImPlot::GetPlotLimits();
            const double xMin = std::min(r.x1, r.x2);
            const double xMax = std::max(r.x1, r.x2);
            const double yMin = std::min(r.y1, r.y2);
            const double yMax = std::max(r.y1, r.y2);
            if (xMax < lim.X.Min || xMin > lim.X.Max ||
                yMax < lim.Y.Min || yMin > lim.Y.Max) {
                return;
            }

            ImDrawList* dl = ImPlot::GetPlotDrawList();
            const ImVec2 a = ImPlot::PlotToPixels(ImPlotPoint(r.x1, r.y1));
            const ImVec2 b = ImPlot::PlotToPixels(ImPlotPoint(r.x2, r.y2));
            const ImU32 col = draft ? IM_COL32(255, 210, 75, 210)
                                    : IM_COL32(255, 210, 75, 235);
            const ImU32 faint = draft ? IM_COL32(255, 210, 75, 75)
                                      : IM_COL32(255, 210, 75, 105);

            dl->AddLine(a, b, col, draft ? 1.5f : 2.0f);
            dl->AddCircleFilled(a, 3.0f, col);
            dl->AddCircleFilled(b, 3.0f, col);
            dl->AddLine(a, ImVec2(b.x, a.y), faint, 1.0f);
            dl->AddLine(b, ImVec2(b.x, a.y), faint, 1.0f);

            const double dt = r.x2 - r.x1;
            const double dy = r.y2 - r.y1;
            char dtBuf[32];
            fmt_dt(dt, dtBuf, sizeof(dtBuf));
            char label[128];
            if (std::fabs(dt) > 1e-12) {
                std::snprintf(label, sizeof(label), "dt %s  dY %.6g  dY/dt %.6g/s",
                              dtBuf, dy, dy / dt);
            } else {
                std::snprintf(label, sizeof(label), "dt %s  dY %.6g", dtBuf, dy);
            }

            const ImVec2 mid((a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f);
            const ImVec2 textSz = ImGui::CalcTextSize(label);
            ImVec2 p(mid.x + 8.0f, mid.y - textSz.y - 6.0f);
            const ImVec2 plotPos = ImPlot::GetPlotPos();
            const ImVec2 plotSize = ImPlot::GetPlotSize();
            p.x = std::clamp(p.x, plotPos.x + 2.0f, plotPos.x + plotSize.x - textSz.x - 6.0f);
            p.y = std::clamp(p.y, plotPos.y + 2.0f, plotPos.y + plotSize.y - textSz.y - 6.0f);
            dl->AddRectFilled(ImVec2(p.x - 4.0f, p.y - 3.0f),
                              ImVec2(p.x + textSz.x + 4.0f, p.y + textSz.y + 3.0f),
                              IM_COL32(20, 22, 26, 205), 3.0f);
            dl->AddRect(ImVec2(p.x - 4.0f, p.y - 3.0f),
                        ImVec2(p.x + textSz.x + 4.0f, p.y + textSz.y + 3.0f),
                        IM_COL32(255, 210, 75, 150), 3.0f);
            dl->AddText(p, IM_COL32(255, 235, 150, 255), label);
        };

        ImPlot::PushPlotClipRect();
        for (const PlotRuler& r : plotRulers_) {
            draw_ruler(r, false);
        }
        if (plotRulerDrawing_) {
            draw_ruler(plotRulerDraft_, true);
        }
        ImPlot::PopPlotClipRect();

        if (plotRulerMode_ && ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
            plotRulerDrawing_ = false;
            ImGui::ClearActiveID();
            ImGui::SetWindowFocus(static_cast<const char*>(nullptr));
            if (ImGuiContext* g = ImGui::GetCurrentContext()) {
                g->NavId = 0;
                g->NavWindow = nullptr;
            }
        }

        if (plotRulerMode_ && ImPlot::IsPlotHovered()) {
            const ImPlotPoint mp = ImPlot::GetPlotMousePos();
            if (!plotRulerDrawing_ && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                plotRulerDraft_ = PlotRuler{mp.x, mp.y, mp.x, mp.y};
                plotRulerDrawing_ = true;
                plotFollowLive_ = false;
                plotXManual_ = true;
                const ImPlotRect lim = ImPlot::GetPlotLimits();
                plotXMin_ = lim.X.Min;
                plotXMax_ = lim.X.Max;
            }
            if (plotRulerDrawing_ && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                plotRulerDraft_.x2 = mp.x;
                plotRulerDraft_.y2 = mp.y;
            }
            if (plotRulerDrawing_ && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                plotRulerDraft_.x2 = mp.x;
                plotRulerDraft_.y2 = mp.y;
                const double dx = plotRulerDraft_.x2 - plotRulerDraft_.x1;
                const double dy = plotRulerDraft_.y2 - plotRulerDraft_.y1;
                if (std::hypot(dx, dy) > 1e-9) {
                    plotRulers_.push_back(plotRulerDraft_);
                }
                plotRulerDrawing_ = false;
            }
        } else if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            plotRulerDrawing_ = false;
        }

        // LMB drag = free pan in any direction; RMB drag = box-zoom (X-only,
        // see Y Lock above). Either gesture exits Live. Double-click and
        // middle-button are intentionally NOT bound -- operator asked for the
        // minimal mouse vocabulary.
        const bool manualPlotMove =
            ImPlot::IsPlotHovered() &&
            !plotRulerMode_ &&
            (ImGui::IsMouseDragging(ImGuiMouseButton_Left, 1.0f) ||
             ImGui::IsMouseDragging(ImGuiMouseButton_Right, 1.0f));
        if (manualPlotMove && plotFollowLive_) {
            // Snapshot the current Live X range as our manual range so the
            // first frame after Live disengages doesn't jump to whatever
            // ImPlot's last cached limits were.
            const ImPlotRect lim = ImPlot::GetPlotLimits();
            plotXMin_ = lim.X.Min;
            plotXMax_ = lim.X.Max;
            plotXManual_ = true;
            plotFollowLive_ = false;
        }

        // Hover an axis strip + wheel = zoom ONLY that axis, no Ctrl/Shift.
        // Detect the strips by geometry: left of the plot frame = Y axis,
        // below it = X axis. This is mutually exclusive with the plot body, so
        // the in-plot Ctrl/Shift handler below is never disturbed. We consume
        // the wheel only when actually over a strip.
        if (io.MouseWheel != 0.0f) {
            const ImVec2 pPos  = ImPlot::GetPlotPos();   // top-left of plot area
            const ImVec2 pSize = ImPlot::GetPlotSize();  // plot area size (px)
            const ImVec2 m     = io.MousePos;
            const bool inYBand = (m.y >= pPos.y && m.y <= pPos.y + pSize.y);
            const bool inXBand = (m.x >= pPos.x && m.x <= pPos.x + pSize.x);
            const bool overYAxis = inYBand && (m.x < pPos.x) && (m.x >= pPos.x - 70.0f);
            const bool overXAxis = inXBand && (m.y > pPos.y + pSize.y) &&
                                   (m.y <= pPos.y + pSize.y + 50.0f);
            if (overXAxis || overYAxis) {
                const ImPlotRect  lim  = ImPlot::GetPlotLimits();
                const ImPlotPoint mp   = ImPlot::GetPlotMousePos();
                const double      zoom = io.MouseWheel > 0.0f ? (1.0 / 1.15) : 1.15;
                if (overXAxis) {
                    if (plotFollowLive_) {
                        // Live: resize the visible time window, right edge pinned.
                        const float scaled = plotWindowSec_ * static_cast<float>(zoom);
                        plotWindowSec_ = std::clamp(scaled, 1.0f, 600.0f);
                        markScenarioDirty("plot window resized via axis-wheel");
                    } else {
                        plotXManual_ = true;
                        plotXMin_ = mp.x + (lim.X.Min - mp.x) * zoom;
                        plotXMax_ = mp.x + (lim.X.Max - mp.x) * zoom;
                    }
                } else {  // over Y axis strip
                    plotYMin_ = mp.y + (lim.Y.Min - mp.y) * zoom;
                    plotYMax_ = mp.y + (lim.Y.Max - mp.y) * zoom;
                    plotYManual_ = true;
                    plotAutoscale_ = false;
                }
                ImGui::GetIO().MouseWheel = 0.0f;
                ImGui::GetIO().MouseWheelH = 0.0f;
            }
        }

        if (ImPlot::IsPlotHovered() && io.MouseWheel != 0.0f) {
            const ImPlotRect lim = ImPlot::GetPlotLimits();
            const ImPlotPoint mp = ImPlot::GetPlotMousePos();
            const double zoom = io.MouseWheel > 0.0f ? (1.0 / 1.15) : 1.15;
            // Wheel modifier mapping (operator-specified):
            //   no mod        -> Y axis only (preserves Live)
            //   Ctrl          -> X axis only (preserves Live; in Live mode
            //                    this resizes plotWindowSec_ instead)
            //   Shift         -> both axes; EXITS Live so the new manual X
            //                    range stays on screen
            // Ctrl+Shift is treated as Shift.
            const bool wantBoth = io.KeyShift;
            const bool wantX    = io.KeyCtrl && !io.KeyShift;
            const bool wantY    = !io.KeyCtrl && !io.KeyShift;

            if (wantY) {
                plotYMin_ = mp.y + (lim.Y.Min - mp.y) * zoom;
                plotYMax_ = mp.y + (lim.Y.Max - mp.y) * zoom;
                plotYManual_ = true;
                plotAutoscale_ = false;
            } else if (wantX) {
                if (plotFollowLive_) {
                    // In Live, Ctrl+wheel resizes the visible time window
                    // (right edge stays pinned to "now"). Live stays on.
                    const float scaled = plotWindowSec_ * static_cast<float>(zoom);
                    plotWindowSec_ = std::clamp(scaled, 1.0f, 600.0f);
                    markScenarioDirty("plot window resized via wheel");
                } else {
                    plotXManual_ = true;
                    plotXMin_ = mp.x + (lim.X.Min - mp.x) * zoom;
                    plotXMax_ = mp.x + (lim.X.Max - mp.x) * zoom;
                }
            } else if (wantBoth) {
                // Both axes around the cursor. Exits Live -- the manual X
                // range we just set has to survive the next frame.
                plotFollowLive_ = false;
                plotXManual_ = true;
                plotXMin_ = mp.x + (lim.X.Min - mp.x) * zoom;
                plotXMax_ = mp.x + (lim.X.Max - mp.x) * zoom;
                plotYMin_ = mp.y + (lim.Y.Min - mp.y) * zoom;
                plotYMax_ = mp.y + (lim.Y.Max - mp.y) * zoom;
                plotYManual_ = true;
                plotAutoscale_ = false;
            }
            ImGui::GetIO().MouseWheel = 0.0f;
            ImGui::GetIO().MouseWheelH = 0.0f; // suppress horizontal scroll on Shift+wheel
        }

        if (plotCrosshair_ && ImPlot::IsPlotHovered() && !plotted.empty()) {
            const ImPlotPoint mp = ImPlot::GetPlotMousePos();
            const double cursorX = mp.x;

            ImDrawList* dl = ImPlot::GetPlotDrawList();
            const ImPlotRect lim = ImPlot::GetPlotLimits();
            const ImVec2 top = ImPlot::PlotToPixels(ImPlotPoint(cursorX, lim.Y.Max));
            const ImVec2 bot = ImPlot::PlotToPixels(ImPlotPoint(cursorX, lim.Y.Min));

            ImPlot::PushPlotClipRect();
            dl->AddLine(top, bot, IM_COL32(220, 220, 230, 140), 1.0f);

            ImGui::BeginTooltip();
            {
                const bool neg = cursorX < 0;
                double t = neg ? -cursorX : cursorX;
                const int hh = static_cast<int>(t / 3600.0);
                const int mm = static_cast<int>(t / 60.0) - hh * 60;
                const double ss = t - hh * 3600.0 - mm * 60.0;
                ImGui::Text("t = %s%02d:%02d:%06.3f",
                            neg ? "-" : "", hh, mm, ss);
            }
            ImGui::Separator();
            for (const PlottedRef& p : plotted) {
                const WatchVar& w = *p.w;
                const size_t pointCount = std::min(w.xs.size(), w.ys.size());
                if (pointCount == 0) continue;
                auto it = std::lower_bound(
                    w.xs.begin(),
                    w.xs.begin() + static_cast<std::ptrdiff_t>(pointCount),
                    cursorX);
                size_t idx = 0;
                if (it == w.xs.begin() + static_cast<std::ptrdiff_t>(pointCount)) {
                    idx = pointCount - 1;
                } else if (it == w.xs.begin()) {
                    idx = 0;
                } else {
                    const size_t k = static_cast<size_t>(it - w.xs.begin());
                    const double dHi = std::abs(w.xs[k] - cursorX);
                    const double dLo = std::abs(w.xs[k - 1] - cursorX);
                    idx = (dLo < dHi) ? (k - 1) : k;
                }
                const double sx = w.xs[idx];
                const double sy = w.ys[idx] * w.plotScale;
                ImGui::ColorButton("##sw", p.color,
                                   ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoBorder,
                                   ImVec2(10, 10));
                ImGui::SameLine();
                const std::string label = plotSeriesLabel(w.name, w.plotScale);
                ImGui::Text("%-24s %.6g", label.c_str(), sy);

                const ImVec2 px = ImPlot::PlotToPixels(ImPlotPoint(sx, sy));
                const ImU32 c32 = ImGui::ColorConvertFloat4ToU32(p.color);
                dl->AddCircleFilled(px, 4.0f, c32);
                dl->AddCircle(px, 5.5f, IM_COL32(255, 255, 255, 220), 0, 1.2f);
            }
            ImGui::EndTooltip();
            ImPlot::PopPlotClipRect();
        }

        ImPlot::EndPlot();
        if (legendRemoveIndex < watches_.size()) {
            legendHidden_.erase(watches_[legendRemoveIndex].name);
            watches_.erase(watches_.begin() + static_cast<std::ptrdiff_t>(legendRemoveIndex));
            pending_.clear();
            pendingSentAt_.clear();
            nextSeq_ = 1;
            pollCursor_ = 0;
            markScenarioDirty("watch removed");
        }
    }

    ImGui::EndChild();
}

void MainUi::drawLogging()
{
    ImGui::InputText("CAN CSV path", canLogPath_.data(), canLogPath_.size());
    if (!canLogger_.active()) {
        if (ImGui::Button("Start CAN logger")) canLogger_.start(canLogPath_.data());
    } else {
        if (ImGui::Button("Stop CAN logger")) canLogger_.stop();
    }
    ImGui::Text("CAN logger: %s", canLogger_.active() ? "running" : "stopped");

    ImGui::Separator();
    ImGui::InputText("Signal CSV path", signalLogPath_.data(), signalLogPath_.size());
    if (!signalLogger_.active()) {
        if (ImGui::Button("Start Signal logger")) signalLogger_.start(signalLogPath_.data());
    } else {
        if (ImGui::Button("Stop Signal logger")) signalLogger_.stop();
    }
    ImGui::Text("Signal logger: %s", signalLogger_.active() ? "running" : "stopped");
}

// =====================================================================
// Diagnostic-mode helpers + Motor / RK tabs
// =====================================================================

bool MainUi::sendMainCmd(uint16_t cmd, const uint8_t payload[8])
{
    if (!activeTransport_ || !activeTransport_->isOpen()) return false;
    CanFrame f{};
    // mod=APP(1)<<28 | cmd<<16 | dst=MAIN(0x01)<<8 | src=PC(0x10)
    f.id        = (uint32_t(1) << 28) | (uint32_t(cmd) << 16) |
                  (uint32_t(kCanAddrMain) << 8) | uint32_t(kCanAddrPc);
    f.extended  = true;
    f.fd        = false;
    f.brs       = false;
    f.rx        = false;
    f.data.assign(payload, payload + 8);
    f.dlc       = 8;
    return sendFrameThreadSafe(f);
}

void MainUi::diagModeRequest(bool enter)
{
    uint8_t p[8] = {0xFF,0xFF,0xFF,0xFF, 'D','D','V','2'};
    p[0] = enter ? 1u : 0u;
    sendMainCmd(0x010 /* MAIN_CMD_DIAG_MODE_SET */, p);
    diagModeOn_ = enter;
    if (!enter) {
        motorProxyDesiredRun_ = false;
        motorSpeedSendPending_ = false;
    }
    if (enter) {
        // Push a heartbeat right away so the WDG doesn't fire before our
        // 500 ms cadence kicks in.
        nextHeartbeatAt_ = ImGui::GetTime();
        nextMotorProxyKeepaliveAt_ = nextHeartbeatAt_ + 1.0;
    }
}

void MainUi::sendMotorProxy(uint8_t action, float speed_pct)
{
    uint8_t p[8] = {0,0,0,0,0,0xFF,0xFF,0xFF};
    p[0] = action;
    std::memcpy(&p[1], &speed_pct, sizeof(float));
    sendMainCmd(0x012 /* MAIN_CMD_MOTOR_PROXY */, p);
    if (action == 0 /* STOP */ || action == 2 /* PARK */) {
        motorProxyDesiredRun_ = false;
    } else if (action == 1 /* START */) {
        motorProxyDesiredRun_ = true;
    } else if (action == 4 /* SET_SPEED */) {
        const bool deviceInRun =
            lastDiagStatus_.valid &&
            (lastDiagStatus_.fsm_state == 3 /* RUN */ ||
             lastDiagStatus_.fsm_state == 4 /* PARKING */);
        motorProxyDesiredRun_ = motorProxyDesiredRun_ || deviceInRun;
    }
    nextMotorProxyKeepaliveAt_ = ImGui::GetTime() + 1.0;
}

bool MainUi::sendMotorCurrentStep(float iq_amps)
{
    /* Direct MTR_CMD_CTRL frame, driverMode 3 (CURRENT):
     *   data[0]    = 3 (driverMode CURRENT)
     *   data[1..4] = float Iq set-point, amps (LE)
     *   data[5..7] = 0xFF padding
     * The motor FW (Events_Motor_Control case 3) hard-clamps iq_amps to
     * +/-MOTOR_IQ_STEP_MAX_A and refuses to arm on a STOP_MASK fault, so
     * no host-side clamp is needed. Sent straight over the active
     * transport so it works on the SLCAN dongle and both Wi-Fi modes. */
    if (!activeTransport_ || !activeTransport_->isOpen()) return false;
    if (nodeIsInBoot(kCanAddrMotor)) return false;

    CanFrame f{};
    f.id = (uint32_t(1) << 28) |
           (uint32_t(kMotorCmdCtrl) << 16) |
           (uint32_t(kCanAddrMotor) << 8) |
           uint32_t(kCanAddrPc);
    f.extended = true;
    f.fd = false;
    f.brs = false;
    f.rx = false;
    f.dlc = 8;
    f.data.assign(8, 0xFFu);
    f.data[0] = 3u;   // driverMode CURRENT
    std::memcpy(&f.data[1], &iq_amps, sizeof(float));
    return sendFrameThreadSafe(f);
}

void MainUi::sendRkProxy(uint8_t subcmd)
{
    uint8_t p[8] = {0,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    p[0] = subcmd;
    sendMainCmd(0x013 /* MAIN_CMD_RK_PROXY */, p);
}

void MainUi::sendPfcProxy(bool enable, bool overrideSafety)
{
    uint8_t p[8] = {0,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    p[0] = enable ? 1u : 0u;
    p[1] = overrideSafety ? 1u : 0u;
    sendMainCmd(0x014 /* MAIN_CMD_PFC_PROXY */, p);
}

bool MainUi::sendMotorThetaConfig(uint8_t action, float value, uint8_t bin)
{
    if (!activeTransport_ || !activeTransport_->isOpen()) {
        motorThetaTxStatus_ = "not connected";
        return false;
    }
    if (nodeIsInBoot(kCanAddrMotor)) {
        motorThetaTxStatus_ = "motor is in BOOT";
        return false;
    }

    CanFrame f{};
    f.id = (uint32_t(1) << 28) |
           (uint32_t(kMotorCmdConfig) << 16) |
           (uint32_t(kCanAddrMotor) << 8) |
           uint32_t(kCanAddrPc);
    f.extended = true;
    f.fd = false;
    f.brs = false;
    f.rx = false;
    f.dlc = 8;
    f.data.assign(8, 0u);
    f.data[0] = kMotorCfgThetaOffset;
    if (action == kMotorThetaGetSector) {
        // Single-byte payload: bin index in data[1], rest don't care
        // (firmware reads only data[1] for this action).
        f.data[1] = bin;
    } else {
        std::memcpy(&f.data[1], &value, sizeof(float));
    }
    f.data[5] = action;
    const bool nonMutating = (action == kMotorThetaGet) || (action == kMotorThetaGetSector);
    if (!nonMutating) {
        f.data[6] = kMotorThetaGuard0;
        f.data[7] = kMotorThetaGuard1;
    }

    const bool ok = sendFrameThreadSafe(f);
    motorThetaTxStatus_ = ok ? "request sent" : "tx failed";
    if (ok) {
        // Track which response goes where. Multi-frame burst (Read all)
        // pushes 7 entries; auto-poll just adds one and the response loop
        // pops it.
        motorThetaPending_.emplace_back(action, bin);
        nextMotorThetaPollAt_ = clock_.nowSeconds() + 0.25;
    }
    return ok;
}

/* CRC32 IEEE 802.3 (reflected, init 0xFFFFFFFF, final XOR 0xFFFFFFFF).
 * Matches MotorConfigCrc32 in motor FW Core/Src/motor_config.c byte-for-byte
 * (verified by side-by-side trace on the bench 2026-05-13). Used by
 * sendMotorBlobSet to stamp the header.payload_crc before pushing the
 * blob; FW recomputes on its side and rejects with BAD_ARG on mismatch.
 *
 * Distinct from motorStm32HwCrc32 below: this one is for the on-flash
 * payload integrity check (zlib-compatible), the other is for the
 * wire-level READ stream integrity (STM32 CRC peripheral default). */
static uint32_t motorConfigCrc32(const uint8_t* data, std::size_t bytes)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < bytes; ++i) {
        crc ^= uint32_t(data[i]);
        for (int b = 0; b < 8; ++b) {
            const uint32_t lsb_mask = uint32_t(-int32_t(crc & 1u));
            crc = (crc >> 1) ^ (0xEDB88320u & lsb_mask);
        }
    }
    return ~crc;
}

/* STM32 hardware CRC peripheral default replay.
 * Poly 0x04C11DB7, init 0xFFFFFFFF, NO input/output reflection, NO
 * final XOR. Bytes packed into 32-bit words **little-endian** (mirroring
 * `((uint32_t*)buf)[i]` on Cortex-M LE: buf[0] -> low byte of word).
 * Tail < 4 bytes padded with **0xFF in HIGH bytes**, same pattern as
 * propCanGetCrc lines 569-582 in motor FW.
 *
 * Used by handleFrame() on the unified READ protocol's FILE_FINISH
 * (cmd 0xDF) to verify the assembled motorBlob_ against the
 * motor-reported fileCrc. Matches simple_crc32_stm32hw() in app_dd's
 * command_server.cpp byte-for-byte. */
uint32_t MainUi::motorStm32HwCrc32(const uint8_t* data, std::size_t bytes)
{
    auto feed_word = [](uint32_t crc, uint32_t word) -> uint32_t {
        crc ^= word;
        for (int b = 0; b < 32; ++b) {
            crc = (crc & 0x80000000u) ? ((crc << 1) ^ 0x04C11DB7u) : (crc << 1);
        }
        return crc;
    };
    uint32_t crc = 0xFFFFFFFFu;
    const std::size_t whole_words = bytes / 4u;
    for (std::size_t i = 0; i < whole_words; ++i) {
        const uint8_t* p = data + i * 4u;
        const uint32_t word =
            (uint32_t(p[0]))       |
            (uint32_t(p[1]) <<  8) |
            (uint32_t(p[2]) << 16) |
            (uint32_t(p[3]) << 24);
        crc = feed_word(crc, word);
    }
    const std::size_t tail = bytes % 4u;
    if (tail != 0u) {
        const uint8_t* p = data + whole_words * 4u;
        const uint32_t word =
            (uint32_t(p[0]))                                            |
            (uint32_t((tail >= 2) ? p[1] : 0xFFu) <<  8)                |
            (uint32_t((tail >= 3) ? p[2] : 0xFFu) << 16)                |
            (uint32_t(0xFFu) << 24);
        crc = feed_word(crc, word);
    }
    return crc;
}

bool MainUi::sendMotorBlobGet()
{
    if (!activeTransport_ || !activeTransport_->isOpen()) {
        motorThetaTxStatus_ = "not connected";
        return false;
    }

    /* Phase C (2026-05-28): on TCP / Wi-Fi-AP transports, hand the
     * whole READ to app_dd's /motor/config/get instead of cranking the
     * 28-frame unified READ through the lossy TCP-CAN bridge. The HTTP
     * path returns in one JSON body once the SoM-owned MotorConfigClient
     * has CRC-verified the blob locally. On SLCAN we stay on direct CAN
     * (pc_tool is on the same bus, no bridge to bypass). */
    if (motorBlobUsesHttpPath()) {
        if (motorBlobHttpRunning_.load(std::memory_order_acquire)) {
            // Auto-poll re-enters every 1.5 s; if a worker is still
            // running from the previous tick, skip silently.
            motorThetaTxStatus_ = "blob GET already in flight";
            return false;
        }
        const bool launched = launchMotorBlobGetHttp();
        motorThetaTxStatus_ = launched
            ? "blob GET HTTP launched"
            : "blob GET HTTP launch failed";
        return launched;
    }

    /* Unified file-transfer READ (2026-05-13). One CAN frame:
     *   cmd  = 0xD2 READ_HEADER_FILE
     *   data = [addr u32 LE][size u32 LE]
     * Motor replies with HEADER_FILE_OK (0xAD2) echo, then streams
     * HEADER_BLOCK (0xD3) + HEADER_MMSG (0xD4) + NГ—DATA_MMSG (0xD5)
     * + FILE_FINISH (0xDF) with the file CRC. handleFrame() above
     * accumulates bytes into motorBlob_ and validates the host-side
     * STM32-HW-CRC32 against the motor-reported fileCrc on FINISH.
     *
     * Replaces the legacy 33-request BLOB-per-chunk GET that lived
     * here through 2026-05-13. WRITE side (BLOB SET_CHUNK on
     * subcmd 0x03) still uses the old per-chunk path -- Phase 3
     * of the unified-migration roadmap. */

    /* Do NOT clear motorBlobValid_ here. This GET runs on the 1.5 s
     * auto-poll; clearing the flag made every config readout (e.g. the
     * Calibration "Motor configuration" strip) flash "config not read
     * yet" for the request round-trip, then snap back -- a visible blink
     * every 1.5 s. Keep the last-known blob on screen while the fresh
     * one streams in; motorBlobValid_ is re-affirmed true on CRC-OK. */
    motorBlobReceivedMask_ = 0u;
    motorReadInFlight_    = false;        // will be set true on HEADER_FILE_OK
    motorReadExpectedSize_ = 0;
    motorReadCurMmsgIdx_   = 0;
    motorReadCurMmsgSize_  = 0;

    CanFrame f{};
    f.id = (uint32_t(1) << 28) |
           (uint32_t(kFtCmdReadHeaderFile) << 16) |
           (uint32_t(kCanAddrMotor) << 8) |
           uint32_t(kCanAddrPc);
    f.extended = true;
    f.rx       = false;
    f.dlc      = 8;
    f.data.assign(8, 0u);
    const uint32_t addr = kMotorConfigFlashAddr;
    const uint32_t size = kMotorCfgBlobTotalBytes;
    std::memcpy(&f.data[0], &addr, sizeof(addr));
    std::memcpy(&f.data[4], &size, sizeof(size));

    const bool ok = sendFrameThreadSafe(f);
    motorThetaTxStatus_ = ok
        ? "unified READ sent (READ_HEADER_FILE, awaiting stream)"
        : "unified READ tx failed";
    return ok;
}

bool MainUi::sendMotorBlobSet(const uint8_t blob_bytes[164])
{
    if (!activeTransport_ || !activeTransport_->isOpen()) {
        motorThetaTxStatus_ = "not connected";
        return false;
    }

    /* Phase C (2026-05-28): on TCP / Wi-Fi-AP transports, send the
     * whole 164-byte blob in one /motor/config/set call. SoM-side
     * MotorConfigClient drives the 28-frame WRITE locally over CAN and
     * waits for FINISH_OK. The host-side worker thread keeps the GUI
     * responsive; drainMotorBlobHttpResult fans the result into the
     * status text on the next tick. */
    if (motorBlobUsesHttpPath()) {
        return launchMotorBlobSetHttp(blob_bytes);
    }

    /* Phase 3 unified WRITE (2026-05-13). Builds the bootloader-style
     * file-transfer frames for a single-block, single-mmsg, 24-DATA
     * transfer of the 164 B config blob. Wire payload formats mirror
     * motor FW prop_can.c::propCanCmdProcess. Motor detects addr ==
     * MOTOR_CONFIG_FLASH_ADDR and routes into staging-mode (no flash
     * write, MotorConfigScatter on FILE_FINISH).
     *
     * Replaces the legacy BLOB SET_CHUNK + SET_RESULT path on subcmd
     * 0x03. Layout per frame:
     *   0xC0 HEADER_FILE  [addr u32 LE][size u32 LE]
     *   0xC1 HEADER_BLOCK [blockCrc u32][blockIdx u16=0][blockTotal u16=1]
     *   0xC2 HEADER_MMSG  [mmsgSize u16=164][mmsgCrc16 u16][mmsgTotal u16=1][rsv 1B][mmsgIdx u8=0]
     *   0xC3 DATA_MMSG Г—24 [7B payload][msgIdx u8]
     *   0xCF FILE_FINISH  [fileCrc u32]
     * All CRCs use the STM32 HW peripheral algorithm (poly
     * 0x04C11DB7, no reflection, LE pack, 0xFF tail padding). */
    constexpr uint32_t addr  = 0x080E0000u;   /* MOTOR_CONFIG_FLASH_ADDR */
    constexpr uint32_t size  = 164u;
    const uint32_t fileCrc   = motorStm32HwCrc32(blob_bytes, size);
    const uint32_t blockCrc  = fileCrc;       /* single block == file */
    const uint16_t mmsgCrcLo = static_cast<uint16_t>(fileCrc & 0xFFFFu);

    /* Inter-frame pacing for the WRITE burst. Motor's prop_can RX path
     * has a finite SW FIFO; back-to-back blasts of 28 frames (HEADER_FILE +
     * BLOCK + MMSG + 24 DATA + FILE_FINISH) outran the dispatcher and the
     * blockCrc/fileCrc check landed on a partial buffer -- the FW silently
     * dropped DATA_MMSG frames, BLOCK CRC mismatched, and pc_tool's "Set"
     * appeared to do nothing (no ANS_GET_FILE_OK ever came back). 800 us
     * per frame puts the full burst at ~22 ms wall-clock -- imperceptible
     * to the operator and well above CAN bus airtime (~260 us/frame at
     * 500 kbit/s) plus motor's per-frame ISR drain time. */
    auto txFrame = [&](uint16_t cmd, const uint8_t payload[8],
                       bool pace) -> bool {
        CanFrame f{};
        f.id = (uint32_t(1) << 28) |
               (uint32_t(cmd) << 16) |
               (uint32_t(kCanAddrMotor) << 8) |
               uint32_t(kCanAddrPc);
        f.extended = true;
        f.rx       = false;
        f.dlc      = 8;
        f.data.assign(payload, payload + 8);
        const bool sent = sendFrameThreadSafe(f);
        if (sent && pace) {
            std::this_thread::sleep_for(std::chrono::microseconds(800));
        }
        return sent;
    };

    uint8_t buf[8] = {0};
    bool ok = true;

    /* HEADER_FILE [addr][size] */
    std::memset(buf, 0, 8);
    std::memcpy(&buf[0], &addr, sizeof(addr));
    std::memcpy(&buf[4], &size, sizeof(size));
    ok = ok && txFrame(kFtCmdHeaderFile, buf, true);

    /* HEADER_BLOCK [blockCrc][blockIdx=0][blockTotal=1] */
    std::memset(buf, 0, 8);
    std::memcpy(&buf[0], &blockCrc, sizeof(blockCrc));
    /* idx=0, total=1 already 0/1 via memset+manual */
    buf[6] = 1; buf[7] = 0;
    ok = ok && txFrame(kFtCmdHeaderBlock, buf, true);

    /* HEADER_MMSG [mmsgSize][mmsgCrc16][mmsgTotal=1][rsv][mmsgIdx=0] */
    std::memset(buf, 0, 8);
    const uint16_t mmsgSize = 164;
    const uint16_t mmsgTotal = 1;
    std::memcpy(&buf[0], &mmsgSize, sizeof(uint16_t));
    std::memcpy(&buf[2], &mmsgCrcLo, sizeof(uint16_t));
    std::memcpy(&buf[4], &mmsgTotal, sizeof(uint16_t));
    /* buf[6] guard 0x0F per motor FW comment, buf[7] mmsgIdx=0 */
    buf[6] = 0x0F;
    buf[7] = 0;
    ok = ok && txFrame(kFtCmdHeaderMmsg, buf, true);

    /* DATA_MMSG Г—24 (last carries 3 valid bytes, padded with 0xFF) */
    constexpr uint8_t msgTotal = 24;            /* ceil(164/7) */
    for (uint8_t m = 0; m < msgTotal && ok; ++m) {
        const uint16_t srcOff = static_cast<uint16_t>(m) * 7;
        const uint8_t  used   = (srcOff + 7 <= size)
            ? 7u : static_cast<uint8_t>(size - srcOff);
        std::memset(buf, 0xFF, 7);
        std::memcpy(buf, &blob_bytes[srcOff], used);
        buf[7] = m;
        /* Pace all DATA frames except the last -- FILE_FINISH that
         * follows already has its own delay slot from the previous
         * iteration's pacing, plus motor needs the FILE_FINISH to
         * arrive quickly to commit before any next request. */
        ok = ok && txFrame(kFtCmdDataMmsg, buf, m + 1 < msgTotal);
    }

    /* FILE_FINISH [fileCrc] */
    std::memset(buf, 0, 8);
    std::memcpy(&buf[0], &fileCrc, sizeof(fileCrc));
    ok = ok && txFrame(kFtCmdFileFinish, buf, false);

    motorBlobLastSetResult_ = 0xFFu;
    motorThetaTxStatus_ = ok
        ? "unified WRITE sent (1 HEADER + BLOCK + MMSG + 24 DATA + FINISH)"
        : "unified WRITE tx failed mid-stream";
    return ok;
}

bool MainUi::motorBlobSnapshot(uint8_t out_buf[164]) const
{
    if (!motorBlobValid_) return false;
    std::memcpy(out_buf, motorBlob_, 164);
    return true;
}

bool MainUi::applyThetaSlotEdit(int slot, float value_rad)
{
    /* Edit-then-resend semantics. We need a baseline blob to splice the
     * single-field change into; SET without a fresh GET would write a
     * mostly-zero blob and stomp PI gains / profiler results / pfc
     * unintentionally. Guard against that. */
    if (!motorBlobValid_) {
        motorThetaTxStatus_ = "blob cache empty -- press Read calibration first";
        return false;
    }
    if (slot < 0 || slot > 6) {
        motorThetaTxStatus_ = "bad slot index";
        return false;
    }

    uint8_t buf[kMotorCfgBlobTotalBytes];
    std::memcpy(buf, motorBlob_, sizeof(buf));

    const uint16_t off = (slot == 0)
        ? kMotorBlobOffThetaOffset
        : static_cast<uint16_t>(kMotorBlobOffThetaSectorLut + (slot - 1) * 4u);
    std::memcpy(&buf[off], &value_rad, sizeof(value_rad));

    /* Recompute payload CRC32 (byte-identical to FW MotorConfigCrc32) and
     * stamp it back into the header. Without this step the SET commit on
     * the FW side would BAD_ARG and the new value would never reach RAM. */
    const uint32_t crc = motorConfigCrc32(
        &buf[kMotorCfgBlobPayloadOffset], kMotorCfgBlobPayloadBytes);
    std::memcpy(&buf[kMotorBlobOffPayloadCrc], &crc, sizeof(crc));

    /* Also update the in-cache baseline so a subsequent per-slot Set
     * builds on this edit instead of re-applying the old value from the
     * last GET. The fresh GET (auto-poll or manual Read) will eventually
     * supersede this if the motor itself drifts, which is fine. */
    std::memcpy(motorBlob_, buf, sizeof(buf));

    if (!sendMotorBlobSet(buf)) return false;

    if (slot == 0) {
        lastMotorTelemetry_.theta_valid = true;
        lastMotorTelemetry_.theta_offset_rad = value_rad;
        if (lastMotorTelemetry_.theta_cal_valid)
            lastMotorTelemetry_.cal_theta_offset_rad = value_rad;
    } else {
        const int s = slot - 1;
        lastMotorTelemetry_.theta_sector_valid[s] = true;
        lastMotorTelemetry_.theta_sector_lut_rad[s] = value_rad;
        lastMotorTelemetry_.theta_sector_receivedAt[s] = ImGui::GetTime();
    }

    /* Make local state authoritative -- prevents auto-poll BLOB GET
     * from reverting the edit (flash sector 11 still has the pre-Save
     * value). See [[motor-blob-pollguard]]. Cleared by Save All / Read
     * All / 5-min timeout. */
    motorBlobAuthoritativeUntil_ = ImGui::GetTime() + 300.0;

    char msg[96] = {};
    if (slot == 0) {
        std::snprintf(msg, sizeof(msg),
                      "blob SET sent: global offset = %.5f rad", value_rad);
    } else {
        std::snprintf(msg, sizeof(msg),
                      "blob SET sent: sector %d = %.5f rad", slot, value_rad);
    }
    motorThetaTxStatus_ = msg;
    return true;
}

bool MainUi::applyMotorConfigBlobSet(uint16_t id, uint32_t value_bits)
{
    /* Edit-then-resend, generic version of applyThetaSlotEdit. Caller
     * (Motor Config tab Set button) hands us a raw u32 value; the param
     * type (f32 vs u32) is already encoded into those 4 bytes because
     * the table layout on FW side puts each field at a fixed 4-byte
     * offset regardless of type.
     *
     * Same baseline constraint as the theta version: SET without a
     * fresh GET would write a mostly-zero blob and stomp every
     * un-touched field. */
    if (!motorBlobValid_) {
        motorThetaTxStatus_ = "blob cache empty -- press Read All first";
        return false;
    }
    const int off = motorConfigBlobOffsetForId(id);
    if (off < 0) {
        motorThetaTxStatus_ = "id is not blob-resident";
        return false;
    }
    if (off + 4 > static_cast<int>(kMotorCfgBlobTotalBytes)) {
        motorThetaTxStatus_ = "id maps past blob bounds";
        return false;
    }

    uint8_t buf[kMotorCfgBlobTotalBytes];
    std::memcpy(buf, motorBlob_, sizeof(buf));
    std::memcpy(&buf[off], &value_bits, sizeof(value_bits));

    /* Recompute payload CRC32 and stamp it into the header. The FW
     * verifies this before accepting the SET commit. */
    const uint32_t crc = motorConfigCrc32(
        &buf[kMotorCfgBlobPayloadOffset], kMotorCfgBlobPayloadBytes);
    std::memcpy(&buf[kMotorBlobOffPayloadCrc], &crc, sizeof(crc));

    /* Update the local baseline so back-to-back Set buttons compound
     * cleanly without waiting for the next GET. */
    std::memcpy(motorBlob_, buf, sizeof(buf));

    if (!sendMotorBlobSet(buf)) return false;

    /* Make local state authoritative for the next 5 minutes. Motor's
     * unified READ still returns flash (pre-Save) bytes; without this,
     * the 1.5 s auto-poll BLOB GET would visibly revert this edit
     * before the user has a chance to click Save All. Cleared by Save
     * All success, explicit Read All, or this timeout. */
    motorBlobAuthoritativeUntil_ = ImGui::GetTime() + 300.0;

    /* Refresh the cache so the table row's "current" column shows the
     * just-sent value immediately, instead of waiting up to 1.5 s for
     * the next BLOB GET round-trip. */
    populateMotorConfigCacheFromBlob();

    char msg[96] = {};
    std::snprintf(msg, sizeof(msg),
                  "blob SET sent: id=%u value_bits=0x%08X", id, value_bits);
    motorThetaTxStatus_ = msg;
    return true;
}

bool MainUi::sendMotorSectorSet(uint8_t bin, float value, bool save)
{
    if (bin >= 6u) {
        motorThetaTxStatus_ = "bad bin index";
        return false;
    }
    if (!activeTransport_ || !activeTransport_->isOpen()) {
        motorThetaTxStatus_ = "not connected";
        return false;
    }
    if (nodeIsInBoot(kCanAddrMotor)) {
        motorThetaTxStatus_ = "motor is in BOOT";
        return false;
    }

    CanFrame f{};
    f.id = (uint32_t(1) << 28) |
           (uint32_t(kMotorCmdConfig) << 16) |
           (uint32_t(kCanAddrMotor) << 8) |
           uint32_t(kCanAddrPc);
    f.extended = true;
    f.fd = false;
    f.brs = false;
    f.rx = false;
    f.dlc = 8;
    f.data.assign(8, 0u);
    // Layout per AppMotorConfigSubcmd_e::APP_MOTOR_CFG_THETA_SECTOR.
    f.data[0] = kMotorCfgThetaSector;
    f.data[1] = static_cast<uint8_t>((bin & 0x07u) | (save ? 0x80u : 0x00u));
    std::memcpy(&f.data[2], &value, sizeof(float));
    f.data[6] = kMotorThetaGuard0;
    f.data[7] = kMotorThetaGuard1;

    const bool ok = sendFrameThreadSafe(f);
    if (ok) {
        motorThetaPending_.emplace_back(kMotorCfgThetaSector, bin);
        char buf[96] = {};
        std::snprintf(buf, sizeof(buf), "sector %u %s sent: %.5f rad",
                      (unsigned)(bin + 1u), save ? "set+save" : "set", value);
        motorThetaTxStatus_ = buf;
    } else {
        motorThetaTxStatus_ = "tx failed";
    }
    return ok;
}

void MainUi::readAllThetaCalibration()
{
    /* 2026-05-13: was 1 global GET + 6 per-sector GETs (subcmd 0x00/0x01),
     * but per-field polling caused row-shift bugs on a single dropped
     * response and FW no longer accepts those actions (returns BAD_ARG).
     * Replaced with a single whole-blob GET; receive parser populates the
     * same lastMotorTelemetry_ slots atomically from the staged buffer. */
    motorThetaPending_.clear();
    for (uint8_t i = 0; i < 6u; i++) {
        lastMotorTelemetry_.theta_sector_valid[i] = false;
    }
    lastMotorTelemetry_.theta_valid = false;
    /* Explicit user request to refresh: drop the post-Set authoritative
     * lock so the incoming stream populates motorBlob_ from flash. */
    motorBlobAuthoritativeUntil_ = 0.0;
    sendMotorBlobGet();
    motorThetaTxStatus_ = "blob GET sent (33 chunks streaming)";
}

void MainUi::maybePollMotorTheta(double now)
{
    if (!motorThetaAutoPoll_) return;
    if (!activeTransport_ || !activeTransport_->isOpen()) return;
    if (now < nextMotorThetaPollAt_) return;

    /* 2026-05-28 root-cause fix for "live data FPS dip while sitting
     * on the Plots tab": this auto-poll fires every 1.5 s and each
     * tick triggers app_dd's /motor/config/get → unified file-transfer
     * READ over CAN (~30 reliable lane frames + retry spam on
     * contention). Symptom: sched_normal_drop_count climbed by 100+
     * frames between snapshots while user wasn't even on
     * Calibration / Motor Config — STREAM_VALUE losing queue slots
     * to redundant blob refreshes.
     *
     * Gate: only poll when the Calibration tab is visible. The Motor
     * Config tab is fully manual now (explicit Get/Set/Save/Reset) — it
     * must NOT be auto-polled, otherwise the background blob GET keeps
     * the HTTP worker busy and the tab's buttons/edit fields are blocked
     * (the buttons gate on motorBlobHttpRunning_). So this auto-poll
     * exists solely to keep the Calibration tab's live theta fresh. */
    {
        const ImGuiWindow* calibWin = ImGui::FindWindowByName("    Calibration");
        const bool calibVisible = calibWin != nullptr && calibWin->DockTabIsVisible;
        if (!calibVisible) {
            /* Still tick the gate forward so a re-enable doesn't fire
             * a stale burst — when the user comes back to Calibration
             * the next poll happens 1.5 s later, same as fresh. */
            nextMotorThetaPollAt_ = now + 1.5;
            return;
        }
    }

    /* 2026-05-13: replaced per-field GET with whole-blob GET. Period
     * bumped 1.0 -> 1.5 s because each poll now generates 33 response
     * chunks (164 B blob @ 5 B/chunk) -- still well under the host's
     * polling budget but no reason to hammer faster than the user can
     * see changes. */
    nextMotorThetaPollAt_ = now + 1.5;
    /* Skip auto-poll while local edits are authoritative. Motor's
     * unified READ would stream back the flash sector (still pre-Save),
     * and the receive path repopulates motorBlob_ from those bytes --
     * silently reverting the user's edits on every poll tick. The
     * window is cleared by a successful PARAM SAVE_ALL, by an explicit
     * Read All click, or after 5 minutes. */
    if (motorBlobAuthoritativeUntil_ > now) return;
    sendMotorBlobGet();
}

// ---- Tone playback -------------------------------------------------------

bool MainUi::sendMotorTone(uint16_t freq_hz, uint16_t duration_ms, uint8_t amp_pct)
{
    if (!activeTransport_ || !activeTransport_->isOpen()) return false;
    if (nodeIsInBoot(kCanAddrMotor)) return false;

    CanFrame f{};
    f.id = (uint32_t(1) << 28) |
           (uint32_t(kMotorCmdTone) << 16) |
           (uint32_t(kCanAddrMotor) << 8) |
           uint32_t(kCanAddrPc);
    f.extended = true;
    f.fd = false;
    f.brs = false;
    f.rx = false;
    f.dlc = 8;
    f.data.assign(8, 0xFFu);
    f.data[0] = (uint8_t)(freq_hz & 0xFF);
    f.data[1] = (uint8_t)((freq_hz >> 8) & 0xFF);
    f.data[2] = (uint8_t)(duration_ms & 0xFF);
    f.data[3] = (uint8_t)((duration_ms >> 8) & 0xFF);
    f.data[4] = amp_pct;
    f.data[5] = 0xFFu;
    f.data[6] = 0xFFu;
    f.data[7] = 0xFFu;
    return sendFrameThreadSafe(f);
}

void MainUi::startMelody(int melodyIdx)
{
    if (melodyIdx < 0 || melodyIdx >= kMelodyCount) return;
    activeMelodyIdx_  = melodyIdx;
    pendingMelodyIdx_ = -1;
    melodyNoteIdx_    = 0;
    nextMelodyNoteAt_ = clock_.nowSeconds();   // play first note ASAP
}

void MainUi::stopMelody()
{
    activeMelodyIdx_  = -1;
    pendingMelodyIdx_ = -1;
    melodyNoteIdx_    = 0;
    nextMelodyNoteAt_ = 0.0;
    // Best-effort silence: send freq=0 with duration=0... actually firmware
    // ignores zero-duration. Just let the in-flight note finish naturally;
    // the motor auto-reverts to driverMode 0 on duration_ms expiry.
}

void MainUi::pumpMelody(double now)
{
    if (activeMelodyIdx_ < 0 || activeMelodyIdx_ >= kMelodyCount) return;
    if (now < nextMelodyNoteAt_) return;
    const Melody& m = kMelodies[activeMelodyIdx_];
    if (melodyNoteIdx_ >= m.count) {
        // End of melody.
        activeMelodyIdx_  = -1;
        melodyNoteIdx_    = 0;
        return;
    }
    const ToneNote& n = m.notes[melodyNoteIdx_++];

    // Apply transpose (octave shift) and tempo scale.
    uint16_t freq = n.freq_hz;
    if (freq > 0) {
        // Octave shift = multiply/divide by 2^|t|.
        if (melodyTransposeOct_ > 0) {
            int t = melodyTransposeOct_;
            while (t-- > 0 && freq < 1500) freq = (uint16_t)std::min<int>(freq * 2, 1500);
        } else if (melodyTransposeOct_ < 0) {
            int t = -melodyTransposeOct_;
            while (t-- > 0 && freq > 30) freq = (uint16_t)std::max<int>(freq / 2, 30);
        }
        if (freq < 30) freq = 30;
        if (freq > 1500) freq = 1500;
    }
    float scale = melodyDurationScale_;
    if (scale < 0.25f) scale = 0.25f;
    if (scale > 4.0f)  scale = 4.0f;
    uint16_t dur = (uint16_t)std::clamp<int>((int)(n.duration_ms * scale), 1, 2000);
    int amp = std::clamp(melodyAmpPct_, 0, 25);

    if (freq > 0) {
        sendMotorTone(freq, dur, (uint8_t)amp);
    }
    // Advance "when to send next note": current note time + small inter-note
    // gap to give the motor a clean restart between notes.
    const int gap_ms = (m.default_internote_ms > 0) ? m.default_internote_ms : melodyInternoteGapMs_;
    nextMelodyNoteAt_ = now + (dur + gap_ms) / 1000.0;
}

void MainUi::maybeSendDiagHeartbeat()
{
    // Auto-send heartbeats while either local UI state or mainPCB status says
    // diag mode is active. Cadence 500 ms, well inside the 2 s WDG window.
    const double now = ImGui::GetTime();
    if (now < nextHeartbeatAt_) return;
    nextHeartbeatAt_ = now + 0.5;
    const bool deviceSaysDiag = lastDiagStatus_.valid && lastDiagStatus_.diag_active;
    if (!diagModeOn_ && !deviceSaysDiag) return;
    uint8_t p[8] = {0,0,0,0,0,0,0,0};
    p[0] = ++hbSeq_;
    sendMainCmd(0x011 /* MAIN_CMD_DIAG_HEARTBEAT */, p);

    if (motorProxyDesiredRun_ && now >= nextMotorProxyKeepaliveAt_) {
        nextMotorProxyKeepaliveAt_ = now + 1.0;
        const bool deviceInRun =
            lastDiagStatus_.valid &&
            (lastDiagStatus_.fsm_state == 3 /* RUN */ ||
             lastDiagStatus_.fsm_state == 4 /* PARKING */);
        sendMotorProxy(deviceInRun ? 4 /* SET_SPEED */ : 1 /* START */,
                       motorSpeedSlider_);
    }
}

static const char* fsmStateName(uint8_t s)
{
    switch (s) {
        case 0: return "WARNING";
        case 1: return "STOP";
        case 2: return "PAUSE";
        case 3: return "RUN";
        case 4: return "PARKING";
        case 5: return "PARK";
        default: return "?";
    }
}

static const char* motorThetaStatusName(uint8_t s)
{
    switch (s) {
        case 0: return "OK";
        case 1: return "BAD_ARG";
        case 2: return "BUSY";
        case 3: return "NOT_READY";
        case 4: return "STORAGE_EMPTY";
        case 5: return "STORAGE_FAIL";
        case 6: return "FAULT";
        case 7: return "RUNNING";
        default: return "?";
    }
}

static const char* motorProfilerStateName(uint8_t s)
{
    switch (s) {
        case 0: return "IDLE";
        case 1: return "RUNNING";
        case 2: return "DONE";
        case 3: return "ERROR";
        case 4: return "ABORTED";
        default: return "?";
    }
}

static const char* motorProfilerTestName(uint8_t t)
{
    switch (t) {
        case 0: return "NONE";
        case 1: return "XX";
        case 2: return "KZ";
        default: return "?";
    }
}

static const char* motorProfilerErrorName(uint8_t e)
{
    switch (e) {
        case 0: return "NONE";
        case 1: return "BUSY";
        case 2: return "NOT_READY";
        case 3: return "FAULT";
        case 4: return "BAD_ARG";
        case 5: return "MOVED";
        case 6: return "SIGNAL";
        case 7: return "RANGE";
        default: return "?";
    }
}

void MainUi::drawMotorTab()
{
    const bool canSend = activeTransport_ && activeTransport_->isOpen();

    ImGui::TextUnformatted("Motor bench control");
    ImGui::SameLine();
    ImGui::TextDisabled("mainPCB diag-mode proxy");

    ImGui::Text("Control owner:");
    ImGui::SameLine();
    if (lastDiagStatus_.valid) {
        if (lastDiagStatus_.diag_active) {
            ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.25f, 1.0f), "PC TOOL (diag)");
        } else {
            ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.45f, 1.0f), "DEVICE (RK / engineer screen)");
        }
        const double age = ImGui::GetTime() - lastDiagStatus_.receivedAt;
        ImGui::SameLine();
        ImGui::TextDisabled("(status %.1fs old)", age);
    } else {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "(no status yet)");
    }

    if (!canSend) {
        ImGui::TextColored(ImVec4(0.95f, 0.4f, 0.4f, 1.0f),
                           "Connect a transport (Connection tab) before sending commands.");
        ImGui::Separator();
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Diagnostic handover");
    ImGui::Spacing();

    // Take / Release control now lives in the shared per-tab header strip
    // (drawDeviceControlBar) -- one common control, no duplicate here.
    bool inDiag = lastDiagStatus_.valid ? lastDiagStatus_.diag_active : diagModeOn_;
    ImGui::TextDisabled("Use the Take control / Release control button in the "
                        "header bar above.");
    ImGui::SameLine();
    ImGui::TextDisabled("WDG: %u ms remaining", (unsigned)lastDiagStatus_.wdg_remaining_ms);

    ImGui::Separator();
    ImGui::TextUnformatted("Motor commands");
    ImGui::Spacing();

    const bool diagReady = canSend && inDiag;
    ImGui::BeginDisabled(!diagReady);

    // Bipolar slider: negative = REVERSE, positive = FORWARD. Motor firmware
    // takes voltage-control percent in [-100, 100] (MTR_CMD_CTRL driverMode 7);
    // app_dd's MotorControl::setMotorSpeed already clamps to that range.
    const bool speedChanged = ImGui::SliderFloat("Speed %", &motorSpeedSlider_, -100.0f, 100.0f, "%.1f");
    const bool speedReleased = ImGui::IsItemDeactivatedAfterEdit();
    if (speedChanged) {
        motorSpeedSendPending_ = true;
    }
    const double now = ImGui::GetTime();
    if (diagReady && motorSpeedSendPending_ &&
        (speedReleased || now >= nextMotorSpeedSendAt_)) {
        sendMotorProxy(4 /* SET_SPEED */, motorSpeedSlider_);
        motorSpeedSendPending_ = false;
        nextMotorSpeedSendAt_ = now + 0.10; // max 10 Hz while dragging
    }

    // Direction badge + quick controls.
    const float absSpeed = std::fabs(motorSpeedSlider_);
    const bool stopped = absSpeed < 0.05f;
    const bool reverse = motorSpeedSlider_ < 0.0f && !stopped;
    ImVec4 dirCol = stopped  ? ImVec4(0.70f, 0.70f, 0.70f, 1.0f)
                  : reverse ? ImVec4(0.95f, 0.55f, 0.30f, 1.0f)
                            : ImVec4(0.45f, 0.85f, 0.45f, 1.0f);
    ImGui::TextUnformatted("Direction:");
    ImGui::SameLine();
    ImGui::TextColored(dirCol, "%s", stopped ? "STOPPED" : (reverse ? "REVERSE" : "FORWARD"));
    ImGui::SameLine();
    if (ImGui::SmallButton("Flip В±")) {
        motorSpeedSlider_ = -motorSpeedSlider_;
        motorSpeedSendPending_ = true;
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Zero")) {
        motorSpeedSlider_ = 0.0f;
        motorSpeedSendPending_ = true;
    }
    ImGui::TextDisabled("Speed is sent automatically while dragging (max 10 Hz). Negative = reverse.");

    if (ImGui::Button("START", ImVec2(110, 0))) {
        sendMotorProxy(1 /* START */, motorSpeedSlider_);
    }
    ImGui::SameLine();
    if (ImGui::Button("STOP", ImVec2(110, 0))) {
        sendMotorProxy(0 /* STOP */, 0.0f);
    }
    ImGui::SameLine();
    if (ImGui::Button("PARK", ImVec2(110, 0))) {
        sendMotorProxy(2 /* PARK */, 0.0f);
    }
    ImGui::SameLine();
    if (ImGui::Button("RESET ERR", ImVec2(110, 0))) {
        sendMotorProxy(3 /* RESET_ERR */, 0.0f);
    }

    ImGui::Spacing();
    bool pfcOn = lastDiagStatus_.pfc_enabled;
    const bool pfcMotorActive =
        lastDiagStatus_.motor_running ||
        lastDiagStatus_.fsm_state == 3 ||   // RUN
        lastDiagStatus_.fsm_state == 4;     // PARKING
    const bool pfcBlockedBySafety = pfcSafetyLock_ && pfcMotorActive;
    ImGui::TextUnformatted("PFC:");
    ImGui::SameLine();
    ImGui::Checkbox("Block while motor running", &pfcSafetyLock_);
    ImGui::SameLine();
    if (pfcSafetyLock_) {
        ImGui::TextDisabled("safe");
    } else {
        ImGui::TextColored(ImVec4(0.95f,0.45f,0.25f,1.0f), "OVERRIDE");
    }

    ImGui::BeginDisabled(pfcBlockedBySafety);
    if (ImGui::Button(pfcOn ? "Disable PFC" : "Enable PFC", ImVec2(160, 0))) {
        const bool newState = !pfcOn;
        sendPfcProxy(newState, !pfcSafetyLock_);
        /* Also stage the persistent boot-default bit so SAVE_ALL from
         * Motor Config tab will pin this choice. Motor FW v2.3.8+ has
         * PARAM_ID_CFG_PFC_DEFAULT=50; older FW returns BAD_ARG harmlessly. */
        const uint32_t bit = newState ? 1u : 0u;
        sendMotorConfigParam(/*SET*/ 1, /*PARAM_ID_CFG_PFC_DEFAULT*/ 50, bit);
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextColored(pfcOn ? ImVec4(0.45f,0.85f,0.45f,1.0f) : ImVec4(0.85f,0.85f,0.85f,1.0f),
                       "%s", pfcOn ? "ENABLED" : "disabled");
    if (pfcBlockedBySafety) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.95f,0.75f,0.25f,1.0f),
                           "blocked: stop motor or clear checkbox");
    }

    ImGui::EndDisabled();

    ImGui::Separator();
    // FOC theta calibration migrated to the Calibration tab (Test 1).
    // See docs/calibration-tab-redesign.md -- the theta LUT editor now
    // lives under Calibration > Test 1, this tab keeps motor telemetry,
    // commands and the melody player only.
    ImGui::TextUnformatted("FOC theta calibration");
    ImGui::SameLine();
    ImGui::TextDisabled("-> moved to the Calibration tab (Test 1)");

    // ---- Tone / melodies (driverMode 8, CAN cmd 0x008) ------------------
    ImGui::Separator();
    ImGui::TextUnformatted("Tone / melodies");
    ImGui::SameLine();
    ImGui::TextDisabled("(motor stator buzzes the note for fun)");

    ImGui::SetNextItemWidth(180.0f);
    ImGui::SliderInt("Volume %##melody-amp", &melodyAmpPct_, 0, 25);
    ImGui::SameLine();
    ImGui::TextDisabled("firmware caps at 25");

    ImGui::SetNextItemWidth(120.0f);
    ImGui::SliderInt("Octave##melody-octave", &melodyTransposeOct_, -2, 2);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120.0f);
    ImGui::SliderFloat("Tempo##melody-tempo", &melodyDurationScale_, 0.5f, 2.0f, "%.2fx");
    ImGui::SameLine();
    ImGui::TextDisabled("(>1 = slower)");

    ImGui::BeginDisabled(!canSend);
    for (int i = 0; i < kMelodyCount; ++i) {
        if (i > 0 && (i % 3) != 0) ImGui::SameLine();
        char btn[64];
        std::snprintf(btn, sizeof(btn), "%s##melody%d", kMelodies[i].name, i);
        const bool isActive = (activeMelodyIdx_ == i);
        if (isActive) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.45f, 0.85f, 0.45f, 0.7f));
        if (ImGui::Button(btn, ImVec2(180, 0))) {
            startMelody(i);
        }
        if (isActive) ImGui::PopStyleColor();
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    if (ImGui::Button("Stop##melody-stop", ImVec2(70, 0))) {
        stopMelody();
    }

    if (activeMelodyIdx_ >= 0 && activeMelodyIdx_ < kMelodyCount) {
        const Melody& m = kMelodies[activeMelodyIdx_];
        ImGui::TextDisabled("Playing: %s вЂ” note %u / %u", m.name,
                            (unsigned)melodyNoteIdx_, (unsigned)m.count);
    } else {
        ImGui::TextDisabled("Idle.");
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Motor telemetry");
    ImGui::Columns(2, "##motor-tel", false);
    ImGui::SetColumnWidth(0, 180.0f);

    ImGui::TextUnformatted("FSM state");
    ImGui::NextColumn();
    if (lastDiagStatus_.valid)
        ImGui::Text("%u (%s)", (unsigned)lastDiagStatus_.fsm_state, fsmStateName(lastDiagStatus_.fsm_state));
    else
        ImGui::TextDisabled("n/a");
    ImGui::NextColumn();

    ImGui::TextUnformatted("Motor running");
    ImGui::NextColumn();
    if (lastDiagStatus_.valid)
        ImGui::TextColored(lastDiagStatus_.motor_running ? ImVec4(0.45f,0.85f,0.45f,1.0f) : ImVec4(0.85f,0.85f,0.85f,1.0f),
                           "%s", lastDiagStatus_.motor_running ? "YES" : "no");
    else
        ImGui::TextDisabled("n/a");
    ImGui::NextColumn();

    ImGui::TextUnformatted("RPM");                   ImGui::NextColumn();
    ImGui::Text("%.1f", lastMotorTelemetry_.rpm);    ImGui::NextColumn();

    ImGui::TextUnformatted("V_dc (V)");              ImGui::NextColumn();
    ImGui::Text("%.2f", lastMotorTelemetry_.voltage_dc); ImGui::NextColumn();

    ImGui::TextUnformatted("V_q (V)");               ImGui::NextColumn();
    ImGui::Text("%.2f", lastMotorTelemetry_.voltage_q); ImGui::NextColumn();

    ImGui::TextUnformatted("Power (W)");             ImGui::NextColumn();
    ImGui::Text("%.1f", lastMotorTelemetry_.power_W); ImGui::NextColumn();

    ImGui::TextUnformatted("Motor temp (deg C)");     ImGui::NextColumn();
    ImGui::Text("%.1f", lastMotorTelemetry_.motor_temp); ImGui::NextColumn();

    ImGui::TextUnformatted("Inverter temp (deg C)");  ImGui::NextColumn();
    ImGui::Text("%.1f", lastMotorTelemetry_.vt_temp); ImGui::NextColumn();

    ImGui::TextUnformatted("Mode (work)");           ImGui::NextColumn();
    ImGui::Text("%u", (unsigned)lastMotorTelemetry_.mode); ImGui::NextColumn();

    ImGui::TextUnformatted("Fault mask");            ImGui::NextColumn();
    if (lastMotorTelemetry_.fault_mask == 0)
        ImGui::TextColored(ImVec4(0.45f,0.85f,0.45f,1.0f), "0x0 (clean)");
    else if ((lastMotorTelemetry_.fault_mask & 0x1FFu) != 0)
        ImGui::TextColored(ImVec4(0.95f,0.4f,0.4f,1.0f), "0x%08X (PWM blocked)", lastMotorTelemetry_.fault_mask);
    else
        ImGui::TextColored(ImVec4(0.95f,0.75f,0.25f,1.0f), "0x%08X (observers only)", lastMotorTelemetry_.fault_mask);
    ImGui::NextColumn();

    ImGui::Columns(1);
}

void MainUi::drawRkTab()
{
    const bool canSend = activeTransport_ && activeTransport_->isOpen();
    bool inDiag = lastDiagStatus_.valid ? lastDiagStatus_.diag_active : diagModeOn_;
    const bool diagReady = canSend && inDiag;

    ImGui::Text("RK bench control");
    ImGui::TextDisabled("Drives RK_CMD_MOTOR_CTRL via mainPCB. Requires diag mode.");

    if (!canSend)
        ImGui::TextColored(ImVec4(0.95f,0.4f,0.4f,1.0f), "Connect a transport first.");
    else if (!inDiag)
        ImGui::TextColored(ImVec4(0.95f,0.75f,0.25f,1.0f), "Enter diag mode above to enable RK commands.");

    ImGui::Separator();
    ImGui::TextUnformatted("Emergency stop (red button):");
    ImGui::Spacing();
    ImGui::BeginDisabled(!diagReady);
    if (ImGui::Button("ARM (e-stop EN)", ImVec2(180, 0))) {
        sendRkProxy(0x01 /* MOTOR_SUB_CMD_EMERGENCY_MODE_EN */);
    }
    ImGui::SameLine();
    if (ImGui::Button("DISARM (e-stop DIS)", ImVec2(180, 0))) {
        sendRkProxy(0x00 /* MOTOR_SUB_CMD_EMERGENCY_MODE_DIS */);
    }
    ImGui::EndDisabled();

    ImGui::Spacing();
    ImGui::TextUnformatted("Parking mode:");
    ImGui::Spacing();
    ImGui::BeginDisabled(!diagReady);
    if (ImGui::Button("PARKING ON", ImVec2(180, 0))) {
        sendRkProxy(0x03 /* MOTOR_SUB_CMD_PARKING_MODE_EN */);
    }
    ImGui::SameLine();
    if (ImGui::Button("PARKING OFF", ImVec2(180, 0))) {
        sendRkProxy(0x02 /* MOTOR_SUB_CMD_PARKING_MODE_DIS */);
    }
    ImGui::EndDisabled();

    ImGui::Separator();
    ImGui::TextUnformatted("Status (mirrored from mainPCB DIAG_STATUS broadcast):");
    if (!lastDiagStatus_.valid) {
        ImGui::TextDisabled("(no status yet - make sure app_dd is built with diag broadcast)");
        return;
    }

    ImGui::Columns(2, "##rk-status", false);
    ImGui::SetColumnWidth(0, 200.0f);
    ImGui::TextUnformatted("RK armed");        ImGui::NextColumn();
    ImGui::TextColored(lastDiagStatus_.rk_armed ? ImVec4(0.45f,0.85f,0.45f,1.0f) : ImVec4(0.95f,0.4f,0.4f,1.0f),
                       "%s", lastDiagStatus_.rk_armed ? "ARMED" : "disarmed");
    ImGui::NextColumn();
    ImGui::TextUnformatted("Motor FSM");       ImGui::NextColumn();
    ImGui::Text("%s", fsmStateName(lastDiagStatus_.fsm_state));
    ImGui::NextColumn();
    ImGui::TextUnformatted("HB seq echo");     ImGui::NextColumn();
    ImGui::Text("%u", (unsigned)lastDiagStatus_.hb_seq_echo);
    ImGui::NextColumn();
    ImGui::Columns(1);
}

// ----------------------------------------------------------------------------
// Triggered Capture session machinery.
//
// State machine: Idle -> Reset -> SetSlot -> SetConfig -> SetTrigger ->
// Arm -> WaitDone (poll) -> ReadChunk (await CAP_DATA stream) -> Apply ->
// CooldownDone -> Idle.
//
// The session state lives in capture_; the trigger button in
// drawCaptureControl() boots it. Cooldown is roughly the worst-case dump
// time + capture window so the user can't queue up a second shot until
// the first has fully arrived; this prevents partial overlap on the
// shared watch xs/ys buffers.
// ----------------------------------------------------------------------------

double MainUi::estimateCaptureCooldownSec() const
{
    if (capture_.bytesPerSample == 0u) return 1.5;
    // Worst case: motor 1365 samples * 24 B = 32 KB.
    // Stream rate empirically ~5 bytes per CAN frame, ~2.5 ms inter-frame
    // when the 400 Hz motor TX path runs unimpaired -> ~2 KB/s.
    // Throw in a 1 s pad for arming + status round-trips.
    const double bytes = static_cast<double>(capture_.bytesExpected);
    return 1.0 + bytes / 1500.0;
}

bool MainUi::isCapturePollSuppressed(const WatchVar& w) const
{
    // Live polling keeps running during a capture session. The MCU sampler
    // is in an ISR / dedicated timer (motor: TIM6 ISR, RK: TIM6 callback,
    // ESP32: esp_timer) and the CAP_DATA stream pacer leaves ~85% of bus
    // arbitration headroom, so READ_MEM responses keep flowing in
    // parallel -- there is no reason to freeze the live trace while we
    // wait for the burst to download.
    //
    // applyCapturedSamples() drops any live-poll samples that fall inside
    // the capture window before inserting the high-resolution burst, so
    // the historical interleave artefact ("staircase in the wavy region")
    // is handled there, not here. Suppressing the poll for the duration
    // of the session was a workaround that left the trace dead for the
    // 1+ s + bytes/1500 cooldown, which is exactly the complaint we are
    // fixing.
    (void)w;
    return false;
}

// ----------------------------------------------------------------------------
// HTTP-driven Triggered Capture path. Used when transport is TCP / WifiAp,
// where pc_tool is not on the same CAN bus as the MCUs and would otherwise
// have to drive every CAP_REQ frame through the app_dd TCP bridge -- adding
// ~5-50 ms of Wi-Fi RTT to each of ~thousands of small messages. The
// /capture endpoint on app_dd runs the entire session locally over CAN and
// returns the result as JSON {ok, samples, slots, total_bytes, ms, crc,
// crc_hex, data_base64} so the host side becomes one HTTP fetch instead of
// a CAP_REQ state machine.
//
// Wire contract (current — owned by app_dd /capture in command_server.cpp):
//   request  : GET /capture?node=2&period_us=...&buffer_kb=...
//                          &slot0=0xADDR:TYPE[:SIZE]&slotN=...
//              TYPE ∈ {u8,i8,u16,i16,u32,i32,f32}; SIZE optional, 1..4
//   response : 200 + JSON; on failure ok=false with `error` and an HTTP
//              status of 400/409/500/503
//
// Transport: Bitvise sexec / plink wraps `wget -qO- ...` on the device,
// captured via runCommandHidden into a std::string. Body is JSON text;
// payload bytes live in data_base64 (decoded locally with a small
// header-only decoder). No nc/printf dance — wget is plenty since we
// only need the body.
// ----------------------------------------------------------------------------

bool MainUi::captureUsesHttpPath() const
{
    return transportKind_ == TransportKind::Tcp ||
           transportKind_ == TransportKind::WifiAp;
}

namespace {
// Map a WatchVar.type string ("float", "uint32", "int8", ...) to the
// short type token accepted by app_dd's /capture endpoint
// (CapClient::parseSlotSpec — see app_ssd202d_ddv2/src/devices/can_bus/
// cap_client.cpp). Token set is u8/i8/u16/i16/u32/i32/f32; unknown types
// fall through to f32 so the slot still gets sampled (pc_tool decodes
// the 4 bytes by WatchVar.type via decodeValue() on the return path).
const char* captureHttpSlotTypeStr(const std::string& type)
{
    if (type == "float" || type == "f32") return "f32";
    if (type == "uint8"  || type == "u8")  return "u8";
    if (type == "int8"   || type == "i8")  return "i8";
    if (type == "uint16" || type == "u16") return "u16";
    if (type == "int16"  || type == "i16") return "i16";
    if (type == "uint32" || type == "u32") return "u32";
    if (type == "int32"  || type == "i32") return "i32";
    return "f32";
}

// --- Minimal JSON value extractor ---------------------------------------
// /capture returns flat top-level JSON objects ({"ok":true,"samples":...})
// so we don't need a real parser. These helpers locate "key": value pairs
// by literal scan and parse the value as either a number, a bare token
// (true/false), or a double-quoted string. Whitespace-tolerant. Returns
// empty / 0 when key is absent. Quoted string values support "\\" / "\""
// escapes only — data_base64 contains only [A-Za-z0-9+/=] so the simple
// path is sufficient. Anything fancier (nested objects, arrays) would
// need a real parser, but the /capture body is intentionally flat.

bool jsonFindValueStart(const std::string& json, const std::string& key,
                        size_t& valueStart)
{
    const std::string needle = "\"" + key + "\"";
    size_t pos = 0;
    while ((pos = json.find(needle, pos)) != std::string::npos) {
        // Make sure the match isn't a substring inside another key
        // (e.g. "data_base64" containing "data"): require the next non-
        // whitespace char to be ':'.
        size_t cur = pos + needle.size();
        while (cur < json.size() &&
               (json[cur] == ' ' || json[cur] == '\t' ||
                json[cur] == '\r' || json[cur] == '\n')) ++cur;
        if (cur < json.size() && json[cur] == ':') {
            ++cur;
            while (cur < json.size() &&
                   (json[cur] == ' ' || json[cur] == '\t' ||
                    json[cur] == '\r' || json[cur] == '\n')) ++cur;
            valueStart = cur;
            return true;
        }
        pos += needle.size();
    }
    return false;
}

std::string jsonString(const std::string& json, const std::string& key)
{
    size_t vs;
    if (!jsonFindValueStart(json, key, vs)) return {};
    if (vs >= json.size() || json[vs] != '"') return {};
    ++vs;
    std::string out;
    out.reserve(64);
    while (vs < json.size()) {
        const char c = json[vs];
        if (c == '\\' && vs + 1 < json.size()) {
            const char n = json[vs + 1];
            if      (n == '"')  out.push_back('"');
            else if (n == '\\') out.push_back('\\');
            else if (n == '/')  out.push_back('/');
            else if (n == 'n')  out.push_back('\n');
            else if (n == 't')  out.push_back('\t');
            else if (n == 'r')  out.push_back('\r');
            else                out.push_back(n);  // unknown escape — pass through
            vs += 2;
            continue;
        }
        if (c == '"') break;
        out.push_back(c);
        ++vs;
    }
    return out;
}

uint64_t jsonNumber(const std::string& json, const std::string& key, bool* found = nullptr)
{
    size_t vs;
    if (!jsonFindValueStart(json, key, vs)) {
        if (found) *found = false;
        return 0;
    }
    if (found) *found = true;
    // Accept "...":N or "...":"0xN". strtoull eats 0x prefix.
    if (vs < json.size() && json[vs] == '"') ++vs;  // tolerate quoted numbers
    char* end = nullptr;
    return static_cast<uint64_t>(std::strtoull(json.c_str() + vs, &end, 0));
}

bool jsonBool(const std::string& json, const std::string& key)
{
    size_t vs;
    if (!jsonFindValueStart(json, key, vs)) return false;
    return (vs + 4 <= json.size() && json.compare(vs, 4, "true") == 0);
}

// --- Base64 decode (RFC 4648, accepts BOTH standard +/  and URL-safe -_
// alphabets; ignores whitespace and '=' padding) -------------------------
bool base64Decode(const std::string& s, std::vector<uint8_t>& out)
{
    static int8_t table[256];
    static bool inited = false;
    if (!inited) {
        for (int i = 0; i < 256; ++i) table[i] = -1;
        const char* abc = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        for (int i = 0; i < 64; ++i) table[static_cast<uint8_t>(abc[i])] = static_cast<int8_t>(i);
        // URL-safe aliases: '-' === '+' (62), '_' === '/' (63).
        table[static_cast<uint8_t>('-')] = 62;
        table[static_cast<uint8_t>('_')] = 63;
        inited = true;
    }
    out.clear();
    out.reserve((s.size() * 3u) / 4u + 4u);
    uint32_t acc = 0;
    int bits = 0;
    for (char c : s) {
        if (c == '=' || c == ' ' || c == '\r' || c == '\n' || c == '\t') continue;
        const int v = table[static_cast<uint8_t>(c)];
        if (v < 0) return false;
        acc = (acc << 6) | static_cast<uint32_t>(v);
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out.push_back(static_cast<uint8_t>((acc >> bits) & 0xFFu));
        }
    }
    return true;
}

} // namespace

std::string MainUi::buildCaptureHttpQuery() const
{
    std::string q;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "node=%u", static_cast<unsigned>(capture_.nodeId));
    q += buf;
    std::snprintf(buf, sizeof(buf), "&duration_ms=%d", capture_.durationMs);
    q += buf;
    std::snprintf(buf, sizeof(buf), "&period_us=%u", static_cast<unsigned>(capture_.periodUs));
    q += buf;
    std::snprintf(buf, sizeof(buf), "&buffer_kb=%u", static_cast<unsigned>(capture_.bufferKb));
    q += buf;
    std::snprintf(buf, sizeof(buf), "&pre_pct=%u", static_cast<unsigned>(capture_.prePct));
    q += buf;
    std::snprintf(buf, sizeof(buf), "&post_pct=%u", static_cast<unsigned>(capture_.postPct));
    q += buf;

    for (size_t slot = 0; slot < capture_.watchIdxBySlot.size() && slot < kCapMaxSlots; ++slot) {
        const size_t wi = capture_.watchIdxBySlot[slot];
        if (wi >= watches_.size()) continue;
        const WatchVar& w = watches_[wi];
        // app_dd /capture slot syntax: slot0=0xADDR:TYPE[:SIZE]
        // TYPE = u8/i8/u16/i16/u32/i32/f32 (see CapClient::parseSlotSpec).
        // SIZE is optional — the type already implies width — so we omit
        // it. Motor packs 4 bytes per slot per sample on the wire today;
        // applyCapturedSamples reads with that stride and decodes by
        // WatchVar.type.
        std::snprintf(buf, sizeof(buf), "&slot%zu=0x%X:", slot,
                      static_cast<unsigned>(w.address));
        q += buf;
        q += captureHttpSlotTypeStr(w.type);
    }
    return q;
}

bool MainUi::fetchCaptureViaHttp(const std::string& host,
                                 const std::string& query_string,
                                 CaptureHttpResult& out)
{
    out = CaptureHttpResult{};

    if (host.empty()) {
        out.error_stage = "host";
        out.error_message = "no host (transport not TCP/AP?)";
        return false;
    }

    /* Async-capture flow (new on 2026-05-28):
     *   1. GET /capture/start?<query>          → returns session_id immediately
     *   2. GET /capture/progress?session_id=N  → polled every ~250 ms; updates
     *                                            captureHttpPhaseCode_, samples,
     *                                            bytes_received, total_bytes
     *                                            so the toolbar tooltip moves.
     *   3. GET /capture/result?session_id=N    → final JSON blob (same shape as
     *                                            legacy /capture body)
     *
     * Why split off the legacy synchronous /capture: BusyBox `wget -qO-` on
     * the device buffers the entire HTTP response before printing the first
     * byte to stdout, so runCommandHidden sees zero output for the full
     * device-side run. That tripped the host-side stall timer ("command
     * stalled for 12s") even when the capture itself was making forward
     * progress over CAN. With three short HTTP calls instead of one long
     * one, each individual SSH+wget round-trip is ~200-500 ms and the
     * progress endpoint reads atomics (no CAN traffic), so the tooltip can
     * genuinely report bytes_received / total_bytes as the device-side
     * READ_CHUNK loop walks the motor's ring buffer.
     */

    constexpr const char* kPassword = "bork2025";
    const std::string discoveredFp = discoveredHostKeyFp(host);
    const std::string sexec = recoveryBitviseSexec();
    const std::string plink = recoveryPlinkExe();
    if (sexec.empty() && plink.empty()) {
        out.error_stage = "ssh";
        out.error_message = "no SSH client (sexec/plink) found";
        return false;
    }

    /* Build the sexec/plink invocation that runs `remoteShell` on the
     * device. Sized for short HTTP fetches: 8s wget timeout is enough for
     * /capture/start and /capture/progress; /capture/result uses 60s
     * because it can carry up to ~32 KiB of base64 + JSON over Wi-Fi. */
    auto buildSshInvoke = [&](const std::string& remoteShell) -> std::string {
        if (!sexec.empty()) {
            return quoteArg(sexec) +
                   " " + quoteArg(std::string("-host=") + host) +
                   " -user=root" +
                   " " + quoteArg(std::string("-pw=") + kPassword) +
                   " -unat=y -exitZero" +
                   (discoveredFp.empty()
                        ? std::string()
                        : std::string(" -hostKeyFp=") +
                          sha256FpForBitvise(discoveredFp)) +
                   " " + quoteArg(std::string("-cmd=") + remoteShell);
        }
        return quoteArg(plink) +
               " -batch -ssh" +
               (discoveredFp.empty()
                    ? std::string()
                    : std::string(" -hostkey ") + quoteArg(discoveredFp)) +
               " -pw " + quoteArg(kPassword) +
               " root@" + host + " " + quoteArg(remoteShell);
    };

    /* Issue one HTTP GET to the device's loopback command-server via SSH,
     * return the body and the HTTP status. We use raw HTTP/1.0 over `nc`
     * (not `wget -qO-`) for two reasons:
     *   1. BusyBox `wget -q` drops the response BODY on any non-2xx status,
     *      so a 404 from `/capture/start` on an app_dd that doesn't have
     *      the async endpoints yet would surface to us as "0 B response"
     *      with no useful error.
     *   2. `wget` buffers the full body before flushing, but we already
     *      switched to async polling so that doesn't matter — what
     *      matters is that nc gives us status + body verbatim. */
    struct HttpReply {
        bool        ok = false;
        int         http_status = 0;
        std::string body;
        std::string stage;       // populated on !ok
        std::string error;
        int         ssh_rc = 0;
    };
    auto httpGet = [&](const std::string& devicePath,
                       int wgetTimeoutSec,
                       uint32_t timeoutMs,
                       uint32_t stallMs) -> HttpReply {
        HttpReply rep;
        char tbuf[16];
        std::snprintf(tbuf, sizeof(tbuf), "%d", wgetTimeoutSec);
        /* Build a raw HTTP/1.0 request. HTTP/1.0 + `Connection: close`
         * makes the server (command_server.cpp) write the full response
         * and close the socket, so `nc -w N` returns cleanly without
         * waiting for more data.
         *
         * Single-quote the request inside the remote shell so `$` /
         * backticks survive unexpanded. `devicePath` is already URL-
         * encoded by buildCaptureHttpQuery on the caller side. */
        const std::string remoteCmd =
            std::string("printf 'GET ") + devicePath +
            " HTTP/1.0\\r\\nHost: 127.0.0.1\\r\\nConnection: close\\r\\n\\r\\n' "
            "| nc -w " + tbuf + " 127.0.0.2 10011";
        const std::string cmd = buildSshInvoke(remoteCmd);

        std::string output;
        output.reserve(8 * 1024);
        const CommandResult result = runCommandHidden(cmd, [&](const std::string& chunk) {
            output += chunk;
        }, timeoutMs, stallMs);
        rep.ssh_rc = result.rc;
        if (output.empty()) {
            rep.stage = "ssh";
            rep.error = "no response from 127.0.0.2:10011 over SSH (rc=" +
                        std::to_string(result.rc) + ")";
            if (!result.output.empty()) rep.error += ": " + tailText(result.output, 200);
            return rep;
        }

        /* Parse "HTTP/1.x NNN <reason>\r\n<headers>\r\n\r\n<body>"
         * Tolerate both CRLF and bare-LF separators (BusyBox sed/nc
         * sometimes mangles CR). */
        size_t headEnd = output.find("\r\n\r\n");
        size_t sepLen  = 4;
        if (headEnd == std::string::npos) {
            headEnd = output.find("\n\n");
            sepLen  = 2;
        }
        const std::string headers = (headEnd == std::string::npos)
            ? output : output.substr(0, headEnd);
        const std::string bodyRaw = (headEnd == std::string::npos)
            ? std::string() : output.substr(headEnd + sepLen);

        // Status line: "HTTP/1.x <status> <reason>"
        size_t sp1 = headers.find(' ');
        if (sp1 != std::string::npos) {
            rep.http_status = static_cast<int>(std::strtol(
                headers.c_str() + sp1 + 1, nullptr, 10));
        }
        rep.body = bodyRaw;
        if (rep.http_status == 0) {
            // No HTTP status parsed — most likely a transport error
            // (sexec banner without a server response, or nc timeout).
            rep.stage = "transport";
            rep.error = "no HTTP status line in response; head=" +
                        tailText(output, 200);
            return rep;
        }
        if (rep.http_status < 200 || rep.http_status >= 300) {
            // Server-side error WITH a JSON body — caller can still parse
            // `error` out of it for a precise message.
            rep.stage = "http_" + std::to_string(rep.http_status);
            const size_t jb = bodyRaw.find('{');
            const size_t je = bodyRaw.rfind('}');
            if (jb != std::string::npos && je != std::string::npos && je >= jb) {
                const std::string j = bodyRaw.substr(jb, je - jb + 1);
                const std::string msg = jsonString(j, "error");
                rep.error = "HTTP " + std::to_string(rep.http_status) +
                            (msg.empty() ? std::string() : (": " + msg));
                rep.body = j;
            } else {
                rep.error = "HTTP " + std::to_string(rep.http_status) +
                            "; body=" + tailText(bodyRaw, 200);
            }
            return rep;
        }

        // 2xx: pull just the JSON object out of the body. The body
        // should already BE the JSON, but tolerate any trailing
        // whitespace / control chars nc might leave behind.
        const size_t jb = bodyRaw.find('{');
        const size_t je = bodyRaw.rfind('}');
        if (jb == std::string::npos || je == std::string::npos || je < jb) {
            rep.stage = "json_parse";
            rep.error = "no JSON object in " + devicePath +
                        " response (HTTP 200, got " +
                        std::to_string(bodyRaw.size()) + " B body)";
            if (!bodyRaw.empty()) rep.error += "; head=" + tailText(bodyRaw, 200);
            return rep;
        }
        rep.body = bodyRaw.substr(jb, je - jb + 1);
        rep.ok = true;
        return rep;
    };

    // ---- Phase 1: /capture/start ----------------------------------------
    recoveryLog(std::string("[capture-http] START /capture/start?") + query_string);
    HttpReply startRep = httpGet("/capture/start?" + query_string,
                                 /*wgetTimeoutSec*/ 8,
                                 /*timeoutMs*/ 12000,
                                 /*stallMs*/ 10000);
    if (!startRep.ok) {
        out.error_stage = startRep.stage;
        out.error_message = startRep.error;
        recoveryLog(std::string("[capture-http] start FAIL: ") + out.error_stage +
                    ": " + out.error_message);
        return false;
    }
    if (!jsonBool(startRep.body, "ok")) {
        const std::string err = jsonString(startRep.body, "error");
        out.error_stage = "start";
        out.error_message = err.empty()
            ? std::string("ok=false from /capture/start; body=") + tailText(startRep.body, 200)
            : err;
        recoveryLog(std::string("[capture-http] start FAIL: ") + out.error_message);
        return false;
    }
    bool haveSid = false;
    const uint64_t sessionId = jsonNumber(startRep.body, "session_id", &haveSid);
    if (!haveSid || sessionId == 0) {
        out.error_stage = "start";
        out.error_message = "no session_id in /capture/start response; body=" +
                            tailText(startRep.body, 200);
        return false;
    }
    char sidQuery[64];
    std::snprintf(sidQuery, sizeof(sidQuery), "?session_id=%llu",
                  static_cast<unsigned long long>(sessionId));

    // Reset progress atomics so the tooltip starts at zero, not whatever
    // the previous run left behind.
    captureHttpBytesRx_.store(0, std::memory_order_relaxed);
    captureHttpBytesExpected_.store(0, std::memory_order_relaxed);
    captureHttpPhaseCode_.store(1 /*PHASE_SETUP*/, std::memory_order_release);
    captureHttpSamples_.store(0, std::memory_order_relaxed);
    captureHttpElapsedMs_.store(0, std::memory_order_relaxed);

    // ---- Phase 2: /capture/progress polling -----------------------------
    /* Total poll envelope sized for the worst observed real-world case:
     * a 32 KiB / 4-slot / 100 us capture under live TCP-CAN bridge load
     * (CAN ~18 %, motor's TX FIFO competing with STREAM broadcasts) ran
     * 54 s on 2026-05-28 once cap_client.h bumped retries to 8 ×
     * 10 s chunk_timeout. 240 s leaves ~3 worst-case windows of
     * head-room without ever artificially capping the device-side
     * recovery time. The inner per-call timeout (6 s) is short -- each
     * progress request just reads atomics on the device. */
    const auto pollStart = std::chrono::steady_clock::now();
    const auto pollDeadline = pollStart + std::chrono::seconds(240);
    bool gotDone = false;
    std::string lastProgressBody;
    int consecutiveSshFails = 0;
    while (std::chrono::steady_clock::now() < pollDeadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        HttpReply progRep = httpGet(std::string("/capture/progress") + sidQuery,
                                    /*wgetTimeoutSec*/ 5,
                                    /*timeoutMs*/  8000,
                                    /*stallMs*/    6000);
        if (!progRep.ok) {
            // Tolerate up to three back-to-back SSH glitches (transient
            // Wi-Fi packet loss is common on STA mode) before bailing.
            if (++consecutiveSshFails >= 3) {
                out.error_stage = progRep.stage.empty() ? "progress" : progRep.stage;
                out.error_message = "polling /capture/progress failed: " + progRep.error;
                recoveryLog(std::string("[capture-http] progress FAIL: ") + out.error_message);
                return false;
            }
            continue;
        }
        consecutiveSshFails = 0;
        lastProgressBody = progRep.body;

        const std::string phaseName = jsonString(progRep.body, "phase");
        const uint64_t phaseCode    = jsonNumber(progRep.body, "phase_code");
        const uint64_t samples      = jsonNumber(progRep.body, "samples");
        const uint64_t bytesRx      = jsonNumber(progRep.body, "bytes_received");
        const uint64_t totalBytes   = jsonNumber(progRep.body, "total_bytes");
        const uint64_t elapsed      = jsonNumber(progRep.body, "elapsed_ms");
        const bool     doneFlag     = jsonBool(progRep.body, "done");

        captureHttpPhaseCode_.store(static_cast<uint8_t>(phaseCode),
                                    std::memory_order_release);
        captureHttpSamples_.store(static_cast<uint16_t>(samples),
                                  std::memory_order_relaxed);
        captureHttpBytesRx_.store(bytesRx, std::memory_order_relaxed);
        captureHttpBytesExpected_.store(totalBytes, std::memory_order_relaxed);
        captureHttpElapsedMs_.store(static_cast<uint32_t>(elapsed),
                                    std::memory_order_relaxed);

        if (doneFlag) {
            gotDone = true;
            // phase_code == 6 means PHASE_ERROR — pick up the device-side
            // error message early so the /capture/result path doesn't
            // need to refetch it.
            if (phaseCode == 6 /*PHASE_ERROR*/) {
                const std::string err = jsonString(progRep.body, "error");
                if (!err.empty()) {
                    out.error_stage = "remote";
                    out.error_message = err;
                }
            }
            (void) phaseName;  // logged below
            break;
        }
    }

    if (!gotDone) {
        out.error_stage = "progress_timeout";
        out.error_message = "polling /capture/progress did not see phase=done within 240 s";
        if (!lastProgressBody.empty()) {
            out.error_message += "; last=" + tailText(lastProgressBody, 200);
        }
        recoveryLog(std::string("[capture-http] FAIL ") + out.error_message);
        return false;
    }

    // ---- Phase 3: /capture/result ---------------------------------------
    recoveryLog(std::string("[capture-http] result fetch session_id=") +
                std::to_string(sessionId));
    const auto tStage3 = std::chrono::steady_clock::now();
    HttpReply resRep = httpGet(std::string("/capture/result") + sidQuery,
                               /*wgetTimeoutSec*/ 50,
                               /*timeoutMs*/ 60000,
                               /*stallMs*/   45000);
    out.stage3_ms = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - tStage3).count());
    if (!resRep.ok) {
        out.error_stage = resRep.stage.empty() ? "result" : resRep.stage;
        out.error_message = "fetching /capture/result failed: " + resRep.error;
        recoveryLog(std::string("[capture-http] result FAIL: ") + out.error_message);
        return false;
    }
    const std::string body = resRep.body;
    out.http_status = 200;

    if (!jsonBool(body, "ok")) {
        const std::string err = jsonString(body, "error");
        out.error_stage = "remote";
        out.error_message = err.empty()
            ? std::string("ok=false (no error field); body=") + tailText(body, 240)
            : err;
        return false;
    }

    bool haveSamples = false, haveSlots = false, haveTotal = false, haveCrc = false;
    out.samples      = static_cast<uint16_t>(jsonNumber(body, "samples", &haveSamples));
    out.slot_count   = static_cast<uint8_t>(jsonNumber(body, "slots", &haveSlots));
    const uint64_t totalBytes = jsonNumber(body, "total_bytes", &haveTotal);
    out.period_us    = static_cast<uint16_t>(jsonNumber(body, "period_us"));
    out.elapsed_ms   = static_cast<uint32_t>(jsonNumber(body, "ms"));
    out.fill_ms      = static_cast<uint32_t>(jsonNumber(body, "fill_ms"));
    out.read_ms      = static_cast<uint32_t>(jsonNumber(body, "read_ms"));
    out.crc32        = static_cast<uint32_t>(jsonNumber(body, "crc", &haveCrc));
    out.trigger_idx  = 0;  // /capture has no trigger semantics — Immediate only
    // Server echoes the slot count it was ASKED for in "requested_slots"
    // (newer app_dd; older deploys won't have it). Cross-check it
    // against the actual slot_count we constructed in the query so a
    // misrouted body (wrong session, server-side bug) gets flagged
    // instead of silently producing garbled samples.
    bool haveReqSlots = false;
    const uint64_t requestedSlots = jsonNumber(body, "requested_slots", &haveReqSlots);

    if (!haveSamples || !haveSlots || !haveTotal) {
        out.error_stage = "json_field";
        out.error_message = "/capture JSON missing samples/slots/total_bytes; body=" +
                            tailText(body, 200);
        return false;
    }
    if (out.samples == 0u || out.slot_count == 0u || totalBytes == 0u) {
        out.error_stage = "empty";
        out.error_message = "/capture returned 0 samples (samples=" +
                            std::to_string(out.samples) + ", slots=" +
                            std::to_string(static_cast<int>(out.slot_count)) +
                            ", total_bytes=" + std::to_string(totalBytes) + ")";
        return false;
    }
    out.bytes_per_sample = static_cast<uint32_t>(totalBytes / out.samples);
    if (out.bytes_per_sample == 0u) {
        out.error_stage = "math";
        out.error_message = "total_bytes < samples (samples=" + std::to_string(out.samples) +
                            ", total_bytes=" + std::to_string(totalBytes) + ")";
        return false;
    }
    // Count slotN= occurrences in our own query so we can verify the
    // server actually captured every slot we asked for. The capture
    // stride code downstream assumes slot_count matches the configured
    // watchIdxBySlot size.
    size_t expectSlotsInQuery = 0;
    for (size_t i = 0; i < kCapMaxSlots; ++i) {
        const std::string needle = "slot" + std::to_string(i) + "=";
        if (query_string.find(needle) != std::string::npos) ++expectSlotsInQuery;
    }
    if (expectSlotsInQuery > 0 && out.slot_count != expectSlotsInQuery) {
        out.error_stage = "slot_count";
        out.error_message = "slot count mismatch: requested " +
                            std::to_string(expectSlotsInQuery) +
                            ", server returned " +
                            std::to_string(static_cast<int>(out.slot_count));
        return false;
    }
    if (haveReqSlots && requestedSlots != expectSlotsInQuery) {
        out.error_stage = "slot_count_echo";
        out.error_message = "server echoed requested_slots=" +
                            std::to_string(requestedSlots) +
                            " but we sent " +
                            std::to_string(expectSlotsInQuery);
        return false;
    }

    const std::string b64 = jsonString(body, "data_base64");
    if (b64.empty()) {
        out.error_stage = "json_field";
        out.error_message = "/capture JSON missing data_base64; body=" + tailText(body, 200);
        return false;
    }
    if (!base64Decode(b64, out.bytes)) {
        out.error_stage = "base64";
        out.error_message = "base64 decode failed (" + std::to_string(b64.size()) + " chars)";
        return false;
    }
    if (out.bytes.size() != totalBytes) {
        out.error_stage = "body_short";
        out.error_message = "decoded body size mismatch: got " +
                            std::to_string(out.bytes.size()) +
                            " B, expected " + std::to_string(totalBytes) + " B";
        return false;
    }

    // CRC-verify locally — the JSON `crc` is what the MCU reported AND
    // what app_dd's CapClient cross-checked after assembly. Both that
    // and our base64 round-trip can independently corrupt bytes; an
    // explicit local check turns silent bit-flips into a clear FAIL
    // instead of garbled samples in the plot.
    const uint32_t localCrc = captureCrc32(out.bytes.data(), out.bytes.size());
    if (haveCrc && localCrc != out.crc32) {
        char hex[40];
        std::snprintf(hex, sizeof(hex), "0x%08X vs remote 0x%08X",
                      static_cast<unsigned>(localCrc),
                      static_cast<unsigned>(out.crc32));
        out.error_stage = "crc";
        out.error_message = std::string("local CRC ") + hex;
        return false;
    }

    {
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "[capture-http] OK samples=%u slots=%u bytes=%u "
                      "crc=0x%08X period_us=%u ms=%u",
                      static_cast<unsigned>(out.samples),
                      static_cast<unsigned>(out.slot_count),
                      static_cast<unsigned>(out.bytes.size()),
                      static_cast<unsigned>(out.crc32),
                      static_cast<unsigned>(out.period_us),
                      static_cast<unsigned>(out.elapsed_ms));
        recoveryLog(buf);
    }

    out.ok = true;
    return true;
}

bool MainUi::launchCaptureHttpWorker()
{
    if (captureHttpRunning_.load(std::memory_order_acquire)) {
        capture_.lastError = "HTTP capture worker already running";
        return false;
    }
    // Join any previous thread before re-launching (the previous run set
    // captureHttpDone_=true and was drained, but the thread object hasn't
    // been join()'d yet).
    if (captureHttpThread_.joinable()) {
        captureHttpThread_.join();
    }

    // Resolve the host from the active transport.
    std::string host;
    if (transportKind_ == TransportKind::Tcp) {
        host = tcpHost_.data();
    } else if (transportKind_ == TransportKind::WifiAp) {
        host = wifiApHost_.data();
    } else {
        capture_.lastError = "HTTP path requires TCP/WiFi-AP transport";
        return false;
    }

    const std::string query = buildCaptureHttpQuery();

    // Reset live status counters BEFORE the worker writes to them.
    captureHttpBytesRx_.store(0, std::memory_order_relaxed);
    captureHttpBytesExpected_.store(0, std::memory_order_relaxed);
    captureHttpStartedAt_ = clock_.nowSeconds();
    {
        std::lock_guard<std::mutex> lk(captureHttpMutex_);
        captureHttpResult_ = CaptureHttpResult{};
    }
    captureHttpDone_.store(false, std::memory_order_release);
    captureHttpRunning_.store(true, std::memory_order_release);

    captureHttpThread_ = std::thread([this, host, query]() {
        CaptureHttpResult r;
        (void) fetchCaptureViaHttp(host, query, r);
        {
            std::lock_guard<std::mutex> lk(captureHttpMutex_);
            captureHttpResult_ = std::move(r);
        }
        captureHttpDone_.store(true, std::memory_order_release);
        captureHttpRunning_.store(false, std::memory_order_release);
    });
    return true;
}

bool MainUi::drainCaptureHttpResult(double now)
{
    if (!captureHttpDone_.load(std::memory_order_acquire)) {
        // Still running -- mirror the polled progress fields (fed by
        // /capture/progress over SSH) into capture_.status so the
        // toolbar text + tooltip move forward. Worker writes through
        // atomics; we just format cheaply here.
        if (captureHttpRunning_.load(std::memory_order_acquire)) {
            const uint8_t phaseCode = captureHttpPhaseCode_.load(std::memory_order_acquire);
            const uint64_t rx       = captureHttpBytesRx_.load(std::memory_order_relaxed);
            const uint64_t exp      = captureHttpBytesExpected_.load(std::memory_order_relaxed);
            const uint16_t samples  = captureHttpSamples_.load(std::memory_order_relaxed);
            const double   secs     = now - captureHttpStartedAt_;
            // Phase code matches CapClient::Progress::Phase:
            //   0=idle 1=setup 2=armed 3=sampling 4=transferring 5=done 6=error
            const char* phaseName =
                (phaseCode == 1) ? "setup" :
                (phaseCode == 2) ? "armed" :
                (phaseCode == 3) ? "sampling" :
                (phaseCode == 4) ? "transferring" :
                (phaseCode == 5) ? "done" :
                (phaseCode == 6) ? "error" :
                                   "starting";
            char buf[128];
            if (phaseCode == 4 /*transferring*/ && exp > 0) {
                // Bytes-received / total is the most precise progress
                // signal once READ_CHUNK has started shipping the ring
                // buffer back to the SoM.
                std::snprintf(buf, sizeof(buf),
                              "%s: %llu / %llu B (%.1fs)",
                              phaseName,
                              static_cast<unsigned long long>(rx),
                              static_cast<unsigned long long>(exp),
                              secs);
            } else if (phaseCode == 3 /*sampling*/) {
                // While the motor is still filling its ring buffer, only
                // the sample count moves. Show it as "sampling N" rather
                // than 0-byte transfer.
                std::snprintf(buf, sizeof(buf),
                              "%s: %u samples (%.1fs)",
                              phaseName,
                              static_cast<unsigned>(samples), secs);
            } else {
                std::snprintf(buf, sizeof(buf),
                              "%s (%.1fs)", phaseName, secs);
            }
            capture_.status = buf;
        }
        return false;
    }

    // Done -- join thread, take result snapshot, advance capture phase.
    if (captureHttpThread_.joinable()) {
        captureHttpThread_.join();
    }
    CaptureHttpResult r;
    {
        std::lock_guard<std::mutex> lk(captureHttpMutex_);
        r = std::move(captureHttpResult_);
        captureHttpResult_ = CaptureHttpResult{};
    }
    captureHttpDone_.store(false, std::memory_order_release);

    if (!r.ok) {
        capture_.lastError = "HTTP capture failed (" + r.error_stage + "): " +
                             r.error_message;
        capture_.status = "error: " + capture_.lastError;
        capture_.cooldownUntil = now + 1.0;
        capture_.phase = CaptureSessionPhase::CooldownDone;
        recoveryLog(std::string("[capture-http] FAIL ") + r.error_stage +
                    ": " + r.error_message);
        return true;
    }

    // Splice the HTTP result into the capture_ struct so the existing
    // applyCapturedSamples pipeline can run unchanged.
    capture_.samplesCaptured = r.samples;
    capture_.triggerSampleIdx = r.trigger_idx;
    capture_.bytesPerSample = static_cast<uint16_t>(r.bytes_per_sample);
    capture_.bytesExpected = static_cast<uint16_t>(
        static_cast<uint32_t>(r.samples) * r.bytes_per_sample);
    capture_.rawBytes = std::move(r.bytes);
    capture_.byteSeen.assign(capture_.bytesExpected, true);

    // If the endpoint returned a tweaked period_us (it can clamp to device-
    // side bounds), pick that up so applyCapturedSamples uses the right dt.
    if (r.period_us > 0) capture_.periodUs = r.period_us;

    // Anchor the captured window to "(now) - elapsed_ms", which is when the
    // device finished sampling. The HTTP body itself adds a few ms of TCP
    // RTT on top, but that's an order of magnitude smaller than the burst
    // and not worth correcting for.
    capture_.doneStatusAt = now - static_cast<double>(r.elapsed_ms) * 1e-3;

    // fetchCaptureViaHttp already verified local CRC == remote CRC, but
    // record both so the toolbar can show "CRC 0xABCD ok" instead of
    // leaving them at the previous run's values.
    capture_.lastCrcLocal = captureCrc32(capture_.rawBytes.data(),
                                         capture_.rawBytes.size());
    capture_.lastCrcRemote = r.crc32;

    // Per-stage throughput summary on the capture toolbar/tooltip:
    //   1 capture (MCU ring fill)  2 motor->SoM (CAN read)  3 SoM->pc_tool (HTTP)
    {
        const double tb = static_cast<double>(capture_.bytesExpected);
        auto kbps = [](double bytes, uint32_t ms) -> double {
            return (ms > 0u) ? (bytes / 1024.0) / (static_cast<double>(ms) / 1000.0) : 0.0;
        };
        char sb[224];
        std::snprintf(sb, sizeof(sb),
            "done: %u B / %u samples  |  1.cap %.1f KB/s (%ums)  "
            "2.mot->som %.1f KB/s (%ums)  3.som->pc %.1f KB/s (%ums)",
            static_cast<unsigned>(capture_.bytesExpected),
            static_cast<unsigned>(r.samples),
            kbps(tb, r.fill_ms),   static_cast<unsigned>(r.fill_ms),
            kbps(tb, r.read_ms),   static_cast<unsigned>(r.read_ms),
            kbps(tb, r.stage3_ms), static_cast<unsigned>(r.stage3_ms));
        capture_.status = sb;
    }

    // Apply samples directly. The non-HTTP path goes through Apply -> CooldownDone
    // via pollCaptureSession; here we do the same in one step.
    applyCapturedSamples();
    capture_.cooldownUntil = now + 0.3;
    capture_.phase = CaptureSessionPhase::CooldownDone;
    return true;
}

// ============================================================================
// HTTP-driven motor-config blob get/set. Mirrors the /capture path: when
// transport is TCP / WifiAp, hand the operation to app_dd's
// /motor/config/{get,set} so the full file-transfer state machine runs
// locally over CAN; pc_tool just consumes one JSON response.
// ============================================================================

bool MainUi::motorBlobUsesHttpPath() const
{
    return transportKind_ == TransportKind::Tcp ||
           transportKind_ == TransportKind::WifiAp;
}

void MainUi::applyMotorBlobReceived(uint16_t bytes, uint32_t crc,
                                    const char* source)
{
    motorBlobValid_ = true;
    motorBlobReceivedAt_ = ImGui::GetTime();

    float thetaOff = 0.0f;
    std::memcpy(&thetaOff,
                &motorBlob_[kMotorBlobOffThetaOffset],
                sizeof(thetaOff));
    lastMotorTelemetry_.theta_valid = true;
    lastMotorTelemetry_.theta_offset_rad = thetaOff;
    lastMotorTelemetry_.theta_receivedAt = ImGui::GetTime();
    if (!motorThetaManualDirty_) motorThetaManualRad_ = thetaOff;
    if (!motorThetaSlotDirty_[0]) motorThetaSlotEdit_[0] = thetaOff;
    for (uint8_t s = 0; s < 6u; ++s) {
        float v = 0.0f;
        std::memcpy(&v,
                    &motorBlob_[kMotorBlobOffThetaSectorLut + s * 4u],
                    sizeof(v));
        lastMotorTelemetry_.theta_sector_valid[s] = true;
        lastMotorTelemetry_.theta_sector_lut_rad[s] = v;
        lastMotorTelemetry_.theta_sector_receivedAt[s] = ImGui::GetTime();
        if (!motorThetaSlotDirty_[s + 1u]) {
            motorThetaSlotEdit_[s + 1u] = v;
        }
    }
    populateMotorConfigCacheFromBlob();
    char buf[128] = {};
    std::snprintf(buf, sizeof(buf),
                  "%s ok (%u B, CRC 0x%08X)",
                  source ? source : "blob",
                  static_cast<unsigned>(bytes),
                  static_cast<unsigned>(crc));
    motorThetaTxStatus_ = buf;
}

bool MainUi::fetchMotorBlobGetHttp(const std::string& host,
                                   MotorBlobHttpResult& out,
                                   const char* endpoint,
                                   int tmoSec,
                                   MotorBlobHttpKind kind)
{
    out = MotorBlobHttpResult{};
    out.kind = kind;

    if (host.empty()) {
        out.error_stage = "host";
        out.error_message = "no host (transport not TCP/AP?)";
        return false;
    }

    constexpr const char* kPassword = "bork2025";
    const std::string discoveredFp = discoveredHostKeyFp(host);
    const std::string sexec = recoveryBitviseSexec();
    const std::string plink = recoveryPlinkExe();
    const std::string url = std::string("http://127.0.0.2:10011") + endpoint;
    const std::string tmo = std::to_string(tmoSec);

    const std::string remoteCmd =
        std::string("if command -v wget >/dev/null 2>&1; then ")
        + "exec wget -q -T " + tmo + " -O- '" + url + "'; "
        + "fi; if command -v curl >/dev/null 2>&1; then "
        + "exec curl -sS --max-time " + tmo + " '" + url + "'; "
        + "fi; echo 'no wget or curl on device' >&2; exit 97";

    std::string cmd;
    if (!sexec.empty()) {
        cmd = quoteArg(sexec) +
              " " + quoteArg(std::string("-host=") + host) +
              " -user=root" +
              " " + quoteArg(std::string("-pw=") + kPassword) +
              " -unat=y -exitZero" +
              (discoveredFp.empty()
                   ? std::string()
                   : std::string(" -hostKeyFp=") + sha256FpForBitvise(discoveredFp)) +
              " " + quoteArg(std::string("-cmd=") + remoteCmd);
    } else if (!plink.empty()) {
        cmd = quoteArg(plink) +
              " -batch -ssh" +
              (discoveredFp.empty()
                   ? std::string()
                   : std::string(" -hostkey ") + quoteArg(discoveredFp)) +
              " -pw " + quoteArg(kPassword) +
              " root@" + host + " " + quoteArg(remoteCmd);
    } else {
        out.error_stage = "ssh";
        out.error_message = "no SSH client (sexec/plink) found";
        return false;
    }

    recoveryLog(std::string("[motor-config-http] GET ") + url);

    /* Host-side caps scale with the remote wget timeout so /reset
     * (RESET+SAVE+read ~ up to 4 s server-side + SSH RTT) doesn't get
     * killed early when it reuses this reader with a larger tmoSec. */
    const int hardMs = (tmoSec + 4) * 1000;
    const int softMs = (tmoSec + 1) * 1000;
    const CommandResult result = runCommandHidden(cmd, {}, hardMs, softMs);
    const std::string& output = result.output;
    if (result.rc != 0 && output.empty()) {
        out.error_stage = "ssh";
        out.error_message = "SSH command exited rc=" + std::to_string(result.rc);
        if (!result.output.empty()) out.error_message += ": " + tailText(result.output, 240);
        recoveryLog(std::string("[motor-config-http] GET FAIL: ") + out.error_message);
        return false;
    }

    const size_t jbeg = output.find('{');
    const size_t jend = output.rfind('}');
    if (jbeg == std::string::npos || jend == std::string::npos || jend < jbeg) {
        out.error_stage = "json_parse";
        out.error_message = "no JSON in /motor/config/get response (got " +
                            std::to_string(output.size()) + " B)";
        if (!output.empty()) out.error_message += "; head=" + tailText(output, 200);
        recoveryLog(std::string("[motor-config-http] GET FAIL: ") + out.error_message);
        return false;
    }
    const std::string body = output.substr(jbeg, jend - jbeg + 1);

    if (!jsonBool(body, "ok")) {
        const std::string err = jsonString(body, "error");
        out.error_stage = "remote";
        out.error_message = err.empty() ? std::string("ok=false: ") + tailText(body, 200) : err;
        recoveryLog(std::string("[motor-config-http] GET FAIL: ") + out.error_message);
        return false;
    }
    out.bytes      = static_cast<uint16_t>(jsonNumber(body, "bytes"));
    out.crc        = static_cast<uint32_t>(jsonNumber(body, "crc"));
    out.elapsed_ms = static_cast<uint32_t>(jsonNumber(body, "ms"));

    if (out.bytes != 164u) {
        out.error_stage = "size";
        out.error_message = "bytes=" + std::to_string(out.bytes) + " != 164";
        recoveryLog(std::string("[motor-config-http] GET FAIL: ") + out.error_message);
        return false;
    }
    const std::string b64 = jsonString(body, "data_base64");
    if (b64.empty()) {
        out.error_stage = "json_field";
        out.error_message = "no data_base64 in body";
        recoveryLog(std::string("[motor-config-http] GET FAIL: ") + out.error_message);
        return false;
    }
    if (!base64Decode(b64, out.blob) || out.blob.size() != 164u) {
        out.error_stage = "base64";
        out.error_message = "decode failed (got " + std::to_string(out.blob.size()) + " B)";
        recoveryLog(std::string("[motor-config-http] GET FAIL: ") + out.error_message);
        return false;
    }
    // Independent CRC verification — SoM already cross-checked against
    // the motor's reported CRC, but base64 round-trip is its own risk.
    const uint32_t localCrc = motorStm32HwCrc32(out.blob.data(), out.blob.size());
    if (localCrc != out.crc) {
        out.error_stage = "crc";
        char hex[64];
        std::snprintf(hex, sizeof(hex), "local 0x%08X vs SoM 0x%08X",
                      static_cast<unsigned>(localCrc),
                      static_cast<unsigned>(out.crc));
        out.error_message = hex;
        recoveryLog(std::string("[motor-config-http] GET FAIL: ") + out.error_message);
        return false;
    }

    {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "[motor-config-http] GET OK bytes=%u crc=0x%08X ms=%u",
                      static_cast<unsigned>(out.bytes),
                      static_cast<unsigned>(out.crc),
                      static_cast<unsigned>(out.elapsed_ms));
        recoveryLog(buf);
    }
    out.ok = true;
    return true;
}

namespace {
/* Base64-encode raw bytes for /motor/config/set?data_base64=... .
 *
 * Uses the URL-safe RFC 4648 §5 alphabet ('-' for '+', '_' for '/')
 * and OMITS '=' padding, so the output survives a GET query unmodified:
 *   - standard '+' would be decoded to space by URL-decoders that
 *     follow application/x-www-form-urlencoded (busybox/HTTP servers
 *     differ);
 *   - '/' is path-significant in some routers / parsers; '_' is not;
 *   - '=' confuses naive `k=v` splitters in middleboxes.
 * The SoM-side decoder in handle_motor_config_set accepts both
 * alphabets, so older /motor/config/set callers using stock base64
 * keep working.
 */
std::string b64Encode(const uint8_t* data, size_t len)
{
    static const char abc[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
    std::string out;
    out.reserve(((len + 2) / 3) * 4);
    size_t i = 0;
    while (i + 2 < len) {
        const uint32_t v = (uint32_t(data[i]) << 16) |
                           (uint32_t(data[i + 1]) << 8) |
                            uint32_t(data[i + 2]);
        out.push_back(abc[(v >> 18) & 0x3Fu]);
        out.push_back(abc[(v >> 12) & 0x3Fu]);
        out.push_back(abc[(v >> 6)  & 0x3Fu]);
        out.push_back(abc[v         & 0x3Fu]);
        i += 3;
    }
    if (i < len) {
        const uint32_t b0 = data[i];
        const uint32_t b1 = (i + 1 < len) ? data[i + 1] : 0u;
        const uint32_t v  = (b0 << 16) | (b1 << 8);
        out.push_back(abc[(v >> 18) & 0x3Fu]);
        out.push_back(abc[(v >> 12) & 0x3Fu]);
        if (i + 1 < len) out.push_back(abc[(v >> 6) & 0x3Fu]);
        // no '=' padding for URL-safe variant
    }
    return out;
}
}  // namespace

bool MainUi::fetchMotorBlobSetHttp(const std::string& host,
                                   const uint8_t blob[164],
                                   uint32_t crc,
                                   MotorBlobHttpResult& out)
{
    out = MotorBlobHttpResult{};
    out.kind = MotorBlobHttpKind::Set;

    if (host.empty()) {
        out.error_stage = "host";
        out.error_message = "no host (transport not TCP/AP?)";
        return false;
    }

    constexpr const char* kPassword = "bork2025";
    const std::string discoveredFp = discoveredHostKeyFp(host);
    const std::string sexec = recoveryBitviseSexec();
    const std::string plink = recoveryPlinkExe();

    const std::string b64 = b64Encode(blob, 164);
    char crcBuf[16];
    std::snprintf(crcBuf, sizeof(crcBuf), "%u", static_cast<unsigned>(crc));
    const std::string url =
        std::string("http://127.0.0.2:10011/motor/config/set?data_base64=") +
        b64 + "&crc=" + crcBuf;

    const std::string remoteCmd =
        std::string("if command -v wget >/dev/null 2>&1; then ")
        + "exec wget -q -T 10 -O- '" + url + "'; "
        + "fi; if command -v curl >/dev/null 2>&1; then "
        + "exec curl -sS --max-time 10 '" + url + "'; "
        + "fi; echo 'no wget or curl on device' >&2; exit 97";

    std::string cmd;
    if (!sexec.empty()) {
        cmd = quoteArg(sexec) +
              " " + quoteArg(std::string("-host=") + host) +
              " -user=root" +
              " " + quoteArg(std::string("-pw=") + kPassword) +
              " -unat=y -exitZero" +
              (discoveredFp.empty()
                   ? std::string()
                   : std::string(" -hostKeyFp=") + sha256FpForBitvise(discoveredFp)) +
              " " + quoteArg(std::string("-cmd=") + remoteCmd);
    } else if (!plink.empty()) {
        cmd = quoteArg(plink) +
              " -batch -ssh" +
              (discoveredFp.empty()
                   ? std::string()
                   : std::string(" -hostkey ") + quoteArg(discoveredFp)) +
              " -pw " + quoteArg(kPassword) +
              " root@" + host + " " + quoteArg(remoteCmd);
    } else {
        out.error_stage = "ssh";
        out.error_message = "no SSH client (sexec/plink) found";
        return false;
    }

    {
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "[motor-config-http] SET bytes=164 crc=0x%08X b64len=%zu (url-safe)",
                      static_cast<unsigned>(crc), b64.size());
        recoveryLog(buf);
    }

    const CommandResult result = runCommandHidden(cmd, {}, 14000, 11000);
    const std::string& output = result.output;
    if (result.rc != 0 && output.empty()) {
        out.error_stage = "ssh";
        out.error_message = "SSH command exited rc=" + std::to_string(result.rc);
        if (!result.output.empty()) out.error_message += ": " + tailText(result.output, 240);
        recoveryLog(std::string("[motor-config-http] SET FAIL: ") + out.error_message);
        return false;
    }

    const size_t jbeg = output.find('{');
    const size_t jend = output.rfind('}');
    if (jbeg == std::string::npos || jend == std::string::npos || jend < jbeg) {
        out.error_stage = "json_parse";
        out.error_message = "no JSON in /motor/config/set response";
        if (!output.empty()) out.error_message += "; head=" + tailText(output, 200);
        recoveryLog(std::string("[motor-config-http] SET FAIL: ") + out.error_message);
        return false;
    }
    const std::string body = output.substr(jbeg, jend - jbeg + 1);
    if (!jsonBool(body, "ok")) {
        const std::string err = jsonString(body, "error");
        out.error_stage = "remote";
        out.error_message = err.empty() ? std::string("ok=false: ") + tailText(body, 200) : err;
        recoveryLog(std::string("[motor-config-http] SET FAIL: ") + out.error_message);
        return false;
    }
    out.bytes      = static_cast<uint16_t>(jsonNumber(body, "bytes"));
    out.crc        = static_cast<uint32_t>(jsonNumber(body, "crc"));
    out.elapsed_ms = static_cast<uint32_t>(jsonNumber(body, "ms"));
    {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "[motor-config-http] SET OK bytes=%u crc=0x%08X ms=%u",
                      static_cast<unsigned>(out.bytes),
                      static_cast<unsigned>(out.crc),
                      static_cast<unsigned>(out.elapsed_ms));
        recoveryLog(buf);
    }
    out.ok = true;
    return true;
}

bool MainUi::launchMotorBlobGetHttp()
{
    if (motorBlobHttpRunning_.load(std::memory_order_acquire)) {
        // Auto-poll cadence is 1.5 s; a stalled worker isn't worth
        // queueing a second behind. Just skip this tick.
        return false;
    }
    if (motorBlobHttpThread_.joinable()) motorBlobHttpThread_.join();

    std::string host;
    if (transportKind_ == TransportKind::Tcp)        host = tcpHost_.data();
    else if (transportKind_ == TransportKind::WifiAp) host = wifiApHost_.data();
    else return false;

    {
        std::lock_guard<std::mutex> lk(motorBlobHttpMutex_);
        motorBlobHttpResult_ = MotorBlobHttpResult{};
    }
    motorBlobHttpStartedAt_ = clock_.nowSeconds();
    motorBlobHttpDone_.store(false, std::memory_order_release);
    motorBlobHttpRunning_.store(true, std::memory_order_release);

    motorBlobHttpThread_ = std::thread([this, host]() {
        MotorBlobHttpResult r;
        (void) fetchMotorBlobGetHttp(host, r);
        {
            std::lock_guard<std::mutex> lk(motorBlobHttpMutex_);
            motorBlobHttpResult_ = std::move(r);
        }
        motorBlobHttpDone_.store(true, std::memory_order_release);
        motorBlobHttpRunning_.store(false, std::memory_order_release);
    });
    return true;
}

bool MainUi::launchMotorBlobSetHttp(const uint8_t blob[164])
{
    if (motorBlobHttpRunning_.load(std::memory_order_acquire)) {
        motorThetaTxStatus_ = "blob op already in flight";
        return false;
    }
    if (motorBlobHttpThread_.joinable()) motorBlobHttpThread_.join();

    std::string host;
    if (transportKind_ == TransportKind::Tcp)        host = tcpHost_.data();
    else if (transportKind_ == TransportKind::WifiAp) host = wifiApHost_.data();
    else return false;

    // Compute CRC up front so the SoM gets a sanity-check value and we
    // can stamp it into status on completion.
    const uint32_t crc = motorStm32HwCrc32(blob, 164);
    std::vector<uint8_t> bytes(blob, blob + 164);

    {
        std::lock_guard<std::mutex> lk(motorBlobHttpMutex_);
        motorBlobHttpResult_ = MotorBlobHttpResult{};
    }
    motorBlobHttpStartedAt_ = clock_.nowSeconds();
    motorBlobHttpDone_.store(false, std::memory_order_release);
    motorBlobHttpRunning_.store(true, std::memory_order_release);

    motorBlobHttpThread_ = std::thread([this, host, bytes = std::move(bytes), crc]() {
        MotorBlobHttpResult r;
        (void) fetchMotorBlobSetHttp(host, bytes.data(), crc, r);
        {
            std::lock_guard<std::mutex> lk(motorBlobHttpMutex_);
            motorBlobHttpResult_ = std::move(r);
        }
        motorBlobHttpDone_.store(true, std::memory_order_release);
        motorBlobHttpRunning_.store(false, std::memory_order_release);
    });
    return true;
}

bool MainUi::fetchMotorBlobSaveHttp(const std::string& host,
                                    MotorBlobHttpResult& out)
{
    out = MotorBlobHttpResult{};
    out.kind = MotorBlobHttpKind::Save;

    if (host.empty()) {
        out.error_stage = "host";
        out.error_message = "no host (transport not TCP/AP?)";
        return false;
    }

    constexpr const char* kPassword = "bork2025";
    const std::string discoveredFp = discoveredHostKeyFp(host);
    const std::string sexec = recoveryBitviseSexec();
    const std::string plink = recoveryPlinkExe();
    const std::string url = "http://127.0.0.2:10011/motor/config/save";

    // SAVE_ALL blocks the motor for ~1.5 s during the flash erase, plus
    // CAN response time and SSH RTT. 12 s host-side timeout gives a
    // comfortable margin (server-side caps the saveAll() call at 3 s).
    const std::string remoteCmd =
        std::string("if command -v wget >/dev/null 2>&1; then ")
        + "exec wget -q -T 10 -O- '" + url + "'; "
        + "fi; if command -v curl >/dev/null 2>&1; then "
        + "exec curl -sS --max-time 10 '" + url + "'; "
        + "fi; echo 'no wget or curl on device' >&2; exit 97";

    std::string cmd;
    if (!sexec.empty()) {
        cmd = quoteArg(sexec) +
              " " + quoteArg(std::string("-host=") + host) +
              " -user=root" +
              " " + quoteArg(std::string("-pw=") + kPassword) +
              " -unat=y -exitZero" +
              (discoveredFp.empty()
                   ? std::string()
                   : std::string(" -hostKeyFp=") + sha256FpForBitvise(discoveredFp)) +
              " " + quoteArg(std::string("-cmd=") + remoteCmd);
    } else if (!plink.empty()) {
        cmd = quoteArg(plink) +
              " -batch -ssh" +
              (discoveredFp.empty()
                   ? std::string()
                   : std::string(" -hostkey ") + quoteArg(discoveredFp)) +
              " -pw " + quoteArg(kPassword) +
              " root@" + host + " " + quoteArg(remoteCmd);
    } else {
        out.error_stage = "ssh";
        out.error_message = "no SSH client (sexec/plink) found";
        return false;
    }

    recoveryLog(std::string("[motor-config-http] SAVE ") + url);

    const CommandResult result = runCommandHidden(cmd, {}, 14000, 12000);
    const std::string& output = result.output;
    if (result.rc != 0 && output.empty()) {
        out.error_stage = "ssh";
        out.error_message = "SSH command exited rc=" + std::to_string(result.rc);
        if (!result.output.empty()) out.error_message += ": " + tailText(result.output, 240);
        recoveryLog(std::string("[motor-config-http] SAVE FAIL: ") + out.error_message);
        return false;
    }

    const size_t jbeg = output.find('{');
    const size_t jend = output.rfind('}');
    if (jbeg == std::string::npos || jend == std::string::npos || jend < jbeg) {
        out.error_stage = "json_parse";
        out.error_message = "no JSON in /motor/config/save response";
        if (!output.empty()) out.error_message += "; head=" + tailText(output, 200);
        recoveryLog(std::string("[motor-config-http] SAVE FAIL: ") + out.error_message);
        return false;
    }
    const std::string body = output.substr(jbeg, jend - jbeg + 1);
    if (!jsonBool(body, "ok")) {
        const std::string err = jsonString(body, "error");
        out.error_stage = "remote";
        out.error_message = err.empty() ? std::string("ok=false: ") + tailText(body, 200) : err;
        recoveryLog(std::string("[motor-config-http] SAVE FAIL: ") + out.error_message);
        return false;
    }
    out.elapsed_ms = static_cast<uint32_t>(jsonNumber(body, "ms"));
    {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "[motor-config-http] SAVE OK ms=%u",
                      static_cast<unsigned>(out.elapsed_ms));
        recoveryLog(buf);
    }
    out.ok = true;
    return true;
}

bool MainUi::launchMotorBlobSaveHttp()
{
    if (motorBlobHttpRunning_.load(std::memory_order_acquire)) {
        motorThetaTxStatus_ = "blob op already in flight";
        return false;
    }
    if (motorBlobHttpThread_.joinable()) motorBlobHttpThread_.join();

    std::string host;
    if (transportKind_ == TransportKind::Tcp)        host = tcpHost_.data();
    else if (transportKind_ == TransportKind::WifiAp) host = wifiApHost_.data();
    else return false;

    {
        std::lock_guard<std::mutex> lk(motorBlobHttpMutex_);
        motorBlobHttpResult_ = MotorBlobHttpResult{};
    }
    motorBlobHttpStartedAt_ = clock_.nowSeconds();
    motorBlobHttpDone_.store(false, std::memory_order_release);
    motorBlobHttpRunning_.store(true, std::memory_order_release);

    motorBlobHttpThread_ = std::thread([this, host]() {
        MotorBlobHttpResult r;
        (void) fetchMotorBlobSaveHttp(host, r);
        {
            std::lock_guard<std::mutex> lk(motorBlobHttpMutex_);
            motorBlobHttpResult_ = std::move(r);
        }
        motorBlobHttpDone_.store(true, std::memory_order_release);
        motorBlobHttpRunning_.store(false, std::memory_order_release);
    });
    return true;
}

bool MainUi::drainMotorBlobHttpResult(double now)
{
    if (!motorBlobHttpDone_.load(std::memory_order_acquire)) return false;
    if (motorBlobHttpThread_.joinable()) motorBlobHttpThread_.join();

    MotorBlobHttpResult r;
    {
        std::lock_guard<std::mutex> lk(motorBlobHttpMutex_);
        r = std::move(motorBlobHttpResult_);
        motorBlobHttpResult_ = MotorBlobHttpResult{};
    }
    motorBlobHttpDone_.store(false, std::memory_order_release);

    const double elapsed = now - motorBlobHttpStartedAt_;

    auto kindStr = [&]() -> const char* {
        switch (r.kind) {
            case MotorBlobHttpKind::Get:  return "blob GET";
            case MotorBlobHttpKind::Set:  return "blob SET";
            case MotorBlobHttpKind::Save: return "PARAM SAVE_ALL";
        }
        return "blob OP";
    };

    if (!r.ok) {
        char buf[192];
        std::snprintf(buf, sizeof(buf),
                      "%s failed (%s): %s",
                      kindStr(),
                      r.error_stage.c_str(),
                      r.error_message.c_str());
        motorThetaTxStatus_ = buf;
        motorConfigSetThenSave_ = false;  // abort any pending set->save chain on failure
        return true;
    }

    if (r.kind == MotorBlobHttpKind::Set) {
        char buf[176];
        // Make the RAM-only nature visible — without this, users see the
        // "ok" and assume the value is persistent, then find it reverted
        // on the next /motor/config/get (which reads flash).
        std::snprintf(buf, sizeof(buf),
                      "blob SET ok — applied to motor RAM, press Save All "
                      "to persist (%u B, CRC 0x%08X, %.2f s)",
                      static_cast<unsigned>(r.bytes),
                      static_cast<unsigned>(r.crc),
                      elapsed);
        motorThetaTxStatus_ = buf;
        // motorBlob_ already reflects the just-written blob (caller
        // applyThetaSlotEdit / Save does memcpy before invoking SET).
        // Authoritative window: subsequent auto-poll GETs return flash
        // bytes (pre-SAVE_ALL) which would otherwise stomp the just-
        // applied RAM edits in the UI. 5-minute window matches the
        // pre-existing SLCAN path constants.
        motorBlobLastSetResult_ = 0x00;  // 0 = OK in motor's wire encoding
        motorBlobLastSetCrcEcho_ = r.crc;
        motorBlobLastSetResultAt_ = ImGui::GetTime();
        motorBlobAuthoritativeUntil_ = ImGui::GetTime() + 300.0;

        motorConfigSetThenSave_ = false;
        return true;
    }

    if (r.kind == MotorBlobHttpKind::Save) {
        // SAVE_ALL alone is just motor's accept-ack. The motor's flash
        // is now updated, but the ONLY visible proof to the operator is
        // the next /motor/config/get returning the new field. Queue
        // that verify-GET on the same worker channel; show the
        // "saved + verified" status when it returns. Until then,
        // status reads "SAVE_ALL ok, verifying ...".
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "PARAM SAVE_ALL ok (%.2f s) — verifying flash via GET…",
                      elapsed);
        motorThetaTxStatus_ = buf;
        motorBlobPendingSaveVerify_ = true;
        // Drop the authoritative-RAM window so the verify-GET reads
        // flash truth (which now matches what we just persisted).
        motorBlobAuthoritativeUntil_ = 0.0;
        // Launch the verify GET on the next tick — we can't recurse
        // into launchMotorBlobGetHttp() right here because the worker
        // thread for the SAVE is still being joined a few lines above
        // and the launch_ check would race with the join.
        // Setting motorBlobHttpRunning_=false (already done) + the
        // pending flag is enough; the main tick's
        // drainMotorBlobHttpResult will not be re-entered until the
        // next loop iteration, which is when we call
        // launchMotorBlobGetHttp() below. Belt-and-braces: detach if
        // join hasn't happened yet (it has, the if() above did it).
        if (!launchMotorBlobGetHttp()) {
            // Fallback if the GET couldn't be launched right now;
            // operator's auto-poll will still re-read in ~1.5 s.
            std::snprintf(buf, sizeof(buf),
                          "PARAM SAVE_ALL ok (%.2f s) — flash updated, "
                          "auto-poll will refresh shortly",
                          elapsed);
            motorThetaTxStatus_ = buf;
            motorBlobPendingSaveVerify_ = false;
        }
        return true;
    }

    // GET path: install bytes and fan out into caches.
    if (r.blob.size() != 164u) {
        motorThetaTxStatus_ = "blob GET: short payload";
        return true;
    }
    std::memcpy(motorBlob_, r.blob.data(), 164);

    if (motorBlobPendingSaveVerify_) {
        // This GET was kicked off by a successful SAVE_ALL. Compose the
        // "saved + verified" status by reading back the same fields a
        // user might check (theta.offset is the canonical edit target).
        motorBlobPendingSaveVerify_ = false;
        float thetaOff = 0.0f;
        std::memcpy(&thetaOff,
                    &motorBlob_[kMotorBlobOffThetaOffset],
                    sizeof(thetaOff));
        char buf[224];
        std::snprintf(buf, sizeof(buf),
                      "saved to flash, verified — theta.offset=%.6f, "
                      "flash CRC 0x%08X",
                      static_cast<double>(thetaOff),
                      static_cast<unsigned>(r.crc));
        // Fan out into the normal caches first so other panels reflect
        // the just-read flash state, then overwrite the status to the
        // saved+verified message (applyMotorBlobReceived sets its own
        // status string, so we set ours after the call).
        applyMotorBlobReceived(static_cast<uint16_t>(r.bytes), r.crc,
                               "blob GET (post-SAVE verify)");
        motorThetaTxStatus_ = buf;
        return true;
    }

    applyMotorBlobReceived(static_cast<uint16_t>(r.bytes), r.crc, "blob GET (HTTP)");
    return true;
}
// end HTTP-driven motor-config blob path

void MainUi::resetCaptureSession(const char* reason)
{
    capture_.phase = CaptureSessionPhase::Idle;
    capture_.slotsConfigured = 0;
    capture_.lastSubSent = 0;
    capture_.samplesCaptured = 0;
    capture_.triggerSampleIdx = 0;
    capture_.bytesPerSample = 0;
    capture_.bytesExpected = 0;
    capture_.rawBytes.clear();
    capture_.byteSeen.clear();
    capture_.watchIdxBySlot.clear();
    capture_.lastDataAt = 0.0;
    capture_.nextStallProbeAt = 0.0;
    capture_.lastResumeOffset = 0;
    capture_.streamInFlight = false;
    capture_.streamOffset = 0;
    capture_.streamEndOffset = 0;
    capture_.nextChunkOffset = 0;
    capture_.streamRequestAt = 0.0;
    capture_.nextChunkAt = 0.0;
    if (reason) {
        capture_.status = reason;
        capture_.lastError = reason;
    } else {
        capture_.status = "idle";
    }
}

bool MainUi::startCaptureSession()
{
    if (capture_.phase != CaptureSessionPhase::Idle &&
        capture_.phase != CaptureSessionPhase::CooldownDone) {
        capture_.lastError = "capture already running";
        return false;
    }
    if (!isConnected()) {
        capture_.lastError = "not connected";
        capture_.status = "not connected";
        return false;
    }

    /* Multi-MCU sequential capture: scan ALL marked / plotted watches,
     * group by nodeId, queue every distinct node. The state machine
     * runs node-by-node -- when one node's CooldownDone hits, the
     * pollCaptureSession loop pops the next node and re-enters via
     * startNodeCapture. End-user observation: one click = every
     * marked variable lands in the plot regardless of which MCU it
     * lives on. Time correlation across nodes is best-effort (each
     * node's burst is timestamped to its own DONE wall-clock); for
     * tight cross-MCU sync use a future broadcast-arm cmd. */
    pendingCaptureNodes_.clear();
    std::unordered_set<uint8_t> seenNodes;

    /* Pass 1: capture-marked watches across every node. */
    for (const WatchVar& w : watches_) {
        if (!w.capture) continue;
        if (nodeIsInBoot(w.nodeId)) continue;
        if (seenNodes.insert(w.nodeId).second) {
            pendingCaptureNodes_.push_back(w.nodeId);
        }
    }
    /* Pass 2 fallback: if nothing was explicitly capture-marked, use
     * the plotted set. The toolbar Trigger is "snap detail of what I'm
     * looking at right now" and shouldn't require right-clicking
     * every series first. */
    if (pendingCaptureNodes_.empty()) {
        for (const WatchVar& w : watches_) {
            if (!w.plot) continue;
            if (nodeIsInBoot(w.nodeId)) continue;
            if (seenNodes.insert(w.nodeId).second) {
                pendingCaptureNodes_.push_back(w.nodeId);
            }
        }
    }
    if (pendingCaptureNodes_.empty()) {
        capture_.lastError = "no variables to capture (plot some or mark cap)";
        capture_.status = capture_.lastError;
        return false;
    }

    /* Pop first node and start its session. The rest queue up for
     * automatic chaining at CooldownDone. */
    const uint8_t firstNode = pendingCaptureNodes_.front();
    pendingCaptureNodes_.pop_front();
    return startNodeCapture(firstNode);
}

bool MainUi::startNodeCapture(uint8_t targetNode)
{
    if (!isConnected()) {
        capture_.lastError = "not connected";
        capture_.status = capture_.lastError;
        return false;
    }
    capture_.watchIdxBySlot.clear();
    /* Prefer capture-marked watches when ANY of them exist in the
     * whole watch list (across all nodes); otherwise fall back to
     * plot-marked. Matches the cross-node selection logic in
     * startCaptureSession so per-node behaviour stays consistent. */
    bool anyCaptureMarked = false;
    for (const WatchVar& w : watches_) {
        if (w.capture) { anyCaptureMarked = true; break; }
    }
    for (size_t i = 0; i < watches_.size(); ++i) {
        const WatchVar& w = watches_[i];
        if (w.nodeId != targetNode) continue;
        if (anyCaptureMarked ? !w.capture : !w.plot) continue;
        if (capture_.watchIdxBySlot.size() >= kCapMaxSlots) break;
        capture_.watchIdxBySlot.push_back(i);
    }
    if (capture_.watchIdxBySlot.empty()) {
        /* No matching variables for this node -- skip and try next. */
        if (!pendingCaptureNodes_.empty()) {
            const uint8_t nextNode = pendingCaptureNodes_.front();
            pendingCaptureNodes_.pop_front();
            return startNodeCapture(nextNode);
        }
        capture_.lastError = "no variables matched any queued node";
        capture_.status = capture_.lastError;
        return false;
    }
    const uint8_t nodeId = targetNode;
    capture_.nodeId = nodeId;
    if (nodeIsInBoot(capture_.nodeId)) {
        capture_.lastError = "node in BOOT";
        capture_.status = capture_.lastError;
        return false;
    }

    // Derive period / buffer from the user-facing duration knob. Override
    // the legacy persisted fields in-session -- we no longer expose those.
    const uint8_t slots = static_cast<uint8_t>(capture_.watchIdxBySlot.size());
    const CaptureBounds bounds = computeCaptureBounds(nodeId, slots, capture_.durationMs);
    capture_.periodUs = bounds.periodUs;
    capture_.bufferKb = bounds.bufferKb;
    capture_.prePct = 0;
    capture_.postPct = 100;
    capture_.triggerMode = 0;       // immediate -- fire on first sample
    capture_.triggerSlot = 0;
    capture_.triggerThreshold = 0;

    // HTTP path: when on TCP / Wi-Fi AP, don't drive the CAN state machine
    // from pc_tool. Hand the whole session off to app_dd's /capture HTTP
    // endpoint via the SSH tunnel; the worker thread takes care of fetching
    // and parsing the response, and pollCaptureSession picks the result up
    // on the next GUI tick via drainCaptureHttpResult(). Keeps the existing
    // CAN-driven path intact for SLCAN / Manual transports.
    if (captureUsesHttpPath()) {
        const double nowHttp = clock_.nowSeconds();
        capture_.bytesPerSample = static_cast<uint16_t>(slots * 4u);
        capture_.bytesExpected = 0;
        capture_.rawBytes.clear();
        capture_.byteSeen.clear();
        capture_.startedAt = nowHttp;
        capture_.phaseDeadline = 0.0;
        capture_.slotsConfigured = 0;
        capture_.samplesCaptured = 0;
        capture_.triggerSampleIdx = 0;
        capture_.lastDataAt = 0.0;
        capture_.streamInFlight = false;
        capture_.streamOffset = 0;
        capture_.streamEndOffset = 0;
        capture_.nextChunkOffset = 0;
        capture_.streamRequestAt = 0.0;
        capture_.nextChunkAt = 0.0;
        capture_.crcAttemptsLeft = 0;
        capture_.crcRequestSentAt = 0.0;
        capture_.crcDeadline = 0.0;
        capture_.lastCrcLocal = 0;
        capture_.lastCrcRemote = 0;
        capture_.lastError.clear();
        if (!launchCaptureHttpWorker()) {
            capture_.status = capture_.lastError.empty()
                ? "failed to start HTTP capture worker"
                : capture_.lastError;
            return false;
        }
        // Park in WaitDone -- pollCaptureSession sees both the phase AND
        // captureHttpRunning_, and routes through drainCaptureHttpResult.
        capture_.phase = CaptureSessionPhase::WaitDone;
        capture_.status = "fetching capture via app_dd /capture...";
        return true;
    }

    // Send RESET first to put the MCU in a known state regardless of
    // any half-completed previous session on the firmware side.
    const double now = clock_.nowSeconds();
    CanFrame tx = debug_.makeCapReset(capture_.nodeId, now);
    if (!sendFrameThreadSafe(tx)) {
        capture_.lastError = "tx failed (reset)";
        capture_.status = capture_.lastError;
        return false;
    }
    capture_.phase = CaptureSessionPhase::Reset;
    capture_.lastSubSent = kCapSubReset;
    capture_.startedAt = now;
    capture_.phaseDeadline = now + 0.6;
    capture_.slotsConfigured = 0;
    capture_.samplesCaptured = 0;
    capture_.triggerSampleIdx = 0;
    capture_.bytesPerSample = static_cast<uint16_t>(capture_.watchIdxBySlot.size() * 4u);
    capture_.bytesExpected = 0;
    capture_.rawBytes.clear();
    capture_.byteSeen.clear();
    // Bumped from 5 в†’ 32 after we observed legitimate "data stream
    // timeout" failures on bigger shots: each gap-fill READ_CHUNK can
    // itself land in a stall window if the bus is loaded, so a tight
    // budget would burn through retries faster than the bus drains.
    capture_.retriesLeft = 32;
    capture_.lastDataAt = 0.0;
    capture_.nextStallProbeAt = 0.0;
    capture_.lastResumeOffset = 0;
    capture_.streamInFlight = false;
    capture_.streamOffset = 0;
    capture_.streamEndOffset = 0;
    capture_.nextChunkOffset = 0;
    capture_.streamRequestAt = 0.0;
    capture_.nextChunkAt = 0.0;
    // Up to 4 full re-pulls if the MCU's CRC32 disagrees with ours. In
    // practice we expect the first attempt to match (CAN itself runs a
    // 15-bit CRC + ack, so byte-level corruption mid-frame is rare); the
    // budget exists so a session with a flaky transport doesn't cooldown
    // endlessly.
    capture_.crcAttemptsLeft = 4;
    capture_.crcRequestSentAt = 0.0;
    capture_.crcDeadline = 0.0;
    capture_.lastCrcLocal = 0;
    capture_.lastCrcRemote = 0;
    capture_.lastError.clear();
    capture_.status = "reset";
    return true;
}

bool MainUi::requestCaptureChunk(uint16_t byteOffset, uint16_t byteCount,
                                 double now, const char* statusPrefix)
{
    if (capture_.bytesExpected == 0u) return false;
    if (byteOffset >= capture_.bytesExpected) return false;

    const uint16_t remaining = static_cast<uint16_t>(capture_.bytesExpected - byteOffset);
    const uint16_t count = std::min<uint16_t>(byteCount == 0u ? remaining : byteCount,
                                              remaining);
    CaptureChunkRequest req;
    req.nodeId = capture_.nodeId;
    req.byteOffset = byteOffset;
    req.byteCount = count;
    CanFrame tx = debug_.makeCapReadChunk(req, now);
    if (!sendFrameThreadSafe(tx)) {
        return false;
    }

    capture_.phase = CaptureSessionPhase::ReadChunk;
    capture_.streamInFlight = true;
    capture_.streamOffset = byteOffset;
    capture_.streamEndOffset = static_cast<uint16_t>(byteOffset + count);
    capture_.streamRequestAt = now;
    capture_.lastDataAt = 0.0;
    capture_.lastResumeOffset = byteOffset;
    capture_.nextStallProbeAt = now + kCaptureChunkStallSec;
    capture_.nextChunkAt = 0.0;
    capture_.phaseDeadline = now + 2.5;

    const uint16_t done = capture_.streamEndOffset;
    capture_.status = std::string(statusPrefix ? statusPrefix : "reading") +
                      " " + std::to_string(done) + "/" +
                      std::to_string(capture_.bytesExpected) + " bytes";
    return true;
}

void MainUi::pollCaptureSession(double now)
{
    auto fail = [&](const char* msg) {
        capture_.lastError = msg;
        capture_.status = std::string("error: ") + msg;
        capture_.cooldownUntil = now + 1.0;
        capture_.phase = CaptureSessionPhase::CooldownDone;
    };

    // HTTP-driven capture takes precedence over the CAN state machine.
    // The worker thread populates capture_.rawBytes (and the other
    // metadata fields) atomically; drain it here on the GUI thread and
    // transition into Apply/CooldownDone in one shot.
    if (captureHttpRunning_.load(std::memory_order_acquire) ||
        captureHttpDone_.load(std::memory_order_acquire)) {
        drainCaptureHttpResult(now);
        return;
    }

    if (capture_.phase != CaptureSessionPhase::Idle &&
        capture_.phase != CaptureSessionPhase::CooldownDone &&
        nodeIsInBoot(capture_.nodeId)) {
        fail("node in BOOT");
        return;
    }

    switch (capture_.phase) {
        case CaptureSessionPhase::Idle:
            return;
        case CaptureSessionPhase::CooldownDone:
            if (now >= capture_.cooldownUntil) {
                /* Multi-MCU sequential capture: if more nodes are
                 * pending from the current Trigger click, pop the
                 * next one and re-enter the state machine via
                 * startNodeCapture. This is how one Trigger click
                 * walks motor -> RK -> ... when the user marked
                 * variables across multiple MCUs. */
                if (!pendingCaptureNodes_.empty()) {
                    const uint8_t nextNode = pendingCaptureNodes_.front();
                    pendingCaptureNodes_.pop_front();
                    char buf[64];
                    std::snprintf(buf, sizeof(buf),
                                  "chaining capture to node %u",
                                  static_cast<unsigned>(nextNode));
                    capture_.status = buf;
                    (void) startNodeCapture(nextNode);
                } else {
                    capture_.phase = CaptureSessionPhase::Idle;
                    capture_.status = "idle";
                }
            }
            return;
        case CaptureSessionPhase::Reset:
        case CaptureSessionPhase::SetSlot:
        case CaptureSessionPhase::SetConfig:
        case CaptureSessionPhase::SetTrigger:
        case CaptureSessionPhase::Arm:
            if (now > capture_.phaseDeadline) {
                fail("ack timeout");
            }
            return;
        case CaptureSessionPhase::WaitDone: {
            if (now >= capture_.nextStatusAt) {
                CanFrame tx = debug_.makeCapStatusReq(capture_.nodeId, now);
                if (!sendFrameThreadSafe(tx)) {
                    fail("tx failed (status)");
                    return;
                }
                capture_.nextStatusAt = now + 0.15;
            }
            // Expire if user-set capture window is clearly elapsed +
            // we still haven't seen DONE -- most likely trigger never
            // fired in the chosen condition.
            if (now > capture_.startedAt + 30.0) {
                fail("trigger never fired (30 s timeout)");
            }
            return;
        }
        case CaptureSessionPhase::ReadChunk:
        case CaptureSessionPhase::Drain:
            // Download is paced by PC-side windows. That keeps CAN load
            // bounded and leaves airtime for live READ_MEM graph updates.
            // If a window stalls, re-request the first missing byte inside
            // that same window; later windows are not considered missing yet.
            // prior request -- we treat that as transient (see ack
            if (capture_.bytesExpected == 0u) return;
            if (!capture_.streamInFlight) {
                if (capture_.nextChunkOffset < capture_.bytesExpected &&
                    now >= capture_.nextChunkAt) {
                    const uint16_t remaining =
                        static_cast<uint16_t>(capture_.bytesExpected - capture_.nextChunkOffset);
                    const uint16_t count = std::min<uint16_t>(remaining, kCaptureReadChunkBytes);
                    if (!requestCaptureChunk(capture_.nextChunkOffset, count, now, "reading")) {
                        fail("tx failed (next chunk)");
                    }
                }
                return;
            }

            if (now > capture_.nextStallProbeAt) {
                const ByteGap gap = findMissingBytes(capture_.byteSeen,
                                                     capture_.streamOffset,
                                                     capture_.streamEndOffset);
                const bool stalled =
                    (capture_.lastDataAt > 0.0) &&
                    (now > capture_.lastDataAt + kCaptureChunkStallSec) &&
                    gap.found;
                const bool noFramesYet =
                    (capture_.lastDataAt == 0.0) &&
                    (now > capture_.streamRequestAt + 1.2);
                if ((stalled || noFramesYet) && capture_.retriesLeft > 0) {
                    capture_.retriesLeft -= 1;
                    const uint16_t off = static_cast<uint16_t>(
                        gap.found ? gap.start : capture_.streamOffset);
                    const uint16_t cnt = static_cast<uint16_t>(capture_.streamEndOffset - off);
                    if (requestCaptureChunk(off, cnt, now, "stall-resume")) {
                        capture_.nextStallProbeAt = now + kCaptureChunkStallSec;
                    } else {
                        capture_.nextStallProbeAt = now + 0.3;
                    }
                }
            }
            if (capture_.streamInFlight && now > capture_.phaseDeadline) {
                fail("data stream timeout");
            }
            return;
        case CaptureSessionPhase::VerifyCrc: {
            if (now <= capture_.crcDeadline) return;
            // CRC reply timed out. Re-issue the request, bounded by
            // crcAttemptsLeft so a totally silent MCU eventually fails.
            if (capture_.crcAttemptsLeft == 0u) {
                fail("CRC request timeout (out of retries)");
                return;
            }
            capture_.crcAttemptsLeft -= 1u;
            CanFrame txCrc = debug_.makeCapCrcReq(capture_.nodeId, now);
            if (!sendFrameThreadSafe(txCrc)) {
                fail("tx failed (crc retry)");
                return;
            }
            capture_.crcRequestSentAt = now;
            capture_.crcDeadline = now + 2.0;
            capture_.status = std::string("verifying CRC32 (retry, ") +
                              std::to_string(capture_.crcAttemptsLeft) + " left)";
            return;
        }
        case CaptureSessionPhase::Apply:
            applyCapturedSamples();
            capture_.cooldownUntil = now + 0.3;
            capture_.phase = CaptureSessionPhase::CooldownDone;
            return;
    }
}

void MainUi::applyCapturedSamples()
{
    if (capture_.rawBytes.size() < capture_.bytesExpected) {
        capture_.status = "incomplete data";
        return;
    }
    const uint16_t samples = capture_.samplesCaptured;
    const uint8_t  slots = static_cast<uint8_t>(capture_.watchIdxBySlot.size());
    if (samples == 0u || slots == 0u) {
        capture_.status = "empty capture";
        return;
    }

    // Anchor the captured window in the past, where the samples were
    // actually taken on the MCU. capture_.doneStatusAt is the wall clock
    // when pc-tool first saw CaptureState::Done -- i.e. roughly when the
    // sampler finished collecting the LAST sample (modulo a few ms of CAN
    // RTT). Sample 0 of the valid window therefore lived at
    // doneStatusAt - samples*dt, the trigger sample at tBase + idx*dt.
    //
    // Anchoring at "now" (the previous behaviour) glued the captured
    // burst to the right edge of the plot, which moved further and
    // further into the future as ReadChunk ran (32 KB ~= 22 s). The
    // operator's mental model is "I clicked Trigger at THAT moment, and
    // the burst should refine the live trace AT that moment" -- past, not
    // future.
    const double rawNow = clock_.nowSeconds() - plotTimeOrigin_;
    const double dt = static_cast<double>(capture_.periodUs) * 1e-6;
    const double tEndAbs = (capture_.doneStatusAt > 0.0)
        ? (capture_.doneStatusAt - plotTimeOrigin_)
        : rawNow;
    const double tBase = tEndAbs - static_cast<double>(samples) * dt;
    const double tLast = tEndAbs;
    for (uint8_t slot = 0; slot < slots; ++slot) {
        const size_t wi = capture_.watchIdxBySlot[slot];
        if (wi >= watches_.size()) continue;
        WatchVar& w = watches_[wi];
        // Drop any live-poll samples that fall inside the window we're
        // about to refill. Without this, ImPlot draws zigzag artefacts
        // where slow READ_MEM points land at random Y between adjacent
        // capture points (the "staircase in the wavy region" the operator
        // reported). Keep everything outside the window untouched.
        if (!w.xs.empty()) {
            std::vector<double> nx, ny;
            nx.reserve(w.xs.size());
            ny.reserve(w.ys.size());
            for (size_t k = 0; k < w.xs.size() && k < w.ys.size(); ++k) {
                if (w.xs[k] >= tBase && w.xs[k] <= tLast) continue;
                nx.push_back(w.xs[k]);
                ny.push_back(w.ys[k]);
            }
            w.xs = std::move(nx);
            w.ys = std::move(ny);
        }
        for (uint16_t i = 0; i < samples; ++i) {
            const size_t off = static_cast<size_t>(i) * static_cast<size_t>(slots * 4u) +
                               static_cast<size_t>(slot) * 4u;
            if (off + 4 > capture_.rawBytes.size()) break;
            std::vector<uint8_t> bytes(capture_.rawBytes.begin() + off,
                                       capture_.rawBytes.begin() + off + 4);
            bool ok = false;
            const double v = decodeValue(bytes, w.type, ok);
            if (!ok) continue;
            const double t = tBase + static_cast<double>(i) * dt;
            w.xs.push_back(t);
            w.ys.push_back(v);
        }
        // Sort xs/ys by time so live + capture appended out-of-order
        // don't break ImPlot's PlotLine. ImPlot needs monotonic X.
        if (w.xs.size() > 1u) {
            std::vector<size_t> idx(w.xs.size());
            for (size_t i = 0; i < idx.size(); ++i) idx[i] = i;
            std::sort(idx.begin(), idx.end(),
                      [&](size_t a, size_t b) { return w.xs[a] < w.xs[b]; });
            std::vector<double> nx(w.xs.size()), ny(w.ys.size());
            for (size_t i = 0; i < idx.size(); ++i) {
                nx[i] = w.xs[idx[i]];
                ny[i] = w.ys[idx[i]];
            }
            w.xs = std::move(nx);
            w.ys = std::move(ny);
        }
        // Prune to plotHistorySec_ window like the live append path does.
        const double keepAfter = (rawNow > static_cast<double>(plotHistorySec_))
            ? (rawNow - static_cast<double>(plotHistorySec_))
            : -1e18;
        size_t first = 0;
        while (first < w.xs.size() && w.xs[first] < keepAfter) ++first;
        if (first > 0) {
            w.xs.erase(w.xs.begin(), w.xs.begin() + static_cast<std::ptrdiff_t>(first));
            w.ys.erase(w.ys.begin(), w.ys.begin() + static_cast<std::ptrdiff_t>(first));
        }
    }

    const double tEnd = tBase + static_cast<double>(samples) * dt;
    capture_.windowsRel.push_back({tBase, tEnd});
    if (capture_.windowsRel.size() > 16) {
        capture_.windowsRel.erase(capture_.windowsRel.begin());
    }
    markScenarioDirty("capture window appended");
    capture_.status = std::string("done: ") + std::to_string(samples) +
                      " samples, trigger @" + std::to_string(capture_.triggerSampleIdx);
}

void MainUi::handleCapFrame(const CanFrame& frame)
{
    if (capture_.phase == CaptureSessionPhase::Idle) {
        // Late frames after a finished session -- ignore.
        return;
    }
    // HTTP capture path: app_dd is driving CAP_REQ on our behalf, and CAP_RSP
    // frames may echo back over the TCP bridge. Don't let the CAN state
    // machine react -- it would re-drive its own RESET / SET_SLOT and double
    // up. drainCaptureHttpResult is the single source of truth for the HTTP
    // session; this guard makes sure stray bus traffic can't trip it.
    if (captureHttpRunning_.load(std::memory_order_acquire) ||
        captureHttpDone_.load(std::memory_order_acquire)) {
        return;
    }
    const double now = clock_.nowSeconds();

    if (auto ack = debug_.parseCapAck(frame); ack.has_value()) {
        if (ack->nodeId != capture_.nodeId) return;
        if (ack->isError || ack->status != 0) {
            // BAD_STATE (status == 2) on a READ_CHUNK we sent during the
            // ReadChunk phase is transient: it just means the MCU is
            // still busy streaming the previous request and refused to
            // reseat. Don't tear the session down -- the active stream
            // is still feeding us frames, and the stall-probe will
            // re-issue once it idles. This is what makes burst-mode
            // streaming + stall recovery composable; without it, every
            // legitimate probe would kill the shot.
            const bool transient =
                (capture_.phase == CaptureSessionPhase::ReadChunk) &&
                (ack->originalSub == kCapSubReadChunk) &&
                (ack->status == 2);
            if (transient) {
                capture_.nextStallProbeAt = now + 0.5;
                return;
            }
            char errBuf[80];
            std::snprintf(errBuf, sizeof(errBuf),
                          "MCU error code %u on sub 0x%02X",
                          static_cast<unsigned>(ack->status),
                          static_cast<unsigned>(ack->originalSub));
            capture_.lastError = errBuf;
            capture_.status = capture_.lastError;
            capture_.phase = CaptureSessionPhase::CooldownDone;
            capture_.cooldownUntil = now + 1.0;
            return;
        }
        // Drive the state machine on the matching ack. After each
        // transition we immediately send the next CAP_REQ so the firmware
        // doesn't sit idle waiting for our next query.
        auto sendOrFail = [&](CanFrame tx, const char* what) {
            if (!sendFrameThreadSafe(tx)) {
                capture_.lastError = std::string("tx failed (") + what + ")";
                capture_.status = capture_.lastError;
                capture_.phase = CaptureSessionPhase::CooldownDone;
                capture_.cooldownUntil = now + 1.0;
                return false;
            }
            return true;
        };

        if (ack->originalSub == kCapSubReadChunk) {
            return;
        }

        if (capture_.phase == CaptureSessionPhase::Reset && ack->originalSub == kCapSubReset) {
            capture_.slotsConfigured = 0;
            capture_.phase = CaptureSessionPhase::SetSlot;
            capture_.status = "set_slot 0";
            // Send first slot.
            const size_t wi = capture_.watchIdxBySlot[0];
            const WatchVar& w = watches_[wi];
            CaptureSlotConfig sc;
            sc.address = w.address;
            sc.size = (w.size > 0u && w.size <= 4u) ? w.size : 4u;
            sc.type = static_cast<CaptureSlotType>(captureSlotTypeFromString(w.type));
            CanFrame tx = debug_.makeCapSetSlot(capture_.nodeId, 0, sc, now);
            if (!sendOrFail(tx, "set_slot")) return;
        } else if (capture_.phase == CaptureSessionPhase::SetSlot && ack->originalSub == kCapSubSetSlot) {
            capture_.slotsConfigured += 1;
            if (capture_.slotsConfigured < capture_.watchIdxBySlot.size()) {
                const uint8_t slotIdx = capture_.slotsConfigured;
                const size_t wi = capture_.watchIdxBySlot[slotIdx];
                const WatchVar& w = watches_[wi];
                CaptureSlotConfig sc;
                sc.address = w.address;
                sc.size = (w.size > 0u && w.size <= 4u) ? w.size : 4u;
                sc.type = static_cast<CaptureSlotType>(captureSlotTypeFromString(w.type));
                capture_.status = "set_slot " + std::to_string(slotIdx);
                CanFrame tx = debug_.makeCapSetSlot(capture_.nodeId, slotIdx, sc, now);
                if (!sendOrFail(tx, "set_slot")) return;
            } else {
                capture_.phase = CaptureSessionPhase::SetConfig;
                capture_.status = "set_config";
                CaptureConfig cfg;
                cfg.nodeId = capture_.nodeId;
                cfg.slotCount = static_cast<uint8_t>(capture_.watchIdxBySlot.size());
                cfg.periodUs = capture_.periodUs;
                cfg.prePct = capture_.prePct;
                cfg.postPct = capture_.postPct;
                cfg.bufferKb = capture_.bufferKb;
                CanFrame tx = debug_.makeCapSetConfig(capture_.nodeId, cfg, now);
                if (!sendOrFail(tx, "set_config")) return;
            }
        } else if (capture_.phase == CaptureSessionPhase::SetConfig && ack->originalSub == kCapSubSetConfig) {
            capture_.phase = CaptureSessionPhase::SetTrigger;
            capture_.status = "set_trigger";
            CaptureConfig cfg;
            cfg.nodeId = capture_.nodeId;
            cfg.slotCount = static_cast<uint8_t>(capture_.watchIdxBySlot.size());
            cfg.trigger = static_cast<CaptureTrigger>(capture_.triggerMode);
            cfg.triggerSlot = capture_.triggerSlot;
            cfg.triggerThreshold = capture_.triggerThreshold;
            CanFrame tx = debug_.makeCapSetTrigger(capture_.nodeId, cfg, now);
            if (!sendOrFail(tx, "set_trigger")) return;
        } else if (capture_.phase == CaptureSessionPhase::SetTrigger && ack->originalSub == kCapSubSetTrigger) {
            capture_.phase = CaptureSessionPhase::Arm;
            capture_.status = "arming";
            CanFrame tx = debug_.makeCapArm(capture_.nodeId, now);
            if (!sendOrFail(tx, "arm")) return;
        } else if (capture_.phase == CaptureSessionPhase::Arm && ack->originalSub == kCapSubArm) {
            capture_.phase = CaptureSessionPhase::WaitDone;
            capture_.status = "armed, waiting trigger";
            capture_.nextStatusAt = now + 0.05;
        }
        capture_.phaseDeadline = now + 0.6;
        return;
    }

    if (auto st = debug_.parseCapStatus(frame); st.has_value()) {
        if (st->nodeId != capture_.nodeId) return;
        if (st->state == CaptureState::Done && capture_.phase == CaptureSessionPhase::WaitDone) {
            capture_.samplesCaptured = st->samplesCaptured;
            capture_.triggerSampleIdx = st->triggerSampleIdx;
            capture_.bytesPerSample = static_cast<uint16_t>(st->slotCount * 4u);
            capture_.bytesExpected = static_cast<uint16_t>(
                static_cast<uint32_t>(capture_.samplesCaptured) *
                static_cast<uint32_t>(capture_.bytesPerSample));
            capture_.rawBytes.assign(capture_.bytesExpected, 0u);
            capture_.byteSeen.assign(capture_.bytesExpected, false);
            capture_.doneStatusAt = now;
            capture_.nextChunkOffset = 0;

            const uint16_t firstCount = std::min<uint16_t>(
                capture_.bytesExpected, kCaptureReadChunkBytes);
            if (!requestCaptureChunk(0, firstCount, now, "reading")) {
                capture_.lastError = "tx failed (read_chunk)";
                capture_.status = capture_.lastError;
                capture_.phase = CaptureSessionPhase::CooldownDone;
                capture_.cooldownUntil = now + 1.0;
            }
        } else if (st->state == CaptureState::Error) {
            capture_.lastError = "MCU reports ERROR (code " + std::to_string(st->errorCode) + ")";
            capture_.status = capture_.lastError;
            capture_.phase = CaptureSessionPhase::CooldownDone;
            capture_.cooldownUntil = now + 1.0;
        }
        return;
    }

    if (auto crc = debug_.parseCapCrc(frame); crc.has_value()) {
        if (crc->nodeId != capture_.nodeId) return;
        if (capture_.phase != CaptureSessionPhase::VerifyCrc) return;
        const uint32_t local = captureCrc32(capture_.rawBytes.data(),
                                            capture_.rawBytes.size());
        capture_.lastCrcLocal = local;
        capture_.lastCrcRemote = crc->crc;
        if (local == crc->crc &&
            crc->totalBytes == static_cast<uint16_t>(capture_.rawBytes.size())) {
            // End-to-end byte fidelity confirmed. Move on to Apply.
            capture_.phase = CaptureSessionPhase::Apply;
            capture_.status = "applying (CRC32 OK)";
            return;
        }
        // Mismatch -- at least one byte was corrupted even though every
        // offset was covered. Wipe the receive buffers and pull the
        // whole window again. Decrement crcAttemptsLeft so a wedged
        // transport eventually gives up instead of looping forever.
        if (capture_.crcAttemptsLeft == 0u) {
            char errBuf[96];
            std::snprintf(errBuf, sizeof(errBuf),
                          "CRC mismatch: local=0x%08X mcu=0x%08X (out of retries)",
                          static_cast<unsigned>(local),
                          static_cast<unsigned>(crc->crc));
            capture_.lastError = errBuf;
            capture_.status = capture_.lastError;
            capture_.phase = CaptureSessionPhase::CooldownDone;
            capture_.cooldownUntil = now + 1.0;
            return;
        }
        capture_.crcAttemptsLeft -= 1u;
        std::fill(capture_.byteSeen.begin(), capture_.byteSeen.end(), false);
        std::fill(capture_.rawBytes.begin(), capture_.rawBytes.end(), 0u);
        capture_.nextChunkOffset = 0;

        const uint16_t firstCount = std::min<uint16_t>(
            capture_.bytesExpected, kCaptureReadChunkBytes);
        if (!requestCaptureChunk(0, firstCount, now, "CRC retry")) {
            capture_.lastError = "tx failed (CRC retry pull)";
            capture_.status = capture_.lastError;
            capture_.phase = CaptureSessionPhase::CooldownDone;
            capture_.cooldownUntil = now + 1.0;
            return;
        }
        capture_.status = std::string("CRC retry (") +
                          std::to_string(capture_.crcAttemptsLeft) + " left)";
        return;
    }

    if (auto data = debug_.parseCapData(frame); data.has_value()) {
        if (data->nodeId != capture_.nodeId) return;
        if (capture_.phase != CaptureSessionPhase::ReadChunk) return;
        const uint16_t base = data->byteOffset;
        for (uint8_t i = 0; i < data->validBytes; ++i) {
            const size_t pos = static_cast<size_t>(base) + i;
            if (pos >= capture_.rawBytes.size()) break;
            capture_.rawBytes[pos] = data->payload[i];
            if (pos < capture_.byteSeen.size()) capture_.byteSeen[pos] = true;
        }
        capture_.lastDataAt = now;
        // Push the stall probe out as long as data keeps arriving. We
        // only re-poke the MCU after ~0.8 s of true silence (see
        // pollCaptureSession).
        capture_.nextStallProbeAt = now + kCaptureChunkStallSec;
        capture_.phaseDeadline = now + 5.0;
        if (data->last) {
            capture_.streamInFlight = false;
            if (capture_.streamEndOffset > capture_.nextChunkOffset) {
                capture_.nextChunkOffset = capture_.streamEndOffset;
            }

            const ByteGap windowGap = findMissingBytes(capture_.byteSeen,
                                                       capture_.streamOffset,
                                                       capture_.streamEndOffset);
            if (windowGap.found) {
                const uint16_t missingBytes = static_cast<uint16_t>(
                    std::min<size_t>(windowGap.end - windowGap.start, 0xFFFFu));
                if (capture_.retriesLeft <= 0) {
                    capture_.lastError = "missing " + std::to_string(missingBytes) +
                                         " bytes after retries";
                    capture_.status = capture_.lastError;
                    capture_.phase = CaptureSessionPhase::CooldownDone;
                    capture_.cooldownUntil = now + 1.0;
                    return;
                }
                capture_.retriesLeft -= 1;
                if (!requestCaptureChunk(static_cast<uint16_t>(windowGap.start),
                                         missingBytes, now, "retry")) {
                    capture_.lastError = "tx failed (retry chunk)";
                    capture_.status = capture_.lastError;
                    capture_.phase = CaptureSessionPhase::CooldownDone;
                    capture_.cooldownUntil = now + 1.0;
                    return;
                }
                return;
            }

            if (capture_.nextChunkOffset < capture_.bytesExpected) {
                /* Deferred dispatch: arm nextChunkAt + return. The
                 * GUI-tick pollCaptureSession picks up the timer on
                 * its next pass and fires requestCaptureChunk from
                 * the main thread. This keeps a kCaptureChunkGapSec
                 * gap between chunks so the motor's prop_can SW TX
                 * FIFO (FIFO_SIZE=20) can drain between bursts.
                 * The 2026-05-13 inline-dispatch experiment skipped
                 * this gate and induced stall-resume loops at ~70-80%
                 * of every capture when telemetry broadcasts shared
                 * the bus -- reverted 2026-05-14. */
                capture_.nextChunkAt = now + kCaptureChunkGapSec;
                capture_.phaseDeadline = capture_.nextChunkAt + 2.0;
                capture_.status = "reading " +
                                  std::to_string(capture_.nextChunkOffset) + "/" +
                                  std::to_string(capture_.bytesExpected) +
                                  " bytes";
                return;
            }

            const ByteGap finalGap = findMissingBytes(capture_.byteSeen,
                                                      0u,
                                                      capture_.byteSeen.size());
            if (finalGap.found) {
                const uint16_t missingBytes = static_cast<uint16_t>(
                    std::min<size_t>(finalGap.end - finalGap.start, 0xFFFFu));
                if (capture_.retriesLeft <= 0) {
                    capture_.lastError = "missing " + std::to_string(missingBytes) +
                                         " bytes after retries";
                    capture_.status = capture_.lastError;
                    capture_.phase = CaptureSessionPhase::CooldownDone;
                    capture_.cooldownUntil = now + 1.0;
                    return;
                }
                capture_.retriesLeft -= 1;
                if (!requestCaptureChunk(static_cast<uint16_t>(finalGap.start),
                                         missingBytes, now, "retry")) {
                    capture_.lastError = "tx failed (retry chunk)";
                    capture_.status = capture_.lastError;
                    capture_.phase = CaptureSessionPhase::CooldownDone;
                    capture_.cooldownUntil = now + 1.0;
                    return;
                }
                return;
            }

            CanFrame txCrc = debug_.makeCapCrcReq(capture_.nodeId, now);
            if (sendFrameThreadSafe(txCrc)) {
                capture_.phase = CaptureSessionPhase::VerifyCrc;
                capture_.status = "verifying CRC32";
                capture_.crcRequestSentAt = now;
                capture_.crcDeadline = now + 2.0;
            } else {
                capture_.phase = CaptureSessionPhase::Apply;
                capture_.status = "applying (no CRC)";
            }
            return;
            // Find first contiguous unseen range. Under heavy CAN load a
            // handful of CAP_DATA frames can drop on the wire (firmware can't
            // detect the loss -- receiver-side check is the only signal).
            size_t gapStart = capture_.byteSeen.size();
            size_t gapEnd = 0;
            for (size_t i = 0; i < capture_.byteSeen.size(); ++i) {
                if (!capture_.byteSeen[i]) {
                    gapStart = i;
                    gapEnd = i;
                    while (gapEnd < capture_.byteSeen.size() && !capture_.byteSeen[gapEnd]) {
                        ++gapEnd;
                    }
                    break;
                }
            }

            const bool complete = (gapStart >= capture_.byteSeen.size());
            if (complete) {
                // Every offset is covered. Before applying, ask the MCU
                // for a CRC32 over the linearised buffer and compare
                // against our local CRC32 over rawBytes. A mismatch
                // means at least one byte was corrupted on the path
                // even though no offset was missing -- without this
                // check, byte-level corruption would silently land in
                // the operator's plot.
                CanFrame txCrc = debug_.makeCapCrcReq(capture_.nodeId, now);
                if (sendFrameThreadSafe(txCrc)) {
                    capture_.phase = CaptureSessionPhase::VerifyCrc;
                    capture_.status = "verifying CRC32";
                    capture_.crcRequestSentAt = now;
                    capture_.crcDeadline = now + 2.0;
                } else {
                    // CRC request itself failed to enqueue; skip the
                    // check and apply with the bytes we have. (This
                    // matches the pre-CRC behaviour and is no worse.)
                    capture_.phase = CaptureSessionPhase::Apply;
                    capture_.status = "applying (no CRC)";
                }
                return;
            }

            // Re-request the missing range. Motor's READ_CHUNK accepts any
            // [offset..offset+count) while state==DONE so as long as the
            // previous stream wrapped up (it did -- we just got CAP_DATA_END)
            // we can dispatch a follow-up immediately.
            const uint16_t missingBytes =
                static_cast<uint16_t>(std::min<size_t>(gapEnd - gapStart, 0xFFFFu));
            if (capture_.retriesLeft <= 0) {
                capture_.lastError = "missing " + std::to_string(missingBytes) +
                                     " bytes after retries";
                capture_.status = capture_.lastError;
                capture_.phase = CaptureSessionPhase::CooldownDone;
                capture_.cooldownUntil = now + 1.0;
                return;
            }
            capture_.retriesLeft -= 1;

            CaptureChunkRequest req;
            req.nodeId = capture_.nodeId;
            req.byteOffset = static_cast<uint16_t>(gapStart);
            req.byteCount = missingBytes;
            CanFrame tx = debug_.makeCapReadChunk(req, now);
            if (!sendFrameThreadSafe(tx)) {
                capture_.lastError = "tx failed (retry chunk)";
                capture_.status = capture_.lastError;
                capture_.phase = CaptureSessionPhase::CooldownDone;
                capture_.cooldownUntil = now + 1.0;
                return;
            }
            capture_.status = "retry " + std::to_string(missingBytes) + "B@" +
                              std::to_string(gapStart);
            capture_.phaseDeadline = now + 5.0 + estimateCaptureCooldownSec();
            capture_.lastDataAt = now;
            capture_.nextStallProbeAt = now + 1.0;
            capture_.lastResumeOffset = static_cast<uint16_t>(gapStart);
        }
        return;
    }
}

/* ========================================================================
 * Motor Config tab -- live view of everything in flash sector 11
 *
 * Wire protocol (matches motor FW v2.3.5 param-table):
 *   Request:  cmd=0x005, src=PC(0x10), dst=MOTOR(0x02) -> ID 0x10050210
 *             data[0]=0x02 (subcmd PARAM)
 *             data[1]=action (0=GET, 1=SET, 2=SAVE_ALL, 3=LOAD_ALL,
 *                             4=RESET_DEFAULTS)
 *             data[2..3]=u16 id LE
 *             data[4..7]=value (f32 or u32 LE)
 *   Response: cmd=0x0AB sub=0x02 -> ID 0x10AB1002
 *             data[1]=status, data[2..3]=id echo, data[4..7]=value
 *
 * Cache populated by handleFrame->handleMotorConfigResponse.
 * Background poll cycles GETs at ~25 ms / param so the entire 30-row
 * table refreshes in ~750 ms.
 * ======================================================================== */

namespace {

struct MotorConfigParamDef {
    uint16_t    id;
    const char* name;
    uint8_t     group;       /* 0=THETA, 1=PI, 2=PROFILER, 4=DIAG */
    uint8_t     type;        /* 0=f32, 1=u32 */
    bool        editable;
};

/* Mirrors param_table.c row order. Updated when FW grows the table. */
static const MotorConfigParamDef kMotorConfigParams[] = {
    /* Theta (read-only here; edit via FOC theta calibration panel) */
    {  0, "theta.offset",      0, 0, false },
    {  1, "theta.sector_0",    0, 0, false },
    {  2, "theta.sector_1",    0, 0, false },
    {  3, "theta.sector_2",    0, 0, false },
    {  4, "theta.sector_3",    0, 0, false },
    {  5, "theta.sector_4",    0, 0, false },
    {  6, "theta.sector_5",    0, 0, false },
    /* PI gains -- editable */
    { 10, "pi.kp_id",          1, 0, true  },
    { 11, "pi.ki_id",          1, 0, true  },
    { 12, "pi.kp_iq",          1, 0, true  },
    { 13, "pi.ki_iq",          1, 0, true  },
    { 14, "pi.kp_omega",       1, 0, true  },
    { 15, "pi.ki_omega",       1, 0, true  },
    /* Profiler -- editable (Profiler wizard writes here later) */
    { 20, "profiler.rs",       2, 0, true  },
    { 21, "profiler.ld",       2, 0, true  },
    { 22, "profiler.lq",       2, 0, true  },
    { 23, "profiler.lambda",   2, 0, true  },
    { 24, "profiler.j",        2, 0, true  },
    { 25, "profiler.b",        2, 0, true  },
    { 26, "profiler.valid",    2, 1, true  },
    { 27, "profiler.uptime",   2, 1, true  },
    /* Diagnostics (READONLY) */
    { 30, "diag.brk_live",         4, 1, false },
    { 31, "diag.brk_low_cnt",      4, 1, false },
    { 32, "diag.brk_at_flt",       4, 1, false },
    { 33, "diag.brk_flt_cnt",      4, 1, false },
    { 34, "diag.wdog_hits",        4, 1, false },
    { 35, "diag.wdog_ms",          4, 1, false },
    { 36, "diag.wdog_spr_brk",     4, 1, false },
    { 37, "cfg.fw_ver",            4, 1, false },
    { 38, "cfg.crc",               4, 1, false },
    /* HW feature defaults (motor FW v2.3.8+) -- persistent boot defaults
     * for external hardware lines. SET pushes immediately to GPIO. */
    { 50, "pfc.enabled",           5, 1, true  },
};
constexpr int kMotorConfigParamCount =
    sizeof(kMotorConfigParams) / sizeof(kMotorConfigParams[0]);

const char* motorConfigGroupName(uint8_t g) {
    switch (g) {
        case 0: return "THETA";
        case 1: return "PI";
        case 2: return "PROFILER";
        case 4: return "DIAG";
        case 5: return "HW";
        default: return "?";
    }
}

} // namespace

bool MainUi::sendMotorConfigParam(uint8_t action, uint16_t id, uint32_t value_bits)
{
    if (!activeTransport_ || !activeTransport_->isOpen()) return false;
    // Phase C.2 (2026-05-28): on TCP / Wi-Fi-AP, route SAVE_ALL through
    // /motor/config/save instead of cranking the single CFG frame
    // through the lossy bridge — the motor blocks the CAN bus ~1.5 s
    // during flash erase, and the bridge can drop the response while
    // the motor is busy. The HTTP path runs the request locally on
    // app_dd and waits the full response window. Other PARAM actions
    // (GET=0, SET=1, LOAD_ALL=3, RESET=4, GET_*=5+) stay on direct CAN
    // — they're cheap and frequent.
    constexpr uint8_t kParamActSaveAll = 2;
    if (action == kParamActSaveAll && motorBlobUsesHttpPath()) {
        return launchMotorBlobSaveHttp();
    }
    CanFrame f{};
    f.id = (1u << 28) | (0x005u << 16) |
           (static_cast<uint32_t>(kCanAddrMotor) << 8) | kCanAddrPc;
    f.extended = true;
    f.rx       = false;
    f.dlc      = 8;
    f.data.assign(8, 0u);
    f.data[0]  = 0x02;     /* subcmd PARAM */
    f.data[1]  = action;
    f.data[2]  = static_cast<uint8_t>(id & 0xFFu);
    f.data[3]  = static_cast<uint8_t>((id >> 8) & 0xFFu);
    std::memcpy(&f.data[4], &value_bits, 4);
    return sendFrameThreadSafe(f);
}

bool MainUi::applyMotorConfigBlobSetAll()
{
    /* Global Set: take the last-read blob as baseline, splice EVERY
     * editable, blob-resident field from its edit buffer, recompute the
     * payload CRC32, and send the whole 164-byte blob once. The FW then
     * verifies the CRC over the entire payload before applying via
     * MotorConfigScatter — so a single bad field can't sneak in. */
    if (!motorBlobValid_) {
        motorThetaTxStatus_ = "config cache empty - press Read Flash first";
        return false;
    }

    uint8_t buf[kMotorCfgBlobTotalBytes];
    std::memcpy(buf, motorBlob_, sizeof(buf));

    int applied = 0;
    std::string bad;
    for (int i = 0; i < kMotorConfigParamCount; ++i) {
        const auto& def = kMotorConfigParams[i];
        if (!def.editable) continue;
        const int off = motorConfigBlobOffsetForId(def.id);
        if (off < 0 || off + 4 > static_cast<int>(sizeof(buf))) continue;
        auto it = motorConfigEditBuffer_.find(def.id);
        if (it == motorConfigEditBuffer_.end() || it->second.empty()) continue;

        uint32_t bits = 0;
        try {
            if (def.type == 0 /* f32 */) {
                float v = std::stof(it->second);
                std::memcpy(&bits, &v, sizeof(bits));
            } else {
                bits = static_cast<uint32_t>(std::stoul(it->second, nullptr, 0));
            }
        } catch (...) {
            if (!bad.empty()) bad += ", ";
            bad += def.name;
            continue;
        }
        std::memcpy(&buf[off], &bits, sizeof(bits));
        ++applied;
    }

    if (!bad.empty()) {
        motorThetaTxStatus_ = std::string("bad value(s): ") + bad + " — fix and retry";
        return false;
    }

    /* Recompute payload CRC32 (byte-identical to FW MotorConfigCrc32) and
     * stamp it into the header — the FW rejects the SET commit otherwise. */
    const uint32_t crc = motorConfigCrc32(
        &buf[kMotorCfgBlobPayloadOffset], kMotorCfgBlobPayloadBytes);
    std::memcpy(&buf[kMotorBlobOffPayloadCrc], &crc, sizeof(crc));

    /* Update the local baseline so the table + a follow-up Save reflect
     * exactly what we just sent, and a second Set compounds cleanly. */
    std::memcpy(motorBlob_, buf, sizeof(buf));

    if (!sendMotorBlobSet(buf)) {
        motorThetaTxStatus_ = "blob SET tx failed (transport not open?)";
        return false;
    }

    /* Local edits are authoritative until Save persists them (a flash GET
     * would otherwise revert the on-screen values). Cleared by Save's
     * verify-GET, an explicit Get, or the 5-min timeout. */
    motorBlobAuthoritativeUntil_ = ImGui::GetTime() + 300.0;
    populateMotorConfigCacheFromBlob();
    return true;
}

bool MainUi::requestMotorConfigLiveRead()
{
    if (!activeTransport_ || !activeTransport_->isOpen()) {
        motorConfigStatus_ = "Read Live failed: transport not open";
        return false;
    }

    bool ok = true;
    for (int i = 0; i < kMotorConfigParamCount; ++i) {
        ok = sendMotorConfigParam(/*GET*/ 0, kMotorConfigParams[i].id, 0u) && ok;
    }
    if (ok) {
        motorConfigLastReadAllStartAt_ = ImGui::GetTime();
        motorConfigStatus_ = "Reading live motor RAM values...";
        motorThetaTxStatus_ = "Read Live requested";
    } else {
        motorConfigStatus_ = "Read Live: one or more GET frames failed";
        motorThetaTxStatus_ = motorConfigStatus_;
    }
    return ok;
}

bool MainUi::sendMotorProfilerCommand(uint8_t action, uint8_t arg, uint32_t value_bits)
{
    if (!activeTransport_ || !activeTransport_->isOpen()) return false;
    CanFrame f{};
    f.id = (1u << 28) | (kMotorCmdConfig << 16) |
           (static_cast<uint32_t>(kCanAddrMotor) << 8) | kCanAddrPc;
    f.extended = true;
    f.rx       = false;
    f.dlc      = 8;
    f.data.assign(8, 0u);
    f.data[0] = kMotorCfgProfiler;
    f.data[1] = action;
    f.data[2] = arg;
    f.data[3] = 0xFFu;
    std::memcpy(&f.data[4], &value_bits, 4);
    return sendFrameThreadSafe(f);
}

bool MainUi::sendMotorProfilerLimits()
{
    auto fbits = [](float v) {
        uint32_t bits = 0;
        std::memcpy(&bits, &v, sizeof(bits));
        return bits;
    };
    bool ok = true;
    ok = sendMotorProfilerCommand(/*SET_LIMIT*/ 1, /*TEST_CURRENT_A*/ 0,
                                  fbits(motorProfilerTestCurrentA_)) && ok;
    ok = sendMotorProfilerCommand(/*SET_LIMIT*/ 1, /*MAX_VOLTAGE_PCT*/ 1,
                                  fbits(motorProfilerMaxVoltagePct_)) && ok;
    ok = sendMotorProfilerCommand(/*SET_LIMIT*/ 1, /*PULSE_MS*/ 2,
                                  fbits(motorProfilerPulseMs_)) && ok;
    ok = sendMotorProfilerCommand(/*SET_LIMIT*/ 1, /*SETTLE_MS*/ 3,
                                  fbits(motorProfilerSettleMs_)) && ok;
    const uint32_t repeats = static_cast<uint32_t>(std::clamp(motorProfilerRepeatCount_, 1, 16));
    ok = sendMotorProfilerCommand(/*SET_LIMIT*/ 1, /*REPEAT_COUNT*/ 4, repeats) && ok;
    ok = sendMotorProfilerCommand(/*SET_LIMIT*/ 1, /*PI_BANDWIDTH_HZ*/ 5,
                                  fbits(motorProfilerBandwidthHz_)) && ok;
    return ok;
}

void MainUi::requestMotorProfilerResults()
{
    for (uint8_t r = 0; r < 7u; ++r) {
        sendMotorProfilerCommand(/*GET_RESULT*/ 4, r, 0u);
    }
}

void MainUi::handleMotorConfigResponse(uint16_t id, uint8_t status, uint32_t value_bits)
{
    auto& e = motorConfigCache_[id];
    e.rawValue  = value_bits;
    e.hasValue  = true;
    e.lastSeenTs = ImGui::GetTime();
    e.lastStatus = status;
}

void MainUi::populateMotorConfigCacheFromBlob()
{
    if (!motorBlobValid_) return;
    const double now = ImGui::GetTime();
    /* Walk the static param table; for every id with a blob offset,
     * copy 4 raw bytes from the staged blob into motorConfigCache_.
     * status=0 (ok) since the FW already verified payload CRC32 on its
     * side before chunking; we trust the snapshot. */
    for (int i = 0; i < kMotorConfigParamCount; ++i) {
        const uint16_t id = kMotorConfigParams[i].id;
        const int off = motorConfigBlobOffsetForId(id);
        if (off < 0) continue;
        if (off + 4 > static_cast<int>(sizeof(motorBlob_))) continue;
        uint32_t bits = 0;
        std::memcpy(&bits, &motorBlob_[off], sizeof(bits));
        auto& e = motorConfigCache_[id];
        e.rawValue   = bits;
        e.hasValue   = true;
        e.lastSeenTs = now;
        e.lastStatus = 0;
    }
}

void MainUi::drawMotorConfigParamRow(const MotorConfigParam& def, MotorConfigEntry& e)
{
    ImGui::PushID(def.id);

    ImGui::TableNextRow();
    /* id */
    ImGui::TableNextColumn(); ImGui::Text("%u", def.id);
    /* name */
    ImGui::TableNextColumn(); ImGui::TextUnformatted(def.name);
    /* current value */
    ImGui::TableNextColumn();
    if (!e.hasValue) {
        ImGui::TextDisabled("--");
    } else if (e.lastStatus != 0) {
        ImGui::TextColored(ImVec4(0.95f, 0.4f, 0.4f, 1.0f),
                           "err=0x%02X", e.lastStatus);
    } else if (def.type == 0 /* f32 */) {
        float v;
        std::memcpy(&v, &e.rawValue, sizeof(v));
        ImGui::Text("%.6f", v);
    } else {
        ImGui::Text("0x%08X (%u)", e.rawValue, e.rawValue);
    }
    /* edit field. No per-row Set anymore — the single "Set" button at the
     * top gathers every editable field and writes the whole blob at once.
     * An empty buffer is lazily seeded from the current value so an
     * untouched field re-writes its own value (a no-op change), keeping
     * the whole-blob CRC self-consistent. */
    ImGui::TableNextColumn();
    if (def.editable) {
        auto& buf = motorConfigEditBuffer_[def.id];
        if (buf.empty() && e.hasValue) {
            char tmp[32];
            if (def.type == 0) {
                float v; std::memcpy(&v, &e.rawValue, sizeof(v));
                std::snprintf(tmp, sizeof(tmp), "%.6f", v);
            } else {
                std::snprintf(tmp, sizeof(tmp), "%u", e.rawValue);
            }
            buf = tmp;
        }
        ImGui::SetNextItemWidth(160.0f);
        char editBuf[64] = {};
        std::strncpy(editBuf, buf.c_str(), sizeof(editBuf) - 1);
        if (ImGui::InputText("##edit", editBuf, sizeof(editBuf))) {
            buf = editBuf;
        }
    } else {
        ImGui::TextDisabled("(read-only)");
    }

    ImGui::PopID();
}

void MainUi::drawMotorProfilerPanel(bool canSend)
{
    const bool inDiag = lastDiagStatus_.valid ? lastDiagStatus_.diag_active : diagModeOn_;
    const bool startReady = canSend && inDiag;

    if (!ImGui::CollapsingHeader("Motor profiler", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }

    ImGui::Text("Expected Rs %.4f Ohm, Ld/Lq %.3f mH",
                14.0507f, 25.22861f);
    ImGui::Text("State: %s  test: %s  progress: %u%%  error: %s",
                motorProfilerStateName(motorProfiler_.state),
                motorProfilerTestName(motorProfiler_.active_test),
                (unsigned)motorProfiler_.progress,
                motorProfilerErrorName(motorProfiler_.error));
    ImGui::ProgressBar(static_cast<float>(motorProfiler_.progress) / 100.0f,
                       ImVec2(-FLT_MIN, 0.0f));
    if (!motorProfilerStatus_.empty()) {
        ImGui::TextWrapped("%s", motorProfilerStatus_.c_str());
    }
    if (!inDiag) {
        ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.25f, 1.0f),
                           "mainPCB diag mode is off");
        ImGui::SameLine();
        ImGui::TextDisabled("(use Take control in the header bar)");
    }

    ImGui::Spacing();
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputFloat("test current A", &motorProfilerTestCurrentA_, 0.05f, 0.25f, "%.3f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputFloat("max voltage %", &motorProfilerMaxVoltagePct_, 0.5f, 1.0f, "%.2f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputFloat("bandwidth Hz", &motorProfilerBandwidthHz_, 10.0f, 50.0f, "%.1f");

    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputFloat("pulse ms", &motorProfilerPulseMs_, 1.0f, 5.0f, "%.1f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputFloat("settle ms", &motorProfilerSettleMs_, 1.0f, 5.0f, "%.1f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputInt("repeat", &motorProfilerRepeatCount_);
    motorProfilerRepeatCount_ = std::clamp(motorProfilerRepeatCount_, 1, 16);

    ImGui::BeginDisabled(!canSend);
    if (ImGui::Button("Apply limits", ImVec2(120, 0))) {
        motorProfilerStatus_ = sendMotorProfilerLimits()
            ? "Profiler limits sent"
            : "Profiler limits failed: transport not open";
    }
    ImGui::SameLine();
    if (ImGui::Button("Status", ImVec2(90, 0))) {
        motorProfilerStatus_ = sendMotorProfilerCommand(/*GET_STATUS*/ 0, 0, 0)
            ? "Profiler status requested"
            : "Profiler status request failed";
    }
    ImGui::SameLine();
    if (ImGui::Button("Read results", ImVec2(110, 0))) {
        requestMotorProfilerResults();
        motorProfilerStatus_ = "Profiler result reads requested";
    }
    ImGui::SameLine();
    if (ImGui::Button("Abort", ImVec2(80, 0))) {
        motorProfilerDoneFetchIssued_ = false;
        motorProfilerStatus_ = sendMotorProfilerCommand(/*ABORT*/ 3, 0, 0)
            ? "Profiler abort sent"
            : "Profiler abort failed";
    }
    ImGui::EndDisabled();

    ImGui::BeginDisabled(!startReady);
    if (ImGui::Button("XX precheck", ImVec2(130, 0))) {
        std::fill(std::begin(motorProfiler_.result_valid),
                  std::end(motorProfiler_.result_valid), false);
        motorProfilerDoneFetchIssued_ = false;
        const bool ok = sendMotorProfilerLimits() &&
                        sendMotorProfilerCommand(/*START*/ 2, /*NOLOAD_PRECHECK*/ 1, 0);
        motorProfilerStatus_ = ok ? "No-load precheck started" : "No-load precheck failed";
    }
    ImGui::SameLine();
    if (ImGui::Button("KZ Rs/Ld/Lq + PI", ImVec2(170, 0))) {
        std::fill(std::begin(motorProfiler_.result_valid),
                  std::end(motorProfiler_.result_valid), false);
        motorProfilerDoneFetchIssued_ = false;
        const bool ok = sendMotorProfilerLimits() &&
                        sendMotorProfilerCommand(/*START*/ 2, /*BLOCKED_RS_L_PI*/ 2, 0);
        motorProfilerStatus_ = ok ? "Locked test started" : "Locked test failed";
    }
    ImGui::EndDisabled();
    if (!startReady) {
        ImGui::SameLine();
        ImGui::TextDisabled("start requires active diag mode");
    }

    ImGui::SameLine();
    ImGui::BeginDisabled(!canSend);
    if (ImGui::Button("Save to flash##profiler", ImVec2(120, 0))) {
        if (sendMotorConfigParam(/*SAVE_ALL*/ 2, 0, 0)) {
            motorConfigStatus_ = "SAVE_ALL sent after profiler";
            motorProfilerStatus_ = "SAVE_ALL sent";
        } else {
            motorProfilerStatus_ = "SAVE_ALL failed";
        }
    }
    ImGui::EndDisabled();

    const double now = ImGui::GetTime();
    if (canSend && motorProfiler_.state == 1 /* RUNNING */ &&
        now >= motorProfilerNextStatusAt_) {
        sendMotorProfilerCommand(/*GET_STATUS*/ 0, 0, 0);
        motorProfilerNextStatusAt_ = now + 0.25;
    }
    if (canSend && motorProfiler_.state == 2 /* DONE */ &&
        !motorProfilerDoneFetchIssued_) {
        requestMotorProfilerResults();
        sendMotorBlobGet();
        motorProfilerDoneFetchIssued_ = true;
        motorProfilerStatus_ = "Profiler done; results and config refresh requested";
    }
    if (motorProfiler_.state != 2 /* DONE */) {
        motorProfilerDoneFetchIssued_ = false;
    }

    ImGui::Spacing();
    static const char* kNames[7] = {
        "Rs Ohm", "Ld H", "Lq H", "Kp Id", "Ki Id", "Kp Iq", "Ki Iq"
    };
    if (ImGui::BeginTable("##profiler_results", 4,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                          ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("result", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("value",  ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("spec",   ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("valid",  ImGuiTableColumnFlags_WidthFixed, 60.0f);
        ImGui::TableHeadersRow();
        for (int i = 0; i < 7; ++i) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextUnformatted(kNames[i]);
            ImGui::TableNextColumn();
            if (motorProfiler_.result_valid[i]) {
                if (i == 1 || i == 2) {
                    ImGui::Text("%.6f  (%.3f mH)",
                                motorProfiler_.results[i],
                                motorProfiler_.results[i] * 1000.0f);
                } else {
                    ImGui::Text("%.6f", motorProfiler_.results[i]);
                }
            } else {
                ImGui::TextDisabled("--");
            }
            ImGui::TableNextColumn();
            if (i == 0)      ImGui::Text("14.05 Ohm");
            else if (i < 3)  ImGui::Text("25.23 mH");
            else             ImGui::TextDisabled("--");
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(motorProfiler_.result_valid[i] ? "yes" : "no");
        }
        ImGui::EndTable();
    }
}

void MainUi::drawMotorConfig()
{
    const bool canSend = activeTransport_ && activeTransport_->isOpen();

    ImGui::TextUnformatted("Motor Config -- flash sector 11 (motor FW param-table)");
    ImGui::Separator();
    ImGui::Spacing();

    if (!canSend) {
        ImGui::TextColored(ImVec4(0.95f, 0.4f, 0.4f, 1.0f),
                           "Connect first (Connection tab).");
        return;
    }

    /* ===== Top status bar: identity from on-flash header ===== */
    {
        const auto& fwIt  = motorConfigCache_.find(37 /* cfg.fw_ver */);
        const auto& crcIt = motorConfigCache_.find(38 /* cfg.crc */);
        ImGui::Text("cfg.fw_ver  = ");
        ImGui::SameLine();
        if (fwIt != motorConfigCache_.end() && fwIt->second.hasValue) {
            const uint32_t v = fwIt->second.rawValue;
            ImGui::Text("0x%08X  (v%u.%u.%u)",
                        v, (v >> 16) & 0xFFu, (v >> 8) & 0xFFu, v & 0xFFu);
        } else { ImGui::TextDisabled("(not yet read)"); }

        ImGui::Text("cfg.crc     = ");
        ImGui::SameLine();
        if (crcIt != motorConfigCache_.end() && crcIt->second.hasValue) {
            ImGui::Text("0x%08X", crcIt->second.rawValue);
        } else { ImGui::TextDisabled("(not yet read)"); }
    }

    ImGui::Spacing();

    /* ===== Action buttons: Read Live / Read Flash / Set / Save All / Reset =====
     * Sources are deliberately explicit:
     *   Read Live  -> PARAM GET from motor RAM/live state.
     *   Read Flash -> read the 164-B blob from flash sector 11.
     *   Set        -> write edited values to motor RAM only (NOT flash).
     *   Save All   -> persist motor RAM to flash, then verify with Read Flash.
     * Reset restores factory defaults and persists them (RESET -> SAVE ->
     * verify). Live and Flash intentionally differ after Set until Save All. */
    const bool busy = motorBlobHttpRunning_.load(std::memory_order_acquire);

    ImGui::BeginDisabled(busy);
    if (ImGui::Button("Read Live")) {
        motorThetaTxStatus_.clear();
        requestMotorConfigLiveRead();
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Read current motor RAM/live parameter values. Use this after Set.");

    ImGui::SameLine();
    ImGui::BeginDisabled(busy);
    if (ImGui::Button("Read Flash")) {
        /* Explicit read: drop the post-Set authoritative lock so the
         * stream repopulates motorBlob_ from flash, then GET the blob. */
        motorBlobAuthoritativeUntil_ = 0.0;
        motorThetaTxStatus_.clear();
        if (sendMotorBlobGet()) {
            motorConfigOpLabel_ = "Reading flash config";
            motorConfigStatus_  = "Reading saved flash config (whole blob)...";
        } else {
            motorConfigStatus_  = "Read Flash failed: transport not open";
        }
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Read the saved flash blob from sector 11. This can differ from live RAM after Set.");

    ImGui::SameLine();
    /* Set needs a baseline blob (from a prior Read Flash) to splice edits onto. */
    ImGui::BeginDisabled(busy || !motorBlobValid_);
    if (ImGui::Button("Set")) {
        motorThetaTxStatus_.clear();
        /* RAM ONLY: write the whole edited blob to motor RAM. Does NOT save
         * to flash — the operator presses "Save All" for that. The drain
         * path reports "applied to motor RAM, press Save All to persist". */
        if (applyMotorConfigBlobSetAll()) {
            motorConfigOpLabel_ = "Writing config to RAM";
            motorConfigStatus_  = "Writing whole config to motor RAM...";
        }
        /* applyMotorConfigBlobSetAll() sets motorThetaTxStatus_ itself on
         * failure (bad value / empty cache / tx fail). */
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(motorBlobValid_
            ? "Write the WHOLE edited config to motor RAM only. Press Save All to persist to flash."
            : "Press Read Flash first - Set needs a baseline blob to splice your edits onto.");

    ImGui::SameLine();
    /* Save All: persist motor RAM -> flash, then verify-GET (drain path
     * shows the read-back flash CRC). Separate from Set on purpose. */
    ImGui::BeginDisabled(busy);
    if (ImGui::Button("Save All")) {
        motorThetaTxStatus_.clear();
        /* SAVE_ALL: on TCP routes to /motor/config/save (HTTP) which then
         * runs its own verify-GET; on SLCAN sends the CFG SAVE_ALL frame. */
        if (sendMotorConfigParam(/*SAVE_ALL*/ 2, 0, 0)) {
            motorConfigOpLabel_ = "Saving to flash";
            motorConfigStatus_  = "Saving motor RAM to flash, then verifying...";
        } else {
            motorConfigStatus_  = "Save failed: transport not open";
        }
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Persist the current motor RAM config to flash, then verify-GET and confirm the flash CRC.");

    ImGui::SameLine(0.0f, 24.0f);
    ImGui::BeginDisabled(busy);
    if (ImGui::Button("Reset defaults")) {
        motorThetaTxStatus_.clear();
        motorConfigEditBuffer_.clear();
        /* Factory defaults to motor RAM, then persist to flash + verify —
         * same one-click semantics as Set. RESET_DEFAULTS is RAM-only, so
         * chain SAVE_ALL; on TCP that runs the HTTP save + verify-GET which
         * refreshes the table from flash. */
        const bool ok = sendMotorConfigParam(/*RESET_DEFAULTS*/ 4, 0, 0)
                     && sendMotorConfigParam(/*SAVE_ALL*/ 2, 0, 0);
        if (ok) {
            motorConfigOpLabel_ = "Reset + saving defaults";
            motorConfigStatus_  = "Factory defaults -> motor RAM, saving to flash...";
        } else {
            motorConfigStatus_  = "Reset failed: transport not open";
        }
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Reset config to factory defaults and persist: defaults -> flash -> verify, in one click.");

    /* ===== Progress / confirmation line ===== */
    ImGui::Spacing();
    if (busy) {
        const double el = clock_.nowSeconds() - motorBlobHttpStartedAt_;
        /* simple ASCII spinner (default ImGui font has no emoji glyphs) */
        const char spin[] = { '|', '/', '-', '\\' };
        const int phase = static_cast<int>(el * 8.0) & 3;
        ImGui::TextColored(ImVec4(0.96f, 0.84f, 0.30f, 1.0f),
                           "[%c] %s ... %.1f s",
                           spin[phase],
                           motorConfigOpLabel_.empty() ? "working" : motorConfigOpLabel_.c_str(),
                           el);
    } else {
        /* Async result (motorThetaTxStatus_, set by drainMotorBlobHttpResult)
         * is the authoritative outcome; fall back to the immediate
         * button-press feedback otherwise. Colour green on success,
         * red on failure so the operator instantly sees whether the new
         * config landed. */
        const std::string& s = !motorThetaTxStatus_.empty() ? motorThetaTxStatus_
                                                             : motorConfigStatus_;
        if (!s.empty()) {
            auto has = [&](const char* k) { return s.find(k) != std::string::npos; };
            ImVec4 col(0.80f, 0.80f, 0.80f, 1.0f);
            if (has("fail") || has("error") || has("bad") || has("timeout") ||
                has("mismatch") || has("empty") || has("not open"))
                col = ImVec4(0.95f, 0.40f, 0.40f, 1.0f);
            else if (has("ok") || has("verified") || has("written") ||
                     has("saved") || has("applied"))
                col = ImVec4(0.40f, 0.85f, 0.45f, 1.0f);
            const char* mark = (col.y > 0.8f) ? "OK - " : "";  // green -> success prefix
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextColored(col, "%s%s", mark, s.c_str());
            ImGui::PopTextWrapPos();
        }
    }

    ImGui::Spacing();
    ImGui::Separator();
    // Motor profiler migrated to the Calibration tab (Test 2 -- Machine
    // parameters). See docs/calibration-tab-redesign.md. This tab is kept
    // as the raw param-table escape hatch (DIAG counters, power users).
    ImGui::TextDisabled("Motor profiler -> moved to the Calibration tab (Test 2)");
    ImGui::Spacing();
    ImGui::Separator();

    /* ===== Param table grouped by group =====
     * Read-only viewer + edit fields. No background polling — the table
     * reflects the last Read Live, Read Flash, or just-sent Set. */
    static const uint8_t kGroupOrder[] = {0, 1, 2, 4};
    const ImGuiTableFlags tflags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                   ImGuiTableFlags_SizingStretchProp;

    for (uint8_t grp : kGroupOrder) {
        const char* hdrName = motorConfigGroupName(grp);
        char hdrLabel[64];
        std::snprintf(hdrLabel, sizeof(hdrLabel), "%s parameters", hdrName);
        if (!ImGui::CollapsingHeader(hdrLabel, ImGuiTreeNodeFlags_DefaultOpen)) continue;

        char tableId[32];
        std::snprintf(tableId, sizeof(tableId), "##tbl_%u", grp);
        if (ImGui::BeginTable(tableId, 4, tflags)) {
            ImGui::TableSetupColumn("id",      ImGuiTableColumnFlags_WidthFixed, 50.0f);
            ImGui::TableSetupColumn("name",    ImGuiTableColumnFlags_WidthStretch, 1.5f);
            ImGui::TableSetupColumn("current", ImGuiTableColumnFlags_WidthStretch, 1.5f);
            ImGui::TableSetupColumn("edit",    ImGuiTableColumnFlags_WidthStretch, 1.5f);
            ImGui::TableHeadersRow();

            for (int i = 0; i < kMotorConfigParamCount; ++i) {
                const auto& def = kMotorConfigParams[i];
                if (def.group != grp) continue;
                MotorConfigParam pubDef{ def.id, def.name, def.group,
                                        def.type, def.editable };
                drawMotorConfigParamRow(pubDef, motorConfigCache_[def.id]);
            }
            ImGui::EndTable();
        }
    }
}

} // namespace drivescope
