#pragma once

#include "elf/ElfSymbolLoader.h"

#include <string>
#include <vector>

namespace drivescope {

struct SlcanSettings {
    bool enabled = true;
    std::string port = "COM3";
    int baud = 115200;
    std::string nominal = "S6";
    std::string data = "Y2";
    bool silent = false;
};

struct TcpSettings {
    bool enabled = true;
    std::string host = "192.168.0.103";
    int port = 45333;
    int busBitrate = 500000;
};

struct WifiApSettings {
    bool enabled = true;
    std::string ssid = "Kitchen_Machine_<UUID>";
    std::string host = "192.168.1.0";
    int port = 45333;
    int busBitrate = 500000;
};

struct WatchConfig {
    std::string name;
    std::string symbolName;
    bool hasAddress = false;
    uint32_t address = 0;
    std::string type = "float";
    uint8_t size = 4;
    uint8_t nodeId = 2;
    float hz = 10.0f;
    bool enabled = true;
    bool plot = true;
    double plotScale = 1.0;
    // Triggered Capture: whether this watch is part of the next capture
    // request. Persisted in the scenario so re-opening DriveScope keeps
    // the user's "subscribe to capture" picks for each variable.
    bool capture = false;
};

struct CaptureWindowPersist {
    double startRel = 0.0;
    double endRel   = 0.0;
};

// Triggered Capture settings, persisted alongside watches. The plot tab now
// exposes only one user knob — the capture-window duration in ms — and
// derives periodUs / bufferKb / pre/post from it. The other fields are kept
// for backwards compatibility with old scenario JSONs, but the runtime no
// longer reads them on Trigger.
struct CaptureConfigPersist {
    uint8_t  nodeId = 2;       // motor by default
    uint16_t periodUs = 250;   // legacy; recomputed from durationMs
    uint8_t  prePct = 0;
    uint8_t  postPct = 100;
    uint8_t  bufferKb = 32;
    uint8_t  triggerMode = 0;  // 0=immediate (user-facing trigger removed)
    uint8_t  triggerSlot = 0;
    int32_t  triggerThreshold = 0;
    int      durationMs = 200; // total capture window — the only knob users see
    // Past capture-window markers (X-axis annotations). Persisted so reopening
    // a scenario shows the exact same brackets the operator saw live.
    std::vector<CaptureWindowPersist> windows;
};

struct UpdatePathsConfig {
    bool present = false;
    std::string motorFirmware;
    std::string rkFirmware;
    std::string esp32FirmwareDir;
    std::string mainAppScript;
    std::string mainResourcesScript;
    std::string recoveryAppDir;
    std::string recoveryOta;
};

struct ToolConfig {
    std::string transport = "tcp";
    bool autoconnect = true;
    SlcanSettings slcan;
    TcpSettings tcp;
    WifiApSettings wifiAp;
    UpdatePathsConfig updates;
    std::vector<std::string> symbolFiles;
    std::vector<WatchConfig> watches;
    CaptureConfigPersist capture;
};

bool loadToolConfig(const std::string& path, ToolConfig& config, std::string& error);

} // namespace drivescope
