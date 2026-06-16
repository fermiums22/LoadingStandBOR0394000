#include "config/ToolConfig.h"

#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>

namespace drivescope {

namespace {
std::string readFile(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::string extractBlock(const std::string& text, const char* key, char open, char close)
{
    const std::string needle = std::string("\"") + key + "\"";
    size_t keyPos = std::string::npos;
    size_t colon = std::string::npos;
    for (size_t search = 0;;) {
        keyPos = text.find(needle, search);
        if (keyPos == std::string::npos) return {};
        colon = keyPos + needle.size();
        while (colon < text.size() && std::isspace(static_cast<unsigned char>(text[colon]))) ++colon;
        if (colon < text.size() && text[colon] == ':') break;
        search = keyPos + needle.size();
    }
    size_t pos = text.find(open, colon + 1);
    if (pos == std::string::npos) return {};

    int depth = 0;
    bool inString = false;
    bool escape = false;
    for (size_t i = pos; i < text.size(); ++i) {
        const char c = text[i];
        if (escape) {
            escape = false;
            continue;
        }
        if (c == '\\' && inString) {
            escape = true;
            continue;
        }
        if (c == '"') {
            inString = !inString;
            continue;
        }
        if (inString) continue;
        if (c == open) ++depth;
        if (c == close) {
            --depth;
            if (depth == 0) return text.substr(pos + 1, i - pos - 1);
        }
    }
    return {};
}

std::string jsonString(const std::string& obj, const char* key, const std::string& fallback = {})
{
    const std::regex re(std::string("\"") + key + "\"\\s*:\\s*\"([^\"]*)\"");
    std::smatch m;
    if (!std::regex_search(obj, m, re)) return fallback;
    return m[1].str();
}

bool jsonBool(const std::string& obj, const char* key, bool fallback)
{
    const std::regex re(std::string("\"") + key + "\"\\s*:\\s*(true|false|0|1)");
    std::smatch m;
    if (!std::regex_search(obj, m, re)) return fallback;
    return m[1] == "true" || m[1] == "1";
}

double jsonNumber(const std::string& obj, const char* key, double fallback)
{
    const std::regex re(std::string("\"") + key + "\"\\s*:\\s*(-?0x[0-9A-Fa-f]+|-?[0-9]+(?:\\.[0-9]+)?)");
    std::smatch m;
    if (!std::regex_search(obj, m, re)) return fallback;
    const std::string s = m[1].str();
    if (s.find("0x") != std::string::npos || s.find("0X") != std::string::npos) {
        return static_cast<double>(std::stoul(s, nullptr, 0));
    }
    return std::stod(s);
}

uint32_t jsonAddress(const std::string& obj, bool& present)
{
    present = false;
    std::string s = jsonString(obj, "address");
    if (!s.empty()) {
        present = true;
        return static_cast<uint32_t>(std::stoul(s, nullptr, 0));
    }
    const std::regex re("\"address\"\\s*:\\s*(0x[0-9A-Fa-f]+|[0-9]+)");
    std::smatch m;
    if (!std::regex_search(obj, m, re)) return 0;
    present = true;
    return static_cast<uint32_t>(std::stoul(m[1].str(), nullptr, 0));
}

std::vector<std::string> jsonStringArray(const std::string& text, const char* key)
{
    std::vector<std::string> out;
    const std::string block = extractBlock(text, key, '[', ']');
    const std::regex re("\"([^\"]+)\"");
    for (auto it = std::sregex_iterator(block.begin(), block.end(), re);
         it != std::sregex_iterator(); ++it) {
        out.push_back((*it)[1].str());
    }
    return out;
}

std::string resolveRelativeToConfig(const std::filesystem::path& configDir,
                                    const std::string& path)
{
    namespace fs = std::filesystem;
    if (path.empty()) return path;

    const fs::path p(path);
    if (p.is_absolute() || fs::exists(p)) return path;

    const fs::path byConfig = configDir / p;
    if (fs::exists(byConfig)) return byConfig.string();

    const fs::path bySymbolsDir = configDir / "symbols" / p.filename();
    if (fs::exists(bySymbolsDir)) return bySymbolsDir.string();

    return path;
}

std::vector<std::string> jsonObjectArray(const std::string& text, const char* key)
{
    std::vector<std::string> out;
    const std::string block = extractBlock(text, key, '[', ']');
    size_t i = 0;
    while (i < block.size()) {
        while (i < block.size() && block[i] != '{') ++i;
        if (i >= block.size()) break;
        size_t start = i;
        int depth = 0;
        bool inString = false;
        bool escape = false;
        for (; i < block.size(); ++i) {
            const char c = block[i];
            if (escape) {
                escape = false;
                continue;
            }
            if (c == '\\' && inString) {
                escape = true;
                continue;
            }
            if (c == '"') {
                inString = !inString;
                continue;
            }
            if (inString) continue;
            if (c == '{') ++depth;
            if (c == '}') {
                --depth;
                if (depth == 0) {
                    out.push_back(block.substr(start, i - start + 1));
                    ++i;
                    break;
                }
            }
        }
    }
    return out;
}

uint8_t sizeForType(const std::string& type)
{
    if (type == "uint8" || type == "int8" || type == "u8" || type == "i8") return 1;
    if (type == "uint16" || type == "int16" || type == "u16" || type == "i16") return 2;
    if (type == "double") return 8;
    return 4;
}

void parseSlcan(const std::string& text, SlcanSettings& slcan)
{
    const std::string obj = extractBlock(text, "slcan", '{', '}');
    if (obj.empty()) return;
    slcan.enabled = jsonBool(obj, "enabled", slcan.enabled);
    slcan.port = jsonString(obj, "port", slcan.port);
    slcan.baud = static_cast<int>(jsonNumber(obj, "baud", slcan.baud));
    slcan.nominal = jsonString(obj, "nominal", slcan.nominal);
    slcan.data = jsonString(obj, "data", slcan.data);
    slcan.silent = jsonBool(obj, "silent", slcan.silent);
}

void parseTcp(const std::string& text, TcpSettings& tcp)
{
    const std::string obj = extractBlock(text, "tcp", '{', '}');
    if (obj.empty()) return;
    tcp.enabled = jsonBool(obj, "enabled", tcp.enabled);
    tcp.host = jsonString(obj, "host", tcp.host);
    tcp.port = static_cast<int>(jsonNumber(obj, "port", tcp.port));
    tcp.busBitrate = static_cast<int>(jsonNumber(obj, "bus_bitrate", tcp.busBitrate));
}

void parseWifiAp(const std::string& text, WifiApSettings& wifiAp)
{
    const std::string obj = extractBlock(text, "wifi_ap", '{', '}');
    if (obj.empty()) return;
    wifiAp.enabled = jsonBool(obj, "enabled", wifiAp.enabled);
    wifiAp.ssid = jsonString(obj, "ssid", wifiAp.ssid);
    wifiAp.host = jsonString(obj, "host", wifiAp.host);
    wifiAp.port = static_cast<int>(jsonNumber(obj, "port", wifiAp.port));
    wifiAp.busBitrate = static_cast<int>(jsonNumber(obj, "bus_bitrate", wifiAp.busBitrate));
}

WatchConfig parseWatch(const std::string& obj)
{
    WatchConfig w;
    w.name = jsonString(obj, "name");
    w.symbolName = jsonString(obj, "symbol", w.name);
    bool hasAddress = false;
    w.address = jsonAddress(obj, hasAddress);
    w.hasAddress = hasAddress;
    w.type = jsonString(obj, "type", w.type);
    w.size = static_cast<uint8_t>(jsonNumber(obj, "size", sizeForType(w.type)));
    w.nodeId = static_cast<uint8_t>(jsonNumber(obj, "node_id", jsonNumber(obj, "nodeId", w.nodeId)));
    const double periodSec = jsonNumber(obj, "period_sec", -1.0);
    if (periodSec > 0.0) {
        w.hz = static_cast<float>(1.0 / periodSec);
    } else {
        w.hz = static_cast<float>(jsonNumber(obj, "hz", w.hz));
    }
    w.enabled = jsonBool(obj, "enabled", w.enabled);
    w.plot = jsonBool(obj, "plot", w.plot);
    w.capture = jsonBool(obj, "capture", w.capture);
    const double plotScale = jsonNumber(obj, "plot_scale", w.plotScale);
    if (std::isfinite(plotScale) && plotScale > 0.0) {
        w.plotScale = plotScale;
    }
    if (w.name.empty()) w.name = w.symbolName;
    return w;
}

void parseCapture(const std::string& text, CaptureConfigPersist& cap)
{
    const std::string obj = extractBlock(text, "capture", '{', '}');
    if (obj.empty()) return;
    cap.nodeId = static_cast<uint8_t>(jsonNumber(obj, "node_id", cap.nodeId));
    cap.periodUs = static_cast<uint16_t>(jsonNumber(obj, "period_us", cap.periodUs));
    cap.prePct = static_cast<uint8_t>(jsonNumber(obj, "pre_pct", cap.prePct));
    cap.postPct = static_cast<uint8_t>(jsonNumber(obj, "post_pct", cap.postPct));
    cap.bufferKb = static_cast<uint8_t>(jsonNumber(obj, "buffer_kb", cap.bufferKb));
    cap.triggerMode = static_cast<uint8_t>(jsonNumber(obj, "trigger_mode", cap.triggerMode));
    cap.triggerSlot = static_cast<uint8_t>(jsonNumber(obj, "trigger_slot", cap.triggerSlot));
    cap.triggerThreshold = static_cast<int32_t>(jsonNumber(obj, "trigger_threshold", cap.triggerThreshold));
    cap.durationMs = static_cast<int>(jsonNumber(obj, "duration_ms", cap.durationMs));

    cap.windows.clear();
    for (const std::string& wobj : jsonObjectArray(obj, "windows")) {
        CaptureWindowPersist w;
        w.startRel = jsonNumber(wobj, "start", 0.0);
        w.endRel   = jsonNumber(wobj, "end",   0.0);
        if (w.endRel >= w.startRel) cap.windows.push_back(w);
    }
}

void parseUpdates(const std::string& text, const std::filesystem::path& configDir,
                  UpdatePathsConfig& updates)
{
    const std::string obj = extractBlock(text, "updates", '{', '}');
    if (obj.empty()) return;

    updates.present = true;
    updates.motorFirmware = resolveRelativeToConfig(
        configDir, jsonString(obj, "motor_firmware", updates.motorFirmware));
    updates.rkFirmware = resolveRelativeToConfig(
        configDir, jsonString(obj, "rk_firmware", updates.rkFirmware));
    updates.esp32FirmwareDir = resolveRelativeToConfig(
        configDir, jsonString(obj, "esp32_firmware_dir", updates.esp32FirmwareDir));
    updates.mainAppScript = resolveRelativeToConfig(
        configDir, jsonString(obj, "main_app_script", updates.mainAppScript));
    updates.mainResourcesScript = resolveRelativeToConfig(
        configDir, jsonString(obj, "main_resources_script", updates.mainResourcesScript));
    updates.recoveryAppDir = resolveRelativeToConfig(
        configDir, jsonString(obj, "recovery_app_dir", updates.recoveryAppDir));
    updates.recoveryOta = resolveRelativeToConfig(
        configDir, jsonString(obj, "recovery_ota", updates.recoveryOta));
}
} // namespace

bool loadToolConfig(const std::string& path, ToolConfig& config, std::string& error)
{
    const std::string text = readFile(path);
    if (text.empty()) {
        error = "empty or missing config: " + path;
        return false;
    }

    ToolConfig out;
    out.transport = jsonString(text, "transport", out.transport);
    out.autoconnect = jsonBool(text, "autoconnect", out.autoconnect);
    parseSlcan(text, out.slcan);
    parseTcp(text, out.tcp);
    parseWifiAp(text, out.wifiAp);
    out.symbolFiles = jsonStringArray(text, "symbols");
    const std::filesystem::path configDir =
        std::filesystem::absolute(std::filesystem::path(path)).parent_path();
    for (std::string& symbolFile : out.symbolFiles) {
        symbolFile = resolveRelativeToConfig(configDir, symbolFile);
    }

    for (const std::string& obj : jsonObjectArray(text, "watches")) {
        out.watches.push_back(parseWatch(obj));
    }

    parseUpdates(text, configDir, out.updates);
    parseCapture(text, out.capture);

    if (out.transport != "tcp" && out.transport != "slcan" &&
        out.transport != "wifi_ap" && out.transport != "manual") {
        error = "transport must be tcp, slcan, wifi_ap or manual";
        return false;
    }
    if (out.transport == "tcp" && !out.tcp.enabled) {
        error = "selected tcp transport is disabled in config";
        return false;
    }
    if (out.transport == "wifi_ap" && !out.wifiAp.enabled) {
        error = "selected wifi_ap transport is disabled in config";
        return false;
    }
    if (out.transport == "slcan" && !out.slcan.enabled) {
        error = "selected slcan transport is disabled in config";
        return false;
    }

    config = std::move(out);
    error.clear();
    return true;
}

} // namespace drivescope
