#include "sniffer/CanSniffer.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ios>

namespace drivescope {

namespace {
constexpr const char* kEnvVar = "DRIVESCOPE_SNIFF_JSONL";

void appendHex(std::string& out, const std::vector<uint8_t>& data)
{
    static const char* h = "0123456789ABCDEF";
    out.reserve(out.size() + data.size() * 2);
    for (uint8_t b : data) {
        out.push_back(h[(b >> 4) & 0xF]);
        out.push_back(h[b & 0xF]);
    }
}

// Minimal JSON string escaping for the decoded text (control chars dropped).
void appendJsonEscaped(std::string& out, const std::string& s)
{
    for (char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (static_cast<unsigned char>(c) >= 0x20) out.push_back(c);
                break;
        }
    }
}
} // namespace

void CanSniffer::ensureInit()
{
    if (tried_) return;
    tried_ = true;
    const char* path = std::getenv(kEnvVar);
    if (path != nullptr && path[0] != '\0') {
        out_.open(path, std::ios::out | std::ios::app);
        active_ = out_.is_open();
    }
}

bool CanSniffer::isActive()
{
    std::lock_guard<std::mutex> lock(mutex_);
    ensureInit();
    return active_;
}

void CanSniffer::record(Dir dir, const char* transport, const CanFrame& frame,
                        const std::string& decoded)
{
    std::lock_guard<std::mutex> lock(mutex_);
    ensureInit();
    if (!active_) return;

    char head[192];
    std::snprintf(head, sizeof(head),
        "{\"ts\":%.6f,\"dir\":\"%s\",\"transport\":\"%s\",\"id\":\"0x%08X\",\"ext\":%s,\"len\":%u,\"data\":\"",
        frame.timestamp,
        dir == Dir::Tx ? "tx" : "rx",
        transport != nullptr ? transport : "?",
        frame.id,
        frame.extended ? "true" : "false",
        static_cast<unsigned>(frame.data.size()));

    std::string line(head);
    appendHex(line, frame.data);
    line += "\",\"decoded\":\"";
    appendJsonEscaped(line, decoded);
    line += "\"}\n";

    out_ << line;
    out_.flush();   // flush per line so a crash still leaves a complete log
    // ofstream IO errors only set failbit/badbit; we deliberately swallow them
    // so the sniffer can never affect send/receive behavior.
}

} // namespace drivescope
