#pragma once

#ifdef _WIN32

#include <string>
#include <vector>

namespace drivescope::wlan {

struct ScanEntry {
    std::string ssid;        // UTF-8
    int  signal_pct = 0;     // 0..100
    bool secured = true;     // WPA/WPA2/WPA3 anything
    bool connected = false;  // currently associated to this SSID
};

struct ScanResult {
    bool ok = false;
    std::vector<ScanEntry> entries;
    std::string error;
};

// Reads visible networks from the OS WLAN cache. Cheap. Filters by SSID prefix
// when non-empty. Issues a background WlanScan() so the cache is fresher next
// call — first-call results may be stale by a few seconds.
ScanResult scan(const std::string& ssid_prefix);

// Connect to a WPA2-Personal network using the given pre-shared key. Builds a
// temporary profile, calls WlanSetProfile + WlanConnect. Returns immediately
// after WlanConnect dispatches; check currentSsid() to confirm association.
bool connect(const std::string& ssid, const std::string& password,
             std::string& error);

// SSID of the network the WLAN interface is currently associated with.
// Empty if not associated or no WLAN interface available.
std::string currentSsid();

} // namespace drivescope::wlan

#endif // _WIN32
