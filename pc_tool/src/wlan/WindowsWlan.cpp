#include "wlan/WindowsWlan.h"

#ifdef _WIN32

#include <windows.h>
#include <wlanapi.h>
#include <objbase.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cwchar>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace drivescope::wlan {
namespace {

struct WlanHandleGuard {
    HANDLE h = nullptr;
    DWORD  negotiated = 0;
    ~WlanHandleGuard() { if (h) WlanCloseHandle(h, nullptr); }
    bool open(std::string& error) {
        const DWORD client_version = 2;
        const DWORD rc = WlanOpenHandle(client_version, nullptr, &negotiated, &h);
        if (rc != ERROR_SUCCESS) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "WlanOpenHandle failed (rc=%lu)", rc);
            error = buf;
            return false;
        }
        return true;
    }
};

template <typename T>
struct WlanFreeGuard {
    T* ptr = nullptr;
    ~WlanFreeGuard() { if (ptr) WlanFreeMemory(ptr); }
};

// First non-virtual interface in connectable state. Most laptops only have one
// real Wi-Fi NIC; the corner case is virtual adapters from VPN clients which
// should be ignored.
bool firstUsableInterface(HANDLE h, GUID& out_guid, std::string& error)
{
    PWLAN_INTERFACE_INFO_LIST list = nullptr;
    const DWORD rc = WlanEnumInterfaces(h, nullptr, &list);
    WlanFreeGuard<WLAN_INTERFACE_INFO_LIST> guard{list};
    if (rc != ERROR_SUCCESS || !list) {
        char buf[80];
        std::snprintf(buf, sizeof(buf), "WlanEnumInterfaces failed (rc=%lu)", rc);
        error = buf;
        return false;
    }
    if (list->dwNumberOfItems == 0) {
        error = "no Wi-Fi interfaces found";
        return false;
    }
    // Prefer one already connected; otherwise first in the list.
    for (DWORD i = 0; i < list->dwNumberOfItems; ++i) {
        if (list->InterfaceInfo[i].isState == wlan_interface_state_connected) {
            out_guid = list->InterfaceInfo[i].InterfaceGuid;
            return true;
        }
    }
    out_guid = list->InterfaceInfo[0].InterfaceGuid;
    return true;
}

std::string ssidToUtf8(const DOT11_SSID& s)
{
    return std::string(reinterpret_cast<const char*>(s.ucSSID),
                       static_cast<size_t>(s.uSSIDLength));
}

// Get currently-associated SSID for a given interface. Returns empty when the
// interface isn't connected or query fails (treat both as "not associated").
std::string queryCurrentSsid(HANDLE h, const GUID& iface)
{
    PWLAN_CONNECTION_ATTRIBUTES attr = nullptr;
    DWORD size = 0;
    WLAN_OPCODE_VALUE_TYPE vt = wlan_opcode_value_type_invalid;
    const DWORD rc = WlanQueryInterface(h, &iface, wlan_intf_opcode_current_connection,
                                        nullptr, &size,
                                        reinterpret_cast<PVOID*>(&attr), &vt);
    WlanFreeGuard<WLAN_CONNECTION_ATTRIBUTES> guard{attr};
    if (rc != ERROR_SUCCESS || !attr) return {};
    if (attr->isState != wlan_interface_state_connected) return {};
    return ssidToUtf8(attr->wlanAssociationAttributes.dot11Ssid);
}

std::wstring utf8ToWide(const std::string& s)
{
    if (s.empty()) return {};
    const int needed = MultiByteToWideChar(CP_UTF8, 0, s.data(),
                                           static_cast<int>(s.size()),
                                           nullptr, 0);
    std::wstring w(static_cast<size_t>(needed), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()),
                        w.data(), needed);
    return w;
}

// XML profile for WPA2-Personal (AES) connection. The Kitchen_Machine_*
// hostapd config bakes wpa=3 (WPA1+WPA2) but we always negotiate WPA2/AES
// because that's what the customer-facing Windows wlan profile uses too
// (see CLAUDE.md note about WPAPSK silently failing).
std::wstring buildProfileXml(const std::string& ssid_utf8,
                             const std::string& password_utf8)
{
    auto xmlEscape = [](const std::string& s) {
        std::string out;
        out.reserve(s.size());
        for (char c : s) {
            switch (c) {
                case '&':  out += "&amp;";  break;
                case '<':  out += "&lt;";   break;
                case '>':  out += "&gt;";   break;
                case '"':  out += "&quot;"; break;
                case '\'': out += "&apos;"; break;
                default:   out += c;        break;
            }
        }
        return out;
    };

    const std::string ssid_x = xmlEscape(ssid_utf8);
    const std::string pwd_x  = xmlEscape(password_utf8);

    std::ostringstream o;
    o << "<?xml version=\"1.0\"?>"
      << "<WLANProfile xmlns=\"http://www.microsoft.com/networking/WLAN/profile/v1\">"
      << "<name>" << ssid_x << "</name>"
      << "<SSIDConfig><SSID><name>" << ssid_x << "</name></SSID>"
      << "<nonBroadcast>false</nonBroadcast></SSIDConfig>"
      << "<connectionType>ESS</connectionType>"
      << "<connectionMode>manual</connectionMode>"
      << "<MSM><security>"
      << "<authEncryption>"
      << "<authentication>WPA2PSK</authentication>"
      << "<encryption>AES</encryption>"
      << "<useOneX>false</useOneX>"
      << "</authEncryption>"
      << "<sharedKey>"
      << "<keyType>passPhrase</keyType>"
      << "<protected>false</protected>"
      << "<keyMaterial>" << pwd_x << "</keyMaterial>"
      << "</sharedKey>"
      << "</security></MSM>"
      << "</WLANProfile>";

    return utf8ToWide(o.str());
}

} // namespace

ScanResult scan(const std::string& ssid_prefix)
{
    ScanResult r;
    WlanHandleGuard guard;
    if (!guard.open(r.error)) return r;

    GUID iface{};
    if (!firstUsableInterface(guard.h, iface, r.error)) return r;

    // Kick a fresh scan; results land in the OS cache asynchronously. We still
    // read the current cache below — first call after launch may be slightly
    // stale, subsequent ticks see updated data.
    WlanScan(guard.h, &iface, nullptr, nullptr, nullptr);

    PWLAN_AVAILABLE_NETWORK_LIST list = nullptr;
    const DWORD rc = WlanGetAvailableNetworkList(
        guard.h, &iface,
        WLAN_AVAILABLE_NETWORK_INCLUDE_ALL_MANUAL_HIDDEN_PROFILES,
        nullptr, &list);
    WlanFreeGuard<WLAN_AVAILABLE_NETWORK_LIST> netGuard{list};
    if (rc != ERROR_SUCCESS || !list) {
        char buf[96];
        std::snprintf(buf, sizeof(buf),
                      "WlanGetAvailableNetworkList failed (rc=%lu)", rc);
        r.error = buf;
        return r;
    }

    const std::string current = queryCurrentSsid(guard.h, iface);

    // De-duplicate by SSID — Windows reports separate entries for each profile
    // / signal strength sample sometimes. Keep best signal per SSID.
    std::vector<ScanEntry> all;
    all.reserve(list->dwNumberOfItems);
    for (DWORD i = 0; i < list->dwNumberOfItems; ++i) {
        const WLAN_AVAILABLE_NETWORK& n = list->Network[i];
        const std::string ssid = ssidToUtf8(n.dot11Ssid);
        if (ssid.empty()) continue;
        if (!ssid_prefix.empty() && ssid.rfind(ssid_prefix, 0) != 0) continue;

        ScanEntry e;
        e.ssid = ssid;
        e.signal_pct = static_cast<int>(n.wlanSignalQuality);
        e.secured = (n.dot11DefaultAuthAlgorithm != DOT11_AUTH_ALGO_80211_OPEN);
        e.connected = (!current.empty() && current == ssid);

        auto it = std::find_if(all.begin(), all.end(),
            [&](const ScanEntry& x){ return x.ssid == ssid; });
        if (it == all.end()) {
            all.push_back(std::move(e));
        } else if (e.signal_pct > it->signal_pct) {
            *it = std::move(e);
        }
    }

    std::sort(all.begin(), all.end(),
              [](const ScanEntry& a, const ScanEntry& b){
                  return a.signal_pct > b.signal_pct;
              });

    r.ok = true;
    r.entries = std::move(all);
    return r;
}

bool connect(const std::string& ssid, const std::string& password,
             std::string& error)
{
    WlanHandleGuard guard;
    if (!guard.open(error)) return false;

    GUID iface{};
    if (!firstUsableInterface(guard.h, iface, error)) return false;

    const std::wstring profile_xml  = buildProfileXml(ssid, password);
    const std::wstring profile_name = utf8ToWide(ssid);

    // Force-disconnect any current association first. Without this, if the
    // adapter is busy connecting to (or roaming to) a different SSID, our
    // WlanConnect request gets queued behind that and may silently never
    // fire. Symptom: user clicks Connect, status sits on "connecting to
    // Kitchen_Machine_4BB7 ..." forever, and the OS-level Wi-Fi state
    // stays "disconnected".
    WlanDisconnect(guard.h, &iface, nullptr);

    // Force a clean profile every time. Stale credentials in a previously
    // saved profile (e.g. wrong password from a prior firmware version)
    // can make Windows mark the network "failed" and refuse re-association.
    // WlanDeleteProfile of a non-existent profile is a no-op (returns 1168
    // ERROR_NOT_FOUND), which we ignore.
    WlanDeleteProfile(guard.h, &iface, profile_name.c_str(), nullptr);

    DWORD reason = 0;
    DWORD rc = WlanSetProfile(guard.h, &iface,
                              0, // dwFlags: per-user
                              profile_xml.c_str(),
                              nullptr, // szDescription
                              TRUE,    // bOverwrite (defensive — we deleted above)
                              nullptr,
                              &reason);
    if (rc != ERROR_SUCCESS) {
        char buf[120];
        std::snprintf(buf, sizeof(buf),
                      "WlanSetProfile failed (rc=%lu, reason=%lu)", rc, reason);
        error = buf;
        return false;
    }

    WLAN_CONNECTION_PARAMETERS params{};
    params.wlanConnectionMode = wlan_connection_mode_profile;
    params.strProfile         = profile_name.c_str();
    DOT11_SSID dot11{};
    dot11.uSSIDLength = static_cast<ULONG>(std::min<size_t>(ssid.size(), 32));
    std::memcpy(dot11.ucSSID, ssid.data(), dot11.uSSIDLength);
    params.pDot11Ssid         = &dot11;
    params.pDesiredBssidList  = nullptr;
    params.dot11BssType       = dot11_BSS_type_infrastructure;
    params.dwFlags            = 0;

    rc = WlanConnect(guard.h, &iface, &params, nullptr);
    if (rc != ERROR_SUCCESS) {
        char buf[80];
        std::snprintf(buf, sizeof(buf), "WlanConnect failed (rc=%lu)", rc);
        error = buf;
        return false;
    }
    return true;
}

std::string currentSsid()
{
    std::string ignored;
    WlanHandleGuard guard;
    if (!guard.open(ignored)) return {};
    GUID iface{};
    if (!firstUsableInterface(guard.h, iface, ignored)) return {};
    return queryCurrentSsid(guard.h, iface);
}

} // namespace drivescope::wlan

#endif // _WIN32
