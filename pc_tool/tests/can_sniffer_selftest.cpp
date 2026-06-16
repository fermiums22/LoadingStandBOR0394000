// Host self-test for src/sniffer/CanSniffer (Slice B).
//
//   g++ -std=c++20 -I src can_sniffer_selftest.cpp src/sniffer/CanSniffer.cpp -o /tmp/sniff && /tmp/sniff
//
// Validates: default-off (no env -> inert), active sink writes one JSONL line
// per event with the expected fields, malformed/empty frames don't crash, and
// decoded text is JSON-escaped.
#include "sniffer/CanSniffer.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

using namespace drivescope;

static int g_failures = 0;
static void check(const char* name, bool ok)
{
    std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_failures;
}

static CanFrame mkFrame(uint32_t id, std::vector<uint8_t> data, double ts)
{
    CanFrame f;
    f.id = id;
    f.extended = true;
    f.data = std::move(data);
    f.dlc = static_cast<uint8_t>(f.data.size());
    f.timestamp = ts;
    return f;
}

static std::string slurp(const std::string& path)
{
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

int main()
{
    const std::string path = "/tmp/ddv2_sniff_selftest.jsonl";
    std::remove(path.c_str());

    // 1) Default OFF: no env var -> inert, no crash, isActive() false.
#if defined(_WIN32)
    _putenv_s("DRIVESCOPE_SNIFF_JSONL", "");
#else
    unsetenv("DRIVESCOPE_SNIFF_JSONL");
#endif
    {
        CanSniffer off;
        check("default off: isActive() == false", !off.isActive());
        off.record(CanSniffer::Dir::Tx, "tcp", mkFrame(0x123u, {1, 2, 3}, 0.1), "ignored");
        // nothing should have been created at our path
        check("default off: no file written", slurp(path).empty());
    }

    // 2) Active: env points at a file -> events are logged.
    setenv("DRIVESCOPE_SNIFF_JSONL", path.c_str(), 1);
    {
        CanSniffer on;
        check("active: isActive() == true", on.isActive());
        on.record(CanSniffer::Dir::Tx, "tcp", mkFrame(0x10E00210u, {0x10, 0, 0, 0, 0, 0, 0, 0}, 1.5),
                  "CAP reset");
        // malformed / empty-data frame must not crash and still log.
        on.record(CanSniffer::Dir::Rx, "slcan", mkFrame(0x10E8FF02u, {}, 2.25),
                  "stream \"value\"\nline2");
    }

    const std::string body = slurp(path);
    // two lines
    std::size_t nl = 0;
    for (char c : body) if (c == '\n') ++nl;
    check("two JSONL lines written", nl == 2);

    auto has = [&](const char* s) { return body.find(s) != std::string::npos; };
    check("tx line: dir/id/transport", has("\"dir\":\"tx\"") && has("\"id\":\"0x10E00210\"") && has("\"transport\":\"tcp\""));
    check("tx line: len", has("\"len\":8"));
    check("tx line: data hex", has("\"data\":\"1000000000000000\""));
    check("rx line: len 0 (empty)", has("\"len\":0"));
    check("tx line: decoded", has("\"decoded\":\"CAP reset\""));
    check("rx line: dir/id/transport", has("\"dir\":\"rx\"") && has("\"id\":\"0x10E8FF02\"") && has("\"transport\":\"slcan\""));
    check("rx line: empty data ok", has("\"data\":\"\""));
    check("rx line: ext flag", has("\"ext\":true"));
    check("decoded JSON-escaped (quote + newline)", has("stream \\\"value\\\"\\nline2"));

    std::remove(path.c_str());
    std::printf("%s\n", g_failures ? "RESULT: FAIL" : "RESULT: PASS");
    return g_failures ? 1 : 0;
}
