// CalibMeta.h -- shared data types + spec constants for the Calibration tab.
//
// Part of the DriveScope Calibration tab redesign (docs/calibration-tab-redesign.md).
// Holds the per-field UI metadata table (units / ranges / blurbs), the
// per-field sync state, the result-row descriptor, the rail/section
// descriptors and the datasheet spec constants. Pure POD types + constexpr;
// no ImGui dependency so it can be included anywhere.

#pragma once

#include <cstdint>

namespace drivescope {

// --------------------------------------------------------------------------
// Datasheet / roadmap spec constants. Single source of truth for the
// profiler "expected Rs / Ld / Lq" band check AND the displayed values, so
// the band check and the label can never drift apart. Values from
// motor-tuning-roadmap.md (Rs 14.05 Ohm, L 25.23 mH).
// --------------------------------------------------------------------------
constexpr float kSpecRsOhm   = 14.0507f;     // expected stator resistance
constexpr float kSpecLdH     = 0.02522861f;  // expected d-axis inductance (H)
constexpr float kSpecLqH     = 0.02522861f;  // expected q-axis inductance (H)
constexpr float kSpecRsTolPct = 10.0f;       // advisory PASS/WARN band
constexpr float kSpecLTolPct  = 15.0f;

// --------------------------------------------------------------------------
// Per-field UI metadata. One row per editable motor-config param-table id.
// Numeric ranges mirror motor FW Core/Src/param_table.c min/max; defaults
// are the datasheet / compile-time targets.
// --------------------------------------------------------------------------
struct CalibParamMeta {
    uint16_t    id;
    const char* label;       // human label, e.g. "Iq loop Kp"
    const char* unit;        // "", "Ohm", "mH", "V*s/rad", ...
    float       uiMin;       // slider min
    float       uiMax;       // slider max
    float       uiDefault;   // datasheet / compile default (spec target)
    bool        isInteger;   // u32/bool params get an integer-stepped slider
    const char* blurb;       // one-sentence INAV-style description
};

// Calibration tab config grid (ids match kMotorConfigParams[] / param_table.c).
static const CalibParamMeta kCalibParamMeta[] = {
    { 10, "Id loop Kp",        "",         0.0f, 10000.0f, 85.0f,   false,
      "Proportional gain of the d-axis current PI. Higher = stiffer current tracking." },
    { 11, "Id loop Ki",        "",         0.0f, 10000.0f, 4.0f,    false,
      "Integral gain of the d-axis current PI (discrete, per 10 kHz tick)." },
    { 12, "Iq loop Kp",        "",         0.0f, 10000.0f, 85.0f,   false,
      "Proportional gain of the q-axis (torque) current PI." },
    { 13, "Iq loop Ki",        "",         0.0f, 10000.0f, 4.0f,    false,
      "Integral gain of the q-axis current PI. Weak Ki = slow disturbance rejection = microvibration." },
    { 14, "Speed loop Kp",     "",         0.0f, 100.0f,   0.01f,   false,
      "Proportional gain of the outer speed PI." },
    { 15, "Speed loop Ki",     "",         0.0f, 1.0f,     1e-6f,   false,
      "Integral gain of the outer speed PI." },
    { 20, "Stator resistance", "Ohm",      0.0f, 1000.0f,  14.0507f, false,
      "Per-phase winding resistance. Measured by Test 2." },
    { 21, "Inductance Ld",     "H",        0.0f, 1.0f,     0.02522861f, false,
      "d-axis inductance. Measured by Test 2." },
    { 22, "Inductance Lq",     "H",        0.0f, 1.0f,     0.02522861f, false,
      "q-axis inductance. Measured by Test 2." },
    { 23, "PM flux lambda",    "V*s/rad",  0.0f, 100.0f,   0.0f,    false,
      "Permanent-magnet flux linkage. Future Test 2 sub-step / Test 4 helper." },
    { 24, "Inertia J",         "kg*m^2",   0.0f, 10.0f,    0.0f,    false,
      "Rotor + load moment of inertia. Measured by Test 4." },
    { 25, "Friction B",        "N*m*s",    0.0f, 10.0f,    0.0f,    false,
      "Viscous friction coefficient. Measured by Test 4." },
    { 50, "PFC enabled",       "bool",     0.0f, 1.0f,     1.0f,    true,
      "Boot-default state of the external PFC enable line." },
};
constexpr int kCalibParamMetaCount =
    sizeof(kCalibParamMeta) / sizeof(kCalibParamMeta[0]);

// Profiler parameter metadata (the 6 fields of drawCalibMachine zone C).
// These are not param-table ids -- they are profiler limit slots -- so they
// are kept separate. Ranges chosen for safe bench use.
struct CalibProfilerParamMeta {
    const char* label;
    const char* unit;
    float       uiMin;
    float       uiMax;
    float       uiDefault;
    const char* blurb;
};

// --------------------------------------------------------------------------
// Per-field sync state -- drives the §5.1 sync chip and the §5.2 range field.
// One instance per editable id, kept in MainUi::calibSync_.
// --------------------------------------------------------------------------
enum class CalibSyncState {
    Synced,   // live value == last confirmed motor value
    Edited,   // local edit staged, not yet sent
    Sent,     // SET sent, awaiting ACK / blob refresh
    Acked,    // motor confirmed (flashes green, then -> Synced)
    Error,    // motor returned non-zero status / SET deadline elapsed
};

struct CalibFieldSync {
    CalibSyncState state    = CalibSyncState::Synced;
    float          staged   = 0.0f;   // edited value (mirrors slider + entry)
    float          sentValue = 0.0f;  // value at the time Set was pressed
    double         sentAt    = 0.0;   // ImGui::GetTime() of the Set
    double         ackedAt   = 0.0;   // when motor confirmed (for the green flash)
    bool           hasStaged = false; // staged initialised from a live value yet?
};

// --------------------------------------------------------------------------
// Result table row (§5.3). Used by Test 1 / 2 / 3 / 4 results.
// --------------------------------------------------------------------------
struct CalibResultRow {
    const char* label;
    bool        hasValue;
    double      value;
    const char* unit;
    bool        hasSpec;     // draw a spec band + verdict
    double      specNominal;
    double      specTolPct;  // +/- band, e.g. 10
};

// --------------------------------------------------------------------------
// Left-rail section result pill state (§2.1).
// --------------------------------------------------------------------------
enum class CalibPill {
    None,    // never run this session       ("--", grey)
    Running, // test in progress             ("RUN", amber)
    Pass,    // last run passed spec         ("PASS", green)
    Fail,    // last run failed / aborted    ("FAIL", red)
    Stale,   // config changed since last run("STALE", grey-amber)
};

} // namespace drivescope
