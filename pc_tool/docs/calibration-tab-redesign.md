# DriveScope — Calibration Tab Redesign (Design Spec)

**Status:** design proposal, ready for implementation.
**Author:** UI/UX design pass, 2026-05-15.
**Audience:** backend engineer implementing the C++/Dear ImGui code.
**Scope:** add a new top-level **Calibration** tab to DriveScope; migrate the
existing FOC theta calibration UI and the Motor Profiler UI into it; design a
compact live motor-config panel; lay out four well-organised test panels.

This document is the single source of truth for layout. The implementer should
not re-decide structure — only fill in glue code. Where the design has a
genuine fork, see [§7 Open questions](#7-open-questions--options).

---

## 1. Audit — what exists today and its problems

DriveScope (`tools/pc-tool`) is a Dear ImGui app. Tabs are dockable windows
created from `kDockTabs[]` in `MainUi::draw()` (`src/ui/MainUi.cpp:3383`):

```
Connection | CAN Monitor | Variables | Plots | Remote Control | Motor Config | Updates
```

Calibration-related UI is **scattered across two tabs**:

### 1.1 Theta (electrical angle) calibration — lives in the *wrong* tab

`MainUi::drawMotorTab()` (`MainUi.cpp:10207-10608`) is rendered inside
**Remote Control**. It is a 400-line monolith mixing four unrelated concerns:

1. Motor telemetry table (FSM state, RPM, V_dc, temps, fault mask).
2. Motor command controls (Speed slider, START/STOP/PARK/RESET ERR, PFC).
3. **FOC theta calibration** (`MainUi.cpp:10355-10507`): a status badge,
   `Align A` input, `Calibrate` / `Read calibration` / `Save All` buttons, and
   a 7-row LUT table (global offset + 6 sector slots, each with current rad /
   deg / delta-deg / editable rad / per-row Set).
4. Tone / melodies (the motor stator buzzes notes — pure novelty).

**Problems**
- Theta calibration — a *calibration* operation — is buried under a "Remote
  Control" label, three scroll-screens down, next to a melody player.
- No visible record of *what frame was sent* or *what ACK came back*. The only
  feedback is one `motorThetaTxStatus_` string and a `pending=N` counter.
- No description of the procedure. An operator who has never run it cannot tell
  what `Align A` does, what "global offset vs sector" means, or whether the
  motor shaft must be free.
- No progress indication during the alignment sweep — the motor energises a
  phase and the UI just shows a stale status until the next poll.

### 1.2 Motor Profiler + whole motor-config — the **Motor Config** tab

`MainUi::drawMotorConfig()` (`MainUi.cpp:12541-12693`) renders the
**Motor Config** tab. It contains:

- A 2-line identity header (`cfg.fw_ver`, `cfg.crc`).
- An action button row: `Read All`, `Auto poll`, `Save to flash`,
  `Load from flash`, `Reset defaults`, `Clear edits`.
- `drawMotorProfilerPanel()` (`MainUi.cpp:12372-12539`) — a `CollapsingHeader`
  with 6 numeric `InputFloat`s (test current / max voltage / bandwidth / pulse
  ms / settle ms / repeat), 6 action buttons (`Apply limits`, `Status`,
  `Read results`, `Abort`, `XX precheck`, `KZ Rs/Ld/Lq + PI`), a progress bar,
  and a 7-row result table (Rs / Ld / Lq / Kp_id / Ki_id / Kp_iq / Ki_iq).
- Four `CollapsingHeader` tables, one per group (THETA / PI / PROFILER / DIAG /
  HW), each a 5-column `BeginTable` (`id | name | current | edit | Set`) driven
  by `kMotorConfigParams[]` (`MainUi.cpp:12126`, 31 rows).

**Problems**
- It is a **full-page sprawl**: ~35 table rows + a profiler panel + 5
  collapsing headers stacked vertically. The user explicitly asked for the
  whole config "compact and clear, not the current full-page sprawl".
- Every field is `id | name | current | edit text-box | Set`. Pure raw
  param-table dump: no ranges, no units, no min/max guard, no slider, no
  description. `pi.kp_iq` is just a number; the operator has no idea the
  datasheet-derived target is ~157.
- "Profiler" is one collapsing header inside a config dump — it is actually
  **Test 2** of a calibration workflow and deserves its own panel.
- Theta rows (`theta.offset`, `theta.sector_0..5`) appear here **read-only**
  while the *editable* theta UI is in a different tab. Two half-views of the
  same data, in two tabs.
- Profiler "Expected Rs 14.05 / Ld 25.23 mH" is hardcoded in a `Text()` call
  (`MainUi.cpp:12381`) instead of being part of a pass/fail spec band.
- No "sent vs received" record anywhere. `Set` fires and you hope.

### 1.3 What is *good* and must be reused

- `motorBlob_` (164-byte whole-config blob) + `sendMotorBlobGet()` /
  `sendMotorBlobSet()` / `applyMotorConfigBlobSet(id, bits)` /
  `applyThetaSlotEdit(slot, rad)` — solid atomic config transport with CRC32
  verify. **Keep as the only config write path.**
- `motorBlobAuthoritativeUntil_` window (suppresses auto-poll stomping
  just-SET RAM values) — keep.
- `motorProfiler_` (`MotorProfilerView`) state struct + `sendMotorProfilerCommand`
  / `sendMotorProfilerLimits` / `requestMotorProfilerResults` — keep, rewrap.
- `kMotorConfigParams[]` + `motorConfigCache_` + `motorConfigGroupName()` —
  keep as the data model; only the *rendering* changes.
- The dockable-window tab mechanism (`kDockTabs[]`) — add one entry.

---

## 2. Overall layout of the new Calibration tab

A new top-level dockable tab **`Calibration`** is added to `kDockTabs[]`,
inserted between `Remote Control` and `Motor Config`:

```
Connection | CAN Monitor | Variables | Plots | Remote Control | Calibration | Motor Config | Updates
```

> Motor Config stays for now as the raw param-table escape hatch (power users,
> DIAG counters). See [§7 Q1](#q1-keep-or-fold-the-old-motor-config-tab).

The Calibration tab uses a **left rail of sub-sections** (INAV-style), a
**persistent compact config strip pinned at the top**, and a **content pane**
on the right. Only the content pane scrolls.

```
+========================================================================================+
|  Calibration                                                            [motor: v2.3.10]|
+----------------------------------------------------------------------------------------+
|  COMPACT CONFIG STRIP  (always visible, collapsible)                                    |
|  Theta  off +0.0123 | PI iq Kp 85.0 Ki 4.0  w Kp 0.010 Ki 1e-6 | RsLdLq 14.0/25/25      |
|  [Read All]  [Save to flash]  [Revert]      sync: 3 fields edited (not on motor)         |
+--------------------+-------------------------------------------------------------------+
|  LEFT RAIL         |  CONTENT PANE  (selected section)                                  |
|                    |                                                                   |
|  > Overview        |   ....................................................            |
|  > Test 1  Theta   |   ....................................................            |
|  > Test 2  Machine |   ....................................................            |
|  > Test 3  PI/Step |   ....................................................            |
|  > Test 4  Inertia |   ....................................................            |
|  - - - - - - - - - |   ....................................................            |
|  > Config (full)   |   ....................................................            |
|                    |                                                                   |
|  TRANSACTION LOG   |                                                                   |
|  (docked bottom-   |   ....................................................            |
|   left, see §5.1)  |   ....................................................            |
+--------------------+-------------------------------------------------------------------+
```

### 2.1 Left rail (`drawCalibrationRail()`)

A fixed-width (`190 px`) child window. Each entry is a selectable row with an
icon, a title, and a small **state pill** showing test result at a glance:

| Pill        | Colour                 | Meaning                              |
|-------------|------------------------|--------------------------------------|
| `--`        | grey                   | never run this session               |
| `RUN`       | amber, pulsing         | test in progress                     |
| `PASS`      | green                  | last run passed spec                 |
| `FAIL`      | red                    | last run failed spec / aborted       |
| `STALE`     | grey-amber             | config changed since last run        |

Rail entries:

1. **Overview** — a dashboard: connection state, motor identity, fault mask
   decode, a checklist of the 4 tests with their pills, and a "recommended
   order" hint (Theta → Machine → PI → Inertia).
2. **Test 1 — Theta calibration**
3. **Test 2 — Machine parameters** (Rs / Ld / Lq)
4. **Test 3 — PI tuning + step response**
5. **Test 4 — Inertia (J) + friction**
6. **Config (full)** — the compact full-config editor (§3).

Selection index lives in `int calibSection_ = 0;`. Switching section does not
stop a running test (tests own their own state machines).

### 2.2 Content pane

Renders the selected section via a dispatch table mirroring `kDockTabs`:

```cpp
struct CalibSectionDef { const char* label; ToolIcon icon; void (MainUi::*body)(); };
static const CalibSectionDef kCalibSections[] = {
    { "Overview",                 ToolIcon::Info,    &MainUi::drawCalibOverview   },
    { "Test 1 - Theta",           ToolIcon::Angle,   &MainUi::drawCalibTheta      },
    { "Test 2 - Machine params",  ToolIcon::Chip,    &MainUi::drawCalibMachine    },
    { "Test 3 - PI + step",       ToolIcon::Wave,    &MainUi::drawCalibPiTuning   },
    { "Test 4 - Inertia",         ToolIcon::Inertia, &MainUi::drawCalibInertia    },
    { "Config (full)",            ToolIcon::Table,   &MainUi::drawCalibConfigFull },
};
```

### 2.3 Common gating

Every section first calls a shared guard:

```cpp
bool MainUi::calibPreconditions(CalibGate& g);   // fills g.canSend, g.inDiag, g.motorIdle, g.faultMask
```

- `canSend`  = `activeTransport_ && activeTransport_->isOpen()`
- `inDiag`   = `lastDiagStatus_.valid ? lastDiagStatus_.diag_active : diagModeOn_`
- `motorIdle`= FSM state not RUN/PARKING and `motorProfiler_.state != RUNNING`
- `faultMask`= `lastMotorTelemetry_.fault_mask`

If not connected: render a single centred amber line "Connect first
(Connection tab)" and return — same pattern as `drawMotorConfig()`.

A test's **Start** button is disabled unless its specific precondition row (a
checklist, see §4) is fully green.

---

## 3. Compact motor-config panel

The whole config must be visible, live, and editable, but dense. Two places
use it: the **top config strip** (read-only summary) and the
**Config (full)** section (editable grid).

### 3.1 Data model (unchanged)

Source of truth: `motorBlob_` (164 B) for persistent fields + `motorConfigCache_`
for DIAG runtime counters. Render from `kMotorConfigParams[]`. **No new
transport** — write goes through `applyMotorConfigBlobSet()` /
`applyThetaSlotEdit()`; bulk read through `sendMotorBlobGet()`.

Add a per-field UI metadata table (new, static, file-scope) so the grid can
show units, ranges, and descriptions. The numeric ranges mirror motor FW
`Core/Src/param_table.c`:

```cpp
struct CalibParamMeta {
    uint16_t    id;
    const char* label;       // human label, e.g. "Iq loop Kp"
    const char* unit;        // "", "Ohm", "mH", "V*s/rad", ...
    float       uiMin;       // slider min  (from param_table.c min_value)
    float       uiMax;       // slider max
    float       uiDefault;   // datasheet/compile default
    const char* blurb;       // one-sentence description (INAV-style)
};
```

Concrete values derived from `param_table.c` and `motor-tuning-roadmap.md`:

| id | label              | unit      | min | max    | default        | blurb |
|----|--------------------|-----------|-----|--------|----------------|-------|
| 10 | Id loop Kp         | —         | 0   | 10000  | 85 (calc ~157) | Proportional gain of the d-axis current PI. Higher = stiffer current tracking. |
| 11 | Id loop Ki         | —         | 0   | 10000  | 4 (calc ~8.8)  | Integral gain of the d-axis current PI (discrete, per 10 kHz tick). |
| 12 | Iq loop Kp         | —         | 0   | 10000  | 85 (calc ~157) | Proportional gain of the q-axis (torque) current PI. |
| 13 | Iq loop Ki         | —         | 0   | 10000  | 4 (calc ~8.8)  | Integral gain of the q-axis current PI. Weak Ki = slow disturbance rejection = microvibration. |
| 14 | Speed loop Kp      | —         | 0   | 100    | 0.01           | Proportional gain of the outer speed PI. |
| 15 | Speed loop Ki      | —         | 0   | 1      | 1e-6           | Integral gain of the outer speed PI. |
| 20 | Stator resistance  | Ohm       | 0   | 1000   | 0 (ds 14.05)   | Per-phase winding resistance. Measured by Test 2. |
| 21 | Inductance Ld      | H         | 0   | 1      | 0 (ds 0.02523) | d-axis inductance. Measured by Test 2. |
| 22 | Inductance Lq      | H         | 0   | 1      | 0 (ds 0.02523) | q-axis inductance. Measured by Test 2. |
| 23 | PM flux lambda     | V*s/rad   | 0   | 100    | 0 (ds ~0.538)  | Permanent-magnet flux linkage. Future Test 2 sub-step. |
| 24 | Inertia J          | kg*m^2    | 0   | 10     | 0              | Rotor + load moment of inertia. Measured by Test 4. |
| 25 | Friction B         | N*m*s     | 0   | 10     | 0              | Viscous friction coefficient. Measured by Test 4. |
| 50 | PFC enabled        | bool      | 0   | 1      | 1              | Boot-default state of the external PFC enable line. |

`theta.offset` + `theta.sector_0..5` (ids 0..6) are shown in the config grid
**read-only** with a "→ Test 1" link button; they are *edited* exclusively in
Test 1. DIAG ids 30..38 stay read-only counters.

### 3.2 Compact config strip (top, always visible)

A single `CollapsingHeader("Motor configuration", DefaultOpen)` pinned above
the rail+content split. One dense row of grouped read-outs + 3 buttons.

```
+----------------------------------------------------------------------------------------+
| v Motor configuration                              motor v2.3.10   cfg.crc 0x9AF2C118   |
|  THETA   off +0.01234 rad   sectors 6/6 cal       PI  Id 85.0/4.0   Iq 85.0/4.0          |
|  w 0.0100/1.0e-6           MACHINE  Rs 14.05  Ld 25.2mH  Lq 25.2mH  lam --   PFC [ON]    |
|                                                                                         |
|  [ Read All ]  [ Save to flash ]  [ Revert RAM ]      o 2 fields edited - not on motor   |
+----------------------------------------------------------------------------------------+
```

- Values pull from `motorBlob_`. Each group (THETA / PI / MACHINE) is one
  colour-tinted segment.
- A field that has a **staged local edit not yet sent** renders in cyan; a
  field **sent but not yet ACKed** renders amber; a field **confirmed by a
  fresh blob GET** renders normal white. This is the global "sent vs received"
  rule (see §5.1).
- `Read All` = `sendMotorBlobGet()` + clears authoritative lock.
- `Save to flash` = `sendMotorConfigParam(SAVE_ALL,0,0)`; disabled unless
  `motorIdle` — show tooltip "motor must be idle, flash erase blocks bus ~1.5 s".
- `Revert RAM` = `sendMotorConfigParam(LOAD_ALL,0,0)`.
- Right-side status text shows the unsaved-edit count.

### 3.3 Config (full) section — the editable dense grid

Replaces the 5 stacked tables. One `BeginTable` with **collapsible group
rows** instead of separate tables, and a fixed compact 6-column layout:

```
+ Group: PI current/speed loop -----------------------------------------------------------+
| field            | live value      | edit                              | spec  | sync   |
|------------------|-----------------|-----------------------------------|-------|--------|
| Id loop Kp       | 85.000          | [=====O-------]  85.0   [Set]      | 157   |  ok    |
| Id loop Ki       | 4.000           | [==O----------]   4.0   [Set]      | 8.8   |  edited|
| Iq loop Kp       | 85.000          | [=====O-------]  85.0   [Set]      | 157   |  sent  |
| Iq loop Ki       | 4.000           | [==O----------]   4.0   [Set]      | 8.8   |  ok    |
| Speed loop Kp    | 0.010000        | [===O---------]  0.010  [Set]      | --    |  ok    |
| Speed loop Ki    | 0.000001        | [O------------]  1e-6   [Set]      | --    |  ok    |
+ Group: MACHINE params (measured by Test 2) ---------------------------------------------+
| Stator resistance| 14.0507 Ohm     | [==O----------]  14.05  [Set]      | 14.05 |  ok    |
| ...                                                                                     |
+ Group: HW --------- + Group: DIAG counters (read-only) --------------------------------+
```

Column spec:

1. **field** — `CalibParamMeta.label`; hover shows `blurb` in a tooltip.
2. **live value** — current `motorBlob_` value formatted with `unit`. This is
   the "received" side.
3. **edit** — the **range-slider+entry component** (§5.2): a slider over
   `[uiMin,uiMax]` plus a numeric box, plus a small `Set` button. `Set` calls
   `applyMotorConfigBlobSet(id, bits)`.
4. **spec** — `uiDefault` / datasheet target, grey; `--` if none.
5. **sync** — the per-field sent/received chip (§5.1): `ok` / `edited` /
   `sent` / `ACK` / `err`.

Group header rows are `ImGuiTreeNodeFlags_SpanAllColumns` collapsible rows so
the operator can collapse DIAG when not needed. Default: PI + MACHINE open,
DIAG/HW collapsed.

Buttons under the grid: `Read All`, `Save to flash`, `Revert RAM`,
`Reset defaults`, `Clear staged edits`. Same handlers as today.

---

## 4. The four test panels

Every test panel follows the **same five-zone template** so the operator
learns one layout:

```
+-- TEST PANEL TEMPLATE -----------------------------------------------------+
| (A) HEADER   title, one-paragraph purpose, current result pill            |
+---------------------------------------------------------------------------+
| (B) PRECONDITION CHECKLIST   green/red rows; Start disabled until all green|
+---------------------------------------------------------------------------+
| (C) PARAMETERS   range-slider+entry rows (§5.2), each with a blurb         |
+---------------------------------------------------------------------------+
| (D) RUN STRIP   [Start] [Abort]  progress bar  current-action line        |
|     live transaction log tail (last 4 sent/recv frames, §5.1)             |
+---------------------------------------------------------------------------+
| (E) RESULTS   numbers vs spec band (pass/fail), plot, [Apply] [Save flash] |
+---------------------------------------------------------------------------+
```

Implemented as a reusable helper `beginTestPanel(...)/endTestPanel()` plus the
shared components in §5. Each `drawCalib*` body fills the five zones.

---

### 4.1 Test 1 — Theta (electrical-angle) calibration

**Migrated from** `drawMotorTab()` lines `10355-10507`. Cut that block out of
`drawMotorTab()` and re-home it as `drawCalibTheta()`. The melody player and
the motor telemetry table stay in Remote Control; only the theta block moves.

**(A) Purpose.** "Calibrates the offset between the Hall sensor reference and
the motor's electrical zero, so field-oriented control commutates correctly.
Runs a controlled current vector that pulls the rotor to a known angle, then
records the measured offset — globally and per 60° Hall sector."

**(B) Preconditions**
- Transport open.
- Motor in diag mode (offer `Take control` button if not).
- FSM idle, `fault_mask & 0x1FF == 0`.
- "Shaft is free to rotate" — a manual operator confirm checkbox (the rotor
  *will* jump to the alignment angle).

**(C) Parameters** (range-slider+entry)
- `Align current` — `0.05 .. 1.00 A`, default `0.30`. Blurb: "Current used to
  pull the rotor to the alignment angle. Higher = firmer pull, more heat."
- `Save result after calibration` — checkbox. If on, uses `kMotorThetaCalSave`
  instead of `kMotorThetaCal`.

**(D) Run**
- `Start` → `sendMotorThetaConfig(action, alignCurrent)`
  (`action = save ? kMotorThetaCalSave : kMotorThetaCal`).
- During the sweep show an indeterminate progress bar (theta cal has no %
  telemetry — see §7 Q3) and a current-action line driven by the theta status
  enum (`motorThetaStatusName()`), e.g. "aligning rotor…", "measuring sector
  3/6…", "done".
- Transaction-log tail shows the CFG 0x00 frame and the 0x0AB ACKs.

**(E) Results** — the existing 7-row LUT table, restyled as the §5.3 results
component:
- Rows: `global offset` + `sector 0..5`. Columns: `current rad` / `deg` /
  `delta vs uniform 60°` / `editable rad` / `Set`.
- A **dial plot**: a polar/clock widget drawing the 6 sector offsets as ticks
  around a circle, so a skewed LUT is visible at a glance (a sector pointing
  far off its nominal 60° centre stands out).
- Pass/fail band: each sector's `delta vs uniform` should be within ±15°
  (advisory). Global offset has no spec — informational.
- `Set` per row → `applyThetaSlotEdit(slot, rad)`. `Save All` →
  `sendMotorConfigParam(SAVE_ALL,0,0)` + drop authoritative lock.

**CAN / code used**
- Request: `MTR_CMD_CONFIG (0x005)` subcmd `APP_MOTOR_CFG_THETA_OFFSET (0x00)`,
  actions `APP_MOTOR_THETA_CAL (3)` / `CAL_SAVE (4)` / `GET (0)` /
  `GET_SECTOR (7)`.
- Response: `MTR_ANS_CONFIG (0x0AB)`.
- Existing helpers: `sendMotorThetaConfig()`, `readAllThetaCalibration()`,
  `applyThetaSlotEdit()`, `maybePollMotorTheta()`, `lastMotorTelemetry_.theta_*`.
  Blob offsets `kMotorBlobOffThetaOffset` / `kMotorBlobOffThetaSectorLut`.

```
+-- Test 1 - Theta calibration ------------------------------- result: PASS -+
| Calibrates the Hall-to-electrical-zero offset so FOC commutates correctly. |
+----------------------------------------------------------------------------+
| Preconditions   [x] transport open   [x] diag mode   [x] no PWM fault      |
|                 [ ] shaft free to rotate  <- operator must confirm         |
+----------------------------------------------------------------------------+
| Align current  [====O--------]  0.30 A     pull strength, more = hotter    |
| [x] Save result to flash after calibration                                |
+----------------------------------------------------------------------------+
| [ Start calibration ]  [ Abort ]   ( aligning rotor... )                   |
|  >> 0x005 sub=00 act=03  d=[..]      << 0x0AB st=00 (OK)                    |
+----------------------------------------------------------------------------+
| slot           | rad      | deg     | d-deg   | edit rad        | Set       |
| global offset  | +0.01234 |  +0.707 |  n/a    | [ 0.01234   ]   | [Set]     |
| sector 0       | +0.51000 | +29.220 |  -0.78  | [ 0.51000   ]   | [Set]     |
| ...                                                                        |
|        ( sector dial plot )           [ Save All to flash ]                |
+----------------------------------------------------------------------------+
```

---

### 4.2 Test 2 — Machine parameter calibration (Profiler: Rs / Ld / Lq)

**Migrated from** `drawMotorProfilerPanel()` (`MainUi.cpp:12372-12539`). Cut it
out of `drawMotorConfig()` and re-home as `drawCalibMachine()`, restyled into
the five-zone template.

**(A) Purpose.** "Auto-identifies the motor's electrical parameters — stator
resistance Rs, and inductances Ld/Lq — by injecting controlled test signals
with the rotor locked, then derives recommended current-loop PI gains. Two
procedures: a no-load pre-check (ХХ, free shaft) that validates wiring and a
locked-rotor measurement (КЗ) that does the actual identification."

**(B) Preconditions**
- Transport open.
- Diag mode active (`Take control` offered).
- `motorIdle`, `fault_mask & 0x1FF == 0`.
- Procedure-specific: for КЗ a manual confirm "rotor is mechanically locked"
  (the operator's locking fixture, per roadmap §arch-decision 6).

**(C) Parameters** (range-slider+entry; replaces the 6 bare `InputFloat`s).
Defaults from current code; ranges chosen for safe bench use:

| param           | unit | min  | max  | default | blurb |
|-----------------|------|------|------|---------|-------|
| Test current    | A    | 0.05 | 3.0  | 0.50    | DC/AC amplitude of the identification signal. |
| Max voltage     | %    | 1.0  | 25.0 | 5.0     | Voltage ceiling as % of DC bus; safety clamp. |
| PI bandwidth    | Hz   | 50   | 2000 | 300     | Target current-loop bandwidth used to derive Kp/Ki. |
| Pulse length    | ms   | 1    | 200  | 20      | Duration of each injection pulse. |
| Settle time     | ms   | 1    | 200  | 20      | Idle gap between pulses for current decay. |
| Repeat count    | #    | 1    | 16   | 4       | Number of pulses averaged per parameter. |

**(D) Run** — two distinct primary buttons (procedures), not one:
- `Run XX pre-check` → `sendMotorProfilerLimits()` +
  `sendMotorProfilerCommand(START, NOLOAD_PRECHECK=1, 0)`.
- `Run КЗ identification` → `sendMotorProfilerLimits()` +
  `sendMotorProfilerCommand(START, BLOCKED_RS_L_PI=2, 0)`.
- `Abort` → `sendMotorProfilerCommand(ABORT,0,0)`.
- Progress bar from `motorProfiler_.progress`; current-action line from
  `motorProfilerStateName()` + `motorProfilerTestName()`.
- While `state == RUNNING`, poll `GET_STATUS` every 250 ms (existing logic).
  On `state == DONE`, auto `requestMotorProfilerResults()` + `sendMotorBlobGet()`.
- Transaction-log tail shows the CFG 0x04 frames + 0x0AC telemetry.

**(E) Results** — restyle the 7-row table with **spec bands and pass/fail**:

| result | measured     | spec band (datasheet ±tol) | verdict |
|--------|--------------|-----------------------------|---------|
| Rs     | 14.02 Ohm    | 14.05 ± 10 %                | PASS    |
| Ld     | 25.6 mH      | 25.23 mH ± 15 %             | PASS    |
| Lq     | 25.1 mH      | 25.23 mH ± 15 %             | PASS    |
| Kp Id  | 156.0        | derived                     | —       |
| Ki Id  | 8.7          | derived                     | —       |
| Kp Iq  | 156.0        | derived                     | —       |
| Ki Iq  | 8.7          | derived                     | —       |

Spec constants from `motor-tuning-roadmap.md` (Rs 14.05 Ω, L 25.23 mH); move
the hardcoded `14.0507f`/`25.22861f` into named `constexpr` so the band check
and the display share them. Tolerances are advisory (PASS/WARN, never blocks).

Results actions:
- `Apply PI gains` — write Kp_id/Ki_id/Kp_iq/Ki_iq into `motorBlob_` via
  `applyMotorConfigBlobSet()` (RAM only). The motor FW already stores them in
  `profiler.*`; "Apply" copies the derived current-loop gains into the live
  `pi.*` ids so the operator can A/B them in Test 3.
- `Save to flash` — `sendMotorConfigParam(SAVE_ALL,0,0)`.

> Per-test plot is **optional** here — Test 2 is mostly scalar identification.
> If the firmware streams identification signals via Triggered Capture
> (roadmap §Phase 6 "each sub-step logs via Triggered Capture"), surface a
> "View signals in Plots" link. Not required for v1.

**CAN / code used**
- Request: `MTR_CMD_CONFIG (0x005)` subcmd `APP_MOTOR_CFG_PROFILER (0x04)`,
  actions `GET_STATUS / SET_LIMIT / START / ABORT / GET_RESULT`
  (`AppMotorProfilerAction_e`). Limits `AppMotorProfilerLimit_e`, tests
  `AppMotorProfilerTest_e`, results `AppMotorProfilerResult_e`.
- Telemetry: `MTR_ANS_PROFILER (0x0AC)` —
  `[state, test, progress, err|sel<<4, value_f32]`.
- Existing helpers: `sendMotorProfilerCommand()`, `sendMotorProfilerLimits()`,
  `requestMotorProfilerResults()`, `motorProfiler_` view, the 6
  `motorProfiler*` parameter fields.

```
+-- Test 2 - Machine parameters ------------------------------ result: PASS -+
| Auto-identifies Rs/Ld/Lq by injecting test signals with the rotor locked, |
| and derives recommended current-loop PI gains.                            |
+----------------------------------------------------------------------------+
| Preconditions  [x] transport  [x] diag mode  [x] no PWM fault             |
|                [ ] rotor mechanically locked  <- required for КЗ          |
+----------------------------------------------------------------------------+
| Test current  [=O----------]  0.50 A    PI bandwidth [==O------] 300 Hz   |
| Max voltage   [=O----------]  5.0 %     Pulse  [==O----] 20 ms             |
| Settle [==O----] 20 ms                  Repeat [===O---] 4                 |
+----------------------------------------------------------------------------+
| [ Run XX pre-check ]  [ Run КЗ identification ]  [ Abort ]                 |
| state RUNNING  test BLOCKED_RS_L_PI   [#######-------] 58%                 |
|  >> 0x005 sub=04 act=02 test=02       << 0x0AC st=01 prog=58               |
+----------------------------------------------------------------------------+
| result | measured   | spec               | verdict                        |
| Rs     | 14.02 Ohm  | 14.05 +/-10%        | PASS                           |
| Ld     | 25.6  mH   | 25.23 mH +/-15%     | PASS                           |
| Lq     | 25.1  mH   | 25.23 mH +/-15%     | PASS                           |
| ...                                                                        |
| [ Apply PI gains -> Test 3 ]   [ Save to flash ]                           |
+----------------------------------------------------------------------------+
```

---

### 4.3 Test 3 — PI regulator tuning + step response

**New panel** (`drawCalibPiTuning()`). No existing UI; built on existing
transport. This is the panel that directly attacks the microvibration goal
(roadmap "main candidate: PI gains under-set ~2×").

**(A) Purpose.** "Tunes the current-loop and speed-loop PI gains and verifies
them with a step-response test. Apply a step set-point, capture the response,
and read back rise time, overshoot and settling time. Lets you A/B gain sets
without re-flashing firmware."

**(B) Preconditions**
- Transport open, diag mode active.
- For the step test that actually spins/torques the motor: `motorIdle` before
  start; a manual confirm "safe to energise / shaft area clear".
- `fault_mask & 0x1FF == 0`.

**(C) Parameters** — two grouped range-slider sets, each with the live value:

*Current loop (Id / Iq)*
- `Iq loop Kp` — `0 .. 10000`, default 85 (calc target 157). Same row also
  shows the live `motorBlob_` value.
- `Iq loop Ki` — `0 .. 10000`, default 4 (calc 8.8).
- `Id loop Kp` / `Id loop Ki` — same ranges.
- Helper button `Compute from Test 2` — fills the four boxes using
  `Kp = bandwidth * L`, `Ki = bandwidth * Rs / 10000` (roadmap formulas) from
  the latest `profiler.*` values. Disabled if Test 2 never ran.

*Speed loop*
- `Speed loop Kp` — `0 .. 100`, default 0.01.
- `Speed loop Ki` — `0 .. 1`, default 1e-6.

*Step test config*
- `Loop under test` — radio: `Iq current step` / `Speed step`.
- `Step from / to` — two entries (units: A for current, rpm for speed).
- `Step duration` — `50 .. 2000 ms`, default 500.
- `Capture rate` — `100 .. 1000 us`, default 200 (drives Triggered Capture).

Each gain row has its own `Set` (writes RAM via `applyMotorConfigBlobSet()`),
so the operator can nudge a gain and immediately re-run the step.

**(D) Run**
- `Apply gains` — pushes all six gain fields to RAM (one blob SET).
- `Run step test`:
  1. Arm Triggered Capture (existing infra, CAN `0x0E0/0x0E1`) on the relevant
     signals: for the current loop — `i_q_ref`, `i_q_meas`, `u_q`; for the
     speed loop — `omega_ref`, `omega_meas`, `i_q_meas`. The variable set is
     marked the same way the Plots tab marks capture variables — Test 3 stages
     these programmatically.
  2. Command the step (set-point low → high) via the motor proxy / CTRL path.
  3. Wait for the capture burst, pull it (same `pollCaptureSession` flow), and
     hand the samples to the §5.3 plot component.
- Progress bar driven by the capture state machine
  (`Idle→Arm→WaitDone→ReadChunk→Apply`).
- `Abort` — stop motor + abort capture.

> **Reuse:** the whole step-response acquisition is *the existing Triggered
> Capture pipeline*. Test 3 does not invent a transport — it scripts a
> capture + a set-point step. See [§7 Q4](#q4-step-response-acquisition).

**(E) Results**
- A plot: set-point vs measured, the step instant marked with the trigger
  inf-line (reuse the `ImPlot::PlotInfLines` marker style from Triggered
  Capture).
- Derived metrics row, computed host-side from the captured samples:
  `rise time (10-90%)`, `overshoot %`, `settling time (±2%)`,
  `steady-state error`.
- Advisory verdict: e.g. overshoot < 20 % and settling < 5 ms (current loop)
  → PASS; tune hint text otherwise ("overshoot high — reduce Kp or raise Ki").
- `Save to flash` — `sendMotorConfigParam(SAVE_ALL,0,0)`.
- A small **A/B memory**: store up to 3 gain-set + metrics snapshots this
  session so the operator can compare "before vs after" numerically.

**CAN / code used**
- PI gains: `applyMotorConfigBlobSet()` for ids 10..15 (RAM), `SAVE_ALL` for
  flash. Per-field live tuning may also use `sendMotorConfigParam(SET,id,bits)`.
- Step capture: Triggered Capture `0x0E0/0x0E1` via `DebugProtocol.*` and the
  `pollCaptureSession` / `applyCapturedSamples` machinery.
- Step command: existing motor CTRL path (`MTR_CMD_CTRL 0x002`,
  `sendMotorProxy`).

```
+-- Test 3 - PI tuning + step response ----------------------- result: WARN -+
| Tune current/speed-loop PI gains; verify with a step response.            |
+----------------------------------------------------------------------------+
| Preconditions  [x] transport  [x] diag mode  [x] no PWM fault  [ ] area clear|
+----------------------------------------------------------------------------+
| CURRENT LOOP   [ Compute from Test 2 ]                                     |
|  Iq Kp [=====O-----] 85.0  (live 85.0)  [Set]   target 157                 |
|  Iq Ki [==O--------]  4.0  (live  4.0)  [Set]   target 8.8                 |
|  Id Kp ...   Id Ki ...                                                     |
| SPEED LOOP   w Kp [===O----] 0.010 [Set]   w Ki [O-------] 1e-6 [Set]      |
| STEP   loop (o) Iq current ( ) Speed     from [0.0] to [1.0] A             |
|        duration [==O----] 500 ms     capture rate [=O----] 200 us          |
+----------------------------------------------------------------------------+
| [ Apply gains ]  [ Run step test ]  [ Abort ]   ( capturing... 70% )       |
+----------------------------------------------------------------------------+
|  ( step response plot: i_q_ref vs i_q_meas, trigger line )                 |
|  rise 1.8 ms | overshoot 24% | settling 6.1 ms | ss-err 0.3%   verdict WARN |
|  hint: overshoot high - lower Kp or raise Ki                               |
|  [ Save to flash ]   A/B: [snap 1] [snap 2] [snap 3]                       |
+----------------------------------------------------------------------------+
```

---

### 4.4 Test 4 — Inertia (J) calibration + friction + future

**New panel** (`drawCalibInertia()`). The motor FW already reserves
`profiler.j` (id 24) and `profiler.b` (id 25) and the param-table has min/max
for them; the *measurement procedure* is roadmap **Phase 6+ "lambda_pm,
inertia/friction"** — still to be firmware-implemented (`motor-tuning-roadmap.md`
§Phase plan: `lambda_pm`, inertia/friction "остаются следующим этапом").

So Test 4 ships in **two tiers**:

**Tier A — available now (manual / derived):**
- Editable `Inertia J` and `Friction B` range-slider rows (ids 24, 25) so a
  known value can be entered and saved (`applyMotorConfigBlobSet` → `SAVE_ALL`).
- A **coast-down estimator**: spin the motor to a set speed, command STOP,
  capture `omega_meas` via Triggered Capture, and host-side fit the decay
  `J·dω/dt = −B·ω` to estimate J and B. This needs *no new firmware* — it
  reuses the capture pipeline + the existing speed command. Show the decay
  curve in the §5.3 plot and the fitted J / B with a confidence note.
- A **lambda_pm helper**: with the motor in speed mode and I_d = 0, read back
  steady-state V_q and ω, compute `lambda = (V_q − Rs·I_q)/omega_elec` (roadmap
  Phase 6 step 4). Surface it as a one-click derived value writing id 23.

**Tier B — firmware-pending (greyed, labelled "requires motor FW Phase 6+"):**
- `Run J/B identification` and `Run lambda_pm` buttons that would call a future
  profiler test enum (`PROF_TEST_*` extension). Render disabled with a tooltip
  "needs motor FW profiler test for inertia (roadmap Phase 6)". This keeps the
  panel forward-compatible without dead UI confusion.

**(A) Purpose.** "Determines the rotor + load moment of inertia J and viscous
friction B — needed to tune the outer speed loop. Provides a coast-down
estimate now; a firmware-side identification routine is planned."

**(B) Preconditions** — transport, diag mode, `motorIdle`, no PWM fault, a
manual "safe to spin the motor" confirm (coast-down spins it up).

**(C) Parameters**
- `Coast-down start speed` — `100 .. 800 rpm`, default 300.
- `Capture rate` — `200 .. 1000 us`, default 500.
- `Inertia J` editable row (id 24) — `0 .. 10 kg*m^2`.
- `Friction B` editable row (id 25) — `0 .. 10 N*m*s`.
- `PM flux lambda` editable row (id 23) — `0 .. 100 V*s/rad`.

**(D) Run** — `Run coast-down`: spin up → STOP → capture ω decay →
host-side fit. Progress from the capture state machine. `Abort` stops + aborts.

**(E) Results** — decay-curve plot, fitted `J` and `B` with a fit-quality
(R²) note, `Apply to RAM` (writes ids 24/25), `Save to flash`.

**CAN / code used**
- Speed command + STOP: existing motor CTRL path.
- ω capture: Triggered Capture `0x0E0/0x0E1`.
- Param write: `applyMotorConfigBlobSet()` ids 23/24/25, `SAVE_ALL` for flash.
- Tier B placeholder: future `APP_MOTOR_CFG_PROFILER` test enum (not yet in
  `app_motor.h` — leave a `// TODO motor FW Phase 6` marker).

```
+-- Test 4 - Inertia (J) + friction --------------------------- result: -- --+
| Determines rotor+load inertia J and viscous friction B for speed-loop tune.|
| Coast-down estimate available now; firmware identification planned.        |
+----------------------------------------------------------------------------+
| Preconditions  [x] transport [x] diag [x] no fault  [ ] safe to spin       |
+----------------------------------------------------------------------------+
| Coast-down start speed [===O----] 300 rpm   capture rate [==O--] 500 us    |
| Inertia J  [O----------] 0.0000 kg*m^2  [Set]                              |
| Friction B [O----------] 0.0000 N*m*s   [Set]                              |
| PM flux    [O----------] 0.0000 V*s/rad [Set]   [ Estimate lambda_pm ]     |
+----------------------------------------------------------------------------+
| [ Run coast-down ]  [ Abort ]    ( spinning up... )                        |
| Tier B (motor FW Phase 6+):  [ Run J/B identification ]  (disabled)        |
+----------------------------------------------------------------------------+
|  ( omega coast-down decay plot + fitted exponential )                      |
|  fitted  J = 0.0042 kg*m^2   B = 0.0011 N*m*s   fit R2 = 0.991             |
|  [ Apply to RAM ]   [ Save to flash ]                                      |
+----------------------------------------------------------------------------+
```

---

## 5. Shared UX patterns / reusable components

Build these **once**; all panels and the config grid use them.

### 5.1 Transaction log + sent/received indicator

The user explicitly wants to see *what was sent* and *what was received/ACKed*.

**Component A — `CalibTxLog`** (a ring buffer + a renderer).

```cpp
struct CalibTxEntry {
    double      t;            // ImGui::GetTime() when logged
    bool        outbound;     // true = TX, false = RX
    uint32_t    canId;
    uint8_t     dlc;
    uint8_t     data[8];
    std::string note;         // decoded summary, e.g. "CFG PARAM SET id=12 -> 157.0"
    enum Kind { Sent, Ack, Nack, Timeout, Info } kind;
};
class CalibTxLog {
    void push(const CalibTxEntry&);
    void render(int maxRows);          // full panel (docked bottom-left, §2)
    void renderTail(int n);            // compact last-n, used in each test's zone D
    std::deque<CalibTxEntry> entries_; // cap ~400
};
```

- Every calibration `send*()` call also pushes a `Sent` entry; every relevant
  RX frame (`0x0AB`, `0x0AC`, capture frames) pushes an `Ack`/`Nack`/`Info`.
- Rendering: TX rows have a `>>` glyph and a cyan-ish tint; RX rows `<<` and
  green (`Ack`) / red (`Nack`/`Timeout`) / grey (`Info`). Each row:
  `t  >>/<<  0xID  d=[..8 bytes hex..]  note`. Hover = per-byte tooltip
  (reuse the CAN Monitor byte-decode style).
- The full panel lives docked bottom-left under the rail (§2); the `renderTail`
  view is embedded in each test's RUN strip (zone D).

**Component B — per-field sync chip** `calibSyncChip(SyncState s)`:

| state    | colour | label    | meaning |
|----------|--------|----------|---------|
| `Synced` | white  | `ok`     | live value == last confirmed motor value |
| `Edited` | cyan   | `edited` | local edit staged, not yet sent |
| `Sent`   | amber  | `sent`   | SET sent, awaiting ACK / blob refresh |
| `Acked`  | green  | `ACK`    | motor confirmed (flash green for ~1 s, then `Synced`) |
| `Error`  | red    | `err`    | motor returned non-zero status |

State transitions are driven by: edit box change → `Edited`; `Set` click →
`Sent` (+ a deadline ~700 ms); a fresh `motorBlob_` value matching the sent
value → `Acked`; a `0x0AB` with non-zero status → `Error`; deadline elapsed
with no match → `Timeout` (shown as `err`). One `CalibFieldSync` struct per
editable id, kept in a `std::unordered_map<uint16_t, CalibFieldSync>`.

### 5.2 Range-slider + entry component

INAV-style: a slider bounded to the parameter's range, a numeric entry box
that accepts exact values, the spec/default, and an inline `Set`.

```cpp
// Returns true if the staged value changed this frame.
bool MainUi::calibRangeField(const CalibParamMeta& m,
                             float liveValue,        // current motor value
                             float& stagedValue,     // in/out edited value
                             CalibFieldSync& sync,    // drives the chip
                             bool   showSetButton);
```

Layout of one row (fixed widths so columns align across all panels):

```
[label 150px]  [ slider 220px ]  [ entry 90px ]  [Set 48px]  [spec 60px]  [chip 56px]
```

Rules:
- Slider and entry are two views of `stagedValue`; editing either updates both.
- Entry clamps to `[uiMin,uiMax]`; out-of-range typed input is rejected and the
  box flashes red for one frame.
- A faint **tick mark** on the slider track marks `uiDefault` (spec target).
- The live motor value is drawn as a thin marker on the track too (so the
  operator sees staged vs live divergence at a glance) and printed dimmed after
  the entry: `(live 85.0)`.
- `blurb` shows as a tooltip on label hover and, in test panels, as a dimmed
  line under the row (INAV "descriptive paragraph per parameter").
- For u32/bool params the slider becomes integer-stepped; bool renders as a
  toggle.

### 5.3 Results / plot component

```cpp
struct CalibResultRow {
    const char* label;
    bool        hasValue;
    double      value;
    const char* unit;
    bool        hasSpec;     // draw a spec band + verdict
    double      specNominal;
    double      specTolPct;  // +/- band, e.g. 10
};
void MainUi::calibResultTable(const CalibResultRow* rows, int n);
void MainUi::calibResultPlot(const char* title,
                             const float* t, const float* y1, const float* y2,
                             int n, double triggerT /* <0 = none */);
```

- `calibResultTable` renders `result | measured | spec | verdict` with the
  verdict cell coloured (`PASS` green / `WARN` amber / `FAIL` red / `—` grey)
  using `|value − nominal| <= nominal*tol`.
- `calibResultPlot` is a thin ImPlot wrapper: two series + an optional vertical
  trigger line (`ImPlot::PlotInfLines`, the same `(0.30,0.85,0.95,0.70)` colour
  as Triggered Capture markers). Used by Test 1 (sector dial — a polar variant),
  Test 3 (step response), Test 4 (coast-down decay).

### 5.4 Test-panel scaffold

`beginTestPanel(title, blurb, ResultPill)` / `endTestPanel()` draw zone (A) and
the separators; `calibPreconditionRow(label, ok)` draws one checklist row in
zone (B) and returns `ok` so callers can `&&`-fold a `Start`-enabled flag.

---

## 6. Component / file plan

### 6.1 New files

| File | Contents |
|------|----------|
| `src/ui/CalibrationTab.cpp` | All `drawCalib*` bodies, the rail, the 5 sections, `calibPreconditions()`, `calibRangeField()`, `calibResultTable/Plot()`, `beginTestPanel()`. Methods of `MainUi` (one class, split across .cpp files like the codebase already does). |
| `src/ui/CalibTxLog.h` / `.cpp` | `CalibTxEntry`, `CalibTxLog` ring buffer + renderer (§5.1 Component A). |
| `src/ui/CalibMeta.h` | `CalibParamMeta` table (§3.1), `CalibFieldSync`, `CalibResultRow`, `CalibSectionDef`, spec `constexpr`s (`kSpecRsOhm = 14.0507f`, `kSpecLmH = 25.22861f`, tolerances). |

> If the project prefers fewer translation units, `CalibrationTab.cpp` may be
> merged into `MainUi.cpp`; given `MainUi.cpp` is already 556 KB, a separate
> file is recommended.

### 6.2 Modified files

| File | Change |
|------|--------|
| `src/ui/MainUi.h` | Add `drawCalibration()` + the 6 `drawCalib*` decls; add `int calibSection_`, `CalibTxLog calibLog_`, `std::unordered_map<uint16_t,CalibFieldSync> calibSync_`, Test 3/4 state structs (`CalibStepTest`, `CalibCoastDown`), the §5 helper decls. |
| `src/ui/MainUi.cpp` | (a) Add `Calibration` to `kDockTabs[]` (`~line 3389`) + bump `kNumTabs`. (b) **Cut** the theta block out of `drawMotorTab()` (`10355-10507`) — moves to `drawCalibTheta()`. (c) **Cut** `drawMotorProfilerPanel()` out of `drawMotorConfig()` — moves to `drawCalibMachine()`. (d) In every motor `send*()` helper, add a `calibLog_.push(...)`. (e) In `handleFrame`, on `0x0AB`/`0x0AC` push RX entries + drive `calibSync_`. |
| `src/ui/MainUi.cpp` `drawMotorConfig()` | Reduced to the raw param-table escape hatch (or removed — §7 Q1). |
| `tools/pc-tool/CMakeLists.txt` | Add the new `.cpp` files to the target sources. |
| `tools/pc-tool/README.md` | Document the new tab. |

### 6.3 Reused as-is (no change)

`motorBlob_`, `sendMotorBlobGet/Set`, `applyMotorConfigBlobSet`,
`applyThetaSlotEdit`, `sendMotorThetaConfig`, `readAllThetaCalibration`,
`maybePollMotorTheta`, `motorProfiler_` + its `send*`, `motorConfigCache_`,
`kMotorConfigParams[]`, `motorConfigGroupName`, the Triggered Capture pipeline
(`DebugProtocol.*`, `pollCaptureSession`, `applyCapturedSamples`), `ICanTransport`,
`lastMotorTelemetry_`, `lastDiagStatus_`, `diagModeRequest()`.

### 6.4 Migration notes for the theta code

The theta UI block to move is **`MainUi.cpp:10355` ("FOC theta calibration")
through `10507`** (the LUT table + its two `TextDisabled` captions),
*excluding* the melody and telemetry blocks that follow. After the move,
`drawMotorTab()` keeps telemetry + commands + melodies; Remote Control loses
nothing operationally except theta, which is now reachable from the
Calibration tab. No transport/state-struct changes — the same
`motorThetaSlotEdit_[]`, `motorThetaSlotDirty_[]`, `lastMotorTelemetry_.theta_*`
fields back the moved widget.

---

## 7. Open questions / options

### Q1 — Keep or fold the old Motor Config tab?
The new "Config (full)" section duplicates most of the Motor Config tab.
- **Option A (recommended):** keep the `Motor Config` tab *only* for the raw
  DIAG counters + `cfg.fw_ver`/`cfg.crc` + the param-table escape hatch; the
  Calibration tab's "Config (full)" becomes the everyday editor. Minimal risk.
- **Option B:** delete `Motor Config` entirely, move DIAG counters into the
  Calibration "Overview" section. Cleaner, one fewer tab, but loses the
  power-user raw view.
- *Recommendation:* ship Option A, revisit Option B after the redesign is
  validated on the bench.

### Q2 — Left rail vs sub-tab strip?
INAV uses a left rail; ImGui's idiom is a `BeginTabBar`.
- **Option A (recommended):** left rail (`Selectable` list). It gives room for
  the result pills and the docked transaction log beneath it, and matches the
  user's INAV inspiration.
- **Option B:** an ImGui `TabBar` across the top. Less code, but no room for
  pills and forces the transaction log elsewhere.
- *Recommendation:* Option A.

### Q3 — Theta calibration has no progress telemetry.
The theta CFG path reports a *status enum*, not a 0-100 % progress. The Test 1
RUN strip therefore shows an **indeterminate** progress bar + a status-text
line. Option: extend motor FW to emit a progress byte for theta cal (small FW
change, mirrors the profiler `0x0AC` progress field). *Recommendation:* ship
indeterminate now; file a FW backlog item if operators want a real bar.

### Q4 — Step-response acquisition (Test 3) — capture vs live poll?
- **Option A (recommended):** reuse Triggered Capture (`0x0E0/0x0E1`) — MCU-side
  ring buffer at 100-1000 µs gives a clean high-rate trace; the pipeline,
  trigger marker and CRC verify already exist. Test 3 just scripts arm + step.
- **Option B:** live `READ_MEM` polling of `i_q_*` — far lower rate (~tens of
  Hz over CAN), too coarse to resolve a current-loop step (rise time ~µs-ms).
- *Recommendation:* Option A. It is the only option that resolves a current
  step; it also reuses the most code.

### Q5 — Test 4 firmware gap.
J/B identification is not in motor FW yet (roadmap Phase 6+). Two ways to ship:
- **Option A (recommended):** ship Tier A (host-side coast-down fit, reuses
  capture) now; render Tier B firmware buttons disabled with an explanatory
  tooltip. The tab is useful immediately and forward-compatible.
- **Option B:** hide Test 4 until FW lands. Cleaner but leaves the user's
  explicit "4 tests" requirement unmet.
- *Recommendation:* Option A.

### Q6 — Auto-tune "Apply" — RAM only or also flash?
All `Apply` buttons write RAM (`applyMotorConfigBlobSet`), never flash, so the
operator can A/B and revert; flash is always an explicit separate `Save to
flash`. This matches the roadmap "live tune = RAM-only, explicit save" idiom
and the `motorBlobAuthoritativeUntil_` design. No fork — stated here so the
implementer does not add an implicit flash write.

---

## 8. Implementation order (suggested)

1. Add the `Calibration` tab shell + left rail + `kCalibSections[]` dispatch +
   `calibPreconditions()`. Empty section bodies.
2. Build §5 shared components (`CalibTxLog`, `calibRangeField`,
   `calibResultTable/Plot`, `beginTestPanel`). Unit-exercise with dummy data.
3. Compact config strip + "Config (full)" grid (§3) — pure re-render of
   existing data, lowest risk.
4. **Test 1** — migrate the theta block (cut from `drawMotorTab()`).
5. **Test 2** — migrate the profiler panel (cut from `drawMotorConfig()`).
6. **Test 3** — new; wire the step test onto Triggered Capture.
7. **Test 4** — new; Tier A coast-down fit, Tier B disabled placeholders.
8. Overview dashboard + result pills wiring.
9. Trim/retire the old Motor Config tab per §7 Q1.

---

## 9. Acceptance checklist for the implemented tab

- [ ] New `Calibration` tab present; theta UI gone from Remote Control; profiler
      UI gone from Motor Config.
- [ ] Every editable field uses the range-slider+entry component with correct
      min/max/unit/default from §3.1.
- [ ] Every `Set` / test command produces a TX row in the transaction log; every
      `0x0AB`/`0x0AC` produces an RX row; per-field sync chip cycles
      `edited → sent → ACK → ok`.
- [ ] All 4 tests render the five-zone template; `Start` is gated by the
      precondition checklist.
- [ ] Test 1 calibrates + shows the 7-row LUT + sector dial.
- [ ] Test 2 runs XX pre-check and КЗ identification, shows Rs/Ld/Lq with
      pass/fail bands.
- [ ] Test 3 applies PI gains and shows a step-response plot with derived
      rise/overshoot/settling metrics.
- [ ] Test 4 Tier A coast-down produces a J/B estimate; Tier B buttons are
      visibly disabled with a tooltip.
- [ ] No implicit flash writes — `Save to flash` is always explicit and gated on
      `motorIdle`.
- [ ] Builds clean; `drivescope.exe` refreshed in `dist/`.
```

