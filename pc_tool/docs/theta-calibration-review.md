# Theta (electrical-angle / Hall-offset) calibration — end-to-end review

**Date:** 2026-05-15
**Reviewer:** motor-control / FOC engineer (read-only diagnostic)
**Symptom:** Operator runs theta calibration from DriveScope; the returned
theta parameters are "too perfectly the same" as before — calibration appears
not to run / not to change values.

---

## TL;DR — root cause

**The calibration *does* run on the motor and *does* compute fresh offsets — but
the DriveScope UI reads them back the wrong way, so the operator never sees
the new values:**

1. The "Start calibration" button correctly sends the CAN command that runs the
   full sweep-and-measure procedure (FW `driverMode = 4`). That part works.
2. The "Read calibration" button — and the auto-poll — read the motor's
   **whole-config BLOB**, which `MotorConfigGather()` builds from the
   **persisted / flash-resident** config payload, **not** from the live
   `theta_actual_el_offset` / `theta_sector_lut[]` that the calibration just
   wrote into RAM.
3. Therefore: unless the operator runs **"Save result to flash after
   calibration"** (action `CAL_SAVE = 4`) the freshly measured RAM values are
   **never persisted**, and the next BLOB GET returns the *old* flash values —
   identical to before. Even with `CAL_SAVE`, the BLOB read races the sweep:
   the BLOB is fetched while the motor is still mid-sweep (3+ s), or it reads a
   stale flash sector, so the LUT still shows pre-cal numbers.
4. Failures along the way are **silently swallowed**: a `BAD_ARG` / `NOT_READY`
   / `BUSY` / `FAULT` reply on the CAL command updates only
   `lastMotorTelemetry_.theta_status`; the operator sees a tiny status string,
   no modal, no red banner — easy to miss.
5. The earlier-seen value **`THETA off +0.52356 rad`** is **the factory default
   `THETA_OFFSET` placeholder**, not a measured value (see §5). Seeing exactly
   that number back is the strongest evidence the measured RAM value never made
   it into the read path.

The calibration is not a no-op in firmware. The bug is a **read-back / persist
path mismatch in pc_tool**, plus a missing completion handshake in the protocol.

---

## 1. End-to-end path

### 1.1 Firmware — the calibration routine (it works)

Files: `MOTOR/app_stm32f4_motor/Core/Src/events_theta.c`,
`Core/Src/events.c`, `Core/Inc/events.h`, `Core/Inc/app_motor.h`.

* **CAN entry point** — `events_theta.c:455 Events_Motor_ConfigCommand()`,
  `case APP_MOTOR_CFG_THETA_OFFSET` (sub-cmd `0x00`), action
  `APP_MOTOR_THETA_CAL (3)` / `APP_MOTOR_THETA_CAL_SAVE (4)`
  (`events_theta.c:491-531`). Pre-conditions checked: guard bytes,
  align-current range, `analogInitDoneFlag`, `APP_FAULT_STOP_MASK`, and
  `driverMode/runFlag/alignFlag/cal_active` busy state. On success it sets
  `i_Sd_Alignment`, calls `MotorThetaOffsetCalPrepare(save?)` and
  `driverMode = 4`, replies `APP_MOTOR_CFG_STATUS_RUNNING (7)`.

* **CAL_SAVE vs CAL** — the only difference is the `save_after_cal` argument
  passed to `MotorThetaOffsetCalPrepare()` (`events_theta.c:352`), which sets
  `theta_offset_cal_save_requested`.

* **The 4-phase sweep** — `events.c:foc_align_phase_logic()` (~line 760) +
  `foc_mode_alignment()` (`events.c:977`). RAMP 0.5 s → SWEEP 3.0 s (5 elec
  revs, per-Hall-sector sin/cos accumulation `events.c:798-804`) → SETTLE 1.5 s
  → SAMPLE 1.0 s (observer integration `events.c:813-819`). Total
  `THETA_ALIGN_WINDOW_TICKS` ≈ **6.0 s** (`events.h:223-231`).

* **Finalize** — `events.c:829 foc_align_finalize()`:
  * global offset `= atan2f(sin_acc, cos_acc)` with a coherence gate, then
    `MotorThetaOffsetSet(calibrated_offset, 0u)` → writes the live global
    `theta_actual_el_offset` (`events.c:854-872`);
  * per-sector LUT `theta_sector_lut[bin] = normalize(avg_drive + offset)`,
    sets `theta_sector_lut_calibrated = 1` (`events.c:875-917`);
  * **persistence only if `global_cal_ok && theta_offset_cal_save_requested`** →
    `MotorThetaOffsetStorageSave()` to RTC backup registers
    (`events.c:920-926`).

  **So: a plain CAL run updates RAM only. Persistence happens only for
  CAL_SAVE, and only into RTC-BKP — not into the flash config sector.**

* **`events_theta.c` uncommitted diff** is unrelated to this bug — it only
  threads `dstAddr` into `Events_Motor_ProfilerConfigCommand` (line 589).

### 1.2 The CAN protocol — what survived the 2026-05-13 cull

`Core/Inc/app_motor.h:268-275 AppMotorThetaAction_e`:

| action | value | status in FW |
|---|---|---|
| `THETA_GET` | 0 | **REMOVED** → `BAD_ARG` (`events_theta.c:555-561`) |
| `THETA_SET` | 1 | **REMOVED** → `BAD_ARG` |
| `THETA_SET_SAVE` | 2 | **REMOVED** → `BAD_ARG` |
| `THETA_CAL` | 3 | **alive** — runs the procedure |
| `THETA_CAL_SAVE` | 4 | **alive** — runs + saves to RTC-BKP |
| `THETA_SAVE` | 5 | **REMOVED** → `BAD_ARG` |
| `THETA_CLEAR` | 6 | **alive** — clears RTC-BKP, restores factory LUT |
| `THETA_GET_SECTOR` | 7 | **REMOVED** → `BAD_ARG` (`default:` branch) |

Sub-cmd `APP_MOTOR_CFG_THETA_SECTOR (0x01)` — **entirely removed**, the
`switch` no longer has the case → falls to `default: BAD_ARG`.

Whole-config GET/SET is now `APP_MOTOR_CFG_BLOB (0x03)`
(`events_theta.c:572-585`, `Events_Motor_BlobConfig` at line 677): 164-byte
blob, 33 chunks of 5 B, request-per-chunk, CRC32-verified. **The blob GET is
built by `blob_build_live_image()` → `MotorConfigGather(&payload)`**
(`events_theta.c:623-638`).

### 1.3 pc_tool / DriveScope

Files: `tools/pc-tool/src/ui/CalibrationTab.cpp`,
`tools/pc-tool/src/ui/MainUi.cpp`.

* **"Start calibration"** — `CalibrationTab.cpp:780-787`. Action `4` if
  `motorThetaSaveAfterCal_` else `3`; calls
  `MainUi::sendMotorThetaConfig(action, motorThetaAlignCurrent_)`.
* **`sendMotorThetaConfig`** — `MainUi.cpp:9566-9613`. Builds an 8-byte
  `MTR_CMD_CONFIG (0x005)` frame, `data[0] = kMotorCfgThetaOffset (0x00)`,
  `data[5] = action`, guard bytes `0xA5/0x5A` for mutating actions, source
  `kCanAddrPc`. **This is correct and matches the surviving FW handler.**
* **"Read calibration"** — `CalibrationTab.cpp:795-798` →
  `MainUi::readAllThetaCalibration()` (`MainUi.cpp:10011-10028`). Comment
  confirms the redesign: *"was 1 global GET + 6 per-sector GETs … Replaced
  with a single whole-blob GET"* → calls **`sendMotorBlobGet()`**.
* **Auto-poll** — `maybePollMotorTheta()` (`MainUi.cpp:10030-10050`), every
  1.5 s, also `sendMotorBlobGet()`.
* **Result table** — `CalibrationTab.cpp:826-899` reads
  `lastMotorTelemetry_.theta_offset_rad` / `theta_sector_lut_rad[]`, which are
  populated **from the BLOB receive parser**, not from any CAL response.

---

## 2. Precise root cause

### 2.1 Primary — pc_tool reads flash-resident config, not live cal RAM

`readAllThetaCalibration()` and the 1.5 s auto-poll both call
`sendMotorBlobGet()`. On the motor side the GET is answered by
`blob_build_live_image()` → `MotorConfigGather(&payload)`
(`events_theta.c:629`).

`MotorConfigGather()` builds the payload from the **motor's config model**
(the same structure that is persisted to / loaded from the flash config
sector — see `motor_config.c`). A plain **`THETA_CAL` (action 3)** run writes
**only** the live FOC globals `theta_actual_el_offset` and
`theta_sector_lut[]` via `MotorThetaOffsetSet()` / the LUT loop in
`foc_align_finalize()`. It does **not** touch the config model and does **not**
call `MotorConfigScatter`/persist.

Consequence: after a plain CAL, the next BLOB GET returns the **pre-calibration
values** — byte-for-byte identical to before. This is exactly the reported
"too perfectly the same" symptom.

### 2.2 Secondary — even CAL_SAVE persists to the wrong store + races the read

* `CAL_SAVE` saves to **RTC backup registers** (`MotorThetaOffsetStorageSave`,
  `events_theta.c:274-307`), **not** to the flash config sector that
  `MotorConfigGather()` reads. So the BLOB GET still does not reflect the cal,
  even with "Save to flash" ticked. (The checkbox label "Save result to flash"
  is itself misleading — the FW path is RTC-BKP, not flash.)
* The only FW path that makes a theta value visible to a subsequent BLOB GET is
  the **BLOB SET** path (`Events_Motor_BlobConfig` → `MotorConfigScatter`,
  `events_theta.c:797-807`), i.e. the operator manually editing a slot and
  pressing "Set". A *calibration* never feeds the config model.
* Timing race: the sweep takes ~6 s. "Read calibration" issues an immediate
  BLOB GET; auto-poll fires every 1.5 s. Every read that lands before
  `foc_align_finalize()` completes returns stale data, and there is **no
  completion event** to tell the host when to re-read (see §3).

### 2.3 No "calibration finished" handshake

The CAL command gets exactly **one** response: `STATUS_RUNNING (7)` at start
(`events_theta.c:529`). When `foc_align_finalize()` finishes there is **no CAN
frame emitted** — `driverMode` simply returns to 0 and
`theta_offset_config_status` is updated as a local variable. The host has no
push notification of completion and no fresh measured values; it must poll, and
its poll (BLOB GET) reads the wrong store anyway (§2.1).

`CalibrationTab.cpp:802-822` infers "running" purely from
`lastMotorTelemetry_.theta_status == 7`. That status is only refreshed when a
theta-`0x00` response arrives — i.e. only at the *start* ack. The animated
"calibrating..." bar therefore never naturally clears from a completion event;
it clears only on the next sub-cmd-`0x00` reply (manual GET — which is
`BAD_ARG` now — or another CAL).

### 2.4 Failures are silently swallowed

The theta-`0x00` response parser is `MainUi.cpp:4696-4762`. For a
`BAD_ARG`/`BUSY`/`NOT_READY`/`FAULT` reply it sets `theta_status`,
`theta_flags`, `theta_driver_mode` and a short `motorThetaTxStatus_` /
`global response status=N` string — **no modal, no error log, no red banner**.
`CalibrationTab.cpp:805-812` maps non-running statuses to a pill colour, but a
`STATUS_BUSY (2)` or `STATUS_NOT_READY (3)` is explicitly *excluded* from the
Fail branch (`st != 2u && st != 3u`), so a rejected calibration shows neither
Pass nor Fail — it just looks idle. An operator who fires "Start calibration"
while the motor is not in diag mode / not idle gets a `BUSY`/`NOT_READY`
rejection that is essentially invisible.

---

## 3. Is there a command that *runs* calibration, distinct from GET/SET?

Yes — `APP_MOTOR_CFG_THETA_OFFSET` sub-cmd, action `THETA_CAL (3)` /
`THETA_CAL_SAVE (4)`. pc_tool **does** send it correctly
(`sendMotorThetaConfig`, `MainUi.cpp:9588-9600`). The start path is not the
bug. The bug is entirely in **read-back** (`sendMotorBlobGet` reads the wrong
store) and **persistence** (CAL writes RAM only; CAL_SAVE writes RTC-BKP, not
the config model the BLOB reads).

---

## 4. Where should new offsets land, and does anything write them?

| store | written by | read by BLOB GET? |
|---|---|---|
| live globals `theta_actual_el_offset`, `theta_sector_lut[]` | **every** CAL run (`MotorThetaOffsetSet` + LUT loop, `events.c:870/913`) | **NO** |
| RTC backup registers (BKP0-8) | CAL_SAVE only (`MotorThetaOffsetStorageSave`) | **NO** |
| flash config sector / `MotorConfig` model | **only** BLOB SET → `MotorConfigScatter` (`events_theta.c:799`); also PARAM SET | **YES** |

The BLOB GET (the only thing pc_tool's "Read calibration" uses) reads the one
store that calibration **never** writes. That is the defect in one sentence.

---

## 5. Is `+0.52356 rad` a measured value or a default?

It is the **factory default placeholder**. `theta_actual_el_offset` is
initialised to the macro `THETA_OFFSET` (`events_theta.c:100`), and
`MotorThetaFactoryGet()` / `THETA_CLEAR` both restore `THETA_OFFSET`
(`events_theta.c:137,542`). The `events_theta.c:74-77` comment explicitly calls
out `0.52356` as the *global* `THETA_OFFSET` value that was historically
row-shifted into the sector-1 LUT slot. `0.52356 rad = 30.0°` exactly — a
round 30°, i.e. a hand-picked default, not a measurement (a real Hall-mounting
offset would be an irregular number like the factory LUT entries
`0.66521`, `1.78417`, …). Seeing exactly `+0.52356` after a "calibration" is
proof the measured value never reached the display.

---

## 6. Recommended fixes

### 6.1 Firmware (`MOTOR/app_stm32f4_motor`)

* **F1 — make CAL feed the config model.** In `foc_align_finalize()`
  (`events.c:920-926`), after a successful global+sector fit, push the new
  `theta_actual_el_offset` / `theta_sector_lut[]` into the `MotorConfig` model
  (the structure `MotorConfigGather` serialises). Without this, no read path
  that goes through the BLOB will ever see a calibration result. If
  `save_requested`, also persist the config model to flash — not only RTC-BKP.
* **F2 — emit a completion frame.** When `foc_align_finalize()` ends, send one
  `APP_MOTOR_ANS_CONFIG (0x0AB)` frame with `subcmd = 0x00`,
  `status = OK/STORAGE_EMPTY/NOT_READY`, the freshly measured global offset in
  `data[2..5]`, and the flag byte. This gives the host a definitive
  "calibration done, here is the result" event and removes the polling race.
  Route it to the request's `src` (same `dstAddr` discipline already applied to
  BLOB/PROFILER).
* **F3 — optional:** have the per-sector results also reportable, or rely on
  the host doing one BLOB GET *after* the F2 completion frame (cheap once F1
  lands).

### 6.2 Protocol

* **P1 — document the CAL completion frame** (F2) in `motor.h` and the protocol
  docs, including for app_dd and the DBC. A new answer/state touches FW +
  app_dd parser + pc_tool — keep all three in sync (see project memory
  *"New CAN ID touches three places"*).
* **P2 — fix the misleading label.** Either rename the FW path so CAL_SAVE
  truly persists to flash (preferred, via F1), or rename the pc_tool checkbox
  from "Save result to flash" to match reality.

### 6.3 pc_tool / DriveScope (`tools/pc-tool`)

* **D1 — re-read *after* completion, not during.** Drive "Read calibration" off
  the F2 completion frame: when a theta-`0x00` response with `status != RUNNING`
  arrives (or driverMode returns to 0), *then* issue one `sendMotorBlobGet()`.
  Currently `readAllThetaCalibration()` (`MainUi.cpp:10011`) and the 1.5 s
  auto-poll fire blindly during the 6 s sweep and read stale data.
* **D2 — surface failures.** In the theta response parser
  (`MainUi.cpp:4696-4762`), when `status` is `BAD_ARG (1)`, `BUSY (2)`,
  `NOT_READY (3)`, `FAULT (6)` or `STORAGE_FAIL (5)`, raise a visible error
  (modal or persistent red banner in the Calibration tab) — not just a 90-char
  status string. In `CalibrationTab.cpp:805-812`, treat `BUSY`/`NOT_READY` as
  Fail (or a distinct "Rejected" pill) instead of letting them read as idle.
* **D3 — don't claim "calibration started" on a rejected command.**
  `CalibrationTab.cpp:783-786` flips the pill to `Running` whenever
  `sendMotorThetaConfig` returns `true`, but `true` only means the CAN frame
  was queued — the motor may answer `BUSY`. Gate `CalibPill::Running` on
  receiving a `STATUS_RUNNING` response, not on the TX result.
* **D4 — completion clear.** The animated "calibrating..." bar
  (`CalibrationTab.cpp:817-819`) only clears when another sub-cmd-`0x00`
  response with `status != 7` arrives. After F2 this happens naturally; until
  then, add a timeout (≈8 s, slightly above `THETA_ALIGN_WINDOW_TICKS`) so the
  bar can't get stuck.
* **D5 — once F1 ships,** the existing BLOB read path is correct; no protocol
  rework needed on the host beyond D1's timing fix.

---

## 7. Minimal-change summary for the impatient

If only one thing is fixed: **F1** — make `foc_align_finalize()` write the
calibrated offset/LUT into the `MotorConfig` model so the BLOB GET reflects it.
That alone makes the values change visibly. **F2 + D1** remove the polling race
and make the result deterministic. **D2 + D3** stop silent rejections from
masquerading as a successful no-op run.
