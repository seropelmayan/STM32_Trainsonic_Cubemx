# MC Workbench Regeneration Checklist (Feed-Forward + Flux-Weakening enable)

Feed-Forward AND Flux-Weakening are now enabled in `Trainsonic.ioc.wb` /
`Trainsonic.wbdef` (`M1_FEED_FORWARD_CURRENT_REG_ENABLING` and
`M1_FLUX_WEAKENING_ENABLING` = true/ENABLE). FW defaults: `FW_VOLTAGE_REF=985`,
`FW_KP_GAIN=3000`, `FW_KI_GAIN=5000` (KPDIV/KIDIV=32768) — these need tuning when
you push above base speed (see the tuning notes).
Regeneration **overwrites the generated headers AND the generated body of
`FOC_CurrControllerM1`** — which holds critical hand-written code. Follow this
order so nothing is lost.

## 0. BEFORE regenerating — commit (mandatory restore point)
```
git add -A && git commit -m "checkpoint before Feed-Forward regen"
```
Everything below is recovered by diffing the regenerated files against this commit.

## 1. Regenerate
Open `Trainsonic.ioc` in MC Workbench → confirm Feed-Forward is checked (Drive
Management → current regulation) → Generate. (If the hand-edited flag didn't take,
tick the checkbox in the GUI — that's authoritative.)

## 2. Re-apply gains in `Inc/drive_parameters.h` (regen reverts these)
| Define | Set to |
|---|---|
| `PID_SPEED_KP_DEFAULT` | `10000/(SPEED_UNIT/10)` |
| `PID_SPEED_KI_DEFAULT` | `10000/(SPEED_UNIT/10)` |
| `PID_TORQUE_KP_DEFAULT` | `150` |
| `PID_TORQUE_KI_DEFAULT` | `150` |
| `PID_FLUX_KP_DEFAULT` | `150` |
| `PID_FLUX_KI_DEFAULT` | `150` |
| `OV_VOLTAGE_THRESHOLD_V` | `70.0` (15S pack; was 60.0 for 13S) |
| `UD_VOLTAGE_THRESHOLD_V` | `42.0` (15S pack; was 20.0 for 13S) |
(Torque/Flux 150 is a STARTING point — re-tune the current loop with FF active.)

### Bus voltage window (15S pack) — why these numbers

15S full = 63 V. The bench scope shows a ~+5 V IR rise at the pack during heavy
regen (58 V rest -> ~64 V), and that rise is what trips the BMS: 3.867 V/cell
resting + 0.333 V/cell rise = **4.20 V/cell**, the stock cell-OVP setpoint. That
is why the trip is state-of-charge dependent (it needs V_pack >= 58 V to reach
4.20) and why lowering the charge voltage was the wrong lever.

Board ratings (confirmed 2026-09-08): caps, FETs, gate drivers and passives are
all **100 V**; the weakest bus part is the **buck converter at 90 V**. The Vbus
sense saturates at ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR = **80.3 V**,
which is the real constraint on any threshold -- above that the ADC clips (it
still reads full-scale, so the OV fault stays safe, but the value is no longer
trustworthy and no proportional regulation is possible).

So the window is: regen peak ~64 V < **OV 70 V** < natural uncontrolled-
rectification ceiling ~75 V (Ke 49.4 V/kRPM x 1.07 kRPM x sqrt2, set by back-EMF
and now survivable on 100 V parts) < sense saturation 80.3 V < buck 90 V.

Raising OV only stops the FIRMWARE faulting. The BMS still trips at 4.20 V/cell;
only a bus-voltage regulator plus a non-battery energy sink fixes that.

The **low-pack taper** (`LOWV_*` in `mc_tasks_foc.c`) is what should actually stop
the machine — it fades the drive's power draw to zero between 51 V and 46 V of
IR-compensated resting voltage, so `UD_VOLTAGE_THRESHOLD_V` is only ever reached
at zero draw. It lives inside `USER CODE ... FOC_CalcCurrRef 1` and survives regen;
verify it is still there and still runs LAST in that block. Calibrate
`g_lowv_r_pack_ohm` (default 0.31 is an ESTIMATE from ~5 V sag at an assumed 16 A).

`UD_VOLTAGE_THRESHOLD_V = 42.0` (2.8 V/cell) is a starting value: it must sit
ABOVE the BMS LVC setpoint so the firmware stops before the BMS opens the pack
under load. **Confirm the BMS LVC and adjust.** Note the discharge sag mirrors
the regen rise (~5 V at load), so the bus reads ~5 V below resting under a hard
pull -- too high a value here will nuisance-trip at moderate SoC.

Also `Src/mc_config_common.c` (regenerated, edits are OUTSIDE user guards): both
`hMaxReliableMecSpeedUnit` initializers (in `ENCODER_M1` and
`VirtualSpeedSensorM1`) are hand-set to `(1150 * SPEED_UNIT) / U_RPM` — the
over-speed fault line, deliberately ABOVE human-reachable pull speed (~1070
observed): pulls are unbraked by design (resistance purity), FW keeps control
up there, and a fault trip at speed cuts PWM into uncontrolled rectification.
Motor-driven overspeed (release) is governed at 600 by `g_spdcap_*` in
`mc_tasks_foc.c`. Re-apply after regen.

## 3. RE-MERGE `Src/mc_tasks_foc.c` → `FOC_CurrControllerM1` (CRITICAL)
This function is regenerated. `git diff` it against the commit and merge back these
hand-written blocks (all were OUTSIDE USER CODE guards):
- **SPI-encoder commutation extrapolation** — the `if (Mci[M1].State == RUN)` block
  using `g_hf_tick_count`, `g_ol_enable`, `g_use_spi_commutation`,
  `g_enc_theta0`/`g_enc_omega_tick`. **Without this, commutation breaks** (falls back
  to TIM3). Highest priority.
- **Current LPF** on measured Iq/Id (`g_iq_lpf_alpha`).
- **avg Iq/Id** diagnostic (8192-sample accumulator → `g_avg_iq`/`g_avg_id`).
- **Step/ripple capture** (`g_step_state==1 && g_step_mode==0`).
- **Dead-time compensation** (`g_dt_comp`, added after `Circle_Limitation`).
- **`FW_DataProcess(&FW_M1, Vqd)` at the function tail** (after the FOCVars
  stores, post-Circle_Limitation Vqd). Feeds the flux-weakening voltage filter
  at 25 kHz; without it FW never engages (avV stays 0) since the MF-rate call
  was removed.
- (dq decoupling FF was trialled and STRIPPED 2026-07-10 -- see commit fe7304d
  for the full parked implementation and FW_OPTIMIZATION_AUDIT.md for the
  re-attempt plan. If it returns, its two calls in this function come back too.
  NEVER reuse .wb FF constants: they overflow int32.)
- Note: new FF code now lives in this function too — keep it; integrate dead-time
  comp after the FF/Circle_Limitation stage.

## 4. Survives regen automatically (verify, don't re-do)
- `USER CODE` / `USER SECTION` blocks: `FOC_CalcCurrRef` (speed-source override +
  step-inject + **cogging feed-forward**), `Ropetow_*` functions, `Ropetow_EncoderUpdate`,
  CDC commands, `main.c` boot defines (`ENC_ONLY`, `STEP_MODE`), `mc_app_hooks.c`.
- Hand-written files NOT generated by Workbench: `as5047.c/.h`, `cogg_table.h`,
  `drv8353.h`, `cogging.py`, `ripple.py`. Confirm `as5047.c` is still in the build
  (`.cproject` / linked resources).

## 5. Build, flash, test
- Confirm it still commutates (smooth spin, `avgId≈0`, `enc=ok`).
- Test at high speed near the wall (≤600 rpm) — listen for whether the harsh
  overmodulation sound is reduced (FF should clean up current control).
- Then re-tune the current loop (`P`/`I`) with FF active using the `g` step capture.

## Expectation
Feed-Forward cleans current control across the range (esp. high speed) but does NOT
move the voltage wall. The ~600–680 rpm wall stays until Flux Weakening (needs
tuning) or a higher Vbus. Don't drive past the wall during this test.
