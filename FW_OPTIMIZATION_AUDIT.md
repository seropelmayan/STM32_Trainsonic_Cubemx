# FW Optimization Audit — findings & roadmap (2026-07-10)

Full read-only audit of the flux-weakening / current-control chain against industry
practice and literature (two web-research passes + line-by-line code review).
State at time of audit: branch `mcsdk-ff-fw`, commits `73b017b` (variant B,
torque-adaptive roll-off LPF) and `2cf8c46` (variant A, fixed 40 ms — **currently
in the tree and on the board**). Nothing below has been changed yet.

## Context: why the motor is well-placed

- Characteristic current `Ich = ψm/Ld = 0.0193 Wb / 1.4 mH ≈ 13.7 A` **< rated 29 A**
  → textbook "optimal flux-weakening" class (Soong & Miller); theoretical CPSR ~8x
  with the 12 A Id budget. We use ≤1.7x, transiently.
- Industry smooth zones: ≤1.5x routine everywhere; 2x = documented SPM comfort
  boundary (Microchip AN1292); production precedent for transient FW with felt-force
  quality = automotive EPS (2.5x). Roughness in literature = deep-FW/overmodulation,
  which we never enter (85% target, linear PWM only).
- Product posture: normal envelope (0–550 rpm cap) needs **no FW at nominal charge**;
  FW is a transient excursion handler for max-effort pulls (to ~1070 rpm observed).

## SMOKING GUNS (fix these, in this order)

### 1. dq decoupling feed-forward — TESTED TWICE AND REFUTED (2026-07-10, closed)
**VERDICT: no benefit on this machine — do not re-attempt unless hardware changes.**
Round 1 (datasheet L=1.4mH constants): Iq err 233/803 vs 155/219 baseline = WORSE + hiss
(cause: L saturates to 1.10mH at working currents → ~27% over-decoupling). Round 2
(bench-measured L=1.10mH, λ=0.0178, + 1 ms anti-hiss smoother): 167/381 = EQUAL to
baseline, still hissing. Conclusion: the 25 kHz current loop already rejects the
cross-coupling almost completely (only 0.15 A residual without FF). Implementations
preserved: fe7304d (round 1), e12be39 (round 2 w/ measured constants + derivation;
the .wb constants overflow int32 — that was the 2026-06-19 violent vibration).
Measured params are the durable spoils: **L = 1.10 mH, λ = 0.0178 Wb, Ich ≈ 16 A.**

Original (now-refuted) theory kept below for the record:
### 1-old. dq decoupling feed-forward is MISSING (biggest win)
No `FF_*` component anywhere in the build. At 900 rpm a 9 A FW Id transient couples
ω·L·ΔId ≈ 24 V into the q axis; the torque PI rejects it *reactively* → every FW burst
perturbs felt force ("vibration at high speed" residual). ST's Feed-Forward component
is enabled in the `.wb` (pending regen, never done) and the verified hand-wiring
recipe exists in session memory (`ropetow-ff-fw-recipe`): reuse ST's FF_ functions +
Workbench constants, do NOT reimplement scaling in float. Wire it like FW_ was wired.

### 2. Current loop deliberately detuned, tuned pre-FW-era
`PID_TORQUE/FLUX KP/KI = 1000/1000` ("loop was too hot at 3688 → voltage thrash").
The thrash was almost certainly undecoupled cross-coupling (gun #1). After #1, retune
hot with the `g` step-capture rig + `step_tune.py`. Guideline: current-loop BW 5–15%
of 25 kHz PWM (1.25–3.75 kHz).

### 3. Unguarded int16 add in dead-time compensation — FIXED 2026-07-10 (commit 20efa8d, saturating add)
`FOC_CurrControllerM1`: `Valphabeta.alpha/beta + dtab2.alpha/beta` is a raw int16 add
AFTER Circle_Limitation. At ceiling-grazing moments (avV hit 32.4k in logs) a
component near +32767 plus ~±45 comp WRAPS negative → one-PWM-cycle reversed voltage
vector (felt as click/pop during extreme events). Fix: saturating add.

## Suboptimal (works, short of textbook)

4. **FW PI not gain-scheduled** — loop gain ∝ ωe/Vbus (PM ~65° @620 rpm → ~50° @1070
   rpm sagged pack; matches ±1.2k avV wobble in logs). Literature remedy: schedule Ki.
5. **Motor params unverified** (LS 1.4 mH, RS 0.74, Ke 49.4 — profiler provenance).
   Decoupling FF accuracy (gun #1) depends on them. Measure L via `g` step rise time
   (τ = L/R) and Ke via coast-down avV before wiring FF.
6. **Stale `ID_DEMAG_A = -8.2`** in `pmsm_motor_parameters.h` vs 12 A runtime budget
   (`FW_DEMAG_A` in mc_tasks_foc.c is authoritative). Regen would resurrect 8.2 —
   checklist item.
7. **Encoder chain at speed**: weak magnet (AGC 216, `enc=BAD` flapping) — angle error
   is AMPLIFIED in the FW region (documented); commutation latency FF (`G`=40 = 4
   ticks) calibrated at low speed, verify at high speed after magnet fix.

## Checked clean (don't re-litigate)

Linear modulation only (no overmod — the #1 literature roughness source) · FW target
850 below MaxVd 950 · 25 kHz FW_DataProcess + 1 kHz PI + 124 Hz filter = textbook
ratios · anti-windup/seeding = TI/ST canonical (ratchet + windup-past-floor both
fixed and verified on bench) · demag 12 A < Ich 13.7 A · Iq-circle math overflow-free.

## Pre-bulk-order hardware checklist (separate from the above)

- **Fault-at-speed / uncontrolled rectification**: PWM drop at 1070 rpm rectifies
  ~75 V DC at the bus. Verify: FET part number (100 V-class = safe; DRV8353 itself is
  100 V), bus-cap voltage rating, and whether the BMS can disconnect the pack during
  regen (that's the worst-case path). Danfoss guideline: >1.2–1.4x base needs
  engineered absorption (TVS/clamp) if parts aren't rated.
- **Encoder magnet** in production BOM (see #7).
- OV threshold 60.0 V / `NOMINAL_BUS_VOLTAGE_V` 60 — revisit only if 15S (needs 80 V
  FETs/caps + OV ~68 + charger/BMS changes; moves the wall to ~825–940 rpm).

## Current validated tuning defaults (boot values, commit 2cf8c46)

A850 · B1500 · Kp60 · q0 · S12000 · hard cap 550 · fault 1150 · brake P 0.06 /
quad 0.0012 / Ki 0.2 / max 16 A / LPF 60 ms / smooth gate 60 rpm / deadband 0.3 A ·
band 120 rpm · roll-off LPF fixed 40 ms (variant A; adaptive variant B = `73b017b`) ·
brake acts on motor-driven motion ONLY (pull purity) · FW ON at boot, legacy off ·
current loop P/I 1000/1000 (see gun #2) · dt comp 45 · telemetry single `[m]` line
20 Hz.

## Also queued (non-FW)

- **ESP-side rewind derating** (biggest felt improvement anywhere): when rewind speed
  > ~250–300 rpm (nobody holding), ESP sends retract torque (~2 A) instead of the
  resistance setting; restores full setting as speed drops. Bonus product feature:
  separate eccentric load %. ~10 lines in the ESP heartbeat/torque path (separate repo).
- Controlled-shutdown path (hold FW Id while decelerating before stop) — production
  polish; never stop the motor at speed (rule).
- SPI-speed sign verification (Live Expressions, hand-turn both directions) →
  governor could then use 1 ms speed for both sign+magnitude → less ripple, less
  filtering, tighter cap-holding.

## Suggested next bench session (items 1–4 = one sitting)

1. Measure L (step rise time) and Ke (coast avV at known rpm) → update/confirm params.
2. Hand-wire ST FF decoupling per recipe (guarded like the FW block; REGEN checklist).
3. Retune current loop hot with `g` + `step_tune.py`.
4. Saturating add in dead-time comp.
5. Re-run the three-regime test (high-torque release / low-torque rewind / low-torque
   fast pull) + one max-effort pull; compare `IdFW`-burst felt-force disturbance
   before/after FF.
