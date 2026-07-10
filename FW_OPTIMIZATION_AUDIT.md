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

## Pre-bulk-order hardware checklist (researched 2026-07-10, actionable)

### A. Fault-at-speed / uncontrolled rectification — SOLVED ON PAPER, verify 3 parts

Key research result: **the event is voltage-bounded, not energy-unbounded.** At 1070 rpm
the rectified ceiling is ~73–75 V DC (Ke 49.4 × 1.07 krpm × √2, minus diode drops); the
body diodes stop conducting once the bus reaches EMF peak. Charging the bus caps to 75 V
takes only ~1–2 J — the ~40 J of drum kinetic energy stays in the drum (friction
coast-down) UNLESS something clamps the bus below 75 V and eats it. DRV8353 is a 100 V
family part (102 V abs max on VDRAIN) — it rides the event out.

Shopping/verify list (in order):
1. **Bridge FETs must be 100 V-class** (industry default for 13S; 60 V on a 54.6 V bus
   is 9% margin and out of line). VERIFY PART NUMBER on the board.
2. **Bus electrolytics ≥80 V rated (prefer 100 V)** — 63 V caps (common on 48 V boards)
   will vent at the 75 V ceiling. VERIFY.
3. Add **TVS Littelfuse SMDJ70A** (~$0.60, DO-214AB) or 5.0SMDJ70A (~$1.50) across the
   bus at the bridge: VBR 77.8–86 V sits ABOVE the 75 V ceiling → conducts zero energy
   during the fault, catches inductive spikes only. (A TVS below the ceiling would try
   to absorb coast-down energy: SMDJ-class handles 3–4 J single-pulse / 5–8 W sustained
   → it dies. Never size a board TVS as the energy sink.)
4. **BMS interlock rule (firmware, both sides)**: never open the pack/charge path while
   the motor spins; ESP commands drive stop BEFORE any disconnect. This is the classic
   VESC field-failure mode (BMS cutout during regen).
5. If FETs turn out 60 V: TVS is mathematically impossible in the 54.6→60 V window —
   either move to 100 V FETs (right fix) or add a brake chopper (ODrive Regen Clamp,
   $89, 12–58 V, ships with 2 Ω/50 W resistor; 40 J is trivial for it).

Timing note: bus caps give only 0.2–4 ms from 54.6 V to the ceiling at 10–20 A rectified
— any clamp must be autonomous hardware; firmware (1 kHz MF task) sees one tick.

### B. Encoder magnet (AGC 216–246 = field below the 35 mT spec floor)

Likely root cause ranking from AMS AN000271 (magnet selection guide):
1. **Ferromagnetic (steel) shaft behind the magnet** — shorts the field into the shaft;
   AMS: "the magnet is weakened substantially. This configuration should be avoided!"
   A steel shaft alone can produce AGC≈246. CHECK FIRST (test shaft with a magnet).
   Fix: brass/aluminum/non-magnetic-SS holder, or a few mm non-magnetic spacer.
2. Airgap too large — target ~0.8–1.5 mm magnet-surface→package-surface (+0.306 mm
   package→die internally).
3. Magnet itself weak/wrong type — must be **diametrically** magnetized cylinder.

Parts:
- **Bench fix now**: Radial Magnets **8996** (Ø6×3 mm N35 diametric, Digi-Key ~$1) —
  0.5 mm taller than the standard 8995, ~10–15% more field at unchanged gap.
- **Production BOM**: Ø6×2.5–3 mm **N35SH** diametric (Bomatec/Dexter/MS-Schramberg,
  clone of AMS ref part AS5000-MD6SH-1, 150 °C) — plain N35/N42/N52 are 80–120 °C parts
  and NdFeB loses ~0.11 %/°C reversibly; SmCo (Ø6×2.5 diametric) is the premium option
  (4× lower tempco). Buy sensor-grade (spec'd magnetization-axis tilt), not craft magnets.
- Acceptance: **AGC ~100–150 cold** (mid-range) so a hot motor never rails at 255.
  Log MAG (reg 0x3FFD) alongside AGC — MAG sagging while AGC pinned = fully below range.
- Mounting: pocket in non-magnetic holder, 2-part epoxy or Loctite 638/648 (no
  cyanoacrylate), concentricity ≤0.1 mm TIR. Eccentricity shows up as a 1×/mech-rev
  angle error → 20×/rev torque modulation on this motor — after the field fix, re-run
  the free-shaft cogging cal; the map absorbs residual eccentricity.
- Free predictive tool: AMS POS-simulator (github.com/ams-OSRAM/POS-simulator) —
  simulate magnet+gap → predicted field/AGC before ordering.

### C. Encoder adequacy verdict (researched 2026-07-11): KEEP the AS5047P

**Verdict: an encoder upgrade cannot be feel-noticeable on this machine. Fix the
magnet; the biggest remaining feel upgrade is firmware (SPI read rate), not silicon.**

The physics correction that reframes it: for a surface PMSM in torque mode with
feedback current control, torque error vs angle error is **T = T*·cos(Δθe)** —
SECOND-order (Pramod, arXiv:2310.00977 Eq. 18). The sin(Δθe) term is d-axis current
(flux churn/heat), not shaft torque. Our measured 0.06° mech INL = 1.2° el =
**0.02% torque ripple** (not the ~2% the sin framing suggested) — 50–100× below
perception, before the anti-cogging map absorbs it. Budget for <1% ripple: 0.41° mech.

Why the magnet still matters (the live defect): weak field (AGC 246, near the 255
MAGL rail) increases **transition noise** specifically — the one error class no map
can absorb (broadband, non-repeatable), and the dominant felt-roughness mechanism at
the low speeds where a cable machine lives (standstill grit, hiss through velocity
estimator × Kd and current loop). Datasheet noise: 0.068° RMS with DAEC, 0.052°
DAEC-off (recommended <100 rpm). Strong field degrades INL; weak field degrades noise.

Industry validation of our architecture: Simucube 2 Pro (best-feel direct-drive
sim wheel) uses a 22-bit encoder AND still ships per-unit cogging/ripple cal — the
map, not arcsecond accuracy, is what makes smoothness. Ben Katz's Mini Cheetah
(21 pp, same class as ours): encoder eccentricity LUT "improved things enormously."

Firmware gap identified (cheap, real): everyone reads the sensor much faster than we
do — ODrive SPI @8 kHz, VESC @20 kHz, moteus @30 kHz in the PWM ISR; we read at 1 kHz
+ extrapolate. Extrapolation is deterministic and fine, but read-timing JITTER maps
as ω·Δt (±10 µs = ±0.66° el at 550 rpm) and the extrapolated angle inherits velocity-
estimator noise. Queue: (a) jitter-free / hardware-phase-locked SPI read, (b) raise
read rate toward the loop rate, (c) PLL velocity estimator ~100–200 Hz (raw 1 kHz
back-difference of 0.068° RMS noise = ~16 rpm RMS velocity noise), (d) DAEC-off
below ~100 rpm.

Production note: our 0.06° INL is THIS unit; datasheet allows ±0.8–1.2° max
(= 4–9% pre-cal ripple on a worst-case unit at 20 pp). Per-unit anti-cogging cal
(which we ship) covers it. If silicon margin is wanted for the bulk order instead:
**MPS MA600** (~$8) — <0.1° after on-chip self-cal AND its TMR front-end works at
10–100 mT, making marginal magnets a non-issue; or **AS5047U** (~$8–10, drop-in
footprint) for 2–6× lower noise via the DFS filter. AksIM-2/optical ($250–3000)
buys nothing feelable here. Map limits to remember: it can't absorb temp drift of
INL (±0.2° over range) or a physically shifted magnet (we saw exactly this,
bin 382→280) — mount the magnet properly.

### D. Other

- OV threshold 60.0 V / `NOMINAL_BUS_VOLTAGE_V` 60 — revisit only if 15S (needs 80 V
  FETs/caps + OV ~68 + charger/BMS changes; moves the wall to ~825–940 rpm).
- ESP-side rewind derating: drafted, see `REWIND_DERATE_ESP_PATCH.md` (review-only;
  apply next session in the ESP repo).

## Current validated tuning defaults (boot values, commit 2cf8c46)

A850 · B1500 · Kp60 · q0 · S12000 · hard cap 550 · fault 1150 · brake P 0.06 /
quad 0.0012 / Ki 0.2 / max 16 A / LPF 60 ms / smooth gate 60 rpm / deadband 0.3 A ·
band 120 rpm · roll-off LPF fixed 40 ms (variant A; adaptive variant B = `73b017b`) ·
brake acts on motor-driven motion ONLY (pull purity) · FW ON at boot, legacy off ·
current loop P/I 1000/1000 (see gun #2) · dt comp 45 · telemetry single `[m]` line
20 Hz.

## Current-loop retune bench card (gun #2 -- the last firmware item)

VESC and ODrive both auto-compute current-loop gains with the SAME source-verified
formula: kp = L*bw [V/A], ki = R*bw [V/(A*s)], bw default 1000 rad/s, ki/kp = R/L
(pole-zero cancellation). Using MEASURED params (L=1.10 mH, R=0.74 ohm, R/L=673 rad/s)
converted to ST units (KPDIV 512, KIDIV 16384, 25 kHz, 992 cnt/A, ~1183 cnt/V @48V):

| Candidate | CDC | Bandwidth | Note |
|---|---|---|---|
| Zero-fix (start) | P1000 I860 | ~1490 rad/s | current bandwidth, corrected damping (old I1000 put the PI zero at 781 vs 673 rad/s) |
| Reference default | P670 I580 | 1000 rad/s | what VESC/ODrive would configure |
| Crisper | P1680 I1450 | 2500 rad/s | verify overshoot with 'g' capture |

Procedure: per candidate set P/I -> 'g' step capture -> step_tune.py (script verified
compatible with current dump format) -> one feel pass. Pick by step response, bake
winner into drive_parameters.h. Note: hotter Kp amplifies ADC noise -- if hiss
appears, that is the noisy ADC line (see below), not the gains.

Research verdicts from the VESC/ODrive source deep-dive (2026-07-10): both ship
decoupling DISABLED (matches our refutation); VESC FW is duty-mapped + off by default
(ours is ahead); our 25 kHz switching is already above the audible band (VESC's
12.5 kHz actual is not); anti-cogging 512-bin interpolated is same class as ODrive's
3600-bin nearest-neighbor. After this retune there is nothing left to copy.

## Hardware noise item (added 2026-07-10)

One ADC current-sense line is suspected noisy (bench observation). Quantify with the
'j' raw per-phase log + a 'g' capture at constant current BEFORE the PCB revision:
asymmetric one-line noise creates position-locked torque ripple, and sense noise x Kp
sets the hiss floor / max usable current-loop gain. Fix on the bulk-order board rev.

## Also queued (non-FW)

- **ESP-side rewind derating** (biggest felt improvement anywhere): DRAFTED — see
  `REWIND_DERATE_ESP_PATCH.md`. Memoryless velocity fade full-weight→2 A floor over
  200–320 rpm inbound in `resistance_tick()`; knobs rw0/rw1/rwf; eccentric-load %
  feature folds into the same mechanism.
- **Encoder read-path upgrade** (from the 2026-07-11 adequacy study, checklist §C):
  jitter-free/faster SPI reads (industry: 8–30 kHz vs our 1 kHz), PLL velocity
  estimator 100–200 Hz, DAEC-off <100 rpm. Do AFTER the magnet fix — quantify with
  a before/after 'g' ripple capture at standstill and slow crawl.
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
