# A — MCSDK Voltage-Wall / Flux Weakening

Cited findings appended as deep-research jobs complete. See [00-INDEX.md](00-INDEX.md).

<!-- FINDINGS APPENDED BELOW -->

## Job 1 — MCSDK Flux Weakening / Feed Forward / overmodulation (2026-06-30)
> Note: the search + verify phases completed (23 claims, 3-0 verified) but the final *synthesis* step failed — session usage cap hit mid-run. These are the verified raw claims; they already answer the core implementation question.

### ★ The torque-mode answer (most important)
- **MESC firmware applies FW to `Id*` while PRESERVING the commanded `Iq*`** — exactly the torque-mode pattern MCSDK lacks. `FW_current` overrides the MTPA d-request *only when more negative*. → `MESC_Common/Src/MESCfoc.c` (github.com/davidmolony/MESC_Firmware). **This is the model to copy.** [3-0]
- **MCSDK API to set both axes manually in torque mode:** `MC_SetCurrentReferenceMotor1(qd_t)` — populate `.d` (flux) and `.q` (torque) in S16A and call it; lets a custom FW loop drive `Id*` while keeping your commanded `Iq*`. [community.st.com 217015, 3-0]

### MCSDK native FW — and why it fails in torque mode
- MCSDK FW = a PI regulator (`PIDFluxWeakeningHandle_M1`) that drives `Id-ref` from 0 (MTPA) negative to exceed rated speed. Reference impl: github.com/AmineFeki/PMSM-Flux-Weakening. [3-0]
- **In Torque mode it OVERWRITES the Iq/torque reference** → "you end up with a Torque Control Mode without any idea of your torque value." ST staff: "Using Flux Weakening in Torque Mode can be counterproductive… the algorithm will set a negative Id value, thus speeding up your motor." [community.st.com 598438, 3-0]
- MCSDK lets you **switch speed/torque mode on the fly, even while spinning**. [MCSDK release notes, 3-0]

### Overmodulation / circle limit (MCSDK)
- SVPWM modulation index capped at **√3/2**; over-modulation extends beyond by riding the hexagon edge: when `T1+T2 > 1`, shrink `|V'cmd|` so `T'1+T'2 = 1`, keeping the angle θ. Raises THD in exchange for usable voltage/speed. [wiki.st.com STM32MotorControl:SDK_Overmodulation, 3-0]

### Feed Forward
- FF decoupling voltages **add to the current-PI outputs** for faster current response by predictively compensating motor dynamics vs. feedback-only. [MathWorks PMSM FeedForward, 3-0]

**Takeaway:** don't enable MCSDK's native FW (breaks torque mode). Build a custom FW PI that outputs `Id*` and feed it via `MC_SetCurrentReferenceMotor1` while preserving your `Iq*` — the MESC pattern. (Full unmerged output: `tasks/wdz0dim2h.output`.)

---

## Job 3 (lightweight) — Voltage-feedback FW + overmodulation + feedforward: implementable spec

Target: 20-pp SPM (Ld≈Lq=Ls), 48 V/13S, 25 kHz (Tpwm 40 µs). Custom FW emits **Id\*** while preserving commanded **Iq\***.

### Voltage budget
- SVPWM linear peak: `Vmax = Vbus/√3` → at 48 V ≈ **27.7 V**. Hexagon/six-step ceiling `2·Vbus/π ≈ 30.6 V` (the circle→hexagon gap is the overmodulation zone).
- Regulator variable: **`M = √(Vd²+Vq²)/Vmax`** (M=1 = inscribed circle). Engage FW *below* M=1 (setpoint **0.85–0.92**) to leave the current PI headroom.

### 1. Modulation-feedback FW law (outputs Id\*)
Closes a loop on *measured* modulation, so it's robust to Ls/ψ_m error (why ODrive & MathWorks both use it):
```
M     = sqrt(Vd^2+Vq^2)/Vmax
e_fw  = M - M_set                         // M_set ~0.88
Id_fw = -Kp_fw*e_fw - Ki_fw*∫e_fw dt      // I-dominant; clamp Id_fw ∈ [Id_min, 0]
Id*   = Id_fw ;  Iq* = Iq_cmd (PRESERVED)
Id_min = -sqrt(Imax^2 - Iq^2)             // current-circle floor protects Iq
```
M above setpoint → e>0 → Id\* more negative → less stator flux/voltage demand → M falls. Two anti-windup layers: (a) back-calculation `I += Ki*e*dt + Kaw*(Id_clamped−Id_unclamped)`, Kaw≈1/Ki..10/Ki; (b) integrate fast engaging, bleed slow releasing.
- **ODrive ref:** `fw_enable`, `fw_mod_setpoint` (M target), `fw_fb_bandwidth` (integral rate); integrates `(mod_setpoint − actual_modulation)` → Id\*, with **priority mod-d clamping** (d-axis gets headroom, q sacrificed) — fixed "spins slower when trying to go faster."
- **MathWorks ref:** same M, internal integral controller → β∈[0,1] bending the current angle toward −Id; block defaults M_th=1, **integral gain 100, anti-windup 10**.

**Tuning starting values (700 rpm, 25 kHz, 20 pp, 48 V):**
| Param | Start | Note |
|---|---|---|
| `M_set` | **0.88** (0.85–0.92) | lower = earlier/softer FW |
| FW loop BW | **150–300 rad/s (25–50 Hz)** | must be **5–10× slower** than current loop |
| Current-loop BW | 1000–2000 rad/s | FW BW ≈ BW_i/8 |
| `Ki_fw` / `Kp_fw` | `Ki_fw≈200`, `Kp_fw=0` (I-only) | add small Kp only if sluggish |
| `Id_min` | `−Imax` (or current circle) | hard floor |

(20 pp → 233 Hz electrical at 700 rpm; keep FW BW low so it doesn't touch the fundamental.)

### 2. bEMF + cross-coupling feedforward (offload the PI)
```
Vd = Rs*Id + Ls*dId/dt − ω*Ls*Iq
Vq = Rs*Iq + Ls*dIq/dt + ω*Ls*Id + ω*ψ_m
Vd_ff = −ω*Ls*Iq ;  Vq_ff = +ω*Ls*Id + ω*ψ_m       // add to PI outputs
```
`ω*ψ_m` (back-EMF) is the dominant term eating the budget — feeding it forward makes FW behave linearly and keeps d/q independent. Decoupled plant ≈ `Ls·s+Rs` → `Kp=BW_i·Ls`, `Ki=BW_i·Rs`.

### 3. Overmodulation strategy
Recommend **d-axis-priority (flux-decreasing) clamp into the inscribed circle (M≈1)**, NOT the hexagon: composes with the FW loop, keeps PWM harmonics at switching sidebands (quiet), and avoids 6th/12th-order tones that — at 20 pp / 233 Hz electrical — land in the audible band (1.4/2.8 kHz). Hexagon/min-phase-error only if you need the last ~5–10% speed and tolerate whine (Microchip: Vq gain <0.1 by M=1.15 — lots of noise for little voltage).

### 4. Chatter avoidance (impulsive cable pulls / regen)
1. **LPF the modulation feedback** (1–3 ms) before the PI. 2. **Hysteresis/deadband** ±0.02 around M_set. 3. **Asymmetric integrator** (fast engage, slow release) — kills the regen unwind/rewind cycle. 4. **Recompute `Vmax=Vbus_meas/√3` every cycle** so a regen bus rise raises the ceiling instead of confusing the loop. 5. **Slew-limit Id\***. 6. Keep **FW BW ≪ current BW ≪ PWM/2**.

### Pseudocode (current ISR)
```c
float Vmax = Vbus_meas * 0.57735f;            // Vbus/sqrt(3)
float M    = sqrtf(Vd*Vd + Vq*Vq) / Vmax;
M_lpf += (M - M_lpf) * alpha;  M = M_lpf;     // 1-3 ms LPF
float e = M - M_set;  if (fabsf(e) < 0.02f) e = 0;   // deadband
float ki = (e > 0) ? Ki_fast : Ki_slow;       // asymmetric
float Id_unc = fw_integ - Kp_fw*e;
float Id_cmd = clampf(Id_unc, Id_min, 0.0f);
fw_integ += (-ki*e*dt) + Kaw*(Id_cmd - Id_unc);
fw_integ  = clampf(fw_integ, Id_min, 0.0f);
Id_star = slew_limit(Id_cmd, Id_prev, max_dId); Id_prev = Id_star;
float Id_floor = -sqrtf(fmaxf(0.f, Imax*Imax - Iq_star*Iq_star));
Id_star = fmaxf(Id_star, Id_floor);  Iq_star = Iq_cmd;   // preserve torque
float w=omega_e; Vd = pi_d(Id_star-Id) - w*Ls*Iq;        // + decoupling FF
Vq = pi_q(Iq_star-Iq) + w*Ls*Id + w*psi_m;
float Vlim=Vmax; Vd=clampf(Vd,-Vlim,Vlim);              // d-priority circle clamp
float Vqr=sqrtf(fmaxf(0.f,Vlim*Vlim-Vd*Vd)); Vq=clampf(Vq,-Vqr,Vqr);
```
**Sources:** ODrive [foc.cpp](https://github.com/odriverobotics/ODrive/blob/master/Firmware/MotorControl/foc.cpp) · [API fw_mod_setpoint/fw_fb_bandwidth](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html) · [changelog (priority mod-d)](https://docs.odriverobotics.com/v/latest/changelog.html) · MathWorks [FW block](https://www.mathworks.com/help/sps/ref/pmsmfieldweakeningcontroller.html) · [Microchip overmodulation](https://developerhelp.microchip.com/xwiki/bin/view/applications/motors/control-algorithms/3-phase-foc/) · US Patent 11,502,631 (LPF on modulation deviation for chatter).

## Job A-deep — MESC law + MCSDK injection (alternative to the native component)
**MESC V2 FW (integral-only, torque-safe)** — `MESCfoc.c`, the simplest robust law:
```c
if (Voltage > Vmag_max) FW_current = 0.99f*FW_current - 0.01f*FW_curr_max; // ceiling -> Id more neg
else                    FW_current = 1.01f*FW_current + 0.0101f*FW_curr_max;// headroom -> relax
if (FW_current > Idq_req.d) FW_current = Idq_req.d;     // never less neg than requested
if (FW_current < -FW_curr_max) FW_current = -FW_curr_max;
// apply to d-axis ERROR only; Idq_req.q (torque) untouched
```
Fixed-step integral with 0.99/1.01 leak (self-zeros off the wall); bandwidth must sit well below the current loop. Circle-limit: `I_qmax = √(I_max² − I_FW²)` (Id priority, Iq sacrificed). This is the fallback if the MCSDK native PI is hard to tune.
**MCSDK injection (what our implementation uses):** in **torque mode** write `FOCVars[M1].Iqdref.d` (S16A) directly in the HF task, or `MC_SetCurrentReferenceMotor1(qd_t{.q,.d})`. Speed mode's speed-PI clobbers `Iqdref.q` every tick — torque mode is mandatory. `Circle_Limitation(Vqd)` uses a 128-entry MMITABLE, scales q & d **equally** when over `MaxModule²`; to get torque-sacrifice ordering also cap `Iqdref.q` before the PIDs. Enable `FF_VqdAddOn` (feed_forward_ctrl.c) for d/q decoupling so injected Id doesn't disturb Iq. ([MESCfoc.c](https://github.com/davidmolony/MESC_Firmware/blob/master/MESC_Common/Src/MESCfoc.c) · [MC_SetCurrentReferenceMotor1](https://community.st.com/t5/stm32-mcus-motor-control/how-to-manually-set-current-reference-in-mc-api-mc/td-p/217015) · full: `tasks/a2ea1a0c30f33d92a.output`)


