# D — Sensored Commutation Quality at Speed

Cited findings appended as deep-research jobs complete. See [00-INDEX.md](00-INDEX.md).

<!-- FINDINGS APPENDED BELOW -->

## Sensored commutation at speed — AS5047P + STM32 (lightweight job)

### Core problem: error ×20, growing with speed
20 pp → electrical angle = mechanical × 20, so **1° mechanical error = 20° electrical**. Electrical-angle error rotates the dq frame: torque ×cos(θ_e), d-axis disturbance ∝ sin(θ_e) → current ripple, noise, heat, "reliability BAD." Two speed-scaling sources dominate: **(a) sensor read/transport latency**, **(b) control-loop sample→actuation delay** — both "angle measured at t, applied at t+Δ".

**Magnitude check:** `θ_e_err = ω_mech × pole_pairs × T_lat`. At 20 pp, even ~200 µs net latency at 1000 rpm (104.7 rad/s) = **0.42 rad ≈ 24° electrical** uncompensated lag. That alone explains commutation degrading at speed.

### 1. AS5047P latency + DAEC
- Static-accurate sensor develops a **speed-proportional dynamic angle error** (magnet moves during propagation). **DAEC** (Dynamic Angle Error Compensation) internally predicts it: at *constant* speed leaves only **±1.9 µs** residual (±0.165° mech at 14.5 krpm). **Leave DAEC ON** for at-speed; off only gives 0.016° rms noise benefit for static/<100 rpm.
- DAEC does **not** compensate **host-side latency** (SPI transaction + read→PWM-apply gap) — that's yours to feed-forward. Use **ANGLECOM** (DAEC) not ANGLEUNC; short SPI frame (~10 MHz, ANGLECOM only). ([ams DAEC AN000687](https://www.mouser.com/pdfDocs/ams_AN000687.pdf), [datasheet](https://www.mouser.com/datasheet/2/588/AS5047P_DS000324_3_00-1843440.pdf))

### 2. Angle extrapolation / velocity feed-forward (1 kHz → 25 kHz)
Predict angle at the apply instant: **`θ_apply = θ_read + ω_elec × T_ff`**, `T_ff` = transport latency + ½ PWM period + compute delay, `ω_elec = ω_mech × 20`.
- **Read the angle at/near 25 kHz (DMA in the current ISR)** — shrinks the extrapolation horizon from 25 steps to ~1 and removes most "stepping"/noise. Best single fix.
- **Filter velocity, not angle** — raw finite-diff velocity from quantized 14-bit @1 kHz is noisy; ×20 makes it worse. Use a **PLL/tracking observer** (angle+velocity state) → low-noise velocity *and* clean bounded-lag extrapolated angle.
- **Pitfall = "racing" angle:** noisy velocity → extrapolated angle jitters → torque ripple/buzz (this is likely part of your noise). **Rate-limit** the predicted delta so a velocity glitch can't jump commutation in one cycle. SimpleFOC `SmoothingSensor` is the reference technique. ([README](https://github.com/simplefoc/Arduino-FOC-drivers/blob/master/src/encoders/smoothing/README.md))

### 3. ABI vs SPI at speed; EMI miscount
- **ABI/TIM3:** zero latency, high rate, but AS5047P ABI = 1024 PPR/4096 steps and at high edge-rate **EMI/ringing causes miscounts that accumulate as permanent electrical drift** until index — classic "commutation drifts then faults."
- **SPI absolute:** drift-free, self-correcting, carries diagnostics; cost = latency (§1/§2).
- **Recommended hybrid:** **SPI ANGLECOM = source of truth for commutation** (extrapolated), TIM3 ABI only for high-rate velocity, and **periodically re-seat ABI against SPI absolute** (disagreement beyond threshold = miscount alarm). EMI on ABI: series 22–100 Ω + RC, TIM input-capture digital filter, shielded/guarded routing away from phases/switch node.

### 4. Why the reliability/over-speed check trips
MCSDK `SPD_IsMecSpeedReliable` flags bad when a speed sample exceeds `hMaxReliableMecSpeedUnit` **or jumps too far** between samples; after `bMaximumSpeedErrorsNumber` consecutive → SPEED_FEEDBACK fault + stop.
- On your setup it's usually a **symptom of §1–§3** (latency error / ABI miscount / racing extrapolated angle → spurious large speed sample → variance trip), **not** an independent fault.
- Also check **pole-pairs=20 and PPR=1024/4096 consistent**, and M-method window quantization noise.
- **Fix angle quality first, then widen margins sanely** (raise `hMaxReliableMecSpeedUnit` w/ headroom, bump `bMaximumSpeedErrorsNumber` modestly, ensure speed LPF). Don't just disable it — it's the encoder-loss safety net.
- **AS5047P diagnostics every read:** check **AGC** (DIAAGC), **MAGH/MAGL** (field hi/lo), **COF**, **parity**. AGC railing / marginal air-gap/centering = "fine slow, noisy fast." A single bad SPI word → 20× electrical spike → reliability trip; reject bad-parity frames.

### Fixes ranked for this user
1. **DAEC ON + host-latency feed-forward**: `θ_apply = θ_read + (ω_mech×20)·T_ff`. Cancels the speed-proportional ×20 lag.
2. **Read absolute angle at ~25 kHz (DMA in current ISR)**, not 1 kHz → horizon 25→1 step, kills stepping/noise. If stuck at 1 kHz, extrapolate with filtered velocity/observer + rate-limit.
3. **SPI ANGLECOM = commutation truth; demote ABI to cross-check** (EMI drift otherwise).
4. **Check AGC + magnet alignment; validate SPI parity/error bits every frame.**
5. **Filter velocity with a PLL/observer** before it feeds commutation extrapolation AND the reliability check (stops false trips).
6. **Re-verify 20 pp / 1024 PPR / direction / zero-offset, then set reliability bounds with headroom** — fix quality first, don't disable the check.
7. **Harden ABI electrically** if kept (series R + RC, TIM IC filter, guarded routing).

Sources: [ams DAEC AN000687](https://www.mouser.com/pdfDocs/ams_AN000687.pdf) · [AS5047P datasheet](https://www.mouser.com/datasheet/2/588/AS5047P_DS000324_3_00-1843440.pdf) · [SimpleFOC SmoothingSensor](https://github.com/simplefoc/Arduino-FOC-drivers/blob/master/src/encoders/smoothing/README.md) · [MCSDK speed-feedback fault](https://community.st.com/t5/stm32-mcus-motor-control/mcsdk-motor-profiler-always-fails-with-speed-feedback-error/td-p/131819) · [STM32 encoder M-method noise](https://pmc.ncbi.nlm.nih.gov/articles/PMC9324733/)

## Job D-deep — implementation detail (latency FF + PLL + AS5047P regs)
**Latency FF:** `T_ff = T_sensor + T_spi + T_compute + 0.5·T_pwm` (½ PWM = dominant: 25 µs @20 kHz). `theta_apply = wrap_2pi(theta_read + omega_elec·T_ff)`, electrical frame, **same theta for fwd+inv Park**. DAEC corrects the chip's ~90–110 µs front-end to ±1.9 µs residual — but NOT host transport, which is the FF above.
**PLL/ATO (type-II, low-noise velocity + clean angle)** — VESC `foc_pll_run`:
```c
float dtheta = phase - *phase_var; utils_norm_angle_rad(&dtheta);
*phase_var += (*speed_var + kp*dtheta)*dt; utils_norm_angle_rad(phase_var);
*speed_var += ki*dtheta*dt;            // <- the low-noise velocity you feed to T_ff
```
Gains: `kp=2·ζ·ω_n`, `ki=ω_n²`, ζ≈1, f_n≈200–500 Hz. Zero steady-state lag at constant speed (free integrator).
**AS5047P regs:** read **ANGLECOM (0x3FFF)** for commutation (DAEC on); **ERRFL (0x0001)** FRERR/INVCOMM/PARERR; **DIAAGC (0x3FFC)** AGC[7:0]/COF(bit9)/MAGH(bit10)/MAGL(bit11) — reject angle if COF, magnet fault on MAGH/MAGL, AGC railing = bad air-gap; **SETTINGS1 (0x0018)** DIR(bit2)/DAECDIS(bit4); **ZPOSM/L (0x16/0x17)** zero offset (or subtract in SW). 16-bit frame, MODE1, ≤10 MHz, bit15=even parity, bit14=EF. **SPI+DMA in the current ISR**, validate parity/EF every frame, on bad frame reuse the PLL-extrapolated angle. (full: `tasks/a7e40efa774b8b720.output`)

