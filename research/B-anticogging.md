# B — Anti-Cogging: Proven Algorithms + Working Code

Cited findings appended as deep-research jobs complete. See [00-INDEX.md](00-INDEX.md).

<!-- FINDINGS APPENDED BELOW -->

> **TL;DR for our case:** only **ODrive** and **moteus** ship working anti-cogging code; SimpleFOC/VESC/MESC/Tinymovr do **not**. We already have the ODrive-style position-indexed Iq LUT — the proven upgrades are (1) make it self-correcting via **iterative learning control**, (2) optionally store it as **harmonic/Fourier coefficients** (noise-robust at speed), and (3) **advance the lookup by velocity×latency** so the feed-forward isn't keyed on a delayed angle (this also ties into our commutation-at-speed problem). Details below.

### ODrive — `anticogging` (most directly applicable, proven)
- **Algorithm:** one-time calibration sweeps the rotor slowly (coarse then fine), records holding current at each encoder position → position-indexed torque map injected as feed-forward. Same architecture you already have, plus two-stage auto-calibration + NVM persistence. ([docs](https://docs.odriverobotics.com/v/latest/guides/anticogging.html))
- **Proven?** Yes, widely used, but officially **experimental**: "introduces a feedforward term which depends on the position estimate, which itself can be delayed or inaccurate, thereby reducing stability margins… requiring you to tune down gains." Forum failure mode = calibration never finishes / oscillates with high PID jitter; best results on low-jitter tunes, NOT high gains. ([runs-away](https://discourse.odriverobotics.com/t/anticogging-runs-away-and-does-nothing/8260), [fails](https://discourse.odriverobotics.com/t/anti-cogging-fails/6665), [issue #221](https://github.com/madcowswe/ODrive/issues/221))
- **Code:** `Firmware/MotorControl/controller.cpp` — `anticogging_calibration()`, `set_map()`/`get_map()` — https://github.com/odriverobotics/ODrive/blob/master/Firmware/MotorControl/controller.cpp · `AnticoggingConfig` (`max_torque`, `calib_*_vel`, `calib_*_tuning_duration`, `calib_coarse_integrator_gain`, `calib_bidirectional`; state `ANTICOGGING_CALIBRATION`). Walkthroughs: [Puccinelli](https://robertpuccinelli.com/resources/2021/05/25/odrive-anticogging.html), [Vickers](https://www.andyvickers.net/2021/02/19/implementing-anti-cogging-on-the-odrive-robotics-controller/)

### Academic basis — Piccoli & Yim "Anticogging" (ModLab UPenn)
- Position-indexed cogging map measured via the existing encoder, played back as feed-forward, iteratively refined — the paper ODrive's method derives from. ([page](https://www.modlabupenn.org/anticogging/), [PDF](https://www.modlabupenn.org/wp-content/uploads/piccoli_matthew_anticogging_torque_ripple_suppression_modeling_and_parameter_selection.pdf))
- **Proven:** Best-paper finalist RSS 2014 / IJRR; across 11 motors, compensated cheap motors beat the best uncompensated motor, removing **up to 88% of torque ripple**. Best published validation for the LUT approach + guidance on map resolution / parameter selection.

### moteus (mjbots) — has cogging compensation
- Position-dependent torque-comp table (added 2022-07-11, sign fixes 2024-05-20); also position-loop `ki`/`ilimit` to absorb cogging at the cost of bandwidth. Author: "works pretty well on a number of motor types." ([release](https://blog.mjbots.com/2022/07/13/moteus-firmware-release-2022-07-11/), [reference.md](https://github.com/mjbots/moteus/blob/main/docs/reference.md))

### NO turnkey anti-cogging (reference only)
- **SimpleFOC:** `CalibratedSensor` corrects encoder eccentricity/offset **only, not cogging**; community LUT threads unmerged. ([drivers repo](https://github.com/simplefoc/Arduino-FOC-drivers), [collab thread](https://community.simplefoc.com/t/anyone-interested-in-collaborating-on-anti-cogging-lowest-hanging-fruit-for-smoother-motion/2560))
- **VESC:** only dead-time distortion comp; anti-cogging requested, not implemented. ([request](https://vesc-project.com/node/3383))
- **MESC:** FOC/MTPA/HFI/observer but no cogging map. **Tinymovr:** velocity-integrator deadband + eccentricity cal, no cogging FF map.

### Academic upgrades (no plug-in STM32 lib)
- **Iterative Learning Control (ILC):** exploits per-rev periodicity to iteratively refine a q-axis correction — principled upgrade over a static LUT, converges online. ([SMA-ILC](https://pmc.ncbi.nlm.nih.gov/articles/PMC10708707/), [PD-type ILC](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9847981/))
- **Harmonic/Fourier injection:** model cogging as a few harmonics of mechanical angle (orders = multiples of slot/pole count), inject canceling current — compact, noise-robust vs a dense LUT. ([MDPI](https://www.mdpi.com/2079-9292/15/6/1240))

### Ranked recommendation (you: 20 pp, ~700 rpm, AS5047P, MCSDK, already have a position-indexed Iq LUT)
1. **Keep your LUT, add ILC refinement** — average residual Iq/position error over many revs and fold back into the table (self-correcting vs one-shot). Reference: ODrive `controller.cpp` coarse+fine integrator-gain calibration.
2. **Store the LUT as harmonic coefficients** — with 20 pp the dominant orders are well-defined; a few coefficients are more noise-robust at 700 rpm than a dense table and cheap on STM32.
3. **Advance the lookup by velocity×latency** — ODrive's own warning: FF keyed on a *delayed* angle erodes stability margins. Index on the phase-advanced angle, not raw measured. (Direct overlap with topic D.)
4. **moteus** = the only other working open reference; SimpleFOC/VESC/MESC/Tinymovr have no turnkey code.

## Job B-deep — implementation detail (ODrive calib + ILC + harmonic)
**ODrive `anticogging_calibration()`** (`controller.cpp`): park at each of **3600 bins/rev**, capture the settled velocity-integrator torque once `|pos_err| ≤ calib_pos_threshold/cpr && |vel| < calib_vel_threshold/cpr` → `cogging_map[index++] = vel_integrator_torque_`. Coarse pass uses `calib_coarse_integrator_gain` (default 25) for fast settle; fine pass refines. `remove_anticogging_bias()` subtracts the map mean (cogging is zero-mean). Apply: `torque += cogging_map[mod(pos,3600)]`. Persist via `pre_calibrated`.
**ILC update law** (per rev, P-type w/ forgetting α≈0.07): `map[k] = (1-α)·map[k] + L·resid_filt[k]`. Convergence needs `|Q(z)(1−L·G(z))|<1` → practically `0<L·G<2`, L≈0.2–0.8/plant-DC-gain. Q-filter (zero-phase LPF) the residual; interpolate between bins; subtract rev-average. Runs offline; deploy as live FF. ([Zhu et al. Sensors 2022, PMC9103240](https://pmc.ncbi.nlm.nih.gov/articles/PMC9103240/))
**Harmonic/Fourier**: cogging period `N_c = LCM(Q_slots, 2p)` cycles/rev; fit `τ(θ)=Σ a_i cos(iN_cθ)+b_i sin(iN_cθ)` via least-squares `ĉ=(ΦᵀΦ)⁻¹Φᵀy` on slow-spin (θ,Iq) samples; ~6 coeffs replace the 3600-float table, smoother + RAM-cheap. For 2p=40: with 24 slots N_c=120, with 48 slots N_c=240 (orders N_c,2N_c,3N_c). (full transcript: `tasks/ae8bc703612405c7f.output`)


