# Task: Implement an encoder non-linearity (INL) calibration table for a PMSM FOC drive

## Objective
Implement a runtime correction that cancels the position-dependent angle error of a
magnetic absolute encoder, to remove a once-per-mechanical-revolution torque ripple.
You will (1) collect measured-vs-expected angle data, (2) build a correction lookup
table indexed by **mechanical** angle, and (3) apply it in the angle path before the
electrical angle is derived. Persist the table and provide a validation routine.

## System context (verify each item against the actual code before acting)
- MCU/framework: **STM32G4 series** running **ST MCSDK 6.2** (MC Workbench–generated
  FOC). The G4 core is Cortex-M4F with a single-precision hardware FPU and CORDIC/FMAC
  math accelerators — single-precision float in the control ISR is cheap, so the runtime
  correction can use float without a fixed-point rewrite. Confirm the exact G4 part number
  and whether MCSDK is configured to use the CORDIC for trig.
- Motor: PMSM, **20 pole pairs**. Electrical angle = mechanical angle × 20.
- Encoder: AMS **AS5047P**, read over **SPI**, 14-bit (16384 counts per mechanical
  revolution). Confirm whether the code reads **ANGLECOM** (DAEC-compensated, register
  0x3FFF) or ANGLEUNC; the calibration and runtime paths MUST use the same one.
- Root cause being fixed: the encoder has a low-spatial-frequency angle error vs.
  mechanical position (magnet eccentricity / INL, on the order of ±1° mechanical,
  dominated by the 1st mechanical harmonic). Because electrical angle = mechanical × 20,
  this error is multiplied ~20× into the electrical angle (up to ~±20° electrical),
  corrupting the Park/inverse-Park transforms once per mechanical revolution and causing
  audible/tactile torque ripple. The error is a **repeatable function of mechanical
  angle**, which is what makes a table-based correction possible.

## Phase 0 — Recon (do this first; report findings and a plan BEFORE writing code)
Inspect the codebase and report:
1. The encoder read function: where the raw SPI angle is acquired and stored.
2. Where mechanical angle is converted to electrical angle (the ×pole_pairs step), and
   how the alignment/offset is applied. In MCSDK 6.2 this lives in the speed/position
   feedback component for the encoder (look for the ENCODER component handle and the
   get-electrical-angle / get-mechanical-angle accessors it exposes, plus the SPI driver
   that reads the AS5047P). Apply the correction in the **mechanical angle** domain inside
   this component, before electrical angle is formed and before any CORDIC trig — confirm
   the exact functions in the actual code rather than assuming names.
3. The control-loop rate (the ISR that consumes the angle) and its timing budget.
4. Telemetry: a **USB CDC** (virtual COM port) link is available for logging — use it to
   stream calibration samples to a host PC. Locate the existing CDC TX path and confirm
   its throughput and whether it transmits from an interrupt/DMA or by polling.
5. Flash storage options for persisting the table (and how MCSDK lays out flash so you
   don't collide with its data).
6. The sign/direction conventions: the AS5047P DIR bit, the timer/count direction, and
   the sign used when forming electrical angle. (These must be consistent end-to-end —
   a single sign flip turns the correction into a doubling of the error.)
Then propose the integration point and table format before implementing.

## Phase 1 — Calibration data collection
Spin the motor at **low, constant speed under light or no load**. Low speed is required
so that the speed-proportional SPI-latency angle error is negligible relative to the INL
you are trying to measure; otherwise it pollutes the table.

Pick ONE reference method based on what the codebase supports:
- **(A) Open-loop forced commutation (preferred).** Drive a rotating voltage vector at a
  constant electrical speed so the rotor follows the commanded angle. Reference =
  commanded mechanical angle (= unwrapped commanded electrical angle / 20). Log measured
  encoder angle vs. commanded angle. This decouples collection from the (currently
  faulty) position feedback, giving clean data.
- **(B) Closed-loop constant speed.** Log the unwrapped measured angle vs. time over many
  revolutions; the expected angle is the best-fit constant-speed line (linear regression
  or a PLL tracking the average speed). Error = measured − expected. Use only if open-loop
  forcing is impractical.

Collect **≥20–50 full mechanical revolutions** and average, to suppress noise, speed
ripple, and cogging interaction. Unwrap angle to handle the 0/2π wrap. Log raw counts
plus timestamps.

Stream each sample over **USB CDC** to the host as it is collected (or buffer in RAM and
stream in bursts). **Do not perform CDC writes from inside the high-priority control
ISR** — push samples into a ring buffer in the ISR and transmit them from the main loop
or a lower-priority context, so logging never perturbs loop timing (which would distort
the very angle you are measuring). Prefer a compact binary frame (raw count + timestamp)
over ASCII for throughput, with a sync marker and a length/CRC so the host can re-frame.

## Phase 2 — Build the correction table (on the host, in Python)
The firmware streams raw samples over USB CDC; **do the table computation on a host PC in
Python** for this first iteration, so the curve can be plotted and sanity-checked before
it is trusted. Provide that host script as a deliverable. It should:
- Define a table indexed by **MECHANICAL** angle over [0, 2π_mech): start with N = 512
  bins (INL is low spatial frequency; large tables gain little). The native 16384 counts
  can be down-sampled to N.
- For each sample, error = wrap_to_pi(measured_mech − expected_mech). Accumulate into the
  bin for that mechanical angle; average per bin across all revolutions.
- Decide and document whether to subtract the mean error (so the table is pure INL and the
  existing alignment offset stays separate) or fold it in. Keep the 1st mechanical harmonic
  (eccentricity) and the next few harmonics — those carry the correction.
- Enforce circular continuity (bin[N-1] wraps to bin[0]); apply light smoothing.
- Optional compact form: fit the first several mechanical harmonics (Fourier) instead of a
  raw LUT — eccentricity is dominated by the 1st harmonic, so a handful of terms reproduce
  the curve smoothly and cheaply.
- **Plot** the captured error vs. mechanical angle (and the fit) for visual confirmation.
- **Emit a ready-to-compile C const array** (in encoder counts or radians — match the
  runtime path) for embedding, or a binary blob to write to flash.
Then the table is delivered to the target either by recompiling with the embedded array,
or by sending the blob back over USB CDC to a flash-write command on the device (your
choice — state which you implement).

## Phase 3 — Apply the correction at runtime
In the angle path, after reading the raw mechanical angle and BEFORE computing electrical
angle:
```
corr      = lut_interp(raw_mech_angle)          // linear interp between adjacent bins
mech_corr = wrap(raw_mech_angle - corr)         // verify sign against DIR convention
elec      = wrap(mech_corr * POLE_PAIRS)         // POLE_PAIRS = 20
```
Constraints:
- O(1), branch-light, same numeric type (fixed/float) as the existing code; must fit the
  control-ISR budget. Pre-scale the interpolation index for speed.
- Guard against applying an unpopulated/invalid table (validity flag + CRC).

## Phase 4 — Persistence and integration
- Persist the table to flash; load and CRC-check at boot.
- **STM32G4 flash specifics:** flash is programmed in 64-bit **doublewords** with ECC, and
  a doubleword cannot be re-programmed without erasing its whole page first (pages are
  2 KB; some G4 parts are dual-bank). Reserve a free flash page that the linker script and
  MCSDK do not already use, erase-then-write the table there, and pad the table to a
  doubleword boundary. Do not assume you can patch the array in place.
- Provide a command/trigger to (re)run calibration and overwrite the stored table.
- Respect the existing startup sequence: power-on → wait for encoder valid (AS5047P needs
  up to ~10 ms) → load table → run MCSDK encoder alignment → normal run. Do not disturb
  the existing alignment step.

## Phase 5 — Validation (required)
- Re-run the Phase 1 measurement with correction enabled, streaming over USB CDC, and
  report peak-to-peak angle error before vs. after, in both mechanical and electrical
  degrees (target: residual electrical error down to a few degrees or less).
- Capture a control-quality metric before vs. after — e.g. Iq ripple or measured-speed
  ripple at the 3.33 Hz mechanical rate (the once-per-rev component) — and report the
  reduction.
- Have the host script plot the before/after error curves overlaid so the user can see
  the eccentricity flattened.

## Make configurable
Calibration speed, revolution count, table size N, reference method (A/B), and storage
location should all be parameters, not hard-coded.

## Critical cautions (repeat-offenders)
- Index the table by **mechanical** angle, never electrical.
- Keep sign/direction consistent end-to-end; verify empirically that enabling the table
  REDUCES error (if it increases, flip the correction sign).
- Calibrate at low speed so SPI-latency error doesn't enter the table.
- Use the same angle source (ANGLECOM vs ANGLEUNC; DAEC on/off) in calibration and at
  runtime — do not mix.

## Deliverables
1. Recon report + integration plan (Phase 0).
2. Calibration data-collection routine that streams over USB CDC (Phase 1).
3. Host-side Python script: parses the CDC stream, builds + plots the table, emits the C
   const array / flash blob (Phase 2).
4. Runtime correction hook (Phase 3).
5. Flash persistence + boot load (Phase 4).
6. Validation routine with before/after metrics and overlaid plots (Phase 5).

Before implementing, confirm against the actual code: the pole-pair count (20), the
encoder angle source in use (ANGLECOM vs ANGLEUNC), the MCSDK 6.2 encoder component's
angle accessors and where to hook the correction, and the chosen reference method — do
not assume.