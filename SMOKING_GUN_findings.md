# Anti-cogging "doesn't cancel even fresh" — code-level smoking gun

> Written while the deep-research web run (`wxiv0xorw`) was in flight. This is the
> **code-confirmed** half; merge with the cited web report when it lands.

## The symptom
- Freshly calibrated map is the right **magnitude** (raw cogging ≈ ±175, map peak ≈ ±155)
  and loads byte-identical every reboot, commutation aligned (avgId≈0).
- Yet cogging is felt with the FF **ON and OFF** — the FF neither cancels nor clearly
  doubles it. So it's a **phase/timing** failure, not a magnitude or persistence failure.

## Smoking gun #1 (code-confirmed): the anti-cogging FF is applied LATE and at only 1 kHz, with NO velocity extrapolation — while calibration happens at ~9 rpm (≈zero latency)

Evidence in `mc_tasks_foc.c` / `parameters_conversion.h`:
- `MEDIUM_FREQUENCY_TASK_RATE = SPEED_LOOP_FREQUENCY_HZ = 1000 Hz`. The FF updates at **1 kHz** (1 ms zero-order hold), not the 25 kHz current loop.
- `FOC_CalcCurrRef()` (which does `FOCVars[M1].Iqdref.q += ff`) is called **inside `TSK_MediumFrequencyTaskM1`** (lines 651/715/773).
- `Ropetow_EncoderUpdate()` (which publishes `g_enc_mech14`) runs in `MC_APP_PostMediumFrequencyHook_M1`, i.e. **AFTER** the MF task. → `FOC_CalcCurrRef` always reads the **previous** cycle's `g_enc_mech14` ⇒ **≥1 ms stale**, then holds that Iqref constant for the next 1 ms.
- The anti-cogging lookup uses the **raw** angle: `uint16_t m = g_enc_mech14;` (line ~932) — **no velocity extrapolation**. Compare commutation, which IS extrapolated: `a = g_enc_theta0 + g_enc_omega_tick*(dt + g_enc_ff_ticks)`. So commutation is latency-compensated; anti-cogging is not.

### Why this destroys cancellation at back-drive speed (the math)
Cogging content is at **1× electrical = 20× mechanical**. Electrical freq vs mech speed:
| mech speed | f_elec = rpm/60·20 | phase lag from 1 ms delay = 360·f·0.001 |
|---|---|---|
| 100 rpm | 33 Hz | **12°** (ok) |
| 300 rpm | 100 Hz | **36°** (cancellation → ~0.8×, degrading) |
| 600 rpm | 200 Hz | **72°** (cos72°=0.3 → barely helps; past 90° it AMPLIFIES) |
| 700 rpm | 233 Hz | **84°** + only ~4 samples/cycle at 1 kHz (near Nyquist) |

- **Calibration is done at ~9 rpm** (COGG_CAL_STEP sweep) → f_elec ≈ 3 Hz → 1 ms lag ≈ 1° → the map is captured at essentially the **true** position.
- **Playback happens while the user yanks the cable to hundreds of rpm** → the same map is applied 36–84° electrical **late** → it no longer lines up with the cogging → no cancellation, and above ~90° lag it makes cogging **worse**.
- This exactly matches "fresh map also fails" (fresh doesn't help because you test it *at speed*) and "cogging both ways" (off = raw cogging; on = mistimed FF ≈ same or worse).

### Clean test to confirm (do this first when back)
Pull the cable **very slowly** vs **fast**, FF ON:
- Smooth when slow, coggy when fast → **confirms latency** (smoking gun #1).
- Coggy even when crawling → there's ALSO a low-speed cause (encoder INL / index phase — see #2).

### Fixes (in order of effort)
1. **Velocity-extrapolate the anti-cogging index**, mirroring the commutation FF:
   `m_lookup = g_enc_mech14 + ω_mech_counts · (latency_ticks + cogg_ff_ticks)` before `>> COGG_SHIFT`. One-line-ish, reuses the existing `g_enc_omega`/`g_enc_ff_ticks` machinery. Expose a tunable lead so you can dial the phase like `G` did for commutation.
2. **Move the lookup+injection into the HF (25 kHz) loop** with a fresh, extrapolated angle — proper fix, kills the 1 kHz ZOH and the 1-cycle staleness.
3. **Calibrate at a representative speed** (not 9 rpm) so capture and playback share the same latency — weaker fix; #1/#2 are better.

## Smoking gun #2 (hypothesis, for the web report to confirm): low-speed residual
If it's coggy even crawling, candidates (all in the deep-research prompt):
- AS5047P **INL ±0.8° mech = ±16° electrical** shifts the 1×-electrical map component → partial cancellation only.
- **DAEC enabled**: speed-dependent angle prediction, no reversal predictor — corrupts a low-speed/reversing map. Datasheet says disable <100 rpm.
- q-axis-current FF cancelling a **current-independent reluctance** cogging: verify the calibration-by-holding-current actually captures it in a playback-valid way.
- Capture/playback **index consistency**: capture (EncoderUpdate, current angle) vs playback (FOC_CalcCurrRef, previous angle) differ by one MF cycle even at cal — negligible at 9 rpm but worth eliminating.

## ✅ Deep-research verdict (task wxiv0xorw) — INDEPENDENTLY CONFIRMS the code finding

19/25 claims verified 3-0. The web research reached the **same** conclusion as the code
analysis: **the failure is phase/timing decorrelation, not magnitude, method, or persistence.**

Ranked root causes (cited):
1. **PRIMARY — latency/phase decorrelation (high, 3-0).** FF computed from a ~1 ms-stale
   1 kHz index, applied to ripple at the *electrical* frequency, at a *continuously
   varying* back-drive speed → the phase error (ω·t_delay) shifts run-to-run, so the FF
   lands at an uncorrelated phase and *on average neither cancels nor doubles* — the exact
   reported signature. Piccoli/Yim ran their current loop at 100 kHz ≫ encoder rate so the
   loop was never limiting; here the 1 kHz FF task IS the limiter. US Patent 7,952,308:
   control/PWM delay = an angle error that MUST be corrected by adding a compensating angle,
   with *more* correction for higher-order (electrical-freq) harmonics.
   **FIX: evaluate the table in the HF loop with a fresh angle, and/or add a
   velocity-proportional phase-advance `bins = ω_mech · t_delay` to the playback index.**
2. **Capture-vs-playback phase must match exactly (high, 3-0).** Kollmorgen: "the reference
   point must always be at the same position as during generation of the table." Because
   the latency offset differs between the constant ~9 rpm cal and the varying-speed
   playback, the effective phase differs even with a byte-identical map + same raw index.
   **FIX: match the delay at capture and playback; DIAGNOSTIC: deliberately shift the table
   ±N bins and confirm a cancellation optimum exists (proves it's phase).**
3. **DAEC enabled (high, 3-0).** ANGLECOM applies DAE = speed·t_delay — ~0 at the 9 rpm
   cal, but nonzero and sign-flipping through back-drive reversals at playback → capture
   and playback angles differ by a speed-dependent amount. **FIX: set DAECDIS, read
   ANGLEUNC (0x3FFE), recalibrate.** (Secondary — magnitude is small at these speeds.)
4. **Mechanical indexing + encoder INL (medium, 2-1).** ±0.8° mech = ±16° electrical on a
   20-pp motor; one electrical cycle spans only ~25.6 bins → INL/eccentricity misplace the
   dominant electrical-frequency content differently at different rotor positions, so one
   static mechanical table can't align everywhere at once. **This mechanism is
   speed-INDEPENDENT → it also explains cogging felt when crawling.** FIX: correct encoder
   INL/eccentricity before building the table (and/or index the dominant order by
   electrical angle).

Ruled OUT / not the bug (all cited):
- q-axis current FF **is** a legitimate way to cancel current-independent cogging (3-0). Method is fine.
- The DFT/harmonic denoise is correct — zero phase lag; do NOT replace with a causal LPF (3-0).
- Persistence/reboot ruled out earlier (byte-identical map, avgId≈0).

## Note on architecture (matches real products)
Real drives calibrate encoder-error + cogging **once**, store to flash (done), and at boot do only a quick commutation align (or skip it via a stored absolute offset). The fix here is not "recalibrate each boot" — it's making **playback phase match capture phase** (velocity extrapolation / HF injection), plus calibrating the **encoder** before the cogging map.
