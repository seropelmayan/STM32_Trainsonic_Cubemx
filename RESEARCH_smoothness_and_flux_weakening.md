# Deep Research — Smoothness Calibration + Flux Weakening (to ≥750 rpm)

> Ropetow / STM32G431 / MCSDK 6.4.2 / 20 pole-pair SPMSM / AS5047P / 54.6 V bus.
> Two deep-research passes (fan-out web search → source fetch → adversarial verify).
> The verify stage was cut short by an Anthropic session rate-limit, so this is a
> hand-synthesis of the fetched claims. Smoothness claims marked ✓ were verified 3-0.
> Flux-weakening claims came back "unverified" only because the voter never ran —
> the sources (ODrive docs, MathWorks, Microchip, imperix, ST Community) are reputable.

---

# PART 1 — EXTREME SMOOTHNESS (ranked by impact)

## #1 (biggest lever) — Calibrate the ENCODER before the cogging map
Your AS5047P has **INL ±0.8° mechanical @25 °C** (±1° over temp, ±1.2° with magnet
displacement) ✓ [ams DS000324]. On a 20-pole-pair motor that is **×20 into the
electrical angle → ±16° electrical**. That error does two bad things:

1. It corrupts **commutation** (wrong θ → Id/Iq cross-leak → torque ripple that grows
   with speed — the same family as the buzz we already chased).
2. It **pollutes the anti-cogging LUT**: your park-and-measure "learns" the encoder
   angle error as if it were cogging. But encoder error is a *different* function of
   position than cogging, so replaying it as an Iq FF can never fully cancel either —
   you get residual roughness that no amount of ILC removes. This is very likely your
   current #1 limiter on "light-load roughness."

**Fix — one-time encoder self-cal (Ben Katz / BuildIts method, the canonical portable one):**
- Spin the rotor **open-loop at a slow constant electrical velocity** (pure voltage
  vector rotating at fixed rate). At constant speed the *true* mechanical angle advances
  linearly, so any deviation of the encoder reading from the linear ramp is the encoder
  error map `e(θ_mech)`.
- Record `raw_encoder − expected_linear` across a full mechanical revolution → store as a
  correction LUT (or, better, as a few harmonics — eccentricity is dominated by the
  **1st mechanical harmonic**, plus a handful of INL harmonics).
- Subtract `e(θ)` from every angle read at runtime **before** commutation and **before**
  the cogging LUT lookup.
- Verified corollary: this per-position correction is **velocity-independent** — coeffs
  measured from ~950 to ~3800 rpm were identical ✓ [Sensors 21/4763]. Calibrate once,
  reuse at all speeds, zero added latency (it's a feed-forward subtract, not a filter).
- Related academic method: gradient-descent **Harmonic Rejection + dual-PLL** rejects DC
  offset, amplitude mismatch, and low/high-order harmonics of a magnetic encoder ✓
  [ResearchGate 327638879].

**DAEC note:** disabling AS5047P DAEC gives **0.016° rms** less noise and ams explicitly
recommends it **<100 rpm / static** ✓ [ams DS000324]. We reverted DAEC-disable because it
was *bundled* with the direction-split that felt worse — but DAEC-disable **on its own**
is a low-speed win. Worth re-testing in isolation (single `s1 |= DAECDIS` bit) after
encoder cal.

**Order of operations that actually converges:** encoder cal → (DAEC choice) → THEN
re-run anti-cogging cal. Doing cogging first over a dirty encoder bakes the error in.

## #2 — Replace naive velocity differencing with a tracking observer
Single-sample differencing of a 14-bit encoder quantizes to ~91 rpm/LSB and
**differentiation-with-LPF is the *worst* low-speed method** — the derivative amplifies
quantization noise, worse as resolution drops and speed falls ✓ [MDPI 16/11/595]. This is
your near-standstill roughness and reversal chatter.

Two portable upgrades:
- **PLL / PI tracking loop** (Jason Sachs, embeddedrelated #530): velocity = output of a
  PI loop driven by position error; gives a smooth velocity with no differentiation. You
  already have a PLL (`g_pll_enable`, default off) — the research says this is the right
  structure; the earlier "PLL made noise worse" was almost certainly mistuning, not the
  method.
- **3rd-order ESO with reference-torque feed-forward** (the strongest result): triple-pole
  placement at `ω0 = 400 rad/s` → `l1=3·ω0, l2=3·ω0², l3=ω0³`, disturbance-comp gain
  `0.2`. Reported **−42% steady-state speed fluctuation vs standard ESO, −90%+ vs
  differentiation** ✓ [MDPI 16/11/595]. Feeding the *commanded* torque (which you know
  exactly) instead of feedback current is the key trick.

## #3 — Dead-time / inverter-nonlinearity compensation (your light-load roughness)
Dead-time distortion is **worst at low current / low modulation** — exactly your light
braking regime ✓ [ScienceDirect S2405896316326441; ResearchGate 263150742]. Three proven,
parameter-light schemes:
- **Disturbance-voltage observer**: estimates the combined dead-time + nonlinearity
  voltage error online, no extra hardware, no offline measurement, adds it back to Vref ✓.
- **d-axis harmonic minimizer**: a PI drives the *sum of squared Id between phase-current
  zero-crossings* to zero; output is a slow compensation voltage. Parameter-free ✓.
- **AFC (Adaptive Feed-forward Cancellation)** (Ben Katz): LMS adapts a sine/cos pair at a
  target harmonic → effectively *infinite loop gain at that frequency* → perfectly rejects
  that current/torque harmonic. Run one AFC per dominant ripple harmonic. This is the
  real-time cousin of your offline DFT denoise.

## #4 — Current-sense per-phase offset + gain calibration
Per-phase **offset** error and **gain mismatch** between the three shunt/CSA channels
inject **1st and 2nd electrical-harmonic** torque ripple. SimpleFOC does zero-current
offset cal automatically at init; add a **per-phase gain match** on top [SimpleFOC docs].
Cheap to add, removes a steady low-order ripple floor.

## #5 — Anti-cogging map refinements (you already have the base)
- Cogging is **position-locked and current-independent** — so a **fixed** LUT is correct;
  it should *not* scale with commanded current [ODrive anticogging]. Your instinct to keep
  it fixed is right. The light-load problem is *accuracy*, not scaling → fix via #1.
- **Fourier Series Controller (adaptive self-tuning)**: model ripple as
  `T_r(θ) = Σ_k b_k·sin(kθ)` and adapt the `b_k` online from the measured harmonic
  distortion — no prior model needed ✓ [PMC3658778]. This is an *online-learning* upgrade
  to your one-shot DFT denoise: it keeps the map correct as temperature/wear drift.
- For a 20-pole-pair machine, target the **measured** dominant spatial harmonics (cogging
  appears at the stator-slot fundamental and its harmonics per mechanical rev; current/
  dead-time ripple appears at the **6th electrical** harmonic). Let the DFT tell you which
  bins carry energy and only compensate those — denoises the map without smearing it.

## Which of your current steps is limiting you
- **Limiting:** naive velocity estimate (#2) and an **un-calibrated encoder** (#1) —
  together these are almost certainly the bulk of the residual roughness.
- **Good, keep:** fixed position-indexed LUT, fwd/rev averaging to split direction-locked
  ripple, ILC, standstill freeze, 10 A stiff cal servo.
- **Next high-impact additions, in order:** encoder self-cal → tracking observer (ESO/PLL)
  → dead-time comp (observer or AFC) → current-sense gain match.

---

# PART 2 — FLUX WEAKENING TO ≥750 rpm

## ⚠️ The gotcha that matters most for YOUR use case
You run **constant braking torque = Current/Torque Control Mode**. ST's own community
warns: **MCSDK voltage-feedback flux weakening in Torque Control Mode reduces Iq and
*overwrites your current reference* → loss of torque control** [ST Community 598438]. ST
does **not** recommend enabling the built-in FW in torque mode. So simply switching on
MCSDK FW will make it fight your braking command. Plan around this (custom Id-only FW,
below).

## Root cause (confirmed math)
Vq is the **output of the q-axis current PI** [ST Community 742549]. As speed rises the
required voltage grows until `|V| = √(Vd²+Vq²)` hits the modulation ceiling
`Vmax = m·Vbus/√3`; the PI then **saturates → SVPWM clips → overmodulation → buzz**
[industrialmonitordirect; imperix]. For your SPMSM:
```
Vd ≈ R·Id − ω_e·Lq·Iq
Vq ≈ R·Iq + ω_e·(λ_PM + Ld·Id)      ω_e = rpm/60 · 20 · 2π   (250 Hz-ish at 750 rpm)
```
The `ω_e·λ_PM` back-EMF term is what runs you into the wall. **Negative Id** shrinks the
effective flux `(λ_PM + Ld·Id)` → lowers the Vq demand → buys speed.

## The levers, ranked by impact-per-effort

**1. Use more of the bus you already have (do this FIRST — free speed, no FW).**
SVPWM already reaches `m = 2/√3 ≈ 1.15` — **~15% more voltage than SPWM** [imperix]. Your
log showed Vq at **89%** then clipping. If MCSDK's circle-limit / `MAX_MODULATION_INDEX`
is set conservative, **raise it toward ~95–100%** and you gain base speed with *zero* FW
and *zero* extra current. ODrive explicitly raised its modulation cap **80%→100%** in
fw0.6.1 purely to get more top speed on the same bus [ODrive changelog]. This alone may
get you most of the way to 750.

**2. BEMF + cross-coupling feed-forward decoupling (this is a big part of "why ODrive is clean").**
Add to the PI outputs so the current loop stops *fighting* the back-EMF:
```
VdFF = −ω_e · Lq · Iq
VqFF = +ω_e · (Ld · Id + λ_PM)
```
[MathWorks pmsmfeedforwardcontrol; verified FF forms in smoothness pass]. ODrive exposes
exactly three FF terms — **ωL cross-coupling, back-EMF, and dI/dt** — as independently
enable-able [ODrive API]. With decoupling, the PI only trims the *residual*, so it stops
thrashing near the wall and the buzz drops even before FW engages.

**3. Voltage-feedback (modulation-index) FW engagement — NOT a speed threshold.**
Trigger FW on `M = √(Vd²+Vq²)/Vmax` crossing a setpoint, output a factor `β∈[0,1]` (or a
negative Id command) [MathWorks pmsmfieldweakeningcontroller]. This is inherently
**chatter-free under your impulsive cable pulls** because it reacts to *actual* voltage
saturation, not to speed crossing a line. Your present fixed −1 A on a speed threshold is
both **too weak** and **chatters** — replace it.

**4. Controlled overmodulation.**
Beyond `m=1` you can extract ~**3–7% extra** voltage practically (theoretical six-step
limit `2√3/π ≈ 1.103`) by capping modulation around **1.05–1.15** with a
**minimum-distortion** limiter to keep acoustic noise down [Microchip MCAF overmodulation].
A little overmodulation + full bus utilization can raise base speed enough that FW barely
has to work.

**5. Anti-chatter shaping.** Hysteresis + rate-limit (ramp) on the FW Id command so regen↔
motor transitions during pulls don't step-change Id.

**6. Raise "max application speed" in MC Workbench.** Even with FW working, MCSDK **caps**
the motor at the configured max application speed — you must raise it or you stay limited
regardless [ST Community 635860].

## The ODrive-vs-MCSDK difference, and how to close it
ODrive stays clean to 700 rpm on your exact motor/battery because it combines: (a) **100%
modulation utilization**, (b) **full BEMF + cross-coupling + dI/dt feed-forward
decoupling**, and (c) a **modulation-index-based FW** regulator (`fw_enable`,
`fw_mod_setpoint`, `fw_fb_bandwidth`) [ODrive API/changelog]. MCSDK defaults are more
conservative on modulation, its FW conflicts with torque mode, and it has **no BEMF FF by
default**. Closing the gap = items 1 + 2 + 3 above.

## Recommended concrete path for Ropetow (torque mode, target 750 rpm)
Because MCSDK's built-in FW hijacks Iq in torque mode, implement a **custom, Id-only FW**
that never touches your braking Iq:
1. **Raise the modulation ceiling** toward ~95% and re-measure the clean top speed.
2. **Add VdFF/VqFF decoupling** (the two equations above) using your known ω, Id, Iq and
   motor `Ld, Lq(≈Ld), λ_PM`. You already log/compute ω; λ_PM comes from the MCSDK motor
   profile (or back-EMF constant).
3. **Custom FW PI on modulation index** → outputs **negative Id only** (feeds your existing
   `Ropetow_SetMcFwDemag` / `FW_M1` Id path). Leave **Iq = your constant braking command
   untouched.** Engage on `M > ~0.92`, release with hysteresis (~0.88), rate-limit Id.
4. Allow modulation up to ~1.05–1.10 (min-distortion) if still short of 750.
5. Bump **max application speed** in the Workbench so nothing caps you.

**Feasibility / caution:** at 750 rpm (`f_e ≈ 250 Hz`) the required
`|Id_fw| ≈ (ω_e·λ_PM − Vmax) / (ω_e·Ld)`. Low phase inductance means you may need a fair
few amps of negative Id for a given flux knockdown — you have ~29 A headroom so it's
feasible, but negative Id is pure loss (heat, no torque). Watch winding/FET temperature
and set a sane Id floor.

---

## Source quality notes
- **Primary/verified:** ams AS5047P datasheet, MDPI (velocity observers, encoder
  misalignment), PMC (Fourier ripple control), ScienceDirect/ResearchGate (dead-time),
  Microchip MCAF, ODrive docs, MathWorks.
- **Forum (directional, not gospel):** ST Community threads on FW/torque-mode and
  "max application speed" — consistent with the primary sources and with ST's design.
- Two IEEE/TI links returned no extractable content (paywalled/unreliable) and were dropped.
