# ESP-side rewind derating — ready-to-review patch (draft 2026-07-10)

Target repo: `C:\Users\serop\Desktop\smart trainer\version 8\Software\Main\trainsonic-firmware`
Target file: `components/motor_control/resistance.c` (+3 lines in `resistance.h` docs).
**Nothing has been applied — this is a review document.** Apply next session with the
ESP repo open.

## What it does

When the cable is rewinding fast (nobody holding it), fade the retract torque from the
full weight setting down to a small "keep-tension" floor (~2 A ≈ 7 kg), and restore it
smoothly as the rewind slows. Fixes the two felt problems in one move:

1. **Free release**: a dumped handle no longer gets accelerated home at 90 kg-equivalent
   force — the drum spools in briskly but gently, staying far away from the STM32
   governor/FW region (the "weird buzzing on release" territory).
2. **Soft catch**: if the user grabs the flying cable mid-rewind, the weight blends back
   in over the fade band instead of hitting like a wall.

Pulls are completely untouched: the derate only applies to inbound (rewind) velocity
while the command is retract-direction. Pull purity is preserved end to end.

## Why torque derating and not the speed cap

`motor_control.c:136` already documents the failed alternative: *"the STM32 governor
limits BOTH directions, so any lower cap throttles the pull-out too."* (Note: that
comment predates the 2026-07-10 STM32 pull-purity redesign — the governor brake now
acts on motor-driven motion only — but the cap approach is still worse: it *brakes*
against the full weight command, burning current to fight ourselves, instead of just
not commanding the force. Keep `speed_limit_rpm = DEFAULT_SPEED_LIMIT_RPM`.)

Design rules carried over from the STM32 governor bench war (all were felt bugs):
- **Memoryless, continuous fade** — a pure function of current velocity. No state, no
  LPF (nothing to leak across direction changes), no binary gate (nothing to relay-
  oscillate), no `>0` comparison against a filtered value.
- Velocity source is the STM32's clean 1 kHz speed from telemetry (`status->speed_rpm`),
  the same signal that already fixed the wall buzz — magnitude and sign from one sensor.

## The patch

### 1. `resistance.c` — add tunables (near the other `static float` knobs, ~line 70)

```c
/* --- rewind derating: fade retract torque to a floor when the cable rewinds
 * fast with nobody holding it (free release). Memoryless fade = pure function
 * of the clean STM32 velocity: no state, no gate, nothing to limit-cycle.
 * Restores full weight smoothly as the rewind slows (soft catch on re-grab). */
static float s_rw_start_rpm = 200.0f;  /* inbound drum rpm where derating begins (~0.47 m/s) */
static float s_rw_full_rpm  = 320.0f;  /* inbound rpm where the floor is reached  (~0.75 m/s) */
static float s_rw_floor_mA  = 2000.0f; /* retract torque at full derate (~7.4 kg keep-tension) */
```

### 2. `resistance.c` — apply the fade in `resistance_tick()`

Insert **after the weight law** (after the `if/else` chain that sets `cmd_f` from
`full_mA`, ~line 186) and **before the home-wall spring block**, so the wall/spring
behavior at home is untouched:

```c
    /* --- rewind derating (see header comment at the knobs) -------------------
       Only when the command is retract-direction AND motion is inbound faster
       than the start threshold. Fade linearly to the floor; never fade UP (a
       light weight setting below the floor stays as-is). Pulls (v >= 0) and the
       home wall are untouched. */
    if (cmd_f < 0.0f && s_vel_m_s < 0.0f) {
        float in_rpm = -s_vel_m_s / RPM_TO_MPS;            /* inbound speed, rpm, >0 */
        if (in_rpm > s_rw_start_rpm) {
            float f = (in_rpm - s_rw_start_rpm) / (s_rw_full_rpm - s_rw_start_rpm);
            if (f > 1.0f) f = 1.0f;
            float floor_f = -s_rw_floor_mA;
            if (floor_f < cmd_f) floor_f = cmd_f;          /* never increase retract */
            cmd_f += (floor_f - cmd_f) * f;
        }
    }
```

Sign walk-through (30 kg example): `cmd_f = -8100`. At 250 rpm inbound, `f = 0.42`,
`floor_f = -2000`, result `-5540`. At ≥320 rpm: `-2000`. At 5 kg (`cmd_f = -1350`):
`floor_f` clamps to `-1350` → no change. Pulling out (`s_vel_m_s > 0`): untouched.

### 3. `resistance.c` — console tuning in `resistance_set_param()`

```c
    else if (!strcmp(name, "rw0"))  s_rw_start_rpm = v;          /* rpm */
    else if (!strcmp(name, "rw1"))  s_rw_full_rpm  = (v > s_rw_start_rpm + 1.0f ? v : s_rw_start_rpm + 1.0f);
    else if (!strcmp(name, "rwf"))  s_rw_floor_mA  = (v < 0.0f ? 0.0f : v);   /* mA */
```

And in `resistance_log_params()` append:
```c
    ESP_LOGW(RTAG, "RWD: rw0=%.0frpm rw1=%.0frpm rwf=%.0fmA",
             s_rw_start_rpm, s_rw_full_rpm, s_rw_floor_mA);
```

## Bench procedure (first flash)

1. Defaults on; set 30 kg. Pull out ~1 m and dump the handle. Expect: brisk-but-calm
   spool-in, no buzz, clean wall stop at home.
2. Dump, then catch mid-flight. Expect: weight blends in over ~a hand-width, no slam.
3. Slow controlled return (< 200 rpm ≈ 0.47 m/s cable). Expect: full weight the whole
   way — derate must be imperceptible in normal eccentric reps.
   If a strong user does fast eccentrics that clip 200 rpm, raise `p rw0 250`.
4. Tune: `rw0` higher = later onset; `rw1 - rw0` wider = softer blend; `rwf` = how
   much tension the free rewind keeps (don't go below ~1000 mA or the cable can
   outrun the drum and go slack → backlash snap at the wall).

## Future product feature (folds in naturally)

Separate **eccentric load %**: replace the fixed floor with
`s_rw_floor_mA = ecc_pct * full_mA` and expose `ecc_pct` in the app. The same fade
then *is* the eccentric-load feature — derating and eccentric % are one mechanism.
