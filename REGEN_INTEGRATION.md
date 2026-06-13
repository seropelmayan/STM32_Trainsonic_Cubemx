# Regeneration Integration Runbook

**Audience: Claude Code (future me), when a freshly regenerated MC Workbench / CubeMX
tree lands from the freelancer.** This repo (`Ropetow`) is the canonical project. The
freelancer regenerates into a separate tree (e.g. `../Trainsonic`); generation
**drops all hand-written code and may reset config**. This runbook is how to fold a
new generation back in without losing our work or re-introducing known bugs.

> Read this top-to-bottom before editing anything. Then read the project memory
> `ropetow-current-conv-overflow.md` and `ropetow-pole-pair-mismatch.md` for the
> bench history behind these rules.

---

## Ground-truth hardware facts (do not "fix" these to match a placeholder)

| Quantity | Real value | Notes |
|---|---|---|
| Current sensing topology | **Three-shunt** (low-side FET→GND) | NOT ICS. Low-side shunts. |
| Shunt | `RSHUNT = 0.005` Ω | entered separately in the 3-shunt model |
| CSA gain | **20 V/V** | DRV8353 internal amp |
| Sensing scale | 0.1 V/A → **±16.5 A** full scale, ~1985 counts/A | |
| ADC mapping (R3_2) | U=ADC1/IN15 (PB0), W=ADC2/IN12 (PB2), **V=shared/IN2 (PA1)** | matches the bench rewire (V moved PB1→PA1) |
| Pole pairs | **20** | |
| Motor Kv | **14.6 rpm/V** → `MOTOR_VOLTAGE_CONSTANT = 48.4` V RMS ph-ph/kRPM | bench free-run ~430 rpm @ 30 V confirms 48.4, not 28.5 |
| LS | ~1.4 mH (freelancer-measured) | RS ~0.74 Ω still unverified |
| Current limits | derated to ~15–16 A (sensing-limited; motor itself rated 29 A) | |
| Encoder | AS5047P, 1024 PPR ABI via TIM3; `AS5047_INVERT_DIR = 1`; AGC~246 (weak-ish, monitor) | |

---

## The integration procedure

### 1. Diff the new tree against this one
```
diff -rq Ropetow/Inc <newtree>/Inc ; diff -rq Ropetow/Src <newtree>/Src
```
Expect: our hand files (`drv8353.*`, `as5047.*`, `log.*`) only in `Ropetow`; generated
MC files + `.ioc`/`.wb` differ. Read the diffs of `power_stage_parameters.h`,
`drive_parameters.h`, `pmsm_motor_parameters.h`, `parameters_conversion.h`, `main.c`.

### 2. CHECK THE CURRENT SCALING (highest-priority bug class)
The 3-shunt formula is `CURRENT_CONV_FACTOR = (uint16_t)(65536·RSHUNT·AMPLIFICATION_GAIN/Vref)`.
- `AMPLIFICATION_GAIN` must be the **CSA gain = 20** (because `RSHUNT=0.005` is a separate
  factor). A value like `0.02` truncates the factor to **1** → motor commands ~0 A and
  under-reads ~2000×. A value like the old composite `0.1` over-counts.
- Verify the factor folds to **~1985** (toolchain check):
  `arm-none-eabi-gcc -O2 -S` a snippet computing the macro; it must be 1985, not 1 or 65535.
- Fix in the **`.ioc` / `.ioc.wb` / `.wbdef`** (`M1_AMPLIFICATION_GAIN=20`), the source of truth.

### 3. PID gains — do NOT hand-edit; require a regen if scaling was wrong
The Workbench autotunes the PID gains from its internal counts/A. If the gain was wrong
when generated, the gains are tuned for the wrong scale; fixing only the macro makes the
loop ~1000× too hot/soft. **Never hand-scale the gains** (we banned that). Instead: set
`AMPLIFICATION_GAIN=20` in the `.ioc`/`.wb` and have the project **regenerated once** so
the Workbench recomputes the factor AND the gains consistently from the real LS. The
autotune from real motor constants is trustworthy — do **not** restore the old empirical
`590/14 ÷ 16384` hack (that compensated for broken scaling + placeholder constants).

### 4. Port the hand-written drivers
Copy into the tree: `drv8353.c/.h`, `log.c/.h`, **and `as5047.c/.h`**.
- **DRV8353**: pure GPIO bit-bang (PA15 CS, PB9 SCLK, PB11 MOSI, PB10 MISO). Works as long
  as those pins are GPIO outputs (the driver re-sets MISO to input itself). Build it.
- **LOG**: USB-CDC. Needs `usbd_cdc_if.*` present (it is, when USB CDC is enabled). Build it.
- **AS5047**: **KEEP THE FILES, BUT DO NOT BUILD OR INITIALIZE IT** unless SPI1 (hardware)
  is enabled in the generation. The recent regens disabled the SPI HAL module / dropped
  `MX_SPI1_Init`. The encoder still feeds FOC through the TIM3 ABI input; only the SPI
  *configuration* (PPR/DIR set, AGC/angle readback) is unavailable.
  - Do **not** add `as5047.c` to the build (`.project` linked resources).
  - Do **not** call `AS5047_Init(&hspi1)` in the boot code.
  - Leave the deferral note in main.c's USER CODE Includes block.

### 5. Add our sources to the build
The `.project` links files individually. Add `<link>` entries for **`drv8353.c` and
`log.c`** (NOT `as5047.c`) after the `main.c` entry, format:
```xml
<link><name>Application/User/drv8353.c</name><type>1</type>
<locationURI>PARENT-1-PROJECT_LOC/Src/drv8353.c</locationURI></link>
```
(Headers in `Inc/` are picked up via the include path; only `.c` files need links.)

### 6. Wire the boot code into USER-CODE regions (survives future regens)
Place all of it inside `/* USER CODE ... */` guards. **Where** depends on the tree:
- **FreeRTOS tree** (has `osKernelStart()`, `app_freertos.c`): the bare-metal `while(1)`
  is dead. Create a low-priority `appTask` thread (in `RTOS_THREADS` user code + a body in
  `USER CODE 4`). USB CDC comes up inside the medium-frequency task, so `osDelay(~1000)`
  at task start before `LOG_Init()`. Order: LOG_Init → DRV8353_Init (before any PWM) →
  boot diagnostics → auto-start → status-log loop with `osDelay(MOTOR_LOG_PERIOD_MS)`.
- **Bare-metal tree**: init in `USER CODE 2`, status loop in the `while(1)`.

Boot code contents (transplant from this repo's `Src/main.c` USER CODE):
- includes: `drv8353.h, log.h, mc_api.h, parameters_conversion.h, mc_config_common.h, mc_config.h`
- `BOOT_TARGET_SPEED_RPM`, `BOOT_SPEED_RAMP_MS`, `MOTOR_LOG_PERIOD_MS`
- helpers `mc_state_name()`, `log_mc_faults()`, `motor_status_log()` (the full
  `Iqref/Iq/Id/Vq/cnt/pwm` diagnostic line)
- `DRV8353_Init()` + `DRV8353_LogStatus()` boot dump
- auto-start: `MC_AcknowledgeFaultMotor1(); MC_ProgramSpeedRampMotor1(...); MC_StartMotor1();`
  (gated on `DRV8353_OK`)
- **`MX_USB_Device_Init()` MUST be called from the app task** (before `LOG_Init()`),
  with `#include "usb_device.h"`. CubeMX emits this call inside the *weak*
  `startMediumFrequencyTask` in main.c, but the MCSDK's *strong*
  `startMediumFrequencyTask` (mc_tasks.c) overrides it, so the generated USB init
  NEVER runs → no CDC device enumerates. This bit us on the FreeRTOS build; the
  bare-metal build didn't have it because USB was init'd in `main()`.
- **Bump `configTOTAL_HEAP_SIZE`** in `FreeRTOSConfig.h` (CubeMX default 3072 is too
  small once the ~512-word appTask is added → `osThreadCreate` silently returns NULL,
  appTask never runs, so NO USB + motor never started). Set to **8192**. Symptom looks
  like a boot crash but is really a silent task-creation failure. (Bare-metal avoids
  this entirely — no per-task heap.)

### 6b. RE-APPLY the current-sense polarity negation (vendor MCLib)
The board's DRV8353 low-side CSA reads the OPPOSITE sign to the MCSDK
`Ia = offset - ADC` convention → the current loop runs on POSITIVE feedback
(rails Vq during alignment, draws huge current, trips UNDER_VOLT via supply sag).
Fix lives in `MCSDK_v6.4.2-Full/.../MCLib/G4xx/Src/r3_2_g4xx_pwm_curr_fdbk.c`: in
`R3_2_GetPhaseCurrents` (and the `_OVM` variant), just before
`pHandle->_Super.Ia = Iab->a;`, negate `Iab->a`/`Iab->b`. A regen may overwrite
MCLib — re-apply. (Diagnosed by a gain-detune test: ÷8 gains made the overdraw
WORSE, proving it's a sign/polarity issue, not tuning.) Long-term: confirm shunt
SP/SN orientation and fix polarity in hardware/Workbench instead of this patch.
Also note: the c4fda24-era empirical current-PI gains and the `/8` diagnostic
detune were both compensating for THIS — once polarity is correct, the Workbench
autotune gains (2365/512, 2501/16384) should be restored.

### 7. Verify / confirm the freelancer's config values (flag, don't silently keep)
- `VBUS_PARTITIONING_FACTOR`: confirm against real divider (91k/3.9k → 0.0411). A regen had 0.0526.
- `OV_VOLTAGE_THRESHOLD_V`: must be **above** charged pack (54.6 V for 13S). A regen had 50 (would trip).
- `UD_VOLTAGE_THRESHOLD_V`: a regen had 8 (very low). Bench-safe ~20.
- `FINAL_I_ALIGNMENT_A`: keep **bench-safe (~1–2 A)** while on a current-limited supply; a
  regen had 15 (trips the bench supply once scaling is correct).
- `MOTOR_VOLTAGE_CONSTANT`: should be **48.4** (Kv 14.6), not 28.5 or 0.1.

### 8. Bring it into the main project tree (`Ropetow`)
Once the new tree builds and a bench run looks sane:
- Copy the regenerated generated files + `.ioc/.ioc.wb/.wbdef` + our integrated `main.c`
  (and `app_freertos.c` if RTOS) + drivers over the `Ropetow` repo, preserving git history.
- `git status` to review; commit on a branch, describe the regen + integration.
- If the project was renamed (e.g. `Trainsonic`), decide with the user whether to keep the
  `Ropetow` repo name (rename `.ioc` back) or adopt the new name. Don't rename without asking.

---

## AS5047 re-enable checklist (do this only when SPI1 is back in the generation)
1. Confirm `HAL_SPI_MODULE_ENABLED` is defined in `stm32g4xx_hal_conf.h` and `MX_SPI1_Init`
   + `hspi1` exist (SPI1 on PA5/PA6/PA7, AF5).
2. Add `as5047.c` to the `.project` build.
3. In boot code: `AS5047_Status enc = AS5047_Init(&hspi1);` + `AS5047_LogStatus(&hspi1);`,
   and gate the motor start on `enc == AS5047_OK` as well.
4. Confirm `AS5047_INVERT_DIR = 1` (in `as5047.h`) unless bench says otherwise.

---

## Quick pre-flash gate (every regen)
- [ ] `CURRENT_CONV_FACTOR` folds to ~1985 (gain=20, RSHUNT=0.005)
- [ ] PID gains came from a regen with gain=20 (not hand-scaled, not the old hack)
- [ ] `drv8353.c` + `log.c` built; `as5047.c` NOT built; `AS5047_Init` NOT called
- [ ] boot/auto-start/status code present in USER-CODE regions
- [ ] alignment current bench-safe; OV/UV/Vbus-divider/Ke confirmed
- [ ] motor start has no field interlock yet — bench only
