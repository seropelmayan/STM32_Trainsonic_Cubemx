# Ropetow FOC — Overnight Research Run

**Started:** 2026-06-30 01:00 · **Target:** ≥4 h (~05:00) · **Mode:** sequential deep-research jobs, results appended to the topic files below.

Each job = fan-out web search → fetch top sources → 3-vote adversarial verification → cited synthesis. No hardware, no flashing. Read-only + file writes.

## Context (the problem being researched)
Sensored FOC PMSM (STM32G431, X-CUBE-MCSDK v6.4), 20 pole-pairs, ~700 rpm direct-drive surface-PM, 13S battery. Hits the **back-EMF voltage wall** at ~600 rpm (Vq → ~89%→100% modulation) → overmodulation buzz. ODrive does 700 rpm clean on the same hardware via **modulation-feedback flux weakening**. MCSDK's native FW is built for speed mode and overwrites the torque reference in torque mode. Goal: proven algorithms + working open-source code to fix FW/voltage-wall + anti-cogging, on MCSDK.

## Topic files
- [A — MCSDK voltage-wall / flux weakening](A-mcsdk-fieldweakening.md)
- [B — Anti-cogging: proven algorithms + working code](B-anticogging.md)
- [C — Open-source FOC libraries (with available code)](C-opensource-foc-libraries.md)
- [D — Sensored commutation quality at speed](D-sensored-commutation.md)

## What ran (strategy note)
The first heavyweight deep-research job (MCSDK FW internals) ran but its **synthesis step failed on the session usage cap** — its 23 verified claims were salvaged into topic A. Each such job costs ~3M tokens, so the 16-job plan was unaffordable. **Switched to 4 lightweight research agents** (~40–52K tokens each, ~183K total — 16× cheaper) that together cover all four topics. All complete.

## Status
| Topic | Source | Status |
|---|---|---|
| A — MCSDK FW / voltage wall | deep-research job 1 (salvaged) + lightweight FW/overmodulation/feedforward spec | ✅ done — incl. full implementable FW spec + C pseudocode |
| B — Anti-cogging proven code | lightweight agent | ✅ done — ODrive + moteus code; ILC/harmonic upgrades; ranked recs |
| C — Open-source FOC libraries | lightweight agent | ✅ done — comparison table + repo links + what to port |
| D — Sensored commutation at speed | lightweight agent | ✅ done — AS5047P latency/DAEC, angle extrapolation, EMI, reliability trips |

### Headline findings
- **A:** don't use MCSDK's native FW (breaks torque mode) — build a custom modulation-feedback FW PI → `Id*` via `MC_SetCurrentReferenceMotor1`, preserving `Iq*` (MESC pattern). Full spec + tuning + pseudocode in `A-…md`.
- **B:** only ODrive & moteus ship working anti-cogging code; you already have the ODrive-style LUT — upgrade with ILC + harmonic modeling + phase-advance the lookup.
- **C:** stay on MCSDK; highest-value port = ODrive anti-cogging; moteus (Apache-2.0, native G4) = best alt platform. (Survey's "FW not needed at 700 rpm" is **wrong for our high-Ke motor** — see note in `C-…md`.)
- **D:** the noise/`enc=BAD` at speed is largely **commutation-angle lag ×20** — fix with DAEC + host-latency feed-forward + reading the angle at PWM rate + SPI-as-truth.
