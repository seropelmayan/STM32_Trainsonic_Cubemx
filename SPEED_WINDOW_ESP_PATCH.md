# Speed-window control: what the ESP32 sends (protocol v4)

STM32 branch `speed-window` replaces the hand-written torque-mode governor with the
MCSDK **speed loop plus per-heartbeat torque limits** ("speed control with torque
limit"). The ESP32 keeps owning all training logic; only the *meaning* of the heartbeat
changes, and one field is added. A v3 ESP (8-byte heartbeat) still works unchanged --
the STM32 fills the missing field from its console default.

## Wire format (TSL_MSG_HEARTBEAT, id 0x01, stream frame, seq byte 0)

```
[0x01][0x00][len=10][ seq u32 LE ][ torque_mA i16 ][ speed_rpm i16 ][ brake_mA i16 ][crc16 LE]
```
SLIP-framed exactly as today; CRC16 over id..payload as today. `len` is 10 (was 8).

| Field | Meaning in speed-window control | Legacy (`#` = torque governor) |
|---|---|---|
| `torque_mA` (signed) | **Drive-side torque limit.** Sign = direction of the speed reference: `< 0` inward (retract = the weight), `> 0` outward (push the cable out, home wall). `0` = no drive torque (slack zone). | plain torque setpoint, as before |
| `speed_rpm` (magnitude) | **Speed reference.** The drive holds `|torque_mA|` until the drum reaches this speed *in the reference direction*, then eases the torque to 0 and holds the speed. `0` or above the STM32 hard ceiling (550) = the hard ceiling. | governor cap, as before |
| `brake_mA` (>= 0, new) | **Brake-side torque limit.** Most torque the drive may apply *against* motion faster than the reference (a released handle overshooting the cap). `0` = never brake, only ease off. Absent (v3 frame) = STM32 console default `%<mA>`, 2000. | ignored |

`TSL_MSG_SETPOINT` (0x11) keeps its 4-byte payload and is treated like a heartbeat with
`brake_mA` absent.

New status flag: `TSL_SFLAG_AT_LIMIT` (0x10) in `tsl_status_t.flags` = the speed PI is
sitting on a torque limit (drive side or brake side). Set while the cable is held,
pulled, or lowered below the cap; clear only while the loop is actually regulating
speed. During homing it doubles as a "leaning on the stop" signal.

## What each ESP mode sends

| Mode | `torque_mA` | `speed_rpm` | `brake_mA` |
|---|---|---|---|
| Homing (seek) | `-810` (3 kg stall force) | `150` | `0` |
| Resistance, cable out | `-W` from the weight law (ramp 0->W over the first 20 mm, as now) | `800` (-> clamped to 550) | `2000` (soft catch) |
| Resistance, at home (slack) | `0` | `800` | `2000` |
| Home wall, past home | `+K * overshoot_mm` (outward), capped at `wall_max` (1500) | `20`..`40` | `0` |
| Calibration keep-alive | `0` | `0` | `0` |
| Link idle / disabled | `0` | `800` | `0` |

Nothing changes for a rep: below the cap the speed PI is saturated on `torque_mA`, so
the pull, the hold and a hand-controlled eccentric feel exactly the commanded weight
at any speed. Only a *released* cable running faster than `speed_rpm` inward sees the
torque ease off and (if `brake_mA` > 0) a brake.

## Minimal ESP diff (trainsonic-firmware)

1. **`components/ts_link/trainsonic_link.h`** -- replace with the STM32 repo's
   `protocol/trainsonic_link.h` (v4, verbatim; it is the shared contract).

2. **`components/ts_link/ts_link.c` / `.h`** -- add the field to the heartbeat sender:
   ```c
   void ts_link_send_heartbeat(int16_t torque_mA, int16_t speed_rpm, int16_t brake_mA)
   { tsl_heartbeat_t hb = { .seq = ++s_hb_seq, .torque_mA = torque_mA,
                            .speed_rpm = speed_rpm, .brake_mA = brake_mA };
     ts_send(TSL_MSG_HEARTBEAT, &hb, sizeof(hb)); }
   ```

3. **`components/motor_control/include/motor_control.h`** -- `mc_cmd_t` gains
   `int brake_mA;`.

4. **`components/motor_control/motor_control.c`** -- fill it per mode in `mc_tick()`:
   `0` for HOMING and IDLE, `2000` for RESISTANCE (make it a `p` tunable, e.g. `brk`).
   The `speed_limit_rpm` values already sent (150 homing / 800 otherwise) are the
   speed references and need no change.

5. **`main/main.cpp`** -- the two `ts_link_send_heartbeat(...)` calls pass
   `cmd.brake_mA` (and `0` in the calibration keep-alive).

6. **`components/motor_control/resistance.c`** (optional, recommended) -- replace the
   `kp`/`kd` home wall with the cascade: when `dir_m < -deadband`, send a *positive*
   drive of `min(wall_max, K_pos * -dir_m)` with `speed_rpm` = 20..40 and
   `brake_mA` = 0. The damping now lives in the STM32 speed PI at 1 kHz, so `kd`
   (the July buzz source) goes away.

7. **Re-home after an STM32 reset** (unrelated bug, bit us on 2026-09-08): in
   `ts_link_task`, if the link has been down > 1 s and comes back, or the reported
   `position` jumps by more than a stroke between ticks, call `mc_start_homing()`.

## STM32 console knobs for the bench (USB CDC)

| Cmd | Effect |
|---|---|
| `#` | toggle speed-window (default) / legacy torque governor; the `[m]` line shows `sch=S` or `sch=T` |
| `t<mA>` | drive-side limit (signed), reference = `V` cap, brake = `%` default -- same as a heartbeat |
| `V<rpm>` | speed reference magnitude (hard-clamped to 550) |
| `$<rpm>` | fade band: speed error over which the torque swings from full drive to full brake (default 120; 0 = manual `p`) |
| `%<mA>` | brake-side limit used when the heartbeat sends none (default 2000) |
| `p<n>` / `i<n>` | speed PI gains; typing `p` switches the band off (manual Kp) |
| `v` / `b` | speed PI fed from the 1 kHz SPI speed / the 16-sample TIM3 average (default) |

`[m]` line, speed-window fields: `ref=` signed reference rpm, `lim=lo/hi` PI output
limits (s16 counts, 993 per amp), `kp=` gain in use, `sat=` `L` on the drive limit,
`H` on the brake limit, `-` regulating.

## Not in this change (next commits)

- Free-fall detection so a hand-controlled fast eccentric never meets the cap.
- Position-based approach cap in the last centimetres before home (ESP side, from
  the cascade formula above).
- Modulation-index fade for heavy weights on 15S (the voltage wall).
