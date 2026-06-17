#!/usr/bin/env python3
"""
Torque (Iq) loop step-response analyzer for the Ropetow FOC drive.

Capture flow on the target:
  - In the USB serial terminal, set gains live:  P<n> = torque Kp, I<n> = torque Ki
  - Press 'g' to inject an Iq step and capture Iqref vs Iq at the loop rate (25 kHz).
  - The board streams:   step_start n=<N> khz=<rate>
                         idx,Iqref,Iq        (one line per sample, s16 counts)
                         ...
                         step_end

Usage:
  - Save the terminal text (the whole step_start..step_end block) to a file, e.g. step.txt
  - python step_tune.py step.txt
  (no arg -> reads logs.md in the same folder)

Reports rise time, overshoot, settling time, steady-state error, and a tuning hint.
Plots Iqref vs Iq if matplotlib is available (optional).
"""
import sys, re

def load(path):
    ref, iq = [], []
    rate_hz = None
    axis = '?'
    started = False
    for ln in open(path, encoding="utf-8", errors="ignore").read().splitlines():
        s = ln.strip()
        m0 = re.search(r'step_start\s+n=(\d+)', s)
        if m0:
            started = True; ref, iq = [], []
            mh = re.search(r'\bhz=(\d+)', s)       # new: explicit sample rate in Hz
            mk = re.search(r'khz=(\d+)', s)         # old dumps: kHz
            rate_hz = int(mh.group(1)) if mh else (int(mk.group(1)) * 1000 if mk else 25000)
            ma = re.search(r'axis=([qds])', s)
            axis = ma.group(1) if ma else '?'
            continue
        if s.startswith('step_end'):
            break
        if not started:
            continue
        m = re.fullmatch(r'(\d+),(-?\d+),(-?\d+)', s)
        if m:
            ref.append(int(m.group(2))); iq.append(int(m.group(3)))
    return ref, iq, (rate_hz or 25000), axis

def analyze(ref, iq, rate_hz, axis='?'):
    n = len(iq)
    if n < 20:
        print("Not enough samples (%d). Capture the full step_start..step_end block." % n)
        return
    dt_us = 1.0e6 / rate_hz   # microseconds per sample
    # find the step in the reference
    r0 = ref[0]
    rf = ref[-1]
    # step index = first sample where ref departs from baseline by >half the step
    half = (rf - r0) / 2.0
    k_step = next((i for i in range(n) if abs(ref[i] - r0) > abs(half)), None)
    if k_step is None or rf == r0:
        print("No step detected in Iqref (r0=%d rf=%d). Did the step inject?" % (r0, rf))
        return
    step = rf - r0
    # baseline Iq (avg before step) and final Iq (avg over last 20% of window)
    base = sum(iq[:k_step]) / max(1, k_step)
    tail = iq[int(n*0.8):]
    final = sum(tail) / len(tail)
    resp = final - base
    seg = iq[k_step:]
    # 10-90% rise time
    lo = base + 0.1 * resp
    hi = base + 0.9 * resp
    def cross(level, rising):
        for j in range(1, len(seg)):
            a, b = seg[j-1], seg[j]
            if (rising and a < level <= b) or (not rising and a > level >= b):
                return j
        return None
    rising = resp >= 0
    j10 = cross(lo, rising); j90 = cross(hi, rising)
    rise = (j90 - j10) * dt_us if (j10 is not None and j90 is not None) else float('nan')
    # overshoot
    peak = max(seg) if rising else min(seg)
    over = 100.0 * (peak - final) / resp if resp != 0 else 0.0
    if not rising:
        over = 100.0 * (final - peak) / resp if resp != 0 else 0.0
    over = abs(over)
    # settling: last time it leaves a +/-5% band around final
    band = 0.05 * abs(resp)
    j_settle = 0
    for j in range(len(seg)):
        if abs(seg[j] - final) > band:
            j_settle = j
    settle = j_settle * dt_us
    sse = 100.0 * (final - rf_in_iq(ref, iq, k_step)) / resp if resp else 0.0

    speed = (axis == 's')
    unit = "rpm" if speed else "s16"
    # times: report ms for the slow speed loop, us for the fast current loop
    if dt_us >= 100.0:
        tdiv, tu = 1000.0, "ms"
    else:
        tdiv, tu = 1.0, "us"
    name = "Speed-loop" if speed else "Current-loop"
    ratestr = ("%d Hz" % rate_hz) if rate_hz < 1000 else ("%g kHz" % (rate_hz/1000.0))
    print("=== %s step response (%s) ===" % (name, "speed" if speed else axis + "-axis"))
    print("samples=%d  rate=%s  window=%.1f ms  step in ref=%d -> %d %s"
          % (n, ratestr, n*dt_us/1000.0, r0, rf, unit))
    print("meas: baseline=%.0f  final=%.0f  response=%.0f %s" % (base, final, resp, unit))
    print("  rise time (10-90%%): %.1f %s" % (rise/tdiv, tu))
    print("  overshoot         : %.1f %%" % over)
    print("  settling (+/-5%%)   : %.1f %s" % (settle/tdiv, tu))
    print("  steady-state vs ref: %.1f %% of step (0%% = tracks ref)"
          % (100.0*(final - rf)/resp if resp else 0.0))
    print()
    hint(rise, over, settle, speed)
    return base, final, k_step, dt_us, axis

def rf_in_iq(ref, iq, k):  # final reference level (target)
    return ref[-1]

def hint(rise, over, settle, speed=False):
    if speed:
        print("--- speed-loop tuning hint (live keys: p<n>=Kp, i<n>=Ki) ---")
        if over > 30:
            print("  Overshoot high (>30%%): lower speed Kp (p), or raise Ki less.")
        elif over < 5 and rise > 80000:   # >80 ms rise
            print("  Sluggish (slow rise, no overshoot): raise speed Kp (p).")
        elif over <= 20:
            print("  Looks well-damped. If steady speed has ripple, that's the once-per-rev")
            print("  roughness -- look at the trace's noise band, not the step shape.")
        else:
            print("  Aim for overshoot ~10-15%% and no sustained oscillation. Nudge Kp/Ki.")
        print("  Sustained oscillation around the target -> Kp too high (limit cycle).")
        print("  Slow creep to target after fast rise -> raise Ki.")
        return
    print("--- current-loop tuning hint (keep Ki/Kp ~ R/L = 528 rad/s) ---")
    if over > 25:
        print("  Overshoot high (>25%%): lower torque Kp (P).")
    elif rise > 800:
        print("  Sluggish (slow rise): raise torque Kp (P).")
    elif over <= 20 and rise <= 800:
        print("  WELL TUNED: fast rise (<800 us) + low overshoot (<20%%), no need to change.")
    else:
        print("  Aim for rise <~500 us and overshoot ~10-15%%. Nudge Kp, keep Ki/Kp ratio.")
    print("  Slow creep after fast rise -> raise Ki (I). Sustained ringing -> lower Kp.")

def plot(ref, iq, rate_hz, axis='?'):
    try:
        import matplotlib.pyplot as plt
    except Exception:
        print("(matplotlib not installed -> skipping plot; metrics above are the key output)")
        return
    t = [i * 1000.0 / rate_hz for i in range(len(iq))]  # ms
    plt.figure(figsize=(9,5))
    plt.plot(t, ref, label="I%sref (s16)" % axis, lw=1.5)
    plt.plot(t, iq,  label="I%s meas (s16)" % axis, lw=1.0)
    plt.xlabel("time (ms)"); plt.ylabel("%s-axis current (s16 counts)" % axis)
    plt.title("Ropetow %s step response @ %g Hz"
              % (("speed-loop" if axis == 's' else axis + "-axis current"), rate_hz))
    plt.legend(); plt.grid(True, alpha=0.3); plt.tight_layout(); plt.show()

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "logs.md"
    ref, iq, rate_hz, axis = load(path)
    analyze(ref, iq, rate_hz, axis)
    plot(ref, iq, rate_hz, axis)
