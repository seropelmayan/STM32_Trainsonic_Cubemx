#!/usr/bin/env python3
"""
Iq-ripple analyzer for the Ropetow FOC drive (STEP_MODE 2 capture).

The board runs at a constant speed and streams a steady-state Iq capture:
    step_start n=<N> hz=<rate> axis=q
    idx,Iqref,Iq          (s16 current units, one line per sample)
    ...
    step_end

This script FFTs the measured Iq, reports the ripple magnitude (pk-pk, RMS) and
the dominant frequencies, and labels each peak by its relation to the mechanical
once-per-rev frequency and the electrical frequency (x POLE_PAIRS).

Usage:
    python ripple.py <dump.txt> [rpm]     (rpm default 150; pole pairs = 20)
    (no file -> logs.md)
"""
import sys, re, math

POLE_PAIRS = 20

def load(path):
    ref, iq, rate, started = [], [], 25000, False
    for ln in open(path, encoding="utf-8", errors="ignore").read().splitlines():
        s = ln.strip()
        if s.startswith("step_start"):
            started = True; ref, iq = [], []
            mh = re.search(r'\bhz=(\d+)', s); mk = re.search(r'khz=(\d+)', s)
            rate = int(mh.group(1)) if mh else (int(mk.group(1))*1000 if mk else 25000)
            continue
        if s.startswith("step_end"):
            break
        if not started:
            continue
        m = re.fullmatch(r'(\d+),(-?\d+),(-?\d+)', s)
        if m:
            ref.append(int(m.group(2))); iq.append(int(m.group(3)))
    return ref, iq, rate

def dft_mag(x, fs):
    """Naive DFT magnitude spectrum of real signal x (linear-detrended, Hann-windowed).
    Detrend removes DC *and* slow drift (e.g. a load ramp during the capture) so it
    doesn't masquerade as low-frequency ripple."""
    n = len(x)
    # least-squares line a*i+b, then subtract
    sx = (n-1)*n/2.0; sxx = sum(i*i for i in range(n)); sy = sum(x); sxy = sum(i*x[i] for i in range(n))
    a = (n*sxy - sx*sy)/(n*sxx - sx*sx); b = (sy - a*sx)/n
    xd = [x[i] - (a*i + b) for i in range(n)]
    w = [0.5 - 0.5*math.cos(2*math.pi*i/(n-1)) for i in range(n)]  # Hann
    xw = [xd[i]*w[i] for i in range(n)]
    wsum = sum(w)
    half = n//2
    freqs, mags = [], []
    for k in range(1, half):          # skip DC
        re_ = im = 0.0
        ang = 2*math.pi*k/n
        for i in range(n):
            re_ += xw[i]*math.cos(ang*i)
            im  -= xw[i]*math.sin(ang*i)
        # amplitude scaling: *2/wsum gives the peak amplitude of that sinusoid
        mags.append(2.0*math.hypot(re_, im)/wsum)
        freqs.append(k*fs/n)
    return freqs, mags

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "logs.md"
    rpm = float(sys.argv[2]) if len(sys.argv) > 2 else 150.0
    ref, iq, fs = load(path)
    n = len(iq)
    if n < 32:
        print("Not enough samples (%d)." % n); return
    f_mech = rpm/60.0
    f_elec = f_mech*POLE_PAIRS
    mean = sum(iq)/n
    ac = [v-mean for v in iq]
    pp = max(iq)-min(iq)
    rms = math.sqrt(sum(v*v for v in ac)/n)
    print("=== Iq ripple @ %g rpm  (f_mech=%.2f Hz, f_elec=%.1f Hz, %d pole-pairs) ==="
          % (rpm, f_mech, f_elec, POLE_PAIRS))
    print("samples=%d  fs=%g Hz  window=%.0f ms  (Nyquist=%g Hz, bin=%.1f Hz)"
          % (n, fs, 1000.0*n/fs, fs/2, fs/n))
    print("Iq: mean=%.0f s16  pk-pk=%d s16  RMS(AC)=%.1f s16  (ripple = %.1f%% of mean)"
          % (mean, pp, rms, 100.0*rms/mean if mean else 0))
    freqs, mags = dft_mag(iq, fs)
    order = sorted(range(len(mags)), key=lambda i: mags[i], reverse=True)
    print("\ndominant ripple components (top 6):")
    print("   freq(Hz)  amp(s16)   nearest harmonic")
    seen = 0
    for i in order:
        if seen >= 6:
            break
        f, a = freqs[i], mags[i]
        # label vs mech / elec
        lbl = []
        if f_mech > 0:
            hm = f/f_mech
            if abs(hm - round(hm)) < 0.15 and 1 <= round(hm) <= 12:
                lbl.append("%dx mech" % round(hm))
        if f_elec > 0:
            he = f/f_elec
            if abs(he - round(he)) < 0.10 and round(he) >= 1:
                lbl.append("%dx elec" % round(he))
        print("   %7.1f   %7.1f    %s" % (f, a, ", ".join(lbl) if lbl else "-"))
        seen += 1
    # explicit amplitude at each electrical harmonic (always shown, even if not a
    # top peak) so you can track e.g. the 6x line as you sweep dead-time comp.
    def amp_at(ftarget):
        if ftarget <= 0 or ftarget >= fs/2:
            return None
        # nearest bin, take the max over +/-2 bins to capture spectral leakage
        j0 = min(range(len(freqs)), key=lambda i: abs(freqs[i]-ftarget))
        lo = max(0, j0-2); hi = min(len(mags), j0+3)
        return max(mags[lo:hi])
    print("\namplitude at electrical harmonics (peak within +/-2 bins):")
    for h in range(1, 9):
        a = amp_at(h*f_elec)
        if a is None:
            break
        tag = "  <-- dead-time / cogging" if h == 6 else ""
        print("  %dx elec (%5.0f Hz): %5.1f s16%s" % (h, h*f_elec, a, tag))
    print("\nhint: 6x elec (=%.0f Hz) is the dead-time/cogging line -- track it as you"
          % (6*f_elec))
    print("sweep D<n>. A 1x-mech peak (needs a long-window capture) = mechanical source.")

if __name__ == "__main__":
    main()
