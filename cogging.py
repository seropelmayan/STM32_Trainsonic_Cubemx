#!/usr/bin/env python3
"""
Anti-cogging map analyzer + table generator for the Ropetow FOC drive.

The board captures, while rotating slowly under speed control, the per-mechanical-
position mean of the measured Iq the current loop fights ('y' to arm, 'Y' to dump):

    cogg_start nbins=<N>
    idx,mean,count        (one line per bin: bin index, mean Iq [s16], sample count)
    ...
    cogg_end

This script:
  * parses one or two captures (run it once per direction -- forward and reverse),
  * fills empty bins by circular interpolation,
  * removes the DC component (= real load/accel torque, NOT cogging),
  * combines fwd+rev to keep the direction-EVEN part (cogging) and drop the
    direction-ODD part (friction/stiction),
  * FFTs the result and reports the dominant MECHANICAL orders (cycles/rev),
  * writes Inc/cogg_table.h with the COGG_TABLE_INIT initializer (feed-forward Iq,
    s16 units), ready to compile + flash.

Usage:
    python cogging.py <forward.txt> [reverse.txt] [--out Inc/cogg_table.h] [--invert]

    --invert  flip the feed-forward sign (use if enabling comp made ripple WORSE).
    Pass just one file to characterize without separating friction (DC still removed).
"""
import sys, re, math, os

POLE_PAIRS = 20


def load(path):
    """Return (nbins, means[list], counts[list]) from a cogg_start..cogg_end block."""
    nbins, started = None, False
    rows = {}
    for ln in open(path, encoding="utf-8", errors="ignore").read().splitlines():
        s = ln.strip()
        if s.startswith("cogg_start"):
            started = True
            rows = {}
            m = re.search(r'\bnbins=(\d+)', s)
            nbins = int(m.group(1)) if m else None
            continue
        if s.startswith("cogg_end"):
            break
        if not started:
            continue
        m = re.fullmatch(r'(\d+),(-?\d+),(\d+)', s)
        if m:
            rows[int(m.group(1))] = (int(m.group(2)), int(m.group(3)))
    if nbins is None:
        # infer from highest index seen
        nbins = (max(rows) + 1) if rows else 0
    means = [0] * nbins
    counts = [0] * nbins
    for i, (mn, cn) in rows.items():
        if 0 <= i < nbins:
            means[i], counts[i] = mn, cn
    return nbins, means, counts


def fill_empty(means, counts):
    """Circular linear-interpolate bins with zero samples (never visited)."""
    n = len(means)
    valid = [i for i in range(n) if counts[i] > 0]
    if not valid:
        return [0.0] * n
    if len(valid) == n:
        return [float(m) for m in means]
    out = [None] * n
    for i in valid:
        out[i] = float(means[i])
    # walk the ring, fill gaps between consecutive valid bins
    for k in range(len(valid)):
        a = valid[k]
        b = valid[(k + 1) % len(valid)]
        va, vb = float(means[a]), float(means[b])
        gap = (b - a) % n
        for step in range(1, gap):
            idx = (a + step) % n
            out[idx] = va + (vb - va) * (step / gap)
    return [x if x is not None else 0.0 for x in out]


def remove_dc(x):
    avg = sum(x) / len(x)
    return [v - avg for v in x], avg


def dft_orders(x):
    """Magnitude of each mechanical order (cycles/rev) up to N/2. x is DC-removed."""
    n = len(x)
    half = n // 2
    out = []
    for k in range(1, half):
        re_ = im = 0.0
        ang = 2 * math.pi * k / n
        for i in range(n):
            re_ += x[i] * math.cos(ang * i)
            im -= x[i] * math.sin(ang * i)
        out.append((k, 2.0 * math.hypot(re_, im) / n))   # k = mechanical order
    return out


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    opts = [a for a in sys.argv[1:] if a.startswith("--")]
    invert = "--invert" in opts
    out_path = "Inc/cogg_table.h"
    for o in opts:
        if o.startswith("--out="):
            out_path = o.split("=", 1)[1]
    if not args:
        print(__doc__)
        return

    caps = []
    nbins = None
    for p in args[:2]:
        nb, means, counts = load(p)
        if nb == 0:
            print("No cogg_start..cogg_end block found in %s" % p)
            return
        nbins = nb if nbins is None else nbins
        if nb != nbins:
            print("nbins mismatch: %s has %d, expected %d" % (p, nb, nbins))
            return
        filled = fill_empty(means, counts)
        ac, dc = remove_dc(filled)
        visited = sum(1 for c in counts if c > 0)
        print("%-24s nbins=%d  visited=%d/%d  DC(mean Iq)=%.0f s16  pk-pk(AC)=%.0f"
              % (os.path.basename(p), nb, visited, nb, dc, max(ac) - min(ac)))
        caps.append(ac)

    if len(caps) == 2:
        # direction-even = cogging; direction-odd = friction/stiction
        cogg = [(caps[0][i] + caps[1][i]) / 2.0 for i in range(nbins)]
        fric = [(caps[0][i] - caps[1][i]) / 2.0 for i in range(nbins)]
        print("combined fwd+rev: cogging pk-pk=%.0f s16   friction(|odd|) pk-pk=%.0f s16"
              % (max(cogg) - min(cogg), max(fric) - min(fric)))
    else:
        cogg = caps[0]
        print("single capture: cannot separate friction; using DC-removed map directly")

    # report dominant mechanical orders
    orders = sorted(dft_orders(cogg), key=lambda t: t[1], reverse=True)
    print("\ndominant cogging orders (cycles per mechanical rev):")
    print("   order   amp(s16)   note")
    for k, a in orders[:8]:
        note = ""
        if k % POLE_PAIRS == 0:
            note = "= %dx electrical" % (k // POLE_PAIRS)
        print("   %5d   %7.1f    %s" % (k, a, note))

    table = [(-v if invert else v) for v in cogg]
    table = [max(-32768, min(32767, int(round(v)))) for v in table]

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("/* Auto-generated by cogging.py -- do not hand-edit.\n")
        f.write("   Anti-cogging feed-forward Iq (s16 units), indexed by mechanical\n")
        f.write("   angle >> COGG_SHIFT. Sign%s inverted. */\n" % ("" if invert else " NOT"))
        f.write("#ifndef COGG_TABLE_H\n#define COGG_TABLE_H\n\n#include <stdint.h>\n\n")
        f.write("#define COGG_NBINS  %d\n" % nbins)
        shift = int(round(math.log2(16384 / nbins)))
        f.write("#define COGG_SHIFT  %d\n\n" % shift)
        f.write("#ifdef COGG_DEFINE_TABLE\n")
        f.write("const int16_t COGG_TABLE_INIT[COGG_NBINS] = {\n")
        for i in range(0, nbins, 12):
            f.write("  " + ",".join("%6d" % v for v in table[i:i + 12]) + ",\n")
        f.write("};\n#else\nextern const int16_t COGG_TABLE_INIT[COGG_NBINS];\n#endif\n\n")
        f.write("#endif /* COGG_TABLE_H */\n")
    print("\nwrote %s (%d entries). Rebuild + flash, then enable with 'K' (disable 'k')."
          % (out_path, nbins))
    print("If enabling makes ripple WORSE, re-run with --invert.")


if __name__ == "__main__":
    main()
