#!/usr/bin/env python3
"""
Anti-cogging map extractor for the Trainsonic FOC drive.

Reads the calibrated map back OUT of the board and, optionally, regenerates
`Inc/cogg_table.h` so the calibration is compiled into the image and survives
any reflash (including a full chip erase).

Two input sources
-----------------
1. FLASH IMAGE (preferred -- no firmware change, works on a running board).
   Either SWD tool works; this script only cares about the bytes.

       # ST's own tool (ships with STM32CubeProgrammer, and inside CubeIDE under
       #   plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.*/tools/bin/)
       STM32_Programmer_CLI -c port=SWD mode=HOTPLUG -u 0x0801F800 1048 cogg.bin

       # or open-source stlink (Arch: pacman -S stlink; Debian: apt install stlink-tools)
       st-flash read cogg.bin 0x0801F800 1048

       ./cogg_read.py cogg.bin

   0x0801F800 is page 63 (the last 2 KB page of the 128 KB G431CB);
   1048 = 24-byte header + 512 bins x int16.

   NOTE: st-flash writes exactly the requested length, while some
   STM32_Programmer_CLI versions round the upload up to a 4-byte boundary. Both
   are fine -- this script reads the header for the real length and ignores any
   trailing bytes.

2. CONSOLE DUMP -- the text the board prints on 'Y' (finish/abort calibration).
   That path prints the LIVE g_cogg_lut, so it also works for a map that was
   loaded from flash at boot:

       cogg_start nbins=512 harm=1
       0,-12,240
       ...
       cogg_end

       ./cogg_read.py capture.txt

Emitting a compiled-in table
----------------------------
       ./cogg_read.py cogg.bin --emit-header Inc/cogg_table.h

   NOTE ON PRECEDENCE: Ropetow_CoggInit() loads COGG_TABLE_INIT first and then
   lets a valid FLASH map OVERRIDE it. So after compiling a map in, erase the
   saved one (console 'n') or the stale flash copy keeps winning.
"""

import argparse
import struct
import sys
import zlib

# Must track mc_tasks_foc.c / cogg_table.h.
MAGIC = 0x43473231          # "CG21"
VERSION = 2
NBINS = 512
HDR_FMT = "<IHHIhBBfI"      # magic, version, nbins, crc, clamp, harm, freeze, gain, pad
HDR_SIZE = struct.calcsize(HDR_FMT)   # 24


def parse_flash(blob):
    """Validate exactly the way Ropetow_CoggLoadFromFlash() does."""
    if len(blob) < HDR_SIZE:
        sys.exit(f"file too short: {len(blob)} bytes, need >= {HDR_SIZE}")
    magic, ver, nbins, crc, clamp, harm, freeze, gain, _pad = struct.unpack(
        HDR_FMT, blob[:HDR_SIZE])

    if magic != MAGIC:
        sys.exit(f"bad magic 0x{magic:08X} (expected 0x{MAGIC:08X} 'CG21') -- "
                 "no saved map at this address, or the page is erased")
    if ver != VERSION:
        sys.exit(f"version {ver}, this tool understands {VERSION}")
    if nbins != NBINS:
        sys.exit(f"nbins {nbins}, firmware is built for {NBINS}")

    need = HDR_SIZE + nbins * 2
    if len(blob) < need:
        sys.exit(f"file too short: {len(blob)} bytes, need {need} for {nbins} bins")

    payload = blob[HDR_SIZE:need]
    calc = zlib.crc32(payload) & 0xFFFFFFFF   # poly 0xEDB88320, init/final ~0
    if calc != crc:
        sys.exit(f"CRC mismatch: stored 0x{crc:08X}, computed 0x{calc:08X} -- "
                 "the firmware would REJECT this map and fall back to the "
                 "compiled-in table")

    lut = list(struct.unpack(f"<{nbins}h", payload))
    knobs = {"clamp": clamp, "gain": gain, "harm_en": harm, "freeze_en": freeze}
    return lut, knobs, True


def parse_console(text):
    """Parse a 'cogg_start ... cogg_end' console capture (idx,mean,count)."""
    lut, counts, seen = [0] * NBINS, [0] * NBINS, 0
    inside = False
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("cogg_start"):
            inside = True
            continue
        if line.startswith("cogg_end"):
            break
        if not inside or "," not in line:
            continue
        parts = line.split(",")
        if len(parts) < 2:
            continue
        try:
            idx, mean = int(parts[0]), int(parts[1])
            cnt = int(parts[2]) if len(parts) > 2 else 0
        except ValueError:
            continue
        if 0 <= idx < NBINS:
            lut[idx], counts[idx] = mean, cnt
            seen += 1
    if seen == 0:
        sys.exit("no 'cogg_start ... cogg_end' block found in that text")
    if seen < NBINS:
        print(f"  WARNING: only {seen}/{NBINS} bins present -- truncated capture",
              file=sys.stderr)
    empty = sum(1 for c in counts if c == 0)
    if empty:
        print(f"  WARNING: {empty} bins had 0 samples (sweep did not cover them)",
              file=sys.stderr)
    return lut, None, False


def report(lut, knobs, from_flash):
    peak = max(abs(v) for v in lut)
    pkbin = max(range(len(lut)), key=lambda i: abs(lut[i]))
    rms = (sum(v * v for v in lut) / len(lut)) ** 0.5
    mean = sum(lut) / len(lut)
    nz = sum(1 for v in lut if v != 0)

    print(f"source          : {'flash image (CRC OK)' if from_flash else 'console dump'}")
    print(f"bins            : {len(lut)}  ({nz} non-zero)")
    print(f"peak            : {peak} s16 at bin {pkbin} "
          f"(mech angle {pkbin * 360.0 / len(lut):.1f} deg)")
    print(f"rms             : {rms:.1f} s16")
    print(f"mean            : {mean:+.2f} s16   (a calibrated map is ~zero-mean)")
    if abs(mean) > 5:
        print("  WARNING: mean is far from zero -- the map carries a DC torque offset")
    if peak == 0:
        print("  WARNING: map is all zeros -- nothing was calibrated")
    if knobs:
        print(f"clamp           : {knobs['clamp']} s16   (runtime |FF| limit)")
        print(f"gain            : {knobs['gain']:.2f}")
        print(f"harm denoise    : {'on' if knobs['harm_en'] else 'off'}")
        print(f"standstill freeze: {'on' if knobs['freeze_en'] else 'off'}")
        if knobs["clamp"] and peak > knobs["clamp"]:
            print(f"  NOTE: peak {peak} exceeds clamp {knobs['clamp']} -- "
                  "the map is being clipped at runtime")


def emit_header(lut, path, knobs):
    prov = "extracted from board flash" if knobs else "extracted from a console dump"
    body = []
    for i in range(0, len(lut), 8):
        body.append("  " + ", ".join(f"{v:6d}" for v in lut[i:i + 8]) + ",")
    text = f'''/**
  ******************************************************************************
  * @file    cogg_table.h
  * @brief   Anti-cogging feed-forward table for the Trainsonic direct-drive PMSM.
  *
  * Position-indexed Iq feed-forward used to cancel cogging / low-speed torque
  * ripple. Index = mechanical rotor angle (AS5047 14-bit, 0..16383) >> COGG_SHIFT.
  * Values are in s16 current units (same scaling as FOCVars.Iqdref.q).
  *
  * THIS TABLE IS GENERATED -- {prov} by cogg_read.py.
  * Compiling it in makes the calibration survive any reflash, including a full
  * chip erase. Ropetow_CoggInit() loads it first and then lets a valid FLASH map
  * OVERRIDE it, so erase the saved map (console 'n') if you want this one to win.
  *
  * This is a hand-written file (NOT generated by MC Workbench); it survives
  * regeneration. The code that consumes it lives in USER CODE guards.
  ******************************************************************************
  */
#ifndef COGG_TABLE_H
#define COGG_TABLE_H

#include <stdint.h>

/* 16384 mechanical counts / 512 bins = 32 counts/bin = 2^5 -> bin = mech >> 5.
   512 bins resolve mechanical orders up to 256/rev: ample for the felt low-order
   "surge" and moderate cogging, while keeping RAM small. Must stay a power of two
   (the FF lookup wraps with & (COGG_NBINS-1)). */
#define COGG_NBINS  {len(lut)}
#define COGG_SHIFT  5

/* Define COGG_DEFINE_TABLE in exactly ONE translation unit before including this
   header so the const table is instantiated there; everyone else gets the extern. */
#ifdef COGG_DEFINE_TABLE
const int16_t COGG_TABLE_INIT[COGG_NBINS] = {{
{chr(10).join(body)}
}};
#else
extern const int16_t COGG_TABLE_INIT[COGG_NBINS];
#endif

#endif /* COGG_TABLE_H */
'''
    with open(path, "w") as f:
        f.write(text)
    print(f"\nwrote {path} ({len(lut)} bins compiled in)")
    if knobs:
        print(f"  REMINDER: clamp={knobs['clamp']} gain={knobs['gain']:.2f} are NOT "
              "carried by the header -- they live in the flash record only.\n"
              "  Set them in mc_tasks_foc.c if you erase the saved map.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help="flash image (.bin) or console capture (.txt)")
    ap.add_argument("--emit-header", metavar="PATH",
                    help="write a compilable cogg_table.h (e.g. Inc/cogg_table.h)")
    ap.add_argument("--csv", metavar="PATH", help="write bin,value CSV")
    args = ap.parse_args()

    raw = open(args.input, "rb").read()
    # A flash image starts with the magic; anything else is treated as text.
    if len(raw) >= 4 and struct.unpack("<I", raw[:4])[0] == MAGIC:
        lut, knobs, from_flash = parse_flash(raw)
    else:
        lut, knobs, from_flash = parse_console(raw.decode("utf-8", "replace"))

    report(lut, knobs, from_flash)

    if args.csv:
        with open(args.csv, "w") as f:
            f.write("bin,value\n")
            for i, v in enumerate(lut):
                f.write(f"{i},{v}\n")
        print(f"\nwrote {args.csv}")
    if args.emit_header:
        emit_header(lut, args.emit_header, knobs)


if __name__ == "__main__":
    main()
