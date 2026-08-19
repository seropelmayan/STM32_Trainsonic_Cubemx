# AS5047P Magnet Mounting on a Hardened-Steel Shaft — Research Report

*Deep-research run 2026-07-11. 15 sources fetched, 66 claims extracted, 25 verified adversarially (11 confirmed 3-0/2-0, 0 refuted); remaining spot-checked manually against primary sources. Context: Ropetow direct-drive PMSM, AS5047P with 8×3 mm diametric magnet, AGC ≈ 246, magnet currently mounted on/in hardened steel.*

---

## 1. The diagnosis is confirmed: the steel shaft is the problem

ams OSRAM's own magnet-selection app note (AN000271, "Magnet Selection Guide — Rotary Magnetic Position Sensors") says, verbatim:

> "If the magnet is mounted in a ferromagnetic material, such as iron, most of the field lines are attracted by the iron and flow inside the metal shaft (see Figure 22). The magnet is weakened substantially. **This configuration should be avoided!**"

Confirmed 3-0 from the official ams PDF and independently 2-0 from Mouser's mirror of the same app note. Hardened steel is ferromagnetic, so this applies directly.

The manufacturer's sanctioned fix (same app note):

> "If the magnet has to be mounted inside a magnetic shaft, a possible solution is to place a non-magnetic spacer between shaft and magnet... While the magnetic field is rather distorted towards the shaft, there are still adequate field lines available towards the sensor IC. The distortion remains reasonably low."

The app note gives **no quantitative minimum spacer thickness** (verified — guidance is qualitative via its Figure 23). The quantitative number comes from the ODrive practitioner docs ("Designing for Magnetic Encoders"):

> "It is strongly suggested that your magnet has at least **2-3mm of stickout** if using a ferromagnetic magnet holder to retain full field strength."

and the same page reports that a **1–2 mm non-magnetic (plastic) spacer** between magnet and steel shaft restored adequate field in a real weak-field case. (Practitioner source, single extraction — treat as engineering guidance, not spec.)

## 2. Your magnet is not the problem — it's literally the datasheet reference part

- The AS5047P datasheet's **reference magnet, used for all magnetic and system specs, is NdFeB N35H, 8 mm diameter × 3 mm thick** — exactly your size class. (Confirmed 3-0.)
- The app note's recommended grade is **N35H** (N35SH only if >120 °C), **diametric magnetization is the preferred type** ("very good homogeneous Bz field"), and the recommended cylinder sizes explicitly include **8×3 mm** and 6×2.5 mm. (Confirmed 3-0.)

So: keep the 8×3 magnet. Do **not** buy a stronger (N52) magnet to "fix" the weak reading — fix the mounting. (Caveat: verify it really is diametrically magnetized and not a cheap axial disc — see §6.)

## 3. Field requirement and what AGC 246 means

- Required field: **Bz = 35–70 mT at the die surface**, measured on a **1.1 mm radius** circle. Below 35 mT it still works but with increased noise. (Datasheet, confirmed 3-0.)
- The die sits **0.306 mm below the TSSOP package surface** — your effective air gap is (magnet-face-to-package gap) + 0.306 mm. (App note Table 1, confirmed 3-0.)
- **MAGL asserts exactly when AGC = 0xFF (255)**; MAGH when AGC = 0x00. Your **AGC 246 is at 96 % of the gain range, 9 counts from weak-field saturation**. (Datasheet, confirmed 3-0.)
- AGC is a closed loop compensating temperature, air gap, and demagnetization. NdFeB remanence drops with temperature, so a warm motor will push AGC *up* — a setup at 246 cold can hit MAGL hot. (Datasheet, confirmed 3-0.) Target after the fix: **AGC well into mid-range (roughly 60–180), MAGL = 0, MAGH = 0**, with margin left for temperature.

## 4. Mechanical placement spec (ams AS5047P adapter-board manual, quoted verbatim)

- Air gap: "The airgap between the magnet surface and the package should be maintained in the range **0.5mm to 3mm**."
- Centering: "...should be centered on the middle of the package with a tolerance of **0.5mm**." (For low INL aim for ≤0.25 mm — centering error maps directly to angle error.)
- Holder material: "The magnet holder **must not be ferromagnetic**. Materials as **brass, copper, aluminum, stainless steel** are the best choices to make this part." (Stainless = austenitic 303/304/316; hardened/martensitic stainless is magnetic — avoid.)

## 5. Practitioner notes (SimpleFOC / ODrive / Hackaday — unverified-tier, consistent)

- **Adhesive:** cyanoacrylate (superglue) is a poor choice — too thin, wicks into motor bearings. Two-part epoxy (JB Weld reported working where others failed) plus a jig that holds the magnet centered while curing.
- **Temperature:** cheap NdFeB magnets can permanently degrade from as low as ~80 °C; use magnets rated ≥100 °C (the "H" in N35H = 120 °C rating). Boiling-water dunk test to check a suspect magnet.
- **Shape tolerance:** shape/diameter are non-critical as long as magnetization is diametric.
- Watch for mechanical interference: one ODrive user's error bit came from the magnet holder touching sensor-mount screws.

## 6. How to verify the magnet is truly diametric

1. **Viewing film / second magnet:** a diametric disc has N and S split across the *face* (a pole line through the center of the face); an axial disc has one whole face N, the other S. With a known magnet, a diametric disc will cling *edge-on* and prefer to spin in-plane.
2. **Sensor test:** rotate the magnet slowly over the AS5047P — a diametric magnet sweeps the full 0–360° output once per revolution; an axial magnet gives weak, erratic, or near-constant angle with high AGC.

## 7. Fix recipe for this motor (concrete)

1. **Make a non-magnetic end cap** for the hardened shaft: aluminum, brass, or 303/304 stainless (POM/printed plastic acceptable — no load on this part).
   - Bore one end to a snug slip fit over the shaft OD (register on the OD for concentricity, fix with epoxy).
   - Face the other end with a magnet pocket: **Ø 8.1–8.2 mm × 1 mm deep**.
   - Leave **≥ 3 mm of non-magnetic material between the magnet's back face and any steel** (satisfies ODrive's 2–3 mm stickout rule with margin; ams gives no number, only "spacer required").
   - Machine spigot bore and pocket **in one lathe setup** — concentricity is the whole point. Target ≤0.05 mm TIR at the pocket.
2. **Glue the 8×3 diametric magnet** into the pocket with 2-part epoxy (not CA). Verify diametric first (§6).
3. **Set the axial gap** so the magnet face is **0.5–1.5 mm from the package top** (spec allows up to 3 mm, but you want AGC headroom; effective die distance adds 0.306 mm).
4. **Center within 0.5 mm of package center** (aim ≤0.25 mm), magnet face parallel to the chip.
5. **Verify:** read DIAAGC — expect AGC to drop from 246 into mid-range; MAGL = 0, MAGH = 0, COF = 0. Re-check warm after a run. Then re-check encoder-dependent tuning (anti-cogging map should only be captured after this fix, per plan).

## Sources

| Source | Tier |
|---|---|
| AS5047P datasheet DS000324 (ams-osram.com official + Mouser mirrors) | Primary |
| ams AN000271 Magnet Selection Guide / On-Axis Angle Position (official + Mouser mirror) | Primary |
| AS5047P-TS_EK_AB adapter board manual (via ManualsLib) | Primary |
| ODrive docs — "Designing for Magnetic Encoders" | Practitioner |
| SimpleFOC community threads (magnet mounting, adhesives, magnet diameter) | Forum |
| Hackaday.io robot-joint build log (AS5047 placement, JB Weld + jig) | Blog |
