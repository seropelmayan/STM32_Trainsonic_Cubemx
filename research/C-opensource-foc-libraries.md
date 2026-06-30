# C — Open-Source FOC Libraries (with available code)

Cited findings appended as deep-research jobs complete. See [00-INDEX.md](00-INDEX.md).

<!-- FINDINGS APPENDED BELOW -->

> ⚠️ **Editor's note (reconcile with our own data):** this survey asserts "FW is likely not needed at 700 rpm." That generic reasoning does NOT hold for *this* motor — our live logs show `Vq` at ~89% modulation at 580 rpm, i.e. it **is** at the voltage wall by 600 rpm. This motor's Ke is high enough that the wall sits at ~600–650 rpm despite the low rpm. So **FW is relevant here** (the survey's anti-cogging / AS5047P / MESC-V2-FW points still stand). Treat the "FW not needed" line as wrong for our case.


