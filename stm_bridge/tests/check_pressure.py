"""pressure.py without ROS: byte -> ADC fraction -> force (N). Expected values by hand.

    python3 stm_bridge/tests/check_pressure.py
"""
import os
import sys
from math import sqrt

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from stm_bridge.pressure import adc_from_byte, force_n_from_adc  # noqa: E402

failures = []


def check(name, got, want, tol=1e-6):
    ok = abs(got - want) <= tol
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {got:.6f}" + ("" if ok else f" (want {want:.6f})"))
    if not ok:
        failures.append(name)


G = 9.80665 / 1000.0      # N per gf

check("byte 255 -> adc 1020/1023 = 0.997067", adc_from_byte(255), 1020 / 1023)
check("byte 0 -> adc 0", adc_from_byte(0), 0.0)
check("byte 128 -> adc 512/1023 = 0.500489", adc_from_byte(128), 512 / 1023)

check("adc 0 -> 0 N", force_n_from_adc(0.0), 0.0)
# adc = Rc / (R + Rc) puts R exactly on a curve point
check("R 60 kOhm (adc 10/70, the 5 gf end) -> open, 0 N", force_n_from_adc(10 / 70), 0.0)
check("R 100 kOhm (adc 10/110) -> open, 0 N", force_n_from_adc(10 / 110), 0.0)
check("R 7.5 kOhm (adc 10/17.5) -> 260 gf = 2.549729 N", force_n_from_adc(10 / 17.5), 260 * G)
check("R 4.0 kOhm (adc 10/14) -> 750 gf = 7.354988 N", force_n_from_adc(10 / 14), 750 * G)
check("R 2.5 kOhm (adc 0.8) -> 4000 gf = 39.226600 N", force_n_from_adc(0.8), 4000 * G)
check("R 1.11 kOhm (adc 0.9) -> past the curve end, 4000 gf", force_n_from_adc(0.9), 4000 * G)
check("adc 1.0 -> 4000 gf", force_n_from_adc(1.0), 4000 * G)
# halfway in log between (5 gf, 60 kOhm) and (55 gf, 19.5 kOhm): R = sqrt(60 * 19.5) = 34.2053, gf = sqrt(5 * 55) = 16.5831
r = sqrt(60 * 19.5)
check("R 34.2053 kOhm (log midpoint) -> 16.5831 gf = 0.162625 N", force_n_from_adc(10 / (10 + r)), sqrt(5 * 55) * G)
check("byte 255 -> 4000 gf (R 0.0294 kOhm)", force_n_from_adc(adc_from_byte(255)), 4000 * G)
check("byte 1 -> adc 0.00391, R 2547 kOhm -> 0 N", force_n_from_adc(adc_from_byte(1)), 0.0)

prev = -1.0
mono = True
for b in range(256):
    f = force_n_from_adc(adc_from_byte(b))
    mono = mono and f >= prev - 1e-12
    prev = f
check("force never falls as the byte rises (0..255)", float(mono), 1.0)

print(f"\n{'all passed' if not failures else f'{len(failures)} failed: ' + ', '.join(failures)}")
sys.exit(1 if failures else 0)
