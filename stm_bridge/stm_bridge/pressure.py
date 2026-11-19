"""Pressure sensor value -> force (N). Pure functions, standard library only (tests/check_pressure.py).

Sensor: RA12P, force from its datasheet resistance curve (gf -> kOhm, the table below; given by the user
2026-11-19). Circuit: supply - RA12P - (output) - Rc = 10 kOhm - ground, so output = Rc / (R + Rc) and
R = Rc * (1 - out) / out. The STM32 sends the top 8 bits of a 10-bit ADC: out = byte * 4 / 1023.
Between curve points: straight line in log(gf) vs log(kOhm). Below 5 gf (R >= 60 kOhm) the sensor is
open: 0 N. Above 4000 gf (R <= 2.5 kOhm): the end value of the curve. No per-sensor calibration.
"""
from bisect import bisect_left
from math import exp, log

RC_KOHM = 10.0
GF_TO_N = 9.80665 / 1000.0
# RA12P datasheet curve: gf -> kOhm (resistance falls as force rises)
CURVE_GF = (5, 55, 100, 165, 260, 350, 470, 590, 750, 975, 1290, 1585, 1920, 2465, 3240, 4000)
CURVE_KOHM = (60, 19.5, 11.5, 8.8, 7.5, 6.8, 5.4, 4.4, 4.0, 3.5, 3.1, 2.9, 2.7, 2.6, 2.55, 2.5)


def adc_from_byte(b: int) -> float:
    return (int(b) * 4) / 1023.0


def force_n_from_adc(adc: float) -> float:
    """Force (N) for a divider output 0..1; 0 when the sensor is open (below 5 gf) or the output is 0."""
    if adc <= 0.0:
        return 0.0
    r = RC_KOHM * (1.0 - adc) / adc if adc < 1.0 else 0.0     # kOhm, R = Rc (1 - out) / out
    if r >= CURVE_KOHM[0]:
        return 0.0
    if r <= CURVE_KOHM[-1]:
        return CURVE_GF[-1] * GF_TO_N
    inc = CURVE_KOHM[::-1]
    j = bisect_left(inc, r)
    lo, hi = len(inc) - 1 - j, len(inc) - j
    t = (log(r) - log(CURVE_KOHM[lo])) / (log(CURVE_KOHM[hi]) - log(CURVE_KOHM[lo]))
    gf = exp(log(CURVE_GF[lo]) + t * (log(CURVE_GF[hi]) - log(CURVE_GF[lo])))
    return gf * GF_TO_N
