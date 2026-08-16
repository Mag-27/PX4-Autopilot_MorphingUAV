#!/usr/bin/env python3
import csv
import sys
import math

def load(path):
    rows = []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append((float(row['t']), float(row['position_rad']), float(row['velocity_rad_s'])))
    return rows

def analyze(path, commanded_rad, onset_threshold_frac=0.1):
    rows = load(path)
    t0 = rows[0][0]
    ts = [t - t0 for t, p, v in rows]
    ps = [p for t, p, v in rows]

    # detect step onset: first time position crosses onset_threshold_frac of commanded value
    thresh = commanded_rad * onset_threshold_frac
    onset_i = None
    for i, p in enumerate(ps):
        if abs(p) >= abs(thresh):
            onset_i = i
            break
    if onset_i is None:
        print("step never detected")
        return
    onset_t = ts[onset_i]

    # steady-state: last 1.0s of the record
    end_t = ts[-1]
    ss_window = [(t, p) for t, p in zip(ts, ps) if t >= end_t - 1.0]
    ss_vals = [p for t, p in ss_window]
    ss_mean = sum(ss_vals) / len(ss_vals)
    ss_err = ss_mean - commanded_rad
    ss_amp = (max(ss_vals) - min(ss_vals))

    # settling time: first time after onset that position stays within +-5% of commanded
    # for the remainder of the window (simple monotonic-from-the-end scan)
    band = 0.05 * abs(commanded_rad)
    settle_t = None
    for i in range(onset_i, len(ts)):
        if all(abs(ps[j] - commanded_rad) <= band for j in range(i, len(ts))):
            settle_t = ts[i] - onset_t
            break

    # peak overshoot
    peak = max(ps[onset_i:], key=lambda p: abs(p - 0))
    overshoot = peak - commanded_rad

    print(f"file: {path}")
    print(f"  commanded: {commanded_rad:.5f} rad ({math.degrees(commanded_rad):.2f} deg)")
    print(f"  step onset (rel t): {onset_t:.3f} s")
    print(f"  peak value: {peak:.5f} rad, overshoot: {overshoot:+.5f} rad ({math.degrees(overshoot):+.2f} deg)")
    print(f"  settling time (5% band): {settle_t if settle_t is None else f'{settle_t:.3f} s'}")
    print(f"  steady-state mean (last 1s): {ss_mean:.5f} rad ({math.degrees(ss_mean):.2f} deg)")
    print(f"  steady-state error: {ss_err:+.5f} rad ({math.degrees(ss_err):+.3f} deg)")
    print(f"  steady-state oscillation (pk-pk, last 1s): {ss_amp:.5f} rad ({math.degrees(ss_amp):.3f} deg)")

if __name__ == '__main__':
    analyze(sys.argv[1], float(sys.argv[2]))
