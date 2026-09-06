#!/usr/bin/env python3
"""Parse a `gz topic --echo` text-protobuf dump of a force_torque (gz.msgs.Wrench)
topic to CSV. Sibling of parse_joint_state.py, same interface and message-splitting
approach.

Usage: parse_wrench.py <capture.log> <out.csv>

Columns: t,fx,fy,fz,tx,ty,tz -- in whatever frame the sensor's <frame> selects
(world/gz FLU for the foldrotor3 bench fixture). Convert to body FRD with
diag(1,-1,-1) before comparing against any spec expectation.
"""
import re
import sys

VEC = re.compile(r'x:\s*(-?[\d.eE+-]+)\s*y:\s*(-?[\d.eE+-]+)\s*z:\s*(-?[\d.eE+-]+)')


def _vector_after(msg, block):
    """The x/y/z of the named top-level block, e.g. 'force' or 'torque'."""
    start = msg.find(f'\n{block} {{')
    if start == -1:
        return None
    m = VEC.search(msg, start)
    return tuple(float(g) for g in m.groups()) if m else None


def parse(logfile):
    with open(logfile) as f:
        text = f.read()

    rows = []
    for msg in re.split(r'(?=^header \{)', text, flags=re.M):
        if not msg.strip():
            continue
        sec = re.search(r'sec:\s*(-?\d+)', msg)
        if not sec:
            continue
        nsec = re.search(r'nsec:\s*(-?\d+)', msg)
        t = int(sec.group(1)) + (int(nsec.group(1)) / 1e9 if nsec else 0.0)

        force = _vector_after('\n' + msg, 'force')
        torque = _vector_after('\n' + msg, 'torque')
        if force and torque:
            rows.append((t,) + force + torque)
    return rows


def main(logfile, outfile):
    rows = parse(logfile)
    with open(outfile, 'w') as f:
        f.write('t,fx,fy,fz,tx,ty,tz\n')
        for row in rows:
            f.write(','.join(f'{v:.9g}' for v in row) + '\n')
    print(f'wrote {len(rows)} rows to {outfile}')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
