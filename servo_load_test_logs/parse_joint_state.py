#!/usr/bin/env python3
"""Parse `gz topic --echo` text-protobuf dump of a Model (joint_state) topic
and extract time + axis1.position for one named joint, to CSV."""
import re
import sys

def main(logfile, joint_name, outfile):
    with open(logfile) as f:
        text = f.read()

    # split into per-message chunks on the repeated "header {" at top level
    messages = re.split(r'(?=^header \{)', text, flags=re.M)

    rows = []
    for msg in messages:
        if not msg.strip():
            continue
        sec_m = re.search(r'sec:\s*(-?\d+)', msg)
        nsec_m = re.search(r'nsec:\s*(-?\d+)', msg)
        if not sec_m:
            continue
        t = int(sec_m.group(1)) + int(nsec_m.group(1)) / 1e9 if nsec_m else int(sec_m.group(1))

        # find the block for the requested joint: `joint {\n  name: "X"\n ... }`
        # joints are top-level (non-nested) blocks; find start index of this joint's block
        jstart = msg.find(f'name: "{joint_name}"')
        if jstart == -1:
            continue
        # search forward from jstart for "position:" that belongs to axis1 (first one after jstart,
        # before the next "joint {" block)
        next_joint = msg.find('\njoint {', jstart)
        segment = msg[jstart: next_joint if next_joint != -1 else len(msg)]
        pos_m = re.search(r'position:\s*(-?[\d.eE+-]+)', segment)
        vel_m = re.search(r'velocity:\s*(-?[\d.eE+-]+)', segment)
        if pos_m:
            rows.append((t, float(pos_m.group(1)), float(vel_m.group(1)) if vel_m else float('nan')))

    with open(outfile, 'w') as f:
        f.write('t,position_rad,velocity_rad_s\n')
        for t, p, v in rows:
            f.write(f'{t:.6f},{p:.6f},{v:.6f}\n')
    print(f'wrote {len(rows)} rows to {outfile}')

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3])
