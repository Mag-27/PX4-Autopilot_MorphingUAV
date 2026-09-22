# Allocation study (2026-09-21)

Exploratory verification of `FoldrotorAllocation` against the SDF forward kinematics and the bench data in
`.claude/specs/force_moment_test.md`. Not a unit test; see `.claude/specs/findings.md` (2026-09-21) for what it found.
`REPORT.md` is the generated output of `study.py` + `study2.py`.

- `drv.cpp` — driver around the REAL `FoldrotorAllocation.hpp` (stdin wrench -> F1 F2 a1 a2 b1 b2 sat).
- `sdf_fk.py` — forward kinematics of `foldrotor3/model.sdf` (rotor position, thrust axis, CoM, inertia).
- `study.py`, `study2.py` — oracle + tables. `logcheck.py` — checks a flight `.ulg` (path hardcoded).

Build the driver (needs an empty stub `px4_boardconfig.h`, the header is otherwise standalone):

    mkdir stub && : > stub/px4_boardconfig.h
    g++ -std=c++17 -I<px4>/src/lib -I<px4>/platforms/common/include -Istub drv.cpp -o drv
    python3 study.py; python3 study2.py
