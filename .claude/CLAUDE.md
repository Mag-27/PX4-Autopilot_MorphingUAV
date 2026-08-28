# Project: Custom PX4 Flight Controller Integration

## What this is
Custom position/attitude controller with independent control allocation,
developed first in MATLAB/Simulink, being integrated into PX4 as a
standalone module and verified in Gazebo SITL.

**The objective is a trustworthy verification pipeline — not reproducing
MATLAB's numerical tracking results.** A vehicle that tracks poorly is not
proof the controller is wrong; the interface chain must be checked first.

## Hard constraints
- Standalone PX4 module only. Do not modify existing PX4 core modules
  (position/attitude/allocation stack).
- Module replaces the full stack (position → rate → actuators) for this
  vehicle — settled decision, not open for silent revision.
- SITL only until the pipeline is verified. No hardware work yet.

## Source of truth
`specs/system.md` — system-level contracts and acceptance criteria.
`specs/controller.md`, `specs/allocation.md` — subsystem I/O contracts.

Read the relevant spec before touching related code. If code and spec
disagree, or a decision isn't covered by any spec, say so explicitly —
don't resolve it silently.

## What to read when
Do not read every file every session. Context spent on irrelevant
reference material degrades the work.

- **Always:** this file, plus the spec(s) covering what you're touching.
- **`specs/findings.md`:** when you need to know what a previous
  exploration found that hasn't been folded into a spec yet. Short by
  design — entries are pruned to a one-line pointer once promoted.
- **`reference/`:** on demand only. `px4-module-patterns.md` is needed
  when writing or reviewing PX4 module code (lifecycle, work queues,
  uORB, params, build wiring) — not when auditing config, editing
  Simulink, or updating specs.

## Working style
1. Explore and understand before proposing changes. Record findings in
   `specs/findings.md` — dated, with which contract-table rows they
   resolve — don't just report them and let them evaporate.
2. Propose an implementation plan before writing code. Flag architectural
   decisions instead of making them.
3. Implement small diffs, one at a time.
4. Every diff needs a test tied to a spec verification criterion.
   Interface tests should not require Gazebo; validation tests do.
5. If a bug reveals an unwritten contract: fix the code, update the
   relevant spec, add a regression test — all three, not just the fix.
6. When a finding gets folded into a spec, prune its `findings.md`
   entry down to a one-line pointer naming where it landed. The spec
   is then the source of truth; git history holds the detail. Findings
   that turn out to be durable reference knowledge rather than
   discoveries about this codebase belong in `reference/`, not
   `findings.md`.
7. Do not generate the controller or allocation algorithm from scratch.
   Do not substitute PX4's default control allocator for the custom one.
8. Engineering/control decisions belong to the user. Explain tradeoffs;
   don't decide.

## Verification before validation
Order of trust-building: interface tests → SITL open-loop actuator tests
→ force/moment direction tests → closed-loop validation (hover, tracking).
Do not skip ahead, and do not treat a validation failure as proof of a
controller bug until the chain above has been checked.
