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

## Working style
1. Explore and understand before proposing changes.
2. Propose an implementation plan before writing code. Flag architectural
   decisions instead of making them.
3. Implement small diffs, one at a time.
4. Every diff needs a test tied to a spec verification criterion.
   Interface tests should not require Gazebo; validation tests do.
5. If a bug reveals an unwritten contract: fix the code, update the
   relevant spec, add a regression test — all three, not just the fix.
6. Do not generate the controller or allocation algorithm from scratch.
   Do not substitute PX4's default control allocator for the custom one.
7. Engineering/control decisions belong to the user. Explain tradeoffs;
   don't decide.

## Verification before validation
Order of trust-building: interface tests → SITL open-loop actuator tests
→ force/moment direction tests → closed-loop validation (hover, tracking).
Do not skip ahead, and do not treat a validation failure as proof of a
controller bug until the chain above has been checked.
