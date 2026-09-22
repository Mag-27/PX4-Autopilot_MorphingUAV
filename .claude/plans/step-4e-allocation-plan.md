# Step 4e — remaining scope: FoldrotorAllocation + actuator publish

## Context

`foldrotor_control` currently runs the full cascade in `Run()` — position/velocity
at 50 Hz, attitude at 250 Hz, rate at 1000 Hz — and computes a real body-frame
wrench (`_F_b`, `_M_b`) every cycle. That wrench is **published nowhere**: the
module is provably inert at the actuator boundary, and `FoldrotorAllocation` does
not exist.

This closes 4e out in one pass: build the allocator as a pure-math class, wire it
into `Run()` so the module actually drives `actuator_motors`/`actuator_servos`,
and add the tests plus a bench-context SITL check that make the first actuator
output trustworthy before any free flight.

This is the diff that first lets this module move the vehicle. Per
`.claude/CLAUDE.md`'s verification-before-validation order, the deliverable is a
*checked* actuator output at a known static setpoint — not a hover attempt.

---

## Item 1 — matrices confirmed available and independently verified

`allocation.md` was updated (uncommitted, this session) and now carries the `M0`
and `Minv` literals in a `matlab` block under "The matrices". `Control_Alloc.m`
itself is still **not in the repo** — `allocation.md`'s own Status section labels
these as transcribed and unverified against the live Simulink model. That
provenance caveat stands and is open item **O-1** below.

Nothing was derived from geometry. What was checked, numerically, this session:

| check | result |
|---|---|
| `det(M0)` | −3.0993e−3 — matches the spec's stated value |
| `cond(M0)` | 58.847 — matches the spec's stated value |
| `max abs(Minv·M0 − I)` | **4.8e−11** |
| `max abs(inv(M0) − Minv)` | 4.79e−11 |

So the transcribed `Minv` is a correct inverse of the transcribed `M0`. Both
describe the mismatched geometry (s_y = ±0.15 vs the SDF's ±0.2684) that
`allocation.md`'s "KNOWN DEFERRED MISMATCH" section records as **deferred by user
decision** — that is not re-opened here.

`max_tilt` is likewise available and settled: **0.79 rad**, applying to all four
joints (fold included), with `F ∈ [0, 15] N` — `allocation.md`, decided
2026-09-06.

---

## Decisions taken into this plan (user, this session)

1. **Publish unconditionally.** The airframe/`rc.mc_defaults` change that stops
   the stock stack is deferred to step 5, per `system.md`. 4e publishes to
   `actuator_motors`/`actuator_servos` with `control_allocator` still running and
   publishing the same topics. The contention is real and is open item **O-4**.
2. **Newtons→normalized conversion lives in `Run()`,** not in the class.
   `FoldrotorAllocation` stays pure spec-math and outputs newtons and radians per
   `allocation.md`'s Interface. The SITL-specific normalization is open item
   **O-5**.

---

## Part A — `FoldrotorAllocation.hpp`

New header-only file in `src/modules/foldrotor_control/`, `namespace foldrotor`,
matching `PositionVelocityControl.hpp` / `AttitudeRateControl.hpp` exactly: pure
math, no uORB, no params read, no `hrt_absolute_time()`, doc-comment block at the
top naming the spec sections and every decision the file encodes.

### Construction — derive `Minv`, don't hardcode it

`allocation.md` explicitly prefers this: *"prefer deriving the inverse from `M0`
at initialisation over hardcoding it — that removes this entire class of
staleness rather than testing for it."* Follow that.

- Geometry constants as named `static constexpr float`: `kDragRatio = 0.017f`,
  `kS1y = 0.15f`, `kS1z = 0.02f`, `kS2y = -0.15f`, `kS2z = 0.02f`. Each carries a
  comment pointing at `allocation.md`'s deferred-mismatch table so a reader hits
  the ≈1.8× roll consequence at the constant, not three files away.
- `M0` built in the constructor from those constants in the spec's exact row
  order (forces rows 0–2 identity pairs; moment rows 3–5 as written).
- `Minv` via PX4's own `matrix::inv<float, 6>(M0, Minv)`
  (`src/lib/matrix/matrix/SquareMatrix.hpp:379`, Gauss-Jordan with partial
  pivoting) — the bool-returning overload, so a singular `M0` is detectable
  rather than silently NaN. Store the success flag; expose it (see `isValid()`).
- The spec's `Minv` literal is **not** copied into the code. It moves into the
  test file as the reference the derived inverse is asserted against (Part C).

### Public interface

```
struct Output {
    float F1, F2;        // N,   [0, 15]
    float alpha1, alpha2; // rad, pinned to 0 (fold), see below
    float beta1, beta2;   // rad, [-0.79, 0.79]
    bool  saturated;      // true if any clamp was active this call
};

Output allocate(const matrix::Vector3f &F_b, const matrix::Vector3f &M_b) const;
bool   isValid() const;   // Minv derivation succeeded
static constexpr float kMaxTilt = 0.79f;
static constexpr float kMaxThrust = 15.f;
```

`allocate()` is `const` — the allocator holds no integrator or history, unlike
the two cascade classes. That is deliberate and worth a comment.

### The math, in order

1. `w = [F_b(0), F_b(1), F_b(2), M_b(0), M_b(1), M_b(2)]` (6×1).
2. `T = Minv * w` → `[Tx1, Ty1, Tz1, Tx2, Ty2, Tz2]`, the two rotor thrust
   vectors.
3. Per rotor `n`, invert `allocation.md`'s thrust vector
   `[F·sin β; −F·cos β·sin α; F·cos β·cos α]`:
   - `F_n = norm(Tx, Ty, Tz)`
   - `α_n = atan2(−Ty, Tz)`
   - `β_n = atan2(Tx, hypot(Ty, Tz))`

   This inversion is read directly off the spec's own expression — note the
   leading minus on the y-component, which is what puts the negation in `α`.
   Write the derivation in the comment so it is auditable without re-deriving.
4. **Degenerate guard:** when `norm(T_n)` is below a small epsilon, `atan2(0,0)`
   is implementation-defined-ish and the angles are meaningless. Set
   `F_n = 0, α_n = 0, β_n = 0` explicitly rather than propagating whatever
   `atan2` returns.
5. **Clamp**, in this order, setting `saturated` if any clamp bites:
   - `F_n` → `[0, 15]` N
   - `β_n` → `[-0.79, +0.79]` rad
   - `α_n` → **pinned to 0**, per the task scope. Do this as an explicit
     `alpha1 = alpha2 = 0.f` with a comment saying it is a scope decision for
     this diff, *not* that the allocator lacks a lateral solution — step 2
     computes a real `α` and it is being discarded. This is open item **O-3**.

Clamping breaks the round trip by construction; the class does not attempt to
re-solve or redistribute after a clamp. `allocation.md` specifies clamping, not
constrained re-allocation — say so in the comment rather than inventing a
fallback.

### Sanity values this design produces (computed, for the test file)

With the wrench expressed **Z-up positive** (see O-2):

| commanded wrench | F1, F2 (N) | β1, β2 (deg) |
|---|---|---|
| hover, Fz = +15.26 | 7.630, 7.630 | 0.00, 0.00 |
| hover + Mx = 0.5 | 9.277, 5.987 | +1.15, −1.78 |
| hover + Mz = 0.2 | 7.733, 7.584 | −4.88, +4.98 |

Round-trip error through the forward map is ≤ 2.2e−15 in all three.

---

## Part B — wiring into `FoldrotorControl::Run()`

Direct calls in `FoldrotorControl.cpp`. No intermediate helper file.

### Members added (`FoldrotorControl.hpp`)

```
foldrotor::FoldrotorAllocation _allocation;      // stateless, no setters
uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
foldrotor::FoldrotorAllocation::Output _alloc_out{};  // held for print_status()
```
Plus `#include <uORB/topics/actuator_motors.h>`, `actuator_servos.h`,
`<uORB/Publication.hpp>`.

### Diff shape in `Run()`

Replace the `_F_b / _M_b now hold the full computed wrench. PUBLISHED NOWHERE`
comment block (currently `FoldrotorControl.cpp:224-226`) with, in order:

1. `_alloc_out = _allocation.allocate(_F_b, _M_b);` — every cycle, at the rate
   loop's 1000 Hz. The allocator is stateless and cheap (one 6×6 multiply plus
   two `atan2`), so it needs no gate of its own; that matches how the rate stage
   is already treated.
2. **Arm/mode gate — reuse what is already there.** The disarm-edge check at
   `FoldrotorControl.cpp:146-151` already reads `_vehicle_control_mode.flag_armed`.
   Gate the publish on the same `flag_armed`, no new condition and no new
   subscription:
   - **armed** → publish the allocated commands.
   - **not armed** → publish `NaN` on every used channel. Per `ActuatorMotors.msg`
     and `ActuatorServos.msg`, *"NaN maps to disarmed"* — this is the message's
     own contract and is what the stock allocator does. Do **not** publish zeros:
     on `actuator_motors` a `0` is a live commanded value that
     `output_limit_calc_single` maps into the ESC range, not an "off".
3. Fill and publish both topics with `timestamp_sample = angular_velocity.timestamp_sample`
   (the same `now` the cascade used) and `timestamp = hrt_absolute_time()`,
   matching PX4 convention. Set unused array entries to `NaN`.

### Channel mapping (from `4026_gz_foldrotor3` + `force_moment_bench_commands.md`)

| topic index | function | joint | value |
|---|---|---|---|
| `motors.control[0]` | Motor1 (101) | `Prop1Joint`, +Y | from `F1` |
| `motors.control[1]` | Motor2 (102) | `Prop2Joint`, −Y | from `F2` |
| `servos.control[0]` | Servo1 (201) | `Arm1FoldJoint` | `−α1 / 0.79` |
| `servos.control[1]` | Servo2 (202) | `Arm1TiltJoint` | `β1 / 0.79` |
| `servos.control[2]` | Servo3 (203) | `Arm2FoldJoint` | `−α2 / 0.79` |
| `servos.control[3]` | Servo4 (204) | `Arm2TiltJoint` | `β2 / 0.79` |

Two things to get right here:

- **The `α` negation** (`allocation.md`, "α carries a sign inversion against the
  SDF joint convention"): positive `α` means −Y thrust, positive `ArmNFoldJoint`
  means +Y thrust, so `α → fold` is **negated** and `β → tilt` is direct. With
  `α` pinned to 0 in this diff the negation is inert numerically, but write it
  into the mapping expression anyway so unpinning `α` later does not silently
  invert the lateral axis.
- **Servo normalization is linear and exact.** `SIM_GZ_SV_MINA/MAXA = ±45.26°`
  (= ±0.79 rad) and `MixingOutput::output_limit_calc_single`
  (`src/lib/mixer_module/mixer_module.cpp:565`) interpolates `[-1,1]` onto
  `[min,max]`, so `β/0.79` is the correct normalized command with no fudge
  factor. Clamp defensively to `[-1,1]` regardless.

### Newtons → normalized motor command (decision 2, open item O-5)

In `Run()`, not in the class. Invert the SDF's own rotor model, whose constants
are in `Tools/simulation/gz/models/foldrotor3/model.sdf:557-559`:

```
F = motorConstant * omega^2,  motorConstant = 5.4844e-06, maxRotVelocity = 2054.42
omega = sqrt(F / motorConstant)
normalized = (omega - SIM_GZ_EC_MIN) / (SIM_GZ_EC_MAX - SIM_GZ_EC_MIN)   // 308..2054
```

Two caveats the implementer must carry, not paper over:

- The exact `[-1,1]`-vs-`[0,1]` semantics of `actuator_motors.control` through
  `FunctionMotors` → `output_limit_calc_single` were traced this session and are
  **not fully pinned down**: `output_limit_calc_single` interpolates `-1..1` onto
  `MIN..MAX`, while the bench's measured ~10 N at `actuator_test -v 0.6` is only
  consistent with `0..1 → MIN..MAX`. `actuator_test` takes a different code path
  (`src/lib/mixer_module/actuator_test.cpp:75-97`) from steady `actuator_motors`
  publication, so the bench number does not settle the question for this path.
  **The bench test in Part D is what resolves it** — that is precisely why the
  static-setpoint check comes before anything closed-loop.
- Hardcode neither 308 nor 2054. Read `SIM_GZ_EC_MIN1`/`MAX1` via `param_find`/
  `param_get` in `parameters_updated()`, so the airframe file stays the single
  source of truth.

### Heartbeat / `print_status()`

Update both. The `(NOT PUBLISHED)` string and the `"publishes nothing to
actuators"` line at `FoldrotorControl.cpp:230-251` become actively misleading the
moment this lands — replace with the allocated `F1/F2/β1/β2`, the `saturated`
flag, and the armed/publishing state. Also update the file-header comment blocks
in `FoldrotorControl.hpp` and `CMakeLists.txt`, which both currently assert the
module is inert at the actuator boundary.

---

## Part C — tests

All in the existing `FoldrotorControlTest.cpp`, following its established style:
every expected value hand-derived and written in a comment above its assertion,
never read back off the implementation. New suite `FoldrotorAllocationTest`.
Pure math, no Gazebo, no uORB — per `.claude/CLAUDE.md` item 4.

**Closes `allocation.md`'s two named-missing tests:**

1. `MinvTimesM0IsIdentity` — the derived `Minv` times `M0` within 1e-6 on all 36
   entries. Also assert `isValid()`. (Note, as `allocation.md` does, that this
   does **not** catch the deferred geometry mismatch — both matrices describe the
   same wrong vehicle.)
2. `DerivedInverseMatchesSpecLiteral` — the runtime-derived `Minv` matches
   `allocation.md`'s transcribed literal within 1e-6. This is the test that keeps
   the spec and the code honest with each other now that the literal is
   deliberately not in the code.
3. `RoundTripReproducesCommandedWrench` — for wrenches inside the unsaturated
   envelope, feed the allocator output back through the forward map
   `[F sinβ; −F cosβ sinα; F cosβ cosα]` and `M0`, and recover the original
   wrench within 1e-4. Use the three cases tabulated in Part A, whose exact
   expected values are already computed.

**Saturation behaviour:**

4. `ThrustClampsToZeroAndFifteen` — a wrench demanding >15 N or <0 N per rotor
   clamps, and `saturated` is set.
5. `TiltClampsAtPointSevenNine` — a wrench demanding β beyond ±0.79 rad clamps
   exactly there, both signs.
6. `MaxTiltConstantIsPointSevenNine` — `allocation.md`'s explicitly requested
   regression guard: assert `kMaxTilt == 0.79f`, **not** 1.0472f.
7. `FoldPinnedToZero` — α1 and α2 are 0 for a wrench with a large `Fy`
   component that would otherwise produce a large α. Pins the scope decision so
   unpinning it later is a deliberate, test-breaking act.
8. `DegenerateZeroWrenchProducesZeroCommands` — the `atan2(0,0)` guard; assert
   all six outputs finite and zero.
9. `NoOutputIsEverNonFinite` — sweep a coarse grid of wrenches including
   extreme/near-singular ones; assert every field `PX4_ISFINITE` and inside its
   declared range. This is `allocation.md`'s "including near-singular inputs".

**Mapping tests (the wiring, not the class):**

10. `AlphaMapsNegatedToFoldChannel` — exercise the mapping expression with a
    nonzero α to prove the negation is applied. Requires factoring the six-line
    mapping in `Run()` into a small `static` free function in the anonymous
    namespace of `FoldrotorControl.cpp` (or a `static` member) so it is testable
    without a work queue. This is the one structural concession; it is not
    another helper *file*.
11. `ThrustToNormalizedInvertsSdfCurve` — `F = 5.4844e-6 · ω²` round-trips: a
    normalized command converted to ω to F and back returns the original.

**Not attempted here:** `allocation.md`'s "Geometry-vs-SDF test" and "α
sign-mapping test (in Gazebo)". The first would assert `M0`'s s_y/s_z against the
SDF and would **fail today by design** — the mismatch is deferred by user
decision, so writing a red test is not this diff's call (open item **O-6**). The
second needs Gazebo and belongs with the α unpinning (O-3).

---

## Part D — bench-context SITL verification

Run **before** any free-flight attempt, and before `system.md`'s last unchecked
Milestone 1 box ("attempt closed-loop Offboard hover"). Module armed but not
flying; reuse the existing bench fixture and FIFO launch pattern from
`force_moment_bench_commands.md` verbatim — no new tooling.

**Sequence:**

1. `make px4_sitl_foldrotor` (the module's own board config) and
   `make tests TESTFILTER=Foldrotor` — Part C must be green first.
2. Launch the bench fixture: `servo_load_test_logs/launch_bench.sh`, defaults
   `work_dir=/tmp/foldrotor3_bench_test`, `gz_model=gz_foldrotor3_bench`.
   Wait for `Gazebo world is ready` and the `pxh>` prompt in `px4.log` —
   commands written to the FIFO before then are silently lost.
3. Confirm the force/torque sensor is live and reads `force.z ≈ −15.26 N` static
   (gz FLU). A near-zero static reading means the fixture is welded through;
   stop, per that doc.
4. Capture ~4 s of actuators-off baseline. Every result below is a delta on the
   15.26 N baseline.
5. Start the module (`foldrotor_control start`) and arm. Command the known static
   setpoint: **hover-weight thrust, zero tilt, zero moment** — the cleanest way
   is to hold the vehicle at its current position with a level, zero-yaw-rate
   setpoint so the cascade's own output settles, then read the wrench the module
   reports.
6. **Hand-check, before interpreting anything else:**
   - `foldrotor_control status` reports `F1 ≈ F2 ≈ 7.63 N`, `β1 ≈ β2 ≈ 0`,
     `α1 = α2 = 0`, `saturated = false`, for a `Fz` of hover weight.
   - `listener actuator_motors` / `listener actuator_servos` show the module's
     values, and confirm **which publisher won** — `control_allocator` is still
     running (O-4). If the stock allocator's output is what reaches Gazebo, every
     number below is meaningless; establish this first.
   - `listener actuator_outputs` shows motor ω near the value the normalization
     predicts, and servo angles near 0.
   - The bench sensor shows lift ≈ 15.26 N against the baseline, with roll/yaw
     residuals at the <0.001 N·m level the 2026-09-06 force/moment test already
     established for a balanced counter-rotating pair.
7. **The disarm check, which is the whole point of the NaN convention:** disarm
   and confirm motor ω drops to the disarmed value and the bench lift delta
   returns to ~0. A module that cannot be switched off at the actuator boundary
   must not fly.

Only after all of the above does closed-loop hover get attempted — that is
`system.md`'s existing unchecked box, and it stays unchecked in this diff.

---

## Open items — explicitly NOT closed by this plan

Same discipline as 4e part 1: recorded, not silently resolved.

- **O-1 — `Control_Alloc.m` provenance.** The `M0`/`Minv` literals are
  mutually consistent (verified 4.8e−11) but transcribed from a Simulink model
  outside version control. `allocation.md`'s own Status section says so. Getting
  `Control_Alloc.m` (or a generated equivalent) under version control is the only
  thing that closes this; this diff does not.
- **O-2 — wrench sign/frame convention. Load-bearing, and unresolved.**
  `allocation.md`'s thrust vector has `Tz = +F·cos β·cos α`, i.e. **thrust
  positive along +Z**. A Z-up-positive hover wrench (`Fz = +15.26`) allocates
  cleanly to `F = 7.63 N, α = β = 0`. The same hover expressed in PX4 body FRD
  (`Fz = −15.26`) allocates to **α = ±180°**, which the ±0.79 rad clamp then
  destroys. `_F_b` out of the cascade is PX4 FRD. This is the same latent
  question as `PositionVelocityControl.hpp`'s **OPEN ITEM (b)** (the literal
  `+9.81` gravity feedforward pointing *down* in NED, "reads like the Simulink
  model was authored Z-up, but that is not confirmed"). The two are one question,
  and it is a control decision, not a code fix — **it must be settled before the
  Part D bench run is interpretable**, because a wrong answer here presents
  exactly as "the allocator saturates at hover". Flag it, do not guess it; a sign
  flipped on a hunch is the failure mode `findings.md` records twice.
- **O-3 — fold (`α`) pinned to 0.** Scope decision for this diff. The allocator
  computes a real `α` and it is discarded, so the vehicle has no lateral
  thrust-vectoring authority until it is unpinned. Unpinning needs the Gazebo α
  sign-mapping test (`allocation.md` Verification) and interacts with the
  fold-carries-tilt kinematic coupling that spec's "exact only at zero fold
  deflection" paragraph describes.
- **O-4 — two publishers on `actuator_motors`/`actuator_servos`.** The stock
  stack still auto-starts via `rc.mc_defaults` in `4026_gz_foldrotor3`; the
  airframe change is deferred to step 5 per user decision. Last writer wins.
  Part D step 6 must establish which one Gazebo actually saw.
- **O-5 — newtons→normalized mapping is in no spec.** It is a SITL-specific
  inversion of the SDF rotor curve, and the exact `[-1,1]`-vs-`[0,1]` semantics of
  `actuator_motors.control` on this code path are not pinned down (see Part B).
  Resolved empirically by Part D, then written into `allocation.md`.
- **O-6 — geometry mismatch stays deferred.** s_y = ±0.15 vs the SDF's ±0.2684 ⇒
  **roll response ≈1.8× commanded**. Deferred by user decision 2026-09-06 and not
  re-opened; `allocation.md`'s "Geometry-vs-SDF test" is deliberately not written,
  since it would be red on arrival. First suspect if roll misbehaves.
- **Carried forward from 4e part 1, untouched:** EKF reset-counter handling still
  not implemented — but it stops being harmless the moment this diff publishes,
  since a reset transient can now reach the actuators. `PositionVelocityControl`
  still has no independent velocity-setpoint path. Integrator reset still handles
  the disarm edge only.
- **Anti-windup becomes live.** `parameters_updated()` currently leaves output and
  integrator limits at ±infinity, with the comment *"the real values come from the
  allocator (4d), not from here."* 4d now exists — but choosing those bounds is a
  control decision (the natural candidates are the allocator's own `[0,15] N` and
  `±0.79 rad`, which are not in the same units as the cascade's wrench output).
  **Not decided in this plan.** Left at ±infinity, anti-windup stays inert, which
  is the honest state; wiring real bounds is a follow-up needing a user call.

---

## Files touched

| file | change |
|---|---|
| `src/modules/foldrotor_control/FoldrotorAllocation.hpp` | **new** — the class |
| `src/modules/foldrotor_control/FoldrotorControl.hpp` | members, includes, header comment |
| `src/modules/foldrotor_control/FoldrotorControl.cpp` | allocate + publish, `parameters_updated()` reads `SIM_GZ_EC_MIN/MAX`, `print_status()`/heartbeat text |
| `src/modules/foldrotor_control/FoldrotorControlTest.cpp` | `FoldrotorAllocationTest` suite (11 tests) |
| `src/modules/foldrotor_control/CMakeLists.txt` | add the header to `SRCS`, update the comment |
| `.claude/specs/allocation.md` | Status/Verification updated once the module exists; record the resolved O-5 mapping |
| `.claude/specs/findings.md` | dated 4e-part-2 entry naming which contract rows this resolves |
| `.claude/specs/system.md` | Controller→Allocation and Allocation→Actuators rows |

No PX4 core module is modified. No new helper file beyond the allocator class.

## Verification commands

```
make px4_sitl_foldrotor                        # builds
make tests TESTFILTER=Foldrotor                # Part C, all green
python3 -m pytest foldrotor3_tests/ -q         # geometry guards still pass
make check_format                              # CI gate
servo_load_test_logs/launch_bench.sh           # Part D bench run
```
