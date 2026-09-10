/****************************************************************************
 *
 * foldrotor_control — control allocation (step 4d): body-frame wrench
 * (F_b, M_b) -> per-rotor thrust/tilt commands (F1, F2, alpha1, alpha2,
 * beta1, beta2).
 *
 * Implements .claude/specs/allocation.md's "Known requirements" and
 * "Actuator geometry / effectiveness matrix" sections: the frozen,
 * alpha=beta=0 effectiveness matrix M0, its inverse Minv, and the
 * per-rotor thrust-vector decomposition
 *     [F sin(beta); -F cos(beta) sin(alpha); F cos(beta) cos(alpha)]
 * inverted via atan2.
 *
 * Scope is just this math. No uORB, no params read, no hrt_absolute_time()
 * -- matching PositionVelocityControl.hpp / AttitudeRateControl.hpp.
 * allocate() is `const`: unlike the two cascade classes, this allocator
 * carries no integrator or history between calls -- there is nothing here
 * for a disarm/mode-entry reset to act on.
 *
 * ---------------------------------------------------------------------
 * Decisions this file encodes.
 *
 * 1. Minv is derived from M0 at construction via matrix::inv(), not
 *    hardcoded -- allocation.md explicitly prefers this ("removes this
 *    entire class of staleness rather than testing for it"). The spec's
 *    transcribed Minv literal is NOT copied into this file; it lives in
 *    FoldrotorControlTest.cpp as the reference the derived inverse is
 *    checked against.
 * 2. Fold (alpha) is unpinned (2026-09-10) -- see OPEN ITEM (a) below for
 *    the bench evidence that resolved the sign-mapping question first.
 *    alpha is clamped to [-kMaxTilt, kMaxTilt] identically to beta.
 * 3. Clamping is exactly what allocation.md specifies: independent
 *    per-channel saturation, not constrained re-allocation. A clamp
 *    breaks the round-trip identity by construction; this class does not
 *    attempt to redistribute authority across the two rotors after a
 *    clamp.
 * 4. The geometry constants (kDragRatio, kS1y, kS1z, kS2y, kS2z) match the
 *    SDF-verified rotor geometry (RESOLVED 2026-09-09 -- see OPEN ITEM
 *    (b)): s_y = +-0.2684 m, s_z = +0.0301 m, per allocation.md's
 *    "Actuator geometry / effectiveness matrix" table. Previously s_y
 *    was +-0.15 m and s_z +0.02 m, a deferred mismatch (user decision
 *    2026-09-06) that produced a ~1.8x roll response versus commanded --
 *    fixed here, not before, because it needed the SITL bench evidence
 *    (findings.md 2026-09-09 (2)) that made it worth resolving now
 *    rather than staying deferred.
 *
 * ---------------------------------------------------------------------
 * OPEN ITEMS — carried, not resolved. Do not "fix" these without a
 * decision; each one changes flight behaviour.
 *
 * (a) Fold (alpha) unpinned -- RESOLVED 2026-09-10. Bench-measured first
 *     (force/torque sensor, both arms, motor + own fold servo, per
 *     force_moment_bench_commands.md): a positive ArmNFoldJoint angle
 *     produces NEGATIVE Y thrust for both arms, matching Control_Alloc's
 *     own +alpha -> -Ty convention directly -- the previously-assumed
 *     "SDF geometry, verified 2026-09-06" claim (positive ArmNFoldJoint
 *     -> +Y thrust, requiring a negated mapping) was disproven by this
 *     measurement. FoldrotorControl::foldToNormalizedServo() was
 *     corrected from a negated to a direct alpha->ArmNFoldJoint mapping
 *     as part of this unpinning -- see allocation.md's "alpha maps
 *     directly onto the SDF joint convention" section for the full
 *     record. The vehicle now has lateral thrust-vectoring authority,
 *     still subject to the "exact only at zero fold deflection"
 *     kinematic-coupling caveat that spec section also describes (M0 is
 *     a frozen, alpha=beta=0 effectiveness matrix -- accuracy degrades,
 *     not fails outright, as |alpha| grows off zero).
 *
 * (b) Geometry mismatch vs. the SDF -- RESOLVED 2026-09-09. kS1y/kS2y/
 *     kS1z/kS2z now match the SDF-verified rotor geometry (see decision
 *     4 above and allocation.md's Status/geometry sections). Minv is
 *     re-derived from the corrected M0 at construction, same as always
 *     -- no other code path changes. allocation.md's
 *     "Geometry-vs-SDF test" (deliberately unwritten while this was
 *     deferred, since it would have been red on arrival by design) can
 *     now be added as a green regression guard instead; not added in
 *     this diff, since this diff's scope is the geometry constants
 *     themselves plus the tests that pin them, not a new test.
 *
 * (c) Wrench sign/frame convention -- RESOLVED 2026-09-08. Confirmed
 *     root cause: allocation.md's math (Control_Alloc.m, matching the
 *     source thesis derivation, Progress_160726.pdf slides 7-10) is
 *     body FLU (Forward-Left-Up, Z-up) -- that's why its thrust vector
 *     has Tz = +F*cos(beta)*cos(alpha), positive along +Z. The cascade
 *     in FoldrotorControl::Run() produces _F_b/_M_b in PX4 body FRD
 *     (Forward-Right-Down, Z-down), per controller.md. Feeding FRD
 *     values into this FLU-derived math misallocates any wrench with a
 *     nonzero Y or Z component -- not just hover.
 *
 *     This class does NOT flip any sign on F_b/M_b -- it still
 *     implements allocation.md's math exactly as written, expecting its
 *     input already in body FLU. The fix lives at the call site instead:
 *     FoldrotorControl::Run() converts _F_b/_M_b from FRD to FLU
 *     (FoldrotorControl::frdToAllocatorFlu(), a 180 deg rotation about
 *     body X -- X unchanged, Y and Z negate, transcribed from
 *     foldrotor3_tests/test_frame_convention.py's FLU_TO_FRD) *before*
 *     calling allocate(). See FoldrotorControl.hpp/.cpp and
 *     allocation.md for the full record. This class's Interface
 *     contract is therefore: F_b/M_b in, body FLU, matching
 *     Control_Alloc.m's own frame -- the caller is responsible for
 *     getting there.
 *
 ****************************************************************************/

#pragma once

#include <matrix/matrix/math.hpp>

#include <float.h>

namespace foldrotor
{

class FoldrotorAllocation
{
public:
	/**
	 * Per-rotor allocated command.
	 */
	struct Output {
		float F1{0.f};      ///< rotor 1 thrust, N,   [0, 15]
		float F2{0.f};      ///< rotor 2 thrust, N,   [0, 15]
		float alpha1{0.f};  ///< Arm1FoldJoint,  rad, [-0.79, 0.79] (unpinned 2026-09-10, see OPEN ITEM (a))
		float alpha2{0.f};  ///< Arm2FoldJoint,  rad, [-0.79, 0.79] (unpinned 2026-09-10, see OPEN ITEM (a))
		float beta1{0.f};   ///< Arm1TiltJoint,  rad, [-0.79, 0.79]
		float beta2{0.f};   ///< Arm2TiltJoint,  rad, [-0.79, 0.79]
		bool  saturated{false}; ///< true if any channel was clamped this call
	};

	/** allocation.md, "Tilt limit" -- 0.79 rad, not 1.0472 rad, all four joints. */
	static constexpr float kMaxTilt = 0.79f;

	/** allocation.md, "Actuator limits" -- F1, F2 in [0, 15] N. */
	static constexpr float kMaxThrust = 15.f;

	FoldrotorAllocation()
	{
		// M0, allocation.md "The matrices", built in the spec's exact row
		// order: force rows 0-2 are the identity pairs (each rotor's own
		// thrust axis maps straight onto Fx/Fy/Fz), moment rows 3-5 as
		// written there.
		_M0(0, 0) = 1.f; _M0(0, 1) = 0.f;      _M0(0, 2) = 0.f;
		_M0(0, 3) = 1.f; _M0(0, 4) = 0.f;      _M0(0, 5) = 0.f;

		_M0(1, 0) = 0.f; _M0(1, 1) = 1.f;      _M0(1, 2) = 0.f;
		_M0(1, 3) = 0.f; _M0(1, 4) = 1.f;      _M0(1, 5) = 0.f;

		_M0(2, 0) = 0.f; _M0(2, 1) = 0.f;      _M0(2, 2) = 1.f;
		_M0(2, 3) = 0.f; _M0(2, 4) = 0.f;      _M0(2, 5) = 1.f;

		_M0(3, 0) = kDragRatio; _M0(3, 1) = -kS1z; _M0(3, 2) = kS1y;
		_M0(3, 3) = -kDragRatio; _M0(3, 4) = -kS2z; _M0(3, 5) = kS2y;

		_M0(4, 0) = kS1z; _M0(4, 1) = kDragRatio; _M0(4, 2) = 0.f;
		_M0(4, 3) = kS2z; _M0(4, 4) = -kDragRatio; _M0(4, 5) = 0.f;

		_M0(5, 0) = -kS1y; _M0(5, 1) = 0.f; _M0(5, 2) = kDragRatio;
		_M0(5, 3) = -kS2y; _M0(5, 4) = 0.f; _M0(5, 5) = -kDragRatio;

		// Minv derived at init, not hardcoded (allocation.md's explicit
		// preference). The bool-returning overload makes a singular M0
		// detectable rather than silently NaN -- stored in _valid,
		// exposed via isValid().
		_valid = matrix::inv<float, 6>(_M0, _Minv);
	}

	/** True if Minv was successfully derived from M0 at construction. */
	bool isValid() const { return _valid; }

	/** Read-only access to the derived inverse, for the Minv*M0 ~= I test. */
	const matrix::SquareMatrix<float, 6> &getMinv() const { return _Minv; }
	const matrix::SquareMatrix<float, 6> &getM0() const { return _M0; }

	/**
	 * Allocate a body-frame wrench to per-rotor thrust/tilt commands.
	 *
	 * @param F_b desired force,  body FLU (N)   -- NOT PX4's body FRD;
	 *            see OPEN ITEM (c). The caller (FoldrotorControl::Run())
	 *            converts from FRD before calling this.
	 * @param M_b desired moment, body FLU (N*m) -- same caveat.
	 * @return per-rotor commands, clamped to the actuator limits above.
	 *         Fold (alpha1, alpha2) is unpinned -- see OPEN ITEM (a).
	 */
	Output allocate(const matrix::Vector3f &F_b, const matrix::Vector3f &M_b) const
	{
		Output out{};

		// 1) Stack the wrench and 2) solve for the two rotors' thrust
		// vectors: T = Minv * w, w = [Fx, Fy, Fz, Mx, My, Mz].
		matrix::Vector<float, 6> w;
		w(0) = F_b(0); w(1) = F_b(1); w(2) = F_b(2);
		w(3) = M_b(0); w(4) = M_b(1); w(5) = M_b(2);

		const matrix::Vector<float, 6> T = _Minv * w;

		allocateRotor(T(0), T(1), T(2), out.F1, out.alpha1, out.beta1, out.saturated);
		allocateRotor(T(3), T(4), T(5), out.F2, out.alpha2, out.beta2, out.saturated);

		// Fold (alpha) unpinned 2026-09-10 -- see decision 2 and OPEN
		// ITEM (a) in the file header. alpha is clamped to
		// [-kMaxTilt, kMaxTilt] the same as beta, inside allocateRotor().
		return out;
	}

private:
	// allocation.md "Actuator geometry / effectiveness matrix": rotor
	// geometry constants, matching the SDF-verified rotor positions
	// (RESOLVED 2026-09-09 -- see OPEN ITEM (b) and decision 4 above;
	// previously s_y = +-0.15 m, s_z = +0.02 m, a deferred mismatch that
	// produced ~1.8x roll response versus commanded).
	static constexpr float kDragRatio = 0.017f; // k, drag/thrust ratio
	static constexpr float kS1y = 0.2684f;      // rotor 1 y-offset, m (SDF-verified)
	static constexpr float kS1z = 0.0301f;      // rotor 1 z-offset, m (SDF-verified)
	static constexpr float kS2y = -0.2684f;     // rotor 2 y-offset, m (SDF-verified)
	static constexpr float kS2z = 0.0301f;      // rotor 2 z-offset, m (SDF-verified)

	/**
	 * Invert one rotor's thrust vector T = (Tx, Ty, Tz) back to
	 * (F, alpha, beta), per allocation.md's forward map
	 *     Tx = F*sin(beta)
	 *     Ty = -F*cos(beta)*sin(alpha)
	 *     Tz = F*cos(beta)*cos(alpha)
	 * Read directly off that expression:
	 *     F     = norm(Tx, Ty, Tz)
	 *     alpha = atan2(-Ty, Tz)      -- the leading minus on Ty is what
	 *                                    puts the negation in alpha
	 *     beta  = atan2(Tx, hypot(Ty, Tz))
	 * then clamps F to [0, kMaxThrust] and alpha/beta to
	 * [-kMaxTilt, kMaxTilt] (allocation.md "Actuator limits" -- alpha
	 * carries the same +-0.79 rad limit as beta, all four joints),
	 * setting `saturated` if any clamp is active. allocation.md
	 * specifies clamping, not constrained re-allocation -- no attempt is
	 * made to redistribute authority to the other rotor after a clamp.
	 */
	static void allocateRotor(float Tx, float Ty, float Tz,
				  float &F, float &alpha, float &beta, bool &saturated)
	{
		const float norm = sqrtf(Tx * Tx + Ty * Ty + Tz * Tz);

		// Degenerate guard: below this, atan2(0,0) is implementation-
		// defined-ish and the angles are meaningless -- set everything
		// to zero explicitly rather than propagate whatever atan2
		// returns.
		if (norm < 1e-6f) {
			F = 0.f;
			alpha = 0.f;
			beta = 0.f;
			return;
		}

		F = norm;
		alpha = atan2f(-Ty, Tz);
		beta = atan2f(Tx, hypotf(Ty, Tz));

		if (F < 0.f) {
			F = 0.f;
			saturated = true;

		} else if (F > kMaxThrust) {
			F = kMaxThrust;
			saturated = true;
		}

		if (beta < -kMaxTilt) {
			beta = -kMaxTilt;
			saturated = true;

		} else if (beta > kMaxTilt) {
			beta = kMaxTilt;
			saturated = true;
		}

		if (alpha < -kMaxTilt) {
			alpha = -kMaxTilt;
			saturated = true;

		} else if (alpha > kMaxTilt) {
			alpha = kMaxTilt;
			saturated = true;
		}
	}

	matrix::SquareMatrix<float, 6> _M0;
	matrix::SquareMatrix<float, 6> _Minv;
	bool _valid{false};
};

} // namespace foldrotor
