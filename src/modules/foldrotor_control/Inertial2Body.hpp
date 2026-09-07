/****************************************************************************
 *
 * foldrotor_control — Inertial2Body force-path rotation stage (step 4b).
 *
 * Fixes the defect recorded in .claude/specs/controller.md, "Identified
 * issues (confirmed) → 1. Missing inertial→body rotation on the force
 * path": the velocity loop produces a desired force in the inertial/NED
 * frame, but Control_Alloc's Fx_d/Fy_d/Fz_d inputs are body-frame
 * (allocation.md Interface). Nothing rotated between the two.
 *
 * Scope is deliberately just the rotation. The position/velocity PID that
 * will produce F_i (step 4a), the attitude/rate PID (4c), the allocator
 * (4d), and the wiring into FoldrotorControl::Run() (4e) are separate
 * diffs — this header depends on none of them, and on no uORB topic.
 *
 * The moment path (Mx_b, My_b, Mz_b) is NOT rotated here and needs no
 * rotation: p/q/r and the body moments are body-frame by convention
 * already (controller.md).
 *
 ****************************************************************************/

#pragma once

#include <matrix/matrix/math.hpp>

namespace foldrotor
{

/**
 * Rt: the inertial → body rotation, i.e. the transpose of the standard
 * ZYX (3-2-1 intrinsic Tait-Bryan) body → inertial DCM.
 *
 * Transcribed verbatim from controller.md's MATLAB reference:
 *
 *   Rt = [ cpsi*cth,                  spsi*cth,                 -sth;
 *          cpsi*sth*sphi - spsi*cphi, spsi*sth*sphi + cpsi*cphi, cth*sphi;
 *          cpsi*sth*cphi + spsi*sphi, spsi*sth*cphi - cpsi*sphi, cth*cphi ];
 *
 * This is the same convention step 3's quaternion→Euler conversion
 * already produces (FoldrotorControl.cpp: matrix::Eulerf(matrix::Quatf(q))),
 * so it composes with _euler without adaptation. It is also exactly
 * matrix::Dcmf(euler).transpose() — PX4's Dcm(const Euler&) constructor
 * (src/lib/matrix/matrix/Dcm.hpp) builds the same standard ZYX matrix.
 * That equivalence is asserted in FoldrotorControlTest.cpp rather than
 * assumed, and it is why controller.md's "standard ZYX" claim holds.
 *
 * The literal transcription is kept (instead of just calling
 * matrix::Dcmf(euler).transpose()) so this stage stays line-by-line
 * traceable to the Simulink/MATLAB reference that is the source of truth
 * for the math — the library equivalence is a cross-check on the
 * transcription, not a substitute for it.
 *
 * @param euler current *estimated* attitude (not the setpoint —
 *              controller.md is explicit about this), ZYX phi/theta/psi
 * @return 3x3 rotation taking an inertial/NED vector to body/FRD
 */
inline matrix::Dcmf inertialToBodyRotation(const matrix::Eulerf &euler)
{
	const float cphi = std::cos(euler.phi());
	const float sphi = std::sin(euler.phi());
	const float cth  = std::cos(euler.theta());
	const float sth  = std::sin(euler.theta());
	const float cpsi = std::cos(euler.psi());
	const float spsi = std::sin(euler.psi());

	matrix::Dcmf Rt;

	Rt(0, 0) = cpsi * cth;
	Rt(0, 1) = spsi * cth;
	Rt(0, 2) = -sth;

	Rt(1, 0) = cpsi * sth * sphi - spsi * cphi;
	Rt(1, 1) = spsi * sth * sphi + cpsi * cphi;
	Rt(1, 2) = cth * sphi;

	Rt(2, 0) = cpsi * sth * cphi + spsi * sphi;
	Rt(2, 1) = spsi * sth * cphi - cpsi * sphi;
	Rt(2, 2) = cth * cphi;

	return Rt;
}

/**
 * Rotate a desired force from the inertial/NED frame into the body/FRD
 * frame: F_b = Rt * F_i.
 *
 * @param F_i desired force, inertial/NED (the velocity loop's output)
 * @param euler current estimated attitude, ZYX phi/theta/psi
 * @return desired force in body/FRD, as allocation.md's Interface requires
 */
inline matrix::Vector3f inertialToBody(const matrix::Vector3f &F_i, const matrix::Eulerf &euler)
{
	return inertialToBodyRotation(euler) * F_i;
}

} // namespace foldrotor
