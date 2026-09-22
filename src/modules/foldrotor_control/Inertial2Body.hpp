/****************************************************************************
 *
 * foldrotor_control — Inertial2Body force-path rotation stage (step 4b).
 *
 * REWORKED (2026-09-21 (7)) to the FULL body<-NED rotation, taken directly
 * from the attitude quaternion's DCM. This reverses the 2026-09-17 change
 * to a yaw-only rotation, which was wrong, and restores the full rotation
 * the stage originally had -- but by a route that has no singularity.
 *
 * WHY THE YAW-ONLY VERSION WAS WRONG. Its premise was "this vehicle
 * translates by independent per-rotor thrust vectoring, not by leaning the
 * body, so assume level." The first half is true and the conclusion does
 * not follow. Assuming level does not make the vehicle level: it makes the
 * controller BLIND to the roll/pitch it actually has. The velocity loop
 * asks for a force in NED -- mostly the ~15.3 N holding the vehicle up --
 * and a yaw-only rotation hands that straight to the allocator as though
 * body-down and NED-down were the same axis. They are not, whenever the
 * vehicle is tilted, and the thrust that was meant to point up then points
 * up-and-sideways in the inertial frame.
 *
 * That error is not small and it is not self-correcting. Measured in SITL
 * (findings.md 2026-09-21 (7)): a sustained 22 deg of pitch turned
 * ~17 N of commanded lift into ~6.4 N of uncommanded NED-horizontal force
 * -- against the 1.0 N the position loop is permitted to answer with
 * (kPosVelForceXYLimit). The loop saturates its horizontal authority
 * instantly and loses by a factor of six, every cycle, in whatever
 * direction the vehicle happens to be leaning. The vehicle climbed to
 * altitude and then departed 272 m downrange in 14 s.
 *
 * The full rotation removes this entirely: F_b = R^T * F_i means the
 * delivered inertial force is R * F_b = F_i, at ANY attitude. This is
 * exactly the property a fully-actuated vehicle is supposed to have, and
 * it is why the "not transferable from mc_pos_control" note in
 * reference/px4-module-patterns.md applies to the attitude SETPOINT
 * synthesis (thrustToAttitude/limitTilt) and not to this rotation.
 * mc_pos_control has no equivalent of this stage precisely because its
 * vehicle cannot use one.
 *
 * NO SINGULARITY, unlike the pre-2026-09-17 full-Euler version. That one
 * reconstructed a rotation matrix from phi/theta/psi and so inherited the
 * 3-2-1 gimbal lock at pitch = +/-90 deg that controller.md and
 * controller_params.md both carry as an open concern. This takes the DCM
 * from the quaternion directly and never forms an Euler triple, so the
 * concern does not apply -- the concern was always about the
 * PARAMETERIZATION, not about using the full attitude.
 *
 * The moment path (Mx_b, My_b, Mz_b) is NOT rotated here and needs no
 * rotation: p/q/r and the body moments are body-frame by convention
 * already (controller.md). Unaffected.
 *
 ****************************************************************************/

#pragma once

#include <matrix/matrix/math.hpp>

namespace foldrotor
{

/**
 * Rotate a desired force from the inertial/NED frame into the body/FRD
 * frame: F_b = R_ned_to_body * F_i.
 *
 * @param F_i          desired force, inertial/NED (the velocity loop's output)
 * @param R_ned_to_body  transpose of vehicle_attitude.q's DCM, i.e. the
 *                     NED -> body/FRD rotation. Passed in rather than
 *                     derived here so the caller does the quaternion
 *                     conversion once per cycle.
 * @return desired force in body/FRD, as allocation.md's Interface requires
 */
inline matrix::Vector3f inertialToBody(const matrix::Vector3f &F_i, const matrix::Dcmf &R_ned_to_body)
{
	return R_ned_to_body * F_i;
}

/**
 * Convenience overload taking the attitude quaternion (body -> NED, as
 * vehicle_attitude.q is defined). Transposes it internally.
 */
inline matrix::Vector3f inertialToBody(const matrix::Vector3f &F_i, const matrix::Quatf &q_body_to_ned)
{
	return matrix::Dcmf(q_body_to_ned).transpose() * F_i;
}

} // namespace foldrotor
