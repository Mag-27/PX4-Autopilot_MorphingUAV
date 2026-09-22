## T0. Geometry / constants: allocator vs SDF forward kinematics (gz FLU, +z up, thrust axis +z at zero angles)

| quantity | allocator (header) | SDF FK (FLU) | note |
|---|---|---|---|
| rotor1 y rel CoM (s1y) | +0.2684 | -0.2674 | sign opposite |
| rotor2 y rel CoM (s2y) | -0.2684 | +0.2694 | sign opposite |
| rotor z rel CoM (s1z=s2z) | +0.0301 | -0.0549 (rel base_link origin: -0.0301) | 0.0301 = rotor z rel. base_link ORIGIN, sign flipped; true CoM sits +0.0248 above origin |
| rotor x rel CoM | 0 | +0.00009 / +0.00009 | ok |
| drag/thrust ratio k | 0.017 | 0.022274 (SDF momentConstant; bench-measured 0.2240/10.066 = 0.02225) | header 31% low |
| mass | - | 1.5571 kg -> weight 15.270 N | FR_VEL_Z_GRAV_FF = 15.260 (uses g=9.8) |
| Ixx / Iyy / Izz about CoM | - | 0.06447 / 0.00309 / 0.06494 kg m^2 | Iyy (pitch) is ~21x smaller than Ixx/Izz |

## T1. Interface check of the allocator's matrices

| check | result |
|---|---|
| max |M0(header) - M0(independent r x T + drag build)| | 1.30e-08 |
| max |Minv(header) - numpy.linalg.inv(M0)| | 1.93e-06 |
| max |Minv*M0 - I| | 5.21e-07 |
| cond(M0) | 58.88 |

## T2. Hover trim, solved by hand vs the real allocator

By hand (FLU, Fx=Fy=0, Fz=W, M=0): Fx: T1x+T2x=0; Fy: T1y+T2y=0; Mx,My,Mz rows -> unique solution T1=T2=(0,0,W/2). So F1=F2=W/2, alpha=beta=0.

|  | F1 | F2 | a1 | a2 | b1 | b2 | sat |
|---|---|---|---|---|---|---|---|
| by hand | 7.6300 | 7.6300 | 0 | 0 | 0 | 0 | 0 |
| real allocator | 7.6300 | 7.6300 | -0.0000 | -0.0000 | 0.0000 | 0.0000 | 0 |

Physical wrench that trim really produces (SDF FK, about true CoM), FLU:

|  | Fx | Fy | Fz | Mx | My | Mz |
|---|---|---|---|---|---|---|
| commanded | 0 | 0 | 15.260 | 0 | 0 | 0 |
| truth | +0.0001 | +0.0001 | +15.2600 | +0.0156 | -0.0013 | -0.0000 |

(Residual Mx +0.0156 is the static CoM/rotor asymmetry; identical to the bench baseline +0.0159 in force_moment_test.md.)

## T3. END-TO-END SIGN TEST: demand a wrench at hover, run the REAL allocator, evaluate the PHYSICAL wrench of its output (SDF FK)

Demand is small (no clamping). 'allocator self-model' = M0*T(output); 'physical' = SDF FK. Frame: gz FLU, same frame allocate() is documented to use.

| demand | allocator output | self-model M (demanded) | PHYSICAL dM (vs trim) | physical / demanded |
|---|---|---|---|---|
| Mx=+0.05 | F1=7.723 F2=7.537 a=(-0.000,-0.000) b=(+0.001,-0.001) | +0.0500 | -0.0501 | -1.00 |
| Mx=-0.05 | F1=7.537 F2=7.723 a=(-0.000,-0.000) b=(-0.001,+0.001) | -0.0500 | +0.0501 | -1.00 |
| My=+0.02 | F1=7.653 F2=7.653 a=(-0.077,+0.077) b=(+0.000,+0.000) | +0.0200 | -0.0262 | -1.31 |
| My=-0.02 | F1=7.653 F2=7.653 a=(+0.077,-0.077) b=(+0.000,+0.000) | -0.0200 | +0.0262 | -1.31 |
| Mz=+0.05 | F1=7.636 F2=7.625 a=(-0.000,-0.000) b=(-0.012,+0.012) | +0.0500 | -0.0501 | -1.00 |
| Mz=-0.05 | F1=7.625 F2=7.636 a=(-0.000,-0.000) b=(+0.012,-0.012) | -0.0500 | +0.0501 | -1.00 |

## T4. Which model reproduces physics? (candidate M0's, evaluated in the oracle only; nothing in the repo is changed)

| model | Mx: physical/demanded | My: physical/demanded | Mz: physical/demanded |
|---|---|---|---|
| A as shipped (s=+0.2684/+0.0301, k=.017) | -1.00 | -1.31 | -1.00 |
| B moment rows negated (s=-0.2684/-0.0301, k=-.017) | +1.00 | +1.31 | +1.00 |
| C SDF-true (s=-0.2684/-0.0549 rel CoM, k=-.0223) | +1.00 | +1.00 | +1.00 |

Ratio +1.00 = allocator delivers what it is asked; -1.00 = delivers the opposite (positive feedback in a closed loop).

## T5. Oracle validation: SDF-FK oracle vs the repo's independent bench measurements (force_moment_test.md, FRD, lift = -Fz)

| case | component (FRD) | oracle | bench measured |
|---|---|---|---|
| -m 1 -v 0.6 (F1=10.066) | Fz | -10.0660 | -10.0660 |
| -m 1 -v 0.6 (F1=10.066) | Mx | -2.6912 | -2.7216 |
| -m 1 -v 0.6 (F1=10.066) | Mz | +0.2242 | +0.2240 |
| -m 2 -v 0.6 (F2=10.066) | Fz | -10.0660 | -10.0660 |
| -m 2 -v 0.6 (F2=10.066) | Mx | +2.7118 | +2.7210 |
| -m 2 -v 0.6 (F2=10.066) | Mz | -0.2242 | -0.2239 |
| -m 1 + -s 2 (F1=10.066, b1=+0.395) | Fx | +3.8735 | +3.8950 |
| -m 1 + -s 2 (F1=10.066, b1=+0.395) | Fz | -9.2909 | -9.2850 |
| -m 1 + -s 2 (F1=10.066, b1=+0.395) | Mx | -2.5703 | -2.5968 |
| -m 1 + -s 2 (F1=10.066, b1=+0.395) | Mz | -0.8287 | -0.8379 |

Oracle reproduces every bench number to a few %, including the sign of the moments: motor 1 (right rotor, +Y in FRD) gives roll-LEFT (Mx<0).

## T6. Python replica of allocate() vs the REAL C++ allocator (500 random wrenches incl. saturating ones)

| metric | value |
|---|---|
| max |F| diff (N) | 5.17e-06 |
| max |angle| diff (rad) | 2.57e-06 |
| saturated-flag mismatches | 0 |
| cases saturated | 458 |

## T7. Moment authority at hover (Fz=W=15.26 N, Fx=Fy=0, other moments 0): largest single-axis demand the allocator can satisfy WITHOUT clamping

| axis | A: as shipped (N·m) | C: SDF-true (N·m) | outputs at C's limit | rate-loop limit (kRate*Limit) | limit / true hover authority |
|---|---|---|---|---|---|
| Mx (roll) | 3.869 | 3.826 | F=(0.82,14.69) a=(+0.00,-0.00) b=(-0.79,+0.04) | 4.0 | 1.0x |
| My (pitch) | 0.262 | 0.342 | F=(10.84,10.80) a=(+0.79,-0.79) b=(+0.00,-0.00) | 0.85 | 2.5x |
| Mz (yaw) | 3.901 | 3.854 | F=(10.04,10.86) a=(-0.00,+0.00) b=(+0.79,-0.72) | 5.5 | 1.4x |

The rate-loop limits were derived from the allocator's maximum moment with the force/other moments free (4.06/0.90/5.77). At hover, with Fx=Fy=0 and Fz=W enforced, the usable authority is much smaller.

## T8. Saturation behaviour at hover: pitch demand sweep (model C = sign-correct), real clamping, PHYSICAL result

| My demanded | alpha1,alpha2 | F1/F2 | sat | phys dMy | phys dFy (N) | phys dFz (N) | phys dMx |
|---|---|---|---|---|---|---|---|
| 0.05 | (+0.15,-0.15) | 7.74/7.68 | 0 | +0.050 | -0.00 | -0.00 | +0.001 |
| 0.10 | (+0.29,-0.29) | 7.98/7.93 | 0 | +0.100 | -0.00 | -0.00 | +0.003 |
| 0.15 | (+0.41,-0.42) | 8.37/8.31 | 0 | +0.150 | -0.00 | -0.00 | +0.004 |
| 0.20 | (+0.53,-0.53) | 8.88/8.83 | 0 | +0.200 | -0.00 | -0.00 | +0.005 |
| 0.30 | (+0.72,-0.73) | 10.20/10.16 | 0 | +0.300 | -0.00 | -0.00 | +0.007 |
| 0.50 | (+0.79,-0.79) | 13.59/13.56 | 1 | +0.429 | -0.02 | +3.84 | +0.018 |
| 0.85 | (+0.79,-0.79) | 15.00/15.00 | 1 | +0.474 | +0.00 | +5.86 | +0.027 |

Rows with sat=1: the clamp (no redistribution) turns a small pitch request into a lateral/vertical FORCE error while delivering a fraction of the moment.

## T9. Sensitivity at hover: allocator output change per unit moment demand (finite difference, REAL allocator, small demand)

| per 1 N·m of | dF1 (N) | dF2 (N) | da1 (rad) | da2 (rad) | db1 (rad) | db2 (rad) |
|---|---|---|---|---|---|---|
| Mx | +1.86 | -1.86 | +0.00 | +0.00 | +0.02 | -0.02 |
| My | +0.00 | +0.00 | -3.85 | +3.85 | +0.00 | +0.00 |
| Mz | +0.12 | -0.12 | +0.00 | +0.00 | -0.24 | +0.24 |

|d alpha/d My| is ~3.8 rad per N·m: My = 0.2 N·m already asks for the full 0.79 rad fold. Roll/yaw need ~0.04–0.05 rad per N·m.

## T10. Force sweeps with zero moment demanded (physical result = SDF FK, model C so signs are right); does the allocator hold M=0?

| demand (FLU) | F1/F2 | alpha1,2 | beta1,2 | sat | phys Fx/Fy/Fz | phys Mx/My/Mz |
|---|---|---|---|---|---|---|
| Fz=W | 7.66/7.60 | (-0.00,+0.00) | (+0.00,-0.00) | 0 | +0.00/+0.00/+15.26 | +0.000/-0.001/-0.000 |
| Fz=25 | 12.55/12.45 | (-0.00,+0.00) | (+0.00,-0.00) | 0 | +0.00/+0.00/+25.00 | +0.000/-0.002/-0.000 |
| Fz=30 | 15.00/14.94 | (-0.00,+0.00) | (+0.00,-0.00) | 1 | +0.00/+0.00/+29.94 | +0.015/-0.003/+0.001 |
| Fz=35 | 15.00/15.00 | (-0.00,+0.00) | (+0.00,-0.00) | 1 | +0.00/+0.00/+30.00 | +0.031/-0.003/+0.003 |
| Fx=+2 | 8.11/8.05 | (+0.31,-0.31) | (+0.12,+0.12) | 0 | +2.00/+0.00/+15.26 | +0.003/+0.073/-0.000 |
| Fx=+5 | 10.15/10.10 | (+0.68,-0.68) | (+0.25,+0.25) | 0 | +5.00/+0.00/+15.26 | +0.007/+0.060/-0.001 |
| Fx=+10 | 15.00/15.00 | (+0.79,-0.79) | (+0.33,+0.33) | 1 | +9.78/+0.01/+19.96 | +0.026/-0.033/-0.006 |
| Fy=+2 | 7.93/7.47 | (-0.13,-0.13) | (+0.00,-0.00) | 0 | +0.00/+2.00/+15.26 | -0.123/-0.001/+0.000 |
| Fy=+5 | 8.54/7.52 | (-0.30,-0.34) | (+0.01,-0.01) | 0 | +0.00/+5.00/+15.26 | -0.301/+0.003/-0.000 |
| Fy=+10 | 10.01/8.27 | (-0.52,-0.65) | (+0.01,-0.01) | 0 | +0.00/+10.00/+15.26 | -0.563/+0.014/-0.004 |

## T11. Rate-loop numbers in the flight regime (what the allocator is being asked for), as-shipped rate limits

| axis | true inertia I (kg m^2) | rate P (N·m/(rad/s)) | P/I (1/s) | rate D used as 'virtual inertia' (FR_RATE_RP_D) | D/I | hover authority C (N·m) | max accel = authority/I (rad/s^2) |
|---|---|---|---|---|---|---|---|
| roll (x) | 0.0645 | 3.5 | 54 | 0.5 | 7.8 | 3.826 | 59 |
| pitch (y) | 0.0031 | 3.5 | 1131 | 0.5 | 162 | 0.342 | 110 |
| yaw (z) | 0.0649 | 2.5 | 38 | 0.0 | 0 | 3.854 | 59 |

## T12. Motor / servo command mapping (FoldrotorControl.cpp): thrust -> omega -> normalized; check inverse

| F (N) | omega (rad/s) | normalized cmd | F recovered by gz (N) |
|---|---|---|---|
| 0.0 | 0.0 | 0.0000 | 0.520 |
| 0.5 | 301.9 | 0.0000 | 0.520 |
| 2.0 | 603.9 | 0.1695 | 2.000 |
| 5.0 | 954.8 | 0.3705 | 5.000 |
| 7.63 | 1179.5 | 0.4991 | 7.630 |
| 10.0 | 1350.3 | 0.5970 | 10.000 |
| 12.0 | 1479.2 | 0.6708 | 12.000 |
| 15.0 | 1653.8 | 0.7708 | 15.000 |

Note: F=0 maps to the 308 rad/s idle floor => 0.52 N minimum thrust per rotor, not 0. Fold/tilt: cmd = angle/0.79 (+-1), gz range +-45.26 deg = 0.79 rad.


## T13. Candidate fixes and closed-loop sim (added after user confirmed: Simulink frame is FLU/rotor1 on +Y; k=SDF 0.0223; arms about true CoM; SDF inertia wins)

| candidate | Mx phys/dem | My | Mz | force test 1 | force test 2 |
|---|---|---|---|---|---|
| X: keep M0 signs, true geometry, relabel rotors (rotor1=Motor2) | +1.000 | +1.000 | +1.000 | Fx=2 -> Fx +2.00, My +0.073 | Fy=2 -> Fy +2.00, Mx -0.108 |
| Y: keep channel map, flip y & drag sign in M0 | +1.000 | +1.000 | +1.000 | Fx=2 -> Fx +2.00, My +0.073 | Fy=2 -> Fy +2.00, Mx -0.108 |

hover authority (unclamped), candidate X/Y (identical geometry): [3.811, 0.343, 3.84] [3.811, 0.343, 3.84]
MX row-check: max diff vs C-model: 
Loop sim with PX4-style filtered rate_dot (IMU_DGYRO_CUTOFF = 30 Hz default), start att (2.9,2.9,0) deg

| configuration | outcome |
|---|---|
| ideal alloc, no clamp, gains as-is (D=0.5) | diverged at 3 ms |
| ideal alloc, no clamp, D=0 | stable, settles to 0.00 deg |
| ideal alloc, clamp at true authority, D=0.5 | stable, settles to 5.44 deg |
| SIGN-FIXED allocator, clamp 4/.85/5.5, gains as-is | stable, settles to 34.35 deg |
| SIGN-FIXED allocator, clamp at true authority, gains as-is | stable, settles to 22.05 deg |
| SIGN-FIXED + true authority + pitch P,D scaled by Iyy/Ixx | stable, settles to 0.08 deg |
| AS SHIPPED (sign bug), clamp 4/.85/5.5 | diverged at 136 ms |

