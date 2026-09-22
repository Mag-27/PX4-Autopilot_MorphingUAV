from study import *
import math
FRD = lambda v: np.array([v[0], -v[1], -v[2], v[3], -v[4], -v[5]])   # FLU -> FRD wrench
mc_A, mc_C = models_A, models_C = None, None
p1, _ = mdl.rotor(1, {}); p2, _ = mdl.rotor(2, {}); _, c0 = mdl.com({})
M_C = M0(p1[1]-c0[1], p1[2]-c0[2], p2[1]-c0[1], p2[2]-c0[2], -KD_SDF)

print("## T5. Oracle validation: SDF-FK oracle vs the repo's independent bench measurements (force_moment_test.md, FRD, lift = -Fz)\n")
rows = []
for nm, o, bench in [("-m 1 -v 0.6 (F1=10.066)", [10.066,0,0,0,0,0], {"Fz":-10.066,"Mx":-2.7216,"Mz":+0.2240}),
                     ("-m 2 -v 0.6 (F2=10.066)", [0,10.066,0,0,0,0], {"Fz":-10.066,"Mx":+2.7210,"Mz":-0.2239}),
                     ("-m 1 + -s 2 (F1=10.066, b1=+0.395)", [10.066,0,0,0,0.395,0], {"Fx":3.895,"Fz":-9.285,"Mx":-2.5968,"Mz":-0.8379})]:
    # bench moments are about the bench mount (base_link origin region, static baseline subtracted) - compare deltas about CoM: same for pure thrust
    f = FRD(truth(o)); base = FRD(truth([0]*6))
    d = f - base; idx = {"Fx":0,"Fy":1,"Fz":2,"Mx":3,"My":4,"Mz":5}
    for k, v in bench.items(): rows.append([nm, k, f"{d[idx[k]]:+.4f}", f"{v:+.4f}"])
tbl(["case", "component (FRD)", "oracle", "bench measured"], rows)
print("Oracle reproduces every bench number to a few %, including the sign of the moments: motor 1 (right rotor, +Y in FRD) gives roll-LEFT (Mx<0).\n")

# replica vs real C++
rng = np.random.default_rng(1); W = [np.concatenate([rng.uniform(-8,8,2), [rng.uniform(0,30)], rng.uniform(-1,1,3)*[3,.8,3]]) for _ in range(500)]
R = real_alloc(W); P = [alloc_py(M_AS, w) for w in W]
print("## T6. Python replica of allocate() vs the REAL C++ allocator (500 random wrenches incl. saturating ones)\n")
tbl(["metric", "value"], [["max |F| diff (N)", f"{max(np.abs(r[:2]-p[:2]).max() for r,p in zip(R,P)):.2e}"],
     ["max |angle| diff (rad)", f"{max(np.abs(r[2:6]-p[2:6]).max() for r,p in zip(R,P)):.2e}"],
     ["saturated-flag mismatches", sum(int(r[6]!=p[6]) for r,p in zip(R,P))], ["cases saturated", sum(int(r[6]) for r in R)]])

def envelope(M, idx, lim_name):
    """max |demand| along idx (hover Fz, others 0) with sat==0; then peak physical moment when pushed past."""
    lo, hi = 0.0, 20.0
    for _ in range(60):
        mid = (lo+hi)/2; w = [0,0,G_W,0,0,0]; w[idx] = mid
        if alloc_py(M, w)[6] == 0: lo = mid
        else: hi = mid
    return lo
print("## T7. Moment authority at hover (Fz=W=15.26 N, Fx=Fy=0, other moments 0): largest single-axis demand the allocator can satisfy WITHOUT clamping\n")
rows = []
for nm, idx, lim in (("Mx (roll)", 3, 4.0), ("My (pitch)", 4, 0.85), ("Mz (yaw)", 5, 5.5)):
    eA, eC = envelope(M_AS, idx, lim), envelope(M_C, idx, lim)
    w = [0,0,G_W,0,0,0]; w[idx] = eC; o = alloc_py(M_C, w)
    rows.append([nm, f"{eA:.3f}", f"{eC:.3f}", f"F=({o[0]:.2f},{o[1]:.2f}) a=({o[2]:+.2f},{o[3]:+.2f}) b=({o[4]:+.2f},{o[5]:+.2f})", f"{lim}", f"{lim/eC:.1f}x" if lim > eC else "within"])
tbl(["axis", "A: as shipped (N·m)", "C: SDF-true (N·m)", "outputs at C's limit", "rate-loop limit (kRate*Limit)", "limit / true hover authority"], rows)
print("The rate-loop limits were derived from the allocator's maximum moment with the force/other moments free (4.06/0.90/5.77). "
      "At hover, with Fx=Fy=0 and Fz=W enforced, the usable authority is much smaller.\n")

print("## T8. Saturation behaviour at hover: pitch demand sweep (model C = sign-correct), real clamping, PHYSICAL result\n")
rows = []
for v in (0.05, 0.10, 0.15, 0.20, 0.30, 0.50, 0.85):
    w = [0,0,G_W,0,v,0]; o = alloc_py(M_C, w); t = truth(o, KD_SDF); base = truth(alloc_py(M_C, [0,0,G_W,0,0,0]))
    d = t - base
    rows.append([f"{v:.2f}", f"({o[2]:+.2f},{o[3]:+.2f})", f"{o[0]:.2f}/{o[1]:.2f}", int(o[6]), f"{d[4]:+.3f}", f"{d[1]:+.2f}", f"{d[2]:+.2f}", f"{d[3]:+.3f}"])
tbl(["My demanded", "alpha1,alpha2", "F1/F2", "sat", "phys dMy", "phys dFy (N)", "phys dFz (N)", "phys dMx"], rows)
print("Rows with sat=1: the clamp (no redistribution) turns a small pitch request into a lateral/vertical FORCE error while delivering a fraction of the moment.\n")

print("## T9. Sensitivity at hover: allocator output change per unit moment demand (finite difference, REAL allocator, small demand)\n")
rows = []
for nm, idx in (("Mx",3),("My",4),("Mz",5)):
    e = 1e-3; w0 = [0,0,G_W,0,0,0]; wp = list(w0); wp[idx] = e; wm = list(w0); wm[idx] = -e
    rr = real_alloc([wp, wm]); d = (rr[0]-rr[1])/(2*e)
    rows.append([nm, f"{d[0]:+.2f}", f"{d[1]:+.2f}", f"{d[2]:+.2f}", f"{d[3]:+.2f}", f"{d[4]:+.2f}", f"{d[5]:+.2f}"])
tbl(["per 1 N·m of", "dF1 (N)", "dF2 (N)", "da1 (rad)", "da2 (rad)", "db1 (rad)", "db2 (rad)"], rows)
print("|d alpha/d My| is ~3.8 rad per N·m: My = 0.2 N·m already asks for the full 0.79 rad fold. Roll/yaw need ~0.04–0.05 rad per N·m.\n")

print("## T10. Force sweeps with zero moment demanded (physical result = SDF FK, model C so signs are right); does the allocator hold M=0?\n")
rows = []
for nm, w in [("Fz=W", [0,0,G_W,0,0,0]), ("Fz=25", [0,0,25,0,0,0]), ("Fz=30", [0,0,30,0,0,0]), ("Fz=35", [0,0,35,0,0,0]),
              ("Fx=+2", [2,0,G_W,0,0,0]), ("Fx=+5", [5,0,G_W,0,0,0]), ("Fx=+10", [10,0,G_W,0,0,0]),
              ("Fy=+2", [0,2,G_W,0,0,0]), ("Fy=+5", [0,5,G_W,0,0,0]), ("Fy=+10", [0,10,G_W,0,0,0])]:
    o = alloc_py(M_C, w); t = truth(o)
    rows.append([nm, f"{o[0]:.2f}/{o[1]:.2f}", f"({o[2]:+.2f},{o[3]:+.2f})", f"({o[4]:+.2f},{o[5]:+.2f})", int(o[6]), f"{t[0]:+.2f}/{t[1]:+.2f}/{t[2]:+.2f}", f"{t[3]:+.3f}/{t[4]:+.3f}/{t[5]:+.3f}"])
tbl(["demand (FLU)", "F1/F2", "alpha1,2", "beta1,2", "sat", "phys Fx/Fy/Fz", "phys Mx/My/Mz"], rows)

print("## T11. Rate-loop numbers in the flight regime (what the allocator is being asked for), as-shipped rate limits\n")
Iyy, Ixx, Izz = mdl.inertia({}, c0)[1,1], mdl.inertia({}, c0)[0,0], mdl.inertia({}, c0)[2,2]
P_rp, D_rp, P_y = 3.5, 0.5, 2.5
tbl(["axis", "true inertia I (kg m^2)", "rate P (N·m/(rad/s))", "P/I (1/s)", "rate D used as 'virtual inertia' (FR_RATE_RP_D)", "D/I", "hover authority C (N·m)", "max accel = authority/I (rad/s^2)"],
    [["roll (x)", f"{Ixx:.4f}", P_rp, f"{P_rp/Ixx:.0f}", D_rp, f"{D_rp/Ixx:.1f}", f"{envelope(M_C,3,0):.3f}", f"{envelope(M_C,3,0)/Ixx:.0f}"],
     ["pitch (y)", f"{Iyy:.4f}", P_rp, f"{P_rp/Iyy:.0f}", D_rp, f"{D_rp/Iyy:.0f}", f"{envelope(M_C,4,0):.3f}", f"{envelope(M_C,4,0)/Iyy:.0f}"],
     ["yaw (z)", f"{Izz:.4f}", P_y, f"{P_y/Izz:.0f}", 0.0, "0", f"{envelope(M_C,5,0):.3f}", f"{envelope(M_C,5,0)/Izz:.0f}"]])

print("## T12. Motor / servo command mapping (FoldrotorControl.cpp): thrust -> omega -> normalized; check inverse\n")
km, emin, emax = 5.4844e-6, 308, 2054
rows = []
for F in (0.0, 0.5, 2.0, 5.0, 7.63, 10.0, 12.0, 15.0):
    om = math.sqrt(F/km); n = min(max((om-emin)/(emax-emin), 0), 1); om_back = emin + n*(emax-emin)
    rows.append([F, f"{om:.1f}", f"{n:.4f}", f"{km*om_back**2:.3f}"])
tbl(["F (N)", "omega (rad/s)", "normalized cmd", "F recovered by gz (N)"], rows)
print("Note: F=0 maps to the 308 rad/s idle floor => 0.52 N minimum thrust per rotor, not 0. Fold/tilt: cmd = angle/0.79 (+-1), gz range +-45.26 deg = 0.79 rad.\n")
