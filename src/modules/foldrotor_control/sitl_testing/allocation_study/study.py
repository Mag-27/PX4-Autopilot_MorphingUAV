"""Allocator study. C++ driver (drv) = REAL FoldrotorAllocation. Python = independent oracle (SDF FK + hand solve)."""
import numpy as np, subprocess, sys
sys.path.insert(0, ".")
from sdf_fk import Model
np.set_printoptions(precision=4, suppress=True, linewidth=160)
MAXT, MAXF = 0.79, 15.0
G_W = 15.260017            # FR_VEL_Z_GRAV_FF
KD_HDR, KD_SDF = 0.017, 0.022274
S_Y, S_Z = 0.2684, 0.0301

def real_alloc(wrenches):
    """Run the REAL allocator (C++ driver). wrenches: list of 6-vectors (body 'allocator' frame)."""
    inp = "\n".join(" ".join(f"{x:.9g}" for x in w) for w in wrenches) + "\n"
    out = subprocess.run(["./drv"], input=inp, capture_output=True, text=True).stdout.split("\n")
    return [np.array([float(v) for v in l.split()]) for l in out if l.strip()]

def M0(s1y, s1z, s2y, s2z, k):
    """Same construction as FoldrotorAllocation.hpp (rows 3-5 = r x T + drag along T, sign pattern as header)."""
    M = np.zeros((6, 6)); M[0,0]=M[0,3]=M[1,1]=M[1,4]=M[2,2]=M[2,5]=1
    M[3,:] = [k, -s1z, s1y, -k, -s2z, s2y]; M[4,:] = [s1z, k, 0, s2z, -k, 0]; M[5,:] = [-s1y, 0, k, -s2y, 0, -k]
    return M

def alloc_py(M, w):
    """Python replica of allocate(): T=Minv w, atan2 inversion, clamp (no redistribution). returns [F1,F2,a1,a2,b1,b2,sat]"""
    T = np.linalg.inv(M) @ np.asarray(w, float); sat = 0; o = []
    for i in (0, 3):
        tx, ty, tz = T[i:i+3]; n = np.sqrt(tx*tx+ty*ty+tz*tz)
        if n < 1e-6: o.append((0, 0, 0)); continue
        F, a, b = n, np.arctan2(-ty, tz), np.arctan2(tx, np.hypot(ty, tz))
        if F > MAXF: F = MAXF; sat = 1
        if abs(b) > MAXT: b = np.sign(b)*MAXT; sat = 1
        if abs(a) > MAXT: a = np.sign(a)*MAXT; sat = 1
        o.append((F, a, b))
    return np.array([o[0][0], o[1][0], o[0][1], o[1][1], o[0][2], o[1][2], sat])

mdl = Model()
def truth(o, kd=KD_SDF):
    """True thrust wrench (gz FLU, about true CoM) for allocator outputs o=[F1,F2,a1,a2,b1,b2,..], from SDF FK.
    Drag torque along the thrust axis, magnitude kd*F, sign from bench (force_moment_test.md: motor1 dMz_FRD=+0.224 => FLU -kd)."""
    F1, F2, a1, a2, b1, b2 = o[:6]
    q = {"Arm1FoldJoint": a1, "Arm1TiltJoint": b1, "Arm2FoldJoint": a2, "Arm2TiltJoint": b2}
    _, c = mdl.com(q); f = np.zeros(3); mo = np.zeros(3)
    for k, F, sgn in ((1, F1, -1.0), (2, F2, +1.0)):
        p, ax = mdl.rotor(k, q); T = F*ax; f += T; mo += np.cross(p-c, T) + sgn*kd*T
    return np.concatenate([f, mo])

def fwd_M0(M, o):   # allocator's own forward model of its clamped output
    F1, F2, a1, a2, b1, b2 = o[:6]
    T = lambda F, a, b: F*np.array([np.sin(b), -np.cos(b)*np.sin(a), np.cos(b)*np.cos(a)])
    return M @ np.concatenate([T(F1, a1, b1), T(F2, a2, b2)])

def tbl(hdr, rows, fmt=None):
    print("| " + " | ".join(hdr) + " |"); print("|" + "|".join("---" for _ in hdr) + "|")
    for r in rows: print("| " + " | ".join(str(x) for x in r) + " |")
    print()

f4 = lambda x: f"{x:+.4f}"; f3 = lambda x: f"{x:+.3f}"
M_AS = M0(S_Y, S_Z, -S_Y, S_Z, KD_HDR)

if __name__ == "__main__":
    # ---------------- T0: constants vs SDF FK
    print("## T0. Geometry / constants: allocator vs SDF forward kinematics (gz FLU, +z up, thrust axis +z at zero angles)\n")
    Mtot, c = mdl.com({}); p1, a1 = mdl.rotor(1, {}); p2, a2 = mdl.rotor(2, {}); I = mdl.inertia({}, c)
    tbl(["quantity", "allocator (header)", "SDF FK (FLU)", "note"], [
        ["rotor1 y rel CoM (s1y)", "+0.2684", f4(p1[1]-c[1]), "sign opposite"],
        ["rotor2 y rel CoM (s2y)", "-0.2684", f4(p2[1]-c[1]), "sign opposite"],
        ["rotor z rel CoM (s1z=s2z)", "+0.0301", f"{p1[2]-c[2]:+.4f} (rel base_link origin: {p1[2]:+.4f})", "0.0301 = rotor z rel. base_link ORIGIN, sign flipped; true CoM sits +0.0248 above origin"],
        ["rotor x rel CoM", "0", f"{p1[0]-c[0]:+.5f} / {p2[0]-c[0]:+.5f}", "ok"],
        ["drag/thrust ratio k", "0.017", "0.022274 (SDF momentConstant; bench-measured 0.2240/10.066 = 0.02225)", "header 31% low"],
        ["mass", "-", f"{Mtot:.4f} kg -> weight {Mtot*9.80665:.3f} N", "FR_VEL_Z_GRAV_FF = 15.260 (uses g=9.8)"],
        ["Ixx / Iyy / Izz about CoM", "-", f"{I[0,0]:.5f} / {I[1,1]:.5f} / {I[2,2]:.5f} kg m^2", "Iyy (pitch) is ~21x smaller than Ixx/Izz"],
    ])
    # ---------------- T1: M0 build
    real = subprocess.run(["./drv", "matrix"], capture_output=True, text=True).stdout.split("\n")
    M0h = np.array([[float(v) for v in l.split()] for l in real[:6]]); Minvh = np.array([[float(v) for v in l.split()] for l in real[6:12]])
    print("## T1. Interface check of the allocator's matrices\n")
    tbl(["check", "result"], [
        ["max |M0(header) - M0(independent r x T + drag build)|", f"{np.abs(M0h-M_AS).max():.2e}"],
        ["max |Minv(header) - numpy.linalg.inv(M0)|", f"{np.abs(Minvh-np.linalg.inv(M0h)).max():.2e}"],
        ["max |Minv*M0 - I|", f"{np.abs(Minvh@M0h-np.eye(6)).max():.2e}"],
        ["cond(M0)", f"{np.linalg.cond(M0h):.2f}"],
    ])
    # ---------------- T2: hover by hand
    print("## T2. Hover trim, solved by hand vs the real allocator\n")
    print("By hand (FLU, Fx=Fy=0, Fz=W, M=0): Fx: T1x+T2x=0; Fy: T1y+T2y=0; Mx,My,Mz rows -> unique solution T1=T2=(0,0,W/2). "
          "So F1=F2=W/2, alpha=beta=0.\n")
    r = real_alloc([[0, 0, G_W, 0, 0, 0]])[0]
    tr = truth(r)
    tbl(["", "F1", "F2", "a1", "a2", "b1", "b2", "sat"], [
        ["by hand", f"{G_W/2:.4f}", f"{G_W/2:.4f}", "0", "0", "0", "0", "0"],
        ["real allocator", *[f"{x:.4f}" for x in r[:6]], int(r[6])]])
    print("Physical wrench that trim really produces (SDF FK, about true CoM), FLU:\n")
    tbl(["", "Fx", "Fy", "Fz", "Mx", "My", "Mz"], [["commanded", "0", "0", f"{G_W:.3f}", "0", "0", "0"], ["truth", *[f4(x) for x in tr]]])
    print("(Residual Mx +0.0156 is the static CoM/rotor asymmetry; identical to the bench baseline +0.0159 in force_moment_test.md.)\n")
    # ---------------- T3: direction of physics response
    print("## T3. END-TO-END SIGN TEST: demand a wrench at hover, run the REAL allocator, evaluate the PHYSICAL wrench of its output (SDF FK)\n")
    print("Demand is small (no clamping). 'allocator self-model' = M0*T(output); 'physical' = SDF FK. Frame: gz FLU, same frame allocate() is documented to use.\n")
    dem = [("Mx", 3, 0.05), ("Mx", 3, -0.05), ("My", 4, 0.02), ("My", 4, -0.02), ("Mz", 5, 0.05), ("Mz", 5, -0.05)]
    rows = []
    for nm, idx, v in dem:
        w = [0, 0, G_W, 0, 0, 0]; w[idx] = v; o = real_alloc([w])[0]; s = fwd_M0(M_AS, o); tr = truth(o)
        base = truth(real_alloc([[0, 0, G_W, 0, 0, 0]])[0])
        rows.append([f"{nm}={v:+.2f}", f"F1={o[0]:.3f} F2={o[1]:.3f} a=({o[2]:+.3f},{o[3]:+.3f}) b=({o[4]:+.3f},{o[5]:+.3f})",
                     f4(s[idx]), f4(tr[idx]-base[idx]), f"{(tr[idx]-base[idx])/v:+.2f}"])
    tbl(["demand", "allocator output", "self-model M (demanded)", "PHYSICAL dM (vs trim)", "physical / demanded"], rows)
    # ---------------- T4: candidate models (oracle-only)
    print("## T4. Which model reproduces physics? (candidate M0's, evaluated in the oracle only; nothing in the repo is changed)\n")
    models = {
        "A as shipped (s=+0.2684/+0.0301, k=.017)": M_AS,
        "B moment rows negated (s=-0.2684/-0.0301, k=-.017)": M0(-S_Y, -S_Z, S_Y, -S_Z, -KD_HDR),
        "C SDF-true (s=-0.2684/-0.0549 rel CoM, k=-.0223)": M0(p1[1]-c[1], p1[2]-c[2], p2[1]-c[1], p2[2]-c[2], -KD_SDF),
    }
    rows = []
    for nm, M in models.items():
        errs = []
        for idx, v in ((3, 0.05), (4, 0.02), (5, 0.05), (3, -0.05), (4, -0.02), (5, -0.05)):
            w = [0, 0, G_W, 0, 0, 0]; w[idx] = v; o = alloc_py(M, w); base = truth(alloc_py(M, [0, 0, G_W, 0, 0, 0]))
            errs.append((truth(o)[idx]-base[idx])/v)
        rows.append([nm, *[f"{e:+.2f}" for e in errs[:3]]])
    tbl(["model", "Mx: physical/demanded", "My: physical/demanded", "Mz: physical/demanded"], rows)
    print("Ratio +1.00 = allocator delivers what it is asked; -1.00 = delivers the opposite (positive feedback in a closed loop).\n")
