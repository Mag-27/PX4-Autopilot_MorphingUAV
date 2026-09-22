"""Attitude+rate loop sim on the SDF rigid body, with the REAL clamping allocator (corrected M0). FLU internally."""
from study import *
zc, KD = -0.0549, KD_SDF
M_FIX = M0(-S_Y, zc, +S_Y, zc, -KD)      # candidate Y (channel map unchanged)
M_SHIP = M_AS
I = mdl.inertia({}, mdl.com({})[1]); Iinv = np.linalg.inv(I)
ATT_P, RP_P, RP_D, RP_I, Y_P = 3.0, 3.5, 0.5, 0.1, 2.5
LIM = np.array([4.0, 0.85, 5.5])

def run(M, gP, gD, lim, T=3.0, dt=1e-3, att0=(0.05,0.05,0.0), Ilim=6.0):
    """Euler angles small-ish; integrate body rates. Returns time, euler history, sat fraction."""
    e = np.array(att0, float); w = np.zeros(3); integ = np.zeros(3); wd_prev = np.zeros(3)
    hist = []; nsat = 0; n = int(T/dt)
    for i in range(n):
        rate_sp = -np.array([e[0], e[1], e[2]]) * ATT_P          # setpoint 0 attitude
        err = rate_sp - w
        wdot = (w - wd_prev)/dt; wd_prev = w.copy()
        Kp = np.array([gP[0], gP[1], gP[2]]); Kd = np.array([gD[0], gD[1], 0.0])
        Mcmd = err*Kp + integ - wdot*Kd
        Mc = np.clip(Mcmd, -lim, lim)
        integ = np.clip(integ + np.array([RP_I, RP_I, 0.0])*err*dt, -Ilim, Ilim)
        # FRD command -> FLU for allocator
        Mflu = np.array([Mc[0], -Mc[1], -Mc[2]]); Fflu = np.array([0, 0, G_W])
        o = alloc_py(M, np.concatenate([Fflu, Mflu])); nsat += o[6]
        t = truth(o, KD)                                          # physical wrench, FLU
        Mphys_frd = np.array([t[3], -t[4], -t[5]])
        w = w + dt*(Iinv @ (Mphys_frd - np.cross(w, I@w)))
        e = e + dt*w                                              # small-angle
        hist.append(np.concatenate([e, w, Mc]))
        if np.abs(e).max() > 1.5: return np.array(hist), nsat/(i+1), False
    return np.array(hist), nsat/n, True

rows = []
for nm, M, gP, gD, lim in [
    ("as shipped (M0 as-is, gains 3.5/0.5/2.5, lim 4/.85/5.5)", M_SHIP, (3.5,3.5,2.5), (0.5,0.5), LIM),
    ("sign fix only, same gains/limits", M_FIX, (3.5,3.5,2.5), (0.5,0.5), LIM),
    ("sign fix + My limit 0.30 (true hover authority)", M_FIX, (3.5,3.5,2.5), (0.5,0.5), np.array([3.8,0.30,3.8])),
    ("sign fix + limits + pitch P/D scaled by Iyy/Ixx", M_FIX, (3.5,3.5*0.0031/0.0645,2.5), (0.5,0.5*0.0031/0.0645), np.array([3.8,0.30,3.8])),
]:
    h, sat, ok = run(M, gP, gD, lim)
    rows.append([nm, "survived 3 s" if ok else f"diverged at {len(h)} ms", f"{np.abs(np.degrees(h[:,:3])).max():.1f}", f"{sat*100:.0f}%"])
tbl(["configuration", "outcome", "max |attitude| (deg)", "allocator saturated"], rows)
