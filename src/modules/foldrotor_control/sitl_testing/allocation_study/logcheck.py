import numpy as np, sys
from pyulog import ULog
u = ULog("/home/magesvarlinux/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-21/06_03_49.ulg")
names = [(d.name, d.multi_id) for d in u.data_list]
print([n for n in names if n[0] in ("debug_array","vehicle_angular_velocity","vehicle_status","vehicle_control_mode")])
va = u.get_dataset("vehicle_angular_velocity").data
cm = u.get_dataset("vehicle_control_mode").data
arm_t = cm["timestamp"][np.where(cm["flag_armed"] > 0)[0]]
print("armed from", arm_t[0]/1e6 if len(arm_t) else None, "to", arm_t[-1]/1e6 if len(arm_t) else None)
dbg = [d for d in u.data_list if d.name == "debug_array"]
w = None
for d in dbg:
    names_ = [bytes(np.array(x, dtype=np.uint8)).split(b"\0")[0].decode() for x in np.stack([d.data[f"name[{i}]"] for i in range(10)], axis=1)] if "name[0]" in d.data else None
    ids = d.data["id"]; print("debug_array multi", d.multi_id, "ids", np.unique(ids), "n", len(ids))
    if 1 in np.unique(ids): w = d
t0 = arm_t[0]; t1 = arm_t[-1]
m = w.data["id"] == 1
tw = w.data["timestamp"][m]; Mx = w.data["data[3]"][m]; My_flu = w.data["data[4]"][m]; Mz_flu = w.data["data[5]"][m]
ts = va["timestamp"]; sel = (ts >= t0) & (ts <= t1)
ts = ts[sel]; wx = va["xyz[0]"][sel]; wy = va["xyz[1]"][sel]; wz = va["xyz[2]"][sel]
# angular rates (FRD) on a common grid, 1 ms
grid = np.arange(t0, t1, 1000)
def rs(t, y): return np.interp(grid, t, y)
Mxg, Myg, Mzg = rs(tw, Mx), rs(tw, My_flu), rs(tw, Mz_flu)   # FLU commanded (Mx same as FRD; My,Mz negated vs FRD)
wxg, wyg, wzg = rs(ts, wx), rs(ts, wy), rs(ts, wz)
# smooth derivative of rate over 10 ms
def dd(y, k=10): return (np.roll(y, -k) - np.roll(y, k)) / (2*k*1e-3)
axg = dd(wxg); ayg = dd(wyg); azg = dd(wzg)
print("armed duration %.3f s, samples %d" % ((t1-t0)/1e6, len(grid)))
for nm, Mc, a, sgn in (("roll  (Mx)", Mxg, axg, +1), ("pitch (My)", Myg, ayg, -1), ("yaw   (Mz)", Mzg, azg, -1)):
    Mfrd = sgn*Mc          # commanded moment in FRD
    print(nm)
    for lag_ms in (0, 5, 10, 20, 40):
        k = lag_ms
        c = np.corrcoef(Mfrd[:len(Mfrd)-k] if k else Mfrd, a[k:] if k else a)[0, 1]
        print(f"   corr(M_cmd_FRD(t), angular_accel_FRD(t+{lag_ms}ms)) = {c:+.3f}")

print("\n=== first 160 ms after arm: commanded moment (FRD) vs measured body rate (FRD), 8 ms steps ===")
print(" t_ms | Mx_cmd  My_cmd  Mz_cmd | wx_rad/s wy_rad/s wz_rad/s")
for i in range(0, 160, 8):
    print(f"{i:5d} | {Mxg[i]:+7.3f} {-Myg[i]:+7.3f} {-Mzg[i]:+7.3f} | {wxg[i]:+8.3f} {wyg[i]:+8.3f} {wzg[i]:+8.3f}")
n = 120
for nm, Mc, a, sgn in (("roll ", Mxg, axg, +1), ("pitch", Myg, ayg, -1), ("yaw  ", Mzg, azg, -1)):
    Mfrd = sgn*Mc
    out = []
    for lag in (10, 20, 30, 40):
        out.append(f"lag{lag}ms: {np.corrcoef(Mfrd[:n], a[lag:n+lag])[0,1]:+.2f}")
    print(nm, "corr(M_cmd, angular_accel(t+lag)) in first 120ms:", "  ".join(out))

print("\n=== YAW ONLY (FR_RATE_YAW_D = 0, so no D-term confound): mean commanded Mz vs realised yaw acceleration ===")
Izz = 0.06494
for a_, b_ in ((40, 150), (100, 250), (150, 400), (250, 600)):
    sl = slice(a_, b_)
    Mz_frd = -Mzg[sl]
    dw = (wzg[b_-1] - wzg[a_]) / ((b_-1-a_)*1e-3)
    print(f"t={a_:3d}-{b_:3d} ms: mean Mz_cmd(FRD)={Mz_frd.mean():+.3f} N·m -> expected accel {Mz_frd.mean()/Izz:+7.1f} rad/s^2 | observed d(wz)/dt={dw:+7.2f} rad/s^2 | wz {wzg[a_]:+.2f} -> {wzg[b_-1]:+.2f}")
