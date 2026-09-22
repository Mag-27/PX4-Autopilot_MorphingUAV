import numpy as np
from pyulog import ULog
u = ULog("/home/magesvarlinux/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-21/06_03_49.ulg")
w = [d for d in u.data_list if d.name=="debug_array"][0].data
m = w["id"]==1
t = w["timestamp"][m]; Mx=w["data[3]"][m]; My=w["data[4]"][m]; Mz=w["data[5]"][m]
cm = u.get_dataset("vehicle_control_mode").data; at = cm["timestamp"][cm["flag_armed"]>0]
sel = (t>=at[0])&(t<=at[-1]); t=t[sel]; Mx=Mx[sel]; My=My[sel]; Mz=Mz[sel]
print("sample dt median (us):", np.median(np.diff(t)))
print("\nconsecutive fr_wrench samples, 30 in a row from 40ms after arm (FLU as published):")
i0 = np.searchsorted(t, at[0]+40000)
for i in range(i0, i0+30):
    print(f"  t+{(t[i]-at[0])/1000:7.2f} ms  Mx {Mx[i]:+7.3f}  My {My[i]:+7.3f}  Mz {Mz[i]:+7.3f}")
for nm, s in (("Mx",Mx),("My",My),("Mz",Mz)):
    d = np.diff(np.sign(s[np.abs(s)>1e-6]))
    print(f"{nm}: sign flips per sample = {np.count_nonzero(d)/len(d):.2f}  (1.00 = flips every single sample, i.e. Nyquist oscillation)")
va = u.get_dataset("vehicle_angular_velocity").data
print("\nvehicle_angular_velocity fields:", [k for k in va.keys() if not k.startswith('time')])
