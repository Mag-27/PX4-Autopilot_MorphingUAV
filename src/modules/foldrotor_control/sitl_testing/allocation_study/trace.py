from loopsim import *
h, sat, ok = run(M_FIX, (3.5,3.5,2.5), (0.5,0.5), LIM, T=3.0)
print("step |  roll   pitch   yaw (deg) |   wx     wy     wz  |  Mx_cmd My_cmd Mz_cmd")
for i in list(range(0,40,5)) + list(range(50,400,50)) + list(range(400, len(h), 300)):
    if i >= len(h): break
    r = h[i]; print(f"{i:5d} | {np.degrees(r[0]):+6.2f} {np.degrees(r[1]):+6.2f} {np.degrees(r[2]):+6.2f} | {r[3]:+6.2f} {r[4]:+6.2f} {r[5]:+6.2f} | {r[6]:+6.2f} {r[7]:+6.2f} {r[8]:+6.2f}")
# is the pure attitude/rate loop stable at all if the allocator were perfect (no clamp, exact moment)?
def ideal(gP, gD, T=3.0, dt=1e-3, att0=(0.05,0.05,0.0), lim=None):
    e=np.array(att0,float); w=np.zeros(3); wp=np.zeros(3)
    for i in range(int(T/dt)):
        err = -e*ATT_P - w; wdot=(w-wp)/dt; wp=w.copy()
        Mc = err*np.array(gP) - wdot*np.array([gD[0],gD[1],0.0])
        if lim is not None: Mc = np.clip(Mc,-lim,lim)
        w = w + dt*(Iinv@(Mc - np.cross(w,I@w))); e = e+dt*w
        if np.abs(e).max()>1.5: return f"diverged at {i} ms"
    return f"stable, max {np.degrees(np.abs(e)).max():.2f} deg"
print("\nIDEAL allocator (commanded moment applied exactly), current gains:", ideal((3.5,3.5,2.5),(0.5,0.5)))
print("IDEAL, no D term:", ideal((3.5,3.5,2.5),(0.0,0.0)))
print("IDEAL, current gains + true-authority limits:", ideal((3.5,3.5,2.5),(0.5,0.5), lim=np.array([3.8,0.30,3.8])))
print("IDEAL, pitch gains scaled by Iyy/Ixx, limits:", ideal((3.5,3.5*0.048,2.5),(0.5,0.5*0.048), lim=np.array([3.8,0.30,3.8])))
