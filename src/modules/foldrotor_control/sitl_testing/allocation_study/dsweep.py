import numpy as np
# Rate loop as implemented: M = FF*e_r + I*int(e_r) - D*LPF(rate_dot)
# plant: wdot = M/I_axis ; rate_dot fed back is vehicle_angular_velocity.xyz_derivative,
# low-passed at IMU_DGYRO_CUTOFF (20 Hz in this log).
IXX, IYY, IZZ = 0.06447, 0.00309, 0.06494
FC = 20.0
def sim(I, D, FF, dt, lim, T=0.4, w0=0.0, wsp=0.0):
    tau = 1.0/(2*np.pi*FC); a = dt/(tau+dt)
    w = w0; wprev = w0; f = 0.0; hist=[]
    for k in range(int(T/dt)):
        raw = (w - wprev)/dt; wprev = w
        f = f + a*(raw - f)                       # 20 Hz LPF on rate_dot
        M = FF*(wsp - w) - D*f
        M = np.clip(M, -lim, lim)
        w = w + dt*M/I
        hist.append((M, w))
    return np.array(hist)

def growth(I, D, dt):
    """closed-loop pole of the D-term/plant/LPF loop"""
    tau = 1.0/(2*np.pi*FC); a = dt/(tau+dt)
    return 1 - a*(1 + D/I)

print("Loop-rate dependence of the D-term pole   (pole = 1 - alpha*(1+D/I), |pole|<1 = stable)")
print(f"{'dt':>8} {'rate':>8} | {'roll D=0.5':>12} {'pitch D=0.024':>14}")
for dt in (0.001, 0.00125, 0.002, 0.004):
    print(f"{dt*1e3:7.2f}ms {1/dt:7.0f}Hz | {growth(IXX,0.5,dt):12.3f} {growth(IYY,0.024,dt):14.3f}"
          + ("   <-- ACTUAL (gz max_step_size)" if dt==0.004 else ""))

print("\nStability limit on D at the actual dt = 4 ms:")
for nm, I, Dnow in (("roll ", IXX, 0.5), ("pitch", IYY, 0.024)):
    dt=0.004; tau=1/(2*np.pi*FC); a=dt/(tau+dt)
    Dmax = I*(2.0/a - 1.0)
    print(f"  {nm}: I={I:.5f}  D now={Dnow:<6} D_max={Dmax:.4f}  -> now is {Dnow/Dmax:.2f}x the limit")

print("\nTime-domain, dt=4ms, seeded with a 0.01 rad/s disturbance:")
for nm, I, D, FF, lim in (("roll ",IXX,0.5,3.5,3.8), ("pitch",IYY,0.024,0.17,0.30)):
    h = sim(I,D,FF,0.004,lim,T=0.06,w0=0.01)
    print(f"  {nm} M(N.m): " + " ".join(f"{m:+6.2f}" for m,_ in h[:12]))

print("\nSame, with D scaled to 0.7x the 4 ms limit:")
for nm, I, D, FF, lim in (("roll ",IXX,0.226,3.5,3.8), ("pitch",IYY,0.0108,0.17,0.30)):
    h = sim(I,D,FF,0.004,lim,T=0.06,w0=0.01)
    print(f"  {nm} M(N.m): " + " ".join(f"{m:+6.2f}" for m,_ in h[:12]))

print("\nSame, with D = 0:")
for nm, I, D, FF, lim in (("roll ",IXX,0.0,3.5,3.8), ("pitch",IYY,0.0,0.17,0.30)):
    h = sim(I,D,FF,0.004,lim,T=0.06,w0=0.01)
    print(f"  {nm} M(N.m): " + " ".join(f"{m:+6.2f}" for m,_ in h[:12]))
