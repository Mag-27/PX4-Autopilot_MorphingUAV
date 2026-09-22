import numpy as np
IXX, IYY, IZZ = 0.06447, 0.00309, 0.06494
FC = 20.0                      # IMU_DGYRO_CUTOFF
def alpha(dt, fc=FC): return dt/(1.0/(2*np.pi*fc) + dt)

print("=== D-term margin at the new 1 kHz loop rate ===")
print(f"{'axis':>6} {'I':>9} {'D':>7} {'pole@1kHz':>10} {'D_max':>8} {'D/D_max':>8}")
for nm, I, D in (("roll", IXX, 0.5), ("pitch", IYY, 0.024), ("yaw", IZZ, 0.0)):
    for dt in (0.001,):
        a = alpha(dt); pole = 1 - a*(1 + D/I); Dmax = I*(2.0/a - 1.0)
        print(f"{nm:>6} {I:9.5f} {D:7.3f} {pole:10.3f} {Dmax:8.4f} {D/Dmax if Dmax else 0:8.2f}")

print("\n=== Full cascade sim, dt=1ms: attitude P -> rate FF/I/D -> plant ===")
def cascade(I, FF, Ki, Kd, lim, ilim, dt=0.001, T=2.0, att0=0.15, w0=0.0, attP=3.0):
    tau=1.0/(2*np.pi*FC); a=dt/(tau+dt)
    att=att0; w=w0; wprev=w0; f=0.0; integ=0.0
    peak=0.0; hist=[]
    for k in range(int(T/dt)):
        raw=(w-wprev)/dt; wprev=w
        f=f+a*(raw-f)
        wsp=attP*(0.0-att)
        err=wsp-w
        M_un=FF*err+integ-Kd*f
        M=float(np.clip(M_un,-lim,lim))
        if abs(M_un)<lim: integ=float(np.clip(integ+Ki*err*dt,-ilim,ilim))
        w=w+dt*M/I
        att=att+dt*w
        peak=max(peak,abs(M)); hist.append((k*dt,att,w,M))
    return hist,peak

for nm,I,FF,Ki,Kd,lim,ilim in (("roll ",IXX,3.5,0.1,0.5,3.8,3.8),
                               ("pitch",IYY,0.17,0.005,0.024,0.30,0.30),
                               ("yaw  ",IZZ,2.5,0.0,0.0,3.8,3.8)):
    h,peak=cascade(I,FF,Ki,Kd,lim,ilim,att0=0.15)
    t=[x[0] for x in h]; att=np.array([x[1] for x in h])
    settle=next((t[i] for i in range(len(att)) if all(abs(att[i:])<0.015)), None)
    overshoot = np.degrees(min(att.min(), 0.0))
    print(f"  {nm} 8.6deg step: peak|M|={peak:.3f} N.m (lim {lim})  settle={settle}s  "
          f"final att={np.degrees(att[-1]):+.3f}deg  overshoot={overshoot:+.3f}deg")

print("\n=== Recovery from the logged arm seed at 1 kHz (roll -0.88, pitch -0.24 rad/s) ===")
for nm,I,FF,Ki,Kd,lim,ilim,w0 in (("roll ",IXX,3.5,0.1,0.5,3.8,3.8,-0.88),
                                  ("pitch",IYY,0.17,0.005,0.024,0.30,0.30,-0.24)):
    h,peak=cascade(I,FF,Ki,Kd,lim,ilim,att0=0.0,w0=w0,T=3.0)
    w=np.array([x[2] for x in h]); att=np.array([x[1] for x in h])
    print(f"  {nm} peak|M|={peak:.3f}  max|att excursion|={np.degrees(np.abs(att).max()):.2f}deg  w@1s={w[1000]:+.4f}  w@3s={w[-1]:+.5f}")

print("\n=== Sensitivity: what loop rate would D=0.5/0.024 go unstable again? ===")
for dt in (0.001,0.0015,0.002,0.0025,0.003,0.004):
    a=alpha(dt)
    print(f"  dt={dt*1e3:4.1f}ms  roll pole={1-a*(1+0.5/IXX):+7.3f}  pitch pole={1-a*(1+0.024/IYY):+7.3f}"
          + ("   <-- unstable" if abs(1-a*(1+0.5/IXX))>1 else ""))
