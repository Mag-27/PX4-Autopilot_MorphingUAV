from loopsim import *
def run2(M, gP, gD, lim, fc=30.0, T=3.0, dt=1e-3, att0=(0.05,0.05,0.0), ideal=False):
    """Same loop, but rate_dot is a LOW-PASS FILTERED derivative (PX4 IMU_DGYRO_CUTOFF, 2nd-order butterworth ~ 1st-order here)."""
    e=np.array(att0,float); w=np.zeros(3); wp=np.zeros(3); integ=np.zeros(3); wd_f=np.zeros(3)
    a = 1.0 - np.exp(-2*np.pi*fc*dt)
    for i in range(int(T/dt)):
        err = -e*ATT_P - w
        wd = (w-wp)/dt; wp=w.copy(); wd_f = wd_f + a*(wd-wd_f)
        Mc = err*np.array(gP) + integ - wd_f*np.array([gD[0],gD[1],0.0])
        integ = np.clip(integ + np.array([RP_I,RP_I,0.0])*err*dt, -6, 6)
        Mc = np.clip(Mc,-lim,lim)
        if ideal: Mphys = Mc
        else:
            o = alloc_py(M, np.concatenate([[0,0,G_W],[Mc[0],-Mc[1],-Mc[2]]])); t = truth(o,KD)
            Mphys = np.array([t[3],-t[4],-t[5]])
        w = w + dt*(Iinv@(Mphys - np.cross(w,I@w))); e = e+dt*w
        if np.abs(e).max()>1.5: return f"diverged at {i} ms"
    return f"stable, settles to {np.degrees(np.abs(e)).max():.2f} deg"
BIG = np.array([1e9,1e9,1e9]); TRUE = np.array([3.8,0.30,3.8])
print("Loop sim with PX4-style filtered rate_dot (IMU_DGYRO_CUTOFF = 30 Hz default), start att (2.9,2.9,0) deg\n")
rows=[]
for nm, M, gP, gD, lim, idl in [
  ("ideal alloc, no clamp, gains as-is (D=0.5)", None,(3.5,3.5,2.5),(0.5,0.5),BIG,True),
  ("ideal alloc, no clamp, D=0", None,(3.5,3.5,2.5),(0.0,0.0),BIG,True),
  ("ideal alloc, clamp at true authority, D=0.5", None,(3.5,3.5,2.5),(0.5,0.5),TRUE,True),
  ("SIGN-FIXED allocator, clamp 4/.85/5.5, gains as-is", M_FIX,(3.5,3.5,2.5),(0.5,0.5),LIM,False),
  ("SIGN-FIXED allocator, clamp at true authority, gains as-is", M_FIX,(3.5,3.5,2.5),(0.5,0.5),TRUE,False),
  ("SIGN-FIXED + true authority + pitch P,D scaled by Iyy/Ixx", M_FIX,(3.5,3.5*0.048,2.5),(0.5,0.5*0.048),TRUE,False),
  ("AS SHIPPED (sign bug), clamp 4/.85/5.5", M_SHIP,(3.5,3.5,2.5),(0.5,0.5),LIM,False),
]:
    rows.append([nm, run2(M,gP,gD,lim,ideal=idl)])
tbl(["configuration","outcome"],rows)
