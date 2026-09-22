import numpy as np
exec(open('envelope.py').read().split('# FLU')[0])
def sim_env(fz,fxy):
    def mx(axis):
        lo,hi=0.0,10.0
        for _ in range(50):
            mid=(lo+hi)/2; M=[0.]*3; M[axis]=mid
            ok=all(feasible((fxy*np.cos(t),fxy*np.sin(t),fz),tuple(M)) for t in np.linspace(0,2*np.pi,17))
            lo,hi=(mid,hi) if ok else (lo,mid)
        return lo
    s=[mx(a) for a in range(3)]
    lo,hi=0.0,1.0
    for _ in range(50):
        mid=(lo+hi)/2
        ok=all(feasible((fxy*np.cos(t),fxy*np.sin(t),fz),tuple(mid*v for v in s)) for t in np.linspace(0,2*np.pi,17))
        lo,hi=(mid,hi) if ok else (lo,mid)
    return [lo*v for v in s]
print("Simultaneous moment envelope vs Fz, at |Fxy| = 1.0 N")
print(f"{'Fz':>6} | {'Mx':>6} | {'My':>6} | {'Mz':>6}")
for fz in [14,15.26,16,17.36,19,21,23,25,27,29]:
    e=sim_env(fz,1.0); print(f"{fz:6.2f} | " + " | ".join(f"{v:6.3f}" for v in e))
