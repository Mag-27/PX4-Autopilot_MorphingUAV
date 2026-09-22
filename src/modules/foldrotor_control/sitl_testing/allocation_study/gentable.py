import numpy as np
exec(open('envelope.py').read().split('# FLU')[0])
FXY=1.0
def sim_env(fz):
    def mx(axis):
        lo,hi=0.0,10.0
        for _ in range(50):
            mid=(lo+hi)/2; M=[0.]*3; M[axis]=mid
            ok=all(feasible((FXY*np.cos(t),FXY*np.sin(t),fz),tuple(M)) for t in np.linspace(0,2*np.pi,17))
            lo,hi=(mid,hi) if ok else (lo,mid)
        return lo
    s=[mx(a) for a in range(3)]
    lo,hi=0.0,1.0
    for _ in range(50):
        mid=(lo+hi)/2
        ok=all(feasible((FXY*np.cos(t),FXY*np.sin(t),fz),tuple(mid*v for v in s)) for t in np.linspace(0,2*np.pi,17))
        lo,hi=(mid,hi) if ok else (lo,mid)
    return [lo*v for v in s]
SAFETY=0.85
grid=list(range(0,31,2))
rows=[]
for fz in grid:
    e=sim_env(float(fz)) if fz>0 else [0.,0.,0.]
    rows.append([SAFETY*v for v in e])
print("// Fz(N) : Mx, My, Mz  (N*m, simultaneous, |Fxy|<=1.0N, x%.2f safety)"%SAFETY)
for fz,r in zip(grid,rows):
    print("\t{%6.3ff, %6.3ff, %6.3ff},   // %2d N" % (r[0],r[1],r[2],fz))
