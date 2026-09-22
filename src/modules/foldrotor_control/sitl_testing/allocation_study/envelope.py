import numpy as np
k=-0.022274; s1y=-0.267583; s1z=+0.017010; s2y=0.269177; s2z=+0.017009  # 2026-09-22 ballast mast
M0=np.zeros((6,6))
M0[0]=[1,0,0, 1,0,0]
M0[1]=[0,1,0, 0,1,0]
M0[2]=[0,0,1, 0,0,1]
M0[3]=[k,-s1z,s1y, -k,-s2z,s2y]
M0[4]=[s1z,k,0, s2z,-k,0]
M0[5]=[-s1y,0,k, -s2y,0,-k]
Minv=np.linalg.inv(M0)
MAXT=15.0; MAXA=0.79

def feasible(F,M):
    T=Minv@np.array([*F,*M])
    for i in (0,3):
        Tx,Ty,Tz=T[i],T[i+1],T[i+2]
        n=np.sqrt(Tx*Tx+Ty*Ty+Tz*Tz)
        if n>MAXT: return False
        if abs(np.arctan2(-Ty,Tz))>MAXA: return False
        if abs(np.arctan2(Tx,np.hypot(Ty,Tz)))>MAXA: return False
    return True

# FLU: hover Fz=+15.26. Reserve moments the rate loop can actually ask for.
RES=[(0,0,0),(3.8,0,0),(0,0.30,0),(0,0,3.8),(2.7,0.21,2.7)]
print("Max |Fxy| (N) feasible vs commanded Fz, for several reserved moments")
print(f"{'Fz':>6} | " + " | ".join(f"{str(r):>18}" for r in RES))
for fz in [15.26,17.36,20.0,22.26,25.0]:
    row=[]
    for M in RES:
        lo,hi=0.0,30.0
        for _ in range(60):
            mid=(lo+hi)/2
            # worst-case horizontal direction
            ok=all(feasible((mid*np.cos(t),mid*np.sin(t),fz),M) for t in np.linspace(0,2*np.pi,25))
            lo,hi=(mid,hi) if ok else (lo,mid)
        row.append(lo)
    print(f"{fz:6.2f} | " + " | ".join(f"{v:18.2f}" for v in row))

print()
print("Implied cone half-angle atan(|Fxy|/Fz) at the moderate reserve (deg):")
for fz in [15.26,17.36,20.0,22.26]:
    lo,hi=0.0,30.0
    for _ in range(60):
        mid=(lo+hi)/2
        ok=all(feasible((mid*np.cos(t),mid*np.sin(t),fz),(2.7,0.21,2.7)) for t in np.linspace(0,2*np.pi,25))
        lo,hi=(mid,hi) if ok else (lo,mid)
    print(f"  Fz={fz:6.2f}  |Fxy|max={lo:5.2f}  angle={np.degrees(np.arctan2(lo,fz)):5.2f}")
