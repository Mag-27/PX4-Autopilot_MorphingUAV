import numpy as np
exec(open('envelope.py').read().split('# FLU')[0])

def maxmom(fz, fxy, axis, others=0.0):
    lo,hi=0.0,10.0
    for _ in range(60):
        mid=(lo+hi)/2
        M=[others]*3; M[axis]=mid
        ok=all(feasible((fxy*np.cos(t),fxy*np.sin(t),fz),tuple(M)) for t in np.linspace(0,2*np.pi,17))
        lo,hi=(mid,hi) if ok else (lo,mid)
    return lo

print("Single-axis moment authority (N*m) vs horizontal force budget, Fz=17.36")
print(f"{'|Fxy|':>6} | {'Mx':>7} | {'My':>7} | {'Mz':>7}")
for fxy in [0.0,0.5,1.0,1.5,2.0,3.0,4.0,5.0]:
    print(f"{fxy:6.1f} | " + " | ".join(f"{maxmom(17.36,fxy,a):7.3f}" for a in (0,1,2)))

print()
print("Simultaneous all-three (equal fraction of single-axis), Fz=17.36:")
for fxy in [0.0,0.5,1.0,1.5,2.0]:
    lo,hi=0.0,1.0
    for _ in range(50):
        mid=(lo+hi)/2
        M=(mid*maxmom(17.36,fxy,0), mid*maxmom(17.36,fxy,1), mid*maxmom(17.36,fxy,2))
        ok=all(feasible((fxy*np.cos(t),fxy*np.sin(t),17.36),M) for t in np.linspace(0,2*np.pi,17))
        lo,hi=(mid,hi) if ok else (lo,mid)
    M=(lo*maxmom(17.36,fxy,0), lo*maxmom(17.36,fxy,1), lo*maxmom(17.36,fxy,2))
    print(f"  |Fxy|={fxy:4.1f} -> Mx={M[0]:6.3f} My={M[1]:6.3f} Mz={M[2]:6.3f}  (frac {lo:.2f})")

print()
print("Angular accel available (rad/s^2) at |Fxy|=1.0, Fz=17.36:")
I=[0.06447,0.00309,0.06494]
for a,n in enumerate("xyz"):
    m=maxmom(17.36,1.0,a); print(f"  M{n}={m:6.3f} -> {m/I[a]:8.1f} rad/s^2")
