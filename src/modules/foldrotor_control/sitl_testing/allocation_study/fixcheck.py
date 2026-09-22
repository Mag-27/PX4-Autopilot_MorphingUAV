from study import *
zc = -0.0549   # rotor z rel true CoM (FLU)
KD = KD_SDF
# X: keep header M0 structure/signs verbatim (Simulink: rotor1 on +Y, drag +k), true FLU z and k; allocator rotor1 = PHYSICAL Motor2/Arm2, rotor2 = Motor1/Arm1
MX = M0(+S_Y, zc, -S_Y, zc, +KD)
# Y: keep channel mapping (rotor1 = Motor1 on -Y), flip y and drag sign in M0
MY = M0(-S_Y, zc, +S_Y, zc, -KD)
def phys_X(o):   # swap outputs onto physical motors: alloc rotor1 -> Motor2, rotor2 -> Motor1
    return truth([o[1], o[0], o[3], o[2], o[5], o[4]])
def phys_Y(o): return truth(o)
base = {n: f(alloc(np.array([0,0,G_W,0,0,0]))) for n, (alloc, f) in {"X": (lambda w: alloc_py(MX, w), phys_X), "Y": (lambda w: alloc_py(MY, w), phys_Y)}.items()}
rows = []
for nm, M, f in (("X: keep M0 signs, true geometry, relabel rotors (rotor1=Motor2)", MX, phys_X), ("Y: keep channel map, flip y & drag sign in M0", MY, phys_Y)):
    b = f(alloc_py(M, [0,0,G_W,0,0,0])); r = []
    for idx, v in ((3,.05),(4,.02),(5,.05)):
        w = [0,0,G_W,0,0,0]; w[idx] = v; r.append((f(alloc_py(M, w))[idx]-b[idx])/v)
    # forces incl. lateral/forward with M=0
    fr = []
    for w in ([2,0,G_W,0,0,0], [0,2,G_W,0,0,0]):
        t = f(alloc_py(M, w)); fr.append(t)
    rows.append([nm, *[f"{x:+.3f}" for x in r], f"Fx=2 -> Fx {fr[0][0]:+.2f}, My {fr[0][4]:+.3f}", f"Fy=2 -> Fy {fr[1][1]:+.2f}, Mx {fr[1][3]:+.3f}"])
tbl(["candidate", "Mx phys/dem", "My", "Mz", "force test 1", "force test 2"], rows)
# authority at hover for X
for nm, idx in (("Mx",3),("My",4),("Mz",5)): print(nm, "hover authority X:", round(envelope_ := (lambda M, idx: (lambda: None))(0,0) or 0,3)) if False else None
def env(M, idx):
    lo, hi = 0.0, 20.0
    for _ in range(60):
        mid=(lo+hi)/2; w=[0,0,G_W,0,0,0]; w[idx]=mid
        if alloc_py(M,w)[6]==0: lo=mid
        else: hi=mid
    return lo
print("hover authority (unclamped), candidate X/Y (identical geometry):", [round(env(MX,i),3) for i in (3,4,5)], [round(env(MY,i),3) for i in (3,4,5)])
print("MX row-check: max diff vs C-model:", np.abs(MX - M0(p1:=0,0,0,0,0)).max() if False else "")
