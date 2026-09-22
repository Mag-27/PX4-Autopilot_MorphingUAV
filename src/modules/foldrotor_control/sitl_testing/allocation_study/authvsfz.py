"""Moment authority is NOT a constant -- it depends on how much of the [0,15] N
per-rotor budget the collective Fz has already spent. kRateM*Limit are constants
sized at hover Fz. Bisect the real allocator for the largest deliverable |M| at
each Fz, the same way RateLimitsDoNotExceedHoverMomentAuthority does at hover."""
import subprocess, numpy as np
M0 = np.array([[float(x) for x in l.split()]
               for l in subprocess.run(['./drv','matrix'],capture_output=True,text=True)
               .stdout.strip().split('\n')[:6]])
def tvec(F,a,b): return np.array([F*np.sin(b), -F*np.cos(b)*np.sin(a), F*np.cos(b)*np.cos(a)])
def deliver(batch):
    out = subprocess.run(['./drv'], input='\n'.join(' '.join('%g'%v for v in w) for w in batch),
                         capture_output=True, text=True).stdout.strip().split('\n')
    res=[]
    for l in out:
        F1,F2,a1,a2,b1,b2,sat=[float(x) for x in l.split()]
        res.append(M0 @ np.concatenate([tvec(F1,a1,b1), tvec(F2,a2,b2)]))
    return res
def authority(fz, axis, tol=1e-3):
    lo, hi = 0.0, 12.0
    for _ in range(40):
        mid=(lo+hi)/2
        M=np.zeros(3); M[axis]=mid
        got=deliver([[0,0,fz,*M]])[0]
        # deliverable if the achieved moment tracks the request on that axis
        if abs(got[3+axis]-mid) < max(tol, 0.02*mid) and abs(got[2]-fz) < 0.02*max(fz,1): lo=mid
        else: hi=mid
    return lo
LIM=(3.8,0.30,3.8)
print("Fz cmd |  hover x  |  Mx auth  Mx lim  |  My auth  My lim  |  Mz auth  Mz lim")
for fz in (15.26, 18.0, 20.0, 22.26, 25.0, 26.83):
    a=[authority(fz,i) for i in range(3)]
    flag=lambda v,l: '  OVER' if l>v else '   ok '
    print("%6.2f | %8.2f  | %8.3f %6.2f%s| %8.3f %6.2f%s| %8.3f %6.2f%s"
          % (fz, fz/15.26, a[0],LIM[0],flag(a[0],LIM[0]), a[1],LIM[1],flag(a[1],LIM[1]), a[2],LIM[2],flag(a[2],LIM[2])))
