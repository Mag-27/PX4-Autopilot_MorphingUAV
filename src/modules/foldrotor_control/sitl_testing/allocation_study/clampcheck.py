"""Replay logged commanded wrenches through the REAL allocator and forward-map
the (clamped) actuator solution back to the wrench actually delivered.
Answers: when the position loop's lateral demand rails the tilt servos, what
moment does the vehicle really get?"""
import subprocess, numpy as np, sys
from pyulog import ULog

LOG = sys.argv[1] if len(sys.argv) > 1 else \
    '/home/magesvarlinux/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-21/07_09_04.ulg'

M0 = np.array([[float(x) for x in l.split()]
               for l in subprocess.run(['./drv','matrix'],capture_output=True,text=True)
               .stdout.strip().split('\n')[:6]])

u = ULog(LOG)
aa = u.get_dataset('actuator_armed').data
t_arm = int(aa['timestamp'][np.argmax(aa['armed'] > 0)])
d = u.get_dataset('debug_array').data
t = d['timestamp'].astype(np.int64); ids = d['id']
m = (ids == 1) & (t >= t_arm - 20000) & (t < t_arm + 520000)
idx = np.where(m)[0]
W = np.array([[d['data[%d]' % k][i] for k in range(6)] for i in idx])

out = subprocess.run(['./drv'], input='\n'.join(' '.join('%g' % v for v in w) for w in W),
                     capture_output=True, text=True).stdout.strip().split('\n')

def tvec(F, a, b):
    return np.array([F*np.sin(b), -F*np.cos(b)*np.sin(a), F*np.cos(b)*np.cos(a)])

print(' t_ms | cmd Fx    Fz   My   | got Fx    Fz    My   | dMy    sat')
worst = (0, None)
for n, (i, line) in enumerate(zip(idx, out)):
    F1,F2,a1,a2,b1,b2,sat = [float(x) for x in line.split()]
    got = M0 @ np.concatenate([tvec(F1,a1,b1), tvec(F2,a2,b2)])
    w = W[n]; dMy = got[4] - w[4]
    if abs(dMy) > abs(worst[0]): worst = (dMy, ((t[i]-t_arm)/1000, w, got, sat))
    if n % max(1, len(idx)//18) == 0:
        print('%6.1f | %6.2f %6.2f %+5.2f | %6.2f %6.2f %+5.2f | %+6.2f  %d'
              % ((t[i]-t_arm)/1000, w[0], w[2], w[4], got[0], got[2], got[4], dMy, sat))

dMy_all = np.array([ (M0 @ np.concatenate([tvec(*[float(x) for x in l.split()][0:1]+[float(x) for x in l.split()][2:3]+[float(x) for x in l.split()][4:5]),
                                           tvec(*[float(x) for x in l.split()][1:2]+[float(x) for x in l.split()][3:4]+[float(x) for x in l.split()][5:6])]))[4] - W[n][4]
                     for n,l in enumerate(out)])
print('\nWorst pitch-moment error: %+.3f N.m at t=%.1f ms (sat=%d)' % (worst[0], worst[1][0], worst[1][3]))
print('  commanded wrench:', np.round(worst[1][1],3))
print('  delivered wrench:', np.round(worst[1][2],3))
print('\n|dMy| : mean %.3f  max %.3f  N.m   (kRateMyLimit = 0.30)' % (np.abs(dMy_all).mean(), np.abs(dMy_all).max()))
print('samples with |dMy| > kRateMyLimit: %d / %d (%.0f%%)'
      % ((np.abs(dMy_all)>0.30).sum(), len(dMy_all), 100*(np.abs(dMy_all)>0.30).mean()))
print('saturated fraction: %.0f%%' % (100*np.mean([float(l.split()[6]) for l in out])))
