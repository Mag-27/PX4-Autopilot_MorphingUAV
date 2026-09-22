"""Forward kinematics of foldrotor3 from model.sdf (independent physical oracle for the allocator)."""
import numpy as np, xml.etree.ElementTree as ET
SDF = "/home/magesvarlinux/PX4-Autopilot/Tools/simulation/gz/models/foldrotor3/model.sdf"

def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]]); Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]]); Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return Rz @ Ry @ Rx
def T(pose):
    v = [float(x) for x in pose.split()]
    M = np.eye(4); M[:3,:3] = rpy(*v[3:]); M[:3,3] = v[:3]; return M
def rot(axis, q):
    a = np.array(axis, float); a /= np.linalg.norm(a); K = np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    M = np.eye(4); M[:3,:3] = np.eye(3) + np.sin(q)*K + (1-np.cos(q))*K@K; return M

class Model:
    def __init__(self):
        m = ET.parse(SDF).getroot().find("model")
        self.links = {l.get("name"): l for l in m.findall("link")}
        self.joints = {j.get("name"): j for j in m.findall("joint")}
        self.child2joint = {j.find("child").text: j.get("name") for j in self.joints.values()}
        self.plugins = m.findall("plugin")
    def link_frame(self, name, q):
        if name == "base_link": return np.eye(4)
        j = self.joints[self.child2joint[name]]
        pf = self.link_frame(j.find("parent").text, q)
        jf = pf @ T(j.find("pose").text)
        ax = j.find("axis/xyz").text.split() if j.find("axis") is not None else None
        Rj = rot([float(x) for x in ax], q.get(j.get("name"), 0.0)) if (ax and j.get("type") == "revolute") else np.eye(4)
        lp = self.links[name].find("pose")
        return jf @ Rj @ (T(lp.text) if lp is not None else np.eye(4))
    def com(self, q):
        M = 0; c = np.zeros(3)
        for n, l in self.links.items():
            i = l.find("inertial"); m = float(i.find("mass").text); ip = i.find("pose")
            p = (self.link_frame(n, q) @ (T(ip.text) if ip is not None else np.eye(4)))[:3,3]
            M += m; c += m*p
        return M, c/M
    def inertia(self, q, about):
        I = np.zeros((3,3))
        for n, l in self.links.items():
            i = l.find("inertial"); m = float(i.find("mass").text); ip = i.find("pose"); Ti = self.link_frame(n, q) @ (T(ip.text) if ip is not None else np.eye(4))
            g = lambda t: float(i.find("inertia/"+t).text) if i.find("inertia/"+t) is not None else 0.0
            Il = np.array([[g("ixx"),g("ixy"),g("ixz")],[g("ixy"),g("iyy"),g("iyz")],[g("ixz"),g("iyz"),g("izz")]])
            R = Ti[:3,:3]; d = Ti[:3,3] - about
            I += R@Il@R.T + m*((d@d)*np.eye(3) - np.outer(d,d))
        return I
    def rotor(self, k, q):
        """position and thrust axis (unit, +z of Prop{k}Link) in base_link frame."""
        Tf = self.link_frame(f"Prop{k}Link", q); return Tf[:3,3], Tf[:3,2]

if __name__ == "__main__":
    m = Model(); np.set_printoptions(precision=5, suppress=True)
    M, c = m.com({}); print("mass", M, "CoM(base_link, zero angles)", c)
    for k in (1, 2):
        p, a = m.rotor(k, {}); print(f"rotor{k}: pos rel CoM {p-c}  thrust axis {a}")
    print("Inertia about CoM (base_link axes):\n", m.inertia({}, c))
