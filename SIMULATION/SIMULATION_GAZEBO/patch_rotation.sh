#!/usr/bin/env bash
# Patch: vraie rotation sur place (pas + pivot des hanches), 90 deg puis marche, 180 deg puis marche inverse
set -e
WS="$HOME/robotdog_ws"
cat > "$WS/src/robotdog_sim/robotdog_sim/kin.py" <<'__ROBODOG_EOF__'
"""Cinematique du robotdog lue directement dans l'URDF (aucune longueur devinee)."""
import os, re, struct
import numpy as np
from xml.etree import ElementTree as ET

LEGS = {  # nom: (hanche, femur, genou, lien tibia)
    'A': ('Revolute_2',  'Revolute_14', 'Revolute_4',  'TibiaR_f'),
    'B': ('Revolute_8',  'Revolute_9',  'Revolute_10', 'robotdog_TibiaL_f'),
    'C': ('Revolute_11', 'Revolute_12', 'Revolute_13', 'robotdog_TibiaL_f_2'),
    'D': ('Revolute_16', 'Revolute_17', 'Revolute_18', 'TibiaR'),
}
PAIRS = (('A', 'B'), ('C', 'D'))      # trot: paires diagonales
ALL_JOINTS = [j for l in LEGS.values() for j in l[:3]]


def _rpy(a):
    x, y, z = a
    cx, sx, cy, sy, cz, sz = np.cos(x), np.sin(x), np.cos(y), np.sin(y), np.cos(z), np.sin(z)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _T(xyz, rpy):
    M = np.eye(4); M[:3, :3] = _rpy(rpy); M[:3, 3] = xyz
    return M


def _vec(s):
    return [float(v) for v in s.split()]


def read_stl(path):
    data = open(path, 'rb').read()
    if len(data) >= 84:
        n = struct.unpack('<I', data[80:84])[0]
        if 84 + 50 * n == len(data):
            a = np.frombuffer(data[84:], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]), count=n)
            return a['v'].reshape(-1, 3).astype(float)
    txt = data.decode('ascii', 'ignore')
    return np.array([[float(x) for x in m] for m in re.findall(
        r'vertex\s+(\S+)\s+(\S+)\s+(\S+)', txt)])


class Robot:
    def __init__(self, urdf_path, pkg_share=None):
        self.root = ET.parse(urdf_path).getroot()
        self.pkg_share = pkg_share
        self.joints = {}
        for j in self.root.findall('joint'):
            o = j.find('origin'); ax = j.find('axis')
            self.joints[j.get('name')] = dict(
                parent=j.find('parent').get('link'), child=j.find('child').get('link'),
                T=_T(_vec(o.get('xyz')), _vec(o.get('rpy'))),
                axis=np.array(_vec(ax.get('xyz'))) if ax is not None else None)
        self.by_child = {j['child']: n for n, j in self.joints.items()}
        self.links = {l.get('name'): l for l in self.root.findall('link')}
        # lien "base_link" (repere CAD) = racine du bras cinematique
        self.base = 'base_link'
        self._tip = {}

    # ---------- FK ----------
    @staticmethod
    def _rot(axis, q):
        a = axis / np.linalg.norm(axis)
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        M = np.eye(4); M[:3, :3] = np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K
        return M

    def link_T(self, link, q):
        """Transformation base_link -> link."""
        chain = []
        while link != self.base:
            n = self.by_child[link]; chain.append(n); link = self.joints[n]['parent']
        M = np.eye(4)
        for n in reversed(chain):
            j = self.joints[n]; M = M @ j['T']
            if j['axis'] is not None:
                M = M @ self._rot(j['axis'], q.get(n, 0.0))
        return M

    # ---------- maillages de collision ----------
    def collision_vertices(self, link):
        """Sommets de collision d'un lien, exprimes dans le repere du lien (metres)."""
        c = self.links[link].find('collision')
        if c is None or self.pkg_share is None:
            return None
        m = c.find('geometry/mesh')
        o = c.find('origin')
        rel = m.get('filename').split('package://')[1].split('/', 1)[1]
        pkg = m.get('filename').split('package://')[1].split('/')[0]
        path = os.path.join(self.pkg_share[pkg], rel) if isinstance(self.pkg_share, dict) else os.path.join(self.pkg_share, rel)
        if not os.path.exists(path):
            return None
        v = read_stl(path) * np.array(_vec(m.get('scale', '1 1 1')))
        T = _T(_vec(o.get('xyz')), _vec(o.get('rpy'))) if o is not None else np.eye(4)
        return (T[:3, :3] @ v.T).T + T[:3, 3]

    def tip_local(self, leg):
        """Point du pied dans le repere du tibia: sommet le plus eloigne de l'axe du genou."""
        if leg in self._tip:
            return self._tip[leg]
        tl = LEGS[leg][3]
        v = self.collision_vertices(tl)
        if v is None:   # secours si les STL sont introuvables
            ci = self.links[tl].find('inertial/origin')
            c = np.array(_vec(ci.get('xyz'))); c[2] = 0
            tip = np.append(c / np.linalg.norm(c) * 0.13, 0.0)[:3]
            self._tip[leg] = (tip, False)
            return self._tip[leg]
        r = np.hypot(v[:, 0], v[:, 1])
        sel = v[r > r.max() - 0.006]
        tip = sel.mean(axis=0)
        self._tip[leg] = (tip, True)
        return self._tip[leg]

    def foot(self, leg, q):
        tip, _ = self.tip_local(leg)
        return (self.link_T(LEGS[leg][3], q) @ np.append(tip, 1.0))[:3]

    # ---------- orientation (quel sens est le bas ?) ----------
    def down_sign(self):
        """+1 si le bas du chien est +Z dans le repere CAD (pieds du cote +Z du corps)."""
        feet = np.mean([self.foot(l, {})[2] for l in LEGS])
        zs = []
        for ln in ('base_link', 'LowerSpine'):
            v = self.collision_vertices(ln)
            if v is not None:
                zs.append(np.mean(v[:, 2]))
        body = np.mean(zs) if zs else 0.03
        return 1.0 if feet > body else -1.0

    # ---------- IK (plan sagittal, hanche = 0) ----------
    def ik(self, leg, target_xz, q0=(0.0, 0.0), iters=40):
        h, f, k, _ = LEGS[leg]
        q = np.array(q0, float)
        for _ in range(iters):
            p = self.foot(leg, {f: q[0], k: q[1]})
            e = np.array([target_xz[0] - p[0], target_xz[1] - p[2]])
            if np.linalg.norm(e) < 1e-5:
                break
            J = np.zeros((2, 2))
            for i in range(2):
                dq = q.copy(); dq[i] += 1e-5
                pp = self.foot(leg, {f: dq[0], k: dq[1]})
                J[:, i] = (np.array([pp[0], pp[2]]) - np.array([p[0], p[2]])) / 1e-5
            step = np.linalg.solve(J.T @ J + 1e-4 * np.eye(2), J.T @ e)
            n = np.linalg.norm(step)
            if n > 0.15: step *= 0.15 / n
            q += step
        p = self.foot(leg, {f: q[0], k: q[1]})
        err = float(np.hypot(target_xz[0] - p[0], target_xz[1] - p[2]))
        return q, err

    def ik3(self, leg, target, q0, iters=30):
        """IK 3D (hanche + femur + genou) vers une cible (x, y, z) dans le repere base_link."""
        h, f, kn, _ = LEGS[leg]
        q = np.array(q0, float)
        tgt = np.array(target, float)
        def pos(qq):
            return self.foot(leg, {h: qq[0], f: qq[1], kn: qq[2]})
        for _ in range(iters):
            p = pos(q); e = tgt - p
            if np.linalg.norm(e) < 1e-5:
                break
            J = np.zeros((3, 3))
            for i in range(3):
                dq = q.copy(); dq[i] += 1e-5
                J[:, i] = (pos(dq) - p) / 1e-5
            step = np.linalg.solve(J.T @ J + 1e-4 * np.eye(3), J.T @ e)
            n = np.linalg.norm(step)
            if n > 0.15: step *= 0.15 / n
            q += step
        return q, float(np.linalg.norm(tgt - pos(q)))
__ROBODOG_EOF__
cat > "$WS/src/robotdog_sim/robotdog_sim/gait.py" <<'__ROBODOG_EOF__'
"""Planificateur: repos -> debout -> marche avant -> marche arriere -> repos."""
import numpy as np
from .kin import Robot, LEGS, PAIRS, ALL_JOINTS

# ---------------- parametres a modifier librement ----------------
STAND_DEPTH = 0.19    # m, distance base_link -> sol (hauteur debout)
REST_DEPTH  = 0.10    # m, position repliee (couche)
STRIDE      = 0.07    # m, longueur de pas
STEP_H      = 0.035   # m, hauteur de levee du pied
PERIOD      = 0.80    # s, duree d'un cycle de trot
DUTY        = 0.62    # part du cycle ou le pied est au sol (>0.5 = recouvrement)
TURN_STRIDE = 0.07    # m, amplitude des pas (avant/arriere) pendant la rotation sur place
TURN_LAT    = 0.10    # m, amplitude laterale (pivot des hanches) pendant la rotation
TURN_90_CYCLES = 4.0  # cycles de pas pour tourner de 90 deg (<<< SEUL REGLAGE A AJUSTER >>>)
                      #   robot tourne trop peu -> augmente / trop -> diminue
TURN_DIR    = +1      # +1 ou -1 : sens de rotation
WALK_TIME   = 8.0     # s, duree de chaque marche
# -------------------------------------------------------------------


TURN_SIGN = {'A': +1, 'D': +1, 'B': -1, 'C': -1}   # pas avant/arriere: cotes opposes
LAT_SIGN  = {'A': +1, 'C': +1, 'B': -1, 'D': -1}   # pas lateraux: avant / arriere opposes


def smooth(t):
    t = np.clip(t, 0.0, 1.0)
    return 0.5 - 0.5 * np.cos(np.pi * t)


class Planner:
    def __init__(self, robot: Robot):
        self.R = robot
        self.nom = {}            # position nominale (x) du pied par patte
        self.nomy = {}
        for l in LEGS:
            p0 = robot.foot(l, {})
            self.nom[l], self.nomy[l] = p0[0], p0[1]
        self.q_stand, self.q_rest = {}, {}
        for l, (h, f, k, _) in LEGS.items():
            qs, es = robot.ik(l, (self.nom[l], STAND_DEPTH))
            qr, er = robot.ik(l, (self.nom[l], REST_DEPTH), q0=qs)
            self.q_stand[l], self.q_rest[l] = qs, qr
            assert es < 2e-3 and er < 2e-3, 'IK impossible pour la patte %s (%.4f, %.4f)' % (l, es, er)
        self._cache = {}

    # --- conversions ---
    def pose_dict(self, qlegs):
        d = {j: 0.0 for j in ALL_JOINTS}
        for l, (h, f, k, _) in LEGS.items():
            q = qlegs[l]
            if len(q) == 3:
                d[h], d[f], d[k] = float(q[0]), float(q[1]), float(q[2])
            else:
                d[f], d[k] = float(q[0]), float(q[1])
        return d

    def rest_pose(self):  return self.pose_dict(self.q_rest)
    def stand_pose(self): return self.pose_dict(self.q_stand)

    def blend(self, qa, qb, s):
        return {l: (1 - s) * qa[l] + s * qb[l] for l in LEGS}

    # --- marche ---
    def foot_target(self, leg, phase, cmd, lift=1.0):
        """phase dans [0,1): appui puis oscillation. cmd=(pas_x, pas_y) signes."""
        sx, sy = cmd
        x0, y0 = self.nom[leg], self.nomy[leg]
        if phase < DUTY:                       # appui: le pied recule par rapport au corps
            s = phase / DUTY
            f = 0.5 - s
            z = STAND_DEPTH
        else:                                  # oscillation
            s = (phase - DUTY) / (1 - DUTY)
            f = -0.5 + (s - np.sin(2 * np.pi * s) / (2 * np.pi))
            z = STAND_DEPTH - lift * STEP_H * np.sin(np.pi * s)
        return x0 + sx * f, y0 + sy * f, z

    def walk_pose(self, t, cmds, seed, lift=1.0):
        qs = {}
        for l in LEGS:
            ph = (t / PERIOD + (0.5 if l in PAIRS[1] else 0.0)) % 1.0
            x, y, z = self.foot_target(l, ph, cmds[l], lift)
            q, err = self.R.ik3(l, (x, y, z), seed[l], iters=25)
            qs[l] = q
        return qs

    # --- timeline complete ---
    def build(self, dt=0.02):
        """Retourne la liste (duree_totale, [(t, dict_articulations), ...], [(t, nom_phase), ...])."""
        seq = []; marks = []; t = 0.0
        cur = {l: self.q_rest[l].copy() for l in LEGS}

        def add(q, name=None):
            nonlocal t
            if name: marks.append((t, name))
            seq.append((t, self.pose_dict(q))); t += dt

        def hold(q, dur, name=None):
            n = max(1, int(dur / dt))
            for i in range(n):
                add(q, name if i == 0 else None)

        def move(qa, qb, dur, name=None):
            n = max(1, int(dur / dt))
            for i in range(n):
                add(self.blend(qa, qb, smooth((i + 1) / n)), name if i == 0 else None)
            return qb

        def walk(cmd_of, dur, name):
            n = int(dur / dt)
            seed = {l: np.array([0.0, self.q_stand[l][0], self.q_stand[l][1]]) for l in LEGS}
            ramp = 1.0 * PERIOD
            for i in range(n):
                tt = i * dt
                amp = smooth(tt / ramp) * smooth((dur - tt) / ramp)   # montee/descente douce
                q = self.walk_pose(tt, {l: (cmd_of[l][0] * amp, cmd_of[l][1] * amp) for l in LEGS},
                                   seed, min(1.0, 4.0 * amp))
                seed = q
                add(q, name if i == 0 else None)

        fwd = {l: (STRIDE, 0.0) for l in LEGS}
        turn = {l: (TURN_DIR * TURN_SIGN[l] * TURN_STRIDE, TURN_DIR * LAT_SIGN[l] * TURN_LAT) for l in LEGS}
        t90 = TURN_90_CYCLES * PERIOD + PERIOD       # + une periode de montee/descente
        t180 = 2 * TURN_90_CYCLES * PERIOD + PERIOD

        hold(self.q_rest, 4.0, 'REPOS')
        move(self.q_rest, self.q_stand, 3.5, 'LEVER')
        hold(self.q_stand, 1.5, 'DEBOUT')
        walk(fwd, WALK_TIME, 'MARCHE AVANT')
        hold(self.q_stand, 1.0, 'DEBOUT')
        walk(turn, t90, 'ROTATION 90 DEG')
        hold(self.q_stand, 1.0, 'DEBOUT')
        walk(fwd, WALK_TIME, 'MARCHE (NOUVELLE DIRECTION)')
        hold(self.q_stand, 1.0, 'DEBOUT')
        walk(turn, t180, 'ROTATION 180 DEG')
        hold(self.q_stand, 1.0, 'DEBOUT')
        walk(fwd, WALK_TIME, 'MARCHE (DIRECTION OPPOSEE)')
        hold(self.q_stand, 1.5, 'DEBOUT')
        move(self.q_stand, self.q_rest, 3.5, 'RETOUR AU REPOS')
        hold(self.q_rest, 2.0, None)
        return t, seq, marks

    def lowest_depth(self, q):
        """Distance entre base_link et le point de collision le plus bas, pour une pose donnee."""
        sgn = self.R.down_sign()
        zmax = max(sgn * self.R.foot(l, q)[2] for l in LEGS)   # au minimum les pieds
        for ln in self.R.links:
            v = self.R.collision_vertices(ln)
            if v is None:
                continue
            T = self.R.link_T(ln, q)
            w = (T[:3, :3] @ v.T).T + T[:3, 3]
            zmax = max(zmax, (sgn * w[:, 2]).max())
        return zmax
__ROBODOG_EOF__
echo "Patch applique."
