#!/usr/bin/env python3
"""
RAS Build Challenge - Robodog in MuJoCo, built from the REAL CAD (robotdog.stl)
===============================================================================
The robot starts exactly as in the CAD file (legs folded, belly on the ground),
UNFOLDS, STANDS UP, walks, turns, then does a JUMP and lands.

Needs:   pip install mujoco numpy trimesh networkx scipy matplotlib pillow imageio imageio-ffmpeg
Run:     python mujoco_dog_cad.py --stl robotdog.stl          # saves video + plots in ./out_cad_real
         python mujoco_dog_cad.py --stl robotdog.stl --view   # live window
         python mujoco_dog_cad.py --stl robotdog.stl --servo-speed 9   # try a faster servo

How the CAD is used
  1. The STL holds 14 separate parts: 2 body plates + (hip bracket, thigh, shank) x 4 legs.
  2. The joint centres were measured on the STL (rings of the hip and knee axes, tip of the shank).
     They are in CAD_LEG below, in millimetres, in the STL frame.
  3. Every part is moved into the frame of its joint, so that "all joint angles = 0" means
     "leg straight down". The CAD pose (folded legs) is then simply joint angles q_fold.
  4. Physics uses simple invisible shapes carrying the masses; the STL meshes are for looks.

PHYSICS HONESTY: masses are ASSUMED (total ~2.5 kg, split body/legs is a guess).
The servo model includes the speed limit (torque falls to 0 at the no-load speed), which is
what limits a jump.  Servo speed 6.5 rad/s (0.16 s/60 deg) is an ASSUMPTION - check the datasheet.
"""
import argparse
import math
import os

import numpy as np
import mujoco
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ==========================================================================
# PART 1 - PARAMETERS
# ==========================================================================
# Joint centres measured on the STL [mm, STL frame]: hip pitch axis, knee axis, tip of the shank blade.
CAD_LEG = {
    "RR": dict(hip=(285.5, 59.0), knee=(373.0, 109.5), tip=(257.7, 174.1)),
    "RL": dict(hip=(285.5, 59.0), knee=(373.0, 109.5), tip=(269.3, 191.3)),
    "FR": dict(hip=(657.5, 59.0), knee=(728.5, 130.5), tip=(600.6, 162.8)),
    "FL": dict(hip=(657.5, 59.0), knee=(734.5, 124.0), tip=(609.8, 167.3)),
}
BASE_Z_MM = 30.0                   # base frame origin height inside the body plates (STL z)

# Masses [kg] - ASSUMPTION (total ~2.5 kg as announced by the CAD team)
M_BODY, M_ROLL, M_THIGH, M_SHANK, M_FOOT = 1.062, 0.112, 0.180, 0.056, 0.011
FOOT_R = 0.012

# DS3218MG servo
TAU_S = 20 * 0.0980665             # stall torque 20 kg.cm = 1.96 N.m
OMEGA0 = 6.5                       # no-load speed [rad/s] (0.16 s / 60 deg)  ASSUMPTION
STALL_A, IDLE_A = 2.6, 0.02        # electrical study
KP, KD = 20.0, 0.5
ARMATURE = 0.004

# Gait
Z0 = -0.17                         # standing foot height under the hip [m]
Y0 = 0.015
STRIDE, TURN_STRIDE = 0.07, 0.05
STEP_H, PERIOD, DUTY = 0.025, 0.6, 0.7
K_ROLL, K_PITCH = 0.10, 0.10       # IMU feedback

# Jump (foot height relative to the hip, metres)
Z_CROUCH, Z_PUSH_END, Z_TUCK, Z_LAND = -0.130, -0.225, -0.150, -0.125
T_CROUCH, T_PUSH, T_AIR, T_LAND = 0.7, 0.10, 0.30, 0.45
X_REST = 0.045                     # feet spread fore/aft while lying (knee cannot fold tighter than ~155 deg)
Z_REST = -0.046                    # feet on the ground while the belly still rests on it

SIM_DT, CTRL_DT = 0.002, 0.02
LEGS = ["FL", "FR", "RL", "RR"]
JOINTS = [f"{l}_{j}" for l in LEGS for j in ("roll", "pitch", "knee")]

# These are filled by build_model_data():
L1 = L2 = 0.0
HIP = {}
HIP_Z = 0.0
Q_FOLD = {}


# ==========================================================================
# PART 2 - CAD -> MuJoCo model
# ==========================================================================
def rot_y(V, a):
    """Rotate points (N,3) about the +y axis by angle a (same convention as a MuJoCo hinge on y)."""
    c, s = math.cos(a), math.sin(a)
    out = V.copy()
    out[:, 0] = V[:, 0] * c + V[:, 2] * s
    out[:, 2] = -V[:, 0] * s + V[:, 2] * c
    return out


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def stl_bytes(V, F):
    return trimesh.Trimesh(vertices=V, faces=F, process=False).export(file_type="stl")


def build_model_data(stl_path):
    """Returns (xml, assets). Also sets the globals L1, L2, HIP, HIP_Z, Q_FOLD."""
    global L1, L2, HIP, HIP_Z, Q_FOLD
    m = trimesh.load(stl_path)
    parts = sorted(m.split(only_watertight=False), key=lambda p: -len(p.faces))
    body = trimesh.util.concatenate(parts[:2])
    bx, by = body.bounds.mean(axis=0)[:2]
    xc = np.mean([v["hip"][0] for v in CAD_LEG.values()])
    yc = by
    C = np.array([xc, yc, BASE_Z_MM])

    legparts = {l: {} for l in LEGS}
    for p in parts[2:]:
        cx, cy = p.centroid[:2]
        leg = ("F" if cx > bx else "R") + ("R" if cy < by else "L")
        nf = len(p.faces)
        typ = "thigh" if nf == 1424 else "bracket" if nf == 1304 else "shank"
        legparts[leg][typ] = p

    assets = {"body.stl": stl_bytes((body.vertices - C) * 1e-3, body.faces)}
    L1s, L2s, info = [], [], {}
    for l in LEGS:
        d, P = CAD_LEG[l], legparts[l]
        yp = 0.5 * (P["thigh"].bounds[0][1] + P["thigh"].bounds[1][1])      # plane of the leg
        h = np.array([d["hip"][0], yp, d["hip"][1]])
        k = np.array([d["knee"][0], yp, d["knee"][1]])
        v = np.array([d["knee"][0] - d["hip"][0], d["knee"][1] - d["hip"][1]])
        s = np.array([d["tip"][0] - d["knee"][0], d["tip"][1] - d["knee"][1]])
        L1s.append(np.hypot(*v)); L2s.append(np.hypot(*s))
        at, as_ = math.atan2(v[0], -v[1]), math.atan2(s[0], -s[1])
        Q_FOLD[l] = (0.0, wrap(-at), wrap(at - as_))            # roll, hip pitch, knee in the CAD pose
        assets[f"{l}_brk.stl"] = stl_bytes((P["bracket"].vertices - h) * 1e-3, P["bracket"].faces)
        assets[f"{l}_thi.stl"] = stl_bytes(rot_y((P["thigh"].vertices - h) * 1e-3, at), P["thigh"].faces)
        assets[f"{l}_sha.stl"] = stl_bytes(rot_y((P["shank"].vertices - k) * 1e-3, as_), P["shank"].faces)
        info[l] = ((h - C) * 1e-3)
        HIP[l] = (info[l][0], info[l][1])
    HIP_Z = float(np.mean([info[l][2] for l in LEGS]))
    L1, L2 = float(np.mean(L1s)) * 1e-3, float(np.mean(L2s)) * 1e-3
    bsize = (body.bounds[1] - body.bounds[0]) * 1e-3 / 2

    def leg(l):
        hx, hy = HIP[l]
        return f"""
      <body name="{l}_roll_link" pos="{hx:.5f} {hy:.5f} {HIP_Z:.5f}">
        <joint name="{l}_roll" type="hinge" axis="1 0 0" range="-0.6 0.6"/>
        <geom type="mesh" mesh="{l}_brk" material="pla" contype="0" conaffinity="0" mass="1e-6"/>
        <geom type="sphere" size="0.012" mass="{M_ROLL}" group="3" contype="0" conaffinity="0"/>
        <body name="{l}_thigh">
          <joint name="{l}_pitch" type="hinge" axis="0 1 0" range="-2.5 1.5"/>
          <geom type="mesh" mesh="{l}_thi" material="pla" contype="0" conaffinity="0" mass="1e-6"/>
          <geom type="capsule" fromto="0 0 0 0 0 {-L1:.5f}" size="0.01" mass="{M_THIGH}" group="3" contype="0" conaffinity="0"/>
          <body name="{l}_shank" pos="0 0 {-L1:.5f}">
            <joint name="{l}_knee" type="hinge" axis="0 1 0" range="-2.7 0"/>
            <geom type="mesh" mesh="{l}_sha" material="pla" contype="0" conaffinity="0" mass="1e-6"/>
            <geom type="capsule" fromto="0 0 0 0 0 {-L2:.5f}" size="0.008" mass="{M_SHANK}" group="3" contype="0" conaffinity="0"/>
            <geom name="{l}_foot" type="sphere" pos="0 0 {-L2:.5f}" size="{FOOT_R}" mass="{M_FOOT}"
                  material="rubber" contype="0" conaffinity="1" friction="1.0 0.02 0.001"/>
          </body>
        </body>
      </body>"""

    meshes = "\n    ".join(
        [f'<mesh name="body" file="body.stl" inertia="shell"/>'] +
        [f'<mesh name="{l}_{t}" file="{l}_{t[:3] if t!="brk" else "brk"}.stl" inertia="shell"/>'
         for l in LEGS for t in ("brk", "thi", "sha")])
    motors = "".join(f'<motor name="{n}" joint="{n}" ctrlrange="-3 3"/>' for n in JOINTS)
    z0 = -min(0.0, 0) + (BASE_Z_MM - 1.0) * 1e-3 + 0.0005
    xml = f"""
<mujoco model="robodog_cad">
  <compiler angle="radian"/>
  <option timestep="{SIM_DT}" gravity="0 0 -9.81" integrator="implicitfast"/>
  <default><joint armature="{ARMATURE}" damping="0.02"/></default>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <quality shadowsize="4096"/>
    <headlight ambient="0.38 0.38 0.40" diffuse="0.45 0.45 0.45" specular="0.1 0.1 0.1"/>
    <map znear="0.01"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.62 0.75 0.92" rgb2="0.10 0.14 0.22" width="512" height="512"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.80 0.82 0.86" rgb2="0.70 0.72 0.77" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="10 10" reflectance="0.15"/>
    <material name="pla" rgba="0.93 0.69 0.08 1" specular="0.55" shininess="0.55"/>
    <material name="black" rgba="0.07 0.07 0.08 1" specular="0.4" shininess="0.5"/>
    <material name="rubber" rgba="0.04 0.04 0.04 1" specular="0.1"/>
    {meshes}
  </asset>
  <worldbody>
    <light pos="1.2 -1.2 2.5" dir="-0.45 0.45 -1" diffuse="0.85 0.85 0.8" specular="0.3 0.3 0.3" castshadow="true"/>
    <geom name="floor" type="plane" size="8 8 0.1" material="grid" contype="1" conaffinity="0" friction="1.0 0.02 0.001"/>
    <body name="base" pos="0 0 {z0:.4f}">
      <freejoint name="root"/>
      <geom type="mesh" mesh="body" material="black" contype="0" conaffinity="0" mass="1e-6"/>
      <geom type="box" size="{bsize[0]:.4f} {bsize[1]:.4f} {bsize[2]:.4f}" mass="{M_BODY}" group="3" contype="0" conaffinity="1"/>
      {''.join(leg(l) for l in LEGS)}
    </body>
  </worldbody>
  <actuator>{motors}</actuator>
</mujoco>"""
    return xml, assets


# ==========================================================================
# PART 3 - KINEMATICS + GAIT + SERVO
# ==========================================================================
def leg_ik(foot):
    """foot (x,y,z) relative to the hip -> (roll, hip pitch, knee) [rad]; knee bends backwards."""
    x, y, z = foot
    q0 = math.atan2(y, -z)
    zp = -math.hypot(y, z)
    d = math.hypot(x, zp)
    d = min(max(d, abs(L1 - L2) + 1e-3), L1 + L2 - 1e-3)
    c = (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    q2 = -math.acos(max(-1.0, min(1.0, c)))
    q1 = math.atan2(-x, -zp) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    return q0, q1, q2


def smooth(u):
    u = min(max(u, 0.0), 1.0)
    return 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5


class Gait:
    OFFSET = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}

    def __init__(self):
        self.phase, self.vx, self.wz = 0.0, 0.0, 0.0

    def targets(self, vx_cmd, wz_cmd, dt):
        a = dt / (0.3 + dt)
        self.vx += a * (vx_cmd - self.vx)
        self.wz += a * (wz_cmd - self.wz)
        s_f, s_t = STRIDE * self.vx, TURN_STRIDE * self.wz
        active = min(1.0, (abs(s_f) + abs(s_t)) / 0.02)
        if active > 1e-3:
            self.phase = (self.phase + dt / PERIOD) % 1.0
        out = {}
        for l in LEGS:
            left = l.endswith("L")
            s = s_f - s_t if left else s_f + s_t
            ph = (self.phase + self.OFFSET[l]) % 1.0
            if ph < DUTY:
                x, z = s / 2 - s * ph / DUTY, 0.0
            else:
                u = (ph - DUTY) / (1 - DUTY)
                x = -s / 2 + s * (u - math.sin(2 * math.pi * u) / (2 * math.pi))
                z = STEP_H * math.sin(math.pi * u) * active
            out[l] = np.array([x, Y0 if left else -Y0, Z0 + z])
        return out


def servo_torque(q_des, q, qd):
    """Position servo with a torque limit AND a speed limit (torque -> 0 at no-load speed)."""
    tau = KP * (q_des - q) - KD * qd
    w = np.abs(qd)
    drive = TAU_S * np.clip(1.0 - w / OMEGA0, 0.0, 1.0)
    lim = np.where(tau * qd > 0, drive, TAU_S)
    return np.clip(tau, -lim, lim)


# Timeline: (label, duration [s], mode)
def build_timeline(n_jumps):
    tl = [("rest: folded legs (CAD pose)", 1.0, "rest"), ("unfold legs", 1.4, "unfold"),
          ("stand up", 2.0, "standup"), ("stand", 1.0, "stand"),
          ("walk forward", 4.0, "walk"), ("turn on the spot", 3.0, "turn"), ("pause", 0.8, "stand")]
    for i in range(n_jumps):
        n = f"jump {i + 1}: "
        tl += [(n + "crouch", T_CROUCH, "crouch"), (n + "push", T_PUSH, "push"), (n + "flight", T_AIR, "air"),
               (n + "landing", T_LAND, "land"), (n + "recover", 1.0, "recover")]
    tl.append(("stand (end)", 1.0, "stand"))
    return tl


def quat_to_rp(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class Brain:
    def __init__(self):
        self.gait = Gait()
        self.q_prev = np.concatenate([Q_FOLD[l] for l in LEGS])

    def foot_q(self, zcmd, roll, pitch, vx=0.0, wz=0.0, fb=True, xoff=0.0):
        tg = self.gait.targets(vx, wz, CTRL_DT)
        q = []
        for l in LEGS:
            p = tg[l].copy()
            p[2] += zcmd - Z0
            p[0] += xoff * np.sign(HIP[l][0])
            if fb:
                hx, hy = HIP[l]
                p[2] += K_ROLL * roll * np.sign(hy) - K_PITCH * pitch * np.sign(hx)
            q += list(leg_ik(p))
        return np.array(q)

    def act(self, mode, u, roll, pitch):
        q_fold = np.concatenate([Q_FOLD[l] for l in LEGS])
        q_rest = self.foot_q(Z_REST, 0, 0, fb=False, xoff=X_REST)
        if mode == "rest":
            q = q_fold
        elif mode == "unfold":
            q = q_fold + (q_rest - q_fold) * smooth(u)
        elif mode == "standup":
            q = self.foot_q(Z_REST + (Z0 - Z_REST) * smooth(u), roll, pitch, xoff=X_REST * (1 - smooth(u)))
        elif mode == "stand":
            q = self.foot_q(Z0, roll, pitch)
        elif mode == "walk":
            q = self.foot_q(Z0, roll, pitch, vx=1.0)
        elif mode == "turn":
            q = self.foot_q(Z0, roll, pitch, wz=1.0)
        elif mode == "crouch":
            q = self.foot_q(Z0 + (Z_CROUCH - Z0) * smooth(u), roll, pitch)
        elif mode == "push":
            q = self.foot_q(Z_CROUCH + (Z_PUSH_END - Z_CROUCH) * u, 0, 0, fb=False)
        elif mode == "air":
            q = self.foot_q(Z_PUSH_END + (Z_TUCK - Z_PUSH_END) * smooth(u * 2), 0, 0, fb=False)
        elif mode == "land":
            q = self.foot_q(Z_TUCK + (Z_LAND - Z_TUCK) * smooth(u * 3), 0, 0, fb=False)
        elif mode == "recover":
            q = self.foot_q(Z_LAND + (Z0 - Z_LAND) * smooth(u), roll, pitch)
        return q


# ==========================================================================
# PART 4 - SIMULATION
# ==========================================================================
def run(args):
    xml, assets = build_model_data(args.stl)
    model = mujoco.MjModel.from_xml_string(xml, assets)
    data = mujoco.MjData(model)
    jid = [model.joint(n).id for n in JOINTS]
    qadr = np.array([model.jnt_qposadr[j] for j in jid])
    vadr = np.array([model.jnt_dofadr[j] for j in jid])
    foot_ids = [model.geom(f"{l}_foot").id for l in LEGS]
    floor_id = model.geom("floor").id
    q_fold = np.concatenate([Q_FOLD[l] for l in LEGS])
    data.qpos[qadr] = q_fold
    mujoco.mj_forward(model, data)

    brain = Brain()
    timeline = build_timeline(args.jumps)
    steps = int(round(CTRL_DT / SIM_DT))
    rec = {k: [] for k in ("t", "ph", "pos", "vel", "rpy", "tau", "qd", "air", "clear", "air_s")}
    frames = []
    renderer = mujoco.Renderer(model, args.height, args.width) if args.video else None
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 0.82, -14, 135
    look = np.array([0.0, 0.0, 0.09])
    viewer = None
    if args.view:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)

    k = 0
    for label, dur, mode in timeline:
        n = int(round(dur / CTRL_DT))
        for i in range(n):
            roll, pitch, yaw = quat_to_rp(data.qpos[3:7])
            q_des = brain.act(mode, i / n, roll, pitch)
            air_s = 0.0
            for _ in range(steps):
                q, qd = data.qpos[qadr], data.qvel[vadr]
                data.ctrl[:] = servo_torque(q_des, q, qd)
                mujoco.mj_step(model, data)
                touching = False
                for c in data.contact[:data.ncon]:
                    if (c.geom1 in foot_ids and c.geom2 == floor_id) or (c.geom2 in foot_ids and c.geom1 == floor_id):
                        touching = True
                        break
                if not touching:
                    air_s += SIM_DT
            contacts = 0 if air_s >= CTRL_DT - 1e-9 else 1
            clear = min(data.geom_xpos[g][2] for g in foot_ids) - FOOT_R      # lowest foot tip above the ground
            rec["t"].append(data.time); rec["ph"].append(label)
            rec["pos"].append(data.qpos[:3].copy()); rec["vel"].append(data.qvel[:3].copy())
            rec["rpy"].append(quat_to_rp(data.qpos[3:7]))
            rec["tau"].append(data.ctrl.copy()); rec["qd"].append(data.qvel[vadr].copy())
            rec["air"].append(contacts == 0); rec["clear"].append(clear); rec["air_s"].append(air_s)
            k += 1
            if renderer is not None and k % 2 == 0:
                look += 0.15 * (data.qpos[:3] - look)
                cam.lookat[:] = look
                cam.azimuth = 135 + 12 * math.sin(data.time / 7.0)
                renderer.update_scene(data, cam)
                frames.append((renderer.render().copy(), label, data.time))
            if viewer is not None:
                viewer.sync()
    if viewer is not None:
        viewer.close()
    for key in ("t", "pos", "vel", "rpy", "tau", "qd", "air", "clear", "air_s"):
        rec[key] = np.array(rec[key])
    rec["ph"] = np.array(rec["ph"])
    return rec, frames


# ==========================================================================
# PART 5 - ANALYSIS, PLOTS, VIDEO
# ==========================================================================
def analyse(rec, out):
    ph, tau, t = rec["ph"], np.abs(rec["tau"]), rec["t"]
    cur = IDLE_A + tau / TAU_S * (STALL_A - IDLE_A)
    tot = cur.sum(1)
    L = [f"Total mass {M_BODY + 4 * (M_ROLL + M_THIGH + M_SHANK + M_FOOT):.2f} kg (assumed split) | L1={L1 * 1e3:.0f} mm  L2={L2 * 1e3:.0f} mm"
         f" | servo {TAU_S:.2f} N.m, no-load {OMEGA0} rad/s", "",
         f"{'phase':<30}{'time':>5}{'dist':>6}{'dYaw':>6}{'maxRoll':>8}{'maxPit':>7}{'z end':>7}{'p99 tau':>8}{'avg I':>7}"]
    for n in dict.fromkeys(ph):
        m = ph == n
        pos, rpy = rec["pos"][m], np.degrees(rec["rpy"][m])
        yu = np.degrees(np.unwrap(np.radians(rpy[:, 2])))
        L.append(f"{n:<30}{t[m][0]:>5.1f}{np.hypot(*(pos[-1, :2] - pos[0, :2])):>6.2f}{yu[-1] - yu[0]:>6.0f}"
                 f"{np.abs(rpy[:, 0]).max():>8.1f}{np.abs(rpy[:, 1]).max():>7.1f}{pos[-1, 2]:>7.3f}"
                 f"{np.percentile(tau[m], 99):>8.2f}{tot[m].mean():>7.1f}")
    L.append("")
    L.append("Torque, 99th percentile over the whole run (limit %.2f N.m):" % TAU_S)
    for jt in ("roll", "pitch", "knee"):
        c = [i for i, n in enumerate(JOINTS) if n.endswith(jt)]
        p99 = np.percentile(tau[:, c], 99)
        L.append(f"  {jt:<6} p99 {p99:.2f}  peak {tau[:, c].max():.2f}  margin(p99) x{TAU_S / p99:.2f}")
    walk = np.isin(ph, ["walk forward", "turn on the spot"])
    L.append(f"Estimated servo current while walking/turning: avg {tot[walk].mean():.1f} A, p99 {np.percentile(tot[walk], 99):.1f} A")
    # stand-up
    su = ph == "stand"
    L.append(f"Stand-up: body height {rec['pos'][ph == 'unfold legs'][-1, 2] * 1e3:.0f} mm after unfolding -> "
             f"{rec['pos'][su][0, 2] * 1e3:.0f} mm standing")
    # jumps (one block per jump)
    air = rec["air"]
    z_stand = np.median(rec["pos"][ph == "stand (end)", 2])
    for jn in sorted({n.split(":")[0] for n in ph if n.startswith("jump")}):
        idx = np.where(np.array([n.startswith(jn) and any(w in n for w in ("push", "flight", "landing")) for n in ph]))[0]
        flight = rec["air_s"][idx].sum()
        L.append(f"{jn.capitalize()}: time with ALL feet off the ground {flight * 1e3:.0f} ms | max foot clearance {1e3 * rec['clear'][idx].max():.0f} mm"
                 f" | max upward speed {rec['vel'][idx, 2].max():.2f} m/s | peak torque {tau[idx].max():.2f} N.m | peak current {tot[idx].max():.1f} A")
    txt = "\n".join(L)
    print(txt)
    open(os.path.join(out, "summary.txt"), "w").write(txt)
    return tot


def plots(rec, tot, out):
    t, ph = rec["t"], rec["ph"]
    names = list(dict.fromkeys(ph))
    starts = [t[np.where(ph == n)[0][0]] for n in names]
    fig, ax = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    ax[0].plot(t, rec["pos"][:, 2] * 1e3); ax[0].set_ylabel("body height [mm]")
    ax[0].fill_between(t, 0, rec["pos"][:, 2].max() * 1e3, where=rec["air_s"] > 0, color="orange", alpha=0.25, label="feet in the air (part of the step)")
    ax[0].legend(loc="upper left")
    for jt, c in (("roll", "C0"), ("pitch", "C1"), ("knee", "C2")):
        cols = [i for i, n in enumerate(JOINTS) if n.endswith(jt)]
        ax[1].plot(t, np.abs(rec["tau"][:, cols]).max(1), c, lw=0.7, label=jt)
    ax[1].axhline(TAU_S, color="r", ls="--", label="20 kg.cm limit"); ax[1].set_ylabel("max |torque| [N.m]"); ax[1].legend(ncol=4, loc="upper left")
    ax[2].plot(t, tot); ax[2].set_ylabel("servo current, est. [A]"); ax[2].set_xlabel("time [s]")
    for a in ax:
        for s_ in starts:
            a.axvline(s_, color="k", lw=0.3, alpha=0.4)
    for s_, n in zip(starts, names):
        ax[0].text(s_ + 0.05, ax[0].get_ylim()[1] * 0.97, n, rotation=90, va="top", fontsize=6)
    fig.tight_layout(); fig.savefig(os.path.join(out, "cad_run.png"), dpi=120); plt.close(fig)


def save_video(frames, out, name="robodog_cad.mp4", footer=""):
    from PIL import Image, ImageDraw, ImageFont
    import imageio
    try:
        font = ImageFont.truetype("DejaVuSans-Bold.ttf", 20)
    except Exception:
        font = ImageFont.load_default()
    path = os.path.join(out, name)
    w = imageio.get_writer(path, fps=25, codec="libx264", quality=8, macro_block_size=1)
    for img, label, tm in frames:
        im = Image.fromarray(img); d = ImageDraw.Draw(im)
        d.rectangle([0, 0, im.width, 34], fill=(0, 0, 0))
        d.text((10, 6), f"{label}", fill=(255, 205, 60), font=font)
        d.text((im.width - 110, 6), f"t = {tm:4.1f} s", fill=(255, 255, 255), font=font)
        d.rectangle([0, im.height - 26, im.width, im.height], fill=(0, 0, 0))
        d.text((10, im.height - 24), footer, fill=(200, 200, 200), font=ImageFont.load_default())
        w.append_data(np.array(im))
    w.close()
    return path


def main():
    global OMEGA0
    ap = argparse.ArgumentParser()
    ap.add_argument("--stl", default="robotdog.stl")
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--no-video", dest="video", action="store_false")
    ap.add_argument("--jumps", type=int, default=2)
    ap.add_argument("--servo-speed", type=float, default=OMEGA0, help="servo no-load speed [rad/s]")
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--height", type=int, default=540)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "out_cad_real"))
    args = ap.parse_args()
    OMEGA0 = args.servo_speed
    os.makedirs(args.out, exist_ok=True)
    rec, frames = run(args)
    tot = analyse(rec, args.out)
    plots(rec, tot, args.out)
    if frames:
        foot = (f"Robodog from CAD (robotdog.stl) | 2.5 kg | servo 20 kg.cm, no-load {OMEGA0:g} rad/s"
                + (" (what-if: faster servo)" if OMEGA0 > 7 else " (assumed DS3218MG)"))
        print("video:", save_video(frames, args.out, "robodog_cad.mp4" if OMEGA0 <= 7 else f"robodog_cad_servo{OMEGA0:g}.mp4", foot))


if __name__ == "__main__":
    main()
