#!/usr/bin/env python3
"""
RAS Build Challenge - Robodog "brain" in MuJoCo (Level 1)
=========================================================
Read this file top to bottom, it is ordered from simple to advanced:

  PART 1  PARAMS           numbers you change (replace with CAD values!)
  PART 2  build_mjcf()     builds the robot model (12 joints, 4 legs)
  PART 3  leg_ik()         inverse kinematics: foot position -> joint angles
  PART 4  Gait             trot gait generator (the "open-loop brain")
  PART 5  Brain            gait + IMU feedback that keeps the body level
  PART 6  run()            simulation loop, logging, plots, video

Run:
  python mujoco_dog.py                 # full test, saves results in ./out_mj
  python mujoco_dog.py --view          # live window (needs a display)
  python mujoco_dog.py --no-feedback   # gait only, to compare with feedback
  python mujoco_dog.py --push 4        # kick the body with a 4 N.s impulse... see --help
"""
import argparse
import math
import os

import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ==========================================================================
# PART 1 - PARAMS (all ASSUMPTIONS until replaced by the CAD values)
# ==========================================================================
# --- Dimensions read from the CAD images (ESTIMATES: confirm with the CAD, see the xlsx sheet) ---
L1, L2 = 0.100, 0.125               # thigh / shank length [m]
HIP_X, HIP_Y = 0.18, 0.0925         # hip position from body centre [m]  (360 x 185 mm spacing)
BODY = (0.25, 0.1176, 0.055)        # body box x,y,z [m]  (117.6 mm wide in the CAD)
# --- Masses [kg]: ASSUMPTIONS until the CAD total mass is measured ---
#   body  = frame 0.35 + LiPo 0.375 + Pi5+case 0.09 + bucks/wiring 0.10 + sensors 0.03
#   leg   = roll servo+bracket 0.10 | thigh (pitch+knee servos+print) 0.16 | shank 0.05 | foot 0.01
M_BODY, M_ROLL, M_THIGH, M_SHANK, M_FOOT = 0.945, 0.10, 0.16, 0.05, 0.01
KNEE_SIGN = -1                      # -1: knee points backwards ; +1: knee points forwards (use --knee)
COL_BODY = "0.10 0.10 0.12 1"       # near-black body (as in the CAD)
COL_LEG = "0.88 0.66 0.10 1"        # gold / yellow printed legs (as in the CAD)
COL_SERVO = "0.25 0.25 0.28 1"
FOOT_R = 0.014

TORQUE_LIMIT = 20 * 0.0980665       # DS3218MG 20 kg.cm = 1.96 N.m
STALL_A, IDLE_A = 2.6, 0.02         # electrical study: 2.6 A stall per servo
ARMATURE = 0.004                    # servo gearbox reflected inertia [kg.m2] (assumption)
KP, KD = 20.0, 0.5                  # servo stiffness [N.m/rad] and damping

Z0 = -0.17                          # standing foot height under the hip [m]
Y0 = 0.02                           # feet placed this far outside the hips [m]
STRIDE, TURN_STRIDE = 0.07, 0.08    # [m]
STEP_H, PERIOD, DUTY = 0.035, 0.6, 0.7

K_ROLL, K_PITCH = 0.10, 0.10        # IMU feedback gains [m of foot height per rad]
SIM_DT = 0.002                      # physics step 500 Hz
CTRL_DT = 0.02                      # controller 50 Hz (what a Pi + PCA9685 can do)

LEGS = ["FL", "FR", "RL", "RR"]
HIP = {"FL": (HIP_X, HIP_Y), "FR": (HIP_X, -HIP_Y), "RL": (-HIP_X, HIP_Y), "RR": (-HIP_X, -HIP_Y)}
JOINTS = [f"{l}_{j}" for l in LEGS for j in ("roll", "pitch", "knee")]


# ==========================================================================
# PART 2 - MODEL
# ==========================================================================
def build_mjcf():
    KR = "-2.6 0" if KNEE_SIGN < 0 else "0 2.6"
    def leg(l):
        hx, hy = HIP[l]
        return f"""
    <body name="{l}_roll_link" pos="{hx} {hy} 0">
      <joint name="{l}_roll" type="hinge" axis="1 0 0" range="-0.6 0.6"/>
      <geom type="box" size="0.02 0.02 0.02" mass="{M_ROLL}" rgba="{COL_SERVO}" contype="0" conaffinity="0"/>
      <body name="{l}_thigh">
        <joint name="{l}_pitch" type="hinge" axis="0 1 0" range="-1.5 1.5"/>
        <geom type="capsule" fromto="0 0 0 0 0 {-L1}" size="0.01" mass="{M_THIGH}" rgba="{COL_LEG}" contype="0" conaffinity="0"/>
        <body name="{l}_shank" pos="0 0 {-L1}">
          <joint name="{l}_knee" type="hinge" axis="0 1 0" range="{KR}"/>
          <geom type="capsule" fromto="0 0 0 0 0 {-L2}" size="0.008" mass="{M_SHANK}" rgba="{COL_LEG}" contype="0" conaffinity="0"/>
          <geom name="{l}_foot" type="sphere" pos="0 0 {-L2}" size="{FOOT_R}" mass="{M_FOOT}" rgba="0.08 0.08 0.08 1" friction="1.0 0.02 0.001"/>
        </body>
      </body>
    </body>"""

    # visual hip arms joining the body ends to the hip axes (no collision, negligible mass)
    arms = "".join(
        f'<geom type="capsule" fromto="{math.copysign(BODY[0] / 2, hx):.4f} 0 0 {hx} {hy} 0" size="0.013" '
        f'mass="0.0001" rgba="{COL_BODY}" contype="0" conaffinity="0"/>'
        for hx, hy in HIP.values())
    acts = "".join(
        f'<position name="{n}" joint="{n}" kp="{KP}" kv="{KD}" forcerange="{-TORQUE_LIMIT} {TORQUE_LIMIT}"/>'
        for n in JOINTS)
    return f"""
<mujoco model="robodog">
  <compiler angle="radian"/>
  <option timestep="{SIM_DT}" gravity="0 0 -9.81" integrator="implicitfast"/>
  <default><joint armature="{ARMATURE}" damping="0.02"/></default>
  <visual><global offwidth="960" offheight="640"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.78 0.80 0.84" rgb2="0.68 0.70 0.75" width="256" height="256"/>
    <material name="grid" texture="grid" texrepeat="12 12" reflectance="0.1"/>
  </asset>
  <worldbody>
    <light pos="0 0 2" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="5 5 0.1" material="grid" friction="1.0 0.02 0.001"/>
    <body name="base" pos="0 0 {-Z0 + FOOT_R + 0.005}">
      <freejoint name="root"/>
      <geom type="box" size="{BODY[0]/2} {BODY[1]/2} {BODY[2]/2}" mass="{M_BODY}" rgba="{COL_BODY}" contype="0" conaffinity="0"/>
      {arms}
      <site name="imu" pos="0 0 0"/>
      {''.join(leg(l) for l in LEGS)}
    </body>
  </worldbody>
  <actuator>{acts}</actuator>
  <sensor>
    <gyro name="gyro" site="imu"/>
    <accelerometer name="accel" site="imu"/>
  </sensor>
</mujoco>"""


# ==========================================================================
# PART 3 - INVERSE KINEMATICS (same maths as the PyBullet script)
# ==========================================================================
def leg_ik(foot):
    """foot = (x, y, z) relative to the hip  ->  (roll, hip pitch, knee) in rad."""
    x, y, z = foot
    q0 = math.atan2(y, -z)
    zp = -math.hypot(y, z)
    d = math.hypot(x, zp)
    d = min(max(d, abs(L1 - L2) + 1e-3), L1 + L2 - 1e-3)       # stay inside the workspace
    c = (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    q2 = KNEE_SIGN * math.acos(max(-1.0, min(1.0, c)))         # knee bend direction
    q1 = math.atan2(-x, -zp) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    return q0, q1, q2


# ==========================================================================
# PART 4 - GAIT GENERATOR (trot: diagonal legs move together)
# ==========================================================================
class Gait:
    OFFSET = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}

    def __init__(self):
        self.phase, self.vx, self.wz = 0.0, 0.0, 0.0

    def targets(self, vx_cmd, wz_cmd, dt):
        a = dt / (0.3 + dt)                              # smooth the commands (tau 0.3 s)
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
            if ph < DUTY:                                # stance: foot on ground slides back
                x, z = s / 2 - s * ph / DUTY, 0.0
            else:                                        # swing: lift and move forward
                u = (ph - DUTY) / (1 - DUTY)
                x = -s / 2 + s * (u - math.sin(2 * math.pi * u) / (2 * math.pi))
                z = STEP_H * math.sin(math.pi * u) * active
            out[l] = np.array([x, Y0 if left else -Y0, Z0 + z])
        return out


# ==========================================================================
# PART 5 - BRAIN = gait + IMU feedback
# ==========================================================================
class Brain:
    """Every control step: read IMU -> correct foot heights -> IK -> 12 joint targets."""

    def __init__(self, feedback=True):
        self.gait = Gait()
        self.feedback = feedback

    def act(self, vx_cmd, wz_cmd, roll, pitch):
        tg = self.gait.targets(vx_cmd, wz_cmd, CTRL_DT)
        q = []
        for l in LEGS:
            p = tg[l].copy()
            if self.feedback:
                hx, hy = HIP[l]
                # left side high (roll>0) -> shorten left legs; nose down (pitch>0) -> extend front legs
                p[2] += K_ROLL * roll * np.sign(hy) - K_PITCH * pitch * np.sign(hx)
            q += list(leg_ik(p))
        return np.array(q)


# ==========================================================================
# PART 6 - SIMULATION
# ==========================================================================
SCENARIO = [("stand", 2.0, 0.0, 0.0), ("walk forward", 5.0, 1.0, 0.0),
            ("turn on the spot", 5.0, 0.0, 1.0), ("walk + turn", 4.0, 0.7, 0.5),
            ("stand (end)", 2.0, 0.0, 0.0)]


def quat_to_rp(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def run(args):
    model = mujoco.MjModel.from_xml_string(build_mjcf())
    data = mujoco.MjData(model)
    jid = [model.joint(n).id for n in JOINTS]
    qadr = [model.jnt_qposadr[j] for j in jid]
    vadr = [model.jnt_dofadr[j] for j in jid]
    base = model.body("base").id

    # start in the nominal standing pose
    q_stand = leg_ik(np.array([0, 0, Z0]))
    for i, a in enumerate(qadr):
        data.qpos[a] = q_stand[i % 3]
    mujoco.mj_forward(model, data)

    brain = Brain(feedback=not args.no_feedback)
    steps_per_ctrl = int(round(CTRL_DT / SIM_DT))
    rec = {k: [] for k in ("t", "ph", "pos", "rpy", "tau", "qd")}
    frames = []
    renderer = mujoco.Renderer(model, 320, 480) if args.gif else None
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 0.9, -22, 140

    viewer = None
    if args.view:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)

    q_des = np.array([q_stand[i % 3] for i in range(12)])
    t_total = sum(d for _, d, _, _ in SCENARIO)
    push_done = False
    k = 0
    for name, dur, vx, wz in SCENARIO:
        for _ in range(int(round(dur / CTRL_DT))):
            roll, pitch, yaw = quat_to_rp(data.qpos[3:7])          # "IMU" attitude
            q_des = brain.act(vx, wz, roll, pitch)
            data.ctrl[:] = q_des
            # disturbance: one sideways kick while walking (tests the feedback)
            if args.push and not push_done and data.time > 5.0:
                data.xfrc_applied[base, 1] = args.push / CTRL_DT / 2   # N for 2 control steps
                push_done = True
            elif push_done and data.time > 5.0 + 2 * CTRL_DT:
                data.xfrc_applied[base, :] = 0
            for _ in range(steps_per_ctrl):
                mujoco.mj_step(model, data)
            rec["t"].append(data.time)
            rec["ph"].append(name)
            rec["pos"].append(data.qpos[:3].copy())
            rec["rpy"].append(quat_to_rp(data.qpos[3:7]))
            rec["tau"].append(data.actuator_force.copy())
            rec["qd"].append(data.qvel[vadr].copy())
            k += 1
            if renderer is not None and k % 3 == 0:
                cam.lookat[:] = data.qpos[:3]
                renderer.update_scene(data, cam)
                frames.append(renderer.render().copy())
            if viewer is not None:
                viewer.sync()
    if viewer is not None:
        viewer.close()
    for key in ("t", "pos", "rpy", "tau", "qd"):
        rec[key] = np.array(rec[key])
    rec["ph"] = np.array(rec["ph"])
    return rec, frames


def analyse(rec, out, tag):
    ph, tau = rec["ph"], rec["tau"]
    cur = IDLE_A + np.abs(tau) / TORQUE_LIMIT * (STALL_A - IDLE_A)
    tot = cur.sum(1)
    # servo bank A = servos 1-6 (FL, FR, RL..), B = 7-12 : here per-LEG-pair
    bankA = cur[:, :6].sum(1)       # FL + FR  (front legs)
    bankB = cur[:, 6:].sum(1)       # RL + RR  (rear legs)
    L = [f"[{tag}]",
         f"{'phase':<18}{'dist':>6}{'dYaw':>7}{'maxRoll':>8}{'maxPitch':>9}{'minZ':>7}{'p99 tau':>8}{'avg I[A]':>9}"]
    for n in dict.fromkeys(ph):
        m = ph == n
        pos, rpy = rec["pos"][m], np.degrees(rec["rpy"][m])
        yaw_u = np.degrees(np.unwrap(np.radians(rpy[:, 2])))
        dy = yaw_u[-1] - yaw_u[0]            # unwrapped: can exceed 180 deg
        L.append(f"{n:<18}{np.hypot(*(pos[-1, :2] - pos[0, :2])):>6.2f}{dy:>7.1f}"
                 f"{np.abs(rpy[:, 0]).max():>8.1f}{np.abs(rpy[:, 1]).max():>9.1f}{pos[:, 2].min():>7.3f}"
                 f"{np.percentile(np.abs(tau[m]), 99):>8.2f}{tot[m].mean():>9.1f}")
    for jt in ("roll", "pitch", "knee"):
        c = [i for i, n in enumerate(JOINTS) if n.endswith(jt)]
        p99 = np.percentile(np.abs(tau[:, c]), 99)
        L.append(f"  {jt:<6} p99 torque {p99:4.2f} N.m  margin x{TORQUE_LIMIT / p99:.2f}   (limit {TORQUE_LIMIT:.2f})")
    walk = np.isin(ph, ["walk forward", "walk + turn"])
    L.append(f"  total servo current while walking: avg {tot[walk].mean():.1f} A, p99 {np.percentile(tot[walk], 99):.1f} A")
    L.append(f"  front legs vs rear legs: avg {bankA[walk].mean():.1f} A / {bankB[walk].mean():.1f} A")
    txt = "\n".join(L)
    print(txt)
    open(os.path.join(out, f"summary_{tag}.txt"), "w").write(txt)
    return tot


def plots(rec, tot, out, tag):
    t = rec["t"]
    fig, ax = plt.subplots(2, 2, figsize=(11, 7))
    ax[0, 0].plot(rec["pos"][:, 0], rec["pos"][:, 1]); ax[0, 0].set_aspect("equal")
    ax[0, 0].set_title("Body path (top view) [m]")
    ax[0, 1].plot(t, np.degrees(rec["rpy"][:, 0]), label="roll")
    ax[0, 1].plot(t, np.degrees(rec["rpy"][:, 1]), label="pitch"); ax[0, 1].legend()
    ax[0, 1].set_title("Body tilt [deg]  (smaller = more stable)")
    for jt, c in (("roll", "C0"), ("pitch", "C1"), ("knee", "C2")):
        cols = [i for i, n in enumerate(JOINTS) if n.endswith(jt)]
        ax[1, 0].plot(t, np.abs(rec["tau"][:, cols]).max(1), c, lw=0.6, label=jt)
    ax[1, 0].axhline(TORQUE_LIMIT, color="r", ls="--", label="20 kg.cm")
    ax[1, 0].legend(); ax[1, 0].set_title("Max |torque| per joint type [N.m]")
    ax[1, 1].plot(t, tot); ax[1, 1].set_title("Total servo current, est. [A]")
    for a in ax[0, 1], ax[1, 0], ax[1, 1]:
        a.set_xlabel("time [s]")
    fig.tight_layout(); fig.savefig(os.path.join(out, f"mujoco_{tag}.png"), dpi=120); plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true", help="open the live MuJoCo window")
    ap.add_argument("--no-feedback", action="store_true", help="disable IMU feedback")
    ap.add_argument("--push", type=float, default=0.0, help="sideways impulse [N.s] at t=5 s (during the walk)")
    ap.add_argument("--knee", choices=["back", "front"], default="back", help="knee direction")
    ap.add_argument("--no-gif", dest="gif", action="store_false")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "out_cad"))
    args = ap.parse_args()
    global KNEE_SIGN
    KNEE_SIGN = -1 if args.knee == "back" else 1
    os.makedirs(args.out, exist_ok=True)
    tag = ("nofb" if args.no_feedback else "fb") + (f"_push{args.push:g}" if args.push else "")
    rec, frames = run(args)
    tot = analyse(rec, args.out, tag)
    plots(rec, tot, args.out, tag)
    if frames:
        from PIL import Image
        im = [Image.fromarray(f) for f in frames]
        im[0].save(os.path.join(args.out, f"mujoco_{tag}.gif"), save_all=True,
                   append_images=im[1:], duration=int(1000 * CTRL_DT * 3), loop=0)
    print("saved in", args.out)


if __name__ == "__main__":
    main()
