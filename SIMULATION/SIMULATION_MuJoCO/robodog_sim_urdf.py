#!/usr/bin/env python3
import argparse, math, os, time
import numpy as np
import pybullet as p
import pybullet_data
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SERVO_TORQUE  = 20 * 0.0980665
SERVO_SPEED   = 6.5
SERVO_STALL_A = 2.5
SERVO_IDLE_A  = 0.02

Z_STAND     = -0.16
STRIDE      = 0.07
TURN_STRIDE = 0.05
STEP_H      = 0.035
PERIOD      = 0.6
DUTY        = 0.7
Y_OFFSET    = 0.03

DT = 1.0/240.0
LEGS = ["FL","FR","RL","RR"]

SCENARIO = [
    ("stand up from rest", 2.5, 0.0, 0.0, True),
    ("stand",              1.0, 0.0, 0.0, False),
    ("walk forward",       5.0, 1.0, 0.0, False),
    ("turn on the spot",   5.0, 0.0, 1.0, False),
    ("walk + turn",        4.0, 0.7, 0.5, False),
    ("stand (end)",        2.0, 0.0, 0.0, False),
]

def load_urdf(urdf_path, base_pos=(0,0,0.30)):
    root = os.path.dirname(os.path.abspath(urdf_path))
    for guess in (root, os.path.join(root,".."), os.path.join(root,"..","..")):
        g = os.path.abspath(guess)
        p.setAdditionalSearchPath(g)
        p.setAdditionalSearchPath(os.path.join(g,"meshes"))
        p.setAdditionalSearchPath(os.path.join(g,"meshes","robotdog"))
    return p.loadURDF(urdf_path, base_pos, useFixedBase=False,
                      flags=p.URDF_USE_INERTIA_FROM_FILE)
def find_legs(robot):
    n2j = {}
    for j in range(p.getNumJoints(robot)):
        n2j[p.getJointInfo(robot, j)[1].decode()] = j
        
    # MAPPING for your Fusion 360 URDF
    # If you get a KeyError here, check the exact spelling in your URDF!
    mapping = {
        "FL": ("Revolute_2", "Revolute_14", "Revolute_4"),
        "FR": ("Revolute_11", "Revolute_12", "Revolute_10"),
        "RL": ("Revolute_8", "Revolute_9", "Revolute_13"),
        "RR": ("Revolute_16", "Revolute_17", "Revolute_18")
    }
    
    legs = {}
    for leg, joint_names in mapping.items():
        try:
            legs[leg] = (n2j[joint_names[0]], n2j[joint_names[1]], n2j[joint_names[2]], -1)
        except KeyError as e:
            print(f"ERROR: Could not find joint {e}. Available joints: {list(n2j.keys())}")
            raise
    return legs, n2j

def leg_fk(L1, L2, q0, q1, q2):
    xp = -L1*math.sin(q1) - L2*math.sin(q1+q2)
    zp = -L1*math.cos(q1) - L2*math.cos(q1+q2)
    return np.array([xp, -zp*math.sin(q0), zp*math.cos(q0)])

def leg_ik(L1, L2, foot, knee_sign=-1):
    x, y, z = foot
    q0 = math.atan2(y, -z)
    zp = -math.hypot(y, z)
    d  = math.hypot(x, zp)
    d  = min(max(d, abs(L1-L2)+1e-3), L1+L2-1e-3)
    c  = (d*d - L1*L1 - L2*L2) / (2*L1*L2)
    q2 = knee_sign * math.acos(max(-1.0, min(1.0, c)))
    th = math.atan2(-x, -zp)
    q1 = th - math.atan2(L2*math.sin(q2), L1+L2*math.cos(q2))
    return q0, q1, q2

class Gait:
    PHASE_OFFSET = {"FL":0.0, "RR":0.0, "FR":0.5, "RL":0.5}
    def __init__(self):
        self.phase = 0.0; self.vx = 0.0; self.wz = 0.0
    def foot_targets(self, vx_cmd, wz_cmd, dt):
        a = dt/(0.3+dt)
        self.vx += a*(vx_cmd - self.vx)
        self.wz += a*(wz_cmd - self.wz)
        s_fwd  = STRIDE      * self.vx
        s_turn = TURN_STRIDE * self.wz
        activity = min(1.0, (abs(s_fwd)+abs(s_turn))/0.02)
        if activity > 1e-3:
            self.phase = (self.phase + dt/PERIOD) % 1.0
        tg = {}
        for leg in LEGS:
            left = leg.endswith("L")
            s = s_fwd - s_turn if left else s_fwd + s_turn
            ph = (self.phase + self.PHASE_OFFSET[leg]) % 1.0
            if ph < DUTY:
                u = ph/DUTY
                x, z = s/2 - s*u, 0.0
            else:
                u = (ph-DUTY)/(1-DUTY)
                x = -s/2 + s*(u - math.sin(2*math.pi*u)/(2*math.pi))
                z = STEP_H*math.sin(math.pi*u)*activity
            tg[leg] = np.array([x, Y_OFFSET if left else -Y_OFFSET, Z_STAND + z])
        return tg

def run(args):
    out = os.path.abspath(args.out); os.makedirs(out, exist_ok=True)
    cid = p.connect(p.GUI if args.gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    p.setTimeStep(DT)
    p.loadURDF("plane.urdf")
    p.changeDynamics(0, -1, lateralFriction=1.0)

    L1, L2 = args.L1, args.L2
    z_start = -Z_STAND + 0.012 + 0.01
    robot = load_urdf(args.urdf, [0,0,z_start])
    p.changeDynamics(robot, -1, lateralFriction=1.0)
    for j in range(p.getNumJoints(robot)):
        p.changeDynamics(robot, j, lateralFriction=1.0)

    legs, n2j = find_legs(robot)
    print("Leg joint indices:", legs)

    jids = []
    for leg in LEGS:
        r,pf,kn,_ = legs[leg]
        jids += [r,pf,kn]

    rng = np.random.default_rng(0); worst = 0.0
    for _ in range(500):
        tgt = np.array([rng.uniform(-0.08,0.08), rng.uniform(-0.04,0.04),
                        rng.uniform(-0.21,-0.13)])
        fk = leg_fk(L1,L2,*leg_ik(L1,L2,tgt))
        worst = max(worst, float(np.linalg.norm(fk-tgt)))
    print(f"IK/FK self-check worst error: {worst*1000:.4f} mm")
    assert worst < 1e-4, "IK/FK mismatch"

    for j in jids: p.resetJointState(robot, j, 0.0)
    p.stepSimulation()

    gait = Gait()
    limit = args.limit
    log = {k: [] for k in ("t","phase","pos","rpy","q","qd","tau")}
    frames = []; cam_every = int(1/(DT*15))
    t = 0.0; step = 0

    for name, dur, vx, wz, is_standup in SCENARIO:
        n = int(dur/DT)
        for i in range(n):
            frac = (i/max(n-1,1)) if is_standup else 1.0
            if is_standup:
                z_tgt = -0.04 + (Z_STAND - -0.04)*frac
            else:
                z_tgt = Z_STAND
            tg_raw = gait.foot_targets(vx, wz, DT)
            q_des = []
            for leg in LEGS:
                foot = np.array([0.0, 0.0, z_tgt]) if is_standup else tg_raw[leg]
                q_des += list(leg_ik(L1,L2,foot))
            for jid, qd in zip(jids, q_des):
                p.setJointMotorControl2(robot, jid, p.POSITION_CONTROL,
                    targetPosition=qd, targetVelocity=0.0, force=limit,
                    positionGain=args.kp, velocityGain=args.kd,
                    maxVelocity=SERVO_SPEED)
            p.stepSimulation(); t += DT; step += 1

            st = p.getJointStates(robot, jids)
            pos, orn = p.getBasePositionAndOrientation(robot)
            log["t"].append(t); log["phase"].append(name)
            log["pos"].append(pos)
            log["rpy"].append(p.getEulerFromQuaternion(orn))
            log["q"].append([s[0] for s in st])
            log["qd"].append([s[1] for s in st])
            log["tau"].append([s[3] for s in st])

            if args.gui: time.sleep(DT)
            if args.gif and step % cam_every == 0:
                view = p.computeViewMatrixFromYawPitchRoll(pos, 0.75, 40, -22, 0, 2)
                proj = p.computeProjectionMatrixFOV(55, 1.5, 0.05, 10)
                img = p.getCameraImage(480,320,view,proj,
                                       renderer=p.ER_TINY_RENDERER)[2]
                frames.append(np.reshape(img,(320,480,4))[:,:,:3].astype(np.uint8))

    p.disconnect(cid)
    for k in ("pos","rpy","q","tau","qd","t"): log[k] = np.array(log[k])
    log["phase"] = np.array(log["phase"])
    return out, log, frames

def analyse(out, log, limit):
    t, tau, qd, ph = log["t"], log["tau"], log["qd"], log["phase"]
    lines = [f"Torque cap   : {limit:.2f} N.m ({limit/0.0980665:.1f} kg.cm)",
             f"Servo rated  : {SERVO_TORQUE:.2f} N.m (20 kg.cm)",
             f"Total mass   : {3.10 + 4*(0.10+0.12+0.04+0.01):.2f} kg", ""]
    lines.append(f"{'phase':<22}{'dist':>7}{'dYaw':>8}{'maxRoll':>9}{'maxPitch':>10}{'minZ':>8}{'p99Tau':>8}{'peakTau':>9}")
    for name in dict.fromkeys(ph):
        m = ph == name; pos = log["pos"][m]; rpy = np.degrees(log["rpy"][m])
        d = float(np.hypot(*(pos[-1,:2]-pos[0,:2])))
        dy = (rpy[-1,2]-rpy[0,2]+180)%360-180
        pk = float(np.max(np.abs(tau[m]))); p99 = float(np.percentile(np.abs(tau[m]),99))
        lines.append(f"{name:<22}{d:>7.2f}{dy:>8.1f}{np.max(np.abs(rpy[:,0])):>9.1f}"
                     f"{np.max(np.abs(rpy[:,1])):>10.1f}{pos[:,2].min():>8.3f}{p99:>8.2f}{pk:>9.2f}")
    for jt, cols in (("roll",[0,3,6,9]),("pitch",[1,4,7,10]),("knee",[2,5,8,11])):
        a = np.abs(tau[:, cols])
        lines.append(f"  {jt:<6} peak {a.max():5.2f}  p99 {np.percentile(a,99):5.2f}  "
                     f"rms {np.sqrt((a**2).mean()):5.2f}  "
                     f"margin(p99) x{SERVO_TORQUE/max(np.percentile(a,99),1e-6):.2f}")
    cur = SERVO_IDLE_A + (np.abs(tau)/SERVO_TORQUE)*(SERVO_STALL_A-SERVO_IDLE_A)
    tot = cur.sum(axis=1)
    walk = np.isin(ph, ["walk forward","walk + turn"])
    lines += ["", f"avg servo current (walking): {tot[walk].mean():.1f} A",
                  f"peak servo current          : {tot.max():.1f} A",
                  f"mech power avg (walking)    : {np.abs(tau*qd).sum(axis=1)[walk].mean():.1f} W"]
    txt = "\n".join(lines)
    open(os.path.join(out,"summary.txt"),"w").write(txt); print(txt)
    return tot

def plots(out, log, tot, limit):
    t, ph = log["t"], log["phase"]
    names = list(dict.fromkeys(ph)); bounds = [t[np.where(ph==n)[0][0]] for n in names]
    def shade(ax):
        for b,n in zip(bounds,names):
            ax.axvline(b,color="k",lw=0.4,alpha=0.4)
            ax.text(b+0.05, ax.get_ylim()[1], n, fontsize=6, va="top")
    fig, axs = plt.subplots(3,1,figsize=(9,8),sharex=True)
    for ax, cols, jt in zip(axs, ([0,3,6,9],[1,4,7,10],[2,5,8,11]), ("roll","pitch","knee")):
        for ci,c in enumerate(cols):
            ax.plot(t, np.abs(log["tau"][:,c]), lw=0.7, label=LEGS[ci])
        ax.axhline(SERVO_TORQUE,color="r",ls="--",label="20 kg.cm")
        ax.set_ylabel(f"|torque| {jt} [N.m]"); ax.legend(ncol=5,fontsize=6,loc="upper right"); shade(ax)
    axs[-1].set_xlabel("time [s]"); fig.suptitle("Joint torque vs DS3218MG limit")
    fig.tight_layout(); fig.savefig(os.path.join(out,"torque.png"),dpi=130); plt.close(fig)

    fig, axs = plt.subplots(2,2,figsize=(10,7))
    pos = log["pos"]
    axs[0,0].plot(pos[:,0],pos[:,1]); axs[0,0].set_aspect("equal")
    axs[0,0].set_title("Body path"); axs[0,0].set_xlabel("x [m]"); axs[0,0].set_ylabel("y [m]")
    axs[0,1].plot(t,pos[:,2]); axs[0,1].set_title("Body height [m]")
    axs[1,0].plot(t,np.degrees(log["rpy"][:,0]),label="roll")
    axs[1,0].plot(t,np.degrees(log["rpy"][:,1]),label="pitch")
    axs[1,0].legend(); axs[1,0].set_title("roll/pitch [deg]")
    axs[1,1].plot(t,tot); axs[1,1].set_title("Est. total servo current [A]")
    for ax in (axs[0,1],axs[1,0],axs[1,1]): ax.set_xlabel("time [s]")
    fig.tight_layout(); fig.savefig(os.path.join(out,"body_motion.png"),dpi=130); plt.close(fig)

    hdr = "t,phase,x,y,z,roll,pitch,yaw," + ",".join(f"tau_{i}" for i in range(12))
    with open(os.path.join(out,"log.csv"),"w") as f:
        f.write(hdr+"\n")
        for i in range(len(t)):
            f.write(f"{t[i]:.4f},{ph[i]},{pos[i,0]:.4f},{pos[i,1]:.4f},{pos[i,2]:.4f},"
                    + ",".join(f"{v:.4f}" for v in log["rpy"][i])
                    + "," + ",".join(f"{v:.4f}" for v in log["tau"][i]) + "\n")

def save_gif(out, frames):
    from PIL import Image
    imgs = [Image.fromarray(f) for f in frames]
    imgs[0].save(os.path.join(out,"robodog.gif"), save_all=True, append_images=imgs[1:],
                 duration=int(1000/15), loop=0)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),"robotdog_visual.urdf"))
    ap.add_argument("--L1", type=float, default=0.11)
    ap.add_argument("--L2", type=float, default=0.12)
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--limit", type=float, default=10.0)
    ap.add_argument("--kp", type=float, default=0.15)
    ap.add_argument("--kd", type=float, default=1.0)
    ap.add_argument("--no-gif", dest="gif", action="store_false")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),"out"))
    args = ap.parse_args()
    out, log, frames = run(args)
    tot = analyse(out, log, args.limit)
    plots(out, log, tot, args.limit)
    if args.gif and frames: save_gif(out, frames)
    print(f"\nSaved results in {out}")

if __name__ == "__main__":
    main()
