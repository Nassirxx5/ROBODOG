"""Simulation PyBullet commandée EN DIRECT par le site web.

Terminal 1 : py server.py
Terminal 2 : py sim_live.py --gui --urdf robotdog_cad.urdf
Navigateur : http://127.0.0.1:5000  (P pour allumer, puis W A S D)

À la fin (Ctrl+C ou fenêtre fermée) : couple max de chaque articulation, chute oui/non,
et le graphe sim_torque.png pour le design book.
"""
import argparse
import os
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import pybullet_data

from gait_core import (LEGS, L1, L2, SERVO_SPEED, SERVO_TORQUE, CommandLink,
                       Controller, leg_angles)

DT = 1.0 / 240.0


def load_urdf(urdf_path, base_pos=(0, 0, 0.30)):
    root = os.path.dirname(os.path.abspath(urdf_path))
    p.setAdditionalSearchPath(root)
    p.setAdditionalSearchPath(os.path.join(root, "meshes"))
    p.setAdditionalSearchPath(os.path.join(root, "meshes", "robotdog"))
    return p.loadURDF(urdf_path, base_pos, useFixedBase=False,
                      flags=p.URDF_USE_INERTIA_FROM_FILE)


def find_legs(robot):
    n2j = {p.getJointInfo(robot, j)[1].decode(): j for j in range(p.getNumJoints(robot))}
    mapping = {
        "FL": ("Revolute_2", "Revolute_14", "Revolute_4"),
        "FR": ("Revolute_11", "Revolute_12", "Revolute_10"),
        "RL": ("Revolute_8", "Revolute_9", "Revolute_13"),
        "RR": ("Revolute_16", "Revolute_17", "Revolute_18"),
    }
    return {leg: tuple(n2j[n] for n in names) for leg, names in mapping.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default="robotdog_cad.urdf")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--server", default="http://127.0.0.1:5000/state")
    ap.add_argument("--limit", type=float, default=SERVO_TORQUE,
                    help="couple max par servo en N·m (défaut : 1,96 = DS3218MG)")
    ap.add_argument("--seconds", type=float, default=0, help="0 = jusqu'à Ctrl+C")
    args = ap.parse_args()

    p.connect(p.GUI if args.gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(DT)
    p.loadURDF("plane.urdf")
    robot = load_urdf(args.urdf, [0, 0, 0.35])
    legs = find_legs(robot)
    jids = [j for leg in LEGS for j in legs[leg]]
    for j in jids:
        p.resetJointState(robot, j, 0.0)

    link = CommandLink(args.server)
    link.start()
    ctrl = Controller()
    max_tau = np.zeros(len(jids))
    log_t, log_tau, fell = [], [], False
    t, k, next_t = 0.0, 0, time.time()
    print(f"Simulation prête (limite {args.limit:.2f} N·m par servo). Ctrl+C pour terminer.")

    try:
        while p.isConnected():
            ang = leg_angles(ctrl.targets(link.cmd, DT))
            for leg in LEGS:
                for jid, qd in zip(legs[leg], ang[leg]):
                    p.setJointMotorControl2(robot, jid, p.POSITION_CONTROL, targetPosition=qd,
                                            force=args.limit, maxVelocity=SERVO_SPEED,
                                            positionGain=0.2, velocityGain=1.0)
            p.stepSimulation()
            t += DT
            k += 1
            tau = np.array([abs(p.getJointState(robot, j)[3]) for j in jids])
            max_tau = np.maximum(max_tau, tau)
            if k % 12 == 0:
                log_t.append(t)
                log_tau.append(tau.max())
            if k % 60 == 0:
                roll, pitch, _ = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(robot)[1])
                fell = fell or abs(roll) > 1.0 or abs(pitch) > 1.0
            if args.seconds and t >= args.seconds:
                break
            next_t += DT
            wait = next_t - time.time()
            time.sleep(wait) if wait > 0 else None
            if wait < -0.5:
                next_t = time.time()     # trop lent : on ne rattrape pas
    except (KeyboardInterrupt, p.error):
        pass

    print("\n=== RÉSULTAT ===")
    print(f"Durée simulée : {t:.1f} s | robot tombé : {'OUI' if fell else 'non'}")
    print(f"Couple max observé : {max_tau.max():.2f} N·m (limite servo {SERVO_TORQUE:.2f} N·m)")
    if len(log_t):
        plt.figure(figsize=(8, 4))
        plt.plot(log_t, log_tau, label="couple max (toutes articulations)")
        plt.axhline(SERVO_TORQUE, color="r", ls="--", label="limite DS3218MG 1,96 N·m")
        plt.xlabel("temps simulé (s)"); plt.ylabel("N·m"); plt.legend()
        plt.title("Couple des servos pendant la simulation")
        plt.tight_layout(); plt.savefig("sim_torque.png", dpi=150)
        print("Graphe enregistré : sim_torque.png")
    if p.isConnected():
        p.disconnect()


if __name__ == "__main__":
    main()
