"""Noyau de marche du Robodog, partagé par la simulation (PyBullet) et le vrai robot (PCA9685).
Reprend le code de l'équipe (leg_ik, Gait) sans dépendance PyBullet.
Les commandes viennent du site web : http://127.0.0.1:5000/state
"""
import json
import math
import threading
import time
import urllib.request

import numpy as np

SERVO_TORQUE = 20 * 0.0980665   # N·m : DS3218MG, 20 kg·cm (= 1,96 N·m)
SERVO_SPEED = 6.5               # rad/s
Z_STAND, Z_SIT = -0.16, -0.05   # hauteur du pied sous la hanche (m)
RISE_SPEED = 0.05               # m/s : vitesse pour se lever / s'asseoir
STRIDE, TURN_STRIDE, STEP_H = 0.04, 0.03, 0.03
PERIOD, DUTY = 0.6, 0.7
L1, L2 = 0.11, 0.12             # cuisse, jambe (m), d'après le CAD
LEGS = ["FL", "FR", "RL", "RR"]


def leg_ik(L1, L2, foot, knee_sign=-1):
    x, y, z = foot
    q0 = math.atan2(y, -z)
    zp = -math.hypot(y, z)
    d = math.hypot(x, zp)
    d = min(max(d, abs(L1 - L2) + 1e-3), L1 + L2 - 1e-3)
    c = (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    q2 = knee_sign * math.acos(max(-1.0, min(1.0, c)))
    th = math.atan2(-x, -zp)
    q1 = th - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    return q0, q1, q2


class Gait:
    PHASE_OFFSET = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}

    def __init__(self):
        self.phase = 0.0
        self.vx = 0.0
        self.wz = 0.0

    def foot_targets(self, vx_cmd, wz_cmd, dt):
        a = dt / (0.3 + dt)
        self.vx += a * (vx_cmd - self.vx)
        self.wz += a * (wz_cmd - self.wz)
        s_fwd = STRIDE * self.vx
        s_turn = TURN_STRIDE * self.wz
        activity = min(1.0, (abs(s_fwd) + abs(s_turn)) / 0.02)
        if activity > 1e-3:
            self.phase = (self.phase + dt / PERIOD) % 1.0
        tg = {}
        for leg in LEGS:
            left = leg.endswith("L")
            s = s_fwd - s_turn if left else s_fwd + s_turn
            ph = (self.phase + self.PHASE_OFFSET[leg]) % 1.0
            if ph < DUTY:
                u = ph / DUTY
                x, z = s / 2 - s * u, 0.0
            else:
                u = (ph - DUTY) / (1 - DUTY)
                x = -s / 2 + s * (u - math.sin(2 * math.pi * u) / (2 * math.pi))
                z = STEP_H * math.sin(math.pi * u) * activity
            tg[leg] = np.array([x, 0.03 if left else -0.03, Z_STAND + z])
        return tg


class Controller:
    """Transforme la commande du site web (mode, vx, turn) en positions de pieds.
    OFF / SIT : s'asseoir.  STOP : figer.  Sinon : se lever, puis marcher."""

    def __init__(self):
        self.gait = Gait()
        self.z = Z_SIT

    def targets(self, cmd, dt):
        mode, vx, wz = cmd["mode"], cmd["vx"], cmd["turn"]
        if mode in ("OFF", "SIT"):
            z_goal, vx, wz = Z_SIT, 0, 0
        elif mode == "STOP":
            z_goal, vx, wz = self.z, 0, 0
        else:
            z_goal = Z_STAND
            if abs(self.z - Z_STAND) > 0.005:   # pas de marche tant qu'il n'est pas debout
                vx = wz = 0
        step = RISE_SPEED * dt
        self.z += max(-step, min(step, z_goal - self.z))
        tg = self.gait.foot_targets(vx, wz, dt)
        for leg in LEGS:
            tg[leg][2] += self.z - Z_STAND
        return tg


def leg_angles(tg):
    """{jambe: (abduction, hanche, genou)} en radians."""
    return {leg: leg_ik(L1, L2, tg[leg]) for leg in LEGS}


STOP_CMD = {"mode": "STOP", "vx": 0, "turn": 0}


class CommandLink(threading.Thread):
    """Lit la commande du serveur web ~25 fois par seconde.
    Si le serveur ne répond plus, la commande devient STOP (sécurité)."""

    def __init__(self, url="http://127.0.0.1:5000/state"):
        super().__init__(daemon=True)
        self.url = url
        self.cmd = dict(STOP_CMD)

    def run(self):
        while True:
            try:
                with urllib.request.urlopen(self.url, timeout=0.3) as r:
                    d = json.loads(r.read())
                self.cmd = {"mode": d["mode"], "vx": d["vx"], "turn": d["turn"]}
            except Exception:
                self.cmd = dict(STOP_CMD)
            time.sleep(0.04)
