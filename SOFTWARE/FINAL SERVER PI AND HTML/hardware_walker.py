"""Marche sur le VRAI robot (Raspberry Pi 5 + PCA9685 + 12 servos DS3218MG).

Terminal 1 : py server.py                      (page web + écran OLED)
Terminal 2 : py hardware_walker.py --dry-run   (essai SANS servos : affiche les angles)
             python3 hardware_walker.py        (robot réel, après calibration)

!! NON TESTÉ SUR LE ROBOT. Les canaux, sens et décalages ci-dessous sont des hypothèses.
!! À calibrer à l'atelier, robot posé sur un support, pattes en l'air.
"""
import argparse
import math
import time

from gait_core import LEGS, CommandLink, Controller, leg_angles

# (canal PCA9685, sens +1/-1, décalage en degrés) pour (abduction, hanche, genou)
CALIB = {
    "FL": [(0, +1, 0.0), (1, +1, 0.0), (2, +1, 0.0)],
    "FR": [(3, +1, 0.0), (4, +1, 0.0), (5, +1, 0.0)],
    "RL": [(6, +1, 0.0), (7, +1, 0.0), (8, +1, 0.0)],
    "RR": [(9, +1, 0.0), (10, +1, 0.0), (11, +1, 0.0)],
}
ACTUATION = 180                       # ° de course du servo : à vérifier (180 ou 270 selon le modèle)
CENTER = ACTUATION / 2
LIMIT = (15.0, ACTUATION - 15.0)      # butées logicielles de protection
PULSE_US = (500, 2500)                # à vérifier dans la datasheet du DS3218MG


def to_servo_angle(q, direction, offset):
    deg = CENTER + direction * math.degrees(q) + offset
    return max(LIMIT[0], min(LIMIT[1], deg))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", default="http://127.0.0.1:5000/state")
    ap.add_argument("--hz", type=float, default=50)
    ap.add_argument("--dry-run", action="store_true", help="n'envoie rien aux servos")
    args = ap.parse_args()

    kit = None
    if not args.dry_run:
        from adafruit_servokit import ServoKit
        kit = ServoKit(channels=16)
        for leg in LEGS:
            for ch, _, _ in CALIB[leg]:
                kit.servo[ch].actuation_range = ACTUATION
                kit.servo[ch].set_pulse_width_range(*PULSE_US)

    link = CommandLink(args.server)
    link.start()
    ctrl = Controller()
    dt = 1.0 / args.hz
    next_t, last_print = time.time(), 0.0
    print("Marche démarrée", "(dry-run)" if args.dry_run else "(servos actifs)", "- Ctrl+C pour arrêter")

    try:
        while True:
            ang = leg_angles(ctrl.targets(link.cmd, dt))
            out = {}
            for leg in LEGS:
                for (ch, sign, off), q in zip(CALIB[leg], ang[leg]):
                    out[ch] = to_servo_angle(q, sign, off)
                    if kit:
                        kit.servo[ch].angle = out[ch]
            if args.dry_run and time.time() - last_print > 0.5:
                last_print = time.time()
                print(link.cmd["mode"].ljust(10), " ".join(f"{out[c]:5.1f}" for c in sorted(out)))
            next_t += dt
            time.sleep(max(0.0, next_t - time.time()))
    except KeyboardInterrupt:
        print("\nArrêt : les servos restent dans leur dernière position.")


if __name__ == "__main__":
    main()
