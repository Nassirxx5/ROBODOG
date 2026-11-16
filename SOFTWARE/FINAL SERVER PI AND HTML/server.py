"""Serveur unifié du Robodog (Flask + Boucle 50 Hz + Capteurs réels + Écran OLED + Buzzer GPIO 17 + Arbitrage Admin/Invité).
Lancer : python server.py
"""
import logging
import math
import os
import socket
import threading
import time

from flask import Flask, jsonify, request, send_from_directory
from oled_display import RobodogOLED, FPS as OLED_FPS, IS_HARDWARE, device

TIMEOUT = 0.4          # s : sans message du navigateur, le robot s'arrête
LEASE_TIMEOUT = 8.0    # s : expiration automatique de la session pilote si déconnecté
ADMIN_TIMEOUT = 1800.0 # s : expiration de la session Admin après 30 min d'inactivité
REQUEST_TIMEOUT = 20.0 # s : expiration d'une demande d'accès invité
LOOP_HZ = 50           # fréquence de la boucle de marche
SENSOR_HZ = 10         # fréquence de lecture des capteurs réels (ToF, IMU)
BUZZER_PIN = 17        # GPIO 17 (Pin physique 11) pour le buzzer actif 3.3V
ADMIN_PIN = os.environ.get("ROBODOG_ADMIN_PIN", "2026")

logging.getLogger("werkzeug").setLevel(logging.ERROR)
app = Flask(__name__)
oled = RobodogOLED()


class BuzzerController:
    """Contrôle non-bloquant du buzzer actif 3.3V sur GPIO 17 (avec repli silencieux sur PC)."""

    def __init__(self, pin=BUZZER_PIN):
        self.buzzer = None
        if not IS_HARDWARE:
            return
        try:
            from gpiozero import Buzzer
            self.buzzer = Buzzer(pin)
            print(f"[BUZZER] Active buzzer ready on GPIO {pin}")
        except Exception:
            self.buzzer = None

    def beep(self, duration=0.1, count=1, gap=0.07):
        if self.buzzer is None:
            return

        def _worker():
            for i in range(count):
                try:
                    self.buzzer.on()
                    time.sleep(duration)
                    self.buzzer.off()
                    if i < count - 1:
                        time.sleep(gap)
                except Exception:
                    break

        threading.Thread(target=_worker, daemon=True).start()


buzzer = BuzzerController()


class HardwareSensors:
    """Lecture automatique des capteurs réels sur Raspberry Pi 5 (VL53L0X, MPU6050).
    Sur PC (simulation sans capteurs I2C), conserve les valeurs par défaut sans erreur.
    """

    def __init__(self):
        self.tof = None
        self.imu = None
        self._init_hardware()

    def _init_hardware(self):
        if not IS_HARDWARE:
            return
        try:
            import board
            import adafruit_vl53l0x
            self.tof = adafruit_vl53l0x.VL53L0X(board.I2C())
            print("[SENSORS] VL53L0X ToF Radar ready")
        except Exception:
            self.tof = None

        try:
            import board
            import adafruit_mpu6050
            self.imu = adafruit_mpu6050.MPU6050(board.I2C())
            print("[SENSORS] MPU6050 IMU ready")
        except Exception:
            self.imu = None

    def read(self):
        readings = {}

        if self.tof is not None:
            try:
                dist = int(self.tof.range)
                if 20 <= dist <= 2000:
                    readings["distance_mm"] = dist
            except Exception:
                pass

        if self.imu is not None:
            try:
                ax, ay, az = self.imu.acceleration
                pitch = math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az)))
                readings["pitch_deg"] = round(pitch, 1)
            except Exception:
                pass

        return readings


class Robot:
    """Point de branchement avec le code de marche (kinematics / gait) + capteurs réels + arbitrage Admin / Invité."""

    def __init__(self):
        self.vx = 0             # -1 reculer, 0 arrêt, 1 avancer
        self.turn = 0           # -1 droite, 0 aucun, 1 gauche
        self.posture = "STAND"  # STAND, SIT, STOP
        self.powered = False    # le robot démarre ÉTEINT (sécurité)
        self.oled_screen = "EYES"

        self.distance_mm = 640
        self.pitch_deg = 0.0
        self.sensors_live = False
        self.last_seen = time.time()

        self.admins = {}             # {sid: last_seen}
        self.session_numbers = {}    # {sid: int} numéro unique par onglet/appareil
        self.next_user_num = 1
        self.pilot_id = None         # sid du pilote actif
        self.pilot_label = ""
        self.pilot_seen = 0.0
        self.pending_request = None  # {"sid": str, "label": str, "ts": float}
        self.lock = threading.Lock()

    def _label_for_sid_locked(self, sid, fallback=""):
        if not sid:
            return fallback or "Guest"
        if sid not in self.session_numbers:
            self.session_numbers[sid] = self.next_user_num
            self.next_user_num += 1
        return f"User #{self.session_numbers[sid]}"

    def _cleanup_locked(self):
        now = time.time()
        if self.pilot_id and (now - self.pilot_seen > LEASE_TIMEOUT):
            self.pilot_id = None
            self.pilot_label = ""
            self.vx, self.turn = 0, 0
        if self.pending_request and (now - self.pending_request["ts"] > REQUEST_TIMEOUT):
            self.pending_request = None
        expired_admins = [s for s, ts in self.admins.items() if now - ts > ADMIN_TIMEOUT]
        for s in expired_admins:
            del self.admins[s]

    def _is_admin_locked(self, sid):
        self._cleanup_locked()
        if sid and sid in self.admins:
            self.admins[sid] = time.time()
            return True
        return False

    def _is_pilot_locked(self, sid):
        self._cleanup_locked()
        if sid and self.pilot_id == sid:
            self.pilot_seen = time.time()
            return True
        return False

    def admin_unlock(self, sid, pin, label="", mode="admin"):
        with self.lock:
            self._cleanup_locked()
            if not sid:
                return False, "Missing session ID"
            if str(pin).strip() != ADMIN_PIN:
                return False, "Invalid Admin PIN"
            user_lbl = self._label_for_sid_locked(sid, label)
            if mode == "guest":
                self.pilot_id = sid
                self.pilot_label = user_lbl
                self.pilot_seen = time.time()
                self.vx, self.turn = 0, 0
                if self.pending_request and self.pending_request["sid"] == sid:
                    self.pending_request = None
                buzzer.beep(0.08, 2)
                return True, "Guest turn unlocked"

            self.admins[sid] = time.time()
            if self.pilot_id is None or self.pilot_id == sid:
                self.pilot_id = sid
                self.pilot_label = f"Admin ({user_lbl})"
                self.pilot_seen = time.time()
                self.vx, self.turn = 0, 0
            buzzer.beep(0.1, 1)
            return True, "Admin unlocked"

    def admin_lock(self, sid):
        with self.lock:
            self._cleanup_locked()
            self.admins.pop(sid, None)
            if self.pilot_id == sid:
                self.pilot_id = None
                self.pilot_label = ""
                self.vx, self.turn = 0, 0
            return True, "Admin locked"

    def take_control(self, sid, label=""):
        with self.lock:
            if not self._is_admin_locked(sid):
                return False, "Admin PIN required"
            user_lbl = self._label_for_sid_locked(sid, label)
            self.pilot_id = sid
            self.pilot_label = f"Admin ({user_lbl})"
            self.pilot_seen = time.time()
            self.vx, self.turn = 0, 0
            if self.pending_request and self.pending_request["sid"] == sid:
                self.pending_request = None
            buzzer.beep(0.09, 1)
            return True, "Admin control active"

    def release_control(self, sid):
        with self.lock:
            self._cleanup_locked()
            if sid and self.pilot_id == sid:
                self.pilot_id = None
                self.pilot_label = ""
                self.vx, self.turn = 0, 0
                return True, "Control released"
            return False, "Not active pilot"

    def request_control(self, sid, label=""):
        with self.lock:
            if self._is_pilot_locked(sid):
                return True, "Already active pilot"
            user_lbl = self._label_for_sid_locked(sid, label)
            if self._is_admin_locked(sid):
                self.pilot_id = sid
                self.pilot_label = f"Admin ({user_lbl})"
                self.pilot_seen = time.time()
                self.vx, self.turn = 0, 0
                return True, "Admin control active"
            if not sid:
                return False, "Missing session ID"
            self.pending_request = {
                "sid": sid,
                "label": user_lbl,
                "ts": time.time(),
            }
            return True, "Request sent to Admin"

    def respond_request(self, sid, approve=True):
        with self.lock:
            if not self._is_admin_locked(sid):
                return False, "Admin only"
            req = self.pending_request
            self.pending_request = None
            if not req:
                return False, "No pending request"
            if approve:
                self.pilot_id = req["sid"]
                self.pilot_label = req["label"]
                self.pilot_seen = time.time()
                self.vx, self.turn = 0, 0
                buzzer.beep(0.08, 2)
                return True, f"Approved {req['label']}"
            return True, "Request denied"

    def set_command(self, vx, turn, sid=""):
        with self.lock:
            if not self._is_pilot_locked(sid):
                return False
            if not self.powered:
                vx, turn = 0, 0
            self.vx, self.turn = vx, turn
            if vx != 0 or turn != 0:
                self.posture = "STAND"
            self.last_seen = time.time()
            return True

    def set_posture(self, posture, sid="", label=""):
        with self.lock:
            if posture == "STOP":
                is_pilot = self._is_pilot_locked(sid)
                is_admin = self._is_admin_locked(sid)
                if not (is_pilot or is_admin):
                    return False
                if self.powered:
                    self.posture = "STOP"
                self.vx, self.turn = 0, 0
                if is_admin and not is_pilot:
                    user_lbl = self._label_for_sid_locked(sid, label)
                    self.pilot_id = sid
                    self.pilot_label = f"Admin ({user_lbl})"
                    self.pilot_seen = time.time()
                self.last_seen = time.time()
                buzzer.beep(0.22, 3, gap=0.06)
                return True

            if not self._is_pilot_locked(sid):
                return False
            if self.powered and posture in ("STAND", "SIT"):
                self.posture = posture
                if posture == "SIT":
                    self.vx, self.turn = 0, 0
                self.last_seen = time.time()
            return True

    def set_power(self, on, sid=""):
        with self.lock:
            if not self._is_pilot_locked(sid):
                return False
            new_state = bool(on)
            if new_state != self.powered:
                if new_state:
                    buzzer.beep(0.14, 1)
                else:
                    buzzer.beep(0.07, 2)
            self.powered = new_state
            self.vx, self.turn = 0, 0
            self.posture = "STAND"
            self.last_seen = time.time()
            return True

    def set_oled_screen(self, screen, sid=""):
        with self.lock:
            if not self._is_pilot_locked(sid):
                return False
            if screen in ("EYES", "BATTERY", "RADAR", "HUD"):
                self.oled_screen = screen
            return True

    def update_sensors(self, distance_mm=None, pitch_deg=None):
        """Met à jour les valeurs capteurs réelles (appelé par sensor_loop ou par ton code matériel)."""
        with self.lock:
            updated = False
            if distance_mm is not None:
                self.distance_mm = max(20, min(2000, int(distance_mm)))
                updated = True
            if pitch_deg is not None:
                self.pitch_deg = round(float(pitch_deg), 1)
                updated = True
            if updated:
                self.sensors_live = True

    @staticmethod
    def _oled_mode(powered, vx, turn, posture):
        if not powered:
            return "OFF"
        if posture in ("STOP", "SIT"):
            return posture
        if vx > 0:
            return "WALK"
        if vx < 0:
            return "BACK"
        if turn > 0:
            return "TURN_LEFT"
        if turn < 0:
            return "TURN_RIGHT"
        return "STAND"

    def get_oled_state(self):
        with self.lock:
            return {
                "powered": self.powered,
                "vx": self.vx,
                "turn": self.turn,
                "mode": self._oled_mode(self.powered, self.vx, self.turn, self.posture),
                "oled_screen": self.oled_screen,
                "distance_mm": self.distance_mm,
                "pitch_deg": self.pitch_deg,
            }

    def get_telemetry(self, sid=""):
        with self.lock:
            if sid:
                self._label_for_sid_locked(sid)
            is_pilot = self._is_pilot_locked(sid)
            is_admin = self._is_admin_locked(sid)
            lease_left = max(0.0, round(LEASE_TIMEOUT - (time.time() - self.pilot_seen), 1)) if self.pilot_id else 0.0
            req_info = None
            if self.pending_request:
                req_info = {
                    "label": self.pending_request["label"],
                    "is_me": bool(sid and self.pending_request["sid"] == sid),
                }
            return {
                "powered": self.powered,
                "vx": self.vx,
                "turn": self.turn,
                "mode": self._oled_mode(self.powered, self.vx, self.turn, self.posture),
                "oled_screen": self.oled_screen,
                "distance_mm": self.distance_mm,
                "pitch_deg": self.pitch_deg,
                "sensors_live": self.sensors_live,
                "pilot_active": self.pilot_id is not None,
                "pilot_label": self.pilot_label,
                "is_pilot": is_pilot,
                "is_admin": is_admin,
                "admin_online": len(self.admins) > 0,
                "lease_left": lease_left,
                "pending_request": req_info,
            }

    def apply(self, vx, turn, posture):
        pass  # TODO : brancher ici le générateur de marche (PCA9685 / IK)

    def sensor_loop(self):
        hw = HardwareSensors()
        while True:
            data = hw.read()
            if data:
                self.update_sensors(**data)
            time.sleep(1.0 / SENSOR_HZ)

    def loop(self):
        prev = None
        while True:
            with self.lock:
                self._cleanup_locked()
                if (self.vx != 0 or self.turn != 0) and (time.time() - self.last_seen > TIMEOUT):
                    self.vx, self.turn = 0, 0
                cmd = (self.vx, self.turn, self.posture if self.powered else "OFF")
            if cmd != prev:
                print(f"commande : avance={cmd[0]:+d} rotation={cmd[1]:+d} posture={cmd[2]}")
                prev = cmd
            self.apply(*cmd)
            time.sleep(1.0 / LOOP_HZ)


robot = Robot()


def clamp(v):
    return max(-1, min(1, int(v)))


def client_label():
    ip = request.remote_addr or "client"
    return f"User #{ip.split('.')[-1]}" if "." in ip else f"User #{ip}"


@app.post("/control")
def control():
    data = request.get_json(silent=True) or {}
    sid = str(data.get("sid", ""))
    act = str(data.get("action", "")).lower()
    lbl = client_label()

    if act == "admin_unlock":
        ok, msg = robot.admin_unlock(sid, data.get("pin", ""), label=lbl, mode="admin")
    elif act == "guest_unlock":
        ok, msg = robot.admin_unlock(sid, data.get("pin", ""), label=lbl, mode="guest")
    elif act == "admin_lock":
        ok, msg = robot.admin_lock(sid)
    elif act == "takeover":
        ok, msg = robot.take_control(sid, label=lbl)
    elif act == "release":
        ok, msg = robot.release_control(sid)
    elif act == "request":
        ok, msg = robot.request_control(sid, label=lbl)
    elif act == "approve":
        ok, msg = robot.respond_request(sid, approve=True)
    elif act == "deny":
        ok, msg = robot.respond_request(sid, approve=False)
    else:
        ok, msg = False, "Unknown control action"

    return jsonify(ok=ok, msg=msg, **robot.get_telemetry(sid))


@app.post("/cmd")
def cmd():
    data = request.get_json(silent=True) or {}
    sid = str(data.get("sid", ""))
    ok = robot.set_command(clamp(data.get("vx", 0)), clamp(data.get("turn", 0)), sid=sid)
    return jsonify(ok=ok, **robot.get_telemetry(sid))


@app.post("/action")
def action():
    data = request.get_json(silent=True) or {}
    sid = str(data.get("sid", ""))
    ok = True
    if "power" in data:
        ok = robot.set_power(data["power"], sid=sid) and ok
    if "posture" in data:
        ok = robot.set_posture(str(data["posture"]).upper(), sid=sid, label=client_label()) and ok
    if "oled_screen" in data:
        ok = robot.set_oled_screen(str(data["oled_screen"]).upper(), sid=sid) and ok
    return jsonify(ok=ok, **robot.get_telemetry(sid))


@app.get("/state")
def state():
    sid = request.args.get("sid", "")
    return jsonify(robot.get_telemetry(sid))


HERE = os.path.dirname(os.path.abspath(__file__))


@app.get("/")
def index():
    if not os.path.exists(os.path.join(HERE, "index.html")):
        return f"index.html est introuvable dans le dossier : {HERE}", 404
    return send_from_directory(HERE, "index.html")


def lan_ip():
    """Adresse du PC / de la Pi sur le Wi-Fi (aucun paquet n'est envoyé)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


@app.get("/info")
def info():
    return jsonify(url=f"http://{lan_ip()}:5000")


def oled_loop():
    """Sur le thread principal pour garder la fenêtre Pygame fluide."""
    emulator = device is not None and not IS_HARDWARE
    if emulator:
        import pygame
    try:
        while True:
            if emulator:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return
            try:
                oled.update(**robot.get_oled_state())
                oled.render()
            except Exception as e:
                print(f"[OLED WARNING] {e}")
            time.sleep(1.0 / OLED_FPS)
    except KeyboardInterrupt:
        pass
    finally:
        if emulator:
            pygame.quit()


if __name__ == "__main__":
    buzzer.beep(0.08, 2)
    threading.Thread(target=robot.loop, daemon=True).start()
    threading.Thread(target=robot.sensor_loop, daemon=True).start()
    threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5000, threaded=True, use_reloader=False),
        daemon=True,
    ).start()

    if device is None:
        mode_str = "PAGE WEB SEULE (pas d'écran OLED)"
    else:
        mode_str = "MATÉRIEL I2C (Raspberry Pi 5)" if IS_HARDWARE else "SIMULATEUR PYGAME (PC)"
    print(f"=== ROBODOG PRÊT [{mode_str}] | ADMIN PIN CONFIGURED ===")
    print("Dossier :", HERE)
    print(f"Sur ton téléphone (même Wi-Fi) : http://{lan_ip()}:5000")
    print("Ouvre http://127.0.0.1:5000 (ou http://<IP_DE_LA_PI>:5000) dans ton navigateur.")
    oled_loop()
    os._exit(0)