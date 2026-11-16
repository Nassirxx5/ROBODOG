"""Affichage OLED 1.3" du Robodog (matériel I2C sur la Pi, simulateur Pygame sur PC).
Sans luma / pygame, device reste None et render() ne fait rien."""
import random
import threading
import time

IS_HARDWARE = False
device = None
canvas = None
FPS = 25
try:
    from luma.core.render import canvas
    try:
        from luma.core.interface.serial import i2c
        from luma.oled.device import sh1106, ssd1306
        serial = i2c(port=1, address=0x3C)
        try:
            device = sh1106(serial, width=128, height=64, rotate=0)
        except Exception:
            device = ssd1306(serial, width=128, height=64, rotate=0)
        IS_HARDWARE = True
        FPS = 10  # limite I2C pour ne pas ralentir la boucle servos 50 Hz
    except Exception:
        import pygame
        from luma.emulator.device import pygame as oled_emulator
        device = oled_emulator(width=128, height=64, mode="RGB", transform="none", scale=4)
        pygame.display.set_caption('ROBODOG - 1.3" OLED Simulator')
except Exception as e:
    print("OLED indisponible, mode page web seule :", e)
    device = None


def col(hex_color):
    """Couleur sur le simulateur PC, blanc sur l'OLED monochrome."""
    return "white" if IS_HARDWARE else hex_color


class RobodogOLED:
    def __init__(self):
        self._lock = threading.Lock()
        self.state = {"mode": "STAND", "oled_screen": "EYES", "oled_banner": "ROBODOG V1",
                      "battery_pct": 88, "distance_mm": 640, "pitch_deg": 0.0}
        self.cur_x, self.cur_y = 0.0, 0.0
        self.cur_w, self.cur_h = 32.0, 36.0
        self.target_x, self.target_y = 0.0, 0.0
        self.last_blink = time.time()
        self.next_blink = 3.2
        self.last_glance = time.time()
        self.is_blinking = False

    def update(self, **kwargs):
        with self._lock:
            self.state.update(kwargs)

    def _get_state(self):
        with self._lock:
            return dict(self.state)

    def _draw_battery(self, draw, st):
        pct = max(0, min(100, int(st["battery_pct"])))
        is_low = pct < 20
        bar_color = col("#ef4444") if is_low else col("#22c55e")
        voltage = 9.9 + (pct / 100.0) * (12.6 - 9.9)  # 3S LiPo : 9,9 V vide, 12,6 V pleine
        draw.rounded_rectangle((14, 12, 108, 44), radius=5, outline="white", width=2)
        draw.rectangle((108, 22, 114, 34), fill="white")
        blink_on = int(time.time() * 3) % 2 == 0
        if not is_low or blink_on:
            inner_w = max(4, int(86 * (pct / 100.0)))
            draw.rounded_rectangle((18, 16, 18 + inner_w, 40), radius=3, fill=bar_color)
        draw.text((48, 23), f"{pct}%", fill="white")
        footer = f"LOW BATT! ({voltage:.1f}V)" if is_low else f"PACK: {voltage:.1f}V (3S LiPo)"
        draw.text((12, 50), footer, fill=bar_color if is_low else "white")

    def _draw_radar(self, draw, st):
        dist = int(st["distance_mm"])
        danger = dist < 200
        bar_color = col("#ef4444") if danger else col("#38bdf8")
        bar_w = int(108 * (max(50, min(1000, dist)) / 1000.0))
        draw.text((10, 6), "VL53L0X FRONT RADAR", fill="white")
        draw.rectangle((10, 24, 118, 40), outline="white", width=1)
        draw.rectangle((12, 26, 12 + bar_w, 38), fill=bar_color)
        msg = f"{dist} mm - AUTO STOP!" if danger else f"CLEARANCE: {dist} mm"
        draw.text((10, 48), msg, fill=bar_color)

    def _draw_hud(self, draw, st):
        bat_col = col("#ef4444") if st["battery_pct"] < 20 else col("#22c55e")
        draw.rectangle(device.bounding_box, outline="white", fill="black")
        draw.text((6, 4), f"MODE : {st['mode']}", fill="white")
        draw.text((6, 18), f"BAT  : {st['battery_pct']}%", fill=bat_col)
        draw.text((6, 32), f"ToF  : {st['distance_mm']} mm | {st['pitch_deg']:+.1f}d", fill="white")
        draw.text((6, 48), f"> {st['oled_banner']}", fill=col("#38bdf8"))

    def render(self):
        if device is None:
            return
        st = self._get_state()
        screen = st["oled_screen"]
        with canvas(device) as draw:
            if st["mode"] == "OFF":
                return  # robot éteint : écran noir
            if screen == "BATTERY":
                return self._draw_battery(draw, st)
            if screen == "RADAR":
                return self._draw_radar(draw, st)
            if screen == "HUD":
                return self._draw_hud(draw, st)

            now = time.time()
            mode = st["mode"]
            if not self.is_blinking and (now - self.last_blink) > self.next_blink:
                self.is_blinking = True
                self.last_blink = now
            elif self.is_blinking and (now - self.last_blink) > 0.14:
                self.is_blinking = False
                self.next_blink = random.uniform(2.5, 4.8)

            target_w, target_h = 32.0, 36.0
            if mode == "TURN_LEFT":
                self.target_x, self.target_y = -9.0, 0.0
            elif mode == "TURN_RIGHT":
                self.target_x, self.target_y = 9.0, 0.0
            elif mode in ("WALK", "BACK"):
                self.target_x, self.target_y, target_h = 0.0, 2.0, 26.0
            elif mode == "SIT":
                self.target_x, self.target_y, target_h = 0.0, 4.0, 6.0
            elif mode == "STOP":
                draw.ellipse((18, 10, 54, 54), outline=col("#ef4444"), width=3)
                draw.ellipse((74, 10, 110, 54), outline=col("#ef4444"), width=3)
                draw.ellipse((32, 28, 40, 36), fill="white")
                draw.ellipse((88, 28, 96, 36), fill="white")
                return
            elif (now - self.last_glance) > 2.2:
                self.target_x = random.choice([-5.0, 0.0, 0.0, 5.0])
                self.target_y = random.choice([-2.0, 0.0, 2.0])
                self.last_glance = now

            if self.is_blinking:
                target_h = 3.0
            self.cur_x += (self.target_x - self.cur_x) * 0.35
            self.cur_y += (self.target_y - self.cur_y) * 0.35
            self.cur_w += (target_w - self.cur_w) * 0.35
            self.cur_h += (target_h - self.cur_h) * 0.45

            lx, rx, yc = 36 + self.cur_x, 92 + self.cur_x, 32 + self.cur_y
            hw, hh = self.cur_w / 2.0, max(1.5, self.cur_h / 2.0)
            r = min(8, int(hh))
            draw.rounded_rectangle((lx - hw, yc - hh, lx + hw, yc + hh), radius=r, fill="white")
            draw.rounded_rectangle((rx - hw, yc - hh, rx + hw, yc + hh), radius=r, fill="white")
            if mode in ("WALK", "BACK") and not self.is_blinking:
                draw.polygon([(18, 12), (54, 21), (54, 12)], fill="black")
                draw.polygon([(74, 21), (110, 12), (74, 12)], fill="black")
