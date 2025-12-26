# code.py — Plasma 2040 (CircuitPython) startup animation + Multiverse bridge
# Uses the same API style as your working plasma2040_Rainbow.py:
#   - NeoPixel strip on board.DATA
#   - Onboard RGB via adafruit_rgbled.RGBLED(board.LED_R/G/B, invert_pwm=True)
#
# Protocol from Pi:
#   HEADER = b"multiverse:data"
#   PAYLOAD = N * (B, G, R, brightness)   (brightness 0..255 scales RGB)

import time
import board
import neopixel
import usb_cdc
import adafruit_rgbled

# ---------- Onboard RGB LED (status / heartbeat) ----------
led = adafruit_rgbled.RGBLED(board.LED_R, board.LED_G, board.LED_B, invert_pwm=True)

def flash(r, g, b, ms=120):
    led.color = (r, g, b)
    time.sleep(ms / 1000.0)
    led.color = (0, 0, 0)

def boot_blink(times=3):
    for _ in range(times):
        flash(0, 0, 255, 250)  # BLUE
        time.sleep(0.25)

def heartbeat(ms=80):
    flash(0, 255, 0, ms)      # GREEN

boot_blink()

# ---------- NeoPixel strip on DA screw terminal ----------
# Matches your example: brightness + auto_write=False
BRIGHTNESS = 0.5
ORDER = neopixel.GRB
DATA_PIN = getattr(board, "DATA", getattr(board, "GP15"))

strip = None
current_n = 0
MAX_N = 1024

def ensure_strip(n: int):
    """(Re)create the strip when N changes."""
    global strip, current_n
    if n <= 0:
        return
    n = min(n, MAX_N)
    if strip is None or n != current_n:
        strip = neopixel.NeoPixel(DATA_PIN, n, brightness=BRIGHTNESS,
                                  auto_write=False, pixel_order=ORDER)
        current_n = n

def set_rgb(i, r, g, b):
    strip[i] = (r & 0xFF, g & 0xFF, b & 0xFF)

# ---------- Startup animation for your first 7 LEDs ----------
def startup_animation(first_n=7, dwell=0.5, fade_steps=60, fade_ms=20, white=255):
    ensure_strip(first_n)
    if strip is None:
        return

    # all off
    for i in range(first_n):
        set_rgb(i, 0, 0, 0)
    strip.show()

    # per-LED: red -> green -> blue -> stay white
    for i in range(first_n):
        set_rgb(i, white, 0, 0); strip.show(); time.sleep(dwell)
        set_rgb(i, 0, white, 0); strip.show(); time.sleep(dwell)
        set_rgb(i, 0, 0, white); strip.show(); time.sleep(dwell)
        set_rgb(i, white, white, white); strip.show(); time.sleep(dwell)

    # fade all to off
    for step in range(fade_steps, -1, -1):
        lvl = int((white * step) / fade_steps)
        for i in range(first_n):
            set_rgb(i, lvl, lvl, lvl)
        strip.show()
        time.sleep(fade_ms / 1000.0)

try:
    startup_animation(first_n=7, dwell=0.5, fade_steps=60, fade_ms=20, white=255)
except Exception:
    # don’t let any animation error kill the bridge
    pass

# ---------- Multiverse USB bridge ----------
HEADER = b"multiverse:data"
ser = usb_cdc.data if getattr(usb_cdc, "data", None) else usb_cdc.console
ser.timeout = 0.01
buf = bytearray()

def apply_frame(payload: bytes) -> bool:
    """payload = N * (B,G,R,brightness)."""
    if not payload or (len(payload) % 4) != 0:
        return False
    n = len(payload) // 4
    ensure_strip(n)
    if strip is None:
        return False

    idx = 0
    for i in range(n):
        b = payload[idx + 0]
        g = payload[idx + 1]
        r = payload[idx + 2]
        br = payload[idx + 3]
        idx += 4
        if br == 0:
            set_rgb(i, 0, 0, 0)
        else:
            s = br / 255.0
            set_rgb(i, int(r * s), int(g * s), int(b * s))
    strip.show()
    heartbeat(60)
    return True

while True:
    # read bytes (data port preferred; falls back to console if data not enabled)
    if ser and ser.in_waiting:
        chunk = ser.read(min(ser.in_waiting, 4096)) or b""
        if chunk:
            buf += chunk

    # parse HEADER ... payload ... [HEADER ...]
    while True:
        i = buf.find(HEADER)
        if i < 0:
            if len(buf) > len(HEADER):
                buf = buf[-len(HEADER):]   # keep short tail in case header split
            break
        if i > 0:
            buf = buf[i:]
        j = buf.find(HEADER, len(HEADER))
        if j < 0:
            payload = buf[len(HEADER):]
            if payload and (len(payload) % 4) == 0:
                apply_frame(payload)
                buf = bytearray()
            break
        else:
            payload = buf[len(HEADER):j]
            if payload and (len(payload) % 4) == 0:
                apply_frame(payload)
            buf = buf[j:]

    time.sleep(0.002)
