#!/usr/bin/env python3
# pm_daemon.py — Event-driven LED daemon (FIFO version, auto-serial)
# Reads JSON lines from /tmp/pm.fifo and drives Plasma 2040 bridge:
#   frame = b"multiverse:data" + N*(B,G,R,br)

import os, sys, time, json, math, signal, select

# ---------- CONFIG ----------
NUM_LEDS = 7
ORDER = list(range(NUM_LEDS))      # change if your physical order differs
BRIGHT_LIMIT = 170                 # cap brightness (0..255)
FPS = 60
FIFO_PATH = "/tmp/pm.fifo"
SYSTEMS_JSON = "/recalbox/share/pixel-multiverse/systems.json"
ES_STATE = "/tmp/es_state.inf"
HEADER = b"multiverse:data"
# --------------------------------

# pyserial (installed via pip --target /recalbox/share/pythonlibs)
USER_SITE = "/recalbox/share/pythonlibs"
if os.path.isdir(USER_SITE) and USER_SITE not in sys.path:
    sys.path.insert(0, USER_SITE)
try:
    import serial   # type: ignore
except Exception as e:
    print("[pm] FATAL: pyserial not available:", e, flush=True)
    sys.exit(1)

running = True
def _stop(*_):
    global running
    running = False
signal.signal(signal.SIGINT, _stop)
signal.signal(signal.SIGTERM, _stop)

def log(*a): print("[pm]", *a, flush=True)
def _clamp(x, lo=0, hi=255): return lo if x < lo else hi if x > hi else x
def lerp(a,b,t): return a + (b-a)*t

def pack_colors(cols):
    payload = bytearray()
    for (b,g,r,br) in cols:
        payload += bytes((_clamp(b), _clamp(g), _clamp(r), _clamp(br)))
    return HEADER + payload

def send_colors(ser, cols):
    mapped = [cols[src] if src < len(cols) else (0,0,0,0) for src in ORDER]
    ser.write(pack_colors(mapped)); ser.flush()

def all_off(): return [(0,0,0,0)] * NUM_LEDS
def solid(b,g,r,br): return [(b,g,r,_clamp(min(br,BRIGHT_LIMIT)))] * NUM_LEDS

def read_es_state(path=ES_STATE):
    out = {}
    try:
        with open(path, "r") as f:
            for line in f:
                if "=" in line:
                    k,v = line.strip().split("=",1)
                    out[k.strip()] = v.strip()
    except Exception:
        pass
    return out

# ---------- systems.json ----------
_cfg = {}
def load_config():
    global _cfg
    try:
        with open(SYSTEMS_JSON, "r") as f:
            _cfg = json.load(f)
        log("loaded systems.json:", ",".join(sorted(_cfg.keys())))
    except Exception as e:
        log("systems.json not loaded:", e)
        _cfg = {}

def get_system_key(evt):
    sysid = (evt.get("system") or "").lower()
    if sysid: return sysid
    st = read_es_state()
    return (st.get("SystemId") or st.get("System") or "").lower()

def get_rom_key(evt):
    rp = evt.get("rom") or evt.get("rompath") or ""
    if not rp:
        st = read_es_state(); rp = st.get("RomPath","")
    base = os.path.basename(rp); name,_ = os.path.splitext(base)
    return name

# ---------- frames ----------
def breath_frame(t, color=(0,0,255,40), speed=1.2, minf=0.2, maxf=1.0):
    bb,bg,br,bbr = color
    f = (math.sin(t*speed)+1.0)/2.0
    f = minf + (maxf-minf)*f
    return [(bb,bg,br,_clamp(int(bbr*f))) for _ in range(NUM_LEDS)]

def wipe_frames(color=(0,64,64,BRIGHT_LIMIT), step_ms=50):
    for i in range(NUM_LEDS):
        cols = all_off()
        for k in range(i+1): cols[k] = color
        yield cols; time.sleep(step_ms/1000)

def fade_all(from_lvl=40, to_lvl=0, ms_total=700):
    steps = max(1, int(ms_total/20))
    for s in range(steps+1):
        lvl = _clamp(int(lerp(from_lvl, to_lvl, s/steps)))
        yield solid(0,0,0,lvl); time.sleep(0.02)

# ---------- layouts from config ----------
def cols_from_layout(layout):
    cols=[]
    for i in range(min(NUM_LEDS, len(layout))):
        item = layout[i]
        if isinstance(item, dict):
            r=int(item.get("r",0)); g=int(item.get("g",0)); b=int(item.get("b",0)); br=int(item.get("br",0))
            cols.append((b,g,r,_clamp(min(br,BRIGHT_LIMIT))))
        elif isinstance(item, str):
            s=item.strip(); br=64
            if ":" in s:
                s,brs = s.split(":",1)
                try: br=int(brs)
                except: br=64
            if s.startswith("#") and len(s)==7:
                r=int(s[1:3],16); g=int(s[3:5],16); b=int(s[5:7],16)
                cols.append((b,g,r,_clamp(min(br,BRIGHT_LIMIT))))
            else:
                cols.append((0,0,0,0))
        else:
            cols.append((0,0,0,0))
    while len(cols) < NUM_LEDS: cols.append((0,0,0,0))
    return cols

def lookup_start_layout(system_key, rom_key):
    syscfg = _cfg.get(system_key or "", {})
    if syscfg and "rom_overrides" in syscfg:
        ro = syscfg["rom_overrides"].get(rom_key or "", None)
        if ro and "start_layout" in ro: return cols_from_layout(ro["start_layout"])
    if syscfg and "start_layout" in syscfg: return cols_from_layout(syscfg["start_layout"])
    return None

def system_accent(system_key):
    syscfg = _cfg.get(system_key or "", {})
    c = syscfg.get("accent")
    if isinstance(c, dict):
        b=int(c.get("b",0)); g=int(c.get("g",0)); r=int(c.get("r",0)); br=_clamp(min(int(c.get("br",24)),BRIGHT_LIMIT))
        return (b,g,r,br)
    return None

def default_menu_color():
    d=_cfg.get("defaults",{}); c=d.get("menu_color")
    if isinstance(c, dict):
        b=int(c.get("b",0)); g=int(c.get("g",0)); r=int(c.get("r",0)); br=_clamp(min(int(c.get("br",24)),BRIGHT_LIMIT))
        return (b,g,r,br)
    return (0,32,64,28)

def default_attract_mode():
    d=_cfg.get("defaults",{}); return (d.get("attract") or "breath").lower()

# ---------- event animations ----------
def anim_menu_pulse(ser, accent=None, seconds=2.0):
    base = accent if accent else default_menu_color()
    t0=time.monotonic()
    while (time.monotonic()-t0) < seconds:
        cols = breath_frame(time.monotonic(), color=base, speed=1.0, minf=0.3, maxf=0.9)
        send_colors(ser, cols); time.sleep(1.0/FPS)

def anim_game_start(ser, system_key=None, rom_key=None):
    layout = lookup_start_layout(system_key, rom_key)
    if layout:
        send_colors(ser, layout); time.sleep(1.0); return
    accent = system_accent(system_key) or (0,64,0,BRIGHT_LIMIT)
    for cols in wipe_frames(color=accent, step_ms=40): send_colors(ser, cols)
    send_colors(ser, solid(0,0,0,18)); time.sleep(0.25)

def anim_game_end(ser):
    for cols in wipe_frames(color=(64,0,0,BRIGHT_LIMIT), step_ms=40): send_colors(ser, cols)
    for cols in fade_all(from_lvl=28, to_lvl=10, ms_total=600): send_colors(ser, cols)

def anim_shutdown(ser):
    for _ in range(3):
        send_colors(ser, solid(0,0,0,BRIGHT_LIMIT)); time.sleep(0.08)
        send_colors(ser, solid(0,0,0,8)); time.sleep(0.1)
    for cols in fade_all(from_lvl=24, to_lvl=0, ms_total=900): send_colors(ser, cols)

def anim_reboot(ser):
    for cols in wipe_frames(color=(0,64,0,BRIGHT_LIMIT), step_ms=35): send_colors(ser, cols)
    for cols in wipe_frames(color=(64,0,0,BRIGHT_LIMIT), step_ms=35): send_colors(ser, cols)
    for cols in fade_all(from_lvl=28, to_lvl=0, ms_total=500): send_colors(ser, cols)

def anim_settings_changed(ser):
    col=(32,32,0,BRIGHT_LIMIT)
    for _ in range(3):
        send_colors(ser, solid(*col)); time.sleep(0.12)
        send_colors(ser, all_off());   time.sleep(0.08)

def idle_menu(accent=None):
    base = accent if accent else default_menu_color()
    t0=time.monotonic()
    while True:
        yield breath_frame(time.monotonic()-t0, color=base, speed=0.8, minf=0.2, maxf=0.8)

def idle_attract(mode="breath"):
    t0=time.monotonic()
    if mode == "rainbow":
        while True:
            t=time.monotonic()-t0; cols=[]
            for i in range(NUM_LEDS):
                hue = (t*0.05 + i/NUM_LEDS) % 1.0
                h6 = hue*6.0; k=int(h6); f=h6-k; v=24; p=0; q=int(v*(1.0-f)); tt=int(v*f)
                if   k==0: rgb=(v,tt,p)
                elif k==1: rgb=(q,v,p)
                elif k==2: rgb=(p,v,tt)
                elif k==3: rgb=(p,q,v)
                elif k==4: rgb=(tt,p,v)
                else:      rgb=(v,p,q)
                cols.append((rgb[0], rgb[1], rgb[2], 20))
            yield cols
    else:
        while True:
            yield breath_frame(time.monotonic()-t0, color=(0,0,0,28), speed=0.6, minf=0.15, maxf=0.6)

# ---------- FIFO helpers ----------
def ensure_fifo(path=FIFO_PATH):
    try:
        if os.path.exists(path):
            if not stat_is_fifo(path):
                os.remove(path)
        if not os.path.exists(path):
            os.mkfifo(path, 0o666)
            os.chmod(path, 0o666)
    except Exception as e:
        log("mkfifo failed:", e)

def stat_is_fifo(path):
    try:
        import stat
        m = os.stat(path).st_mode
        return stat.S_ISFIFO(m)
    except Exception:
        return False

def open_fifo_reader(path=FIFO_PATH):
    rfd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    rdr = os.fdopen(rfd, 'r', buffering=1)  # line-buffered
    dummy_w = open(path, 'w')               # keep writer to prevent EOF
    return rdr, dummy_w

# ---------- Auto-detect serial port ----------
def find_serial_port():
    # Allow override
    env = os.environ.get("PM_PORT")
    if env and os.path.exists(env):
        return env
    byid = "/dev/serial/by-id"
    cand = []
    try:
        for name in os.listdir(byid):
            low = name.lower()
            if ("picade" in low or "pimoroni" in low or "max" in low):
                cand.append(os.path.join(byid, name))
    except Exception:
        pass
    # Prefer data interface (often -if01) then console (-if00)
    cand.sort(key=lambda p: (("if02" not in p), p))
    for p in cand:
        try:
            s = serial.Serial(p, 115200, timeout=0.1)
            s.close()
            return p
        except Exception:
            continue
    # Fallback to common ACM paths
    for p in ("/dev/ttyACM0", "/dev/ttyACM1"):
        if os.path.exists(p):
            try:
                s = serial.Serial(p, 115200, timeout=0.1); s.close(); return p
            except Exception:
                pass
    return None

# ---------- Main ----------
def main():
    load_config()
    ensure_fifo()

    port = find_serial_port()
    if not port:
        log("ERROR: no serial port found (is the Picade Max connected & code.py running?)")
        return
    ser = serial.Serial(port, 115200, timeout=0.05)
    log("daemon started; PORT =", port, "FIFO =", FIFO_PATH)

    current_idle = idle_menu()
    last_idle = 0.0

    rdr, dummy_w = open_fifo_reader()
    poll = select.poll()
    poll.register(rdr, select.POLLIN)

    try:
        while running:
            events = poll.poll(50)  # 50ms
            if events:
                try:
                    line = rdr.readline()
                except Exception:
                    line = ""
                if line:
                    line = line.strip()
                    if line:
                        try:
                            evt = json.loads(line)
                        except Exception:
                            evt = {}
                        name = (evt.get("event") or "").lower()

                        if name == "reload-config":
                            load_config()

                        else:
                            syskey = get_system_key(evt)
                            romkey = get_rom_key(evt)

                            if name == "menu":
                                accent = system_accent(syskey)
                                anim_menu_pulse(ser, accent=accent, seconds=2.0)
                                current_idle = idle_menu(accent=accent)

                            elif name == "game-start":
                                anim_game_start(ser, system_key=syskey, rom_key=romkey)
                                current_idle = idle_menu(accent=system_accent(syskey))

                            elif name == "game-end":
                                anim_game_end(ser)
                                current_idle = idle_menu(accent=system_accent(syskey))

                            elif name == "shutdown":
                                anim_shutdown(ser)

                            elif name == "reboot":
                                anim_reboot(ser)

                            elif name in ("settings-changed","controls-changed"):
                                anim_settings_changed(ser)

                            elif name == "attract-on":
                                current_idle = idle_attract(mode=default_attract_mode())

                            elif name == "attract-off":
                                current_idle = idle_menu(accent=system_accent(syskey))

                            elif name == "solid":
                                b=int(evt.get("b",0)); g=int(evt.get("g",0)); r=int(evt.get("r",0)); br=int(evt.get("br",24))
                                send_colors(ser, solid(b,g,r,br))

                            elif name == "off":
                                send_colors(ser, all_off())

                        last_idle = 0.0  # next idle immediately

            if time.monotonic() - last_idle >= (1.0/30.0):
                try: cols = next(current_idle)
                except StopIteration:
                    current_idle = idle_menu(); cols = next(current_idle)
                send_colors(ser, cols); last_idle = time.monotonic()

    finally:
        try: send_colors(ser, all_off()); ser.close()
        except Exception: pass
        try:
            poll.unregister(rdr); rdr.close(); dummy_w.close()
            if os.path.exists(FIFO_PATH): os.chmod(FIFO_PATH, 0o666)
        except Exception: pass
        log("daemon stopped")

if __name__ == "__main__":
    main()
PY
chmod +x /recalbox/share/pixel-multiverse/pm_daemon.py
