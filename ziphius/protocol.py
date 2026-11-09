"""Protocollo seriale del firmware di Claudio: costruzione dei comandi e lettura delle risposte.

Pacchetto: A<cmd>X<cmd>X...Z. Il firmware risponde ACK <n> a ogni pacchetto,
IDLE quando coda e servo sono fermi, READY a fine avvio.
"""

import re
from dataclasses import dataclass, field

START, SEP, END = "A", "X", "Z"
STOP_PACKET = "AstopZ"  # gestito dal firmware fuori dalla coda

NUM_MOTORS = 4
# Limiti del firmware (motorLimits)
LIMITS = {0: (0, 160), 1: (0, 180), 2: (30, 130), 3: (30, 100)}
MOTOR_NAMES = {0: "Laser (verticale)", 1: "Corpo (orizzontale)", 2: "Braccio", 3: "Pinza"}
GESTURES = ("hello", "shy", "yes", "no")

MAX_CMDS_PER_PACKET = 4
MAX_PACKET_LEN = 100  # il buffer seriale del firmware e' di 128 caratteri
MAX_CMD_LEN = 32


def clamp_angle(motor, angle):
    lo, hi = LIMITS[motor]
    return max(lo, min(hi, int(round(angle))))


def move(motor, angle, force=False):
    """Comando di movimento: "<m><ang>" oppure "F<m><ang>" (ignora la soglia DELTA_ANG)."""
    if motor not in LIMITS:
        raise ValueError(f"motore {motor} inesistente")
    return f"{'F' if force else ''}{motor}{clamp_angle(motor, angle)}"


def laser(on):
    return "L1" if on else "L0"


def validate(cmd):
    """Un comando non puo' contenere i marcatori di separazione e fine pacchetto."""
    cmd = cmd.strip()
    if not cmd:
        raise ValueError("comando vuoto")
    if SEP in cmd or END in cmd:
        raise ValueError(f"'{cmd}': i caratteri {SEP} e {END} sono riservati al protocollo")
    if len(cmd) > MAX_CMD_LEN:
        raise ValueError(f"'{cmd}': troppo lungo")
    return cmd


def split_commands(text):
    """"P1lXw500XP1c" -> ["P1l", "w500", "P1c"] (accetta anche A...Z attorno)."""
    text = text.strip()
    if text.startswith(START) and text.endswith(END):
        text = text[1:-1]
    return [validate(c) for c in text.split(SEP) if c.strip()]


def packets(cmds, max_cmds=MAX_CMDS_PER_PACKET, max_len=MAX_PACKET_LEN):
    """Raggruppa i comandi in pacchetti A...Z rispettando numero e lunghezza massimi."""
    out, cur = [], []
    for cmd in cmds:
        cmd = validate(cmd)
        candidate = cur + [cmd]
        if cur and (len(candidate) > max_cmds or len(encode(candidate)) > max_len):
            out.append(encode(cur))
            cur = [cmd]
        else:
            cur = candidate
    if cur:
        out.append(encode(cur))
    return out


def encode(cmds):
    return START + SEP.join(cmds) + END


# ---------------------------------------------------------------------------
# Risposte del firmware
# ---------------------------------------------------------------------------

@dataclass
class Event:
    kind: str
    data: dict = field(default_factory=dict)
    line: str = ""


_PATTERNS = [
    ("ack", re.compile(r"^ACK (\d+)$"), lambda m: {"queue": int(m[1])}),
    ("moved", re.compile(r"^Motor (\d) moved to (\d+)$"), lambda m: {"motor": int(m[1]), "angle": int(m[2])}),
    ("touch", re.compile(r"^TOUCH (-?\d+)$"), lambda m: {"distance": int(m[1])}),
    ("presence", re.compile(r"^PRESENCE (-?\d+)$"), lambda m: {"distance": int(m[1])}),
    ("distance", re.compile(r"^Distance: (-?\d+) cm$"), lambda m: {"distance": int(m[1])}),
    ("gesture", re.compile(r"^Gesture: (\w+)$"), lambda m: {"name": m[1]}),
    ("delta", re.compile(r"^DELTA_ANG: (\d+)$"), lambda m: {"value": int(m[1])}),
    ("guard", re.compile(r"^Auto restore interval: (\d+) ms$"), lambda m: {"value": int(m[1])}),
    ("error", re.compile(r"^ERROR (.*)$"), lambda m: {"message": m[1]}),
]

_SIMPLE = {
    "READY": "ready",
    "IDLE": "idle",
    "STOPPED": "stopped",
    "GONE": "gone",
    "Laser ON": "laser_on",
    "Laser OFF": "laser_off",
    "Laser safety timeout.": "laser_timeout",
    "Random movement executed.": "random_move",
    "Periodic mode activated.": "periodic_on",
    "Periodic mode disabled.": "periodic_disabled",
    "Periodic mode enabled after inactivity.": "periodic_enabled",
    "Touch enabled.": "touch_enabled",
    "Touch disabled.": "touch_disabled",
}


def parse_line(line):
    """Trasforma una riga del firmware in un Event (kind "info" se non riconosciuta)."""
    line = line.strip()
    if line in _SIMPLE:
        return Event(_SIMPLE[line], {}, line)
    if line.startswith("Positions:"):
        return Event("positions", _parse_positions(line), line)
    for kind, rx, conv in _PATTERNS:
        m = rx.match(line)
        if m:
            return Event(kind, conv(m), line)
    return Event("info", {}, line)


def _parse_positions(line):
    # "Positions: M0:80 M1:110 M2:50 M3:65 L:0 D:-1 R:0 E:1 T:1 G:20000"
    data = {"positions": [None] * NUM_MOTORS}
    for key, val in re.findall(r"(\w+):(-?\d+)", line):
        if key.startswith("M") and key[1:].isdigit():
            data["positions"][int(key[1:])] = int(val)
        else:
            data[key] = int(val)
    return data
