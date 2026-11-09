"""Configurazione persistente di Ziphius (JSON).

Percorso: $ZIPHIUS_CONFIG, altrimenti ~/.config/ziphius/config.json.
Il file contiene il PIN della pagina web: viene salvato con permessi 600.
"""

import copy
import json
import os
import secrets
import tempfile

DEFAULTS = {
    "serial_port": "auto",       # "auto" = prima porta /dev/ttyACM*, /dev/ttyUSB*, ...
    "baudrate": 9600,
    "host": "0.0.0.0",           # raggiungibile in LAN (protetto da PIN)
    "port": 8042,
    "pin": None,                 # generato al primo avvio
    "delta_ang": 1,              # soglia del firmware impostata alla connessione (1 = controllo fine)
    "on_connect": ["read"],      # comandi inviati dopo READY
    "poses": {
        "riposo": [80, 110, 50, 65],
    },
    # Geometria della parete (vedi geometry.WallProjector)
    "wall": {
        "K": 3.0,
        "pan_center": 70,
        "tilt_center": 90,
        "pan_gain": 1.0,
        "tilt_gain": 1.0,
        "arm_pose": 50,
        "step_deg": 2.0,
        "max_active_servos": 1,
    },
    # Limiti di sicurezza per il disegno (gradi): sottoinsieme dei limiti del firmware
    "draw_limits": {"0": [70, 110], "1": [30, 110]},
}


def default_path():
    env = os.environ.get("ZIPHIUS_CONFIG")
    if env:
        return env
    return os.path.join(os.path.expanduser("~"), ".config", "ziphius", "config.json")


def _merge(base, extra):
    out = copy.deepcopy(base)
    for k, v in extra.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict) and k != "poses":
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


class Config(dict):
    def __init__(self, path=None, data=None):
        super().__init__(_merge(DEFAULTS, data or {}))
        self.path = path

    @classmethod
    def load(cls, path=None):
        path = path or default_path()
        data = {}
        if os.path.exists(path):
            with open(path) as f:
                data = json.load(f)
        cfg = cls(path, data)
        # Calibrazione della V3, se presente nella cartella corrente e non ancora importata
        if "wall" not in data and os.path.exists("claudio_wall.json"):
            with open("claudio_wall.json") as f:
                cfg["wall"].update(json.load(f))
        if not cfg.get("pin"):
            cfg["pin"] = f"{secrets.randbelow(10**6):06d}"
            cfg.save()
        return cfg

    def save(self):
        if not self.path:
            return
        d = os.path.dirname(self.path) or "."
        os.makedirs(d, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=d, prefix=".ziphius-", suffix=".json")
        with os.fdopen(fd, "w") as f:
            json.dump(dict(self), f, indent=2, ensure_ascii=False)
        os.chmod(tmp, 0o600)
        os.replace(tmp, self.path)
