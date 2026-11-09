"""Stato di Claudio ricostruito dalle risposte del firmware."""

from dataclasses import asdict, dataclass, field

from .protocol import NUM_MOTORS


@dataclass
class RobotState:
    connected: bool = False
    ready: bool = False
    port: str = ""
    positions: list = field(default_factory=lambda: [None] * NUM_MOTORS)
    laser: bool = False
    distance: int = -1
    zone: str = "far"            # far | presence | touch
    periodic: bool = False       # modalita' random attiva ora
    periodic_enabled: bool = True
    touch_enabled: bool = True
    guard_ms: int = 20000
    delta_ang: int = 10
    busy: bool = False
    last_error: str = ""

    def apply(self, ev):
        """Aggiorna lo stato con un Event. Ritorna True se e' cambiato qualcosa."""
        before = asdict(self)
        k, d = ev.kind, ev.data
        if k == "ready":
            self.ready = True
        elif k == "moved":
            self.positions[d["motor"]] = d["angle"]
            self.busy = True
        elif k == "laser_on":
            self.laser = True
        elif k in ("laser_off", "laser_timeout"):
            self.laser = False
        elif k == "touch":
            self.zone, self.distance, self.laser, self.periodic = "touch", d["distance"], False, False
        elif k == "presence":
            self.zone, self.distance, self.periodic = "presence", d["distance"], False
        elif k == "gone":
            self.zone = "far"
        elif k == "distance":
            self.distance = d["distance"]
        elif k == "positions":
            for i, a in enumerate(d["positions"]):
                if a is not None:
                    self.positions[i] = a
            self.laser = bool(d.get("L", self.laser))
            self.distance = d.get("D", self.distance)
            self.periodic = bool(d.get("R", self.periodic))
            self.periodic_enabled = bool(d.get("E", self.periodic_enabled))
            self.touch_enabled = bool(d.get("T", self.touch_enabled))
            self.guard_ms = d.get("G", self.guard_ms)
        elif k in ("periodic_on", "random_move"):
            self.periodic = True
            self.periodic_enabled = True
        elif k == "periodic_disabled":
            self.periodic = False
            self.periodic_enabled = False
        elif k == "periodic_enabled":
            self.periodic_enabled = True
        elif k == "touch_enabled":
            self.touch_enabled = True
        elif k == "touch_disabled":
            self.touch_enabled, self.zone = False, "far"
        elif k == "delta":
            self.delta_ang = d["value"]
        elif k == "guard":
            self.guard_ms = d["value"]
        elif k == "ack":
            self.busy = True
            self.periodic = False  # ogni pacchetto annulla la modalita' random forzata
        elif k == "idle":
            self.busy = False
        elif k == "stopped":
            self.laser = False
            self.periodic = False
        elif k == "error":
            self.last_error = d["message"]
        return asdict(self) != before

    def to_dict(self):
        return asdict(self)
