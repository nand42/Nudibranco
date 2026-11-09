"""Controller di Claudio: stato, comandi interattivi, lavori (sequenze lunghe) e arresto.

Regole di invio (un pacchetto alla volta, il successivo solo dopo IDLE):
1. comandi interattivi (jog, laser, gesti): hanno la precedenza e vengono accorpati;
   piu' jog sullo stesso motore mentre Claudio e' occupato diventano un solo movimento;
2. lavori (es. un disegno): un pacchetto per volta, sospendibili e annullabili tra un
   pacchetto e l'altro.
"""

import asyncio
import itertools
import logging
from dataclasses import dataclass, field

from . import protocol as P
from .link import LinkError, PySerialTransport, SerialLink, find_ports
from .state import RobotState

log = logging.getLogger("ziphius.controller")


@dataclass
class Job:
    name: str
    packets: list
    id: int = 0
    sent: int = 0
    status: str = "queued"   # queued | running | paused | done | cancelled | error
    error: str = ""
    _resume: asyncio.Event = field(default_factory=asyncio.Event)

    def to_dict(self):
        return {"id": self.id, "name": self.name, "sent": self.sent,
                "total": len(self.packets), "status": self.status, "error": self.error}


class Controller:
    def __init__(self, config, transport_factory=None):
        self.config = config
        self.state = RobotState()
        self.link = SerialLink(transport_factory or self._default_transport, self._on_event)
        self.listeners = set()          # callback(msg: dict) per la pagina web
        self.log_lines = []             # ultime righe per i nuovi client
        self.jobs = []
        self._ids = itertools.count(1)
        self._pending_moves = {}        # motore -> angolo (accorpati)
        self._commanded = {}            # motore -> ultimo angolo inviato o riportato dal firmware
        self._pending_cmds = []         # altri comandi interattivi, in ordine
        self._wake = None               # creati in start(): con Python 3.9 un asyncio.Event
        self._ready = None              # si lega al loop attivo alla creazione
        self._writer_task = None

    # ------------------------------------------------------------------ avvio
    def _default_transport(self):
        port = self.config["serial_port"]
        if port == "auto":
            ports = find_ports()
            if not ports:
                raise LinkError("nessuna porta seriale trovata")
            port = ports[0]
        return PySerialTransport(port, self.config["baudrate"])

    async def start(self):
        self._wake, self._ready = asyncio.Event(), asyncio.Event()
        self.link.start()
        self._writer_task = asyncio.create_task(self._writer())

    async def close(self):
        if self._writer_task:
            self._writer_task.cancel()
        self.link.close()

    # ------------------------------------------------------------------ eventi
    def _on_event(self, ev):
        if ev.kind == "connected":
            self.state.connected, self.state.port, self.state.ready = True, ev.data["port"], False
            self._ready.clear()
            self._log(f"collegato a {ev.data['port']}, attendo READY")
        elif ev.kind == "disconnected":
            self.state.connected = self.state.ready = False
            self._ready.clear()
            self._log(f"scollegato: {ev.data['reason']}")
        else:
            self._log(ev.line, ev.kind)
        if ev.kind == "moved":
            self._commanded[ev.data["motor"]] = ev.data["angle"]
        elif ev.kind == "positions":
            self._commanded.update({m: a for m, a in enumerate(ev.data["positions"]) if a is not None})
        changed = self.state.apply(ev)
        if ev.kind == "ready":
            self._ready.set()
            init = [f"delta{int(self.config['delta_ang'])}"] + list(self.config.get("on_connect", []))
            self._pending_cmds = init + self._pending_cmds
            self._wake.set()
        if ev.kind in ("connected", "disconnected") or changed:
            self._broadcast({"type": "state", "state": self.state.to_dict()})
        self._broadcast({"type": "event", "kind": ev.kind, "data": ev.data})

    def _log(self, line, kind="sys"):
        entry = {"type": "log", "line": line, "kind": kind}
        self.log_lines = (self.log_lines + [entry])[-200:]
        self._broadcast(entry)

    def _broadcast(self, msg):
        for cb in list(self.listeners):
            try:
                cb(msg)
            except Exception:
                log.exception("listener")

    def snapshot(self):
        return {"type": "snapshot", "state": self.state.to_dict(),
                "jobs": [j.to_dict() for j in self.jobs[-10:]],
                "poses": self.config["poses"], "log": self.log_lines[-100:],
                "limits": P.LIMITS, "gestures": P.GESTURES}

    # ------------------------------------------------------------------ comandi interattivi
    def target(self, motor):
        """Angolo verso cui sta andando il motore (pendente) o posizione nota."""
        if motor in self._pending_moves:
            return self._pending_moves[motor]
        if motor in self._commanded:
            return self._commanded[motor]
        pos = self.state.positions[motor]
        return pos if pos is not None else sum(P.LIMITS[motor]) // 2

    def set_angle(self, motor, angle):
        self._pending_moves[motor] = P.clamp_angle(motor, angle)
        self._wake.set()

    def jog(self, motor, delta):
        self.set_angle(motor, self.target(motor) + delta)

    def command(self, *cmds):
        """Comandi interattivi generici (laser, gesti, configurazione)."""
        for c in cmds:
            P.validate(c)
        self._pending_cmds += list(cmds)
        self._wake.set()

    def laser(self, on):
        self._pending_cmds = [c for c in self._pending_cmds if c not in ("L0", "L1")]
        self.command(P.laser(on))

    def gesture(self, name):
        if name not in P.GESTURES:
            raise ValueError(f"gesto sconosciuto: {name}")
        self.command(name)

    def go_pose(self, name):
        for m, a in enumerate(self.config["poses"][name]):
            self.set_angle(m, a)

    def save_pose(self, name, angles=None):
        name = name.strip()
        if not name:
            raise ValueError("nome della posa vuoto")
        angles = angles or [self.target(m) for m in range(P.NUM_MOTORS)]
        self.config["poses"][name] = [int(a) for a in angles]
        self.config.save()
        self._broadcast({"type": "poses", "poses": self.config["poses"]})

    def delete_pose(self, name):
        self.config["poses"].pop(name, None)
        self.config.save()
        self._broadcast({"type": "poses", "poses": self.config["poses"]})

    def stop(self):
        """Arresto: svuota tutto lato PC e lato firmware, laser spento."""
        self._pending_moves.clear()
        self._pending_cmds.clear()
        for j in self.jobs:
            if j.status in ("queued", "running", "paused"):
                j.status = "cancelled"
                j._resume.set()
        self._broadcast_jobs()
        self._pending_cmds.append("read")  # riallinea le posizioni dopo l'arresto
        try:
            self.link.stop_now()
        except LinkError as e:
            self._log(f"arresto non inviato: {e}")
        self._wake.set()

    # ------------------------------------------------------------------ lavori
    def submit(self, name, cmds):
        job = Job(name=name, packets=P.packets(cmds), id=next(self._ids))
        job._resume.set()
        self.jobs.append(job)
        self.jobs = self.jobs[-50:]
        self._broadcast_jobs()
        self._wake.set()
        return job

    def _job(self, job_id):
        for j in self.jobs:
            if j.id == job_id:
                return j
        raise KeyError(job_id)

    def pause(self, job_id):
        j = self._job(job_id)
        if j.status in ("queued", "running"):
            j.status = "paused"
            j._resume.clear()
            self._broadcast_jobs()

    def resume(self, job_id):
        j = self._job(job_id)
        if j.status == "paused":
            j.status = "running" if j.sent else "queued"
            j._resume.set()
            self._broadcast_jobs()
            self._wake.set()

    def cancel(self, job_id):
        j = self._job(job_id)
        if j.status in ("queued", "running", "paused"):
            j.status = "cancelled"
            j._resume.set()
            self._broadcast_jobs()

    def _broadcast_jobs(self):
        self._broadcast({"type": "jobs", "jobs": [j.to_dict() for j in self.jobs[-10:]]})

    def _active_job(self):
        for j in self.jobs:
            if j.status in ("queued", "running"):
                return j
        return None

    # ------------------------------------------------------------------ invio
    def _take_interactive(self):
        cmds = []
        while self._pending_cmds and len(cmds) < P.MAX_CMDS_PER_PACKET:
            cmds.append(self._pending_cmds.pop(0))
        while self._pending_moves and len(cmds) < P.MAX_CMDS_PER_PACKET:
            m, a = self._pending_moves.popitem()
            if a != self._commanded.get(m, self.state.positions[m]):
                cmds.append(P.move(m, a))
                self._commanded[m] = a
        return cmds

    async def _writer(self):
        while True:
            await self._wake.wait()
            self._wake.clear()
            while True:
                if not self.state.connected or not self._ready.is_set():
                    break
                cmds = self._take_interactive()
                if cmds:
                    await self._send(P.encode(cmds))
                    continue
                job = self._active_job()
                if job is None:
                    break
                job.status = "running"
                pkt = job.packets[job.sent]
                ok = await self._send(pkt, cancel=lambda: job.status != "running")
                if job.status == "running":
                    if ok:
                        job.sent += 1
                        if job.sent >= len(job.packets):
                            job.status = "done"
                    else:
                        job.status, job.error = "error", self.state.last_error or "invio fallito"
                self._broadcast_jobs()

    async def _send(self, packet, cancel=None):
        self._log(f"> {packet}", "tx")
        try:
            await self.link.send_packet(packet, cancel=cancel)
            return True
        except LinkError as e:
            self.state.last_error = str(e)
            self._log(f"errore: {e}")
            return False
