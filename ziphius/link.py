"""Collegamento seriale con Claudio: lettura in un thread, consegna degli eventi al loop asyncio,
riconnessione automatica e invio dei pacchetti con controllo di flusso (ACK, poi IDLE)."""

import asyncio
import glob
import logging
import threading
import time

from .protocol import STOP_PACKET, parse_line

log = logging.getLogger("ziphius.link")

PORT_PATTERNS = ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/tty.usbmodem*", "/dev/cu.usbmodem*")


def find_ports():
    """Porte seriali candidate (Arduino Uno: /dev/ttyACM0 su Linux/Raspberry)."""
    ports = []
    for pattern in PORT_PATTERNS:
        ports += sorted(glob.glob(pattern))
    if not ports:
        try:
            from serial.tools import list_ports
            ports = [p.device for p in list_ports.comports()]
        except Exception:
            pass
    return ports


class PySerialTransport:
    """Trasporto reale su pyserial. readline() ritorna "" allo scadere del timeout."""

    def __init__(self, port, baudrate=9600):
        import serial
        self.port = port
        self.ser = serial.Serial(port, baudrate, timeout=0.2)

    def readline(self):
        return self.ser.readline().decode(errors="replace").strip()

    def write(self, data):
        self.ser.write(data)

    def close(self):
        self.ser.close()


class LinkError(Exception):
    pass


class SerialLink:
    """
    transport_factory(): ritorna un oggetto con readline()/write(bytes)/close() e attributo port.
    on_event(Event): chiamata nel loop asyncio per ogni riga ricevuta e per gli eventi
    sintetici "connected" / "disconnected".
    """

    def __init__(self, transport_factory, on_event, reconnect_delay=2.0):
        self.transport_factory = transport_factory
        self.on_event = on_event
        self.reconnect_delay = reconnect_delay
        self.transport = None
        self.loop = None
        self._subscribers = set()
        self._stop = threading.Event()
        self._thread = None
        self._write_lock = threading.Lock()

    # ------------------------------------------------------------------ ciclo di vita
    def start(self):
        self.loop = asyncio.get_running_loop()
        self._thread = threading.Thread(target=self._reader, name="ziphius-serial", daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self._close_transport()

    @property
    def connected(self):
        return self.transport is not None

    def _close_transport(self):
        t, self.transport = self.transport, None
        if t is not None:
            try:
                t.close()
            except Exception:
                pass

    def _reader(self):
        from .protocol import Event
        while not self._stop.is_set():
            if self.transport is None:
                try:
                    self.transport = self.transport_factory()
                except Exception as e:
                    log.debug("connessione fallita: %s", e)
                    self._stop.wait(self.reconnect_delay)
                    continue
                port = getattr(self.transport, "port", "")
                log.info("collegato a %s", port)
                self._emit(Event("connected", {"port": port}))
            try:
                line = self.transport.readline()
            except Exception as e:
                log.warning("seriale persa: %s", e)
                self._close_transport()
                self._emit(Event("disconnected", {"reason": str(e)}))
                self._stop.wait(self.reconnect_delay)
                continue
            if line:
                self._emit(parse_line(line))

    def _emit(self, ev):
        if self.loop is not None and not self.loop.is_closed():
            self.loop.call_soon_threadsafe(self._dispatch, ev)

    def _dispatch(self, ev):
        for q in list(self._subscribers):
            q.put_nowait(ev)
        self.on_event(ev)

    # ------------------------------------------------------------------ scrittura
    def write(self, text):
        if self.transport is None:
            raise LinkError("Claudio non collegato")
        with self._write_lock:
            self.transport.write(text.encode())

    def stop_now(self):
        """Arresto di emergenza: scavalca il controllo di flusso."""
        self.write(STOP_PACKET)

    async def send_packet(self, packet, ack_timeout=5.0, idle_timeout=120.0, cancel=None):
        """
        Invia un pacchetto e attende che il firmware lo abbia eseguito.
        Gli eventi vengono letti in ordine da una coda dedicata registrata PRIMA della
        scrittura: un IDLE vale solo se arriva dopo l'ACK di questo pacchetto.
        Ritorna la lista degli eventi ricevuti; LinkError su timeout, errore o disconnessione.
        """
        q = asyncio.Queue()
        self._subscribers.add(q)
        events = []
        try:
            self.write(packet)
            acked = False
            deadline = time.monotonic() + ack_timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise LinkError("timeout in attesa di " + ("IDLE" if acked else "ACK"))
                try:
                    ev = await asyncio.wait_for(q.get(), timeout=min(remaining, 0.5))
                except asyncio.TimeoutError:
                    if cancel is not None and cancel():
                        raise LinkError("annullato")
                    continue
                events.append(ev)
                if ev.kind == "disconnected":
                    raise LinkError("Claudio scollegato")
                if not acked:
                    if ev.kind == "ack":
                        acked = True
                        deadline = time.monotonic() + idle_timeout
                    elif ev.kind == "error":
                        raise LinkError(ev.data["message"])
                    elif ev.kind == "stopped":
                        raise LinkError("arrestato")
                elif ev.kind == "idle":
                    return events
        finally:
            self._subscribers.discard(q)
