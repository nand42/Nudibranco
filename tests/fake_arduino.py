"""Arduino finto per i test: imita le risposte del firmware di Claudio (ACK, movimenti, IDLE)."""

import queue
import re
import threading
import time


class FakeArduino:
    LIMITS = {0: (0, 160), 1: (0, 180), 2: (30, 130), 3: (30, 100)}

    def __init__(self, port="/dev/fake", move_time=0.01, boot=True):
        self.port = port
        self.move_time = move_time
        self.out = queue.Queue()
        self.received = []          # pacchetti ricevuti
        self.executed = []          # comandi eseguiti in ordine
        self.positions = [80, 110, 50, 65]
        self.laser = False
        self.delta = 10
        self.closed = False
        self._lock = threading.Lock()
        self._buf = ""
        self._work = queue.Queue()
        self._stop_flag = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()
        if boot:
            self.out.put("Initial reset sequence completed.")
            self.out.put("READY")

    # interfaccia di trasporto
    def readline(self):
        if self.closed:
            raise OSError("closed")
        try:
            return self.out.get(timeout=0.05)
        except queue.Empty:
            return ""

    def write(self, data):
        self._buf += data.decode()
        while "Z" in self._buf:
            pkt, self._buf = self._buf.split("Z", 1)
            pkt = pkt[pkt.index("A") + 1:]
            self.received.append(pkt)
            if pkt.lower() == "stop":
                self._stop_flag.set()
                with self._work.mutex:
                    self._work.queue.clear()
                self.laser = False
                self.out.put("Laser OFF")
                self.out.put("STOPPED")
                self.out.put("IDLE")
                continue
            self.out.put(f"ACK {len(pkt)}")
            self._work.put(pkt.split("X"))

    def close(self):
        self.closed = True

    # esecuzione
    def _run(self):
        while True:
            cmds = self._work.get()
            self._stop_flag.clear()
            for c in cmds:
                if self._stop_flag.is_set():
                    break
                self._exec(c)
            if not self._stop_flag.is_set():
                self.out.put("IDLE")

    def _exec(self, c):
        self.executed.append(c)
        m = re.fullmatch(r"(F?)(\d)(\d{1,3})", c)
        if m:
            motor, angle = int(m[2]), int(m[3])
            lo, hi = self.LIMITS[motor]
            angle = max(lo, min(hi, angle))
            if m[1] or abs(angle - self.positions[motor]) >= self.delta:
                time.sleep(self.move_time)
                self.positions[motor] = angle
                self.out.put(f"Motor {motor} moved to {angle}")
        elif c in ("L1", "L0"):
            self.laser = c == "L1"
            self.out.put("Laser ON" if self.laser else "Laser OFF")
        elif c.startswith("delta"):
            self.delta = int(c[5:])
            self.out.put(f"DELTA_ANG: {self.delta}")
        elif c == "read":
            p = self.positions
            self.out.put(f"Positions: M0:{p[0]} M1:{p[1]} M2:{p[2]} M3:{p[3]} "
                         f"L:{int(self.laser)} D:-1 R:0 E:1 T:1 G:20000")
        elif c in ("hello", "shy", "yes", "no"):
            self.out.put(f"Gesture: {c}")
        elif c.startswith("w"):
            time.sleep(0.01)
        else:
            self.out.put("ERROR Invalid motor number.")
