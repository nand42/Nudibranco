"""Proiezione del laser su una parete nota (portata dalla V3, senza numpy).

Il tilt (M0) e' montato sul pan (M1). Origine nel perno del laser, z verso la parete
(piano z = K), u a destra, v in alto, misurati dal piede della perpendicolare F.
Direzione del raggio: d = (sin p cos t, sin t, cos p cos t)
  =>  u = K tan p,   v = K tan t / cos p
  =>  p = atan2(u, K),   t = atan2(v, sqrt(K^2 + u^2))
Angolo servo = centro + guadagno * angolo reale (il guadagno ingloba verso e scala del servo).
Si trascura l'offset tra il perno del laser e l'asse di M1 (errore ~ offset / K).
"""

import math


class WallProjector:
    def __init__(self, wall, draw_limits=None):
        self.wall = dict(wall)
        lim = draw_limits or {"0": (0, 160), "1": (0, 180)}
        self.limits = {int(k): tuple(v) for k, v in lim.items()}

    def wall_to_angles(self, u, v):
        """Punto sulla parete (m) -> angoli servo (gradi, float) di M1 (pan) e M0 (tilt)."""
        w = self.wall
        p = math.degrees(math.atan2(u, w["K"]))
        t = math.degrees(math.atan2(v, math.hypot(w["K"], u)))
        return w["pan_center"] + w["pan_gain"] * p, w["tilt_center"] + w["tilt_gain"] * t

    def angles_to_wall(self, pan, tilt):
        """Angoli servo -> punto colpito sulla parete (m)."""
        w = self.wall
        p = math.radians((pan - w["pan_center"]) / w["pan_gain"])
        t = math.radians((tilt - w["tilt_center"]) / w["tilt_gain"])
        return w["K"] * math.tan(p), w["K"] * math.tan(t) / math.cos(p)

    def reachable(self, u, v):
        pan, tilt = self.wall_to_angles(u, v)
        (plo, phi), (tlo, thi) = self.limits[1], self.limits[0]
        return plo <= pan <= phi and tlo <= tilt <= thi

    def resolution(self):
        """Spostamento del punto sulla parete per 1 grado di servo, al centro (m)."""
        return self.wall["K"] * math.tan(math.radians(1 / abs(self.wall["pan_gain"])))

    def _clip(self, motor, angle):
        lo, hi = self.limits[motor]
        return min(max(int(round(angle)), lo), hi)

    def plan_strokes(self, strokes):
        """
        strokes: lista di spezzate, ciascuna lista di punti (u, v) in metri.
        Laser spento negli spostamenti tra una spezzata e l'altra.
        Ogni segmento e' interpolato in linea retta SULLA PARETE con passi tali che
        nessun motore si sposti piu' di step_deg; con un motore alla volta il punto
        avanza a scalini (prima M1 poi M0).
        Ritorna (comandi, percorso [(pan, tilt, laser_acceso)]).
        """
        step = self.wall["step_deg"]
        cmds = ["L0", f"F2{int(self.wall['arm_pose'])}"]
        path = []
        cur = None
        for stroke in strokes:
            if len(stroke) == 0:
                continue
            pan, tilt = self.wall_to_angles(*stroke[0])
            start = (self._clip(1, pan), self._clip(0, tilt))
            if cur is None or start != cur:
                if path and path[-1][2]:
                    cmds.append("L0")
                cmds += [f"F1{start[0]}", f"F0{start[1]}"]
                path.append((start[0], start[1], False))
                cur = start
            cmds.append("L1")
            path.append((cur[0], cur[1], True))
            for a, b in zip(stroke[:-1], stroke[1:]):
                pa, ta = self.wall_to_angles(*a)
                pb, tb = self.wall_to_angles(*b)
                n = max(1, math.ceil(max(abs(pb - pa), abs(tb - ta)) / step))
                for k in range(1, n + 1):
                    q = (a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
                    pan, tilt = self.wall_to_angles(*q)
                    nxt = (self._clip(1, pan), self._clip(0, tilt))
                    if nxt[0] != cur[0]:
                        cmds.append(f"1{nxt[0]}")
                        path.append((nxt[0], cur[1], True))
                    if nxt[1] != cur[1]:
                        cmds.append(f"0{nxt[1]}")
                        path.append((nxt[0], nxt[1], True))
                    cur = nxt
            cmds.append("L0")
            path.append((cur[0], cur[1], False))
        return cmds, path

    def estimate_time(self, path):
        """Durata stimata in s (firmware: 250 ms + 3 ms/grado, max 500 ms per movimento)."""
        t = 0.0
        for (p0, t0, _), (p1, t1, _) in zip(path[:-1], path[1:]):
            d = abs(p1 - p0) + abs(t1 - t0)
            if d:
                t += min(0.25 + 0.003 * d, 0.5)
        if self.wall.get("max_active_servos", 1) >= 2:
            t /= 2
        return t
