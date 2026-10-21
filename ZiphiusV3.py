#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Claudio - controllo da PC del braccio robotico con laser.

Modalita':
  python ZiphiusV3.py draw [--dry-run] [--no-plot]   disegna la traiettoria sulla parete
  python ZiphiusV3.py calibrate                      calibra la geometria della parete
  python ZiphiusV3.py send "P1lXwait500XP1c"         invia comandi grezzi
"""

import argparse
import json
import math
import os
import numpy as np
import time
import threading
import serial  # Assicurati di aver installato pyserial
import random

# ================================
# Dizionario dei Parametri
# ================================
PARAMETERS = {
    'print_info': False,
    # Parametri del modello della traiettoria
    'Lx': 1.0,
    'Ly': 1.0,
    'n': 1, # 4
    'd': 1, # 1
    'omega': 1.0,
    # Parametrizzazione di M in funzione di r = n/d
    'M_values': { 'r_equal_1': 4, 'r_less_than_1': 10, 'r_greater_than_1': 5 },
    'num_points': 20,

    # Parametri per la segmentazione (RDP)
    'epsilon': 0.05,  # tolleranza per l'algoritmo RDP (può essere interpretata come una percentuale)

    # Parametri per la conversione spazio-angolo
    'K': 3.0,           # distanza in metri
    'R': 0.75,          # percentuale riempimento schermo
    'DELTA_ANG': 10,    # in gradi, angolo minimo richiesto per ogni spostamento

    # Parametri per la comunicazione seriale
    'serial_port': '/dev/tty.usbmodem14101',  # Modifica in base al sistema (es. 'COM3' per Windows)
    'baudrate': 9600,
    'timeout': 1,
    'start_char': 'A',
    'end_char': 'Z',
    'sep_char': 'X',
    'len_str': 4,      # numero di comandi per stringa
    'delta_T': 1,       # intervallo di invio in secondi
    'ready_timeout': 20,   # s: l'Uno si resetta all'apertura della seriale e invia "READY" a fine setup
    'idle_timeout': 60,    # s: attesa massima di "IDLE" dopo un pacchetto

    # Parametri per i comandi custom
    'default_wait': 200,  # in millisecondi, per 'wait' senza specifica
    'preset_commands': ["wait", "check", "wait2000"],
    'custom_commands': ["wait2000", "check"],

    # Nuovi parametri: limiti minimi e massimi per ogni motore
    'angle_limits': {
        '0': (70, 110),   # M0: Laser (verticale)
        '1': (30, 110),   # M1: Corpo (orizzontale)
        '2': (30, 130),   # M2: Braccio (longitudinale)
        '3': (30, 100)    # M3: Pinza
    },

    # Proiezione su parete nota (sovrascritti da calibration_file se presente)
    'calibration_file': 'claudio_wall.json',
    'wall': {
        'K': 3.0,            # m, distanza perpendicolare laser-parete
        'pan_center': 70,    # gradi M1 con il raggio perpendicolare alla parete
        'tilt_center': 90,   # gradi M0 con il raggio orizzontale
        'pan_gain': 1.0,     # gradi servo per grado reale (segno = verso)
        'tilt_gain': 1.0,
        'arm_pose': 50,      # gradi M2 durante disegno e calibrazione (posa fissa)
        'center_u': 0.0,     # m, centro del disegno rispetto al piede della perpendicolare
        'center_v': 0.0,
        'width': 1.0,        # m, dimensioni del disegno sulla parete
        'height': 1.0,
        'step_deg': 2.0,     # gradi, passo massimo di ciascun motore lungo un segmento
        'max_active_servos': 1,  # deve coincidere con MAX_ACTIVE_SERVOS dello sketch
    },

}


# ================================
# Classe: Debug
# ================================
class Debug:
    def __init__(self, params):
        # Se la chiave 'print_info' o 'print_loop' non è presente, viene considerata False
        self.print_info = params.get('print_info', False)
        self.print_loop = params.get('print_loop', False)

    def info(self, text):
        if self.print_info:
            before = "[INFO] - "
            print(before + str(text))

    def loop(self, text):
        if self.print_loop:
            before = "[LOOP] - "
            print(before + str(text))



# ================================
# Classe: TrajectoryModel
# ================================
class TrajectoryModel:
    def __init__(self, params):
        self.params = params
        self.Lx = params['Lx']
        self.Ly = params['Ly']
        self.n = params['n']
        self.d = params['d']
        self.omega = params['omega']
        # Calcola il rapporto r e determina M
        r = self.n / self.d
        if abs(r - 1) < 1e-6:
            self.M = params['M_values']['r_equal_1']
        elif r < 1:
            self.M = params['M_values']['r_less_than_1']
        else:
            self.M = params['M_values']['r_greater_than_1']

    def generate_trajectory(self):
        """Genera la traiettoria continua per theta da 0 a M*pi."""
        num_points = self.params['num_points']
        theta_max = self.M * math.pi
        theta_vals = np.linspace(0, theta_max, num_points)

        x_vals = self.Lx * np.cos((self.n/self.d) * theta_vals) * np.cos(theta_vals)
        y_vals = self.Ly * np.cos((self.n/self.d) * theta_vals) * np.sin(theta_vals)
        return np.column_stack((x_vals, y_vals))

# ================================
# Classe: Segmenter (utilizza Ramer–Douglas–Peucker)
# ================================
class Segmenter:
    def __init__(self, params):
        self.params = params
        self.converter = Converter(params)

    def _point_line_distance(self, point, start, end):
        """Calcola la distanza tra un punto e la linea definita da start ed end."""
        if np.allclose(start, end):
            return np.linalg.norm(point - start)
        else:
            # prodotto vettoriale 2D esplicito (np.cross su vettori 2D e' deprecato in NumPy 2)
            d = end - start
            w = start - point
            return abs(d[0] * w[1] - d[1] * w[0]) / np.linalg.norm(d)

    def _rdp(self, points, epsilon):
        """Implementazione ricorsiva dell'algoritmo Ramer–Douglas–Peucker."""
        if points.shape[0] < 3:
            return points

        start, end = points[0], points[-1]
        distances = np.array([self._point_line_distance(point, start, end) for point in points])
        index = np.argmax(distances)
        dmax = distances[index]

        if dmax >= epsilon:
            results1 = self._rdp(points[:index+1], epsilon)
            results2 = self._rdp(points[index:], epsilon)
            return np.vstack((results1[:-1], results2))
        else:
            return np.array([start, end])

    def segment(self, continuous_points):
        """Applica RDP per segmentare la traiettoria continua."""
        epsilon = self.params['epsilon']
        return self._rdp(continuous_points, epsilon)

    def segment_enforce(self, continuous_points):
        """Applica RDP per segmentare la traiettoria continua.
        e Verifica per ogni segmento"""
        epsilon = self.params['epsilon']
        self.segmented_points = self._rdp(continuous_points, epsilon)
        return self.enforce_delta_ang(self.segmented_points, self.converter)

    def enforce_delta_ang(self, segmented_points, converter):
        """
        Verifica per ogni segmento se la variazione convertita in angolo
        raggiunge almeno DELTA_ANG. Se non viene raggiunta, unisce i segmenti.
        """
        final_points = [segmented_points[0]]
        for p in segmented_points[1:]:
            last = final_points[-1]
            delta = np.linalg.norm(np.array(p) - np.array(last))
            # Usa la funzione di conversione passata (oggetto Converter)
            angle = converter.convert(delta)
            if angle < self.params['DELTA_ANG']:
                final_points[-1] = p
            else:
                final_points.append(p)
        return np.array(final_points)

# ================================
# Classe: Converter (spazio-angolo)
# ================================
class Converter:
    def __init__(self, params):
        self.params = params
        self.K = params['K']
        self.DELTA_ANG = params['DELTA_ANG']

    def convert(self, delta_space):
        """
        Converte un incremento spaziale (delta_space) in una variazione angolare in gradi.
        Formula: angolo = atan(delta_space / K) * (180/π)
        """
        return math.atan((delta_space / self.K)) * (180 / math.pi)

    def meets_delta_ang(self, delta_space):
        """Verifica se l'incremento spaziale converte in un angolo >= DELTA_ANG."""
        return self.convert(delta_space) >= self.DELTA_ANG


# Classe: Mapper (mappatura lineare angolo-schermo, mantenuta per compatibilita')
class Mapper:
    def __init__(self, params):
        self.params = params
        self.K = params['K']        # metri
        self.R = params['R']        # percentuale riempimento schermo
        self.Lx = params['Lx']
        self.Ly = params['Ly']
        angle_limits = PARAMETERS['angle_limits']
        self.DegXmin = angle_limits['1'][0]
        self.DegXmax = angle_limits['1'][1]
        self.DegYmin = angle_limits['0'][0]
        self.DegYmax = angle_limits['0'][1]
        self.DegXcenter = 0         # gradi
        self.DegYcenter = 0         # gradi
        self.DegXavg = 0            # gradi
        self.DegYavg = 0            # gradi
        self.Dx = 0                 # metri
        self.Dy = 0                 # metri
        # calcolo delle posizioni e dimensioni dello schermo
        self._calc_screen()
        self.debug = Debug(params)

    def _calc_screen(self):
        # Calcola valore medio tra angoli min e max
        self.DegXavg = abs(self.DegXmax - self.DegXmin)/2
        self.DegYavg = abs(self.DegYmax - self.DegYmin)/2
        # Calcola la posizione del centro dello schermo in gradi
        self.DegXcenter = self.DegXavg + self.DegXmin
        self.DegYcenter = self.DegYavg + self.DegYmin
        # semi-dimensioni dello schermo in metri (math.tan vuole radianti)
        self.Dx = self.K * math.tan(math.radians(self.DegXavg))
        self.Dy = self.K * math.tan(math.radians(self.DegYavg))
        self.screen = {  'Xcenter': self.DegXcenter
                  , 'Ycenter': self.DegYcenter
                  , 'xDim': self.Dx
                  , 'yDim': self.Dy
                  , 'P1': [self.DegXmax , self.DegYcenter]
                  , 'P2': [self.DegXcenter , self.DegYmax]
                  , 'P3': [self.DegXmin , self.DegYcenter]
                  , 'P4': [self.DegXcenter , self.DegYmin]
                  }
        """       , 'P1': [1,0]
                  , 'P2': [0,1]
                  , 'P3': [-1,0]
                  , 'P4': [0,-1]
        """
        return self.screen


    def _rescale(self, value, in_min, in_max, out_min, out_max):
        # Mappa 'value' da un intervallo [in_min, in_max] a un intervallo [out_min, out_max].
        # Calcola la normalizzazione
        norm = (value - in_min) / (in_max - in_min)
        # Assicura che norm sia tra 0 e 1
        norm = max(0, min(1, norm))
        return out_min + norm * (out_max - out_min)

    def ang_to_command(self, motor, ang):
        motor = str(motor)
        # command = f"{motor}{ang:03d}"
        return f"{motor}{ang:03d}"


    def map_to_range(self, points, as_commands=True):
        cmds_x = []
        cmds_y = []
        if as_commands:
            for pt in points:
                ang_x = int(self._rescale(pt[0], -self.Lx, self.Lx, self.DegXmin, self.DegXmax ))
                ang_y = int(self._rescale(pt[1], -self.Ly, self.Ly, self.DegYmin, self.DegYmax ))
                cmd_x = self.ang_to_command('1', ang_x)  # '1' per il motore M1 (Corpo)
                cmd_y = self.ang_to_command('0', ang_y)  # '0' per il motore M0 (Laser)
                self.debug.info({cmd_x, cmd_y})
                cmds_x.append(cmd_x)
                cmds_y.append(cmd_y)
            return np.column_stack((cmds_x, cmds_y))
        else:
            vals_x = []
            vals_y = []
            for pt in points:
                ang_x = int(self._rescale(pt[0], -self.Lx, self.Lx, self.DegXmin, self.DegXmax ))
                ang_y = int(self._rescale(pt[1], -self.Ly, self.Ly, self.DegYmin, self.DegYmax ))
                vals_x.append(ang_x)
                vals_y.append(ang_y)
            return np.column_stack((vals_x, vals_y))


# ================================
# Classe: WallProjector (proiezione su parete nota)
# ================================
class WallProjector:
    """
    Geometria pan-tilt con il tilt (M0) montato sul pan (M1).
    Origine nel perno del laser, z verso la parete (piano z = K), u a destra, v in alto.
    Direzione del raggio: d = (sin p cos t, sin t, cos p cos t)
      =>  u = K tan p,   v = K tan t / cos p
      =>  p = atan2(u, K),   t = atan2(v, sqrt(K^2 + u^2))
    Angolo servo = centro + guadagno * angolo reale (il guadagno assorbe verso e
    scala reale del servo, un SG90 comandato a 180° ne fa tipicamente 170-185).
    Si trascura l'offset tra il perno del laser e l'asse di M1 (errore ~ offset/K).
    """
    def __init__(self, params):
        self.params = params
        self.wall = dict(params['wall'])
        self.load_calibration(params.get('calibration_file'))
        self.limits = params['angle_limits']

    def load_calibration(self, path):
        if path and os.path.exists(path):
            with open(path) as f:
                self.wall.update(json.load(f))
            print(f"Calibrazione caricata da {path}")

    def save_calibration(self, path):
        keys = ['K', 'pan_center', 'tilt_center', 'pan_gain', 'tilt_gain', 'arm_pose']
        with open(path, 'w') as f:
            json.dump({k: self.wall[k] for k in keys}, f, indent=2)
        print(f"Calibrazione salvata in {path}")

    def wall_to_angles(self, u, v):
        """Punto sulla parete (m) -> angoli servo (gradi, float) di M1 e M0."""
        K = self.wall['K']
        p = math.degrees(math.atan2(u, K))
        t = math.degrees(math.atan2(v, math.hypot(K, u)))
        pan = self.wall['pan_center'] + self.wall['pan_gain'] * p
        tilt = self.wall['tilt_center'] + self.wall['tilt_gain'] * t
        return pan, tilt

    def angles_to_wall(self, pan, tilt):
        """Angoli servo -> punto colpito sulla parete (m)."""
        K = self.wall['K']
        p = math.radians((pan - self.wall['pan_center']) / self.wall['pan_gain'])
        t = math.radians((tilt - self.wall['tilt_center']) / self.wall['tilt_gain'])
        return K * math.tan(p), K * math.tan(t) / math.cos(p)

    def shape_to_wall(self, points):
        """Traiettoria normalizzata in [-Lx, Lx] x [-Ly, Ly] -> coordinate sulla parete (m)."""
        w = self.wall
        u = w['center_u'] + points[:, 0] / self.params['Lx'] * w['width'] / 2
        v = w['center_v'] + points[:, 1] / self.params['Ly'] * w['height'] / 2
        return np.column_stack((u, v))

    def _clip(self, motor, angle):
        lo, hi = self.limits[str(motor)]
        a = int(round(angle))
        if a < lo or a > hi:
            print(f"Attenzione: M{motor} {a}° fuori dai limiti [{lo}, {hi}], troncato")
        return min(max(a, lo), hi)

    def plan_commands(self, wall_points):
        """
        Genera i comandi per disegnare la spezzata wall_points.
        Ogni segmento e' interpolato in linea retta SULLA PARETE con passi tali che
        nessun motore si sposti piu' di step_deg: con un motore alla volta il punto
        procede a scalini (prima M1 poi M0) che approssimano il segmento.
        Ritorna (comandi, sequenza degli angoli (pan, tilt) effettivamente raggiunti).
        """
        step = self.wall['step_deg']
        cmds = ["L0", "delta1", f"F2{int(self.wall['arm_pose'])}"]
        pan, tilt = self.wall_to_angles(*wall_points[0])
        cur = (self._clip(1, pan), self._clip(0, tilt))
        cmds += [f"F1{cur[0]}", f"F0{cur[1]}", "L1"]
        path = [cur]
        for a, b in zip(wall_points[:-1], wall_points[1:]):
            pa, ta = self.wall_to_angles(*a)
            pb, tb = self.wall_to_angles(*b)
            n = max(1, math.ceil(max(abs(pb - pa), abs(tb - ta)) / step))
            for k in range(1, n + 1):
                q = a + (b - a) * k / n
                pan, tilt = self.wall_to_angles(*q)
                nxt = (self._clip(1, pan), self._clip(0, tilt))
                if nxt[0] != cur[0]:
                    cmds.append(f"1{nxt[0]}")
                    path.append((nxt[0], cur[1]))
                if nxt[1] != cur[1]:
                    cmds.append(f"0{nxt[1]}")
                    path.append(nxt)
                cur = nxt
        cmds += ["L0", "delta10"]
        return cmds, path

    def estimate_time(self, path):
        """Durata stimata in s (tempo di alimentazione dello sketch: 250 ms + 3 ms/°, max 500 ms)."""
        t = 0.0
        for (p0, t0), (p1, t1) in zip(path[:-1], path[1:]):
            d = abs(p1 - p0) + abs(t1 - t0)
            t += min(0.25 + 0.003 * d, 0.5)
        if self.wall['max_active_servos'] >= 2:
            t /= 2
        return t


# ================================
# Classe: SerialComm (Comunicazione Seriale)
# ================================
class SerialComm:
    def __init__(self, params, dry_run=False):
        self.params = params
        self.port = params['serial_port']
        self.baudrate = params['baudrate']
        self.timeout = params['timeout']
        self.start_char = params['start_char']
        self.end_char = params['end_char']
        self.sep_char = params['sep_char']
        self.len_str = params['len_str']
        self.delta_T = params['delta_T']
        self.ser = None

        if not dry_run:
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            except Exception as e:
                print(f"Errore nell'apertura della porta seriale: {e}")
                self.ser = None
            if self.ser is not None:
                # L'apertura della porta resetta l'Uno: si attende la fine del setup
                if not self.wait_for("READY", params['ready_timeout']):
                    print("READY non ricevuto, proseguo comunque.")

        self.command_buffer = []
        self.lock = threading.Lock()

    def add_command(self, cmd):
        """Aggiunge un comando alla coda."""
        with self.lock:
            self.command_buffer.append(cmd)

    def add_custom_command(self, cmd_name, arg=""):
        """Aggiunge un comando custom (es. wait, reset, check, etc.)."""
        full_cmd = cmd_name + str(arg)
        self.add_command(full_cmd)

    def build_command_string(self):
        """Costruisce la stringa da inviare con len_str comandi."""
        with self.lock:
            cmds = self.command_buffer[:self.len_str]
            self.command_buffer = self.command_buffer[self.len_str:]
        command_str = self.start_char + self.sep_char.join(cmds) + self.end_char
        return command_str

    def read_line(self):
        if self.ser is None:
            return ""
        return self.ser.readline().decode(errors='replace').strip()

    def wait_for(self, token, timeout):
        """Legge e stampa le righe di Arduino finche' una inizia con token. False su timeout o errore."""
        end = time.time() + timeout
        while time.time() < end:
            line = self.read_line()
            if not line:
                continue
            print(f"Feedback ricevuto: {line}")
            if line.startswith(token):
                return True
            if token == "IDLE" and line.startswith("ERROR"):
                print("Errore ricevuto da Arduino.")
        return False

    def send_packet(self, command_str):
        """Invia un pacchetto e attende che Arduino lo abbia eseguito (ACK, poi IDLE)."""
        print(f"Invio: {command_str}")
        if self.ser is None:
            return True
        self.ser.write(command_str.encode())
        if not self.wait_for("ACK", 5):
            print("Nessun ACK ricevuto.")
            return False
        if not self.wait_for("IDLE", self.params['idle_timeout']):
            print("Timeout in attesa di IDLE.")
            return False
        return True

    def send_commands(self):
        """Invia i comandi a pacchetti di len_str: il pacchetto successivo parte solo
        dopo che Arduino ha eseguito il precedente (controllo di flusso via IDLE)."""
        while True:
            command_str = self.build_command_string()
            # Se non ci sono comandi, termina
            if command_str == self.start_char + self.end_char:
                break
            if not self.send_packet(command_str):
                time.sleep(self.delta_T)

    def send_now(self, cmds):
        for cmd in cmds:
            self.add_command(cmd)
        self.send_commands()

# ================================
# Classe: Visualizer
# ================================
class Visualizer:
    def __init__(self, params, save_path=None):
        self.params = params
        self.save_path = save_path

    def _show(self, plt, suffix):
        if self.save_path:
            root, ext = os.path.splitext(self.save_path)
            plt.savefig(f"{root}_{suffix}{ext or '.png'}", dpi=120)
            plt.close()
        else:
            plt.show()

    def plot_trajectories(self, continuous_points, segmented_points):
        """Plotta la traiettoria continua (blu) e quella segmentata (rossa, con punti e segmenti)."""
        import matplotlib.pyplot as plt
        plt.figure(figsize=(8, 6))
        plt.plot(continuous_points[:, 0], continuous_points[:, 1], color='blue', label='Traiettoria continua')
        if segmented_points.shape[0] > 0:
            plt.plot(segmented_points[:, 0], segmented_points[:, 1],
                     color='red', linestyle='-', marker='o', label='Traiettoria segmentata')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.title('Traiettoria continua vs. segmentata')
        plt.grid(True)
        self._show(plt, "traiettoria")

    def plot_wall(self, wall_points, projector, path):
        """Disegno richiesto sulla parete vs. percorso reale del punto laser (angoli interi, un motore alla volta)."""
        import matplotlib.pyplot as plt
        spot = np.array([projector.angles_to_wall(p, t) for p, t in path])
        plt.figure(figsize=(8, 6))
        plt.plot(wall_points[:, 0], wall_points[:, 1], color='red', marker='o', label='Segmenti richiesti')
        plt.plot(spot[:, 0], spot[:, 1], color='green', linewidth=1, label='Percorso del laser')
        plt.xlabel('u [m]')
        plt.ylabel('v [m]')
        plt.axis('equal')
        plt.legend()
        plt.title(f"Parete a K = {projector.wall['K']} m")
        plt.grid(True)
        self._show(plt, "parete")


# ================================
# Calibrazione interattiva
# ================================
def jog(serial_comm, pan, tilt, prompt):
    """Sposta il laser con w/a/s/d (1°) o W/A/S/D (5°) finche' l'utente scrive 'ok'."""
    print(prompt)
    print("  a/d = M1 -/+, s/w = M0 -/+ (maiuscole = 5°), più tasti per riga ammessi, 'ok' per confermare")
    while True:
        line = input(f"[M1={pan} M0={tilt}] > ").strip()
        if line.lower() == "ok":
            return pan, tilt
        old = (pan, tilt)
        for c in line:
            step = 5 if c.isupper() else 1
            c = c.lower()
            if c == 'a': pan -= step
            elif c == 'd': pan += step
            elif c == 's': tilt -= step
            elif c == 'w': tilt += step
        pan = min(max(pan, 0), 180)
        tilt = min(max(tilt, 0), 160)
        cmds = []
        if pan != old[0]: cmds.append(f"F1{pan}")
        if tilt != old[1]: cmds.append(f"F0{tilt}")
        if cmds:
            serial_comm.send_now(cmds + ["L1"])


def calibrate(params, serial_comm):
    projector = WallProjector(params)
    w = projector.wall
    print("=== Calibrazione parete ===")
    print("Segna sulla parete il punto F: il piede della perpendicolare dal laser (stessa altezza del laser).")
    K = float(input(f"Distanza perpendicolare laser-parete K in metri [{w['K']}]: ") or w['K'])
    serial_comm.send_now(["rand0", "guard0", "delta1", f"F2{int(w['arm_pose'])}",
                          f"F1{int(w['pan_center'])}", f"F0{int(w['tilt_center'])}", "L1"])
    pan0, tilt0 = jog(serial_comm, int(w['pan_center']), int(w['tilt_center']),
                      "Porta il punto laser su F.")
    u1 = float(input("Distanza orizzontale in m di un secondo segno a DESTRA di F (es. 1.0): "))
    pan1, _ = jog(serial_comm, pan0, tilt0, "Porta il punto laser sul segno a destra (muovi solo a/d).")
    serial_comm.send_now([f"F1{pan0}"])
    v1 = float(input("Distanza verticale in m di un terzo segno SOPRA F (es. 0.5): "))
    _, tilt1 = jog(serial_comm, pan0, tilt0, "Porta il punto laser sul segno in alto (muovi solo w/s).")
    w.update({
        'K': K,
        'pan_center': pan0,
        'tilt_center': tilt0,
        'pan_gain': (pan1 - pan0) / math.degrees(math.atan2(u1, K)),
        'tilt_gain': (tilt1 - tilt0) / math.degrees(math.atan2(v1, K)),
    })
    if w['pan_gain'] == 0 or w['tilt_gain'] == 0:
        print("Guadagno nullo: i segni devono richiedere un movimento del laser. Calibrazione annullata.")
    else:
        print(f"pan_gain = {w['pan_gain']:.3f}, tilt_gain = {w['tilt_gain']:.3f}")
        projector.save_calibration(params['calibration_file'])
    serial_comm.send_now(["L0", "delta10", "guard20000", "rand1"])


# ================================
# Disegno sulla parete
# ================================
def draw(params, serial_comm, visualizer):
    # Inizializza le classi passando il dizionario PARAMETERS
    traj_model = TrajectoryModel(params)
    segmenter = Segmenter(params)
    projector = WallProjector(params)

    # 1. Generazione della traiettoria continua
    continuous_points = traj_model.generate_trajectory()

    # 2. Segmentazione della traiettoria (RDP)
    segmented_points = segmenter.segment(continuous_points)
    # Applica una logica per garantire che ogni spostamento abbia una variazione angolare minima
    # segmented_points = segmenter.enforce_delta_ang(segmented_points, converter)

    # 3. Proiezione sulla parete e pianificazione dei comandi
    wall_points = projector.shape_to_wall(segmented_points)
    func_commands, path = projector.plan_commands(wall_points)

    # 4. Visualizzazione: traiettoria continua/segmentata e percorso reale del laser sulla parete
    if visualizer is not None:
        visualizer.plot_trajectories(continuous_points, segmented_points)
        visualizer.plot_wall(wall_points, projector, path)

    all_commands = params['preset_commands'] + func_commands + params['custom_commands']
    print(all_commands)
    print(f"{len(func_commands)} comandi di disegno, durata stimata {projector.estimate_time(path):.0f} s")

    # 5. Invio con controllo di flusso
    serial_comm.send_now(all_commands)
    print("Termino l'invio dei comandi.")


# ================================
# Funzione: main()
# ================================
def main():
    parser = argparse.ArgumentParser(description="Claudio - controllo del braccio robotico con laser")
    parser.add_argument('mode', nargs='?', default='draw', choices=['draw', 'calibrate', 'send'])
    parser.add_argument('commands', nargs='?', default='', help="per 'send': comandi separati da X")
    parser.add_argument('--port', default=PARAMETERS['serial_port'])
    parser.add_argument('--dry-run', action='store_true', help="non apre la seriale, stampa soltanto")
    parser.add_argument('--no-plot', action='store_true')
    parser.add_argument('--save-plot', default=None, help="salva i grafici in PNG invece di mostrarli")
    args = parser.parse_args()

    PARAMETERS['serial_port'] = args.port
    serial_comm = SerialComm(PARAMETERS, dry_run=args.dry_run)
    if serial_comm.ser is None and not args.dry_run:
        print("Porta seriale non disponibile.")
        return

    if args.mode == 'calibrate':
        calibrate(PARAMETERS, serial_comm)
    elif args.mode == 'send':
        serial_comm.send_now([c for c in args.commands.split(PARAMETERS['sep_char']) if c])
    else:
        visualizer = None if args.no_plot else Visualizer(PARAMETERS, args.save_plot)
        draw(PARAMETERS, serial_comm, visualizer)

if __name__ == '__main__':
    main()
