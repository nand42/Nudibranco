#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import matplotlib.pyplot as plt
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
    
    # Parametri per i comandi custom
    'default_wait': 200,  # in millisecondi, per 'wait' senza specifica

    # Nuovi parametri: limiti minimi e massimi per ogni motore
    'angle_limits': {
        '0': (70, 110),   # M0: Laser (verticale)
        '1': (30, 110),   # M1: Corpo (orizzontale)
        '2': (30, 130),   # M2: Braccio (longitudinale)
        '3': (30, 100)    # M3: Pinza
    }

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
            return np.linalg.norm(np.cross(end - start, start - point)) / np.linalg.norm(end - start)

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
        Formula: angolo = (delta_space / K) * (180/π)
        """
        return math.atan((delta_space / self.K)) * (180 / math.pi)
    
    def meets_delta_ang(self, delta_space):
        """Verifica se l'incremento spaziale converte in un angolo >= DELTA_ANG."""
        return self.convert(delta_space) >= self.DELTA_ANG


# Classe: Mapper
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
        self.Dx = self.K * math.tan(self.DegXavg)
        self.Dy = self.K * math.tan(self.DegYavg)
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
# Classe: SerialComm (Comunicazione Seriale)
# ================================
class SerialComm:
    def __init__(self, params):
        self.params = params
        self.port = params['serial_port']
        self.baudrate = params['baudrate']
        self.timeout = params['timeout']
        self.start_char = params['start_char']
        self.end_char = params['end_char']
        self.sep_char = params['sep_char']
        self.len_str = params['len_str']
        self.delta_T = params['delta_T']
        
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        except Exception as e:
            print(f"Errore nell'apertura della porta seriale: {e}")
            self.ser = None
        
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
    
    def send_commands(self):
        """Invia periodicamente i comandi e gestisce il feedback da Arduino."""
        while True:
            if self.ser is None:
                print("Porta seriale non disponibile. Uscita dal ciclo di invio.")
                break
            
            command_str = self.build_command_string()
            # Se non ci sono comandi, attendi delta_T secondi
            if command_str == self.start_char + self.end_char:
                time.sleep(self.delta_T)
                continue
            
            print(f"Invio: {command_str}")
            self.ser.write(command_str.encode())
            
            # Attende il feedback
            feedback = self.ser.readline().decode().strip()
            if feedback:
                print(f"Feedback ricevuto: {feedback}")
                if "error" in feedback.lower():
                    print("Errore ricevuto, ritrasmissione...")
                    time.sleep(1)
                    continue
            else:
                print("Nessun feedback ricevuto, attendo...")
                time.sleep(1)
                continue
            
            time.sleep(self.delta_T)

# ================================
# Classe: Visualizer
# ================================
class Visualizer:
    def __init__(self, params):
        self.params = params
    
    def plot_trajectories(self, continuous_points, segmented_points):
        """Plotta la traiettoria continua (blu) e quella segmentata (rossa, con punti e segmenti)."""
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
        plt.show()




# ================================
# Funzione: main()
# ================================
def main():
    # Inizializza le classi passando il dizionario PARAMETERS
    traj_model = TrajectoryModel(PARAMETERS)
    segmenter = Segmenter(PARAMETERS)
    converter = Converter(PARAMETERS)
    visualizer = Visualizer(PARAMETERS)
    serial_comm = SerialComm(PARAMETERS)
    mapper = Mapper(PARAMETERS)

    # 1. Generazione della traiettoria continua
    continuous_points = traj_model.generate_trajectory()
    
    # 2. Segmentazione della traiettoria (RDP)
    segmented_points = segmenter.segment(continuous_points)
    # Applica una logica per garantire che ogni spostamento abbia una variazione angolare minima
    # segmented_points = segmenter.enforce_delta_ang(segmented_points, converter)
    
    # 2.bis Segmentazione ed enforce  DA TESTARE
    # segmented_points = segmenter.segment_enforce(continuous_points)


    # 3. Visualizzazione: plottiamo la traiettoria continua (blu) e quella segmentata (rossa)
    visualizer.plot_trajectories(continuous_points, segmented_points)
    
    # 4. Generazione dei comandi per i motori (esempio di mapping)
    # Per ogni punto segmentato, generiamo un comando per l'asse x e uno per l'asse y.
    # Qui, usiamo una normalizzazione semplice supponendo che Lx e Ly rappresentino i massimi.
    func_commands = []
    # Recupera i limiti definiti per ciascun motore dal dizionario PARAMETERS
    angle_limits = PARAMETERS['angle_limits']

    ang_commands = mapper.map_to_range(segmented_points)

    # Aggiungiamo alcuni comandi custom prima dell'invio delle coordinate
    preset_commands = ["wait", "check", "wait2000"]

    for pt in ang_commands:
        func_commands.append(pt[0])
        func_commands.append(pt[1])
    
    # Aggiungiamo alcuni comandi custom alla fine
    custom_commands = ["wait2000", "check"]
    
    all_commands =  preset_commands + func_commands + custom_commands

    # Inseriamo i comandi nel buffer della comunicazione seriale
    for cmd in all_commands:
        serial_comm.add_command(cmd)
    
    print(all_commands)

    # Avvio del thread per l'invio periodico dei comandi via seriale
    serial_thread = threading.Thread(target=serial_comm.send_commands, daemon=True)
    serial_thread.start()
    
    # Lasciamo in esecuzione il thread per un certo periodo (ad es. 30 secondi)
    time.sleep(60)
    print("Termino l'invio dei comandi.")

if __name__ == '__main__':
    main()
