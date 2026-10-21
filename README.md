# Nudibranco — Claudio
Arduino and Python project to control robotic arm equipped with a laser

Diagrammi di flusso (firmware vecchio e nuovo, tatto, protocollo): [docs/FLOWCHART.md](docs/FLOWCHART.md)

## Hardware

| Componente | Pin Arduino Uno | Note |
|---|---|---|
| M0 laser (verticale), SG90 | 6 | limiti 0–160° |
| M1 corpo (orizzontale), SG90 | 5 | limiti 0–180° |
| M2 braccio, SG90 | 11 | limiti 30–130° |
| M3 pinza, SG90 | 10 | limiti 30–100° |
| Laser KY-008 (segnale S) | **7** | `-` a GND. Il modulo assorbe ~30 mA dal pin: se il pin scalda, pilotalo con un NPN (BC547 + 1 kΩ) |
| HC-SR04 Trig | **8** | |
| HC-SR04 Echo | **9** | VCC a 5V, GND a GND |
| LED giallo / blu / rosso / verde | 2 / 4 / 12 / 13 | con resistenza |

### Alimentazione
- **Oggi (USB):** `MAX_ACTIVE_SERVOS = 1`. La USB fornisce circa 500 mA, mentre un SG90 in stallo ne assorbe 650–750: si alimenta un servo alla volta e gli altri restano staccati.
- **Upgrade:** alimentatore 5V da almeno 3 A sulla barra dei servo, con **GND in comune** con Arduino e un condensatore da 470–1000 µF sulla barra. Poi imposta `MAX_ACTIVE_SERVOS = 2` (il laser traccia diagonali) o fino a 4 nello sketch, e `max_active_servos` in `ZiphiusV3.py`.

## Protocollo seriale (9600 baud)
Pacchetto: `A<cmd>X<cmd>X...Z`. Ogni pacchetto riceve la risposta `ACK <lunghezza coda>`. Quando la coda è vuota e i servo sono fermi arriva `IDLE`. A fine avvio Arduino invia `READY`. Gli errori iniziano con `ERROR` e fanno lampeggiare il LED rosso 3 volte.

| Comando | Effetto |
|---|---|
| `<m><ang>` | motore m (0–3) all'angolo ang; ignorato se lo spostamento è minore di `DELTA_ANG` |
| `F<m><ang>` | movimento forzato: ignora la soglia |
| `P0bd` `P0bu` `P0u` `P0fu` `P0fd` | posizioni predefinite di M0 |
| `P1ll` `P1l` `P1c` `P1r` `P1rr` | posizioni predefinite di M1 |
| `P2f` `P2l` `P2mm` `P2m` `P2c` | posizioni predefinite di M2 |
| `P3oo` `P3o` `P3c` | posizioni predefinite di M3 |
| `w<ms>` / `wait<ms>` | pausa (500 ms se manca il numero) |
| `reset` | posizione iniziale |
| `restore` | ritorno forzato all'ultima posizione comandata |
| `check` | sequenza di verifica di tutti i motori |
| `raimbow` / `lgbt` | animazione dei LED + reset |
| `random` / `rand0` / `rand1` | modalità random: attiva subito / disattivata / attiva dopo 60 s senza seriale |
| `read` | posizioni, laser, distanza |
| `L1` / `L0` | laser acceso / spento (si spegne da solo dopo 2 min senza movimenti) |
| `guard<ms>` | intervallo di ritorno automatico in posizione (`guard0` = disattivato, default 20000) |
| `delta<deg>` | soglia `DELTA_ANG` (default 10; il disegno laser usa 1) |
| `sonar` | misura della distanza |
| `touch0` / `touch1` | tatto disattivato / attivo |
| `hello` `shy` `yes` `no` | gesti: a fine gesto i motori tornano alla posa precedente |
| `help` | elenco dei comandi |

### Messaggi del tatto
`PRESENCE <cm>` (meno di 40 cm, gesto `hello`), `TOUCH <cm>` (meno di 8 cm: laser spento e gesto `shy`), `GONE`.

### LED
| LED | Significato |
|---|---|
| Giallo | acceso fisso: laser acceso; lampeggiante: `wait` |
| Blu | un motore è alimentato |
| Rosso | impulso: comando ricevuto; 3 lampeggi: errore; battito lento: presenza; lampeggio rapido: tocco |
| Verde | battito ogni 2 s: vivo; lampeggio: modalità random |

## Python (`ZiphiusV3.py`)
```
pip install numpy matplotlib pyserial
python ZiphiusV3.py calibrate --port /dev/tty.usbmodem14101
python ZiphiusV3.py draw      --port /dev/tty.usbmodem14101
python ZiphiusV3.py draw --dry-run --save-plot prova.png    # senza Arduino
python ZiphiusV3.py send "P1lXwait500XP1cXhello"
```

### Proiezione su parete
Il tilt (M0) è montato sul pan (M1). L'origine è nel perno del laser, la parete è il piano z = K, e (u, v) è il punto sulla parete misurato dal piede della perpendicolare:

- p = atan2(u, K)
- t = atan2(v, √(K² + u²))
- angolo servo = centro + guadagno · angolo

La calibrazione chiede di puntare tre segni sulla parete (il piede F, un segno a destra, un segno in alto). Da questi ricava `pan_center`, `tilt_center`, `pan_gain` e `tilt_gain` e li salva in `claudio_wall.json`. I segmenti vengono interpolati in linea retta sulla parete, con passi di al massimo `step_deg` per motore. Con un motore alla volta il punto avanza a scalini.

Limite fisico: i servo ricevono angoli interi, quindi a K = 3 m la risoluzione è K·tan 1° ≈ 5,2 cm.
