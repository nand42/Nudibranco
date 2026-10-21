# Claudio — diagrammi di flusso

I diagrammi sono in Mermaid: GitHub li disegna direttamente.

- [1. Firmware vecchio (V33V6.1)](#1-firmware-vecchio-v33v61)
- [2. Firmware nuovo](#2-firmware-nuovo)
- [3. Tatto (HC-SR04)](#3-tatto-hc-sr04)
- [4. Python ↔ Arduino: controllo di flusso](#4-python--arduino-controllo-di-flusso)

---

## 1. Firmware vecchio (V33V6.1)

I nodi rossi indicano i difetti corretti nella versione nuova.

### setup()

```mermaid
flowchart TD
    S0([Accensione]) --> S1[Serial 9600, delay 1 s]
    S1 --> S2[Configura LED]
    S2 --> S3["raimbowLEDS(true)<br/>bloccante, 1.6 s"]
    S3 --> S4[Posizioni virtuali = iniziali]
    S4 --> S5["buildResetSequence()<br/>P0uXP1cXP2mXP3o in coda"]
    S5 --> S6[resetInProgress = true]
    S6 --> S7{coda non vuota<br/>o servo attivo?}
    S7 -- sì --> S8{nessun servo attivo<br/>e coda non vuota?}
    S8 -- sì --> S9[estrai comando<br/>processCommand]
    S8 -- no --> S10
    S9 --> S10[checkServoDetach<br/>updateLEDs<br/>delay 50 ms]
    S10 --> S7
    S7 -- no --> S11[resetInProgress = false<br/>detach di tutti i servo]
    S11 --> S12["raimbowLEDS(false)"]:::bug
    S12 --> S13([loop])
    classDef bug fill:#f8d7da,stroke:#b02a37,color:#000
```

`raimbowLEDS(false)` non accende nulla: il ciclo `for (i = N-1; i > N; i--)` non entra mai.

### loop()

```mermaid
flowchart TD
    L0([loop]) --> L1[processIncomingSerial]
    L1 --> L1a["A...Z: aggiunge i comandi alla coda<br/>separatore finale SOLO se il pacchetto non ne contiene"]:::bug
    L1a --> L2{millis < waitUntil?}
    L2 -- sì --> L3[LED giallo]
    L2 -- no --> L4{nessun servo attivo<br/>e coda non vuota?}
    L4 -- sì --> L5[estrai comando<br/>processCommand]
    L4 -- no --> L6
    L3 --> L6
    L5 --> L6{60 s senza seriale<br/>e coda vuota e servo fermi?}
    L6 -- sì --> L7["processPeriodicTasks<br/>moveMotor su 4 motori NELLO STESSO ciclo"]:::bug
    L6 -- no --> L8
    L7 --> L8["checkServoDetach<br/>(detach dopo 500 ms fissi)"]
    L8 --> L9[updateLEDs]
    L9 --> L0
    classDef bug fill:#f8d7da,stroke:#b02a37,color:#000
```

### processCommand()

```mermaid
flowchart TD
    C0([comando]) --> C1["lastSerialTime = millis()"]:::bug
    C1 --> C2{inizia con w/W?}
    C2 -- sì --> C3["wait: numero letto solo se lunghezza > 4<br/>(w200 diventa 500 ms)"]:::bug
    C2 -- no --> C4{random / rand?}
    C4 -- sì --> C5["solo LED<br/>(non attiva la modalità)"]:::bug
    C4 -- no --> C6{read?}
    C6 -- sì --> C7[stampa posizioni]
    C6 -- no --> C8{check?}
    C8 -- sì --> C9[accoda la sequenza di verifica in FONDO]
    C8 -- no --> C10{reset?}
    C10 -- sì --> C11["accoda P0uXP1cXP2mXP3o<br/>in FONDO e SENZA separatore finale"]:::bug
    C10 -- no --> C12{raimbow / lgbt?}
    C12 -- sì --> C13[LED bloccanti + accoda wait/reset/wait]
    C12 -- no --> C14{P0..P3?}
    C14 -- sì --> C15[posizione predefinita<br/>moveMotor]
    C14 -- no --> C16["[m][angolo]<br/>angolo non validato ('0abc' = 0°)"]:::bug
    C16 --> C17[moveMotor]
    classDef bug fill:#f8d7da,stroke:#b02a37,color:#000
```

Conseguenze dei difetti principali:

| Difetto | Effetto |
|---|---|
| Separatore finale mancante | Python invia 4 comandi per pacchetto; l'ultimo (`1050`) si fonde con il primo del pacchetto successivo (`0090`) e diventa `10500090`: M1 va a 180°. |
| Random su 4 motori insieme | Viola "un motore alla volta": fino a ~3 A richiesti alla USB, con rischio di reset dell'Uno. |
| `reset` in fondo alla coda | Nel `check` il reset iniziale viene eseguito alla fine della sequenza. |
| `lastSerialTime` in `processCommand` | Se i comandi interni passano dalla coda, la modalità random si riattiva una sola volta. |

---

## 2. Firmware nuovo

I nodi verdi sono nuovi o modificati.

### setup()

```mermaid
flowchart TD
    S0([Accensione]) --> S0a[Laser OFF, TRIG/ECHO configurati]:::new
    S0a --> S1[Serial 9600, delay 1 s]
    S1 --> S2[Configura LED]
    S2 --> S3["raimbowLEDS(true)<br/>+ waitLEDAnimation"]:::new
    S3 --> S4["Posizioni virtuali = iniziali<br/>servo.write() PRIMA dell'attach"]:::new
    S4 --> S5["enqueue(buildResetSequence())"]
    S5 --> S6[resetInProgress = true]
    S6 --> S7{coda non vuota<br/>o servo attivo?}
    S7 -- sì --> S8{"servo attivi < MAX_ACTIVE_SERVOS<br/>e coda non vuota?"}:::new
    S8 -- sì --> S9[popCommand<br/>processCommand]
    S8 -- no --> S10
    S9 --> S10[checkServoDetach<br/>updateLEDs<br/>delay 50 ms]
    S10 --> S7
    S7 -- no --> S11[detach di tutti i servo]
    S11 --> S12["raimbowLEDS(false) (corretto)"]:::new
    S12 --> S13["invia READY"]:::new
    S13 --> S14([loop])
    classDef new fill:#d1e7dd,stroke:#146c43,color:#000
```

### loop()

```mermaid
flowchart TD
    L0([loop]) --> L1["processIncomingSerial<br/>separatore finale SEMPRE, coda max 240 car.<br/>risponde ACK n, annulla la modalità random"]:::new
    L1 --> L2{in attesa wait?}
    L2 -- sì --> L6
    L2 -- no --> L4{"servo attivi < MAX_ACTIVE_SERVOS<br/>e coda non vuota?"}:::new
    L4 -- sì --> L5[popCommand<br/>processCommand]
    L4 -- no --> L6
    L5 --> L6{"a riposo e modalità random?<br/>(comando random o 60 s senza seriale)"}
    L6 -- sì --> L7["processPeriodicTasks<br/>ACCODA i movimenti: un motore alla volta"]:::new
    L6 -- no --> L7a{"a riposo e 20 s senza movimenti?"}:::new
    L7a -- sì --> L7b["resetPositions(false)<br/>F[m][ultima posizione] in testa alla coda"]:::new
    L7a -- no --> L8
    L7 --> L8
    L7b --> L8["processSonar (tatto)"]:::new
    L8 --> L9["laser acceso da > 2 min? → OFF"]:::new
    L9 --> L10["checkServoDetach<br/>hold = 250 ms + 3 ms/°, max 500 ms<br/>(forzati: 500 ms)"]:::new
    L10 --> L11{"passato a riposo?"}:::new
    L11 -- sì --> L12["invia IDLE"]:::new
    L11 -- no --> L13
    L12 --> L13["updateLEDs<br/>animazione > impulso | lampeggio | stato"]:::new
    L13 --> L0
    classDef new fill:#d1e7dd,stroke:#146c43,color:#000
```

### processCommand()

```mermaid
flowchart TD
    C0([comando]) --> C2{w / wait?}
    C2 -- sì --> C3["wait: numero dopo 'w' o 'wait'<br/>(w200 = 200 ms)"]:::new
    C2 -- no --> C4{random / rand0 / rand1?}
    C4 -- sì --> C5[attiva / disattiva la modalità random]:::new
    C4 -- no --> C6{read?}
    C6 -- sì --> C7[posizioni + laser + distanza]:::new
    C6 -- no --> C8{"check / reset / restore / raimbow?"}
    C8 -- sì --> C9["macro espansa in TESTA alla coda<br/>(queueFront)"]:::new
    C8 -- no --> C10{"guard / delta / sonar / touch / help?"}:::new
    C10 -- sì --> C11[configurazione / diagnostica]:::new
    C10 -- no --> C12{"gesto?<br/>hello, shy, yes, no"}:::new
    C12 -- sì --> C13["sequenza del gesto + F[m][posa precedente]<br/>in testa alla coda"]:::new
    C12 -- no --> C14{L1 / L0?}:::new
    C14 -- sì --> C15[laser ON / OFF]:::new
    C14 -- no --> C16{P0..P3?}
    C16 -- sì --> C17[posizione predefinita<br/>moveMotor]
    C16 -- no --> C18{"inizia con F?"}:::new
    C18 -- sì --> C19[force = true, togli F]:::new
    C18 -- no --> C20
    C19 --> C20["[m][angolo]: motore 0..3, angolo 1..3 cifre<br/>altrimenti ERROR"]:::new
    C20 --> C21[moveMotor]
    classDef new fill:#d1e7dd,stroke:#146c43,color:#000
```

---

## 3. Tatto (HC-SR04)

Una misura ogni 100 ms. La zona cambia dopo 3 misure concordi, e tra due reazioni passano almeno 4 s.

```mermaid
stateDiagram-v2
    [*] --> LONTANO
    LONTANO --> PRESENZA: d < 40 cm
    PRESENZA --> TOCCO: d < 8 cm
    LONTANO --> TOCCO: d < 8 cm
    TOCCO --> PRESENZA: 8 ≤ d < 40
    PRESENZA --> LONTANO: d ≥ 40 o nessun eco
    TOCCO --> LONTANO: d ≥ 40 o nessun eco

    note right of PRESENZA
      invia PRESENCE d
      se arriva da LONTANO e la coda è vuota: gesto "hello"
      esce dalla modalità random
      LED rosso: battito lento
    end note
    note right of TOCCO
      invia TOUCH d
      laser OFF subito
      gesto "shy" (si ritrae, poi torna)
      LED rosso: lampeggio rapido
    end note
    note left of LONTANO
      invia GONE
    end note
```

---

## 4. Python ↔ Arduino: controllo di flusso

```mermaid
sequenceDiagram
    participant P as ZiphiusV3.py
    participant A as Arduino (Claudio)
    P->>A: apre la porta seriale (l'Uno si resetta)
    A-->>P: ... reset iniziale ...
    A-->>P: READY
    loop per ogni pacchetto di 4 comandi
        P->>A: A cmd1 X cmd2 X cmd3 X cmd4 Z
        A-->>P: ACK n
        A-->>P: Motor m moved to ang / Laser ON / ...
        A-->>P: IDLE
    end
```

Il pacchetto successivo parte solo dopo `IDLE`, cioè quando la coda è vuota e i servo sono staccati. Prima Python inviava a tempo fisso e leggeva una sola riga di risposta: le risposte si accumulavano e i comandi si fondevano.
