#include <Servo.h>

// =====================================================
// CLAUDIO - braccio robotico con laser
// Arduino Uno + 4 servo SG90 + laser KY-008 + HC-SR04 + 4 LED
// =====================================================

// =====================================================
// Alimentazione dei servomotori
// =====================================================
// Numero massimo di servomotori alimentati (attach) contemporaneamente.
// 1 = servo alimentati dal 5V di Arduino/USB (~500 mA disponibili, un SG90
//     in stallo ne assorbe ~650-750): un solo motore alla volta.
// 2..4 = alimentatore esterno 5V (>= 1 A per servo, GND in comune con Arduino).
//     Con 2 motori attivi il laser puo' tracciare diagonali (M0 e M1 insieme).
const int MAX_ACTIVE_SERVOS = 1;

// =====================================================
// Configurazione Servomotori
// =====================================================
const int NUM_MOTORS = 4;
Servo servos[NUM_MOTORS];
const int servoPins[NUM_MOTORS] = {6, 5, 11, 10};

// Limiti degli angoli per ciascun motore
const int motorLimits[NUM_MOTORS][2] = {
  {0, 160},   // M0: Laser   (verticale)
  {0, 180},   // M1: Corpo   (orizzontale)
  {30, 130},  // M2: Braccio (longitudinale)
  {30, 100}   // M3: Pinza
};

// Posizioni predefinite per ciascun motore
// M0
const int P0_back_down    = 160; // era 180: fuori dal limite di M0 (160), veniva troncato
const int P0_back_up      = 125;
const int P0_up           = 80;
const int P0_forward_up   = 40;
const int P0_forward_down = 0;
// M1
const int P1_extrema_sinistra = 180;
const int P1_sinistra         = 150;
const int P1_centro           = 110;
const int P1_destra           = 50;
const int P1_extrema_destra   = 0;
// M2
const int P2_max  = 130;
const int P2_med3 = 105;
const int P2_med2 = 75;
const int P2_med1 = 50;
const int P2_min  = 30;
// M3
const int P3_openWide = 100;
const int P3_open     = 65;
const int P3_closed   = 30;

// Posizioni iniziali (default) e correnti
int initPositions[NUM_MOTORS]    = {P0_up, P1_centro, P2_med1, P3_open};
int currentPositions[NUM_MOTORS] = {P0_up, P1_centro, P2_med1, P3_open};
int lastCmdPositions[NUM_MOTORS] = {P0_up, P1_centro, P2_med1, P3_open};

// Array di comandi di reset associati ad ogni motore.
// funzione che costruisce la sequenza di reset in modo automatico: buildResetSequence
// Questi valori vanno scelti in base al significato che abbiamo dato nelle variabili iniziali.
// Ad esempio:
// - Per il motore 0, se la posizione iniziale è P0_up, associamo "P0u".
// - Per il motore 1, se la posizione iniziale è P1_centro, associamo "P1c".
// - Per il motore 2, se la posizione iniziale è P2_med1, associamo "P2m".
// - Per il motore 3, se la posizione iniziale è P3_open, associamo "P3o".
const char* resetCodes[NUM_MOTORS] = {"P0u", "P1c", "P2m", "P3o"};


// =====================================================
// Controllo movimenti e attach/detach dei servomotori
// =====================================================
unsigned long lastServoMoveTime[NUM_MOTORS] = {0, 0, 0, 0};
bool servoAttached[NUM_MOTORS] = {false, false, false, false};
const unsigned long servoDetachDelay = 500; // ms, tempo massimo di alimentazione per movimento

// Tempo di alimentazione proporzionale all'ampiezza del movimento:
// hold = servoMinHold + |delta| * servoMsPerDegree, limitato a servoDetachDelay.
// SG90: ~0.1 s/60° a 4.8 V senza carico (~1.7 ms/°); 3 ms/° lascia margine per la caduta di tensione su USB.
// I movimenti forzati (ripristino posizione) usano sempre servoDetachDelay,
// perche' la posizione reale del servo spinto a mano non e' nota.
const unsigned long servoMinHold     = 250; // ms
const unsigned long servoMsPerDegree = 3;   // ms/grado
unsigned long servoHoldTime[NUM_MOTORS] = {servoDetachDelay, servoDetachDelay, servoDetachDelay, servoDetachDelay};

// Tempi di movimento (valori default modificabili dal codice)
const unsigned long movementTime  = 500; // tempo esecuzione movimento (ms)
const unsigned long movementDelay = 500; // ritardo tra movimenti (ms)

// Flag per la modalità di attach/detach:
// Se true: ogni movimento forza attach/detach, se false: modalità "intelligente"
bool forceAlwaysAttach = true;

// =====================================================
// Ritorno in posizione (i servo staccati possono essere spostati a mano)
// =====================================================
// Gli SG90 non forniscono la posizione reale: se dopo autoRestoreInterval ms
// senza movimenti Claudio e' fermo, ri-afferma l'ultima posizione comandata
// motore per motore con resetPositions(false). 0 = disattivato (comando "guard<ms>").
unsigned long autoRestoreInterval = 20000;
unsigned long lastRestoreTime = 0;

// =====================================================
// Gestione della coda dei comandi
// =====================================================
// Marcatori configurabili (default: A = inizio, Z = fine, X = separatore)
char CMD_START     = 'A';
char CMD_END       = 'Z';
char CMD_SEPARATOR = 'X';
// Coda dei comandi (stringa concatenata; verrà processata in sequenza)
String commandQueue = "";
// Lunghezza massima della coda (la RAM dell'Uno e' 2 KB)
const unsigned int MAX_QUEUE_LEN  = 240;
const unsigned int MAX_SERIAL_BUF = 128;

// =====================================================
// Variabile globale per il comando wait
// =====================================================
unsigned long waitUntil = 0;
// Variabile globale per indicare che si sta eseguendo la sequenza di reset iniziale
bool resetInProgress = false;
// true mentre ci sono comandi da eseguire: al passaggio a riposo si invia "IDLE"
bool wasBusy = false;


// =====================================================
// Comunicazione Serial e gestione tempistiche
// =====================================================
unsigned long lastSerialTime = 0;
const unsigned long serialTimeout = 60000; // 60 sec di inattività

// =====================================================
// Attività Periodica (modalità "random" dopo 1 min di inattività)
// =====================================================
unsigned long lastRandomMoveTime = 0;
const unsigned long randomMoveInterval = 4200; // circa 4.2 sec
int DELTA_RAND = 30;
int DELTA_ANG  = 10; // variazione minima per eseguire un movimento
bool periodicMode    = false; // forzata dal comando "random", annullata dal prossimo pacchetto seriale
bool periodicEnabled = true;  // "rand0" disattiva la modalità random automatica, "rand1" la riattiva

// =====================================================
// Laser (KY-008)
// =====================================================
const int LASER_PIN = 7;
bool laserOn = false;
unsigned long laserOnSince = 0;
const unsigned long laserMaxOnTime = 120000; // spegnimento di sicurezza dopo 2 min

// =====================================================
// Tatto: sensore a ultrasuoni HC-SR04
// =====================================================
const int TRIG_PIN = 8;
const int ECHO_PIN = 9;  // pin 9: PWM disabilitato dalla libreria Servo (Timer1), come ingresso digitale va bene
bool touchEnabled = true;
const unsigned long sonarInterval  = 100;   // ms tra due misure
const unsigned long sonarTimeoutUs = 15000; // ~2.5 m: oltre si considera "nessuno"
const int TOUCH_CM    = 8;   // sotto questa distanza: tocco
const int PRESENCE_CM = 40;  // sotto questa distanza: presenza
const int SONAR_CONFIRM = 3; // misure consecutive concordi per cambiare zona
const unsigned long touchCooldown = 4000; // ms minimi tra due reazioni
enum SonarZone { ZONE_FAR, ZONE_PRESENCE, ZONE_TOUCH };
byte sonarZone = ZONE_FAR;
byte candidateZone = ZONE_FAR;
byte candidateCount = 0;
int lastDistanceCm = -1;
unsigned long lastSonarTime = 0;
unsigned long lastTouchReaction = 0;

// =====================================================
// Gestione LED (impulsi brevi, lampeggi e stato)
// =====================================================
struct LEDImpulse {
  int pin;
  bool active;
  unsigned long startTime;
  unsigned long duration;
  // lampeggio a conteggio (es. errore: 3 lampeggi)
  byte blinkCount;
  unsigned long blinkPeriod;
  unsigned long blinkStart;
  bool output; // ultimo valore scritto sul pin
};

const int NUM_LEDS = 4;
LEDImpulse leds[NUM_LEDS];
const int ledPins[NUM_LEDS] = {2, 4, 12, 13}; // Giallo, Blu, Rosso, Verde
const char* ledNames[NUM_LEDS] = {"Yellow", "Blue", "Red", "Green"};
// Significato dei LED (stato di base, gli impulsi si sommano):
// Giallo: laser acceso (fisso) / attesa "wait" (lampeggio)
// Blu:    motore alimentato
// Rosso:  comando ricevuto (impulso), errore (3 lampeggi), presenza (battito lento), tocco (lampeggio rapido)
// Verde:  vivo (battito ogni 2 s), modalità random (lampeggio)

// Animazione arcobaleno non bloccante
const unsigned long RAINBOW_STEP = 400;  // ms tra un LED e il successivo
const unsigned long RAINBOW_ON   = 1000; // ms di accensione di ciascun LED
const unsigned long RAINBOW_LEN  = RAINBOW_STEP * (NUM_LEDS - 1) + RAINBOW_ON;
byte animMode = 0; // 0 = nessuna, 1 = crescente, 2 = decrescente, 3 = crescente + decrescente
unsigned long animStart = 0;

// =====================================================
// Debug flags
// =====================================================
bool DEBUG_print = false;
bool DEBUG_Hard_print = false; // Abilitato per la stampa dettagliata

// =====================================================
// Dichiarazione funzioni
// =====================================================
void processIncomingSerial();
void processCommand(String cmd);
void moveMotor(byte motor, int angle, bool force);
void checkServoDetach(bool force);
bool anyServoActive();
int activeServoCount();
void attachMotor(int motor);
void detachMotor(int motor);
void updateLEDs();
void impulseLED(int ledIndex, unsigned long duration = 500);
void blinkLED(int ledIndex, byte count, unsigned long period = 200);
void raimbowLEDS(bool ordine = true); // true = crescente / false = decrescente
void waitLEDAnimation();
void processPeriodicTasks();
void resetPositions(bool reset); // reset=true => posizione iniziale, false => ultima posizione comandata
void sendResponse(String msg);
void sendError(String msg);
void enqueue(const String &cmds);
void queueFront(const String &cmds);
String popCommand();
void setLaser(bool on);
void processSonar();
int readDistanceCm();
bool playGesture(String name);


// Funzioni per i comandi specifici (P0, P1, ecc.)
void moveMotorP0(String command);
void moveMotorP1(String command);
void moveMotorP2(String command);
void moveMotorP3(String command);
void resetMotor0();
void resetMotor1();
void resetMotor2();
void resetMotor3();


unsigned long setupStart = millis();
// Funzione che costruisce la sequenza di reset in modo automatico
String buildResetSequence() {
  String seq = "";
  for (int i = 0; i < NUM_MOTORS; i++) {
    seq += resetCodes[i];
    if (i < NUM_MOTORS - 1) {
      seq += CMD_SEPARATOR;  // aggiunge il separatore fra i comandi
    }
  }
  return seq;
}


// =====================================================
// Setup
// =====================================================
void setup() {
  // Laser spento prima di ogni altra cosa
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(9600);
  delay(1000);
  // Configura i LED
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].pin = ledPins[i];
    leds[i].active = false;
    leds[i].duration = 500; // default 500 ms
    leds[i].blinkCount = 0;
    leds[i].output = false;
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }

  raimbowLEDS(true);
  waitLEDAnimation();

  // Inizializza le variabili "virtuali" per i motori
  // In questo caso le impostiamo ai valori correnti, perché il reset iniziale forzerà i movimenti
  for (int i = 0; i < NUM_MOTORS; i++){
    currentPositions[i] = initPositions[i];
    lastCmdPositions[i] = initPositions[i];
    lastServoMoveTime[i] = millis();
    servoAttached[i] = false;
    servos[i].write(initPositions[i]); // primo impulso gia' corretto al primo attach
  }

  // Costruisce la sequenza di reset utilizzando la funzione buildResetSequence()
  // Ad esempio, se buildResetSequence() restituisce "P0uXP1cXP2mXP3o"
  String resetSequence = buildResetSequence();

  // Inserisce la sequenza di reset nella coda dei comandi
  enqueue(resetSequence);
  sendResponse("Initial reset sequence inserted: " + resetSequence);

  // Attiva il flag per forzare i movimenti
  resetInProgress = true;

  // Processa la coda dei comandi fino a che la sequenza di reset non è esaurita
  // In questo modo ogni comando della sequenza verrà eseguito forzando il movimento
  while (commandQueue.length() > 0 || anyServoActive()) {
    if (activeServoCount() < MAX_ACTIVE_SERVOS && commandQueue.length() > 0) {
      String cmd = popCommand();
      if (cmd.length() > 0) {
        // Qui viene chiamato processCommand() che, grazie al flag resetInProgress,
        // dovrà passare force=true alle funzioni che invocano moveMotor().
        processCommand(cmd);
      }
    }
    // Aggiorna la logica di detach e gli impulsi LED
    checkServoDetach(false);
    updateLEDs();
    delay(50); // Breve pausa per evitare loop troppo serrati
  }

  // Termina la modalità di reset forzato
  resetInProgress = false;

  // Al termine del reset iniziale, distacca i servomotori (stato "pulito")
  for (int i = 0; i < NUM_MOTORS; i++){
    if (servoAttached[i]) {
      detachMotor(i);
    }
  }
  delay(1000);
  raimbowLEDS(false);
  waitLEDAnimation();
  lastSerialTime = millis();
  lastRestoreTime = millis();
  sendResponse(F("Initial reset sequence completed."));
  sendResponse(F("READY"));
}

// =====================================================
// Loop principale
// =====================================================
void loop() {
  // Gestione lettura seriale (non bloccante)
  processIncomingSerial();

  // Se la flag DEBUG_Hard_print è attiva, stampa lo stato della coda
  if (DEBUG_Hard_print && commandQueue != "") {
    Serial.print(F("CommandQueue: "));
    Serial.println(commandQueue);
  }

  // Se siamo in fase di attesa, non processiamo la coda dei comandi
  if (millis() < waitUntil) {
    // Facoltativamente, puoi stampare un messaggio di debug
    if (DEBUG_Hard_print) {
      Serial.print(F("Waiting... remaining: "));
      Serial.println(waitUntil - millis());
    }
    // Il LED giallo lampeggia (stato gestito da updateLEDs)
  }
  else if (activeServoCount() < MAX_ACTIVE_SERVOS && commandQueue.length() > 0) {
    String cmd = popCommand();
    if (cmd.length() > 0) {
      processCommand(cmd);
    }
  }

  bool idle = !anyServoActive() && commandQueue.length() == 0 && millis() >= waitUntil;
  bool periodicActive = periodicMode || (periodicEnabled && (millis() - lastSerialTime > serialTimeout));

  // Esegui le attività periodiche solo se la coda è vuota
  if (periodicActive && idle) {
    processPeriodicTasks();
  }
  // Ritorno in posizione se qualcuno ha spostato i motori a mano
  else if (!periodicActive && idle && autoRestoreInterval > 0 &&
           (millis() - lastRestoreTime >= autoRestoreInterval)) {
    if (DEBUG_print) Serial.println(F("Auto restore."));
    resetPositions(false);
    lastRestoreTime = millis();
  }

  // Tatto
  processSonar();

  // Spegnimento di sicurezza del laser
  if (laserOn && (millis() - laserOnSince >= laserMaxOnTime)) {
    setLaser(false);
    sendResponse(F("Laser safety timeout."));
  }

  // Verifica se è il momento di distaccare i servomotori
  checkServoDetach(false);

  // Segnala il passaggio a riposo (usato da Python per il controllo di flusso)
  idle = !anyServoActive() && commandQueue.length() == 0 && millis() >= waitUntil;
  if (idle && wasBusy) {
    sendResponse(F("IDLE"));
  }
  wasBusy = !idle;

  // Aggiorna lo stato dei LED (per spegnere gli impulsi scaduti)
  updateLEDs();
}

// =====================================================
// Funzioni di gestione della seriale e coda
// =====================================================
void processIncomingSerial() {
  // Accumula caratteri in un buffer statico fino al marcatore di fine
  static String serialBuffer = "";
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') continue;
    serialBuffer += c;
    if (c == CMD_END) {
      int startIdx = serialBuffer.indexOf(CMD_START);
      int endIdx = serialBuffer.lastIndexOf(CMD_END);
      if (startIdx >= 0 && endIdx > startIdx) {
        String cmds = serialBuffer.substring(startIdx + 1, endIdx);
        if (commandQueue.length() + cmds.length() + 1 > MAX_QUEUE_LEN) {
          sendError(F("Queue full, packet discarded."));
        } else {
          // Aggiunge i comandi estratti alla coda, sempre chiusi da un separatore:
          // senza, l'ultimo comando si fonderebbe con il primo del pacchetto successivo
          enqueue(cmds);
          wasBusy = true; // garantisce un "IDLE" anche per pacchetti eseguiti nello stesso ciclo
          sendResponse(String(F("ACK ")) + commandQueue.length());
        }
        periodicMode = false;
      } else {
        sendError(F("Invalid command format."));
      }
      serialBuffer = "";
      lastSerialTime = millis();
    } else if (serialBuffer.length() > MAX_SERIAL_BUF) {
      serialBuffer = "";
      sendError(F("Serial buffer overflow."));
    }
  }
}

// Aggiunge uno o più comandi in fondo alla coda (sempre terminati dal separatore)
void enqueue(const String &cmds) {
  if (cmds.length() == 0) return;
  commandQueue += cmds;
  if (cmds.charAt(cmds.length() - 1) != CMD_SEPARATOR) commandQueue += CMD_SEPARATOR;
}

// Inserisce uno o più comandi in testa alla coda: le macro (reset, check, gesti)
// vengono espanse nel punto in cui compaiono, non in fondo alla coda
void queueFront(const String &cmds) {
  if (cmds.length() == 0) return;
  String seq = cmds;
  if (seq.charAt(seq.length() - 1) != CMD_SEPARATOR) seq += CMD_SEPARATOR;
  commandQueue = seq + commandQueue;
}

// Estrae il primo comando dalla coda
String popCommand() {
  int sepIndex = commandQueue.indexOf(CMD_SEPARATOR);
  String cmd;
  if (sepIndex >= 0) {
    cmd = commandQueue.substring(0, sepIndex);
    commandQueue = commandQueue.substring(sepIndex + 1);
  } else {
    cmd = commandQueue;
    commandQueue = "";
  }
  return cmd;
}

// true se la stringa contiene solo cifre (1..3)
bool isAngle(const String &s) {
  if (s.length() == 0 || s.length() > 3) return false;
  for (unsigned int i = 0; i < s.length(); i++) {
    if (!isDigit(s.charAt(i))) return false;
  }
  return true;
}


// =====================================================
// Processa un singolo comando
// =====================================================
void processCommand(String command) {
  command.trim();
  if (command.length() == 0) return;
  if (DEBUG_print) {
    Serial.print(F("Processing command: "));
    Serial.println(command);
  }
  // lastSerialTime NON viene aggiornato qui: i comandi generati internamente
  // (random, ritorno in posizione) non devono contare come attività seriale

  // Impulso LED per indicare la ricezione del comando (es. LED Rosso, index 2)
  impulseLED(2, 200);

  // --- Nuova gestione per il comando "wait" o "w" ---
  if (command.startsWith("wait") || command.startsWith("WAIT") || command.startsWith("w") || command.startsWith("W")) {
    int waitTime = 500; // default in ms
    // Estrae il valore numerico dopo "wait" oppure dopo "w" (es. "w200", "wait1000")
    String timeStr;
    if (command.length() >= 4 && command.substring(0, 4).equalsIgnoreCase("wait"))
      timeStr = command.substring(4);
    else // comando inizia con "w"
      timeStr = command.substring(1);
    if (timeStr.length() > 0) {
      waitTime = timeStr.toInt();
      if (waitTime <= 0)
        waitTime = 500;
    }
    waitUntil = millis() + waitTime;
    sendResponse(String(F("Waiting for ")) + waitTime + F(" ms"));
    return;
  }
  // --- Fine gestione comando wait ---


  // Comandi ad alta priorità
  if (command.equalsIgnoreCase("random") || command.equalsIgnoreCase("rand")) {
    raimbowLEDS(false);
    periodicMode = true;
    periodicEnabled = true;
    sendResponse(F("Periodic mode activated."));
    return;
  }
  if (command.equalsIgnoreCase("rand0")) {
    periodicMode = false;
    periodicEnabled = false;
    sendResponse(F("Periodic mode disabled."));
    return;
  }
  if (command.equalsIgnoreCase("rand1")) {
    periodicEnabled = true;
    sendResponse(F("Periodic mode enabled after inactivity."));
    return;
  }
  if (command.equalsIgnoreCase("read")) {
    String pos = "Positions: ";
    for (int i = 0; i < NUM_MOTORS; i++) {
      pos += "M" + String(i) + ":" + String(currentPositions[i]) + " ";
    }
    pos += String(F("L:")) + (laserOn ? 1 : 0) + F(" D:") + lastDistanceCm;
    sendResponse(pos);
    return;
  }
  if (command.equalsIgnoreCase("check")) {
    raimbowLEDS(true);
    // Sequenza di verifica di tutti i motori, espansa in testa alla coda:
    // Per il motore 0: "P0bdXP0uXP0fdXP0u"
    // Per il motore 1: "P1rXP1cXP1llXP1c"
    // Per il motore 2: "P2cXP2mmXP2lXP2c"
    // Per il motore 3: "P3oXP3ooXP3cXP3o"
    queueFront(F("resetX"
                 "wait1000X"
                 "P0bdXP0uXP0fdXP0uX"
                 "waitX"
                 "P1rXP1cXP1llXP1cX"
                 "waitX"
                 "P2cXP2mmXP2lXP2cX"
                 "waitX"
                 "P3oXP3ooXP3cXP3oX"
                 "wait1000X"
                 "reset"));
    sendResponse(F("Check sequence inserted in queue."));
    return;
  }
  // Implementazione del comando "reset" in modo analogo a "check"
  if (command.equalsIgnoreCase("reset")) {
    // Costruisce la sequenza di reset partendo dai comandi associati alle posizioni iniziali
    String resetSequence = buildResetSequence();
    // Inserisce in testa alla coda la sequenza, con il separatore finale
    queueFront(resetSequence);
    sendResponse("Reset sequence inserted in queue: " + resetSequence);
    return;
  }
  // Ritorno all'ultima posizione comandata (movimenti forzati)
  if (command.equalsIgnoreCase("restore")) {
    resetPositions(false);
    sendResponse(F("Restore sequence inserted in queue."));
    return;
  }
  if (command.equalsIgnoreCase("raimbow") || command.equalsIgnoreCase("lgbt")) {
    animMode = 3; // arcobaleno crescente + decrescente, non bloccante
    animStart = millis();
    queueFront(F("wait2000X"
                 "resetX"
                 "wait2000"));
    sendResponse(F("LGBT activated."));
    return;
  }
  // "guard<ms>": intervallo di ritorno in posizione, guard0 = disattivato
  if (command.startsWith("guard")) {
    autoRestoreInterval = (unsigned long)command.substring(5).toInt();
    lastRestoreTime = millis();
    sendResponse(String(F("Auto restore interval: ")) + autoRestoreInterval + F(" ms"));
    return;
  }
  // "delta<n>": soglia minima di movimento in gradi (1..90). Il disegno laser usa delta1, il default e' 10
  if (command.startsWith("delta")) {
    DELTA_ANG = constrain((int)command.substring(5).toInt(), 1, 90);
    sendResponse(String(F("DELTA_ANG: ")) + DELTA_ANG);
    return;
  }
  if (command.equalsIgnoreCase("sonar")) {
    sendResponse(String(F("Distance: ")) + readDistanceCm() + F(" cm"));
    return;
  }
  if (command.equalsIgnoreCase("touch0") || command.equalsIgnoreCase("touch1")) {
    touchEnabled = (command.charAt(5) == '1');
    sonarZone = ZONE_FAR;
    candidateCount = 0;
    sendResponse(touchEnabled ? F("Touch enabled.") : F("Touch disabled."));
    return;
  }
  if (command.equalsIgnoreCase("help")) {
    Serial.println(F("Commands: <m><ang> F<m><ang> P0bd/bu/u/fu/fd P1ll/l/c/r/rr P2f/l/mm/m/c P3oo/o/c"));
    Serial.println(F("w<ms> wait<ms> reset restore check raimbow random rand0 rand1 read"));
    Serial.println(F("L1 L0 guard<ms> delta<deg> sonar touch0 touch1 hello shy yes no help"));
    return;
  }
  if (playGesture(command)) return;

  // Laser: "L1" acceso, "L0" spento
  if (command == "L1" || command == "L0") {
    setLaser(command.charAt(1) == '1');
    return;
  }

  // Comandi specifici per motore (P0, P1, P2, P3)
  if (command.startsWith("P0")) { moveMotorP0(command); return; }
  if (command.startsWith("P1")) { moveMotorP1(command); return; }
  if (command.startsWith("P2")) { moveMotorP2(command); return; }
  if (command.startsWith("P3")) { moveMotorP3(command); return; }

  // Movimento forzato "F[motor][angle]" (es. "F0100"): ignora la soglia DELTA_ANG.
  // Usato per il ritorno in posizione e per il disegno laser a passi piccoli.
  bool force = resetInProgress;
  if (command.charAt(0) == 'F') {
    force = true;
    command = command.substring(1);
  }

  // Comando nel formato [motor][angle] (es. "0100": motore 0 -> 100 gradi)
  char motorChar = command.charAt(0);
  int motor = motorChar - '0';
  if (motor < 0 || motor >= NUM_MOTORS) {
    sendError(F("Invalid motor number."));
    return;
  }
  if (!isAngle(command.substring(1))) {
    sendError(F("Invalid angle."));
    return;
  }
  int angle = constrain(command.substring(1).toInt(), motorLimits[motor][0], motorLimits[motor][1]);
  moveMotor(motorChar, angle, force);
  lastCmdPositions[motor] = angle;
}

void attachMotor(int motor) {
  servos[motor].attach(servoPins[motor]);
  servoAttached[motor] = true;
}

void detachMotor(int motor) {
  servos[motor].detach();
  servoAttached[motor] = false;
}

// Se forzato oppure se il tempo di alimentazione è trascorso, distacca i servomotori
void checkServoDetach(bool force) {
  unsigned long now = millis();
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (servoAttached[i] && (force || (now - lastServoMoveTime[i] >= servoHoldTime[i]))) {
      detachMotor(i);
    }
  }
}

// Ritorna true se almeno un motore è attivo
bool anyServoActive() {
  return activeServoCount() > 0;
}

// Numero di motori attualmente alimentati
int activeServoCount() {
  int n = 0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (servoAttached[i]) n++;
  }
  return n;
}

// =====================================================
// Laser
// =====================================================
void setLaser(bool on) {
  digitalWrite(LASER_PIN, on ? HIGH : LOW);
  if (on) laserOnSince = millis(); // ogni "L1" riarma il timer di sicurezza
  laserOn = on;
  sendResponse(on ? F("Laser ON") : F("Laser OFF"));
}

// =====================================================
// Gesti: macro di comandi espanse in testa alla coda.
// Al termine ogni gesto riporta i motori usati all'ultima posizione comandata.
// =====================================================
bool playGesture(String name) {
  String seq;
  byte mask; // bit i = motore i usato dal gesto
  if (name.equalsIgnoreCase("hello"))    { seq = F("P3ooXP3cXP3ooXP3cXP3oo"); mask = 0b1000; }
  else if (name.equalsIgnoreCase("shy")) { seq = F("L0XP3cXP2cXw1500");       mask = 0b1100; }
  else if (name.equalsIgnoreCase("yes")) { seq = F("P0fuXP0uXP0fuXP0u");      mask = 0b0001; }
  else if (name.equalsIgnoreCase("no"))  { seq = F("P1lXP1rXP1l");            mask = 0b0010; }
  else return false;
  // Ritorno alla posa precedente (fotografata ora, prima che il gesto la modifichi)
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (mask & (1 << i)) {
      seq += CMD_SEPARATOR;
      seq += "F" + String(i) + String(lastCmdPositions[i]);
    }
  }
  queueFront(seq);
  sendResponse("Gesture: " + name);
  return true;
}

// =====================================================
// Tatto (HC-SR04)
// =====================================================
// Ritorna la distanza in cm, -1 se nessun eco entro sonarTimeoutUs
int readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long us = pulseIn(ECHO_PIN, HIGH, sonarTimeoutUs);
  if (us == 0) return -1;
  // velocità del suono ~343 m/s: andata e ritorno => 58.3 us/cm
  return (int)(us / 58);
}

void processSonar() {
  if (!touchEnabled) return;
  unsigned long now = millis();
  if (now - lastSonarTime < sonarInterval) return;
  lastSonarTime = now;

  lastDistanceCm = readDistanceCm();
  byte zone = ZONE_FAR;
  if (lastDistanceCm > 0 && lastDistanceCm < TOUCH_CM) zone = ZONE_TOUCH;
  else if (lastDistanceCm > 0 && lastDistanceCm < PRESENCE_CM) zone = ZONE_PRESENCE;

  // Debounce: la zona cambia solo dopo SONAR_CONFIRM misure concordi
  if (zone == sonarZone) { candidateCount = 0; return; }
  if (zone != candidateZone) { candidateZone = zone; candidateCount = 0; }
  if (++candidateCount < SONAR_CONFIRM) return;
  candidateCount = 0;

  byte oldZone = sonarZone;
  sonarZone = zone;
  bool canReact = (now - lastTouchReaction >= touchCooldown);

  if (zone == ZONE_TOUCH) {
    // Qualcuno e' vicinissimo: laser spento subito, Claudio si ritrae
    setLaser(false);
    periodicMode = false;
    lastSerialTime = now; // il contatto conta come attività
    blinkLED(2, 3, 150);
    sendResponse(String(F("TOUCH ")) + lastDistanceCm);
    if (canReact) { playGesture(F("shy")); lastTouchReaction = now; }
  } else if (zone == ZONE_PRESENCE) {
    periodicMode = false;
    lastSerialTime = now;
    sendResponse(String(F("PRESENCE ")) + lastDistanceCm);
    // Saluta solo chi arriva da lontano e se non sta eseguendo altro
    if (oldZone == ZONE_FAR && canReact && commandQueue.length() == 0) {
      playGesture(F("hello"));
      lastTouchReaction = now;
    }
  } else {
    sendResponse(F("GONE"));
  }
}

// =====================================================
// Gestione LED
// =====================================================
// Accende il LED indicato per la durata specificata (non bloccante)
void impulseLED(int ledIndex, unsigned long duration) {
  if (ledIndex < 0 || ledIndex >= NUM_LEDS) return;
  leds[ledIndex].active = true;
  leds[ledIndex].startTime = millis();
  leds[ledIndex].duration = duration;
}

// Fa lampeggiare il LED indicato count volte (non bloccante)
void blinkLED(int ledIndex, byte count, unsigned long period) {
  if (ledIndex < 0 || ledIndex >= NUM_LEDS) return;
  leds[ledIndex].blinkCount = count;
  leds[ledIndex].blinkPeriod = period;
  leds[ledIndex].blinkStart = millis();
}

// Stato di base di ciascun LED in funzione dello stato di Claudio
bool ledBaseState(int i, unsigned long now) {
  switch (i) {
    case 0: // Giallo
      if (laserOn) return true;
      if (now < waitUntil) return (now % 300) < 150;
      return false;
    case 1: // Blu
      return anyServoActive();
    case 2: // Rosso
      if (sonarZone == ZONE_TOUCH) return (now % 200) < 100;
      if (sonarZone == ZONE_PRESENCE) return (now % 1500) < 80;
      return false;
    case 3: // Verde
      if (periodicMode || (periodicEnabled && (now - lastSerialTime > serialTimeout)))
        return (now % 600) < 300;
      return (now % 2000) < 60;
  }
  return false;
}

// Calcola e scrive lo stato dei LED: animazione > (impulso | lampeggio | stato di base)
void updateLEDs() {
  unsigned long now = millis();
  unsigned long t = now - animStart;
  if (animMode != 0 && t >= (animMode == 3 ? 2 * RAINBOW_LEN : RAINBOW_LEN)) animMode = 0;

  for (int i = 0; i < NUM_LEDS; i++) {
    bool out;
    if (animMode != 0) {
      bool asc = (animMode == 1) || (animMode == 3 && t < RAINBOW_LEN);
      unsigned long tt = (animMode == 3 && t >= RAINBOW_LEN) ? t - RAINBOW_LEN : t;
      unsigned long k = asc ? i : (NUM_LEDS - 1 - i);
      out = (tt >= k * RAINBOW_STEP) && (tt < k * RAINBOW_STEP + RAINBOW_ON);
    } else {
      if (leds[i].active && (now - leds[i].startTime >= leds[i].duration)) {
        leds[i].active = false;
      }
      bool blink = false;
      if (leds[i].blinkCount > 0) {
        unsigned long bt = now - leds[i].blinkStart;
        if (bt >= leds[i].blinkCount * leds[i].blinkPeriod) leds[i].blinkCount = 0;
        else blink = (bt % leds[i].blinkPeriod) < leds[i].blinkPeriod / 2;
      }
      out = leds[i].active || blink || ledBaseState(i, now);
    }
    if (out != leds[i].output) {
      digitalWrite(leds[i].pin, out ? HIGH : LOW);
      leds[i].output = out;
    }
  }
}

// Avvia l'animazione arcobaleno (non bloccante)
void raimbowLEDS(bool ordine) { // true = crescente / false = decrescente
  animMode = ordine ? 1 : 2;
  animStart = millis();
}

// Attende la fine dell'animazione (solo nel setup)
void waitLEDAnimation() {
  while (animMode != 0) {
    updateLEDs();
    delay(10);
  }
}


// =====================================================
// Attività Periodiche (movimenti random)
// =====================================================
void processPeriodicTasks() {
  unsigned long now = millis();
  if (now - lastRandomMoveTime >= randomMoveInterval) {
    if (!anyServoActive()) {
      if (laserOn) setLaser(false);
      // Per ciascun motore, la variazione dell'angolo è esattamente -DELTA_RAND, 0 oppure +DELTA_RAND.
      // I movimenti vengono accodati: la coda li esegue un motore alla volta.
      for (int i = 0; i < NUM_MOTORS; i++) {
        int r = random(0, 3);  // genera 0, 1, o 2
        int delta = (r == 0) ? -DELTA_RAND : (r == 1) ? DELTA_RAND : 0;
        if (delta == 0) continue;
        int newAngle = currentPositions[i] + delta;
        newAngle = constrain(newAngle, motorLimits[i][0], motorLimits[i][1]);
        enqueue(String(i) + String(newAngle));
      }
      lastRandomMoveTime = now;
      sendResponse(F("Random movement executed."));
    }
  }
}

// =====================================================
// Reset posizioni
// =====================================================
// Se reset=true, riporta i motori alle posizioni iniziali,
// altrimenti ripristina l'ultima posizione comandata.
// I movimenti sono forzati ("F") e inseriti in testa alla coda: un motore alla volta.
void resetPositions(bool reset) {
  impulseLED(2, 200); // RED
  String seq = "";
  for (int i = 0; i < NUM_MOTORS; i++) {
    int target = reset ? initPositions[i] : lastCmdPositions[i];
    seq += "F" + String(i) + String(target);
    if (i < NUM_MOTORS - 1) seq += CMD_SEPARATOR;
  }
  queueFront(seq);
}

// =====================================================
// Movimento dei motori
// =====================================================
void moveMotor(byte motorChar, int angle, bool force = false) {
  int motor = motorChar - '0';
  if (motor < 0 || motor >= NUM_MOTORS) {
    sendError(F("Invalid motor number in moveMotor."));
    return;
  }
  angle = constrain(angle, motorLimits[motor][0], motorLimits[motor][1]);
  int delta = abs(angle - currentPositions[motor]);
  if (!force && (delta < DELTA_ANG)) {
    if (DEBUG_print) {
      Serial.print(F("Motor "));
      Serial.print(motor);
      Serial.println(F(" change below threshold."));
    }
    return;
  }
  // Prima la posizione, poi l'attach: il primo impulso e' gia' quello giusto
  servos[motor].write(angle);
  if (forceAlwaysAttach || !servoAttached[motor])
    attachMotor(motor);
  currentPositions[motor] = angle;
  lastServoMoveTime[motor] = millis();
  servoHoldTime[motor] = force ? servoDetachDelay
                               : constrain(servoMinHold + (unsigned long)delta * servoMsPerDegree,
                                           servoMinHold, servoDetachDelay);
  lastRestoreTime = millis();
  if (laserOn) laserOnSince = millis(); // il laser si sta muovendo: e' sotto controllo
  servoAttached[motor] = true;
  impulseLED(1, 200); // BLU
  sendResponse(String(F("Motor ")) + motor + F(" moved to ") + angle);
}

// =====================================================
// Comandi specifici per i motori
// =====================================================
void moveMotorP0(String command) {
  int targetAngle = currentPositions[0];
  if (command == "P0bd")       targetAngle = P0_back_down;
  else if (command == "P0bu")  targetAngle = P0_back_up;
  else if (command == "P0u")   targetAngle = P0_up;
  else if (command == "P0fu")  targetAngle = P0_forward_up;
  else if (command == "P0fd")  targetAngle = P0_forward_down;
  else { sendError(F("Invalid P0 command.")); return; }
  moveMotor('0', targetAngle, resetInProgress);
  lastCmdPositions[0] = targetAngle;
}

void moveMotorP1(String command) {
  int targetAngle = currentPositions[1];
  if (command == "P1ll")       targetAngle = P1_extrema_sinistra;
  else if (command == "P1l")   targetAngle = P1_sinistra;
  else if (command == "P1c")   targetAngle = P1_centro;
  else if (command == "P1r")   targetAngle = P1_destra;
  else if (command == "P1rr")  targetAngle = P1_extrema_destra;
  else { sendError(F("Invalid P1 command.")); return; }
  moveMotor('1', targetAngle, resetInProgress);
  lastCmdPositions[1] = targetAngle;
}

void moveMotorP2(String command) {
  int targetAngle = currentPositions[2];
  if (command == "P2f")       targetAngle = P2_max;
  else if (command == "P2l")  targetAngle = P2_med3;
  else if (command == "P2mm") targetAngle = P2_med2;
  else if (command == "P2m")  targetAngle = P2_med1;
  else if (command == "P2c")  targetAngle = P2_min;
  else { sendError(F("Invalid P2 command.")); return; }
  moveMotor('2', targetAngle, resetInProgress);
  lastCmdPositions[2] = targetAngle;
}

void moveMotorP3(String command) {
  int targetAngle = currentPositions[3];
  if (command == "P3o")       targetAngle = P3_open;
  else if (command == "P3c")  targetAngle = P3_closed;
  else if (command == "P3oo") targetAngle = P3_openWide;
  else { sendError(F("Invalid P3 command.")); return; }
  moveMotor('3', targetAngle, resetInProgress);
  lastCmdPositions[3] = targetAngle;
}


// =====================================================
// Comunicazione verso il PC/Python
// =====================================================
void sendResponse(String msg) {
  Serial.println(msg);
}

// Errore: messaggio con prefisso "ERROR" e 3 lampeggi del LED rosso
void sendError(String msg) {
  blinkLED(2, 3, 200);
  sendResponse("ERROR " + msg);
}
