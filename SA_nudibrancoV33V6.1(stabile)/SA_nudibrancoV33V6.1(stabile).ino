#include <Servo.h>

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
const int P0_back_down    = 180;
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
const unsigned long servoDetachDelay = 500; // ms

// Tempi di movimento (valori default modificabili dal codice)
const unsigned long movementTime  = 500; // tempo esecuzione movimento (ms)
const unsigned long movementDelay = 500; // ritardo tra movimenti (ms)

// Flag per la modalità di attach/detach:
// Se true: ogni movimento forza attach/detach, se false: modalità "intelligente"
bool forceAlwaysAttach = true;

// =====================================================
// Gestione della coda dei comandi
// =====================================================
// Marcatori configurabili (default: A = inizio, Z = fine, X = separatore)
char CMD_START     = 'A';
char CMD_END       = 'Z';
char CMD_SEPARATOR = 'X';
// Coda dei comandi (stringa concatenata; verrà processata in sequenza)
String commandQueue = "";

// =====================================================
// Variabile globale per il comando wait
// =====================================================
unsigned long waitUntil = 0;
// Variabile globale per indicare che si sta eseguendo la sequenza di reset iniziale
bool resetInProgress = false;


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

// =====================================================
// Gestione LED (impulsi brevi)
// =====================================================
struct LEDImpulse {
  int pin;
  bool active;
  unsigned long startTime;
  unsigned long duration;
};

const int NUM_LEDS = 4;
LEDImpulse leds[NUM_LEDS];
const int ledPins[NUM_LEDS] = {2, 4, 12, 13}; // Giallo, Blu, Rosso, Verde
const char* ledNames[NUM_LEDS] = {"Yellow", "Blue", "Red", "Green"};

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
void attachMotor(int motor);
void detachMotor(int motor);
void updateLEDs();
void impulseLED(int ledIndex, unsigned long duration = 500);
void raimbowLEDS(bool ordine = true); // true = crescente / false = decrescente
void processPeriodicTasks();
void resetPositions(bool reset); // reset=true => posizione iniziale, false => ultima posizione comandata
void sendResponse(String msg);


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
  Serial.begin(9600);
  delay(1000);
  // Configura i LED
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].pin = ledPins[i];
    leds[i].active = false;
    leds[i].duration = 500; // default 500 ms
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }

  raimbowLEDS(true);
  
  // Inizializza le variabili "virtuali" per i motori
  // In questo caso le impostiamo ai valori correnti, perché il reset iniziale forzerà i movimenti
  for (int i = 0; i < NUM_MOTORS; i++){
    currentPositions[i] = initPositions[i];
    lastCmdPositions[i] = initPositions[i];
    lastServoMoveTime[i] = millis();
    servoAttached[i] = false;
  }
  
  // Costruisce la sequenza di reset utilizzando la funzione buildResetSequence()
  // Ad esempio, se buildResetSequence() restituisce "P0uXP1cXP2mXP3o"
  String resetSequence = buildResetSequence();
  
  // Inserisce la sequenza di reset nella coda dei comandi
  commandQueue = resetSequence + CMD_SEPARATOR;
  sendResponse("Initial reset sequence inserted: " + resetSequence);
  
  // Attiva il flag per forzare i movimenti
  resetInProgress = true;
  
  // Processa la coda dei comandi fino a che la sequenza di reset non è esaurita
  // In questo modo ogni comando della sequenza verrà eseguito forzando il movimento
  while (commandQueue.length() > 0 || anyServoActive()) {
    if (!anyServoActive() && commandQueue.length() > 0) {
      int sepIndex = commandQueue.indexOf(CMD_SEPARATOR);
      String cmd;
      if (sepIndex >= 0) {
        cmd = commandQueue.substring(0, sepIndex);
        commandQueue = commandQueue.substring(sepIndex + 1);
      } else {
        cmd = commandQueue;
        commandQueue = "";
      }
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
  sendResponse("Initial reset sequence completed.");
}

// =====================================================
// Loop principale
// =====================================================
void loop() {
  // Gestione lettura seriale (non bloccante)
  processIncomingSerial();

  // Se la flag DEBUG_Hard_print è attiva, stampa lo stato della coda
  if (DEBUG_Hard_print && commandQueue != "") {
    Serial.print("CommandQueue: ");
    Serial.println(commandQueue);
  }
  
  // Se siamo in fase di attesa, non processiamo la coda dei comandi
  if (millis() < waitUntil) {
    // Facoltativamente, puoi stampare un messaggio di debug
    if (DEBUG_Hard_print) {
      Serial.print("Waiting... remaining: ");
      Serial.println(waitUntil - millis());
    }
    impulseLED(0, 500); // GIALLO
  }
  else if (!anyServoActive() && commandQueue.length() > 0) {
    int sepIndex = commandQueue.indexOf(CMD_SEPARATOR);
    String cmd;
    if (sepIndex >= 0) {
      cmd = commandQueue.substring(0, sepIndex);
      commandQueue = commandQueue.substring(sepIndex + 1);
    } else {
      cmd = commandQueue;
      commandQueue = "";
    }
    if (cmd.length() > 0) {
      processCommand(cmd);
    }
  }
  
  // Esegui le attività periodiche solo se la coda è vuota
  if ((millis() - lastSerialTime > serialTimeout) && !anyServoActive() && commandQueue.length() == 0) {
    processPeriodicTasks();
  }
  
  // Verifica se è il momento di distaccare i servomotori
  checkServoDetach(false);
  
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
    serialBuffer += c;
    if (c == CMD_END) {
      int startIdx = serialBuffer.indexOf(CMD_START);
      int endIdx = serialBuffer.lastIndexOf(CMD_END);
      if (startIdx >= 0 && endIdx > startIdx) {
        String cmds = serialBuffer.substring(startIdx + 1, endIdx);
        // Aggiunge i comandi estratti alla coda
        commandQueue += cmds;
        // Se non ci sono separatori, aggiungine uno finale
        if (cmds.indexOf(CMD_SEPARATOR) < 0) {
          commandQueue += CMD_SEPARATOR;
        }
      } else {
        sendResponse("Invalid command format.");
      }
      serialBuffer = "";
      lastSerialTime = millis();
    }
  }
}


// =====================================================
// Processa un singolo comando
// =====================================================
void processCommand(String command) {
  command.trim();
  if (DEBUG_print) {
    Serial.print("Processing command: ");
    Serial.println(command);
  }
  lastSerialTime = millis();  // Aggiorna il timestamp della seriale
  
  // Impulso LED per indicare la ricezione del comando (es. LED Rosso, index 2)
  impulseLED(2, 200);
  
  // --- Nuova gestione per il comando "wait" o "w" ---
  if (command.startsWith("wait") || command.startsWith("WAIT") || command.startsWith("w") || command.startsWith("W")) {
    int waitTime = 500; // default in ms
    // Se il comando contiene un valore numerico, lo estrae:
    if (command.length() > 4) {
      String timeStr;
      if (command.startsWith("wait"))
        timeStr = command.substring(4);
      else // comando inizia con "w"
        timeStr = command.substring(1);
      waitTime = timeStr.toInt();
      if (waitTime <= 0)
        waitTime = 500;
    }
    waitUntil = millis() + waitTime;
    sendResponse("Waiting for " + String(waitTime) + " ms");
    return;
  }
  // --- Fine gestione comando wait ---


  // Comandi ad alta priorità
  if (command.equalsIgnoreCase("random") || command.equalsIgnoreCase("rand")) {
    raimbowLEDS(false);
    sendResponse("Periodic mode activated.");
    return;
  }
  if (command.equalsIgnoreCase("read")) {
    String pos = "Positions: ";
    for (int i = 0; i < NUM_MOTORS; i++) {
      pos += "M" + String(i) + ":" + String(currentPositions[i]) + " ";
    }
    sendResponse(pos);
    return;
  }
  if (command.equalsIgnoreCase("check")) {
    raimbowLEDS(true);
    // Inserisci nella coda la sequenza di reset per tutti i motori, ad es.:
    // Per il motore 0: "P0bdXP0uXP0fdXP0u"
    // Per il motore 1: "P1lXP1cXP1rXP1c"
    // Per il motore 2: "P2cXP2mmXP2lXP2c"
    // Per il motore 3: "P3oXP3ooXP3cXP3o"
    // Puoi concatenare queste stringhe, separandole (se vuoi che vengano eseguite in sequenza) oppure
    // inserirle in coda separatamente.
    String resetSequence = "resetX"
                           "wait1000X"
                           "P0bdXP0uXP0fdXP0uX"
                           "waitX"
                           "P1rXP1cXP1llXP1cX" 
                           "waitX"
                           "P2cXP2mmXP2lXP2cX" 
                           "waitX"
                           "P3oXP3ooXP3cXP3oX"
                           "wait1000X"
                           "reset";
    // Se vuoi inserire il comando nella coda come un unico blocco,
    // puoi usare direttamente il formato completo con delimitatori A e Z, oppure affidarti
    // al sistema di separazione già esistente.
    // Esempio: se la coda è gestita senza marcatori interni (dato che li aggiungiamo via seriale),
    // basta concatenare i comandi con il separatore:
    commandQueue += resetSequence + CMD_SEPARATOR;
    sendResponse("Check sequence inserted in queue.");
    return;
  }
  // Implementazione del comando "reset" in modo analogo a "check"
  if (command.equalsIgnoreCase("reset")) {
    // Attiva il flag per forzare i movimenti
    // resetInProgress = true;
    // Costruisce la sequenza di reset partendo dai comandi associati alle posizioni iniziali
    String resetSequence = buildResetSequence();
    // Inserisce nella coda la sequenza, aggiungendo il separatore finale
    commandQueue +=  resetSequence;
    sendResponse("Reset sequence inserted in queue: " + resetSequence);
    return;
  }
  if (command.equalsIgnoreCase("raimbow") || command.equalsIgnoreCase("lgbt")) {
    raimbowLEDS(true);
    String resetSequence = "wait2000X"
                           "resetX"
                           "wait2000";
    commandQueue += resetSequence + CMD_SEPARATOR;
    delay(100);
    raimbowLEDS(false);
    sendResponse("LGBT activated.");
    return;
  }


  // Comandi specifici per motore (P0, P1, P2, P3)
  if (command.startsWith("P0")) { moveMotorP0(command); return; }
  if (command.startsWith("P1")) { moveMotorP1(command); return; }
  if (command.startsWith("P2")) { moveMotorP2(command); return; }
  if (command.startsWith("P3")) { moveMotorP3(command); return; }
  
  // Comando nel formato [motor][angle] (es. "0100": motore 0 -> 100 gradi)
  char motorChar = command.charAt(0);
  int motor = motorChar - '0';
  if (motor < 0 || motor >= NUM_MOTORS) {
    sendResponse("Invalid motor number.");
    return;
  }
  int angle = command.substring(1).toInt();
  moveMotor(motorChar, angle, resetInProgress);
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

// Se forzato oppure se il tempo di attesa è trascorso, distacca i servomotori
void checkServoDetach(bool force) {
  unsigned long now = millis();
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (servoAttached[i] && (force || (now - lastServoMoveTime[i] >= servoDetachDelay))) {
      detachMotor(i);
    }
  }
}

// Ritorna true se almeno un motore è attivo
bool anyServoActive() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (servoAttached[i]) return true;
  }
  return false;
}

// =====================================================
// Gestione LED
// =====================================================
// Accende il LED indicato per la durata specificata (non bloccante)
void impulseLED(int ledIndex, unsigned long duration) {
  if (ledIndex < 0 || ledIndex >= NUM_LEDS) return;
  digitalWrite(leds[ledIndex].pin, HIGH);
  leds[ledIndex].active = true;
  leds[ledIndex].startTime = millis();
  leds[ledIndex].duration = duration;
}

// Spegne i LED che hanno superato il tempo di impulso
void updateLEDs() {
  unsigned long now = millis();
  for (int i = 0; i < NUM_LEDS; i++) {
    if (leds[i].active && (now - leds[i].startTime >= leds[i].duration)) {
      digitalWrite(leds[i].pin, LOW);
      leds[i].active = false;
    }
  }
}

void raimbowLEDS(bool ordine = true) { // true = crescente / false = decrescente
  if (ordine) {
    for (int i = 0; i < NUM_LEDS; i++) {
      impulseLED(i, 1000);  // Accende il LED i per 1000 ms
      delay(400);           // Attende 500 ms prima di accendere il LED successivo
    }
  } else {
    for (int i = NUM_LEDS-1; i > NUM_LEDS; i--) {
      impulseLED(i, 1000);  // Accende il LED i per 1000 ms
      delay(400);           // Attende 500 ms prima di accendere il LED successivo
    }
  }
}


// =====================================================
// Attività Periodiche (movimenti random)
// =====================================================
void processPeriodicTasks() {
  unsigned long now = millis();
  if (now - lastRandomMoveTime >= randomMoveInterval) {
    if (!anyServoActive()) {
      // Per ciascun motore, la variazione dell'angolo è esattamente -DELTA_RAND, 0 oppure +DELTA_RAND.
      for (int i = 0; i < NUM_MOTORS; i++) {
        int r = random(0, 3);  // genera 0, 1, o 2
        int delta = (r == 0) ? -DELTA_RAND : (r == 1) ? DELTA_RAND : 0;
        int newAngle = currentPositions[i] + delta;
        newAngle = constrain(newAngle, motorLimits[i][0], motorLimits[i][1]);
        moveMotor('0' + i, newAngle, resetInProgress);
      }
      lastRandomMoveTime = now;
      sendResponse("Random movement executed.");
    }
  }
}

// =====================================================
// Reset posizioni
// =====================================================
// Se reset=true, riporta i motori alle posizioni iniziali,
// altrimenti ripristina l'ultima posizione comandata.
void resetPositions(bool reset) {
  impulseLED(2, 200); // RED
  if (anyServoActive()) return;
  for (int i = 0; i < NUM_MOTORS; i++) {
    int target = reset ? initPositions[i] : lastCmdPositions[i];
    moveMotor('0' + i, target, true);
  }
  impulseLED(2, 200); // RED
}

// =====================================================
// Movimento dei motori
// =====================================================
void moveMotor(byte motorChar, int angle, bool force = false) {
  int motor = motorChar - '0';
  if (motor < 0 || motor >= NUM_MOTORS) {
    sendResponse("Invalid motor number in moveMotor.");
    return;
  }
  angle = constrain(angle, motorLimits[motor][0], motorLimits[motor][1]);
  if (!force && (abs(angle - currentPositions[motor]) < DELTA_ANG)) {
    if (DEBUG_print) {
      Serial.print("Motor ");
      Serial.print(motor);
      Serial.println(" change below threshold.");
    }
    return;
  }
  if (forceAlwaysAttach || !servoAttached[motor])
    attachMotor(motor);
  servos[motor].write(angle);
  currentPositions[motor] = angle;
  lastServoMoveTime[motor] = millis();
  servoAttached[motor] = true;
  impulseLED(1, 200); // BLU
  sendResponse("Motor " + String(motor) + " moved to " + String(angle));
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
  else { sendResponse("Invalid P0 command."); return; }
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
  else { sendResponse("Invalid P1 command."); return; }
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
  else { sendResponse("Invalid P2 command."); return; }
  moveMotor('2', targetAngle, resetInProgress);
  lastCmdPositions[2] = targetAngle;
}

void moveMotorP3(String command) {
  int targetAngle = currentPositions[3];
  if (command == "P3o")       targetAngle = P3_open;
  else if (command == "P3c")  targetAngle = P3_closed;
  else if (command == "P3oo") targetAngle = P3_openWide;
  else { sendResponse("Invalid P3 command."); return; }
  moveMotor('3', targetAngle, resetInProgress);
  lastCmdPositions[3] = targetAngle;
}


// =====================================================
// Comunicazione verso il PC/Python
// =====================================================
void sendResponse(String msg) {
  Serial.println(msg);
}
