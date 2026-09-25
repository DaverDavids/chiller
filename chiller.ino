/*
  Chiller - ESP32-C3 Super Mini firmware
  HVAC compressor + pump controller with a hysteretic capacitor charge
  interlock, mode-select switch, addressable status LEDs, WiFi (STA +
  captive-portal fallback), OTA, mDNS, flash-persisted settings, and a
  web UI for live status + manual function testing.

  Pin map:
    GPIO0   - ADC: capacitor voltage sense (charge interlock)
    GPIO1   - Capacitor charge output (LOW at boot)
    GPIO2   - ADC: 12V current sense (overcurrent gate)
    GPIO3   - Switch position 3 input (CHILLER ONLY) [active low, pull-up]
    GPIO4   - Timer on/off input
    GPIO5   - Fan output
    GPIO6   - Buzzer output
    GPIO7   - 12V enable output (powers the pump)
    GPIO8   - Compressor output (see CAPACITOR INTERLOCK below)
    GPIO9   - 5V enable output
    GPIO10  - Switch position 4 input (BOTH)       [active low, pull-up]
    GPIO20  - Switch position 2 input (PUMP ONLY)  [active low, pull-up]
    GPIO21  - Addressable status LED data line
    n/a     - Switch position 1 input (OFF) - NOT WIRED, see PIN_SWITCH_1

  MODE LOGIC (from 4-position switch):
    Switch inputs are ACTIVE LOW: a position is "active" when its pin is
    pulled to GND by the switch, otherwise the internal pull-up holds it
    HIGH.
    Position 1 (not wired) -> everything off
    Position 2 -> pump only (12V enable), compressor never requested
    Position 3 -> chiller only (compressor path)
    Position 4 -> pump AND chiller both requested simultaneously

  CAPACITOR INTERLOCK (the compressor start path):
    The capacitor is charged by holding the charge output (GPIO1) HIGH, and
    that output is only ever energized while the compressor is running. Once
    the compressor is off the capacitor bleeds back down, and the compressor
    output may not go HIGH again until the capacitor has been seen at or
    below CAP_DISCHARGED_ADC_MAX. So a start can never land on a charged
    capacitor, and a restart can never skip the discharge. The sequence is
    hysteretic on two thresholds of the capacitor sense (GPIO0) and lives
    entirely in updateCompressorInterlock(), which is the ONLY function that
    writes the charge output or the compressor output. There is no flag,
    endpoint, or state that reaches the compressor without passing through it.
    A start additionally requires the 12V current sense (GPIO2) to be clear of
    overcurrent, and charging must complete within PRECHARGE_TIMEOUT_MS or the
    fault latches rather than closing the contacts anyway.

  PUMP ANTI-SHORT-CYCLE:
    The 12V enable output (GPIO7) is NOT switched off the instant pump demand
    drops. It's held on for PUMP_OFF_DELAY_MS after demand goes away so that
    flipping through switch positions doesn't stop/restart the pump every time
    - see update12VEnable().

  SAFETY NOTE: PIN_CAP_CHARGE and PIN_COMPRESSOR are written in exactly one
  place, updateCompressorInterlock(). Do not add any other digitalWrite() on
  either pin elsewhere. PIN_12V_EN is written in exactly one place,
  update12VEnable().
*/

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h> // TODO: install "Adafruit NeoPixel" via Library Manager
#include <Secrets.h>   // must define MYSSIDIOT and MYPSKIOT
#include "html.h"

// ============================== DEBUG =================================
#define DEBUG 1
#if DEBUG
  #define DBG_BEGIN(x)      Serial.begin(x)
  #define DBG_PRINT(x)      Serial.print(x)
  #define DBG_PRINTLN(x)    Serial.println(x)
  #define DBG_PRINTF(...)   Serial.printf(__VA_ARGS__)
#else
  #define DBG_BEGIN(x)
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
  #define DBG_PRINTF(...)
#endif

// Throttle for the noisy/periodic debug prints (safety check, status, etc.)
// so serial doesn't get spammed every single loop() iteration.
const unsigned long DEBUG_PRINT_INTERVAL_MS = 2000; // TODO: tune to taste

// ============================= NETWORK =================================
#define HOSTNAME "chiller"
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000;
const unsigned long WIFI_RETRY_INTERVAL_MS  = 30000;

// ============================= PIN MAP ==================================
#define PIN_CAP_SENSE  0   // ADC1 channel, capacitor voltage sense
#define PIN_CAP_CHARGE 1   // capacitor charge output, LOW at boot
#define PIN_ADC_12V    2   // ADC1 channel, 12V current sense
#define PIN_TIMER_IN   4   // timer on/off input
#define PIN_FAN        5
#define PIN_BUZZER     6
#define PIN_12V_EN     7   // 12V enable - powers the pump
#define PIN_COMPRESSOR 8
#define PIN_5V_EN      9

// Mode-select switch inputs (one-of-four rotary/slide switch), active low.
#define PIN_SWITCH_1   -1  // position 1 - OFF. NOT WIRED: no pin assigned yet.
                          // Any negative value reads permanently inactive, so
                          // position 1 can never be selected until a real pin
                          // is set here. MODE_OFF is still reachable through
                          // the fail-safe path in getSwitchMode().
#define PIN_SWITCH_2   20  // position 2 - pump only
#define PIN_SWITCH_3   3   // position 3 - chiller only
#define PIN_SWITCH_4   10  // position 4 - pump + chiller

// Addressable status LEDs. GPIO21 was the hardware UART0 TX pin on most
// supermini boards - using it for LED data means Serial debug has to run over
// the native USB port (GPIO18/19) instead. TODO: confirm LED_COUNT.
#define LED_DATA_PIN   21
#define LED_COUNT      8   // PLACEHOLDER - set to actual strip length
Adafruit_NeoPixel statusLeds(LED_COUNT, LED_DATA_PIN, NEO_GRB + NEO_KHZ800);

// ==================== SAFETY THRESHOLDS (PSEUDO/TODO) ====================
// Capacitor charge interlock, read on PIN_CAP_SENSE. Two separate thresholds
// so the sequence is hysteretic - a start requires a proven-discharged
// capacitor first, then a proven-charged capacitor before the contacts close.
// TODO: calibrate both against the real capacitor charge curve on the bench.
int CAP_DISCHARGED_ADC_MAX = 500; // at/below this the capacitor counts as
                                  // discharged and a start may be armed
int CAP_CHARGED_ADC_MIN   = 3000; // at/above this the capacitor counts as
                                  // charged and the contacts may close
// 12V current sense (PIN_ADC_12V): the compressor is allowed ON only while the
// reading is at or below this. Higher reading == more current == unsafe.
int COMPRESSOR_SAFE_ADC_MAX = 500;      // placeholder, 0-4095 (12-bit ADC)
const uint8_t ADC_SAMPLE_COUNT = 8;     // simple averaging, tune later

// How long the charge output may be energized without the capacitor reaching
// CAP_CHARGED_ADC_MIN before the fault latches and the contacts stay open.
const unsigned long PRECHARGE_TIMEOUT_MS = 30000; // TODO: tune to taste

// Once the fault has latched, the chiller demand must be removed for this long
// before it clears, so a bad capacitor or charge path can't be re-energized by
// simply bouncing the switch.
const unsigned long FAULT_RESET_HOLD_MS = 5000; // TODO: tune to taste

// Pump anti-short-cycle hold time. 12V enable stays on this long after pump
// demand drops, so switching modes doesn't stop/restart the pump every time.
const unsigned long PUMP_OFF_DELAY_MS = 5000; // TODO: tune to taste

// ============================ MODES / STATE ==============================
enum ChillerMode {
  MODE_OFF = 0,
  MODE_PUMP_ONLY,
  MODE_CHILLER_ONLY,
  MODE_BOTH
};
ChillerMode currentMode = MODE_OFF;

enum ChillerState {
  STATE_IDLE,
  STATE_PRECHARGE,
  STATE_RUN,
  STATE_FAULT
};
ChillerState state = STATE_IDLE;

bool compressorRequested = false;  // demand flag only, NEVER write the pin from here
bool pumpRequested = false;        // demand flag only, NEVER write the pin from here

// Compressor start sequence, owned entirely by updateCompressorInterlock().
// This is the authoritative state of the capacitor/contacts - ChillerState
// above is only a display/LED view derived from it by runStateMachine().
enum CapSeq {
  CAP_SEQ_WAIT_DISCHARGE,  // not armed: contacts open, no charging. Needs to
                           // see the capacitor at/below CAP_DISCHARGED_ADC_MAX.
  CAP_SEQ_CHARGING,        // armed and requested: charging, contacts still open
  CAP_SEQ_RUNNING          // contacts closed
};
CapSeq capSeq = CAP_SEQ_WAIT_DISCHARGE;

bool compressorRunning = false;  // true only while the compressor output is HIGH
bool compressorFault = false;    // latched precharge timeout; blocks all starts
bool faultResetArmed = false;    // demand-removal hold for clearing the fault
unsigned long faultResetStart = 0;
unsigned long capChargeStart = 0; // when the charge output was asserted

// Switch position states, updated each loop by readSwitchInputs()
bool switch1State = false;
bool switch2State = false;
bool switch3State = false;
bool switch4State = false;

// Manual test overrides, settable from the web UI (see /test/* handlers).
bool testFanOverride       = false;
bool testFanOverrideActive = false;
bool testBuzzerOverride       = false;
bool testBuzzerOverrideActive = false;
bool testCompressorDemandOverride       = false; // still gated by ADC safety
bool testCompressorDemandOverrideActive = false;

Preferences prefs;
WebServer server(80);
DNSServer dnsServer;
bool apMode = false;
unsigned long lastWifiAttempt = 0;

String cfgSsid;
String cfgPass;

// ------------------------------ forward decl -----------------------------
int  readADC(uint8_t pin);
int  readCapacitorADC();
int  read12VCurrentADC();
bool is12VCurrentSafe();
void updateCompressorInterlock(bool wantCompressor);
void update12VEnable(bool pumpDemandNow, bool compressorIsRunning);
void runStateMachine();
void readSwitchInputs();
bool readSwitchActive(int pin);
void setupSwitchPin(int pin);
ChillerMode getSwitchMode();
void applyMode();
void updateStatusLeds();
void connectWiFi();
void startCaptivePortal();
void handleRoot();
void handleSave();
void handleStatusJson();
void handleTestFan();
void handleTestBuzzer();
void handleTestCompressor();
void setupOTA();
void loadSettings();
void saveSettings(const String &ssid, const String &pass);
const char* stateName();
const char* modeName();
const char* capSeqName();
String pinLevel(int pin);

void setup() {
  DBG_BEGIN(115200);

  // ---- GPIO defaults: ALL outputs LOW at boot, except 5V enable = HIGH ----
  // The charge output must be LOW at boot: the capacitor starts discharged
  // and must not be energized until the interlock routine decides to.
  pinMode(PIN_FAN, OUTPUT);        digitalWrite(PIN_FAN, LOW);
  pinMode(PIN_BUZZER, OUTPUT);     digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_12V_EN, OUTPUT);     digitalWrite(PIN_12V_EN, LOW);   // 12V rail off at boot
  pinMode(PIN_COMPRESSOR, OUTPUT); digitalWrite(PIN_COMPRESSOR, LOW);
  pinMode(PIN_CAP_CHARGE, OUTPUT); digitalWrite(PIN_CAP_CHARGE, LOW); // no charging at boot
  pinMode(PIN_5V_EN, OUTPUT);      digitalWrite(PIN_5V_EN, HIGH);   // default HIGH per spec
  pinMode(PIN_CAP_SENSE, INPUT);   // analog: capacitor voltage sense
  pinMode(PIN_ADC_12V, INPUT);     // analog: 12V current sense
  pinMode(PIN_TIMER_IN, INPUT);    // TODO: confirm pull-up/pull-down and active level

  // Mode-select switch inputs - ACTIVE LOW: the switch shorts the pin to GND
  // to close its position, internal pull-up holds it HIGH when open. Positions
  // with no pin assigned (PIN_SWITCH_1) are skipped.
  setupSwitchPin(PIN_SWITCH_1);
  setupSwitchPin(PIN_SWITCH_2);
  setupSwitchPin(PIN_SWITCH_3);
  setupSwitchPin(PIN_SWITCH_4);

  analogReadResolution(12); // 0-4095

  // Contacts open, no charge, fault clear - the safe resting state. The cap
  // sequence only leaves WAIT_DISCHARGE after seeing a discharged capacitor.
  capSeq = CAP_SEQ_WAIT_DISCHARGE;
  compressorRunning = false;
  compressorFault = false;

  statusLeds.begin();
  statusLeds.show(); // all off at boot

  loadSettings();
  connectWiFi();

  if (!apMode) {
    setupOTA();
    if (MDNS.begin(HOSTNAME)) {
      DBG_PRINTLN("mDNS started: " HOSTNAME ".local");
    }
    server.on("/", handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/status", handleStatusJson);
    server.on("/test/fan", handleTestFan);
    server.on("/test/buzzer", handleTestBuzzer);
    server.on("/test/compressor", handleTestCompressor);
    server.begin();
  }
}

void loop() {
  // ---- read inputs first ----
  readSwitchInputs();
  currentMode = getSwitchMode();
  applyMode(); // sets compressorRequested / pumpRequested from mode + overrides

  // ---- fault latch / display state, derived from the interlock ----
  runStateMachine();

  // ---- SAFETY CRITICAL PATH: runs every loop iteration, never blocked ----
  // Sole owner of the charge output and the compressor output.
  updateCompressorInterlock(compressorRequested);

  // ---- 12V enable with anti-short-cycle hold ----
  update12VEnable(pumpRequested, compressorRunning);

  // ---- manual test outputs for fan/buzzer (non-safety-critical) ----
  digitalWrite(PIN_FAN, (testFanOverrideActive ? testFanOverride : false) ? HIGH : LOW);
  digitalWrite(PIN_BUZZER, (testBuzzerOverrideActive ? testBuzzerOverride : false) ? HIGH : LOW);

  // ---- status LEDs ----
  updateStatusLeds();

  // ---- network maintenance (non-blocking, no delay() in loop) ----
  if (apMode) {
    dnsServer.processNextRequest();
    server.handleClient();
  } else {
    if (WiFi.status() != WL_CONNECTED) {
      if (millis() - lastWifiAttempt > WIFI_RETRY_INTERVAL_MS) {
        connectWiFi();
      }
    } else {
      ArduinoOTA.handle();
      server.handleClient();
    }
  }
}

// =========================================================================
// ANALOG INPUTS - averaged reads of the two sense lines
// =========================================================================
int readADC(uint8_t pin) {
  long sum = 0;
  for (uint8_t i = 0; i < ADC_SAMPLE_COUNT; i++) {
    sum += analogRead(pin);
  }
  return (int)(sum / ADC_SAMPLE_COUNT);
}

int readCapacitorADC() {
  return readADC(PIN_CAP_SENSE);
}

int read12VCurrentADC() {
  return readADC(PIN_ADC_12V);
}

// Overcurrent gate on the 12V sense line. Higher reading == more current.
// A start may not proceed, and a running compressor may not stay closed,
// while this is false.
bool is12VCurrentSafe() {
  int adcValue = read12VCurrentADC();
  bool safe = (adcValue <= COMPRESSOR_SAFE_ADC_MAX);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= DEBUG_PRINT_INTERVAL_MS) {
    DBG_PRINTF("[CURRENT] adc=%d limit=%d safe=%d\n", adcValue, COMPRESSOR_SAFE_ADC_MAX, safe);
    lastPrint = millis();
  }
  return safe;
}

// =========================================================================
// CAPACITOR INTERLOCK - the ONLY function that may write PIN_CAP_CHARGE or
// PIN_COMPRESSOR. Do not add a digitalWrite() on either pin anywhere else, and
// do not add a path that reaches the compressor without coming through here.
//
// Hysteretic on two thresholds of the capacitor sense, so a start can never
// land on a charged capacitor and a restart can never skip the discharge:
//
//   WAIT_DISCHARGE -> contacts open, no charging. Re-arm only once the
//                     capacitor is seen at/below CAP_DISCHARGED_ADC_MAX, and
//                     only if the compressor is actually being requested. A
//                     stuck-high or floating sense line can therefore never
//                     arm a start.
//   CHARGING       -> armed and requested: charge output HIGH, contacts still
//                     open. Nothing closes until the capacitor is seen at/above
//                     CAP_CHARGED_ADC_MIN and the 12V current sense is clear.
//   RUNNING        -> contacts closed, charge output held HIGH.
//   Any stop       -> contacts open, charging stops, and the next start must
//                     re-arm from a discharged capacitor.
//   Contacts not closed within PRECHARGE_TIMEOUT_MS of charging starting ->
//                     latched fault. This bounds the whole charging phase, not
//                     just the "not charged yet" part, so the charge output
//                     can never be left energized indefinitely. The contacts
//                     are never closed on an unproven capacitor.
// =========================================================================
void updateCompressorInterlock(bool wantCompressor) {
  // Latched fault: everything off, and no charging, until the state machine
  // clears it after the reset hold.
  if (compressorFault) {
    digitalWrite(PIN_COMPRESSOR, LOW);
    digitalWrite(PIN_CAP_CHARGE, LOW);
    compressorRunning = false;
    capSeq = CAP_SEQ_WAIT_DISCHARGE;
    return;
  }

  int cap = readCapacitorADC();

  switch (capSeq) {
    case CAP_SEQ_WAIT_DISCHARGE:
      digitalWrite(PIN_COMPRESSOR, LOW);
      digitalWrite(PIN_CAP_CHARGE, LOW);
      compressorRunning = false;
      if (wantCompressor && (cap <= CAP_DISCHARGED_ADC_MAX)) {
        capSeq = CAP_SEQ_CHARGING;
        capChargeStart = millis();
      }
      break;

    case CAP_SEQ_CHARGING:
      // Contacts stay open for the whole charge, no matter how long it takes.
      digitalWrite(PIN_COMPRESSOR, LOW);
      compressorRunning = false;

      if (!wantCompressor) {
        // Demand went away mid-charge. Stop charging and go back to waiting
        // for a full discharge before the next attempt.
        capSeq = CAP_SEQ_WAIT_DISCHARGE;
        break;
      }

      digitalWrite(PIN_CAP_CHARGE, HIGH);

      // Bound the entire charging phase, not just the "not charged yet" part,
      // so the charge output can never be left energized indefinitely if the
      // contacts refuse to close - e.g. the capacitor reads charged but the
      // 12V sense stays overcurrent. Latch a fault instead of sitting here.
      if (millis() - capChargeStart >= PRECHARGE_TIMEOUT_MS) {
        compressorFault = true;
        DBG_PRINTF("[FAULT] contacts did not close in %lums: cap=%d (need >=%d) currentSafe=%d\n",
                   (unsigned long)PRECHARGE_TIMEOUT_MS, cap, CAP_CHARGED_ADC_MIN,
                   is12VCurrentSafe() ? 1 : 0);
        break;
      }

      if (cap >= CAP_CHARGED_ADC_MIN && is12VCurrentSafe()) {
        // Capacitor proven charged AND the 12V sense clear of overcurrent -
        // only now may the contacts close.
        digitalWrite(PIN_COMPRESSOR, HIGH);
        compressorRunning = true;
        capSeq = CAP_SEQ_RUNNING;
      }
      break;

    case CAP_SEQ_RUNNING: {
      bool overcurrent = !is12VCurrentSafe();
      if (!wantCompressor || overcurrent) {
        // Stop requested, or overcurrent while running: open the contacts on
        // the very next loop and stop charging. Restarting needs a fresh
        // discharge, so this cannot become a fast on/off cycle.
        digitalWrite(PIN_COMPRESSOR, LOW);
        digitalWrite(PIN_CAP_CHARGE, LOW);
        compressorRunning = false;
        capSeq = CAP_SEQ_WAIT_DISCHARGE;
        DBG_PRINTF("[CURRENT] contacts opened (%s)\n",
                   overcurrent ? "overcurrent" : "demand removed");
      } else {
        // Keep the capacitor topped up for the life of the run.
        digitalWrite(PIN_CAP_CHARGE, HIGH);
        digitalWrite(PIN_COMPRESSOR, HIGH);
        compressorRunning = true;
      }
      break;
    }
  }
}

// =========================================================================
// 12V ENABLE OUTPUT - powers the pump, anti-short-cycle.
// Single writer of PIN_12V_EN. Stays on while the compressor is running too,
// so the rail is never cut out from under a closed contact.
// =========================================================================
unsigned long pumpOffTimerStart = 0;
bool pumpOffTimerActive = false;

void update12VEnable(bool pumpDemandNow, bool compressorIsRunning) {
  bool wantEnableOn = pumpDemandNow || compressorIsRunning;
  static bool lastWantEnableOn = false;

  if (wantEnableOn) {
    pumpOffTimerActive = false;
    digitalWrite(PIN_12V_EN, HIGH);
  } else {
    if (lastWantEnableOn && !pumpOffTimerActive) {
      // demand just dropped - start the anti-short-cycle hold window
      pumpOffTimerActive = true;
      pumpOffTimerStart = millis();
    }
    if (pumpOffTimerActive && (millis() - pumpOffTimerStart < PUMP_OFF_DELAY_MS)) {
      digitalWrite(PIN_12V_EN, HIGH); // still holding on during the delay
    } else {
      digitalWrite(PIN_12V_EN, LOW);
      pumpOffTimerActive = false;
    }
  }
  lastWantEnableOn = wantEnableOn;
}

// =========================================================================
// MODE-SELECT SWITCH INPUTS - ACTIVE LOW (switch closes to GND), internal
// pull-ups enabled in setup(). TODO: add debounce on position change.
// =========================================================================
void setupSwitchPin(int pin) {
  if (pin < 0) return; // position not wired - nothing to configure
  pinMode(pin, INPUT_PULLUP);
}

bool readSwitchActive(int pin) {
  if (pin < 0) return false; // position not wired, so it can never be active
  return digitalRead(pin) == LOW;
}

void readSwitchInputs() {
  switch1State = readSwitchActive(PIN_SWITCH_1);
  switch2State = readSwitchActive(PIN_SWITCH_2);
  switch3State = readSwitchActive(PIN_SWITCH_3);
  switch4State = readSwitchActive(PIN_SWITCH_4);
}

ChillerMode getSwitchMode() {
  // Expected to be a one-of-four switch. If none or more than one read
  // active (wiring fault, transition glitch, etc.) fail safe to OFF.
  uint8_t activeCount = switch1State + switch2State + switch3State + switch4State;
  if (activeCount != 1) {
    return MODE_OFF;
  }
  if (switch2State) return MODE_PUMP_ONLY;
  if (switch3State) return MODE_CHILLER_ONLY;
  if (switch4State) return MODE_BOTH;
  return MODE_OFF; // switch1State, or fallback
}

void applyMode() {
  bool timerEnabled = (digitalRead(PIN_TIMER_IN) == HIGH); // TODO: confirm active level and desired AND/OR logic with mode

  bool modePump     = (currentMode == MODE_PUMP_ONLY || currentMode == MODE_BOTH);
  bool modeChiller   = (currentMode == MODE_CHILLER_ONLY || currentMode == MODE_BOTH);

  pumpRequested = modePump;
  compressorRequested = modeChiller && timerEnabled; // TODO: confirm timer should gate compressor demand

  // Manual test overrides from the web UI take priority for the compressor
  // demand path only. This only sets the DEMAND flag - it cannot reach the
  // compressor output, which stays behind updateCompressorInterlock().
  if (testCompressorDemandOverrideActive) {
    compressorRequested = testCompressorDemandOverride;
  }
}

// =========================================================================
// STATUS LEDS - pseudo color mapping, adjust colors/patterns as desired
// =========================================================================
void updateStatusLeds() {
  uint32_t color;
  switch (state) {
    case STATE_FAULT:
      // slow flash red
      color = ((millis() / 500) % 2 == 0) ? statusLeds.Color(255, 0, 0) : statusLeds.Color(0, 0, 0);
      break;
    case STATE_RUN:
      color = pumpRequested ? statusLeds.Color(128, 0, 128)   // chiller + pump = purple
                             : statusLeds.Color(255, 60, 0);  // chiller only = orange
      break;
    case STATE_PRECHARGE:
      color = statusLeds.Color(255, 255, 0); // charging capacitor = yellow
      break;
    default: // STATE_IDLE
      if (pumpRequested || pumpOffTimerActive) {
        color = statusLeds.Color(0, 0, 255); // pump running = blue
      } else {
        color = statusLeds.Color(0, 0, 0);   // off
      }
      break;
  }

  for (uint16_t i = 0; i < LED_COUNT; i++) {
    statusLeds.setPixelColor(i, color);
  }
  statusLeds.show();
  // TODO: consider per-LED assignment (e.g. LED0=pump, LED1=compressor,
  // LED2=fault, LED3=wifi) instead of a single strip-wide color.
}

// =========================================================================
// FAULT LATCH + DISPLAY STATE
// The start sequence itself lives in updateCompressorInterlock(), which owns
// the charge and compressor outputs and raises compressorFault if the
// capacitor never charges. All this does is latch that into STATE_FAULT,
// decide when it is safe to clear again, and project CapSeq into the
// STATE_* value used for the LEDs, web UI and debug output.
// =========================================================================
void runStateMachine() {
  if (compressorFault) {
    state = STATE_FAULT;

    // Clear only after the chiller demand has been removed for the full reset
    // hold, so bouncing the switch cannot immediately re-charge a bad
    // capacitor. While the fault is latched the interlock keeps both the
    // charge and compressor outputs off regardless of demand.
    if (compressorRequested) {
      faultResetArmed = false;
    } else {
      if (!faultResetArmed) {
        faultResetArmed = true;
        faultResetStart = millis();
      } else if (millis() - faultResetStart >= FAULT_RESET_HOLD_MS) {
        compressorFault = false;
        faultResetArmed = false;
        DBG_PRINTLN("[FAULT] cleared, interlock reset to WAIT_DISCHARGE");
      }
    }
    return;
  }

  faultResetArmed = false;

  switch (capSeq) {
    case CAP_SEQ_CHARGING: state = STATE_PRECHARGE; break;
    case CAP_SEQ_RUNNING:  state = STATE_RUN;       break;
    default:               state = STATE_IDLE;      break;
  }
}

// =========================================================================
// WIFI / OTA / CAPTIVE PORTAL
// =========================================================================
void connectWiFi() {
  lastWifiAttempt = millis();
  apMode = false;
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
  WiFi.setTxPower(WIFI_POWER_8_5dBm); // set right after WiFi.begin() per spec

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    DBG_PRINTLN("WiFi connected: " + WiFi.localIP().toString());
  } else {
    DBG_PRINTLN("WiFi failed, starting captive portal");
    startCaptivePortal();
  }
}

void startCaptivePortal() {
  apMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(HOSTNAME); // open AP named "chiller" -- TODO: add AP password if desired
  dnsServer.start(53, "*", WiFi.softAPIP());
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.onNotFound(handleRoot); // catch-all so captive portal detection works
  server.begin();
  DBG_PRINTLN("Captive portal started, AP IP: " + WiFi.softAPIP().toString());
}

void setupOTA() {
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.onStart([]() { DBG_PRINTLN("OTA start"); });
  ArduinoOTA.onEnd([]()   { DBG_PRINTLN("OTA end"); });
  ArduinoOTA.onError([](ota_error_t error) { DBG_PRINTF("OTA error[%u]\n", error); });
  ArduinoOTA.begin();
}

// =========================================================================
// WEB HANDLERS (markup lives in html.h)
// =========================================================================
const char* stateName() {
  switch (state) {
    case STATE_IDLE:      return "IDLE";
    case STATE_PRECHARGE: return "PRECHARGE";
    case STATE_RUN:        return "RUN";
    case STATE_FAULT:      return "FAULT";
    default:               return "UNKNOWN";
  }
}

const char* modeName() {
  switch (currentMode) {
    case MODE_OFF:          return "OFF";
    case MODE_PUMP_ONLY:    return "PUMP_ONLY";
    case MODE_CHILLER_ONLY: return "CHILLER_ONLY";
    case MODE_BOTH:         return "BOTH";
    default:                 return "UNKNOWN";
  }
}

void handleRoot() {
  if (apMode) {
    server.send(200, "text/html", PAGE_CONFIG);
  } else {
    server.send(200, "text/html", PAGE_STATUS);
  }
}

void handleSave() {
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  if (ssid.length() > 0) {
    saveSettings(ssid, pass);
    server.send(200, "text/html", "Saved. Rebooting...");
    delay(500);
    ESP.restart();
  } else {
    server.send(400, "text/plain", "Missing SSID");
  }
}

// "<gpio>=<HIGH|LOW>" so the web UI can show the raw pin state next to each
// label. Reads the pin directly - never writes it. A negative pin means the
// position is not wired, so report it as such rather than reading a bogus pin.
String pinLevel(int pin) {
  if (pin < 0) return "n/a";
  return String(pin) + "=" + (digitalRead(pin) == HIGH ? "HIGH" : "LOW");
}

const char* capSeqName() {
  switch (capSeq) {
    case CAP_SEQ_WAIT_DISCHARGE: return "WAIT_DISCHARGE";
    case CAP_SEQ_CHARGING:       return "CHARGING";
    case CAP_SEQ_RUNNING:        return "RUNNING";
    default:                     return "UNKNOWN";
  }
}

// Live JSON status for the web UI to poll (used by html.h page via fetch()).
void handleStatusJson() {
  int capAdcValue     = readCapacitorADC();
  int currentAdcValue = read12VCurrentADC();
  String json = "{";
  json += "\"mode\":\"" + String(modeName()) + "\",";
  json += "\"state\":\"" + String(stateName()) + "\",";
  json += "\"capSeq\":\"" + String(capSeqName()) + "\",";
  json += "\"capArmed\":" + String(capSeq != CAP_SEQ_WAIT_DISCHARGE ? "true" : "false") + ",";
  json += "\"capAdc\":" + String(capAdcValue) + ",";
  json += "\"capDischargedMax\":" + String(CAP_DISCHARGED_ADC_MAX) + ",";
  json += "\"capChargedMin\":" + String(CAP_CHARGED_ADC_MIN) + ",";
  json += "\"capCharge\":" + String(digitalRead(PIN_CAP_CHARGE) == HIGH ? "true" : "false") + ",";
  json += "\"currentAdc\":" + String(currentAdcValue) + ",";
  json += "\"currentLimit\":" + String(COMPRESSOR_SAFE_ADC_MAX) + ",";
  json += "\"currentSafe\":" + String(is12VCurrentSafe() ? "true" : "false") + ",";
  json += "\"fan\":" + String(digitalRead(PIN_FAN) == HIGH ? "true" : "false") + ",";
  json += "\"buzzer\":" + String(digitalRead(PIN_BUZZER) == HIGH ? "true" : "false") + ",";
  json += "\"enable12v\":" + String(digitalRead(PIN_12V_EN) == HIGH ? "true" : "false") + ",";
  json += "\"enable12vHold\":" + String(pumpOffTimerActive ? "true" : "false") + ",";
  json += "\"compressor\":" + String(digitalRead(PIN_COMPRESSOR) == HIGH ? "true" : "false") + ",";
  json += "\"fault\":" + String(compressorFault ? "true" : "false") + ",";
  json += "\"switch1\":" + String(switch1State ? "true" : "false") + ",";
  json += "\"switch2\":" + String(switch2State ? "true" : "false") + ",";
  json += "\"switch3\":" + String(switch3State ? "true" : "false") + ",";
  json += "\"switch4\":" + String(switch4State ? "true" : "false") + ",";
  json += "\"testFanActive\":" + String(testFanOverrideActive ? "true" : "false") + ",";
  json += "\"testBuzzerActive\":" + String(testBuzzerOverrideActive ? "true" : "false") + ",";
  json += "\"testCompressorActive\":" + String(testCompressorDemandOverrideActive ? "true" : "false") + ",";
  json += "\"pins\":{";
  json += "\"capSense\":\""    + String(PIN_CAP_SENSE) + "=ADC"     + "\",";
  json += "\"capCharge\":\""    + pinLevel(PIN_CAP_CHARGE)          + "\",";
  json += "\"currentSense\":\"" + String(PIN_ADC_12V) + "=ADC"       + "\",";
  json += "\"enable12v\":\""    + pinLevel(PIN_12V_EN)               + "\",";
  json += "\"fan\":\""          + pinLevel(PIN_FAN)                  + "\",";
  json += "\"buzzer\":\""       + pinLevel(PIN_BUZZER)               + "\",";
  json += "\"compressor\":\""   + pinLevel(PIN_COMPRESSOR)           + "\",";
  json += "\"switch1\":\""      + pinLevel(PIN_SWITCH_1)             + "\",";
  json += "\"switch2\":\""      + pinLevel(PIN_SWITCH_2)             + "\",";
  json += "\"switch3\":\""      + pinLevel(PIN_SWITCH_3)             + "\",";
  json += "\"switch4\":\""      + pinLevel(PIN_SWITCH_4)             + "\"";
  json += "}";
  json += "}";
  server.send(200, "application/json", json);
}

// Manual test endpoints - fan/buzzer are simple direct overrides (non-safety-
// critical). Compressor test only sets the DEMAND flag; it still has to pass
// through the full capacitor interlock and the current-sense gate every loop,
// so this can be used to verify the interlock itself without ever bypassing it.
void handleTestFan() {
  String s = server.arg("state");
  if (s == "on")       { testFanOverrideActive = true;  testFanOverride = true;  }
  else if (s == "off") { testFanOverrideActive = true;  testFanOverride = false; }
  else if (s == "auto"){ testFanOverrideActive = false; }
  server.send(200, "text/plain", "ok");
}

void handleTestBuzzer() {
  String s = server.arg("state");
  if (s == "on")        { testBuzzerOverrideActive = true;  testBuzzerOverride = true;  }
  else if (s == "off")  { testBuzzerOverrideActive = true;  testBuzzerOverride = false; }
  else if (s == "auto") { testBuzzerOverrideActive = false; }
  server.send(200, "text/plain", "ok");
}

void handleTestCompressor() {
  String s = server.arg("state");
  if (s == "on")        { testCompressorDemandOverrideActive = true;  testCompressorDemandOverride = true;  }
  else if (s == "off")  { testCompressorDemandOverrideActive = true;  testCompressorDemandOverride = false; }
  else if (s == "auto") { testCompressorDemandOverrideActive = false; }
  server.send(200, "text/plain", "ok");
}

// =========================================================================
// FLASH-PERSISTED SETTINGS
// =========================================================================
void loadSettings() {
  prefs.begin("chiller", false);
  cfgSsid = prefs.getString("ssid", MYSSIDIOT);
  cfgPass = prefs.getString("pass", MYPSKIOT);
  COMPRESSOR_SAFE_ADC_MAX = prefs.getInt("adcmax", COMPRESSOR_SAFE_ADC_MAX);
  prefs.end();
}

void saveSettings(const String &ssid, const String &pass) {
  prefs.begin("chiller", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
  cfgSsid = ssid;
  cfgPass = pass;
}
