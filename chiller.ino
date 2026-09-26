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
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 10000;
const unsigned long WIFI_RETRY_INTERVAL_MS  = 3000;

// Connection is a non-blocking state machine rather than a blocking
// wait-for-connected loop. A blocking attempt would stall loop() for up to
// WIFI_CONNECT_TIMEOUT_MS, and loop() is what runs the compressor interlock -
// so every retry would delay a 12V overcurrent trip by that much, with the
// contacts held closed. Never block here.
enum WifiPhase {
  WIFI_PHASE_IDLE,       // not connected, no attempt in flight
  WIFI_PHASE_CONNECTING, // attempt in flight, being polled each loop
  WIFI_PHASE_CONNECTED   // STA up and usable
};
WifiPhase wifiPhase = WIFI_PHASE_IDLE;
unsigned long wifiConnectStart = 0;
unsigned long lastWifiAttempt = 0;
bool otaReady = false;
bool mdnsReady = false;
// Set by handleSave() so the next loop() starts a connect attempt with the new
// credentials. Using a flag rather than delay() keeps the interlock running.
bool wifiReconnectRequested = false;

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

// ==================== ADC <-> VOLTS CONVERSION ============================
// The capacitor sense line reaches GPIO0 through a 1k series resistor with no
// divider, so the pin sees the capacitor voltage directly and the scaling
// ratio is 1.0. Change CAP_SENSE_DIVIDER if a divider is ever fitted.
//
// The 12V current sense on GPIO2 has its own, separate scaling - it runs through
// a sense amplifier, and CURRENT_SENSE_DIVIDER is the factor that turns the pin
// voltage back into the real 12V rail current sense. It is UNCALIBRATED: the
// placeholder of 1.0 makes the volts readout equal the pin voltage, so treat it
// as indicative until you measure the real ratio against a known load current.
//
// The C3's ADC is measurably nonlinear, especially near the rails, so treat the
// volts figures as INDICATIVE and calibrate against a multimeter. The interlock
// itself works entirely in raw ADC counts, so a volts error can never make it
// unsafe - it would only mis-set a threshold.
const float ADC_REFERENCE_V     = 3.3f;
const float CAP_SENSE_DIVIDER   = 1.0f;
const float CURRENT_SENSE_DIVIDER = 1.0f; // TODO: calibrate against a known load
const int   ADC_FULL_SCALE      = 4095;

float adcToVoltsScaled(int adc, float divider) {
  return (adc * (ADC_REFERENCE_V / (float)ADC_FULL_SCALE)) * divider;
}

int voltsToAdcScaled(float volts, float divider) {
  if (divider <= 0.0f) return 0;
  float counts = volts / (ADC_REFERENCE_V * divider) * (float)ADC_FULL_SCALE;
  if (counts < 0.0f) counts = 0.0f;
  if (counts > (float)ADC_FULL_SCALE) counts = (float)ADC_FULL_SCALE;
  return (int)(counts + 0.5f);
}

// Capacitor sense (GPIO0) helpers.
float adcToVolts(int adc)         { return adcToVoltsScaled(adc, CAP_SENSE_DIVIDER); }
int   voltsToAdc(float volts)     { return voltsToAdcScaled(volts, CAP_SENSE_DIVIDER); }

// 12V current sense (GPIO2) helpers.
float currentAdcToVolts(int adc)  { return adcToVoltsScaled(adc, CURRENT_SENSE_DIVIDER); }
int   currentVoltsToAdc(float v)  { return voltsToAdcScaled(v, CURRENT_SENSE_DIVIDER); }

// Minimum gap that must stay between the two capacitor thresholds, in ADC
// counts. The gap between them IS the hysteresis band: with it, a start always
// discharges first and charges second. Enforced on every change so the web UI
// (or a corrupted flash value) can't collapse the band and let the contacts
// close on a charged capacitor.
const int CAP_THRESHOLD_MIN_GAP = 200; // ~0.16V at ratio 1.0

// How long the charge output may be energized without the capacitor reaching
// CAP_CHARGED_ADC_MIN before the fault latches and the contacts stay open.
const unsigned long PRECHARGE_TIMEOUT_MS = 30000; // TODO: tune to taste

// Once the fault has latched, the chiller demand must be removed for this long
// before it clears, so a bad capacitor or charge path can't be re-energized by
// simply bouncing the switch.
const unsigned long FAULT_RESET_HOLD_MS = 5000; // TODO: tune to taste

// Compressor minimum run time. Once the contacts close they stay closed for at
// least this long, so a brief demand dip (switch bounce, a momentary timer
// low) can't short-cycle the compressor.
// A safety trip (12V overcurrent) still opens the contacts immediately with no
// hold at all - this only covers a demand-driven stop.
const unsigned long COMPRESSOR_MIN_RUN_MS = 5000; // TODO: tune to taste

// Compressor off-delay. SEPARATE from the minimum run time above: once the
// minimum run time is satisfied, removing demand does not open the contacts
// straight away - they stay closed for this long first. If demand comes back
// inside that window the compressor never stopped at all, so it just resumes
// running: no stop, no capacitor discharge, no re-charge, no contactor cycle.
//
// This is why the two are separate. A minimum run time alone cannot do this:
// once a compressor has been running longer than the minimum, removing demand
// satisfies the "minimum elapsed" test immediately and it would stop on the
// spot, then have to discharge and re-charge to restart.
//
// The contacts only open when BOTH have elapsed: the minimum run time since
// the last start, and this off-delay since demand was removed.
const unsigned long COMPRESSOR_OFF_DELAY_MS = 5000; // TODO: tune to taste

// Switch input debounce. A raw reading only becomes the accepted position
// after it has been stable this long. Mode switches bounce for a few ms, and
// any transient 0-or-2-active read is treated as a fault that stops
// everything, so require a settled reading before acting on it.
const unsigned long SWITCH_DEBOUNCE_MS = 500; // TODO: tune to taste

// Pump anti-short-cycle hold time. 12V enable stays on this long after pump
// demand drops, so switching modes doesn't stop/restart the pump every time.
const unsigned long PUMP_OFF_DELAY_MS = 1000; // TODO: tune to taste

// Buzzer test tone frequency, in Hz. The buzzer is driven as a pulsed square
// wave at this rate - it is never held at a static DC level, which would either
// produce no sound (most piezo elements need AC) or overheat the element.
int buzzerFrequency = 2000;                 // persisted, settable from the web UI
const int BUZZER_FREQ_MIN = 100;
const int BUZZER_FREQ_MAX = 8000;

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
unsigned long compressorRunStart = 0; // when the contacts closed
// When demand was last removed from a running compressor. 0 means "no stop
// pending". Demand returning clears it, which is how a brief OFF-then-ON
// resumes the same run instead of starting a new one.
unsigned long compressorStopRequestedAt = 0;

// Capacitor discharge timing. Measures how long the capacitor has been sitting
// below CAP_CHARGED_ADC_MIN - i.e. bleeding down since it last read charged.
// Purely a health indicator for the discharge path: a capacitor that never
// bleeds down points at a failed bleeder, and would block the next start.
unsigned long capDischargeStart = 0;
unsigned long lastCapDischargeMs = 0; // duration of the most completed discharge
bool capDischargeTiming = false;

// Buzzer tone state. The tone is started/stopped on transitions only, so
// updateBuzzerOutput() costs a boolean compare per loop when idle.
bool buzzerToneActive = false;

// Switch debounce state, one entry per position. Reads raw from the pin but
// only publishes switchNState once the raw value has been stable for
// SWITCH_DEBOUNCE_MS.
struct DebouncedSwitch {
  bool stable = false;            // last accepted value
  bool candidate = false;         // value currently being confirmed
  unsigned long candidateSince = 0;
};
DebouncedSwitch switchDebounce[4];
const uint8_t SWITCH_COUNT = 4;

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
void updateDebounced(DebouncedSwitch &d, bool raw);
ChillerMode getSwitchMode();
void applyMode();
void updateStatusLeds();
void serviceWifi();
void startWifiAttempt();
void startCaptivePortal();
void leaveCaptivePortal();
void registerStatusRoutes();
void onWifiConnected();
void handleRoot();
void handleNotFound();
void handleSave();
void handleStatusJson();
void handleTestFan();
void handleTestBuzzer();
void handleTestCompressor();
void handleSetThresholds();
void handleSetCurrentLimit();
void handleSetBuzzer();
void setupOTA();
void loadSettings();
void saveThresholds();
void saveBuzzerFrequency();
void saveSettings(const String &ssid, const String &pass);
void updateBuzzerOutput();
const char* stateName();
const char* modeName();
const char* capSeqName();
String pinLevel(int pin);

void setup() {
  DBG_BEGIN(115200);

  // ---- GPIO defaults: ALL outputs LOW at boot, except 5V enable = HIGH ----
  // The charge output must be LOW at boot: the capacitor starts discharged
  // and must not be energized until the interlock routine decides to.
  //
  // Every output level is written BEFORE the pin is switched to OUTPUT, so the
  // pin never drives an unintended level during the mode change. Order matters:
  // do not collapse these back into "pinMode() then digitalWrite()".
  digitalWrite(PIN_FAN, LOW);        pinMode(PIN_FAN, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);     pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_12V_EN, LOW);     pinMode(PIN_12V_EN, OUTPUT);   // 12V rail off at boot
  digitalWrite(PIN_COMPRESSOR, LOW); pinMode(PIN_COMPRESSOR, OUTPUT);
  digitalWrite(PIN_CAP_CHARGE, LOW); pinMode(PIN_CAP_CHARGE, OUTPUT); // no charging at boot
  // GPIO9 is an ESP32-C3 strapping pin: it must be HIGH for SPI boot, and LOW
  // forces UART download mode. It is parked HIGH (still set before enabling the
  // output) and must never be moved to LOW here.
  digitalWrite(PIN_5V_EN, HIGH);     pinMode(PIN_5V_EN, OUTPUT);
  pinMode(PIN_CAP_SENSE, INPUT);   // analog: capacitor voltage sense
  pinMode(PIN_ADC_12V, INPUT);     // analog: 12V current sense
  // Timer input is treated as active-HIGH (see applyMode()). Configured with
  // an explicit pull-DOWN so an unwired or floating timer deterministically
  // reads LOW = inactive, which blocks compressor demand, instead of randomly
  // enabling or disabling the chiller.
  // TODO: confirm against the real timer circuit. If it needs a pull-UP
  // instead, change this and the read in applyMode() together.
  pinMode(PIN_TIMER_IN, INPUT_PULLDOWN);

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

  // Non-blocking first attempt - serviceWifi() in loop() drives it to
  // completion, so setup() never waits on the network and never delays the
  // interlock coming up.
  startWifiAttempt();
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
  updateBuzzerOutput();

  // ---- status LEDs ----
  updateStatusLeds();

  // ---- network maintenance (non-blocking, no delay() in loop) ----
  // Connect/retry runs in ALL modes, including while the AP is up. The old code
  // only retried when apMode was false, so a device that dropped onto the
  // captive portal retried nothing and stayed there until it was power cycled.
  serviceWifi();

  if (apMode) {
    dnsServer.processNextRequest();
  } else {
    if (wifiPhase == WIFI_PHASE_CONNECTED) ArduinoOTA.handle();
  }
  server.handleClient();
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
//   Minimum run    -> once the contacts close they stay closed for at least
//                     COMPRESSOR_MIN_RUN_MS, and for a further
//                     COMPRESSOR_OFF_DELAY_MS after demand is removed. Demand
//                     returning inside that second window clears the pending
//                     stop, so the compressor resumes the SAME run: no stop, no
//                     discharge, no re-charge. Overcurrent is checked BEFORE
//                     either hold, so a safety trip still opens the contacts
//                     immediately.
//   Contacts not closed within PRECHARGE_TIMEOUT_MS of charging starting ->
//                     latched fault. This bounds the whole charging phase, not
//                     just the "not charged yet" part, so the charge output
//                     can never be left energized indefinitely. The contacts
//                     are never closed on an unproven capacitor.
// =========================================================================
void updateCompressorInterlock(bool wantCompressor) {
  int cap = readCapacitorADC();

  // Discharge timing runs every loop, faulted or not, so the web UI keeps
  // showing a live reading. Restarts each time the cap rises back to charged.
  if (cap < CAP_CHARGED_ADC_MIN) {
    if (!capDischargeTiming) {
      capDischargeTiming = true;
      capDischargeStart = millis();
    }
  } else if (capDischargeTiming) {
    capDischargeTiming = false;
    lastCapDischargeMs = millis() - capDischargeStart;
  }

  // Latched fault: everything off, and no charging, until the state machine
  // clears it after the reset hold.
  if (compressorFault) {
    digitalWrite(PIN_COMPRESSOR, LOW);
    digitalWrite(PIN_CAP_CHARGE, LOW);
    compressorRunning = false;
    compressorStopRequestedAt = 0;
    capSeq = CAP_SEQ_WAIT_DISCHARGE;
    return;
  }

  switch (capSeq) {
    case CAP_SEQ_WAIT_DISCHARGE:
      // Contacts open, and the charge output is parked LOW here UNCONDITIONALLY
      // - before the arm test below, not inside it. That ordering is the point:
      // a run may be requested while the capacitor is still charged, and the
      // charge output must be off in that case so the capacitor is free to bleed
      // down to CAP_DISCHARGED_ADC_MAX. If charging were left on here it would
      // hold the capacitor at the charged voltage forever, the arm test would
      // never be satisfied, and the compressor would never run again.
      digitalWrite(PIN_COMPRESSOR, LOW);
      digitalWrite(PIN_CAP_CHARGE, LOW);
      compressorRunning = false;
      compressorStopRequestedAt = 0;
      // Re-arm only once the capacitor is actually seen discharged AND the
      // compressor is being requested. Merely requesting a run is not enough.
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
        // Demand went away mid-charge. Stop charging now rather than leaving
        // the charge output HIGH until the next loop, so the pin state is
        // deterministic within this same iteration.
        digitalWrite(PIN_CAP_CHARGE, LOW);
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
        compressorRunStart = millis();
        capSeq = CAP_SEQ_RUNNING;
      }
      break;

    case CAP_SEQ_RUNNING: {
      // Safety trip, checked before the off-delay hold so it can never be
      // delayed by it: 12V overcurrent opens the contacts on this very loop.
      if (!is12VCurrentSafe()) {
        digitalWrite(PIN_COMPRESSOR, LOW);
        digitalWrite(PIN_CAP_CHARGE, LOW);
        compressorRunning = false;
        compressorStopRequestedAt = 0;
        capSeq = CAP_SEQ_WAIT_DISCHARGE;
        DBG_PRINTLN("[CURRENT] contacts opened (overcurrent)");
        break;
      }

      // Demand still present: stay closed. This must be checked before the
      // stop-hold logic below - previously, wantCompressor==true zeroed
      // compressorStopRequestedAt and then fell through into the "normal
      // stop" code every single loop, so the compressor could never stay
      // running for more than one iteration. Clear the stop-pending marker
      // and keep the capacitor topped up for the life of the run.
      if (wantCompressor) {
        compressorStopRequestedAt = 0;
        digitalWrite(PIN_CAP_CHARGE, HIGH);
        digitalWrite(PIN_COMPRESSOR, HIGH);
        compressorRunning = true;
        break;
      }

      // Demand has been removed. Stamp when the stop was first requested (if
      // not already stamped), so an OFF-then-ON inside the off-delay window
      // can clear this and resume the same run instead of opening the
      // contacts and starting over.
      if (compressorStopRequestedAt == 0) {
        compressorStopRequestedAt = millis();
      }

      bool minRunElapsed  = (millis() - compressorRunStart) >= COMPRESSOR_MIN_RUN_MS;
      bool offDelayElapsed = (millis() - compressorStopRequestedAt) >= COMPRESSOR_OFF_DELAY_MS;

      if (!(minRunElapsed && offDelayElapsed)) {
        // Either the minimum run time hasn't been met yet, or the off-delay is
        // still running. Stay closed and keep the capacitor charged, so a brief
        // demand dip or an OFF-then-ON can't short-cycle the compressor.
        digitalWrite(PIN_CAP_CHARGE, HIGH);
        digitalWrite(PIN_COMPRESSOR, HIGH);
        compressorRunning = true;
        break;
      }

      // Both holds satisfied - normal stop. Contacts open, charging stops, and
      // the next start must re-arm from a discharged capacitor.
      digitalWrite(PIN_COMPRESSOR, LOW);
      digitalWrite(PIN_CAP_CHARGE, LOW);
      compressorRunning = false;
      compressorStopRequestedAt = 0;
      capSeq = CAP_SEQ_WAIT_DISCHARGE;
      DBG_PRINTLN("[COMPRESSOR] min run + off delay elapsed, contacts opened");
      break;
    }
  }
}

// =========================================================================
// BUZZER OUTPUT - pulsed, never static DC
// The buzzer element is driven as a square wave at buzzerFrequency. Holding a
// buzzer at a steady DC level produces no sound from a piezo element and will
// cook a magnetic one, so it must be pulsed. tone() is LEDC-backed and
// non-blocking, so the pulsing costs nothing on the safety-critical path -
// tone()/noTone() are only called on an actual on/off transition, never per
// loop.
// =========================================================================
void updateBuzzerOutput() {
  bool wantTone = testBuzzerOverrideActive && testBuzzerOverride;

  if (wantTone) {
    if (!buzzerToneActive) {
      tone(PIN_BUZZER, buzzerFrequency);
      buzzerToneActive = true;
    }
  } else {
    if (buzzerToneActive) {
      noTone(PIN_BUZZER);
      buzzerToneActive = false;
    }
    // noTone() leaves the pin as an output, so make sure it is parked LOW
    // rather than left floating once the tone stops.
    if (!buzzerToneActive) digitalWrite(PIN_BUZZER, LOW);
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
// pull-ups enabled in setup(), plus a stability filter per input.
// =========================================================================
void setupSwitchPin(int pin) {
  if (pin < 0) return; // position not wired - nothing to configure
  pinMode(pin, INPUT_PULLUP);
}

bool readSwitchActive(int pin) {
  if (pin < 0) return false; // position not wired, so it can never be active
  return digitalRead(pin) == LOW;
}

// Publish `raw` as the accepted value only once it has been stable for
// SWITCH_DEBOUNCE_MS. Any change restarts the window, so a bouncing contact
// never reaches the mode selection.
void updateDebounced(DebouncedSwitch &d, bool raw) {
  if (raw != d.candidate) {
    // Different from the reading being confirmed - restart the window.
    d.candidate = raw;
    d.candidateSince = millis();
    return;
  }
  if (d.candidate != d.stable && (millis() - d.candidateSince >= SWITCH_DEBOUNCE_MS)) {
    d.stable = d.candidate;
  }
}

void readSwitchInputs() {
  // int, not uint8_t: an unwired position is -1, which won't narrow.
  const int pins[SWITCH_COUNT] = {
    PIN_SWITCH_1, PIN_SWITCH_2, PIN_SWITCH_3, PIN_SWITCH_4
  };
  bool states[SWITCH_COUNT];

  for (uint8_t i = 0; i < SWITCH_COUNT; i++) {
    updateDebounced(switchDebounce[i], readSwitchActive(pins[i]));
    states[i] = switchDebounce[i].stable;
  }

  switch1State = states[0];
  switch2State = states[1];
  switch3State = states[2];
  switch4State = states[3];
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
  // Active level matches the INPUT_PULLDOWN in setup(): HIGH = timer on. With
  // the pin unwired the pull-down holds this LOW, so chiller demand is blocked
  // rather than floating.
  // TODO: confirm the timer should gate compressor demand at all, and how it
  // should combine with the switch mode.
  bool timerEnabled = (digitalRead(PIN_TIMER_IN) == HIGH);

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
// All of this is polled from serviceWifi() once per loop() and never blocks.
// =========================================================================

// 404 for unknown paths on the real network. Installed over the captive
// portal's catch-all, which would otherwise keep answering every mistyped URL
// with the config page.
void handleNotFound() {
  if (apMode) {
    server.send(200, "text/html", PAGE_CONFIG); // portal detection probe
  } else {
    server.send(404, "text/plain", "Not found");
  }
}

// Register the live-status routes. Called on every successful STA connection so
// a device that reached the AP first still ends up with a working status page.
void registerStatusRoutes() {
  server.on("/", handleRoot);
  server.on("/status", handleStatusJson);
  server.on("/test/fan", handleTestFan);
  server.on("/test/buzzer", handleTestBuzzer);
  server.on("/test/compressor", handleTestCompressor);
  server.on("/set/thresholds", handleSetThresholds);
  server.on("/set/current", handleSetCurrentLimit);
  server.on("/set/buzzer", handleSetBuzzer);
  server.onNotFound(handleNotFound);
  server.begin();
}

// Tear the AP down when STA comes up, and swap the status routes in. The
// captive portal's catch-all has to go, or every request on the real network
// would keep being answered with the config page.
void leaveCaptivePortal() {
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  apMode = false;
  WiFi.mode(WIFI_STA);
  registerStatusRoutes();
  DBG_PRINTLN("[WIFI] left captive portal, status page now available");
}

void onWifiConnected() {
  wifiPhase = WIFI_PHASE_CONNECTED;
  if (apMode) leaveCaptivePortal();

  if (!otaReady) { setupOTA(); otaReady = true; }
  if (!mdnsReady) {
    if (MDNS.begin(HOSTNAME)) {
      mdnsReady = true;
      DBG_PRINTLN("mDNS started: " HOSTNAME ".local");
    }
  }
  lastWifiAttempt = millis();
  DBG_PRINTLN("[WIFI] connected: " + WiFi.localIP().toString());
}

// Kick off a non-blocking connection attempt. No waiting here - the result is
// picked up by serviceWifi() on the following loops.
void startWifiAttempt() {
  lastWifiAttempt = millis();
  wifiConnectStart = millis();
  wifiPhase = WIFI_PHASE_CONNECTING;
  // Stay in APSTA while the AP is up so the captive portal remains reachable
  // during a retry; drop to plain STA if it isn't.
  WiFi.mode(apMode ? WIFI_AP_STA : WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
  WiFi.setTxPower(WIFI_POWER_8_5dBm); // right after begin() per spec
  DBG_PRINTLN("[WIFI] connecting" + String(apMode ? " (AP still up)" : ""));
}

void startCaptivePortal() {
  apMode = true;
  WiFi.mode(WIFI_AP_STA); // APSTA, not AP: STA must stay available to retry on
  WiFi.softAP(HOSTNAME); // open AP named "chiller" -- TODO: add AP password
  dnsServer.start(53, "*", WiFi.softAPIP());
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.onNotFound(handleRoot); // catch-all so captive portal detection works
  server.begin();
  DBG_PRINTLN("[WIFI] captive portal started, AP IP: " + WiFi.softAPIP().toString() +
              " - will keep retrying the saved network every " +
              String(WIFI_RETRY_INTERVAL_MS / 1000) + "s");
}

// Non-blocking connection manager. Called every loop(), before the web server.
void serviceWifi() {
  if (wifiReconnectRequested) {
    wifiReconnectRequested = false;
    startWifiAttempt();
    return;
  }

  if (wifiPhase == WIFI_PHASE_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      onWifiConnected();
    } else if (millis() - wifiConnectStart >= WIFI_CONNECT_TIMEOUT_MS) {
      // This attempt gave up. Not fatal - fall back to retrying on the interval.
      // Drop to the AP if we aren't in it yet, so the user has a way in.
      DBG_PRINTLN("[WIFI] attempt timed out");
      wifiPhase = WIFI_PHASE_IDLE;
      if (!apMode) startCaptivePortal();
    }
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    // Came up without us starting the phase (e.g. the AP scan finished).
    if (wifiPhase != WIFI_PHASE_CONNECTED) onWifiConnected();
    return;
  }

  // Idle and not connected - retry on the interval. This runs whether or not
  // the AP is up, which is the point: the old code only retried in the
  // !apMode branch, so once the captive portal started nothing ever retried
  // again and the device needed a power cycle to get back on the network.
  if (millis() - lastWifiAttempt >= WIFI_RETRY_INTERVAL_MS) {
    startWifiAttempt();
  }
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

// Save new credentials and reconnect without rebooting. The reboot used to be
// the only way back onto the network after a portal visit; with the retry
// state machine in place we can just reconnect, which keeps the running chiller
// uninterrupted. The response is written to the socket first, and the actual
// attempt is deferred to the next loop() via a flag, so there is no delay()
// stalling the interlock.
void handleSave() {
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  if (ssid.length() > 0) {
    saveSettings(ssid, pass);
    server.send(200, "text/html",
                "<!doctype html><meta name=viewport content='width=device-width'>"
                "<body style='font-family:sans-serif;padding:2em'>"
                "<h2>Saved</h2><p>Connecting to " + ssid + "...</p>"
                "<p>This page will not update. Reconnect to your normal network "
                "and browse to <b>http://" HOSTNAME ".local</b> or the IP shown "
                "in the serial log.</p>"
                "<p>The access point stays up while it retries, so you can come "
                "back here if it does not connect.</p>");
    wifiReconnectRequested = true;
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
  json += "\"capVolts\":" + String(adcToVolts(capAdcValue), 2) + ",";
  json += "\"capDischargedMax\":" + String(CAP_DISCHARGED_ADC_MAX) + ",";
  json += "\"capChargedMin\":" + String(CAP_CHARGED_ADC_MIN) + ",";
  json += "\"capDischargedVolts\":" + String(adcToVolts(CAP_DISCHARGED_ADC_MAX), 2) + ",";
  json += "\"capChargedVolts\":" + String(adcToVolts(CAP_CHARGED_ADC_MIN), 2) + ",";
  json += "\"capCharge\":" + String(digitalRead(PIN_CAP_CHARGE) == HIGH ? "true" : "false") + ",";
  json += "\"capDischarging\":" + String(capDischargeTiming ? "true" : "false") + ",";
  json += "\"capDischargeMs\":" + String(capDischargeTiming ? (millis() - capDischargeStart) : 0) + ",";
  json += "\"capLastDischargeMs\":" + String(lastCapDischargeMs) + ",";
  json += "\"compressorRunMs\":" + String(compressorRunning ? (millis() - compressorRunStart) : 0) + ",";
  // A non-zero stop request means the contacts are held closed for the off-delay
  // even though demand is gone. The UI shows the countdown so the hold is visible
  // rather than looking like the switch was ignored.
  json += "\"compressorStopPending\":" + String(compressorStopRequestedAt != 0 ? "true" : "false") + ",";
  json += "\"compressorOffDelayRemaining\":" + String(
      (compressorStopRequestedAt != 0 &&
       millis() - compressorStopRequestedAt < COMPRESSOR_OFF_DELAY_MS)
        ? (COMPRESSOR_OFF_DELAY_MS - (millis() - compressorStopRequestedAt)) : 0) + ",";
  json += "\"currentAdc\":" + String(currentAdcValue) + ",";
  json += "\"currentVolts\":" + String(currentAdcToVolts(currentAdcValue), 2) + ",";
  json += "\"currentLimit\":" + String(COMPRESSOR_SAFE_ADC_MAX) + ",";
  json += "\"currentLimitVolts\":" + String(currentAdcToVolts(COMPRESSOR_SAFE_ADC_MAX), 2) + ",";
  json += "\"currentSafe\":" + String(is12VCurrentSafe() ? "true" : "false") + ",";
  json += "\"fan\":" + String(digitalRead(PIN_FAN) == HIGH ? "true" : "false") + ",";
  json += "\"buzzer\":" + String(digitalRead(PIN_BUZZER) == HIGH ? "true" : "false") + ",";
  json += "\"buzzerHz\":" + String(buzzerFrequency) + ",";
  json += "\"buzzerToneActive\":" + String(buzzerToneActive ? "true" : "false") + ",";
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
// SETTINGS ENDPOINTS
// Capacitor thresholds are accepted in VOLTS from the UI and converted here,
// so the conversion happens once, in one place, on the authoritative side.
// The resulting ADC counts are what the interlock actually uses.
// =========================================================================
void handleSetThresholds() {
  if (!server.hasArg("vdischarged") || !server.hasArg("vcharged")) {
    server.send(400, "text/plain", "need vdischarged and vcharged");
    return;
  }

  int newDischarged = voltsToAdc(server.arg("vdischarged").toFloat());
  int newCharged    = voltsToAdc(server.arg("vcharged").toFloat());

  // Enforce the hysteresis band. Without this, setting the two thresholds
  // equal or inverted would let a charged capacitor satisfy the "discharged"
  // test, and the contacts could close on a charged cap - defeating the whole
  // interlock. Refuse the change rather than silently clamping.
  if (newCharged - newDischarged < CAP_THRESHOLD_MIN_GAP) {
    char msg[96];
    snprintf(msg, sizeof(msg),
             "rejected: charged must exceed discharged by at least %d counts (%.2fV)",
             CAP_THRESHOLD_MIN_GAP, adcToVolts(CAP_THRESHOLD_MIN_GAP));
    server.send(400, "text/plain", msg);
    return;
  }

  CAP_DISCHARGED_ADC_MAX = newDischarged;
  CAP_CHARGED_ADC_MIN   = newCharged;
  saveThresholds();
  server.send(200, "text/plain", "ok");
}

// 12V current-sense overcurrent limit, accepted in volts. Converted here so the
// authoritative value the gate uses stays in raw ADC counts.
void handleSetCurrentLimit() {
  if (!server.hasArg("volts")) {
    server.send(400, "text/plain", "need volts");
    return;
  }
  int newLimit = currentVoltsToAdc(server.arg("volts").toFloat());
  if (newLimit < 1) newLimit = 1; // a limit of 0 counts would never be safe
  COMPRESSOR_SAFE_ADC_MAX = newLimit;
  prefs.begin("chiller", false);
  prefs.putInt("adcmax", COMPRESSOR_SAFE_ADC_MAX);
  prefs.end();
  server.send(200, "text/plain", "ok");
}

void handleSetBuzzer() {
  if (server.hasArg("hz")) {
    int hz = server.arg("hz").toInt();
    if (hz < BUZZER_FREQ_MIN || hz > BUZZER_FREQ_MAX) {
      char msg[80];
      snprintf(msg, sizeof(msg), "rejected: hz must be %d-%d", BUZZER_FREQ_MIN, BUZZER_FREQ_MAX);
      server.send(400, "text/plain", msg);
      return;
    }
    buzzerFrequency = hz;
    // Re-issue the tone if one is currently running, so the new frequency
    // takes effect immediately rather than at the next on/off transition.
    if (buzzerToneActive) {
      noTone(PIN_BUZZER);
      tone(PIN_BUZZER, buzzerFrequency);
    }
    saveBuzzerFrequency();
  }
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
  CAP_DISCHARGED_ADC_MAX  = prefs.getInt("capdis", CAP_DISCHARGED_ADC_MAX);
  CAP_CHARGED_ADC_MIN    = prefs.getInt("capchg", CAP_CHARGED_ADC_MIN);
  buzzerFrequency        = prefs.getInt("buzzerhz", buzzerFrequency);
  prefs.end();

  // A corrupted or hand-edited flash value must not be able to collapse the
  // hysteresis band, so re-assert the band here as well as on write.
  if (CAP_CHARGED_ADC_MIN - CAP_DISCHARGED_ADC_MAX < CAP_THRESHOLD_MIN_GAP) {
    DBG_PRINTLN("[SETTINGS] stored capacitor thresholds violate the hysteresis band, reverting to defaults");
    CAP_DISCHARGED_ADC_MAX = 500;
    CAP_CHARGED_ADC_MIN   = 3000;
  }
  if (buzzerFrequency < BUZZER_FREQ_MIN || buzzerFrequency > BUZZER_FREQ_MAX) {
    buzzerFrequency = 2000;
  }
}

void saveThresholds() {
  prefs.begin("chiller", false);
  prefs.putInt("capdis", CAP_DISCHARGED_ADC_MAX);
  prefs.putInt("capchg", CAP_CHARGED_ADC_MIN);
  prefs.end();
}

void saveBuzzerFrequency() {
  prefs.begin("chiller", false);
  prefs.putInt("buzzerhz", buzzerFrequency);
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
