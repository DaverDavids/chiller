/*
  Chiller - ESP32-C3 Super Mini firmware
  HVAC compressor + pump controller with capacitor pre-charge ADC safety
  interlock, mode-select switch, addressable status LEDs, WiFi (STA +
  captive-portal fallback), OTA, mDNS, flash-persisted settings, and a
  web UI for live status + manual function testing.

  Pin map:
    GPIO5   - Fan output
    GPIO6   - Buzzer output
    GPIO7   - 12V enable output (pump AND pre-charge capacitor charge)
    GPIO8   - Compressor output (gated ONLY by isSafeToRunCompressor())
    GPIO9   - 5V enable output
    GPIO2   - ADC: 12V current sense
    GPIO4   - Timer on/off input
    GPIO0   - Switch position 1 input (OFF)        [active low, pull-up]
    GPIO20  - Switch position 2 input (PUMP ONLY)  [active low, pull-up]
    GPIO3   - Switch position 3 input (CHILLER ONLY) [active low, pull-up]
    GPIO10  - Switch position 4 input (BOTH)       [active low, pull-up]
    LED_DATA_PIN - Addressable status LED data line (TODO: assign, see below)

  MODE LOGIC (from 4-position switch):
    Switch inputs are ACTIVE LOW: a position is "active" when its pin is
    pulled to GND by the switch, otherwise the internal pull-up holds it
    HIGH. Position 1 is the default/off position.
    Position 1 (default) -> everything off
    Position 2 -> pump only (12V enable), compressor never requested
    Position 3 -> chiller only (compressor path); note the pump/12V-enable
                  output is physically shared with the pre-charge capacitor,
                  so it will still pulse on briefly during precharge even in
                  this mode - that's expected/required for the safety interlock.
    Position 4 -> pump AND chiller both requested simultaneously

  PUMP ANTI-SHORT-CYCLE:
    The pump (12V enable) output is NOT switched off the instant demand
    drops. It's held on for PUMP_OFF_DELAY_MS after demand goes away so
    that flipping through switch positions doesn't stop/restart the pump
    every time - see updatePumpOutput().

  SAFETY NOTE: PIN_COMPRESSOR is written in exactly one place,
  setCompressorOutput(), which re-checks the ADC every single call.
  Do not add any other digitalWrite(PIN_COMPRESSOR, ...) elsewhere.
  PIN_12V_EN is written in exactly one place, updatePumpOutput().
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
#define PIN_FAN        5
#define PIN_BUZZER     6
#define PIN_12V_EN     7   // pump AND pre-charge capacitor charge (shared)
#define PIN_COMPRESSOR 8
#define PIN_5V_EN      9
#define PIN_ADC_12V    2   // ADC1 channel, 12V current-sense
#define PIN_TIMER_IN   4   // timer on/off input

// Mode-select switch inputs (one-of-four rotary/slide switch)
#define PIN_SWITCH_1   0   // position 1 - OFF (default)
#define PIN_SWITCH_2   20   // position 2 - pump only
#define PIN_SWITCH_3   3  // position 3 - chiller only
#define PIN_SWITCH_4   10  // position 4 - pump + chiller

// Addressable status LEDs.
// TODO: GPIO budget is tight on the C3 supermini - every GPIO 0-10 is now
// spoken for. GPIO20/21 are the hardware UART0 (Serial/debug) pins, and
// GPIO18/19 are used for native USB on most supermini boards. Pick
// whichever of those your board can spare once you decide how you're
// wiring Serial vs USB, or move a lower-priority input (e.g. buzzer/fan)
// if you need a truly free pin. Left as a placeholder define for now.
#define LED_DATA_PIN   20  // PLACEHOLDER - verify against your final wiring
#define LED_COUNT      8   // PLACEHOLDER - set to actual strip length
Adafruit_NeoPixel statusLeds(LED_COUNT, LED_DATA_PIN, NEO_GRB + NEO_KHZ800);

// ==================== SAFETY THRESHOLDS (PSEUDO/TODO) ====================
// Compressor is allowed ON only when adcValue <= COMPRESSOR_SAFE_ADC_MAX.
// TODO: calibrate against the real capacitor charge curve on the bench.
int COMPRESSOR_SAFE_ADC_MAX = 500;      // placeholder, 0-4095 (12-bit ADC)
const uint8_t ADC_SAMPLE_COUNT = 8;     // simple averaging, tune later

// Pump anti-short-cycle hold time. Pump stays on this long after demand
// drops, so switching modes doesn't stop/restart it every time.
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
int  readCompressorSafetyADC();
bool isSafeToRunCompressor();
void setCompressorOutput(bool wantOn);
void updatePumpOutput(bool pumpDemandNow, bool compressorNeedsPower);
void runStateMachine();
void readSwitchInputs();
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

void setup() {
  DBG_BEGIN(115200);

  // ---- GPIO defaults: ALL outputs LOW at boot, except 5V enable = HIGH ----
  pinMode(PIN_FAN, OUTPUT);        digitalWrite(PIN_FAN, LOW);
  pinMode(PIN_BUZZER, OUTPUT);     digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_12V_EN, OUTPUT);     digitalWrite(PIN_12V_EN, LOW);   // pump/capacitor stays low at boot
  pinMode(PIN_COMPRESSOR, OUTPUT); digitalWrite(PIN_COMPRESSOR, LOW);
  pinMode(PIN_5V_EN, OUTPUT);      digitalWrite(PIN_5V_EN, HIGH);   // default HIGH per spec
  pinMode(PIN_ADC_12V, INPUT);
  pinMode(PIN_TIMER_IN, INPUT);    // TODO: confirm pull-up/pull-down and active level

  // Mode-select switch inputs - ACTIVE LOW: the switch shorts the pin to GND
  // to close its position, internal pull-up holds it HIGH when open.
  pinMode(PIN_SWITCH_1, INPUT_PULLUP);
  pinMode(PIN_SWITCH_2, INPUT_PULLUP);
  pinMode(PIN_SWITCH_3, INPUT_PULLUP);
  pinMode(PIN_SWITCH_4, INPUT_PULLUP);

  analogReadResolution(12); // 0-4095

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

  // ---- sequencing / fault state machine (pseudo, fill in later) ----
  runStateMachine();

  // ---- SAFETY CRITICAL PATH: runs every loop iteration, never blocked ----
  setCompressorOutput(compressorRequested);

  // ---- pump output with anti-short-cycle hold ----
  bool compressorNeedsPower = (state == STATE_PRECHARGE || state == STATE_RUN);
  updatePumpOutput(pumpRequested, compressorNeedsPower);

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
// SAFETY-CRITICAL COMPRESSOR GATE
// This is the ONLY function that may write PIN_COMPRESSOR.
// =========================================================================
int readCompressorSafetyADC() {
  long sum = 0;
  for (uint8_t i = 0; i < ADC_SAMPLE_COUNT; i++) {
    sum += analogRead(PIN_ADC_12V);
  }
  return (int)(sum / ADC_SAMPLE_COUNT);
}

bool isSafeToRunCompressor() {
  int adcValue = readCompressorSafetyADC();
  bool safe = (adcValue <= COMPRESSOR_SAFE_ADC_MAX);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= DEBUG_PRINT_INTERVAL_MS) {
    DBG_PRINTF("[SAFETY] ADC=%d limit=%d safe=%d\n", adcValue, COMPRESSOR_SAFE_ADC_MAX, safe);
    lastPrint = millis();
  }
  return safe;
}

void setCompressorOutput(bool wantOn) {
  bool safe = isSafeToRunCompressor();
  if (wantOn && safe) {
    digitalWrite(PIN_COMPRESSOR, HIGH);
  } else {
    digitalWrite(PIN_COMPRESSOR, LOW);   // any unsafe condition forces OFF immediately
  }
}

// =========================================================================
// PUMP / 12V-ENABLE OUTPUT - anti-short-cycle, single writer of PIN_12V_EN
// =========================================================================
unsigned long pumpOffTimerStart = 0;
bool pumpOffTimerActive = false;

void updatePumpOutput(bool pumpDemandNow, bool compressorNeedsPower) {
  bool wantPumpOn = pumpDemandNow || compressorNeedsPower;
  static bool lastWantPumpOn = false;

  if (wantPumpOn) {
    pumpOffTimerActive = false;
    digitalWrite(PIN_12V_EN, HIGH);
  } else {
    if (lastWantPumpOn && !pumpOffTimerActive) {
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
  lastWantPumpOn = wantPumpOn;
}

// =========================================================================
// MODE-SELECT SWITCH INPUTS - ACTIVE LOW (switch closes to GND), internal
// pull-ups enabled in setup(). TODO: add debounce on position change.
// =========================================================================
void readSwitchInputs() {
  switch1State = (digitalRead(PIN_SWITCH_1) == LOW);
  switch2State = (digitalRead(PIN_SWITCH_2) == LOW);
  switch3State = (digitalRead(PIN_SWITCH_3) == LOW);
  switch4State = (digitalRead(PIN_SWITCH_4) == LOW);
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
  // demand path only (still passes through the ADC safety gate either way).
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
      color = statusLeds.Color(255, 255, 0); // precharging = yellow
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
// STATE MACHINE - pseudo sequencing, fill in real fault/timeout logic
// =========================================================================
void runStateMachine() {
  switch (state) {
    case STATE_IDLE:
      if (compressorRequested) {
        state = STATE_PRECHARGE; // updatePumpOutput() will energize 12V_EN for us
      }
      break;

    case STATE_PRECHARGE:
      // TODO: add a precharge timeout -> STATE_FAULT if never reaches safe level
      if (isSafeToRunCompressor()) {
        state = STATE_RUN;
      }
      if (!compressorRequested) {
        state = STATE_IDLE;
      }
      break;

    case STATE_RUN:
      // compressor pin itself is gated every loop by setCompressorOutput()
      if (!compressorRequested) {
        state = STATE_IDLE;
      }
      // TODO: runtime fault detection (overcurrent, stall, etc.) -> STATE_FAULT
      break;

    case STATE_FAULT:
      // TODO: real fault handling / latch / buzzer pattern / remote alert
      break;
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

// Live JSON status for the web UI to poll (used by html.h page via fetch()).
void handleStatusJson() {
  int adcValue = readCompressorSafetyADC();
  String json = "{";
  json += "\"mode\":\"" + String(modeName()) + "\",";
  json += "\"state\":\"" + String(stateName()) + "\",";
  json += "\"adc\":" + String(adcValue) + ",";
  json += "\"adcLimit\":" + String(COMPRESSOR_SAFE_ADC_MAX) + ",";
  json += "\"safe\":" + String(isSafeToRunCompressor() ? "true" : "false") + ",";
  json += "\"fan\":" + String(digitalRead(PIN_FAN) == HIGH ? "true" : "false") + ",";
  json += "\"buzzer\":" + String(digitalRead(PIN_BUZZER) == HIGH ? "true" : "false") + ",";
  json += "\"pump12v\":" + String(digitalRead(PIN_12V_EN) == HIGH ? "true" : "false") + ",";
  json += "\"compressor\":" + String(digitalRead(PIN_COMPRESSOR) == HIGH ? "true" : "false") + ",";
  json += "\"pumpOffHold\":" + String(pumpOffTimerActive ? "true" : "false") + ",";
  json += "\"switch1\":" + String(switch1State ? "true" : "false") + ",";
  json += "\"switch2\":" + String(switch2State ? "true" : "false") + ",";
  json += "\"switch3\":" + String(switch3State ? "true" : "false") + ",";
  json += "\"switch4\":" + String(switch4State ? "true" : "false") + ",";
  json += "\"testFanActive\":" + String(testFanOverrideActive ? "true" : "false") + ",";
  json += "\"testBuzzerActive\":" + String(testBuzzerOverrideActive ? "true" : "false") + ",";
  json += "\"testCompressorActive\":" + String(testCompressorDemandOverrideActive ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

// Manual test endpoints - fan/buzzer are simple direct overrides (non-safety-
// critical). Compressor test only sets the DEMAND flag; it still has to pass
// isSafeToRunCompressor() every loop, so this can be used to verify the
// interlock itself without ever bypassing it.
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
