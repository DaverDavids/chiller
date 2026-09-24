/*
  Chiller - ESP32-C3 Super Mini firmware
  HVAC compressor controller with capacitor pre-charge ADC safety interlock.
  WiFi (STA + captive-portal fallback), OTA, mDNS, flash-persisted settings.

  Pin map:
    GPIO5  - Fan output
    GPIO6  - Buzzer output
    GPIO7  - 12V enable output (charges pre-charge capacitor)
    GPIO8  - Compressor output (gated ONLY by isSafeToRunCompressor())
    GPIO9  - 5V enable output
    GPIO2  - ADC: 12V current sense
    GPIO4  - Timer on/off input

  SAFETY NOTE: PIN_COMPRESSOR is written in exactly one place,
  setCompressorOutput(), which re-checks the ADC every single call.
  Do not add any other digitalWrite(PIN_COMPRESSOR, ...) elsewhere.
*/

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
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

// ============================= NETWORK =================================
#define HOSTNAME "chiller"
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000;
const unsigned long WIFI_RETRY_INTERVAL_MS  = 30000;

// ============================= PIN MAP ==================================
#define PIN_FAN        5
#define PIN_BUZZER     6
#define PIN_12V_EN     7   // also charges pre-charge capacitor
#define PIN_COMPRESSOR 8
#define PIN_5V_EN      9
#define PIN_ADC_12V    2   // ADC1 channel, 12V current-sense
#define PIN_TIMER_IN   4   // timer on/off input

// ==================== SAFETY THRESHOLDS (PSEUDO/TODO) ====================
// Compressor is allowed ON only when adcValue <= COMPRESSOR_SAFE_ADC_MAX.
// TODO: calibrate against the real capacitor charge curve on the bench.
int COMPRESSOR_SAFE_ADC_MAX = 500;      // placeholder, 0-4095 (12-bit ADC)
const uint8_t ADC_SAMPLE_COUNT = 8;     // simple averaging, tune later

// ============================ STATE MACHINE ==============================
enum ChillerState {
  STATE_IDLE,
  STATE_PRECHARGE,
  STATE_RUN,
  STATE_FAULT
};
ChillerState state = STATE_IDLE;

bool compressorRequested = false;  // demand flag only, NEVER write the pin from here

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
void runStateMachine();
void connectWiFi();
void startCaptivePortal();
void handleRoot();
void handleSave();
void setupOTA();
void loadSettings();
void saveSettings(const String &ssid, const String &pass);

void setup() {
  DBG_BEGIN(115200);

  // ---- GPIO defaults: ALL outputs LOW at boot, except 5V enable = HIGH ----
  pinMode(PIN_FAN, OUTPUT);        digitalWrite(PIN_FAN, LOW);
  pinMode(PIN_BUZZER, OUTPUT);     digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_12V_EN, OUTPUT);     digitalWrite(PIN_12V_EN, LOW);   // capacitor charge stays low at boot
  pinMode(PIN_COMPRESSOR, OUTPUT); digitalWrite(PIN_COMPRESSOR, LOW);
  pinMode(PIN_5V_EN, OUTPUT);      digitalWrite(PIN_5V_EN, HIGH);   // default HIGH per spec
  pinMode(PIN_ADC_12V, INPUT);
  pinMode(PIN_TIMER_IN, INPUT);    // TODO: confirm pull-up/pull-down and active level

  analogReadResolution(12); // 0-4095

  loadSettings();
  connectWiFi();

  if (!apMode) {
    setupOTA();
    if (MDNS.begin(HOSTNAME)) {
      DBG_PRINTLN("mDNS started: " HOSTNAME ".local");
    }
    server.on("/", handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.begin();
  }
}

void loop() {
  // ---- SAFETY CRITICAL PATH: runs every loop iteration, never blocked ----
  setCompressorOutput(compressorRequested);

  // ---- timer input -> demand flag (does not touch the compressor pin) ----
  compressorRequested = (digitalRead(PIN_TIMER_IN) == HIGH); // TODO: confirm active level

  // ---- sequencing / fault state machine (pseudo, fill in later) ----
  runStateMachine();

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
  DBG_PRINTF("[SAFETY] ADC=%d limit=%d safe=%d\n", adcValue, COMPRESSOR_SAFE_ADC_MAX, safe);
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
// STATE MACHINE - pseudo sequencing, fill in real fault/timeout logic
// =========================================================================
void runStateMachine() {
  switch (state) {
    case STATE_IDLE:
      digitalWrite(PIN_12V_EN, LOW);
      if (compressorRequested) {
        digitalWrite(PIN_12V_EN, HIGH); // begin charging pre-charge capacitor
        state = STATE_PRECHARGE;
      }
      break;

    case STATE_PRECHARGE:
      // TODO: add a precharge timeout -> STATE_FAULT if never reaches safe level
      if (isSafeToRunCompressor()) {
        state = STATE_RUN;
      }
      if (!compressorRequested) {
        digitalWrite(PIN_12V_EN, LOW);
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
      digitalWrite(PIN_BUZZER, HIGH); // placeholder
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
    DBG_PRINT(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    DBG_PRINTLN("\nWiFi connected: " + WiFi.localIP().toString());
  } else {
    DBG_PRINTLN("\nWiFi failed, starting captive portal");
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
void handleRoot() {
  if (apMode) {
    server.send(200, "text/html", PAGE_CONFIG);
  } else {
    // TODO: template in live values (ADC reading, state, output states, etc.)
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
