/*
  Chiller - ESP32-C3 Super Mini firmware
  HVAC compressor + pump controller with a hysteretic capacitor charge
  interlock, mode-select switch, addressable status LEDs, WiFi (STA +
  captive-portal fallback), OTA, mDNS, flash-persisted settings, and a
  web UI for live status + manual function testing.

  Pin map:
    GPIO0   - ADC: capacitor voltage sense (charge interlock)
    GPIO1   - Capacitor charge output (LOW at boot)
    GPIO2   - ADC: 12V rail current sense (pump stall ceiling / fan floor)
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
    Position 1 (not wired) -> everything off. It is selected implicitly:
      whenever none of positions 2/3/4 are active, the switch is in position 1.
    Position 2 -> pump only (12V enable), compressor never requested
    Position 3 -> chiller only (compressor path)
    Position 4 -> pump AND chiller both requested simultaneously
    A newly selected position is NOT acted on straight away - the previous
    position keeps running for MODE_SWITCH_DELAY_MS first, so the outputs as
    well as the LEDs wait it out. See updateModeSelection().

  STATUS LED RING:
    LED_COUNT WS2812Bs joined into a circle, one animation per mode: blue
    twinkling back and forth for chiller only, a red chase going round the ring
    for pump only, both at once for both, black for off. The ring wipes the new
    mode's animation in from LED 0 during the same MODE_SWITCH_DELAY_MS the
    mode itself is held for, so the circle is fully lit at the instant the mode
    takes effect. See updateStatusLeds() and the tunables block by LED_COUNT.

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
    A start requires nothing but a proven discharged capacitor; charging must
    complete within PRECHARGE_TIMEOUT_MS or the fault latches rather than
    closing the contacts anyway. The 12V current sense (GPIO2) plays NO part in
    this interlock - it is a rail monitor belonging to the pump and the fan, see
    the section below.

  PUMP ANTI-SHORT-CYCLE:
    The 12V enable output (GPIO7) is NOT switched off the instant pump demand
    drops. It's held on for PUMP_OFF_DELAY_MS after demand goes away so that
    flipping through switch positions doesn't stop/restart the pump every time
    - see update12VEnable().

  12V CURRENT SENSE (GPIO2) - A RAIL MONITOR, NOT A COMPRESSOR SENSOR:
    It reads everything on the 12V rail, which is the fan plus the pump. It is
    read against two limits, and neither of them belongs to the compressor:
      - a CEILING (PUMP_STALL_ADC_MAX) for the pump. Above it the pump is
        stalled or jammed, so the 12V enable is cut on that very loop and the
        pump stays cut until the rail has been clean for PUMP_STALL_RETRY_MS.
        That hold is latched across a mode change on purpose.
      - a FLOOR (FAN_RUN_ADC_MIN) for the fan, checked only while the
        compressor runs. Currently 0, which no reading can fall below, so the
        check is inert until a real running current is measured. It reports
        only; it never opens the compressor contacts, because an uncalibrated
        floor would otherwise stop a healthy chiller.
    The compressor is interlocked by its own capacitor, not by this sensor. The
    contacts close on "capacitor proven charged" alone. GPIO7 is the PUMP enable
    only - it is not the rail for the whole machine - so cutting the pump on a
    stall leaves a running chiller completely unaffected.

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
// so every retry would delay the compressor interlock by that much, with the
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
#define PIN_12V_EN     7   // 12V enable - PUMP ONLY, not the whole rail
#define PIN_COMPRESSOR 8
#define PIN_5V_EN      9

// Mode-select switch inputs (one-of-four rotary/slide switch), active low.
#define PIN_SWITCH_1   -1  // position 1 - OFF. NOT WIRED: no pin assigned yet.
                          // Any negative value reads permanently inactive, so
                          // the pin itself can never select anything. Position 1
                          // does not need it: getSwitchMode() picks OFF whenever
                          // none of the wired positions are active, so parking
                          // the switch here still turns everything off.
#define PIN_SWITCH_2   20  // position 2 - pump only
#define PIN_SWITCH_3   3   // position 3 - chiller only
#define PIN_SWITCH_4   10  // position 4 - pump + chiller

// Addressable status LEDs, wired as a RING - LED_COUNT WS2812Bs joined end to
// end into a circle. Every animation below is written to close on itself:
// whole numbers of wave crests only, and the chase tail wraps behind the head.
// GPIO21 was the hardware UART0 TX pin on most supermini boards - using it for
// LED data means Serial debug has to run over the native USB port (GPIO18/19)
// instead.
#define LED_DATA_PIN   21
#define LED_COUNT      10  // number of LEDs in the circle
Adafruit_NeoPixel statusLeds(LED_COUNT, LED_DATA_PIN, NEO_GRB + NEO_KHZ800);

// ===================== LED RING ANIMATION TUNABLES =======================
// Every knob that decides how the ring LOOKS is in this one block, so the
// pattern can be dialled in on the bench without reading the renderer.
//
// The reveal window is deliberately NOT here: it is derived from
// MODE_SWITCH_DELAY_MS (see updateStatusLeds) so the wipe-in and the mode
// change physically cannot drift apart.

// How often the ring is redrawn. Every redraw ends in the blocking
// statusLeds.show(), so it is never run for nothing.
const unsigned long LED_FRAME_INTERVAL_MS = 30; // ~33fps

// ---- chiller pattern: blue light twinkling back and forth ----
// How many bright crests the wave carries around the ring. MUST be a whole
// number or the wave will not join up where the ring closes.
const uint8_t LED_TWINKLE_CREST_COUNT = 2;
// Time for one there-and-back sweep of the crests. Smaller is faster.
const unsigned long LED_TWINKLE_SWEEP_MS = 2600;
// +1 or -1: which way round the ring the crests drift before turning back.
const int LED_TWINKLE_DIRECTION = 1;
// Brightness of a crest, and of the trough between crests. 0-255.
const uint8_t LED_TWINKLE_BRIGHT = 120;
const uint8_t LED_TWINKLE_DIM    = 25;

// ---- pump pattern: a red chase slowly going round the ring ----
// Time between chase steps. One full lap is this multiplied by LED_COUNT.
const unsigned long LED_CHASE_STEP_MS = 260;
// Brightness of the head of the chase, 0-255. Everything further back than the
// tail below is black.
const uint8_t LED_CHASE_HEAD = 200;
// Brightness 1, 2, 3 ... LEDs behind the head. Add or remove entries to change
// how far the tail reaches.
const uint8_t LED_CHASE_TAIL[] = {110, 55, 20};

// ---- colour ramps: each pattern fades between its own dim and bright shade ----
// The twinkle runs through blueish shades, deep blue up to pale blue.
const uint8_t LED_CHILLER_DIM_COLOR[3]    = {  5,  15,  80};
const uint8_t LED_CHILLER_BRIGHT_COLOR[3] = { 80, 160, 255};
// The chase runs from black up into red, so the tail fades away instead of
// stopping dead.
const uint8_t LED_PUMP_DIM_COLOR[3]    = {  0,   0,   0};
const uint8_t LED_PUMP_BRIGHT_COLOR[3] = {255,  45,  25};

// ==================== SAFETY THRESHOLDS (PSEUDO/TODO) ====================
// Capacitor charge interlock, read on PIN_CAP_SENSE. Two separate thresholds
// so the sequence is hysteretic - a start requires a proven-discharged
// capacitor first, then a proven-charged capacitor before the contacts close.
// TODO: calibrate both against the real capacitor charge curve on the bench.
int CAP_DISCHARGED_ADC_MAX = 500; // at/below this the capacitor counts as
                                  // discharged and a start may be armed
int CAP_CHARGED_ADC_MIN   = 3000; // at/above this the capacitor counts as
                                  // charged and the contacts may close
// 12V current sense (PIN_ADC_12V).
//
// This is a RAIL monitor, not a compressor measurement - it has nothing to do
// with the compressor's own capacitor interlock. It sees everything drawing from
// the 12V rail, which in practice means the fan (whenever the compressor is
// running) plus the pump (whenever the 12V enable is on). That is why it is
// read against a FLOOR and a CEILING rather than as one limit:
//
//   PUMP_STALL_ADC_MAX  the ceiling. Above this the pump is drawing too much -
//                       a stalled or jammed impeller. Cuts the pump instantly.
//   FAN_RUN_ADC_MIN     the floor, checked only while the compressor is running.
//                       Below this the fan is not turning - a seized fan on a
//                       running compressor will cook it. ZERO, so the check is
//                       inert for now: the sense amplifier's offset and the
//                       fan's running current have not been measured yet, and
//                       guessing a floor would trip on noise. Set it once a
//                       real running-current figure is known.
//
// The two overlap on purpose: while the compressor runs the reading includes
// the fan's baseline, so the ceiling has to sit above that baseline or the pump
// would be cut for the fan's sake. Calibrate both against a meter.
int PUMP_STALL_ADC_MAX = 500;           // placeholder, 0-4095 (12-bit ADC)
int FAN_RUN_ADC_MIN    = 0;             // 0 = fan check disabled, see above
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
// There is no longer any safety trip that bypasses this hold, because the only
// safety input the compressor has is the capacitor interlock itself. A 12V
// overcurrent is a PUMP fault and does not touch these contacts.
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

// How long the pump stays cut after a stall, measured from the LAST overcurrent
// sample rather than from the first. Restarting the clock on every overcurrent
// sample means the hold can only elapse once the rail has been clean for the
// whole window, so a pump that trips as soon as it is re-energised never gets
// to run.
//
// Deliberately NOT reset by pump demand dropping. A stall is a real fault and
// the hold is latched across a mode change: putting the switch to OFF and back
// again does not buy a free restart, it just burns wall-clock time while the
// pump stays cut.
const unsigned long PUMP_STALL_RETRY_MS = 5000; // TODO: tune to taste

// Mode change delay. A newly selected switch position is NOT acted on the
// moment it is read: the mode actually in effect keeps running the PREVIOUS
// position for this long first. See updateModeSelection().
//
// This gates the hardware, not just the LEDs. The pump and compressor demand
// flags are derived from the mode in effect, so both outputs wait the window
// out as well - a switch nudged into a new position and back cannot reach the
// pump or the compressor, and there is no way to skip a delay by re-reading the
// switch.
//
// The LED ring spends exactly this same window wiping the NEW mode's animation
// in from LED 0 upwards, so the last LED lights on the same tick the mode takes
// effect and the animation is never caught half-revealed.
const unsigned long MODE_SWITCH_DELAY_MS = 3000;

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
// The debounced switch reading, and the mode actually in effect. They are the
// same value except during the MODE_SWITCH_DELAY_MS hold after a change:
// selectedMode is what the operator asked for, currentMode is what is running.
// updateModeSelection() is the only writer of any of them, so the delay cannot
// be bypassed by reading currentMode instead.
ChillerMode selectedMode = MODE_OFF;
ChillerMode currentMode   = MODE_OFF;
// The mode the LED ring is wiping away from. Reset to currentMode every time
// the selection changes, so a switch that bounces mid-hold still wipes out of
// what was actually on the ring rather than out of a mode that never ran.
ChillerMode outgoingMode  = MODE_OFF;
unsigned long modeChangeStart = 0; // when selectedMode last changed

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

// Pump stall, from the 12V rail current exceeding PUMP_STALL_ADC_MAX. Cuts the
// 12V enable immediately and keeps the pump cut until the rail has been clean
// for PUMP_STALL_RETRY_MS. Separate from compressorFault on purpose: the stall
// says nothing about the capacitor interlock, so it must not be able to latch
// the chiller off, and the chiller interlock must not be able to clear it.
bool pumpStalled = false;
unsigned long pumpStallStart = 0;   // last overcurrent sample; drives the retry hold

// Fan not turning, from the 12V rail current sitting below FAN_RUN_ADC_MIN
// while the compressor runs. INERT while FAN_RUN_ADC_MIN is 0 - a reading can
// never be below 0, so the check cannot fire until a real floor is measured.
// Reported for visibility only; it drives no output.
bool fanStalled = false;
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
bool is12VOvercurrent();
bool isFanDrawingCurrent();
void updateCompressorInterlock(bool wantCompressor);
void update12VEnable(bool pumpDemandNow);
unsigned long pumpStalledRetryMs();
void runStateMachine();
void readSwitchInputs();
bool readSwitchActive(int pin);
void setupSwitchPin(int pin);
void updateDebounced(DebouncedSwitch &d, bool raw);
ChillerMode getSwitchMode();
void updateModeSelection(ChillerMode selected);
void applyMode();
void updateStatusLeds();
uint8_t twinkleLevel(uint8_t index, unsigned long now);
uint8_t chaseLevel(uint8_t index, unsigned long now);
int rampChannel(uint8_t dim, uint8_t bright, uint8_t level);
uint8_t ledRevealCount();
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
const char* selectedModeName();
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
  // A newly selected position is held back for MODE_SWITCH_DELAY_MS before it
  // becomes the mode in effect, so it gates the demand flags below as well as
  // the LED ring.
  updateModeSelection(getSwitchMode());
  applyMode(); // sets compressorRequested / pumpRequested from mode + overrides

  // ---- fault latch / display state, derived from the interlock ----
  runStateMachine();

  // ---- SAFETY CRITICAL PATH: runs every loop iteration, never blocked ----
  // Sole owner of the charge output and the compressor output.
  updateCompressorInterlock(compressorRequested);

  // ---- 12V enable: pump only, with anti-short-cycle hold and stall cut ----
  // compressorRunning is deliberately NOT passed in. This pin gates the pump;
  // the compressor runs off its own interlocked contacts.
  update12VEnable(pumpRequested);

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
// True when the 12V rail is drawing more than the pump stall ceiling - i.e. the
// pump is jammed or shorted. This is a PUMP fault. It says nothing about the
// compressor, which is interlocked by its capacitor, so it must never gate the
// compressor contacts either way.
bool is12VOvercurrent() {
  int adcValue = read12VCurrentADC();
  bool over = (adcValue > PUMP_STALL_ADC_MAX);

  static unsigned long lastPrint = 0;
  if (over || millis() - lastPrint >= DEBUG_PRINT_INTERVAL_MS) {
    DBG_PRINTF("[CURRENT] adc=%d stallMax=%d overcurrent=%d\n",
               adcValue, PUMP_STALL_ADC_MAX, over ? 1 : 0);
    lastPrint = millis();
  }
  return over;
}

// True when the rail current is at or above the fan floor, i.e. the fan is
// pulling current and so is turning. A 0 floor means "not measured yet": no
// reading can be below 0, so this is unconditionally true and the check stays
// inert until FAN_RUN_ADC_MIN is calibrated. Deliberately returns a bool rather
// than comparing inline at the call site, so the disabled case is one obvious
// place to look.
bool isFanDrawingCurrent() {
  if (FAN_RUN_ADC_MIN <= 0) return true; // disabled until measured
  return read12VCurrentADC() >= FAN_RUN_ADC_MIN;
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
//                     CAP_CHARGED_ADC_MIN. That is the ONLY condition.
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
    fanStalled = false;
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

      // Bound the entire charging phase, not just the "not charged yet" part, so
      // the charge output can never be left energized indefinitely if the
      // capacitor will not charge. Latch a fault instead of sitting here.
      //
      // With the 12V current sense removed as a start precondition (it measures
      // the RAIL - fan plus pump - and says nothing about the compressor's
      // capacitor), failing to charge is the only reason the contacts stay open
      // here, so this is now purely a "capacitor or charge path is dead" trip.
      //
      // Checked BEFORE the charge output is set HIGH, and parked LOW on the
      // trip, so the pin state is deterministic within this same iteration
      // rather than waiting for the next loop to clear it - the same invariant
      // the demand-lost branch above relies on. The charge output is never
      // energized on the iteration that gives up.
      if (millis() - capChargeStart >= PRECHARGE_TIMEOUT_MS) {
        digitalWrite(PIN_CAP_CHARGE, LOW);
        capSeq = CAP_SEQ_WAIT_DISCHARGE;
        compressorFault = true;
        DBG_PRINTF("[FAULT] capacitor did not charge in %lums: cap=%d (need >=%d)\n",
                   (unsigned long)PRECHARGE_TIMEOUT_MS, cap, CAP_CHARGED_ADC_MIN);
        break;
      }

      digitalWrite(PIN_CAP_CHARGE, HIGH);

      // Capacitor proven charged - the ONLY condition for closing the contacts.
      // The 12V rail current is deliberately not consulted: the rail belongs to
      // the fan and the pump, and a pump fault is handled by update12VEnable(),
      // which cuts the pump. Gating the compressor on it would mean a stalled
      // pump could stop the chiller, and a healthy quiet rail could stop it too.
      if (cap >= CAP_CHARGED_ADC_MIN) {
        digitalWrite(PIN_COMPRESSOR, HIGH);
        compressorRunning = true;
        compressorRunStart = millis();
        capSeq = CAP_SEQ_RUNNING;
      }
      break;

    case CAP_SEQ_RUNNING: {
      // The compressor's only safety trip is its own minimum run time, checked
      // below. There is deliberately NO 12V overcurrent trip here any more: the
      // current sense measures the rail, which is the fan and the pump, so an
      // overcurrent is a pump fault and belongs to update12VEnable(), which cuts
      // the pump and leaves the chiller running. A pump jam no longer stops the
      // chiller.
      //
      // What replaced it is a report-only fan check. A seized fan on a running
      // compressor will overheat it, so the floor is sampled every loop while
      // the contacts are closed - but FAN_RUN_ADC_MIN is 0, which no reading can
      // fall below, so this cannot fire until the floor has been measured
      // against a real fan. It sets a flag for the web UI and does NOT open the
      // contacts, because a spurious trip on an uncalibrated floor would stop a
      // healthy chiller.
      fanStalled = !isFanDrawingCurrent();

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

  // The fan floor is only meaningful while the contacts are closed, so clear it
  // on every other path rather than letting a stale true from the last run sit
  // on the web UI with the compressor stopped. Done here as well as in the
  // latched-fault early return above, so there is one rule and one place.
  if (capSeq != CAP_SEQ_RUNNING) fanStalled = false;
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
// 12V ENABLE OUTPUT - PUMP ONLY, anti-short-cycle, stall cut.
//
// Single writer of PIN_12V_EN.
//
// This pin gates the PUMP and nothing else. It is NOT the 12V rail for the
// whole machine, and the compressor is not fed through it - the compressor has
// its own interlocked contacts on PIN_COMPRESSOR. So an earlier version of this
// function ORed compressorIsRunning in "so the rail is never cut out from under
// a closed contact"; that was wrong, and it had two bad consequences. It kept
// the pump powered whenever the chiller ran, and - much worse - it meant there
// was no way to cut an overcurrenting pump without also killing a running
// chiller. The OR is gone.
//
// Because the pin is pump-only, a pump stall can now be handled properly and
// locally: cut the pump on the very first overcurrent sample, keep it cut until
// the rail has been clean for PUMP_STALL_RETRY_MS, and leave the compressor
// untouched throughout.
// =========================================================================
unsigned long pumpOffTimerStart = 0;
bool pumpOffTimerActive = false;

// How much longer the pump stays cut. 0 when not stalled. Clamped rather than a
// bare unsigned subtraction, so it can never report a wrapped 49-day hold.
unsigned long pumpStalledRetryMs() {
  if (!pumpStalled) return 0;
  unsigned long elapsed = millis() - pumpStallStart;
  if (elapsed >= PUMP_STALL_RETRY_MS) return 0;
  return PUMP_STALL_RETRY_MS - elapsed;
}

void update12VEnable(bool pumpDemandNow) {
  // ---- stall detection first, so the cut lands on this very loop ----
  // is12VOvercurrent() also does the debug print, rate-limited unless it trips.
  bool overcurrent = is12VOvercurrent();

  if (overcurrent) {
    if (!pumpStalled) {
      DBG_PRINTF("[PUMP] 12V overcurrent, cutting 12V enable now; retry hold %lums\n",
                 (unsigned long)PUMP_STALL_RETRY_MS);
    }
    pumpStalled = true;
    // Restarted on EVERY overcurrent sample, not just the first. The retry hold
    // is therefore measured from the last bad reading and can only elapse after
    // the rail has been clean for the entire window - a pump that trips the
    // instant it is re-energised never gets a second attempt.
    pumpStallStart = millis();
  } else if (pumpStalled && (millis() - pumpStallStart >= PUMP_STALL_RETRY_MS)) {
    pumpStalled = false;
    DBG_PRINTLN("[PUMP] rail clean for the retry hold, stall cleared");
  }

  // ---- now decide the pin ----
  static bool lastPumpDemand = false;
  bool enable = false;

  if (pumpStalled) {
    // Cut, and keep the anti-short-cycle hold from ever re-energising it. That
    // hold exists to smooth a demand transition; honouring it here would hold an
    // overcurrenting pump powered for up to PUMP_OFF_DELAY_MS after the fault,
    // which is exactly the delay the stall cut exists to avoid. Cancelling it
    // rather than pausing it also means a demand edge during a stall cannot
    // leave a stale timer that fires later and re-enables the pin.
    pumpOffTimerActive = false;
  } else if (pumpDemandNow) {
    pumpOffTimerActive = false;
    enable = true;
  } else {
    // Normal demand-driven stop, with the anti-short-cycle hold.
    if (lastPumpDemand && !pumpOffTimerActive) {
      pumpOffTimerActive = true;
      pumpOffTimerStart = millis();
    }
    if (pumpOffTimerActive && (millis() - pumpOffTimerStart < PUMP_OFF_DELAY_MS)) {
      enable = true; // still holding on during the delay
    } else {
      pumpOffTimerActive = false;
    }
  }

  digitalWrite(PIN_12V_EN, enable ? HIGH : LOW);
  // Tracks DEMAND, not the final pin state. Edge-detecting on the pin instead
  // would miss a demand drop that happened while a stall was cutting the pump,
  // and the anti-short-cycle hold would then never start.
  lastPumpDemand = pumpDemandNow;
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
  // Position 1 is OFF and has no pin of its own (PIN_SWITCH_1 is -1), so it is
  // selected implicitly: whenever none of the wired positions 2/3/4 read
  // active, the switch is sitting in position 1. Its own input is ignored for
  // selection either way, which is what makes an unwired position 1 usable.
  //
  // Anything else - more than one wired position active - is a wiring fault or a
  // transition glitch. The debounce in readSwitchInputs() should have filtered
  // the glitch already, but if one gets through it still fails safe to OFF
  // rather than picking an arbitrary position and running the wrong hardware.
  if (!switch2State && !switch3State && !switch4State) return MODE_OFF;
  if (switch2State && switch3State) return MODE_OFF;
  if (switch2State && switch4State) return MODE_OFF;
  if (switch3State && switch4State) return MODE_OFF;
  if (switch2State) return MODE_PUMP_ONLY;
  if (switch3State) return MODE_CHILLER_ONLY;
  return MODE_BOTH; // switch4State
}

// Sole owner of selectedMode, currentMode and outgoingMode, called every loop
// before applyMode(). A change to the switch reading is stamped and then ignored
// until the delay has run out, so the mode in effect cannot change without the
// hold being served first. Runs before updateStatusLeds(), which relies on the
// two modes agreeing on when the hold ends so the wipe and the switch land on
// the same tick.
void updateModeSelection(ChillerMode selected) {
  if (selected != selectedMode) {
    // Whatever is on the ring right now is what the wipe starts from. Taken
    // from currentMode, not from the selection just abandoned, so a switch that
    // bounces part way through a hold still wipes out of the mode that was
    // actually running.
    outgoingMode = currentMode;
    selectedMode = selected;
    modeChangeStart = millis();
    return; // the hold starts now; the mode in effect does not move yet
  }
  if (currentMode == selectedMode) return;                  // already caught up
  if (millis() - modeChangeStart < MODE_SWITCH_DELAY_MS) return; // still holding
  currentMode = selectedMode;
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
// STATUS LEDS - one animation per mode, wiped in over the mode change delay.
//
// The ring is a circle, so the two patterns are written to close on themselves:
// the twinkle carries a whole number of crests around the loop, and the chase
// measures distance backwards so its tail wraps behind its head.
//
// CHILLER_ONLY  blueish shades twinkling back and forth around the ring
// PUMP_ONLY     a red chase stepping slowly around the ring
// BOTH          both of the above at once, mixed additively
// OFF           black
//
// A mode change is not instantaneous. updateModeSelection() keeps the previous
// mode running for MODE_SWITCH_DELAY_MS, and the ring spends exactly that same
// window switching over from LED 0 upwards, showing the pattern of the mode
// that is ABOUT to take effect. The last LED switches on the same tick the mode
// becomes current, so the animation is never caught half-revealed.
//
// LEDs the wipe has not reached yet still show the mode being REPLACED, not
// black. That is what makes the wipe work in both directions from one rule:
// filling up from OFF reveals the new animation LED by LED, and going to OFF -
// whose pattern is black - extinguishes the old animation LED by LED instead of
// blanking the whole ring in a single step.
//
// This reads the mode state only; it drives no output pin and cannot reach the
// pump or the compressor.
// =========================================================================

// Chiller pattern brightness for one LED, 0-255: a smooth wave carrying
// LED_TWINKLE_CREST_COUNT crests around the ring, drifting back and forth
// between the two ends of the ring. The wave phase is modulated by a sine
// rather than driven linearly, so the crests slow to a stop at each end and
// turn around smoothly instead of snapping back and jerking.
uint8_t twinkleLevel(uint8_t index, unsigned long now) {
  // Reduced to a single sweep BEFORE converting to float, deliberately. millis()
  // runs for weeks, and 2*pi*sweep on a sweep count in the hundreds of thousands
  // has so little left in a 32-bit float mantissa that the argument arrives at
  // cosf() quantised to whole radians and the twinkle grinds to a standstill.
  // Reducing in integer arithmetic keeps the argument inside 0..2*pi for the
  // life of the sketch, and costs nothing.
  float sweep = (float)(now % LED_TWINKLE_SWEEP_MS) / (float)LED_TWINKLE_SWEEP_MS;
  float phase = PI * (float)LED_TWINKLE_CREST_COUNT * (float)LED_TWINKLE_DIRECTION
              * sinf(2.0f * PI * sweep);
  float angle = 2.0f * PI * (float)LED_TWINKLE_CREST_COUNT
              * (float)index / (float)LED_COUNT - phase;
  float wave = 0.5f + 0.5f * cosf(angle); // 0..1
  // Scaled in float and only rounded at the very end. Truncating the wave to an
  // int first would collapse it to 0 or 1 and turn the twinkle into a hard
  // on/off blink between two brightnesses instead of a gradient.
  float level = (float)LED_TWINKLE_DIM
              + ((float)LED_TWINKLE_BRIGHT - (float)LED_TWINKLE_DIM) * wave;
  if (level < 0.0f) level = 0.0f;
  if (level > 255.0f) level = 255.0f;
  return (uint8_t)(level + 0.5f);
}

// Pump pattern brightness for one LED, 0-255: one bright head stepping around
// the ring, with a short tail fading out behind it and black everywhere else.
// Distance is counted backwards from the head so the tail wraps the circle
// rather than running off the end of the strip.
uint8_t chaseLevel(uint8_t index, unsigned long now) {
  uint8_t head = (uint8_t)((now / LED_CHASE_STEP_MS) % LED_COUNT);
  uint8_t behind = (uint8_t)((head + LED_COUNT - index) % LED_COUNT);
  if (behind == 0) return LED_CHASE_HEAD;
  if ((size_t)(behind - 1) >=
      (sizeof(LED_CHASE_TAIL) / sizeof(LED_CHASE_TAIL[0]))) return 0;
  return LED_CHASE_TAIL[behind - 1];
}

// Scale a 0-255 brightness onto one channel of a dim->bright colour ramp.
int rampChannel(uint8_t dim, uint8_t bright, uint8_t level) {
  return (int)dim + ((int)bright - (int)dim) * (int)level / 255;
}

// How far the mode-change wipe has got: the number of LEDs from index 0 upwards
// that have already switched to the new mode. Every LED in the ring once the
// mode has caught up.
//
// Derived from the same timer updateModeSelection() uses rather than from a
// second counter, so the wipe and the mode change cannot drift apart, and so
// handleStatusJson() can report the ring's real progress without re-deriving
// the arithmetic a second time and getting it subtly wrong.
uint8_t ledRevealCount() {
  if (currentMode == selectedMode) return LED_COUNT; // settled
  unsigned long elapsed = millis() - modeChangeStart;
  if (elapsed >= MODE_SWITCH_DELAY_MS) return LED_COUNT;
  // elapsed is below the delay, so the multiply cannot overflow.
  uint8_t lit = (uint8_t)(elapsed * LED_COUNT / MODE_SWITCH_DELAY_MS);
  if (lit > LED_COUNT) lit = LED_COUNT;
  return lit;
}

void updateStatusLeds() {
  static unsigned long lastFrameAt = 0;
  static uint8_t lastLit = 0xFF; // no LED is 0xFF, so frame 0 always renders

  unsigned long now = millis();

  // LEDs below this index have switched to the new mode, the rest are still
  // showing the old one.
  uint8_t lit = ledRevealCount();

  // Redraw on the animation clock, and immediately whenever the wipe gains a
  // LED so a mode change is never held up by a frame. Everything in between is
  // skipped: statusLeds.show() is the one blocking call on an otherwise
  // non-blocking loop, and there is nothing new to push to the strip.
  if (lit == lastLit && (now - lastFrameAt) < LED_FRAME_INTERVAL_MS) return;
  lastFrameAt = now;
  lastLit = lit;

  // Driven by selectedMode, not currentMode: during the hold the ring is
  // showing the animation that is about to start, so the wipe reveals the new
  // pattern rather than fading the old one out. outgoingMode is what the wipe
  // has not reached yet, so it stays on screen until each LED is taken over.
  bool chillerOn = (selectedMode == MODE_CHILLER_ONLY || selectedMode == MODE_BOTH);
  bool pumpOn    = (selectedMode == MODE_PUMP_ONLY   || selectedMode == MODE_BOTH);
  bool outChiller = (outgoingMode == MODE_CHILLER_ONLY || outgoingMode == MODE_BOTH);
  bool outPump    = (outgoingMode == MODE_PUMP_ONLY   || outgoingMode == MODE_BOTH);

  for (uint8_t i = 0; i < LED_COUNT; i++) {
    bool reached = (i < lit);
    bool chillerHere = reached ? chillerOn : outChiller;
    bool pumpHere    = reached ? pumpOn    : outPump;

    // Each pattern contributes only while it is running on this LED. Gating on
    // the pattern and not on its brightness matters: the colour ramps have a dim
    // end, not a black end, so a pattern that was merely scaled to zero would
    // still paint its dim colour over every LED instead of disappearing.
    int r = 0, g = 0, b = 0;
    if (chillerHere) {
      uint8_t level = twinkleLevel(i, now);
      r += rampChannel(LED_CHILLER_DIM_COLOR[0], LED_CHILLER_BRIGHT_COLOR[0], level);
      g += rampChannel(LED_CHILLER_DIM_COLOR[1], LED_CHILLER_BRIGHT_COLOR[1], level);
      b += rampChannel(LED_CHILLER_DIM_COLOR[2], LED_CHILLER_BRIGHT_COLOR[2], level);
    }
    if (pumpHere) {
      uint8_t level = chaseLevel(i, now);
      r += rampChannel(LED_PUMP_DIM_COLOR[0], LED_PUMP_BRIGHT_COLOR[0], level);
      g += rampChannel(LED_PUMP_DIM_COLOR[1], LED_PUMP_BRIGHT_COLOR[1], level);
      b += rampChannel(LED_PUMP_DIM_COLOR[2], LED_PUMP_BRIGHT_COLOR[2], level);
    }
    // The two patterns are summed per channel and clamped, so in MODE_BOTH the
    // twinkle and the chase run at the same time and the overlap between them
    // comes out magenta rather than one pattern washing the other out.
    if (r > 255) r = 255;
    if (g > 255) g = 255;
    if (b > 255) b = 255;
    statusLeds.setPixelColor(i, statusLeds.Color(r, g, b));
  }
  statusLeds.show();
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
    default:                return "UNKNOWN";
  }
}

// The position the switch is actually sitting in, which differs from the mode
// in effect only while the MODE_SWITCH_DELAY_MS hold is running.
const char* selectedModeName() {
  switch (selectedMode) {
    case MODE_OFF:          return "OFF";
    case MODE_PUMP_ONLY:    return "PUMP_ONLY";
    case MODE_CHILLER_ONLY: return "CHILLER_ONLY";
    case MODE_BOTH:         return "BOTH";
    default:                return "UNKNOWN";
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
  // The switch position vs the mode in effect. They differ only while the
  // MODE_SWITCH_DELAY_MS hold is running, and the countdown is what the UI
  // shows so the delay reads as a deliberate wait rather than the switch having
  // been ignored.
  json += "\"selectedMode\":\"" + String(selectedModeName()) + "\",";
  json += "\"modeChangeRemainingMs\":" + String(
      (currentMode != selectedMode &&
       millis() - modeChangeStart < MODE_SWITCH_DELAY_MS)
        ? (MODE_SWITCH_DELAY_MS - (millis() - modeChangeStart)) : 0) + ",";
  json += "\"state\":\"" + String(stateName()) + "\",";
  json += "\"ledRevealCount\":" + String((unsigned long)ledRevealCount()) + ",";
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
  json += "\"pumpStallLimit\":" + String(PUMP_STALL_ADC_MAX) + ",";
  json += "\"pumpStallLimitVolts\":" + String(currentAdcToVolts(PUMP_STALL_ADC_MAX), 2) + ",";
  json += "\"fanRunMin\":" + String(FAN_RUN_ADC_MIN) + ",";
  json += "\"fanRunMinVolts\":" + String(currentAdcToVolts(FAN_RUN_ADC_MIN), 2) + ",";
  json += "\"pumpStalled\":" + String(pumpStalled ? "true" : "false") + ",";
  // Guarded rather than trusting the subtraction: update12VEnable() runs earlier
  // in this same loop and clears pumpStalled the moment the hold elapses, so
  // elapsed is always below the hold here - but an unsigned underflow would show
  // up as a 49-day countdown in the UI, so clamp rather than rely on it.
  json += "\"pumpStallRetryRemainingMs\":" + String(pumpStalledRetryMs()) + ",";
  json += "\"fanStalled\":" + String(fanStalled ? "true" : "false") + ",";
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

// 12V rail pump-stall ceiling, accepted in volts. Converted here so the
// authoritative value the pump logic uses stays in raw ADC counts.
void handleSetCurrentLimit() {
  if (!server.hasArg("volts")) {
    server.send(400, "text/plain", "need volts");
    return;
  }
  int newLimit = currentVoltsToAdc(server.arg("volts").toFloat());
  if (newLimit < 1) newLimit = 1; // a ceiling of 0 would trip on any reading
  // The prefs key stays "adcmax" on purpose: it is the same number as before,
  // only the thing it means has changed, so existing calibration carries over.
  PUMP_STALL_ADC_MAX = newLimit;
  prefs.begin("chiller", false);
  prefs.putInt("adcmax", PUMP_STALL_ADC_MAX);
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
  PUMP_STALL_ADC_MAX = prefs.getInt("adcmax", PUMP_STALL_ADC_MAX);
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
