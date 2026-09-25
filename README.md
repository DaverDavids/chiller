# chiller

ESP32-C3 Super Mini firmware (Arduino IDE) for an HVAC chiller controller:
compressor + pump control with a capacitor pre-charge / ADC safety
interlock, a 4-position mode switch, addressable status LEDs, WiFi with
captive-portal fallback, OTA updates, mDNS, flash-persisted settings, and
a web UI for live status and manual function testing.

## Pin map

| GPIO | Function                              | Boot default |
|------|----------------------------------------|---------------|
| 5    | Fan output                            | LOW |
| 6    | Buzzer output                         | LOW |
| 7    | 12V enable output (pump + pre-charge capacitor, shared) | LOW |
| 8    | Compressor output                     | LOW |
| 9    | 5V enable output                      | HIGH |
| 2    | ADC - 12V current sense (input)       | n/a |
| 4    | Timer on/off input                    | n/a |
| 0    | Switch position 1 - OFF (default)     | n/a |
| 1    | Switch position 2 - pump only          | n/a |
| 3    | Switch position 3 - chiller only       | n/a |
| 10   | Switch position 4 - pump + chiller     | n/a |
| LED_DATA_PIN (placeholder GPIO20) | Addressable status LED data | n/a |

GPIO budget note: every GPIO 0-10 is now assigned. `LED_DATA_PIN` is a
placeholder on GPIO20 (normally UART0/Serial) - pick a pin that actually
matches how you wire Serial vs USB on your board, or free one up by moving
a lower-priority signal.

## Mode switch behavior

A 4-position switch selects one of:

- Position 1 (default): everything off.
- Position 2: pump only (12V enable on), compressor never requested.
- Position 3: chiller only (compressor path). Note the 12V-enable output
  is physically shared with the pre-charge capacitor, so it still pulses
  on briefly during the precharge sequence even in this mode.
- Position 4: pump and chiller both requested simultaneously.

If the switch reads zero or more than one position active at once
(wiring fault, transition glitch), the firmware fails safe to OFF.

## Safety design

The compressor output (GPIO8) is written in exactly one function,
`setCompressorOutput()`, which re-reads and re-checks the ADC (GPIO2) on
*every* call before allowing the pin HIGH. The compressor is only permitted
on when `adcValue <= COMPRESSOR_SAFE_ADC_MAX`. This runs every `loop()`
iteration ahead of any network/web handling, so a hung network task cannot
keep the compressor on longer than the safety condition allows.

The pump/12V-enable output (GPIO7) is written in exactly one function,
`updatePumpOutput()`.

Do not add any other `digitalWrite()` calls on either pin anywhere else in
the sketch - all control must go through these two single gates.

## Pump anti-short-cycle delay

The pump is not switched off the instant demand disappears. `updatePumpOutput()`
holds it on for `PUMP_OFF_DELAY_MS` (placeholder: 30 seconds) after demand
drops, so flipping through switch positions doesn't stop and immediately
restart the pump. Adjust `PUMP_OFF_DELAY_MS` to taste.

## Status LEDs

An addressable LED strip (Adafruit_NeoPixel) shows current status:

- Off: idle, nothing running
- Blue: pump running (pump-only mode, or in the off-delay hold window)
- Yellow: precharging before compressor start
- Orange: chiller running, pump not otherwise requested
- Purple: chiller and pump both running
- Flashing red: fault state

This is a strip-wide single-color scheme for now - see the TODO in
`updateStatusLeds()` for per-LED per-function assignment.

## Web UI

Connect to `http://chiller.local/` (or the device IP) for a live status
page that polls `/status` (JSON) every second and shows: mode, state
machine state, ADC reading vs. safety limit, fan/buzzer/pump/compressor
output states, and all 4 switch positions.

Manual test controls are provided for:

- Fan on/off/auto
- Buzzer on/off/auto
- Compressor demand on/off/auto (still passes through the ADC safety gate,
  so this can be used to verify the interlock itself without bypassing it)

## Debug output

Serial debug is now throttled (`DEBUG_PRINT_INTERVAL_MS`, default 2000 ms)
instead of printing every loop iteration, so it no longer spams the
console. Toggle all serial debugging on/off via `#define DEBUG 1` / `0` at
the top of `chiller.ino`.

## Networking

- Hostname (WiFi/mDNS/OTA): `chiller`
- WiFi TX power is set to 8.5 dBm immediately after `WiFi.begin()`.
- Credentials come from `Secrets.h` (`MYSSIDIOT` / `MYPSKIOT`) on first boot,
  and are overridden by whatever is saved via the captive portal (stored in
  flash with the `Preferences` library).
- If WiFi fails to connect within `WIFI_CONNECT_TIMEOUT_MS`, the device
  starts an open AP named `chiller` with a captive-portal config page at
  `/` where new credentials can be entered and saved.
- Reconnection is attempted automatically in `loop()` on a
  `WIFI_RETRY_INTERVAL_MS` timer, without blocking other logic.

## Files

- `chiller.ino` - main sketch (setup/loop, safety gates, mode logic, state
  machine, status LEDs, WiFi/OTA, web handlers, flash settings)
- `html.h` - all HTML/JS markup (captive portal page + live status/test page)
- `Secrets.h.example` - copy to `Secrets.h` and fill in real WiFi credentials (not committed)

## TODO / left for calibration

- `COMPRESSOR_SAFE_ADC_MAX` placeholder (500) needs calibration against the
  real capacitor charge curve.
- Precharge timeout and real fault-state handling in `runStateMachine()`.
- Active level / debounce for the timer input and all 4 switch inputs.
- Confirm whether the timer input should gate/AND with the switch mode for
  compressor demand, or be removed/repurposed now that the switch exists.
- `LED_DATA_PIN` and `LED_COUNT` are placeholders - assign a real pin (GPIO
  budget is tight, see Pin map note above) and set the actual strip length.
- Per-LED status mapping instead of single strip-wide color.
- Live status page could add fan/buzzer auto-off warnings, mode override, etc.
