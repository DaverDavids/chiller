# chiller

ESP32-C3 Super Mini firmware (Arduino IDE) for an HVAC compressor controller
with a capacitor pre-charge / ADC safety interlock, WiFi with captive-portal
fallback, OTA updates, mDNS, and flash-persisted settings.

## Pin map

| GPIO | Function                              | Boot default |
|------|----------------------------------------|---------------|
| 5    | Fan output                            | LOW |
| 6    | Buzzer output                         | LOW |
| 7    | 12V enable output (charges pre-charge capacitor) | LOW |
| 8    | Compressor output                     | LOW |
| 9    | 5V enable output                      | HIGH |
| 2    | ADC - 12V current sense (input)       | n/a |
| 4    | Timer on/off input                    | n/a |
| 0    | Switch position 1 input (placeholder, can be reassigned) | n/a |
| 1    | Switch position 2 input (placeholder, can be reassigned) | n/a |
| 3    | Switch position 3 input (placeholder, can be reassigned) | n/a |
| 10   | Switch position 4 input (placeholder, can be reassigned) | n/a |

## Safety design

The compressor output (GPIO8) is written in exactly one function,
`setCompressorOutput()`, which re-reads and re-checks the ADC (GPIO2) on
*every* call before allowing the pin HIGH. The compressor is only permitted
on when `adcValue <= COMPRESSOR_SAFE_ADC_MAX`. This function is called every
`loop()` iteration ahead of any network/web handling, so a hung network task
cannot keep the compressor on longer than the safety condition allows.

Do not add any other `digitalWrite(PIN_COMPRESSOR, ...)` calls anywhere else
in the sketch -- all compressor control must go through this single gate.

## Switch position inputs

Four digital inputs (`PIN_SWITCH_1..4`) are read each loop by
`readSwitchInputs()` into `switch1State..switch4State`. These currently sit
on GPIO0, 1, 3, and 10 as placeholders -- reassign them once the final
pinout/wiring is decided, and confirm active level / debounce / pull-up vs
pull-down requirements. They are not yet wired into any control logic.

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

- `chiller.ino` - main sketch (setup/loop, safety gate, state machine, WiFi/OTA, web handlers, flash settings)
- `html.h` - all HTML markup (captive portal page + status/control page), included from the main sketch
- `Secrets.h.example` - copy to `Secrets.h` and fill in real WiFi credentials (not committed)

## TODO / left for calibration

- `COMPRESSOR_SAFE_ADC_MAX` placeholder value (500) needs to be calibrated against the real capacitor charge curve.
- Precharge timeout and fault-state handling in `runStateMachine()`.
- Active level / debounce for the timer input (GPIO4) and the 4 switch position inputs (GPIO0/1/3/10).
- Reassign switch position GPIOs if 0/1/3/10 conflict with strapping/USB/boot requirements on the final board.
- Live values (ADC, state, output status, switch states) in the status web page.
- Manual test controls (fan/buzzer/compressor) on the web UI.
