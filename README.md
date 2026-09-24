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

## Safety design

The compressor output (GPIO8) is written in exactly one function,
`setCompressorOutput()`, which re-reads and re-checks the ADC (GPIO2) on
*every* call before allowing the pin HIGH. The compressor is only permitted
on when `adcValue <= COMPRESSOR_SAFE_ADC_MAX`. This function is called every
`loop()` iteration ahead of any network/web handling, so a hung network task
cannot keep the compressor on longer than the safety condition allows.

Do not add any other `digitalWrite(PIN_COMPRESSOR, ...)` calls anywhere else
in the sketch -- all compressor control must go through this single gate.

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
- Active level / debounce for the timer input (GPIO4).
- Live values (ADC, state, output status) in the status web page.
- Manual test controls (fan/buzzer/compressor) on the web UI.
