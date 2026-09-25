# chiller

ESP32-C3 Super Mini firmware (Arduino IDE) for an HVAC chiller controller:
compressor + pump control with a hysteretic capacitor charge interlock, a
4-position mode switch, addressable status LEDs, WiFi with captive-portal
fallback, OTA updates, mDNS, flash-persisted settings, and a web UI for live
status and manual function testing.

## Pin map

| GPIO | Function                              | Boot default |
|------|----------------------------------------|---------------|
| 0    | ADC - capacitor voltage sense (input)  | n/a |
| 1    | Capacitor charge output                | LOW |
| 2    | ADC - 12V current sense (input)        | n/a |
| 3    | Switch position 3 - chiller only, active low | n/a |
| 4    | Timer on/off input                    | n/a |
| 5    | Fan output                            | LOW |
| 6    | Buzzer output                         | LOW |
| 7    | 12V enable output (powers the pump)    | LOW |
| 8    | Compressor output                     | LOW |
| 9    | 5V enable output                      | HIGH |
| 10   | Switch position 4 - pump + chiller, active low | n/a |
| 20   | Switch position 2 - pump only, active low | n/a |
| 21   | Addressable status LED data           | n/a |
| n/a  | Switch position 1 - OFF, **not wired** | n/a |

`PIN_SWITCH_1` is set to `-1`. Any negative pin reads permanently inactive and
is skipped in `pinMode()`, so position 1 can never be selected until a real pin
is assigned. `MODE_OFF` is still reachable through the fail-safe path in
`getSwitchMode()` (zero or more than one position reading active).

GPIO21 was previously the hardware UART0 TX pin on most supermini boards, so
Serial debug has to run over the native USB port (GPIO18/19) with the LED data
line on 21.

## Capacitor interlock (the compressor start path)

The capacitor is charged by holding the charge output (GPIO1) HIGH, and that
output is only ever energized while the compressor is running. Once the
compressor is off the capacitor bleeds back down, and the compressor output
may not go HIGH again until the capacitor has been seen at or below
`CAP_DISCHARGED_ADC_MAX`.

The sequence is hysteretic on two thresholds of the capacitor sense (GPIO0), so
a start can never land on a charged capacitor and a restart can never skip the
discharge:

| Phase          | Charge out | Compressor | Exits when |
|----------------|-----------|------------|------------|
| `WAIT_DISCHARGE` | LOW     | LOW        | cap &le; `CAP_DISCHARGED_ADC_MAX` **and** compressor requested |
| `CHARGING`       | HIGH    | LOW        | cap &ge; `CAP_CHARGED_ADC_MIN` **and** 12V current sense clear |
| `RUNNING`        | HIGH    | HIGH       | demand removed or 12V current sense shows overcurrent |

Two independent conditions must hold to close the contacts: the capacitor has
to be proven charged, and the 12V current sense (GPIO2) has to be at or below
`COMPRESSOR_SAFE_ADC_MAX`. Charging that does not reach
`CAP_CHARGED_ADC_MIN` within `PRECHARGE_TIMEOUT_MS` latches a fault instead of
closing on an unproven capacitor. A latched fault forces both the charge and
compressor outputs off and only clears after the chiller demand has been
removed for `FAULT_RESET_HOLD_MS`, so bouncing the switch cannot immediately
re-charge a bad capacitor.

`updateCompressorInterlock()` is the **only** function that writes the charge
output or the compressor output. There is no flag, endpoint, or state that
reaches the compressor without passing through it. `runStateMachine()` only
latches/clears the fault and projects the phase into the `STATE_*` value used
for the LEDs, web UI and debug output.

Do not add any other `digitalWrite()` calls on GPIO1 or GPIO8 anywhere else in
the sketch - all control must go through `updateCompressorInterlock()`. The
12V enable output (GPIO7) is written in exactly one place, `update12VEnable()`.

### GPIO0 warning

GPIO0 is an ESP32-C3 **strapping** pin: held LOW at reset it forces the chip
into the ROM downloader, and it skips normal boot entirely. A capacitor-sense
divider idles LOW whenever the capacitor is discharged, which is the normal
resting state - so as wired this would drop the board into the bootloader on
most power-ups. Confirm the sense divider's idle level, or move
`PIN_CAP_SENSE` to a non-strapping ADC1 pin (GPIO1, 3 or 4, if you can free
one up).

## Mode switch behavior

The 4-position switch inputs are **active low**: the switch shorts the pin
to GND to close a position, and the ESP32's internal pull-up holds it HIGH
when that position is open. All wired switch pins are configured as
`INPUT_PULLUP`. So "active" in the code and in `/status` means *low*.

A 4-position switch selects one of:

- Position 1 (not wired): everything off.
- Position 2: pump only (12V enable on), compressor never requested.
- Position 3: chiller only (compressor path).
- Position 4: pump and chiller both requested simultaneously.

If the switch reads zero or more than one position active at once
(wiring fault, transition glitch), the firmware fails safe to OFF. With
position 1 unwired, the only way to reach OFF is that fail-safe path.


## Pump anti-short-cycle delay

The 12V enable output is not switched off the instant pump demand disappears.
`update12VEnable()` holds it on for `PUMP_OFF_DELAY_MS` (placeholder: 5
seconds) after demand drops, so flipping through switch positions doesn't stop
and immediately restart the pump. It also stays on whenever the compressor is
running, so the 12V rail is never cut out from under a closed contact. Adjust
`PUMP_OFF_DELAY_MS` to taste.

## Status LEDs

An addressable LED strip (Adafruit_NeoPixel) shows current status:

- Off: idle, nothing running
- Blue: pump running (pump-only mode, or in the off-delay hold window)
- Yellow: charging the capacitor, contacts still open
- Orange: chiller running, pump not otherwise requested
- Purple: chiller and pump both running
- Flashing red: fault latched

This is a strip-wide single-color scheme for now - see the TODO in
`updateStatusLeds()` for per-LED per-function assignment.

## Web UI

Connect to `http://chiller.local/` (or the device IP) for a live status
page that polls `/status` (JSON) every second. Every row shows the raw GPIO
number and pin level next to its label (e.g. `Compressor  8=LOW`), read
straight from `digitalRead()` so what you see is the actual pin, not just the
derived flag. Rows cover: mode, state, capacitor sequence phase and arm state,
capacitor reading against both thresholds, charge output, 12V current reading
against its limit, latched fault, fan/buzzer/12V-enable/compressor outputs,
and all 4 switch positions.

Manual test controls are provided for:

- Fan on/off/auto
- Buzzer on/off/auto
- Compressor demand on/off/auto - this only sets the *demand* flag. It still
  passes through the full capacitor interlock and the current-sense gate every
  loop, so it can be used to verify the interlock itself without ever bypassing
  it. Watch the `Cap sequence` row to step through `WAIT_DISCHARGE` ->
  `CHARGING` -> `RUNNING`.

## Debug output

Serial debug is now throttled (`DEBUG_PRINT_INTERVAL_MS`, default 2000 ms)
instead of printing every loop iteration, so it no longer spams the
console. Toggle all serial debugging on/off via `#define DEBUG 1` / `0` at
the top of `chiller.ino`. The interlock logs phase changes, overcurrent drops
and precharge timeouts.

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

- `chiller.ino` - main sketch (setup/loop, capacitor interlock, fault latch,
  mode logic, status LEDs, WiFi/OTA, web handlers, flash settings)
- `html.h` - all HTML/JS markup (captive portal page + live status/test page)
- `Secrets.h.example` - copy to `Secrets.h` and fill in real WiFi credentials (not committed)

## TODO / left for calibration

- `CAP_DISCHARGED_ADC_MAX` (500) and `CAP_CHARGED_ADC_MIN` (3000) are
  placeholders and must be calibrated against the real capacitor charge curve.
  Keep a wide gap between them - that gap is the hysteresis band, and a narrow
  one risks chatter around a threshold in the noise.
- `COMPRESSOR_SAFE_ADC_MAX` placeholder (500) needs calibration against the
  real 12V current-sense curve.
- `PRECHARGE_TIMEOUT_MS` and `FAULT_RESET_HOLD_MS` are guesses.
- **Resolve the GPIO0 strapping conflict** - see the warning above.
- `PIN_SWITCH_1` is unwired; assign a real pin if position 1 (OFF) is needed as
  a deliberate selection rather than only via the fail-safe path.
- Debounce on switch position change (and on the timer input).
- Active level / debounce for the timer input.
- Confirm whether the timer input should gate/AND with the switch mode for
  compressor demand, or be removed/repurposed now that the switch exists.
- `LED_COUNT` is a placeholder - set it to the actual strip length.
- Per-LED status mapping instead of single strip-wide color.
- Fan/buzzer have no automatic behaviour at all yet - they are manual-test
  outputs only. They need real demand logic.
- Live status page could add a timer input row and mode override.
