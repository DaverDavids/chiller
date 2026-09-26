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
| 7    | 12V enable output (PUMP ONLY)          | LOW |
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
| `CHARGING`       | HIGH    | LOW        | cap &ge; `CAP_CHARGED_ADC_MIN` |
| `RUNNING`        | HIGH    | HIGH       | demand removed |

Exactly one condition closes the contacts: the capacitor has to be proven
charged. Charging that does not reach `CAP_CHARGED_ADC_MIN` within
`PRECHARGE_TIMEOUT_MS` latches a fault instead of closing on an unproven
capacitor. The 12V current sense plays **no** part in this interlock - it is a
rail monitor belonging to the pump and the fan, see
[12V rail current sense](#12v-rail-current-sense) below. A latched fault forces both the charge and
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

### Strapping pins (GPIO2, GPIO8, GPIO9) - read this before first power-up

The ESP32-C3 has exactly three strapping pins: **GPIO2, GPIO8 and GPIO9**. (This
is *not* the original ESP32's set - **GPIO0 is not a strapping pin on the C3**,
so `PIN_CAP_SENSE` on GPIO0 is fine and needs no change. GPIO0 boot-strapping is
original-ESP32 behavior.)

The C3 datasheet boot table requires:

| Pin | Used as | Requirement at reset |
|-----|---------|----------------------|
| GPIO2 | `PIN_ADC_12V` (12V current sense) | must read **1** for *both* SPI boot and download boot |
| GPIO8 | `PIN_COMPRESSOR` | "don't care" for SPI boot, so driving it LOW at boot is safe |
| GPIO9 | `PIN_5V_EN` | **1** for SPI boot; **0** forces UART download mode |

Two things to check on the bench, with the circuits actually attached, on a
real cold power-cycle (brownout/reset is also a strapping sample point):

1. **GPIO2 is the one to worry about.** A 12V current-sense line typically
   idles LOW with no current flowing - no drop across the sense element. But
   GPIO2 must latch `1` at reset on the C3, so a sense output that rests LOW
   may prevent normal boot. Measure GPIO2's resting voltage during a cold boot.
   If it rests low, `PIN_ADC_12V` has to move to a non-strapping ADC1 pin
   (GPIO0, 1, 3 or 4 - whichever you can free up).
2. **Nothing external may pull GPIO9 low** before the sketch runs (no pull-down
   on that MOSFET/relay gate net), or the chip drops into UART download mode
   instead of booting. `PIN_5V_EN` defaulting HIGH is the correct direction.
   Note the SuperMini's BOOT button pulls GPIO9 to ground, and that board's
   onboard status LED sits on GPIO8 (active low), so the compressor output
   will mirror onto that LED.

### The 12V enable is the pump, not the rail

`PIN_12V_EN` (GPIO7) gates **the pump only**. The compressor is not fed through
it - the compressor runs off its own interlocked contacts on `PIN_COMPRESSOR`,
and the capacitor is charged straight from GPIO1 through a diode.

An earlier version of `update12VEnable()` ORed `compressorIsRunning` into the
enable so "the rail is never cut out from under a closed contact". That was
wrong twice over: it kept the pump powered whenever the chiller ran, and it
meant there was **no way to cut an overcurrenting pump without also killing a
running chiller**. The OR is gone, which is what makes the pump stall cut below
possible at all.

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


## Buzzer output

The buzzer is driven as a **pulsed square wave** at `buzzerFrequency` (via
`tone()`, LEDC-backed) and is never held at a static DC level. A piezo element
produces no sound from DC, and a magnetic buzzer will overheat if held on one.

`tone()`/`noTone()` are only called on an actual on/off transition, never once
per loop, so the pulsing costs a single boolean compare per iteration and does
not disturb the safety-critical path. The frequency is settable from the web UI
(100-8000 Hz, default 2000) and is persisted to flash. Changing it while a tone
is running re-issues the tone immediately rather than waiting for the next
on/off transition.

## Calibration inputs (volts + raw ADC)

The web UI has a **Calibration** section with an input per threshold, and the
status page shows the volts figure **and** the raw ADC count side by side for
both analog inputs. The point of showing both is that the ADC count is what the
interlock actually gates on, so you can sanity-check the conversion rather than
having to trust it.

| Input | Setting | JSON | Key |
| --- | --- | --- | --- |
| Capacitor discharged at/below | volts | `capDischargedVolts` | `capdis` |
| Capacitor charged at/above | volts | `capChargedVolts` | `capchg` |
| 12V pump stall ceiling | volts | `pumpStallLimitVolts` | `adcmax` |
| Buzzer frequency | Hz | `buzzerHz` | `buzzerhz` |

Endpoints: `/set/thresholds?vdischarged=&vcharged=`, `/set/current?volts=`,
`/set/buzzer?hz=`. Volts are converted once on the device, so the stored and
authoritative values remain raw ADC counts.

Each analog input has its own scaling constant, because the two sensing paths are
not the same circuit:

- `CAP_SENSE_DIVIDER` = `1.0` - GPIO0 sees capacitor voltage directly (1k
  series, no divider).
- `CURRENT_SENSE_DIVIDER` = `1.0` - **UNCALIBRATED PLACEHOLDER.** GPIO2 goes
  through a sense amplifier, and this is the factor that turns pin voltage back
  into real 12V rail current. Until you measure it against a known load, the
  12V volts readout is just the pin voltage. The UI labels it *uncalibrated*.

The C3's ADC is measurably nonlinear, especially near the rails, so treat volts
as **indicative** and calibrate against a multimeter. A volts error can only
mis-set a threshold, never make the interlock unsafe.

The hysteresis band on the capacitor pair is enforced on every write and on every
load. A new pair is rejected unless the charged threshold exceeds the discharged
one by at least `CAP_THRESHOLD_MIN_GAP` (200 counts, ~0.16V). This is not
defensive decoration: with the two equal or inverted, a *charged* capacitor would
satisfy the "discharged" test and the contacts could close on a charged
capacitor, defeating the interlock entirely. `loadSettings()` re-asserts the band
so a corrupted or hand-edited flash value cannot collapse it either.

The pump stall ceiling has no such guard because a ceiling of 0 counts would be
*less* conservative, not more, so it is merely floored at 1 count.

### Why the charge output is parked LOW while waiting to discharge

In `WAIT_DISCHARGE` the charge output is written LOW **unconditionally, before**
the re-arm test - not inside it. That ordering is the whole point.

A run can be requested while the capacitor is still charged, and that is the
normal case after any stop. If the charge output were left energised while
waiting, it would hold the capacitor at the charged voltage, the `cap <=
CAP_DISCHARGED_ADC_MAX` test would never be satisfied, and the compressor would
**never run again**. Parking it LOW is what lets the capacitor bleed down to the
discharged threshold so the start can arm. Requesting a run is not sufficient on
its own: the capacitor has to actually be seen discharged.

## Timers

Two live counters, both reported in `/status` and on the status page:

- **Compressor run time** - how long the contacts have been closed this run.
  Resets on every start. During a minimum-run or off-delay hold the compressor
  stays closed after demand is removed, so this counter keeps running through
  both holds and only stops when the contacts actually open. A **stop pending**
  countdown shows how much off-delay is left, so the hold is visible rather than
  looking like the switch was ignored.
- **Capacitor discharge time** - how long the capacitor has been sitting below
  `CAP_CHARGED_ADC_MIN`, i.e. how long it has been bleeding down since it last
  read charged. Restarts each time the capacitor rises back above that
  threshold, and the last completed measurement is retained. This is a health
  indicator for the discharge path: a capacitor that never bleeds down points at
  a failed bleeder, and would block the next start.

The discharge timer is currently a **measurement only** - it is not compared
against a limit and does not raise a fault. Wiring a `CAP_MAX_DISCHARGE_MS`
limit onto it would catch a failed bleeder automatically; that was left out
deliberately rather than picking an uncalibrated trip point.

## Compressor minimum run time and off-delay

Two separate holds, and the distinction matters:

- `COMPRESSOR_MIN_RUN_MS` (placeholder: 5s) - once the contacts close they stay
  closed for at least this long.
- `COMPRESSOR_OFF_DELAY_MS` (placeholder: 5s) - after that minimum is satisfied,
  removing demand does **not** open the contacts straight away. They stay closed
  for a further `COMPRESSOR_OFF_DELAY_MS`.

The contacts open only when **both** have elapsed.

**Why they have to be two constants.** A minimum run time on its own cannot give
you the seamless resume. Once a compressor has been running longer than the
minimum, removing demand satisfies "minimum elapsed" on the very first loop, so
the contacts open immediately. Flipping the switch back to chiller then starts
from `WAIT_DISCHARGE` and has to bleed the capacitor down and re-charge it - a
full contactor cycle, which is exactly what you are trying to avoid.

With the off-delay, the sequence runs like this:

1. Compressor has been running longer than the minimum.
2. Switch to OFF. `compressorStopRequestedAt` is stamped, contacts stay closed,
   capacitor stays charged. The UI shows a "stop pending" countdown.
3. Switch back to chiller inside the window. `compressorStopRequestedAt` is
   cleared to 0, the state is still `RUNNING`, and the compressor just carries
   on - no stop, no discharge, no re-charge, no contactor cycle. The program
   recognises it was still running.

If demand does *not* return, the contacts open when the off-delay expires and the
next start re-arms from a discharged capacitor as normal.

There is no safety trip that bypasses these holds any more, because the only
safety input the compressor has is its own capacitor interlock. A latched
`compressorFault` still overrides everything and forces both outputs off.

## Pump anti-short-cycle delay

The 12V enable output is not switched off the instant pump demand disappears.
`update12VEnable()` holds it on for `PUMP_OFF_DELAY_MS` (1 second) after demand
drops, so flipping through switch positions doesn't stop and immediately restart
the pump. Adjust `PUMP_OFF_DELAY_MS` to taste.

This hold is cancelled, not merely paused, while a pump stall is cutting the
pump - see below. Honouring it there would hold an overcurrenting pump powered
for another second after the fault, which is exactly the delay the stall cut
exists to avoid.

## 12V rail current sense

`PIN_ADC_12V` (GPIO2) is a **rail monitor, not a compressor sensor**. It reads
everything on the 12V rail, which in practice means the fan (whenever the
compressor is running) plus the pump (whenever the 12V enable is on). That is
why it is read against a floor and a ceiling rather than as one limit, and why
neither limit belongs to the compressor.

| Limit | Value | Meaning |
|-------|-------|---------|
| `PUMP_STALL_ADC_MAX` | 500 (placeholder) | Ceiling. Above it the pump is stalled or jammed. |
| `FAN_RUN_ADC_MIN` | 0 | Floor, checked only while the compressor runs. `0` = disabled. |

### Pump stall: cut instantly, retry after 5 seconds

An overcurrent **cuts the 12V enable on that very loop** - no delay, no
anti-short-cycle hold, nothing in the way. The pump then stays cut until the
rail has been clean for `PUMP_STALL_RETRY_MS` (5 seconds).

The hold is measured from the **last** overcurrent sample rather than the first,
and the clock is restarted on every overcurrent sample. So it can only elapse
once the rail has been clean for the entire window, and a pump that trips the
instant it is re-energised never gets a second attempt.

The hold is deliberately **not** reset by pump demand dropping. A stall is a real
fault, so putting the switch to OFF and back again does not buy a free restart -
it just burns wall-clock time while the pump stays cut.

Because the enable pin is pump-only, **a pump stall leaves a running chiller
completely unaffected**. That is the whole point of separating the two.

### Fan floor: present but inert

A seized fan on a running compressor will overheat it, so the floor is sampled
every loop while the contacts are closed. But `FAN_RUN_ADC_MIN` is `0`, and no
reading can fall below `0`, so **the check cannot fire yet** - the sense
amplifier's offset and the fan's running current have not been measured, and
guessing a floor would trip on noise.

It is report-only even once calibrated: it sets `fanStalled` for the web UI and
does **not** open the compressor contacts, because a spurious trip on an
uncalibrated floor would stop a healthy chiller.

Note the two limits overlap by design: while the compressor runs, the reading
includes the fan's baseline, so the ceiling has to sit above that baseline or
the pump would be cut for the fan's sake.

## Mode change delay

A newly selected switch position is **not** acted on when it is read. The mode
that was already in effect keeps running for `MODE_SWITCH_DELAY_MS` (3 s) first,
and only then does the new mode take effect. `updateModeSelection()` owns this
and is the only writer of the mode state.

This gates the hardware, not just the LEDs. `pumpRequested` and
`compressorRequested` are derived from the mode *in effect*, so the pump and the
compressor both wait the window out. A switch nudged into a new position and
back again can never reach either output, and the delay cannot be skipped by
re-reading the switch: during the hold the reading is already the new mode, it
just isn't in effect yet.

The web UI shows both the switch position and the mode in effect while the hold
is running, with a countdown, so the delay reads as a deliberate wait rather
than the switch having been ignored.

## Status LEDs

`LED_COUNT` WS2812Bs joined end to end into a **ring**, with one animation per
mode:

| Mode | Animation |
|---|---|
| Off (position 1) | black |
| Pump only | a red chase stepping slowly around the ring, with a short tail fading out behind the head |
| Chiller only | blueish shades twinkling back and forth around the ring |
| Both | the twinkle and the chase at the same time, mixed additively so the overlap goes magenta |

Both patterns are written to close on themselves, because the strip is a circle:
the twinkle carries a whole number of crests around the loop, and the chase
measures distance backwards so its tail wraps behind its head rather than
running off the end.

Every knob that decides how the ring looks - crest count, sweep time, chase
step, brightnesses, tail length and the dim/bright colour of each pattern - is
in a single tunables block next to `LED_COUNT` in `chiller.ino`, so the pattern
can be dialled in without reading the renderer.

### The wipe on a mode change

A mode change is not instantaneous visually either. The ring spends exactly the
same `MODE_SWITCH_DELAY_MS` window switching over from LED 0 upwards, showing
the animation that is *about to* take effect. The last LED switches on the same
tick the mode becomes current, so the animation is never caught half-revealed.

LEDs the wipe has not reached yet still show the mode being **replaced**, not
black. One rule then covers both directions: filling up from off reveals the new
animation LED by LED, and going to off - whose animation is black - extinguishes
the old animation LED by LED instead of blanking the whole ring in one step.

### State is no longer shown on the ring

The ring now shows *mode*, not machine *state*. The old scheme signalled the
fault latch (flashing red) and precharge (yellow) with strip-wide colours; both
were dropped, because the mode animations now own the ring and a chiller-only
twinkle is a better read of "the chiller is selected" than a fixed colour was.
**A latched fault is therefore no longer visible on the LEDs** - it is still
reported in the web UI, on `/status`, and in the serial log. If you want the
fault back on the ring, the clean way is to overlay it in `updateStatusLeds()`
rather than fold it into the mode patterns, so the wipe is not disturbed.

`updateStatusLeds()` only redraws on `LED_FRAME_INTERVAL_MS` (30 ms) or when the
wipe gains an LED, because `statusLeds.show()` is the one blocking call on an
otherwise non-blocking loop and there is no point pushing an identical frame to
the strip. `updateStatusLeds()` reads the mode state only; it drives no output
pin and cannot reach the pump or the compressor.

## Web UI

Connect to `http://chiller.local/` (or the device IP) for a live status page
that polls `/status` (JSON) every second.

**Switch dial.** A dial graphic at the top mirrors the physical 4-position
switch: **up = 1 (off), right = 2 (pump), down = 3 (chiller), left = 4 (both)**.
The needle animates to the debounced active position, the active position number
is highlighted, and the mode is shown beside it. Position 1 has no input of its
own (`PIN_SWITCH_1` is unwired) and is selected implicitly whenever none of
positions 2/3/4 read active. If the firmware reads more than one wired position
active it is in the failsafe-OFF state, so the needle greys out and the panel
reads `none / failsafe OFF` - the dial will show that rather than a false
position.

**Mode colors** match the status LED ring colors exactly, so the two always
agree: OFF = grey, pump only = red, chiller only = blue, both = magenta. The
color is applied to the Mode row in the table and to the mode readout beside the
dial. While a mode change is being held, the dial also shows the switch position
and a countdown to when it takes effect, and the table gains a `Pending mode`
row with the reveal progress.

Every row also shows the raw GPIO number and pin level next to its label (e.g.
`Compressor  8=LOW`), read straight from `digitalRead()` so what you see is the
actual pin, not just the derived flag. Rows cover: mode, state, capacitor
sequence phase and arm state, capacitor reading in volts and ADC against both
thresholds, charge output, live discharge timer, 12V current reading against its
limit, latched fault, fan/buzzer/12V-enable/compressor outputs, compressor run
timer, and all 4 switch positions.

**Settings** (both persisted to flash):

- Capacitor thresholds, entered in volts, shown next to the raw ADC count.
- 12V pump stall ceiling, entered in volts, shown next to the raw
  ADC count. Marked *uncalibrated* until `CURRENT_SENSE_DIVIDER` is measured.
- Buzzer test frequency in Hz.

Form fields are not overwritten by the 1s poll while you have them focused, so
a refresh can't clobber what you're typing. A rejected threshold change reports
the reason from the device instead of silently clamping.

Manual test controls:

- Fan on/off/auto
- Buzzer on/off/auto - pulsed, at the configured frequency
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
- **Retries keep running while the captive portal is up.** The AP is brought up in
  `WIFI_AP_STA` mode, so the station half stays active and keeps retrying the saved
  network every `WIFI_RETRY_INTERVAL_MS`. When the network comes back, the device
  tears the AP down, swaps the portal's catch-all route for the real status routes,
  and carries on. The serial log shows `[WIFI] connecting` every interval while it
  is retrying, and `[WIFI] connected` with the IP when it lands.
- **Connection is fully non-blocking.** `WiFi.begin()` is issued and then polled
  from `loop()`; nothing waits for `WL_CONNECTED` in a loop. The previous
  implementation blocked for up to `WIFI_CONNECT_TIMEOUT_MS` (10s) per attempt -
  and `loop()` is what runs the compressor interlock, so every retry would have
  delayed the compressor interlock by up to 10s with the contacts held closed.
  There is no `delay()` and no `ESP.restart()` anywhere in the runtime path.
- Saving new credentials in the portal no longer reboots the device. It stores them
  and immediately attempts a connection, so a running chiller is not interrupted
  just to change WiFi settings.

## Files

- `chiller.ino` - main sketch (setup/loop, capacitor interlock, fault latch,
  mode logic, status LEDs, WiFi/OTA, web handlers, flash settings)
- `html.h` - all HTML/JS markup (captive portal page + live status/test page)
- `Secrets.h.example` - copy to `Secrets.h` and fill in real WiFi credentials (not committed)

## TODO / left for calibration

- `CAP_DISCHARGED_ADC_MAX` (0.40V) and `CAP_CHARGED_ADC_MIN` (2.42V) defaults
  need calibration against the real capacitor charge curve. Keep a wide gap
  between them - that gap is the hysteresis band, and a narrow one risks chatter
  around a threshold in the noise.
- `PUMP_STALL_ADC_MAX` placeholder (500) needs calibration against the real
  12V current-sense curve, **and** `CURRENT_SENSE_DIVIDER` needs a measured
  ratio so the volts readout means anything. Measure the GPIO2 pin voltage at a
  known 12V load current, set the constant, then set the ceiling in volts from
  the UI. Measure the *combined* fan+pump rail, not the pump alone.
- `FAN_RUN_ADC_MIN` is 0 and therefore inert. It needs a real fan-only running
  current before it can detect a seized fan at all.
- The pump stall cut is currently **not** a latched fault: a persistent
  overcurrent will keep cutting and retrying every 5 seconds forever. Decide
  whether a sustained stall should latch and need a manual reset, as
  `compressorFault` does.
- Verify the pulsed buzzer actually sounds on hardware - the `tone()`/LEDC path
  should be confirmed once on the real board.
- `PRECHARGE_TIMEOUT_MS`, `COMPRESSOR_MIN_RUN_MS`, `COMPRESSOR_OFF_DELAY_MS` and
  `FAULT_RESET_HOLD_MS` are guesses.
- **Verify the strapping pins on a cold boot** - especially GPIO2, which must
  latch HIGH. See the strapping section above.
- `PIN_SWITCH_1` is unwired, so OFF has no input of its own. It is now selected
  *implicitly* whenever none of positions 2/3/4 read active, so an unwired
  position 1 works as a deliberate "off" position. Assign a real pin if you ever
  want to distinguish "parked at 1" from a wiring fault.
- Debounce is implemented per switch input at `SWITCH_DEBOUNCE_MS` (500 ms) -
  a raw reading must be stable that long before it is acted on. Tune to taste;
  too long feels laggy, too short defeats the purpose. The timer input is
  **not** debounced yet.
- `PIN_TIMER_IN` now has an explicit `INPUT_PULLDOWN` so it can never float
  into `compressorRequested`, but the active level is still unconfirmed. With the
  pull-down an unwired timer reads inactive, which blocks chiller demand - so if
  that timer is not actually wired, the compressor will never run.
- Confirm whether the timer input should gate/AND with the switch mode for
  compressor demand, or be removed/repurposed now that the switch exists.
- `LED_COUNT` is set to 10 for the ring, but the physical LED count has not been
  confirmed against the hardware - check it and adjust the one `#define`.
- The ring no longer shows a latched fault. If that is wanted back, overlay it in
  `updateStatusLeds()` rather than folding it into the mode patterns.
- The LED animation tunables (crest count, sweep, chase step, brightnesses, tail,
  colours) are untested on the bench - they were chosen to be gentle and need a
  real eye on the ring.
- `MODE_SWITCH_DELAY_MS` (3 s) is a judgement call. It is long enough that a
  switch bounce cannot reach the outputs, but it does mean a deliberate mode
  change has a visible dead time before anything happens.
- Fan/buzzer have no automatic behaviour at all yet - they are manual-test
  outputs only. They need real demand logic.
- Optionally add a `CAP_MAX_DISCHARGE_MS` limit so a capacitor that never bleeds
  down (failed bleeder) latches a fault instead of only being reported.
- The dial needle is repositioned via the SVG `transform` attribute, so it snaps
  rather than animating between positions. Cosmetic only. The selected position is
  marked by an arrow plus an enlarged number.
- Live status page could add a timer input row and mode override.
