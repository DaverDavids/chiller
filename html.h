#pragma once
#include <Arduino.h>

// Captive-portal WiFi configuration page
const char PAGE_CONFIG[] PROGMEM = R"====(
<!DOCTYPE html>
<html>
<head><title>Chiller Setup</title></head>
<body>
<h2>Chiller WiFi Setup</h2>
<form action="/save" method="POST">
  SSID: <input type="text" name="ssid"><br>
  Password: <input type="password" name="pass"><br>
  <input type="submit" value="Save &amp; Reboot">
</form>
</body>
</html>
)====";

// Normal-operation status/control page.
// Pulls live values from /status (JSON) every second and lets you drive
// the non-safety-critical outputs directly for testing. The compressor
// test toggle only sets the DEMAND flag - it still has to pass the ADC
// safety check every loop, so you can use it to verify the interlock.
const char PAGE_STATUS[] PROGMEM = R"====(
<!DOCTYPE html>
<html>
<head>
<title>Chiller Control</title>
<style>
  body { font-family: sans-serif; max-width: 480px; margin: 20px auto; }
  table { width: 100%; border-collapse: collapse; margin-bottom: 16px; }
  td { padding: 4px 6px; border-bottom: 1px solid #ddd; }
  .ok { color: green; font-weight: bold; }
  .bad { color: #b00; font-weight: bold; }
  .gpio { color: #777; font-family: monospace; font-size: 11px; white-space: nowrap; }
  button { margin: 2px; padding: 6px 10px; }
  h3 { margin-top: 24px; }
  input[type=number] { width: 70px; padding: 4px; }
  .card { border: 1px solid #ddd; border-radius: 6px; padding: 10px; margin-bottom: 16px; }
  .dialwrap { display: flex; align-items: center; gap: 14px; }
  .dialwrap svg { flex: 0 0 auto; }
  .dialInfo { font-size: 13px; line-height: 1.5; }
  .posLabel { font-size: 11px; fill: #999; font-family: monospace; text-anchor: middle; }
  .posLabel.active { font-size: 20px; font-weight: bold; fill: #000; }
  .needle { transform-origin: 70px 70px; }
  .dialcard.off { border-color: #d33; }
  .modeOFF { color: #666; }
  .modePUMP_ONLY { color: #c0392b; font-weight: bold; }
  .modeCHILLER_ONLY { color: #1f6fd0; font-weight: bold; }
  .modeBOTH { color: #a335b0; font-weight: bold; }
  #setMsg { font-size: 12px; margin-left: 6px; }
</style>
</head>
<body>
<h2>Chiller Status</h2>

<div class="card" id="dialCard">
  <div class="dialwrap">
    <svg width="150" height="150" viewBox="0 0 140 140" id="dial">
      <circle cx="70" cy="70" r="62" fill="#fafafa" stroke="#ccc"></circle>
      <circle cx="70" cy="70" r="46" fill="none" stroke="#eee"></circle>
      <polygon id="dialArrow" points="70,20 62,34 78,34" fill="#666" opacity="0"></polygon>
      <g id="needle" class="needle">
        <polygon points="70,30 63,52 77,52" fill="#333"></polygon>
      </g>
      <circle cx="70" cy="70" r="7" fill="#333"></circle>
      <text class="posLabel" id="pos1" x="70"  y="18" text-anchor="middle">1</text>
      <text class="posLabel" id="pos2" x="128" y="75" text-anchor="middle">2</text>
      <text class="posLabel" id="pos3" x="70"  y="134" text-anchor="middle">3</text>
      <text class="posLabel" id="pos4" x="12"  y="75" text-anchor="middle">4</text>
    </svg>
    <div class="dialInfo">
      <div>Selected: <b id="dialPos">-</b></div>
      <div>Mode: <b id="dialMode">-</b></div>
      <div id="dialPending" style="color:#888;font-size:11px"></div>
      <div style="color:#888;font-size:11px">up=1 off, right=2 pump,<br>down=3 chiller, left=4 both</div>
    </div>
  </div>
</div>

<table id="statusTable"></table>

<h3>Calibration</h3>

<div id="calWarn"></div>

<div class="card">
  <div style="font-weight:bold;margin-bottom:4px">Live readings</div>
  <div style="color:#888;font-size:11px;margin-bottom:6px">
    The volts figures come from a placeholder divider, so treat the
    <b>raw counts</b> as the number to write down. Record the counts in each state
    below, then fit CURRENT_SENSE_DIVIDER to them in the sketch.
  </div>
  <div id="calLive"></div>
</div>

<div class="card">
  <div style="font-weight:bold;margin-bottom:4px">Capacitor sense (GPIO0)</div>
  <div>
    Discharged at or below
    <input type="number" id="vdis" step="0.01" min="0" max="3.3"> V
    <span class="gpio" id="acdis"></span>
  </div>
  <div style="margin-top:6px">
    Charged at or above
    <input type="number" id="vchg" step="0.01" min="0" max="3.3"> V
    <span class="gpio" id="acchg"></span>
  </div>
  <div style="margin-top:8px">
    <button onclick="saveThresholds()">Save capacitor</button>
    <span id="setMsg"></span>
  </div>
  <div style="color:#888;font-size:11px;margin-top:8px">
    The gap between the two is the hysteresis band - the firmware refuses a
    pair closer than the minimum, because the contacts must never be able to
    close on a charged capacitor.
  </div>
</div>

<div class="card">
  <div style="font-weight:bold;margin:0 0 4px">12V current sense (GPIO2)</div>
  <div style="color:#888;font-size:11px;margin-bottom:6px">
    Rail monitor: fan + pump. Ceiling = pump stall. Floor = fan turning.
    Set either to <b>0</b> to disable that check while you measure - a guessed
    threshold otherwise cuts the pump every few seconds and ruins the reading.
    The pump ceiling is <b>never saved</b> at 0, so a reboot always re-arms it.
  </div>
  <div>
    Pump stall ceiling
    <input type="number" id="vcur" step="0.01" min="0" max="3.3"> V
    <span class="gpio" id="acccur"></span>
  </div>
  <div style="margin-top:6px">
    Fan running floor
    <input type="number" id="vfan" step="0.01" min="0" max="3.3"> V
    <span class="gpio" id="accfan"></span>
  </div>
  <div style="margin-top:8px">
    <button onclick="saveCurrentLimit()">Save ceiling</button>
    <button onclick="saveFanFloor()">Save floor</button>
    <button onclick="disableCurrentChecks()">Disable both to calibrate</button>
    <span id="cmsg"></span>
  </div>

  <div style="margin-top:12px;border-top:1px solid #333;padding-top:8px">
    <div style="font-weight:bold;margin-bottom:4px">Peak hold since reset</div>
    <div id="calPeak"></div>
    <div style="margin-top:6px">
      <button onclick="resetCapture()">Reset capture</button>
    </div>
    <div style="color:#888;font-size:11px;margin-top:6px">
      A stall is a fast transient - the averaged sample may only cross the
      threshold for a few loops. This holds the highest and lowest counts seen
      since the last reset, so a pump start spike can be read back afterwards.
    </div>
  </div>
</div>

<h3>Fan test</h3>
<button onclick="testCall('/test/fan?state=on')">On</button>
<button onclick="testCall('/test/fan?state=off')">Off</button>
<button onclick="testCall('/test/fan?state=auto')">Auto</button>
<div style="color:#888;font-size:11px;margin-top:6px">
  Auto = follow the compressor. The fan runs automatically whenever the
  compressor contacts are closed, so this only needs On to test it with the
  compressor stopped. <b>Off cannot switch the fan off while the compressor is
  running</b> - the compressor wins, because a compressor with no condenser
  airflow will overheat.
</div>

<h3>Buzzer test</h3>
<div>
  Tone
  <input type="number" id="bhz" step="10" min="100" max="8000"> Hz
  <button onclick="saveBuzzer()">Set</button>
  <span id="bmsg"></span>
</div>
<div style="margin-top:6px">
  <button onclick="testCall('/test/buzzer?state=on')">On</button>
  <button onclick="testCall('/test/buzzer?state=off')">Off</button>
  <button onclick="testCall('/test/buzzer?state=auto')">Auto</button>
</div>
<div style="color:#888;font-size:11px;margin-top:6px">
  Driven as a pulsed square wave, never held at DC. The frequency is saved to
  flash and applies immediately, even while the tone is running.
</div>

<h3>Compressor demand test (still safety-gated by the capacitor interlock)</h3>
<button onclick="testCall('/test/compressor?state=on')">Request On</button>
<button onclick="testCall('/test/compressor?state=off')">Request Off</button>
<button onclick="testCall('/test/compressor?state=auto')">Auto (switch-controlled)</button>

<script>
function testCall(url) {
  fetch(url).then(refresh);
}

function row(label, value, pin) {
  return '<tr><td>' + label + (pin ? ' <span class="gpio">' + pin + '</span>' : '') + '</td><td>' + value + '</td></tr>';
}

function boolCell(v) {
  return v ? '<span class="ok">ON</span>' : '<span>off</span>';
}

function fmtMs(ms) {
  if (!ms || ms < 0) return '0.0s';
  var s = ms / 1000;
  if (s < 60) return s.toFixed(1) + 's';
  var m = Math.floor(s / 60);
  s = s - (m * 60);
  if (m < 60) return m + 'm ' + s.toFixed(0) + 's';
  var h = Math.floor(m / 60);
  return h + 'h ' + (m - (h * 60)) + 'm';
}

// Mode colors match the status LED ring colors so the two always agree:
// OFF=grey, pump only=red, chiller only=blue, both=magenta (red chase + blue
// twinkle lit at the same time).
var MODE_COLOR = {
  'OFF':           '#666666',
  'PUMP_ONLY':     '#c0392b',
  'CHILLER_ONLY':  '#1f6fd0',
  'BOTH':          '#a335b0'
};

function modeSpan(mode) {
  return '<span style="color:' + (MODE_COLOR[mode] || '#000') + ';font-weight:bold">' + mode + '</span>';
}

// Switch dial: position 1 is up, 2 right, 3 down, 4 left. The needle angle is
// measured from straight up, so 0/90/180/270 degrees respectively.
var DIAL_ANGLE = { 1: 0, 2: 90, 3: 180, 4: 270 };

// Arrow marker parked just inside the dial edge at each position, pointing
// outward at the number. One element is moved rather than four being toggled.
var DIAL_ARROW = {
  1: '70,36 62,50 78,50',
  2: '104,70 90,62 90,78',
  3: '70,104 62,90 78,90',
  4: '36,70 50,62 50,78'
};

function activePosition(d) {
  // Mirrors getSwitchMode(): position 1 has no input of its own, so it is
  // selected implicitly whenever none of the wired positions 2/3/4 are active.
  // More than one wired position active is a fault, and the firmware fails safe
  // to OFF - report no position there rather than a definite one the firmware
  // disagrees with, which is the last thing you want on a control indicator.
  var wired = (d.switch2 ? 1 : 0) + (d.switch3 ? 1 : 0) + (d.switch4 ? 1 : 0);
  if (wired === 0) return 1; // position 1 (off), selected by the absence of 2/3/4
  if (wired > 1) return 0;   // ambiguous -> failsafe OFF
  if (d.switch2) return 2;
  if (d.switch3) return 3;
  return 4;
}

function updateDial(d) {
  var pos = activePosition(d);
  var needle = document.getElementById('needle');
  var arrow = document.getElementById('dialArrow');
  var card = document.getElementById('dialCard');
  var color = MODE_COLOR[d.mode] || '#666';

  if (pos === 0) {
    // No settled position: park the pointer up (position 1) and grey it out.
    // Position 1 can't actually be selected while it is unwired, so don't
    // highlight it or show an arrow - this is a fault state, not a selection.
    needle.setAttribute('transform', 'rotate(0 70 70)');
    needle.setAttribute('opacity', '0.25');
    arrow.setAttribute('opacity', '0');
    card.className = 'card dialcard off';
  } else {
    needle.setAttribute('transform', 'rotate(' + DIAL_ANGLE[pos] + ' 70 70)');
    needle.setAttribute('opacity', '1');
    arrow.setAttribute('points', DIAL_ARROW[pos]);
    arrow.setAttribute('fill', color);
    arrow.setAttribute('opacity', '1');
    card.className = 'card';
  }

  for (var i = 1; i <= 4; i++) {
    document.getElementById('pos' + i).setAttribute(
      'class', 'posLabel' + (i === pos ? ' active' : ''));
  }

  document.getElementById('dialPos').textContent =
    pos === 0 ? 'none / failsafe OFF' : pos;
  document.getElementById('dialMode').innerHTML = modeSpan(d.mode);

  // While a mode change is being held the switch and the mode in effect
  // legitimately disagree, so say so with a countdown rather than let the dial
  // look like the switch was ignored.
  var pend = document.getElementById('dialPending');
  if (d.modeChangeRemainingMs > 0) {
    pend.innerHTML = 'switch is on ' + modeSpan(d.selectedMode) +
      ' &middot; takes effect in ' + fmtMs(d.modeChangeRemainingMs);
  } else {
    pend.textContent = '';
  }
}

// Only push form values from the device when the user isn't mid-edit, so a
// 1s poll can't clobber what they're typing.
var editing = {};
function setInput(id, value, key) {
  if (editing[key]) return;
  document.getElementById(id).value = value;
}

function saveThresholds() {
  var vd = document.getElementById('vdis').value;
  var vc = document.getElementById('vchg').value;
  editing.v = true;
  fetch('/set/thresholds?vdischarged=' + encodeURIComponent(vd) + '&vcharged=' + encodeURIComponent(vc))
    .then(function (r) { return r.text().then(function (t) { return { ok: r.ok, msg: t }; }); })
    .then(function (res) {
      document.getElementById('setMsg').innerHTML =
        res.ok ? '<span class="ok">saved</span>' : '<span class="bad">' + res.msg + '</span>';
      if (res.ok) { editing.v = false; refresh(); }
    })
    .catch(function () {
      document.getElementById('setMsg').innerHTML = '<span class="bad">request failed</span>';
    });
}

function saveCurrentLimit() {
  var v = document.getElementById('vcur').value;
  editing.c = true;
  fetch('/set/current?volts=' + encodeURIComponent(v))
    .then(function (r) { return r.text().then(function (t) { return { ok: r.ok, msg: t }; }); })
    .then(function (res) {
      document.getElementById('cmsg').innerHTML =
        res.ok ? '<span class="ok">saved</span>' : '<span class="bad">' + res.msg + '</span>';
      if (res.ok) { editing.c = false; refresh(); }
    })
    .catch(function () {
      document.getElementById('cmsg').innerHTML = '<span class="bad">request failed</span>';
    });
}

function saveFanFloor() {
  var v = document.getElementById('vfan').value;
  editing.f = true;
  fetch('/set/fanfloor?volts=' + encodeURIComponent(v))
    .then(function (r) { return r.text().then(function (t) { return { ok: r.ok, msg: t }; }); })
    .then(function (res) {
      document.getElementById('cmsg').innerHTML =
        res.ok ? '<span class="ok">saved</span>' : '<span class="bad">' + res.msg + '</span>';
      if (res.ok) { editing.f = false; refresh(); }
    })
    .catch(function () {
      document.getElementById('cmsg').innerHTML = '<span class="bad">request failed</span>';
    });
}

// One-click version of setting both thresholds to 0, which is the state you want
// while measuring the sense line. Post sequentially so a failure on the first
// cannot leave the two halves disagreeing silently.
function disableCurrentChecks() {
  editing.c = true; editing.f = true;
  fetch('/set/current?volts=0')
    .then(function (r) { return r.ok ? r.text() : Promise.reject('ceiling rejected'); })
    .then(function () { return fetch('/set/fanfloor?volts=0'); })
    .then(function (r) {
      document.getElementById('cmsg').innerHTML =
        r.ok ? '<span class="ok">both checks disabled</span>'
             : '<span class="bad">ceiling disabled, floor failed</span>';
      editing.c = false; editing.f = false;
      refresh();
    })
    .catch(function (e) {
      document.getElementById('cmsg').innerHTML =
        '<span class="bad">failed: ' + e + '</span>';
      editing.c = false; editing.f = false;
      refresh();
    });
}

function resetCapture() {
  fetch('/cal/reset')
    .then(function (r) {
      document.getElementById('cmsg').innerHTML =
        r.ok ? '<span class="ok">capture reset</span>' : '<span class="bad">failed</span>';
      refresh();
    })
    .catch(function () {
      document.getElementById('cmsg').innerHTML = '<span class="bad">request failed</span>';
    });
}

function saveBuzzer() {
  var hz = document.getElementById('bhz').value;
  editing.hz = true;
  fetch('/set/buzzer?hz=' + encodeURIComponent(hz))
    .then(function (r) { return r.text().then(function (t) { return { ok: r.ok, msg: t }; }); })
    .then(function (res) {
      document.getElementById('bmsg').innerHTML =
        res.ok ? '<span class="ok">saved</span>' : '<span class="bad">' + res.msg + '</span>';
      if (res.ok) { editing.hz = false; refresh(); }
    })
    .catch(function () {
      document.getElementById('bmsg').innerHTML = '<span class="bad">request failed</span>';
    });
}

['vdis', 'vchg', 'vcur', 'vfan'].forEach(function (id) {
  var key = { vdis: 'v', vchg: 'v', vcur: 'c', vfan: 'f' }[id];
  document.getElementById(id).addEventListener('focus', function () { editing[key] = true; });
  document.getElementById(id).addEventListener('blur', function () { editing[key] = false; });
});
document.getElementById('bhz').addEventListener('focus', function () { editing.hz = true; });
document.getElementById('bhz').addEventListener('blur', function () { editing.hz = false; });

function refresh() {
  fetch('/status').then(r => r.json()).then(d => {
    let html = '';
    let p = d.pins || {};
    updateDial(d);
    setInput('vdis', d.capDischargedVolts, 'v');
    setInput('vchg', d.capChargedVolts, 'v');
    setInput('vcur', d.pumpStallLimitVolts, 'c');
    setInput('vfan', d.fanRunMinVolts, 'f');
    setInput('bhz', d.buzzerHz, 'hz');

    // ---- calibration panel ----
    // Raw counts beside every input, so the numbers written down during a
    // calibration session are the ones the firmware actually compares, not
    // volts that have been through a placeholder divider.
    document.getElementById('acdis').textContent =
      'now ' + d.capAdc + '  dis<=' + d.capDischargedMax + '  chg>=' + d.capChargedMin;
    document.getElementById('acchg').textContent =
      'now ' + d.capAdc + '  dis<=' + d.capDischargedMax + '  chg>=' + d.capChargedMin;
    document.getElementById('acccur').textContent = d.pumpStallDisabled
      ? 'adc ' + d.pumpStallLimit + ' - DISABLED'
      : 'adc ' + d.pumpStallLimit;
    document.getElementById('accfan').textContent = d.fanFloorDisabled
      ? 'adc ' + d.fanRunMin + ' - DISABLED'
      : 'adc ' + d.fanRunMin;

    let live = row('12V sense now', d.currentVolts + ' V <span class="gpio">adc ' +
                   d.currentAdc + '</span>');
    live += row('Cap sense now', d.capVolts + ' V <span class="gpio">adc ' +
                d.capAdc + '</span>');
    if (d.pumpStalled) {
      live += row('Pump', '<span class="bad">STALLED</span> &middot; retry in ' +
                  fmtMs(d.pumpStallRetryRemainingMs), '');
    }
    document.getElementById('calLive').innerHTML = live;

    document.getElementById('calPeak').innerHTML =
      row('Low seen', '<span class="gpio">adc ' + d.currentAdcMin + '</span>') +
      row('High seen', '<span class="gpio">adc ' + d.currentAdcMax + '</span>') +
      row('Spread', '<span class="gpio">' +
        (d.currentAdcMax - d.currentAdcMin) + ' counts</span>');

    // Loud, and in the same place as the inputs, because 0 means "this
    // protection is off right now" and that is not something to discover later.
    let warn = '';
    if (d.pumpStallDisabled) {
      warn += '<div class="card" style="border:2px solid #b00">' +
        '<span class="bad">PUMP STALL CUT IS DISABLED</span> ' +
        '<span style="font-size:11px">- the pump can be jammed and will not be ' +
        'cut. Rebooting re-arms it automatically.</span></div>';
    }
    if (d.fanFloorDisabled) {
      warn += '<div class="card" style="border:1px solid #a80">' +
        '<span class="bad">FAN RUNNING CHECK IS DISABLED</span> ' +
        '<span style="font-size:11px">- a seized fan will not be reported. ' +
        'This one stays disabled until a floor is measured.</span></div>';
    }
    document.getElementById('calWarn').innerHTML = warn;
    html += row('Mode', modeSpan(d.mode));
    // System enable is the master gate, so it sits directly under the mode: it
    // is the single answer to "is anything allowed to run right now".
    if (d.systemEnable) {
      html += row('System enable', '<span class="ok">ENABLED</span> ' +
                  '<span class="gpio">mechanical timer running, contact closed</span>',
                  p.systemEnable);
    } else if (d.systemEnableHold) {
      html += row('System enable', '<span class="bad">TIMER STOPPED</span> &middot; restart window ' +
                  fmtMs(d.systemEnableHoldRemainingMs) + ' left &middot; ' +
                  '<span class="gpio">move the mode switch to re-arm</span>',
                  p.systemEnable);
    } else {
      html += row('System enable', '<span class="bad">DISABLED</span> ' +
                  '<span class="gpio">nothing can start in any mode until the ' +
                  'mechanical timer closes its contact</span>', p.systemEnable);
    }
    if (d.modeChangeRemainingMs > 0) {
      html += row('Pending mode', modeSpan(d.selectedMode) + ' in ' +
                  fmtMs(d.modeChangeRemainingMs) +
                  ' <span class="gpio">ring reveal ' + d.ledRevealCount + ' LEDs</span>');
    }
    html += row('State', d.state);
    html += row('Cap sequence', d.capSeq + ' (armed: ' + (d.capArmed ? 'yes' : 'no') + ')');
    html += row('Capacitor', d.capVolts + ' V <span class="gpio">adc ' + d.capAdc + '</span> &middot; arms &le;' +
                d.capDischargedVolts + ' V, closes &ge;' + d.capChargedVolts + ' V', p.capSense);
    html += row('Cap charge out', boolCell(d.capCharge), p.capCharge);
    html += row('Cap discharging', d.capDischarging
                ? '<span class="ok">' + fmtMs(d.capDischargeMs) + ' and counting</span>'
                : 'no &middot; last took ' + fmtMs(d.capLastDischargeMs));
    html += row('12V rail current', d.currentVolts + ' V <span class="gpio">adc ' + d.currentAdc + '</span>' +
                ' &middot; pump stall ceiling ' + d.pumpStallLimitVolts + ' V' +
                ' <span class="gpio">adc ' + d.pumpStallLimit + '</span>' +
                ' &middot; fan floor ' +
                (d.fanRunMin > 0 ? d.fanRunMinVolts + ' V <span class="gpio">adc ' + d.fanRunMin + '</span>'
                                 : '<span class="gpio">not measured</span>') +
                ' <span class="gpio">uncalibrated</span>', p.currentSense);
    if (d.pumpStalled) {
      html += row('Pump', '<span class="bad">STALLED</span> &middot; cut now, retrying in ' +
                  fmtMs(d.pumpStallRetryRemainingMs) +
                  ' <span class="gpio">chiller unaffected</span>', p.currentSense);
    }
    // One row for the fan, stating WHY it is on, so "off" is visibly wrong the
    // moment the compressor is running rather than looking like a normal state.
    let fanText = boolCell(d.fan);
    if (d.compressor) {
      fanText += ' <span class="gpio">following compressor contacts</span>';
    } else if (d.testFanActive) {
      fanText += ' <span class="gpio">manual test</span>';
    } else {
      fanText += ' <span class="gpio">auto, compressor stopped</span>';
    }
    if (d.fanStalled) {
      fanText += ' <span class="bad">NOT DRAWING</span>' +
                 ' <span class="gpio">seized fan or sensor fault, report only</span>';
    }
    html += row('Condenser fan', fanText, p.fan);
    html += row('Fault', d.fault
      ? '<span class="bad">LATCHED</span> &middot; ' +
        (d.faultReason || 'unknown') +
        ' <span class="gpio">turn the switch to off and leave it ' +
        Math.round(d.faultResetHoldMs / 1000) + 's to clear</span>'
      : '<span>none</span>');
    html += row('Buzzer', boolCell(d.buzzerToneActive) + ' &middot; pulsed at ' + d.buzzerHz + ' Hz' +
                (d.testBuzzerActive ? ' (manual)' : ''), p.buzzer);
    html += row('12V enable (pump)', boolCell(d.enable12v) + (d.enable12vHold ? ' (hold-off delay active)' : ''), p.enable12v);
    html += row('Compressor', boolCell(d.compressor) + (d.testCompressorActive ? ' (manual demand)' : ''), p.compressor);
    html += row('Compressor run time', d.compressor ? '<span class="ok">' + fmtMs(d.compressorRunMs) + '</span>' : '0.0s');
    html += row('Compressor stop pending', d.compressorStopPending
                ? '<span>holding ' + fmtMs(d.compressorOffDelayRemaining) + ' left (off-delay)</span>'
                : 'no');
    html += row('Switch 1 (Off)', d.switch1 ? boolCell(true) : 'not wired', p.switch1);
    html += row('Switch 2 (Pump only)', boolCell(d.switch2), p.switch2);
    html += row('Switch 3 (Chiller only)', boolCell(d.switch3), p.switch3);
    html += row('Switch 4 (Both)', boolCell(d.switch4), p.switch4);
    document.getElementById('statusTable').innerHTML = html;
  }).catch(() => {});
}

refresh();
setInterval(refresh, 1000);
</script>
</body>
</html>
)====";
