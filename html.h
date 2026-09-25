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
  button { margin: 2px; padding: 6px 10px; }
  h3 { margin-top: 24px; }
</style>
</head>
<body>
<h2>Chiller Status</h2>
<table id="statusTable"></table>

<h3>Fan test</h3>
<button onclick="testCall('/test/fan?state=on')">On</button>
<button onclick="testCall('/test/fan?state=off')">Off</button>
<button onclick="testCall('/test/fan?state=auto')">Auto</button>

<h3>Buzzer test</h3>
<button onclick="testCall('/test/buzzer?state=on')">On</button>
<button onclick="testCall('/test/buzzer?state=off')">Off</button>
<button onclick="testCall('/test/buzzer?state=auto')">Auto</button>

<h3>Compressor demand test (still safety-gated by ADC)</h3>
<button onclick="testCall('/test/compressor?state=on')">Request On</button>
<button onclick="testCall('/test/compressor?state=off')">Request Off</button>
<button onclick="testCall('/test/compressor?state=auto')">Auto (switch-controlled)</button>

<script>
function testCall(url) {
  fetch(url).then(refresh);
}

function row(label, value) {
  return '<tr><td>' + label + '</td><td>' + value + '</td></tr>';
}

function boolCell(v) {
  return v ? '<span class="ok">ON</span>' : '<span>off</span>';
}

function refresh() {
  fetch('/status').then(r => r.json()).then(d => {
    let html = '';
    html += row('Mode', d.mode);
    html += row('State', d.state);
    html += row('ADC (12V sense)', d.adc + ' / limit ' + d.adcLimit + (d.safe ? ' <span class="ok">SAFE</span>' : ' <span class="bad">UNSAFE</span>'));
    html += row('Fan', boolCell(d.fan) + (d.testFanActive ? ' (manual)' : ''));
    html += row('Buzzer', boolCell(d.buzzer) + (d.testBuzzerActive ? ' (manual)' : ''));
    html += row('Pump / 12V enable', boolCell(d.pump12v) + (d.pumpOffHold ? ' (hold-off delay active)' : ''));
    html += row('Compressor', boolCell(d.compressor) + (d.testCompressorActive ? ' (manual demand)' : ''));
    html += row('Switch 1 (Off)', boolCell(d.switch1));
    html += row('Switch 2 (Pump only)', boolCell(d.switch2));
    html += row('Switch 3 (Chiller only)', boolCell(d.switch3));
    html += row('Switch 4 (Both)', boolCell(d.switch4));
    document.getElementById('statusTable').innerHTML = html;
  }).catch(() => {});
}

refresh();
setInterval(refresh, 1000);
</script>
</body>
</html>
)====";
