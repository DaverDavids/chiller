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

// Normal-operation status/control page
// TODO: add live ADC readout, state machine status, and manual
// fan/buzzer/compressor test controls once behavior is finalized.
const char PAGE_STATUS[] PROGMEM = R"====(
<!DOCTYPE html>
<html>
<head><title>Chiller Status</title></head>
<body>
<h2>Chiller Control</h2>
<p>TODO: live status (ADC value, state, fan/compressor status)</p>
<!-- TODO: add controls for fan/buzzer test, timer override, etc. -->
</body>
</html>
)====";
