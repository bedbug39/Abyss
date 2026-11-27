#include <WiFi.h>
#include <esp_arduino_version.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <math.h>
#include <string.h>

// ============================================================
// ROVER CONTROL - SPORT EDITION
// ESP32 Arduino Core 2.x and 3.x compatible
// BTS7960 dual-side tank drive
// ============================================================

// --- Wi-Fi Access Point ---
const char* AP_SSID = "Rover-Control";
const char* AP_PASS = "Podmar123";  // WPA2 password: at least 8 characters

// --- BTS7960 Pins (keep the same wiring as your previous code) ---
const int L_RPWM = 4;
const int L_LPWM = 5;
const int R_RPWM = 17;
const int R_LPWM = 18;
const int PIN_EN = 6;  // IMPORTANT: GPIO6 is Flash-connected on many ESP32-WROOM boards.
                      // If using ESP32-WROOM/DevKit, move EN wiring to a safe GPIO (e.g. 23)
                      // and change this value to 23 before uploading.

// --- PWM ---
const int CH_LR = 0;
const int CH_LL = 1;
const int CH_RR = 2;
const int CH_RL = 3;
const int PWM_FREQ = 20000;
const int PWM_BITS = 10;
const int MAX_DUTY = 1023;

// --- Motor polarity calibration ---
// These signs are kept from the last version because Forward/Reverse are now correct.
int leftSign  = +1;
int rightSign = -1;
bool driverEnabled = true;

// --- Telemetry ---
int curLR = 0;
int curLL = 0;
int curRR = 0;
int curRL = 0;
String activeMode = "IDLE";

// --- Watchdog ---
unsigned long lastCmdTime = 0;
unsigned long lastTelemTime = 0;
const unsigned long WATCHDOG_MS = 400;

WebServer server(80);
WebSocketsServer webSocket(81);

// ============================================================
// SPORT DASHBOARD
// ============================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<meta name="theme-color" content="#090b0e">
<title>ROVER // SPORT CONTROL</title>
<style>
  :root {
    --bg: #090b0e;
    --panel: #14191f;
    --panel-2: #1b222b;
    --line: #303944;
    --muted: #89939f;
    --white: #f4f6f8;
    --orange: #ff5a1f;
    --orange-2: #ff8a24;
    --red: #ef3038;
    --green: #a3ff3f;
    --cyan: #55d9ff;
  }
  * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; user-select: none; -webkit-user-select: none; }
  html { background: var(--bg); }
  body {
    margin: 0; min-height: 100vh; padding: 16px 12px 26px;
    color: var(--white); font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    background:
      radial-gradient(ellipse at 85% 0%, rgba(255,90,31,.14), transparent 34%),
      radial-gradient(ellipse at 0% 42%, rgba(85,217,255,.07), transparent 32%),
      linear-gradient(180deg, #0b0e12 0%, #080a0d 100%);
    display: flex; justify-content: center;
  }
  .app { width: 100%; max-width: 520px; }
  .topbar { display:flex; justify-content:space-between; align-items:center; gap:12px; margin-bottom:14px; }
  .brand { display:flex; align-items:center; gap:10px; min-width:0; }
  .brand-mark { width:38px; height:38px; flex:none; display:grid; place-items:center; background:linear-gradient(145deg,var(--orange-2),var(--red)); clip-path:polygon(0 0,100% 0,100% 72%,72% 100%,0 100%); font-size:18px; font-weight:1000; color:#111; }
  .brand-copy { min-width:0; }
  .brand-title { font-size:16px; line-height:1; letter-spacing:1.8px; font-weight:950; }
  .brand-sub { margin-top:5px; color:var(--muted); font-size:9px; letter-spacing:2.1px; font-weight:800; }
  .connection { display:flex; align-items:center; gap:7px; border:1px solid var(--line); background:#11161c; padding:8px 10px; border-radius:5px; font-size:10px; letter-spacing:1px; font-weight:900; white-space:nowrap; }
  .dot { width:7px; height:7px; border-radius:50%; background:var(--red); box-shadow:0 0 10px rgba(239,48,56,.7); }
  .connection.online .dot { background:var(--green); box-shadow:0 0 10px rgba(163,255,63,.7); }
  .hero { position:relative; overflow:hidden; padding:18px 17px 16px; margin-bottom:12px; border:1px solid #543124; background:linear-gradient(115deg,rgba(255,90,31,.17),rgba(20,25,31,.96) 52%,rgba(20,25,31,.96)); clip-path:polygon(0 0,calc(100% - 18px) 0,100% 18px,100% 100%,0 100%); }
  .hero:after { content:""; position:absolute; right:28px; top:-42px; width:90px; height:180px; transform:rotate(32deg); border-left:2px solid rgba(255,138,36,.18); border-right:2px solid rgba(255,138,36,.12); pointer-events:none; }
  .eyebrow { color:var(--orange-2); font-size:9px; letter-spacing:2.8px; font-weight:950; }
  .hero h1 { margin:7px 0 5px; font-size:clamp(25px,7vw,34px); line-height:.95; letter-spacing:1px; font-weight:1000; font-style:italic; }
  .hero h1 span { color:var(--orange); }
  .hero-foot { display:flex; justify-content:space-between; align-items:center; gap:8px; color:#aeb7c1; font-size:10px; letter-spacing:1.2px; font-weight:800; }
  .mode-chip { border:1px solid #60402f; background:rgba(255,90,31,.1); color:#ffad79; padding:5px 8px; font-family:ui-monospace,monospace; font-size:10px; letter-spacing:.5px; }
  .panel { background:linear-gradient(145deg,rgba(25,32,40,.98),rgba(16,21,27,.98)); border:1px solid var(--line); padding:14px; margin-bottom:12px; position:relative; }
  .panel-head { display:flex; justify-content:space-between; align-items:center; gap:10px; margin-bottom:12px; }
  .panel-title { margin:0; color:#dce2e8; font-size:11px; letter-spacing:1.8px; font-weight:950; text-transform:uppercase; }
  .panel-tag { color:var(--muted); font:800 9px ui-monospace,monospace; letter-spacing:1px; }
  .speed-value { display:flex; align-items:baseline; gap:5px; margin:2px 0 12px; }
  .speed-value strong { font-size:34px; font-style:italic; line-height:1; letter-spacing:-1px; }
  .speed-value span { color:var(--orange-2); font-size:11px; font-weight:950; letter-spacing:1.5px; }
  .presets { display:grid; grid-template-columns:repeat(3,1fr); gap:7px; margin-bottom:13px; }
  button { font:inherit; border:0; cursor:pointer; touch-action:manipulation; }
  .preset { padding:10px 5px; background:#0d1116; color:#aeb8c3; border:1px solid #333d48; border-radius:4px; font-size:10px; letter-spacing:1px; font-weight:950; }
  .preset.active { color:#111; background:linear-gradient(110deg,var(--orange-2),var(--orange)); border-color:var(--orange-2); box-shadow:0 0 16px rgba(255,90,31,.18); }
  .slider-wrap { display:flex; align-items:center; gap:11px; color:var(--muted); font-size:10px; font-weight:850; letter-spacing:1px; }
  input[type=range] { flex:1; min-width:0; accent-color:var(--orange); cursor:pointer; }
  .drive-panel { padding-bottom:13px; }
  .hold-note { color:var(--muted); font-size:9px; font-weight:800; letter-spacing:1.3px; }
  .dpad { display:grid; grid-template-columns:repeat(3,minmax(0,1fr)); gap:8px; }
  .drive-btn { min-height:67px; border:1px solid #3a4652; border-radius:5px; color:#e8edf2; background:linear-gradient(145deg,#29323d,#171d24); font-size:11px; font-weight:1000; letter-spacing:1px; position:relative; overflow:hidden; box-shadow:inset 0 1px rgba(255,255,255,.05),0 5px 12px rgba(0,0,0,.2); }
  .drive-btn .arrow { display:block; font-size:22px; line-height:1.1; color:var(--orange-2); margin-bottom:4px; }
  .drive-btn:active,.drive-btn.pressed { background:linear-gradient(145deg,#ff8a24,#f0441d); color:#111; border-color:#ff9b48; transform:translateY(1px); }
  .drive-btn:active .arrow,.drive-btn.pressed .arrow { color:#111; }
  .drive-btn.stop { background:linear-gradient(145deg,#a51f27,#5d1218); border-color:#ed3b43; color:white; font-size:13px; letter-spacing:2px; box-shadow:0 0 18px rgba(239,48,56,.16); }
  .drive-btn.stop .arrow { color:white; font-size:17px; }
  .blank { min-height:1px; }
  .iso-grid { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:7px; }
  .iso-btn { padding:11px 6px; border-radius:4px; background:#0d141a; color:var(--cyan); border:1px solid #28566a; font-size:9px; letter-spacing:.8px; font-weight:950; }
  .iso-btn:active,.iso-btn.pressed { background:#14506a; color:#fff; }
  .telemetry { display:grid; grid-template-columns:repeat(2,1fr); gap:8px; margin:12px 0 2px; }
  .metric { background:#0c1116; border:1px solid #29333d; padding:10px; }
  .metric-label { color:var(--muted); font-size:8px; font-weight:900; letter-spacing:1.4px; }
  .metric-value { margin-top:5px; font:900 14px ui-monospace,monospace; color:var(--green); }
  .metric-value.bad { color:#ff646b; }
  table { width:100%; border-collapse:collapse; margin-top:12px; font:11px ui-monospace,monospace; }
  th,td { padding:8px 5px; text-align:center; border:1px solid #303a44; }
  th { color:#9aa6b2; background:#0c1116; font-size:9px; letter-spacing:.8px; }
  td { color:#e4e9ee; }
  td:first-child { color:#b7c1ca; font-weight:900; }
  .ctrl-row { display:grid; grid-template-columns:repeat(3,minmax(0,1fr)); gap:6px; margin-top:10px; }
  .mini-btn { padding:9px 4px; border:1px solid #39434e; border-radius:4px; background:#202832; color:#c8d0d8; font-size:8px; letter-spacing:.4px; font-weight:900; }
  .mini-btn:active { background:#374452; }
  .footer { display:flex; justify-content:space-between; gap:8px; padding:3px 2px 0; color:#65717d; font:800 8px ui-monospace,monospace; letter-spacing:1px; }
  .accent-line { height:3px; width:58px; background:linear-gradient(90deg,var(--orange),transparent); margin-top:10px; }
  @media (max-width:360px) { body{padding:12px 8px 20px}.panel{padding:11px}.drive-btn{min-height:61px;font-size:10px}.brand-title{font-size:14px}.connection{font-size:9px;padding:7px 8px} }
</style>
</head>
<body>
<div class="app">
  <header class="topbar">
    <div class="brand">
      <div class="brand-mark">R</div>
      <div class="brand-copy">
        <div class="brand-title">ROVER CONTROL</div>
        <div class="brand-sub">4WD // TANK DRIVE SYSTEM</div>
      </div>
    </div>
    <div id="connection" class="connection"><span class="dot"></span><span id="status">OFFLINE</span></div>
  </header>

  <section class="hero">
    <div class="eyebrow">FIELD OPERATIONS // ACTIVE LINK</div>
    <h1>BUILT TO <span>ROAM.</span></h1>
    <div class="hero-foot"><span>PRECISION DRIVE INTERFACE</span><span id="modeBadge" class="mode-chip">MODE: IDLE</span></div>
    <div class="accent-line"></div>
  </section>

  <section class="panel">
    <div class="panel-head"><h2 class="panel-title">Throttle limiter</h2><span class="panel-tag">POWER / 01</span></div>
    <div class="speed-value"><strong id="spdVal">30</strong><span> PERCENT OUTPUT</span></div>
    <div class="presets">
      <button class="preset active" onclick="setSpd(30,this)">ECO 30</button>
      <button class="preset" onclick="setSpd(60,this)">SPORT 60</button>
      <button class="preset" onclick="setSpd(100,this)">MAX 100</button>
    </div>
    <div class="slider-wrap"><span>15%</span><input type="range" id="spd" min="15" max="100" value="30" oninput="onSlide(this.value)"><span>100%</span></div>
  </section>

  <section class="panel drive-panel">
    <div class="panel-head"><h2 class="panel-title">Drive controls</h2><span class="hold-note">HOLD TO MOVE / RELEASE TO STOP</span></div>
    <div class="dpad">
      <!-- Turn input signs are swapped to match the rover's observed physical LEFT/RIGHT movement. -->
      <button class="drive-btn" data-cmd="D,0.6,-0.4"><span class="arrow">↖</span>ARC LEFT</button>
      <button class="drive-btn" data-cmd="D,1,0"><span class="arrow">↑</span>FORWARD</button>
      <button class="drive-btn" data-cmd="D,0.6,0.4"><span class="arrow">↗</span>ARC RIGHT</button>
      <button class="drive-btn" data-cmd="D,0,-1"><span class="arrow">←</span>LEFT</button>
      <button class="drive-btn stop" data-cmd="D,0,0"><span class="arrow">■</span>STOP</button>
      <button class="drive-btn" data-cmd="D,0,1"><span class="arrow">→</span>RIGHT</button>
      <div class="blank"></div>
      <button class="drive-btn" data-cmd="D,-1,0"><span class="arrow">↓</span>REVERSE</button>
      <div class="blank"></div>
    </div>
  </section>

  <section class="panel">
    <div class="panel-head"><h2 class="panel-title">Motor diagnostics</h2><span class="panel-tag">SYSTEM / 02</span></div>
    <div class="iso-grid">
      <button class="iso-btn" data-cmd="I,LF">LEFT SIDE / FWD</button>
      <button class="iso-btn" data-cmd="I,RF">RIGHT SIDE / FWD</button>
      <button class="iso-btn" data-cmd="I,LR">LEFT SIDE / REV</button>
      <button class="iso-btn" data-cmd="I,RR">RIGHT SIDE / REV</button>
    </div>
    <div class="telemetry">
      <div class="metric"><div class="metric-label">DRIVER ENABLE</div><div id="tEN" class="metric-value">HIGH</div></div>
      <div class="metric"><div class="metric-label">WATCHDOG</div><div id="tWD" class="metric-value">OK</div></div>
    </div>
    <table>
      <thead><tr><th>SIDE / GPIO</th><th>RPWM</th><th>LPWM</th><th>POLARITY</th></tr></thead>
      <tbody>
        <tr><td>LEFT / 4,5</td><td id="tLR">0</td><td id="tLL">0</td><td id="tLS">+1</td></tr>
        <tr><td>RIGHT / 17,18</td><td id="tRR">0</td><td id="tRL">0</td><td id="tRS">-1</td></tr>
      </tbody>
    </table>
    <div class="ctrl-row">
      <button class="mini-btn" onclick="sendCfg('FLIP_L')">FLIP LEFT POLARITY</button>
      <button class="mini-btn" onclick="sendCfg('FLIP_R')">FLIP RIGHT POLARITY</button>
      <button class="mini-btn" onclick="sendCfg('TOGGLE_EN')">TOGGLE DRIVER EN</button>
    </div>
  </section>
  <footer class="footer"><span>ROVER // SPORT UI</span><span>WASD / ARROW KEYS ENABLED</span></footer>
</div>

<script>
  let ws = null;
  let activeCmd = "D,0,0";
  const statusEl = document.getElementById('status');
  const connectionEl = document.getElementById('connection');
  const spdEl = document.getElementById('spd');
  const spdVal = document.getElementById('spdVal');

  function setConnection(online) {
    statusEl.textContent = online ? 'ONLINE' : 'OFFLINE';
    connectionEl.classList.toggle('online', online);
  }

  function connect() {
    ws = new WebSocket('ws://' + location.hostname + ':81/');
    ws.onopen = () => setConnection(true);
    ws.onclose = () => {
      setConnection(false);
      releaseAll();
      setTimeout(connect, 1000);
    };
    ws.onerror = () => setConnection(false);
    ws.onmessage = (event) => {
      const p = event.data.split(',');
      if (p.length !== 9) return;
      document.getElementById('tLR').textContent = p[0];
      document.getElementById('tLL').textContent = p[1];
      document.getElementById('tRR').textContent = p[2];
      document.getElementById('tRL').textContent = p[3];
      document.getElementById('tEN').textContent = p[4] === '1' ? 'ENABLED' : 'DISABLED';
      document.getElementById('tEN').classList.toggle('bad', p[4] !== '1');
      document.getElementById('tLS').textContent = p[5];
      document.getElementById('tRS').textContent = p[6];
      document.getElementById('tWD').textContent = p[7];
      document.getElementById('tWD').classList.toggle('bad', p[7] !== 'OK');
      document.getElementById('modeBadge').textContent = 'MODE: ' + p[8];
    };
  }
  connect();

  function setSpd(value, button) {
    spdEl.value = value;
    spdVal.textContent = value;
    document.querySelectorAll('.preset').forEach(b => b.classList.remove('active'));
    if (button) button.classList.add('active');
  }
  function onSlide(value) {
    spdVal.textContent = value;
    document.querySelectorAll('.preset').forEach(b => b.classList.remove('active'));
  }
  function sendCfg(command) {
    if (ws && ws.readyState === WebSocket.OPEN) ws.send('C,' + command);
  }
  function releaseAll() {
    activeCmd = 'D,0,0';
    document.querySelectorAll('.pressed').forEach(b => b.classList.remove('pressed'));
    Object.keys(keys).forEach(k => keys[k] = false);
  }

  // Refresh the command frequently; the ESP32 watchdog stops the rover if packets stop.
  setInterval(() => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      const scale = (Number(spdEl.value) / 100).toFixed(2);
      ws.send(activeCmd + ',' + scale);
    }
  }, 90);

  // Pointer controls: release/cancel always sends a neutral command on the next packet.
  document.querySelectorAll('[data-cmd]').forEach(button => {
    const down = (event) => {
      event.preventDefault();
      activeCmd = button.dataset.cmd;
      document.querySelectorAll('.pressed').forEach(b => b.classList.remove('pressed'));
      button.classList.add('pressed');
    };
    const up = (event) => {
      event.preventDefault();
      activeCmd = 'D,0,0';
      button.classList.remove('pressed');
    };
    button.addEventListener('pointerdown', down);
    button.addEventListener('pointerup', up);
    button.addEventListener('pointerleave', up);
    button.addEventListener('pointercancel', up);
  });

  // Keyboard: invert only steering input to match the observed rover response.
  const keys = {};
  window.addEventListener('keydown', (event) => {
    const key = event.key.toLowerCase();
    if (['arrowup','arrowdown','arrowleft','arrowright',' '].includes(key)) event.preventDefault();
    if (event.repeat) return;
    keys[key] = true;
    updateKeys();
  });
  window.addEventListener('keyup', (event) => {
    keys[event.key.toLowerCase()] = false;
    updateKeys();
  });
  window.addEventListener('blur', releaseAll);
  document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); });

  function updateKeys() {
    const throttle = (keys['w'] || keys['arrowup'] ? 1 : 0) - (keys['s'] || keys['arrowdown'] ? 1 : 0);
    // A/LEFT now sends -1; D/RIGHT sends +1 (steering input swapped).
    const turn = (keys['d'] || keys['arrowright'] ? 1 : 0) - (keys['a'] || keys['arrowleft'] ? 1 : 0);
    activeCmd = `D,${throttle},${turn}`;
  }
</script>
</body>
</html>
)rawliteral";

// ============================================================
// PWM HELPERS - Core 2.x / 3.x
// ============================================================
void setupPwm(int pin, int channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (!ledcAttach(pin, PWM_FREQ, PWM_BITS)) {
    digitalWrite(PIN_EN, LOW);
    Serial.printf("PWM attach failed on GPIO %d\n", pin);
    while (true) delay(1000);
  }
  ledcWrite(pin, 0);
#else
  if (ledcSetup(channel, PWM_FREQ, PWM_BITS) == 0) {
    digitalWrite(PIN_EN, LOW);
    Serial.printf("PWM setup failed on GPIO %d\n", pin);
    while (true) delay(1000);
  }
  ledcAttachPin(pin, channel);
  ledcWrite(channel, 0);
#endif
}

void writePwm(int pin, int channel, int duty) {
  duty = constrain(duty, 0, MAX_DUTY);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

void setLeftRaw(int duty) {
  duty = constrain(duty, -MAX_DUTY, MAX_DUTY);
  if (duty >= 0) {
    curLL = 0;
    curLR = duty;
  } else {
    curLR = 0;
    curLL = -duty;
  }
  writePwm(L_LPWM, CH_LL, curLL);
  writePwm(L_RPWM, CH_LR, curLR);
}

void setRightRaw(int duty) {
  duty = constrain(duty, -MAX_DUTY, MAX_DUTY);
  if (duty >= 0) {
    curRL = 0;
    curRR = duty;
  } else {
    curRR = 0;
    curRL = -duty;
  }
  writePwm(R_LPWM, CH_RL, curRL);
  writePwm(R_RPWM, CH_RR, curRR);
}

void stopAll() {
  setLeftRaw(0);
  setRightRaw(0);
  activeMode = "IDLE";
}

void driveTank(float throttle, float turn) {
  // Keep the motor mixing and motor polarity unchanged.
  // UI/keyboard steering commands are inverted to correct observed L/R response.
  float left = throttle - turn;
  float right = throttle + turn;
  float peak = fmaxf(fabsf(left), fabsf(right));
  if (peak > 1.0f) {
    left /= peak;
    right /= peak;
  }
  setLeftRaw(leftSign * (int)(left * MAX_DUTY));
  setRightRaw(rightSign * (int)(right * MAX_DUTY));
}

// ============================================================
// WEBSOCKET COMMAND HANDLER
// ============================================================
void onWebSocketEvent(uint8_t clientNum, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    // WebSocket payload is length-delimited; copy it into a bounded, null-terminated buffer.
    char msg[64];
    if (length == 0 || length >= sizeof(msg)) return;
    memcpy(msg, payload, length);
    msg[length] = '\0';

    if (strncmp(msg, "C,", 2) == 0) {
      if (strcmp(msg + 2, "FLIP_L") == 0) {
        leftSign = -leftSign;
      } else if (strcmp(msg + 2, "FLIP_R") == 0) {
        rightSign = -rightSign;
      } else if (strcmp(msg + 2, "TOGGLE_EN") == 0) {
        driverEnabled = !driverEnabled;
        if (!driverEnabled) stopAll();
        digitalWrite(PIN_EN, driverEnabled ? HIGH : LOW);
      }
      return;
    }

    if (strncmp(msg, "D,", 2) == 0) {
      float throttle = 0.0f;
      float turn = 0.0f;
      float scale = 0.3f;
      if (sscanf(msg + 2, "%f,%f,%f", &throttle, &turn, &scale) == 3) {
        if (!isfinite(throttle) || !isfinite(turn) || !isfinite(scale)) return;
        throttle = constrain(throttle, -1.0f, 1.0f);
        turn = constrain(turn, -1.0f, 1.0f);
        scale = constrain(scale, 0.0f, 1.0f);
        if (driverEnabled) {
          driveTank(throttle * scale, turn * scale);
          activeMode = (throttle == 0.0f && turn == 0.0f) ? "IDLE" : "TANK";
        } else {
          stopAll();
        }
        lastCmdTime = millis();
      }
      return;
    }

    if (strncmp(msg, "I,", 2) == 0) {
      char channel[4] = {0};
      float scale = 0.3f;
      if (sscanf(msg + 2, "%3[^,],%f", channel, &scale) == 2) {
        if (!isfinite(scale)) return;
        int duty = (int)(constrain(scale, 0.0f, 1.0f) * MAX_DUTY);
        stopAll();
        if (driverEnabled) {
          if (strcmp(channel, "LF") == 0) { setLeftRaw(leftSign * duty); activeMode = "ISO_L_FWD"; }
          else if (strcmp(channel, "LR") == 0) { setLeftRaw(-leftSign * duty); activeMode = "ISO_L_REV"; }
          else if (strcmp(channel, "RF") == 0) { setRightRaw(rightSign * duty); activeMode = "ISO_R_FWD"; }
          else if (strcmp(channel, "RR") == 0) { setRightRaw(-rightSign * duty); activeMode = "ISO_R_REV"; }
        }
        lastCmdTime = millis();
      }
    }
  } else if (type == WStype_DISCONNECTED) {
    stopAll();
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW);

  setupPwm(L_RPWM, CH_LR);
  setupPwm(L_LPWM, CH_LL);
  setupPwm(R_RPWM, CH_RR);
  setupPwm(R_LPWM, CH_RL);

  digitalWrite(PIN_EN, HIGH);
  driverEnabled = true;
  lastCmdTime = millis();

  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(AP_SSID, AP_PASS);
  if (!apStarted) {
    Serial.println("ERROR: Wi-Fi AP failed to start.");
  }
  Serial.print("Dashboard: http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", []() {
    server.send_P(200, "text/html", INDEX_HTML);
  });
  server.begin();

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

void loop() {
  server.handleClient();
  webSocket.loop();

  // USB serial emergency stop: send x, X, or space. Send ? for AP IP.
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'x' || c == 'X' || c == ' ') {
      stopAll();
    } else if (c == '?') {
      Serial.print("AP IP: ");
      Serial.println(WiFi.softAPIP());
    }
  }

  bool watchdogTripped = (millis() - lastCmdTime > WATCHDOG_MS);
  if (watchdogTripped && activeMode != "IDLE") {
    stopAll();
  }

  if (millis() - lastTelemTime >= 150) {
    lastTelemTime = millis();
    String telemetry = String(curLR) + "," + String(curLL) + "," +
                       String(curRR) + "," + String(curRL) + "," +
                       String(digitalRead(PIN_EN)) + "," +
                       (leftSign > 0 ? "+1" : "-1") + "," +
                       (rightSign > 0 ? "+1" : "-1") + "," +
                       (watchdogTripped ? "STOP" : "OK") + "," +
                       activeMode;
    webSocket.broadcastTXT(telemetry);
  }
}
