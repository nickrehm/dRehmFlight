// ==========================================
// NETWORK CONFIGURATION & SERVER
// ==========================================
// PID tuning web interface. Serves a single-page app that lets you
// tune Roll/Pitch/Yaw x Rate/Angle PID gains live over WiFi, then
// persist them to flash (NVS) once you're happy with them.

AsyncWebServer server(80);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>dRehmFlight PID Tuner</title>
  <style>
    :root {
      --bg-color: #121212;
      --card-bg: #1e1e1e;
      --accent: #00adb5;
      --text: #eeeeee;
      --text-dim: #a0a0a0;
    }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background-color: var(--bg-color);
      color: var(--text);
      margin: 0;
      padding: 15px;
      display: flex;
      justify-content: center;
    }
    .container {
      width: 100%;
      max-width: 400px;
    }
    h2 { text-align: center; margin-bottom: 20px; font-weight: 500; }
    
    /* Mode Selection Tabs */
    .mode-tabs {
      display: flex;
      gap: 8px;
      margin-bottom: 15px;
      background: var(--mode-bg);
      padding: 4px;
      border-radius: 10px;
    }
    .mode-btn {
      flex: 1;
      padding: 10px 0;
      background: transparent;
      border: none;
      border-radius: 8px;
      color: var(--text-dim);
      font-size: 15px;
      font-weight: bold;
      cursor: pointer;
      transition: 0.2s;
    }
    .mode-btn.active {
      background: var(--accent);
      color: #fff;
    }

    /* Axis Tabs */
    .tabs {
      display: flex;
      gap: 8px;
      margin-bottom: 15px;
    }
    .tab-btn {
      flex: 1;
      padding: 10px 0;
      background: var(--card-bg);
      border: 1px solid #333;
      border-radius: 8px;
      color: var(--text-dim);
      font-size: 15px;
      font-weight: bold;
      cursor: pointer;
      transition: 0.2s;
    }
    .tab-btn.active {
      background: var(--accent);
      color: #fff;
      border-color: var(--accent);
    }

    /* PID Card */
    .card {
      background: var(--card-bg);
      padding: 20px;
      border-radius: 12px;
      box-shadow: 0 4px 10px rgba(0,0,0,0.3);
    }
    .slider-group {
      margin-bottom: 20px;
    }
    .slider-group:last-child { margin-bottom: 0; }
    .slider-label {
      display: flex;
      justify-content: space-between;
      font-size: 16px;
      margin-bottom: 8px;
    }
    .slider-label span { font-weight: bold; color: var(--accent); }
    input[type=range] {
      width: 100%;
      height: 6px;
      border-radius: 3px;
      background: #333;
      outline: none;
      accent-color: var(--accent);
    }

    /* Commit Button */
    .btn-commit {
      width: 100%;
      padding: 15px;
      margin-top: 15px;
      background: #34a508;
      color: white;
      border: none;
      border-radius: 8px;
      font-size: 16px;
      font-weight: bold;
      cursor: pointer;
      box-shadow: 0 4px 6px rgba(230, 57, 70, 0.2);
    }
    .btn-commit:active { transform: scale(0.98); }

    /* Reset Button */
    .btn-reset {
      width: 100%;
      padding: 15px;
      margin-top: 15px;
      background: #e63946;
      color: white;
      border: none;
      border-radius: 8px;
      font-size: 16px;
      font-weight: bold;
      cursor: pointer;
      box-shadow: 0 4px 6px rgba(230, 57, 70, 0.2);
    }
    .btn-reset:hover { background-color: #c0392b; }

    #status {
      text-align: center;
      margin-top: 14px;
      font-size: 14px;
      min-height: 18px;
    }
  </style>
</head>
<body>

<div class="container">
  <h2>dRehmFlight PID Tuner</h2>

  <!-- Mode Tabs (Top) -->
  <div class="tabs mode-tabs">
    <button class="tab-btn mode-btn active" id="btn-rate" onclick="selectMode('rate')">Rate Gains</button>
    <button class="tab-btn mode-btn" id="btn-angle" onclick="selectMode('angle')">Angle Gains</button>
  </div>

  <!-- Axis Sub-Tabs -->
  <div class="tabs axis-tabs">
    <button class="tab-btn axis-btn active" id="btn-roll" onclick="selectAxis('roll')">Roll</button>
    <button class="tab-btn axis-btn" id="btn-pitch" onclick="selectAxis('pitch')">Pitch</button>
    <button class="tab-btn axis-btn" id="btn-yaw" onclick="selectAxis('yaw')">Yaw</button>
  </div>

  <!-- PID Controls Card -->
  <div class="card">
    <div class="slider-group">
      <div class="slider-label">P-Gain: <span id="val-p">0.15</span></div>
      <input type="range" id="slider-p" min="0" max="1" step="0.001" oninput="updateValues()">
    </div>
    <div class="slider-group">
      <div class="slider-label">I-Gain: <span id="val-i">0.20</span></div>
      <input type="range" id="slider-i" min="0" max="1" step="0.001" oninput="updateValues()">
    </div>
    <div class="slider-group">
      <div class="slider-label">D-Gain: <span id="val-d">0.0002</span></div>
      <input type="range" id="slider-d" min="0" max="0.01" step="0.00001" oninput="updateValues()">
    </div>
  </div>

  <!-- Flash Button -->
  <button class="btn-commit" onclick="commitToFlash()">Commit to Flash</button>
  <button class="btn-reset"  onclick="resetDefaults()">Reset Defaults</button>

  <div id="status"></div>
</div>

<script>
  const AXES = ['roll', 'pitch', 'yaw'];
  const MODES = ['rate', 'angle'];

  // Nested local cache: gains[axis][mode] = {p, i, d}
  let gains = {};
  let currentAxis = 'roll';
  let currentMode = 'rate';
  let sendTimer = null;

  window.onload = function () {
    fetchPIDs();
  };

  // Pull the full set of gains (all axes, both modes) from the ESP32
  function fetchPIDs() {
    setStatus('Reading from flight controller...', '#00adb5');
    fetch('/getPIDs')
      .then(res => {
        if (!res.ok) throw new Error();
        return res.json();
      })
      .then(data => {
        gains = data;
        renderGains();
        setStatus('Connected.', '#00ff88');
      })
      .catch(() => setStatus('Error connecting to ESP32.', '#ff4444'));
  }

  function selectMode(mode) {
    currentMode = mode;
    document.getElementById('btn-rate').classList.toggle('active', mode === 'rate');
    document.getElementById('btn-angle').classList.toggle('active', mode === 'angle');
    renderGains();
  }

  function selectAxis(axis) {
    currentAxis = axis;
    AXES.forEach(a => {
      document.getElementById('btn-' + a).classList.toggle('active', a === axis);
    });
    renderGains();
  }

  // Push the current axis/mode's gains into the sliders + labels
  function renderGains() {
    let g = (gains[currentAxis] && gains[currentAxis][currentMode]) || { p: 0, i: 0, d: 0 };

    document.getElementById('slider-p').value = g.p;
    document.getElementById('slider-i').value = g.i;
    document.getElementById('slider-d').value = g.d;

    document.getElementById('val-p').innerText = Number(g.p).toFixed(3);
    document.getElementById('val-i').innerText = Number(g.i).toFixed(3);
    document.getElementById('val-d').innerText = Number(g.d).toFixed(5);
  }

  // Called on every slider drag. Updates the label immediately and
  // debounces a live push to the flight controller so we don't flood
  // the ESP32 with a request on every pixel of slider movement.
  function updateValues() {
    let p = parseFloat(document.getElementById('slider-p').value);
    let i = parseFloat(document.getElementById('slider-i').value);
    let d = parseFloat(document.getElementById('slider-d').value);

    document.getElementById('val-p').innerText = p.toFixed(3);
    document.getElementById('val-i').innerText = i.toFixed(3);
    document.getElementById('val-d').innerText = d.toFixed(5);

    if (!gains[currentAxis]) gains[currentAxis] = {};
    gains[currentAxis][currentMode] = { p, i, d };

    clearTimeout(sendTimer);
    sendTimer = setTimeout(() => sendLiveUpdate(p, i, d), 150);
  }

  // Push the active axis/mode's gains to RAM on the ESP32 (not flash yet)
  function sendLiveUpdate(p, i, d) {
    let url = `/setPID?axis=${currentAxis}&mode=${currentMode}&p=${p}&i=${i}&d=${d}`;
    fetch(url)
      .then(res => { if (!res.ok) throw new Error(); })
      .catch(() => setStatus('Failed to reach ESP32.', '#ff4444'));
  }

  // Persist everything currently in RAM (all axes/modes) to NVS flash
  function commitToFlash() {
    setStatus('Saving to flash...', '#00adb5');
    fetch('/commitFlash', { method: 'POST' })
      .then(res => {
        if (!res.ok) throw new Error('HTTP ' + res.status);
        return res.text();
      })
      .then(() => setStatus('Saved to flash!', '#00ff88'))
      .catch(err => {
        console.error('commitToFlash failed:', err);
        setStatus('Failed to save to flash.', '#ff4444');
      });
  }

  // Reset all gains (RAM + flash) to the firmware defaults
  function resetDefaults() {
    if (!confirm('Reset ALL PID gains (rate + angle, all axes) to firmware defaults?')) return;
    setStatus('Resetting...', '#00adb5');
    fetch('/reset', { method: 'POST' })
      .then(res => {
        if (!res.ok) throw new Error('HTTP ' + res.status);
        return res.text();
      })
      .then(() => {
        fetchPIDs();
        setStatus('Reset to defaults.', '#00ff88');
      })
      .catch(err => {
        console.error('resetDefaults failed:', err);
        setStatus('Failed to reset.', '#ff4444');
      });
  }

  function setStatus(text, color) {
    let el = document.getElementById('status');
    if (!el) {
      console.warn('[setStatus] #status element not found in DOM; message was:', text);
      return;
    }
    el.innerText = text;
    el.style.color = color;
  }
</script>
</body>
</html>
)rawliteral";

// ==========================================
// FLASH (NVS) PERSISTENCE
// ==========================================
// Key names here intentionally match loadGainsFromFlash() / resetPIDToDefaults()
// in the main .ino so a commit here is actually readable on next boot.

// Save current active (RAM) PID gains for all axes/modes into flash
void saveGainsToFlash() {
    preferences.begin("pid_gains", false);

    // Roll
    preferences.putFloat("r_kp_rate", rollPID.kp_rate.load(std::memory_order_relaxed));
    preferences.putFloat("r_ki_rate", rollPID.ki_rate.load(std::memory_order_relaxed));
    preferences.putFloat("r_kd_rate", rollPID.kd_rate.load(std::memory_order_relaxed));
    preferences.putFloat("r_kp_ang",  rollPID.kp_angle.load(std::memory_order_relaxed));
    preferences.putFloat("r_ki_ang",  rollPID.ki_angle.load(std::memory_order_relaxed));
    preferences.putFloat("r_kd_ang",  rollPID.kd_angle.load(std::memory_order_relaxed));

    // Pitch
    preferences.putFloat("p_kp_rate", pitchPID.kp_rate.load(std::memory_order_relaxed));
    preferences.putFloat("p_ki_rate", pitchPID.ki_rate.load(std::memory_order_relaxed));
    preferences.putFloat("p_kd_rate", pitchPID.kd_rate.load(std::memory_order_relaxed));
    preferences.putFloat("p_kp_ang",  pitchPID.kp_angle.load(std::memory_order_relaxed));
    preferences.putFloat("p_ki_ang",  pitchPID.ki_angle.load(std::memory_order_relaxed));
    preferences.putFloat("p_kd_ang",  pitchPID.kd_angle.load(std::memory_order_relaxed));

    // Yaw
    preferences.putFloat("y_kp_rate", yawPID.kp_rate.load(std::memory_order_relaxed));
    preferences.putFloat("y_ki_rate", yawPID.ki_rate.load(std::memory_order_relaxed));
    preferences.putFloat("y_kd_rate", yawPID.kd_rate.load(std::memory_order_relaxed));
    preferences.putFloat("y_kp_ang",  yawPID.kp_angle.load(std::memory_order_relaxed));
    preferences.putFloat("y_ki_ang",  yawPID.ki_angle.load(std::memory_order_relaxed));
    preferences.putFloat("y_kd_ang",  yawPID.kd_angle.load(std::memory_order_relaxed));

    preferences.end();
}

// Load saved PID gains from ESP32 Flash Memory on startup
void loadGainsFromFlash() {
    preferences.begin("pid_gains", true); // Open in read-only mode

    // -------------------------------------------------------------------
    // Load Rate Mode Gains
    // -------------------------------------------------------------------
    float r_kp_rate = preferences.getFloat("r_kp_rate", DEFAULT_KP_ROLL_RATE);
    float r_ki_rate = preferences.getFloat("r_ki_rate", DEFAULT_KI_ROLL_RATE);
    float r_kd_rate = preferences.getFloat("r_kd_rate", DEFAULT_KD_ROLL_RATE);

    float p_kp_rate = preferences.getFloat("p_kp_rate", DEFAULT_KP_PITCH_RATE);
    float p_ki_rate = preferences.getFloat("p_ki_rate", DEFAULT_KI_PITCH_RATE);
    float p_kd_rate = preferences.getFloat("p_kd_rate", DEFAULT_KD_PITCH_RATE);

    float y_kp_rate = preferences.getFloat("y_kp_rate", DEFAULT_KP_YAW_RATE);
    float y_ki_rate = preferences.getFloat("y_ki_rate", DEFAULT_KI_YAW_RATE);
    float y_kd_rate = preferences.getFloat("y_kd_rate", DEFAULT_KD_YAW_RATE);

    // -------------------------------------------------------------------
    // Load Angle Mode Gains
    // -------------------------------------------------------------------
    float r_kp_ang = preferences.getFloat("r_kp_ang", DEFAULT_KP_ROLL_ANGLE);
    float r_ki_ang = preferences.getFloat("r_ki_ang", DEFAULT_KI_ROLL_ANGLE);
    float r_kd_ang = preferences.getFloat("r_kd_ang", DEFAULT_KD_ROLL_ANGLE);

    float p_kp_ang = preferences.getFloat("p_kp_ang", DEFAULT_KP_PITCH_ANGLE);
    float p_ki_ang = preferences.getFloat("p_ki_ang", DEFAULT_KI_PITCH_ANGLE);
    float p_kd_ang = preferences.getFloat("p_kd_ang", DEFAULT_KD_PITCH_ANGLE);

    float y_kp_ang = preferences.getFloat("y_kp_ang", DEFAULT_KP_YAW_ANGLE);
    float y_ki_ang = preferences.getFloat("y_ki_ang", DEFAULT_KI_YAW_ANGLE);
    float y_kd_ang = preferences.getFloat("y_kd_ang", DEFAULT_KD_YAW_ANGLE);
    preferences.end(); // Close NVS handle

    // -------------------------------------------------------------------
    // Store loaded values into atomic variables for the flight loop
    // -------------------------------------------------------------------
    // Roll
    rollPID.kp_rate.store(r_kp_rate);
    rollPID.ki_rate.store(r_ki_rate);
    rollPID.kd_rate.store(r_kd_rate);
    rollPID.kp_angle.store(r_kp_ang);
    rollPID.ki_angle.store(r_ki_ang);
    rollPID.kd_angle.store(r_kd_ang);

    // Pitch
    pitchPID.kp_rate.store(p_kp_rate);
    pitchPID.ki_rate.store(p_ki_rate);
    pitchPID.kd_rate.store(p_kd_rate);
    pitchPID.kp_angle.store(p_kp_ang);
    pitchPID.ki_angle.store(p_ki_ang);
    pitchPID.kd_angle.store(p_kd_ang);

    // Yaw
    yawPID.kp_rate.store(y_kp_rate);
    yawPID.ki_rate.store(y_ki_rate);
    yawPID.kd_rate.store(y_kd_rate);
    yawPID.kp_angle.store(y_kp_ang);
    yawPID.ki_angle.store(y_ki_ang);
    yawPID.kd_angle.store(y_kd_ang);
}

void resetPIDToDefaults() {
    preferences.begin("pid_gains", false);
    
    // Rate Mode
    preferences.putFloat("r_kp_rate", DEFAULT_KP_ROLL_RATE);
    preferences.putFloat("r_ki_rate", DEFAULT_KI_ROLL_RATE);
    preferences.putFloat("r_kd_rate", DEFAULT_KD_ROLL_RATE);
    preferences.putFloat("p_kp_rate", DEFAULT_KP_PITCH_RATE);
    preferences.putFloat("p_ki_rate", DEFAULT_KI_PITCH_RATE);
    preferences.putFloat("p_kd_rate", DEFAULT_KD_PITCH_RATE);
    preferences.putFloat("y_kp_rate", DEFAULT_KP_YAW_RATE);
    preferences.putFloat("y_ki_rate", DEFAULT_KI_YAW_RATE);
    preferences.putFloat("y_kd_rate", DEFAULT_KD_YAW_RATE);

    // Angle Mode
    preferences.putFloat("r_kp_ang", DEFAULT_KP_ROLL_ANGLE);
    preferences.putFloat("r_ki_ang", DEFAULT_KI_ROLL_ANGLE);
    preferences.putFloat("r_kd_ang", DEFAULT_KD_ROLL_ANGLE);
    preferences.putFloat("p_kp_ang", DEFAULT_KP_PITCH_ANGLE);
    preferences.putFloat("p_ki_ang", DEFAULT_KI_PITCH_ANGLE);
    preferences.putFloat("p_kd_ang", DEFAULT_KD_PITCH_ANGLE);
    preferences.putFloat("y_kp_ang", DEFAULT_KP_YAW_ANGLE);
    preferences.putFloat("y_ki_ang", DEFAULT_KI_YAW_ANGLE);
    preferences.putFloat("y_kd_ang", DEFAULT_KD_YAW_ANGLE);
    preferences.end();

    // Update active memory
    rollPID.kp_rate.store(DEFAULT_KP_ROLL_RATE);
    rollPID.ki_rate.store(DEFAULT_KI_ROLL_RATE);
    rollPID.kd_rate.store(DEFAULT_KD_ROLL_RATE);
    rollPID.kp_angle.store(DEFAULT_KP_ROLL_ANGLE);
    rollPID.ki_angle.store(DEFAULT_KI_ROLL_ANGLE);
    rollPID.kd_angle.store(DEFAULT_KD_ROLL_ANGLE);

    pitchPID.kp_rate.store(DEFAULT_KP_PITCH_RATE);
    pitchPID.ki_rate.store(DEFAULT_KI_PITCH_RATE);
    pitchPID.kd_rate.store(DEFAULT_KD_PITCH_RATE);
    pitchPID.kp_angle.store(DEFAULT_KP_PITCH_ANGLE);
    pitchPID.ki_angle.store(DEFAULT_KI_PITCH_ANGLE);
    pitchPID.kd_angle.store(DEFAULT_KD_PITCH_ANGLE);

    yawPID.kp_rate.store(DEFAULT_KP_YAW_RATE);
    yawPID.ki_rate.store(DEFAULT_KI_YAW_RATE);
    yawPID.kd_rate.store(DEFAULT_KD_YAW_RATE);
    yawPID.kp_angle.store(DEFAULT_KP_YAW_ANGLE);
    yawPID.ki_angle.store(DEFAULT_KI_YAW_ANGLE);
    yawPID.kd_angle.store(DEFAULT_KD_YAW_ANGLE);

    Serial.println("PID parameters have been reset to default values!");
}



// ==========================================
// WEB SERVER
// ==========================================

// Helper: map an axis name string to its PIDGains struct.
// Returns nullptr if the axis name isn't recognized.
PIDGains* axisFromName(const String &axis) {
    if (axis == "roll")  return &rollPID;
    if (axis == "pitch") return &pitchPID;
    if (axis == "yaw")   return &yawPID;
    return nullptr;
}


void initWebServer() {
  // Start Wi-Fi Access Point
  WiFi.softAP(ssid, password);
  Serial.print("\n\nAccess Point IP: ");    //192.168.4.1
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send_P(200, "text/html", index_html);
  });

  // Return every axis/mode gain currently held in RAM, e.g.:
  // { "roll": { "rate": {"p":.., "i":.., "d":..}, "angle": {...} }, "pitch": {...}, "yaw": {...} }
  server.on("/getPIDs", HTTP_GET, [](AsyncWebServerRequest *request){
      JsonDocument doc;

      PIDGains* axes[3]    = { &rollPID, &pitchPID, &yawPID };
      const char* names[3] = { "roll", "pitch", "yaw" };

      for (int a = 0; a < 3; a++) {
          PIDGains* pid = axes[a];
          doc[names[a]]["rate"]["p"]  = pid->kp_rate.load(std::memory_order_relaxed);
          doc[names[a]]["rate"]["i"]  = pid->ki_rate.load(std::memory_order_relaxed);
          doc[names[a]]["rate"]["d"]  = pid->kd_rate.load(std::memory_order_relaxed);
          doc[names[a]]["angle"]["p"] = pid->kp_angle.load(std::memory_order_relaxed);
          doc[names[a]]["angle"]["i"] = pid->ki_angle.load(std::memory_order_relaxed);
          doc[names[a]]["angle"]["d"] = pid->kd_angle.load(std::memory_order_relaxed);
      }

      String response;
      serializeJson(doc, response);
      Serial.print("GET /getPIDs -> ");
      Serial.println(response);

      AsyncWebServerResponse *httpResponse = request->beginResponse(200, "application/json", response);
      httpResponse->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
      request->send(httpResponse);
  });

  // Live-update a single axis/mode's gains in RAM (does NOT touch flash).
  // /setPID?axis=roll&mode=rate&p=0.15&i=0.2&d=0.0002
  server.on("/setPID", HTTP_GET, [](AsyncWebServerRequest *request){
      if (!(request->hasParam("axis") && request->hasParam("mode") &&
            request->hasParam("p") && request->hasParam("i") && request->hasParam("d"))) {
          request->send(400, "text/plain", "Missing parameters");
          return;
      }

      String axis = request->getParam("axis")->value();
      String mode = request->getParam("mode")->value();
      float p = request->getParam("p")->value().toFloat();
      float i = request->getParam("i")->value().toFloat();
      float d = request->getParam("d")->value().toFloat();

      PIDGains* target = axisFromName(axis);
      if (!target) {
          request->send(400, "text/plain", "Invalid axis");
          return;
      }

      Serial.printf("GET /setPID axis=%s mode=%s p=%.5f i=%.5f d=%.5f\n",
              axis.c_str(), mode.c_str(), p, i, d);

      if (mode == "rate") {
          target->kp_rate.store(p, std::memory_order_relaxed);
          target->ki_rate.store(i, std::memory_order_relaxed);
          target->kd_rate.store(d, std::memory_order_relaxed);
      } else if (mode == "angle") {
          target->kp_angle.store(p, std::memory_order_relaxed);
          target->ki_angle.store(i, std::memory_order_relaxed);
          target->kd_angle.store(d, std::memory_order_relaxed);
      } else {
          request->send(400, "text/plain", "Invalid mode");
          return;
      }

      request->send(200, "text/plain", "OK");
  });

  // Persist everything currently in RAM (all axes, both modes) to NVS flash
  server.on("/commitFlash", HTTP_POST, [](AsyncWebServerRequest *request){
      Serial.println("POST /commitFlash received - saving all gains to NVS...");
      saveGainsToFlash();
      Serial.println("saveGainsToFlash() complete.");
      request->send(200, "text/plain", "Gains Saved!");
  });

  // Reset all gains (RAM + flash) back to firmware defaults
  server.on("/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
    Serial.println("POST /reset received - resetting all gains to defaults...");
    resetPIDToDefaults();
    request->send(200, "text/plain", "OK");
  });

  server.begin();
}

