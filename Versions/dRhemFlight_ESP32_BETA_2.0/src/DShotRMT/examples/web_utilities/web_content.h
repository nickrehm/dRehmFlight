/**
 * @file web_content.h
 * @brief Contains the HTML, CSS, and JavaScript for the main DShot control web page.
 * @author Wastl Kraus
 * @date 2025-10-04
 * @license MIT
 */

#pragma once

// Web Site Content
const char *index_html = R"rawliteral(
<!DOCTYPE html>
<html lang="de">

<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>DShotRMT Web Client</title>
    <style>
        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
            display: flex;
            flex-direction: column;
            align-items: center;
            background-color: #2c3e50;
            color: #ecf0f1;
            margin: 0;
            height: 100dvh;
            justify-content: center;
        }

        h1 {
            font-size: 1.5em;
            font-weight: bold;
            margin-bottom: 20px;
        }

        .control-container {
            background-color: #34495e;
            padding: 30px;
            border-radius: 12px;
            box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
            text-align: center;
            width: 85%;
            max-width: 500px;
        }

        /* Arming Switch Styles */
        .arming-section {
            margin-bottom: 25px;
            padding: 15px;
            background-color: #2c3e50;
            border-radius: 8px;
            border: 2px solid #e74c3c;
        }

        .arming-switch {
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 15px;
            margin-bottom: 10px;
        }

        .switch {
            position: relative;
            display: inline-block;
            width: 60px;
            height: 34px;
        }

        .switch input {
            opacity: 0;
            width: 0;
            height: 0;
        }

        .slider-switch {
            position: absolute;
            cursor: pointer;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background-color: #e74c3c;
            -webkit-transition: .4s;
            transition: .4s;
            border-radius: 34px;
        }

        .slider-switch:before {
            position: absolute;
            content: "";
            height: 26px;
            width: 26px;
            left: 4px;
            bottom: 4px;
            background-color: white;
            -webkit-transition: .4s;
            transition: .4s;
            border-radius: 50%;
        }

        input:checked+.slider-switch {
            background-color: #27ae60;
        }

        input:checked+.slider-switch:before {
            -webkit-transform: translateX(26px);
            -ms-transform: translateX(26px);
            transform: translateX(26px);
        }

        .arming-label {
            font-size: 1.2em;
            font-weight: bold;
        }

        .arming-status {
            font-size: 0.9em;
            margin-top: 5px;
        }

        .status-disarmed {
            color: #e74c3c;
        }

        .status-armed {
            color: #27ae60;
        }

        /* Throttle Section */
        .throttle-section {
            opacity: 0.3;
            transition: opacity 0.3s ease;
        }

        .throttle-section.armed {
            opacity: 1;
        }

        #throttleValue {
            font-size: 2.5em;
            font-weight: bold;
            color: #3498db;
            margin-bottom: 20px;
        }

        #throttleSlider {
            appearance: none;
            width: 100%;
            height: 25px;
            background: #2c3e50;
            outline: none;
            border-radius: 12px;
        }

        #throttleSlider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 40px;
            height: 40px;
            background: #3498db;
            cursor: pointer;
            border-radius: 50%;
        }

        #throttleSlider::-moz-range-thumb {
            width: 40px;
            height: 40px;
            background: #3498db;
            cursor: pointer;
            border-radius: 50%;
            border: none;
        }

        .stats {
            margin-top: 20px;
            font-size: 1.2em;
        }

        .stats span {
            font-weight: bold;
            color: #e67e22;
        }

        .warning-text {
            color: #e74c3c;
            font-size: 0.9em;
            margin-top: 10px;
            font-style: italic;
        }
    </style>
</head>

<body>
    <h1>DShotRMT Web Client</h1>
    <div class="control-container">
        <!-- Arming Section -->
        <div class="arming-section">
            <div class="arming-switch">
                <span class="arming-label">ARMING SWITCH</span>
                <label class="switch">
                    <input type="checkbox" id="armingSwitch">
                    <span class="slider-switch"></span>
                </label>
            </div>
            <div class="arming-status">
                <span id="armingStatus" class="status-disarmed">DISARMED</span>
            </div>
            <div class="warning-text">
                ⚠️ Disabled when disarmed!
            </div>
        </div>

        <!-- Throttle Section -->
        <div class="throttle-section" id="throttleSection">
            <div id="throttleValue">0</div>
            <div id="throttlePercent" style="font-size: 1.2rem; color: #bdc3c7; margin-top: -15px; margin-bottom: 15px;">0%</div>
            <input type="range" min="48" max="2047" value="0" id="throttleSlider" disabled>
        </div>

        <div class="stats">
            RPM: <span id="rpmValue">--</span>
        </div>
    </div>

    <script>
        const gateway = `ws://${window.location.hostname}/ws`;
        let websocket;
        let isArmed = false;

        // Init WebSocket
        window.addEventListener('load', () => {
            initWebSocket();
        });

        function initWebSocket() {
            console.log('Trying to open a WebSocket connection...');

            websocket = new WebSocket(gateway);
            websocket.onopen = onOpen;
            websocket.onclose = onClose;
            websocket.onmessage = onMessage;
        }

        function onOpen(event) {
            console.log('Connection opened');
        }

        function onClose(event) {
            console.log('Connection closed');
            setTimeout(initWebSocket, 2000);
        }

        function updateThrottleDisplays(rawValue) {
            const DSHOT_MIN = 48;
            const DSHOT_MAX = 2047;
            const clampedValue = Math.max(0, Math.min(DSHOT_MAX, rawValue));
            document.getElementById('throttleValue').innerText = clampedValue;
            let percent = 0;
            if (clampedValue > 0) {
                percent = (clampedValue - DSHOT_MIN) / (DSHOT_MAX - DSHOT_MIN) * 100;
            }
            document.getElementById('throttlePercent').innerText = Math.round(percent) + '%';
            document.getElementById('throttleSlider').value = clampedValue;
        }

        // Getting data from sketch
        function onMessage(event) {
            try {
                const data = JSON.parse(event.data);

                if (data.rpm !== undefined) {
                    document.getElementById('rpmValue').innerText = data.rpm;
                }

                // Sync web and serial throttle inputs
                if (data.throttle !== undefined) {
                    if (isArmed) {
                        updateThrottleDisplays(data.throttle);
                    }
                }

                // Sync arming status if received from ESP32
                if (data.armed !== undefined) {
                    isArmed = data.armed;
                    updateArmingUI();
                }

            } catch (e) {
                console.error("Error parsing JSON: ", e);
            }
        }

        // Elements
        const slider = document.getElementById('throttleSlider');
        const sliderValue = document.getElementById('throttleValue');
        const armingSwitch = document.getElementById('armingSwitch');
        const armingStatus = document.getElementById('armingStatus');
        const throttleSection = document.getElementById('throttleSection');

        // Arming switch event
        armingSwitch.addEventListener('change', () => {
            isArmed = armingSwitch.checked;
            updateArmingUI();

            // Send arming status to ESP32
            const message = JSON.stringify({
                "armed": isArmed,
                "throttle": isArmed ? parseInt(slider.value) : 0
            });

            console.log("Sending arming status: ", message);
            websocket.send(message);

            // If disarmed, set throttle to 0
            if (!isArmed) {
                updateThrottleDisplays(0);
            }
        });

        // Update UI based on arming status
        function updateArmingUI() {
            
            // Synch checkbox, as well
            armingSwitch.checked = isArmed;

            if (isArmed) {
                armingStatus.innerText = 'ARMED';
                armingStatus.className = 'status-armed';
                throttleSection.classList.add('armed');
                slider.disabled = false;
            } else {
                armingStatus.innerText = 'DISARMED';
                armingStatus.className = 'status-disarmed';
                throttleSection.classList.remove('armed');
                slider.disabled = true;
                updateThrottleDisplays(0);
            }
        }

        // Throttle slider event
        slider.addEventListener('input', () => {
            if (!isArmed) {
                slider.disabled = true;
                updateThrottleDisplays(0);
                return;
            }

            const throttle = slider.value;
            updateThrottleDisplays(throttle);

            const message = JSON.stringify({
                "throttle": parseInt(throttle),
                "armed": isArmed
            });

            console.log("Sending throttle: ", message);
            websocket.send(message);
        });

        // Initialize UI
        updateArmingUI();
    </script>
</body>

</html>
)rawliteral";
