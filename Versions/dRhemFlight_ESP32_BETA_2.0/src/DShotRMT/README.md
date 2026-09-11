# DShotRMT - ESP32 RMT DShot Driver

[![Arduino CI](https://github.com/derdoktor667/DShotRMT/actions/workflows/ci.yml/badge.svg)](https://github.com/derdoktor667/DShotRMT/actions/workflows/ci.yml)
[![Arduino Library](https://img.shields.io/badge/Arduino-Library-blue.svg)](https://www.arduinolibraries.com/libraries/dshot-rmt)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

An Arduino IDE library for generating DShot signals on ESP32 microcontrollers using the **latest ESP-IDF 5.5 RMT Encoder API** (`rmt_tx.h` / `rmt_rx.h`). This library specifically leverages the official `rmt_bytes_encoder` API for an efficient, hardware-timed and maintainable implementation. It provides a simple way to control BLHeli ESCs in both Arduino and ESP-IDF projects.

### ✨ Experimental Bidirectional DShot Support Activated! ✨

> [!CAUTION]
> **This feature is currently EXPERIMENTAL and under active development.**
> If you enable bidirectional DShot, you **MUST** connect an external pull-up resistor (e.g., 2k Ohm to 3.3V) to the DShot GPIO pin. This resistor is absolutely crucial for the ESC to properly send telemetry data back to the ESP32. Without it, bidirectional telemetry will **NOT** function correctly. Use at your own risk.

 The legacy version using the old `rmt.h` API is available in the `oldAPI` branch.

---

### DShot300 Example Output

Here's an example of the output from the `dshot300` example sketch, now showing full telemetry:

![DShot300 Example Output](img/dshot300.png)

## 🚀 Core Features

- **Multiple DShot Modes:** Supports DSHOT150, DSHOT300, DSHOT600, and DSHOT1200.
- **Robust Bidirectional DShot Support:** Now features full GCR-dekodierte telemetry data (temperature, voltage, current, consumption, and RPM) from the ESC. The library automatically differentiates between eRPM-only and full telemetry frames. This significantly enhances feedback capabilities for advanced applications.
- **Hardware-Timed Signals:** Precise signal generation using the ESP32 RMT peripheral, ensuring stable and reliable motor control.
- **Simple API:** Easy-to-use C++ class with intuitive methods like `sendThrottlePercent()`.
- **Enhanced Error Handling:** Provides detailed feedback on operation success or failure via an enhanced `dshot_result_t` struct, now including specific error codes, eRPM data, and a `dshot_telemetry_data_t` struct for full GCR-decoded telemetry.
- **Lightweight:** The core library has no external dependencies.
- **Arduino and ESP-IDF Compatible:** Can be used in both Arduino and ESP-IDF projects.

## How it Works

The library is architected around a single C++ class, `DShotRMT`. It abstracts the ESP32's RMT (Remote Control) peripheral, which is a hardware timer peripheral capable of generating and receiving precisely timed signals. For a more detailed explanation of the DShot protocol, refer to this excellent article: [DShot and Bidirectional DShot](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/).

1.  **Signal Generation (TX):** The library uses an RMT 'bytes_encoder'. This encoder is configured with the specific pulse durations for DShot '0' and '1' bits based on the selected speed (e.g., DSHOT300, DSHOT600). When a user calls `sendThrottle()`, the library constructs a 16-bit DShot frame (11-bit throttle, 1-bit telemetry request, 4-bit CRC) and hands it to the RMT encoder. The RMT hardware then autonomously generates the correct electrical signal on the specified GPIO pin.

2.  **Bidirectional Telemetry (RX) - Now with Full GCR Telemetry:** **Note: For bidirectional DShot, an external pull-up resistor (e.g., 2k Ohm to 3.3V) is required on the DShot GPIO pin for proper telemetry reception.** For bidirectional communication, the library configures a second RMT channel in receive mode on the same GPIO. An interrupt service routine (`_on_rx_done`) is registered. When the ESC sends back a telemetry signal, the RMT peripheral captures it. The interrupt code intelligently differentiates between eRPM-only frames (21 GCR bits) and full telemetry frames (110 GCR bits). It then decodes the GCR-encoded signal (including 5B/4B GCR decoding for full telemetry), validates its CRC, and stores the resulting eRPM value or full telemetry data (temperature, voltage, current, consumption, RPM) in thread-safe `atomic` variables. The main application can then poll for this data using the `getTelemetry()` method, which now returns a comprehensive `dshot_result_t` with all available telemetry fields.

## ⏱️ DShot Timing Information

The DShot protocol defines specific timing characteristics for each mode. The following table outlines the bit length, T1H (high time for a '1' bit), T0H (high time for a '0' bit), and frame length for the supported DShot modes:

| DShot Mode | Bit Length (µs) | T1H Length (µs) | T0H Length (µs) | Frame Length (µs) |
| :--------- | :-------------- | :-------------- | :-------------- | :---------------- |
| DSHOT150   | 6.67            | 5.00            | 2.50            | 106.72            |
| DSHOT300   | 3.33            | 2.50            | 1.25            | 53.28             |
| DSHOT600   | 1.67            | 1.25            | 0.625           | 26.72             |
| DSHOT1200  | 0.83            | 0.67            | 0.335           | 13.28             |

## 📦 Installation

### Arduino IDE

1.  Open the Arduino Library Manager (`Sketch` > `Include Library` > `Manage Libraries...`).
2.  Search for "DShotRMT" and click "Install".
3.  Alternatively, you can clone this repository or download it as a ZIP file and place it in your Arduino libraries folder (`~/Arduino/libraries/DShotRMT/`).

## ⚡ Quick Start

Here's a basic example of how to use the `DShotRMT` library to control a motor. Note that `DShotRMT.h` now includes all necessary dependencies, so you only need to include this single header. Please use example sketches for more detailes:

```cpp
#include <Arduino.h>
#include <DShotRMT.h>

// Define the GPIO pin connected to the motor ESC
const gpio_num_t MOTOR_PIN = GPIO_NUM_27;

// Create a DShotRMT instance for DSHOT300 with bidirectional telemetry enabled
DShotRMT motor(MOTOR_PIN, DSHOT300, true);

void setup() {
  Serial.begin(115200);

  // Initialize the DShot motor
  motor.begin();

  // Print CPU Info
  printCpuInfo(Serial);

  Serial.println("Motor initialized. Ramping up to 25% throttle...");
  }

void loop() {
  // Ramp up to 25% throttle over 2.5 seconds
  for (int i = 0; i <= 25; i++) {
    motor.sendThrottlePercent(i);
    delay(200);
  }
  
  Serial.println("Stopping motor.");
  motor.sendThrottlePercent(0);

  // Print DShot Info, which now includes detailed telemetry
  printDShotInfo(motor, Serial);

  // Take a break before next bench run
  delay(3000);
}
```

## 🎮 Examples

The `examples` folder contains more advanced examples:

- **`throttle_percent`:** A focused example showing how to control motor speed using percentage values (0-100) via the serial monitor.
- **`dshot300`:** A more advanced example demonstrating how to send raw DShot commands and **receive comprehensive telemetry** via the serial monitor.
- **`web_control`:** A full-featured web application for controlling a motor from a web browser. It creates a WiFi access point and serves a web page with a throttle slider and arming switch.
- **`web_client`:** A variation of the `web_control` example that connects to an existing WiFi network instead of creating its own access point.

### Dependencies for Web Examples

The `web_control` and `web_client` examples require the following additional libraries:

- [ArduinoJson](https://github.com/bblanchon/ArduinoJson)
- [ESPAsyncWebServer](https://github.com/ESP32Async/ESPAsyncWebServer)
- [AsyncTCP](https://github.com/ESP32Async/AsyncTCP)

## 📚 API Reference

The main class is `DShotRMT`. Here are the most important methods:

- `DShotRMT(gpio_num_t gpio, dshot_mode_t mode = DSHOT300, bool is_bidirectional = false, uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT)`: Constructor to create a new DShotRMT instance.
- `begin()`: Initializes the DShot RMT channels and encoder.
- `sendThrottlePercent(float percent)`: Sends a throttle value as a percentage (0.0-100.0) to the ESC.
- `sendThrottle(uint16_t throttle)`: Sends a raw throttle value (48-2047) to the ESC. A value of 0 sends a motor stop command.
- `sendCommand(dshotCommands_e command)`: Sends a DShot command to the ESC. Automatically handles repetitions and delays for specific commands (e.g., `DSHOT_CMD_SAVE_SETTINGS`).
- `sendCommand(dshotCommands_e command, uint16_t repeat_count, uint16_t delay_us)`: Sends a DShot command to the ESC with a specified repeat count and delay. This is a blocking function.
- `sendCommand(uint16_t command_value)`: Sends a DShot command to the ESC by accepting an integer value. It validates the input and then calls `sendCommand(dshotCommands_e command)`.
- `sendCustomCommand(uint16_t command_value, uint16_t repeat_count, uint16_t delay_us)`: Sends a custom DShot command to the ESC. Advanced feature, use with caution.
- `getTelemetry()`: Retrieves telemetry data from the ESC. If bidirectional DShot is enabled, this function now returns a comprehensive `dshot_result_t` containing both eRPM and a fully GCR-decoded `dshot_telemetry_data_t` struct (temperature, voltage, current, consumption, RPM) if available.
- `setMotorSpinDirection(bool reversed)`: Sets the motor spin direction. `true` for reversed, `false` for normal.
- `saveESCSettings()`: Sends a command to the ESC to save its current settings. Use with caution as this writes to ESC's non-volatile memory.
- `getMode()`: Gets the current DShot mode.
- `isBidirectional()`: Checks if bidirectional DShot is enabled.
- `getThrottleValue()`: Gets the last transmitted throttle value.
- `getEncodedFrameValue()`: Gets the last encoded DShot frame value.

## ⚙️ ESP-IDF Integration

This library is built upon the ESP-IDF framework, specifically leveraging its RMT (Remote Control Peripheral) module for precise signal generation. The library is tested with **ESP-IDF v5.5.1** and makes extensive use of its modern RMT APIs. For detailed information on the underlying ESP-IDF components and their usage, please refer to the official ESP-IDF documentation:

*   [ESP-IDF v5.5.1 Documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.1/)

---

## 🤝 Contributing

Contributions are welcome! Please fork the repository, create a feature branch, and submit a pull request.

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
