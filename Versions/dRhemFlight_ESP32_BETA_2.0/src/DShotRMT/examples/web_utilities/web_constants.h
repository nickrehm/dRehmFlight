/**
 * @file web_constants.h
 * @brief Shared constants for DShotRMT web examples.
 * @author Wastl Kraus
 * @date 2025-11-29
 * @license MIT
 */

#pragma once

// ============================================================================
// Serial Configuration & Commands
// ============================================================================
static constexpr auto& USB_SERIAL = Serial;
static constexpr auto USB_SERIAL_BAUD = 115200;

// Serial command strings
namespace SerialCmd {
    constexpr char ARM[] = "arm";
    constexpr char DISARM[] = "disarm";
    constexpr char STOP[] = "0";
    constexpr char INFO[] = "info";
    constexpr char RPM[] = "rpm";
    constexpr char CMD_PREFIX[] = "cmd ";
    constexpr char HELP[] = "h";
    constexpr char HELP_ALT[] = "help";
    constexpr char STATUS[] = "status";
    constexpr char WIFI_STATUS[] = "wifi";
    constexpr char RECONNECT[] = "reconnect";
    constexpr char OTA_INFO[] = "ota";
}

// ============================================================================
// Motor & DShot Configuration
// ============================================================================
static constexpr gpio_num_t MOTOR01_PIN = GPIO_NUM_27;
static constexpr dshot_mode_t DSHOT_MODE = DSHOT300;
static constexpr bool IS_BIDIRECTIONAL = false;
static constexpr uint16_t MOTOR01_MAGNET_COUNT = 14;

// ============================================================================
// Web Server & WebSocket Configuration
// ============================================================================
static constexpr uint16_t WEBSERVER_PORT = 80;
static constexpr char WEBSOCKET_PATH[] = "/ws";

// Keys for JSON communication
namespace JsonKey {
    constexpr char THROTTLE[] = "throttle";
    constexpr char ARMED[] = "armed";
    constexpr char RPM[] = "rpm";
}

// ============================================================================
// Timing Intervals
// ============================================================================
static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000; // 20 seconds
static constexpr uint64_t MOTOR_STATS_UPDATE_INTERVAL_US = 3000000; // 3 seconds
static constexpr uint64_t WIFI_RECONNECT_CHECK_INTERVAL_US = 30000000; // 30 seconds
static constexpr uint32_t WEBSOCKET_RECONNECT_DELAY_MS = 2000; // 2 seconds
static constexpr uint32_t SERIAL_RAMP_DELAY_MS = 200; // 200 ms
static constexpr uint32_t OTA_PROGRESS_UPDATE_INTERVAL_MS = 2000; // 2 seconds
