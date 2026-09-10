/**
 * @file dshot_definitions.h
 * @brief Defines DShot protocol constants, data structures, and command enums for DShotRMT library
 * @author Wastl Kraus
 * @date 2025-11-29
 * @license MIT
 */

#pragma once

#include <cstdint>
#include <driver/rmt_common.h>

// =================================================================================
// DShot Protocol Constants
// =================================================================================

// --- Frame Structure ---
static constexpr uint16_t DSHOT_BITS_PER_FRAME = 16;
static constexpr uint16_t DSHOT_TELEMETRY_BIT_POSITION = 11;
static constexpr uint16_t DSHOT_CRC_BIT_SHIFT = 4;
static constexpr uint16_t DSHOT_CRC_MASK = 0x000F;

// --- Throttle & Command Values ---
static constexpr uint16_t DSHOT_THROTTLE_MAX = 2047;     // Maximum throttle value (0-2047)
static constexpr uint16_t DSHOT_THROTTLE_MIN = 48;       // Minimum throttle value for motor spin
static constexpr float DSHOT_PERCENT_MIN = 0.0f;
static constexpr float DSHOT_PERCENT_MAX = 100.0f;
static constexpr uint16_t DSHOT_CMD_MIN = 0;             // Minimum command value
static constexpr uint16_t DSHOT_CMD_MAX = 47;            // Maximum command value
static constexpr uint16_t DSHOT_NULL_PACKET = 0;
static constexpr uint16_t DSHOT_FULL_PACKET = 0xFFFF;
static constexpr uint16_t DSHOT_TELEMETRY_INVALID = DSHOT_THROTTLE_MAX;
static constexpr uint16_t NO_DSHOT_TELEMETRY = 0;

// --- Command Behavior ---
static constexpr uint16_t DEFAULT_CMD_DELAY_US = 10;
static constexpr uint16_t DEFAULT_CMD_REPEAT_COUNT = 1;
static constexpr uint16_t SETTINGS_COMMAND_REPEATS = 10;
static constexpr uint16_t SETTINGS_COMMAND_DELAY_US = 5;


// =================================================================================
// DShot Telemetry Constants
// =================================================================================

// --- GCR Frame Structure ---
static constexpr uint16_t DSHOT_ERPM_FRAME_GCR_BITS = 21;      // GCR bits in a DShot answer frame for eRPM
static constexpr uint16_t DSHOT_TELEMETRY_FULL_GCR_BITS = 110; // GCR bits for a full 10-byte telemetry frame (80 data bits + 8 CRC bits = 88 bits, 88 * 5/4 = 110 GCR bits)

// --- Telemetry Payload Structure ---
static constexpr uint16_t DSHOT_TELEMETRY_FRAME_LENGTH_BITS = 80; // 10 bytes * 8 bits/byte
static constexpr uint16_t DSHOT_TELEMETRY_FRAME_LENGTH_BYTES = 10;
static constexpr uint16_t DSHOT_TELEMETRY_CRC_LENGTH_BITS = 8;    // 8-bit CRC for telemetry

// --- Motor Properties for RPM Calculation ---
static constexpr uint16_t DEFAULT_MOTOR_MAGNET_COUNT = 14;
static constexpr uint16_t POLE_PAIRS_MIN = 1;
static constexpr uint16_t MAGNETS_PER_POLE_PAIR = 2;


// =================================================================================
// DShot Timing Constants (Protocol Level)
// =================================================================================
static constexpr uint16_t DSHOT_PADDING_US = 20; // Pause between frames

// Defines the bit length and high time for a '1' bit in microseconds for each DShot mode.
typedef struct dshot_timing
{
    double bit_length_us; // Total duration of one bit in microseconds.
    double t1h_lenght_us; // High time duration for a '1' bit in microseconds.
} dshot_timing_us_t;

// Timing parameters for each DShot mode
const dshot_timing_us_t DSHOT_TIMING_US[] = {
    {0.00, 0.00}, // DSHOT_OFF
    {6.67, 5.00}, // DSHOT150
    {3.33, 2.50}, // DSHOT300
    {1.67, 1.25}, // DSHOT600
    {0.83, 0.67}  // DSHOT1200
};


// =================================================================================
// ESP32 RMT Hardware Implementation Constants
// =================================================================================

// --- RMT Clock & Buffer Configuration ---
static constexpr auto DSHOT_CLOCK_SRC_DEFAULT = RMT_CLK_SRC_DEFAULT;
static constexpr auto DSHOT_RMT_RESOLUTION = 8000000;                    // 8 MHz resolution
static constexpr auto RMT_TICKS_PER_US = DSHOT_RMT_RESOLUTION / 1000000; // RMT Ticks per microsecond
static constexpr auto RMT_TX_BUFFER_SYMBOLS = 48;
static constexpr auto RMT_RX_BUFFER_SYMBOLS = DSHOT_TELEMETRY_FULL_GCR_BITS;
static constexpr auto RMT_QUEUE_DEPTH = 1;

// --- RMT Receiver Configuration ---
static constexpr uint32_t DSHOT_PULSE_MIN_NS = 800;  // 0.8us minimum pulse duration for receiver
static constexpr uint32_t DSHOT_PULSE_MAX_NS = 8000; // 8.0us maximum pulse duration for receiver

// Stores pre-calculated timing values in RMT ticks for efficient signal generation.
typedef struct rmt_ticks
{
    uint16_t bit_length_ticks; // Total duration of one bit in RMT ticks.
    uint16_t t1h_ticks;        // High time duration for a '1' bit in RMT ticks.
    uint16_t t1l_ticks;        // Low time duration for a '1' bit in RMT ticks.
    uint16_t t0h_ticks;        // High time duration for a '0' bit in RMT ticks.
    uint16_t t0l_ticks;        // Low time duration for a '0' bit in RMT ticks.
} rmt_ticks_t;


// =================================================================================
// Library Data Structures & Enums
// =================================================================================

// --- DShot Modes ---
enum dshot_mode_t : uint8_t
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
};

// --- DShot Packet Structure ---
// Represents the 16-bit DShot data packet sent to the ESC.
typedef struct dshot_packet
{
    uint16_t throttle_value : 11; // 11-bit throttle value or command.
    bool telemetric_request : 1;  // 1-bit telemetry request flag.
    uint16_t checksum : 4;        // 4-bit CRC checksum.
} dshot_packet_t;

// --- Telemetry Data Structure ---
// Structure for decoded DShot telemetry data (from ESC)
typedef struct dshot_telemetry_data
{
    uint16_t rpm;         // Motor RPM
    uint16_t voltage;     // Voltage in mV
    uint16_t current;     // Current in mA
    uint16_t consumption; // Consumption in mAh
    int8_t temperature;   // Temperature in Celsius
    uint8_t errors;       // Error flags / count
} dshot_telemetry_data_t;

// --- Library Result Codes & Structure ---
enum dshot_msg_code_t
{
    DSHOT_NONE = 0,
    DSHOT_UNKNOWN,
    DSHOT_TX_INIT_FAILED,
    DSHOT_RX_INIT_FAILED,
    DSHOT_ENCODER_INIT_FAILED,
    DSHOT_CALLBACK_REGISTERING_FAILED,
    DSHOT_RECEIVER_FAILED,
    DSHOT_TRANSMISSION_FAILED,
    DSHOT_THROTTLE_NOT_IN_RANGE,
    DSHOT_PERCENT_NOT_IN_RANGE,
    DSHOT_COMMAND_NOT_VALID,
    DSHOT_BIDIR_NOT_ENABLED,
    DSHOT_TELEMETRY_FAILED,
    DSHOT_INVALID_MAGNET_COUNT,
    DSHOT_INVALID_COMMAND,
    DSHOT_TIMING_CORRECTION,
    DSHOT_INIT_FAILED,
    DSHOT_INIT_SUCCESS,
    DSHOT_TX_INIT_SUCCESS,
    DSHOT_RX_INIT_SUCCESS,
    DSHOT_ENCODER_INIT_SUCCESS,
    DSHOT_ENCODING_SUCCESS,
    DSHOT_TRANSMISSION_SUCCESS,
    DSHOT_TELEMETRY_SUCCESS,
    DSHOT_TELEMETRY_DATA_AVAILABLE,
    DSHOT_COMMAND_SUCCESS
};

// Contains the success status, an error code, and optional telemetry data.
typedef struct dshot_result
{
    bool success;
    dshot_msg_code_t result_code;          // Specific error or success code.
    uint16_t erpm;                         // Electrical RPM (eRPM) if telemetry is successful.
    uint16_t motor_rpm;                    // Motor RPM if telemetry is successful and magnet count is known.
    dshot_telemetry_data_t telemetry_data; // Full telemetry data if available
    bool telemetry_available;              // Flag to indicate if telemetry_data is valid
} dshot_result_t;

// --- DShot Commands ---
// Standard DShot commands by "betaflight"
enum dshotCommands_e
{
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO,
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST,
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE,
    DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON,
    DSHOT_CMD_LED1_ON,
    DSHOT_CMD_LED2_ON,
    DSHOT_CMD_LED3_ON,
    DSHOT_CMD_LED0_OFF,
    DSHOT_CMD_LED1_OFF,
    DSHOT_CMD_LED2_OFF,
    DSHOT_CMD_LED3_OFF,
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30,
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31,
    DSHOT_CMD_MAX_VALUE = 47
};

// --- General Status & Helper Constants ---
static constexpr int DSHOT_OK = 0;
static constexpr float CONVERSION_FACTOR_MILLI_TO_UNITS = 1000.0f;