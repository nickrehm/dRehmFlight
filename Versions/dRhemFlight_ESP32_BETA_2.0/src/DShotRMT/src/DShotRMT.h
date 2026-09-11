/**
 * @file DShotRMT.h
 * @brief DShot signal generation using ESP32 RMT with bidirectional support
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#pragma once

#include <atomic>
#include <Arduino.h>

#include <driver/gpio.h>
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>

#include "dshot_definitions.h"
#include "dshot_init.h"

// DShotRMT Library Version
static constexpr uint8_t DSHOTRMT_MAJOR_VERSION = 0;
static constexpr uint8_t DSHOTRMT_MINOR_VERSION = 9;
static constexpr uint8_t DSHOTRMT_PATCH_VERSION = 5;

// DShot Protocol Constants
static constexpr auto DSHOT_THROTTLE_FAILSAFE = 0;

// DShotRMT class for generating DShot signals and receiving telemetry.
class DShotRMT
{
public:
    // Constructs a new DShotRMT object using a GPIO pin.
    DShotRMT(gpio_num_t gpio, dshot_mode_t mode = DSHOT300, bool is_bidirectional = false, uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT);

    // Constructs a new DShotRMT object using an Arduino-style integer pin number.
    DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional = false, uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT);

    // Destructor
    ~DShotRMT();

    // Initialize DShotRMT
    dshot_result_t begin();

    // Sends a raw throttle value to the ESC.
    dshot_result_t sendThrottle(uint16_t throttle);

    // Sends a throttle value as a percentage to the ESC.
    dshot_result_t sendThrottlePercent(float percent);

    // Sends a DShot command to the ESC by accepting an integer value.
    dshot_result_t sendCommand(uint16_t command_value);

    // Sends a DShot command to the ESC.
    dshot_result_t sendCommand(dshotCommands_e command);

    // Sends a DShot command to the ESC with a specified repeat count and delay.
    dshot_result_t sendCommand(dshotCommands_e command, uint16_t repeat_count, uint16_t delay_us);

    /**
     * @brief Sends a custom DShot command to the ESC. Advanced feature, use with caution.
     * @param command_value The raw command value (0-47).
     * @param repeat_count The number of times to send the command.
     * @param delay_us The delay in microseconds between repetitions.
     * @return dshot_result_t The result of the operation.
     */
    dshot_result_t sendCustomCommand(uint16_t command_value, uint16_t repeat_count, uint16_t delay_us);

    // Retrieves telemetry data from the ESC.
    dshot_result_t getTelemetry();

    // Sets the motor spin direction.
    dshot_result_t setMotorSpinDirection(bool reversed);

    // Sends a command to the ESC to save its current settings.
    dshot_result_t saveESCSettings();

    // Getters for DShot info
    dshot_mode_t getMode() const { return _mode; }
    bool isBidirectional() const { return _is_bidirectional; }
    uint16_t getThrottleValue() const { return _last_throttle; }
    uint16_t getEncodedFrameValue() const { return _encoded_frame_value; }

private:
    dshot_result_t _sendRawDshotFrame(uint16_t value);
    static bool IRAM_ATTR _on_rx_done(rmt_channel_handle_t rmt_rx_channel, const rmt_rx_done_event_data_t *edata, void *user_data);

    // DShot Configuration Parameters
    gpio_num_t _gpio;                // GPIO pin used for DShot communication
    dshot_mode_t _mode;              // DShot mode (e.g., DSHOT300, DSHOT600)
    bool _is_bidirectional;          // True if bidirectional DShot is enabled
    uint16_t _motor_magnet_count;    // Number of magnets in the motor for RPM calculation
    dshot_timing_us_t _dshot_timing; // DShot timing parameters in microseconds

    // RMT Hardware Handles and Configuration
    rmt_channel_handle_t _rmt_tx_channel = nullptr; // RMT transmit channel handle
    rmt_channel_handle_t _rmt_rx_channel = nullptr; // RMT receive channel handle
    rmt_encoder_handle_t _dshot_encoder = nullptr;  // DShot RMT encoder handle
    rmt_ticks_t _rmt_ticks;                         // Pre-calculated RMT timing ticks
    uint16_t _pulse_level = 1;                      // Output level for a pulse (typically high)
    uint16_t _idle_level = 0;                       // Output level for idle (typically low)

    // DShot Frame Timing and State Variables
    uint64_t _last_transmission_time_us = 0; // Timestamp of the last DShot frame transmission
    uint64_t _frame_timer_us = 0;            // Minimum time required between DShot frames
    float _percent_to_throttle_ratio = 0.0f; // Pre-calculated ratio for throttle percentage conversion
    uint16_t _last_throttle = 0;             // Last transmitted throttle value
    dshot_packet_t _packet;                  // Current DShot packet being processed
    uint16_t _encoded_frame_value = 0;       // Last encoded 16-bit DShot frame value

    // Telemetry Related Variables
    std::atomic<uint16_t> _last_erpm_atomic = 0;                          // Atomically stored last received eRPM value
    std::atomic<bool> _telemetry_ready_flag_atomic = false;               // Atomically stored flag indicating new telemetry data
    std::atomic<dshot_telemetry_data_t> _last_telemetry_data_atomic = {}; // Atomically stored last received full telemetry data
    std::atomic<bool> _full_telemetry_ready_flag_atomic = false;          // Atomically stored flag indicating new full telemetry data
    rmt_rx_event_callbacks_t _rx_event_callbacks = {
        // RMT receive event callbacks
        .on_recv_done = _on_rx_done,
    };

    // Private Helper Functions for DShot Protocol Logic
    bool _isValidCommand(dshotCommands_e command) const;                                                          // Checks if a given DShot command is valid
    dshot_packet_t _buildDShotPacket(const uint16_t &value) const;                                                // Builds a DShot packet from a value (throttle or command)
    uint16_t _buildDShotFrameValue(const dshot_packet_t &packet) const;                                           // Combines packet data into a 16-bit DShot frame value
    uint16_t _calculateCRC(const uint16_t &data) const;                                                           // Calculates the 4-bit CRC for a DShot frame
    uint8_t _calculateTelemetryCRC(const uint8_t *data, size_t len) const;                                        // Calculates the 8-bit CRC for telemetry data
    void _extractTelemetryData(const uint8_t *raw_telemetry_bytes, dshot_telemetry_data_t &telemetry_data) const; // Extracts telemetry data from raw bytes
    void _preCalculateRMTTicks();                                                                                 // Pre-calculates RMT timing ticks for the selected DShot mode
    dshot_result_t _sendPacket(const dshot_packet_t &packet);                                                     // Sends a DShot frame via RMT TX channel
    uint16_t IRAM_ATTR _decodeDShotFrame(const rmt_symbol_word_t *symbols) const;                                 // Decodes a received RMT symbol array into an eRPM value
    void IRAM_ATTR _processFullTelemetryFrame(const rmt_symbol_word_t *symbols, size_t num_symbols);              // Processes a full telemetry frame
    bool IRAM_ATTR _isFrameIntervalElapsed() const;                                                               // Checks if enough time has passed since the last frame transmission
    void _recordFrameTransmissionTime();                                                                          // Records the current time as the last frame transmission time

    dshot_result_t _sendRepeatedCommand(uint16_t value, uint16_t repeat_count, uint16_t delay_us);

    // Static Callback Function for RMT RX Events
    void _cleanupRmtResources();
};

#include "dshot_utils.h" // Include for helper functions
