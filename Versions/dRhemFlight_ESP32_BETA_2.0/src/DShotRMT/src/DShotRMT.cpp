/**
 * @file DShotRMT.cpp
 * @brief DShot signal generation using ESP32 RMT with bidirectional support
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#include "DShotRMT.h"
#include <cstring>

// Constructor with GPIO number
DShotRMT::DShotRMT(gpio_num_t gpio, dshot_mode_t mode, bool is_bidirectional, uint16_t magnet_count)
    : _gpio(gpio),
      _mode(mode),
      _is_bidirectional(is_bidirectional),
      _motor_magnet_count(magnet_count),
      _dshot_timing(DSHOT_TIMING_US[_mode])
{
    // Pre-calculate timing and ratios for performance
    _preCalculateRMTTicks();
    _percent_to_throttle_ratio = (static_cast<float>(DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN)) / DSHOT_PERCENT_MAX;
}

// Constructor using pin number
DShotRMT::DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional, uint16_t magnet_count)
    : DShotRMT(static_cast<gpio_num_t>(pin_nr), mode, is_bidirectional, magnet_count)
{
    // Delegates to primary constructor with type cast
}

// Destructor
DShotRMT::~DShotRMT()
{
    _cleanupRmtResources();
}

// Initialize DShotRMT
dshot_result_t DShotRMT::begin()
{
    dshot_result_t result = init_rmt_tx_channel(_gpio, &_rmt_tx_channel, _is_bidirectional);

    if (!result.success)
    {
        _cleanupRmtResources(); // Clean up any allocated resources on failure
        return result;
    }

    if (_is_bidirectional)
    {
        result = init_rmt_rx_channel(_gpio, &_rmt_rx_channel, &_rx_event_callbacks, this);
        if (!result.success)
        {
            _cleanupRmtResources(); // Clean up any allocated resources on failure
            return result;
        }
    }

    result = init_dshot_encoder(&_dshot_encoder, _rmt_ticks, _pulse_level, _idle_level);

    if (!result.success)
    {
        _cleanupRmtResources(); // Clean up any allocated resources on failure
        return result;
    }

    return {true, DSHOT_INIT_SUCCESS};
}

// Send throttle value
dshot_result_t DShotRMT::sendThrottle(uint16_t throttle)
{
    // Per DShot specification, a throttle value of 0 is a disarm command.
    if (throttle == 0)
    {
        _last_throttle = 0;
        return sendCommand(DSHOT_CMD_MOTOR_STOP);
    }

    // Constrain throttle to the valid DShot range.
    _last_throttle = constrain(throttle, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);

    _packet = _buildDShotPacket(_last_throttle);
    return _sendPacket(_packet);
}

// Send throttle value as a percentage
dshot_result_t DShotRMT::sendThrottlePercent(float percent)
{
    if (percent < DSHOT_PERCENT_MIN || percent > DSHOT_PERCENT_MAX)
    {
        return {false, DSHOT_PERCENT_NOT_IN_RANGE};
    }

    // Map percent to DShot throttle range using pre-calculated ratio.
    uint16_t throttle = static_cast<uint16_t>(DSHOT_THROTTLE_MIN + _percent_to_throttle_ratio * percent);
    return sendThrottle(throttle);
}

// Sends a DShot command (0-47) to the ESC by accepting an integer value.
dshot_result_t DShotRMT::sendCommand(uint16_t command_value)
{
    // Validate the integer command value before casting.
    if (command_value < DSHOT_CMD_MOTOR_STOP || command_value > DSHOT_CMD_MAX_VALUE)
    {
        return {false, DSHOT_COMMAND_NOT_VALID};
    }
    return sendCommand(static_cast<dshotCommands_e>(command_value));
}

// Sends a DShot command (0-47) to the ESC.
dshot_result_t DShotRMT::sendCommand(dshotCommands_e command)
{
    uint16_t repeat_count = DEFAULT_CMD_REPEAT_COUNT;
    uint16_t delay_us = DEFAULT_CMD_DELAY_US;

    // Certain commands require more repetitions to be reliably accepted by the ESC.
    switch (command)
    {
    case DSHOT_CMD_MOTOR_STOP:
    case DSHOT_CMD_SAVE_SETTINGS:
    case DSHOT_CMD_SPIN_DIRECTION_NORMAL:
    case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
        repeat_count = SETTINGS_COMMAND_REPEATS;
        delay_us = SETTINGS_COMMAND_DELAY_US;
        break;
    default:
        // For other commands, use default repeat and delay.
        break;
    }

    return sendCommand(command, repeat_count, delay_us);
}

// Sends a DShot command (0-47) to the ESC with a specified repeat count and delay.
dshot_result_t DShotRMT::sendCommand(dshotCommands_e command, uint16_t repeat_count, uint16_t delay_us)
{
    if (!_isValidCommand(command))
    {
        return {false, DSHOT_INVALID_COMMAND};
    }
    return _sendRepeatedCommand(static_cast<uint16_t>(command), repeat_count, delay_us);
}

// Get telemetry data
dshot_result_t DShotRMT::getTelemetry()
{
    dshot_result_t result = {false, DSHOT_TELEMETRY_FAILED};

    if (!_is_bidirectional)
    {
        result.result_code = DSHOT_BIDIR_NOT_ENABLED;
        return result;
    }

    // Prioritize checking for full telemetry data, as it is richer.
    if (_full_telemetry_ready_flag_atomic)
    {
        _full_telemetry_ready_flag_atomic = false; // Reset the flag
        result.telemetry_data = _last_telemetry_data_atomic; // Read the atomic variable
        result.telemetry_available = true;

        // Also populate eRPM fields from the full telemetry data for consistency.
        result.erpm = result.telemetry_data.rpm;
        if (_motor_magnet_count >= MAGNETS_PER_POLE_PAIR) {
            uint8_t pole_pairs = _motor_magnet_count / MAGNETS_PER_POLE_PAIR;
            result.motor_rpm = result.telemetry_data.rpm / pole_pairs;
        }
        
        result.success = true;
        result.result_code = DSHOT_TELEMETRY_DATA_AVAILABLE;
        return result;
    }

    // If no full telemetry, check for eRPM-only data.
    if (_telemetry_ready_flag_atomic)
    {
        _telemetry_ready_flag_atomic = false; // Reset the flag
        uint16_t erpm = _last_erpm_atomic;    // Read the atomic variable

        if (erpm != DSHOT_NULL_PACKET && _motor_magnet_count >= MAGNETS_PER_POLE_PAIR)
        {
            // Calculate motor RPM from eRPM and magnet count
            uint8_t pole_pairs = _motor_magnet_count / MAGNETS_PER_POLE_PAIR;
            result.erpm = erpm;
            result.motor_rpm = (erpm / pole_pairs);
            result.success = true;
            result.result_code = DSHOT_TELEMETRY_SUCCESS;
        }
    }

    return result;
}

// Reverse motor direction directly
dshot_result_t DShotRMT::setMotorSpinDirection(bool reversed)
{
    dshotCommands_e command = reversed ? dshotCommands_e::DSHOT_CMD_SPIN_DIRECTION_REVERSED : dshotCommands_e::DSHOT_CMD_SPIN_DIRECTION_NORMAL;
    return sendCommand(command, SETTINGS_COMMAND_REPEATS, SETTINGS_COMMAND_DELAY_US);
}

dshot_result_t DShotRMT::sendCustomCommand(uint16_t command_value, uint16_t repeat_count, uint16_t delay_us)
{
    // Validate the integer command value.
    if (command_value < DSHOT_CMD_MIN || command_value > DSHOT_CMD_MAX)
    {
        return {false, DSHOT_COMMAND_NOT_VALID};
    }
    return _sendRepeatedCommand(command_value, repeat_count, delay_us);
}

// Writes settings to the ESC's non-volatile memory; use with caution.
dshot_result_t DShotRMT::saveESCSettings()
{
    return sendCommand(dshotCommands_e::DSHOT_CMD_SAVE_SETTINGS, SETTINGS_COMMAND_REPEATS, SETTINGS_COMMAND_DELAY_US);
}

// Private helper to send a command value multiple times.
dshot_result_t DShotRMT::_sendRepeatedCommand(uint16_t value, uint16_t repeat_count, uint16_t delay_us)
{
    bool all_successful = true;
    dshot_result_t last_result = {true, DSHOT_COMMAND_SUCCESS};

    for (uint16_t i = 0; i < repeat_count; i++)
    {
        last_result = _sendRawDshotFrame(value);

        if (!last_result.success)
        {
            all_successful = false;
            break;
        }

        if (i < repeat_count - 1)
        {
            delayMicroseconds(delay_us);
        }
    }

    if (all_successful)
    {
        return {true, DSHOT_COMMAND_SUCCESS};
    }
    else
    {
        // Return the result from the failed transmission.
        return last_result;
    }
}

// Simple check for valid command range.
bool DShotRMT::_isValidCommand(dshotCommands_e command) const
{
    return (command >= dshotCommands_e::DSHOT_CMD_MOTOR_STOP && command <= DSHOT_CMD_MAX);
}

dshot_result_t DShotRMT::_sendRawDshotFrame(uint16_t value)
{
    _packet = _buildDShotPacket(value);
    return _sendPacket(_packet);
}

// Private Packet Management Functions
dshot_packet_t DShotRMT::_buildDShotPacket(const uint16_t &value) const
{
    dshot_packet_t packet = {};

    packet.throttle_value = value & DSHOT_THROTTLE_MAX;
    packet.telemetric_request = _is_bidirectional ? 1 : 0;

    // The data for CRC calculation includes the 11-bit value and the 1-bit telemetry flag.
    uint16_t data_for_crc = (packet.throttle_value << 1) | packet.telemetric_request;
    packet.checksum = _calculateCRC(data_for_crc);

    return packet;
}

uint16_t DShotRMT::_buildDShotFrameValue(const dshot_packet_t &packet) const
{
    // Combine throttle, telemetry bit, and CRC into a single 16-bit frame.
    uint16_t data_and_telemetry = (packet.throttle_value << 1) | packet.telemetric_request;
    return (data_and_telemetry << 4) | packet.checksum;
}

uint16_t DShotRMT::_calculateCRC(const uint16_t &data) const
{
    // Standard DShot CRC calculation using XOR.
    uint16_t crc = (data ^ (data >> 4) ^ (data >> 8)) & DSHOT_CRC_MASK;

    // For bidirectional DShot, the CRC is inverted per specification.
    if (_is_bidirectional)
    {
        crc = (~crc) & DSHOT_CRC_MASK;
    }
    return crc;
}

uint8_t DShotRMT::_calculateTelemetryCRC(const uint8_t *data, size_t len) const
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x07; // DSHOT telemetry uses CRC-8 with polynomial 0x07.
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void DShotRMT::_extractTelemetryData(const uint8_t *raw_telemetry_bytes, dshot_telemetry_data_t &telemetry_data) const
{
    // Ensure the telemetry_data struct is cleared before filling.
    memset(&telemetry_data, 0, sizeof(dshot_telemetry_data_t));

    // Telemetry data is typically ordered as:
    // Byte 0: Temperature (signed 8-bit)
    // Byte 1-2: Voltage (16-bit, MSB first)
    // Byte 3-4: Current (16-bit, MSB first)
    // Byte 5-6: Consumption (16-bit, MSB first)
    // Byte 7-8: RPM (16-bit, MSB first)
    // Byte 9: CRC (8-bit) - checked separately

    telemetry_data.temperature = static_cast<int8_t>(raw_telemetry_bytes[0]);
    telemetry_data.voltage = (static_cast<uint16_t>(raw_telemetry_bytes[1]) << 8) | raw_telemetry_bytes[2];
    telemetry_data.current = (static_cast<uint16_t>(raw_telemetry_bytes[3]) << 8) | raw_telemetry_bytes[4];
    telemetry_data.consumption = (static_cast<uint16_t>(raw_telemetry_bytes[5]) << 8) | raw_telemetry_bytes[6];
    telemetry_data.rpm = (static_cast<uint16_t>(raw_telemetry_bytes[7]) << 8) | raw_telemetry_bytes[8];
}

void DShotRMT::_preCalculateRMTTicks()
{
    // Pre-calculate all timing values in RMT ticks to save CPU cycles during operation.
    _rmt_ticks.bit_length_ticks = static_cast<uint16_t>(_dshot_timing.bit_length_us * RMT_TICKS_PER_US);
    _rmt_ticks.t1h_ticks = static_cast<uint16_t>(_dshot_timing.t1h_lenght_us * RMT_TICKS_PER_US);
    _rmt_ticks.t0h_ticks = _rmt_ticks.t1h_ticks >> 1; // High time for a '0' bit is half of a '1' bit.
    _rmt_ticks.t1l_ticks = _rmt_ticks.bit_length_ticks - _rmt_ticks.t1h_ticks;
    _rmt_ticks.t0l_ticks = _rmt_ticks.bit_length_ticks - _rmt_ticks.t0h_ticks;

    // Calculate the minimum time required between frames to prevent signal collision.
    _frame_timer_us = (static_cast<uint64_t>(_dshot_timing.bit_length_us * DSHOT_BITS_PER_FRAME) << 1) + DSHOT_PADDING_US;

    if (_is_bidirectional)
    {
        _frame_timer_us = (_frame_timer_us << 1);
    }
}

// Private Frame Processing Functions
dshot_result_t DShotRMT::_sendPacket(const dshot_packet_t &packet)
{
    // Ensure enough time has passed since the last transmission.
    if (!_isFrameIntervalElapsed())
    {
        return {true, DSHOT_NONE};
    }

    if (_is_bidirectional)
    {
        // Start the RMT receiver to wait for the ESC's telemetry response.
        rmt_symbol_word_t rx_symbols[DSHOT_TELEMETRY_FULL_GCR_BITS];
        size_t rx_size_bytes = DSHOT_TELEMETRY_FULL_GCR_BITS * sizeof(rmt_symbol_word_t);

        rmt_receive_config_t rmt_rx_config = {
            .signal_range_min_ns = DSHOT_PULSE_MIN_NS,
            .signal_range_max_ns = DSHOT_PULSE_MAX_NS,
        };

        if (rmt_receive(_rmt_rx_channel, rx_symbols, rx_size_bytes, &rmt_rx_config) != DSHOT_OK)
        {
            return {false, DSHOT_RECEIVER_FAILED};
        }
    }

    _encoded_frame_value = _buildDShotFrameValue(packet);

    // Byte-swap the 16-bit value for correct transmission order.
    // The RMT bytes encoder sends MSB of each byte first.
    uint16_t swapped_value = __builtin_bswap16(_encoded_frame_value);

    // The DShot frame is 16 bits, which is 2 bytes.
    size_t tx_size_bytes = sizeof(swapped_value);

    rmt_transmit_config_t tx_config = {.loop_count = 0}; // No automatic loops.

    // In bidirectional mode, the RMT RX channel must be disabled before transmitting
    // to prevent the receiver from picking up the outgoing signal (loopback).
    if (_is_bidirectional)
    {
        if (rmt_disable(_rmt_rx_channel) != DSHOT_OK)
        {
            return {false, DSHOT_RECEIVER_FAILED};
        }
    }

    if (rmt_transmit(_rmt_tx_channel, _dshot_encoder, &swapped_value, tx_size_bytes, &tx_config) != DSHOT_OK)
    {
        return {false, DSHOT_TRANSMISSION_FAILED};
    }

    // Re-enable RMT RX immediately after transmission to catch the response.
    if (_is_bidirectional)
    {
        if (rmt_enable(_rmt_rx_channel) != DSHOT_OK)
        {
            return {false, DSHOT_RECEIVER_FAILED};
        }
    }

    _recordFrameTransmissionTime(); // Reset the timer for the next frame.

    return {true, DSHOT_TRANSMISSION_SUCCESS};
}

// This function is placed in IRAM for high performance, as it may be
// called from an ISR context depending on RMT driver implementation.
uint16_t IRAM_ATTR DShotRMT::_decodeDShotFrame(const rmt_symbol_word_t *symbols) const
{
    uint32_t gcr_value = 0;

    // Decode RMT symbols into a 21-bit GCR (Group Code Recording) value.
    // The ESC sends back a signal where the duration of high vs. low determines the bit value.
    for (size_t i = 0; i < DSHOT_ERPM_FRAME_GCR_BITS; ++i)
    {
        bool bit_is_one = symbols[i].duration0 > symbols[i].duration1;
        gcr_value = (gcr_value << 1) | bit_is_one;
    }

    // Perform GCR decoding (GCR = Value ^ (Value >> 1)).
    uint32_t decoded_frame = gcr_value ^ (gcr_value >> 1);

    // Extract the 16-bit DShot-like frame from the decoded data.
    uint16_t data_and_crc = (decoded_frame & DSHOT_FULL_PACKET);

    // The eRPM telemetry frame consists of 12 data bits and 4 CRC bits.
    uint16_t received_data = data_and_crc >> DSHOT_CRC_BIT_SHIFT;
    uint16_t received_crc = data_and_crc & DSHOT_CRC_MASK;

    // Calculate and validate the CRC for the received data.
    uint16_t calculated_crc = _calculateCRC(received_data);
    if (received_crc != calculated_crc)
    {
        return DSHOT_NULL_PACKET;
    }

    // Return the eRPM value (the first 11 bits of the data).
    return received_data & DSHOT_THROTTLE_MAX;
}

// Timing Control Functions
bool IRAM_ATTR DShotRMT::_isFrameIntervalElapsed() const
{
    // Check if the minimum interval between frames has passed.
    uint64_t current_time = esp_timer_get_time();
    uint64_t elapsed = current_time - _last_transmission_time_us;
    return elapsed >= _frame_timer_us;
}

void DShotRMT::_recordFrameTransmissionTime()
{
    // Record the time of the current transmission.
    _last_transmission_time_us = esp_timer_get_time();
}

// Static Callback Functions
// Processes a full telemetry frame from the RMT RX ISR.
void IRAM_ATTR DShotRMT::_processFullTelemetryFrame(const rmt_symbol_word_t *symbols, size_t num_symbols)
{
    if (num_symbols != DSHOT_TELEMETRY_FULL_GCR_BITS)
    {
        return; // Incorrect number of symbols for a full telemetry frame.
    }

    uint8_t gcr_decoded_bytes[DSHOT_TELEMETRY_FRAME_LENGTH_BYTES + 1]; // 10 data bytes + 1 CRC byte.
    memset(gcr_decoded_bytes, 0, sizeof(gcr_decoded_bytes));

    uint8_t data_bit_idx = 0;
    for (size_t i = 0; i < DSHOT_TELEMETRY_FULL_GCR_BITS; i += 5)
    {
        uint8_t gcr_group_5bits = 0;
        for (size_t j = 0; j < 5; ++j)
        {
            if (i + j < DSHOT_TELEMETRY_FULL_GCR_BITS)
            {
                gcr_group_5bits = (gcr_group_5bits << 1) | ((symbols[i + j].duration0 > symbols[i + j].duration1) ? 1 : 0);
            }
        }

        uint8_t decoded_nibble; // 4 data bits.
        switch (gcr_group_5bits)
        {
        case 0b11110: decoded_nibble = 0b0000; break;
        case 0b01001: decoded_nibble = 0b0001; break;
        case 0b10100: decoded_nibble = 0b0010; break;
        case 0b10101: decoded_nibble = 0b0011; break;
        case 0b01010: decoded_nibble = 0b0100; break;
        case 0b01011: decoded_nibble = 0b0101; break;
        case 0b01110: decoded_nibble = 0b0110; break;
        case 0b01111: decoded_nibble = 0b0111; break;
        case 0b10010: decoded_nibble = 0b1000; break;
        case 0b10011: decoded_nibble = 0b1001; break;
        case 0b10110: decoded_nibble = 0b1010; break;
        case 0b10111: decoded_nibble = 0b1011; break;
        case 0b11010: decoded_nibble = 0b1100; break;
        case 0b11011: decoded_nibble = 0b1101; break;
        case 0b11100: decoded_nibble = 0b1110; break;
        case 0b11101: decoded_nibble = 0b1111; break;
        default: return; // Invalid GCR group, discard frame.
        }

        // Place the 4 decoded bits into the data_bytes array.
        for (int k = 3; k >= 0; --k)
        {
            if (data_bit_idx < (DSHOT_TELEMETRY_FRAME_LENGTH_BITS + DSHOT_TELEMETRY_CRC_LENGTH_BITS))
            {
                size_t byte_idx = data_bit_idx / 8;
                size_t bit_pos = data_bit_idx % 8;
                if (byte_idx < sizeof(gcr_decoded_bytes))
                {
                    gcr_decoded_bytes[byte_idx] |= ((decoded_nibble >> k) & 1) << (7 - bit_pos);
                }
                data_bit_idx++;
            }
        }
    }

    // The gcr_decoded_bytes array now contains the 10 telemetry bytes + 1 CRC byte.
    // Perform CRC validation.
    uint8_t received_crc = gcr_decoded_bytes[DSHOT_TELEMETRY_FRAME_LENGTH_BYTES];
    uint8_t calculated_crc = _calculateTelemetryCRC(gcr_decoded_bytes, DSHOT_TELEMETRY_FRAME_LENGTH_BYTES);

    if (received_crc == calculated_crc)
    {
        dshot_telemetry_data_t telemetry_data;
        // Extract from the first 10 bytes (excluding the CRC byte).
        _extractTelemetryData(gcr_decoded_bytes, telemetry_data);

        _last_telemetry_data_atomic.store(telemetry_data);
        _full_telemetry_ready_flag_atomic.store(true);
    }
}

// This function is called by the RMT driver's ISR when a frame is received.
bool IRAM_ATTR DShotRMT::_on_rx_done(rmt_channel_handle_t rmt_rx_channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    DShotRMT *instance = static_cast<DShotRMT *>(user_data);

    if (edata)
    {
        if (edata->num_symbols == DSHOT_TELEMETRY_FULL_GCR_BITS)
        {
            instance->_processFullTelemetryFrame(edata->received_symbols, edata->num_symbols);
        }
        else if (edata->num_symbols == DSHOT_ERPM_FRAME_GCR_BITS)
        {
            uint16_t erpm = instance->_decodeDShotFrame(edata->received_symbols);

            if (erpm != DSHOT_NULL_PACKET)
            {
                // Atomically store the new eRPM value and set the flag.
                instance->_last_erpm_atomic.store(erpm);
                instance->_telemetry_ready_flag_atomic.store(true);
            }
        }
    }

    return false;
}

void DShotRMT::_cleanupRmtResources()
{
    if (_rmt_tx_channel)
    {
        rmt_disable(_rmt_tx_channel);
        rmt_del_channel(_rmt_tx_channel);
        _rmt_tx_channel = nullptr;
    }

    if (_rmt_rx_channel)
    {
        rmt_disable(_rmt_rx_channel);
        rmt_del_channel(_rmt_rx_channel);
        _rmt_rx_channel = nullptr;
    }

    if (_dshot_encoder)
    {
        rmt_del_encoder(_dshot_encoder);
        _dshot_encoder = nullptr;
    }
}
