/**
 * @file dshot_utils.h
 * @brief Utility functions for DShotRMT library
 * @author Wastl Kraus
 * @date 2025-10-04
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include "dshot_definitions.h"
#include "DShotRMT.h"

// Error Messages
static constexpr char NONE[] = "";
static constexpr char UNKNOWN_ERROR[] = "Unknown Error!";
static constexpr char INIT_SUCCESS[] = "SignalGeneratorRMT initialized successfully";
static constexpr char INIT_FAILED[] = "SignalGeneratorRMT init failed!";
static constexpr char TX_INIT_SUCCESS[] = "TX RMT channel initialized successfully";
static constexpr char TX_INIT_FAILED[] = "TX RMT channel init failed!";
static constexpr char RX_INIT_SUCCESS[] = "RX RMT channel initialized successfully";
static constexpr char RX_INIT_FAILED[] = "RX RMT channel init failed!";
static constexpr char ENCODER_INIT_SUCCESS[] = "RMT encoder initialized successfully";
static constexpr char ENCODER_INIT_FAILED[] = "RMT encoder init failed!";
static constexpr char ENCODING_SUCCESS[] = "Packet encoded successfully";
static constexpr char TRANSMISSION_SUCCESS[] = "Transmission successfully";
static constexpr char TRANSMISSION_FAILED[] = "Transmission failed!";
static constexpr char RECEIVER_FAILED[] = "RMT receiver failed!";
static constexpr char THROTTLE_NOT_IN_RANGE[] = "Throttle not in range!";
static constexpr char PERCENT_NOT_IN_RANGE[] = "Percent not in range!";
static constexpr char COMMAND_NOT_VALID[] = "Command not valid!";
static constexpr char BIDIR_NOT_ENABLED[] = "Bidirectional DShot not enabled!";
static constexpr char TELEMETRY_SUCCESS[] = "Valid Telemetric Frame received!";
static constexpr char TELEMETRY_FAILED[] = "No valid Telemetric Frame received!";
static constexpr char INVALID_MAGNET_COUNT[] = "Invalid motor magnet count!";
static constexpr char TIMING_CORRECTION[] = "Timing correction!";
static constexpr char CALLBACK_REGISTERING_FAILED[] = "RMT RX Callback registering failed!";
static constexpr char INVALID_COMMAND[] = "Invalid command!";
static constexpr char COMMAND_SUCCESS[] = "DShot command sent successfully";

// Helper to get result code string
inline const char *get_result_code_str(dshot_msg_code_t code)
{
    switch (code)
    {
    case DSHOT_NONE:
        return NONE;
    case DSHOT_UNKNOWN:
        return UNKNOWN_ERROR;
    case DSHOT_TX_INIT_FAILED:
        return TX_INIT_FAILED;
    case DSHOT_RX_INIT_FAILED:
        return RX_INIT_FAILED;
    case DSHOT_ENCODER_INIT_FAILED:
        return ENCODER_INIT_FAILED;
    case DSHOT_CALLBACK_REGISTERING_FAILED:
        return CALLBACK_REGISTERING_FAILED;
    case DSHOT_RECEIVER_FAILED:
        return RECEIVER_FAILED;
    case DSHOT_TRANSMISSION_FAILED:
        return TRANSMISSION_FAILED;
    case DSHOT_THROTTLE_NOT_IN_RANGE:
        return THROTTLE_NOT_IN_RANGE;
    case DSHOT_PERCENT_NOT_IN_RANGE:
        return PERCENT_NOT_IN_RANGE;
    case DSHOT_COMMAND_NOT_VALID:
        return COMMAND_NOT_VALID;
    case DSHOT_BIDIR_NOT_ENABLED:
        return BIDIR_NOT_ENABLED;
    case DSHOT_TELEMETRY_FAILED:
        return TELEMETRY_FAILED;
    case DSHOT_INVALID_MAGNET_COUNT:
        return INVALID_MAGNET_COUNT;
    case DSHOT_INVALID_COMMAND:
        return INVALID_COMMAND;
    case DSHOT_TIMING_CORRECTION:
        return TIMING_CORRECTION;
    case DSHOT_INIT_FAILED:
        return INIT_FAILED;
    case DSHOT_INIT_SUCCESS:
        return INIT_SUCCESS;
    case DSHOT_TX_INIT_SUCCESS:
        return TX_INIT_SUCCESS;
    case DSHOT_RX_INIT_SUCCESS:
        return RX_INIT_SUCCESS;
    case DSHOT_ENCODER_INIT_SUCCESS:
        return ENCODER_INIT_SUCCESS;
    case DSHOT_ENCODING_SUCCESS:
        return ENCODING_SUCCESS;
    case DSHOT_TRANSMISSION_SUCCESS:
        return TRANSMISSION_SUCCESS;
    case DSHOT_TELEMETRY_SUCCESS:
        return TELEMETRY_SUCCESS;
    case DSHOT_COMMAND_SUCCESS:
        return COMMAND_SUCCESS;
    default:
        return UNKNOWN_ERROR;
    }
}

// Helper to get DShot mode string
inline const char *get_dshot_mode_str(dshot_mode_t mode)
{
    switch (mode)
    {
    case DSHOT150:
        return "DSHOT150";
    case DSHOT300:
        return "DSHOT300";
    case DSHOT600:
        return "DSHOT600";
    case DSHOT1200:
        return "DSHOT1200";
    default:
        return "DSHOT_OFF";
    }
}

// Helper to quick print DShot result codes
inline void printDShotResult(dshot_result_t &result, Stream &output = Serial)
{
    output.printf("Status: %s - %s", result.success ? "SUCCESS" : "FAILED", get_result_code_str(result.result_code));

    // Print telemetry data if available
    if (result.success && (result.erpm > 0 || result.motor_rpm > 0))
    {
        output.printf(" | eRPM: %u, Motor RPM: %u", result.erpm, result.motor_rpm);
    }

    output.println();
}

// Helper to print DShot signal info
inline void printDShotInfo(DShotRMT &dshot_rmt, Stream &output = Serial)
{
    output.println("\n=== DShot Info ===");
    output.printf("Library Version: %d.%d.%d\n", DSHOTRMT_MAJOR_VERSION, DSHOTRMT_MINOR_VERSION, DSHOTRMT_PATCH_VERSION);
    output.printf("Mode: %s\n", get_dshot_mode_str(dshot_rmt.getMode()));
    output.printf("Bidirectional: %s\n", dshot_rmt.isBidirectional() ? "YES" : "NO");
    output.printf("Last Throttle: %u\n", dshot_rmt.getThrottleValue());

    output.print("Packet (binary): ");
    for (int i = DSHOT_BITS_PER_FRAME - 1; i >= 0; --i)
    {
        output.print((dshot_rmt.getEncodedFrameValue() >> i) & 1);
    }
    output.println();

    // --- Telemetry Data ---
    if (dshot_rmt.isBidirectional())
    {
        dshot_result_t telemetry_result = dshot_rmt.getTelemetry();

        output.print("Telemetry: ");
        if (telemetry_result.success)
        {
            output.printf("OK (%s)\n", get_result_code_str(telemetry_result.result_code));

            if (telemetry_result.erpm > 0 || telemetry_result.motor_rpm > 0)
            {
                output.printf("  eRPM: %u, Motor RPM: %u\n", telemetry_result.erpm, telemetry_result.motor_rpm);
            }

            if (telemetry_result.telemetry_available)
            {
                output.println("  --- Full Telemetry Details ---");
                output.printf("  Temp: %d C | Volt: %.2f V | Curr: %.2f A | Cons: %u mAh\n",
                              telemetry_result.telemetry_data.temperature,
                              (float)telemetry_result.telemetry_data.voltage / CONVERSION_FACTOR_MILLI_TO_UNITS, // Convert mV to V
                              (float)telemetry_result.telemetry_data.current / CONVERSION_FACTOR_MILLI_TO_UNITS, // Convert mA to A
                              telemetry_result.telemetry_data.consumption);
                output.printf("  Telemetry RPM: %u\n", telemetry_result.telemetry_data.rpm);
            }
            else
            {
                output.println("  (Full telemetry not yet available or CRC failed for full frame)");
            }
        }
        else
        {
            output.printf("FAILED (%s)\n", get_result_code_str(telemetry_result.result_code));
        }
    }
    else
    {
        output.println("Telemetry: Disabled (Bidirectional mode OFF)");
    }
    output.println("===========================\n"); // End separator
}

// Helper to print CPU info
inline void printCpuInfo(Stream &output = Serial)
{
    output.println("\n ===  CPU Info  === ");
    output.printf("Chip Model: %s\n", ESP.getChipModel());
    output.printf("Chip Revision: %d\n", ESP.getChipRevision());
    output.printf("CPU Freq = %lu MHz\n", ESP.getCpuFreqMHz());
    output.printf("XTAL Freq = %lu MHz\n", getXtalFrequencyMhz());
    output.printf("APB Freq = %lu Hz\n", getApbFrequency());
}
