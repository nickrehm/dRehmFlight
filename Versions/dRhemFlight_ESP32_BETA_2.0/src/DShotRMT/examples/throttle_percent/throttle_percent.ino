/**
 * @file throttle_percent.ino
 * @brief Example sketch for DShotRMT library demonstrating throttle control by percentage
 * @author Wastl Kraus
 * @date 2025-09-20
 * @license MIT
 */

#include <Arduino.h>
#include <DShotRMT.h>

// USB serial port settings
static constexpr auto &USB_SERIAL = Serial0;
static constexpr auto USB_SERIAL_BAUD = 115200;

// Motor configuration - Pin number or GPIO_PIN
static constexpr gpio_num_t MOTOR01_PIN = GPIO_NUM_27;

// Supported: DSHOT150, DSHOT300, DSHOT600, (DSHOT1200)
static constexpr dshot_mode_t DSHOT_MODE = DSHOT300;

// BiDirectional DShot Support (default: false)
// Note: Bidirectional DShot is currently not officially supported 
// due to instability and external hardware requirements.
static constexpr auto IS_BIDIRECTIONAL = false; 

// Motor magnet count for RPM calculation
static constexpr auto MOTOR01_MAGNET_COUNT = 14;

// Creates the motor instance
DShotRMT motor01(MOTOR01_PIN, DSHOT_MODE, IS_BIDIRECTIONAL, DEFAULT_MOTOR_MAGNET_COUNT);

// Forward declaration
void handleSerialInput(const String &input);
void printMenu();

//
void setup()
{
    // Starts the USB Serial Port
    USB_SERIAL.begin(USB_SERIAL_BAUD);

    // Initialize DShot Signal
    motor01.begin();

    // Print CPU Info
    printCpuInfo(USB_SERIAL);

    //
    printMenu();
}

//
void loop()
{
    // Handle serial input
    if (USB_SERIAL.available() > 0)
    {
        String input = USB_SERIAL.readStringUntil('\n');
        input.trim();

        if (input.length() > 0)
        {
            handleSerialInput(input);
        }
    }
}

//
void printMenu()
{
    USB_SERIAL.println(" ");
    USB_SERIAL.println("*******************************************");
    USB_SERIAL.println("            DShotRMT Percent Demo          ");
    USB_SERIAL.println("*******************************************");
    USB_SERIAL.println(" <value>      - Set throttle (0 - 100)");
    USB_SERIAL.println(" 0            - Stop motor");
    USB_SERIAL.println("*******************************************");
    USB_SERIAL.println(" cmd <number> - Send DShot command (0 - 47)");
    USB_SERIAL.println(" info         - Show motor info");
    if (IS_BIDIRECTIONAL)
    {
        USB_SERIAL.println(" rpm          - Get telemetry data");
    }
    USB_SERIAL.println("*******************************************");
    USB_SERIAL.println(" h / help     - Show this Menu");
    USB_SERIAL.println("*******************************************");
}

//
void handleSerialInput(const String &input)
{
    if (input == "0")
    {
        // Stop motor
        dshot_result_t result = motor01.sendThrottlePercent(0.0f);
        printDShotResult(result);
    }
    else if (input == "info")
    {
        printDShotInfo(motor01, USB_SERIAL);
    }
    else if (input == "rpm" && IS_BIDIRECTIONAL)
    {
        dshot_result_t result = motor01.getTelemetry();
        printDShotResult(result);
    }
    else if (input.startsWith("cmd "))
    {
        // Send DShot command
        int cmd_num = input.substring(4).toInt();

        if (cmd_num >= DSHOT_CMD_MOTOR_STOP && cmd_num <= DSHOT_CMD_MAX)
        {
            dshot_result_t result = motor01.sendCommand(cmd_num);
            printDShotResult(result);
        }
        else
        {
            USB_SERIAL.printf("Invalid command: %d (valid range: 0 - %d)\n", cmd_num, DSHOT_CMD_MAX);
        }
    }
    else if (input == "h" || input == "help")
    {
        printMenu();
    }
    else
    {
        // Parse input throttle value as a percentage
        float throttle_percent = input.toFloat();

        if (throttle_percent >= 0.0f && throttle_percent <= 100.0f)
        {
            dshot_result_t result = motor01.sendThrottlePercent(throttle_percent);
            printDShotResult(result);
        }
        else
        {
            USB_SERIAL.println(" ");
            USB_SERIAL.printf("Invalid input: '%s'\n", input.c_str());
            USB_SERIAL.printf("Valid throttle range: 0.0 - 100.0\n");
        }
    }
}
