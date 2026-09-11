/**
 * @file dshot300.ino
 * @brief Demo sketch for DShotRMT library
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#include <DShotRMT.h>

// USB serial port settings
static constexpr auto &USB_SERIAL = Serial0;
static constexpr auto USB_SERIAL_BAUD = 115200;

// Motor configuration - Pin number or GPIO_PIN
static constexpr gpio_num_t MOTOR01_PIN = GPIO_NUM_38;
// static constexpr auto MOTOR01_PIN = 27;

// Supported: DSHOT150, DSHOT300, DSHOT600, (DSHOT1200)
static constexpr dshot_mode_t DSHOT_MODE = DSHOT300;

// BiDirectional DShot Support (default: false)
// re-enabled for testing
static constexpr auto IS_BIDIRECTIONAL = true;

// Motor magnet count for RPM calculation
// static constexpr auto MOTOR01_MAGNET_COUNT = 14;

// Creates the motor instance
DShotRMT motor01(MOTOR01_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);

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
    // Safety first
    static uint16_t throttle = DSHOT_CMD_MOTOR_STOP;

    // Initialize the esc with "0"
    static bool continuous_throttle = true;

    // Time Measurement
    static uint64_t last_stats_print = 0;

    // Handle serial input
    if (USB_SERIAL.available() > 0)
    {
        String input = USB_SERIAL.readStringUntil('\n');
        input.trim();

        if (input.length() > 0)
        {
            handleSerialInput(input, throttle, continuous_throttle, motor01);
        }
    }

    // Send throttle value in continuous mode
    if (continuous_throttle)
    {
        motor01.sendThrottle(throttle);
    }

    // Print motor stats every 3 seconds in continuous mode
    if (continuous_throttle && (esp_timer_get_time() - last_stats_print >= 3000000))
    {
        printDShotInfo(motor01, USB_SERIAL);

        // Get Motor RPM if bidirectional
        if (IS_BIDIRECTIONAL)
        {
            // dshot_result_t telem_result = motor01.getTelemetry();
            // printDShotResult(telem_result);
        }

        USB_SERIAL.println("Type 'help' to show Menu");

        // Time Stamp
        last_stats_print = esp_timer_get_time();
    }
}

//
void printMenu()
{
    USB_SERIAL.println(" ");
    USB_SERIAL.println("*******************************************");
    USB_SERIAL.println("               DShotRMT Demo               ");
    USB_SERIAL.println("*******************************************");
    USB_SERIAL.println(" <value>      - Set throttle (48 – 2047)");
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
void handleSerialInput(const String &input, uint16_t &throttle, bool &continuous_throttle, DShotRMT &session)
{
    if (input == "0")
    {
        // Stop motor
        throttle = 0;
        continuous_throttle = true;
        dshot_result_t result = session.sendCommand(DSHOT_CMD_MOTOR_STOP);
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
        continuous_throttle = false;

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
        // Parse input throttle value
        int throttle_value = input.toInt();

        if (throttle_value >= DSHOT_THROTTLE_MIN && throttle_value <= DSHOT_THROTTLE_MAX)
        {
            throttle = throttle_value;
            continuous_throttle = true;

            dshot_result_t result = motor01.sendThrottle(throttle);
            printDShotResult(result);
        }
        else
        {
            USB_SERIAL.println(" ");
            USB_SERIAL.printf("Invalid input: '%s'\n", input);
            USB_SERIAL.printf("Valid throttle range: %d - %d\n", DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
        }
    }
}
