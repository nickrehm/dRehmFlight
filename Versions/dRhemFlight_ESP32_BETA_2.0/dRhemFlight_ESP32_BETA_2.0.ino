//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 7/29/2026
//Version: ESP32 Beta 2.0
 
//========================================================================================================================//

//CREDITS + SPECIAL THANKS
/*
Some elements inspired by:
http://www.brokking.net/ymfc-32_main.html

Madgwick filter function adapted from:
https://github.com/arduino-libraries/MadgwickAHRS

MPU9250 implementation based on MPU9250 library by:
brian.taylor@bolderflight.com
http://www.bolderflight.com

Adafruit LSM6DSOX implementation added by:
bjones@aggiejones.com
http://nvrtd.design

Thank you to:
RcGroups 'jihlein' - IMU implementation overhaul + SBUS implementation.
Everyone that sends me pictures and videos of your flying creations! -Nick


ADD MORE CREDITS TO UPDATE
*/


//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //                                                                 
//========================================================================================================================//

// Uncomment only one receiver type
//#define USE_PWM_RX    // deprecated
//#define USE_PPM_RX    // deprecated
//#define USE_DSM_RX    // deprecated
// static const uint8_t num_DSM_channels = 6; //If using DSM RX, change this to match the number of transmitter channels you have
#define USE_SBUS_RX
//#define USE_CRSF_RX


// Uncomment only one IMU
//#define USE_MPU6050_I2C //Default
//#define USE_MPU9250_SPI
#define USE_LSM6DSOX_SPI
// any other IMUs to add?

// Uncomment only one motor protocol -- PWM, ONESHOT125, DSHOT
// #define USE_MOTOR_PROTOCOL_PWM
// #define USE_MOTOR_PROTOCOL_ONESHOT
#define USE_MOTOR_PROTOCOL_DSHOT



// Enable to startup a web server at 192.168.4.1 for dynamic PID tuning
#define USE_WEBSERVER 1
const char* ssid = "ESP32-FlightController";
const char* password = "tuningpassword";




//========================================================================================================================//


// REQUIRED LIBRARIES (included with download in main sketch folder)
#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication
#include <WiFi.h>

#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <atomic>
#include "driver/mcpwm_prelude.h"

#if defined USE_MOTOR_PROTOCOL_ONESHOT
  // OneShot125 Constants
  #define ONESHOT125_MIN_US 125
  #define ONESHOT125_MAX_US 250
#endif

#if defined USE_MOTOR_PROTOCOL_DSHOT
  #include "src/DShotRMT/DShotRMT.h"

  // DShot Constants
  // Supported: DSHOT150, DSHOT300, DSHOT600, (DSHOT1200)
  #define DSHOT_MODE DSHOT300
  // BiDirectional DShot Support (default: false)
  #define DSHOT_TELEM false
  // Set the throttle minimum for DSHOT (airmode)
  #define DSHOT_MINTHROTTLE 200
#endif



// The onboard LED pin for the ESP32 is pin 2
#define LEDPIN 2


#if defined USE_SBUS_RX
  #include <sbus.h>
  #define SBUS_RX_PIN 18

  // Modern Bolder Flight Systems SBUS library (v8.1.4). Its ESP32
  // constructor takes the RX pin (and inversion) directly, so a single
  // sbus.Begin() call configures the UART correctly -- no more need for
  // a separate manual Serial2.begin() call after the library's own.
  bfs::SbusRx sbus(&Serial2, SBUS_RX_PIN, -1, true);
  bool sbusFailSafe;
  bool sbusLostFrame;
#endif

#if defined USE_DSM_RX    // deprecated
  // #include "src/DSMRX/DSMRX.h"  
  // DSM1024 DSM;
#endif

#if defined USE_CRSF_RX
  #include <AlfredoCRSF.h>
  #include <HardwareSerial.h>
  #define CRSF_RX_PIN 18
  #define CRSF_TX_PIN 17

  HardwareSerial crsfSerial(1);
  AlfredoCRSF crsf;
  bool crsfFailSafe = true;
#endif





// Servo configuration structure.  Used for defining servo and motor parameters.
struct ServoConfig {
  uint8_t pin;
  int refresh;
  int minUs;
  int maxUs;
  float trim;
  int initAngle;
  int minDuty;  // auto set via servoConfig
  int maxDuty;  // auto set via servoConfig
};



//========================================================================================================================//




//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//

// The number of input radio channels.  Default is 6 channels:  TAER, Arm switch, mode switch
#define NUM_CHANNELS 6
#define NUM_MOTORS 4
// PWM servo or ESC outputs:
// How many servos (or PWM ESCs) do you have?
#define NUM_SERVOS 7

//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
// The order is Thr, Ail, Ele, Rud, Arm, aux1, 2, etc.
const unsigned long channel_fs[NUM_CHANNELS] = {1000, 1500, 1500, 1500, 2000, 2000};


//Controller parameters (take note of defaults before modifying!): 
const float i_limit  = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
// Angle mode and rate mode use roll/pitch limits with different physical units
// (degrees of tilt vs degrees/sec of rotation) -- like the RATE vs ANGLE PID
// gains below, they get separate constants instead of one shared value.
const float maxRoll_angle  = 30.0;   //Max roll angle in degrees for angle mode (maximum ~70 degrees)
const float maxPitch_angle = 30.0;   //Max pitch angle in degrees for angle mode (maximum ~70 degrees)
const float maxRoll_rate   = 160.0;  //Max roll rate in deg/sec for rate mode
const float maxPitch_rate  = 160.0;  //Max pitch rate in deg/sec for rate mode
const float maxYaw         = 160.0;  //Max yaw rate in deg/sec (both modes)


// Hard-coded default PID parameters.  These will be used as the default when "Reset to default" is used on the web interface or uncommented function.
// RATE MODE defaults
const float DEFAULT_KP_ROLL_RATE = 0.30;
const float DEFAULT_KI_ROLL_RATE = 0.20;
const float DEFAULT_KD_ROLL_RATE = 0.00015;

const float DEFAULT_KP_PITCH_RATE = 0.30;
const float DEFAULT_KI_PITCH_RATE = 0.20;
const float DEFAULT_KD_PITCH_RATE = 0.00015;

const float DEFAULT_KP_YAW_RATE = 0.30;
const float DEFAULT_KI_YAW_RATE = 0.05;
const float DEFAULT_KD_YAW_RATE = 0.00015;

// ANGLE MODE defaults
const float DEFAULT_KP_ROLL_ANGLE = 0.50;   // 0.2
const float DEFAULT_KI_ROLL_ANGLE = 0.60;   // 0.3
const float DEFAULT_KD_ROLL_ANGLE = 0.15;   // 0.05

const float DEFAULT_KP_PITCH_ANGLE = 0.50;
const float DEFAULT_KI_PITCH_ANGLE = 0.60;
const float DEFAULT_KD_PITCH_ANGLE = 0.15;

const float DEFAULT_KP_YAW_ANGLE = 0.30; // Yaw uses rate stabilization even in angle mode
const float DEFAULT_KI_YAW_ANGLE = 0.05;
const float DEFAULT_KD_YAW_ANGLE = 0.00015;



//========================================================================================================================//
//                                                     DECLARE PINS                                                       //                           
//========================================================================================================================//                                          


//NOTE: Pin 2 is reserved for onboard LED, pins 10-13 are reserved for the IMU SPI interface under default setup
//Radio:
//If using SBUS, connect to pin 18.  Channel order is currently hardcoded for TAER, arm switch, and mode switch.
//If using Crossfire (CRSF), connect RX to pin 18 and TX to pin 17.  Channel order is currently hardcoded for TAER, arm switch, and mode switch.


#if defined USE_MOTOR_PROTOCOL_DSHOT
  DShotRMT* dshot_motors[NUM_MOTORS];
#endif

// Set the parameters for each motor here (ignore that it says ServoConfig):
ServoConfig motors[NUM_MOTORS] = {
/*
  Values for configuring servos:
  1) Pin - what pin on the ESP32 to use.  Most pins are available.  Check your specific board

  ** The remaining values only matter if you're using PWM ESCs.  OneShot and DShot ESCs ignore these **

  2) Servo refresh rate.  The default for nearly all PWM servos and ESCs is 50Hz.  Digital servos and some ESCs can go faster.  If you hear extra noise or heat from the servo, back this down.
      options are commonly 50 Hz (default), 150 Hz, and 333 Hz.  Other values may work as well.  YMMV.
  3) Minimum servo travel in micro-seconds (us). (default 900 us)
  4) Maximum servo travel in micro-seconds (us). (default 2100 us)
      The default signals output for servos are between 1000 and 2000.  You can adjust these for additional travel, but the code is currently constrained to 1000-2000.  That may change in future releases. 
  5) Servo Trim.  This floating point value is added to the inputs to adjust servo travel.  increment and decrement as needed to get your servo trimmed properly.  (Range: +/- 0.5, but you will need small increments)
  6) Initialization Angle.  This value (0-180) is applied to the servo at startup.  For control surfaces, you'll probably want this in the center (90). For ESCs, you'll want this at the minimum (0) for proper startup.
*/
// 1)  2)  3)   4)    5)   6)
  {38,  50, 900, 2100, 0.0, 0},
  {39,  50, 900, 2100, 0.0, 0},
  {40,  50, 900, 2100, 0.0, 0},
  {41,  50, 900, 2100, 0.0, 0}
};



// Set the parameters for each servo here:
ServoConfig servos[NUM_SERVOS] = {
/*
  Values for configuring servos:
  1) Pin - what pin on the ESP32 to use.  Most pins are available.  Check your specific board
  2) Servo refresh rate.  The default for nearly all PWM servos and ESCs is 50Hz.  Digital servos and some ESCs can go faster.  If you hear extra noise or heat from the servo, back this down.
      options are commonly 50 Hz (default), 150 Hz, and 333 Hz.  Other values may work as well.  YMMV.
  3) Minimum servo travel in micro-seconds (us). (default 900 us)
  4) Maximum servo travel in micro-seconds (us). (default 2100 us)
      The default signals output for servos are between 1000 and 2000.  You can adjust these for additional travel, but the code is currently constrained to 1000-2000.  That may change in future releases. 
  5) Servo Trim.  This floating point value is added to the inputs to adjust servo travel.  increment and decrement as needed to get your servo trimmed properly.  (Range: +/- 0.5, but you will need small increments)
  6) Initialization Angle.  This value (0-180) is applied to the servo at startup.  For control surfaces, you'll probably want this in the center (90). For ESCs, you'll want this at the minimum (0) for proper startup.
*/
// 1)  2)  3)   4)    5)   6)
  {4,  50, 900, 2100, 0.0, 90},
  {5,  50, 900, 2100, 0.0, 90},
  {6,  50, 900, 2100, 0.0, 90},
  {7,  50, 900, 2100, 0.0, 90},
  {14, 50, 900, 2100, 0.0, 90},
  {15, 50, 900, 2100, 0.0, 90},
  {21, 50, 900, 2100, 0.0, 90}
};


//========================================================================================================================//

//DECLARE GLOBAL VARIABLES

//General stuff
uint32_t print_counter = 0;
float dt = 0.0f;
uint32_t prev_time = 0;

unsigned long current_time;
unsigned long serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

//Radio communication:
unsigned long channel_pwm[NUM_CHANNELS] = { 0 };
unsigned long channel_pwm_prev[NUM_CHANNELS] = { 0 };

//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

//IMU:
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float MagX, MagY, MagZ;

//Controller:
float roll_PID, pitch_PID, yaw_PID = 0;

//Mixer
// Comparator handles for up to 8+ motors
mcpwm_cmpr_handle_t comparators[NUM_MOTORS];
float m_command_scaled[NUM_MOTORS] = { 0 };
int m_command_PWM[NUM_MOTORS] = { 0 };
float s_command_scaled[NUM_SERVOS] = { 0 };
int s_command_PWM[NUM_SERVOS] = { 0 };

//Flight status
bool armedFly = false;

// Forward declare functions in other .ino files
void printRadioData();
void printDesiredState();
void printGyroData();     
void printAccelData();    
void printMagData();      
void printRollPitchYaw(); 
void printPIDoutput();    
void printMotorCommands();
void printServoCommands();
void printLoopRate();     

void initWebServer();
void loadGainsFromFlash();
// End Function Delcares

// ==========================================
// THREAD-SAFE PID DATA STRUCTURE
// ==========================================
struct PIDGains {
    // Rate Mode Gains
    std::atomic<float> kp_rate{0.15f};
    std::atomic<float> ki_rate{0.20f};
    std::atomic<float> kd_rate{0.0002f};

    // Angle Mode Gains
    std::atomic<float> kp_angle{0.20f};
    std::atomic<float> ki_angle{0.01f};
    std::atomic<float> kd_angle{0.05f};
};

PIDGains rollPID;
PIDGains pitchPID;
PIDGains yawPID;
Preferences preferences; // Object for flash NVS storage








// ==========================================
// CORE 1: FLIGHT CONTROL TASK (2000 Hz Loop)
// ==========================================
void flightTask(void *pvParameters) {
  const uint32_t TARGET_LOOP_TIME_US = 500; // 500 us = 2000 Hz (2 kHz)
  uint32_t last_loop_time = micros();
  
  // Static persists across loops; initialized only ONCE on task startup
  static uint32_t last_time = 0; 
  
  // Seed initial time before entering the loop to avoid a massive first dt spike
  last_time = micros();
  
  for (;;) {
    uint32_t loop_start_time = micros();
    // UPDATE CURRENT_TIME FOR LOOPBLINK & OTHER FUNCTIONS
    current_time = loop_start_time;

    // Calculate dt properly using a local static or task-persistent variable
    dt = (loop_start_time - last_time) / 1000000.0f;
    last_time = loop_start_time;

    // 1. Load atomic web gains
    float Kp_roll_rate = rollPID.kp_rate.load(std::memory_order_relaxed);
    float Ki_roll_rate = rollPID.ki_rate.load(std::memory_order_relaxed);
    float Kd_roll_rate = rollPID.kd_rate.load(std::memory_order_relaxed);

    float Kp_pitch_rate = pitchPID.kp_rate.load(std::memory_order_relaxed);
    float Ki_pitch_rate = pitchPID.ki_rate.load(std::memory_order_relaxed);
    float Kd_pitch_rate = pitchPID.kd_rate.load(std::memory_order_relaxed);

    float Kp_yaw_rate = yawPID.kp_rate.load(std::memory_order_relaxed);
    float Ki_yaw_rate = yawPID.ki_rate.load(std::memory_order_relaxed);
    float Kd_yaw_rate = yawPID.kd_rate.load(std::memory_order_relaxed);

    float Kp_roll_angle = rollPID.kp_angle.load(std::memory_order_relaxed);
    float Ki_roll_angle = rollPID.ki_angle.load(std::memory_order_relaxed);
    float Kd_roll_angle = rollPID.kd_angle.load(std::memory_order_relaxed);

    float Kp_pitch_angle = pitchPID.kp_angle.load(std::memory_order_relaxed);
    float Ki_pitch_angle = pitchPID.ki_angle.load(std::memory_order_relaxed);
    float Kd_pitch_angle = pitchPID.kd_angle.load(std::memory_order_relaxed);

    float Kp_yaw_angle = yawPID.kp_angle.load(std::memory_order_relaxed);
    float Ki_yaw_angle = yawPID.ki_angle.load(std::memory_order_relaxed);
    float Kd_yaw_angle = yawPID.kd_angle.load(std::memory_order_relaxed);


    // 2. Run dRehmFlight 1.3 pipeline
    loopBlink(); //Indicate we are in main loop with short blink every 1.5 seconds

    //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
    //printRadioData();     //Prints radio pwm values (expected: 1000 to 2000)
    //printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
    //printGyroData();      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
    //printAccelData();     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
    //printMagData();       //Prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
    printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
    //printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
    //printMotorCommands(); //Prints the values being written to the motors (expected: 120 to 250)
    //printServoCommands(); //Prints the values being written to the servos (expected: 0 to 180)
    //printPIDvaluesRate();       //BJ - Prints PID values that, once tuned, can be hard coded into the rate/angle mode stabilization routines
    //printPIDvaluesAngle();       //BJ - Prints PID values that, once tuned, can be hard coded into the rate/angle mode stabilization routines
    //printLoopRate();      //Prints the time between loops in microseconds (expected: microseconds between loop iterations)

    // Get arming status
    armedStatus(); //Check if the throttle cut is off and throttle is low.

    //Get vehicle state
    getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
    Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)

    //Compute desired state
    getDesState(); //Convert raw commands to normalized values based on saturated control limits
    
    //PID Controller - SELECT ONE:
    //controlANGLE(); //Stabilize on angle setpoint
    //controlANGLE2(); //Stabilize on angle setpoint using cascaded method. Rate controller must be tuned well first!
    //controlRATE(); //Stabilize on rate setpoint
    // OR use a mode switch to select the mode:
    if (channel_pwm[5] > 900 && channel_pwm[5] < 1200) {
      controlANGLE2();
    } else if (channel_pwm[5] < 1800 && channel_pwm[5] > 1200) {
      controlANGLE();
    } else  {
      controlRATE();
    }


    //Actuator mixing and scaling to PWM values
    controlMixer(); //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
    scaleCommands(); //Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)

    //Throttle cut check
    throttleCut(); //Directly sets motor commands to low based on state of ch5

    //Command actuators
    commandMotors(); //Sends command pulses to each motor pin using OneShot125 protocol
    commandServos(); //Sends command pulses to each servo pin using PWM protocol

    //Get vehicle commands for next loop iteration
    getCommands(); //Pulls current available radio commands
    failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup


    // 2. --- TIMING & WATCHDOG YIELD ---
    uint32_t elapsed_time = micros() - loop_start_time;

    // Busy-wait for the exact remaining microseconds
    while ((micros() - loop_start_time) < TARGET_LOOP_TIME_US) {
      // Yielding briefly gives the ESP32 background tasks 
      // CPU time to prevent Task Watchdog Timer (TWDT) panics
      taskYIELD(); 
    }
  }
}

// ==========================================
// MAIN SETUP
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(500);

    // Uncomment this line to force a reset of PID values on reboot:
    //resetPIDToDefaults();

    // 1. Load saved PID values from NVS Flash before starting flight tasks
    loadGainsFromFlash();

    // Initializes a Webserver at 192.168.4.1 to dynamically update PIDs
    initWebServer();

    //Set built in LED to turn on to signal startup    
    pinMode(LEDPIN, OUTPUT); // LED blinker on board, do not modify 
    digitalWrite(LEDPIN, HIGH);
    delay(5);

    //Initialize all motors
    initMotors();
    // Initialize all servos
    initServos();
    delay(5);

    //Initialize radio communication
    radioSetup();
    delay(5);
    
    //Initialize IMU communication
    IMUinit();
    delay(5);

    // CALIBRATION ROUTINES
    // Enable these as needed to calibrate the craft.  Execution stops after each one of these, so use one at a time and comment them out after setup is completed.

    // 1) Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
    //calculate_IMU_error(); //Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.

    // 2) If using MPU9250 IMU, uncomment for one-time magnetometer calibration (may need to repeat for new locations)
    // calibrateMagnetometer(); //Generates magentometer error and scale factors to be pasted in user-specified variables section

    // 3) Uncomment to calibrate the ESCs.  Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle to zero after the beeps.
    // calibrateESCs();         //  PROPS OFF!! //
    // END CALIBRATION ROUTINES

    //Arm OneShot125 motors
    armMotors(); //Loop over commandMotors() until ESCs happily arm
    delay(5);
    
    // Indicate entering main loop with 3 quick blinks
    setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)

    // 4. Pin Flight Loop to Core 1
    xTaskCreatePinnedToCore(
        flightTask,
        "FlightLoop",
        8192,
        NULL,
        10,
        NULL,
        1
    );
}

void loop() {
    // Small delay to keep loopTask from hogging CPU when idle
    vTaskDelay(pdMS_TO_TICKS(10)); 
}






//========================================================================================================================//
//                                                      FUNCTIONS                                                         //                           
//========================================================================================================================//


void controlMixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
   * normalized (0 to 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with 
   * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables are used in scaleCommands() 
   * in preparation to be sent to the motor ESCs and servos.
   * 
   *Relevant variables:
   *thro_des - direct thottle control
   *roll_PID, pitch_PID, yaw_PID - stabilized axis variables
   *roll_passthru, pitch_passthru, yaw_passthru - direct unstabilized command passthrough
   *channel_6_pwm - free auxillary channel, can be used to toggle things with an 'if' statement
   */
   
  //Quad mixing - EXAMPLE
  m_command_scaled[0] = thro_des - pitch_PID + roll_PID + yaw_PID; //Front Left
  m_command_scaled[1] = thro_des - pitch_PID - roll_PID - yaw_PID; //Front Right
  m_command_scaled[2] = thro_des + pitch_PID - roll_PID + yaw_PID; //Back Right
  m_command_scaled[3] = thro_des + pitch_PID + roll_PID - yaw_PID; //Back Left
  // for (int i=0; i < NUM_MOTORS; i++) {
  //   m_command_scaled[i] = 0;
  // }
    
  //Quad mixing - EXAMPLE w/ SERVOS
  s_command_scaled[0] = thro_des - pitch_PID + roll_PID + yaw_PID; //Front Left
  s_command_scaled[1] = thro_des - pitch_PID - roll_PID - yaw_PID; //Front Right
  s_command_scaled[2] = thro_des + pitch_PID - roll_PID + yaw_PID; //Back Right
  s_command_scaled[3] = thro_des + pitch_PID + roll_PID - yaw_PID; //Back Left
  s_command_scaled[4] = 0;
  s_command_scaled[5] = 0;
  s_command_scaled[6] = 0;

  // 0.5 is centered servo, 0.0 is zero throttle if connecting to ESC for conventional PWM, 1.0 is max throttle
  // for (int i = 0; i < NUM_SERVOS; i++) {
  //   s_command_scaled[i] = 0;
  // }
 
}

void armedStatus() {
  //DESCRIPTION: Check if the throttle cut is off and the throttle input is low to prepare for flight.
  if ((channel_pwm[4] < 1500) && (channel_pwm[0] < 1050)) {
    armedFly = true;
  }
}


void getDesState() {
  //DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
   * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
   * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
   * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
   */
  thro_des = (channel_pwm[0] - 1000.0)/1000.0; //Between 0 and 1
  roll_des = (channel_pwm[1] - 1500.0)/500.0; //Between -1 and 1
  pitch_des = (channel_pwm[2] - 1500.0)/500.0; //Between -1 and 1
  yaw_des = (channel_pwm[3] - 1500.0)/500.0; //Between -1 and 1
  roll_passthru = roll_des/2.0; //Between -0.5 and 0.5
  pitch_passthru = pitch_des/2.0; //Between -0.5 and 0.5
  yaw_passthru = yaw_des/2.0; //Between -0.5 and 0.5
  
  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
  // Use the rate-mode or angle-mode limits depending on which mode is
  // currently active (same channel_pwm[5] switch flightTask() uses to
  // choose controlRATE() vs controlANGLE()).
  bool rateModeActive = (channel_pwm[5] > 1800);
  float maxRollNow  = rateModeActive ? maxRoll_rate  : maxRoll_angle;
  float maxPitchNow = rateModeActive ? maxPitch_rate : maxPitch_angle;

  roll_des = constrain(roll_des, -1.0, 1.0)*maxRollNow; //Between -maxRollNow and +maxRollNow
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitchNow; //Between -maxPitchNow and +maxPitchNow
  yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw
  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}


void scaleCommands() {
  //DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo protocol
  /*
   * mX_command_scaled variables from the mixer function are scaled to 125-250us for OneShot125 protocol. sX_command_scaled variables from
   * the mixer function are scaled to 0-180 for the servo library using standard PWM.
   * mX_command_PWM are updated here which are used to command the motors in commandMotors(). sX_command_PWM are updated 
   * which are used to command the servos.
   */

  for (int i=0; i<NUM_MOTORS; i++) {
    // The different ESC protocols use different ranges
    // PWM ESCs
    #if defined USE_MOTOR_PROTOCOL_PWM
      m_command_PWM[i] = m_command_scaled[i] * 180;
      m_command_PWM[i] = constrain(m_command_PWM[i], 0, 180);
    #endif
    //OneShot125 ESCs:
    #if defined USE_MOTOR_PROTOCOL_ONESHOT
      //Scaled to 125us - 250us for oneshot125 protocol
      m_command_PWM[i] = m_command_scaled[i]*125 + 125;
      //Constrain commands to motors within oneshot125 bounds
      m_command_PWM[i] = constrain(m_command_PWM[i], 125, 250);
    #endif
    //DShot ESCs:
    #if defined USE_MOTOR_PROTOCOL_DSHOT
      // DShot has a "stop" command of 0.  Use that when disarmed
      if (!armedFly) {
        m_command_PWM[i] = 0;
      }
      else {
        int dshot_scale = 2000 - DSHOT_MINTHROTTLE;
        m_command_PWM[i] = m_command_scaled[i] * dshot_scale + (DSHOT_MINTHROTTLE + 48);
        m_command_PWM[i] = constrain(m_command_PWM[i], DSHOT_MINTHROTTLE, 2047);
      }
    #endif
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
    //Add servo trim value and scaled to 0-180 for servo library
    s_command_PWM[i] = (s_command_scaled[i] + servos[i].trim) * 180;
    s_command_PWM[i] = constrain(s_command_PWM[i], 0, 180);
  }
}

void getCommands() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of 
   * the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which 
   * is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the alues are pulled from the SBUS library directly.
   * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise. 
   */

  #if defined USE_PPM_RX || defined USE_PWM_RX
    // for (int i = 0; i < NUM_CHANNELS; i++) {
    //   channel_pwm[i] = getRadioPWM(i);
    // }
    
  #elif defined USE_SBUS_RX
    if (sbus.Read()) {
      bfs::SbusData sbusData = sbus.data();
      sbusFailSafe = sbusData.failsafe;
      sbusLostFrame = sbusData.lost_frame;
      //sBus scaling below is for Taranis-Plus and X4R-SB
      float scale = 0.615;  
      float bias  = 895.0; 
      for (int i = 0; i < NUM_CHANNELS; i++) {
        channel_pwm[i] = sbusData.ch[i] * scale + bias;
      }
    }

  #elif defined USE_DSM_RX
    // if (DSM.timedOut(micros())) {
    //     //Serial.println("*** DSM RX TIMED OUT ***");
    // }
    // else if (DSM.gotNewFrame()) {
    //   uint16_t values[num_DSM_channels];
    //   DSM.getChannelValues(values, num_DSM_channels);

    //   for (int i = 0; i < NUM_CHANNELS; i++) {
    //     channel_pwm[i] = values[i];
    //   }
    // }

  #elif defined USE_CRSF_RX
    crsf.update();   // must be called regularly

    if (crsf.isLinkUp()) {
      crsfFailSafe = false;
      for (int i = 0; i < NUM_CHANNELS; i++) {
        // ELRS uses arrays starting a 1, we use 0 starting arrays
        channel_pwm[i] = crsf.getChannel(i+1);
      }
    } else {
      // Failsafe activated
      crsfFailSafe = true;
    }
  #endif
  
  //Low-pass the critical commands and update previous values
  float b = 0.7; //Lower=slower, higher=noiser
  for (int i = 0; i < 4; i++) {
    channel_pwm[i] = (1.0 - b)*channel_pwm_prev[i] + b*channel_pwm[i];
    channel_pwm_prev[i] = channel_pwm[i];
  }
}

void failSafe() {
  //DESCRIPTION: If radio gives garbage values, set all commands to default values
  /*
   * Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of 
   * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
   * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands 
   * channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting 
   * your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
   * 
   * When using SBUS, the library handles the detection of lost frames and failsafe.  Some receivers handle failsafe differently.
   * Testing with FrSky SBUS receivers immediately triggered failsafe when Tx disconnected.  A Radiomaster RP3-H ELRS receiver would
   * not trigger failsafe when set to "No Pulses", but would trigger when set to "Last Position" after a few seconds.
   * Test your receiver's behavior! You can do this with the example sketch in the "Bolder Flight Systems SBUS" library.
   */

  //If using SBUS, this will only check the boolean failsafe value from the SBUS library.  If using PWM RX, it checks each channel.
  #if !defined(USE_SBUS_RX) && !defined(USE_CRSF_RX)
    unsigned minVal = 900;
    unsigned maxVal = 2100;
    int check[NUM_CHANNELS] = { 0 };
    int checksum = 0;

    //Triggers for failure criteria
    for (int i = 0; i < NUM_CHANNELS; i++) {
      if (channel_pwm[i] > maxVal || channel_pwm[i] < minVal) check[i] = 1;
    }
  #endif

  //If any failures, set to default failsafe values
  #if defined USE_SBUS_RX
  if (sbusFailSafe) {
  #elif defined USE_CRSF_RX
  if (crsfFailSafe) {
  #else
  // if somehow you got here w/o the receiver type defined, failsafe
  if (true) {
  #endif
    for (int i = 0; i < NUM_CHANNELS; i++) {
      channel_pwm[i] = channel_fs[i];
    }
  }
}

void initMotors() {
  // PWM-based ESCs:
  #if defined USE_MOTOR_PROTOCOL_PWM
    for (int i = 0; i < NUM_MOTORS; i++) {
      // Compute duty cycle (in us) from Hz
      int dutyCycle = 1000000 / motors[i].refresh;
      
      // Convert minUs and maxUs to 12-bit duty cycles and set to servo object
      motors[i].minDuty = (motors[i].minUs * 4095) / dutyCycle;
      motors[i].maxDuty = (motors[i].maxUs * 4095) / dutyCycle;

      // ledcAttach(pin, frequency_hz, resolution_bits)
      ledcAttach(motors[i].pin, motors[i].refresh, 12); 

      //Arm servo channels
      writeServo(motors[i], motors[i].initAngle);
    }
  #endif
  
  // Initialization for OneShot125 Protocol
  #if defined USE_MOTOR_PROTOCOL_ONESHOT
    mcpwm_timer_handle_t timer_grp0 = NULL;
    mcpwm_timer_handle_t timer_grp1 = NULL;

    // Set motor pins for OUTPUT mode
    for (int i = 0; i < NUM_MOTORS; i++) {
      pinMode(motors[i].pin, OUTPUT);
    }

    // 1. Configure MCPWM Timer for Group 0 (Motors 0 to 5)
    mcpwm_timer_config_t timer_config_0 = {};
    timer_config_0.group_id = 0;
    timer_config_0.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config_0.resolution_hz = 1000000; // 1 MHz tick = 1 us resolution
    timer_config_0.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config_0.period_ticks = 500;      // 500 us period (2 kHz update rate)

    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config_0, &timer_grp0));

    // 2. Configure MCPWM Timer for Group 1 (Motors 6 to 7+, if applicable)
    mcpwm_timer_config_t timer_config_1 = {};
    timer_config_1.group_id = 1;
    timer_config_1.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config_1.resolution_hz = 1000000;
    timer_config_1.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config_1.period_ticks = 500;

    if (NUM_MOTORS > 6) {
      ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config_1, &timer_grp1));
    }

    mcpwm_oper_handle_t current_oper = NULL;

    // 3. Assign each motor pin to an MCPWM Operator & Generator
    for (int i = 0; i < NUM_MOTORS; i++) {
      int group_id = i / 6; // Group 0 for motors 0-5, Group 1 for motors 6-11
      mcpwm_timer_handle_t current_timer = (group_id == 0) ? timer_grp0 : timer_grp1;

      // Each MCPWM operator can handle up to 2 generator outputs
      if (i % 2 == 0) {
          mcpwm_operator_config_t oper_config = {};
          oper_config.group_id = group_id;
          ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &current_oper));
          ESP_ERROR_CHECK(mcpwm_operator_connect_timer(current_oper, current_timer));
      }

      // Configure pulse comparator (updates at zero to avoid glitches)
      mcpwm_comparator_config_t cmpr_config = {};
      cmpr_config.flags.update_cmp_on_tez = true;
      ESP_ERROR_CHECK(mcpwm_new_comparator(current_oper, &cmpr_config, &comparators[i]));

      // Attach generator output to the motor pin
      mcpwm_generator_config_t gen_config = {};
      gen_config.gen_gpio_num = (gpio_num_t)motors[i].pin;

      mcpwm_gen_handle_t generator = NULL;
      ESP_ERROR_CHECK(mcpwm_new_generator(current_oper, &gen_config, &generator));

      // Signal Logic: Drive HIGH at period start (0), Drive LOW when comparator match occurs
      ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generator,
          MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
          MCPWM_GEN_TIMER_EVENT_ACTION_END()));
      
      ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generator,
          MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i], MCPWM_GEN_ACTION_LOW),
          MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

      // Default state: 125 us idle signal
      mcpwm_comparator_set_compare_value(comparators[i], ONESHOT125_MIN_US);
    }

    // 4. Start MCPWM timers
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer_grp0));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_grp0, MCPWM_TIMER_START_NO_STOP));

    if (NUM_MOTORS > 6) {
      ESP_ERROR_CHECK(mcpwm_timer_enable(timer_grp1));
      ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_grp1, MCPWM_TIMER_START_NO_STOP));
    }
  #endif

  // DShot ESC Protocol
  #if defined USE_MOTOR_PROTOCOL_DSHOT
    for (int i = 0; i < NUM_MOTORS; i++) {
      dshot_motors[i] = new DShotRMT(motors[i].pin, DSHOT_MODE, false);
      dshot_result_t dshot_result = dshot_motors[i]->begin();
      printDShotResult(dshot_result);
    }
  #endif
}


void commandMotors() {
  // Loop over the motors and send the output commands
  for (int i = 0; i < NUM_MOTORS; i++) {
    // For PWM ESCs
    #if defined USE_MOTOR_PROTOCOL_PWM
      writeServo(motors[i], m_command_PWM[i]);
    #endif

    // For OneShot ESCs
    #if defined USE_MOTOR_PROTOCOL_ONESHOT
      uint32_t pulse_us;

      if (!armedFly) {
        pulse_us = ONESHOT125_MIN_US;
      } else {
        pulse_us = constrain(m_command_PWM[i], ONESHOT125_MIN_US, ONESHOT125_MAX_US);
      }

      // Update hardware compare register instantly
      mcpwm_comparator_set_compare_value(comparators[i], pulse_us);
    #endif

    // For DShot ESCs
    #if defined USE_MOTOR_PROTOCOL_DSHOT
      dshot_result_t dshot_result = dshot_motors[i]->sendThrottle(m_command_PWM[i]);
      // printDShotResult(dshot_result);
    #endif
  }
}



void armMotors() {
  //DESCRIPTION: Sends many command pulses to the motors, to be used to arm motors in the void setup()
  /*  
   *  Loops over the commandMotors() function 50 times with a delay in between, simulating how the commandMotors()
   *  function is used in the main loop. Ensures motors arm within the void setup() where there are some delays
   *  for other processes that sometimes prevent motors from arming.
   */

  // Set motor command to minimum throttle (arming0)
  for (int i=0; i<NUM_MOTORS; i++) {
    #if defined(USE_MOTOR_PROTOCOL_PWM) || defined(USE_MOTOR_PROTOCOL_DSHOT)
      m_command_PWM[i] = 0;
    #else //Oneshot125
      m_command_PWM[i] = 125;
    #endif
  }

  // Loop through and send pulses to arm
  for (int i = 0; i <= 50; i++) {
    commandMotors();
    delay(2);
  }
}








void calibrateESCs() {
  //DESCRIPTION: Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
  /*  
   *  Simulates the void loop(), but only for the purpose of providing throttle pass through to the motors, so that you can
   *  power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero. This function should only be
   *  uncommented when performing an ESC calibration.
   */
   while (true) {
      prev_time = current_time;      
      current_time = micros();      
      dt = (current_time - prev_time)/1000000.0;
    
      digitalWrite(LEDPIN, HIGH); //LED on to indicate we are not in main loop

      getCommands(); //Pulls current available radio commands
      failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
      getDesState(); //Convert raw commands to normalized values based on saturated control limits

      // These aren't needed for an ESC calibration
      // getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
      // Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)
      // getDesState(); //Convert raw commands to normalized values based on saturated control limits
      
      // Set the outputs all to the raw throttle command
      // comment out any unneeded outputs
      for (int i=0; i<NUM_MOTORS; i++) {
        m_command_scaled[i] = thro_des;
      }
      for (int i = 0; i < NUM_SERVOS; i++) {
        s_command_scaled[i] = thro_des;
      }
      scaleCommands(); //Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)
    
      //throttleCut(); //Directly sets motor commands to low based on state of ch5
      
      // Send the outputs to servos/motors
      commandServos(); //Sends command pulses to each servo pin using PWM protocol
      commandMotors(); //Sends command pulses to each motor pin using OneShot125 protocol
      
      // Print the radio data as a guide that something is happening
      printRadioData(); //Radio pwm values (expected: 1000 to 2000)
      
      loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
   }
}




float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq){
  //DESCRIPTION: Linearly fades a float type variable between min and max bounds based on desired high or low state and time
  /*  
   *  Takes in a float variable, desired minimum and maximum bounds, fade time, high or low desired state, and the loop frequency 
   *  and linearly interpolates that param variable between the maximum and minimum bounds. This function can be called in controlMixer()
   *  and high/low states can be determined by monitoring the state of an auxillarly radio channel. For example, if channel_6_pwm is being 
   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical 
   *  statements in order to fade controller gains, for example between the two dynamic configurations. The 'state' (1 or 0) can be used
   *  to designate the two final options for that control gain based on the dynamic configuration assignment to the auxillary radio channel.
   *  
   */
  float diffParam = (param_max - param_min)/(fadeTime*loopFreq); //Difference to add or subtract from param for each loop iteration for desired fadeTime

  if (state == 1) { //Maximum param bound desired, increase param by diffParam for each loop iteration
    param = param + diffParam;
  }
  else if (state == 0) { //Minimum param bound desired, decrease param by diffParam for each loop iteration
    param = param - diffParam;
  }

  param = constrain(param, param_min, param_max); //Constrain param within max bounds
  
  return param;
}


float floatFaderLinear2(float param, float param_des, float param_lower, float param_upper, float fadeTime_up, float fadeTime_down, int loopFreq){
  //DESCRIPTION: Linearly fades a float type variable from its current value to the desired value, up or down
  /*  
   *  Takes in a float variable to be modified, desired new position, upper value, lower value, fade time, and the loop frequency 
   *  and linearly fades that param variable up or down to the desired value. This function can be called in controlMixer()
   *  to fade up or down between flight modes monitored by an auxillary radio channel. For example, if channel_6_pwm is being 
   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical 
   *  statements in order to fade controller gains, for example between the two dynamic configurations. 
   *  
   */
  if (param > param_des) { //Need to fade down to get to desired
    float diffParam = (param_upper - param_des)/(fadeTime_down*loopFreq);
    param = param - diffParam;
  }
  else if (param < param_des) { //Need to fade up to get to desired
    float diffParam = (param_des - param_lower)/(fadeTime_up*loopFreq);
    param = param + diffParam;
  }

  param = constrain(param, param_lower, param_upper); //Constrain param within max bounds
  
  return param;
}

void switchRollYaw(int reverseRoll, int reverseYaw) {
  //DESCRIPTION: Switches roll_des and yaw_des variables for tailsitter-type configurations
  /*
   * Takes in two integers (either 1 or -1) corresponding to the desired reversing of the roll axis and yaw axis, respectively.
   * Reversing of the roll or yaw axis may be needed when switching between the two for some dynamic configurations. Inputs of 1, 1 does not 
   * reverse either of them, while -1, 1 will reverse the output corresponding to the new roll axis. 
   * This function may be replaced in the future by a function that switches the IMU data instead (so that angle can also be estimated with the 
   * IMU tilted 90 degrees from default level).
   */
  float switch_holder;

  switch_holder = yaw_des;
  yaw_des = reverseYaw*roll_des;
  roll_des = reverseRoll*switch_holder;
}

void throttleCut() {
  //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
      Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (120 is
      minimum for oneshot125 protocol, 0 is minimum for standard PWM servo library used) if channel 5 is high. This is the last function
      called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
      the motors to anything other than minimum value. Safety first.

      channel_5_pwm is LOW then throttle cut is OFF and throttle value can change. (ThrottleCut is DEACTIVATED)
      channel_5_pwm is HIGH then throttle cut is ON and throttle value = 120 only. (ThrottleCut is ACTIVATED), (drone is DISARMED)
  */
  if ((channel_pwm[4] > 1500) || (armedFly == false)) {
    armedFly = false;
    for (int i=0; i<NUM_MOTORS; i++) {
      // the values for the motor commands are different if they are PWM, OneShot, or DShot ESCs:
      #if defined(USE_MOTOR_PROTOCOL_PWM) || defined(USE_MOTOR_PROTOCOL_DSHOT)
        m_command_PWM[i] = 0;
      #else //Oneshot125
        m_command_PWM[i] = 120;
      #endif        
    }
  }
}


// Loop over the servos and send the commands
void commandServos() {
  for (int i=0; i<NUM_SERVOS; i++) {
    writeServo(servos[i], s_command_PWM[i]);
  }
}

// Helper function to write angles (0-180 deg) using 50Hz PWM
void writeServo(ServoConfig &s, int angle) {
  // Constrain angle between 0 and 180
  angle = constrain(angle, 0, 180);
  
  // map angle to duty cycle
  int duty = map(angle, 0, 180, s.minDuty, s.maxDuty);
  
  // write duty cycle to servo pin
  ledcWrite(s.pin, duty);
}


// Helper function to write angles (0-180 deg) using 50Hz PWM
void initServos() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    // Compute duty cycle (in us) from Hz
    int dutyCycle = 1000000 / servos[i].refresh;
    
    // Convert minUs and maxUs to 12-bit duty cycles and set to servo object
    servos[i].minDuty = (servos[i].minUs * 4095) / dutyCycle;
    servos[i].maxDuty = (servos[i].maxUs * 4095) / dutyCycle;

    // ledcAttach(pin, frequency_hz, resolution_bits)
    ledcAttach(servos[i].pin, servos[i].refresh, 12); 

    //Arm servo channels
    writeServo(servos[i], servos[i].initAngle);
  }
}














// TODO this probably needs to go away
void loopRate(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}


void loopBlink() {
  //DESCRIPTION: Blink LED on board to indicate main loop is running
  /*
   * It looks cool.
   */
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(LEDPIN, blinkAlternate); //Pin 2 is built in LED
    
    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
      }
    else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
      }
  }
}

void setupBlink(int numBlinks,int upTime, int downTime) {
  //DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j<= numBlinks; j++) {
    digitalWrite(LEDPIN, LOW);
    delay(downTime);
    digitalWrite(LEDPIN, HIGH);
    delay(upTime);
  }
}
