//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 7/29/2022
//Version: Beta 2.0
 
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

*/


//REQUIRED LIBRARIES (included with download in main sketch folder)
// #include "src/dRhemFlight_hardware.h"
#include "src/IMU_Wrapper/IMU_Wrapper.h"
#include "src/RadioComm/RadioComm.h"
#include "src/Actuators/Actuators.h"


//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//



//Controller parameters (take note of defaults before modifying!): 
float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 160.0;     //Max yaw rate in deg/sec



//========================================================================================================================//
//                                                     DECLARE PINS                                                       //                           
//========================================================================================================================//                                          

//NOTE: Pin 13 is reserved for onboard LED


//OneShot125 ESC pin outputs.  Set the number of Servos you have and update the pin outputs:
const uint8_t numMotors = 6;
uint8_t motorPins[numMotors] = {0, 1, 2, 3, 4, 5};
// Mixer commands from 0.0 to 1.0 (will be scaled in the library)
float m_command_scaled[numMotors] = {0.0};  // Initialize in idle
Actuators Motors(ONESHOT125, numMotors, motorPins);


//PWM servo outputs.  Set the number of Servos you have and update the pin outputs:
const uint8_t numServos = 2;
uint8_t servoPins[numServos] = {6, 7};
// Mixer commands from 0.0 to 1.0 (will be scaled in the library)
float s_command_scaled[numServos] = {0.5};  // Default to centered.  This can be customized.
Actuators Servos(SERVO, numServos, servoPins);


//PWM ESC outputs.  Set the number of ESCs you have and update the pin outputs:
const uint8_t numESCs = 2;
uint8_t escPins[numESCs] = {8, 9};
// Mixer commands from 0.0 to 1.0 (will be scaled in the library)
float e_command_scaled[numESCs] = {0.0};  // Initialize in idle
Actuators Escs(ESC, numESCs, escPins);



//========================================================================================================================//
// declare IMU and pins
IMU_Wrapper imu(LSM6DSOX_SPI);
// values are MPU6050_I2C (Default), MPU9250_SPI, LSM6DSOX_SPI
// Defaults to  GYRO_250DPS and ACCEL_2G.  You can change the sensitivity by altering these in the call above.
// example:  IMU imu(LSM6DSOX_SPI, GYRO_1000DPS, ACCEL_8G);

// Needed for MPU9250_SPI and LSM6DSOX_SPI.  Comment these out for MPU6050_I2C
#define IMU_CS   10    // CS--green-yellow
#define IMU_SCK  13    // SCK/SCL--white
#define IMU_MISO 12    // MISO/DO--red
#define IMU_MOSI 11    // MOSI/SDA--black

/*
IMU Pin locations for IMU:

defaults for Teensy 4.0:
  default pins for MPU6050 using I2C:
    SDA -> pin 18
    SCL -> pin 19

  ** NOTE:  You can use pins 10-13 for SPI communication, but it will take away from the available servo/motor outputs **
  default pins for MPU9250 using SPI:
    CS   -> pin 36  OR  10
    SCK  -> pin 37  OR  13
    MISO -> pin 34  OR  12
	  MOSI -> pin 35  OR  11

  default pins for Adafruit LSM6DSOX using SPI:
    CS   -> pin 36  OR  10
    SCK  -> pin 37  OR  13
    MISO -> pin 34  OR  12
	  MOSI -> pin 35  OR  11
*/
//========================================================================================================================//



//========================================================================================================================//
// Radio communication:
// How many input channels do you have?
// Max number of channels is 8 for PPM and 16 for SBUS.
#define NUM_CHANNELS  8

// Note: If using SBUS, connect to pin 21 (RX5), if using DSM, connect to pin 15 (RX3)
// For PPM:
const int PPM_Pin = 23;

// Define the radio object
// Set the type of RC radio you are using
// Options options are:  PPM, DSM, or sBUS.
//RadioComm radio(sBUS, NUM_CHANNELS);
// The default channel order for dRhemFlight is TAER (Thr, Ail, Ele, Rud), if your radio/receiver outputs AETR (Ail, Ele, Thr, Rud), you can use that channel order like this:
// TAER is the default, AETR is the only other option right now.
RadioComm radio(PPM, NUM_CHANNELS, AETR);

// Holder array for radio PWM data
uint16_t channel_pwm[NUM_CHANNELS];

//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
uint16_t channel_fs[NUM_CHANNELS] = {1000, 1500, 1500, 1500, 1000, 1000, 1000, 1000};
//                                   thr,  ail,  ele,  rud,  arm,  aux1, aux2, aux3
//========================================================================================================================//



// DECLARE PRINTING FUNCTIONS (IN printFunctions.ino)
void printRadioData();     //Prints radio pwm values (expected: 1000 to 2000)
void printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
void printGyroData();      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
void printAccelData();     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
void printMagData();       //Prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
void printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
void printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
void printMotorCommands(); //Prints the values being written to the motors (expected: 120 to 250)
void printServoCommands(); //Prints the values being written to the servos (expected: 0 to 180)
void printLoopRate();      //Prints the time between loops in microseconds (expected: microseconds between loop iterations)
void printLoopRateAndProfiledTimes();


//DECLARE SYSTEM GLOBAL VARIABLES
//General stuff
#define PRINTLOOPINTERVAL 10000
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

//Flight status
bool armedFly = false;

// Loop rate profiling
unsigned long loopRateLastPrintMicros = 0;
unsigned long loopRateCounter = 0;

// Profiling timers for individual functions
unsigned long time_getIMUdata_us = 0, time_Madgwick_us = 0, time_getDesState_us = 0, time_faderLogic_us = 0, time_controlANGLE_us = 0, time_controlMixer_us = 0;
unsigned long time_scaleCommands_us = 0, time_commandMotors_us = 0, time_servoWrites_us = 0, time_getCommands_us = 0, time_failSafe_us = 0, time_totalLoopInternal_us = 0;

//Normalized desired state:
// NOTE: Additional controls, such as collective pitch, tilt servos, etc. can be added here.
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;




//========================================================================================================================//
//                                                      VOID SETUP                                                        //                           
//========================================================================================================================//

void setup() {
  Serial.begin(500000); //USB serial
  delay(500);
  
  //Initialize all pins
  // pinMode(13, OUTPUT); //Pin 13 LED blinker on board, do not modify 

  //Initialize and arm servo channels
  Motors.begin();
  Servos.begin();
  Escs.begin();

  //Set built in LED to turn on to signal startup
  // digitalWrite(13, HIGH);

  delay(500);


  //========================================================================================================================//
  //Initialize radio communication
  //radio.begin();   //sBUS or DSM
  radio.begin(PPM_Pin);   //PPM

  //Set radio channels to default (safe) values before entering main loop
  radio.setFailsafe(channel_fs);
  // Set the radio object in failsafe mode until it comes online.  In my testing, the SBUS library doesn't return any data until it's successfully connected
  radio.failSafe(true);

  //========================================================================================================================//
  

  //========================================================================================================================//
  //Initialize IMU communication
  // Needed for MPU9250_SPI and LSM6DSOX_SPI.  Comment this out for MPU6050_I2C
  imu.setSPIpins(IMU_CS, IMU_SCK, IMU_MISO, IMU_MOSI);

  if (!imu.begin()) {  
    Serial.println("IMU init failed!");
    while (1);
  }
  delay(500);

  // Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  // imu.calculate_IMU_error(); //Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.
  
  // Replace the following with the values from the calculate_IMU_error() output above:
  imu.setAccError(0.0, 0.0, 0.0);
  imu.setGyroError(0.0, 0.0, 0.0);

  // If using MPU9250 IMU, uncomment for one-time magnetometer calibration (may need to repeat for new locations)
  // imu.calibrateMagnetometer(); //Generates magentometer error and scale factors to be pasted in user-specified variables section

  // If using MPU9250 IMU, replace the below with the output from calibrateMagnetometer() above
  // imu.setMagError(0.0, 0.0, 0.0);
  // imu.setMagScale(1.0, 1.0, 1.0);

  // This sets the PID values for Rate and Angle modes based on Nick's Github defaults.  These can be altered within the control loop as well for adjustments in flight either with fader variables, control inputs, etc.
  // I put these here as an example of use.
  imu.setRollAnglePID(0.2, 0.3, 0.05);
  imu.setPitchAnglePID(0.2, 0.3, 0.05);

  imu.setRollRatePID(0.15, 0.2, 0.0002);
  imu.setPitchRatePID(0.15, 0.2, 0.0002);

  imu.setYawRatePID(0.3, 0.05, 0.00015);



  //========================================================================================================================//

  delay(500);

  //calibrateESCs(); //PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle to zero after the beeps
  //Code will not proceed past here if this function is uncommented!


  //Indicate entering main loop with 3 quick blinks
  // setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)
}



//========================================================================================================================//
//                                                       MAIN LOOP                                                        //                           
//========================================================================================================================//
                                                  
void loop() {
  //Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;
  loopRateCounter++;
  unsigned long section_start_us;

  // loopBlink(); //Indicate we are in main loop with short blink every 1.5 seconds

  //Get vehicle state
  section_start_us = micros();
  imu.getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  time_getIMUdata_us = micros() - section_start_us;

  section_start_us = micros();
  imu.Madgwick(dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
  time_Madgwick_us = micros() - section_start_us;

  section_start_us = micros();
  //Get vehicle commands for next loop iteration
  radio.getCommands(channel_pwm); //Pulls current available radio commands
  time_getCommands_us = micros() - section_start_us;
  
  section_start_us = micros();
  // radio.failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
  time_failSafe_us = micros() - section_start_us;

  //Compute desired state
  section_start_us = micros();
  getDesState();  //Convert raw commands to normalized values based on saturated control limits
  time_getDesState_us = micros() - section_start_us;


  section_start_us = micros();
  //PID Controller - SELECT ONE:
  imu.controlANGLE(roll_des, pitch_des, yaw_des, dt, channel_pwm[0]); //Stabilize on angle setpoint
  //imu.controlANGLE2(roll_des, pitch_des, yaw_des, dt, channel_pwm[0]); //Stabilize on angle setpoint using cascaded method. Rate controller must be tuned well first!
  //imu.controlRATE(roll_des, pitch_des, yaw_des, dt, channel_pwm[0]); //Stabilize on rate setpoint
  time_controlANGLE_us = micros() - section_start_us;

  //Actuator mixing and scaling to PWM values
  section_start_us = micros();
  controlMixer();
  time_controlMixer_us = micros() - section_start_us;


  // Get arming status
  armedStatus(); //Check if the throttle cut is off and throttle is low.
  //Throttle cut check
  throttleCut(); //Directly sets motor commands to low based on state of ch5


  // Command actuators
  // Write outputs to all servos, ESCs, and motors
  section_start_us = micros();
  Motors.commandAll(m_command_scaled);
  time_commandMotors_us = micros() - section_start_us;

  section_start_us = micros();
  Escs.commandAll(e_command_scaled);
  Servos.commandAll(s_command_scaled);
  time_servoWrites_us = micros() - section_start_us;

  // ----------------------------------------------------------------
  // Debugging & Loop Rate Control
  // ----------------------------------------------------------------
  time_totalLoopInternal_us = micros() - current_time;



  //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
  printRadioData();     //Prints radio pwm values (expected: 1000 to 2000)
  //printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
  //printGyroData();      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
  //printAccelData();     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
  //printMagData();       //Prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
  //printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
  //printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
  //printMotorCommands(); //Prints the values being written to the motors (expected: 120 to 250)
  //printServoCommands(); //Prints the values being written to the servos (expected: 0 to 180)
  printLoopRate();      //Prints the time between loops in microseconds (expected: microseconds between loop iterations)
  //printLoopRateAndProfiledTimes();


  //Regulate loop rate
  loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}



//========================================================================================================================//
//                                                 MIXER FUNCTIONS                                                        //                           
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
  m_command_scaled[0] = thro_des - imu.pitch_PID + imu.roll_PID + imu.yaw_PID; //Front Left
  m_command_scaled[1] = thro_des - imu.pitch_PID - imu.roll_PID - imu.yaw_PID; //Front Right
  m_command_scaled[2] = thro_des + imu.pitch_PID - imu.roll_PID + imu.yaw_PID; //Back Right
  m_command_scaled[3] = thro_des + imu.pitch_PID + imu.roll_PID - imu.yaw_PID; //Back Left
  m_command_scaled[4] = 0;
  m_command_scaled[5] = 0;

  s_command_scaled[0] = thro_des - imu.pitch_PID + imu.roll_PID + imu.yaw_PID; //Front Left
  s_command_scaled[1] = thro_des - imu.pitch_PID + imu.roll_PID + imu.yaw_PID; //Front Left

  e_command_scaled[0] = thro_des;
  e_command_scaled[1] = thro_des;



  //0.5 is centered servo, 0.0 is zero throttle if connecting to ESC for conventional PWM, 1.0 is max throttle
  // for (int i=0; i<numServos; i++) {
  //   s_command_scaled[i] = 0.5;
  // }
  // for (int i=0; i<numESCs; i++) {
  //   e_command_scaled[i] = 0.0;
  // }

 
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





//========================================================================================================================//
//                                 ADDITIONAL STARTUP / SUPPORT FUNCTIONS                                                 //                           
//========================================================================================================================//

void getDesState() {
  //DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
   * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
   * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
   * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
   */

  // Note that additional controls, such as collective pitch, tilt servos, etc. can be added here.

  thro_des = (channel_pwm[0] - 1000.0)/1000.0; //Between 0 and 1
  roll_des = (channel_pwm[1] - 1500.0)/500.0; //Between -1 and 1
  pitch_des = (channel_pwm[2] - 1500.0)/500.0; //Between -1 and 1
  yaw_des = (channel_pwm[3] - 1500.0)/500.0; //Between -1 and 1
  roll_passthru = roll_des/2.0; //Between -0.5 and 0.5
  pitch_passthru = pitch_des/2.0; //Between -0.5 and 0.5
  yaw_passthru = yaw_des/2.0; //Between -0.5 and 0.5
  
  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw
  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

void armedStatus() {
  //DESCRIPTION: Check if the throttle cut is off and the throttle input is low to prepare for flight.
  if ((channel_pwm[4] > 1500) && (channel_pwm[0] < 1050)) {
    if (armedFly == false) {
      Serial.println(F("ARMED!"));
    }
    armedFly = true;
  }
}

void calibrateAttitude() {
  //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
  //Assuming vehicle is powered up on level surface!
  /*
   * This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds
   * to boot. 
   */
  //Warm up IMU and madgwick filter in simulated main loop
  for (int i = 0; i <= 10000; i++) {
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0; 
    imu.getIMUdata();
    imu.Madgwick(dt);
    loopRate(2000); //do not exceed 2000Hz
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
    
      // digitalWrite(13, HIGH); //LED on to indicate we are not in main loop

      radio.getCommands(channel_pwm); //Pulls current available radio commands
      // radio.failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
      // getDesState(); //Convert raw commands to normalized values based on saturated control limits -- this was in here 2x.  Not sure that's needed
      imu.getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
      imu.Madgwick(dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)
      getDesState(); //Convert raw commands to normalized values based on saturated control limits

      // Set all outputs tied to the throttle input
      for (int i=0; i<numMotors; i++) {
        m_command_scaled[i] = thro_des;
      }
      for (int i=0; i<numServos; i++) {
        s_command_scaled[i] = thro_des;
      }
      for (int i=0; i<numESCs; i++) {
        e_command_scaled[i] = thro_des;
      }
    
      //throttleCut(); //Directly sets motor commands to low based on state of ch5

      Servos.commandAll(s_command_scaled);
      Escs.commandAll(e_command_scaled);
      Motors.commandAll(m_command_scaled);

      // printRadioData(); //Radio pwm values (expected: 1000 to 2000)
      
      loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
   }
}

void throttleCut() {
  //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
      Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (120 is
      minimum for oneshot125 protocol, 0 is minimum for standard PWM servo library used) if channel 5 is low. This is the last function
      called before commandAll() is called so that the last thing checked is if the user is giving permission to command
      the motors to anything other than minimum value. Safety first.

      channel_5_pwm is LOW then throttle cut is OFF and throttle value can change. (ThrottleCut is DEACTIVATED)
      channel_5_pwm is HIGH then throttle cut is ON and throttle value = 120 only. (ThrottleCut is ACTIVATED), (drone is DISARMED)
  */
  if ((channel_pwm[4] < 1500) || (armedFly == false)) {
    if (armedFly == true) Serial.println(F("DISARMED"));

    armedFly = false;

    for (int i=0; i<numMotors; i++) {
      m_command_scaled[i] = 0.0;
    }
    //Uncomment if using servo PWM variables to control motor ESCs
    for (int i=0; i<numESCs; i++) {
      e_command_scaled[i] = 0.0;
    }
  }
}



//========================================================================================================================//
//                                             LOOP REGULATING FUNCTIONS                                                  //                           
//========================================================================================================================//

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
    digitalWrite(13, blinkAlternate); //Pin 13 is built in LED
    
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
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}

