//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Version: Beta 1.1

/*
 * 
 * If you are using this for an academic or scholarly project, please credit me in any presentations or publications:
 *
 * Nicholas Rehm
 * Department of Aerospace Engineering
 * University of Maryland
 * College Park 20742
 * Email: nrehm@umd.edu
 *
 */

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//CREDITS
/*
Some elements inspired by:
http://www.brokking.net/ymfc-32_main.html

Skeleton code for reading and initializing MPU6050 borrowed from:
https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/

Madgwick filter function adapted from:
https://github.com/arduino-libraries/MadgwickAHRS
*/




//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




//REQUIRED LIBRARIES
#include <Wire.h> //I2c communication
#include <PWMServo.h> //commanding any extra actuators, installed with teensyduino installer




//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




//DECLARE PINS
//NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the IMU
//Radio:
const int ch1Pin = 15; //throttle
const int ch2Pin = 16; //ail
const int ch3Pin = 17; //ele
const int ch4Pin = 20; //rudd
const int ch5Pin = 21; //gear (throttle cut)
const int ch6Pin = 22; //aux1 (free aux channel)
const int PPM_Pin = 23;
//MPU6050:
const int MPU = 0x68; //MPU6050 I2C address -- pins 19 = SCL & 18 = SDA
//Motor pin outputs:
const int m1Pin = 0;
const int m2Pin = 1;
const int m3Pin = 2;
const int m4Pin = 3;
const int m5Pin = 4;
const int m6Pin = 5;
//PWM outputs:
const int servo1Pin = 6;
const int servo2Pin = 7;
const int servo3Pin = 8;
const int servo4Pin = 9;
const int servo5Pin = 10;
const int servo6Pin = 11;
const int servo7Pin = 12;
PWMServo servo1;  //create servo object to control a servo
PWMServo servo2;
PWMServo servo3;
PWMServo servo4;
PWMServo servo5;
PWMServo servo6;
PWMServo servo7;




//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




//DECLARE GLOBAL VARIABLES
//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;
//Radio comm:
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;
int radio_command = 1;
//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
float roll_correction, pitch_correction;
float beta = 0.04; //madgwick filter parameter 
float q0 = 1.0f; //initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;
//Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;
//Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;




//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




//USER-SPECIFIED VARIABLES
//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
unsigned long channel_1_fs = 1000; //thro
unsigned long channel_2_fs = 1500; //ail
unsigned long channel_3_fs = 1500; //elev
unsigned long channel_4_fs = 1500; //rudd
unsigned long channel_5_fs = 2000; //gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = 2000; //aux1

//Controller parameters (take note of defaults before modifying!): 
float i_limit = 25.0;    //integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 30.0;    //max roll angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode (maximum ~400)
float maxPitch = 30.0;   //max pitch angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode (maximum ~400)
float maxYaw = 160.0;    //max yaw rate in deg/sec (maximum ~400)

float Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
float Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (if using controlANGLE2, MUST be 0.0. Suggested default for controlANGLE is 0.05)
float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (if using controlANGLE2, MUST be 0.0. Suggested default for controlANGLE is 0.05)

float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

float Kp_yaw = 0.3;      //Yaw P-gain
float Ki_yaw = 0.05;     //Yaw I-gain
float Kd_yaw = 0.00015;   //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)




//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




//SETUP
void setup() {
  Serial.begin(500000); //usb serial
  delay(1500);
  
  //Initialize all pins
  pinMode(13, OUTPUT); //pin 13 LED blinker on board, do not modify 
  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  pinMode(m3Pin, OUTPUT);
  pinMode(m4Pin, OUTPUT);
  pinMode(m5Pin, OUTPUT);
  pinMode(m6Pin, OUTPUT);
  servo1.attach(servo1Pin, 900, 2100); //pin, min PWM value, max PWM value
  servo2.attach(servo2Pin, 900, 2100);
  servo3.attach(servo3Pin, 900, 2100);
  servo4.attach(servo4Pin, 900, 2100);
  servo5.attach(servo5Pin, 900, 2100);
  servo6.attach(servo6Pin, 900, 2100);
  servo7.attach(servo7Pin, 900, 2100);

  //Set built in LED to turn on to signal startup & not to disturb vehicle during IMU calibration
  digitalWrite(13, HIGH);

  delay(20);

  //Initialize radio communication - SELECT ONE 
  readPWM_setup(ch1Pin, ch2Pin, ch3Pin, ch4Pin, ch5Pin, ch6Pin); //uncomment if using 6 channel pwm receiver
  //readPPM_setup(PPM_Pin) //uncomment if using ppm receiver

  //Set radio channels to default (safe) values
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;

  //Initialize IMU communication
  IMUinit();

  delay(20);

  //Get IMU error to calibrate attitude, assuming vehicle is level
  calculate_IMU_error();
  calibrateAttitude(); //helps to warm up IMU and Madgwick filter

  delay(20);

  //Arm servo channels
  servo1.write(0); //command servo angle from 0-180 degrees (1000 - 2000 PWM)
  servo2.write(0);
  servo3.write(0);
  servo4.write(0);
  servo5.write(0);
  servo6.write(0);
  servo7.write(0);
  
  delay(20);

  //Arm motors
  m1_command_PWM = 125;
  m2_command_PWM = 125;
  m3_command_PWM = 125;
  m4_command_PWM = 125;
  m5_command_PWM = 125;
  m6_command_PWM = 125;
  commandMotors();
  
  delay(200);
  
  //Indicate entering main loop with 3 quick blinks
  setupBlink(3,150,70); //numBlinks, upTime (ms), downTime (ms) -- 3 quick blinks indicates entering main loop!
}




//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




//MAIN LOOP
void loop() {
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0; 

  loopBlink(); //indicate we are in main loop with short blink every 1.5 seconds

  //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE
  //printRadioData(); //radio pwm values 
  //printDesiredState(); //desired vehicle state commanded in either degrees or degrees/sec
  //printGyroData(); //prints filtered gyro data direct from IMU
  //printAccelData(); //prints filtered accelerometer data direct from IMU
  //printRollPitchYaw(); //prints roll, pitch, and yaw angles in degrees from Madgwick filter 
  //printPIDoutput(); //prints computed stabilized PID variables from controller and desired setpoint
  //printMotorCommands(); //prints the values being written to the motors
  //printLoopRate(); //prints the time between loops in microseconds

  //Get vehicle state
  getIMUdata(); //pulls raw gyro and accel data from IMU and LP filters to remove noise
  Madgwick(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, dt); //updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)

  //Compute desired state
  getDesState(); //convert raw commands to normalized values based on saturated control limits
  
  //PID Controller - SELECT ONE
  controlANGLE(); //stabilize on angle setpoint
  //controlANGLE2(); //stabilize on angle setpoint using cascaded method 
  //controlRATE(); //stabilize on rate setpoint

  //Actuator mixing and scaling to PWM values
  controlMixer(); //mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleCommands(); //scales motor commands to 125-250 range (oneshot125 protocol) and servo commands to 0-180 (for servo library)

  //Throttle cut check
  throttleCut(); //directly sets motor commands to low based on state of ch5

  //Command actuators
  commandMotors(); //sends command pulses to each motor pin using OneShot125 protocol
  servo1.write(s1_command_PWM); 
  servo2.write(s2_command_PWM);
  servo3.write(s3_command_PWM);
  servo4.write(s4_command_PWM);
  servo5.write(s5_command_PWM);
  servo6.write(s6_command_PWM);
  servo7.write(s7_command_PWM);
    
  //Get vehicle commands for next loop iteration
  getCommands(); //pulls current available radio commands
  failSafe(); //prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

  //Regulate loop rate
  loopRate(2000); //do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}




//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




//FUNCTIONS

void IMUinit() {
  //DESCRIPTION: Initialize IMU I2C connection
  /*
   * Don't worry about how this works
   */
  Wire.begin(); //Initialize communication
  delay(20);
  Wire.setClock(1000000); 
  delay(20);
  Wire.beginTransmission(MPU); //Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B); //Talk to the register 6B
  Wire.write(0x00); //Make reset - place a 0 into the 6B register
  Wire.endTransmission(true); //End the transmission
}

void getIMUdata() {
  //DESCRIPTION: Request full dataset from IMU and LP filter gyro and accelerometer data
  /*
   * Reads accelerometer and gyro data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ. These values are scaled 
   * according to the IMU datasheet to put them into correct units of g's and degree/sec. A simple first-order 
   * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut 
   * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
   * the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally, 
   * the constant errors found in calculate_IMU_error() on startup are subtracted from the readings.
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
  
  //Get accel data
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); //Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); //Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AcX = (Wire.read() << 8 | Wire.read()); //X-axis value
  AcY = (Wire.read() << 8 | Wire.read()); //Y-axis value
  AcZ = (Wire.read() << 8 | Wire.read()); //Z-axis value
  AccX = AcX / 16384.0;
  AccY = AcY / 16384.0;
  AccZ = AcZ / 16384.0;
  //LP filter accelerometer data
  float B_accel = 0.14; //0.01
  AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
  AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
  AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;
  //Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;

  //Get gyro data
  Wire.beginTransmission(MPU);
  Wire.write(0x43); //Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); //Read 4 registers total, each axis value is stored in 2 registers
  //For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyX = (Wire.read() << 8 | Wire.read());
  GyY = (Wire.read() << 8 | Wire.read());
  GyZ = (Wire.read() << 8 | Wire.read());
  GyroX = GyX / 131.0;
  GyroY = GyY / 131.0;
  GyroZ = GyZ / 131.0;
  //LP filter gyro data
  float B_gyro = 0.1; //0.13 sets cutoff just past 80Hz for about 3000Hz loop rate
  GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
  GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
  GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;
  //Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
}

void calculate_IMU_error() {
  //DESCRIPTION: Computes IMU error on startup. Note: vehicle should be powered up on flat surface
  /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
   * measurement. 
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
  
  //Read accelerometer values 12000 times
  int c = 0;
  while (c < 12000) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AcX = (Wire.read() << 8 | Wire.read());
    AcY = (Wire.read() << 8 | Wire.read());
    AcZ = (Wire.read() << 8 | Wire.read());
    AccX = AcX / 16384.0;
    AccY = AcY / 16384.0;
    AccZ = AcZ / 16384.0;
    // Sum all readings
    AccErrorX = AccErrorX + AccX;
    AccErrorY = AccErrorY + AccY;
    AccErrorZ = AccErrorZ + AccZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX = AccErrorX / 12000.0;
  AccErrorY = AccErrorY / 12000.0;
  AccErrorZ = AccErrorZ / 12000.0 - 1.0;
  c = 0;
  //Read gyro values 12000 times
  while (c < 12000) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();
    GyroX = GyX / 131.0;
    GyroY = GyY / 131.0;
    GyroZ = GyZ / 131.0;
    // Sum all readings
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  GyroErrorX = GyroErrorX / 12000.0;
  GyroErrorY = GyroErrorY / 12000.0;
  GyroErrorZ = GyroErrorZ / 12000.0;
}

void calibrateAttitude() {
  //DESCRIPTION: Extra function to calibrate IMU attitude estimate on startup, can be used to warm up everything before entering main loop
  //Assuming vehicle is powered up on level surface!
  /*
   * This function is used on startup to warm up the attitude estimation. Originally used to eliminate additional error in the signal
   * but no longer used for that purpose as it is not needed on the Teensy. This function is what causes startup to take a few seconds
   * to boot. The roll_correction and pitch_correction values can be applied to the roll and pitch attitude estimates using 
   * correctRollPitch() in the main loop after the madgwick filter function. Again, we don't use this for that purpose but just to warm
   * up the IMU and attitude estimation algorithm.
   */
  //Warm up IMU and madgwick filter
  for (int i = 0; i <= 10000; i++) {
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0; 
    getIMUdata();
    Madgwick(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, dt);
    loopRate(2000); //do not exceed 2000Hz
  }
  //Grab mean roll and pitch values after everything is warmed up
  for (int j = 1; j <= 2000; j++) {
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0; 
    getIMUdata();
    Madgwick(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, dt);
    roll_correction = roll_IMU + roll_correction;
    pitch_correction = pitch_IMU + pitch_correction;
    loopRate(2000); //do not exceed 2000Hz
  }
  //These are applied to roll and pitch after Madgwick filter in main loop if desired using correctRollPitch()
  roll_correction = roll_correction/2000.0;
  pitch_correction = pitch_correction/2000.0;
}

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion
  /*
   * This function fuses the accelerometer and gyro readings AccX, AccY, AccZ, GyroX, GyroY, GyroZ for attitude estimation.
   * Don't worry about the math. There is a tunable parameter called beta in the variable declaration section which basically
   * adjusts the weight of accelerometer and gyro data in the state estimate. Higher beta leads to noisier estimate, lower 
   * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the roll_IMU,
   * pitch_IMU, and yaw_IMU variables which are in degrees. 
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //compute angles
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = asin(-2.0f * (q1*q3 - q0*q2))*57.29577951; //degrees
  yaw_IMU = atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
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
  thro_des = (channel_1_pwm - 1000.0)/1000.0; //between 0 and 1
  roll_des = (channel_2_pwm - 1500.0)/500.0; //between -1 and 1
  pitch_des = (channel_3_pwm - 1500.0)/500.0; //between -1 and 1
  yaw_des = (channel_4_pwm - 1500.0)/500.0; //between -1 and 1
  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0); //between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //between -maxYaw and +maxYaw

  roll_passthru = roll_des/(2*maxRoll);
  pitch_passthru = pitch_des/(2*maxPitch);
  yaw_passthru = yaw_des/(2*maxYaw);
}

void controlANGLE() {
  //DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
   * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */
  
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX;
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void controlANGLE2() {
  //DESCRIPTION: Computes control commands based on state error (angle) in cascaded scheme
  /*
   * Gives better performance than controlANGLE() but requires much more tuning. Not reccommended for first-time setup.
   */
  //Outer loop - PID on angle
  float roll_des_ol, pitch_des_ol;
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll_ol = integral_roll_prev_ol + error_roll*dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_roll_ol = 0;
  }
  integral_roll_ol = constrain(integral_roll_ol, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_roll = (roll_IMU - roll_IMU_prev)/dt; 
  roll_des_ol = Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll_ol - Kd_roll_angle*derivative_roll;

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch_ol = integral_pitch_prev_ol + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_pitch_ol = 0;
  }
  integral_pitch_ol = constrain(integral_pitch_ol, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_pitch = (pitch_IMU - pitch_IMU_prev)/dt;
  pitch_des_ol = Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch_ol - Kd_pitch_angle*derivative_pitch;

  //Apply loop gain, constrain, and filter
  float Kl = 30.0;
  roll_des_ol = Kl*roll_des_ol;
  pitch_des_ol = Kl*pitch_des_ol;
  roll_des_ol = constrain(roll_des_ol, -240.0, 240.0);
  pitch_des_ol = constrain(pitch_des_ol, -240.0, 240.0);
  float B_loop = 0.9; //LP filter parameter for outer to inner loop, acts as damping term
  roll_des_ol = (1.0 - B_loop)*roll_des_prev + B_loop*roll_des_ol;
  pitch_des_ol = (1.0 - B_loop)*pitch_des_prev + B_loop*pitch_des_ol;

  //Inner loop - PID on rate
  //Roll
  error_roll = roll_des_ol - GyroX;
  integral_roll_il = integral_roll_prev_il + error_roll*dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_roll_il = 0;
  }
  integral_roll_il = constrain(integral_roll_il, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev)/dt; 
  roll_PID = .01*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll_il + Kd_roll_rate*derivative_roll); //scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des_ol - GyroY;
  integral_pitch_il = integral_pitch_prev_il + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_pitch_il = 0;
  }
  integral_pitch_il = constrain(integral_pitch_il, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  pitch_PID = .01*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch_il + Kd_pitch_rate*derivative_pitch); //scaled by .01 to bring within -1 to 1 range
  
  //Yaw
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //scaled by .01 to bring within -1 to 1 range
  
  //Update roll variables
  integral_roll_prev_ol = integral_roll_ol;
  integral_roll_prev_il = integral_roll_il;
  error_roll_prev = error_roll;
  roll_IMU_prev = roll_IMU;
  roll_des_prev = roll_des_ol;
  //Update pitch variables
  integral_pitch_prev_ol = integral_pitch_ol;
  integral_pitch_prev_il = integral_pitch_il;
  error_pitch_prev = error_pitch;
  pitch_IMU_prev = pitch_IMU;
  pitch_des_prev = pitch_des_ol;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;

}

void controlRATE() {
  //DESCRIPTION: Computes control commands based on state error (rate)
  /*
   * See explanation for controlANGLE(). Everything is the same here except the error is now the desired rate - raw gyro reading.
   */
  //Roll
  error_roll = roll_des - GyroX;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev)/dt; 
  roll_PID = .01*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll + Kd_roll_rate*derivative_roll); //scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  pitch_PID = .01*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch + Kd_pitch_rate*derivative_pitch); //scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  error_roll_prev = error_roll;
  integral_roll_prev = integral_roll;
  GyroX_prev = GyroX;
  //Update pitch variables
  error_pitch_prev = error_pitch;
  integral_pitch_prev = integral_pitch;
  GyroY_prev = GyroY;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void controlMixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
   * normalized (0 - 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with 
   * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables are used in scaleCommands() 
   * in preparation to be sent to the motors and servos.
   */
  //Quad mixing
  //m1 = front left, m2 = front right, m3 = back right, m4 = back left
  m1_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID;
  m2_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID;
  m3_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID;
  m4_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID;
  m5_command_scaled = 0;
  m6_command_scaled = 0;

  //0.5 is centered servo, 0 is zero throttle if connecting to ESC for conventional PWM, 1 is max throttle
  s1_command_scaled = 0;
  s2_command_scaled = 0;
  s3_command_scaled = 0;
  s4_command_scaled = 0;
  s5_command_scaled = 0;
  s6_command_scaled = 0;
  s7_command_scaled = 0;

  //Example use of the linear fader for float type variables. Linearly interpolates between minimum and maximum values for Kp_pitch_rate variable based on state of channel 6
  /*
  if (channel_6_pwm > 1500){
    Kp_pitch_rate = floatFaderLinear(Kp_pitch_rate, 0.1, 0.3, 5.5, 1, 2000); //parameter, minimum value, maximum value, fadeTime (seconds), state (0 min or 1 max), loop frequency
  }
  else if (channel_6_pwm < 1500) {
    Kp_pitch_rate = floatFaderLinear(Kp_pitch_rate, 0.1, 0.3, 2.5, 0, 2000); //parameter, minimum value, maximum value, fadeTime, state (0 min or 1 max), loop frequency
  }
  */
}

void scaleCommands() {
  //DESCRIPTION: Scale normalized actuator commands to values for ESC protocol
  /*
   * mX_command_scaled variables from the mixer function are scaled to 125-250us for OneShot125 protocol. sX_command_scaled variables from
   * the mixer function are scaled to 0-180 for the servo library using standard PWM.
   * mX_command_PWM are updated here which are used to command the motors in commandMotors(). sX_command_PWM are updated 
   * which are used to command the servos.
   */
  //Scaled to 125us - 250us for oneshot125 protocol
  m1_command_PWM = m1_command_scaled*125 + 125;
  m2_command_PWM = m2_command_scaled*125 + 125;
  m3_command_PWM = m3_command_scaled*125 + 125;
  m4_command_PWM = m4_command_scaled*125 + 125;
  m5_command_PWM = m5_command_scaled*125 + 125;
  m6_command_PWM = m6_command_scaled*125 + 125;
  //Constrain commands to motors within oneshot125 bounds
  m1_command_PWM = constrain(m1_command_PWM, 125, 250);
  m2_command_PWM = constrain(m2_command_PWM, 125, 250);
  m3_command_PWM = constrain(m3_command_PWM, 125, 250);
  m4_command_PWM = constrain(m4_command_PWM, 125, 250);
  m5_command_PWM = constrain(m5_command_PWM, 125, 250);
  m6_command_PWM = constrain(m6_command_PWM, 125, 250);

  //Scaled to 0-180 for servo library
  s1_command_PWM = s1_command_scaled*180;
  s2_command_PWM = s2_command_scaled*180;
  s3_command_PWM = s3_command_scaled*180;
  s4_command_PWM = s4_command_scaled*180;
  s5_command_PWM = s5_command_scaled*180;
  s6_command_PWM = s6_command_scaled*180;
  s7_command_PWM = s7_command_scaled*180;
  //Constrain commands to servos within servo library bounds
  s1_command_PWM = constrain(s1_command_PWM, 0, 180);
  s2_command_PWM = constrain(s2_command_PWM, 0, 180);
  s3_command_PWM = constrain(s3_command_PWM, 0, 180);
  s4_command_PWM = constrain(s4_command_PWM, 0, 180);
  s5_command_PWM = constrain(s5_command_PWM, 0, 180);
  s6_command_PWM = constrain(s6_command_PWM, 0, 180);
  s7_command_PWM = constrain(s7_command_PWM, 0, 180);

}

void getCommands() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of 
   * the loop. The radio commands are retrieved from a function in the readPWM
   * file separate from this one which is running a bunch of interrupts to continuously update the radio readings. 
   * The raw radio commands are being filtered with a first order low-pass filter to eliminate any really high frequency noise. 
   */

  radio_command = 1;
  channel_1_pwm = getRadioPWM(1);
  channel_2_pwm = getRadioPWM(2);
  channel_3_pwm = getRadioPWM(3);
  channel_4_pwm = getRadioPWM(4);
  channel_5_pwm = getRadioPWM(5);
  channel_6_pwm = getRadioPWM(6);
  
  //Low-pass the critical commands and update previous values
  float b = 0.2;
  channel_1_pwm = (1.0 - b)*channel_1_pwm_prev + b*channel_1_pwm;
  channel_2_pwm = (1.0 - b)*channel_2_pwm_prev + b*channel_2_pwm;
  channel_3_pwm = (1.0 - b)*channel_3_pwm_prev + b*channel_3_pwm;
  channel_4_pwm = (1.0 - b)*channel_4_pwm_prev + b*channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

void failSafe() {
  //DESCRIPTION: If radio gives garbage values, set all commands to default values
  /*
   * Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of 
   * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
   * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands 
   * channel_x_pwm are set to default failsafe values specified in the setup.
   */
  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;

  //Triggers for failure criteria
  if (channel_1_pwm > maxVal || channel_1_pwm < minVal) check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal) check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal) check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal) check4 = 1;
  if (channel_5_pwm > maxVal || channel_5_pwm < minVal) check5 = 1;
  if (channel_6_pwm > maxVal || channel_6_pwm < minVal) check6 = 1;

  //If any failures, set to default failsafe values
  if ((check1 + check2 + check3 + check4 + check5 + check6) > 0) {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;
  }
}

void commandMotors() {
  //DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
  /*
   * My crude implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin). The pulselengths being
   * sent are mX_command_PWM, computed in scaleCommands().
   */
  int wentLow = 0;
  int pulseStart, timer;
  int flagM1 = 0;
  int flagM2 = 0;
  int flagM3 = 0;
  int flagM4 = 0;
  int flagM5 = 0;
  int flagM6 = 0;
  
  //Write all motor pins high
  digitalWrite(m1Pin, HIGH);
  digitalWrite(m2Pin, HIGH);
  digitalWrite(m3Pin, HIGH);
  digitalWrite(m4Pin, HIGH);
  digitalWrite(m5Pin, HIGH);
  digitalWrite(m6Pin, HIGH);
  pulseStart = micros();

  //Write each motor pin low as correct pulse length is reached
  while (wentLow < 6 ) { //keep going until final (6th) pulse is finished, then done
    timer = micros();
    if ((m1_command_PWM <= timer - pulseStart) && (flagM1==0)) {
      digitalWrite(m1Pin, LOW);
      wentLow = wentLow + 1;
      flagM1 = 1;
    }
    if ((m2_command_PWM <= timer - pulseStart) && (flagM2==0)) {
      digitalWrite(m2Pin, LOW);
      wentLow = wentLow + 1;
      flagM2 = 1;
    }
    if ((m3_command_PWM <= timer - pulseStart) && (flagM3==0)) {
      digitalWrite(m3Pin, LOW);
      wentLow = wentLow + 1;
      flagM3 = 1;
    }
    if ((m4_command_PWM <= timer - pulseStart) && (flagM4==0)) {
      digitalWrite(m4Pin, LOW);
      wentLow = wentLow + 1;
      flagM4 = 1;
    } 
    if ((m5_command_PWM <= timer - pulseStart) && (flagM5==0)) {
      digitalWrite(m5Pin, LOW);
      wentLow = wentLow + 1;
      flagM5 = 1;
    } 
    if ((m6_command_PWM <= timer - pulseStart) && (flagM6==0)) {
      digitalWrite(m6Pin, LOW);
      wentLow = wentLow + 1;
      flagM6 = 1;
    } 
  }
}

float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq){
  //DESCRIPTION: Linearly fades a float type variable between min and max bounds based on desired high or low state and time
  /*  Takes in a float variable, desired minimum and maximum bounds, fade time, high or low desired state, and the loop frequency 
   *  and linearly interpolates that param variable between the maximum and minimum bounds. This function can be called in controlMixer()
   *  and high/low states can be determined by monitoring the state of an auxillarly radio channel. For example, if channel_6_pwm is being 
   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical 
   *  statements in order to fade controller gains, for example between the two dynamic configurations. The 'state' (1 or 0) can be used
   *  to designate the two final options for that control gain based on the dynamic configuration assignment to the auxillary radio channel.
   */
  float diffParam = (param_max - param_min)/(fadeTime*loopFreq); //difference to add or subtract from param for each loop iteration for desired fadeTime

  if (state == 1) { //maximum param bound desired, increase param by diffParam for each loop iteration
    param = param + diffParam;
  }
  else if (state == 0) { //minimum param bound desired, decrease param by diffParam for each loop iteration
    param = param - diffParam;
  }

  param = constrain(param, param_min, param_max); //constrain param within max bounds
  
  return param;
}


void throttleCut() {
  //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
   * Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (120 is
   * minimum for oneshot125 protocol, 1000 is minimum for standard PWM) if channel 5 is high. This is the last function 
   * called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
   * the motors to anything other than minimum value. Safety first. 
   */
  if (channel_5_pwm > 1500) {
    m1_command_PWM = 120;
    m2_command_PWM = 120;
    m3_command_PWM = 120;
    m4_command_PWM = 120;
    m5_command_PWM = 120;
    m6_command_PWM = 120;
  }
}

void loopRate(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters scattered around this code.
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
    digitalWrite(13, blinkAlternate); //pin 13 is built in LED
    
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

void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F(" CH1: "));
    Serial.print(channel_1_pwm);
    Serial.print(F(" CH2: "));
    Serial.print(channel_2_pwm);
    Serial.print(F(" CH3: "));
    Serial.print(channel_3_pwm);
    Serial.print(F(" CH4: "));
    Serial.print(channel_4_pwm);
    Serial.print(F(" CH5: "));
    Serial.print(channel_5_pwm);
    Serial.print(F(" CH6: "));
    Serial.println(channel_6_pwm);
  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("thro_des: "));
    Serial.print(thro_des);
    Serial.print(F(" roll_des: "));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des: "));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des: "));
    Serial.println(yaw_des);
  }
}

void printGyroData() {
    if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("GyroX: "));
    Serial.print(GyroX);
    Serial.print(F(" GyroY: "));
    Serial.print(GyroY);
    Serial.print(F(" GyroZ: "));
    Serial.println(GyroZ);
  }
}

void printAccelData() {
    if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("AccX: "));
    Serial.print(AccX);
    Serial.print(F(" AccY: "));
    Serial.print(AccY);
    Serial.print(F(" AccZ: "));
    Serial.println(AccZ);
  }
}

void printRollPitchYaw() {
    if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll: "));
    Serial.print(roll_IMU);
    Serial.print(F(" pitch: "));
    Serial.print(pitch_IMU);
    Serial.print(F(" yaw: "));
    Serial.println(yaw_IMU);
  }
}

void printPIDoutput() {
    if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll_PID: "));
    Serial.print(roll_PID);
    Serial.print(F(" pitch_PID: "));
    Serial.print(pitch_PID);
    Serial.print(F(" yaw_PID: "));
    Serial.println(yaw_PID);
  }
}

void printMotorCommands() {
    if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("m1_command: "));
    Serial.print(m1_command_PWM);
    Serial.print(F(" m2_command: "));
    Serial.print(m2_command_PWM);
    Serial.print(F(" m3_command: "));
    Serial.print(m3_command_PWM);
    Serial.print(F(" m4_command: "));
    Serial.print(m4_command_PWM);
    Serial.print(F(" m5_command: "));
    Serial.print(m5_command_PWM);
    Serial.print(F(" m6_command: "));
    Serial.println(m6_command_PWM);
  }
}

void printLoopRate() {
    if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("dt = "));
    Serial.println(dt*1000000.0);
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//HELPER FUNCTIONS

float invSqrt(float x) {
  //Fast inverse sqrt
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
}
