#ifndef dRhemFlight_hardware_h
#define dRhemFlight_hardware_h

#include <Arduino.h>

//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //                                                                 
//========================================================================================================================//

//Uncomment only one receiver type
//#define USE_PWM_RX
//#define USE_PPM_RX
#define USE_SBUS_RX
//#define USE_DSM_RX
static const uint8_t num_DSM_channels = 6; //If using DSM RX, change this to match the number of transmitter channels you have


// capture number of input channels

// Define input pins on the board of your choice



//Uncomment only one IMU
//#define USE_MPU6050_I2C //Default
//#define USE_MPU9250_SPI
//#define USE_LSM6DSOX_SPI

// Uncomment only one full scale gyro range (deg/sec)
// #define GYRO_250DPS //Default
// #define GYRO_500DPS
// #define GYRO_1000DPS
// #define GYRO_2000DPS

// Uncomment only one full scale accelerometer range (G's)
// #define ACCEL_2G //Default
// #define ACCEL_4G
// #define ACCEL_8G
// #define ACCEL_16G




//Capture number of output channels:  Motors (DSHOT125, etc.), Servos (50Hz), and possibly other Digital Servos



//Define output pins based on above and board type / craft type













// Other defines that depend on the above


#if defined USE_SBUS_RX
  #include "SBUS/SBUS.h"   //sBus interface
#endif

#if defined USE_DSM_RX
  #include "DSMRX/DSMRX.h"  
#endif






#endif