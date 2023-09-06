/* File Name: TFMPlus.h
 * Version: 1.5.0
 * Described: Arduino Library for the Benewake TFMini-Plus Lidar sensor
 *            The TFMini-Plus is a unique product, and the various
 *            TFMini Libraries are not compatible with the Plus.
 * Developer: Bud Ryerson
 * Inception: v0.2.0 - 31 JAN 2019 
 * v1.0.0 - 25FEB19 - Initial release
 * v1.0.1 - 09MAR19 - 'build()' function always returned TRUE.
      Corrected to return FALSE if serial data is not available.
      And other minor corrections to textual descriptions.
 * v1.1.0 - 13MAR19 - To simplify, all interface functions now
      return boolean.  Status code is still set and can be read.
      'testSum()' is deleted and 'makeSum()' is used instead.
      Example code is updated.
 * v1.1.1 - 14MAR19 - Two commands: RESTORE_FACTORY_SETTINGS
      and SAVE_SETTINGS were not defined correctly.
 * v1.2.1 - 02APR19 - Rewrote 'getData()' function to make it faster
      and more robust when serial read skips a byte or fails entirely.
 * v1.3.1 - 08APR19 - Redefined commands to include response length
   **********************     IMPORTANT    ************************
   ****  Changed name of 'buildCommand()' to 'sendCommand()'.  ****
   ****************************************************************
 * v.1.3.2 - Added a line to getData() to flush the serial buffer
        of all but last frame of data before reading.  This does not
        effect usage, but will ensure that the latest data is read.
 * v.1.3.3 - 20MAY19 - Changed 'sendCommand()' to add a second byte,
        to the HEADER recognition routine, the reply length byte.
        Zeroed out 'data frame' snd  'command reply' buffer arrays
        completely before reading from device.  Added but did not
        implement some I2C command codes.
 * v.1.3.4 - 07JUN19 - Added 'TFMP_' to all error status defines.
        The ubiquitous 'Arduino.h' also contains a 'SERIAL' define.
 * v.1.3.5 - 25OCT19 - Added missing underscore to paramter:
        #define    BAUD_14400         0x003840
 * v.1.3.6 - 27APR20 - a little cleanup in 'getData()'
 * v.1.4.0 - 15JUN20
      1. Changed all data variables from unsigned to signed integers.
      2. Defined abnormal data codes as per TFMini-S Producut Manual
      -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
      Distance | Strength    |  Comment
          -1     Other value   Strength ≤ 100
          -2     -1            Signal strength saturation
          -4     Other value   Ambient light saturation
      -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
 * v.1.4.1 - 22JUL20 - Fixed bugs in TFMPlus.cpp
 * v.1.5.0 - 06SEP21 - Corrected (reversed) Enable/Disable commands
             Changed three command names
               OBTAIN_FIRMWARE_VERSION is now GET_FIRMWARE_VERSION
               RESTORE_FACTORY_SETTINGS is now HARD_RESET
               SYSTEM_RESET is now SOFT_RESET
 *
 * Default settings for the TFMini-Plus are a 115200 serial baud rate
 * and a 100Hz measurement frame rate. The device will begin returning
 * measurement data immediately on power up.
 *
 * 'begin()' function passes a serial stream to library and
 *  returns TRUE/FALSE whether serial data is available.
 *  Function also sets a public one byte status code.
 *  Status codes are defined within the library.
 *
 * 'getData( dist, flux, temp)' passes back measurement data
 *  • dist = distance in centimeters,
 *  • flux = signal strength in arbitrary units, and
 *  • temp = an encoded number in degrees centigrade
 *  Function returns TRUE/FALSE whether completed without error.
 *  Error, if any, is saved as a one byte 'status' code.
 *
 * 'sendCommand( cmnd, param)' sends a 32bit command code (cmnd)
 *  and a 32bit parameter value (param). Returns TRUE/FALSE and
 *  sets a one byte status code.
 *  Commands are selected from the library's list of defined commands.
 *  Parameter values can be entered directly (115200, 250, etc) but
 *  for safety, they should be chosen from the Library's defined lists.
 *  An incorrect value can render the device uncommunicative.
 *
 */

#ifndef TFMPLUS_H       // Guard to compile only once
#define TFMPLUS_H

#include <Arduino.h>    // Always include this. It's important.

// Buffer sizes
#define TFMP_FRAME_SIZE         9   // Size of data frame = 9 bytes
#define TFMP_REPLY_SIZE         8   // Longest command reply = 8 bytes
#define TFMP_COMMAND_MAX        8   // Longest command = 8 bytes

// Timeout Limits for various functions
#define TFMP_MAX_READS           20   // readData() sets SERIAL error
#define MAX_BYTES_BEFORE_HEADER  20   // getData() sets HEADER error
#define MAX_ATTEMPTS_TO_MEASURE  20

#define TFMP_DEFAULT_ADDRESS   0x10   // default I2C slave address
                                      // as hexidecimal integer
// System Error Status Condition
#define TFMP_READY           0  // no error
#define TFMP_SERIAL          1  // serial timeout
#define TFMP_HEADER          2  // no header found
#define TFMP_CHECKSUM        3  // checksum doesn't match
#define TFMP_TIMEOUT         4  // I2C timeout
#define TFMP_PASS            5  // reply from some system commands
#define TFMP_FAIL            6  //           "
#define TFMP_I2CREAD         7
#define TFMP_I2CWRITE        8
#define TFMP_I2CLENGTH       9
#define TFMP_WEAK           10  // Signal Strength ≤ 100             
#define TFMP_STRONG         11  // Signal Strength saturation
#define TFMP_FLOOD          12  // Ambient Light saturation
#define TFMP_MEASURE        13


/* - - - - - - - - -  TFMini Plus  - - - - - - - - -
  Data Frame format:
  Byte0  Byte1  Byte2   Byte3   Byte4   Byte5   Byte6   Byte7   Byte8
  0x59   0x59   Dist_L  Dist_H  Flux_L  Flux_H  Temp_L  Temp_H  CheckSum_
  Data Frame Header character: Hex 0x59, Decimal 89, or "Y"

  Command format:
  Byte0  Byte1   Byte2   Byte3 to Len-2  Byte Len-1
  0x5A   Length  Cmd ID  Payload if any   Checksum
 - - - - - - - - - - - - - - - - - - - - - - - - - */

// The library 'sendCommand( cmnd, param)' function
// defines a command (cmnd) in the the following format:
// 0x     00       00       00       00
//     one byte  command  command   reply
//     payload   number   length    length
#define    GET_FIRMWARE_VERSION       0x00010407   // returns 3 byte firmware version
#define    TRIGGER_DETECTION          0x00040400   // must have set frame rate to zero
                                                   // returns a 9 byte data frame
#define    SOFT_RESET                 0x00020405   // returns a 1 byte pass/fail (0/1)
#define    HARD_RESET                 0x00100405   //           "
#define    SAVE_SETTINGS              0x00110405   // This must follow every command
                                                   // that modifies volatile parameters.
                                                   // Returns a 1 byte pass/fail (0/1)
                                                   
#define    SET_FRAME_RATE             0x00030606   // These commands each return
#define    SET_BAUD_RATE              0x00060808   // an echo of the command
#define    STANDARD_FORMAT_CM         0x01050505   //           "
#define    PIXHAWK_FORMAT             0x02050505   //           "
#define    STANDARD_FORMAT_MM         0x06050505   //           "
#define    ENABLE_OUTPUT              0x01070505   //           "
#define    DISABLE_OUTPUT             0x00070505   //           "
#define    SET_I2C_ADDRESS            0x100B0505   //           "

#define    SET_SERIAL_MODE            0x000A0500   // default is Serial (UART)
#define    SET_I2C_MODE               0x010A0500   // set device as I2C slave

#define    I2C_FORMAT_CM              0x01000500   // returns a 9 byte data frame
#define    I2C_FORMAT_MM              0x06000500   //           "

// *  *  *  *  *  *  *  Description of I/O Mode  *  *  *  *  *  *  * 
// Normally, device Pin 3 is either Serial transmit (TX) or I2C clock (SCL).
// When 'I/O Mode' is set other than 'Standard,' Pin 3 becomes a simple HI/LO
// (near/far) binary output.  Thereafter, only Pin 2, the Serial RX line, is
// functional, and only Serial data sent to the device is possible.
//#define    SET_IO_MODE_STANDARD     0x003B0900   // 'Standard' is default mode
//#define    SET_IO_MODE_HILO         0x013B0900   // I/O, near high and far low
//#define    SET_IO_MODE_LOHI         0x023B0900   // I/O, near low and far high
// *  *  *  This library does not support the I/O Mode interface  *  *  *

// Command Parameters
#define    BAUD_9600          0x002580   // UART serial baud rate
#define    BAUD_14400         0x003840   // expressed in hexidecimal
#define    BAUD_19200         0x004B00
#define    BAUD_56000         0x00DAC0
#define    BAUD_115200        0x01C200
#define    BAUD_460800        0x070800
#define    BAUD_921600        0x0E1000

#define    FRAME_0            0x0000    // internal measurement rate
#define    FRAME_1            0x0001    // expressed in hexidecimal
#define    FRAME_2            0x0002
#define    FRAME_5            0x0005    // incorrectly set to 3 in prior versions
#define    FRAME_10           0x000A
#define    FRAME_20           0x0014
#define    FRAME_25           0x0019
#define    FRAME_50           0x0032
#define    FRAME_100          0x0064
#define    FRAME_125          0x007D
#define    FRAME_200          0x00C8
#define    FRAME_250          0x00FA
#define    FRAME_500          0x01F4
#define    FRAME_1000         0x03E8

// Object Class Definitions
class TFMPlus
{
  public:
    TFMPlus();
    ~TFMPlus();

    uint8_t version[ 3];   // to save firmware version
    uint8_t status;        // to save library error status

    // Return T/F whether serial data available, set error status if not.
    bool begin( Stream *streamPtr);
    // Read device data and pass back three values
    bool getData( int16_t &dist, int16_t &flux, int16_t &temp);
    // Short version, passes back distance data only
    bool getData( int16_t &dist);
    // Build and send a command, and check response
    bool sendCommand( uint32_t cmnd, uint32_t param);
    
    //  For testing purposes: print frame or reply data and status
    //  as a string of HEX characters
    void printFrame();
    void printReply();    
    bool getResponse();

  private:
    Stream* pStream;      // pointer to the device serial stream
    // The data buffers are one byte longer than necessary
    // because we read one byte into the last position, then
    // shift the whole array left by one byte after each read.
    uint8_t frame[ TFMP_FRAME_SIZE + 1];
    uint8_t reply[ TFMP_REPLY_SIZE + 1];

    uint16_t chkSum;     // to calculate the check sum byte.

    // for testing - called by 'printFrame()' or 'printReply()'
    void printStatus();

};

#endif
