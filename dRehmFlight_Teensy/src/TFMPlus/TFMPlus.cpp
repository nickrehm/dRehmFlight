/* File Name: TFMPlus.cpp
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
        This makes recognition of the command reply more robust.
        Zeroed out 'data frame' snd  'command reply' buffer arrays
        completely before reading from device.  Added but did not
        implement some I2C command codes.
 * v.1.3.4 - 07JUN19 - Added 'TFMP_' to all error status defines.
        The ubiquitous 'Arduino.h' also contains a 'SERIAL' define.
 * v.1.3.5 - 25OCT19 - Added missing underscore to parameter
        in header file TFMPlus.h:
        Line 138   #define   BAUD_14400   0x003840
 * v.1.3.6 - 27APR20 - a little cleanup in 'getData()'
 * v.1.4.0 - 15JUN20 - Changed all data variables from unsigned
             to signed integers.  Defined abnormal data codes
             as per TFMini-S Producut Manual
           -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
            Dist | Strength    |  Comment
             -1    Other value   Strength ≤ 100
             -2    -1            Signal strength saturation
             -4    Other value   Ambient light saturation
           -  -  -  -  -  -  -  -  -  -  -  -  -  -  -   
 * v.1.4.1 - 22JUL20 - Fixed bug in sendCommand() checksum calculation
           - Changed two printf()s to Serial.print()s
	   - Fixed printReply() to show data from 'reply' rather than 'frame'
 * v.1.4.2 - 19MAY21 - Changed command paramter 'FRAME_5' to correct value.
             It was set to 0x0003.  Now it's set to 0x0005
 * v.1.5.0 - 06SEP21 - Corrected (reversed) Enable/Disable commands in 'TFMPlus.h'
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

#include "TFMPlus.h"
//#include <Wire.h>          //  Future I2C Implementation

// Constructor
TFMPlus::TFMPlus(){}
TFMPlus::~TFMPlus(){}

// Return TRUE/FALSE whether receiving serial data from
// device, and set system status to provide more information.
bool TFMPlus::begin(Stream *streamPtr)
{
    pStream = streamPtr;          // Save reference to stream/serial object.
    delay( 10);                   // Delay for device data in serial buffer.
    if( (*pStream).available())   // If data present...
    {
        status = TFMP_READY;      // set status to READY
        return true;              // and return TRUE.
    }
    else                          // Otherwise...
    {
         status = TFMP_SERIAL;    // set status to SERIAL error
         return false;            // and return false.
    }
}

bool TFMPlus::getData( int16_t &dist, int16_t &flux, int16_t &temp)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 1 - Get data from the device.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Set timer to one second timeout if HEADER never appears
    // or serial data never becomes available.
    uint32_t serialTimeout = millis() + 1000;

    if( !(*pStream).available())
    {
        return false;
    }
    
    // Flush all but last frame of data from the serial buffer.
    while( (*pStream).available() > TFMP_FRAME_SIZE) (*pStream).read();

	  // Zero out the entire frame data buffer.
    memset( frame, 0, sizeof( frame));

    // Read one byte from the serial buffer into the last byte of
    // the frame buffer, then left shift the whole array one byte.
    // Repeat until the two HEADER bytes show up as the first
    // two bytes in the array.
    while( ( frame[ 0] != 0x59) || ( frame[ 1] != 0x59))
    {
        if( (*pStream).available())
        {
            // Read one byte into the framebuffer's
            // last plus one position.
            frame[ TFMP_FRAME_SIZE] = (*pStream).read();
            // Shift the last nine bytes one byte left.
            memcpy( frame, frame + 1, TFMP_FRAME_SIZE);
        }
        // If HEADER or serial data are not available
        // after more than one second...
        if( millis() >  serialTimeout)
        {
            status = TFMP_HEADER;   // then set error...
            return false;           // and return "false".
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 2 - Perform a checksum test.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Clear the 'chkSum' variable declared in 'TFMPlus.h'
    chkSum = 0;
    // Add together all bytes but the last.
    for( uint8_t i = 0; i < ( TFMP_FRAME_SIZE - 1); i++) chkSum += frame[ i];
    //  If the low order byte does not equal the last byte...
    if( ( uint8_t)chkSum != frame[ TFMP_FRAME_SIZE - 1])
    {
      status = TFMP_CHECKSUM;  // then set error...
      return false;            // and return "false."
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 3 - Interpret the frame data
    //          and if okay, then go home
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    dist = frame[ 2] + ( frame[ 3] << 8);
    flux = frame[ 4] + ( frame[ 5] << 8);
    temp = frame[ 6] + ( frame[ 7] << 8);
    // Convert temp code to degrees Celsius.
    temp = ( temp >> 3) - 256;
    // Convert Celsius to degrees Farenheit
    // temp = uint8_t( temp * 9 / 5) + 32;

    // - - Evaluate Abnormal Data Values - -
    // Values are from the TFMini-S Product Manual
    // Signal strength <= 100
    if( dist == -1) status = TFMP_WEAK;
    // Signal Strength saturation
    else if( flux == -1) status = TFMP_STRONG;
    // Ambient Light saturation
    else if( dist == -4) status = TFMP_FLOOD;
    // Data is apparently okay
    else status = TFMP_READY;
    
    if( status != TFMP_READY) return false;
    else return true;
}

// Pass back only the distance data
bool TFMPlus::getData( int16_t &dist)
{
  static int16_t flux, temp;
  return getData( dist, flux, temp);
}

// = = = = =  SEND A COMMAND TO THE DEVICE  = = = = = = = = = =0
//
// Create a proper command byte array, send the command,
// get a repsonse, and return the status
bool TFMPlus::sendCommand( uint32_t cmnd, uint32_t param)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 1 - Build the command data to send to the device
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    static uint8_t cmndLen;             // Length of command
    static uint8_t replyLen;            // Length of command reply data
    static uint8_t cmndData[ 8];        // 8 byte send command array

    memset( cmndData, 0, 8);            // Clear the send command array.
    memcpy( &cmndData[ 0], &cmnd, 4);   // Copy 4 bytes of data: reply length,
                                        // command length, command number and
                                        // a one byte parameter, all encoded as
                                        // a 32 bit unsigned integer.

    replyLen = cmndData[ 0];            // Save the first byte as reply length.
    cmndLen = cmndData[ 1];             // Save the second byte as command length.
    cmndData[ 0] = 0x5A;                // Set the first byte to HEADER code.

    if( cmnd == SET_FRAME_RATE)           // If the command is Set FrameRate...
    {
      memcpy( &cmndData[ 3], &param, 2);  // add the 2 byte FrameRate parameter.
    }
    else if( cmnd == SET_BAUD_RATE)       // If the command is Set BaudRate...
    {
      memcpy( &cmndData[ 3], &param, 4);  // add the 3 byte BaudRate parameter.
    }

    // Create a checksum byte for the command data array.
    chkSum = 0;
    // Add together all bytes but the last...
    for( uint8_t i = 0; i < ( cmndLen - 1); i++) chkSum += cmndData[ i];
    // and save it as the last byte of command data.
    cmndData[ cmndLen - 1] = (uint8_t)chkSum;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 2 - Send the command data array to the device
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    while( (*pStream).available()) (*pStream).read();  // flush input buffer
    (*pStream).flush();                                // flush output buffer
    for( uint8_t i = 0; i < cmndLen; i++) (*pStream).write( cmndData[ i]);


    // + + + + + + + + + + + + + + + + + + + + + + + + +
    // If the command does not expect a reply, then we're
    // finished here. Call the getData() function instead.
    if( replyLen == 0) return true;
    // + + + + + + + + + + + + + + + + + + + + + + + + +


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 3 - Get command reply data back from the device.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Set a one second timer to timeout if HEADER never appears
    // or serial data never becomes available
    uint32_t serialTimeout = millis() + 1000;
	  // Clear out the entire command reply data buffer
    memset( reply, 0, sizeof( reply));
    // Read one byte from the serial buffer into the end of
    // the reply buffer and then left shift the whole array.
    // Repeat until the HEADER byte and reply length byte
    // show up as the first two bytes in the array.
    while( ( reply[ 0] != 0x5A) || ( reply[ 1] != replyLen))
    {
        if( (*pStream).available())
        {
            // Read one byte into the reply buffer's
            // last-plus-one position.
            reply[ replyLen] = (*pStream).read();
            // Shift the last nine bytes one byte left.
            memcpy( reply, reply+1, TFMP_REPLY_SIZE);
        }
        // If HEADER pattern or Serial data are not available
        // after more than one second...
        if( millis() >  serialTimeout)
        {
            status = TFMP_TIMEOUT;  // then set error...
            return false;           // and return "false".
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 4 - Perform a checksum test.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Clear the 'chkSum' variable declared in 'TFMPlus.h'
    chkSum = 0;
    // Add together all bytes but the last...
    for( uint8_t i = 0; i < ( replyLen - 1); i++) chkSum += reply[ i];
    // If the low order byte of the Sum does not equal the last byte...
    if( reply[ replyLen - 1] != (uint8_t)chkSum)
    {
      status = TFMP_CHECKSUM;  // then set error...
      return false;            // and return "false."
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 5 - Interpret different command responses.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if( cmnd == GET_FIRMWARE_VERSION)
    {
        version[ 0] = reply[5];  // set firmware version.
        version[ 1] = reply[4];
        version[ 2] = reply[3];
    }
    else
    {
        if( cmnd == SOFT_RESET ||
            cmnd == HARD_RESET ||
            cmnd == SAVE_SETTINGS )
        {
            if( reply[ 3] == 1)      // If PASS/FAIL byte not zero ...
            {
                status = TFMP_FAIL;  // set status 'FAIL'...
                return false;        // and return 'false'.
            }
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 6 - Set READY status and go home
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    status = TFMP_READY;
    return true;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - -    The following is for testing purposes    - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// Called by either 'printFrame()' or 'printReply()'
// Print status condition either 'READY' or error type
void TFMPlus::printStatus()
{
    Serial.print("Status: ");
    if( status == TFMP_READY)          Serial.print( "READY");
    else if( status == TFMP_SERIAL)    Serial.print( "SERIAL");
    else if( status == TFMP_HEADER)    Serial.print( "HEADER");
    else if( status == TFMP_CHECKSUM)  Serial.print( "CHECKSUM");
    else if( status == TFMP_TIMEOUT)   Serial.print( "TIMEOUT");
    else if( status == TFMP_PASS)      Serial.print( "PASS");
    else if( status == TFMP_FAIL)      Serial.print( "FAIL");
    else if( status == TFMP_I2CREAD)   Serial.print( "I2C-READ");
    else if( status == TFMP_I2CWRITE)  Serial.print( "I2C-WRITE");
    else if( status == TFMP_I2CLENGTH) Serial.print( "I2C-LENGTH");
    else if( status == TFMP_WEAK)      Serial.print( "Signal weak");
    else if( status == TFMP_STRONG)    Serial.print( "Signal saturation");
    else if( status == TFMP_FLOOD)     Serial.print( "Ambient light saturation");
    else Serial.print( "OTHER");
    Serial.println();
}

// Print error type and HEX values
// of each byte in the data frame
void TFMPlus::printFrame()
{
    printStatus();
    // Print the Hex value of each byte of data
    Serial.print("Data:");
    for( uint8_t i = 0; i < TFMP_FRAME_SIZE; i++)
    {
      Serial.print(" ");
      Serial.print( frame[ i] < 16 ? "0" : "");
      Serial.print( frame[ i], HEX);
    }
    Serial.println();
}

// Print error type and HEX values of
// each byte in the command response frame.
void TFMPlus::printReply()
{
    printStatus();
    // Print the Hex value of each byte
    for( uint8_t i = 0; i < TFMP_REPLY_SIZE; i++)
    {
      Serial.print(" ");
      Serial.print( reply[ i] < 16 ? "0" : "");
      Serial.print( reply[ i], HEX);
    }
    Serial.println();
}
