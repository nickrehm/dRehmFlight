/*
 * Interface to the RC IBus protocol
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either  
 * version 2.1 of the License, or (at your option) any later version.
 *   
 * Created 12 March 2019 Bart Mellink
 */
#ifndef IBusBM_h
#define IBusBM_h

#include <inttypes.h>

#if defined(ARDUINO_ARCH_MBED)
#include "mbed.h"
#include "HardwareSerial.h"
#endif

// if you have an opentx transciever you can add additional sensor types here.
// see https://github.com/cleanflight/cleanflight/blob/7cd417959b3cb605aa574fc8c0f16759943527ef/src/main/telemetry/ibus_shared.h
// below the values supported by the Turnigy FS-MT6 transceiver
#define IBUSS_INTV 0x00 // Internal voltage (in 0.01)
#define IBUSS_TEMP 0x01 // Temperature (in 0.1 degrees, where 0=-40'C)
#define IBUSS_RPM  0x02 // RPM
#define IBUSS_EXTV 0x03 // External voltage (in 0.01)
#define IBUS_PRESS 0x41 // Pressure (in Pa)
#define IBUS_SERVO 0xfd // Servo value


#if defined(ARDUINO_ARCH_MBED)
#define HardwareSerial arduino::HardwareSerial
#else
  #if !defined(ARDUINO_ARCH_MEGAAVR)
class HardwareSerial;
  #endif
#endif
class Stream;

class IBusBM {

public:
#if defined(_VARIANT_ARDUINO_STM32_)
  #if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
    #error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
  #endif
  #define IBUSBM_NOTIMER NULL // no timer interrupt used
  void begin(HardwareSerial &serial, TIM_TypeDef * timerid=TIM1, int8_t rxPin=-1, int8_t txPin=-1);
#else
  #define IBUSBM_NOTIMER -1 // no timer interrupt used
  void begin(HardwareSerial &serial, int8_t timerid=0, int8_t rxPin=-1, int8_t txPin=-1);
#endif
  uint16_t readChannel(uint8_t channelNr); // read servo channel 0..9
  uint8_t addSensor(uint8_t type, uint8_t len=2); // add sensor type and data length (2 or 4), returns address
  void setSensorMeasurement(uint8_t adr, int32_t value);

  void loop(void); // used internally for interrupt handline, but needs to be defined as public
  
  volatile uint8_t cnt_poll; // count received number of sensor poll messages
  volatile uint8_t cnt_sensor; // count times a sensor value has been sent back
  volatile uint8_t cnt_rec; // count received number of servo messages
  
private:
  enum State {GET_LENGTH, GET_DATA, GET_CHKSUML, GET_CHKSUMH, DISCARD};

  static const uint8_t PROTOCOL_LENGTH = 0x20;
  static const uint8_t PROTOCOL_OVERHEAD = 3; // packet is <len><cmd><data....><chkl><chkh>, overhead=cmd+chk bytes
  static const uint8_t PROTOCOL_TIMEGAP = 3; // Packets are received very ~7ms so use ~half that for the gap
  static const uint8_t PROTOCOL_CHANNELS = 14;
  static const uint8_t PROTOCOL_COMMAND40 = 0x40;        // Command to set servo or motor speed is always 0x40
  static const uint8_t PROTOCOL_COMMAND_DISCOVER = 0x80; // Command discover sensor (lowest 4 bits are sensor)
  static const uint8_t PROTOCOL_COMMAND_TYPE = 0x90;     // Command discover sensor (lowest 4 bits are sensor)
  static const uint8_t PROTOCOL_COMMAND_VALUE = 0xA0;    // Command send sensor data (lowest 4 bits are sensor)
  static const uint8_t SENSORMAX = 10; // Max number of sensors
  
  uint8_t state;                    // state machine state for iBUS protocol
  HardwareSerial *stream;           // serial port
  uint32_t last;                    // milis() of prior message
  uint8_t buffer[PROTOCOL_LENGTH];  // message buffer
  uint8_t ptr;                      // pointer in buffer
  uint8_t len;                      // message length
  uint16_t channel[PROTOCOL_CHANNELS]; // servo data received
  uint16_t chksum;                  // checksum calculation
  uint8_t lchksum;                  // checksum lower byte received
  typedef struct {
    uint8_t sensorType;             // sensor type (0,1,2,3, etc)
    uint8_t sensorLength;           // data length for defined sensor (can be 2 or 4)
    int32_t sensorValue;            // sensor data for defined sensors (16 or 32 bits)
  } sensorinfo;
  sensorinfo sensors[SENSORMAX];
  uint8_t NumberSensors = 0;        // number of sensors
  IBusBM* IBusBMnext = NULL;        // pointer to the next class instance to be used to call the loop() method from timer interrupt
};

#endif
