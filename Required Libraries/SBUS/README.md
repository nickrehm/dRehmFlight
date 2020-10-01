# SBUS
Arduino library for communicating with SBUS receivers and servos. SBUS uses inverted serial logic with a baud rate of 100000, 8 data bits, even parity bit, and 2 stop bits. This library works with Teensy 3.x and LC devices, the [STM32L4](https://github.com/simondlevy/grumpyoldpizza), the Maple Mini, and ESP32 (see below). If you have other Arduino devices or port this library, I would appreciate pull requests to update this to work with as many devices as possible.

# License
This library is licensed under the GPLV3. Please contact us at [support@bolderflight.com](mailto:support@bolderflight.com) to obtain other licenses.

## Using SBUS on ESP32
ESP32 may need a logic level converter from 5V to 3.3V.

Additionally, to use uninverted sbus FrSky signal pin, please refer to [Oscar Liang's post](https://oscarliang.com/uninverted-sbus-smart-port-frsky-receivers/).

## Using SBUS on Arduino Mega 2560
Since SBUS uses inverted serial logic, you need to connect the receivers and servors through a signal inverter circuit as explained [here](https://dev.px4.io/en/tutorials/linux_sbus.html). Basically, you need to use an NPN transistor with the following connections:
* connect SBUS signal to the base of the NPN transistor through a 1K resistor
* connect GND of the Arduino board to the emitter of the transistor
* connect Arduino operating voltage (5V for Mega 2560) to the collector of the transistor through a 10K resistor
* connect RX port of the used Arduino serial port to the collector of the transistor

# Description
SBUS is a protocol for RC receivers to send commands to servos. Unlike PWM, SBUS uses a bus architecture where a single signal line can be connected up to 16 servos with each receiving a unique command. SBUS capable servos are required; each can be programmed with a unique address (Channel 0 - 15) using an SBUS servo programmer. Advantages of SBUS include the reduction of wiring clutter and ease of parsing commands from RC receivers.

Much of the information gathered on the SBUS protocol comes from [Uwe Gartmann](https://developer.mbed.org/users/Digixx/notebook/futaba-s-bus-controlled-by-mbed/). Implementation of sending SBUS packet data to servos on Teensy devices was greatly aided by this discussion and feedback from [Paul Stroffregen](https://forum.pjrc.com/archive/index.php/t-23956.html).

The SBUS protocol uses inverted serial logic with a baud rate of 100000, 8 data bits, even parity bit, and 2 stop bits. The SBUS packet is 25 bytes long consisting of:
* Byte[0]: SBUS Header, 0x0F
* Byte[1-22]: 16 servo channels, 11 bits per servo channel
* Byte[23]:
   * Bit 7: digital channel 17 (0x80)
   * Bit 6: digital channel 18 (0x40)
   * Bit 5: frame lost (0x20)
   * Bit 4: failsafe activated (0x10)
   * Bit 0 - 3: n/a
* Byte[24]: SBUS End Byte, 0x00

A table mapping bytes[1-22] to servo channels is [included](https://github.com/bolderflight/SBUS/blob/master/extras/bit-mapping.pdf).

This library has two basic modes of functionality:

1. Reading and parsing SBUS packets. This is useful for using an SBUS capable receiver to receive commands from a RC transmitter and parse the SBUS packet for vehicle control (closed loop control laws, servo mapping, mode change commands) and logging. SBUS packet reading and parsing is available with: raw count data, calibrated to a +/- 1.0 float with adjustable endpoints, and calibrated with a polynomial.
2. Writing SBUS packets. This is useful for commanding up to 16 SBUS capable servos from the Teensy device. SBUS can be written with raw count data, +/- 1.0 floats with adjustable endpoints, and polynomial calibration.  

This library has been tested using FrSky SBUS capable receivers (X8R and X4R) and FrSky SBUS capable servos (D25MA). Feedback from users, especially with other brand receivers and servos (i.e. Futaba), would be greatly appreciated. Digital channels 17 and 18 have not been implmented yet, due to lack of support with FrSky products; further testing is needed from users with Futaba devices.

# Usage
Simply clone or download and extract the zipped library into your Arduino/libraries folder.

Bind your SBUS capable receiver to your transmitter. Setup your SBUS capable servos by programming each with a unique channel number.

**SBUS(HardwareSerial& bus)**
A SBUS object should be declared, specifying the hardware serial port the SBUS receiver and servos are connected to. For example, the following code declares a SBUS object called *x8r* located on hardware serial port 1:

```C++
SBUS x8r(Serial1);
```

**void begin()**
This should be called in your setup function. It initializes the serial communication between the device and SBUS receiver and servos.

```C++
x8r.begin();
```

**bool read(uint16_t&ast; channels, bool&ast; failsafe, bool&ast; lostFrame)**
*read(uint16_t&ast; channels, bool&ast; failsafe, bool&ast; lostFrame)* reads data from the SBUS receiver and parses the SBUS packet. When a complete packet is received, *read(uint16_t&ast; channels, bool&ast; failsafe, bool&ast; lostFrame)* returns *true* and the *channels[0-15]*, *failsafe*, and *lost frame* data is available. Note that *lost frame* is true if the frame was lost and failsafe is true if the receiver has entered failsafe mode. Lost frame is typically used as part of a counter to collect lost frames data for evaluating receiver performance. For example, placing the following code in the loop function will print the value of *channel 0* every time a valid SBUS packet is received.

```C++
uint16_t channels[16];
bool failSafe;
bool lostFrame;

if(x8r.read(&channels[0], &failSafe, &lostFrame)){
	Serial.println(channels[0]);
}
```

**bool readCal(float&ast; channels, bool&ast; failsafe, bool&ast; lostFrame)**
*readCal(float&ast; channels, bool&ast; failsafe, bool&ast; lostFrame)* reads data from the SBUS receiver and parses the SBUS packet. The data from *channels[0-15]* is calibrated to a +/- 1.0 float value assuming a linear relationship based on the minimum and maximum value, or endpoints, for each channel. By default these are set to 172 and 1811, respectively, using FrSky set to 0-100%. The functions described below describe how to modify these endpoints. This is useful if you have endpoints setup differently in your transmitter, but would still like to map to a +/- 1.0 value. Following scaling based on endpoint values, the channels data can optionally be calibrated via polynomial evaluation. The functions described below describe how to setup these polynomials. This could be useful for cases where you would like to receive input data in terms of resulting control surface position for a fixed wing aircraft instead of +/- 1.0 scaled values.

 When a complete packet is received, *readCal(float&ast; channels, bool&ast; failsafe, bool&ast; lostFrame)* returns *true* and the *channels[0-15]*, *failsafe*, and *lost frame* data is available. Note that *lost frame* is true if the frame was lost and failsafe is true if the receiver has entered failsafe mode. Lost frame is typically used as part of a counter to collect lost frames data for evaluating receiver performance. For example, placing the following code in the loop function will print the calibrated value of *channel 0* every time a valid SBUS packet is received.

```C++
float channels[16];
bool failSafe;
bool lostFrame;

if(x8r.readCal(&channels[0], &failSafe, &lostFrame)){
	Serial.println(channels[0]);
}
```

**void write(uint16_t&ast; channels)**
*write(uint16_t&ast; channels)* writes the SBUS packet to SBUS capable servos given position commands from *channels[0-15]*. Note that this function simply creates and sends the SBUS packet, but does not handle timing (i.e. the time between sending subsequent SBUS packets). This timing must be handled by the calling function. For example, placing the following code in the loop function will create and send the SBUS packet to servos every time a valid SBUS packet is received. For FrSky products, a packet is typically sent every 10 ms. Futaba products typically send packets every 7 ms or 14 ms.

```C++
uint16_t channels[16];
bool failSafe;
bool lostFrame;

// look for a good SBUS packet from the receiver
if(x8r.read(&channels[0], &failSafe, &lostFrame)){
	// write the SBUS packet to SBUS compatible servos
    x8r.write(&channels[0]);
}
```

**void writeCal(float&ast; channels)**
*writeCal(float&ast; channels)* writes the SBUS packet to SBUS capable servos given position commands from *channels[0-15]*. In this case, instead of expecting raw count data, writeCal expects a +/- 1.0 calibrated value and converts that to count data using the endpoints set for each channel. Additionally a polynomial can be applied to each channel, which can be useful for mapping an angle based control surface command to an SBUS command. Methods below describe this in more detail.

Note that this function simply creates and sends the SBUS packet, but does not handle timing (i.e. the time between sending subsequent SBUS packets). This timing must be handled by the calling function. For example, placing the following code in the loop function will create and send the SBUS packet to servos every time a valid SBUS packet is received. For FrSky products, a packet is typically sent every 10 ms. Futaba products typically send packets every 7 ms or 14 ms.

```C++
float channels[16];
bool failSafe;
bool lostFrame;

// look for a good SBUS packet from the receiver
if(x8r.readCal(&channels[0], &failSafe, &lostFrame)){
	// write the SBUS packet to SBUS compatible servos
    x8r.writeCal(&channels[0]);
}
```

**void setEndPoints(uint8_t channel,uint16_t min,uint16_t max)**
This sets the endpoints for readCal and writeCal for the given channel number. By default endpoints are set to 172 to 1811.

**void getEndPoints(uint8_t channel,uint16_t&ast; min,uint16_t&ast; max)**
This method retrieves the endpoints for the given channel number.

**void setReadCal(uint8_t channel,float&ast; coeff,uint8_t len)**
This method sets a polynomial for the given channel used by the *readCal* method. The coefficients should be an array of length *len* given in descending order. For example, *float cal[2] = {2,0}* would apply a scale factor of two, with zero bias, to the *readCal* data from the given channel resulting in data ranging from +/- 2.0 instead of +/- 1.0.

**void getReadCal(uint8_t channel,float&ast; coeff,uint8_t len)**
This method retrieves the polynomial coefficients for the given channel storing them in an array, *coeff*. *len* should give the maximum dimensions of the array and is used to avoid buffer overflows.

**void setWriteCal(uint8_t channel,float&ast; coeff,uint8_t len)**
This method sets a polynomial for the given channel used by the *writeCal* method. The coefficients should be an array of length *len* given in descending order. For example, *float cal[2] = {0.033,0}* would apply a scale factor 1/30, with zero bias, to the input data for the given channel before converting from +/- 1.0 to raw counts using the endpoints. This is useful if you would like to command servos based on control surface position instead of raw counts (172 - 1811) or normalized (+/- 1.0) inputs.

**void getWriteCal(uint8_t channel,float&ast; coeff,uint8_t len)**
This method retrieves the polynomial coefficients for the given channel storing them in an array, *coeff*. *len* should give the maximum dimensions of the array and is used to avoid buffer overflows.

# Wiring
Please refer to your microcontroller's pin diagram for hardware serial port pin information. The SBUS capable receiver ground should be connected to microcontroller ground. The receiver power may be connected to a 5V power source. Receiver signal should be connected to the microcontroller RX pin on the specified hardware serial port. The SBUS capable servo ground should be connected to the microcontroller ground. **Servo power must come from an external power source**; your microcontroller is likely not capable of supplying the necessary current. Servo signal should be connected to the microcontroller TX pin on the specified hardware serial port.
