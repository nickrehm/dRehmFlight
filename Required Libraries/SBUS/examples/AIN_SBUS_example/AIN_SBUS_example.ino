/*
SBUS_example.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2016 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// This example reads 10 analog inputs, linearly maps them to SBUS
// servo commands and sends the command to the servos. In this case
// an interrupt is used to control packet timing.

#include <TimerOne.h>
#include "SBUS.h"

// a SBUS object, which is on hardware
// serial port 1
SBUS x8r(Serial1);

// analog read values, 16 bit counts
uint16_t ain[10];

void setup() {

	// serial to display the channel commands for debugging
	Serial.begin(115200);

	// begin the SBUS communication
	x8r.begin();

	// setup the analog read resolution to 16 bits
	analogReadResolution(16);

	// setup an interrupt to send packets every 9 ms
	Timer1.initialize(9000);
	Timer1.attachInterrupt(sendSBUS);
}

void loop() {

}

/* reads analog inputs and sends an SBUS packet */
void sendSBUS() {
	float scaleFactor = 1639.0f / 65535.0f;
	float bias = 172.0f;
	uint16_t channels[16];

	// read the analog inputs
	for(uint8_t i = 14; i < 24; i++) {
		ain[i-14] = analogRead(i);
	}

	// linearly map the analog measurements (0-65535)
	// to the SBUS commands (172-1811)
	for(uint8_t i = 0; i < 10; i++) {
		channels[i] = (uint16_t)(((float)ain[i]) * scaleFactor + bias);
		Serial.print(channels[i]); // print the channel command (172-1811)
		Serial.print("\t");
	}
	Serial.println();

	// write the SBUS packet to an SBUS compatible servo
  x8r.write(&channels[0]);
}

