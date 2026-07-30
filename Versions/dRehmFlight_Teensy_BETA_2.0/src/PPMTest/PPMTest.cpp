/*
Copyright 2016 Aapo Nikkilä
Copyright 2021 Dmitry Grigoryev

This file is part of PPM Reader.

PPM Reader is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

PPM Reader is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with PPM Reader.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "PPMTest.h"

PPMTest *PPMTest::ppm;

void PPMTest::PPM_ISR(void) {
  ppm->handleInterrupt(); 
}


PPMTest::PPMTest(byte interruptPin, byte channelAmount):
    interruptPin(interruptPin), channelAmount(channelAmount) {
    // Setup an array for storing channel values
    rawValues = new unsigned [channelAmount];
    _channel_pwm = new uint16_t [channelAmount];

    for (int i = 0; i < channelAmount; ++i) {
        rawValues[i] = 0;
    }
    // Attach an interrupt to the pin
    pinMode(interruptPin, INPUT);
    if(ppm == NULL) {
        ppm = this;
        attachInterrupt(digitalPinToInterrupt(interruptPin), PPM_ISR, RISING);
    }
}

PPMTest::~PPMTest(void) {
    detachInterrupt(digitalPinToInterrupt(interruptPin));
    if(ppm == this) ppm = NULL;
    delete [] rawValues;
}

void PPMTest::handleInterrupt(void) {
    // Remember the current micros() and calculate the time since the last pulseReceived()
    // unsigned long previousMicros = microsAtLastPulse;
    // microsAtLastPulse = micros();
    // unsigned long time = microsAtLastPulse - previousMicros;

    // if (time > blankTime) {
    //     // Blank detected: restart from channel 1 
    //     pulseCounter = 0;
    // }
    // else {
    //     // Store times between pulses as channel values
    //     if (pulseCounter < channelAmount) {
    //         rawValues[pulseCounter] = time;
    //         ++pulseCounter;
    //     }
    // }

    unsigned long dt_ppm;
    dt_ppm = micros() - time_ms;
    time_ms = micros();

    
    if (dt_ppm > 5000) { //Waiting for long pulse to indicate a new pulse train has arrived
        ppm_counter = 0;
        for (uint8_t i=0; i<channelAmount; i++) {
            _channel_pwm[i] = rawValues[i];
        })
    }
    else if (ppm_counter < channelAmount) {
        rawValues[ppm_counter] = time;
        ++ppm_counter;
    }
}


unsigned PPMTest::rawChannelValue(byte channel) {
    // Check for channel's validity and return the latest raw channel value or 0
    unsigned value = 1000;
    if (channel >= 1 && channel <= channelAmount) {
        noInterrupts();
        value = rawValues[channel-1];
        interrupts();
    }
    return value;
}

void PPMTest::getValues(uint16_t* returnArray) {
    for (uint8_t i=0; i<channelAmount; i++) {
        returnArray[i] = _channel_pwm[i];
    }

}


// unsigned PPMTest::latestValidChannelValue(byte channel, unsigned defaultValue) {
//     // Check for channel's validity and return the latest valid channel value or defaultValue.
//     unsigned value = defaultValue;
// 	unsigned long timeout;
// 	noInterrupts();
// 	timeout = micros() - microsAtLastPulse;
// 	interrupts();
//     if ((timeout < failsafeTimeout) && (channel >= 1) && (channel <= channelAmount)) {
//         noInterrupts();
//         value = rawValues[channel-1];
// 		interrupts();
// 		if (value >= minChannelValue - channelValueMaxError && value <= maxChannelValue + channelValueMaxError) {
// 			value = constrain(value, minChannelValue, maxChannelValue);
// 		}
//         else value = defaultValue;
//     }
//     return value;
// }
