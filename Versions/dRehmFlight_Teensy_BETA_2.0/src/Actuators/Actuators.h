//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Additional Author: Brian Jones
//Project Start: 1/6/2020
//Last Updated: 8/11/2025
//Version: Beta 2.0

//========================================================================================================================//

// This is a wrapper for outputs supported by dRehmFlight.  It consolidates all the output (Servo, ESC) handling into a single library that most people won't need to mess with.

#ifndef Actuators_h
#define Actuators_h

#include <Arduino.h>
#include <PWMServo.h> //Commanding any extra actuators, installed with teensyduino installer

enum OutputType {
    SERVO,
    ESC,
    ONESHOT125
    // Add more types here
};

#define SERVO_PWM_MIN   900
#define SERVO_PWM_MAX  2100

#define DEFAULT_SERVO_POS   90
#define DEFAULT_ESC_POS      0
#define DEFAULT_OS125_POS  0.0

class Actuators {
public:
	Actuators(OutputType type, uint8_t numActuators, uint8_t* actuatorPins);

	bool begin();

    // Expect all of our outputs to be 0.0 to 1.0 from the Mixer.  Will scale appropriately
    void commandOne(uint8_t index, float output);
    void commandAll(float* output);
    void commandAllSame(float output);

private:
	OutputType  _type;

    PWMServo* _servos;

    uint8_t _numActuators = 1;
    uint8_t _actuatorPins[];
};

#endif