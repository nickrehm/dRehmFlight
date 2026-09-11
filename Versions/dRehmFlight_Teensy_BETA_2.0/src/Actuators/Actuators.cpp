//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Additional Author: Brian Jones
//Project Start: 1/6/2020
//Last Updated: 8/11/2025
//Version: Beta 2.0

//========================================================================================================================//

// This is a wrapper for outputs supported by dRehmFlight.  It consolidates all the output (Servo, ESC) handling into a single library that most people won't need to mess with.

#include "Actuators.h"

Actuators::Actuators(OutputType type, uint8_t numActuators, uint8_t* actuatorPins) {
    _servos = nullptr;
    uint8_t i;

    _type = type;
    _numActuators = numActuators;
    for (i=0; i<_numActuators; i++) {
        _actuatorPins[i] = actuatorPins[i];
    }

    // Servos and PWM ESCs behave the same, but the initialization is a little different
    if (_type == SERVO) {
        _servos = new PWMServo[_numActuators];
    }
    else if (_type == ESC) {
        _servos = new PWMServo[_numActuators];
    }
    else if (_type == ONESHOT125) {
        for (i=0; i<_numActuators; i++) {
            pinMode(_actuatorPins[i], OUTPUT);
        }
    }
    else {
        // error, unknown type
    }
}
  

bool Actuators::begin() {
    uint8_t i;

    switch (_type) {
        case SERVO:
        {
            for (i=0; i<_numActuators; i++) {
                _servos[i].attach(_actuatorPins[i], SERVO_PWM_MIN, SERVO_PWM_MAX); //Pin, min PWM value, max PWM value
                _servos[i].write(DEFAULT_SERVO_POS);
            }
            return true;
        }

        case ESC:
        {
            for (i=0; i<_numActuators; i++) {
                _servos[i].attach(_actuatorPins[i], SERVO_PWM_MIN, SERVO_PWM_MAX); //Pin, min PWM value, max PWM value
                _servos[i].write(DEFAULT_ESC_POS);
            }
            return true;
        }
        
        case ONESHOT125:
        {
            //Arm OneShot125 motors
            //was in armMotors(); 
            //Loop over commandMotors() until ESCs happily arm
            for (int i = 0; i <= 50; i++) {
                commandAllSame(DEFAULT_OS125_POS);
                delay(2);
            }
        
            return true;
        }

        default:
            return false;
    }
}


void Actuators::commandOne(uint8_t index, float output) {
    //DESCRIPTION: Used to command a single Servo or Motor
    /*
    * The input is the index in the motor or servo array, and the output is between 0.0 and 1.0.
    * The output will be scaled appropriately and sent to the correct output type.
    */

    // input safety check.  If index is out of bounds, nothing happens
    if (index < _numActuators) {
        if (_type == SERVO || _type == ESC) {
            int angle;
            //Scaled to 0-180 for servo library
            angle = output * 180;
            //Constrain commands to servos within servo library bounds
            angle = constrain(angle, 0, 180);

            // Write output
            _servos[index].write(angle);
        }
        else if (_type == ONESHOT125) {
            int wentLow = 0;
            int pulseStart, timer;
            int flagM = 0;
            int OSoutput = 125;
            
            //Write motor pins high
            digitalWrite(_actuatorPins[index], HIGH);
            pulseStart = micros();

            //Write each motor pin low as correct pulse length is reached
            while (wentLow < 1 ) { //Keep going until pulse is finished, then done
                timer = micros();
                    
                //Scaled to 125us - 250us for oneshot125 protocol
                OSoutput = output*125 + 125;
                //Constrain commands to motors within oneshot125 bounds
                OSoutput = constrain(OSoutput, 125, 250);

                if ((OSoutput <= timer - pulseStart) && (flagM == 0)) {
                    digitalWrite(_actuatorPins[index], LOW);
                    wentLow = wentLow + 1;
                    flagM = 1;
                }
            }
        }
        else {
            // Error
        }
    }
}

void Actuators::commandAll(float* output) {
    //DESCRIPTION: Used to command all outputs at once.
    /*
    * The input is the array of outputs for each actuator.
    * The function will loop through each actuator and assign it the matching output.
    * Outputs need to be between 0.0 and 1.0.
    * The output will be scaled appropriately and sent to the correct output type.
    */

    if (_type == SERVO || _type == ESC) {
        int angle;
        for (uint8_t i = 0; i < _numActuators; i++) {
            //Scaled to 0-180 for servo library
            angle = output[i] * 180;
            //Constrain commands to servos within servo library bounds
            angle = constrain(angle, 0, 180);

            // Write output
            _servos[i].write(angle);
        }
    }
    else if (_type == ONESHOT125) {
        //DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
        /*
        * My crude implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin). The pulselengths being
        * sent are mX_command_PWM, computed in scaleCommands(). This may be replaced by something more efficient in the future.
        */
        int wentLow = 0;
        int pulseStart, timer;
        int flagM[_numActuators] = {0};
        int OSoutput = 125;
        
        //Write all motor pins high
        for (uint8_t i = 0; i < _numActuators; i++) {
            digitalWrite(_actuatorPins[i], HIGH);
        }
        pulseStart = micros();

        //Write each motor pin low as correct pulse length is reached
        while (wentLow < _numActuators ) { //Keep going until final pulse is finished, then done
            timer = micros();
            for (uint8_t i = 0; i < _numActuators; i++) {
                //Scaled to 125us - 250us for oneshot125 protocol
                OSoutput = output[i]*125 + 125;
                //Constrain commands to motors within oneshot125 bounds
                OSoutput = constrain(OSoutput, 125, 250);

                if ((OSoutput <= timer - pulseStart) && (flagM[i]==0)) {
                    digitalWrite(_actuatorPins[i], LOW);
                    wentLow = wentLow + 1;
                    flagM[i] = 1;
                }
            }
        }
    }
    else {
        // Error
    }  
}

void Actuators::commandAllSame(float output) {
    //DESCRIPTION: Used to command all outputs at once to the same output value.
    /*
    * The input is the single output desired for all actuators.
    * The function will loop through each actuator and assign it the output.
    * Output need to be between 0.0 and 1.0.
    * The output will be scaled appropriately and sent to the correct output type.
    */

    if (_type == SERVO || _type == ESC) {
        int angle;
        for (uint8_t i = 0; i < _numActuators; i++) {
            //Scaled to 0-180 for servo library
            angle = output * 180;
            //Constrain commands to servos within servo library bounds
            angle = constrain(angle, 0, 180);

            // Write output
            _servos[i].write(angle);
        }
    }
    else if (_type == ONESHOT125) {
        //DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
        /*
        * My crude implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin). The pulselengths being
        * sent are mX_command_PWM, computed in scaleCommands(). This may be replaced by something more efficient in the future.
        */
        int wentLow = 0;
        int pulseStart, timer;
        int flagM[_numActuators] = {0};
        int OSoutput = 125;
        
        //Write all motor pins high
        for (uint8_t i = 0; i < _numActuators; i++) {
            digitalWrite(_actuatorPins[i], HIGH);
        }
        pulseStart = micros();

        //Write each motor pin low as correct pulse length is reached
        while (wentLow < _numActuators ) { //Keep going until final pulse is finished, then done
            timer = micros();
            for (uint8_t i = 0; i < _numActuators; i++) {
                //Scaled to 125us - 250us for oneshot125 protocol
                OSoutput = output*125 + 125;
                //Constrain commands to motors within oneshot125 bounds
                OSoutput = constrain(OSoutput, 125, 250);

                if ((OSoutput <= timer - pulseStart) && (flagM[i]==0)) {
                    digitalWrite(_actuatorPins[i], LOW);
                    wentLow = wentLow + 1;
                    flagM[i] = 1;
                }
            }
        }
    }
    else {
        // Error
    }  
}

