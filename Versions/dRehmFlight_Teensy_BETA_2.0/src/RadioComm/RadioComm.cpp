//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Additional Author: Brian Jones
//Project Start: 1/6/2020
//Last Updated: 8/11/2025
//Version: Beta 2.0

//========================================================================================================================//

//This file contains all necessary functions and code used for radio communication to avoid cluttering the main code

#include "RadioComm.h"

RadioComm::RadioComm(RadioType type): RadioComm(type, 6, TAER) {};
RadioComm::RadioComm(RadioType type, uint8_t num_channels): RadioComm(type, num_channels, TAER) {};

RadioComm::RadioComm(RadioType type, uint8_t num_channels, ChannelOrder order) {
    _sbus = nullptr;
    // _sbus_data = nullptr;
    _dsm  = nullptr;
    _ppm  = nullptr;

    _type = type;
    _num_channels = num_channels;
    _channel_order = order;
}


void RadioComm::begin() {
    if (_type == sBUS) {
        // _sbus = new bfs::SbusRx(&Serial5, true, false);
        // bfs::SbusData _sbus_data;
        // _sbus->Begin();

        _sbus = new SBUS(Serial5);
        _sbus->begin();
    }
    
    else if (_type == DSM) {
        _dsm = new DSM1024();
        Serial3.begin(115000);
    }

    else {
        Serial.println("No Serial RX type defined...");
        while(1);
    }
}


void RadioComm::begin(int PPM_Pin) {
    _PPM_Pin = PPM_Pin;

    if (_type == PPM) {
        // Using PPMReader
        _ppm = new PPMReader(_PPM_Pin, _num_channels);
        _ppm->minChannelValue = PWMMINVALUE;
        _ppm->maxChannelValue = PWMMAXVALUE;
        _ppm->blankTime = 3000;
    }

    else {
        Serial.println("No PPM type defined...");
        while(1);
    }
}


void RadioComm::getCommands(uint16_t* returnArray) {
    //DESCRIPTION: Get raw PWM values for every channel from the radio
    /*
    * Updates radio PWM commands in loop based on current available commands. The array _channel_pwm[] is the raw command used in the rest of the loop.
    * The raw radio commands of the first four control channels are filtered with a first order low-pass filter to eliminate any really high frequency noise. 
    */

    int8_t i;
    bool FdataRec = false;

    if (_type == sBUS) {
        // _sbus->Read();
        // /* Grab the received data */
        // _sbus_data = _sbus->data();
        // /* Display the received data */
        // for (i = 0; i < _num_channels; i++) {
        //     _channel_pwm[i] = _sbus_data.ch[i]* _sbus_scale + _sbus_bias; 
        // }
        // _sbusFailSafe = _sbus_data.failsafe;
        // _sbusLostFrame = _sbus_data.lost_frame;

        if (_sbus->read(_sbusChannels, &_sbusFailSafe, &_sbusLostFrame)) {
            FdataRec = true;
            fs_timeout = 0;
            // for (int8_t i = 0; i < _num_channels; i++) {
            //     Serial.print(_sbusChannels[i]);
            //     Serial.print("\t");
            // }
            // /* Display lost frames and failsafe data */
            // Serial.print(_sbusLostFrame);
            // Serial.print("\t");
            // Serial.println(_sbusFailSafe);

            for (i=0; i < _num_channels; i++) {
                _channel_pwm[i] = _sbusChannels[i] * _sbus_scale + _sbus_bias;
            }
            this->channelMap();
        }
        else {
            fs_timeout++;
        }
    }

    else if (_type == PPM) {
        // Tried using PPMReader;
        uint16_t tempVal = 0;
        for (i=0; i < _num_channels; i++) {
            tempVal = _ppm->latestValidChannelValue(i+1,0);
            if (tempVal < PWMMINVALUE) {
                fs_timeout++;
                FdataRec = false;
                // Serial.print("XX\t");
                // break;
            }
            else {
                FdataRec = true;
                _channel_pwm[i] = tempVal;
                // Serial.print(tempVal);
                // Serial.print("\t");

            }
        }


        if (FdataRec == true) {
            fs_timeout = 0;
            this->channelMap();
        }
        // Serial.println("");




        //     tempVal = _ppm->latestValidChannelValue(i+1,0);
        //     if (tempVal < PWMMINVALUE) {
        //         fs_timeout++;
        //         FdataRec = false;
        //         Serial.print("XX\t");
        //         break;
        //     }
        //     else {
        //         FdataRec = true;
        //         _channel_pwm[i] = tempVal;
        //         Serial.print(tempVal);
        //         Serial.print("\t");

        //     }
        // }



    }

    else if (_type == DSM) {
        if (_dsm->timedOut(micros())) {
            //Serial.println("*** DSM RX TIMED OUT ***");
            fs_timeout++;
        }
        else if (_dsm->gotNewFrame()) {
            FdataRec = true;
            fs_timeout = 0;
            // uint16_t values[_num_channels];
            _dsm->getChannelValues(_sbusChannels, _num_channels);

            for (i=0; i < _num_channels; i++) {
                _channel_pwm[i] = _sbusChannels[i];
            }
            this->channelMap();
        }
    }

    else {
        // Should probably have an error here
    }

    if (FdataRec == true) {
        // Low-pass the critical commands and update previous values
        float b = 0.7; //Lower=slower, higher=noiser

        for (i = 0; i < 4; i++) {
            _channel_pwm[i] = (1.0 - b)*_channel_pwm_prev[i] + b*_channel_pwm[i];
            _channel_pwm_prev[i] = _channel_pwm[i];
        }
    }


    // if more than a number of iterations through the loop did not yield new data, force failsafe mode.
    if (fs_timeout > LOOP_AUTO_FAILSAFE) {
        Serial.println("ENTERED LOOP AUTO FAILSAFE");
        this->failSafe(true);
    }
    else if (AUTOFAILSAFE == 1) {
        // Automatically check for failsafe conditions and apply as necessary
        this->failSafe();
    }


    // return the values to the function
    for (i = 0; i < _num_channels; i++) {
        returnArray[i] = _channel_pwm[i];
    }
}


void RadioComm::setFailsafe(uint16_t* failsafeArray) {
    //DESCRIPTION: Used after initialization to set failsafe values for the receiver
    /*
    * This sets the failsafe values, typically throttle off and control surfaces neutral, so that in the event of a loss
    * of radio signal, the craft will likely crash instead of fly away.
    * 
    * Input values should be in the range of 1000-2000 microseconds, as normally delivered by the RC receiver.
    * 1000 us == low throttle position
    * 1500 us == center servo position
    */

    for (int8_t i=0; i<_num_channels; i++) {
        _channel_fs[i] = failsafeArray[i];
    }
}


void RadioComm::failSafe(bool force) {
    //DESCRIPTION: If radio gives garbage values, set all commands to default values
    /*
    * Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of 
    * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
    * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands 
    * channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting 
    * your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
    * 
    * When using SBUS, the library handles the detection of lost frames and failsafe.  Some receivers handle failsafe differently.
    * Testing with FrSky SBUS receivers immediately triggered failsafe when Tx disconnected.  A Radiomaster RP3-H ELRS receiver would
    * not trigger failsafe when set to "No Pulses", but would trigger when set to "Last Position" after a few seconds.
    * Test your receiver's behavior! You can do this with the example sketch in the "Bolder Flight Systems SBUS" library.
    */
    int8_t i;

    // If failsafe is forced, just set the values and return.
    if (force) {
        for (i=0; i<_num_channels; i++) {
            _channel_pwm[i] = _channel_fs[i];
        }
        // Serial.println(F("****    FAILSAFE ON    ****"));
        return;
    }

    //If using SBUS, this will only check the boolean failsafe value from the SBUS library.  If using PPM/DSM RX, it checks each channel.
    if (_type == sBUS) {
        if (_sbusFailSafe) {
            for (i=0; i<_num_channels; i++) {
                _channel_pwm[i] = _channel_fs[i];
            }
            // Serial.println(F("****    FAILSAFE ON    ****"));
        }
        
    }
    
    else {
        int check[6] = {0};
        int sum = 0;

        //Triggers for failure criteria -- if the PWM value < minvalue or > maxvalue, set a flag
        for (i=0; i<6; i++) {
            if (_channel_pwm[i] > PWMMAXVALUE || _channel_pwm[i] < PWMMINVALUE) check[i] = 1;
        }
        
        // Add up all the flags (if not failsafe, this will be 0)
        for (i=0; i<6; i++) {
            sum += check[i];
        }

        // If sum of all checks > 0, enable failsafe
        if (sum > 0) {
            for (i=0; i<_num_channels; i++) {
                _channel_pwm[i] = _channel_fs[i];
            }
            // Serial.println(F("****    FAILSAFE ON    ****"));
        }
    }
}



void RadioComm::channelMap() {
    if (_channel_order == AETR) {
        uint16_t tmp_thr;
        uint16_t tmp_ail;
        uint16_t tmp_ele;

        // assign AET inputs to temp vars - they will be in the wrong order for our use
        tmp_ail = _channel_pwm[0];
        tmp_ele = _channel_pwm[1];
        tmp_thr = _channel_pwm[2];

        // Flip them around for TAER for use in DF:  
        _channel_pwm[0] = tmp_thr;
        _channel_pwm[1] = tmp_ail;
        _channel_pwm[2] = tmp_ele;
    }
    else {      // TAER do nothing
        return;
    }
}


