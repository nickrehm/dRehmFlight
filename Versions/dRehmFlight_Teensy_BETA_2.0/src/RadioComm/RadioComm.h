//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Additional Author: Brian Jones
//Project Start: 1/6/2020
//Last Updated: 8/11/2025
//Version: Beta 2.0

//========================================================================================================================//

//This file contains all necessary functions and code used for radio communication to avoid cluttering the main code

#ifndef RadioComm_h
#define RadioComm_h

#include <Arduino.h>

// Include all possible radio libraries
#include "../SBUS/SBUS.h"
// #include "sbus.h" // Bolder Flight Systems SBUS library -- had issues with loop timing using this library
#include <PPMReader.h>
#include "../DSMRX/DSMRX.h"

// Number of times through the loop with no new data that should cause a failsafe
#define LOOP_AUTO_FAILSAFE  32
// Should the getCommands() function also check for failsafe?
#define AUTOFAILSAFE        1

#define PWMMINVALUE 800
#define PWMMAXVALUE 2200




enum RadioType {
    PPM,
    PPMTEST,
    DSM,
    sBUS
    // Removed PWM option because 1) it's not very popular anymore, and 2) The ISRs required to make it work were difficult to implement.
};

enum ChannelOrder {
    TAER,   // (Thr, Ail, Ele, Rud)
    AETR    // (Ail, Ele, Thr, Rud)
    // These are the only options so far and are the most common
};

class RadioComm {
public:
    RadioComm(RadioType type);
    RadioComm(RadioType type, uint8_t num_channels);
    RadioComm(RadioType type, uint8_t num_channels, ChannelOrder order);

    void begin();            //DSM & //SBUS
    void begin(int PPM_Pin); //PPM

    void getCommands(uint16_t* returnArray);
    void setFailsafe(uint16_t* failsafeArray);
    void failSafe(bool force=false);


private:
	RadioType _type;
    ChannelOrder _channel_order;

    // bfs::SbusRx* _sbus;
    // bfs::SbusData _sbus_data;
    SBUS* _sbus;
    DSMRX* _dsm;
    PPMReader* _ppm;
    
    uint8_t  _num_channels = 6;    
    uint16_t _sbusChannels[16] = {0};
    uint16_t _channel_pwm[16] = {0}; 
    uint16_t _channel_pwm_prev[4] = {0};
    uint16_t _channel_fs[16] = {0};
    
    bool _sbusFailSafe = true;
    bool _sbusLostFrame = true;

    uint8_t fs_timeout = 0;

    //sBus scaling below is for Taranis-Plus and X4R-SB
    const float _sbus_scale = 0.615;  
    const float _sbus_bias  = 895.0; 
    int _PPM_Pin = 0;
    unsigned long time_ms = 0;

    void channelMap();

};


#endif