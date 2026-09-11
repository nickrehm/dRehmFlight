#ifndef PPMTest
#define PPMTest

#include <Arduino.h>

class PPMTest {

    public:
    
    // The range of a channel's possible values (microseconds)
    unsigned minChannelValue = 1000;
    unsigned maxChannelValue = 2000;

    // The maximum error (in either direction) in channel value with which the channel value is still considered valid
    unsigned channelValueMaxError = 10;

    // The minimum blanking time (microseconds) after which the frame current is considered to be finished
	// Should be bigger than maxChannelValue + channelValueMaxError
    unsigned blankTime = 2100;
	
	// The timeout (microseconds) after which the channels which were not updated are considered invalid
	unsigned long failsafeTimeout = 500000L;


    private:

        // The pin from which to listen for interrupts
        byte interruptPin = 0;

        // The number of channels to be expected in the PPM signal
        byte channelAmount = 0;

        // Arrays for keeping track of channel values
        volatile unsigned *rawValues = NULL;
        volatile unsigned ppmcounter = 0;
        volatile unsigned long time_ms = 0;

        // A counter variable for determining which channel is being read next
        volatile byte pulseCounter = 0;

        // A time variable to remember when the last pulse was read
        volatile unsigned long microsAtLastPulse = 0;

        // Pointer to PPMTest object used by ISR. Replace by an array if multiple PPM reader instances are needed
        static PPMTest *ppm;
        
        uint16_t *_channel_pwm; 

    public:

        PPMTest(byte interruptPin, byte channelAmount);
        ~PPMTest(void);

        // Returns the latest raw (not necessarily valid) value for the channel (starting from 1)
        unsigned rawChannelValue(byte channel);

        // Returns the latest received value that was considered valid for the channel (starting from 1)
        // Returns defaultValue if the channel hasn't received any valid values yet, or the PPM signal was absent for more than failsafeTimeout
        // unsigned latestValidChannelValue(byte channel, unsigned defaultValue);
        void getValues(uint16_t* returnArray);


    private:

        // An interrupt service routine for handling the interrupts activated by PPM pulses
        void handleInterrupt(void);

        // Interrupt service routine function compatible with attachInterrupt. Add more funcitons if multiple PPM reader instances are needed
        static void PPM_ISR(void);

};

#endif