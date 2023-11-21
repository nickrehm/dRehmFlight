/*
   Spektrum DSM receiver class

   Copyright (C) Simon D. Levy 2017

   This file is part of DSMRX.

   DSMRX is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   DSMRX is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with DSMRX.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

class DSMRX {

    private:

        static const uint8_t BUFFER_SIZE  = 16;
        static const uint8_t MAX_CHANS    = 8;

        // Modified by serial-event handler
        volatile uint8_t  _rxBuf[BUFFER_SIZE];
        volatile uint8_t  _rxBufPos;
        volatile uint16_t _rcValue[MAX_CHANS];
        volatile uint32_t _lastInterruptMicros;

        uint8_t _rcChans;
        uint8_t _chanShift;
        uint8_t _chanMask;
        uint8_t _valShift;
        uint8_t _fadeCount;

        bool _gotNewFrame;

    protected:

        DSMRX(uint8_t rc, uint8_t cs, uint8_t cm, uint8_t vs);

    public:

        void handleSerialEvent(uint8_t value, uint32_t usec);

        bool gotNewFrame(void);

        /**
         * Returns channel values in [1000,2000] interval
         */
        void getChannelValues(uint16_t values[], uint8_t count=8);

        /**
         * Returns channel values in [-1,+1] interval
         */
        void getChannelValuesNormalized(float values[], uint8_t count=8);

        uint8_t getFadeCount(void);

        bool timedOut(uint32_t usec, uint32_t maxMicros=40000);
};

class DSM1024 : public DSMRX {

    public:

        DSM1024(void);

};

class DSM2048 : public DSMRX {

    public:

        DSM2048(void);
};

