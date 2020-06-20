# dRehmFlight
Teensy based, no BS flight controller for all types of VTOL and multirotor platforms.

## Overview
This project is a work in progress, and was originally developed for research vehicle platforms at the University of Maryland Alfred Gessow Rotorcraft Center. Simply put, engineers and hobbyists may not possess the required software skillset to fully understand other flight control software packages available today. We are not all software engineers, and that's okay. This software package is an attempt to develope an easy to understand flight controller solution that can be easily modified for almost any vehicle configuration. The flight control code follows a sequential step-by-step method to achieve full functionality that is easy to follow, debug, and modify for your purposes. The code is designed to run on a Teensy 4.0 microcontroller with the MPU6050 IMU which together cost less than $30. The code is compiled and uploaded to the board using the arduino IDE and the Teensyduino add-on. 

This code is entirely free to use. If you found this helpful for your project, donations are appreciated: [Dontate](paypal.me/NicholasRehm)

Full disclosure: I am not a sotware engineer, and my code has never been perfect. I barely know how to use github. This code is meant to be easy to understand and follow along, NOT be highly optimized. The Teensy 4.0 board allows for some sloppy, unoptimized code to run at more than adequate speeds for our purposes, favoring unstandability over optimization. This code, while not "industry standard," should allow anyone who is technically inclined to follow along relatively easily.


### Hardware Requirements
This flight controller is based off of the Teensy 4.0 microcontroller and MPU650 6DOF IMU. The following components (available on Amazon) are required to complete the flight controller assembly:


**Teensy 4.0**: https://amzn.to/2V1q2Gw

**MPU6050 IMU**: https://amzn.to/3edF1Vn

**4x6cm Proto PCB**: https://amzn.to/37KGXlE

**Pin Headers**: https://amzn.to/2YjU4rc

**24awg Silicone Wire**: https://amzn.to/2UYX1Lq


### Software Requirments
Code is uploaded to the board using the Arduino IDE; download the latest version here: https://www.arduino.cc/en/main/software

To connect to the Teensy, you must also download and install the Teensyduino arduino add-on; download and instructions available here: https://www.pjrc.com/teensy/td_download.html


## Hardware Setup


## Software Setup and Working With Teensyduino


## Setting Up Your Radio Connection


## Troubleshooting


## FAQ
**Q: Why is it named 'dRehmFlight'?**

**A:** Because I wrote it so I get to name it what I want.
