## Versions


### ESP32 Microcontroller

**Beta 2.0:**

Brian Jones - July 29, 2026

- Rewrote the code based on Teensy 1.3 to the ESP32 processor
- Removed support for PWM, PPM, and DSM receivers
- Added support to CRSF protocol (eLRS)
- Added wifi PID controller tuning
- Code reorganization - moved print routines and IMU_PID routines to separate .ino files



### Teensy Microcontroller

**Beta 2.0:**

Brian Jones - Aug. 9, 2025

- Code organization - moved most of the user-defined variables to "dRhemFlight_hardware.h"
- created wrapper libraries to generalize the use of Motors, Servos, ESCs, radios, and IMUs


**Beta 1.5:**

Brian Jones - Aug. 9, 2025

- Code organization - moved debug print routines to separate .ino file


**Beta 1.4:**

Brian Jones - Forked from 1.3 (Jan. 23, 2025)

- Adafruit LSM6DSOX IMU support
- Improved SBUS failsafe detection


**Beta 1.3:**

Last official release on Nick Rehm's Github (Nov. 18, 2023)

- Spektrum DSM Satellite RX Support
- One-time IMU calibration
- ESC calibration functionality
- Motor arming bug fix
- Code and comment clean up
- Small bug fixes


