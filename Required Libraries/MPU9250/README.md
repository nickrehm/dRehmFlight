# MPU9250
Arduino library for communicating with the [MPU-9250](https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/) and MPU-9255 nine-axis Inertial Measurement Units (IMU).

# License
This library is licensed under the GPLV3. Please contact us at [support@bolderflight.com](mailto:support@bolderflight.com) to obtain other licenses.

# Description
The InvenSense MPU-9250 is a System in Package (SiP) that combines two chips: the MPU-6500 three-axis gyroscope and three-axis accelerometer; and the AK8963 three-axis magnetometer. The MPU-9250 supports I2C, up to 400 kHz, and SPI communication, up to 1 MHz for register setup and 20 MHz for data reading. The following selectable full scale sensor ranges are available:

| Gyroscope Full Scale Range | Accelerometer Full Scale Range | Magnetometer Full Scale Range |
| --- | --- | ---  |
| +/- 250 (deg/s)  | +/- 2 (g)  | +/- 4800 (uT) |
| +/- 500 (deg/s)  | +/- 4 (g)  | |
| +/- 1000 (deg/s) | +/- 8 (g)  | |
| +/- 2000 (deg/s) | +/- 16 (g) | |

The MPU-9250 samples the gyroscopes, accelerometers, and magnetometers with 16 bit analog to digital converters. It also features programmable digital filters, a precision clock, an embedded temperature sensor, programmable interrupts (including wake on motion), and a 512 byte FIFO buffer.

# Usage
This library supports both I2C and SPI commmunication with the MPU-9250.

## Installation
Simply clone or download this library into your Arduino/libraries folder.

## Function Description
This library supports both I2C and SPI communication with the MPU-9250. The *MPU9250* object declaration is overloaded with different declarations for I2C and SPI communication. All other functions remain the same. Additionally, a derived class, *MPU250FIFO*, is included, which provides FIFO setup and data collection functionality in addition to all of the functionality included in the base *MPU9250* class.  

## MPU9250 Class

### I2C Object Declaration

**MPU9250(TwoWire &bus,uint8_t address)**
An MPU9250 object should be declared, specifying the I2C bus and MPU-9250 I2C address. The MPU-9250 I2C address will be 0x68 if the AD0 pin is grounded or 0x69 if the AD0 pin is pulled high. For example, the following code declares an MPU9250 object called *IMU* with an MPU-9250 sensor located on I2C bus 0 with a sensor address of 0x68 (AD0 grounded).

```C++
MPU9250 IMU(Wire,0x68);
```

### SPI Object Declaratioon

**MPU9250(SPIClass &bus,uint8_t csPin)**
An MPU9250 object should be declared, specifying the SPI bus and chip select pin used. Multiple MPU-9250 or other SPI objects could be used on the same SPI bus, each with their own chip select pin. The chip select pin can be any available digital pin. For example, the following code declares an MPU9250 object called *IMU* with an MPU-9250 sensor located on SPI bus 0 with chip select pin 10.

```C++
MPU9250 IMU(SPI,10);
```

### Common Setup Functions
The following functions are used to setup the MPU-9250 sensor. These should be called once before data collection, typically this is done in the Arduino *void setup()* function. The *begin* function should always be used. Optionally, the *setAccelRange* and *setGyroRange*, *setDlpfBandwidth*, and *setSrd* functions can be used to set the accelerometer and gyroscope full scale ranges, DLPF bandwidth, and SRD to values other than default. The *enableDataReadyInterrupt* and *disableDataReadyInterrupt* control whether the MPU-9250 generates an interrupt on data ready. The *enableWakeOnMotion* puts the MPU-9250 into a low power mode and enables an interrupt when motion detected is above a given threshold. Finally, *enableFifo* sets up and enables the FIFO buffer. These functions are described in detail, below.

**int begin()**
This should be called in your setup function. It initializes communication with the MPU-9250, sets up the sensor for reading data, and estimates the gyro bias, which is removed from the sensor data. This function returns a positive value on a successful initialization and returns a negative value on an unsuccesful initialization. If unsuccessful, please check your wiring or try resetting power to the sensor. The following is an example of setting up the MPU-9250.

```C++
int status;
status = IMU.begin();
```

#### Configuration Functions

**(optional) int setAccelRange(AccelRange range)**
This function sets the accelerometer full scale range to the given  value. By default, if this function is not called, a full scale range of +/- 16 g will be used. The enumerated accelerometer full scale ranges are:

| Accelerometer Name | Accelerometer Full Scale Range | 
| ------------------ | ------------------------------ | 
| ACCEL_RANGE_2G     | +/- 2 (g)                      |
| ACCEL_RANGE_4G     | +/- 4 (g)                      | 
| ACCEL_RANGE_8G     | +/- 8 (g)                      |
| ACCEL_RANGE_16G    | +/- 16 (g)                     | 

This function returns a positive value on success and a negative value on failure. Please see the *Advanced_I2C example*. The following is an example of selecting an accelerometer full scale range of +/- 8g.

```C++
status = IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
```

**(optional) int setGyroRange(GyroRange range)**
This function sets the gyroscope full scale range to the given  value. By default, if this function is not called, a full scale range of +/- 2000 deg/s will be used. The enumerated gyroscope full scale ranges are:

| Gyroscope Name     | Gyroscope Full Scale Range |
| ------------------ | -------------------------- |
| GYRO_RANGE_250DPS  | +/- 250 (deg/s)            |
| GYRO_RANGE_500DPS  | +/- 500 (deg/s)            |
| GYRO_RANGE_1000DPS | +/- 1000 (deg/s)           |
| GYRO_RANGE_2000DPS | +/- 2000 (deg/s)           | 

This function returns a positive value on success and a negative value on failure. Please see the *Advanced_I2C example*. The following is an example of selecting an gyroscope full scale range of +/- 250 deg/s.

```C++
status = IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
```

**(optional) int setDlpfBandwidth(DlpfBandwidth bandwidth)**
This is an optional function to set the programmable Digital Low Pass Filter (DLPF) bandwidth. By default, if this function is not called, a DLPF bandwidth of 184 Hz is used. The following DLPF bandwidths are supported:

| Bandwidth Name | DLPF Bandwidth | Gyroscope Delay | Accelerometer Delay | Temperature Bandwidth | Temperature Delay |
| --- | --- | --- | --- | --- | --- |
| DLPF_BANDWIDTH_184HZ | 184 Hz | 2.9 ms   | 5.8 ms   | 188 Hz | 1.9 ms  |
| DLPF_BANDWIDTH_92HZ  | 92 Hz  | 3.9 ms   | 7.8 ms   | 98 Hz  | 2.8 ms  |
| DLPF_BANDWIDTH_41HZ  | 41 Hz  | 5.9 ms   | 11.8 ms  | 42 Hz  | 4.8 ms  |
| DLPF_BANDWIDTH_20HZ  | 20 Hz  | 9.9 ms   | 19.8 ms  | 20 Hz  | 8.3 ms  |
| DLPF_BANDWIDTH_10HZ  | 10 Hz  | 17.85 ms | 35.7 ms  | 10 Hz  | 13.4 ms |
| DLPF_BANDWIDTH_5HZ   | 5 Hz   | 33.48 ms | 66.96 ms | 5 Hz   | 18.6 ms |

This function returns a positive value on success and a negative value on failure. Please see the *Advanced_I2C example*. The following is an example of selecting a DLPF bandwidth of 20 Hz.

```C++
status = IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
```

**(optional) int setSrd(uint8_t srd)**
This is an optional function to set the data output rate. The data output rate is set by a sample rate divider, *uint8_t SRD*. The data output rate is then given by:

*Data Output Rate = 1000 / (1 + SRD)*

By default, if this function is not called, an SRD of 0 is used resulting in a data output rate of 1000 Hz. This allows the data output rate for the gyroscopes, accelerometers, and temperature sensor to be set between 3.9 Hz and 1000 Hz. Note that data should be read at or above the selected rate. In order to prevent aliasing, the data should be sampled at twice the frequency of the DLPF bandwidth or higher. For example, this means for a DLPF bandwidth set to 41 Hz, the data output rate and data collection should be at frequencies of 82 Hz or higher.

The magnetometer is fixed to an output rate of: 
* 100 Hz for frequencies of 100 Hz or above (SRD less than or equal to 9)
* 8 Hz for frequencies below 100 Hz (SRD greater than 9)

When the data is read above the selected output rate, the read data will be stagnant. For example, when the output rate is selected to 1000 Hz, the magnetometer data will be the same for 10 sequential frames. 

This function returns a positive value on success and a negative value on failure. Please see the *Advanced_I2C example*. The following is an example of selecting an SRD of 9, resulting in a data output rate of 100 Hz.

```C++
status = IMU.setSrd(9);
```

**(optional) int enableDataReadyInterrupt()**
An interrupt is tied to the data output rate. The MPU-9250 *INT* pin will issue a 50us pulse when data is ready. This is extremely useful for using interrupts to clock data collection that should occur at a regular interval. Please see the *Interrupt_SPI example*. This function enables this interrupt, which will occur at a frequency given by the SRD. This function returns a positive value on success and a negative value on failure. The following is an example of enabling the data ready interrupt.

```C++
status = IMU.enableDataReadyInterrupt();
```

**(optional) int disableDataReadyInterrupt()**
This function disables the data ready interrupt, described above. This function returns a positive value on success and a negative value on failure. The following is an example of disabling the data ready interrupt.

```C++
status = IMU.disableDataReadyInterrupt();
```

#### Calibration Functions

**(optional) int calibrateGyro()**
The gyro bias is automatically estimated during the *begin()* function and removed from sensor measurements. This function will re-estimate the gyro bias and remove the new bias from future sensor measurements. The sensor should be stationary during this process. This function returns a positive value on success and a negative value on failure. The following is an example of estimating new gyro biases.

```C++
status = IMU.calibrateGyro();
```

**(optional) float getGyroBiasX_rads()**
This function returns the current gyro bias in the X direction in units of rad/s. 

```C++
float gxb;
gxb = IMU.getGyroBiasX_rads();
```

**(optional) float getGyroBiasY_rads()**
This function returns the current gyro bias in the Y direction in units of rad/s.

```C++
float gyb;
gyb = IMU.getGyroBiasY_rads();
```

**(optional) float getGyroBiasZ_rads()**
This function returns the current gyro bias in the Z direction in units of rad/s.

```C++
float gzb;
gzb = IMU.getGyroBiasZ_rads();
```

**(optional) void setGyroBiasX_rads(float bias)**
This function sets the gyro bias being used in the X direction to the input value in units of rad/s.

```C++
float gxb = 0.001; // gyro bias of 0.001 rad/s
IMU.setGyroBiasX_rads(gxb);
```

**(optional) void setGyroBiasY_rads(float bias)**
This function sets the gyro bias being used in the Y direction to the input value in units of rad/s.

```C++
float gyb = 0.001; // gyro bias of 0.001 rad/s
IMU.setGyroBiasY_rads(gyb);
```

**(optional) void setGyroBiasZ_rads(float bias)**
This function sets the gyro bias being used in the Z direction to the input value in units of rad/s.

```C++
float gzb = 0.001; // gyro bias of 0.001 rad/s
IMU.setGyroBiasZ_rads(gzb);
```

**(optional) int calibrateAccel()**
This function will estimate the bias and scale factor needed to calibrate the accelerometers. This function works one axis at a time and needs to be run for all 6 sensor orientations. After it has collected enough sensor data, it will estimate the bias and scale factor for all three accelerometer channels and apply these corrections to the measured data. Accelerometer calibration only needs to be performed once on the IMU, the get and set functions detailed below can be used to retrieve the estimated bias and scale factors and use them during future power cycles or operations with the IMU. This function returns a positive value on success and a negative value on failure.

```C++
status = IMU.calibrateAccel();
```

**(optional) float getAccelBiasX_mss()**
This function returns the current accelerometer bias in the X direction in units of m/s/s.

```C++
float axb;
axb = IMU.getAccelBiasX_mss();
```

**(optional) float getAccelScaleFactorX()**
This function returns the current accelerometer scale factor in the X direction.

```C++
float axs;
axs = IMU.getAccelScaleFactorX();
```

**(optional) float getAccelBiasY_mss()**
This function returns the current accelerometer bias in the Y direction in units of m/s/s.

```C++
float ayb;
ayb = IMU.getAccelBiasY_mss();
```

**(optional) float getAccelScaleFactorY()**
This function returns the current accelerometer scale factor in the Y direction.

```C++
float ays;
ays = IMU.getAccelScaleFactorY();
```

**(optional) float getAccelBiasZ_mss()**
This function returns the current accelerometer bias in the Z direction in units of m/s/s.

```C++
float azb;
azb = IMU.getAccelBiasZ_mss();
```

**(optional) float getAccelScaleFactorZ()**
This function returns the current accelerometer scale factor in the Z direction.

```C++
float azs;
azs = IMU.getAccelScaleFactorZ();
```

**(optional) void setAccelCalX(float bias,float scaleFactor)**
This function sets the accelerometer bias (m/s/s) and scale factor being used in the X direction to the input values.

```C++
float axb = 0.01; // accel bias of 0.01 m/s/s
float axs = 0.97; // accel scale factor of 0.97
IMU.setAccelCalX(axb,axs);
```

**(optional) void setAccelCalY(float bias,float scaleFactor)**
This function sets the accelerometer bias (m/s/s) and scale factor being used in the Y direction to the input values.

```C++
float ayb = 0.01; // accel bias of 0.01 m/s/s
float ays = 0.97; // accel scale factor of 0.97
IMU.setAccelCalY(ayb,ays);
```

**(optional) void setAccelCalZ(float bias,float scaleFactor)**
This function sets the accelerometer bias (m/s/s) and scale factor being used in the Z direction to the input values.

```C++
float azb = 0.01; // accel bias of 0.01 m/s/s
float azs = 0.97; // accel scale factor of 0.97
IMU.setAccelCalZ(azb,azs);
```

**(optional) int calibrateMag()**
This function will estimate the bias and scale factor needed to calibrate the magnetometers. This function works on all the sensor axes at once, you should continuously and slowly move the sensor in a figure 8 while the function is running. After it has collected enough sensor data, it will estimate the bias and scale factor for all three magnetometer channels and apply these corrections to the measured data. Magnetometer calibration only needs to be performed once on the IMU, unless the eletrical or magnetic environment changes. The get and set functions detailed below can be used to retrieve the estimated bias and scale factors and use them during future power cycles or operations with the IMU. This function returns a positive value on success and a negative value on failure.

```C++
status = IMU.calibrateMag();
```

**(optional) float getMagBiasX_uT()**
This function returns the current magnetometer bias in the X direction in units of uT.

```C++
float hxb;
hxb = IMU.getMagBiasX_uT();
```

**(optional) float getMagScaleFactorX()**
This function returns the current magnetometer scale factor in the X direction.

```C++
float hxs;
hxs = IMU.getMagScaleFactorX();
```

**(optional) float getMagBiasY_uT()**
This function returns the current magnetometer bias in the Y direction in units of uT.

```C++
float hyb;
hyb = IMU.getMagBiasY_uT();
```

**(optional) float getMagScaleFactorY()**
This function returns the current magnetometer scale factor in the Y direction.

```C++
float hys;
hys = IMU.getMagScaleFactorY();
```

**(optional) float getMagBiasZ_uT()**
This function returns the current magnetometer bias in the Z direction in units of uT.

```C++
float hzb;
hzb = IMU.getMagBiasZ_uT();
```

**(optional) float getMagScaleFactorZ()**
This function returns the current magnetometer scale factor in the Z direction.

```C++
float hzs;
hzs = IMU.getMagScaleFactorZ();
```

**(optional) void setMagCalX(float bias,float scaleFactor)**
This function sets the magnetometer bias (uT) and scale factor being used in the X direction to the input values.

```C++
float hxb = 10.0; // mag bias of 10 uT
float hxs = 0.97; // mag scale factor of 0.97
IMU.setMagCalX(hxb,hxs);
```

**(optional) void setMagCalY(float bias,float scaleFactor)**
This function sets the magnetometer bias (uT) and scale factor being used in the Y direction to the input values.

```C++
float hyb = 10.0; // mag bias of 10 uT
float hys = 0.97; // mag scale factor of 0.97
IMU.setMagCalY(hyb,hys);
```

**(optional) void setMagCalZ(float bias,float scaleFactor)**
This function sets the magnetometer bias (uT) and scale factor being used in the Z direction to the input values.

```C++
float hzb = 10.0; // mag bias of 10 uT
float hzs = 0.97; // mag scale factor of 0.97
IMU.setMagCalZ(hzb,hzs);
```

#### Wake on Motion Setup

**(optional) int enableWakeOnMotion(float womThresh_mg,LpAccelOdr odr)**
This function enables the MPU-9250 wake on motion interrupt functionality. It places the MPU-9250 into a low power state, with the MPU-9250 waking up at an interval determined by the Low Power Accelerometer Output Data Rate. If the accelerometer detects motion in excess of the threshold given, it generates a 50us pulse from the MPU-9250 INT pin. The following enumerated Low Power Accelerometer Output Data Rates are supported:

| LpAccelOdr Name      | Output Data Rate |
| ------------------   | ---------------- |
| LP_ACCEL_ODR_0_24HZ  | 0.24 Hz          |
| LP_ACCEL_ODR_0_49HZ  | 0.49 Hz          |
| LP_ACCEL_ODR_0_98HZ  | 0.98 Hz          |
| LP_ACCEL_ODR_1_95HZ  | 1.95 Hz          | 
| LP_ACCEL_ODR_3_91HZ  | 3.91 Hz          |
| LP_ACCEL_ODR_7_81HZ  | 7.81 Hz          |
| LP_ACCEL_ODR_15_63HZ | 15.63 Hz         |
| LP_ACCEL_ODR_31_25HZ | 31.25 Hz         |
| LP_ACCEL_ODR_62_50HZ | 62.50 Hz         |
| LP_ACCEL_ODR_125HZ   | 125 Hz           |
| LP_ACCEL_ODR_250HZ   | 250 Hz           |
| LP_ACCEL_ODR_500HZ   | 500 Hz           |

The motion threshold is given as a float value between 0 and 1020 mg mapped, which is internally mapped to a single byte, 0-255 value. This function returns a positive value on success and a negative value on failure. Please see the *WOM_I2C example*. The following is an example of enabling the wake on motion with a 400 mg threshold and a ODR of 31.25 Hz.

```C++
status = IMU.enableWakeOnMotion(400,MPU9250::LP_ACCEL_ODR_31_25HZ);
```

### Common Data Collection Functions
The functions below are used to collect data from the MPU-9250 sensor. Data is returned scaled to engineering units and transformed to a [common axis system](#sensor-orientation).

#### Real-Time Data Collection
**int readSensor()** reads the sensor and stores the newest data in a buffer, it should be called every time you would like to retrieve data from the sensor. This function returns a positive value on success and a negative value on failure.

```C++
IMU.readSensor();
```

**float getAccelX_mss()** gets the accelerometer value from the data buffer in the X direction and returns it in units of m/s/s.

```C++
float ax;
ax = IMU.getAccelX_mss();
```

**float getAccelY_mss()** gets the accelerometer value from the data buffer in the Y direction and returns it in units of m/s/s.

```C++
float ay;
ay = IMU.getAccelY_mss();
```

**float getAccelZ_mss()** gets the accelerometer value from the data buffer in the Z direction and returns it in units of m/s/s.

```C++
float az;
az = IMU.getAccelZ_mss();
```

**float getGyroX_rads()** gets the gyroscope value from the data buffer in the X direction and returns it in units of rad/s.

```C++
float gx;
gx = IMU.getGyroX_rads();
```

**float getGyroY_rads()** gets the gyroscope value from the data buffer in the Y direction and returns it in units of rad/s.

```C++
float gy;
gy = IMU.getGyroY_rads();
```

**float getGyroZ_rads()** gets the gyroscope value from the data buffer in the Z direction and returns it in units of rad/s.

```C++
float gz;
gz = IMU.getGyroZ_rads();
```

**float getMagX_uT()** gets the magnetometer value from the data buffer in the X direction and returns it in units of uT.

```C++
float hx;
hx = IMU.getMagX_uT();
```

**float getMagY_uT()** gets the magnetometer value from the data buffer in the Y direction and returns it in units of uT.

```C++
float hy;
hy = IMU.getMagY_uT();
```

**float getMagZ_uT()** gets the magnetometer value from the data buffer in the Z direction and returns it in units of uT.

```C++
float hz;
hz = IMU.getMagZ_uT();
```

**float getTemperature_C()** gets the die temperature value from the data buffer and returns it in units of C.

```C++
float temperature;
temperature = IMU.getTemperature_C();
```

## MPU9250FIFO Class
The *MPU9250FIFO* derived class extends the functionality provided by the *MPU9250* base class by providing support for setting up and reading the MPU-9250 FIFO buffer. All of the functions described above, as part of the *MPU9250* class are also available to the *MPU9250FIFO* class. 

### I2C Object Declaration

**MPU9250FIFO(TwoWire &bus,uint8_t address)**
An MPU9250FIFO object should be declared, specifying the I2C bus and MPU-9250 I2C address. The MPU-9250 I2C address will be 0x68 if the AD0 pin is grounded or 0x69 if the AD0 pin is pulled high. For example, the following code declares an MPU9250FIFO object called *IMU* with an MPU-9250 sensor located on I2C bus 0 with a sensor address of 0x68 (AD0 grounded).

```C++
MPU9250FIFO IMU(Wire,0x68);
```

### SPI Object Declaratioon

**MPU9250FIFO(SPIClass &bus,uint8_t csPin)**
An MPU9250FIFO object should be declared, specifying the SPI bus and chip select pin used. Multiple MPU-9250 or other SPI objects could be used on the same SPI bus, each with their own chip select pin. The chip select pin can be any available digital pin. For example, the following code declares an MPU9250FIFO object called *IMU* with an MPU-9250 sensor located on SPI bus 0 with chip select pin 10.

```C++
MPU9250FIFO IMU(SPI,10);
```

### FIFO Setup
**(optional) int enableFifo(bool accel,bool gyro,bool mag,bool temp)**
This function configures and enables the MPU-9250 FIFO buffer. This 512 byte buffer samples data at the data output rate set by the SRD and enables the microcontroller to bulk read the data, reducing microcontroller workload for certain applications. It is configured with a set of boolean values describing which data to buffer in the FIFO: accelerometer, gyroscope, magnetometer, or temperature. The accelerometer and gyroscope data each take 6 bytes of space per sample while the magnetometer takes 7 bytes of space and the temperature 2 bytes. It's important to select only the data sources desired to ensure that the FIFO does not overrun between reading it. For example, enabling all of the data sources would take 21 bytes per sample allowing the FIFO to hold only 24 samples before overflowing. If only the accelerometer data is needed, this increases to 85 samples before overflowing. This function returns a positive value on success and a negative value on failure. Please see the *FIFO_SPI example*. The following is an example of enabling the FIFO to buffer accelerometer and gyroscope data. 

```C++
status = IMU.enableFifo(true,true,false,false);
```

### FIFO Data Collection
**int readFifo()** reads the FIFO buffer from the MPU-9250, parses it and stores the data in buffers on the microcontroller. It should be called every time you would like to retrieve data from the FIFO buffer. This function returns a positive value on success and a negative value on failure.

```C++
IMU.readFifo();
```

**void getFifoAccelX_mss(size_t *size,float* data)** gets the accelerometer value from the data buffer in the X direction and returns it in units of m/s/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C++
float ax[100];
size_t samples;
IMU.getFifoAccelX_mss(&samples,ax);
```

**void getFifoAccelY_mss(size_t *size,float* data)** gets the accelerometer value from the data buffer in the Y direction and returns it in units of m/s/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C++
float ay[100];
size_t samples;
IMU.getFifoAccelY_mss(&samples,ay);
```

**void getFifoAccelZ_mss(size_t *size,float* data)** gets the accelerometer value from the data buffer in the Z direction and returns it in units of m/s/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C++
float az[100];
size_t samples;
IMU.getFifoAccelZ_mss(&samples,az);
```

**void getFifoGyroX_rads(size_t *size,float* data)** gets the gyroscope value from the data buffer in the X direction and returns it in units of rad/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C++
float gx[100];
size_t samples;
IMU.getFifoGyroX_rads(&samples,gx);
```

**void getFifoGyroY_rads(size_t *size,float* data)** gets the gyroscope value from the data buffer in the Y direction and returns it in units of rad/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C++
float gy[100];
size_t samples;
IMU.getFifoGyroY_rads(&samples,gy);
```

**void getFifoGyroZ_rads(size_t *size,float* data)** gets the gyroscope value from the data buffer in the Z direction and returns it in units of rad/s. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C++
float gz[100];
size_t samples;
IMU.getFifoGyroZ_rads(&samples,gx);
```

**void getFifoMagX_uT(size_t *size,float* data)** gets the magnetometer value from the data buffer in the X direction and returns it in units of uT. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C++
float hx[100];
size_t samples;
IMU.getFifoMagX_uT(&samples,hx);
```

**void getFifoMagY_uT(size_t *size,float* data)** gets the magnetometer value from the data buffer in the Y direction and returns it in units of uT. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C++
float hy[100];
size_t samples;
IMU.getFifoMagY_uT(&samples,hy);
```

**void getFifoMagZ_uT(size_t *size,float* data)** gets the magnetometer value from the data buffer in the Z direction and returns it in units of uT. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C++
float hz[100];
size_t samples;
IMU.getFifoMagZ_uT(&samples,hz);
```

**void getFifoTemperature_C(size_t *size,float* data)** gets the die temperature value from the data buffer and returns it in units of C. The data is returned as an array along with the number of elements within that array. Ensure that the buffer you are transfering to has enough capacity to store the data.

```C++
float temp[100];
size_t samples;
IMU.getFifoTemperature_C(&samples,temp);
```

## <a name="sensor-orientation"></a>Sensor Orientation
This library transforms all data to a common axis system before it is returned. This axis system is shown below. It is a right handed coordinate system with the z-axis positive down, common in aircraft dynamics.

<img src="https://github.com/bolderflight/MPU9250/blob/master/extras/MPU-9250-AXIS.png" alt="Common Axis System" width="250">

**Caution!** This axis system is shown relative to the MPU-9250 sensor. The sensor may be rotated relative to the breakout board. 

## Example List

* **Basic_I2C**: demonstrates declaring an *MPU9250* object, initializing the sensor, and collecting data. I2C is used to communicate with the MPU-9250 sensor.
* **Basic_SPI**: demonstrates declaring an *MPU9250* object, initializing the sensor, and collecting data. SPI is used to communicate with the MPU-9250 sensor.
* **Advanced_I2C**: demonstrates a more advanced setup. In this case, the accelerometer and gyroscope full scale ranges, DLPF, and SRD are set to non-default values. I2C is used to communicate with the MPU-9250 sensor.
* **Interrupt_SPI**: demonstrates having the MPU-9250 sensor create an interrupt pulse when data is ready, which is used to drive data collection at the specified rate. SPI is used to communicate with the MPU-9250 sensor.
* **WOM_I2C**: demonstrates setting up and using the wake on motion interrupt. I2C is used to communicate with the MPU-9250 sensor.
* **FIFO_SPI**: demonstrates setting up and using the FIFO buffer. SPI is used to communicate with the MPU-9250 sensor.

# Wiring and Pullups 

Please refer to the [MPU-9250 datasheet](https://github.com/bolderflight/MPU9250/blob/master/docs/MPU-9250-Datasheet.pdf) and your microcontroller's pinout diagram. This library was developed using the [Embedded Masters breakout board](https://store.invensense.com/Controls/www.embeddedmasters.com/ProductDetail/EMSENSRMPU9250-Embedded-Masters/552444/) v1.1 for the MPU-9250. The data sheet for this breakout board is located [here](https://github.com/bolderflight/MPU9250/blob/master/docs/Embedded-Masters-MPU-9250-Breakout.pdf). This library should work well for other breakout boards or embedded sensors, please refer to your vendor's pinout diagram.

## I2C

The MPU-9250 pins should be connected as:
   * VDD: this should be a 2.4V to 3.6V power source.
   * GND: ground.
   * VDDI: digital I/O supply voltage. This should be between 1.71V and VDD.
   * FSYNC: not used, should be grounded.
   * INT: (optional) used for the interrupt output setup in *enableDataReadyInterrupt* and *enableWakeOnMotion*. Connect to interruptable pin on microcontroller.
   * SDA / SDI: connect to SDA.
   * SCL / SCLK: connect to SCL.
   * AD0 / SDO: ground to select I2C address 0x68. Pull high to VDD to select I2C address 0x69.
   * nCS: no connect.
   * AUXDA: not used.
   * AUXCL: not used.

4.7 kOhm resistors should be used as pullups on SDA and SCL, these resistors should pullup with a 3.3V source.

## SPI

The MPU-9250 pins should be connected as:
   * VDD: this should be a 2.4V to 3.6V power source.
   * GND: ground.
   * VDDI: digital I/O supply voltage. This should be between 1.71V and VDD.
   * FSYNC: not used, should be grounded.
   * INT: (optional) used for the interrupt output setup in *enableDataReadyInterrupt* and *enableWakeOnMotion*. Connect to interruptable pin on microcontroller.
   * SDA / SDI: connect to MOSI.
   * SCL / SCLK: connect to SCK.
   * AD0 / SDO: connect to MISO.
   * nCS: connect to chip select pin. Pin 10 was used in the code snippets in this document and the included examples, but any digital I/O pin can be used. 
   * AUXDA: not used.
   * AUXCL: not used.

Some breakout boards, including the Embedded Masters breakout board, require slight modification to enable SPI. Please refer to your vendor's documentation.
