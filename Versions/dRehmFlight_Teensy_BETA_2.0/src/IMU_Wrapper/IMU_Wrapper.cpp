//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Additional Author: Brian Jones
//Project Start: 1/6/2020
//Last Updated: 8/11/2025
//Version: Beta 2.0

//========================================================================================================================//

// This is a wrapper for IMU chips supported by dRehmFlight.  It consolidates all the IMU handling and PID subrutines into a single library that most people won't need to mess with.

#include "IMU_Wrapper.h"

IMU_Wrapper::IMU_Wrapper(IMUType type): IMU_Wrapper(type, GYRO_250DPS, ACCEL_2G) {};

IMU_Wrapper::IMU_Wrapper(IMUType type, Gyro_DPS gyroDPS, Accel_G accelG) {
    _mpu6050 = nullptr;
    _mpu9250 = nullptr;
    _lsm6dsox = nullptr;

    _type = type;
    _gyroDPS = gyroDPS;
    _accelG = accelG;
    
    //Setup gyro and accel full scale value selection and scale factor
    switch (_gyroDPS) {
      case GYRO_250DPS:
        _gyro_scale_factor = 131.0;
        break;
      case GYRO_500DPS:
        _gyro_scale_factor = 65.5;
        break;
      case GYRO_1000DPS:
        _gyro_scale_factor = 32.8;
        break;
      case GYRO_2000DPS:
        _gyro_scale_factor = 16.4;
        break;
    }

    switch (_accelG) {
      case ACCEL_2G:
        _accel_scale_factor = 16384.0;
        break;
      case ACCEL_4G:
        _accel_scale_factor = 8192.0;
        break;
      case ACCEL_8G:
        _accel_scale_factor = 4096.0;
        break;
      case ACCEL_16G:
        _accel_scale_factor = 2048.0;
        break;
    }
}
    

bool IMU_Wrapper::begin() {
  switch (_type) {
    case MPU6050_I2C:
    {
      Wire.begin();
      Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...

      _mpu6050 = new MPU6050();
      _mpu6050->initialize();
            
      if (_mpu6050->testConnection() == false) {
        Serial.print("DeviceID: ");
        Serial.println(_mpu6050->getDeviceID());
        Serial.println("MPU6050 initialization unsuccessful");
        Serial.println("Check MPU6050 wiring or try cycling power");
        return false;
      }
      else {
        Serial.print("OK!  DeviceID: ");
        Serial.println(_mpu6050->getDeviceID());
        Serial.println("MPU6050 initialization successful");
      }

      //From the reset state all registers should be 0x00, so we should be at
      //max sample rate with digital low pass filter(s) off.  All we need to
      //do is set the desired fullscale ranges
      switch (_gyroDPS) {
        case GYRO_250DPS:
          _mpu6050->setFullScaleGyroRange(MPU6050_GYRO_FS_250);
          break;
        case GYRO_500DPS:
          _mpu6050->setFullScaleGyroRange(MPU6050_GYRO_FS_500);
          break;
        case GYRO_1000DPS:
          _mpu6050->setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
          break;
        case GYRO_2000DPS:
          _mpu6050->setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
          break;
      }
      switch (_accelG) {
        case ACCEL_2G:
          _mpu6050->setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
          break;
        case ACCEL_4G:
          _mpu6050->setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
          break;
        case ACCEL_8G:
          _mpu6050->setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
          break;
        case ACCEL_16G:
          _mpu6050->setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
          break;
      }
      return true;
    }

    case MPU9250_SPI:
    {
      _mpu9250 = new MPU9250(SPI2,36);
      int status = _mpu9250->begin();    

      if (status < 0) {
        Serial.println("MPU9250 initialization unsuccessful");
        Serial.println("Check MPU9250 wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        return false;
      }
      else {
        Serial.println("MPU9250 initialization successful");
      }

      //From the reset state all registers should be 0x00, so we should be at
      //max sample rate with digital low pass filter(s) off.  All we need to
      //do is set the desired fullscale ranges
      switch (_gyroDPS) {
        case GYRO_250DPS:
          _mpu9250->setGyroRange(_mpu9250->GYRO_RANGE_250DPS);
          break;
        case GYRO_500DPS:
          _mpu9250->setGyroRange(_mpu9250->GYRO_RANGE_500DPS);
          break;
        case GYRO_1000DPS:
          _mpu9250->setGyroRange(_mpu9250->GYRO_RANGE_1000DPS);
          break;
        case GYRO_2000DPS:
          _mpu9250->setGyroRange(_mpu9250->GYRO_RANGE_2000DPS);
          break;
        }
        switch (_accelG) {
          case ACCEL_2G:
            _mpu9250->setAccelRange(_mpu9250->ACCEL_RANGE_2G);
            break;
          case ACCEL_4G:
            _mpu9250->setAccelRange(_mpu9250->ACCEL_RANGE_4G);
            break;
          case ACCEL_8G:
            _mpu9250->setAccelRange(_mpu9250->ACCEL_RANGE_8G);
            break;
          case ACCEL_16G:
            _mpu9250->setAccelRange(_mpu9250->ACCEL_RANGE_16G);
            break;
        }

        _mpu9250->setMagCalX(_MagErrorX, _MagScaleX);
        _mpu9250->setMagCalY(_MagErrorY, _MagScaleY);
        _mpu9250->setMagCalZ(_MagErrorZ, _MagScaleZ);
        _mpu9250->setSrd(0); //sets gyro and accel read to 1khz, magnetometer read to 100hz
        return true;
    }
    
    case LSM6DSOX_SPI:
    {
      _lsm6dsox = new Adafruit_LSM6DSOX();
      if (!_lsm6dsox->begin_SPI(_cs_pin, _sck_pin, _miso_pin, _mosi_pin)) {
        Serial.println("Failed to find LSM6DSOX chip");
        Serial.println("LSM6DSOX initialization unsuccessful");
        Serial.println("Check LSM6DSOX wiring or try cycling power");
        return false;
      }
      else {
        Serial.println("LSM6DSOX initialization successful");
      }
  
      switch (_gyroDPS) {
        case GYRO_250DPS:
          _lsm6dsox->setGyroRange(lsm6ds_gyro_range_t::LSM6DS_GYRO_RANGE_250_DPS);
          break;
        case GYRO_500DPS:
            _lsm6dsox->setGyroRange(lsm6ds_gyro_range_t::LSM6DS_GYRO_RANGE_500_DPS);
            break;
          case GYRO_1000DPS:
            _lsm6dsox->setGyroRange(lsm6ds_gyro_range_t::LSM6DS_GYRO_RANGE_1000_DPS);
            break;
          case GYRO_2000DPS:
            _lsm6dsox->setGyroRange(lsm6ds_gyro_range_t::LSM6DS_GYRO_RANGE_2000_DPS);
            break;
        }
        switch (_accelG) {
          case ACCEL_2G:
            _lsm6dsox->setAccelRange(lsm6ds_accel_range_t::LSM6DS_ACCEL_RANGE_2_G);
            break;
          case ACCEL_4G:
            _lsm6dsox->setAccelRange(lsm6ds_accel_range_t::LSM6DS_ACCEL_RANGE_4_G);
            break;
          case ACCEL_8G:
            _lsm6dsox->setAccelRange(lsm6ds_accel_range_t::LSM6DS_ACCEL_RANGE_8_G);
            break;
          case ACCEL_16G:
            _lsm6dsox->setAccelRange(lsm6ds_accel_range_t::LSM6DS_ACCEL_RANGE_16_G);
            break;
        }
        return true;
      }

      default:
        return false;
    }
}


// Sets the Class SPI Pins
void IMU_Wrapper::setSPIpins(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin, int8_t mosi_pin) {
  _cs_pin = cs_pin;
	_sck_pin = sck_pin;
	_miso_pin = miso_pin;
	_mosi_pin = mosi_pin;
}


void IMU_Wrapper::getIMUdata() {
  //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
  /*
   * Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ. 
   * These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. A simple first-order
   * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut
   * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
   * the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally,
   * the constant errors found in calculate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.
   * 
   * Notes from BrianJones:
   * IN STANDARD MOUNTING FORM:
   * MPU6060
   * Gyro
   *    Roll RIGHT shows + X activity
   *    Pitch FORWARD shows + Y activity
   *    Yaw LEFT shows + Z activity
   * Accel
   *    Roll RIGHT shows +1 G on Y
   *    Pitch BACKWARD shows +1G on X
   *    +Z is normal gravity
   * 
   * The LSMDSOX has the X and Y axis swapped, and the Y axis inverted.  This is handled below to make it work like the original MPU6050 code from Nick
   * I don't have a MPU9250 to test with.  I've kept Nick's code unchanged. 
   * 
   */
  
  int16_t AcX = 0;
  int16_t AcY = 0;
  int16_t AcZ = 0;
  int16_t GyX = 0;
  int16_t GyY = 0;
  int16_t GyZ = 0;
  int16_t MgX = 0;
  int16_t MgY = 0;
  int16_t MgZ = 0;

  if (_type == MPU6050_I2C) {
    _mpu6050->getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
  }
  else if (_type == MPU9250_SPI) {
    _mpu9250->getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
  }
  else if (_type == LSM6DSOX_SPI) {
    _lsm6dsox->getEvent(&accel, &gyro, &temp);

    // THe LSM6DSOX has the X and Y reversed from the MPU6050 as well has having the X/Y accelerometers reversed and Y gyro reversed    
    AccX = accel.acceleration.y * MS2G;  // Convert m/s to G-factor
    AccY = accel.acceleration.x * -MS2G;
    AccZ = accel.acceleration.z * MS2G;
    GyroX = gyro.gyro.y * RAD2DEG;  // convert rad/s back to deg/sec
    GyroY = gyro.gyro.x * -RAD2DEG;
    GyroZ = gyro.gyro.z * RAD2DEG;
  }
  else {
    // should throw an error probably
  }
  
  // The MPU6050 and MPU9250 IMUs have a different scale factor from the LSM6DSOX 
  if (_type == MPU6050_I2C || _type == MPU9250_SPI) {
    //Accelerometer
    AccX = AcX / _accel_scale_factor; //G's
    AccY = AcY / _accel_scale_factor;
    AccZ = AcZ / _accel_scale_factor;

    //Gyro
    GyroX = GyX / _gyro_scale_factor; //deg/sec
    GyroY = GyY / _gyro_scale_factor;
    GyroZ = GyZ / _gyro_scale_factor;

    //Magnetometer
    MagX = MgX/6.0; //uT
    MagY = MgY/6.0;
    MagZ = MgZ/6.0;
  }

  //Correct the outputs with the calculated error values
  AccX = AccX - _AccErrorX;
  AccY = AccY - _AccErrorY;
  AccZ = AccZ - _AccErrorZ;
  //LP filter accelerometer data
  AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
  AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
  AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  //Correct the outputs with the calculated error values
  GyroX = GyroX - _GyroErrorX;
  GyroY = GyroY - _GyroErrorY;
  GyroZ = GyroZ - _GyroErrorZ;
  //LP filter gyro data
  GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
  GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
  GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;

  //Correct the outputs with the calculated error values
  MagX = (MagX - _MagErrorX)*_MagScaleX;
  MagY = (MagY - _MagErrorY)*_MagScaleY;
  MagZ = (MagZ - _MagErrorZ)*_MagScaleZ;
  //LP filter magnetometer data
  MagX = (1.0 - B_mag)*MagX_prev + B_mag*MagX;
  MagY = (1.0 - B_mag)*MagY_prev + B_mag*MagY;
  MagZ = (1.0 - B_mag)*MagZ_prev + B_mag*MagZ;
  MagX_prev = MagX;
  MagY_prev = MagY;
  MagZ_prev = MagZ;
}


void IMU_Wrapper::calculate_IMU_error() {
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
   * measurement. 
   */
  int16_t AcX = 0;
  int16_t AcY = 0;
  int16_t AcZ = 0;
  int16_t GyX = 0;
  int16_t GyY = 0;
  int16_t GyZ = 0;
  int16_t MgX = 0;
  int16_t MgY = 0;
  int16_t MgZ = 0;
  
  _AccErrorX = 0.0;
  _AccErrorY = 0.0;
  _AccErrorZ = 0.0;
  _GyroErrorX = 0.0;
  _GyroErrorY= 0.0;
  _GyroErrorZ = 0.0;

  Serial.println("Beginning gyro/accel calibration...");
  delay(5000);    // Some IMUs (like the LSM6DSOX) take a bit to settle.  I get way different readings for about the first 5-10s of power on.
  
  //Read IMU values 12000 times
  int c = 0;
  while (c < 12000) {
    if (_type == MPU6050_I2C) {
      _mpu6050->getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    }
    else if (_type == MPU9250_SPI) {
      _mpu9250->getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
    }
    else if (_type == LSM6DSOX_SPI) {
      _lsm6dsox->getEvent(&accel, &gyro, &temp);

      // THe LSM6DSOX has the X and Y reversed from the MPU6050 as well has having Y gyro flipped
      AccX = accel.acceleration.y * MS2G;  // Convert m/s to G-factor
      AccY = accel.acceleration.x * -MS2G;
      AccZ = accel.acceleration.z * MS2G;
      GyroX = gyro.gyro.y * RAD2DEG;  // convert rad/s back to deg/sec
      GyroY = gyro.gyro.x * -RAD2DEG;
      GyroZ = gyro.gyro.z * RAD2DEG;
    }
    else {
      // should throw an error probably
    }
    
    // The MPU6050 and MPU9250 IMUs have a different scale factor from the LSM6DSOX 
    if (_type == MPU6050_I2C || _type == MPU9250_SPI) {
      //Accelerometer
      AccX = AcX / _accel_scale_factor; //G's
      AccY = AcY / _accel_scale_factor;
      AccZ = AcZ / _accel_scale_factor;

      //Gyro
      GyroX = GyX / _gyro_scale_factor; //deg/sec
      GyroY = GyY / _gyro_scale_factor;
      GyroZ = GyZ / _gyro_scale_factor;
    }

    //Sum all readings
    _AccErrorX  = _AccErrorX + AccX;
    _AccErrorY  = _AccErrorY + AccY;
    _AccErrorZ  = _AccErrorZ + AccZ;
    _GyroErrorX = _GyroErrorX + GyroX;
    _GyroErrorY = _GyroErrorY + GyroY;
    _GyroErrorZ = _GyroErrorZ + GyroZ;
    c++;
  }

  //Divide the sum by 12000 to get the error value
  _AccErrorX  = _AccErrorX / c;
  _AccErrorY  = _AccErrorY / c;
  _AccErrorZ  = _AccErrorZ / c - 1.0;
  _GyroErrorX = _GyroErrorX / c;
  _GyroErrorY = _GyroErrorY / c;
  _GyroErrorZ = _GyroErrorZ / c;

  Serial.println("Paste these lines in void setup() and comment out imu.calculate_IMU_error().");
  Serial.print("imu.setAccError(");
  Serial.print(_AccErrorX);
  Serial.print(", ");
  Serial.print(_AccErrorY);
  Serial.print(", ");
  Serial.print(_AccErrorZ);
  Serial.println(");");

  Serial.print("imu.setGyroError(");
  Serial.print(_GyroErrorX);
  Serial.print(", ");
  Serial.print(_GyroErrorY);
  Serial.print(", ");
  Serial.print(_GyroErrorZ);
  Serial.println(");");

  while(1);
}


void IMU_Wrapper::calibrateMagnetometer() {
  if (_type == MPU9250_SPI) {
    float success;
    Serial.println("Beginning magnetometer calibration in");
    Serial.println("3...");
    delay(1000);
    Serial.println("2...");
    delay(1000);
    Serial.println("1...");
    delay(1000);
    Serial.println("Rotate the IMU about all axes until complete.");
    Serial.println(" ");
    success = _mpu9250->calibrateMag();
    if(success) {
      Serial.println("Calibration Successful!");
      Serial.println("Please comment out the calibrateMagnetometer() function and copy these lines into the code:");
      Serial.print("myIMU.setMagError(");
      Serial.print(_mpu9250->getMagBiasX_uT());
      Serial.print(", ");
      Serial.print(_mpu9250->getMagBiasY_uT());
      Serial.print(", ");
      Serial.print(_mpu9250->getMagBiasZ_uT());
      Serial.println(");");
      Serial.print("myIMU.setMagScale(");
      Serial.print(_mpu9250->getMagScaleFactorX());
      Serial.print(", ");
      Serial.print(_mpu9250->getMagScaleFactorY());
      Serial.print(", ");
      Serial.print(_mpu9250->getMagScaleFactorZ());
      Serial.println(");");
      Serial.println(" ");
      Serial.println("If you are having trouble with your attitude estimate at a new flying location, repeat this process as needed.");
    }
    else {
      Serial.println("Calibration Unsuccessful. Please reset the board and try again.");
      while(1); //Halt code so it won't enter main loop until this function commented out
    }
  }
  else {
    Serial.println("Error: MPU9250 not selected. Cannot calibrate non-existent magnetometer.");
    while(1); //Halt code so it won't enter main loop until this function commented out
  }
}


void IMU_Wrapper::Madgwick(float invSampleFreq) {
	float gx = GyroX;
	float gy = -GyroY;
	float gz = -GyroZ;
	float ax = -AccX;
	float ay = AccY;
	float az = AccZ;
	float mx = MagY;
	float my = -MagX;
	float mz = MagZ;

  //DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
  /*
   * This function fuses the accelerometer gyro, and magnetometer readings AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, and MagZ for attitude estimation.
   * Don't worry about the math. There is a tunable parameter B_madgwick in the user specified variable section which basically
   * adjusts the weight of gyro data in the state estimate. Higher beta leads to noisier estimate, lower 
   * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the roll_IMU,
   * pitch_IMU, and yaw_IMU variables which are in degrees. If magnetometer data is not available, this function calls Madgwick6DOF() instead.
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  //use 6DOF algorithm if MPU6050 is being used
  #if defined USE_MPU6050_I2C 
    Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;
  #endif
  
  //Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;
  }

  //Convert gyroscope degrees/sec to radians/sec
  gx *= DEG2RAD;
  gy *= DEG2RAD;
  gz *= DEG2RAD;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    //Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    //Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalize quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  
  //compute angles - NWU
  roll_IMU  = atan2 (q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)                  *57.29577951; //degrees
  pitch_IMU = -asin (constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
  yaw_IMU   = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)                  *57.29577951; //degrees
}

void IMU_Wrapper::Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= DEG2RAD;
  gy *= DEG2RAD;
  gz *= DEG2RAD;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  roll_IMU  = atan2 (q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)                  *57.29577951; //degrees
  pitch_IMU = -asin (constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
  yaw_IMU   = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)                  *57.29577951; //degrees
}


void IMU_Wrapper::controlANGLE(float roll_des, float pitch_des, float yaw_des, float dt, unsigned long throttle) {
  //DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
   * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */
  
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (throttle < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX;
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (throttle < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (throttle < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}


void IMU_Wrapper::controlANGLE2(float roll_des, float pitch_des, float yaw_des, float dt, unsigned long throttle) {
  //DESCRIPTION: Computes control commands based on state error (angle) in cascaded scheme
  /*
   * Gives better performance than controlANGLE() but requires much more tuning. Not reccommended for first-time setup.
   * See the documentation for tuning this controller.
   */
  //Outer loop - PID on angle
  float roll_des_ol, pitch_des_ol;
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll_ol = integral_roll_prev_ol + error_roll*dt;
  if (throttle < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll_ol = 0;
  }
  integral_roll_ol = constrain(integral_roll_ol, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (roll_IMU - roll_IMU_prev)/dt; 
  roll_des_ol = Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll_ol;// - Kd_roll_angle*derivative_roll;

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch_ol = integral_pitch_prev_ol + error_pitch*dt;
  if (throttle < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch_ol = 0;
  }
  integral_pitch_ol = constrain(integral_pitch_ol, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_pitch = (pitch_IMU - pitch_IMU_prev)/dt;
  pitch_des_ol = Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch_ol;// - Kd_pitch_angle*derivative_pitch;

  //Apply loop gain, constrain, and LP filter for artificial damping
  float Kl = 30.0;
  roll_des_ol = Kl*roll_des_ol;
  pitch_des_ol = Kl*pitch_des_ol;
  roll_des_ol = constrain(roll_des_ol, -240.0, 240.0);
  pitch_des_ol = constrain(pitch_des_ol, -240.0, 240.0);
  roll_des_ol = (1.0 - B_loop_roll)*roll_des_prev + B_loop_roll*roll_des_ol;
  pitch_des_ol = (1.0 - B_loop_pitch)*pitch_des_prev + B_loop_pitch*pitch_des_ol;

  //Inner loop - PID on rate
  //Roll
  error_roll = roll_des_ol - GyroX;
  integral_roll_il = integral_roll_prev_il + error_roll*dt;
  if (throttle < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll_il = 0;
  }
  integral_roll_il = constrain(integral_roll_il, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev)/dt; 
  roll_PID = 0.01*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll_il + Kd_roll_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des_ol - GyroY;
  integral_pitch_il = integral_pitch_prev_il + error_pitch*dt;
  if (throttle < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch_il = 0;
  }
  integral_pitch_il = constrain(integral_pitch_il, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  pitch_PID = 0.01*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch_il + Kd_pitch_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range
  
  //Yaw
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (throttle < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = 0.01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range
  
  //Update roll variables
  integral_roll_prev_ol = integral_roll_ol;
  integral_roll_prev_il = integral_roll_il;
  error_roll_prev = error_roll;
  roll_IMU_prev = roll_IMU;
  roll_des_prev = roll_des_ol;
  //Update pitch variables
  integral_pitch_prev_ol = integral_pitch_ol;
  integral_pitch_prev_il = integral_pitch_il;
  error_pitch_prev = error_pitch;
  pitch_IMU_prev = pitch_IMU;
  pitch_des_prev = pitch_des_ol;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}


void IMU_Wrapper::controlRATE(float roll_des, float pitch_des, float yaw_des, float dt, unsigned long throttle) {
  //DESCRIPTION: Computes control commands based on state error (rate)
  /*
   * See explanation for controlANGLE(). Everything is the same here except the error is now the desired rate - raw gyro reading.
   */
  //Roll
  error_roll = roll_des - GyroX;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (throttle < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev)/dt; 
  roll_PID = .01*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll + Kd_roll_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (throttle < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  pitch_PID = .01*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch + Kd_pitch_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (throttle < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  error_roll_prev = error_roll;
  integral_roll_prev = integral_roll;
  GyroX_prev = GyroX;
  //Update pitch variables
  error_pitch_prev = error_pitch;
  integral_pitch_prev = integral_pitch;
  GyroY_prev = GyroY;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}


// Functions to return individial Gyro, Accel, and Mag values.
float IMU_Wrapper::getGyro(char dir) {
	if      (dir == 'X') return GyroX;
  else if (dir == 'Y') return GyroY;
	else if (dir == 'Z') return GyroZ;
	else                 return 0.0;
}

float IMU_Wrapper::getAccel(char dir) {
	if      (dir == 'X') return AccX;
	else if (dir == 'Y') return AccY;
  else if (dir == 'Z') return AccZ;
	else                 return 0.0;
}

float IMU_Wrapper::getMag(char dir) {
	if      (dir == 'X') return MagX;
	else if (dir == 'Y') return MagY;
	else if (dir == 'Z') return MagZ;
	else                 return 0.0;
}


// Functions to set the error correction values for each sensor:  Gyro, Accel, and Mag

void IMU_Wrapper::setGyroError(float GyroErrorX, float GyroErrorY, float GyroErrorZ) {
  _GyroErrorX = GyroErrorX;
	_GyroErrorY = GyroErrorY;
	_GyroErrorZ = GyroErrorZ;
}

void IMU_Wrapper::setAccError(float AccErrorX, float AccErrorY, float AccErrorZ) {
  _AccErrorX = AccErrorX;
  _AccErrorY = AccErrorY;
	_AccErrorZ = AccErrorZ;
}

void IMU_Wrapper::setMagError(float MagErrorX, float MagErrorY, float MagErrorZ) {
  _MagErrorX = MagErrorX;
	_MagErrorY = MagErrorY;
	_MagErrorZ = MagErrorZ;
}

void IMU_Wrapper::setMagScale(float MagScaleX, float MagScaleY, float MagScaleZ) {
  _MagScaleX = MagScaleX;
	_MagScaleY = MagScaleY;
	_MagScaleZ = MagScaleZ;
}


// Functions to set the PID values for each of the three axis (roll, pitch, yaw) for
// Angle mode and Rate mode. (Yaw only has rate mode)
void IMU_Wrapper::setRollAnglePID(float Kp, float Ki, float Kd) {
    Kp_roll_angle = Kp;
    Ki_roll_angle = Ki;
    Kd_roll_angle = Kd;
}

void IMU_Wrapper::setPitchAnglePID(float Kp, float Ki, float Kd) {
    Kp_pitch_angle = Kp;
    Ki_pitch_angle = Ki;
    Kd_pitch_angle = Kd;
}
	
void IMU_Wrapper::setRollRatePID(float Kp, float Ki, float Kd) {
    Kp_roll_rate = Kp;
    Ki_roll_rate = Ki;
    Kd_roll_rate = Kd;
}

void IMU_Wrapper::setPitchRatePID(float Kp, float Ki, float Kd) {
    Kp_pitch_rate = Kp;
    Ki_pitch_rate = Ki;
    Kd_pitch_rate = Kd;
}

void IMU_Wrapper::setYawRatePID(float Kp, float Ki, float Kd) {
    Kp_yaw = Kp;
    Ki_yaw = Ki;
    Kd_yaw = Kd;
}



//HELPER FUNCTIONS

float IMU_Wrapper::invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  /*
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
  */
  return 1.0/sqrtf(x); //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}
