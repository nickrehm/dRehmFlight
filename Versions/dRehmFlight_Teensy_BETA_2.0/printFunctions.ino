void printRadioData() {
  if (current_time - print_counter > PRINTLOOPINTERVAL) {
    print_counter = micros();
    Serial.print(F("LOW:1000.0"));
    Serial.print(F(" HIGH:2000.0"));
    for (int8_t i = 0; i < NUM_CHANNELS; i++) {
      Serial.print(F(" CH"));
      Serial.print(i+1);
      Serial.print(F(":"));
      Serial.print(channel_pwm[i]);
    }
    Serial.println(F(""));
  }
}

void printDesiredState() {
  if (current_time - print_counter > PRINTLOOPINTERVAL) {
    print_counter = micros();
    Serial.print(F("LOW:-180.0"));
    Serial.print(F(" HIGH:180.0"));
    Serial.print(F(" thro_des:"));
    Serial.print(thro_des);
    Serial.print(F(" roll_des:"));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des:"));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des:"));
    Serial.println(yaw_des);
  }
}

void printGyroData() {
  if (current_time - print_counter > PRINTLOOPINTERVAL) {
    print_counter = micros();
    Serial.print(F("LOW:-300.0"));
    Serial.print(F(" HIGH:300.0"));
    Serial.print(F(" GyroX:"));
    Serial.print(imu.GyroX);
    Serial.print(F(" GyroY:"));
    Serial.print(imu.GyroY);
    Serial.print(F(" GyroZ:"));
    Serial.println(imu.GyroZ);
  }
}

void printAccelData() {
  if (current_time - print_counter > PRINTLOOPINTERVAL) {
    print_counter = micros();
    Serial.print(F("LOW:-2.0"));
    Serial.print(F(" HIGH:2.0"));
    Serial.print(F(" AccX:"));
    Serial.print(imu.AccX);
    Serial.print(F(" AccY:"));
    Serial.print(imu.AccY);
    Serial.print(F(" AccZ:"));
    Serial.println(imu.AccZ);
  }
}

void printMagData() {
  if (current_time - print_counter > PRINTLOOPINTERVAL) {
    print_counter = micros();
    Serial.print(F("LOW:-300.0"));
    Serial.print(F(" HIGH:300.0"));
    Serial.print(F(" MagX:"));
    Serial.print(imu.MagX);
    Serial.print(F(" MagY:"));
    Serial.print(imu.MagY);
    Serial.print(F(" MagZ:"));
    Serial.println(imu.MagZ);
  }
}

void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("LOW:-180.0"));
    Serial.print(F(" HIGH:180.0"));
    Serial.print(F(" roll:"));
    Serial.print(imu.roll_IMU);
    Serial.print(F(" pitch:"));
    Serial.print(imu.pitch_IMU);
    Serial.print(F(" yaw:"));
    Serial.println(imu.yaw_IMU);
  }
}

void printPIDoutput() {
  if (current_time - print_counter > PRINTLOOPINTERVAL) {
    print_counter = micros();
    Serial.print(F("LOW:-1.0"));
    Serial.print(F(" HIGH:1.0"));
    Serial.print(F(" roll_PID:"));
    Serial.print(imu.roll_PID);
    Serial.print(F(" pitch_PID:"));
    Serial.print(imu.pitch_PID);
    Serial.print(F(" yaw_PID:"));
    Serial.println(imu.yaw_PID);
  }
}

void printMotorCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("LOW:125.0"));
    Serial.print(F(" HIGH:250.0"));
    for (int i=0; i<numMotors; i++) {
      Serial.print(F(" m"));
      Serial.print(i+1);
      Serial.print(F("_command:"));
      Serial.print(m_command_scaled[i]);
    }
    Serial.println("");
  }
}

void printServoCommands() {
  if (current_time - print_counter > PRINTLOOPINTERVAL) {
    print_counter = micros();
    Serial.print(F("LOW:0.0"));
    Serial.print(F(" HIGH:1.0"));
    for (int i=0; i<numServos; i++) {
      Serial.print(F(" s"));
      Serial.print(i+1);
      Serial.print(F("_command:"));
      Serial.print(s_command_scaled[i]);
    }
    Serial.println("");
  }
}

void printLoopRate() {
  if (current_time - print_counter > PRINTLOOPINTERVAL) {
    print_counter = micros();
    Serial.print(F("dt:"));
    Serial.println(dt*1000000.0);
  }
}

void printLoopRateAndProfiledTimes() {
  // DESCRIPTION: Prints average loop rate and detailed execution times of profiled functions.
  unsigned long currentTimeMicros = micros();
  if (currentTimeMicros - loopRateLastPrintMicros > PRINTLOOPINTERVAL) {
    unsigned long elapsedMicrosInterval = currentTimeMicros - loopRateLastPrintMicros;
    if (loopRateCounter > 0 && elapsedMicrosInterval > 0) {
      float avgFreq = (float)loopRateCounter * 1000000.0f / elapsedMicrosInterval;
      Serial.print("AvgHz: "); Serial.print(avgFreq, 1);

      Serial.print(" | TotInt_us: "); Serial.print(time_totalLoopInternal_us);
      Serial.print(" | IMU: "); Serial.print(time_getIMUdata_us);
      Serial.print(" | Madg: "); Serial.print(time_Madgwick_us);
      Serial.print(" | DesSt: "); Serial.print(time_getDesState_us);
      Serial.print(" | Fadr: "); Serial.print(time_faderLogic_us);
      Serial.print(" | Angl: "); Serial.print(time_controlANGLE_us);
      Serial.print(" | Mix: "); Serial.print(time_controlMixer_us);
      Serial.print(" | Scale: "); Serial.print(time_scaleCommands_us);
      Serial.print(" | CmdMot: "); Serial.print(time_commandMotors_us);
      Serial.print(" | Servos: "); Serial.print(time_servoWrites_us);
      Serial.print(" | GetCmd: "); Serial.print(time_getCommands_us);
      Serial.print(" | FS: "); Serial.println(time_failSafe_us);
    }

    // Reset stats for the next printing interval
    loopRateLastPrintMicros = currentTimeMicros;
    loopRateCounter = 0;
  }
}

