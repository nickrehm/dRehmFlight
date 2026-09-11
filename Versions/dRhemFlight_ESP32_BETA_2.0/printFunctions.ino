void printRadioData() {
  uint32_t now = micros();
  
  // Print every 10,000 us (10 ms)
  if (now - print_counter >= 10000) { 
    print_counter = now;
    if (dt > 0.0f) {
      for (int i = 0; i < NUM_CHANNELS; i++) {
        Serial.printf(" CH%d:%d", i, channel_pwm[i]);
      }
      Serial.println();
    }
  }
}

void printDesiredState() {
  uint32_t now = micros();
  
  // Print every 100,00 us (10 ms)
  if (now - print_counter >= 10000) { 
    print_counter = now;
    if (dt > 0.0f) {
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
}

void printGyroData() {
  uint32_t now = micros();
  
  // Print every 10,000 us (10 ms)
  if (now - print_counter >= 10000) { 
    print_counter = now;
    if (dt > 0.0f) {
      Serial.print(F("LOW:-300.0"));
      Serial.print(F(" HIGH:300.0"));
      Serial.print(F(" GyroX:"));
      Serial.print(GyroX);
      Serial.print(F(" GyroY:"));
      Serial.print(GyroY);
      Serial.print(F(" GyroZ:"));
      Serial.println(GyroZ);
    }
  }
}

void printAccelData() {
  uint32_t now = micros();
  
  // Print every 10,000 us (10 ms)
  if (now - print_counter >= 10000) { 
    print_counter = now;
    if (dt > 0.0f) {
      Serial.print(F("LOW:-2.0"));
      Serial.print(F(" HIGH:2.0"));
      Serial.print(F(" AccX:"));
      Serial.print(AccX);
      Serial.print(F(" AccY:"));
      Serial.print(AccY);
      Serial.print(F(" AccZ:"));
      Serial.println(AccZ);
    }
  }
}

void printMagData() {
  uint32_t now = micros();
  
  // Print every 10,000 us (10 ms)
  if (now - print_counter >= 10000) { 
    print_counter = now;
    if (dt > 0.0f) {
      Serial.print(F("LOW:-300.0"));
      Serial.print(F(" HIGH:300.0"));
      Serial.print(F(" MagX:"));
      Serial.print(MagX);
      Serial.print(F(" MagY:"));
      Serial.print(MagY);
      Serial.print(F(" MagZ:"));
      Serial.println(MagZ);
    }
  }
}

void printRollPitchYaw() {
  uint32_t now = micros();
  
  // Print every 10,000 us (10 ms)
  if (now - print_counter >= 10000) { 
    print_counter = now;
    if (dt > 0.0f) {
      Serial.print(F("LOW:-90.0"));
      Serial.print(F(" HIGH:90.0"));
      Serial.print(F(" roll:"));
      Serial.print(roll_IMU);
      Serial.print(F(" pitch:"));
      Serial.print(pitch_IMU);
      Serial.print(F(" yaw:"));
      Serial.println(yaw_IMU);
    }
  }
}

void printPIDoutput() {
  uint32_t now = micros();
  
  // Print every 10,000 us (10 ms)
  if (now - print_counter >= 10000) { 
    print_counter = now;
    if (dt > 0.0f) {
      Serial.print(F("LOW:-1.0"));
      Serial.print(F(" HIGH:1.0"));
      Serial.print(F(" roll_PID:"));
      Serial.print(roll_PID);
      Serial.print(F(" pitch_PID:"));
      Serial.print(pitch_PID);
      Serial.print(F(" yaw_PID:"));
      Serial.println(yaw_PID);
    }
  }
}

void printMotorCommands() {
  uint32_t now = micros();
  
  // Print every 10,000 us (10 ms)
  if (now - print_counter >= 10000) { 
    print_counter = now;
    if (dt > 0.0f) {
      Serial.print(F("LOW:125.0"));
      Serial.print(F(" HIGH:250.0"));

      for (int i=0; i<NUM_MOTORS; i++) {
        Serial.printf(" m%d_command_PWM:%d", i, m_command_PWM[i]);
      }
      Serial.println();
    }
  }
}

void printServoCommands() {
  uint32_t now = micros();
  
  // Print every 10,000 us (10 ms)
  if (now - print_counter >= 10000) { 
    print_counter = now;
    if (dt > 0.0f) {
      Serial.print(F("LOW:0.0"));
      Serial.print(F(" HIGH:180.0"));

      for (int i = 0; i < NUM_SERVOS; i++) {
        Serial.printf(" s%d_command_PWM:%d", i, s_command_PWM[i]);
      }
      Serial.println();
    }
  }
}

void printPIDvaluesRate() {
  uint32_t now = micros();

  // Print every 100,000 us (100 ms)
  if (now - print_counter >= 100000) { 
    print_counter = now;
    if (dt > 0.0f) {
      // Load PID values
      float Kp_roll_rate = rollPID.kp_rate.load(std::memory_order_relaxed);
      float Ki_roll_rate = rollPID.ki_rate.load(std::memory_order_relaxed);
      float Kd_roll_rate = rollPID.kd_rate.load(std::memory_order_relaxed);

      float Kp_pitch_rate = pitchPID.kp_rate.load(std::memory_order_relaxed);
      float Ki_pitch_rate = pitchPID.ki_rate.load(std::memory_order_relaxed);
      float Kd_pitch_rate = pitchPID.kd_rate.load(std::memory_order_relaxed);

      float Kp_yaw_rate = yawPID.kp_rate.load(std::memory_order_relaxed);
      float Ki_yaw_rate = yawPID.ki_rate.load(std::memory_order_relaxed);
      float Kd_yaw_rate = yawPID.kd_rate.load(std::memory_order_relaxed);

      Serial.print(F("Rroll_PID: "));
      Serial.print(Kp_roll_rate, 4);
      Serial.print(F("  "));
      Serial.print(Ki_roll_rate, 4);
      Serial.print(F("  "));
      Serial.print(Kd_roll_rate, 4);

      Serial.print(F("  |  Rpitch_PID: "));
      Serial.print(Kp_pitch_rate, 4);
      Serial.print(F("  "));
      Serial.print(Ki_pitch_rate, 4);
      Serial.print(F("  "));
      Serial.print(Kd_pitch_rate, 4);
      
      Serial.print(F("  |  Ryaw_PID: "));
      Serial.print(Kp_yaw_rate, 4);
      Serial.print(F("  "));
      Serial.print(Ki_yaw_rate, 4);
      Serial.print(F("  "));
      Serial.println(Kd_yaw_rate, 4);
    }
  }
}


void printPIDvaluesAngle() {
  uint32_t now = micros();
  
  // Print every 100,000 us (100 ms)
  if (now - print_counter >= 100000) { 
    print_counter = now;
    if (dt > 0.0f) {
      // Load PID values
      float Kp_roll_angle = rollPID.kp_angle.load(std::memory_order_relaxed);
      float Ki_roll_angle = rollPID.ki_angle.load(std::memory_order_relaxed);
      float Kd_roll_angle = rollPID.kd_angle.load(std::memory_order_relaxed);

      float Kp_pitch_angle = pitchPID.kp_angle.load(std::memory_order_relaxed);
      float Ki_pitch_angle = pitchPID.ki_angle.load(std::memory_order_relaxed);
      float Kd_pitch_angle = pitchPID.kd_angle.load(std::memory_order_relaxed);

      float Kp_yaw_angle = yawPID.kp_angle.load(std::memory_order_relaxed);
      float Ki_yaw_angle = yawPID.ki_angle.load(std::memory_order_relaxed);
      float Kd_yaw_angle = yawPID.kd_angle.load(std::memory_order_relaxed);

      Serial.print(F("Aroll_PID: "));
      Serial.print(Kp_roll_angle, 4);
      Serial.print(F("  "));
      Serial.print(Ki_roll_angle, 4);
      Serial.print(F("  "));
      Serial.print(Kd_roll_angle, 4);

      Serial.print(F("  |  Apitch_PID: "));
      Serial.print(Kp_pitch_angle, 4);
      Serial.print(F("  "));
      Serial.print(Ki_pitch_angle, 4);
      Serial.print(F("  "));
      Serial.print(Kd_pitch_angle, 4);
      
      Serial.print(F("  |  Ayaw_PID: "));
      Serial.print(Kp_yaw_angle, 4);
      Serial.print(F("  "));
      Serial.print(Ki_yaw_angle, 4);
      Serial.print(F("  "));
      Serial.println(Kd_yaw_angle, 4);
    }
  }
}



void printLoopRate() {
  uint32_t now = micros();
  
  // Print every 100,000 us (100 ms)
  if (now - print_counter >= 100000) { 
    print_counter = now;
    if (dt > 0.0f) {
      Serial.printf("dt (us): %.2f | Freq: %.1f Hz\n", dt * 1000000.0f, 1.0f / dt);
    }
  }
}