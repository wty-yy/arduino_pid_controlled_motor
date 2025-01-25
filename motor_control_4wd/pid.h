struct PID {
  // double kp = 1.0;  // proportional gain
  // double ki = 0.5;  // integral gain
  // double kd = 0.1;  // derivative gain
  // try PID constants from: https://github.com/linorobot/linorobot2_hardware/blob/galactic/config/lino_base_config.h#L37
  double kp = 5.0;  // proportional gain
  double ki = 0.8;  // integral gain
  double kd = 0.5;  // derivative gain
  double setpoint = 0;  // target speed in RPM
  double error = 0;
  double last_error = 0;
  double integral = 0;
  double derivative = 0;
  double output = 0;
  double max_output = 255;
  double min_output = -255;
  bool verbose = false;
  void update(double input) {
    error = setpoint - input;
    integral += error;
    derivative = error - last_error;
    if (setpoint == 0 && error == 0) {
      integral = 0;
      derivative = 0;
    }
    output = kp * error + ki * integral + kd * derivative;
    output = constrain(output, min_output, max_output);
    if (verbose) {
      Serial.print(">encoder_count:"); Serial.println(input);
      Serial.print(">setpoint:"); Serial.println(setpoint);
      Serial.print(">error:"); Serial.println(error);
      Serial.print(">output:"); Serial.println(output);
    }
    last_error = error;
  }
  void reset() {
    error = 0;
    last_error = 0;
    integral = 0;
    derivative = 0;
    output = 0;
  }
};
