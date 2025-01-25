// for 2wd - 2 wheel drive robot
#include "pid.h"
#define ENCODE_RESOLUTION 442  // encoder pulses per motor one turn
#define PID_FREQUENCY 30  // Hz

struct motor_control {
  int input[2];  // in1, in2
  int encoder_pin[2];  // A port, B port
  int low_pin = 0;  // 0 or 1
  int pwm;  // 0~255
  volatile int encoder_count = 0;  // A port pull up count for PID
  volatile unsigned long echo_count = 0;  // A port pull up count for ECHO
  volatile unsigned long last_time = 0;  // last time the encoder was updated
  PID pid;
} motor1, motor2;

void encoder1ISR() {  // interrupt service routine for motor1
  ++motor1.encoder_count;
  ++motor1.echo_count;
}
void encoder2ISR() {  // interrupt service routine for motor2
  ++motor2.encoder_count;
  ++motor2.echo_count;
}

bool start_pid = false;

// define timer1 compare interrupt
ISR(TIMER1_COMPA_vect) {
  pid_control(motor1);
  pid_control(motor2);
}

void setup() {
  Serial.begin(57600);
  /******************* motor1 *************************/
  motor1.input[0] = 2;  // L298N in1, support pwm
  motor1.input[1] = 3;  // L298N in2, support pwm
  motor1.encoder_pin[0] = 20;  // A port, support interrupt
  motor1.encoder_pin[1] = 21;  // B port, support interrupt
  /******************* motor2 *************************/
  motor2.input[0] = 4;  // L298N in1, support pwm
  motor2.input[1] = 5;  // L298N in2, support pwm
  motor2.encoder_pin[0] = 18;  // A port, support interrupt
  motor2.encoder_pin[1] = 19;  // B port, support interrupt

  motor1.pid.verbose = false;

  for (int i = 0; i < 2; ++i) {
    pinMode(motor1.input[i], OUTPUT);
    pinMode(motor1.encoder_pin[i], INPUT);
    pinMode(motor2.input[i], OUTPUT);
    pinMode(motor2.encoder_pin[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(motor1.encoder_pin[0]), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(motor2.encoder_pin[0]), encoder2ISR, RISING);
  motor1.last_time = millis();
  motor2.last_time = millis();

  cli();//stop interrupts
  // interrupt control pid frequency
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = (F_CPU / 1024 / PID_FREQUENCY) - 1;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();//allow interrupts

  Serial.println("-----------------------------------------------");
  Serial.print("ENCODER_RESOLUTION:"); Serial.println(ENCODE_RESOLUTION);
  Serial.print("PID_FREQUENCY:"); Serial.print(PID_FREQUENCY); Serial.println("Hz");
  Serial.println("Commands:");
  Serial.println("r: reset encoder count for motor1 and motor2");
  Serial.println("e: echo encoder count for motor1 and motor2");
  Serial.println("p <pwm1> <pwm2>: set pwm for motor1 and motor2");
  Serial.println("s <rpm1> <rpm2>: set rpm for motor1 and motor2");
  Serial.println("-----------------------------------------------");
}

void set_pwm(motor_control& motor, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm < 0) {
    motor.low_pin = 1;
    pwm = -pwm;
  } else {
    motor.low_pin = 0;
  }
  motor.pwm = pwm;
  analogWrite(motor.input[!motor.low_pin], pwm);
  digitalWrite(motor.input[motor.low_pin], 0);
}

void set_rpm(motor_control& motor, double rpm) {
  rpm = constrain(rpm, -250, 250);
  double target_count = rpm * ENCODE_RESOLUTION / 60 / PID_FREQUENCY;
  motor.pid.setpoint = target_count;
}

void pid_control(motor_control& motor) {
  if (!start_pid) return;
  double current_time = millis();
  motor.pid.update(motor.encoder_count * (motor.low_pin ? -1 : 1));
  set_pwm(motor, motor.pid.output);
  motor.encoder_count = 0;
  motor.last_time = current_time;
}

int arg = 0, index = 0;
char cmd, argv[2][16];

void reset_cmd() {
  cmd = 0;
  memset(argv, 0, sizeof(argv));
  arg = 0;
  index = 0;
}

void close_pid() {
  start_pid = false;
  motor1.pid.reset();
  motor2.pid.reset();
  set_pwm(motor1, 0);
  set_pwm(motor2, 0);
}

void run_cmd() {
  int arg1 = atoi(argv[0]);
  int arg2 = atoi(argv[1]);
  switch (cmd) {
    case 'r':  // reset encoder count
      close_pid();
      motor1.encoder_count = 0;
      motor1.echo_count = 0;
      motor2.encoder_count = 0;
      motor2.echo_count = 0;
      Serial.println("Reset OK");
      break;
    case 'e':  // encoder count
      Serial.print(">motor1 PULL counter:"); Serial.println(motor1.echo_count);
      Serial.print(">motor2 PULL counter:"); Serial.println(motor2.echo_count);
      break;
    case 'p':  // pwm
      close_pid();
      set_pwm(motor1, arg1);
      set_pwm(motor2, arg2);
      Serial.print("Set PWM: ");
      Serial.print(arg1); Serial.print(" "); Serial.println(arg2);
      break;
    case 's':  // set rpm
      if (!start_pid) {
        start_pid = true;
        motor1.encoder_count = 0;
        motor1.last_time = millis();
        motor2.encoder_count = 0;
        motor2.last_time = millis();
      }
      set_rpm(motor1, arg1);
      set_rpm(motor2, arg2);
      if (arg1 == 0 && arg2 == 0) close_pid();
      Serial.print("Set RPM: ");
      Serial.print(arg1); Serial.print(" "); Serial.println(arg2);
      break;
    default:
      Serial.print("Invalid Command: "); Serial.println(cmd);
      break;
  }
}

void loop() {
  while (Serial.available() > 0) {
    char chr = Serial.read();
    if (chr == 13) {
      if (arg > 0) argv[arg-1][index] = '\0';
      run_cmd();
      reset_cmd();
    } else if (chr == ' ') {
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv[arg-1][index] = '\0';
        arg = 2;
        index = 0;
      }
      continue;
    } else {
      if (arg == 0) {
        cmd = chr;
      } else {
        argv[arg-1][index++] = chr;
      }
    }
  }
}