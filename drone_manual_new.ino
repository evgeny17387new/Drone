#include <Arduino.h>
#include <Wire.h>

#define FLIGHT_MODE_RATES 0
#define FLIGHT_MODE_ANGLES 1
char flight_mode = FLIGHT_MODE_ANGLES;

// #define DEBUG_LOOP_TIMER

// #define DEBUG_RATES
// #define DEBUG_DESIRED_RATES
// #define DEBUG_ERROR_RATES
// #define DEBUG_RATE_ROLL
// #define DEBUG_RATE_PITCH
// #define DEBUG_RATE_YAW

#define DEBUG_ANGLES_ROLL
// #define DEBUG_ANGLES_PITCH

// #define DEBUG_INPUTS
// #define DEBUG_MOTORS

// #define DEBUG_THROTTLE

#define RX_THROTTLE_PWM_PIN 2
#define RX_ROLL_PWM_PIN 3
#define RX_PITCH_PWM_PIN 4
#define RX_YAW_PWM_PIN 5

#define MOTOR_1_PWM_PIN 6
#define MOTOR_2_PWM_PIN 7
#define MOTOR_3_PWM_PIN 8
#define MOTOR_4_PWM_PIN 9

#define LED_PIN 13

#define PWM_FREQUENCY 250

#define MOTORS_CUTOFF 1000
#define THROTTLE_MAX 1500
#define MOTORS_MAX 20000
#define THROTTLE_IDLE 1050

#define P_RATE_ROLL 0.6
#define P_RATE_PITCH 0.6
#define P_RATE_YAW 2
#define I_RATE_ROLL 3.5
#define I_RATE_PITCH 3.5
#define I_RATE_YAW 12
#define D_RATE_ROLL 0.03
#define D_RATE_PITCH 0.03
#define D_RATE_YAW 0

#define P_ANGLE_ROLL 2
#define P_ANGLE_PITCH 2
#define I_ANGLE_ROLL 0
#define I_ANGLE_PITCH 0
#define D_ANGLE_ROLL 0
#define D_ANGLE_PITCH 0

uint32_t LoopTimer;

float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;

float AccX, AccY, AccZ;
float AngleRollAccl, AnglePitchAccl;

volatile uint32_t lastRiseThrottle = 0, lastRiseRoll = 0, lastRisePitch = 0, lastRiseYaw = 0;
volatile unsigned long InputThrottle = 0, DesiredRoll = 0, DesiredPitch = 0, DesiredYaw = 0;

float RXRollCenter;
float RXPitchCenter;
float RXYawCenter;

float AngleRollGyro = 0, AnglePitchGyro = 0;

float Kalman1DOutput[] = {0, 0};

float AngleRollKalman = 0, AnglePitchKalman = 0;
float KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanUncertaintyAnglePitch = 2 * 2;

float DesiredRatePitch, DesiredRateRoll, DesiredRateYaw;
float DesiredAngleRoll, DesiredAnglePitch;

float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;

float PrevErrorRateRoll = 0, PrevItermRateRoll = 0;
float PrevErrorRatePitch = 0, PrevItermRatePitch = 0;
float PrevErrorRateYaw = 0, PrevItermRateYaw = 0;

float PIDReturn[] = {0, 0};

float InputRoll, InputPitch, InputYaw;

int MotorInput1;
int MotorInput2;
int MotorInput3;
int MotorInput4;

void gyro_setup() {
  // Wake up the MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Set DLPF to 5 Hz
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Set Gyro Full Scale Range to ±500 degrees/sec
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // Set Accelerometer Full Scale Range to ±8g
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
}

void gyro_signals() {
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  int16_t AccXLSB;
  int16_t AccYLSB;
  int16_t AccZLSB;

  if (flight_mode == FLIGHT_MODE_ANGLES) {

    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(); 
    Wire.requestFrom(0x68, 6);
    AccXLSB = Wire.read() << 8 | Wire.read();
    AccYLSB = Wire.read() << 8 | Wire.read();
    AccZLSB = Wire.read() << 8 | Wire.read();

  }

  RateRoll = (float)GyroY / 65.5;
  RatePitch = (float)GyroX / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  if (flight_mode == FLIGHT_MODE_ANGLES) {

    AccX = (float)AccXLSB / 4096;
    AccY = (float)AccYLSB / 4096;
    AccZ = (float)AccZLSB / 4096;

  }

}

void gyro_calibration() {
  for (int RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
}

// TODO: on rc disconnect, values may stay the same, need to implement rc connected check
void isrThrottle() {
  if (digitalRead(RX_THROTTLE_PWM_PIN)) {
    lastRiseThrottle = micros();
  } else {
    InputThrottle = micros() - lastRiseThrottle;
    // TODO: might not be good for calibration
    if (InputThrottle > THROTTLE_MAX) {
      InputThrottle = THROTTLE_MAX;
    }
  }
}
void isrRoll() {
  if (digitalRead(RX_ROLL_PWM_PIN)) {
    lastRiseRoll = micros();
  } else {
    DesiredRoll = micros() - lastRiseRoll;
  }
}
void isrPitch() {
  if (digitalRead(RX_PITCH_PWM_PIN)) {
    lastRisePitch = micros();
  } else {
    DesiredPitch = micros() - lastRisePitch;
  }
}
void isrYaw() {
  if (digitalRead(RX_YAW_PWM_PIN)) {
    lastRiseYaw = micros();
  } else {
    DesiredYaw = micros() - lastRiseYaw;
  }
}

void receiver_center_calibration() {
  RXRollCenter = 0;
  RXPitchCenter = 0;
  RXYawCenter = 0;
  for (int i = 0; i < 2000; i++) {
    RXRollCenter += DesiredRoll;
    RXPitchCenter += DesiredPitch;
    RXYawCenter += DesiredYaw;
    delay(1);
  }
  RXRollCenter /= 2000;
  RXPitchCenter /= 2000;
  RXYawCenter /= 2000;
}

void setup() {

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(115200);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  gyro_setup();

  gyro_calibration();

  pinMode(RX_THROTTLE_PWM_PIN, INPUT);
  pinMode(RX_ROLL_PWM_PIN, INPUT);
  pinMode(RX_PITCH_PWM_PIN, INPUT);
  pinMode(RX_YAW_PWM_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(RX_THROTTLE_PWM_PIN), isrThrottle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_ROLL_PWM_PIN), isrRoll, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_PITCH_PWM_PIN), isrPitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_YAW_PWM_PIN), isrYaw, CHANGE);

  pinMode(MOTOR_1_PWM_PIN, OUTPUT);
  pinMode(MOTOR_2_PWM_PIN, OUTPUT);
  pinMode(MOTOR_3_PWM_PIN, OUTPUT);
  pinMode(MOTOR_4_PWM_PIN, OUTPUT);

  analogWriteFrequency(MOTOR_1_PWM_PIN, PWM_FREQUENCY);
  analogWriteFrequency(MOTOR_2_PWM_PIN, PWM_FREQUENCY);
  analogWriteFrequency(MOTOR_3_PWM_PIN, PWM_FREQUENCY);
  analogWriteFrequency(MOTOR_4_PWM_PIN, PWM_FREQUENCY);

  analogWriteResolution(12);

  analogWrite(MOTOR_1_PWM_PIN, MOTORS_CUTOFF);
  analogWrite(MOTOR_2_PWM_PIN, MOTORS_CUTOFF);
  analogWrite(MOTOR_3_PWM_PIN, MOTORS_CUTOFF);
  analogWrite(MOTOR_4_PWM_PIN, MOTORS_CUTOFF);

  Serial.println("Waiting for receiver to connect...");

  // 1000 is for receiver not connected
  // 1050 is for throttle not idle
  while (InputThrottle < 1000 || InputThrottle > 1050) {
    delay(20);
  }

  Serial.println("Receiver connected, calibrating...");

  receiver_center_calibration();

  digitalWrite(LED_PIN, LOW);

  LoopTimer = micros();
}

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;

  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
  if (Iterm > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;

  float Dterm = D * (Error - PrevError) / 0.004;

  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400) PIDOutput = 400;
  else if (PIDOutput < -400) PIDOutput =- 400;

  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Iterm;
}

void reset_pid(void) {
  PrevErrorRateRoll = 0;
  PrevItermRateRoll = 0;
  PrevErrorRatePitch = 0;
  PrevItermRatePitch = 0;
  PrevErrorRateYaw = 0;
  PrevItermRateYaw = 0;
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {

  KalmanState = KalmanState + 0.004 * KalmanInput;

  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;

  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);

  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);

  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  Kalman1DOutput[0] = KalmanState; 
  Kalman1DOutput[1] = KalmanUncertainty;
}

void loop() {

// ************************************************************************************************* Gyro signals

  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;

  // physicaly inverted axis
  RatePitch = -RatePitch;
  RateYaw = -RateYaw;

// ************************************************************************************************* Angles

  if (flight_mode == FLIGHT_MODE_ANGLES) {

    AngleRollAccl = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180 / 3.142);
    AnglePitchAccl = -atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180 / 3.142);

    AngleRollGyro = AngleRollGyro + 0.004 * RateRoll;
    AnglePitchGyro = AnglePitchGyro + 0.004 * RatePitch;

    kalman_1d(AngleRollKalman, KalmanUncertaintyAngleRoll, RateRoll, AngleRollAccl);
    AngleRollKalman = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

    kalman_1d(AnglePitchKalman, KalmanUncertaintyAnglePitch, RatePitch, AnglePitchAccl);
    AnglePitchKalman = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

    // 0.1 of max/min of +-50 deg angle
    DesiredAngleRoll = 0.1 * ((float)DesiredRoll - RXRollCenter);
    DesiredAnglePitch = 0.1 * ((float)DesiredPitch - RXPitchCenter);

  }

// ************************************************************************************************* Rates

  if (flight_mode == FLIGHT_MODE_ANGLES) {

    DesiredRateRoll = P_ANGLE_ROLL * (DesiredAngleRoll - AngleRollKalman);
    DesiredRatePitch = P_ANGLE_PITCH * (DesiredAnglePitch - AnglePitchKalman);

  } else {

    // 0.15 of max/min of +-75 deg/sec rate
    DesiredRateRoll = 0.15 * ((float)DesiredRoll - RXRollCenter);
    DesiredRatePitch = 0.15 * ((float)DesiredPitch - RXPitchCenter);

  }

  DesiredRateYaw = 0.15 * ((float)DesiredYaw - RXYawCenter);

  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  pid_equation(ErrorRateRoll, P_RATE_ROLL, I_RATE_ROLL, D_RATE_ROLL, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll = PIDReturn[0];
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = PIDReturn[1];

  pid_equation(ErrorRatePitch, P_RATE_PITCH, I_RATE_PITCH, D_RATE_PITCH, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = PIDReturn[1];

  pid_equation(ErrorRateYaw, P_RATE_YAW, I_RATE_YAW, D_RATE_YAW, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw = PIDReturn[0];
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = PIDReturn[1];

// ************************************************************************************************* Motors

  MotorInput1 = 1.024 * (InputThrottle - InputRoll - InputPitch + InputYaw);
  MotorInput2 = 1.024 * (InputThrottle - InputRoll + InputPitch - InputYaw);
  MotorInput3 = 1.024 * (InputThrottle + InputRoll + InputPitch + InputYaw);
  MotorInput4 = 1.024 * (InputThrottle + InputRoll - InputPitch - InputYaw);

  if (MotorInput1 > MOTORS_MAX) MotorInput1 = MOTORS_MAX;
  if (MotorInput2 > MOTORS_MAX) MotorInput2 = MOTORS_MAX;
  if (MotorInput3 > MOTORS_MAX) MotorInput3 = MOTORS_MAX;
  if (MotorInput4 > MOTORS_MAX) MotorInput4 = MOTORS_MAX;

  // TODO: currently causing motors spin very fast
  // if (MotorInput1 < THROTTLE_IDLE) MotorInput1 = THROTTLE_IDLE;
  // if (MotorInput2 < THROTTLE_IDLE) MotorInput2 = THROTTLE_IDLE;
  // if (MotorInput3 < THROTTLE_IDLE) MotorInput3 = THROTTLE_IDLE;
  // if (MotorInput4 < THROTTLE_IDLE) MotorInput4 = THROTTLE_IDLE;

  // In case RC is disconnected, set motors to InputThrottle might be harmful
  if (InputThrottle < THROTTLE_IDLE) {
    MotorInput1 = MOTORS_CUTOFF;
    MotorInput2 = MOTORS_CUTOFF;
    MotorInput3 = MOTORS_CUTOFF;
    MotorInput4 = MOTORS_CUTOFF;
    reset_pid();
  }

  analogWrite(MOTOR_1_PWM_PIN, MotorInput1);
  analogWrite(MOTOR_2_PWM_PIN, MotorInput2);
  analogWrite(MOTOR_3_PWM_PIN, MotorInput3);
  analogWrite(MOTOR_4_PWM_PIN, MotorInput4);

// ************************************************************************************************* Debbugging

#if defined(DEBUG_LOOP_TIMER)
  // LKG value:
  // Rates mode: 841 us
  // Angles mode: 1680 us
  Serial.println(String(micros() - LoopTimer));
#elif defined(DEBUG_RATES)
  Serial.println(
                 "Min:" + String(-75) +
                 ",Max:" + String(75) +
                 ",RateRoll:" + String(RateRoll) +
                 ",RatePitch:" + String(RatePitch) +
                 ",RateYaw:" + String(RateYaw)
  );
#elif defined(DEBUG_DESIRED_RATES)
  Serial.println(
                 "Min:" + String(-75) +
                 ",Max:" + String(75) +
                 ",DesiredRateRoll:" + String(DesiredRateRoll) +
                 ",DesireRatedPitch:" + String(DesiredRatePitch) +
                 ",DesiredRateYaw:" + String(DesiredRateYaw)
  );
#elif defined(DEBUG_ERROR_RATES)
  Serial.println(
                 "Min:" + String(-75) +
                 ",Max:" + String(75) +
                 ",ErrorRateRoll:" + String(ErrorRateRoll) +
                 ",ErrorRatePitch:" + String(ErrorRatePitch) +
                 ",ErrorRateYaw:" + String(ErrorRateYaw)
  );
#elif defined(DEBUG_RATE_ROLL)
  Serial.println(
                 "Min:" + String(-75) +
                 ",Max:" + String(75) +
                 ",RateRoll:" + String(RateRoll) +
                 ",DesiredRateRoll:" + String(DesiredRateRoll) +
                 ",ErrorRateRoll:" + String(ErrorRateRoll)
  );
#elif defined(DEBUG_RATE_PITCH)
  Serial.println(
                 "Min:" + String(-75) +
                 ",Max:" + String(75) +
                 ",RatePitch:" + String(RatePitch) +
                 ",DesiredRatePitch:" + String(DesiredRatePitch) +
                 ",ErrorRatePitch:" + String(ErrorRatePitch)
  );
#elif defined(DEBUG_RATE_YAW)
  Serial.println(
                 "Min:" + String(-75) +
                 ",Max:" + String(75) +
                 ",RateYaw:" + String(RateYaw) +
                 ",DesiredRateYaw:" + String(DesiredRateYaw) +
                 ",ErrorRateYaw:" + String(ErrorRateYaw)
  );
#elif defined(DEBUG_ANGLES_ROLL)
  Serial.println(
                 "Min:" + String(-90) +
                 ",Max:" + String(90) +
                 ",AngleRollAccl:" + String(AngleRollAccl) +
                 ",AngleRollGyro:" + String(AngleRollGyro) +
                 ",AngleRollKalman:" + String(AngleRollKalman)
  );
#elif defined(DEBUG_ANGLES_PITCH)
  Serial.println(
                 "Min:" + String(-90) +
                 ",Max:" + String(90) +
                 ",AnglePitchAccl:" + String(AnglePitchAccl) +
                 ",AnglePitchGyro:" + String(AnglePitchGyro) +
                 ",AnglePitchKalman:" + String(AnglePitchKalman)
  );
  #elif defined(DEBUG_INPUTS)
  Serial.println(
                 "Min:" + String(0) +
                 ",Max:" + String(500) +
                 ",InputRoll:" + String(InputRoll) +
                 ",InputPitch:" + String(InputPitch) +
                 ",InputYaw:" + String(InputYaw)
  );
#elif defined(DEBUG_MOTORS)
  Serial.println(
                 "Min:" + String(1000) +
                 ",Max:" + String(2000) +
                 ",MotorInput1:" + String(MotorInput1) +
                 ",MotorInput2:" + String(MotorInput2) +
                 ",MotorInput3:" + String(MotorInput3) +
                 ",MotorInput4:" + String(MotorInput4)
  );
#elif defined(DEBUG_THROTTLE)
  Serial.println("InputThrottle:" + String(InputThrottle));
#endif

// ************************************************************************************************* Lopp timer

  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}
