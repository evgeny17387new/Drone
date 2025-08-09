#include <Wire.h>

// #define DEBUG_ACCELEROMETER
// #define DEBUG_ANGLES_ROLL
#define DEBUG_ANGLES_PITCH
// #define DEBUG_LOOP_TIMER

uint32_t LoopTimer;

float AccX, AccY, AccZ;
float AngleRollAccl, AnglePitchAccl;

float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
float AngleRollGyro = 0, AnglePitchGyro = 0;

float Kalman1DOutput[] = {0, 0};

float AngleRollKalman = 0, AnglePitchKalman = 0;
float KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanUncertaintyAnglePitch = 2 * 2;

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

  // Set Accelerometer Full Scale Range to ±8g
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Set Gyro Full Scale Range to ±500 degrees/sec
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read()<<8 | Wire.read();
  int16_t GyroY = Wire.read()<<8 | Wire.read();
  int16_t GyroZ = Wire.read()<<8 | Wire.read();

  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  RateRoll = (float)GyroY / 65.5;
  RatePitch = (float)GyroX / 65.5;
  RateYaw = (float)GyroZ / 65.5;
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

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  gyro_setup();

  gyro_calibration();

  LoopTimer = micros();
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
  gyro_signals();

  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;

  RatePitch = -RatePitch;

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

#ifdef DEBUG_ACCELEROMETER
  Serial.println(
                 "Min:" + String(-1) +
                 ",Max:" + String(1) +
                 ",Acceleration_X:" + String(AccX) +
                 ",Acceleration_Y:" + String(AccY) +
                 ",Acceleration_Z:" + String(AccZ)
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
#elif defined(DEBUG_LOOP_TIMER)
  Serial.println(String(micros() - LoopTimer));
#endif

  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}
