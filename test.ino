#include <Wire.h>

// #define DEBUG_ACCELEROMETER
#define DEBUG_ANGLES

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

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
}

void gyro_signals(void) {

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180 / 3.142);
  AnglePitch =- atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180 / 3.142);
}

void loop() {
  gyro_signals();

#ifdef DEBUG_ACCELEROMETER
  Serial.println(
                 "Min:" + String(-1) +
                 ",Max:" + String(1) +
                 ",Acceleration_X:" + String(AccX) +
                 ",Acceleration_Y:" + String(AccY) +
                 ",Acceleration_Z:" + String(AccZ)
  );
#elif defined(DEBUG_ANGLES)
  Serial.println(
                 "Min:" + String(-45) +
                 ",Max:" + String(45) +
                 ",AngleRoll:" + String(AngleRoll) +
                 ",AnglePitch:" + String(AnglePitch)
  );
#endif

  delay(50);
}
