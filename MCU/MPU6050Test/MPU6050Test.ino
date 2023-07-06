#include <Wire.h>

float RateRoll, RatePitch, RateYaw;

float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw; //Calibration values
int RateCalibrationNumber;
int CalibrationTime = 2000;

float AccX, AccY, AccZ; // Accelerometer variabler
float AngleRoll, AnglePitch;

float AccCalibrationX = -0.03;
float AccCalibrationY = -0.01;
float AccCalibrationZ = 0.02;

float LoopTimer;

float KalmanAngleRoll = 0, //Første "gaet" på initierende vinkel
      KalmanUncertaintyAngleRoll = 2 * 2; //Usikkerhed på initierende vinkel på 2 grader
float KalmanAnglePitch = 0,
      KalmanUncertaintyAnglePitch = 2 * 2;

void Gyro_signals(void) {
  Wire.beginTransmission(0x68); //default value for MPU-6050 er 0x68 for the I2C adress
  Wire.write(0x1A); // Low pass filter (Digital low pass filter: DLPF)
  Wire.write(0x05); // Setting up filter for 10Hz bandwith,
  Wire.endTransmission();

  Wire.beginTransmission(0x68); //Accelerometer output
  Wire.write(0x1C); //Accelerometer register (Hex)
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //Accelerometer measurements storage register
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6); //Henter 6 bytes
  int16_t AccXLSB = Wire.read() << 8 | Wire.read(); //Henter målingerne fra sensoren
  int16_t AccYLSB = Wire.read() << 8 | Wire.read(); // -||-
  int16_t AccZLSB = Wire.read() << 8 | Wire.read(); // -||-

  Wire.beginTransmission(0x68);
  Wire.write(0x1B); //Sensitivity scale factor
  Wire.write(0x8); //Setting the sensitivity
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x43); // Acces registers storing measurements for gyro, first entry is 43
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6); //Requesting 6-bites to pull information from the storing measurements
  // the 6 entries from measurements

  int16_t GyroX = Wire.read() << 8 | Wire.read(); //Reading the measurements from the x-axis, need for 2 reads as there are 2 entrys
  int16_t GyroY = Wire.read() << 8 | Wire.read(); //Reading the measurements from the y-axis
  int16_t GyroZ = Wire.read() << 8 | Wire.read(); //Reading the measurements from the z-axis

  //Converting gyroscope measurements from LSB to degree/second
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  //Converting accelerometer measurements to physical values
  //Der kan kalibreres her for små uligheder i de 3 retninger
  AccX = (float)AccXLSB / 4096 + AccCalibrationX;
  AccY = (float)AccYLSB / 4096 + AccCalibrationY;
  AccZ = (float)AccZLSB / 4096 + AccCalibrationZ;

  //Udregning af vinkler i grader:
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.145 / 180);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.145 / 180);
}

void GyroPrint() { //Printer gyroskopets målinger med grader i sekundet
  RateRoll -= RateCalibrationRoll; //Correct values
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  Serial.print("Roll rate = ");
  Serial.print(RateRoll);
  Serial.print("Pitch rate = ");
  Serial.print(RatePitch);
  Serial.print("Yaw rate = ");
  Serial.println(RateYaw);
  delay(50);
}
void AccPrint() { //Printer tyngdeaccelerationen i aksernes retning
  Serial.print("Accerelation X [g]= ");
  Serial.print(AccX);
  Serial.print("Accerelation Y [g]= ");
  Serial.print(AccY);
  Serial.print("Acceleration Z [g]= ");
  Serial.println(AccZ);
  delay(50);
}

void RollPitchPrint() {
  Serial.print("Roll angle = ");
  Serial.print(AnSgleRoll);
  Serial.print("Pitch angle = ");
  Serial.println(AnglePitch);
  delay(50);
}

void setup() {
  Serial.begin(112000);
  // pinMode(X, OUTPUT);
  // digitalWrite(X, HIGH);

  Wire.setClock(400000); //Clock speed for I2C, MPU-6050 supports up to 400kHz
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68); //Write to power management mode
  Wire.write(0x6B);
  Wire.write(00);
  Wire.endTransmission();

  for (RateCalibrationNumber = 0; RateCalibrationNumber < CalibrationTime;
       RateCalibrationNumber++) { //Calibration measurements
    Gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
    //Gyroskopet må ikke flyttes under denne del, da der skal kalibreres først
  }
  RateCalibrationRoll /= CalibrationTime;
  RateCalibrationPitch /= CalibrationTime;
  RateCalibrationYaw /= CalibrationTime;
}

void loop() {
  // Printing the degree pr second measurements in the serial monitor for each axis.
  // Note, the measurements are not the current angle of the sensor
  Gyro_signals();

  //For print
  //GyroPrint();
  //AccPrint();
  RollPitchPrint();


}
