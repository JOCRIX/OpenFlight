#include <Wire.h>
#include <math.h>

// Debugging and Features
#define PRINTDISTMATRIX 0
#define PRINTMOTOROMEGA 0
#define PRINTACCELDATA 0
#define PRINTACCOFFSETS 0
#define PRINTDRONEANGLES 0
#define PRINTGYRODATA 0
#define PRINTGYROOFFSETS 0
#define PRINTDRONEORIENTATION 0
#define PRINTULTRASOUNDALTITUDE 0
#define PRINTDRONESTATUS 1
#define USETACHOMETERS 0
#define USEI2C 1
#define ACTIVATEIMU1 1
#define USEULTRASOUND 1
#define USEBAROMETER 0

// Fan Motor Defines
#define F1Tach 21
#define F2Tach 20
#define F3Tach 19
#define F4Tach 18
#define F1PWM 6
#define F2PWM 7
#define F3PWM 8
#define F4PWM 9
//ESC PWM
#define M1 2
#define M2 3
#define M3 10
#define M4 11
// External IMU module settings
#define IMUADDR 0x68
#define IMUAccelSens 4096
// External Altimeter settings
#define USTRIGPIN 5
#define USECHOPIN 4

volatile uint32_t tach1Count, tach2Count, tach3Count, tach4Count;
uint16_t F1RPM, F2RPM, F3RPM, F4RPM;
struct droneParameters
{
  float Cl, Cd, r, m;
} drone1;
struct IMUParameters
{
  uint8_t I2CAddr;
  float accSens, gyroSens;
  float accXOffset, accYOffset, accZOffset;
  float gyroXOffset, gyroYOffset, gyroZOffset;
  double gyroXDriftComp, gyroYDriftComp, gyroZDriftComp;
} IMU1;
struct accData
{
  float accXSc, accYSc, accZSc;
};
struct gyroData
{
  float gyroXSc, gyroYSc, gyroZSc;
};
struct angleData
{
  float rollAngle, pitchAngle, yawAngle;
} dA; // droneAngle
struct altimeterParameters
{
  struct ultrasound
  {
    uint32_t timeOut;
    uint16_t interval;
    uint16_t trigPulseLen;
    uint8_t pulsePinPE;
  } us;
  struct barometer
  {
  } bar;
} Alt1;
struct altitudeData
{
  struct ultrasound
  {
    int16_t echoPulseLen;
    float altitude;
    uint16_t dT;
  } us;
} aD;
struct MCUParameters
{
  struct Clock
  {
    uint32_t clkFrq;
    float clkPeriod;
  } CLK;
  struct Timer5
  {
    float OCRCompCLKS;
  } TMR5;
} MCU;
struct droneSetpoint
{
  float pitch, roll, yaw, altitude;
} dSP;
struct sysPIDS
{
  struct pitch
  {
    float P, I, D;
  } p;
  struct roll
  {
    float P, I, D;
  } r;
  struct yaw
  {
    float P, I, D;
  } y;
  struct altitude
  {
    float P, I, D;
  } a;
} mpid;
struct valPID
{
  float cP, cI, cIOld, cD;
  float sum;
};

// Movements
enum droneSetMove : char
{
  altitude = 'A',
  pitch = 'P',
  roll = 'R',
  yaw = 'Y'
};
enum pid
{
  alt = 0,
  p,
  r,
  y
};

// Physical constant
const float gravityAccel = 9.82;
const uint16_t soundSpeed = 340; // 340 m/s

/*
DistMatrixLiteral() will calculate the matrix vector product of the input torque-vector and the distribution matrix.
The function returns a pointer to motor speed vector.
*/
float *TorqueDistLiteral(float *torque, droneParameters *param)
{
  float distributionMatrix[4][4] = {
      {1 / (4 * param->Cl * param->r), -1 * (1 / (4 * param->Cl * param->r)), 1 / (4 * param->Cd), 1 / (4 * param->Cl * param->r)},
      {1 / (4 * param->Cl * param->r), 1 / (4 * param->Cl * param->r), -1 * (1 / (4 * param->Cd)), 1 / (4 * param->Cl * param->r)},
      {-1 * (1 / (4 * param->Cl * param->r)), 1 / (4 * param->Cl * param->r), 1 / (4 * param->Cd), 1 / (4 * param->Cl * param->r)},
      {-1 * (1 / (4 * param->Cl)), -1 * (1 / (4 * param->Cl)), -1 * (1 / (4 * param->Cd)), 1 * (1 / (4 * param->Cl))}};
  float Fg = param->m * gravityAccel;
  float forceOffsetVector[4] = {0, 0, 0, Fg}; // Gravity compensation in z-axis
  static float motorW[4];
  for (uint8_t i = 0; i < 4; i++)
  {
    for (uint8_t j = 0; j < 4; j++)
    {
      motorW[i] += (distributionMatrix[i][j] * ((*torque) + forceOffsetVector[j]));
      torque++;
    }
    torque -= 4;
  }

#if PRINTDISTMATRIX == 1
  // Print Distribution matrix
  Serial.print("Distribution Matrix start");
  Serial.println();
  for (uint8_t i = 0; i < 4; i++)
  {
    for (uint8_t j = 0; j < 4; j++)
    {
      Serial.print("[");
      Serial.print(distributionMatrix[i][j]);
      Serial.print("]");
      Serial.print(" ");
      if (j = 3)
      {
        Serial.println();
      }
    }
  }
  Serial.print("Distribution Matrix end");
  Serial.println();
#endif

#if PRINTMOTOROMEGA == 1
  // Print MotorSpeed vector
  Serial.print("Motor Omega start");
  Serial.println();
  for (uint8_t i = 0; i < 4; i++)
  {
    Serial.print("[");
    Serial.print(motorOmega[i]);
    Serial.print("]");
    Serial.println();
  }
  Serial.print("Motor Omega end");
  Serial.println();
#endif

  return &motorW[0];
}
/*
TestTorqueDistLiteral() will input pitch,roll,yaw and thrust into TorqueDistLiteral() and
print the contents of the returned 4-row vector from TorqueDistLiteral.
*/
void TestTorqueDistLiteral(float pitch, float roll, float yaw, float forceThrust)
{
  static float torque[4] = {0, 0, 0, 0};
  torque[0] = pitch;
  torque[1] = roll;
  torque[2] = yaw;
  torque[3] = forceThrust;
  float *vector = TorqueDistLiteral(&torque[0], &drone1);
  Serial.print("W1: ");
  Serial.print(*vector);
  Serial.println();
  vector++;
  Serial.print("W2: ");
  Serial.print(*vector);
  Serial.println();
  vector++;
  Serial.print("W3: ");
  Serial.print(*vector);
  Serial.println();
  vector++;
  Serial.print("W4: ");
  Serial.print(*vector);
  Serial.println();
}
void CalRPM(uint16_t intervalms)
{
  static uint32_t previousTime = 0;
  uint32_t currentTime;
  uint16_t interval = intervalms;

  currentTime = millis();
  if (currentTime - previousTime >= interval)
  {
    previousTime = currentTime;
    F1RPM = (tach1Count * 60) * (1000 / interval);
    tach1Count = 0;

    F2RPM = (tach2Count * 60) * (1000 / interval);
    tach2Count = 0;

    F3RPM = (tach3Count * 60) * (1000 / interval);
    tach3Count = 0;

    F4RPM = (tach4Count * 60) * (1000 / interval);
    tach4Count = 0;
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();

    Serial.print("F1 RPM:");
    Serial.print(F1RPM);
    Serial.println();
    Serial.print("F2 RPM:");
    Serial.print(F2RPM);
    Serial.println();
    Serial.print("F3 RPM:");
    Serial.print(F3RPM);
    Serial.println();
    Serial.print("F4 RPM:");
    Serial.print(F4RPM);
    Serial.println();
  }
}
void ISRTach1(void)
{
  tach1Count++;
}
void ISRTach2(void)
{
  tach2Count++;
}
void ISRTach3(void)
{
  tach3Count++;
}
void ISRTach4(void)
{
  tach4Count++;
}
void PrintDroneAngles(void)
{
  uint16_t interval = 1000;
  uint32_t currentTime = millis();
  static uint32_t previousTime = 0;

  if (currentTime - previousTime >= interval)
  {
    previousTime = currentTime;
    Serial.print("Pitch Angle: ");
    Serial.print(dA.pitchAngle);
    Serial.print("  Roll Angle: ");
    Serial.print(dA.rollAngle);
    Serial.print("  Yaw Angle: ");
    Serial.print(dA.yawAngle);
    Serial.println();
  }
}
/*
GetIMUAccel() will retrieve the raw X,Y,Z acceleration data from the IMU over I2C.
GetIMUAccel() will scale the raw X,Y,Z acceleration data with the correct value as
specified in IMUX.accSens. The value is 4096 for +/- 8g full-scale.
*/
accData *GetIMUAccel(void)
{
  // Read accelerometer registers
  static accData acc;
  static accData *accPtr = &acc;
  int16_t accX, accY, accZ;
  Wire.beginTransmission(IMU1.I2CAddr);
  Wire.write(0x3B); // 43
                    //  Wire.write(0x43);  //43
  Wire.endTransmission();
  Wire.requestFrom(IMU1.I2CAddr, 6, true); // Addr, 6 byes, release I2C bus efter
  accX = (Wire.read() << 8 | Wire.read());
  accY = (Wire.read() << 8 | Wire.read());
  accZ = (Wire.read() << 8 | Wire.read());
  // De skal skaleres med 4096 iflg. datasheet da der er valgt 8g full scale range.
  acc.accXSc = (((float)accX) / IMU1.accSens) - IMU1.accXOffset; // 4096.0;
  acc.accYSc = (((float)accY) / IMU1.accSens) - IMU1.accYOffset; // 4096.0;
  acc.accZSc = (((float)accZ) / IMU1.accSens) - IMU1.accZOffset; // 4096.0;
/*
  acc.accXSc = ((float)accX) * 0.061035;  // - IMU1.accXOffset;  //4096.0;
  acc.accYSc = ((float)accY) * 0.061035;  // - IMU1.accYOffset;  //4096.0;
  acc.accZSc = ((float)accZ) * 0.061035;  // - IMU1.accZOffset;  //4096.0;
*/
#if PRINTACCELDATA == 1
  delay(10);
  Serial.print("AccX:");
  Serial.print(acc.accXSc);

  Serial.print("  AccY: ");
  Serial.print(acc.accYSc);

  Serial.print("  AccZ: ");
  Serial.print(acc.accZSc);
  Serial.println();
#endif
  return accPtr;
}
/*
SetIMUAccelOffsets() should be called at system time t ~= 0. It will take an average of
the IMU acceleration data and update the IMUParameters.accNOffset variables with the
values necessary for a true zero value at startup.
*/
void SetIMUAccOffsets(IMUParameters *ptrIMU, uint16_t iterations)
{
  accData *acc;
  float accumX = 0, accumY = 0, accumZ = 0;
  for (int i = 0; i < iterations; i++)
  {
    acc = GetIMUAccel();
    accumX += acc->accXSc;
    accumY += acc->accYSc;
    accumZ += acc->accZSc;
  }
  ptrIMU->accXOffset = accumX / (float)iterations;
  ptrIMU->accYOffset = accumY / (float)iterations;
  ptrIMU->accZOffset = accumZ / (float)iterations;
#if PRINTACCOFFSETS == 1
  Serial.print("Accel X-Offset: ");
  Serial.print(ptrIMU->accXOffset);

  Serial.print("  Accel Y-Offset: ");
  Serial.print(ptrIMU->accYOffset);

  Serial.print("  Accel Z-Offset: ");
  Serial.print(ptrIMU->accZOffset);
  Serial.println();
  delay(10000);
#endif
}
/*
GetIMUGyro() will retrieve the raw X,Y,Z angular velocity data from the IMU over I2C.
GetIMUGyro() will scale the raw X,Y,Z angular velocity data with the correct value as
specified in IMUX.gyroSens. The value is 16.4 for +/- 2000 deg /s full-scale range.
*/
gyroData *GetIMUGyro(void)
{
  static gyroData gD;
  static gyroData *gdPtr = &gD;
  int16_t gyroX, gyroY, gyroZ;
  Wire.beginTransmission(IMU1.I2CAddr);
  Wire.write(0x43); // First gyro reg
  Wire.endTransmission();
  Wire.requestFrom(IMU1.I2CAddr, 6, true); // Addr, 6 byes, release I2C bus efter
  gyroX = (Wire.read() << 8 | Wire.read());
  gyroY = (Wire.read() << 8 | Wire.read());
  gyroZ = (Wire.read() << 8 | Wire.read());
  // De skal skaleres med gyroSens 16.4 (dvs 1/16.4) iflg. datasheet da der er valgt +/- 2000 deg/s FSR
  gD.gyroXSc = (((float)gyroX) / IMU1.gyroSens) - IMU1.gyroXOffset;
  gD.gyroYSc = (((float)gyroY) / IMU1.gyroSens) - IMU1.gyroYOffset;
  gD.gyroZSc = (((float)gyroZ) / IMU1.gyroSens) - IMU1.gyroZOffset;

#if PRINTGYRODATA == 1
  delay(10);
  Serial.print("gyroX: ");
  Serial.print(gD.gyroXSc);

  Serial.print("  gyroY: ");
  Serial.print(gD.gyroYSc);

  Serial.print("  gyroZ: ");
  Serial.print(gD.gyroZSc);
  Serial.println();
#endif
  return gdPtr;
}
/*
SetIMUGyroOffsets() should be called at system time t ~= 0. It will take an average of
the IMU gyroscopic data and update the IMUParameters.gyroNOffset variables with the
values necessary for a true zero gyro value at startup. This reduces gyroscopic drift.
*/
void SetIMUGyroOffsets(IMUParameters *ptrIMU, uint16_t iterations)
{
  gyroData *gD;
  float accumX = 0, accumY = 0, accumZ = 0;
  for (int i = 0; i < iterations; i++)
  {
    gD = GetIMUGyro();
    accumX += gD->gyroXSc;
    accumY += gD->gyroYSc;
    accumZ += gD->gyroZSc;
  }
  ptrIMU->gyroXOffset = accumX / (float)iterations;
  ptrIMU->gyroYOffset = accumY / (float)iterations;
  ptrIMU->gyroZOffset = accumZ / (float)iterations;
#if PRINTGYROOFFSETS == 1
  Serial.print("Gyro X-Offset: ");
  Serial.print(ptrIMU->gyroXOffset);

  Serial.print("  Gyro Y-Offset: ");
  Serial.print(ptrIMU->gyroYOffset);

  Serial.print("  Gyro Z-Offset: ");
  Serial.print(ptrIMU->gyroZOffset);
  Serial.println();
  delay(10000);
#endif
}
/*
CalAngleData() will compute the pitch, roll and yaw angles with the gyroscopic data recieved from GetIMUGyro().
It will calculate a new set of angles every 'interval'.

The function works by discrete time integrating the angular velocities from the IMU by dT to give actual angles.
The function will update the global dA struct with the angles.
*/
void SetAngleData(uint16_t interval)
{
  static uint32_t previousTime = 0;
  uint32_t currentTime = millis();
  uint32_t dt = 0;
  gyroData *gD;
  if (currentTime - previousTime >= interval)
  {
    // previousTime = currentTime;
    dt = currentTime - previousTime;
    gD = GetIMUGyro();                                  // Hent nye vinkelhastigheder
    dA.pitchAngle += (gD->gyroXSc * (float)dt * 0.001); //*0.001 because dt is in milliseconds and we want to convert it to seconds.
    dA.rollAngle += (gD->gyroYSc * (float)dt) * 0.001;  //
    dA.yawAngle += (gD->gyroZSc * (float)dt) * 0.001;   //
    previousTime = currentTime;
  }

#if PRINTDRONEORIENTATION == 1
  Serial.print("Pitch: ");
  Serial.print(dA.pitchAngle);
  Serial.print("  Roll: ");
  Serial.print(dA.rollAngle);
  Serial.print("  Yaw: ");
  Serial.print(dA.yawAngle);
  Serial.println();
#endif
}
/*
SetAltitudeData() will generate a trigger signal for the HCSR04 ultrasonic ranging module with the GenPulse() function
and measure the positive width of the returned square wave with the pulseIn() function. The altitude is computed by the
CalcAltitudeUS() function. The result is saved in the global aD struct.

This measurement is repeated at every SetAltidetude(uint16_t interval) interval.
*/
void SetAltitudeData(uint16_t interval)
{
#if USEULTRASOUND == 1
  static uint16_t previousMillis = 0;
  uint32_t pulseLenMeas = 0;
  uint16_t currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    GenPulse(Alt1.us.pulsePinPE, Alt1.us.trigPulseLen);       // Generate trigger signal for ultrasonic sensor
    pulseLenMeas = pulseIn(USECHOPIN, HIGH, Alt1.us.timeOut); // Alt1.us.timeOut);
    aD.us.altitude = CalcAltitudeUS(pulseLenMeas);
  }
#endif
#if PRINTULTRASOUNDALTITUDE == 1
  Serial.print("US Altitude: ");
  Serial.print(aD.us.altitude);
  Serial.print("  [m]");
  Serial.println();
#endif
}
/*
CalcAltitudeUS() will take the pulse length of the returned square wave and compute the distance measured
by using the formula provided in the datasheet for the SRHC04.

The speed of sound is ~340 m/s. (Depends on temperature, density of atmospheric air etc, but 340 m/s is a typical value at ).
*/
float CalcAltitudeUS(int16_t pulseLen)
{
  float altitude;
  altitude = ((pulseLen * 0.000001) * 343.21) / 2;
  return altitude;
}
/*
GenPulse() will generate a single one-shot high logic level output pulse on an arbitrary PORT E I/O pin.
The pulse will have the length specified in the pulseLenUS parameter (microseconds).

pulsePinPE refers to the PORT E pin. PORTE = 0b00000001 is pin 1 etc.

*/
void GenPulse(uint8_t pulsePinPE, uint16_t pulseLenUS)
{ // {
  /*
https://ww1.microchip.com/downloads/en/devicedoc/atmel-2549-8-bit-avr-microcontroller-atmega640-1280-1281-2560-2561_datasheet.pdf
Interrupt response time afsnit 7.8.1
Interrupt vector afsnit 14.1
16-bit timers afsnit 17
Block diagram figure 17-1
Using Timer5 in CTC mode.
Pulse length:
TMR5 CLK = 16MHz
Tclk = 1/16MHz = 62.5ns
For 10 us high output pulse
10us / 62.5 ns = 160
preload OCR5C with 160 for compare match

OCR5C = 160: 13.468 us pulse. The modules wastes some CLK cycles when switching to the ISR, loading the PORTE pin from the Alt1 struct and exiting the ISR.
Compensating OCR5C to get 10us pulse. It has been found through testing that the machine uses ~30 CLK cycles aka 30*62.5ns doing this. So this must be accounted for
when finding OCR5C. The Arduino uses several other timers. I suspect these are causing the issue as the Timer5 compare match interrupt vector has lower priority (#50) vs
f.eks Timer0 Compare match interrupt vector at #22 (Datasheet page 101). The interrupts are handled according to their program address and Timer0, Timer1 and Timer2 are higher up.

OCR5C will be found withCalcCMVal();

Only works on PORTE!
*/
  // Pre-Load TMR5 Compare Match Register to get desired pulse length.
  OCR5C = CalcCMVal(pulseLenUS); // Get Compare-match register value for desired pulse length
  // Clear counter 5.
  TCNT5 = 0;
  // Set Trig Pin High. Pulse Starts.
  PORTE |= 1 << pulsePinPE;
  TIMSK5 |= 0b00001000; // OCIE5C interrupt enabled
}
/*
CalcCMVal() will take the desired pulselength as input and calculate the value required for the Compare-Match OCR register to make it happen.
It will account for "wasted" CLK cycles by the MCU.
*/
uint16_t CalcCMVal(uint16_t pulseLenUS)
{
  uint16_t CMVal = 0;
  float pulseLenShifted = (float)pulseLenUS / 1000000.0; // Convert to microseconds
  // The factor we need to compensate with to get correct length output. OCRComp is the factor between the desired output and actual output.
  CMVal = ceil((pulseLenShifted - (MCU.TMR5.OCRCompCLKS * MCU.CLK.clkPeriod)) / MCU.CLK.clkPeriod);
  return CMVal;
}
/*
ISR(TIMER5_COMPC_vect) is used by the GenPulse function.
It will clear the pulsePinPE on TMR5 compare match interrupt.
*/
ISR(TIMER5_COMPC_vect)
{
  PORTE &= ~(1 << Alt1.us.pulsePinPE); // Clear porten
  /*
//TIMSK5 = TIMSK5 & ~(0b00001000); AVR disables interrupts in ISR's automatically just need to reenable to run again.
//TIFR5 = TIFR5| 0b00001000; //Læg flaget ned.. NVM Det sker automatisk på AVR
*/
}

void SerialPrintDroneState(altitudeData *ptraD, angleData *ptrdA, droneSetpoint *ptrdSP, uint16_t interval)
{
  static uint16_t previousMillis = 0;
  uint16_t currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    static char convertBuf[8], stringBuf[15];
    // Clear monitor
    for (uint8_t i = 0; i < 10; i++)
    {
      Serial.print("\n");
    }
    // SetPoint
    Serial.print("# SetPoint Values #\n");
    dtostrf(ptrdSP->pitch, 0, 2, convertBuf);
    sprintf(&stringBuf[0], "Angle-> P[ %s ]", convertBuf);
    Serial.print(stringBuf);
    dtostrf(ptrdSP->roll, 0, 2, convertBuf);
    sprintf(&stringBuf[0], " R[ %s ]", convertBuf);
    Serial.print(stringBuf);
    dtostrf(ptrdSP->yaw, 0, 2, convertBuf);
    sprintf(&stringBuf[0], " Y[ %s ]", convertBuf);
    Serial.print(stringBuf);
    dtostrf(ptrdSP->altitude, 0, 2, convertBuf);
    sprintf(&stringBuf[0], " Altitude-> L[ %s ]\n", convertBuf);
    Serial.print(stringBuf);
    // Measured
    Serial.print("# Measured Values #\n");
    dtostrf(ptrdA->pitchAngle, 0, 2, convertBuf);
    sprintf(&stringBuf[0], "Angle-> P[ %s ]", convertBuf);
    Serial.print(stringBuf);
    dtostrf(ptrdA->rollAngle, 0, 2, convertBuf);
    sprintf(&stringBuf[0], " R[ %s ]", convertBuf);
    Serial.print(stringBuf);
    dtostrf(ptrdA->yawAngle, 0, 2, convertBuf);
    sprintf(&stringBuf[0], " Y[ %s ]", convertBuf);
    Serial.print(stringBuf);
    dtostrf(ptraD->us.altitude, 0, 2, convertBuf);
    sprintf(&stringBuf[0], " Altitude-> L[ %s ]\n", convertBuf);
    Serial.print(stringBuf);
    previousMillis = currentMillis;
  }

  /*
static char altitudeString[30], angleSetPointString[50], altitudeSetPointString[30], angleDataString[50];
static char altiStrBuf[3], anglePitchStrBuf[8], angleRollStrBuf[8],angleYawStrBuf[8];
static char setPointPitchStrBuf[8], setPointRollStrBuf[8], setPointYawStrBuf[8], setPointAltitudeStrBuf[8];
static char stateStr[70];
char measData[] = "Sensor Measurements";
char spData[] = "SetPoint Values";
static uint16_t previousMillis = 0;
uint16_t currentMillis = millis();

  if (currentMillis - previousMillis >= 100) {
    previousMillis = currentMillis;
  dtostrf(ptraD->us.altitude, 3,2, altiStrBuf); //Arduino Serial.print() can't print sprintf floating points. Gotta convert to string type.
  sprintf(&altitudeString[0], "US Altitude->\tL[ %s ]\t", altiStrBuf);
  dtostrf(ptrdA->pitchAngle, 0,2, anglePitchStrBuf);
  dtostrf(ptrdA->rollAngle, 0,2, angleRollStrBuf);
  dtostrf(ptrdA->yawAngle, 0,2, angleYawStrBuf);
  sprintf(&angleDataString[0], "Gyro Angles->\t P[ %s ] R[ %s ] Y[ %s ]\n",  anglePitchStrBuf, angleRollStrBuf, angleYawStrBuf);

  dtostrf(ptrdSP->pitch, 0,2, setPointPitchStrBuf);
  dtostrf(ptrdSP->roll, 0,2, setPointRollStrBuf);
  dtostrf(ptrdSP->yaw, 0,2, setPointYawStrBuf);
  dtostrf(ptrdSP->altitude, 0,2, setPointAltitudeStrBuf);
  sprintf(&altitudeSetPointString[0], "SP Altitude->\tL[ %s ]", setPointAltitudeStrBuf );
  sprintf(&angleSetPointString[0], "\tSetP Angles->\t P[ %s ] R[ %s ] Y[ %s ]\n",  setPointPitchStrBuf, setPointRollStrBuf, setPointYawStrBuf);
  Serial.print(altitudeString);
  Serial.print(angleDataString);
  Serial.print(altitudeSetPointString);
  Serial.print(angleSetPointString);
  }
  */
}

void CheckSerialMove(void)
{
  char incData = 0;
  static float rxFloat = 0;

  if (Serial.available() > 0)
  {
    incData = Serial.read();
    rxFloat = Serial.parseFloat(SKIP_WHITESPACE);
  }
  switch (incData)
  {
  case altitude:
    Serial.print("Set Altitude: ");
    dSP.altitude = rxFloat;
    Serial.print(dSP.altitude);
    Serial.print("\n");
    break;
  case pitch:
    Serial.print("Set Pitch: ");
    dSP.pitch = rxFloat;
    Serial.print(dSP.pitch);
    Serial.print("\n");
    break;
  case roll:
    Serial.print("Set Roll: ");
    dSP.roll = rxFloat;
    Serial.print(dSP.roll);
    Serial.print("\n");
    break;
  case yaw:
    Serial.print("Set Yaw: ");
    dSP.yaw = rxFloat;
    Serial.print(dSP.yaw);
    Serial.print("\n");
    break;
  }
}

void MotorSetPoint(uint16_t intervalus)
{
  static uint16_t previousMicros = 0;
  uint16_t currentMicros = micros();
  static float *torque;
  static float *motorW;

  if (currentMicros - previousMicros >= intervalus)
  {
    float dT = (float)(currentMicros - previousMicros) * 0.000001; // convert to s
    torque = CalcPIDTorque(&dSP, &aD, &dA, dT);                    // Update PID loops
    motorW = TorqueDistLiteral(torque, &drone1);                   // Input PID torque to dist matrix
    SetESCPWM(motorW, 60);                                             // Update motors PWM
  }
}

float *CalcPIDTorque(droneSetpoint *ptrdSP, altitudeData *ptraD, angleData *ptrdA, float dT) 
{
  static float torque[4];
  static valPID vA, vP, vR, vY;
  vA.cIOld = 0, vP.cIOld = 0, vR.cIOld = 0, vY.cIOld = 0;
  static float prevAESig = 0, prevPESig = 0, prevRESig = 0, prevYESig = 0;
  float aESig = ptrdSP->altitude - ptraD->us.altitude; // Ultrasound
  float pESig = ptrdSP->pitch - ptrdA->pitchAngle;     // pitch
  float rESig = ptrdSP->roll - ptrdA->rollAngle;       // roll
  float yESig = ptrdSP->yaw - ptrdA->yawAngle;         // yaw
  for (uint8_t i = alt; i <= yaw; i++)
  {
    switch (i)
    {
    case alt: // Altitude PID
      vA.cP = CalcP(aESig, mpid.a.P);
      vA.cI = CalcI(dT, aESig, vA.cIOld);
      vA.cIOld = vA.cI;
      vA.cI *= mpid.a.I;
      vA.cD = CalcD(dT, aESig, mpid.a.D, prevAESig);
      prevAESig = aESig;
      vA.sum = vA.cP + vA.cI + vA.cD;
      break;
    case p: // Pitch PID
      vP.cP = CalcP(pESig, mpid.p.P);
      vP.cI = CalcI(dT, pESig, vP.cIOld);
      vP.cIOld = vP.cI;
      vP.cI *= mpid.p.I;
      vP.cD = CalcD(dT, pESig, mpid.p.D, prevPESig);
      prevPESig = pESig;
      vP.sum = vP.cP + vP.cI + vP.cD;
      break;
    case r: // roll PID
      vR.cP = CalcP(rESig, mpid.r.P);
      vR.cI = CalcI(dT, rESig, vR.cIOld);
      vR.cIOld = vR.cI;
      vR.cI *= mpid.r.I;
      vR.cD = CalcD(dT, rESig, mpid.r.D, prevRESig);
      prevPESig = rESig;
      vR.sum = vR.cP + vR.cI + vR.cD;
      break;
    case y: // yaw PID
      vY.cP = CalcP(yESig, mpid.y.P);
      vY.cI = CalcI(dT, yESig, vY.cIOld);
      vY.cIOld = vY.cI;
      vY.cI *= mpid.y.I;
      vY.cD = CalcD(dT, yESig, mpid.y.D, prevYESig);
      prevPESig = yESig;
      vY.sum = vY.cP + vY.cI + vY.cD;
      break;
    }
  }
  torque[0] = vP.sum;
  torque[1] = vR.sum;
  torque[2] = vY.sum;
  torque[3] = vA.sum;
  return &torque[0];
}
float CalcP(float eSig, float P)
{
  float cP = eSig * P;
  return cP;
}
float CalcI(float dT, float eSig, float cIOld)
{
  float cI = ((eSig * dT) + cIOld);
  return cI;
}
float CalcD(float dT, float eSig, float D, float prevESig)
{
  float cD = ((eSig - prevESig) / dT) * D;
  return cD;
}

void SetESCPWM(float *motorW, uint16_t offset)
{
  analogWrite(M1, *motorW + offset);
  motorW++;
  analogWrite(M2, *motorW + offset);
  motorW++;
  analogWrite(M3, *motorW + offset);
  motorW++;
  analogWrite(M4, *motorW + offset);
}

void BootESC(){
  uint8_t motorPins[4] = {M1, M2, M3, M4};
  Serial.print("ESC booting..\n");
  analogWrite(M1, 120);
  analogWrite(M2, 120);
  analogWrite(M3, 120);
  analogWrite(M4, 120);
  delay(5000);
  for(uint8_t i = 0; i < 4; i++){
    analogWrite(motorPins[i], 150);
    delay(2000);
    analogWrite(motorPins[i], 130);
  }
  Serial.print("ESC boot complete\n");
  
}

void setup()
{
  Serial.begin(115200);
  // Load MCU Struct with relevant data.
  MCU.CLK.clkFrq = 16000000;
  MCU.CLK.clkPeriod = 1 / (float)MCU.CLK.clkFrq;
  // Default setpoint
  dSP.altitude = 0, dSP.pitch = 0, dSP.roll = 0, dSP.yaw = 0;
  // Set PID vals
  // Set PID Params for Altitude
  mpid.a.P = 1.5;
  mpid.a.I = 0.1;
  mpid.a.D = 0.001;
  // Set PID Params for Pitch
  mpid.p.P = 1.5;
  mpid.p.I = 0.1;
  mpid.p.D = 0.001;
  // Set PID Params for Roll
  mpid.r.P = 1.5;
  mpid.r.I = 0.1;
  mpid.r.D = 0.001;
  // Set PID Params for Yaw
  mpid.y.P = 1.5;
  mpid.y.I = 0.1;
  mpid.y.D = 0.001;

#if USEI2C == 0
  pinMode(F1Tach, INPUT_PULLUP);
  pinMode(F2Tach, INPUT_PULLUP);
#endif
  pinMode(F3Tach, INPUT_PULLUP);
  pinMode(F4Tach, INPUT_PULLUP);
  pinMode(F1PWM, OUTPUT);
  pinMode(F2PWM, OUTPUT);
  pinMode(F3PWM, OUTPUT);
  pinMode(F4PWM, OUTPUT);
  analogWrite(F1PWM, 0);
  analogWrite(F2PWM, 0);
  analogWrite(F3PWM, 0);
  analogWrite(F4PWM, 0);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  digitalWrite(M3, LOW);
  digitalWrite(M4, LOW);

#if USETACHOMETERS == 1
  attachInterrupt(digitalPinToInterrupt(F1Tach), ISRTach1, FALLING);
  attachInterrupt(digitalPinToInterrupt(F2Tach), ISRTach2, FALLING);
  attachInterrupt(digitalPinToInterrupt(F3Tach), ISRTach3, FALLING);
  attachInterrupt(digitalPinToInterrupt(F4Tach), ISRTach4, FALLING);
#endif;
  // Set drone parameters
  drone1.Cl = 1.0;
  drone1.Cd = 1.0;
  drone1.r = 1.0;
  drone1.m = 1.0;

#if ACTIVATEIMU1 == 1
  // Load relevant struct
  IMU1.I2CAddr = 0x68;
  IMU1.accXOffset = 0, IMU1.accYOffset = 0, IMU1.accZOffset = 0;
  IMU1.gyroXOffset = 0, IMU1.gyroYOffset = 0, IMU1.gyroZOffset = 0;
  // MPU6050 Setup
  Wire.setClock(400000); // I2C Fast-Mode
  // Reset MPU
  Wire.begin();
  Wire.beginTransmission(IMU1.I2CAddr);
  Wire.write(0x6B); // Power management register
  Wire.write(00);
  Wire.endTransmission();
  // Set Accelerometer sensitivity +/- 8g full scale
  Wire.beginTransmission(IMU1.I2CAddr);
  Wire.write(0x1C);
  Wire.write(0x10); //+/- 8g
  Wire.endTransmission();
  IMU1.accSens = 4096.0;
  // Set LPF
  Wire.beginTransmission(IMU1.I2CAddr); // default value for MPU-6050 er 0x68 for the I2C adress
  Wire.write(0x1A);                     // Low pass filter
  Wire.write(0x05);                     // Setting up filter for 10Hz
  Wire.endTransmission();
  // Set Gyro Sensitivty
  Wire.beginTransmission(IMU1.I2CAddr);
  Wire.write(0x1B); // Sensitivity scale factor
  // Wire.write(0x16);   //Setting the sensitivity
  Wire.write(0b11000); // Full Scale Range = 2000 deg/s - 16.4 LSB/deg/s
  Wire.endTransmission();
  IMU1.gyroSens = 16.4; /*16.4 because FSR is 2000 deg/s as per IMU6050 datasheet.*/
  // zero out accelerometer and Gyroscope
  SetIMUAccOffsets(&IMU1, 10);
  SetIMUGyroOffsets(&IMU1, 1000);
#endif

#if USEULTRASOUND == 1
  /*
  HC-SR04 Ultrasonic ranging module:
  https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf?_gl=1*1dvzhg5*_ga*NTkzOTIwNS4xNjg4NDk3NDc0*_ga_T369JS7J9N*MTY4ODY1OTc3OC4zLjAuMTY4ODY1OTc3OC4wLjAuMA..
  */
  Alt1.us.interval = 1;      // 1 ms
  Alt1.us.timeOut = 24000;   // 24ms (24000 us for pulseIn()). maximum possible sensor range is 4m. Timeout period should be: 4 = (t*340)/2 -> t > 0.023529 [s]
  Alt1.us.trigPulseLen = 10; // 10 us
  Alt1.us.pulsePinPE = 3;    // PORTE pin 3 == Arduino Mega pin 5.
  aD.us.altitude = 0;
  MCU.TMR5.OCRCompCLKS = 30;  // This value was found experimentally. See GenPulse()
  pinMode(USTRIGPIN, OUTPUT); // Same as PulsePin. USTRIGPIN = PIN 3 on PORTE
  digitalWrite(USTRIGPIN, LOW);
  pinMode(USECHOPIN, INPUT);
  // Timer5 setup for GenPulse();
  // Set Timer5 to CTC mode of operation.
  TCCR5A = 0;
  TCCR5B = 0;
  TCCR5B = TCCR5B | 0b00001000; // WGM52 = 1 CTC mode
  TCCR5B = TCCR5B | 0b00000001;

#endif
  Serial.print("IM ALIVE!! \n");
  BootESC();
}

void loop()
{
  SetAngleData(1);
  SetAltitudeData(100);
 // CheckSerialMove();
 // MotorSetPoint(500);

//PWM skal være 120 først
/*
analogWrite(M1, 150); //M1 150 min efter start
analogWrite(M2, 150); //M2 150 min efter start
analogWrite(M3, 130); //M2 150 min efter start
analogWrite(M4, 150); //M2 150 min efter start
*/

#if PRINTDRONESTATUS == 1
  SerialPrintDroneState(&aD, &dA, &dSP, 1000);
#endif
#if PRINTDRONEANGLES == 1
  PrintDroneAngles();
#endif
#if USETACHOMETERS == 1
  CalRPM(1000);
#endif
}

/*


int add(int a, int b){return a+b;}
int mult(int a, int b){return a*b;}
int do_operation(int (*op)(int, int), int x, int y){
  return op(x,y);
}

  int result = do_operation(add, 5,2);
  Serial.print("Result is: \t");
  Serial.print(result);


*/
