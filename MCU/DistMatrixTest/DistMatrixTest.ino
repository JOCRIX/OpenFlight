#define PRINTDISTMATRIX 0
#define PRINTMOTOROMEGA 0

#define F1Tach 21
#define F2Tach 20
#define F3Tach 19
#define F4Tach 18

#define F1PWM 6
#define F2PWM 7
#define F3PWM 8
#define F4PWM 9

volatile uint32_t tach1Count, tach2Count, tach3Count, tach4Count;
uint16_t F1RPM, F2RPM, F3RPM, F4RPM;
struct droneParameters {
  float Cl, Cd, r, m;
} drone1;
const float gravity = 9.82;
//droneParameters *dPtr = &drone1;
//struct

//drone1.Cl = 1.0;

//;, droneOne.Cd = 1, droneOne.r = 1;

/*
DistMatrixLiteral will calculate the matrix vector product of the input torque-vector and the distribution matrix.
The function returns a pointer to motor speed vector.
*/
float *TorqueDistLiteral(float *torque, droneParameters *param) {
  float distributionMatrix[4][4] = {
    { 1 / (4 * param->Cl * param->r), -1 * (1 / (4 * param->Cl * param->r)), 1 / (4 * param->Cd), 1 / (4 * param->Cl * param->r) },
    { 1 / (4 * param->Cl * param->r), 1 / (4 * param->Cl * param->r), -1 * (1 / (4 * param->Cd)), 1 / (4 * param->Cl * param->r) },
    { -1 * (1 / (4 * param->Cl * param->r)), 1 / (4 * param->Cl * param->r), 1 / (4 * param->Cd), 1 / (4 * param->Cl * param->r) },
    { -1 * (1 / (4 * param->Cl)), -1 * (1 / (4 * param->Cl)), -1 * (1 / (4 * param->Cd)), 1 * (1 / (4 * param->Cl)) }
  };
  float Fg = param->m * gravity;
  float forceOffsetVector[4] = { 0, 0, 0, Fg };  //Gravity compensation in z-axis
  static float motorOmega[4];
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      motorOmega[i] += (distributionMatrix[i][j] * ((*torque) + forceOffsetVector[j]));
      torque++;
    }
    torque -= 4;
  }

#if PRINTDISTMATRIX == 1
  //Print Distribution matrix
  Serial.print("Distribution Matrix start");
  Serial.println();
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      Serial.print("[");
      Serial.print(distributionMatrix[i][j]);
      Serial.print("]");
      Serial.print(" ");
      if (j = 3) {
        Serial.println();
      }
    }
  }
  Serial.print("Distribution Matrix end");
  Serial.println();
#endif

#if PRINTMOTOROMEGA == 1
  //Print MotorSpeed vector
  Serial.print("Motor Omega start");
  Serial.println();
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print("[");
    Serial.print(motorOmega[i]);
    Serial.print("]");
    Serial.println();
  }
  Serial.print("Motor Omega end");
  Serial.println();
#endif

  return &motorOmega[0];
}

/*
TestTorqueDistLiteral() will print the contents of the returned 4-row vector from TorqueDistLiteral.
*/
void TestTorqueDistLiteral() {
  static float torque[4] = { 0, 0, 0, 0 };
  float *vector = TorqueDistLiteral(&torque[0], &drone1);

  /*
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
  */
}

void CalRPM(uint16_t intervalms) {
  static uint32_t previousTime = 0;
  uint32_t currentTime;
  uint16_t interval = intervalms;

  currentTime = millis();
  if (currentTime - previousTime >= interval) {
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

void ISRTach1() {
  tach1Count++;
}
void ISRTach2() {
  tach2Count++;
}
void ISRTach3() {
  tach3Count++;
}
void ISRTach4() {
  tach4Count++;
}

void setup() {
  Serial.begin(115200);
  pinMode(F1Tach, INPUT_PULLUP);
  pinMode(F2Tach, INPUT_PULLUP);
  pinMode(F3Tach, INPUT_PULLUP);
  pinMode(F4Tach, INPUT_PULLUP);
  pinMode(F1PWM, OUTPUT);
  pinMode(F2PWM, OUTPUT);
  pinMode(F3PWM, OUTPUT);
  pinMode(F4PWM, OUTPUT);

  analogWrite(F1PWM, 50);
  analogWrite(F2PWM, 50);
  analogWrite(F3PWM, 50);
  analogWrite(F4PWM, 50);

  attachInterrupt(digitalPinToInterrupt(F1Tach), ISRTach1, FALLING);
  attachInterrupt(digitalPinToInterrupt(F2Tach), ISRTach2, FALLING);
  attachInterrupt(digitalPinToInterrupt(F3Tach), ISRTach3, FALLING);
  attachInterrupt(digitalPinToInterrupt(F4Tach), ISRTach4, FALLING);
  drone1.Cl = 1.0;
  drone1.Cd = 1.0;
  drone1.r = 1.0;
  drone1.m = 1.0;

  TestTorqueDistLiteral();
}

void loop() {
  //CalRPM(1000);
}
