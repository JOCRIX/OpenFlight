#define EncPhaseA 2
#define EncPhaseB 3
#define CW 1
#define CCW 2
#define MotorPWM 9
#define SweepUp 1
#define SweepDown 2
#define PotMeter A5

volatile uint8_t phAFall = 0;
uint32_t encCount = 0;
uint32_t *pencCount = &encCount;
double rpm = 0;
double rpsMeasured = 0;
uint8_t dir = 0;
uint8_t newStatus = 0;
double setPoint;
/*
double pVal = 21.8; //42
double iVal = 8.5;
double dVal = 3.5;//3.5;
*/
//https://pidexplained.com/how-to-tune-a-pid-controller/
double pVal = 15.5; //72/2 36
double iVal = 6.1;
double dVal = 1.2;//3.5;
//
double error;
double pCalDebug, iCalDebug, dCalDebug;
double PIDSum;
double PIDDeltaT;
uint8_t pwm;


void EncISR() {
  phAFall = 1;
}

void CheckEncPhase() {
  if (phAFall == 1 && digitalRead(EncPhaseB)) {
    dir = CCW;
    encCount++;
    newStatus = 1;
    phAFall = 0;
  } else if (phAFall == 1 && !digitalRead(EncPhaseB)) {
    dir = CW;
    encCount++;
    newStatus = 1;
    phAFall = 0;
  }
}

void PrintDir(uint8_t dir, uint16_t rpm) {
  if (newStatus == 1) {
    if (dir == CW) {
      Serial.print("CW");
      Serial.print("  Count: ");
      Serial.print(encCount);
      Serial.print("  RPM: ");
      Serial.print(rpm);
      Serial.println();
    } else if (dir == CCW) {
      Serial.print("CCW");
      Serial.print("  Count: ");
      Serial.print(encCount);
      Serial.print("  RPM: ");
      Serial.print(rpm);
      Serial.println();
    }
    newStatus = 0;
  }
}

void CalcRPS(uint32_t *encCount) {
  static uint32_t previousms;
  static const uint16_t interval = 100;
  static const uint8_t pulsePrRev = 200;
  uint32_t nowms = millis();
  double deltat;

  if (nowms - previousms >= interval) {
    deltat = (double)(nowms - previousms) / 1000;

    rpsMeasured = ((double)*encCount / (double)pulsePrRev) / deltat;

    *encCount = 0;
    previousms = nowms;
  }
}

void PrintStatus(uint8_t monitorOrPlot) {

  uint16_t interval = 500;
  uint32_t nowms;
  static uint32_t previousms;
  double ub = 4, lb = 0;
  nowms = millis();
  if (nowms - previousms >= interval) {
    if(monitorOrPlot == 0){
    // Serial.println();
    //Serial.print("Upper Bound: ");
    // Serial.print(ub);
    Serial.println();
    Serial.println();


    Serial.print("SetPoint RPS: ");
    Serial.print(setPoint);


    Serial.print("  RPS Measured: ");
    Serial.print(rpsMeasured);


    Serial.print("  Error Signal: ");
    Serial.print(error);

    Serial.println();
    Serial.print("pCal: ");
    Serial.print(pCalDebug);
    Serial.print("  iCal: ");
    Serial.print(iCalDebug);
    Serial.print("  dCal: ");
    Serial.print(dCalDebug);
    Serial.print("  PID: ");
    Serial.print(PIDSum);
    Serial.println();
    Serial.print("PID Delta T: ");
    Serial.print(PIDDeltaT);
    Serial.print("  PWM: ");
    Serial.print(pwm);

    Serial.println();
    Serial.println();
    //  Serial.print("Lower Bound: ");
    // Serial.print(lb);
    //  Serial.println();
    }
    else{
    Serial.print("P:");
    Serial.print(pCalDebug);
    Serial.println();
    Serial.print("I:");
    Serial.print(iCalDebug);
    Serial.println();
    Serial.print("D:");
    Serial.print(dCalDebug);
    Serial.println();
    Serial.print("PID:");
    Serial.print(PIDSum);
    Serial.println();
    Serial.print("SetPoint RPS:");
    Serial.print(setPoint);
    Serial.println();
    Serial.print("RPS Measured:");
    Serial.print(rpsMeasured);
    Serial.println();
    Serial.print("Error Signal:");
    Serial.print(error);
    Serial.println();
    Serial.print("Upper Limit:");
    Serial.print(4);
    Serial.println();
    Serial.print("Lower Limit:");
    Serial.print(0);
    Serial.println();
    }
    previousms = nowms;
  }
}

void SetPoint() {
  uint16_t pwmVal = analogRead(PotMeter) / 4;
  //analogWrite(MotorPWM, pwmVal);
  setPoint = (3.55 / 255) * ((double)pwmVal);  //3.55RPS = max ved 12.1V
  // Serial.print("SetPoint:");
  // Serial.println(setPoint);
  if (setPoint < 0.05){
    setPoint = 0;
  }
  //Serial.println();
}

void SweepPWM(uint8_t pin, uint16_t incrementms) {
  static uint8_t sweepState = SweepUp;
  static uint32_t previousms;
  uint32_t nowms = millis();
  static uint8_t pwmVal = 0;

  if (sweepState == SweepUp) {
    if (nowms - previousms >= incrementms) {
      previousms = nowms;
      if (pwmVal < 255) {
        pwmVal++;
        analogWrite(pin, pwmVal);
      } else {
        sweepState = SweepDown;
      }
    }
  } else if (sweepState == SweepDown) {
    if (nowms - previousms >= incrementms) {
      previousms = nowms;
      if (pwmVal > 0) {
        pwmVal--;
        analogWrite(pin, pwmVal);
      } else {
        sweepState = SweepUp;
      }
    }
  }
}

void PID() {
  static double previousError;
  double errorSig = 0.0;
  static double iCalOld = 0;
  double iCal = 0, pCal = 0, dCal = 0;
  static const uint16_t interval = 100;
  static uint32_t previousms;
  uint32_t nowms = millis();
  static double deltat;
  static double pidOutSum;

  if (nowms - previousms >= interval) {

    deltat = (double)(nowms - previousms) / 1000.0;
    errorSig = setPoint - rpsMeasured;

    //pCal = errorSig * pVal;
    pCal = errorSig;

    iCal = (((errorSig * deltat)) + iCalOld); //* iVal; 
    iCalOld = iCal;
/*
    if (iCal > 100){
      iCal = 100;
    }
 */
    dCal = (((errorSig - previousError) / deltat));// * dVal;

    previousError = errorSig;
    pidOutSum = ((pCal * pVal) + (iCal * iVal) + (dCal * dVal));  //pCal + iCal + dCal;  //((pCal * p) + (iCal * i) + (dCal * d));
    //pidOutSum = ((pCal + dCal) + iCal);
   
    //Til print
    error = errorSig;
    pCalDebug = pCal;
    iCalDebug = iCal;
    dCalDebug = dCal;
    PIDSum = pidOutSum;
    PIDDeltaT = deltat;
    //
/*
  if (pidOutSum > 255){
    pidOutSum = 255;
  }*/
    SetMotorPWM(pidOutSum);
    previousms = nowms;
  }
}

void SetMotorPWM(double pidOutSum) {
  
  if (pidOutSum < 0){
    pidOutSum *= -1;
  }

  uint8_t pwmVal = (uint8_t)pidOutSum;// * (255/3.55); //+ 50;  //(255/3.55);

  if (pwmVal > 255) {
    pwmVal = 255;
  } else if (pwmVal < 0) {
    pwmVal = 0;
  }
  pwm = pwmVal;
  analogWrite(MotorPWM, pwmVal);
}


void setup() {
  Serial.begin(115200);
  pinMode(EncPhaseA, INPUT_PULLUP);
  pinMode(EncPhaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(EncPhaseA), EncISR, FALLING);
  pinMode(MotorPWM, OUTPUT);
  digitalWrite(MotorPWM, 0);
}

void loop() {
  CheckEncPhase();
  SetPoint();
  CalcRPS(pencCount);
  PID();
  PrintStatus(1);
  //analogWrite(MotorPWM, 255);
  // SweepPWM(MotorPWM, 50);
  //PrintStatus(dir, rpm);
}
