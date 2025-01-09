// Global variables
int i = 0;                // counter
String matlabStr = "";    // receives the string from matlab, it is empty at first
bool readyToSend = false; // flag to indicate a command was received and now ready to send back to matlab

char c;                   // characters received from matlab
float val1 = 0.0;         // input1 from matlab
float val2 = 0.0;         // input2 from matlab

// Define constants for pulses per revolution (PPR), gear ratio (GR), and cycles per revolution (CPR)
const float PPR = 3575.0855;
const float GR = 50;
const float CPR = 3; // Cycles per revolution

// Variables for tracking encoder positions
long counter_m1 = 0;
long counter_m2 = 0;
int aLastState_m1;
int aLastState_m2;

// Pin definitions
const int encoderPinA_m1 = 2;
const int encoderPinB_m1 = 10;
const int encoderPinA_m2 = 3;
const int encoderPinB_m2 = 11;

const int motorPin1_m1 = 4;
const int motorPin2_m1 = 5;
const int motorPin1_m2 = 7;
const int motorPin2_m2 = 8;

const int enablePin_m1 = 6;
const int enablePin_m2 = 9;

// Variables for encoder positions and desired positions
long currentPosition_m1 = 0;
long currentPosition_m2 = 0;
float demandPositionInDegrees_m1 = 0;
float demandPositionInDegrees_m2 = 0;
float currentPositionInDegrees_m1;
float currentPositionInDegrees_m2;

// Time parameters
unsigned long currentTime;
unsigned long previousTime = 0;
float deltaT;

// PID gains
float Kp_m1 = 5.2, Kd_m1 = 0.13, Ki_m1 = 0.015;
float Kp_m2 = 7.5, Kd_m2 = 0.1, Ki_m2 = 0.18;

// Error values
float errorPositionInDegrees_prev_m1 = 0, errorPositionInDegrees_sum_m1 = 0;
float errorPositionInDegrees_prev_m2 = 0, errorPositionInDegrees_sum_m2 = 0;

// Communication variables
//bool readyToSend = false; 

void setup() {
  Serial.begin(115200);

  pinMode(encoderPinA_m1, INPUT_PULLUP);
  pinMode(encoderPinA_m2, INPUT_PULLUP);
  pinMode(encoderPinB_m1, INPUT_PULLUP);
  pinMode(encoderPinB_m2, INPUT_PULLUP);

  pinMode(motorPin1_m1, OUTPUT);
  pinMode(motorPin2_m1, OUTPUT);
  pinMode(enablePin_m1, OUTPUT);
  pinMode(motorPin1_m2, OUTPUT);
  pinMode(motorPin2_m2, OUTPUT);
  pinMode(enablePin_m2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA_m1), updateEncoder_m1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_m2), updateEncoder_m2, CHANGE);

  aLastState_m1 = digitalRead(encoderPinA_m1);
  aLastState_m2 = digitalRead(encoderPinA_m2);

  delay(3000);
  previousTime = micros();
}

void loop() {
  // Communication handling
  if (!readyToSend) {
    receiveMatlabData();
  } else {
    sendArduinoData();
  }

  // Update positions
  currentPositionInDegrees_m1 = (counter_m1 * 360) / (CPR * GR * 2);
  currentPositionInDegrees_m2 = (counter_m2 * 360) / (CPR * GR * 2);

  boundPosition(counter_m1, currentPositionInDegrees_m1);
  boundPosition(counter_m2, currentPositionInDegrees_m2);

  // PID control
  currentTime = micros();
  deltaT = (currentTime - previousTime) / 1e6; // Convert to seconds
  if (deltaT >= 0.04) { // Run at ~25 Hz
    controlMotor(1, demandPositionInDegrees_m1, currentPositionInDegrees_m1, Kp_m1, Ki_m1, Kd_m1, errorPositionInDegrees_prev_m1, errorPositionInDegrees_sum_m1);
    controlMotor(2, demandPositionInDegrees_m2, currentPositionInDegrees_m2, Kp_m2, Ki_m2, Kd_m2, errorPositionInDegrees_prev_m2, errorPositionInDegrees_sum_m2);

    previousTime = currentTime;
  }
}

// Helper functions
void receiveMatlabData() {
  while (Serial.available() > 0) {
    c = Serial.read();
    matlabStr += c;

    if (matlabStr.indexOf(";") != -1) {
      int posComma = matlabStr.indexOf(",");
      val1 = matlabStr.substring(1, posComma).toFloat();
      demandPositionInDegrees_m1 = val1;
      
      int posEnd = matlabStr.indexOf(";");
      val2 = matlabStr.substring(posComma + 1, posEnd).toFloat();
      demandPositionInDegrees_m2 = val2;
      
      matlabStr = "";
      readyToSend = true;
    }
  }
}

void sendArduinoData() {
  Serial.print("c");
  Serial.print(currentPositionInDegrees_m1 / 30);
  Serial.print(",");
  Serial.print(currentPositionInDegrees_m2 / 30);
  Serial.write(13);  // carriage return
  Serial.write(10);  // newline
}

void boundPosition(long &counter, float &position) {
  if (position >= 360 || position <= -360) {
    counter -= (GR * CPR * 2) * (int)(position / 360);
  }
}

void controlMotor(int motor, float demand, float current, float Kp, float Ki, float Kd, float &errorPrev, float &errorSum) {
  float error = demand - current;
  errorSum += error;
  float errorDiff = (error - errorPrev) / deltaT;
  errorPrev = error;

  float output = Kp * error + Ki * errorSum * deltaT + Kd * errorDiff;
  output = constrain(output, -255, 255);

  int motorPin1 = (motor == 1) ? motorPin1_m1 : motorPin1_m2;
  int motorPin2 = (motor == 1) ? motorPin2_m1 : motorPin2_m2;
  int enablePin = (motor == 1) ? enablePin_m1 : enablePin_m2;

  if (output > 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    analogWrite(enablePin, output);
  } else {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin, -output);
  }
}

void updateEncoder_m1() {
  int aState = digitalRead(encoderPinA_m1);
  if (aState != aLastState_m1) {
    counter_m1 += (digitalRead(encoderPinB_m1) != aState) ? 1 : -1;
    aLastState_m1 = aState;
  }
}

void updateEncoder_m2() {
  int aState = digitalRead(encoderPinA_m2);
  if (aState != aLastState_m2) {
    counter_m2 += (digitalRead(encoderPinB_m2) != aState) ? 1 : -1;
    aLastState_m2 = aState;
  }
}
