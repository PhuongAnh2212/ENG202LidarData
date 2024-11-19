
// Motor control pins
const int enL = 7;   // PWM pin for left motor
const int inL1 = 8;  // Motor direction pin 1 for left motor
const int inL2 = 9;  // Motor direction pin 2 for left motor
const int enR = 12;  // PWM pin for right motor
const int inR1 = 10; // Motor direction pin 1 for right motor
const int inR2 = 11; // Motor direction pin 2 for right motor

// Encoder pins
const int enLA = 2;  // Encoder A pin for left motor
const int enLB = 3;  // Encoder B pin for left motor
const int enRA = 18; // Encoder A pin for right motor
const int enRB = 19; // Encoder B pin for right motor

// Control constants
const float d = 0.189;    // Distance between wheels (meters)
const float r = 0.0225; // Radius of the wheels (meters)
const float T = 1.2;      // Time step (seconds)
const float h = 0.21;     // Offset in angle calculation

// Encoder constantss
const int encoderCountsPerRevolution = 350; // 1:50 encoder with 7 pulses per revolution
const float wheelDiameter = 0.0225*2; // Diameter of the wheel in meters
//const float M_PI = 3.14159265358979323846;
const float wheelCircumference = wheelDiameter * M_PI ; // Wheel circumference

// Robot state variables
float x = 0.0;     // Current X position (meters)
float y = 0.0;     // Current Y position (meters)
float theta = 0.0; // Current orientation (radians)

// Encoder counts and speed control
volatile int leftEnCount = 0;
volatile int rightEnCount = 0;
const int K = 30;  // Adjustment factor for speed control

unsigned long previousMillis = 0;
const long interval = 100;
bool isMoving = false; 
void setup() {
  Serial.begin(115200);

  // Setup encoder interrupts
  attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enLB), leftEnISRB, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);

  // Set all the motor control pins to outputs
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval){
    previousMillis = currentMillis;
    if (isMoving){
      calculateCoordinates();
      printCoordinates();
    }
  }
  if (Serial.available()> 0){
    char command = Serial.read();
    if (command == '2'){
      moveForward();
    }
    else if (command == '3'){
      moveBackward();
    }
    else if(command =='4'){
      turnLeft();
    }
    else if(command =='5'){
      turnRight();
    }
    else if(command =='0'){
      stop();
    }
  }
  delay(10);
}

// Function to get the current speed of the left wheel
float get_speedL() {
  // Convert encoder counts to speed in m/s
  float countsPerSecond = leftEnCount / T; // Counts per second
  float speedRPM = (countsPerSecond * 60) / encoderCountsPerRevolution; // RPM
  float speedMetersPerSecond = speedRPM * (wheelCircumference / 60); // m/s
  leftEnCount = 0; // Reset count after reading
  return speedMetersPerSecond;
}

// Function to get the current speed of the right wheel
float get_speedR() {
  // Convert encoder counts to speed in m/s
  float countsPerSecond = rightEnCount / T; // Counts per second
  float speedRPM = (countsPerSecond * 60) / encoderCountsPerRevolution; // RPM
  float speedMetersPerSecond = speedRPM * (wheelCircumference / 60); // m/s
  rightEnCount = 0; // Reset count after reading
  return speedMetersPerSecond;
}

void moveForward() {
    analogWrite(enR, 60);
    analogWrite(enL, 60);
    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, HIGH);
    isMoving = true;
}

void moveBackward(){
    analogWrite(enR, 60);
    analogWrite(enL, 60);
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, HIGH);
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW);
    isMoving = true; 
}
void turnLeft(){
   analogWrite(enR, 90);
   analogWrite(enL, 50);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
  isMoving = true;
  
}
void turnRight() {
  analogWrite(enR, 50);
  analogWrite(enL, 90);
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
  isMoving =true;
}
// Encoder ISR for left wheel
void leftEnISRA() {
  leftEnCount++;
}

void leftEnISRB() {
  leftEnCount++;
}

// Encoder ISR for right wheel
void rightEnISRA() {
  rightEnCount++;
}

void rightEnISRB() {
  rightEnCount++;
}

// Stop the motors
void stop() {
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  isMoving = false; 
}

void calculateCoordinates(){
  float v1 = get_speedL();
  float v2 = get_speedR();
  x += (v1 + v2) / 2.0 * cos(theta) * T;
  y += (v1 + v2) / 2.0 * sin(theta) * T;
  theta += (v2 - v1) / d * T;

}
void printCoordinates() {
  Serial.print(x);
  Serial.print(",");
  Serial.println(y);
}
