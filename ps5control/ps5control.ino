
//ESP32 PS5 Controller - DC Motor Test
/*
  ESP32 PS5 Controller - DC Motor Test

  Control two DC motors on ESP32 using PS5 controller
  Requires TB6612FNG H-Bridge (can also use L298N)
  TB6612FNG Hookup - PWMA=22, AIN2=5, AIN1=18, BIN1=19, BIN2=21, PWMB=23
 
  char messageString[200];
  sprintf(messageString, "%4d,%4d,%4d,%4d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d",
  ps5.LStickX(),
  ps5.LStickY(),
  ps5.RStickX(),
  ps5.RStickY(),
  ps5.Left(),
  ps5.Down(),
  ps5.Right(),
  ps5.Up(),
  ps5.Square(),
  ps5.Cross(),
  ps5.Circle(),
  ps5.Triangle(),
  ps5.L1(),
  ps5.R1(),
  ps5.L2(),
  ps5.R2(),  
  ps5.Share(),
  ps5.Options(),
  ps5.PSButton(),
  ps5.Touchpad(),
  ps5.Charging(),
  ps5.Audio(),
  ps5.Mic(),
  ps5.Battery());
*/



 
// Include PS5 Controller library
#include <ps5Controller.h>
 
// Define motor driver pins
#define PWMA_PIN 22
#define AIN1_PIN 18
#define AIN2_PIN 5
#define PWMB_PIN 23
#define BIN1_PIN 19
#define BIN2_PIN 21
 
// Define PWM Parameters
const int motorFreq = 1000;
const int motorResolution = 8;
 
// Define channels for each motor
const int motorAChannel = 3;
const int motorBChannel = 4;
 
// Variables for Motor PWM values
int motorAPWM = 0;
int motorBPWM = 0;
 
// Variables for motor direction - true=forward
bool motorDir = true;
 
// Variables for joystick values
int rightX = 0;
int rightY = 0;
 
// Callback Function
void notify() {
 
  // Get Joystick value
  rightX = (ps5.data.analog.stick.rx);
  rightY = (ps5.data.analog.stick.ry);
  //rightTrigger = (ps5.data.button.l2);
 
  //Determine direction from Y axis position
  if (rightY < 0) {
    // Direction is forward
    motorDir = true;
  } else {
    // Direction is reverse
    motorDir = false;
  }
 
  // Convert joystick values to positive 0 - 255
  int speedX = (abs(rightX) * 2);
  int speedY = (abs(rightY) * 2);
 
  // Factor in the X axis value to determine motor speeds (assume Motor A is Left motor going forward)
  if (rightX < -10) {
    // Motor B faster than Motor A
    motorAPWM = speedY - speedX;
    motorBPWM = speedY + speedX;
 
  } else if (rightX > 10) {
    // Motor A faster than Motor B
    motorAPWM = speedY + speedX;
    motorBPWM = speedY - speedX;
 
  } else {
    // Control is in middle, both motors same speed
    motorAPWM = speedY;
    motorBPWM = speedY;
  }
 
  // Ensure that speed values remain in range of 0 - 255
  motorAPWM = constrain(motorAPWM, 0, 255);
  motorBPWM = constrain(motorBPWM, 0, 255);
 
  // Drive the motors
  moveMotors(motorAPWM, motorBPWM, motorDir);
 
  // Print to Serial Monitor
  Serial.print("X value = ");
  Serial.print(rightX);
  Serial.print(" - Y value = ");
  Serial.print(rightY);
  Serial.print(" - Motor A = ");
  Serial.print(motorAPWM);
  Serial.print(" - Motor B = ");
  Serial.println(motorBPWM);
}
 
// On Connection function
void onConnect() {
  // Print to Serial Monitor
  Serial.println("Connected.");
}
 
// Motor movement function
void moveMotors(int mtrAspeed, int mtrBspeed, bool mtrdir) {
  // Set direction pins
  if (!mtrdir) {
    // Move in reverse
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
 
  } else {
    // Move Forward
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
  }
 
  // Drive motors with PWM
  // analogWrite(motorAChannel, mtrAspeed);
  // analogWrite(motorBChannel, mtrBspeed);

  analogWrite(PWMA_PIN, mtrAspeed);
  analogWrite(PWMB_PIN, mtrBspeed);
}
 
void setup() {
 
  // Setup Serial Monitor for testing
  Serial.begin(115200);
  // Define Callback Function
  ps5.attach(notify);
  // Define On Connection Function
  ps5.attachOnConnect(onConnect);
  // Emulate console as specific MAC address (change as required)
  ps5.begin("24:a6:fa:c5:84:30");
 
  // Set motor controller pins as outputs
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
 
    // Set initial PWM values to 0 (motors off)
  // ledcAttachChannel(PWMA_PIN, motorFreq, motorResolution, motorAChannel);
  // ledcAttachChannel(PWMB_PIN, motorFreq, motorResolution, motorBChannel);
  analogWrite(PWMA_PIN, 0);
  analogWrite(PWMB_PIN, 0);
  // Print to Serial Monitor
  Serial.println("Ready.");
}
 
void loop() {
  if (!ps5.isConnected())
    return;
  delay(2000);
}