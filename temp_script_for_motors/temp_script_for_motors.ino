// Include Wire Library for I2C Communications
#include <Wire.h>
 
// Include Adafruit PWM Library
#include <Adafruit_PWMServoDriver.h>
 
#define MIN_PULSE_WIDTH       150
#define MAX_PULSE_WIDTH       450
#define FREQUENCY             50
 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
 
// Define Potentiometer Inputs
 
int controlA = A0;
//int controlB = A1;
//int controlC = A2;
//int controlD = A3;
 
// Define Motor Outputs on PCA9685 board
 
int motorA = 0;
int motorB = 1;
int motorC = 2;
int motorD = 3;
int motorE = 4;
 
void setup() 
{
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  Serial.begin(9600);
}
 
 
void moveMotor(int controlIn, int motorOut)
{
  int pulse_wide, potVal;
  
  // Read values from potentiometer
  potVal = analogRead(controlIn);
  
  // Convert to pulse width
  pulse_wide = map(potVal, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  
  //Control Motor
  pwm.setPWM(motorOut, 0, pulse_wide);

  Serial.println(pulse_wide);

}
 
void loop() {
 
  //Control Motor A
  moveMotor(controlA, motorA);
  
  //Control Motor B
  moveMotor(controlA, motorB);
    
  //Control Motor C
  moveMotor(controlA, motorC);
  
  //Control Motor D
  moveMotor(controlA, motorD);
  
  //Control Motor E
  moveMotor(controlA, motorE);
 
 
}