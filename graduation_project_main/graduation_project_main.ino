// Include Wire Library for I2C Communications
#include <Wire.h>
 
// Include Adafruit PWM Library
#include <Adafruit_PWMServoDriver.h>
 
#define MIN_PULSE_WIDTH       550
#define MAX_PULSE_WIDTH       2300
#define FREQUENCY             50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
 
// Define EMGsensor Input Pin.
int emgSensor = A1;
int potentio = A0;

int sensorReadings = 0;
int range = 0;
int openVal = 900;
int closeVal = 0;
 
// Define Motor Outputs on PCA9685 board
int pinkieFing = 0;
int ringFing = 4;
int middleFing = 8;
int indexFing = 12;
int thumbFing = 15;
 
void setup() {
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  Serial.begin(9600);
  pinMode(emgSensor,INPUT);
}

void grabFinger(int inputValue, int finger) {
  int pulseWide = 0, pulseWidth = 0, sensorVal = 0;
  // Convert to pulse width
  pulseWide = map(inputValue, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulseWidth = int(float(pulseWide) / 1000000 * FREQUENCY * 4096);
  // Control/grab Motor/finger
  pwm.setPWM(finger, 0, pulseWidth);
}

void clenchHand(int motorMovement){
  //Control little finger
  grabFinger(motorMovement, pinkieFing);
  
  //Control ring finger
  grabFinger(motorMovement, ringFing);
    
  //Control middle finger
  grabFinger(motorMovement, middleFing);
  
  //Control index finger
  grabFinger(motorMovement, indexFing);
  
  //Control thumb finger
  grabFinger(motorMovement, thumbFing);
}
 
void loop() {
  
  sensorReadings = analogRead(emgSensor);
  range = analogRead(potentio);
  // printing sensor and potentio readings
  Serial.print("sensor:  ");
  Serial.print(sensorReadings);
  Serial.print("  potentio:  ");
  Serial.println(range);

  if(sensorReadings > range) {
    // Close The Arm    
    clenchHand(closeVal);
  } 
  else {
    //Open The Arm    
    clenchHand(openVal);
  }
}
