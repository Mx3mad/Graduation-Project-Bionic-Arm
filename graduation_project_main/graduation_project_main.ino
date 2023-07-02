// Include Wire Library for I2C Communications
#include <Wire.h>
 
// Include Adafruit PWM Library
#include <Adafruit_PWMServoDriver.h>
 
#define MIN_PULSE_WIDTH       600
#define MAX_PULSE_WIDTH       150
#define FREQUENCY             60

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
 
// Define EMGsensor Input Pin.
int emgSensor = A5;
int potentio = A0;

int sensorReadings = 0;
int range = 0;
int openAngle = 0;
int closeAngle = 140;
 
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

void grabFinger(int deg, int finger) {
  pwm.setPWM(finger, 0, angleToPulse(deg));
}

void clenchHand(int motorDeg){
  //Control little finger
  grabFinger(motorDeg, pinkieFing);
  
  //Control ring finger
  grabFinger(motorDeg, ringFing);
    
  //Control middle finger
  grabFinger(motorDeg, middleFing);
  
  //Control index finger
  grabFinger(motorDeg, indexFing);
  
  //Control thumb finger
  grabFinger(motorDeg, thumbFing);
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
    clenchHand(closeAngle);
  } 
  else {
    //Open The Arm    
    clenchHand(openAngle);
  }
}

uint16_t angleToPulse(int angle) {
  uint16_t pulse = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  return pulse;
}