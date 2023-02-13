// Include Wire Library for I2C Communications
#include <Wire.h>
 
// Include Adafruit PWM Library
#include <Adafruit_PWMServoDriver.h>
 
#define MIN_PULSE_WIDTH       550
#define MAX_PULSE_WIDTH       2300
#define FREQUENCY             50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
 
// define potentio pin
int potentio = A0;
int range = 0;
 
// Define Motor Outputs on PCA9685 board
int littleFing = 0;
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
 
 

void loop() {
    range = analogRead(potentio);
  // printing potentio readings
    Serial.println(range);

    //Control little finger
    grabFinger(range, littleFing);
    
    //Control ring finger
    // grabFinger(range, ringFing);
      
    //Control middle finger
    // grabFinger(range, middleFing);
    
    //Control index finger
    // grabFinger(range, indexFing);
    
    //Control thumb finger
    // grabFinger(range, thumbFing);

}

void grabFinger(int inputeValue, int finger) {
  int pulseWide = 0, pulseWidth = 0, sensorVal = 0;

  // Convert to pulse width
  pulseWide = map(inputeValue, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulseWidth = int(float(pulseWide) / 1000000 * FREQUENCY * 4096);
  // Control/grab Motor/finger
  pwm.setPWM(finger, 0, pulseWidth);
}

