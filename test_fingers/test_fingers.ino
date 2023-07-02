// Include Wire Library for I2C Communications
#include <Wire.h>
 
// Include Adafruit PWM Library
#include <Adafruit_PWMServoDriver.h>
 
#define MIN_PULSE_WIDTH       600
#define MAX_PULSE_WIDTH       150
#define FREQUENCY             60

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
 
// define potentio pin
int potentio = A0;
int range = 135;
 
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
  //pinMode(emgSensor,INPUT);
}
 
 

void loop() {
    grabFinger(0, littleFing);
    grabFinger(0, ringFing);
    grabFinger(0, middleFing);
    grabFinger(0, indexFing);
    grabFinger(0, thumbFing);

    delay(2000);
    grabFinger(150, littleFing);
    grabFinger(150, ringFing);
    grabFinger(150, middleFing);
    delay(500);
    grabFinger(150, indexFing);
    delay(700);
    grabFinger(100, thumbFing);
    delay(2000);

}

void grabFinger(int deg, int finger) {
  pwm.setPWM(finger, 0, angleToPulse(deg));
}

uint16_t angleToPulse(int angle) {
  uint16_t pulse = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  return pulse;
}
