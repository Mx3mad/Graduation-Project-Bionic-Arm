// Include Wire Library for I2C Communications
#include <Wire.h>
 
// Include Adafruit PWM Library
#include <Adafruit_PWMServoDriver.h>
 
#define MIN_PULSE_WIDTH       550
#define MAX_PULSE_WIDTH       2300
#define FREQUENCY             50
 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
 
// Define EMGsensor Input Pin.
int emgSensor = A0;
int sensorReadings = 0;
 
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
 
 
void grabFinger(int sensorPin, int finger) {
  int pulseWide = 0, pulseWidth = 0, sensorVal = 0;
  
  // Read values from the sensor
  sensorVal = analogRead(sensorPin);
  
  // Convert to pulse width
  pulseWide = map(sensorVal, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulseWidth = int(float(pulseWide) / 1000000 * FREQUENCY * 4096);
  
  //Control/grab Motor/finger
  pwm.setPWM(finger, 0, pulseWidth);
}
 
void loop() {
 
  // //Control little finger
  // grabFinger(emgSensor, littleFing);
  
  // //Control ring finger
  // grabFinger(emgSensor, ringFing);
    
  // //Control middle finger
  // grabFinger(emgSensor, middleFing);
  
  // //Control index finger
  // grabFinger(emgSensor, indexFing);
  
  // //Control thumb finger
  // grabFinger(emgSensor, thumbFing);
  
  sensorReadings = analogRead(emgSensor);
  int pulse = int(float(map(sensorReadings, 0, 1023, 0, 180)));

  Serial.println(pulse);

   /*
    if(pulse > 70) {
      // close the hand
    } else {
      // open the hand
    }
  
  */
  
  delay(20);
}