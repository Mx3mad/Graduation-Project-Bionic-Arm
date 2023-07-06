// Include Adafruit PWM Library
#include <Adafruit_PWMServoDriver.h>
 
#define MIN_PULSE_WIDTH       600
#define MAX_PULSE_WIDTH       150
#define FREQUENCY             60

// Instantiating an object to control the servo driver
Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();
 
// Declaring variables to define/store the EMG sensor and potentiometer pins.
int emg_sensor = A0;
int potentiometer = A1;
 
// Declaring variables to define/store the motors attached to
// each finger with there pins on PCA9685 board
int pinky_finger = 0;
int ring_finger = 4;
int middle_finger = 8;
int index_finger = 12;
int thumb_finger = 15;

// Declaring variables to store the sensor reading and the threshold
int sensor_reading;
int threshold;

void setup() 
{
  servoDriver.begin();
  servoDriver.setPWMFreq(FREQUENCY);
  Serial.begin(9600);
  pinMode(emg_sensor, INPUT);
  pinMode(potentiometer, INPUT);
}

void loop() 
{
  sensor_reading = analogRead(emg_sensor);
  threshold = analogRead(potentiometer);

  // printing sensor and potentiometer readings
  Serial.println("sensor:  " + String(sensor_reading) + "  potentiometer:  " + String(threshold));

  if(sensor_reading > threshold) 
  {
    // Clench The Hand    
    clenchHand();
  } 
  else 
  {
    // Release The Hand   
    releaseHand();
  }
}

void grabFinger(int deg, int finger) 
{
  servoDriver.setPWM(finger, 0, angleToPulse(deg));
}

void clenchHand()
{
  //Control little finger
  grabFinger(140, pinky_finger);
  
  //Control ring finger
  grabFinger(140, ring_finger);
    
  //Control middle finger
  grabFinger(140, middle_finger);
  
  //Control index finger
  grabFinger(140, index_finger);
  
  delay(200);
  //Control thumb finger
  grabFinger(140, thumb_finger);
}

void releaseHand()
{
  //Control little finger
  grabFinger(0, pinky_finger);
  
  //Control ring finger
  grabFinger(0, ring_finger);
    
  //Control middle finger
  grabFinger(0, middle_finger);
  
  //Control index finger
  grabFinger(0, index_finger);
  
  //Control thumb finger
  grabFinger(0, thumb_finger);
}

int angleToPulse(int angle) 
{
  int pulse_width = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  return pulse_width;
}