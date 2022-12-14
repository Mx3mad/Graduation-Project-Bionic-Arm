// using servo motor library
#include <Servo.h>

// defining both maximum and minimum angles the motors will rotate (Temporary values).
#define MAX_ROTATION_ANGLE 150
#define MIN_ROTATION_ANGLE 50

int sumOfSignalsAmp = 0;  // summation of all input signals amplitudes until a certain times.
int numOfSignals = 0;     // number of all input signals amplitudes until a certain times.
int EMGSensor = A0;

Servo motor1;  // 1st motor variable/object.

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);  // setup serial monitor.
  motor1.attach(3);    // attaching the motor1 object to digital pin 3.
}

void loop() {
  // put your main code here, to run repeatedly:

  // a variable that stores the value of the amplitude of the signal coming from the EMG sensor.
  int inputSignalAmp = analogRead(EMGSensor);

  // a condition that resets the values of sumOfSignalsAmp & numOfSignals after a time = delay * numOfSignals value.
  if (numOfSignals == 25) {
    sumOfSignalsAmp = 0;
    numOfSignals = 0;
  }

  // adding each new input signal amplitude to the total,
  // and increasing the number of signals by 1 each cycle of the main loop.
  sumOfSignalsAmp += inputSignalAmp;
  numOfSignals++;

  // calculating the average of all signals amplitudes up to this point in time.
  int avgInputSignalAmp = sumOfSignalsAmp / numOfSignals;

  // a condition that changes the rotation of the motor depending on the avgInputSignalAmp
  // (whether the hand is gripping or not).
  if (avgInputSignalAmp > 20) {
    // if the avgInputSignalAmp is > than 20 it will rotate the motor to the MAX_ROTATION_ANGLE.
    motor1.write(MAX_ROTATION_ANGLE);

    // the Serial print is for debugging purposes.
    Serial.print("Grip ");
    Serial.println(avgInputSignalAmp);
  }

  else {
    // if the avgInputSignalAmp is <= to 20 it will rotate the motor back to the MIN_ROTATION_ANGLE.
    motor1.write(MIN_ROTATION_ANGLE);

    // the Serial print is for debugging purposes.
    Serial.print("Release ");
    Serial.println(avgInputSignalAmp);
  }

  delay(10);
}
