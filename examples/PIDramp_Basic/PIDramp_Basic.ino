/*
* PIDramp Example
* Features: PID, output saturation, output rate limiting, feedforward addition.
*/

#include "PIDramp.h"

float kp=5., ki=2., kd=1.2, lower_limit=-50., upper_limit=50., rate_limit=4.;

PIDramp pidramp(kp, ki, kd, lower_limit, upper_limit, rate_limit);

float setpoint = 30;
float measured_value = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(pid.compute(setpoint, measured_value));
  delay(10);
  measured_value++;
}
