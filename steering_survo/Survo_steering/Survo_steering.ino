#include <Servo.h>
Servo steering;
double angle = 0; // output
double currentTime = 0;
double prevTime = 0;
double prevAngle;
double pwmAngle;

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 bps attachInterrupt (digitalPinToInterrupt (3), eStop, RISING);
  steering.attach(9);
  steering.write(90);
  delay(1000);
}

void loop() {
  while (Serial.available() == 0)
  {
     
  }
  angle = Serial.read();
  while (Serial.available() == 0)
  {
     
  }
  
  
  Serial.println("angle");
  Serial.println(angle);

  if (angle < -90) angle = -90; // maximum left turn
  if (angle > 90) angle = 90; // maximum left turn

  pwmAngle = map(angle, -90, 90, 2350, 600);  // if(angle<0) left turn, if(angle > 0) right turn
  steering.writeMicroseconds(pwmAngle);

  delay(1);
}