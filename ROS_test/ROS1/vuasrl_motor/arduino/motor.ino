
#include <Arduino.h>


#include <ros.h>
#include <Servo.h> 
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDrive.h>
#include "math.h"

float PWM = 100;

int servoPin = 8;  // Servo pinNum

int Motor_F = 5;  // DC Motor pin (Front)
int Motor_B = 4;  // DC Motor pin (Back)

int NEUTRAL_STEERING_ANGLE = 90;
int MIN_STEERING_ANGLE = 14;
int MAX_STEERING_ANGLE = 166;

float MIN_THROTTLE = 20;
float MAX_THROTTLE= 110;

Servo steeringServo;

void ackermannCallback(const ackermann_msgs::AckermannDrive & msg);

ros::Subscriber<ackermann_msgs::AckermannDrive> ackermannSubscriber("/ackermann/cmd_vel", & ackermannCallback);
ros::NodeHandle nodeHandle;


void ackermannCallback(const ackermann_msgs::AckermannDrive & msg)
{

    
  int steering_angle = msg.steering_angle * (180 / M_PI) + 90;


  if (steering_angle < MAX_STEERING_ANGLE)
    analogWrite(5, 200);
    analogWrite(4, LOW);
    

  if (steering_angle > MIN_STEERING_ANGLE and steering_angle > MAX_STEERING_ANGLE)
    analogWrite(5, 200);
    analogWrite(4, LOW);

  //if (steering_angle > MAX_STEERING_ANGLE)
  //  analogWrite(5, 100);
  //  analogWrite(4, LOW);

  if (steering_angle < MIN_STEERING_ANGLE)
    analogWrite(5, 100);
    analogWrite(4, LOW);

  if (steering_angle < MIN_STEERING_ANGLE)
    steering_angle = MIN_STEERING_ANGLE;

  if (steering_angle > MAX_STEERING_ANGLE)
    steering_angle = MAX_STEERING_ANGLE;
    
  steeringServo.write(steering_angle);
  Serial.print(steering_angle);
}


void setup() {
  
  pinMode(Motor_F, OUTPUT);
  pinMode(Motor_B, OUTPUT);
  
  steeringServo.attach(8);

  nodeHandle.initNode();
  nodeHandle.subscribe(ackermannSubscriber);

  steeringServo.write(NEUTRAL_STEERING_ANGLE);

  
}

void loop(){
  if(!nodeHandle.connected()){
    steeringServo.write(NEUTRAL_STEERING_ANGLE);
  }
  
  nodeHandle.spinOnce();
  delay(1000);
}
