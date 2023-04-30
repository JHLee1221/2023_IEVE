#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <Servo.h> 
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include "math.h"

int Motor_F = 5;  // DC Motor pin (Front)
int Motor_B = 4;  // DC Motor pin (Back)


ros::NodeHandle  nh;

Servo servo;

void motor(const geometry_msgs::Twist& msg){
  float a = msg.angular.z;
  float v = msg.linear.x;
  int angle = 83 + msg.angular.z * 40;
  int vel = abs(510*msg.linear.x);  // DC Motor max rpm is 255
  
  if(a==0 && v==0.5){           // Forward
    digitalWrite(Motor_F, HIGH);
    digitalWrite(Motor_B, LOW);
    analogWrite(5, vel);
    servo.write(angle);
  }
  else if(a==1 && v==0.5){      // Turn right
    digitalWrite(Motor_F, HIGH);
    digitalWrite(Motor_B, LOW);
    analogWrite(5, vel);
    servo.write(angle);
  }
  else if(a==-1 && v==0.5){     // Turn left
    digitalWrite(Motor_F, HIGH);
    digitalWrite(Motor_B, LOW);
    analogWrite(5, vel);
    servo.write(angle);
  }
  else if(a==1 && v==0){        // Stop and only steering angle is turn right 
    digitalWrite(Motor_F, LOW);
    digitalWrite(Motor_B, LOW);
    analogWrite(5, vel);
    servo.write(angle);
  }
  else if(a==-1 && v==0){       // Stop and only steering angle is turn left
    digitalWrite(Motor_F, LOW);
    digitalWrite(Motor_B, LOW);
    analogWrite(5, vel);
    servo.write(angle);
  }
  else if(a==0 && v==-0.5){     // Backward
    digitalWrite(Motor_F, LOW);
    digitalWrite(Motor_B, HIGH);
    analogWrite(5, vel);
    servo.write(angle);
  }
  else if(a==1 && v==-0.5){     // Backward for right side
    digitalWrite(Motor_F, LOW);
    digitalWrite(Motor_B, HIGH);
    analogWrite(5, vel);
    servo.write(angle);
  }
  else if(a==-1 && v==-0.5){    // Backward for left side
    digitalWrite(Motor_F, LOW);
    digitalWrite(Motor_B, HIGH);
    analogWrite(5, vel);
    servo.write(angle);
  }
  else if(a==0 && v==0){       // Stop
    digitalWrite(Motor_F, LOW);
    digitalWrite(Motor_B, LOW);
    analogWrite(5, vel);
    servo.write(angle);
  }

}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", motor);   // Subscribe

void setup(){
  nh.initNode();
  nh.subscribe(sub);
  servo.write(83);  // Servo is set to midpoint
  servo.attach(8); // Servo attach it to pin 7
}

void loop(){
  nh.spinOnce();
  delay(1);
}
