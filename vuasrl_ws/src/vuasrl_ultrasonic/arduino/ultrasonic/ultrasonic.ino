#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

std_msgs::Int32 right_distance;
std_msgs::Int32 left_distance;

ros::Publisher chatter_right("right_ultrasonic", &right_distance);
ros::Publisher chatter_left("left_ultrasonic", &left_distance);

const int trigger1 = 13;
const int echo1 = 12;
const int trigger2 = 10;
const int echo2 = 9;

long timeElapsed = 0;
long rightDist = 0;
long leftDist = 0;

void setup() {
    pinMode(trigger1, OUTPUT);
    pinMode(echo1, INPUT);
    pinMode(trigger2, OUTPUT);
    pinMode(echo2, INPUT);
    Serial.begin(115200);
    nh.initNode();
    nh.advertise(chatter_right);
    nh.advertise(chatter_left);
}

void ultrasonic(const int trigger, const int echo, long& distance) {
    // Clear trigger
    digitalWrite(trigger,LOW);
    delayMicroseconds(2);

    // send 10 microseond trigger
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);

    // time to get echo
    timeElapsed = pulseIn(echo, HIGH);

    // convert time to distance
    // Divide by 1000000 (convert microseconds to seconds)
    // Divide by 2 for distance there and back
    // Multiply by speed of sound 34300cm/s
    distance = timeElapsed * 170 / 10000; // in cm
}

void loop() {
    ultrasonic(trigger1, echo1, rightDist); 
    
    right_distance.data = rightDist; //위에서 계산한 거리값을 Distance 데이터에 저장
    chatter_right.publish(&right_distance); //chatter 발행자가 'chatter' 토픽으로 str_msg를 publish하도록 함
    nh.spinOnce(); //콜백함수 호출을 위한 함수, 메시지 수신 대기
    
    ultrasonic(trigger2, echo2, leftDist);
    
    left_distance.data = leftDist; //위에서 계산한 거리값을 Distance 데이터에 저장
    chatter_left.publish(&left_distance); //chatter 발행자가 'chatter' 토픽으로 str_msg를 publish하도록 함
    nh.spinOnce(); //콜백함수 호출을 위한 함수, 메시지 수신 대기
    
    delay(500);
}
