const int trigger1 = 13;
const int echo1 = 12;
const int trigger2 = 10;
const int echo2 = 9;

long timeElapsed = 0;
long right_distance = 0;
long left_distance = 0;

void setup() {
    pinMode(trigger1, OUTPUT);
    pinMode(echo1, INPUT);
    pinMode(trigger2, OUTPUT);
    pinMode(echo2, INPUT);
    Serial.begin(115200);
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
    ultrasonic(trigger1, echo1, right_distance);
    Serial.print("Type1:");
    Serial.println(right_distance);
    
    ultrasonic(trigger2, echo2, left_distance);
    Serial.print("Type2:");
    Serial.println(left_distance);
    delay(500);
}
