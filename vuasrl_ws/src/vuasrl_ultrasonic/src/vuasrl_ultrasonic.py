#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial, rospy
from std_msgs.msg import Int32MultiArray

class ultrasonic_pub:
    ser = serial.Serial(port='/dev/ttySONIC', baudrate=115200,) # port, baundrate 변경하기
    US = [0,0,0,0,0,0,0,0,0]
 
    def __init__(self):
        self.DEBUG = False
        self.DEBUGERRMSG = [
            "Received data has been corrupted. (-3)",
            "Received data has been corrupted. (-2)",
            "No data received. (-1)"
        ]
        self.pub = rospy.Publisher('vuasrl_ultrasonic', Int32MultiArray, queue_size=1)
        self.ultra_sonic = Int32MultiArray()
        
# 초음파 판독 값 저장
    def send(self):
        for i in range(0,8):
            idx, val, err, self.US[0] = 0, 0, 0, 0
            idx, val, err = self.read_value(self.ser.readline())

            self.US[idx] = val

            if self.DEBUG:
                if self.US[0] < 0:
                    print("ULTRASONIC : ", self.DEBUGERRMSG[self.US[0]], " location : ", i, " value : ", errF)
# 버퍼 초기화
        self.ser.flushInput()
# 초음파 값  목록 
        self.ultra_sonic.data = [self.US[1], self.US[2], self.US[3], self.US[4], self.US[5], self.US[6], self.US[7], self.US[8]]

        if self.DEBUG:
            print(self.ultra_sonic.data)

        self.pub.publish(self.ultra_sonic)
 
    def read_value(self, serial_value):
        stri = serial_value[:-2].decode('ascii')
        if len(stri) == 0:
            return 0, -1, True

        number = ""
        for word in stri:
            if word.isdigit():
                number += word
                continue

        if len(stri) != len(number):
            return 0, -2, len(stri) - len(number)
 
        int_serial_value = int(number)
 
        value_number = int_serial_value % 10
        value = int_serial_value // 10
 
        if (value_number > 8) or (value_number < 1):
            return 0, -3, value_number
 
        return value_number, value, False

    def close(self):
        self.ser.close()

rospy.init_node('vuasrl_ultrasonic', anonymous=False)
ultrasonic = ultrasonic_pub()  
 
while not rospy.is_shutdown():
   ultrasonic.send()

ultrasonic.close()
        
