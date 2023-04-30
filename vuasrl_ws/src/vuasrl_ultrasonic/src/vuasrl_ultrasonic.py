#!/usr/bin/python3

import serial, time, rospy
from std_msgs.msg import Int32

ser = serial.Serial(
    port='/dev/ttyACM2',   # serial port name
    baudrate=115200        # baud rate
)

def read_sensor():
    while ser.in_waiting:
        serial_data = ser.readline().decode().strip()
        if serial_data.startswith('Type1'):
            # extract data from Type1 message
            data = int(serial_data.split(':')[1])
            pub1.publish(data)
        elif serial_data.startswith('Type2'):
            # extract data from Type2 message
            data = int(serial_data.split(':')[1])
            pub2.publish(data)

if __name__ == '__main__':
    rospy.init_node('ultrasonic_pub', anonymous=False) # initialize node
    pub1 = rospy.Publisher('right_ultrasonic', Int32, queue_size=1)
    pub2 = rospy.Publisher('left_ultrasonic', Int32, queue_size=1)

    while not rospy.is_shutdown():
        read_sensor() 
        time.sleep(0.1)
   
    ser.close()

