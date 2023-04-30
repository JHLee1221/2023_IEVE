#!/usr/bin/python3

import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive


def cmd_callback(data):
    global ackermann_pub
    global wheelbase
    
    velocity = data.linear.x
    steering = data.angular.z
    
    msg = AckermannDrive()
    msg.steering_angle = steering
    msg.speed = velocity
    ackermann_pub.publish(msg)
    
    
if __name__ == '__main__':
    
    rospy.init_node('convert', anonymous = False)

    rospy.Subscriber('/cmd_vel', Twist, cmd_callback, queue_size = 1)
    
    ackermann_pub = rospy.Publisher('/ackermann/cmd_vel', AckermannDrive, queue_size = 1)
    
    print('convert')
    
    rospy.spin()



    
    


    

    if __name__ == '__main__': 
        try:
            
            rospy.init_node('ackermann_drive')
            
            rospy.Subscriber('/cmd_vel', Twist, cmd_callback, queue_size=1)
            
            rospy.spin()
            
        except rospy.ROSInterruptException:
            pass