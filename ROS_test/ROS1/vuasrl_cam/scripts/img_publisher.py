#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

cap = cv2.VideoCapture(0)
print(cap.isOpened())
bridge = CvBridge()

def Img_Pub():
    pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size = 1)
    rospy.init_node('image', anonymous = False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ret, frame  = cap.read()
        if not ret:
            break
        
        msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
        pub.publish(msg)
        
        if cv2.waitKey(1) & 0XFF == ord('q'):
            break
        
        if rospy.is_shutdown():
            cap.release()

if __name__ == '__main__':
    try:
        Img_Pub()
    except rospy.ROSInterruptException:
        pass
