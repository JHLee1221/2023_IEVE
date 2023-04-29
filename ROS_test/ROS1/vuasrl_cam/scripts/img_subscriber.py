#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bridge = CvBridge()
cv_image = np.empty(shape=[0])

def image_callback(msg):
    print("Recieved Webcam")
    
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        frame = cv_image
        
    except CvBridgeError as e:
        print(e)
    else:
        cv2.imshow('frame', frame)
        
        cv2.waitKey(1)
        
def main():
    rospy.init_node('image_sub', anonymous = False)
    
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback, queue_size =1)
    
    rospy.spin()
    
if __name__ == '__main__':
    main()