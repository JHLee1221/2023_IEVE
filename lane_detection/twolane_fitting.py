#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

class LaneFollower:

    def __init__(self):

        self.image_sub = rospy.Subscriber("/smart_tugcar/camera/rgb/image_raw/compressed", CompressedImage, self.callback)
        self.cmd_vel_pub = rospy.Publisher('/smart_tugcar/cmd_vel', Twist, queue_size=10)
        self.img_bgr = None
    
    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_hsv = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2HSV)

        lower_wlane = np.array([0,0,185])
        upper_wlane = np.array([30,60,255])

        lower_ylane = np.array([10,100,100])
        upper_ylane = np.array([40,255,255])

        img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)
        img_ylane = cv2.inRange(img_hsv, lower_ylane, upper_ylane)

        img_lane = cv2.bitwise_or(img_wlane, img_ylane)

        kernel = np.ones((5,5), np.uint8)
        img_lane = cv2.morphologyEx(img_lane, cv2.MORPH_OPEN, kernel)
        img_lane = cv2.morphologyEx(img_lane, cv2.MORPH_CLOSE, kernel)

        contours, hierarchy = cv2.findContours(img_lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        lanes = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 5:  # 차선 인식 안되면 이부분 조금 낮추기 (차선 굵기)
                lanes.append(cnt)

        if len(lanes) == 2:
            M1 = cv2.moments(lanes[0])
            M2 = cv2.moments(lanes[1])

            cx1 = int(M1['m10']/M1['m00'])
            cy1 = int(M1['m01']/M1['m00'])

            cx2 = int(M2['m10']/M2['m00'])
            cy2 = int(M2['m01']/M2['m00'])

            cv2.circle(self.img_bgr, (cx1, cy1), 7, (255, 255, 255), -1)
            cv2.circle(self.img_bgr, (cx2, cy2), 7, (255, 255, 255), -1)

            cx_avg = (cx1 + cx2) // 2
            cv2.circle(self.img_bgr, (cx_avg, (cy1 + cy2) // 2), 7, (0, 0, 255), -1)

            error = cx_avg - self.img_bgr.shape[1] // 2
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = -float(error) / 100
            self.cmd_vel_pub.publish(twist)

        cv2.imshow("Lane Detection", self.img_bgr)
        cv2.waitKey(1)

if __name__ == '__main__':

    rospy.init_node('lane_follower', anonymous=True)

    lane_follower = LaneFollower()

    rospy.spin()
       

