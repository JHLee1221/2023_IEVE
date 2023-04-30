#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32
from vuasrl_msgs.msg import vuasrl_motor

bridge = CvBridge()
cv_image = np.empty(shape=[0])
ultra_msg = None  # 초음파 데이터를 담을 변수
ultra_data = None  # 초음파 토픽의 필터링에 사용할 변수

class MovingAverage:    #이동 평균값 계산 함수

    def __init__(self, n):
        self.samples = n
        self.data = []
        self.weights = list(range(1, n + 1))

    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data = self.data[1:] + [new_sample]
            
    def get_sample_count(self):
        return len(self.data)
        
    # 이동평균값을 구하는 함수
    def get_mavg(self):
        return int(sum(self.data)) / len(self.data)

    # 중앙값을 사용해서 이동평균값을 구하는 함수
    def get_mmed(self):
        return int(np.median(self.data))

    # 가중치를 적용하여 이동평균값을 구하는 함수        
    def get_wmavg(self):
        s = 0
        for i, x in enumerate(self.data):
            s += x * self.weights[i]
        return int(s) / sum(self.weights[:len(self.data)])

class LaneFollower:

    def __init__(self):
                
        self.motor_pub = rospy.Publisher('vuasrl_motor', vuasrl_motor, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)
        self.ult_sub1 = rospy.Subscriber("right_ultrasonic", Int32, self.ultra_right_callback, queue_size=1)
        self.ult_sub2 = rospy.Subscriber("left_ultrasonic", Int32, self.ultra_left_callback, queue_size=1)
        self.img_bgr = None
        self.min_distance = int(9999)    

        avg_count = 5  # 이동평균값을 계산할 데이터 묶음 갯수
        self.ultra_mvavg = [MovingAverage(avg_count) for i in range(2)]

    def ultra_right_callback(self, data):
        global ultra_msg, ultra_data

        ultra_data = int(data.data)
        print(type(ultra_data))
        # 이동평균필터를 적용해서 튀는 값을 제거해서 ultrnoea_msg_ft에 담기
        for i in range(2):
            self.ultra_mvavg[i].add_sample(int(data.data))

        ultra_msg = int(self.ultra_mvavg[0].get_mmed())

        print("Minimum right distance:", round(ultra_msg, 1))

        if (round(ultra_msg, 1) <= 20):
            self.obastacle()
            
    def ultra_left_callback(self, data):
        global ultra_msg, ultra_data

        ultra_data = int(data.data)
        print(type(ultra_data))
        # 이동평균필터를 적용해서 튀는 값을 제거해서 ultrnoea_msg_ft에 담기
        for i in range(2):
            self.ultra_mvavg[i].add_sample(int(data.data))

        ultra_msg = int(self.ultra_mvavg[0].get_mmed())

        print("Minimum left distance:", round(ultra_msg, 1))

        if (round(ultra_msg, 1) <= 20):
            self.obastacle()

            
    def obastacle(self):
        msg = vuasrl_motor()
        msg.angle = 0.0
        msg.speed = 0.0
        self.motor_pub.publish(msg)
        #print("obstacle_mode") 
            
    def callback(self, data):
        
        global cv_image
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
        frame = cv_image
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
#----------------------------bev--------------------------------------#
        p1 = [0, 0] # 좌하
        p2 = [640, 0] # 우하
        p3 = [640, 480] # 우상
        p4 = [0, 480] # 좌상

        # corners_point_arr는 변환 이전 이미지 좌표 4개 
        corner_points_arr = np.float32([p1, p2, p3, p4])
        height, width = img_hsv.shape[:2]

        image_p1 = [0, 0]
        image_p2 = [width, 0]
        image_p3 = [width, height]
        image_p4 = [0, height]

        image_params = np.float32([image_p1, image_p2, image_p3, image_p4])

        mat = cv2.getPerspectiveTransform(corner_points_arr, image_params)
        # mat = 변환행렬(3*3 행렬) 반
        image_transformed = cv2.warpPerspective(img_hsv, mat, (width, height))
#------------------------------------------------------------------#    

        lower_wlane = np.array([0,0,185])
        upper_wlane = np.array([30,60,255])

        lower_ylane = np.array([10,100,100])
        upper_ylane = np.array([40,255,255])

        img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)
        img_ylane = cv2.inRange(img_hsv, lower_ylane, upper_ylane)

        # combine the two binary images
        img_lane = cv2.bitwise_or(img_wlane, img_ylane)

        # apply morphological operations to reduce noise
        kernel = np.ones((5,5), np.uint8)
        img_lane = cv2.morphologyEx(img_lane, cv2.MORPH_OPEN, kernel)
        img_lane = cv2.morphologyEx(img_lane, cv2.MORPH_CLOSE, kernel)

        # find contours
        contours, hierarchy = cv2.findContours(img_lane, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # find left and right edges
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            # find edges using Canny edge detection
            img_bgr = cv2.cvtColor(image_transformed, cv2.COLOR_HSV2BGR)
            img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
            img_edges = cv2.Canny(img_gray, 100, 200)
            
            left_edge = np.argmax(img_edges[cy,:cx])
            right_edge = cx + np.argmax(img_edges[cy,cx:])

            # calculate the midpoint between the left and right edges
            midpoint = (left_edge + right_edge) // 2
            
            # draw left and right edges
            cv2.line(image_transformed, (left_edge, cy), (cx, cy), (0, 255, 0), 2)
            cv2.line(image_transformed, (cx, cy), (right_edge, cy), (0, 255, 0), 4)
            
            # draw the midpoint as a vertical line
            cv2.line(image_transformed, (midpoint, 0), (midpoint, image_transformed.shape[0]), (0, 0, 255), 2)

            # move the robot based on the position of the center of the lane and check for obstacles
            error = ((cx + 210) - image_transformed.shape[1]/2) + 1
            msg = vuasrl_motor()
            ##
            msg.speed = 5
            
            msg.angle = -int(error) / 400
            self.motor_pub.publish(msg)    
        #image_transformed_bgr = cv2.cvtColor(image_transformed, cv2.COLOR_BGR2GRAY)
        # display the image
        cv2.imshow("Lane Detection", image_transformed)
        #cv2.imshow("Lane Detection", frame)
        cv2.waitKey(1) 

if __name__ == '__main__':

    rospy.init_node('lane_follower', anonymous=True)

    lane_follower = LaneFollower()

    rospy.spin()
