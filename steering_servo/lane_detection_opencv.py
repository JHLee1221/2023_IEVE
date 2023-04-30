# import cv2
# import numpy as np

# # Load the input image
# img = cv2.imread("/home/jhlee98/Downloads/road3.jpg")

# # Define the four source points (the corners of the input image)
# src_points = np.float32([[450, 450], [800, 450], [1000, 700], [250, 700]])

# # Define the four destination points (the corners of the output image)
# dst_points = np.float32([[450, 450], [800, 450], [1000, 700], [250, 700]])

# # Compute the perspective transform matrix and apply it to the input image
# M = cv2.getPerspectiveTransform(src_points, dst_points)
# output_img = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))

# # Display the output image
# cv2.imshow("Bird's Eye View", output_img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

import cv2
import numpy as np
import serial
import time

capture = cv2.VideoCapture(0)

while capture.isOpened():
    # 한프레임씩 이미지 읽기
    ret, frame = capture.read()
    if not ret:
        continue

    # 컬러지정
    image = frame
    # 이미지 화면 출력
    cv2.imshow('image', image)

    key = cv2.waitKey(1)
    if key == 27: # ESC 누르면 종료
        break

    # 카메라 밑 부분만 라인검출하는 좌표
    p1 =  [200, 400]  # 좌상
    p2 =  [439, 400] # 좌하
    p3 =  [539, 479] # 우상
    p4 = [100, 479]  # 우하

    # 전체 화면 대상으로 라인검출하는 좌표
    # p1 = [0, 0]  # 좌상
    # p2 = [639, 0]  # 좌하
    # p3 = [639,479]  # 우상
    # p4 = [0,479]  # 우하

    # corners_point_arr는 변환 이전 이미지 좌표 4개
    corner_points_arr = np.float32([p1, p2, p3, p4])
    height, width = image.shape[:2]


    image_p1 = [0, 0]
    image_p2 = [width, 0]
    image_p3 = [width, height]
    image_p4 = [0, height]

    image_params = np.float32([image_p1, image_p2, image_p3, image_p4])


    mat = cv2.getPerspectiveTransform(corner_points_arr, image_params)
    # mat = 변환행렬(3*3 행렬) 반
    image_transformed = cv2.warpPerspective(image, mat, (width, height))

    gray = cv2.cvtColor(image_transformed, cv2.COLOR_BGR2GRAY)

    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply Canny edge detection to the blurred image
    edges = cv2.Canny(blur, 10, 100)

    mask = np.zeros_like(edges)
    height, width = image_transformed.shape[:2]
    vertices = np.array([[(0, height), (width/2, height/2), (width, height)]], dtype=np.int32)
    cv2.fillPoly(mask, vertices, 255)
    masked_edges = cv2.bitwise_and(edges, mask)

    # Apply Hough line transform to detect the lanes
    lines = cv2.HoughLinesP(masked_edges, rho=1, theta=np.pi/180, threshold=20, minLineLength=20, maxLineGap=300)

    if lines is None or len(lines)<2:
        continue

    # Draw the detected lines on the input image
    for line in lines:
        x1, y1, x2, y2 = line[0]

        cv2.line(image_transformed, (x1, y1), (x2, y2), (0,0,255), 2)
        cv2.line(image_transformed, (0, 479), (639, 479), (0,255,0), 2)


        x1_1, y1_1, x2_1, y2_1 = lines[0][0]
        x1_2, y1_2, x2_2, y2_2 = lines[1][0]

        # 두 직선의 중심점 좌표 계산
        center_x1 = (x1_1 + x1_2) // 2
        center_y1 = (y1_1 + y1_2) // 2
        center_x2 = (x2_1 + x2_2) // 2
        center_y2 = (y2_1 + y2_2) // 2

        # 두 직선의 중심선 그리기
        cv2.line(image_transformed, (center_x1, center_y1), (center_x2, center_y2), (255, 0, 0), 2)
        # 이미지에 직선 표시하기
        image_transformed = cv2.cvtColor(image_transformed, cv2.COLOR_BGR2RGB)
        cv2.imshow('transformed image', image_transformed)

    # 차선 중심선과 중심선 각도 구하기 방법 1
    vector1 = (np.array([center_x2 - center_x1, center_y2 - center_y1]))
    vector2 = (np.array([1, 1]))

    radian = np.arccos(np.dot(vector1, vector2)/(np.linalg.norm(vector1)*np.linalg.norm(vector2)))
    theta = radian * 180/np.pi

    theta = 2 * (90 - np.fmin(theta, 180.0-theta))

    print(theta)

    # 너무 빠르게 인식할 때 전혀 다른 각도로 팅기는 현상이 있어 일부러 딜레이를 줌
    time.sleep(0.2)

    # # serial통신
    # arduino = serial.Serial('COM9', 9600)
    # arduino.write(theta)

capture.release()
cv2.destroyAllWindows()

