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
import matplotlib.pyplot as plt

image = cv2.imread("/home/jhlee98/Downloads/road3.jpg")
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.show()

p1 =  [310, 200]  # 좌상
p2 =  [410, 200] # 좌하
p3 =  [600, 330] # 우상
p4 = [300, 330]  # 우하

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

# Draw the detected lines on the input image
for line in lines:
    x1, y1, x2, y2 = line[0]
    cv2.line(image_transformed, (x1, y1), (x2, y2), (0, 0, 255), 2)

plt.imshow(cv2.cvtColor(image_transformed, cv2.COLOR_BGR2RGB))
plt.show()
