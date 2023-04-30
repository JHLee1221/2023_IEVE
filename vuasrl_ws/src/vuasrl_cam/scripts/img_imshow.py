#!/usr/bin/python3

import cv2

cap = cv2.VideoCapture(0)
print(cap.isOpened())

if not cap.isOpened():
    print("camera is not working")

while True:
    ret, frame = cap.read()
    
    if not ret:
        break
    cv2.imshow('frame', frame)
    
    if cv2.waitKey(1) == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()