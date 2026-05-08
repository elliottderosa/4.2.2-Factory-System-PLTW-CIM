import cv2
import numpy as np
import pickle
import sys
import select
import math
import pigpio
import time

pi = pigpio.pi()

shot_pin = 18
hood_pin = 14

shot_speed = 1600
shot_angle = 0

pi.set_mode(shot_pin, pigpio.OUTPUT)

def shoot(speed, length):
    shot_speed = speed
    pi.set_servo_pulsewidth(shot_pin, speed)
    time.sleep(length)

def stop(pin):
    while shot_speed > 1500:
        shot_speed -= 2
        pi.set_servo_pulsewidth(18, shot_speed)
        time.sleep(0.02)
    pi.set_servo_pulsewidth(18, 1500)

with open("camera_calibration.pkl", "rb") as f:
    mtx, dist = pickle.load(f)

TAG_SIZE = 0.16  # meters

aruco_dict = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_APRILTAG_36h11
)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, params)

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Vision system started. Press Q to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = detector.detectMarkers(gray)

    target_found = False
    yaw = 0.0
    distance = 0.0
    tag_id = -1

    if ids is not None and len(ids) > 0:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, TAG_SIZE, mtx, dist
        )

        best_index = -1
        best_distance = 9999

        for i in range(len(ids)):
            tvec = tvecs[i][0]
            dist_i = np.linalg.norm(tvec)

            if dist_i < best_distance:
                best_distance = dist_i
                best_index = i

        if best_index != -1:
            tvec = tvecs[best_index][0]
            tag_id = int(ids[best_index][0])

            x = tvec[0]
            z = tvec[2]

            yaw = math.degrees(math.atan2(x, z))
            distance = np.linalg.norm(tvec)
            target_found = True

            shoot(1700, 2)
            stop()
        print(yaw, distance, target_found)
    print("alive")
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        if sys.stdin.readline().strip() == "q":
            break

ppi.stop()
cap.release()
cv2.destroyAllWindows()
