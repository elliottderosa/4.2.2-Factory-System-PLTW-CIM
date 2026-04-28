import cv2
import numpy as np
import pickle
import math

with open("camera_calibration.pkl", "rb") as f:
    mtx, dist = pickle.load(f)

TAG_SIZE = 0.16  # meters

aruco_dict = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_APRILTAG_36h11
)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, params)

cap = cv2.VideoCapture(0)
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

    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    cv2.putText(
        frame,
        f"ID:{tag_id} Yaw:{yaw:.2f} Dist:{distance:.2f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2
    )

    cv2.imshow("vision", frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
