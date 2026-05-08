import cv2
import numpy as np
import pickle

# =========================
# SETTINGS
# =========================
CHECKERBOARD = (7, 7)   # 8x8 squares → 7x7 inner corners
REQUIRED_FRAMES = 20

# =========================
# OBJECT POINTS
# =========================
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

objpoints = []
imgpoints = []

# =========================
# CAMERA SETUP
# =========================
print("a")
cap = cv2.VideoCapture(0)
print("b")

if not cap.isOpened():
    print("ERROR: Cannot open camera")
    exit()

print("\nControls:")
print("  S = capture frame")
print("  Q = quit\n")

collected = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame read failed")
        break

    # 🔧 IMPORTANT: resize helps detection a LOT
    frame = cv2.resize(frame, (640, 480))

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 🔧 improve contrast
    gray = cv2.equalizeHist(gray)

    # =========================
    # DETECTION (ROBUST VERSION)
    # =========================
    found, corners = cv2.findChessboardCornersSB(gray, CHECKERBOARD)

    display = frame.copy()

    if found:
        cv2.drawChessboardCorners(display, CHECKERBOARD, corners, found)
        status = "Chessboard detected"
    else:
        status = "NOT detected"

    cv2.putText(display, status, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.putText(display, f"Captured: {collected}/{REQUIRED_FRAMES}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    cv2.imshow("Calibration", display)

    # =========================
    # KEY HANDLING
    # =========================
    key = cv2.waitKey(10) & 0xFF

    if key == ord('s'):
        print("S pressed")

        if found:
            objpoints.append(objp)
            imgpoints.append(corners)
            collected += 1
            print(f"[OK] captured {collected}")
        else:
            print("[SKIP] no chessboard detected")

    elif key == ord('q'):
        print("Quitting")
        break

    if collected >= REQUIRED_FRAMES:
        print("Enough frames collected")
        break

cap.release()
cv2.destroyAllWindows()

# =========================
# CALIBRATION
# =========================
if collected < 5:
    print("Not enough data")
    exit()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints,
    imgpoints,
    gray.shape[::-1],
    None,
    None
)

print("\n=== CAMERA MATRIX ===")
print(mtx)

print("\n=== DISTORTION ===")
print(dist)

# =========================
# SAVE
# =========================
with open("camera_calibration.pkl", "wb") as f:
    pickle.dump((mtx, dist), f)

print("\nSaved calibration file")
