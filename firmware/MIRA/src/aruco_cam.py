import cv2
import numpy as np

cap = cv2.VideoCapture(0)

# Use an ArUco dictionary (like "AprilTag-lite")
aruco = cv2.aruco
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
params = aruco.DetectorParameters()

detector = aruco.ArucoDetector(dictionary, params)

print("Press q to quit")

while True:
    ok, frame = cap.read()
    if not ok:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

        for i in range(len(ids)):
            pts = corners[i][0]  # 4 corners
            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))
            tag_id = int(ids[i][0])

            cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)
            cv2.putText(frame, f"ID {tag_id}", (cx + 10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            print(f"Tag {tag_id} at pixel ({cx}, {cy})")

    cv2.imshow("ArUco Webcam", frame)
    if (cv2.waitKey(1) & 0xFF) == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()



