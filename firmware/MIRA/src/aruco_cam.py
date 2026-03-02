import cv2
import numpy as np

# ----------------------------
# Config
# ----------------------------
GRID_ROWS, GRID_COLS = 8, 8
CELL_PX = 80  # rectified pixels per cell (debug resolution)
W, H = GRID_COLS * CELL_PX, GRID_ROWS * CELL_PX
MARGIN_CELLS = 0.0 # margin around board


# Corner tag IDs (YOU should print/put these on the 4 corners)
# Recommended: 0,1,2,3
CORNER_TAG_IDS = {
    0: "TR",
    1: "BR",
    2: "BL",
    3: "TL",
}

# Add margin corners to the rectified board (for better visualization)
RECT_CORNERS = {
    "TL": (-MARGIN_CELLS * CELL_PX, -MARGIN_CELLS * CELL_PX),
    "TR": (W - 1 + MARGIN_CELLS * CELL_PX, -MARGIN_CELLS * CELL_PX),
    "BR": (W - 1 + MARGIN_CELLS * CELL_PX, H - 1 + MARGIN_CELLS * CELL_PX),
    "BL": (-MARGIN_CELLS * CELL_PX, H - 1 + MARGIN_CELLS * CELL_PX),
}

# ----------------------------
# Helpers
# ----------------------------
def inner_corner_by_center(tag_pts, img_center):
    # tag_pts: (4,2)
    d2 = np.sum((tag_pts - img_center) ** 2, axis=1)
    return tag_pts[np.argmin(d2)]

def extreme_corner(tag_pts: np.ndarray, label: str) -> np.ndarray:
    # tag_pts: (4,2)
    x = tag_pts[:, 0]
    y = tag_pts[:, 1]
    s = x + y
    d = x - y

    if label == "TL":
        return tag_pts[np.argmin(s)]
    if label == "TR":
        return tag_pts[np.argmax(d)]
    if label == "BR":
        return tag_pts[np.argmax(s)]
    if label == "BL":
        return tag_pts[np.argmin(d)]
    raise ValueError("bad label")

def compute_homography_inner(corners, ids, img_center):
    if ids is None:
        return None, None

    found = {}
    for i in range(len(ids)):
        tid = int(ids[i][0])
        if tid in CORNER_TAG_IDS:
            label = CORNER_TAG_IDS[tid]
            tag_pts = corners[i][0].astype(np.float32)  # (4,2)
            found[label] = inner_corner_by_center(tag_pts, img_center)

    if any(k not in found for k in ["TL", "TR", "BR", "BL"]):
        return None, None

    src = np.array([found["TL"], found["TR"], found["BR"], found["BL"]], dtype=np.float32)
    dst = np.array([RECT_CORNERS["TL"], RECT_CORNERS["TR"], RECT_CORNERS["BR"], RECT_CORNERS["BL"]], dtype=np.float32)

    Hmat = cv2.getPerspectiveTransform(src, dst)
    return Hmat, found  # return the points used

def rect_to_cell(xr: float, yr: float):
    col = int(xr // CELL_PX)
    row = int(yr // CELL_PX)
    if 0 <= row < GRID_ROWS and 0 <= col < GRID_COLS:
        return row, col
    return None

def draw_grid(img):
    # draw 8x8 grid lines on rectified image
    for c in range(1, GRID_COLS):
        x = c * CELL_PX
        cv2.line(img, (x, 0), (x, H - 1), (255, 255, 255), 1)
    for r in range(1, GRID_ROWS):
        y = r * CELL_PX
        cv2.line(img, (0, y), (W - 1, y), (255, 255, 255), 1)

def send_serial_command(cmd):
    # Placeholder for sending command to microcontroller
    print(f"SERIAL CMD: {cmd}") 

# ----------------------------
# Main
# ----------------------------
cap = cv2.VideoCapture(1)

aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
params = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, params)

print("Press q to quit")
print("Corner tag IDs mapping:", CORNER_TAG_IDS)

Hmat = None

while True:
    ok, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    if not ok:
        break

    h, w = frame.shape[:2]
    img_center = np.array([w / 2, h / 2], dtype=np.float32)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners)

        # Update homography if corner tags are all visible
        
        newH, found_pts = compute_homography_inner(corners, ids, img_center)

        # Debug: draw the found corner points and quadrilateral
        if found_pts is not None:
            quad = np.array([found_pts["TL"], found_pts["TR"], found_pts["BR"], found_pts["BL"]], dtype=np.int32)
            cv2.polylines(frame, [quad], True, (255, 0, 0), 3)
            for lab in ["TL", "TR", "BR", "BL"]:
                p = found_pts[lab]
                cv2.circle(frame, (int(p[0]), int(p[1])), 10, (255, 0, 0), 3)
                cv2.putText(frame, lab, (int(p[0]) + 5, int(p[1]) + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)
        if newH is not None:
            Hmat = newH
            cv2.putText(frame, "H: LOCKED", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "H: NEED 4 CORNERS", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        # If we have H, prepare rectified debug view
        rectified = None
        if Hmat is not None:
            rectified = cv2.warpPerspective(frame, Hmat, (W, H))
            draw_grid(rectified)

        for i in range(len(ids)):
            pts = corners[i][0]
            cx, cy = np.mean(pts[:, 0]), np.mean(pts[:, 1])
            tag_id = int(ids[i][0])

            # Mark center in camera image
            cv2.circle(frame, (int(cx), int(cy)), 3, (0, 255, 0), -1)

            if Hmat is not None:
                p = np.array([[[cx, cy]]], dtype=np.float32)
                pr = cv2.perspectiveTransform(p, Hmat)[0][0]
                xr, yr = float(pr[0]), float(pr[1])

                cell = rect_to_cell(xr, yr)

                # Corner tags: show label
                corner_label = CORNER_TAG_IDS.get(tag_id, None)
                if corner_label is not None:
                    txt = f"CORNER {corner_label} (ID {tag_id})"
                else:
                    txt = f"ID {tag_id}"

                if cell is not None:
                    row, col = cell
                    # Send ID + Row/Col to microcontroller (placeholder function)
                    # send_serial_command(f"{tag_id},{row},{col}")
                    txt += f" -> cell ({row},{col})"
                    cv2.putText(frame, txt, (int(cx) + 10, int(cy)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                    if rectified is not None:
                        cv2.circle(rectified, (int(xr), int(yr)), 6, (0, 255, 0), -1)
                        cv2.putText(rectified, f"{tag_id} ({row},{col})",
                                    (int(xr) + 6, int(yr) - 6),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                else:
                    cv2.putText(frame, txt + " -> off-board", (int(cx) + 10, int(cy)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            else:
                cv2.putText(frame, f"ID {tag_id} (no H)", (int(cx) + 10, int(cy)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    cv2.imshow("Camera", frame)
    

    # Show rectified view if homography is locked
    if Hmat is not None:
        rectified = cv2.warpPerspective(frame, Hmat, (W, H))
        draw_grid(rectified)
        cv2.imshow("Rectified Board (8x8)", rectified)
      
    if (cv2.waitKey(1) & 0xFF) == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()


# import cv2
# import numpy as np

# cap = cv2.VideoCapture(1)

# # Use an ArUco dictionary (like "AprilTag-lite")
# aruco = cv2.aruco
# dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
# params = aruco.DetectorParameters()

# detector = aruco.ArucoDetector(dictionary, params)

# print("Press q to quit")

# while True:
#     ok, frame = cap.read()
#     if not ok:
#         break

#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     corners, ids, rejected = detector.detectMarkers(gray)

#     if ids is not None:
#         aruco.drawDetectedMarkers(frame, corners, ids)

#         for i in range(len(ids)):
#             pts = corners[i][0]  # 4 corners
#             cx = int(np.mean(pts[:, 0]))
#             cy = int(np.mean(pts[:, 1]))
#             tag_id = int(ids[i][0])

#             cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)
#             cv2.putText(frame, f"ID {tag_id}", (cx + 10, cy),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

#             print(f"Tag {tag_id} at pixel ({cx}, {cy})")

#     cv2.imshow("ArUco Webcam", frame)
#     if (cv2.waitKey(1) & 0xFF) == ord("q"):
#         break

# cap.release()
# cv2.destroyAllWindows()




