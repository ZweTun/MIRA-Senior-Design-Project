# import cv2
# import numpy as np
# import serial  # For serial communication 

# # --- Serial protocol helpers ---
# SERIAL_ENABLE = False
# SERIAL_PORT = "COM11"
# SERIAL_BAUD = 250000

# # Serial command format: "
# # tag_id -> {"cell": (r,c) or None, "candidate": (r,c), "count": int, "last_seen": frame_idx}
# track = {}
# FRAME_IDX = 0
# STABLE_N = 3          # must see same cell this many frames to commit
# STALE_FRAMES = 15     # if not seen for this many frames, consider removed

# ser = None
# if SERIAL_ENABLE:
#     try:
#         ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.01)
#         ser.reset_input_buffer()
#         ser.reset_output_buffer()
#     except serial.SerialException as e:
#         print("Serial open failed:", e)
#         ser = None

# def send_serial_line(line: str):
#     # Newline is the frame delimiter (sync)
#     if ser is None:
#         return
#     # For debugging without serial, just print the line
#     print("SERIAL OUT:", line)
#     ser.write((line + "\n").encode("ascii", errors="ignore"))





# # ----------------------------
# # Config
# # ----------------------------
# GRID_ROWS, GRID_COLS = 8, 8
# CELL_PX = 80  # rectified pixels per cell (debug resolution)
# W, H = GRID_COLS * CELL_PX, GRID_ROWS * CELL_PX
# MARGIN_CELLS = 0.0 # margin around board

# # Corner tag IDs (YOU should print/put these on the 4 corners)
# # Recommended: 0,1,2,3
# CORNER_TAG_IDS = {
#     0: "TR",
#     1: "BR",
#     2: "BL",
#     3: "TL",
# }

# # Add margin corners to the rectified board (for better visualization)
# RECT_CORNERS = {
#     "TL": (-MARGIN_CELLS * CELL_PX, -MARGIN_CELLS * CELL_PX),
#     "TR": (W - 1 + MARGIN_CELLS * CELL_PX, -MARGIN_CELLS * CELL_PX),
#     "BR": (W - 1 + MARGIN_CELLS * CELL_PX, H - 1 + MARGIN_CELLS * CELL_PX),
#     "BL": (-MARGIN_CELLS * CELL_PX, H - 1 + MARGIN_CELLS * CELL_PX),
# }

# # ----------------------------
# # Helpers
# # ----------------------------
# def inner_corner_by_center(tag_pts, img_center):
#     # tag_pts: (4,2)
#     d2 = np.sum((tag_pts - img_center) ** 2, axis=1)
#     return tag_pts[np.argmin(d2)]

# def extreme_corner(tag_pts: np.ndarray, label: str) -> np.ndarray:
#     # tag_pts: (4,2)
#     x = tag_pts[:, 0]
#     y = tag_pts[:, 1]
#     s = x + y
#     d = x - y

#     if label == "TL":
#         return tag_pts[np.argmin(s)]
#     if label == "TR":
#         return tag_pts[np.argmax(d)]
#     if label == "BR":
#         return tag_pts[np.argmax(s)]
#     if label == "BL":
#         return tag_pts[np.argmin(d)]
#     raise ValueError("bad label")

# def compute_homography_inner(corners, ids, img_center):
#     if ids is None:
#         return None, None

#     found = {}
#     for i in range(len(ids)):
#         tid = int(ids[i][0])
#         if tid in CORNER_TAG_IDS:
#             label = CORNER_TAG_IDS[tid]
#             tag_pts = corners[i][0].astype(np.float32)  # (4,2)
#             found[label] = inner_corner_by_center(tag_pts, img_center)

#     if any(k not in found for k in ["TL", "TR", "BR", "BL"]):
#         return None, None

#     src = np.array([found["TL"], found["TR"], found["BR"], found["BL"]], dtype=np.float32)
#     dst = np.array([RECT_CORNERS["TL"], RECT_CORNERS["TR"], RECT_CORNERS["BR"], RECT_CORNERS["BL"]], dtype=np.float32)

#     Hmat = cv2.getPerspectiveTransform(src, dst)
#     return Hmat, found  # return the points used

# def rect_to_cell(xr: float, yr: float):
#     col = int(xr // CELL_PX)
#     row = int(yr // CELL_PX)
#     if 0 <= row < GRID_ROWS and 0 <= col < GRID_COLS:
#         return row, col
#     return None

# def draw_grid(img):
#     # draw 8x8 grid lines on rectified image
#     for c in range(1, GRID_COLS):
#         x = c * CELL_PX
#         cv2.line(img, (x, 0), (x, H - 1), (255, 255, 255), 1)
#     for r in range(1, GRID_ROWS):
#         y = r * CELL_PX
#         cv2.line(img, (0, y), (W - 1, y), (255, 255, 255), 1)

# def handle_stale():
#     # Call this periodically to remove stale tags
#     to_remove = []
#     for tag_id, state in track.items():
#         if FRAME_IDX - state["last_seen"] > STALE_FRAMES:
#             to_remove.append(tag_id)

#     for tag_id in to_remove:
#         del track[tag_id]
#         send_serial_line(f"P,{tag_id},{-1},{-1}")
#         # print("TX:", f"R,{tag_id}")

# def handle_offboard(tag_id: int):
#     # Call this if you detect a tag is off-board (e.g. via vision)
#     if tag_id in track and track[tag_id]["cell"] is not None:
#         track[tag_id]["cell"] = None
#         track[tag_id]["candidate"] = None
#         track[tag_id]["count"] = 0
#         send_serial_line(f"P,{tag_id},{-1},{-1}")
#         print("P:", f"P,{tag_id},{-1},{-1}")

# def handle_state(tag_id: int, row: int, col: int):
#     state = track.get(tag_id, {"cell": None, "candidate": None, "count": 0, "last_seen": FRAME_IDX})
#     state["last_seen"] = FRAME_IDX

#     cand = state["candidate"]
#     if cand == (row, col):
#         state["count"] += 1
#     else:
#         state["candidate"] = (row, col)
#         state["count"] = 1

#     # Commit if stable
#     # P command format: P,tag_id,row,col
#     if state["count"] >= STABLE_N and state["cell"] != state["candidate"]:
#         state["cell"] = state["candidate"]
#         r, c = state["cell"]
#         send_serial_line(f"P,{tag_id},{r},{c}")
#         # print("TX:", f"P,{tag_id},{r},{c}")

#     track[tag_id] = state


# # ----------------------------
# # Main
# # ----------------------------
# cap = cv2.VideoCapture(1)

# aruco = cv2.aruco
# dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
# params = aruco.DetectorParameters()
# detector = aruco.ArucoDetector(dictionary, params)

# print("Press q to quit")
# print("Corner tag IDs mapping:", CORNER_TAG_IDS)

# Hmat = None

# while True:
#     ok, frame = cap.read()
#     frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

#     if not ok:
#         break

#     # Increment frame index and handle stale tags
#     FRAME_IDX += 1          
      
#     h, w = frame.shape[:2]
#     img_center = np.array([w / 2, h / 2], dtype=np.float32)

#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     corners, ids, rejected = detector.detectMarkers(gray)

#     if ids is not None:
#         aruco.drawDetectedMarkers(frame, corners)

#         # Update homography if corner tags are all visible
        
#         newH, found_pts = compute_homography_inner(corners, ids, img_center)

#         # Debug: draw the found corner points and quadrilateral
#         if found_pts is not None:
#             quad = np.array([found_pts["TL"], found_pts["TR"], found_pts["BR"], found_pts["BL"]], dtype=np.int32)
#             cv2.polylines(frame, [quad], True, (255, 0, 0), 3)
#             for lab in ["TL", "TR", "BR", "BL"]:
#                 p = found_pts[lab]
#                 cv2.circle(frame, (int(p[0]), int(p[1])), 10, (255, 0, 0), 3)
#                 cv2.putText(frame, lab, (int(p[0]) + 5, int(p[1]) + 5),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)
#         if newH is not None:
#             Hmat = newH
#             cv2.putText(frame, "H: LOCKED", (10, 30),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
#         else:
#             cv2.putText(frame, "H: NEED 4 CORNERS", (10, 30),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

#         # If we have H, prepare rectified debug view
#         rectified = None
#         if Hmat is not None:
#             rectified = cv2.warpPerspective(frame, Hmat, (W, H))
#             draw_grid(rectified)

#         for i in range(len(ids)):
#             pts = corners[i][0]
#             cx, cy = np.mean(pts[:, 0]), np.mean(pts[:, 1])
#             tag_id = int(ids[i][0])

#             # Mark center in camera image
#             cv2.circle(frame, (int(cx), int(cy)), 3, (0, 255, 0), -1)

#             if Hmat is not None:
#                 p = np.array([[[cx, cy]]], dtype=np.float32)
#                 pr = cv2.perspectiveTransform(p, Hmat)[0][0]
#                 xr, yr = float(pr[0]), float(pr[1])

#                 cell = rect_to_cell(xr, yr)

#                 # Corner tags: show label
#                 corner_label = CORNER_TAG_IDS.get(tag_id, None)
#                 if corner_label is not None:
#                     txt = f"CORNER {corner_label} (ID {tag_id})"
#                 else:
#                     txt = f"ID {tag_id}"

#                 if cell is not None:
#                     row, col = cell
#                     # Send ID + Row/Col to microcontroller (placeholder function)
                    

#                     # Serail stability tracking
#                     if tag_id not in CORNER_TAG_IDS:
#                         handle_state(tag_id, row, col)
                    
#                     txt += f" -> cell ({row},{col})"
#                     cv2.putText(frame, txt, (int(cx) + 10, int(cy)),
#                                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

#                     if rectified is not None:
#                         cv2.circle(rectified, (int(xr), int(yr)), 6, (0, 255, 0), -1)
#                         cv2.putText(rectified, f"{tag_id} ({row},{col})",
#                                     (int(xr) + 6, int(yr) - 6),
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
#                 else:
#                     # Off-board (seen but outside grid)
#                     handle_offboard(tag_id)
#                     cv2.putText(frame, txt + " -> off-board", (int(cx) + 10, int(cy)),
#                                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)


              
#             else:
#                 cv2.putText(frame, f"ID {tag_id} (no H)", (int(cx) + 10, int(cy)),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
#     if FRAME_IDX % 40 == 0:
#         handle_stale()        
#     # if ser.in_waiting:
#     #     print("ESP:", ser.readline().decode(errors="ignore").strip())   
       

#     cv2.imshow("Camera", frame)
    

#     # Show rectified view if homography is locked
#     if Hmat is not None:
#         rectified = cv2.warpPerspective(frame, Hmat, (W, H))
#         draw_grid(rectified)
#         cv2.imshow("Rectified Board (8x8)", rectified)
      
#     if (cv2.waitKey(1) & 0xFF) == ord("q"):
#         break

# cap.release()
# cv2.destroyAllWindows()


# # import cv2
# # import numpy as np

# # cap = cv2.VideoCapture(1)

# # # Use an ArUco dictionary (like "AprilTag-lite")
# # aruco = cv2.aruco
# # dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
# # params = aruco.DetectorParameters()

# # detector = aruco.ArucoDetector(dictionary, params)

# # print("Press q to quit")

# # while True:
# #     ok, frame = cap.read()
# #     if not ok:
# #         break

# #     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# #     corners, ids, rejected = detector.detectMarkers(gray)

# #     if ids is not None:
# #         aruco.drawDetectedMarkers(frame, corners, ids)

# #         for i in range(len(ids)):
# #             pts = corners[i][0]  # 4 corners
# #             cx = int(np.mean(pts[:, 0]))
# #             cy = int(np.mean(pts[:, 1]))
# #             tag_id = int(ids[i][0])

# #             cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)
# #             cv2.putText(frame, f"ID {tag_id}", (cx + 10, cy),
# #                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

# #             print(f"Tag {tag_id} at pixel ({cx}, {cy})")

# #     cv2.imshow("ArUco Webcam", frame)
# #     if (cv2.waitKey(1) & 0xFF) == ord("q"):
# #         break

# # cap.release()
# # cv2.destroyAllWindows()




# import numpy as np
# import cv2 as cv
# cap = cv.VideoCapture(0)
# ret, frame1 = cap.read()
# prvs = cv.cvtColor(frame1, cv.COLOR_BGR2GRAY)
# hsv = np.zeros_like(frame1)
# hsv[..., 1] = 255
# while(1):
#     ret, frame2 = cap.read()
#     if not ret:
#         print('No frames grabbed!')
#         break
#     next = cv.cvtColor(frame2, cv.COLOR_BGR2GRAY)
#     flow = cv.calcOpticalFlowFarneback(prvs, next, None, 0.5, 3, 15, 3, 5, 1.2, 0)
#     mag, ang = cv.cartToPolar(flow[..., 0], flow[..., 1])
#     hsv[..., 0] = ang*180/np.pi/2
#     hsv[..., 2] = cv.normalize(mag, None, 0, 255, cv.NORM_MINMAX)
#     bgr = cv.cvtColor(hsv, cv.COLOR_HSV2BGR)
#     cv.imshow('frame2', bgr)
#     k = cv.waitKey(30) & 0xff
#     if k == 27:
#         break
#     elif k == ord('s'):
#         cv.imwrite('opticalfb.png', frame2)
#         cv.imwrite('opticalhsv.png', bgr)
#     prvs = next
# cv.destroyAllWindows()

import cv2 as cv
import numpy as np

# ---------------------------
# Globals
# ---------------------------
curr_x, curr_y = 0, 0
points = None

# ---------------------------
# Function: detect AprilTag and update hand ROI
# ---------------------------
def define_roi_apriltag(frame, detector):
    global curr_x, curr_y
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        # just take the first detected tag for simplicity
        pts = corners[0][0]
        curr_x = int(np.mean(pts[:, 0]))
        curr_y = int(np.mean(pts[:, 1]))
        tag_id = int(ids[0][0])
        # draw tag
        cv.circle(frame, (curr_x, curr_y), 6, (0, 255, 0), -1)
        cv.putText(frame, f"ID {tag_id}", (curr_x + 10, curr_y),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return frame

# ---------------------------
# Initialize ArUco detector
# ---------------------------
aruco = cv.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

# ---------------------------
# Initialize video
# ---------------------------
cap = cv.VideoCapture(0)
ret, frame1 = cap.read()
prvs = cv.cvtColor(frame1, cv.COLOR_BGR2GRAY)

# ---------------------------
# Main loop
# ---------------------------
while True:
    ret, frame2 = cap.read()
    if not ret:
        break
    gray = cv.cvtColor(frame2, cv.COLOR_BGR2GRAY)

    # Detect AprilTag & define ROI
    define_roi_apriltag(frame2, detector)
    roi_size = 100
    roi_x1 = max(0, curr_x - roi_size)
    roi_y1 = max(0, curr_y - roi_size)
    roi_x2 = min(frame2.shape[1], curr_x + roi_size)
    roi_y2 = min(frame2.shape[0], curr_y + roi_size)

    # Draw ROI rectangle
    cv.rectangle(frame2, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 0, 0), 2)

    # ---------------------------
    # Initialize points if not yet
    # ---------------------------
    if points is None:
        roi_gray = gray[roi_y1:roi_y2, roi_x1:roi_x2]
        points = cv.goodFeaturesToTrack(roi_gray, maxCorners=20,
                                        qualityLevel=0.3, minDistance=7)
        if points is not None:
            # offset points to global frame coordinates
            points += np.array([[roi_x1, roi_y1]], dtype=np.float32)

    # ---------------------------
    # Track points using Lucas–Kanade optical flow
    # ---------------------------
    if points is not None:
        new_points, status, _ = cv.calcOpticalFlowPyrLK(prvs, gray, points, None)
        good_new = new_points[status == 1]
        good_old = points[status == 1]

        # Draw tracked points
        for pt in good_new:
            x, y = pt.ravel()
            cv.circle(frame2, (int(x), int(y)), 4, (0, 0, 255), -1)

        # Estimate blob centroid
        if len(good_new) > 0:
            c_x = int(np.mean(good_new[:, 0]))
            c_y = int(np.mean(good_new[:, 1]))
            cv.circle(frame2, (c_x, c_y), 8, (0, 255, 255), -1)
            cv.putText(frame2, "Tracked block", (c_x + 10, c_y),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Update points for next frame
        points = good_new.reshape(-1, 1, 2)

    # Show result
    cv.imshow("Optical Flow Block Tracker", frame2)
    k = cv.waitKey(30) & 0xff
    if k == 27:
        break

    # Update previous frame
    prvs = gray.copy()

cap.release()
cv.destroyAllWindows()