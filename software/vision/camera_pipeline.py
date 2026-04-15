from enum import auto
import importlib

import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional
from planner.models import CellPos

from planner.mosaic import COLOR_TO_ID, ID_TO_COLOR, TAG_TO_COLOR_ID

_picamera2_spec = importlib.util.find_spec("picamera2")
if _picamera2_spec is not None:
    Picamera2 = importlib.import_module("picamera2").Picamera2
else:
    Picamera2 = None



GRID_ROWS, GRID_COLS = 8, 8
CELL_PX = 80
W, H = GRID_COLS * CELL_PX, GRID_ROWS * CELL_PX
MARGIN_CELLS = 0.

USER_TAG = 4

WRIST_TAG_CONFIGS = [
    4,
    5,
    6
]

# Map cell to hsv offsets from baseline for color classification, 
# to account for lighting variations
HUE_OFFSETS = {}
SAT_OFFSETS = {}
VAL_OFFSETS = {}

 
CORNER_TAG_IDS = {
    0: "TR",
    1: "BR",
    2: "BL",
    3: "TL",
}

RECT_CORNERS = {
    "TL": (-MARGIN_CELLS * CELL_PX, -MARGIN_CELLS * CELL_PX),
    "TR": (W - 1 + MARGIN_CELLS * CELL_PX, -MARGIN_CELLS * CELL_PX),
    "BR": (W - 1 + MARGIN_CELLS * CELL_PX, H - 1 + MARGIN_CELLS * CELL_PX),
    "BL": (-MARGIN_CELLS * CELL_PX, H - 1 + MARGIN_CELLS * CELL_PX),
}

COLOR_TO_BGR = {
    # "red": (0, 0, 255),
    "green": (0, 255, 0),
    # "magenta": (255, 0, 255),
    # "cyan": (255, 255, 0),
    "yellow": (0, 255, 255),
    "blue": (255, 0, 0),
    "brown": (19, 69, 139),
    "black": (0, 0, 0),
}

REFERENCE_HUES = {
    "yellow": 24,
    # "orange": 15,
    # "red": 5,
    "green": 38,
    "blue": 105,
    # "cyan": 90,
    # "magenta": 145,
    "black": 0,
}




@dataclass
class TileEvent:
    tag_id: int
    row: int
    col: int
    kind: str  # "onboard", "removed", "offboard"


class CameraPipeline:
    def __init__(
        self,
        grid,
        camera_index: int = 1,
        stable_n: int = 3,
        stale_frames: int = 15,
        use_picamera2: bool = False,
        picamera_size: tuple[int, int] = (1280, 720),
    ):
        self.cap = None
        self.picam2 = None
        self.use_picamera2 = False

        if use_picamera2:
            if Picamera2 is None:
                print("Picamera2 requested but not installed; falling back to OpenCV VideoCapture.")
            else:
                try:
                    self.picam2 = Picamera2()
                    config = self.picam2.create_preview_configuration(
                        main={"format": "BGB888", "size": picamera_size}
                    )
                    self.picam2.configure(config)
                    self.picam2.start()
                    self.use_picamera2 = True
                    print(f"Using Picamera2 capture at {picamera_size}.")
                except Exception as e:
                    print(f"Picamera2 initialization failed ({e}); falling back to OpenCV VideoCapture.")
                    self.picam2 = None

        if not self.use_picamera2:
            self.cap = cv2.VideoCapture(camera_index)
        self.grid = grid

        self.aruco = cv2.aruco
        self.dictionary = self.aruco.getPredefinedDictionary(self.aruco.DICT_APRILTAG_36h11)
        self.params = cv2.aruco.DetectorParameters()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.params.adaptiveThreshWinSizeMin = 3
        self.params.adaptiveThreshWinSizeMax = 23
        self.params.adaptiveThreshWinSizeStep = 4
        self.detector = self.aruco.ArucoDetector(self.dictionary, self.params)

        self.track = {}
        self.frame_idx = 0
        self.stable_n = stable_n
        self.stale_frames = stale_frames
        self.hover_stable_n = max(1, stable_n // 3)
        self.hover_snap_margin_px = 10
        self.Hmat = None

        self.baseline_hsv = {}
        self.init = 0

        self.block_hsv_lower = None
        self.block_hsv_upper = None
        self.last_frame = None

        self.sample_interval = 30  # resample every 30 frames
        self.last_sample_frame = -999
        self.roi_h = 120
        self.roi_w = 130

        self.locked_hue = None
        # lower hue lock tolerance means more stable color detection
        # but less adaptability to lighting changes; 
        # higher tolerance allows more drift but can handle more variation
        self.hue_lock_tolerance = 25 
        self.hue_drift_count = 0

        self.user_stable_frames = 5
        self.skin_hsv_lower = None
        self.skin_hsv_upper = None
        self.curr_color = None

        self.contour_count = 0

        self.skin_calibrated_frame = -999
        self.skin_recalibrate_interval = 300  # every 10s at 30fps
        self.sample_miss_count = 0
        self.max_sample_miss_before_reset = 10
        self.last_forced_unlock_frame = -999
        self.unlock_reacquire_window = 60
        self.skin_recalibration_after_unlock_cooldown = 120

        self.raw_block_cell = None
        self.raw_block_candidate = None
        self.raw_block_count = 0

    def classify_color_from_hsv(self, h, s, v):
        if v < 50:
            return "empty"

        if s < 10:
            return "unknown"

        best_color = None
        best_dist = float("inf")

        for color, ref_h in REFERENCE_HUES.items():
            dist = abs(h - ref_h)
            dist = min(dist, 180 - dist)

            if dist < best_dist:
                best_dist = dist
                best_color = color

        # Not confident if hue is too far from any reference
        if best_dist > 20:
            return "unknown"

        return best_color

    def build_right_aligned_roi(self, tag_pts, tag_id=USER_TAG, roi_forward=None, roi_height=None, gap_px=10):
        tl, tr, br, bl = tag_pts.astype(np.float32)
        center = np.mean(tag_pts, axis=0)

        right_vec = tr - tl
        right_len = np.linalg.norm(right_vec)
        right_unit = right_vec / right_len if right_len > 1e-6 else np.array([1.0, 0.0])

        perp_unit = np.array([-right_unit[1], right_unit[0]], dtype=np.float32)

        marker_width = 0.5 * (np.linalg.norm(tr - tl) + np.linalg.norm(br - bl))
        marker_half_width = marker_width * 0.5

        # Scale ROI with tag size — bigger tag (closer) = bigger ROI

        scale = marker_width / 50.0
        forward = float(self.roi_w if roi_forward is None else roi_forward) * scale
        height = float(self.roi_h if roi_height is None else roi_height) * scale
        offset = perp_unit * (height * 0.2 + gap_px)  # gap between tag and ROI

        roi_center = center + right_unit * (marker_half_width + gap_px + forward * 0.5) + offset
  

        half_forward = right_unit * (forward * 0.5)
        half_height = perp_unit * (height * 0.5)
        # Angle ROI perpendicularly 
        # by 30 degrees for better block detection when user is placing from the side
        angle_rad = np.radians(230)
        cos_angle = np.cos(angle_rad)
        sin_angle = np.sin(angle_rad)
        half_forward = np.array([
            half_forward[0] * cos_angle - half_forward[1] * sin_angle,
            half_forward[0] * sin_angle + half_forward[1] * cos_angle
        ], dtype=np.float32)
        half_height = np.array([
            half_height[0] * cos_angle - half_height[1] * sin_angle,
            half_height[0] * sin_angle + half_height[1] * cos_angle
        ], dtype=np.float32)

        p1 = roi_center - half_forward - half_height
        p2 = roi_center + half_forward - half_height
        p3 = roi_center + half_forward + half_height
        p4 = roi_center - half_forward + half_height
        return np.array([p1, p2, p3, p4], dtype=np.float32)

    def build_skin_mask(self, hsv):
        skin_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        if self.skin_hsv_lower is not None and self.skin_hsv_upper is not None:
            calibrated_mask = cv2.inRange(hsv, self.skin_hsv_lower, self.skin_hsv_upper)
            skin_mask = cv2.bitwise_or(skin_mask, calibrated_mask)

        # Conservative fallback catches typical skin tones but avoids very saturated red blocks.
        # Keep fallback skin hue narrow to avoid masking yellow blocks.
        fallback_lower = np.array([0, 25, 70], dtype=np.uint8)
        fallback_upper = np.array([18, 150, 255], dtype=np.uint8)
        fallback_mask = cv2.inRange(hsv, fallback_lower, fallback_upper)
        skin_mask = cv2.bitwise_or(skin_mask, fallback_mask)

        return skin_mask

    def detect_held_block_cell(self, frame, tag_pts, best_user_tag):
        if self.block_hsv_lower is None or self.block_hsv_upper is None:
            return None

        roi_poly = self.build_right_aligned_roi(tag_pts, tag_id=best_user_tag)
        if roi_poly is None:
            return None
        poly_i = np.round(roi_poly).astype(np.int32)

        roi_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.fillConvexPoly(roi_mask, poly_i, 255)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        color_mask = cv2.inRange(hsv, self.block_hsv_lower, self.block_hsv_upper)
        mask = cv2.bitwise_and(color_mask, color_mask, mask=roi_mask)
        # Remove skin-colored pixels to avoid interference, then apply block color mask
        skin_mask = self.build_skin_mask(hsv)
        non_skin = cv2.bitwise_not(skin_mask)
        mask = cv2.bitwise_and(mask, mask, mask=non_skin)

        # Draw oriented ROI for debugging.
        cv2.polylines(frame, [poly_i], True, (255, 0, 0), 2)
        # finds the contours in the masked image and keeps the largest one, 
        # which should correspond to the held block, 
        # then finds its center and maps to cell coordinates
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.contour_count += 1
            # Print hsv thresholds and debug info for why block detection might be failing
            # print(f"Block detection: no contours found. HSV lower: {self.block_hsv_lower}, upper: {self.block_hsv_upper}"   )
            # print("No contours found for held block detection.")
            if self.contour_count > 8:
                # print("No contours for locked color; forcing color unlock for resample.")
                self.last_forced_unlock_frame = self.frame_idx
                self.reset_block_color()
                self.contour_count = 0
            return None

        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 200:
            # print(f"Contour found but too small (area={cv2.contourArea(c)}), likely noise.")
            # if countour found but small count is 
            # more than threshold reset block color to allow resampling, 
            # as user might be placing a different block
            self.contour_count += 1
            if self.contour_count > 20:  # Example threshold, adjust as needed
                # print("Multiple frames with small contours, resetting block color to allow resampling.")
                self.reset_block_color()
                self.contour_count = 0
            return None
        self.contour_count = 0  # reset count if we find a valid contour
        M = cv2.moments(c)
        if M["m00"] == 0:
            return None

        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]

        p = np.array([[[cx, cy]]], dtype=np.float32)
        pr = cv2.perspectiveTransform(p, self.Hmat)[0][0]
        cell = self.rect_to_cell_with_hysteresis(
            float(pr[0]),
            float(pr[1]),
            self.raw_block_candidate,
        )

        
        if cell is None:
            return None

        cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
       
        return cell
    def compute_global_lighting_offset(self):
        if not SAT_OFFSETS or not VAL_OFFSETS:
            return 0, 0
        avg_s_offset = np.mean(list(SAT_OFFSETS.values()))
        avg_v_offset = np.mean(list(VAL_OFFSETS.values()))
        return avg_s_offset, avg_v_offset
    
    # Sample block color from user tag ROI, 
    # using motion and size to find the most likely block region, and lock hue for stability during placement
    def sample_block_color(self, frame, tag_pts, best_user_tag):
        if self.last_frame is None:
            self.last_frame = frame.copy()
            return False, None

        roi_poly = self.build_right_aligned_roi(tag_pts, tag_id=best_user_tag)
        if roi_poly is None:
            print("Cannot build ROI for block sampling, likely due to tag orientation. Resetting block color lock.")
            return False, None
        poly_i = np.round(roi_poly).astype(np.int32)

        roi_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.fillConvexPoly(roi_mask, poly_i, 255)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        skin_mask_full = self.build_skin_mask(hsv)

        pixels = hsv[roi_mask == 255].astype(np.float32)
        skin_pixels = skin_mask_full[roi_mask == 255] > 0

        if len(pixels) < 50:
            print("Not enough pixels in ROI for block sampling.")
            return False, None

        dark_mask = pixels[:, 2] < 40
        valid = pixels[~skin_pixels & ~dark_mask]

        if len(valid) < 50:
            print(f"Not enough valid pixels after skin/dark filtering: {len(valid)}")
            return False, None

        # Derive thresholds from the current ROI so sampling adapts to lighting.
        sat_values = valid[:, 1]
        val_values = valid[:, 2]
        dynamic_min_sat = float(np.percentile(sat_values, 35))
        dynamic_min_val = float(np.percentile(val_values, 30))
        dynamic_min_cluster_pixels = int(max(np.sqrt(len(valid)), len(valid) * 0.05))

        # Get more clusters to find multiple candidate blocks
        k = min(3, len(valid) // 50)
        if k < 2:
            k = 2

        _, labels, centers = cv2.kmeans(
            valid, k, None,
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0),
            3, cv2.KMEANS_RANDOM_CENTERS
        )

        # Compute optical flow in ROI to find moving pixels
        prev_gray = cv2.cvtColor(self.last_frame, cv2.COLOR_BGR2GRAY)
        curr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        flow = cv2.calcOpticalFlowFarneback(
            prev_gray, curr_gray, None,
            0.5, 3, 15, 3, 5, 1.2, 0
        )
        flow_mag = np.sqrt(flow[..., 0]**2 + flow[..., 1]**2)  # motion magnitude per pixel
        roi_motion = flow_mag[roi_mask > 0]
        motion_floor = float(np.percentile(roi_motion, 70)) if roi_motion.size > 0 else 0.0
        dynamic_score_threshold = motion_floor * np.log1p(max(dynamic_min_cluster_pixels, 1)) * 0.35
        # between 5 and 12.0 based on observed scores; adjust as needed
        dynamic_score_threshold = float(np.clip(dynamic_score_threshold, 5, 12.0))

        # Re-run inRange per cluster center to get pixel membership in full frame
        best_cluster = None
        best_score = -1

        for ci, center in enumerate(centers):
            h, s, v = center
            if s < dynamic_min_sat or v < dynamic_min_val:
                continue

            sat_margin = float(np.std(sat_values))
            val_margin = float(np.std(val_values))
            lower = np.array([max(0, h - 20), max(0, s - sat_margin), max(0, v - val_margin)], dtype=np.uint8)
            upper = np.array([min(180, h+20), 255, 255], dtype=np.uint8)

            cluster_mask = cv2.inRange(hsv, lower, upper)
            cluster_mask = cv2.bitwise_and(cluster_mask, cluster_mask, mask=roi_mask)

            cluster_pixels = np.sum(cluster_mask > 0)
            if cluster_pixels < dynamic_min_cluster_pixels:
                continue

            skin_fraction = np.mean(skin_mask_full[cluster_mask > 0] > 0)
            if skin_fraction > 0.35:
                continue

            # Average motion magnitude for this cluster's pixels
            avg_motion = np.mean(flow_mag[cluster_mask > 0])
            hue_penalty = 1.0
            if self.locked_hue is not None:
                hue_dist = min(abs(h - self.locked_hue), 180 - abs(h - self.locked_hue))
                hue_penalty = 1.0 / (1.0 + hue_dist / 30.0)  # drops off with distance

            # weight
            w1 = 1.0 # motion weight
       
            score = w1 * avg_motion * np.log1p(cluster_pixels) * hue_penalty



            # print(f"Cluster {ci}: h={h:.1f} s={s:.1f} motion={avg_motion:.2f} px={cluster_pixels} score={score:.2f}")

            if score > best_score:
                best_score = score
                best_cluster = center

        if best_cluster is None:
            # self.sample_miss_count += 1
            # if self.sample_miss_count >= self.max_sample_miss_before_reset and self.locked_hue is not None:
            #     print("Sampling repeatedly failed; resetting stale hue lock")
            #     self.reset_block_color()
            #     self.sample_miss_count = 0
            return False, None
        
        if best_score < dynamic_score_threshold:
            # print(f"No cluster passed the score threshold. Best score: {best_score:.2f}")
            # self.sample_miss_count += 1
            # if self.sample_miss_count >= self.max_sample_miss_before_reset and self.locked_hue is not None:
            #     print("Low-score sampling persisted; resetting stale hue lock")
            #     self.reset_block_color()
            #     self.sample_miss_count = 0
            return False, None


        h, s, v = best_cluster
        if s < dynamic_min_sat or v < dynamic_min_val:
            return False, None

        sampled_color = self.classify_color_from_hsv(h, s, v)

        # Ignore non-block samples so the black board cannot become the locked color.
        if sampled_color in ("black", "empty", "unknown"):
            return False, None

        candidate_lower = np.array([max(0, h - 20), max(0, s - 60), max(0, v - 60)], dtype=np.uint8)
        candidate_upper = np.array([min(180, h + 20), 255, 255], dtype=np.uint8)

        # Require a substantial non-skin colored blob in ROI before accepting a held block color.
        candidate_mask = cv2.inRange(hsv, candidate_lower, candidate_upper)
        candidate_mask = cv2.bitwise_and(candidate_mask, candidate_mask, mask=roi_mask)
        non_skin = cv2.bitwise_not(skin_mask_full)
        candidate_mask = cv2.bitwise_and(candidate_mask, candidate_mask, mask=non_skin)
        candidate_mask = cv2.morphologyEx(
            candidate_mask,
            cv2.MORPH_OPEN,
            np.ones((3, 3), dtype=np.uint8),
        )

        candidate_contours, _ = cv2.findContours(candidate_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not candidate_contours:
            return False, None

        largest_candidate_area = cv2.contourArea(max(candidate_contours, key=cv2.contourArea))
        roi_area = float(max(np.count_nonzero(roi_mask), 1))
        min_candidate_area = max(180.0, roi_area * 0.015)
        if largest_candidate_area < min_candidate_area:
            return False, None

        # If we locked hue, only accept if new sample is close
        if self.locked_hue is not None:
            hue_dist = min(abs(h - self.locked_hue), 180 - abs(h - self.locked_hue))
            if hue_dist > self.hue_lock_tolerance:
                # print(f"Resample rejected: h={h:.1f} too far from locked h={self.locked_hue:.1f}")
                self.hue_drift_count = getattr(self, 'hue_drift_count', 0) + 1
                if self.hue_drift_count > 0:  # consistently seeing different hue = new block
                    print(f"Hue drift detected ({self.hue_drift_count} frames), resetting lock")
                    self.reset_block_color()
                    self.hue_drift_count = 0
        
                return False, None  # keep existing bounds

        self.hue_drift_count = 0
        self.sample_miss_count = 0
        self.locked_hue = h
        # print(f"Selected block HSV: h={h:.1f} s={s:.1f} v={v:.1f}")
        self.block_hsv_lower = candidate_lower
        self.block_hsv_upper = candidate_upper
        self.last_frame = frame.copy()
        color = sampled_color
        # BLOCK USER HOLDING
        # print(f"Sampled block color: {color} (h={h:.1f}, s={s:.1f}, v={v:.1f}) with score {best_score:.2f}")
        return True, color 
    
    def reset_block_color(self):
    
        self.block_hsv_lower = None
        self.block_hsv_upper = None
        self.locked_hue = None
        self.curr_color = None
        self.last_sample_frame = -999
        self.raw_block_candidate = None
        self.raw_block_count = 0
        self.raw_block_cell = None
        # print("Block color reset")
 

    def extract_avg_hsv_of_cell(self, frame, row, col):
        x1 = col * CELL_PX
        y1 = row * CELL_PX
        x2 = x1 + CELL_PX
        y2 = y1 + CELL_PX

        margin = int(CELL_PX * 0.35)  # 30% inset on each side

        x1 += margin
        y1 += margin
        x2 -= margin
        y2 -= margin

        cell_img = frame[y1:y2, x1:x2]
        # Debug drawing of cell region
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        hsv_cell = cv2.cvtColor(cell_img, cv2.COLOR_BGR2HSV)
        avg_hsv = np.median(hsv_cell.reshape(-1, 3), axis=0)
        return avg_hsv
    

    
    def extract_baseline_hsv(self, frame):
        baseline_hsv = {}

        for row in range(GRID_ROWS):
            for col in range(GRID_COLS):
                curr = self.extract_avg_hsv_of_cell(frame, row, col)

            
                HUE_OFFSETS[(row, col)] = 0

                # Calibrate saturation and value offsets to 
                # make all cells appear as "empty" (black) at baseline,
                SAT_OFFSETS[(row, col)] = -curr[1]
                VAL_OFFSETS[(row, col)] = -curr[2]

                adjusted_h = curr[0]
                adjusted_s = max(0, min(255, curr[1] + SAT_OFFSETS[(row, col)]))
                adjusted_v = max(0, min(255, curr[2] + VAL_OFFSETS[(row, col)]))

                baseline_hsv[(row, col)] = (adjusted_h, adjusted_s, adjusted_v)

                print(f"Raw baseline HSV for cell ({row}, {col}): {curr}")
                print(f"Adjusted baseline HSV for cell ({row}, {col}): {baseline_hsv[(row, col)]}")

        return baseline_hsv

    def update_grid(self, grid):
        self.grid = grid

    def draw_mosaic_overlay(self, base_img, grid, alpha=0.2):
        overlay = base_img.copy()

        for row in range(len(grid)):
            for col in range(len(grid[row])):
                color_name = grid[row][col]
                # print(f"Drawing cell ({row}, {col}) with color {color_name}")
                color = COLOR_TO_BGR[color_name]

                x1 = col * CELL_PX
                y1 = row * CELL_PX
                x2 = x1 + CELL_PX
                y2 = y1 + CELL_PX

                cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)

        return cv2.addWeighted(overlay, alpha, base_img, 1 - alpha, 0)

    def close(self):
        if self.picam2 is not None:
            try:
                self.picam2.stop()
                self.picam2.close()
            except Exception:
                pass
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()

    def read_frame(self):
        if self.use_picamera2 and self.picam2 is not None:
            try:
                frame = self.picam2.capture_array()
            except Exception:
                return False, None
            if frame is None:
                return False, None
            
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            return True, frame

        if self.cap is None:
            return False, None
        return self.cap.read()

    def inner_corner_by_center(self, tag_pts, img_center):
        d2 = np.sum((tag_pts - img_center) ** 2, axis=1)
        return tag_pts[np.argmin(d2)]

    def compute_homography_inner(self, corners, ids, img_center):
        if ids is None:
            return None, None

        found = {}
        for i in range(len(ids)):
            tid = int(ids[i][0])
            if tid in CORNER_TAG_IDS:
                label = CORNER_TAG_IDS[tid]
                tag_pts = corners[i][0].astype(np.float32)
                found[label] = self.inner_corner_by_center(tag_pts, img_center)

        if any(k not in found for k in ["TL", "TR", "BR", "BL"]):
            return None, None

        src = np.array(
            [found["TL"], found["TR"], found["BR"], found["BL"]],
            dtype=np.float32
        )
        dst = np.array(
            [RECT_CORNERS["TL"], RECT_CORNERS["TR"], RECT_CORNERS["BR"], RECT_CORNERS["BL"]],
            dtype=np.float32
        )

        Hmat = cv2.getPerspectiveTransform(src, dst)
        return Hmat, found

    def rect_to_cell(self, xr: float, yr: float) -> Optional[tuple[int, int]]:
        col = int(xr // CELL_PX)
        row = int(yr // CELL_PX)
        if 0 <= row < GRID_ROWS and 0 <= col < GRID_COLS:
            return row, col
        return None

    def rect_to_cell_with_hysteresis(
        self,
        xr: float,
        yr: float,
        prev_cell: Optional[tuple[int, int]],
    ) -> Optional[tuple[int, int]]:
        snapped = self.rect_to_cell(xr, yr)
        if prev_cell is None:
            return snapped

        prev_row, prev_col = prev_cell
        if not (0 <= prev_row < GRID_ROWS and 0 <= prev_col < GRID_COLS):
            return snapped

        m = float(max(0, self.hover_snap_margin_px))
        x1 = prev_col * CELL_PX - m
        x2 = (prev_col + 1) * CELL_PX + m
        y1 = prev_row * CELL_PX - m
        y2 = (prev_row + 1) * CELL_PX + m

        # Stick to previous cell until centroid moves clearly into a neighbor.
        if x1 <= xr < x2 and y1 <= yr < y2:
            return prev_cell
        return snapped

    def draw_grid(self, img):
        for c in range(1, GRID_COLS):
            x = c * CELL_PX
            cv2.line(img, (x, 0), (x, H - 1), (255, 255, 255), 1)
        for r in range(1, GRID_ROWS):
            y = r * CELL_PX
            cv2.line(img, (0, y), (W - 1, y), (255, 255, 255), 1)

    def ensure_track_state(self, tag_id: int):
        state = self.track.setdefault(tag_id, {})
        state.setdefault("cell", None)
        state.setdefault("candidate", None)
        state.setdefault("count", 0)
        state.setdefault("last_seen", self.frame_idx)
        return state

    def handle_state(self, tag_id: int, row: int, col: int):
        state = self.ensure_track_state(tag_id)
        state["last_seen"] = self.frame_idx
        curr_pos = (row, col)

        cand = state["candidate"]
        if cand == curr_pos:
            state["count"] += 1
        else:
            state["candidate"] = curr_pos
            state["count"] = 1

        event = None
        if state["count"] >= self.stable_n and state["cell"] != state["candidate"]:
            state["cell"] = state["candidate"]
            r, c = state["cell"]
            event = TileEvent(tag_id=tag_id, row=r, col=c, kind="onboard")
            

        self.track[tag_id] = state
        return event

    def handle_offboard(self, tag_id: int):
        if tag_id not in self.track:
            return None
        
    
        state = self.ensure_track_state(tag_id)
        if state["cell"] is not None:
            state["cell"] = None
            state["candidate"] = None
            state["count"] = 0
            return TileEvent(tag_id=tag_id, row=-1, col=-1, kind="removed")
        return None

    def handle_stale(self):
        events = []
        to_remove = []
        for tag_id, state in self.track.items():
            last_seen = state.get("last_seen", self.frame_idx)
            if tag_id in WRIST_TAG_CONFIGS:
                if self.frame_idx - last_seen > self.user_stable_frames:
                    # Only reset if ALL wrist tags are stale
                    any_wrist_visible = any(
                        self.track.get(wid, {}).get("last_seen", 0) > self.frame_idx - self.user_stable_frames
                        for wid in WRIST_TAG_CONFIGS
                    )
                    if not any_wrist_visible:
                        self.reset_block_color()
            
            if self.frame_idx - last_seen > self.stale_frames:
                to_remove.append(tag_id)

        for tag_id in to_remove:
            del self.track[tag_id]
            events.append(TileEvent(tag_id=tag_id, row=-1, col=-1, kind="removed"))

        return events

    def calibrate_skin(self, frame, tag_pts):
        roi_poly = self.build_right_aligned_roi(tag_pts)
        if roi_poly is None:
            return False
        poly_i = np.round(roi_poly).astype(np.int32)

        roi_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.fillConvexPoly(roi_mask, poly_i, 255)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        pixels = hsv[roi_mask == 255].astype(np.float32)

        if len(pixels) < 80:
            return False

        # Keep only plausible skin-like pixels so dark board/background cannot dominate.
        candidate_mask = (
            (pixels[:, 0] <= 20) &
            (pixels[:, 1] >= 20) &
            (pixels[:, 1] <= 200) &
            (pixels[:, 2] >= 60)
        )
        candidates = pixels[candidate_mask]

        if len(candidates) < 60:
            print(f"Skin calibration skipped: not enough skin-like pixels ({len(candidates)}).")
            return False

        # Robust center estimate from percentiles to avoid outliers.
        h = float(np.median(candidates[:, 0]))
        s = float(np.median(candidates[:, 1]))
        v = float(np.median(candidates[:, 2]))

        if s < 20 or v < 60:
            print(f"Skin calibration rejected: weak sample (h={h:.1f}, s={s:.1f}, v={v:.1f}).")
            return False

        self.skin_hsv_lower = np.array(
            [max(0, h - 12), max(10, s - 55), max(30, v - 85)],
            dtype=np.uint8,
        )
        self.skin_hsv_upper = np.array(
            [min(180, h + 12), min(220, s + 65), 255],
            dtype=np.uint8,
        )
        self.skin_calibrated_frame = self.frame_idx
        print(
            f"Skin calibrated: h={h:.1f}, s={s:.1f}, v={v:.1f}, "
            f"range=({self.skin_hsv_lower.tolist()} to {self.skin_hsv_upper.tolist()})"
        )
        return True
    
    def update_user_tag(self, ids, corners):
        best_wrist_tag = None
        best_wrist_size = 0

        for i in range(len(ids)):
            pts = corners[i][0]
            tag_id = int(ids[i][0])
            
            if tag_id in WRIST_TAG_CONFIGS:
                marker_width = 0.5 * (
                    np.linalg.norm(pts[1] - pts[0]) + 
                    np.linalg.norm(pts[2] - pts[3])
                )
                if marker_width > best_wrist_size:
                    best_wrist_size = marker_width
                    best_wrist_tag = (tag_id, pts, i)
        return best_wrist_tag

    # Takes list of cells to check 
    def step(self, cells_to_check=None):
        ok, frame = self.read_frame()
        if not ok:
            return None, [], True, []
        
       
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        self.frame_idx += 1
        self.raw_block_cell = None

        events = []
        # boolean array will be init if cells to check not None
        color_list = None
      
           
        h, w = frame.shape[:2]
        img_center = np.array([w / 2, h / 2], dtype=np.float32)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            self.aruco.drawDetectedMarkers(frame, corners)

            newH, found_pts = self.compute_homography_inner(corners, ids, img_center)
            if newH is not None:
                self.Hmat = newH
                cv2.putText(frame, "H: LOCKED", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            else:
                cv2.putText(frame, "H: NEED 4 CORNERS", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            
            best_user_tag = self.update_user_tag(ids, corners) if ids is not None else None
            best_tag_id, best_tag_pts, best_tag_idx = best_user_tag if best_user_tag else (None, None, None)

            for i in range(len(ids)):
                pts = corners[i][0]
                cx, cy = np.mean(pts[:, 0]), np.mean(pts[:, 1])
                tag_id = int(ids[i][0])

                cv2.circle(frame, (int(cx), int(cy)), 3, (0, 255, 0), -1)

                cell = None
                if self.Hmat is not None:
                    p = np.array([[[cx, cy]]], dtype=np.float32)
                    pr = cv2.perspectiveTransform(p, self.Hmat)[0][0]
                    xr, yr = float(pr[0]), float(pr[1])
                    cell = self.rect_to_cell(xr, yr)
                    # Special handling for user tag to detect held block and sample color
                
                if best_user_tag is not None and self.Hmat is not None and tag_id == best_tag_id:
                    if self.skin_hsv_lower is None or self.skin_hsv_upper is None:
                        if self.calibrate_skin(frame, best_tag_pts):
                            print("Initial skin calibration complete.")
                    user_state = self.ensure_track_state(best_tag_id)
                    user_state["last_cx"] = cx
                    user_state["last_cy"] = cy
                    user_state["last_seen"] = self.frame_idx
                    # Record tag orientation for better block detection 
                    # user_state["orientation"] = self.compute_tag_orientation(pts)

                    # Auto-resample block color every N frames
                 
                    active_lock = self.block_hsv_lower is not None or self.locked_hue is not None
                    recently_forced_unlocked = (
                        self.frame_idx - self.last_forced_unlock_frame <= self.unlock_reacquire_window
                    )
                    sample_interval_frames = 3 if (active_lock or recently_forced_unlocked) else self.sample_interval
                    if self.frame_idx - self.last_sample_frame >= sample_interval_frames:
                        ok, color = self.sample_block_color(frame, best_tag_pts, best_tag_id)
                        if ok:
                            self.curr_color = color
                            self.last_sample_frame = self.frame_idx

                    should_recalibrate_skin = (
                        self.frame_idx - self.skin_calibrated_frame > self.skin_recalibrate_interval
                    )
                    # Recalibrate only when there is no active block lock at all.
                    no_active_block_lock = (
                        self.curr_color in (None, "empty", "black")
                        and self.block_hsv_lower is None
                        and self.block_hsv_upper is None
                        and self.locked_hue is None
                    )
                    out_of_unlock_cooldown = (
                        self.frame_idx - self.last_forced_unlock_frame > self.skin_recalibration_after_unlock_cooldown
                    )
                    can_recalibrate_skin = no_active_block_lock and out_of_unlock_cooldown
                    if should_recalibrate_skin and can_recalibrate_skin:
                        if self.calibrate_skin(frame, best_tag_pts):
                            print("Skin recalibrated based on user tag ROI.")

                    if self.block_hsv_lower is not None:
                        color_before_detection = self.curr_color
                        block_cell = self.detect_held_block_cell(frame, best_tag_pts, best_tag_id)
                        # Print block cell for debugging
                        
                        if block_cell:
                            if block_cell == self.raw_block_candidate:
                                self.raw_block_count += 1
                            else:
                                self.raw_block_candidate = block_cell
                                self.raw_block_count = 1

                            if self.raw_block_count >= self.hover_stable_n:
                                self.raw_block_cell = block_cell
                            display_color = self.curr_color if self.curr_color not in (None, "empty", "black") else "unknown"
                            # print(f"User is holding a {display_color} block at cell ({block_cell[0]}, {block_cell[1]})")
                            # Encode color of block in tag_id for event handling
                            color_tag_id = COLOR_TO_ID.get(display_color)
                            if color_tag_id is not None:
                                event = self.handle_state(color_tag_id, block_cell[0], block_cell[1])
                                if event:
                                    events.append(event)
                        else:
                            color_tag_id = COLOR_TO_ID.get(color_before_detection)
                            if color_tag_id is not None:
                                # Reset per-color placement state once tile leaves the board.
                                self.handle_offboard(color_tag_id)
                                # Keep publishing color targets while tile is held off-board.
                                if self.block_hsv_lower is not None and self.curr_color is not None and self.curr_color not in (None, "empty", "black"):
                                    events.append(TileEvent(tag_id=color_tag_id, row=-1, col=-1, kind="offboard"))
                                else:
                                    # Color lock was reset, emit removed so backend clears stale state.
                                    events.append(TileEvent(tag_id=color_tag_id, row=-1, col=-1, kind="removed"))
                                    self.curr_color = None
                                    self.raw_block_candidate = None
                                    self.raw_block_count = 0
                                    

                elif tag_id in WRIST_TAG_CONFIGS:
                    pass
                elif cell is not None:
                    row, col = cell
                    if tag_id not in CORNER_TAG_IDS:
                        event = self.handle_state(tag_id, row, col)
                        if event is not None:
                            events.append(event)
                else:
                    event = self.handle_offboard(tag_id)
                    if event is not None:
                        events.append(event)
        
        if self.frame_idx % 40 == 0:
            events.extend(self.handle_stale())


        cv2.imshow("Camera", frame)

        if self.Hmat is not None:
            rectified = cv2.warpPerspective(frame, self.Hmat, (W, H))
            # Extract baseline HSV values if not already done
            if self.init == 0:
                # All tiles should be black at start 
                self.baseline_hsv = self.extract_baseline_hsv(rectified)
                print("Baseline HSV values established for all cells.")
                self.init = 1

            # Get hsv values of cells in rectified image for debugging
            if cells_to_check is not None:
                color_list = []
                for cell in cells_to_check:
                    baseline = self.baseline_hsv.get(cell, (0, 0, 0))
                    avg_hsv = self.extract_avg_hsv_of_cell(rectified, cell[0], cell[1])

                    dh = min(abs(avg_hsv[0] - baseline[0]), 180 - abs(avg_hsv[0] - baseline[0]))
                    ds = abs(avg_hsv[1] - baseline[1])
                    dv = abs(avg_hsv[2] - baseline[2])

                    # empty 
                    if ds < 25 and dv < 25:
                        observed_color = "empty"
                    else:
                        # Classify using HSV
                        observed_color = self.classify_color_from_hsv(avg_hsv[0], avg_hsv[1], avg_hsv[2])

                    color_list.append(observed_color)


                    # HSV DEBUGGING     
                    # if observed_color == "yellow" or observed_color == "blue":
                    #     print(f"Cell ({cell[0]}, {cell[1]}), Baseline HSV: {baseline}, Avg HSV: {avg_hsv}, Classified Color: {observed_color}")
                    # Only print debug cell
                    # if (cell[0] == 7 and cell[1] == 3):
                    #     print(f"Cell ({cell[0]}, {cell[1]}), Baseline HSV: {baseline}, Avg HSV: {avg_hsv}, Classified Color: {observed_color}")
                       
              
            # rectified = self.draw_mosaic_overlay(rectified, self.grid)
            self.draw_grid(rectified)
            cv2.imshow("Rectified Board (8x8)", rectified)

        should_quit = (cv2.waitKey(1) & 0xFF) == ord("q")
        # print(color_list) 
        # At the bottom of step(), before return:
        self.last_frame = frame.copy()
        return frame, events, should_quit, color_list