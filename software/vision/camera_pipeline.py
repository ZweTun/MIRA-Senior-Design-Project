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
    "magenta": (255, 0, 255),
    "cyan": (255, 255, 0),
    "yellow": (0, 255, 255),
    "blue": (255, 0, 0),
    "brown": (19, 69, 139),
    "black": (0, 0, 0),
}

REFERENCE_HUES_BOARD = {
    "white": 14,
    "yellow": 24,
    # "orange": 15,
    # "red": 5,
    "green": 38,
    "cyan": 94,
    "blue": 105,
    "magenta": 170,
    "black": 0,
}


# Treat dark pixels as board/empty and low-saturation pixels as non-block (white/gray glove, glare).
BLACK_VALUE_MAX = 50
WHITE_GRAY_SAT_MAX = 10

# CYAN Hard Coded 
CYAN_LOW_SAT_MAX = 45
CYAN_LOW_SAT_MIN = 12
CYAN_LOW_SAT_V_MIN = 150
CYAN_LOW_SAT_H_MIN = 90
CYAN_LOW_SAT_H_MAX = 106

# White glove HSV ranges.
# Primary band: observed glove cluster around H~16, S~65, V~218.
# Bright band: catches desaturated glove highlights that can drift toward yellow hue.
GLOVE_HSV_PRIMARY_LOWER = np.array([8, 35, 160], dtype=np.uint8)
GLOVE_HSV_PRIMARY_UPPER = np.array([28, 180, 255], dtype=np.uint8)
GLOVE_HSV_BRIGHT_LOWER = np.array([5, 10, 190], dtype=np.uint8)
GLOVE_HSV_BRIGHT_UPPER = np.array([35, 120, 255], dtype=np.uint8)




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
        stable_n: int = 1,
        stale_frames: int = 15,
        use_picamera2: bool = False,
        picamera_size: tuple[int, int] = (1280, 720),
        picamera_frame_order: str = "rgb",
    ):
        self.cap = None
        self.picam2 = None
        self.use_picamera2 = False
        self.picamera_frame_order = picamera_frame_order.lower()

        if use_picamera2:
            if Picamera2 is None:
                print("Picamera2 requested but not installed; falling back to OpenCV VideoCapture.")
            else:
                try:
                    self.picam2 = Picamera2()
                    config = self.picam2.create_preview_configuration(
                        main={"format": "BGR888", "size": picamera_size}
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
        self.hover_snap_margin_px = 7
        # Extend board bounds slightly for held-block snapping near edges.
        self.board_edge_buffer_px = CELL_PX
        self.Hmat = None
        self.corner_points_cache = {}
        # Keep last-seen corner points for a short grace window so lock survives brief occlusions.
        self.corner_cache_max_age_frames = 20

        self.baseline_hsv = {}
        self.init = 0

        self.block_hsv_lower = None
        self.block_hsv_upper = None
        self.last_frame = None

        self.sample_interval = 30  # resample every 30 frames
        self.last_sample_frame = -999
        self.roi_h = 19
        self.roi_w = 25

        self.locked_hue = None
        self.locked_color_name = None
        # lower hue lock tolerance means more stable color detection
        # but less adaptability to lighting changes; 
        # higher tolerance allows more drift but can handle more variation
        self.hue_lock_tolerance = 25 
        self.lock_follow_tolerance = 14
        self.hue_drift_count = 0
        # Semi-permanent hue lock: keep first valid block hue and only reset
        # after repeated inability to match that hue.
        self.lock_miss_count = 0
        self.lock_max_miss_frames = 12

        self.user_stable_frames = 5
        self.curr_color = None
        # Require repeated color agreement before publishing/switching held-block color.
        self.color_confirm_candidate = None
        self.color_confirm_count = 0
        self.color_confirm_frames = 3
        self.color_switch_confirm_frames = 2

        self.contour_count = 0

        self.sample_miss_count = 0
        self.max_sample_miss_before_reset = 10
        self.last_forced_unlock_frame = -999
        self.unlock_reacquire_window = 60

        # Stabilize which wrist tag drives held-block detection.
        # Prevent rapid flip-flops when two wrist tags are similarly visible.
        self.active_wrist_tag_id = None
        self.wrist_switch_candidate = None
        self.wrist_switch_count = 0
        self.wrist_switch_stable_n = 10
        # Keep active wrist tag unless a challenger is clearly larger for sustained frames.
        self.wrist_keep_ratio = 1.2
        # Dead zone: if two tags are within this ratio, treat as equal visibility.
        self.wrist_size_dead_zone = 0.08
        # After a switch, hold the new tag for this long to allow ROI and block detection to stabilize.
        self.wrist_switch_cooldown_frames = 30
        self.last_wrist_switch_frame = -999

        # Smooth ROI geometry so small AprilTag corner jitter does not move the sample window every frame.
        self.roi_smooth_alpha = 0.35
        self.smoothed_roi_poly = None
        self.smoothed_roi_tag_id = None

        self.raw_block_cell = None
        self.raw_block_candidate = None
        self.raw_block_count = 0
        # Block detection hysteresis: grace period before reporting block as missing.
        self.block_detect_miss_count = 0
        self.block_detect_miss_grace = 5

    def classify_color_from_hsv(self, h, s, v, reference_hues):
        if v <= BLACK_VALUE_MAX:
            return "empty"

        if s <= WHITE_GRAY_SAT_MAX:
            return "unknown"

        # Cyan/blue split for low-saturation bright samples.
        # Example failure mode: H~103, S~21, V~230 was being read as blue.
        # Keep this override narrow so strong-saturation blue still maps to blue.
        if (
            "cyan" in reference_hues
            and CYAN_LOW_SAT_H_MIN <= h <= CYAN_LOW_SAT_H_MAX
            and CYAN_LOW_SAT_MIN <= s <= CYAN_LOW_SAT_MAX
            and v >= CYAN_LOW_SAT_V_MIN
        ):
            return "cyan"

        best_color = None
        best_dist = float("inf")

        for color, ref_h in reference_hues.items():
            dist = abs(h - ref_h)
            dist = min(dist, 180 - dist)

            if dist < best_dist:
                best_dist = dist
                best_color = color

        # Not confident if hue is too far from any reference
        if best_dist > 20:
            return "unknown"

        # Black/white should never be treated as placeable block colors.
        if best_color in ("black", "white"):
            return "unknown"

        return best_color

    def build_hue_mask(
        self,
        hsv,
        color_name,
        reference_hues,
        hue_tolerance=18,
    ):
        ref_h = reference_hues.get(color_name)
        if ref_h is None:
            return None

        if color_name == "black":
            lower = np.array([0, 0, 0], dtype=np.uint8)
            upper = np.array([180, 255, BLACK_VALUE_MAX], dtype=np.uint8)
            return cv2.inRange(hsv, lower, upper)

        # Yellow is the closest hue to glove tones; require stronger saturation
        # to avoid glove highlights being selected as yellow blobs.
        sat_min = 55 if color_name == "yellow" else 20
        lower = np.array([max(0, ref_h - hue_tolerance), sat_min, 40], dtype=np.uint8)
        upper = np.array([min(180, ref_h + hue_tolerance), 255, 255], dtype=np.uint8)
        return cv2.inRange(hsv, lower, upper)

    def find_largest_color_blob(
        self,
        hsv,
        roi_mask,
        reference_hues,
        allowed_colors=None,
        exclusion_mask=None,
        hue_tolerance=18,
    ):
        roi_area = float(max(np.count_nonzero(roi_mask), 1))
        min_blob_area = max(120.0, roi_area * 0.01)
        colors = allowed_colors if allowed_colors is not None else [
            color for color in reference_hues.keys() if color not in ("black", "white")
        ]

        best = None
        best_area = 0.0
        best_hue_dist = float("inf")

        kernel = np.ones((3, 3), dtype=np.uint8)

        for color_name in colors:
            ref_h = reference_hues.get(color_name)
            color_mask = self.build_hue_mask(
                hsv,
                color_name,
                reference_hues,
                hue_tolerance=hue_tolerance,
            )
            if color_mask is None or ref_h is None:
                continue

            mask = cv2.bitwise_and(color_mask, color_mask, mask=roi_mask)
            if exclusion_mask is not None:
                keep_mask = cv2.bitwise_not(exclusion_mask)
                mask = cv2.bitwise_and(mask, mask, mask=keep_mask)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)
            if area < min_blob_area:
                continue

            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue

            cx = moments["m10"] / moments["m00"]
            cy = moments["m01"] / moments["m00"]
            pixels = hsv[mask > 0]
            if len(pixels) == 0:
                continue

            median_hsv = np.median(pixels.reshape(-1, 3), axis=0)
            median_h = float(median_hsv[0])
            hue_dist = min(abs(median_h - ref_h), 180 - abs(median_h - ref_h))

            # Prefer larger blobs, but when areas are similar, prefer closer hue match.
            should_select = False
            if best is None:
                should_select = True
            elif area > best_area * 1.05:
                should_select = True
            elif area >= best_area * 0.90 and hue_dist < best_hue_dist:
                should_select = True

            if not should_select:
                continue

            best = {
                "color": color_name,
                "area": area,
                "center": (float(cx), float(cy)),
                "median_hsv": median_hsv,
                "mask": mask,
            }
            best_area = area
            best_hue_dist = hue_dist

        return best

    def build_right_aligned_roi(self, tag_pts, tag_id=USER_TAG, roi_forward=None, roi_height=None, gap_px=10):
        tl, tr, br, bl = tag_pts.astype(np.float32)
        center = np.mean(tag_pts, axis=0)

        right_vec = tr - tl
        right_len = np.linalg.norm(right_vec)
        right_unit = right_vec / right_len if right_len > 1e-6 else np.array([1.0, 0.0])

        perp_unit = np.array([-right_unit[1], right_unit[0]], dtype=np.float32)

        marker_width = 0.5 * (np.linalg.norm(tr - tl) + np.linalg.norm(br - bl))
        marker_height = 0.5 * (np.linalg.norm(tr - br) + np.linalg.norm(tl - bl))
        marker_half_width = marker_width * 0.5

        if tag_id == USER_TAG:
            # Flip the height logic only for tag 4:
            # - more visible / larger tag => smaller ROI height
            # - more angled / foreshortened tag => larger ROI height
            # Keep the ROI from collapsing or exploding with simple clamps.
            visibility_scale = 55.0 / max(marker_width, 1.0)
            aspect_ratio = min(marker_width, marker_height) / max(marker_width, marker_height, 1.0)
            angle_scale = 1.0 + (1.0 - aspect_ratio) * 0.85
            height_scale = float(np.clip(visibility_scale * angle_scale, 0.75, 3.0)) / 1.2
        else:
            # Preserve the original behavior for all other tags.

            height_scale = marker_width / 10.0

        forward = float(self.roi_w if roi_forward is None else roi_forward) * height_scale
        height = float(self.roi_h if roi_height is None else roi_height) * height_scale
        offset = perp_unit * (height * 0.2 + gap_px)  # gap between tag and ROI

        roi_center = center + right_unit * (marker_half_width + gap_px + forward * 0.5) + offset
  

        half_forward = right_unit * (forward * 0.5)
        half_height = perp_unit * (height * 0.5)
        # Angle ROI perpendicularly 
        # by 30 degrees for better block detection when user is placing from the side
        angle_rad = np.radians(30)
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

        roi_poly = np.array([p1, p2, p3, p4], dtype=np.float32)

        if self.smoothed_roi_tag_id != tag_id:
            self.smoothed_roi_tag_id = tag_id
            self.smoothed_roi_poly = roi_poly
            return roi_poly

        if self.smoothed_roi_poly is None:
            self.smoothed_roi_poly = roi_poly
            return roi_poly

        self.smoothed_roi_poly = (
            self.roi_smooth_alpha * roi_poly
            + (1.0 - self.roi_smooth_alpha) * self.smoothed_roi_poly
        ).astype(np.float32)
        return self.smoothed_roi_poly

    def build_glove_mask(self, hsv):
        primary = cv2.inRange(hsv, GLOVE_HSV_PRIMARY_LOWER, GLOVE_HSV_PRIMARY_UPPER)
        bright = cv2.inRange(hsv, GLOVE_HSV_BRIGHT_LOWER, GLOVE_HSV_BRIGHT_UPPER)
        return cv2.bitwise_or(primary, bright)

    def build_exclusion_mask(self, hsv):
        return self.build_glove_mask(hsv)

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
        # Remove glove pixels to avoid interference, then apply block color mask.
        exclusion_mask = self.build_exclusion_mask(hsv)
        non_excluded = cv2.bitwise_not(exclusion_mask)
        mask = cv2.bitwise_and(mask, mask, mask=non_excluded)

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
            if self.contour_count > 24:
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
        roi_poly = self.build_right_aligned_roi(tag_pts, tag_id=best_user_tag)
        if roi_poly is None:
            print("Cannot build ROI for block sampling, likely due to tag orientation. Resetting block color lock.")
            return False, None
        poly_i = np.round(roi_poly).astype(np.int32)

        roi_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.fillConvexPoly(roi_mask, poly_i, 255)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        exclusion_mask = self.build_exclusion_mask(hsv)

        # Always search all candidate colors; lock influences tolerance, not candidates.
        candidate_colors = [
            color for color in REFERENCE_HUES_BOARD.keys()
            if color not in ("black", "white")
        ]

        # Use normal hue tolerance for discovery, even when locked.
        # The lock will be validated separately by hue_dist check below.
        best_blob = self.find_largest_color_blob(
            hsv,
            roi_mask,
            REFERENCE_HUES_BOARD,
            allowed_colors=candidate_colors,
            exclusion_mask=exclusion_mask,
            hue_tolerance=18,
        )

        if best_blob is None:
            if self.locked_hue is not None:
                self.lock_miss_count += 1
                if self.lock_miss_count > self.lock_max_miss_frames:
                    print("Locked hue lost for sustained frames; resetting block color lock.")
                    self.reset_block_color()
            return False, None

        sampled_color = best_blob["color"]
        median_hsv = best_blob["median_hsv"]
        h, s, v = float(median_hsv[0]), float(median_hsv[1]), float(median_hsv[2])

        # Extra guard: reject warm, bright, low-sat glove-like samples
        # even if a color blob still slips through exclusion masking.
        if 5 <= h <= 35 and 10 <= s <= 120 and v >= 185:
            self.color_confirm_candidate = None
            self.color_confirm_count = 0
            return False, None

        # Blob selection can still prefer blue when cyan appears desaturated/bright.
        # Mirror the board-cell cyan override for held-block sampling.
        if (
            sampled_color == "blue"
            and CYAN_LOW_SAT_H_MIN <= h <= CYAN_LOW_SAT_H_MAX
            and CYAN_LOW_SAT_MIN <= s <= CYAN_LOW_SAT_MAX
            and v >= CYAN_LOW_SAT_V_MIN
        ):
            sampled_color = "cyan"

        # Ignore non-block samples so the black board cannot become the locked color.
        if sampled_color in ("black", "empty", "unknown"):
            self.color_confirm_candidate = None
            self.color_confirm_count = 0
            return False, None

        # Temporal confirmation: do not publish a color immediately.
        # This filters one-frame spikes from glare/glove contamination.
        if self.locked_color_name is None:
            if sampled_color != self.color_confirm_candidate:
                self.color_confirm_candidate = sampled_color
                self.color_confirm_count = 1
                return False, None
            self.color_confirm_count += 1
            if self.color_confirm_count < self.color_confirm_frames:
                return False, None
        elif sampled_color != self.locked_color_name:
            if sampled_color != self.color_confirm_candidate:
                self.color_confirm_candidate = sampled_color
                self.color_confirm_count = 1
                return False, None
            self.color_confirm_count += 1
            if self.color_confirm_count < self.color_switch_confirm_frames:
                return False, None
        else:
            self.color_confirm_candidate = sampled_color
            self.color_confirm_count = 0

        if self.locked_hue is not None:
            hue_dist = min(abs(h - self.locked_hue), 180 - abs(h - self.locked_hue))
            if hue_dist > self.lock_follow_tolerance:
                self.lock_miss_count += 1
                if self.lock_miss_count > self.lock_max_miss_frames:
                    print("Hue drift persisted; resetting block color lock.")
                    self.reset_block_color()
                return False, None

            if self.locked_color_name is not None and sampled_color != self.locked_color_name:
                # If detected color is clearly different, reset immediately.
                # Yellow (24) vs Green (38) is 14° apart, so any hue shift >= 12° indicates color change.
                if hue_dist >= 12:
                    print(f"Lock color changed from {self.locked_color_name} to {sampled_color} (hue shift {hue_dist:.1f}); resetting immediately.")
                    self.reset_block_color()
                    return True, sampled_color
                
                self.lock_miss_count += 1
                if self.lock_miss_count > self.lock_max_miss_frames:
                    print("Locked color identity lost; resetting block color lock.")
                    self.reset_block_color()
                return False, None

        self.hue_drift_count = 0
        self.lock_miss_count = 0
        self.sample_miss_count = 0
        if self.locked_hue is None:
            self.locked_hue = h
        if self.locked_color_name is None:
            self.locked_color_name = sampled_color

        lock_h = self.locked_hue if self.locked_hue is not None else h
        self.block_hsv_lower = np.array([max(0, lock_h - 20), max(0, s - 60), max(0, v - 60)], dtype=np.uint8)
        self.block_hsv_upper = np.array([min(180, lock_h + 20), 255, 255], dtype=np.uint8)
        self.color_confirm_candidate = sampled_color
        self.color_confirm_count = 0
        self.last_frame = frame.copy()
        # print(f"Sampled block color: {sampled_color} (hue={h:.1f}), locked hue: {self.locked_hue:.1f}, locked color: {self.locked_color_name}")
        return True, sampled_color 
    
    def reset_block_color(self):
    
        self.block_hsv_lower = None
        self.block_hsv_upper = None
        self.locked_hue = None
        self.locked_color_name = None
        self.curr_color = None
        self.color_confirm_candidate = None
        self.color_confirm_count = 0
        self.last_sample_frame = -999
        self.lock_miss_count = 0
        self.raw_block_candidate = None
        self.raw_block_count = 0
        self.raw_block_cell = None
        self.block_detect_miss_count = 0
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
            if self.picamera_frame_order == "rgb":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif self.picamera_frame_order == "bgr":
                pass
            else:
                print(f"Unknown picamera_frame_order={self.picamera_frame_order!r}; assuming BGR.")
            return True, frame

        if self.cap is None:
            return False, None
        return self.cap.read()

    def inner_corner_by_center(self, tag_pts, img_center):
        d2 = np.sum((tag_pts - img_center) ** 2, axis=1)
        return tag_pts[np.argmin(d2)]

    def compute_homography_inner(self, corners, ids, img_center):
        required_labels = ["TL", "TR", "BR", "BL"]

        # Update per-corner cache from currently visible corner tags.
        if ids is not None:
            for i in range(len(ids)):
                tid = int(ids[i][0])
                if tid in CORNER_TAG_IDS:
                    label = CORNER_TAG_IDS[tid]
                    tag_pts = corners[i][0].astype(np.float32)
                    self.corner_points_cache[label] = {
                        "pt": self.inner_corner_by_center(tag_pts, img_center),
                        "frame": self.frame_idx,
                    }

        # Build found corners from cache if still fresh.
        found = {}
        for label in required_labels:
            cached = self.corner_points_cache.get(label)
            if cached is None:
                continue
            age = self.frame_idx - int(cached["frame"])
            if age <= self.corner_cache_max_age_frames:
                found[label] = cached["pt"]

        if any(k not in found for k in required_labels):
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

    def rect_to_cell(
        self,
        xr: float,
        yr: float,
        edge_buffer_px: float = 0.0,
    ) -> Optional[tuple[int, int]]:
        b = float(max(0.0, edge_buffer_px))
        if b > 0.0:
            if not (-b <= xr < W + b and -b <= yr < H + b):
                return None
            # Clamp points within buffer back to board bounds so edge cells still report as held.
            xr = min(max(xr, 0.0), float(W) - 1e-6)
            yr = min(max(yr, 0.0), float(H) - 1e-6)

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
        snapped = self.rect_to_cell(xr, yr, edge_buffer_px=self.board_edge_buffer_px)
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

    def update_user_tag(self, ids, corners):
        visible_wrist_tags = {}

        for i in range(len(ids)):
            pts = corners[i][0]
            tag_id = int(ids[i][0])

            if tag_id in WRIST_TAG_CONFIGS:
                marker_width = 0.5 * (
                    np.linalg.norm(pts[1] - pts[0]) +
                    np.linalg.norm(pts[2] - pts[3])
                )
                visible_wrist_tags[tag_id] = (float(marker_width), pts, i)

        if not visible_wrist_tags:
            self.wrist_switch_candidate = None
            self.wrist_switch_count = 0
            return None

        best_id, (best_size, best_pts, best_idx) = max(
            visible_wrist_tags.items(), key=lambda item: item[1][0]
        )

        # If no active tag yet (or active tag not visible), adopt the current best immediately.
        if self.active_wrist_tag_id not in visible_wrist_tags:
            self.active_wrist_tag_id = best_id
            self.wrist_switch_candidate = None
            self.wrist_switch_count = 0
            self.last_wrist_switch_frame = self.frame_idx
            return self.active_wrist_tag_id, best_pts, best_idx

        active_size, active_pts, active_idx = visible_wrist_tags[self.active_wrist_tag_id]

        # Dead zone: if tags are within 8% of size, treat as equal and keep active tag.
        size_ratio = best_size / active_size if active_size > 0 else 0.0
        if 1.0 - self.wrist_size_dead_zone <= size_ratio <= 1.0 + self.wrist_size_dead_zone:
            # Tags are equally visible; stick with current tag.
            self.wrist_switch_candidate = None
            self.wrist_switch_count = 0
            return self.active_wrist_tag_id, active_pts, active_idx

        # Keep current active wrist tag unless another tag is meaningfully clearer.
        if best_id != self.active_wrist_tag_id and active_size >= best_size * self.wrist_keep_ratio:
            self.wrist_switch_candidate = None
            self.wrist_switch_count = 0
            return self.active_wrist_tag_id, active_pts, active_idx

        if best_id == self.active_wrist_tag_id:
            self.wrist_switch_candidate = None
            self.wrist_switch_count = 0
            return self.active_wrist_tag_id, active_pts, active_idx

        # After switching, hold the active tag briefly to avoid rapid oscillation.
        if self.frame_idx - self.last_wrist_switch_frame < self.wrist_switch_cooldown_frames:
            self.wrist_switch_candidate = None
            self.wrist_switch_count = 0
            return self.active_wrist_tag_id, active_pts, active_idx

        # A different tag is consistently better: require a few stable frames before switching.
        if self.wrist_switch_candidate == best_id:
            self.wrist_switch_count += 1
        else:
            self.wrist_switch_candidate = best_id
            self.wrist_switch_count = 1

        if self.wrist_switch_count >= self.wrist_switch_stable_n:
            self.active_wrist_tag_id = best_id
            self.wrist_switch_candidate = None
            self.wrist_switch_count = 0
            self.last_wrist_switch_frame = self.frame_idx
            return self.active_wrist_tag_id, best_pts, best_idx

        return self.active_wrist_tag_id, active_pts, active_idx

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
                    sample_interval_frames = 2 if (active_lock or recently_forced_unlocked) else self.sample_interval
                    if self.frame_idx - self.last_sample_frame >= sample_interval_frames:
                        ok, color = self.sample_block_color(frame, best_tag_pts, best_tag_id)
                        if ok:
                            self.curr_color = color
                            self.last_sample_frame = self.frame_idx

                    if self.block_hsv_lower is not None:
                        color_before_detection = self.curr_color
                        block_cell = self.detect_held_block_cell(frame, best_tag_pts, best_tag_id)
                        # Print block cell for debugging
                        
                        if block_cell:
                            # Block found: reset hysteresis counter and process as before.
                            self.block_detect_miss_count = 0
                            if block_cell == self.raw_block_candidate:
                                self.raw_block_count += 1
                            else:
                                self.raw_block_candidate = block_cell
                                self.raw_block_count = 1

                            if self.raw_block_count >= self.hover_stable_n:
                                self.raw_block_cell = block_cell
                            display_color = str(self.curr_color).strip().lower() if self.curr_color not in (None, "empty", "black") else "unknown"
                            # Encode color of block in tag_id for event handling
                            color_tag_id = COLOR_TO_ID.get(display_color)
                            if color_tag_id is not None:
                                event = self.handle_state(color_tag_id, block_cell[0], block_cell[1])
                                if event:
                                    events.append(event)
                        else:
                            # Block not found: increment miss counter.
                            self.block_detect_miss_count += 1
                            color_name = str(color_before_detection).strip().lower() if color_before_detection not in (None, "empty", "black") else None
                            color_tag_id = COLOR_TO_ID.get(color_name) if color_name is not None else None
                            if color_tag_id is not None:
                                # Only emit events if miss count exceeds grace period, allowing tag switch ROI to stabilize.
                                if self.block_detect_miss_count > self.block_detect_miss_grace:
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
                        observed_color = self.classify_color_from_hsv(
                            avg_hsv[0],
                            avg_hsv[1],
                            avg_hsv[2],
                            REFERENCE_HUES_BOARD,
                        )

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