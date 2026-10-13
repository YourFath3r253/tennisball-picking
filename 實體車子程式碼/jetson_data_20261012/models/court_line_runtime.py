"""Realtime four-yarn court-boundary detector for Jetson Nano.

This module is deliberately independent from the offline analysis script.  It
uses the same frozen color prototypes and the same geometry-first tracking,
but is compatible with the Python versions commonly shipped with JetPack 4.
"""

import json
import math

import cv2
import numpy as np


VALID_COLORS = set(("brown", "pink", "purple", "yellow"))
PINK_PURPLE_AMBIGUITY_MIN_CONFIDENCE = 0.32


class CourtObservation(object):
    """One camera observation formatted for the STM32 UART protocol."""

    def __init__(
        self,
        state,
        line_side,
        color,
        near_x_norm=None,
        near_y_norm=None,
        near_confirm_count=0,
    ):
        self.state = state
        self.line_side = line_side
        self.color = color
        self.near_x_norm = near_x_norm
        self.near_y_norm = near_y_norm
        self.near_confirm_count = near_confirm_count

    def packet(self):
        return "COURT,%s,%s,%s" % (
            self.state,
            self.line_side,
            self.color,
        )


def nearest_line_point(line):
    """Return the endpoint nearest the vehicle in image coordinates."""
    x1, y1, x2, y2 = line
    if y1 > y2:
        return float(x1), float(y1)
    if y2 > y1:
        return float(x2), float(y2)
    return (float(x1) + float(x2)) / 2.0, (float(y1) + float(y2)) / 2.0


class CourtProtocolEstimator(object):
    """Translate tracked lines into SAFE/EDGE camera observations.

    ``line_side`` is deliberately a camera observation, not a steering
    direction.  STM32 combines it with its own patrol state and sensors.
    """

    def __init__(
        self,
        edge_y=0.48,
        edge_confirm_frames=3,
        center_left=0.40,
        center_right=0.60,
        color_confidence_min=0.35,
    ):
        if not 0.0 < edge_y < 1.0:
            raise ValueError("edge_y must be within (0, 1)")
        if edge_confirm_frames < 1:
            raise ValueError("edge_confirm_frames must be positive")
        self.edge_y = edge_y
        self.edge_confirm_frames = edge_confirm_frames
        self.center_left = center_left
        self.center_right = center_right
        self.color_confidence_min = color_confidence_min
        self._near_count = 0
        self._edge_active = False
        self._edge_side = "NONE"
        self._edge_color = "UNKNOWN"

    def _confirmed_color(self, label, confidence, color_status):
        if (
            color_status == "confirmed"
            and label in VALID_COLORS
            and confidence >= self.color_confidence_min
        ):
            return label.upper()
        return "UNKNOWN"

    def _side_for_x(self, x_norm):
        if x_norm < self.center_left:
            return "LEFT"
        if x_norm > self.center_right:
            return "RIGHT"
        return "CENTER"

    def update(
        self,
        candidate_line,
        frame_width,
        frame_height,
        position_state,
        stable_color,
        stable_confidence,
        color_status,
    ):
        if frame_width <= 0 or frame_height <= 0:
            raise ValueError("frame dimensions must be positive")

        if position_state == "observed" and candidate_line is not None:
            near_x, near_y = nearest_line_point(candidate_line)
            x_norm = near_x / float(frame_width)
            y_norm = near_y / float(frame_height)
            current_color = self._confirmed_color(
                stable_color,
                stable_confidence,
                color_status,
            )

            # A bright floor seam may be geometrically stable but has no
            # usable four-colour evidence.  It must never override driving.
            if y_norm >= self.edge_y and current_color != "UNKNOWN":
                self._near_count += 1
                if self._near_count >= self.edge_confirm_frames:
                    self._edge_active = True
                    self._edge_side = self._side_for_x(x_norm)
                    if current_color != "UNKNOWN":
                        self._edge_color = current_color
            else:
                self._near_count = 0
                self._edge_active = False
                self._edge_side = "NONE"
                self._edge_color = "UNKNOWN"

            if self._edge_active:
                if self._edge_color == "UNKNOWN" and current_color != "UNKNOWN":
                    self._edge_color = current_color
                return CourtObservation(
                    "EDGE",
                    self._edge_side,
                    self._edge_color,
                    x_norm,
                    y_norm,
                    self._near_count,
                )

            # Distant color is not sent; the current model is intentionally
            # conservative until the line reaches the EDGE zone.
            return CourtObservation(
                "SAFE",
                "NONE",
                "UNKNOWN",
                x_norm,
                y_norm,
                self._near_count,
            )

        # A held/missing frame cannot complete a three-fresh-observation
        # decision.  STM32 owns its own avoiding latch after an EDGE packet.
        self._near_count = 0
        if position_state == "held" and self._edge_active:
            return CourtObservation(
                "EDGE",
                self._edge_side,
                self._edge_color,
            )

        self._edge_active = False
        self._edge_side = "NONE"
        self._edge_color = "UNKNOWN"
        return CourtObservation("SAFE", "NONE", "UNKNOWN")


class CourtLineResult(object):
    """Diagnostic result returned by :meth:`CourtLineRuntime.observe`."""

    def __init__(
        self,
        observation,
        line,
        position_state,
        color_label,
        color_confidence,
        color_status,
        raw_label,
        raw_confidence,
        color_model_loaded,
    ):
        self.observation = observation
        self.line = line
        self.position_state = position_state
        self.color_label = color_label
        self.color_confidence = color_confidence
        self.color_status = color_status
        self.raw_label = raw_label
        self.raw_confidence = raw_confidence
        self.color_model_loaded = color_model_loaded

    def packet(self):
        return self.observation.packet()


def line_angle_deg(x1, y1, x2, y2):
    return math.degrees(math.atan2(y2 - y1, x2 - x1))


def line_length(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def make_band_mask(shape, p1, p2, thickness):
    mask = np.zeros(shape[:2], dtype=np.uint8)
    cv2.line(mask, p1, p2, 255, thickness)
    return mask


def shift_line(x1, y1, x2, y2, offset):
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)
    if length < 1:
        return x1, y1, x2, y2
    nx = -dy / length
    ny = dx / length
    return (
        int(round(x1 + nx * offset)),
        int(round(y1 + ny * offset)),
        int(round(x2 + nx * offset)),
        int(round(y2 + ny * offset)),
    )


def line_geometry(line):
    x1, y1, x2, y2 = line
    if x1 > x2:
        x1, y1, x2, y2 = x2, y2, x1, y1
    return x1, y1, x2, y2


def association_cost(candidate, previous, max_y=45.0, max_angle=10.0, min_overlap=0.30):
    a = line_geometry(candidate["line"])
    b = line_geometry(previous["line"])
    overlap_left = max(a[0], b[0])
    overlap_right = min(a[2], b[2])
    shorter = min(a[2] - a[0], b[2] - b[0])
    if shorter <= 0 or overlap_right <= overlap_left:
        return None
    overlap = (overlap_right - overlap_left) / float(shorter)
    if overlap < min_overlap:
        return None
    x = (overlap_left + overlap_right) / 2.0

    def y_at(segment):
        return segment[1] + (x - segment[0]) * (segment[3] - segment[1]) / (
            segment[2] - segment[0]
        )

    dy = abs(y_at(a) - y_at(b))
    da = abs((candidate["angle"] - previous["angle"] + 90.0) % 180.0 - 90.0)
    if dy > max_y or da > max_angle:
        return None
    return dy / max_y + da / max_angle + 0.25 * (1.0 - overlap)


def select_matching_candidate(candidates, previous, max_y=45.0, max_angle=10.0):
    ranked = []
    for candidate in candidates:
        cost = association_cost(candidate, previous, max_y, max_angle)
        if cost is not None:
            ranked.append((cost, candidate))
    if ranked:
        return min(ranked, key=lambda item: item[0])[1]

    relaxed = []
    for candidate in candidates:
        cost = association_cost(
            candidate,
            previous,
            max_y * 2.0,
            max_angle * 1.5,
            min_overlap=0.10,
        )
        if cost is not None:
            relaxed.append((cost, candidate))
    if relaxed:
        return min(relaxed, key=lambda item: item[0])[1]
    return None


def initial_color_state():
    return {
        "label": "unknown",
        "confidence": 0.0,
        "pending_label": None,
        "pending_count": 0,
    }


def update_color_state(state, observed_label, observed_confidence,
                       confirm_frames=3, switch_frames=4, ema_alpha=0.30):
    state = dict(state)
    confirmed = state["label"]
    if observed_label == "unknown":
        if confirmed != "unknown":
            return state, "holding"
        return state, "unknown"

    if observed_label == confirmed:
        state["pending_label"] = None
        state["pending_count"] = 0
        state["confidence"] = (
            (1.0 - ema_alpha) * state["confidence"]
            + ema_alpha * observed_confidence
        )
        return state, "confirmed"

    if observed_label == state["pending_label"]:
        state["pending_count"] += 1
    else:
        state["pending_label"] = observed_label
        state["pending_count"] = 1

    required = confirm_frames if confirmed == "unknown" else switch_frames
    if state["pending_count"] >= required:
        state["label"] = observed_label
        state["confidence"] = observed_confidence
        state["pending_label"] = None
        state["pending_count"] = 0
        if confirmed == "unknown":
            return state, "confirmed"
        return state, "switched"

    if confirmed == "unknown":
        return state, "confirming"
    return state, "switching"


def reject_ambiguous_pink_purple(label, ranking, confidence):
    if len(ranking) < 2:
        return label
    top_pair = set((ranking[0][0], ranking[1][0]))
    if (
        top_pair == set(("pink", "purple"))
        and confidence < PINK_PURPLE_AMBIGUITY_MIN_CONFIDENCE
    ):
        return "unknown"
    return label


class CourtLineRuntime(object):
    """Find, track and classify a colored yarn boundary in a camera frame."""

    # The ball model already consumes most of the Nano's GPU/CPU budget.
    # Boundary avoidance only needs relative line position, so analyse a
    # smaller copy and scale the displayed line back to the camera frame.
    PROCESS_W = 320
    PROCESS_H = 240
    ROI_Y0 = 0.25
    ROI_Y1 = 0.98
    MAX_ANGLE_DEG = 15.0
    MIN_LENGTH_RATIO = 0.20
    LINE_HALF_WIDTH = 5
    BG_OFFSET_NEAR = 10
    BG_OFFSET_FAR = 24
    MERGE_Y_THRESHOLD = 25
    MERGE_ANGLE_THRESHOLD = 6.0
    LINE_ON_CONFIRM_FRAMES = 2
    LINE_OFF_CONFIRM_FRAMES = 2
    COLOR_ON_CONFIRM_FRAMES = 3
    COLOR_SWITCH_CONFIRM_FRAMES = 4
    CLASSIFY_CONFIDENCE_MIN = 0.25
    COLOR_EVIDENCE_MIN_CONFIDENCE = 0.30
    EMA_ALPHA = 0.30
    TRACK_Y_THRESHOLD = 45
    TRACK_ANGLE_THRESHOLD = 10.0
    # Hough can return hundreds of floor-texture segments.  Measuring colour
    # around every one of them is too slow for Jetson Nano, so only the
    # longest plausible horizontal segments reach the expensive colour stage.
    MAX_RAW_CANDIDATES = 16

    def __init__(
        self,
        color_model_path,
        edge_y=0.48,
        edge_confirm_frames=3,
        color_confidence_min=0.35,
    ):
        self.models = self._load_color_models(color_model_path)
        self.color_model_loaded = bool(self.models)
        self.protocol = CourtProtocolEstimator(
            edge_y=edge_y,
            edge_confirm_frames=edge_confirm_frames,
            color_confidence_min=color_confidence_min,
        )
        self.track = None
        self.pending_candidate = None
        self.pending_count = 0
        self.miss_count = 0
        self.color_state = initial_color_state()

    @staticmethod
    def _load_color_models(path):
        try:
            with open(path, "r") as handle:
                saved = json.load(handle)
        except (IOError, OSError, ValueError):
            return {}

        models = {}
        for name, item in saved.items():
            if name not in VALID_COLORS:
                continue
            median = np.asarray(item.get("median"), dtype=np.float32)
            scale = np.asarray(item.get("scale"), dtype=np.float32)
            if median.size == 0 or scale.size != median.size:
                continue
            models[name] = {
                "median": median,
                "scale": np.maximum(scale, 1e-6),
            }
        return models

    def _extract_line_feature(self, frame, line, hsv, lab, bgr):
        x1, y1, x2, y2 = line
        h, w = frame.shape[:2]
        line_mask = make_band_mask(
            frame.shape,
            (x1, y1),
            (x2, y2),
            self.LINE_HALF_WIDTH * 2 + 1,
        )

        bg_masks = []
        for offset in (
            -self.BG_OFFSET_FAR,
            -self.BG_OFFSET_NEAR,
            self.BG_OFFSET_NEAR,
            self.BG_OFFSET_FAR,
        ):
            sx1, sy1, sx2, sy2 = shift_line(x1, y1, x2, y2, offset)
            if (
                min(sx1, sx2) < 0
                or max(sx1, sx2) >= w
                or min(sy1, sy2) < 0
                or max(sy1, sy2) >= h
            ):
                continue
            bg_masks.append(
                make_band_mask(frame.shape, (sx1, sy1), (sx2, sy2), 7)
            )

        if not bg_masks:
            return None

        bg_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        for mask in bg_masks:
            bg_mask = cv2.bitwise_or(bg_mask, mask)
        bg_mask[line_mask > 0] = 0
        line_idx = line_mask > 0
        bg_idx = bg_mask > 0
        if np.count_nonzero(line_idx) < 30 or np.count_nonzero(bg_idx) < 50:
            return None

        line_bgr = np.median(bgr[line_idx], axis=0)
        line_hsv = np.median(hsv[line_idx], axis=0)
        line_lab = np.median(lab[line_idx], axis=0)
        bg_bgr = np.median(bgr[bg_idx], axis=0)
        bg_lab = np.median(lab[bg_idx], axis=0)

        blue = float(line_bgr[0])
        green = float(line_bgr[1])
        red = float(line_bgr[2])
        total = red + green + blue + 1.0
        d_l = (float(line_lab[0]) - float(bg_lab[0])) / 255.0
        d_a = (float(line_lab[1]) - float(bg_lab[1])) / 127.0
        d_b = (float(line_lab[2]) - float(bg_lab[2])) / 127.0

        feature = np.array(
            [
                red / total,
                green / total,
                blue / total,
                float(line_hsv[1]) / 255.0,
                float(line_hsv[2]) / 255.0,
                (float(line_lab[1]) - 128.0) / 127.0,
                (float(line_lab[2]) - 128.0) / 127.0,
                d_l,
                d_a,
                d_b,
            ],
            dtype=np.float32,
        )

        return {
            "feature": feature,
            "contrast": math.sqrt(d_l * d_l + d_a * d_a + d_b * d_b),
            "line_hsv": line_hsv,
            "line_lab": line_lab,
            "line_bgr": line_bgr,
            "bg_bgr": bg_bgr,
            "bg_lab": bg_lab,
        }

    def _candidate_from_line(self, frame, line, hsv, lab, bgr):
        x1, y1, x2, y2 = line
        angle = line_angle_deg(x1, y1, x2, y2)
        if abs(angle) > self.MAX_ANGLE_DEG:
            return None
        length = line_length(x1, y1, x2, y2)
        h, w = frame.shape[:2]
        info = self._extract_line_feature(frame, line, hsv, lab, bgr)
        if info is None:
            return None
        length_ratio = length / float(w)
        return {
            "line": (int(x1), int(y1), int(x2), int(y2)),
            "angle": angle,
            "length": length,
            "length_ratio": length_ratio,
            "feature": info["feature"],
            "contrast": info["contrast"],
            "score": 2.0 * length_ratio + 6.0 * info["contrast"],
            "line_hsv": info["line_hsv"],
            "line_lab": info["line_lab"],
        }

    def _merge_candidates(self, frame, candidates, hsv, lab, bgr):
        if len(candidates) <= 1:
            return candidates

        groups = []
        for candidate in candidates:
            x1, y1, x2, y2 = candidate["line"]
            y_mid = (y1 + y2) / 2.0
            angle = candidate["angle"]
            matched = False
            for group in groups:
                if (
                    abs(y_mid - group["y_mean"]) <= self.MERGE_Y_THRESHOLD
                    and abs(angle - group["angle_mean"]) <= self.MERGE_ANGLE_THRESHOLD
                ):
                    group["items"].append(candidate)
                    group["y_mean"] = float(np.mean([
                        (item["line"][1] + item["line"][3]) / 2.0
                        for item in group["items"]
                    ]))
                    group["angle_mean"] = float(np.mean([
                        item["angle"] for item in group["items"]
                    ]))
                    matched = True
                    break
            if not matched:
                groups.append({
                    "items": [candidate],
                    "y_mean": y_mid,
                    "angle_mean": angle,
                })

        merged = []
        for group in groups:
            points = []
            for candidate in group["items"]:
                x1, y1, x2, y2 = candidate["line"]
                points.append((x1, y1))
                points.append((x2, y2))
            points.sort(key=lambda point: point[0])
            line = (points[0][0], points[0][1], points[-1][0], points[-1][1])
            merged_candidate = self._candidate_from_line(frame, line, hsv, lab, bgr)
            if merged_candidate is not None:
                merged.append(merged_candidate)

        merged.sort(key=lambda item: item["score"], reverse=True)
        return merged

    def _find_candidate_lines(self, frame):
        h, w = frame.shape[:2]
        roi_y0 = int(h * self.ROI_Y0)
        roi_y1 = int(h * self.ROI_Y1)
        roi = frame[roi_y0:roi_y1, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, 20, 80)
        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180.0,
            threshold=30,
            minLineLength=int(w * self.MIN_LENGTH_RATIO),
            maxLineGap=45,
        )
        if lines is None:
            return []

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB).astype(np.float32)
        bgr = frame.astype(np.float32)
        # Filter geometry first.  _candidate_from_line() creates several
        # full-frame masks, so it must not be called once for every Hough
        # segment from floor texture.
        raw_candidates = []
        for item in lines:
            x1, y1, x2, y2 = np.asarray(item).reshape(-1)[:4]
            line = (int(x1), int(y1 + roi_y0), int(x2), int(y2 + roi_y0))
            if abs(line_angle_deg(*line)) > self.MAX_ANGLE_DEG:
                continue
            if line_length(*line) < w * self.MIN_LENGTH_RATIO:
                continue
            raw_candidates.append(line)

        raw_candidates.sort(
            key=lambda item: line_length(*item), reverse=True
        )

        candidates = []
        for line in raw_candidates[:self.MAX_RAW_CANDIDATES]:
            candidate = self._candidate_from_line(frame, line, hsv, lab, bgr)
            if candidate is not None:
                candidates.append(candidate)
        candidates.sort(key=lambda item: item["score"], reverse=True)
        return self._merge_candidates(frame, candidates, hsv, lab, bgr)

    def _classify_feature(self, feature):
        if not self.models:
            return "unknown", 999.0, 0.0, []

        ranking = []
        for color_name, model in self.models.items():
            z = (feature - model["median"]) / model["scale"]
            distance = float(np.sqrt(np.mean(z * z)))
            ranking.append((color_name, distance))
        ranking.sort(key=lambda item: item[1])
        best_color = ranking[0][0]
        best_distance = ranking[0][1]
        second_distance = ranking[1][1] if len(ranking) >= 2 else 999.0
        confidence = (second_distance - best_distance) / (second_distance + 1e-6)
        if best_distance > 3.0 or confidence < self.CLASSIFY_CONFIDENCE_MIN:
            label = "unknown"
        else:
            label = best_color
        label = reject_ambiguous_pink_purple(label, ranking, confidence)
        return label, best_distance, confidence, ranking

    def _update_geometry_track(self, candidate):
        if self.track is None:
            if candidate is None:
                self.pending_candidate = None
                self.pending_count = 0
                return

            if (
                self.pending_candidate is not None
                and association_cost(
                    candidate,
                    self.pending_candidate,
                    self.TRACK_Y_THRESHOLD,
                    self.TRACK_ANGLE_THRESHOLD,
                ) is not None
            ):
                self.pending_count += 1
            else:
                self.pending_count = 1
            self.pending_candidate = candidate
            if self.pending_count >= self.LINE_ON_CONFIRM_FRAMES:
                x1, y1, x2, y2 = candidate["line"]
                self.track = {
                    "last_observation": candidate,
                    "line": candidate["line"],
                    "y": (y1 + y2) / 2.0,
                    "angle": candidate["angle"],
                }
                self.miss_count = 0
                self.pending_candidate = None
                self.pending_count = 0
            return

        if candidate is not None:
            self.track["last_observation"] = candidate
            old_line = self.track["line"]
            new_line = candidate["line"]
            alpha = self.EMA_ALPHA
            smoothed = tuple(
                int(round((1.0 - alpha) * old + alpha * new))
                for old, new in zip(old_line, new_line)
            )
            self.track["line"] = smoothed
            self.track["y"] = (smoothed[1] + smoothed[3]) / 2.0
            self.track["angle"] = (
                (1.0 - alpha) * self.track["angle"]
                + alpha * candidate["angle"]
            )
            self.miss_count = 0
        else:
            self.miss_count += 1

        if self.track is not None and self.miss_count >= self.LINE_OFF_CONFIRM_FRAMES:
            self.track = None
            self.color_state = initial_color_state()
            self.miss_count = 0

    def observe(self, frame):
        """Process one fresh 640x480 camera frame and return a court result."""
        frame_height, frame_width = frame.shape[:2]
        work_frame = cv2.resize(
            frame, (self.PROCESS_W, self.PROCESS_H), interpolation=cv2.INTER_AREA
        )
        work_height, work_width = work_frame.shape[:2]
        candidates = self._find_candidate_lines(work_frame)
        candidate = None
        raw_label = "unknown"
        raw_confidence = 0.0

        if self.track is not None:
            candidate = select_matching_candidate(
                candidates,
                self.track.get("last_observation", self.track),
                self.TRACK_Y_THRESHOLD,
                self.TRACK_ANGLE_THRESHOLD,
            )
        elif candidates:
            candidate = candidates[0]

        if candidate is not None:
            # Geometry is found at 320x240 for speed, but the colour models
            # were measured from the original camera view.  Re-sample only
            # the one selected line at 640x480; doing this for every Hough
            # candidate was the main source of the earlier FPS collapse.
            scale_x = frame_width / float(work_width)
            scale_y = frame_height / float(work_height)
            full_line = tuple(
                int(round(value * (scale_x if index % 2 == 0 else scale_y)))
                for index, value in enumerate(candidate["line"])
            )
            full_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            full_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB).astype(np.float32)
            full_bgr = frame.astype(np.float32)
            full_info = self._extract_line_feature(
                frame, full_line, full_hsv, full_lab, full_bgr
            )
            if full_info is not None:
                raw_label, _distance, raw_confidence, _ranking = (
                    self._classify_feature(full_info["feature"])
                )

        self._update_geometry_track(candidate)

        color_status = "no_line"
        color_evidence_label = "unknown"
        if self.track is not None:
            if candidate is not None:
                if raw_confidence >= self.COLOR_EVIDENCE_MIN_CONFIDENCE:
                    color_evidence_label = raw_label
                self.color_state, color_status = update_color_state(
                    self.color_state,
                    color_evidence_label,
                    raw_confidence,
                    self.COLOR_ON_CONFIRM_FRAMES,
                    self.COLOR_SWITCH_CONFIRM_FRAMES,
                    self.EMA_ALPHA,
                )
            else:
                color_status = "position_held"

        if self.track is not None:
            position_state = "observed" if candidate is not None else "held"
            line = self.track["line"]
            color_label = self.color_state["label"]
            color_confidence = self.color_state["confidence"]
        else:
            position_state = "unavailable"
            line = None
            color_label = "unknown"
            color_confidence = 0.0

        observation = self.protocol.update(
            candidate["line"] if position_state == "observed" else None,
            work_width,
            work_height,
            position_state,
            color_label,
            color_confidence,
            color_status,
        )

        # Tracking stays in the inexpensive working resolution.  Only the
        # overlay is converted back to the 640x480 video coordinate system.
        display_line = None
        if line is not None:
            scale_x = frame_width / float(work_width)
            scale_y = frame_height / float(work_height)
            display_line = tuple(
                int(round(value * (scale_x if index % 2 == 0 else scale_y)))
                for index, value in enumerate(line)
            )

        return CourtLineResult(
            observation,
            display_line,
            position_state,
            color_label,
            color_confidence,
            color_status,
            raw_label,
            raw_confidence,
            self.color_model_loaded,
        )
