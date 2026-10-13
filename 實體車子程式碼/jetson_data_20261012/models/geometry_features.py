"""Geometry and ten color/background features only. No multiclass model or state.
Source SHA256: 7b412fa9109d5d28411aa2ee5e7c6080c3b25cbee67d8adc98a8a0e2f8b3e65d
"""
import math
import cv2
import numpy as np

def line_angle_deg(x1, y1, x2, y2):
    return (math.degrees(math.atan2(y2 - y1, x2 - x1)) + 90.0) % 180.0 - 90.0

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
        return (x1, y1, x2, y2)
    nx = -dy / length
    ny = dx / length
    return (int(round(x1 + nx * offset)), int(round(y1 + ny * offset)), int(round(x2 + nx * offset)), int(round(y2 + ny * offset)))

def line_geometry(line):
    x1, y1, x2, y2 = line
    if x1 > x2 or (x1 == x2 and y1 > y2):
        x1, y1, x2, y2 = (x2, y2, x1, y1)
    return (x1, y1, x2, y2)

def association_cost(candidate, previous, max_y=45.0, max_angle=10.0, min_overlap=0.3):
    a = np.asarray(line_geometry(candidate['line']),dtype=np.float64).reshape(2,2)
    b = np.asarray(line_geometry(previous['line']),dtype=np.float64).reshape(2,2)
    direction=b[1]-b[0]
    length=float(np.linalg.norm(direction))
    if length<1:return None
    direction/=length
    normal=np.array([-direction[1],direction[0]])
    ap=sorted(np.dot(a,direction));bp=sorted(np.dot(b,direction))
    shorter=min(ap[1]-ap[0],bp[1]-bp[0])
    if shorter<=0:return None
    overlap=max(0.,min(ap[1],bp[1])-max(ap[0],bp[0]))/shorter
    if overlap<min_overlap:return None
    dy=abs(float(np.dot(a.mean(0)-b.mean(0),normal)))
    da = abs((candidate['angle'] - previous['angle'] + 90.0) % 180.0 - 90.0)
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
        cost = association_cost(candidate, previous, max_y * 2.0, max_angle * 1.5, min_overlap=0.1)
        if cost is not None:
            relaxed.append((cost, candidate))
    if relaxed:
        return min(relaxed, key=lambda item: item[0])[1]
    return None

class GeometryFeatures:
    PROCESS_W = 320
    PROCESS_H = 240
    ROI_Y0 = 0.25
    ROI_Y1 = 0.98
    MAX_ANGLE_DEG = 90.0
    MIN_LENGTH_RATIO = 0.2
    LINE_HALF_WIDTH = 5
    BG_OFFSET_NEAR = 10
    BG_OFFSET_FAR = 24
    MERGE_Y_THRESHOLD = 25
    MERGE_ANGLE_THRESHOLD = 6.0
    MAX_RAW_CANDIDATES = 16

    def _extract_line_feature(self, frame, line, hsv, lab, bgr):
        x1, y1, x2, y2 = line
        # Only the local line/background bands contribute. Cropping avoids
        # repeatedly constructing full-frame masks for every Hough segment.
        margin=self.BG_OFFSET_FAR+self.LINE_HALF_WIDTH+8
        left=max(0,min(x1,x2)-margin);top=max(0,min(y1,y2)-margin)
        right=min(frame.shape[1],max(x1,x2)+margin+1)
        bottom=min(frame.shape[0],max(y1,y2)+margin+1)
        frame=frame[top:bottom,left:right]
        hsv=hsv[top:bottom,left:right];lab=lab[top:bottom,left:right];bgr=bgr[top:bottom,left:right]
        x1-=left;x2-=left;y1-=top;y2-=top
        h, w = frame.shape[:2]
        line_mask = make_band_mask(frame.shape, (x1, y1), (x2, y2), self.LINE_HALF_WIDTH * 2 + 1)
        bg_masks = []
        for offset in (-self.BG_OFFSET_FAR, -self.BG_OFFSET_NEAR, self.BG_OFFSET_NEAR, self.BG_OFFSET_FAR):
            sx1, sy1, sx2, sy2 = shift_line(x1, y1, x2, y2, offset)
            # OpenCV clips the band to the image. Crossing one image edge
            # must not discard an otherwise useful background band.
            bg_masks.append(make_band_mask(frame.shape, (sx1, sy1), (sx2, sy2), 7))
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
        feature = np.array([red / total, green / total, blue / total, float(line_hsv[1]) / 255.0, float(line_hsv[2]) / 255.0, (float(line_lab[1]) - 128.0) / 127.0, (float(line_lab[2]) - 128.0) / 127.0, d_l, d_a, d_b], dtype=np.float32)
        return {'feature': feature, 'contrast': math.sqrt(d_l * d_l + d_a * d_a + d_b * d_b), 'line_hsv': line_hsv, 'line_lab': line_lab, 'line_bgr': line_bgr, 'bg_bgr': bg_bgr, 'bg_lab': bg_lab}

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
        return {'line': (int(x1), int(y1), int(x2), int(y2)), 'angle': angle, 'length': length, 'length_ratio': length_ratio, 'feature': info['feature'], 'contrast': info['contrast'], 'score': 2.0 * length_ratio + 6.0 * info['contrast'], 'line_hsv': info['line_hsv'], 'line_lab': info['line_lab']}

    def _merge_candidates(self, frame, candidates, hsv, lab, bgr):
        if len(candidates) <= 1:
            return candidates
        groups = []
        for candidate in candidates:
            x1, y1, x2, y2 = candidate['line']
            y_mid = (y1 + y2) / 2.0
            angle = candidate['angle']
            matched = False
            for group in groups:
                reference=group['items'][0]
                da=abs((angle-reference['angle']+90.)%180.-90.)
                p=np.asarray(reference['line'],dtype=np.float64).reshape(2,2)
                direction=p[1]-p[0];direction/=max(1.,float(np.linalg.norm(direction)))
                normal=np.array([-direction[1],direction[0]])
                q=np.array([[x1,y1],[x2,y2]],dtype=np.float64)
                separation=abs(float(np.dot(q.mean(0)-p.mean(0),normal)))
                pp=sorted(np.dot(p,direction));qp=sorted(np.dot(q,direction))
                gap=max(pp[0]-qp[1],qp[0]-pp[1],0.)
                if separation<=6.0 and gap<=30.0 and da<=self.MERGE_ANGLE_THRESHOLD:
                    group['items'].append(candidate)
                    group['y_mean'] = float(np.mean([(item['line'][1] + item['line'][3]) / 2.0 for item in group['items']]))
                    group['angle_mean'] = float(np.mean([item['angle'] for item in group['items']]))
                    matched = True
                    break
            if not matched:
                groups.append({'items': [candidate], 'y_mean': y_mid, 'angle_mean': angle})
        merged = []
        for group in groups:
            if len(group['items'])==1:
                continue
            points = []
            for candidate in group['items']:
                x1, y1, x2, y2 = candidate['line']
                points.append((x1, y1))
                points.append((x2, y2))
            ref=group['items'][0]['line']
            direction=np.array([ref[2]-ref[0],ref[3]-ref[1]],dtype=np.float64)
            points.sort(key=lambda point: float(np.dot(point,direction)))
            line = (points[0][0], points[0][1], points[-1][0], points[-1][1])
            merged_candidate = self._candidate_from_line(frame, line, hsv, lab, bgr)
            if merged_candidate is not None:
                merged.append(merged_candidate)
        # Keep measured subsegments too: a curved yarn can be diluted by a
        # merged chord, while individual pieces still carry yellow evidence.
        unique={tuple(c['line']):c for c in candidates+merged}
        return sorted(unique.values(),key=lambda item:item['score'],reverse=True)

    def _find_candidate_lines(self, frame, exclusion_mask=None):
        h, w = frame.shape[:2]
        roi_y0 = int(h * self.ROI_Y0)
        roi_y1 = int(h * self.ROI_Y1)
        roi = frame[roi_y0:roi_y1, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, 20, 80)
        if exclusion_mask is not None:
            edges[exclusion_mask[roi_y0:roi_y1,:]>0]=0
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180.0, threshold=30, minLineLength=int(w * self.MIN_LENGTH_RATIO), maxLineGap=45)
        if lines is None:
            return []
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB).astype(np.float32)
        bgr = frame.astype(np.float32)
        raw_candidates = []
        for item in lines:
            x1, y1, x2, y2 = np.asarray(item).reshape(-1)[:4]
            line = line_geometry((int(x1), int(y1 + roi_y0), int(x2), int(y2 + roi_y0)))
            if abs(line_angle_deg(*line)) > self.MAX_ANGLE_DEG:
                continue
            if line_length(*line) < w * self.MIN_LENGTH_RATIO:
                continue
            raw_candidates.append(line)
        raw_candidates.sort(key=lambda item: line_length(*item), reverse=True)
        candidates = []
        for line in raw_candidates[:self.MAX_RAW_CANDIDATES]:
            candidate = self._candidate_from_line(frame, line, hsv, lab, bgr)
            if candidate is not None:
                candidates.append(candidate)
        candidates.sort(key=lambda item: item['score'], reverse=True)
        return self._merge_candidates(frame, candidates, hsv, lab, bgr)
