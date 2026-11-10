import cv2
import numpy as np

class Observation:
    def __init__(self, state="SAFE", side="NONE",
                 near_x=None, near_y=None, count=0):
        self.state = state
        self.line_side = side
        self.color = "ORANGE" if state == "EDGE" else "UNKNOWN"
        self.near_x_norm = near_x
        self.near_y_norm = near_y
        self.near_confirm_count = count

    def packet(self):
        return "COURT,%s,%s,%s" % (
            self.state, self.line_side, self.color
        )

class Result:
    def packet(self):
        return self.observation.packet()

class OrangeRuntime:
    def __init__(self, edge_y=0.68, edge_confirm_frames=2):
        self.edge_y = edge_y
        self.edge_confirm_frames = edge_confirm_frames
        self.color_model_loaded = True
        self.previous = None
        self.count = 0

    def observe(self, frame, exclusion_mask=None):
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

        H = hsv[:,:,0]
        S = hsv[:,:,1]
        V = hsv[:,:,2]
        B = lab[:,:,2]

        # Indoor-only tighter orange validation, preserving existing geometry.
        # In particular, gray/black grout with warm reflections should not pass.
        blue = frame[:, :, 0].astype(np.int16)
        green = frame[:, :, 1].astype(np.int16)
        red = frame[:, :, 2].astype(np.int16)

        color = (
            ((H <= 25) | (H >= 175)) &
            (S >= 55) &
            (V >= 90) &
            (B >= 140) &
            ((red - green) >= 20) &
            ((red - blue) >= 30)
        )

        raw = (color.astype(np.uint8) * 255)
        raw[:int(h*0.50)] = 0

        if exclusion_mask is not None:
            ex = cv2.resize(
                exclusion_mask, (w,h),
                interpolation=cv2.INTER_NEAREST
            )
            raw[ex > 0] = 0

        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (9,3)
        )
        mask = cv2.morphologyEx(
            raw, cv2.MORPH_CLOSE, kernel
        )

        contours = cv2.findContours(
            mask, cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )[-2]

        candidates = []

        for cnt in contours:
            x,y,cw,ch = cv2.boundingRect(cnt)

            region = raw[y:y+ch,x:x+cw]
            pixels = cv2.countNonZero(region)

            if pixels < 60:
                continue

            # V5: 排除過厚或不規則的色塊
            rect = cv2.minAreaRect(cnt)
            rw,rh = rect[1]
            long_side = max(rw,rh)
            short_side = min(rw,rh)

            if long_side < w*0.25:
                continue
            if short_side > h*0.12:
                continue
            if long_side/max(short_side,1) < 4.0:
                continue

            # V5: 檢查候選區域與附近背景的顏色差異
            # 只比較原始橘色像素，避免大量背景稀釋線條
            inside = raw[y:y+ch,x:x+cw] > 0

            if np.count_nonzero(inside) < 60:
                continue

            line_s = float(np.median(
                S[y:y+ch,x:x+cw][inside]
            ))
            line_b = float(np.median(
                B[y:y+ch,x:x+cw][inside]
            ))

            margin = 24
            x0 = max(0,x-margin)
            y0 = max(0,y-margin)
            x1 = min(w,x+cw+margin)
            y1 = min(h,y+ch+margin)

            bg_mask = np.ones(
                (y1-y0,x1-x0), dtype=bool
            )

            # 排除候選區域及其附近，取得周圍地板
            ix0 = max(0,x-x0-4)
            iy0 = max(0,y-y0-4)
            ix1 = min(x1-x0,x+cw-x0+4)
            iy1 = min(y1-y0,y+ch-y0+4)
            bg_mask[iy0:iy1,ix0:ix1] = False

            if np.count_nonzero(bg_mask) < 100:
                continue

            bg_s = float(np.median(
                S[y0:y1,x0:x1][bg_mask]
            ))
            bg_b = float(np.median(
                B[y0:y1,x0:x1][bg_mask]
            ))

            delta_s = line_s-bg_s
            delta_b = line_b-bg_b

            # 背景對比：使用兩種色彩特徵
            if delta_s < 5 or delta_b < 2:
                continue

            near_y = (y+ch)/float(h)
            center_x = (x+cw/2.0)/float(w)

            candidates.append((
                pixels, x,y,cw,ch,center_x,near_y,
                delta_s,delta_b,None
            ))


        # V6: Hough fallback for diagonal and crossing orange lines.
        # Only used when the original contour detector has no candidate.
        if not candidates:
            segments = cv2.HoughLinesP(
                mask,
                rho=1,
                theta=np.pi / 180,
                threshold=25,
                minLineLength=max(35, int(w * 0.20)),
                maxLineGap=14
            )

            if segments is not None:
                for seg in segments[:, 0]:
                    x1, y1, x2, y2 = map(int, seg)
                    length = np.hypot(x2-x1, y2-y1)

                    if length < w * 0.20:
                        continue

                    # Sample pixels along the detected segment.
                    n = max(30, int(length / 3))
                    xs = np.rint(
                        np.linspace(x1, x2, n)
                    ).astype(int)
                    ys = np.rint(
                        np.linspace(y1, y2, n)
                    ).astype(int)

                    xs = np.clip(xs, 0, w-1)
                    ys = np.clip(ys, 0, h-1)

                    valid = raw[ys, xs] > 0

                    if np.mean(valid) < 0.60:
                        continue

                    if np.count_nonzero(valid) < 20:
                        continue

                    # Compare orange line with nearby background.
                    nx = -(y2-y1) / max(length, 1)
                    ny = (x2-x1) / max(length, 1)

                    bg_s_values = []
                    bg_b_values = []

                    for offset in (-20, 20):
                        bx = np.rint(xs + nx*offset).astype(int)
                        by = np.rint(ys + ny*offset).astype(int)

                        inside = (
                            (bx >= 0) & (bx < w) &
                            (by >= 0) & (by < h)
                        )

                        bx = bx[inside]
                        by = by[inside]

                        if len(bx) == 0:
                            continue

                        bg_valid = raw[by, bx] == 0
                        bg_s_values.extend(
                            S[by[bg_valid], bx[bg_valid]].tolist()
                        )
                        bg_b_values.extend(
                            B[by[bg_valid], bx[bg_valid]].tolist()
                        )

                    if len(bg_s_values) < 20:
                        continue

                    line_s = float(np.median(S[ys[valid], xs[valid]]))
                    line_b = float(np.median(B[ys[valid], xs[valid]]))

                    delta_s = line_s - float(np.median(bg_s_values))
                    delta_b = line_b - float(np.median(bg_b_values))

                    if delta_s < 5 or delta_b < 2:
                        continue

                    # Use the endpoint nearest the vehicle.
                    if y1 >= y2:
                        near_x, near_pixel_y = x1, y1
                    else:
                        near_x, near_pixel_y = x2, y2

                    cx = near_x / float(w)
                    near_y = near_pixel_y / float(h)

                    candidates.append((
                        int(np.count_nonzero(valid)),
                        min(x1,x2), min(y1,y2),
                        abs(x2-x1)+1, abs(y2-y1)+1,
                        cx, near_y, delta_s, delta_b,
                        (x1,y1,x2,y2)
                    ))

        result = Result()
        result.observation = Observation()
        result.raw_label = "unknown"
        result.raw_confidence = 0.0
        result.position_state = "unavailable"
        result.color_status = "no_line"
        result.color_label = "unknown"
        result.color_confidence = 0.0
        result.color_model_loaded = True
        result.line = None

        if not candidates:
            self.previous = None
            self.count = 0
            return result

        # 優先考慮靠近車體的候選線，
        # 再根據橘色像素數量排序
        candidates.sort(
            key=lambda a: (a[6] >= self.edge_y,a[0]),
            reverse=True
        )

        pixels,x,y,cw,ch,cx,ny,ds,db,segment = candidates[0]

        current = (cx,ny)

        same = False
        if self.previous is not None:
            px,py = self.previous
            same = (
                abs(cx-px) < 0.25 and
                abs(ny-py) < 0.25
            )

        self.previous = current

        if ny >= self.edge_y:
            self.count = self.count+1 if same else 1
        else:
            self.count = 0

        state = (
            "EDGE"
            if self.count >= self.edge_confirm_frames
            else "SAFE"
        )

        side = (
            "LEFT" if cx < 0.4 else
            "RIGHT" if cx > 0.6 else "CENTER"
        )

        result.observation = Observation(
            state,
            side if state == "EDGE" else "NONE",
            cx,ny,self.count
        )

        result.raw_label = "orange"
        # 規則式辨識沒有機率信心值
        result.raw_confidence = 1.0
        result.position_state = "observed"
        result.color_status = "confirmed"
        result.color_label = "orange"
        result.color_confidence = 1.0

        if segment is not None:
            result.line = segment
        else:
            ly = int(y+ch/2.0)
            result.line = (x,ly,x+cw,ly)

        return result
