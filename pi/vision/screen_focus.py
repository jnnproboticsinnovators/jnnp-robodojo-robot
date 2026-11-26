#!/usr/bin/env python3
"""
JNNP RoboDojo - find the TABLET SCREEN in the camera picture, so the waste AI looks only at the
picture shown on the screen (not the bezel, the plastic holder, or the room behind it).

How it finds the screen:
  * looks for 4-cornered shapes (edges) in the picture
  * a screen's edge is BRIGHTER on the inside (the picture) than on the outside (the dark bezel);
    the tablet's outer edge is the other way round - so the inner screen edge is picked,
    never the plastic around it
  * the corners are smoothed over a few pictures so the box does not shake, and it is kept for a
    few seconds if the screen is briefly missed (e.g. while the picture on it changes)
  * the screen is then "flattened" (straightened even if the tablet is tilted) and a small
    margin is cut off, and only that goes to the AI

Used by waste_live.py (Pi) and waste_laptop.py (laptop). Test on a photo:
    python3 screen_focus.py photo.jpg       -> saves photo_screen.jpg (outline) and photo_crop.jpg
"""
import sys
import time

import cv2
import numpy as np

MIN_AREA = 0.03       # screen must cover at least 3% of the picture ...
MAX_AREA = 0.92       # ... and not (almost) all of it
INNER_MARGIN = 0.03   # cut 3% off each side of the screen (removes the last bit of bezel)
CONTRAST = 12         # inside must be at least this much brighter than the border (0-255)
KEEP_SECONDS = 3.0    # keep the last screen this long if it is not found in a picture
SMOOTH = 0.5          # 0 = jump to new corners at once, 0.9 = very slow/steady


def _order(pts):
    """4 corners -> top-left, top-right, bottom-right, bottom-left."""
    pts = np.asarray(pts, dtype=np.float32).reshape(4, 2)
    s = pts.sum(1)
    d = np.diff(pts, axis=1).ravel()
    return np.array([pts[np.argmin(s)], pts[np.argmin(d)], pts[np.argmax(s)], pts[np.argmax(d)]],
                    dtype=np.float32)


def _band_means(gray, quad):
    """Mean brightness just INSIDE and just OUTSIDE the quad's edge."""
    h, w = gray.shape
    mask = np.zeros((h, w), np.uint8)
    cv2.fillConvexPoly(mask, quad.astype(np.int32), 255)
    k = max(3, int(0.025 * min(h, w)) | 1)
    kernel = np.ones((k, k), np.uint8)
    inside = cv2.subtract(mask, cv2.erode(mask, kernel))
    outside = cv2.subtract(cv2.dilate(mask, kernel), mask)
    if inside.sum() == 0 or outside.sum() == 0:
        return 0.0, 0.0
    return cv2.mean(gray, inside)[0], cv2.mean(gray, outside)[0]


def detect_screen(frame_bgr):
    """Returns the screen's 4 corners (float32 4x2, full-picture pixels) or None."""
    H, W = frame_bgr.shape[:2]
    scale = 320.0 / W
    small = cv2.resize(frame_bgr, (320, int(round(H * scale))), interpolation=cv2.INTER_AREA)
    gray = cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    h, w = gray.shape
    cnts = []
    for lo, hi in ((30, 100), (10, 35)):            # strong edges, then weak ones (dark pictures)
        edges = cv2.dilate(cv2.Canny(gray, lo, hi), np.ones((3, 3), np.uint8), iterations=1)
        cnts += list(cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0])
    cands = []                                      # (area, quad, contrast)
    for c in cnts:
        area = cv2.contourArea(c)
        if area < MIN_AREA * w * h or area > MAX_AREA * w * h:
            continue
        hull = cv2.convexHull(c)
        quad = cv2.approxPolyDP(hull, 0.04 * cv2.arcLength(hull, True), True)
        if len(quad) != 4:
            continue
        quad = _order(quad)
        qa = cv2.contourArea(quad)
        if qa < MIN_AREA * w * h or area < 0.8 * qa:          # must really be a filled 4-sided shape
            continue
        sides = np.linalg.norm(quad - np.roll(quad, -1, axis=0), axis=1)
        if sides.min() < 0.15 * sides.max():                   # not a thin sliver
            continue
        inside, outside = _band_means(gray, quad)
        if inside - outside < 3:                               # screen glows, bezel is dark
            continue
        cands.append((qa, quad, inside - outside))
    top = [t for t in cands if t[2] >= CONTRAST]
    if not top:
        return None
    # Start with the biggest bright 4-sided shape (may be the tablet holder or the tablet itself),
    # then step INSIDE it while there is another bright 4-sided shape within it: the screen is
    # always the innermost one (holder -> tablet -> screen). The picture's own content is
    # not 4-sided, and a shape covering < 25% of its parent is treated as content, not a screen.
    cands.sort(key=lambda t: -t[0])
    area, best, _ = max(top, key=lambda t: t[0])
    while True:
        inner = [(a, q) for a, q, _ in cands
                 if 0.25 * area < a < 0.92 * area
                 and all(cv2.pointPolygonTest(best.reshape(-1, 1, 2), (float(x), float(y)), True) > -3
                         for x, y in q)]
        if not inner:
            break
        area, best = inner[0]
    return best / scale


def detect_bright_screen(frame_bgr):
    """Second method: a screen GLOWS - find the large bright area (joining parts that the picture's
    darker objects split apart) and fit 4 corners around it. Returns corners or None."""
    H, W = frame_bgr.shape[:2]
    scale = 320.0 / W
    small = cv2.resize(frame_bgr, (320, int(round(H * scale))), interpolation=cv2.INTER_AREA)
    v = cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2HSV)[:, :, 2], (7, 7), 0)
    h, w = v.shape
    otsu, _ = cv2.threshold(v, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    thr = max(otsu, 0.92 * np.percentile(v, 90))
    m = (v >= thr).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    cnts = [c for c in cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
            if cv2.contourArea(c) > 0.008 * w * h]
    if not cnts:
        return None
    main = max(cnts, key=cv2.contourArea)
    x, y, bw, bh = cv2.boundingRect(main)
    gap = 0.08 * w
    # the picture's objects (fruit, bottle) split the white screen into pieces side by side or
    # above each other: join every bright piece that lines up with the main one
    parts = [c for c in cnts if c is main or _near(cv2.boundingRect(c), (x, y, bw, bh), gap)
             or _lines_up(cv2.boundingRect(c), (x, y, bw, bh))]
    hull = cv2.convexHull(np.vstack(parts))
    if cv2.contourArea(hull) < MIN_AREA * w * h or cv2.contourArea(hull) > MAX_AREA * w * h:
        return None
    quad = cv2.approxPolyDP(hull, 0.04 * cv2.arcLength(hull, True), True)
    quad = _order(quad if len(quad) == 4 else cv2.boxPoints(cv2.minAreaRect(hull)))
    quad[:, 0] = np.clip(quad[:, 0], 0, w - 1)
    quad[:, 1] = np.clip(quad[:, 1], 0, h - 1)
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    inside, outside = _band_means(gray, quad)
    if inside - outside < CONTRAST:
        return None
    return quad / scale


def enhance(img):
    """Make the screen picture look like the real photo again before the AI sees it:
      * colours: the camera often makes a white screen look BLUE (wrong white balance) - the
        brightest parts of a screen are white, so each colour is scaled until they are white
      * sharpness: a gentle sharpening against the camera blur"""
    f = img.astype(np.float32)
    top = np.percentile(f.reshape(-1, 3), 97, axis=0)
    if top.min() > 40:                                   # skip on a very dark picture
        f *= 235.0 / top                                 # brightest parts -> white
    f = np.clip(f, 0, 255)
    blur = cv2.GaussianBlur(f, (0, 0), 2.0)
    return np.clip(cv2.addWeighted(f, 1.6, blur, -0.6, 0), 0, 255).astype(np.uint8)


def _lines_up(a, b):
    """Two pieces share most of their height (side by side) or width (above each other)."""
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    oy = min(ay + ah, by + bh) - max(ay, by)
    ox = min(ax + aw, bx + bw) - max(ax, bx)
    return oy > 0.6 * min(ah, bh) or ox > 0.6 * min(aw, bw)


def _fit_quad(points):
    hull = cv2.convexHull(np.asarray(points, np.float32).reshape(-1, 1, 2))
    quad = cv2.approxPolyDP(hull, 0.04 * cv2.arcLength(hull, True), True)
    return _order(quad if len(quad) == 4 else cv2.boxPoints(cv2.minAreaRect(hull)))


def _near(a, b, gap):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    return ax < bx + bw + gap and bx < ax + aw + gap and ay < by + bh + gap and by < ay + ah + gap


def _combine(qe, qb, W):
    """Both methods found something - decide which is the screen."""
    big, small = (qe, qb) if cv2.contourArea(qe) >= cv2.contourArea(qb) else (qb, qe)
    if cv2.contourArea(small) > 0.8 * cv2.contourArea(big):
        return big                                       # same thing found twice
    lx, ly, lw, lh = cv2.boundingRect(big.astype(np.int32))
    sx, sy, sw, sh = cv2.boundingRect(small.astype(np.int32))
    m = 0.01 * W
    inside = sx >= lx - m and sy >= ly - m and sx + sw <= lx + lw + m and sy + sh <= ly + lh + m
    if inside:
        gaps = [sx - lx, sy - ly, lx + lw - sx - sw, ly + lh - sy - sh]
        # a border all round (frame/bezel around the screen) -> the inner one is the screen;
        # touching the outer edges -> it is only a PIECE of the screen, the big one is right
        return small if min(gaps) > 0.02 * W else big
    if _lines_up((lx, ly, lw, lh), (sx, sy, sw, sh)):
        return _fit_quad(np.vstack([qe, qb]))            # side by side pieces: join them
    return big


class ScreenFocus:
    """Keeps track of the screen over time and cuts it out for the AI."""

    def __init__(self, fallback_box=(0.55, 0.65)):
        self.fallback_box = fallback_box
        self.quad = None
        self.last_seen = 0.0

    def update(self, frame_bgr):
        qe = detect_screen(frame_bgr)          # by its edges
        qb = detect_bright_screen(frame_bgr)   # by its glow
        q = qe if qb is None else qb if qe is None else None
        if q is None:
            q = _combine(qe, qb, frame_bgr.shape[1])
        now = time.time()
        if q is not None:
            if self.quad is not None and np.abs(q - self.quad).max() < 0.15 * frame_bgr.shape[1]:
                q = SMOOTH * self.quad + (1 - SMOOTH) * q      # same screen: smooth the corners
            self.quad, self.last_seen = q.astype(np.float32), now
        elif self.quad is not None and now - self.last_seen > KEEP_SECONDS:
            self.quad = None
        return self.quad

    def found(self):
        return self.quad is not None

    def crop(self, frame_bgr):
        """Returns (crop_image, to_frame) - to_frame(x1,y1,x2,y2) maps a box on the crop back
        to a box on the full picture."""
        H, W = frame_bgr.shape[:2]
        if self.quad is None:                                  # no screen: middle focus box
            bw, bh = int(W * self.fallback_box[0]), int(H * self.fallback_box[1])
            bx, by = (W - bw) // 2, (H - bh) // 2
            return frame_bgr[by:by + bh, bx:bx + bw], \
                lambda x1, y1, x2, y2: (x1 + bx, y1 + by, x2 + bx, y2 + by)
        q = self.quad
        cw = int(max(np.linalg.norm(q[0] - q[1]), np.linalg.norm(q[3] - q[2])))
        ch = int(max(np.linalg.norm(q[0] - q[3]), np.linalg.norm(q[1] - q[2])))
        cw, ch = max(cw, 16), max(ch, 16)
        mx, my = INNER_MARGIN * cw, INNER_MARGIN * ch
        dst = np.array([[-mx, -my], [cw - 1 + mx, -my], [cw - 1 + mx, ch - 1 + my], [-mx, ch - 1 + my]],
                       dtype=np.float32)
        M = cv2.getPerspectiveTransform(q, dst)
        Minv = np.linalg.inv(M)
        crop = enhance(cv2.warpPerspective(frame_bgr, M, (cw, ch), flags=cv2.INTER_CUBIC))

        def to_frame(x1, y1, x2, y2):
            pts = np.array([[[x1, y1], [x2, y1], [x2, y2], [x1, y2]]], dtype=np.float32)
            p = cv2.perspectiveTransform(pts, Minv)[0]
            return (int(max(0, p[:, 0].min())), int(max(0, p[:, 1].min())),
                    int(min(W - 1, p[:, 0].max())), int(min(H - 1, p[:, 1].max())))
        return crop, to_frame

    def draw(self, frame):
        H, W = frame.shape[:2]
        if self.quad is not None:
            cv2.polylines(frame, [self.quad.astype(np.int32)], True, (0, 255, 0), 2)
            cv2.putText(frame, "SCREEN", tuple(int(v) for v in self.quad[0] + (4, -6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            bw, bh = int(W * self.fallback_box[0]), int(H * self.fallback_box[1])
            bx, by = (W - bw) // 2, (H - bh) // 2
            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (0, 255, 255), 2)
            cv2.putText(frame, "no screen found - middle box", (bx + 4, by - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        return frame


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit("Usage: python3 screen_focus.py photo.jpg")
    img = cv2.imread(sys.argv[1])
    if img is None:
        sys.exit(f"Could not open {sys.argv[1]}")
    t = time.time()
    sf = ScreenFocus()
    sf.update(img)
    ms = (time.time() - t) * 1000
    crop, _ = sf.crop(img)
    base = sys.argv[1].rsplit(".", 1)[0]
    cv2.imwrite(base + "_screen.jpg", sf.draw(img.copy()))
    cv2.imwrite(base + "_crop.jpg", crop)
    print(("Screen FOUND" if sf.found() else "No screen found (middle box used)") + f"  ({ms:.0f} ms)")
    print(f"Saved {base}_screen.jpg and {base}_crop.jpg")
