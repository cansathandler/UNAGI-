"""
virtual_camera.py
=================

Renders a synthetic TOP-VIEW image of the robot with REAL AprilTag
bitmaps (DICT_APRILTAG_36h11) pasted on each link, so that the existing
`src/Detection_Pipeline/Tracker/april_tracker.py` can be pointed at this
image stream unmodified.

Camera model: pinhole, optical axis pointing straight down (-Z world),
robot swims in the z = 0 plane.

    p_cam = R (p_world - C),   R = diag(1, -1, -1),  C = (cx, cy, H)
    u = fx * Xc/Zc + u0 ,  v = fy * Yc/Zc + v0

Optional realism knobs: motion blur, Gaussian noise, radial distortion,
frame dropping, latency.  Turn them on one at a time -- that is how you
find out which one breaks your control loop before you get wet.
"""

from __future__ import annotations

import cv2
import numpy as np


class TopViewCamera:
    def __init__(self, width=1280, height=720, fx=900.0, fy=900.0,
                 height_m=1.20, centre_m=(0.0, 0.0),
                 noise_sigma=0.0, blur_px=0, jpeg_quality=None,
                 dist_coeffs=None):
        self.W, self.H = int(width), int(height)
        self.fx, self.fy = float(fx), float(fy)
        self.u0, self.v0 = self.W / 2.0, self.H / 2.0
        self.h = float(height_m)
        self.C = np.array([centre_m[0], centre_m[1], self.h], dtype=float)
        self.noise_sigma = noise_sigma
        self.blur_px = blur_px
        self.jpeg_quality = jpeg_quality

        self.camera_matrix = np.array([[self.fx, 0, self.u0],
                                       [0, self.fy, self.v0],
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = (np.zeros((5, 1), dtype=np.float32)
                            if dist_coeffs is None
                            else np.asarray(dist_coeffs, np.float32).reshape(-1, 1))

        self._tag_cache = {}

    # ------------------------------------------------------------------
    @classmethod
    def from_fov(cls, fov_width_m, width=1920, height=1200, fx=1800.0, **kw):
        """Place the camera so that `fov_width_m` spans the image width.

        This is the honest way to size the rig: you pick how much tank you
        need to see, and the resolution per tag falls out of it.
        """
        h = fx * fov_width_m / width
        return cls(width=width, height=height, fx=fx, fy=fx,
                   height_m=h, **kw)

    def metres_per_pixel(self):
        return self.h / self.fx

    def tag_pixels(self, tag_size_m):
        """Side length of the black tag in pixels -- the number that decides
        whether detection works at all.  36h11 wants >= ~30 px, >= 60 px
        for reliable sub-degree orientation."""
        return tag_size_m / self.metres_per_pixel()

    def project(self, pts_xy, z=0.0):
        """pts_xy (..., 2) world metres -> (..., 2) pixels."""
        p = np.atleast_2d(np.asarray(pts_xy, dtype=float))
        Xc = p[:, 0] - self.C[0]
        Yc = self.C[1] - p[:, 1]
        Zc = self.C[2] - z
        u = self.fx * Xc / Zc + self.u0
        v = self.fy * Yc / Zc + self.v0
        return np.column_stack([u, v])

    # ------------------------------------------------------------------
    def _tag_bitmap(self, tag_id, modules_px=32, quiet_modules=2):
        """36h11 tag with a white quiet zone (the detector needs it).

        OpenCV renders DICT_APRILTAG_36h11 as 8 modules across
        (6 data bits + 1 black border ring on each side).
        """
        key = (tag_id, modules_px, quiet_modules)
        if key in self._tag_cache:
            return self._tag_cache[key]

        n_mod = 8
        px = n_mod * modules_px
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        marker = cv2.aruco.generateImageMarker(d, int(tag_id), px)
        pad = modules_px * quiet_modules
        img = cv2.copyMakeBorder(marker, pad, pad, pad, pad,
                                 cv2.BORDER_CONSTANT, value=255)
        self._tag_cache[key] = (img, n_mod, quiet_modules)
        return self._tag_cache[key]

    # ------------------------------------------------------------------
    def render(self, chain, nodes, phi, tag_ids=None, background=245,
               link_width_m=None):
        """Return a BGR uint8 image of the chain seen from above.

        link_width_m : optional.  Draws each link as a plain rectangle of
                       this width, purely so the video is readable.  It is
                       a drawing hint, not part of the model, and it has no
                       effect on detection.  None = spine only.
        """
        img = np.full((self.H, self.W, 3), background, np.uint8)

        # ---- links ----------------------------------------------------
        if link_width_m:
            hw = link_width_m / 2.0
            for k in range(chain.n_links):
                a, b = nodes[k], nodes[k + 1]
                nx, ny = -np.sin(phi[k]), np.cos(phi[k])
                off = np.array([nx, ny]) * hw
                quad = np.array([a + off, b + off, b - off, a - off])
                poly = self.project(quad).astype(np.int32)
                cv2.fillPoly(img, [poly], (176, 176, 176))
                cv2.polylines(img, [poly], True, (110, 110, 110), 1,
                              cv2.LINE_AA)

        # ---- spine and joints -----------------------------------------
        spine = self.project(nodes).astype(np.int32)
        cv2.polylines(img, [spine], False, (70, 70, 70), 1, cv2.LINE_AA)
        for p in spine[1:-1]:
            cv2.circle(img, tuple(p), 4, (40, 40, 40), -1, cv2.LINE_AA)

        # ---- tags -----------------------------------------------------
        centres, headings = chain.tag_poses(nodes, phi)
        if tag_ids is None:
            tag_ids = np.arange(len(centres))

        s = chain.tag_size
        for k, (c, a, tid) in enumerate(zip(centres, headings, tag_ids)):
            bitmap, nm, qm = self._tag_bitmap(int(tid))
            # the bitmap includes a quiet zone -> the physical white
            # square is larger than the black tag itself
            full = s * (nm + 2 * qm) / nm
            hs = full / 2.0
            ca, sa = np.cos(a), np.sin(a)
            R = np.array([[ca, -sa], [sa, ca]])
            # bitmap corner order is TL, TR, BR, BL in IMAGE coordinates.
            # world +y maps to image -v, so the world-frame equivalents are
            # (-,+), (+,+), (+,-), (-,-).  Getting this wrong renders the
            # tag MIRRORED: the detector still finds the quad but the bit
            # decode fails and every tag lands in `rejected`.
            local = np.array([[-hs, hs], [hs, hs], [hs, -hs], [-hs, -hs]])
            world = (R @ local.T).T + c
            dst = self.project(world).astype(np.float32)

            hpx, wpx = bitmap.shape[:2]
            src = np.array([[0, 0], [wpx - 1, 0],
                            [wpx - 1, hpx - 1], [0, hpx - 1]], np.float32)
            M = cv2.getPerspectiveTransform(src, dst)

            tag_bgr = cv2.cvtColor(bitmap, cv2.COLOR_GRAY2BGR)
            warped = cv2.warpPerspective(tag_bgr, M, (self.W, self.H),
                                         flags=cv2.INTER_AREA,
                                         borderMode=cv2.BORDER_CONSTANT,
                                         borderValue=(0, 0, 0))
            mask = np.zeros((self.H, self.W), np.uint8)
            cv2.fillConvexPoly(mask, dst.astype(np.int32), 255)
            img[mask > 0] = warped[mask > 0]

        # ---- degradation ---------------------------------------------
        if self.blur_px and self.blur_px > 0:
            kk = int(self.blur_px) | 1
            img = cv2.GaussianBlur(img, (kk, kk), 0)
        if self.noise_sigma > 0:
            n = np.random.normal(0, self.noise_sigma, img.shape)
            img = np.clip(img.astype(np.float32) + n, 0, 255).astype(np.uint8)
        if self.jpeg_quality:
            enc = cv2.imencode(".jpg", img,
                               [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])[1]
            img = cv2.imdecode(enc, cv2.IMREAD_COLOR)
        return img


# ----------------------------------------------------------------------
class TagAngleEstimator:
    """Detect the tags and turn them into measured JOINT angles.

    This is deliberately written the same way the real pipeline must
    work: it only sees the image, never the simulator state.
    """

    def __init__(self, n_links, tag_size_m=0.025,
                 dictionary=cv2.aruco.DICT_APRILTAG_36h11):
        self.n_links = n_links
        self.tag_size = tag_size_m
        d = cv2.aruco.getPredefinedDictionary(dictionary)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(d, params)
        self.last_phi = np.zeros(n_links)
        self.last_centre = np.zeros((n_links, 2))

    def __call__(self, frame):
        """Returns (theta_meas (n_links-1,), phi (n_links,), centres, seen)"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        phi = self.last_phi.copy()
        centre = self.last_centre.copy()
        seen = np.zeros(self.n_links, bool)

        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                i = int(i)
                if not (0 <= i < self.n_links):
                    continue
                p = c.reshape(4, 2)
                # tag +x axis in image = midpoint(c0,c3) -> midpoint(c1,c2)
                fwd = ((p[1] + p[2]) - (p[0] + p[3])) / 2.0
                # image v axis points down -> flip to get world yaw
                phi[i] = np.arctan2(-fwd[1], fwd[0])
                centre[i] = p.mean(axis=0)
                seen[i] = True

        self.last_phi, self.last_centre = phi, centre
        theta = np.diff(np.unwrap(phi))
        return theta, phi, centre, seen
