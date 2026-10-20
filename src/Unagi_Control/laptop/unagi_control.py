#!/usr/bin/env python3
"""
UNAGI laptop controller
=======================

    camera -> AprilTag (pupil-apriltags) -> joint angles
           -> CPG reference (phase oscillators) + feedforward + PD
           -> UDP duty vector -> head -> RS-485 -> 10 motor nodes

One control step per camera frame. The head and nodes have no sensors and no
control law: everything closed-loop lives here.

Run modes
---------
  python unagi_control.py                      closed loop (camera)   [keys: g z space q]
  python unagi_control.py --open-loop          CPG feedforward only, no camera (bench test)
  python unagi_control.py --calibrate          per-joint zero + static gain (deg/duty) -> unagi_calib.json
  python unagi_control.py --mode carangiform --freq 2.0 --amp 20

Keys (closed-loop window)
-------------------------
  g      start / stop the gait (amplitude ramps smoothly)
  z      capture zero offsets (robot must be straight, gait off)
  space  emergency stop (STOP + disarm)
  q/Esc  quit (STOP + disarm)

Dependencies: numpy, opencv-python, pupil-apriltags   (+ pypylon for Basler)
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import struct
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# =============================================================================
# CONFIG  — edit here
# =============================================================================
N_JOINTS = 10

HEAD_IP, HEAD_PORT = "192.168.4.1", 4210     # head runs a WiFi AP

CAMERA = "opencv"            # "opencv" or "basler"
CAMERA_INDEX = 0
CAMERA_SIZE = (1280, 720)    # opencv only
CAMERA_FPS = 120             # requested (opencv only)

TAG_FAMILY = "tag36h11"
HEAD_TAG_ID = 0
JOINT_TAG_IDS = list(range(1, N_JOINTS + 1))   # tag on top of motor i  (i = 1..N)
TAIL_TAG_ID = 11             # tag near the tail tip; None -> last joint runs feedforward only
TAG_HOLD_S = 0.10            # reuse last tag position for this long after a dropout

DUTY_LIMIT = 800             # |duty| clamp sent to the head (firmware clamps at 1000)
ANGLE_LIMIT_DEG = 25.0       # reference clamp (mechanical limit is +/-30)

# Controller (all gains act in "degrees", then divided by the static gain deg/duty)
KP = 0.8                     # deg of extra command per deg of error
KD = 0.02                    # deg of extra command per deg/s of error rate
LATENCY_S = 0.025            # camera + detection + WiFi + bus delay, used for phase advance
ANGLE_LPF_HZ = 25.0          # measured angle low-pass
VEL_LPF_HZ = 10.0            # measured angular-velocity low-pass

DEFAULT_GAIN_DEG_PER_DUTY = 30.0 / 1000.0   # replaced by --calibrate results
CALIB_FILE = Path(__file__).with_name("unagi_calib.json")
LOG_DIR = Path(__file__).with_name("logs")

# Gait presets: amplitude envelope A(s) = a0 + (1-a0) * s^p  along the body (s = 0 head .. 1 tail),
# body_waves = number of wavelengths on the body (total head->tail phase lag = 2*pi*body_waves).
# Starting values only — tune / optimise.
GAITS = {
    "anguilliform":   dict(a0=0.55, p=1.0, body_waves=1.2),
    "subcarangiform": dict(a0=0.20, p=1.5, body_waves=1.0),
    "carangiform":    dict(a0=0.05, p=2.5, body_waves=0.9),
    "thunniform":     dict(a0=0.00, p=6.0, body_waves=0.8),
    "snake":          dict(a0=1.00, p=1.0, body_waves=1.0),
}


# =============================================================================
# Head link (UDP)
# =============================================================================
class HeadLink:
    CMD, STOP, ARM, DISARM, PING = 1, 2, 3, 4, 5
    TX_HDR = struct.Struct("<2sBBHI")          # 'UN', type, n, seq, t_ms      (10 B)
    RX = struct.Struct("<2sHIffffB")           # 'UT', seq, ms, V, I, P, E, flags (25 B)
    FLAG_NAMES = {1: "ARMED", 2: "STREAM", 4: "TIMEOUT", 8: "OVERCURRENT", 16: "INA_FAIL"}

    def __init__(self, ip: str | None = None, port: int | None = None):
        self.addr = (ip or HEAD_IP, port or HEAD_PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.seq = 0
        self.t0 = time.perf_counter()
        self.telemetry = None          # (seq, ms, V, I, P, E, flags)
        self.rx_count = 0

    def _send(self, typ: int, duties=()):
        self.seq = (self.seq + 1) & 0xFFFF
        t_ms = int((time.perf_counter() - self.t0) * 1000) & 0xFFFFFFFF
        pkt = self.TX_HDR.pack(b"UN", typ, len(duties), self.seq, t_ms)
        if duties:
            pkt += struct.pack(f"<{len(duties)}h", *duties)
        try:
            self.sock.sendto(pkt, self.addr)
        except OSError:
            pass

    def send_duty(self, duty: np.ndarray):
        d = np.clip(np.round(duty), -DUTY_LIMIT, DUTY_LIMIT).astype(int).tolist()
        self._send(self.CMD, d)

    def stop(self, repeat: int = 3):
        for _ in range(repeat):
            self._send(self.STOP)

    def arm(self):
        self._send(self.ARM)

    def disarm(self, repeat: int = 3):
        for _ in range(repeat):
            self._send(self.DISARM)

    def ping(self):
        self._send(self.PING)

    def poll(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(64)
            except (BlockingIOError, OSError):
                break
            if len(data) == self.RX.size and data[:2] == b"UT":
                self.telemetry = self.RX.unpack(data)[1:]
                self.rx_count += 1
        return self.telemetry

    @classmethod
    def flag_str(cls, flags: int) -> str:
        return "|".join(n for b, n in cls.FLAG_NAMES.items() if flags & b) or "-"


# =============================================================================
# Camera
# =============================================================================
class Camera:
    def __init__(self, kind: str = CAMERA):
        self.kind = kind
        if kind == "basler":
            from pypylon import pylon
            self.pylon = pylon
            self.cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
            self.cam.Open()
            self.cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        else:
            import cv2
            self.cv2 = cv2
            self.cap = cv2.VideoCapture(CAMERA_INDEX)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_SIZE[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_SIZE[1])
            self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # newest frame, not a queued one
            if not self.cap.isOpened():
                raise RuntimeError(f"cannot open camera {CAMERA_INDEX}")

    def read(self):
        """Return (gray uint8 image, capture time [perf_counter s])."""
        if self.kind == "basler":
            r = self.cam.RetrieveResult(1000, self.pylon.TimeoutHandling_ThrowException)
            t = time.perf_counter()
            img = r.Array.copy()
            r.Release()
            if img.ndim == 3:
                img = img[..., 0]
            return img, t
        ok, frame = self.cap.read()
        t = time.perf_counter()
        if not ok:
            raise RuntimeError("camera read failed")
        return self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2GRAY), t

    def close(self):
        if self.kind == "basler":
            self.cam.StopGrabbing()
            self.cam.Close()
        else:
            self.cap.release()


# =============================================================================
# AprilTag -> joint angles
# =============================================================================
def signed_angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    """Signed angle from vector a to vector b (image coordinates), degrees."""
    return math.degrees(math.atan2(a[0] * b[1] - a[1] * b[0], a[0] * b[0] + a[1] * b[1]))


class JointEstimator:
    """
    Points along the body: P0 = head tag, P1..PN = motor tags, P(N+1) = tail tag.
    Joint i (motor i) angle = signed angle between (Pi - Pi-1) and (Pi+1 - Pi), minus zero offset.
    Positions (tag centres) are used, not tag yaw, so the result doesn't depend on which
    segment each tag is glued to. A joint with a missing neighbour is NaN -> feedforward only.
    """

    def __init__(self, zero_deg=None):
        self.ids = [HEAD_TAG_ID] + JOINT_TAG_IDS + ([TAIL_TAG_ID] if TAIL_TAG_ID is not None else [])
        self.last_pos = {i: None for i in self.ids}
        self.last_seen = {i: -1e9 for i in self.ids}
        self.zero = np.zeros(N_JOINTS) if zero_deg is None else np.asarray(zero_deg, float)
        self.theta = np.full(N_JOINTS, np.nan)     # filtered angle, deg
        self.omega = np.zeros(N_JOINTS)            # filtered rate, deg/s
        self.t_prev = None
        self.points = []                           # for drawing

    def raw_angles(self, dets, t: float) -> np.ndarray:
        for d in dets:
            if d.tag_id in self.last_pos:
                self.last_pos[d.tag_id] = np.asarray(d.center, float)
                self.last_seen[d.tag_id] = t
        pts = [self.last_pos[i] if (t - self.last_seen[i]) <= TAG_HOLD_S else None for i in self.ids]
        self.points = pts
        out = np.full(N_JOINTS, np.nan)
        for j in range(N_JOINTS):
            k = j + 1
            if k + 1 >= len(pts):
                break                                  # no tail tag -> last joint unobserved
            a, b, c = pts[k - 1], pts[k], pts[k + 1]
            if a is not None and b is not None and c is not None:
                out[j] = signed_angle_deg(b - a, c - b)
        return out

    def update(self, dets, t: float):
        raw = self.raw_angles(dets, t) - self.zero
        if self.t_prev is None:
            self.theta = raw.copy()
            self.t_prev = t
            return self.theta, self.omega
        dt = max(t - self.t_prev, 1e-4)
        self.t_prev = t
        a_th = 1.0 - math.exp(-2 * math.pi * ANGLE_LPF_HZ * dt)
        a_om = 1.0 - math.exp(-2 * math.pi * VEL_LPF_HZ * dt)
        for j in range(N_JOINTS):
            if np.isnan(raw[j]):
                self.theta[j] = np.nan
                self.omega[j] = 0.0
            elif np.isnan(self.theta[j]):
                self.theta[j] = raw[j]
                self.omega[j] = 0.0
            else:
                new = self.theta[j] + a_th * (raw[j] - self.theta[j])
                self.omega[j] += a_om * ((new - self.theta[j]) / dt - self.omega[j])
                self.theta[j] = new
        return self.theta, self.omega


def make_detector():
    from pupil_apriltags import Detector
    return Detector(families=TAG_FAMILY, nthreads=4, quad_decimate=2.0,
                    quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25)


# =============================================================================
# CPG: chain of phase oscillators with nearest-neighbour coupling
# =============================================================================
class CPG:
    def __init__(self, n: int, freq_hz: float, amp_deg: float, gait: str, coupling: float = 6.0,
                 amp_tau_s: float = 0.8):
        self.n = n
        self.coupling = coupling
        self.amp_tau = amp_tau_s
        self.set_gait(freq_hz, amp_deg, gait)
        self.phi = -np.arange(n) * self.lag          # start at the travelling-wave equilibrium
        self.amp = np.zeros(n)                       # current amplitudes (ramped)
        self.running = False

    def set_gait(self, freq_hz: float, amp_deg: float, gait: str):
        g = GAITS[gait]
        s = np.linspace(0.0, 1.0, self.n)
        env = g["a0"] + (1.0 - g["a0"]) * s ** g["p"]
        self.freq = freq_hz
        self.amp_target = np.minimum(amp_deg * env, ANGLE_LIMIT_DEG)
        self.lag = 2 * math.pi * g["body_waves"] / self.n   # phase lag per joint
        self.gait = gait

    def step(self, dt: float):
        w = 2 * math.pi * self.freq
        dphi = np.full(self.n, w)
        dphi[1:] += self.coupling * np.sin(self.phi[:-1] - self.phi[1:] - self.lag)
        dphi[:-1] += self.coupling * np.sin(self.phi[1:] - self.phi[:-1] + self.lag)
        self.phi += dphi * dt
        target = self.amp_target if self.running else np.zeros(self.n)
        self.amp += (target - self.amp) * (1 - math.exp(-dt / self.amp_tau))

    def reference(self, advance_s: float = 0.0):
        """Joint reference (deg) and its rate (deg/s), phase-advanced by advance_s."""
        w = 2 * math.pi * self.freq
        ph = self.phi + w * advance_s
        ref = np.clip(self.amp * np.sin(ph), -ANGLE_LIMIT_DEG, ANGLE_LIMIT_DEG)
        dref = self.amp * w * np.cos(ph)
        return ref, dref


# =============================================================================
# Controller: static-inverse feedforward + PD, per joint
# =============================================================================
class Controller:
    def __init__(self, gain_deg_per_duty):
        self.g = np.asarray(gain_deg_per_duty, float)    # signed: includes polarity

    def compute(self, ref, dref, theta, omega):
        cmd_deg = ref.copy()                              # feedforward
        ok = ~np.isnan(theta)
        cmd_deg[ok] += KP * (ref[ok] - theta[ok]) + KD * (dref[ok] - omega[ok])
        duty = cmd_deg / self.g
        return np.clip(duty, -DUTY_LIMIT, DUTY_LIMIT)


# =============================================================================
# Calibration file
# =============================================================================
def load_calib():
    if CALIB_FILE.exists():
        c = json.loads(CALIB_FILE.read_text())
        print(f"loaded {CALIB_FILE.name}")
        return np.array(c["zero_deg"], float), np.array(c["gain_deg_per_duty"], float)
    print("no calibration file -> zero offsets 0, default gain")
    return np.zeros(N_JOINTS), np.full(N_JOINTS, DEFAULT_GAIN_DEG_PER_DUTY)


def save_calib(zero, gain):
    CALIB_FILE.write_text(json.dumps({"zero_deg": list(map(float, zero)),
                                      "gain_deg_per_duty": list(map(float, gain))}, indent=2))
    print(f"saved {CALIB_FILE}")


# =============================================================================
# Logging
# =============================================================================
class Logger:
    def __init__(self, tag: str):
        LOG_DIR.mkdir(exist_ok=True)
        path = LOG_DIR / time.strftime(f"unagi_{tag}_%Y%m%d_%H%M%S.csv")
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        hdr = ["t", "dt", "seq", "running"]
        for j in range(1, N_JOINTS + 1):
            hdr += [f"ref{j}", f"theta{j}", f"duty{j}"]
        hdr += ["V", "I", "P", "E_J", "flags"]
        self.w.writerow(hdr)
        print(f"logging to {path}")

    def row(self, t, dt, seq, running, ref, theta, duty, tel):
        r = [f"{t:.4f}", f"{dt:.4f}", seq, int(running)]
        for j in range(N_JOINTS):
            r += [f"{ref[j]:.2f}", "" if np.isnan(theta[j]) else f"{theta[j]:.2f}", int(duty[j])]
        if tel:
            r += [f"{tel[2]:.3f}", f"{tel[3]:.3f}", f"{tel[4]:.3f}", f"{tel[5]:.2f}", tel[6]]
        else:
            r += ["", "", "", "", ""]
        self.w.writerow(r)

    def close(self):
        self.f.close()


# =============================================================================
# Run modes
# =============================================================================
def run_open_loop(args):
    """No camera: CPG feedforward at a fixed rate. For bench checks of bus + coils."""
    _, gain = load_calib()
    link, cpg, ctrl = HeadLink(), CPG(N_JOINTS, args.freq, args.amp, args.mode), Controller(gain)
    log = Logger("openloop")
    rate = 100.0
    nan = np.full(N_JOINTS, np.nan)
    link.arm()
    time.sleep(0.2)
    cpg.running = True
    t0 = t_prev = time.perf_counter()
    next_t = t0
    try:
        while time.perf_counter() - t0 < args.duration:
            next_t += 1.0 / rate
            while time.perf_counter() < next_t:
                pass
            t = time.perf_counter()
            dt, t_prev = t - t_prev, t
            if t - t0 > args.duration - 1.5:
                cpg.running = False                   # ramp down before the end
            cpg.step(dt)
            ref, _ = cpg.reference()
            duty = ctrl.compute(ref, np.zeros(N_JOINTS), nan, nan)
            link.send_duty(duty)
            tel = link.poll()
            log.row(t - t0, dt, link.seq, cpg.running, ref, nan, duty, tel)
    except KeyboardInterrupt:
        pass
    finally:
        link.stop(); link.disarm(); log.close()
        print(f"done. telemetry packets received: {link.rx_count}")


def run_calibrate(args):
    """Per joint: zero offset at duty 0, then a duty staircase -> static gain (deg/duty)."""
    cam, det, link = Camera(), make_detector(), HeadLink()
    est = JointEstimator()
    duties = [-600, -300, 0, 300, 600]
    zero_all = np.zeros(N_JOINTS)
    gain = np.full(N_JOINTS, DEFAULT_GAIN_DEG_PER_DUTY)

    def hold(duty_vec, seconds):
        samples = []
        t_end = time.perf_counter() + seconds
        while time.perf_counter() < t_end:
            img, t = cam.read()
            th, _ = est.update(det.detect(img), t)
            link.send_duty(duty_vec)                  # keep feeding the head watchdog
            link.poll()
            if time.perf_counter() > t_end - seconds / 2:
                samples.append(th.copy())
        return np.nanmean(np.array(samples), axis=0) if samples else np.full(N_JOINTS, np.nan)

    link.arm()
    time.sleep(0.2)
    try:
        print("zero offsets (all duty 0, robot straight)...")
        zero_all = hold(np.zeros(N_JOINTS), 1.5)
        est.zero = np.nan_to_num(zero_all)
        for j in range(N_JOINTS):
            xs, ys = [], []
            for d in duties:
                v = np.zeros(N_JOINTS)
                v[j] = d
                m = hold(v, 0.8)
                if not np.isnan(m[j]) and abs(m[j]) < 28:
                    xs.append(d); ys.append(m[j])
            hold(np.zeros(N_JOINTS), 0.4)
            if len(xs) >= 3:
                gain[j] = np.polyfit(xs, ys, 1)[0]
                print(f"joint {j+1:2d}: {gain[j]*1000:+.1f} deg per 1000 duty  ({len(xs)} pts)")
            else:
                print(f"joint {j+1:2d}: not observed -> default gain kept")
            if abs(gain[j]) < 1e-4:
                gain[j] = DEFAULT_GAIN_DEG_PER_DUTY
        save_calib(np.nan_to_num(zero_all), gain)
    finally:
        link.stop(); link.disarm(); cam.close()


def run_closed_loop(args):
    import cv2
    zero, gain = load_calib()
    cam, det, link = Camera(), make_detector(), HeadLink()
    est = JointEstimator(zero)
    cpg = CPG(N_JOINTS, args.freq, args.amp, args.mode)
    ctrl = Controller(gain)
    log = Logger(args.mode)

    link.arm()
    time.sleep(0.2)
    t0 = t_prev = None
    frame_i = 0
    fps = 0.0
    try:
        while True:
            img, t = cam.read()
            if t0 is None:
                t0 = t_prev = t
            dt, t_prev = max(t - t_prev, 1e-4), t
            fps += (1.0 / dt - fps) * 0.05

            dets = det.detect(img)
            theta, omega = est.update(dets, t)
            cpg.step(dt)
            ref, dref = cpg.reference(advance_s=LATENCY_S)
            duty = ctrl.compute(ref, dref, theta, omega)
            link.send_duty(duty)
            tel = link.poll()
            log.row(t - t0, dt, link.seq, cpg.running, ref, theta, duty, tel)

            frame_i += 1
            if frame_i % 3 == 0:                      # drawing costs time: every 3rd frame
                vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                pts = [p for p in est.points if p is not None]
                for p in pts:
                    cv2.circle(vis, tuple(int(v) for v in p), 6, (0, 255, 0), 2)
                if len(pts) > 1:
                    cv2.polylines(vis, [np.int32(pts)], False, (0, 200, 255), 2)
                status = (f"{fps:5.1f} fps  {cpg.gait} {cpg.freq:.2f}Hz  gait={'ON' if cpg.running else 'off'}  "
                          f"seen={int(np.sum(~np.isnan(theta)))}/{N_JOINTS}")
                if tel:
                    status += f"  {tel[2]:.1f}V {tel[3]:.2f}A  {HeadLink.flag_str(tel[6])}"
                cv2.putText(vis, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow("UNAGI", vis)
                k = cv2.waitKey(1) & 0xFF
                if k in (ord("q"), 27):
                    break
                if k == ord("g"):
                    cpg.running = not cpg.running
                if k == ord(" "):
                    cpg.running = False
                    link.stop(); link.disarm()
                    print("EMERGENCY STOP — press q to quit, or a (re-arm)")
                if k == ord("a"):
                    link.arm()
                if k == ord("z") and not cpg.running:
                    est.zero = est.zero + np.nan_to_num(theta)
                    save_calib(est.zero, ctrl.g)
    except KeyboardInterrupt:
        pass
    finally:
        link.stop(); link.disarm(); log.close(); cam.close()
        cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description="UNAGI laptop controller")
    ap.add_argument("--mode", default="carangiform", choices=list(GAITS))
    ap.add_argument("--freq", type=float, default=1.5, help="tail-beat frequency [Hz]")
    ap.add_argument("--amp", type=float, default=20.0, help="max joint amplitude [deg]")
    ap.add_argument("--open-loop", action="store_true", help="no camera, feedforward only")
    ap.add_argument("--duration", type=float, default=10.0, help="open-loop run time [s]")
    ap.add_argument("--calibrate", action="store_true")
    ap.add_argument("--head", default=None, help="head IP (default from CONFIG)")
    args = ap.parse_args()

    global HEAD_IP
    if args.head:
        HEAD_IP = args.head
    if args.calibrate:
        run_calibrate(args)
    elif args.open_loop:
        run_open_loop(args)
    else:
        run_closed_loop(args)


if __name__ == "__main__":
    main()
