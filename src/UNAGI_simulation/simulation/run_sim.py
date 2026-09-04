"""
run_sim.py
==========

Closed-loop hardware-in-the-nothing simulation of UNAGI.

    biological midline CSV
            |
            v
    GaitGenerator  ->  theta_ref(t)
            |
            v            +------------------------------+
         PIDArray  --u-->|  JointPlant (motor+water)     |
            ^            +------------------------------+
            |                        | theta_true
            |                        v
            |               TopViewCamera  (renders real AprilTags)
            |                        | image
            |                        v
            +---- theta_meas <-- TagAngleEstimator  (+ camera latency)

Nothing in the feedback path uses the simulator's internal state: the
controller only ever sees detected tag corners, exactly like the real
rig.  Swap TopViewCamera+TagAngleEstimator for the Basler + your
april_tracker.py and the rest of the code is unchanged.

Usage
-----
    python -m simulation.run_sim --csv Midline_Thunnus_albacares.csv \
        --clip 254 --joints 10 --freq 1.5 --duration 6 --video
"""

from __future__ import annotations

import argparse
import os

import cv2
import numpy as np
import pandas as pd

from .midline_to_joints import (extract_joint_series, fit_harmonics,
                                wave_parameters, GaitGenerator, list_clips)
from .fish_model import FishKinematics, JointPlant
from .virtual_camera import TopViewCamera, TagAngleEstimator
from .controller import PIDArray, LatencyBuffer, computed_torque_gains


def build(args):
    series = extract_joint_series(args.csv, args.clip, args.joints,
                                  args.head_fraction)
    fit = fit_harmonics(series["frames"], series["theta"],
                        n_harm=args.harmonics)
    wave = wave_parameters(fit, series["link_fractions"])
    gait = GaitGenerator(series, fit, freq_hz=args.freq,
                         amp_scale=args.amp_scale, mode=args.gait_mode,
                         clip_deg=args.joint_limit)
    kin = FishKinematics(series["link_fractions"],
                         body_length_m=args.body_length,
                         tag_size_m=args.tag_size)
    return series, fit, wave, gait, kin


def run(args):
    series, fit, wave, gait, kin = build(args)
    os.makedirs(args.out, exist_ok=True)

    n_j = kin.n_joints
    plant = JointPlant.from_kinematics(
        kin, body_mass_kg=args.body_mass, stall_torque=args.stall_torque,
        flexure_stiffness=args.flexure, limit_deg=args.joint_limit)
    plant.reset(gait(0.0))
    print("[plant] J [kg m^2] =", np.array2string(plant.J, precision=6))
    print("[plant] wet natural freq [Hz] =",
          np.array2string(plant.natural_frequency_hz(), precision=2))

    if args.bandwidth > 0:
        g = computed_torque_gains(plant, args.bandwidth, args.zeta)
        pid = PIDArray(n_j, i_limit=args.i_limit, **g)
        print(f"[gains] bandwidth = {args.bandwidth} Hz -> "
              f"Kp = {np.array2string(g['kp'], precision=2)}")
    else:
        pid = PIDArray(n_j, kp=args.kp, ki=args.ki, kd=args.kd,
                       kff=args.kff, i_limit=args.i_limit)

    cam = TopViewCamera.from_fov(args.fov_width, width=args.img_w,
                                 height=args.img_h, fx=args.fx,
                                 centre_m=(args.body_length * 0.50, 0.0),
                                 noise_sigma=args.noise, blur_px=args.blur)
    est = TagAngleEstimator(kin.n_links, tag_size_m=args.tag_size)
    print(f"[camera] {cam.metres_per_pixel()*1000:.3f} mm/px, "
          f"tag = {cam.tag_pixels(args.tag_size):.0f} px, "
          f"height = {cam.h:.2f} m")
    if cam.tag_pixels(args.tag_size) < 30:
        print("  !! tag < 30 px -- 36h11 detection will be unreliable")

    dt = 1.0 / args.ctrl_hz
    cam_every = max(1, int(round(args.ctrl_hz / args.cam_hz)))
    lat = LatencyBuffer(int(round(args.latency_ms * 1e-3 * args.ctrl_hz)), (n_j,))

    # --- optional inner loop running ON the XIAO ESP32-C3 -------------
    # AS5600 gives 12 bit over 360 deg -> 0.088 deg quantisation, and the
    # ESP32 can close this loop at 1 kHz with ~1 ms of delay.  The camera
    # then only has to correct slow shape errors.
    inner = None
    if args.arch == "cascade":
        gi = computed_torque_gains(plant, args.inner_bandwidth, args.zeta)
        inner = PIDArray(n_j, i_limit=args.i_limit, **gi)
        enc_lat = LatencyBuffer(int(round(0.001 * args.ctrl_hz)), (n_j,))
        # phase-indexed iterative learning correction.  The gait is
        # strictly periodic, so the camera does not have to close a fast
        # loop at all: it only has to say "last cycle, at phase 0.37,
        # joint 8 was 3 deg short".  Because the correction is indexed by
        # the phase at which the measurement was TAKEN, the 25 ms of
        # camera + WiFi delay stops mattering.
        NB = args.ilc_bins
        ilc_corr = np.zeros((NB, n_j))
        ilc_sum = np.zeros((NB, n_j))
        ilc_cnt = np.zeros(NB)
        cycle_idx = 0
        lat_s = args.latency_ms * 1e-3
        outer_corr = np.zeros(n_j)

    n_steps = int(args.duration * args.ctrl_hz)
    log = []
    frames_out = []
    theta_meas = gait(0.0)

    writer = None
    if args.video:
        path = os.path.join(args.out, "unagi_topview.mp4")
        writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"),
                                 args.cam_hz, (args.img_w, args.img_h))

    for i in range(n_steps):
        t = i * dt
        ref = gait(t)
        ref_rate = (gait(t + 1e-4) - gait(t - 1e-4)) / 2e-4

        # ---- camera / vision at its own rate -------------------------
        if i % cam_every == 0:
            if args.no_vision:
                # fast surrogate: skip rendering, inject equivalent noise
                theta_meas_raw = plant.theta + np.random.normal(
                    0.0, np.radians(args.meas_noise_deg), n_j)
            else:
                nodes, phi = kin.forward(plant.theta)
                img = cam.render(kin, nodes, phi)
                th_m, phi_m, centres, seen = est(img)
                theta_meas_raw = th_m
                if writer is not None:
                    writer.write(draw_hud(img.copy(), t, ref,
                                          plant.theta, th_m, seen))
                if args.save_frames and i % (cam_every * args.save_every) == 0:
                    frames_out.append((t, img.copy()))
        theta_meas = lat(theta_meas_raw)

        # ---- control -------------------------------------------------
        if inner is None:
            u, err = pid(ref, theta_meas, dt, ref_rate=ref_rate)
        else:
            # ---- outer loop on the PC: ILC, updated once per cycle ----
            if i % cam_every == 0 and t > lat_s:
                t_meas = t - lat_s                 # when the photo was taken
                b = int((t_meas * args.freq % 1.0) * args.ilc_bins)
                ilc_sum[b] += gait(t_meas) - theta_meas
                ilc_cnt[b] += 1
            c_now = int(t * args.freq)
            if c_now > cycle_idx:
                cycle_idx = c_now
                m = ilc_cnt > 0
                ilc_corr[m] += args.ilc_gain * (ilc_sum[m]
                                                / ilc_cnt[m][:, None])
                # Q-filter: a pure integral update along the phase axis is
                # only marginally stable -- it keeps learning the tag
                # measurement noise and slowly diverges after ~10 cycles.
                # Circular low-pass across phase bins + a small leak keeps
                # it converged indefinitely.  (This is the standard ILC
                # Q-filter; without it the demo looks fine for 5 cycles
                # and then falls apart, which is exactly the failure mode
                # you do NOT want to discover in the water.)
                if args.ilc_qwin > 1:
                    w = args.ilc_qwin
                    ker = np.ones(w) / w
                    pad = np.vstack([ilc_corr[-w:], ilc_corr, ilc_corr[:w]])
                    sm = np.apply_along_axis(
                        lambda c: np.convolve(c, ker, mode="same"), 0, pad)
                    ilc_corr = sm[w:w + args.ilc_bins]
                ilc_corr *= args.ilc_leak
                ilc_corr = np.clip(ilc_corr, -np.radians(20), np.radians(20))
                ilc_sum[:] = 0.0
                ilc_cnt[:] = 0.0
            outer_corr = ilc_corr[int((t * args.freq % 1.0) * args.ilc_bins)]

            # ---- inner loop on the ESP32-C3: AS5600 encoder -----------
            enc = enc_lat(np.round(plant.theta / np.radians(0.088))
                          * np.radians(0.088))
            u, err = inner(ref + outer_corr, enc, dt)
            err = ref - theta_meas          # log the vision-side error

        if args.ff == "model":
            ref_acc = (gait(t + 1e-3) - 2 * ref + gait(t - 1e-3)) / 1e-6
            u = u + (plant.J * ref_acc + plant.c * ref_rate
                     + plant.k * ref) / plant.Kt
        theta_true = plant.step(u, dt)

        log.append(np.concatenate([[t], ref, theta_true, theta_meas, u, err]))

    if writer is not None:
        writer.release()

    cols = (["t"]
            + [f"ref_{k+1}" for k in range(n_j)]
            + [f"true_{k+1}" for k in range(n_j)]
            + [f"meas_{k+1}" for k in range(n_j)]
            + [f"u_{k+1}" for k in range(n_j)]
            + [f"err_{k+1}" for k in range(n_j)])
    df = pd.DataFrame(log, columns=cols)
    df.to_csv(os.path.join(args.out, "sim_log.csv"), index=False)

    return series, fit, wave, gait, kin, df, frames_out


def draw_hud(img, t, ref, true, meas, seen):
    txt = [f"t = {t:6.3f} s",
           f"RMS(ref-meas) = {np.degrees(np.sqrt(np.mean((ref-meas)**2))):5.2f} deg",
           f"tags seen: {int(seen.sum())}/{len(seen)}"]
    for i, s in enumerate(txt):
        cv2.putText(img, s, (14, 30 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (20, 20, 20), 2, cv2.LINE_AA)
    return img


# ----------------------------------------------------------------------
def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--csv", default="Midline_Thunnus_albacares.csv")
    p.add_argument("--clip", type=int, default=254)
    p.add_argument("--joints", type=int, default=10)
    p.add_argument("--head-fraction", type=float, default=0.22)
    p.add_argument("--harmonics", type=int, default=1)
    p.add_argument("--gait-mode", default="harmonic",
                   choices=["harmonic", "spline"])

    p.add_argument("--freq", type=float, default=1.5, help="tail beat [Hz]")
    p.add_argument("--amp-scale", type=float, default=1.0)
    p.add_argument("--joint-limit", type=float, default=45.0)
    p.add_argument("--body-length", type=float, default=0.40)
    p.add_argument("--tag-size", type=float, default=0.018)

    p.add_argument("--arch", default="vision", choices=["vision", "cascade"],
                   help="vision = PID directly on the camera measurement; "
                        "cascade = fast AS5600 loop on the ESP32-C3 + slow "
                        "camera correction on the PC")
    p.add_argument("--inner-bandwidth", type=float, default=12.0)
    p.add_argument("--ilc-gain", type=float, default=0.5,
                   help="iterative learning gain applied once per tail beat")
    p.add_argument("--ilc-bins", type=int, default=32)
    p.add_argument("--ilc-qwin", type=int, default=5,
                   help="ILC Q-filter width in phase bins (1 = off)")
    p.add_argument("--ilc-leak", type=float, default=0.97)

    p.add_argument("--bandwidth", type=float, default=8.0,
                   help="closed-loop bandwidth [Hz]; 0 = use raw kp/ki/kd")
    p.add_argument("--zeta", type=float, default=0.9)

    p.add_argument("--body-mass", type=float, default=0.55)
    p.add_argument("--stall-torque", type=float, default=0.045)
    p.add_argument("--flexure", type=float, default=0.030)

    p.add_argument("--kp", type=float, default=6.0)
    p.add_argument("--ki", type=float, default=4.0)
    p.add_argument("--kd", type=float, default=0.08)
    p.add_argument("--kff", type=float, default=0.0)
    p.add_argument("--i-limit", type=float, default=0.6)

    p.add_argument("--ctrl-hz", type=float, default=200.0)
    p.add_argument("--cam-hz", type=float, default=60.0)
    p.add_argument("--latency-ms", type=float, default=25.0)
    p.add_argument("--duration", type=float, default=6.0)

    p.add_argument("--img-w", type=int, default=1920)
    p.add_argument("--img-h", type=int, default=1200)
    p.add_argument("--fx", type=float, default=1800.0)
    p.add_argument("--fov-width", type=float, default=0.60,
                   help="tank width covered by the image [m]")
    p.add_argument("--noise", type=float, default=0.0)
    p.add_argument("--blur", type=int, default=0)

    p.add_argument("--ff", default="none", choices=["none", "model"],
                   help="model = inverse-dynamics feed-forward")
    p.add_argument("--no-vision", action="store_true",
                   help="skip rendering; inject equivalent angle noise (fast)")
    p.add_argument("--meas-noise-deg", type=float, default=0.5,
                   help="1-sigma joint-angle noise used by --no-vision")
    p.add_argument("--out", default="outputs")
    p.add_argument("--video", action="store_true")
    p.add_argument("--save-frames", action="store_true")
    p.add_argument("--save-every", type=int, default=10)
    args = p.parse_args(argv)

    series, fit, wave, gait, kin, df, frames = run(args)

    print(f"clip {series['clip_id']}  snapshots={len(series['frames'])}")
    print(f"tail-beat period  = {fit['period_frames']:.1f} video frames")
    print(f"body wavelength   = {wave['wavelength_BL']:.2f} BL")
    print(f"head->tail lag    = {wave['total_lag_deg']:.0f} deg")
    print("joint | amp[deg] | phase[deg] | R^2")
    for k in range(kin.n_joints):
        print(f"  {k+1:2d}  |  {np.degrees(fit['amp'][k,0]):6.2f}  |"
              f"  {np.degrees(fit['phase'][k,0]):7.1f}   | {fit['r2'][k]:.3f}")
    e = df[[c for c in df.columns if c.startswith("err_")]].to_numpy()
    print(f"tracking RMS error = {np.degrees(np.sqrt((e**2).mean())):.2f} deg")
    return series, fit, wave, gait, kin, df


if __name__ == "__main__":
    main()
