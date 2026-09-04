"""
make_report.py
==============

Runs the whole study and writes everything a first design review needs:

  outputs/fig1_midline_fit.png     biological midline vs 11-link chain
  outputs/fig2_gait_params.png     amplitude / phase envelope, wave fit
  outputs/fig3_tracking.png        reference vs measured, both architectures
  outputs/fig4_latency_study.png   RMS error vs camera latency
  outputs/fig5_tag_resolution.png  AprilTag angle error vs tag size in pixels
  outputs/gait_params.csv          CPG table: offset / amplitude / phase
  outputs/joint_reference_cycle.csv  64-step lookup table for the ESP32
  outputs/unagi_topview.mp4        synthetic camera view

    python -m simulation.make_report
"""

from __future__ import annotations

import contextlib
import io
import os

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from .midline_to_joints import (extract_joint_series, fit_harmonics,
                                wave_parameters, GaitGenerator, choose_clip)
import glob
from .link_chain import LinkChain, JointPlant
from .virtual_camera import TopViewCamera, TagAngleEstimator
from .run_sim import main as run_main

OUT = "outputs"
CSV = "Midline_*.csv"    # glob: the files overlap, they are read as one
SPECIES = "Anguilla_rostrata"
CLIP = None              # None = pick the cleanest clip automatically
NJ = 10
TAG = 0.025           # black square; 33 mm is the largest that fits
HEAD_MM = 149.85      # snout -> first motor axis
PITCH_MM = 53.57      # motor axis centre-to-centre
TAIL_MM = 50.0        # last motor axis -> tail tip
FOV = 0.90            # tank width covered by the image [m]
LINK_W = 0.045        # drawing hint only

plt.rcParams.update({"figure.dpi": 130, "font.size": 9,
                     "axes.grid": True, "grid.alpha": 0.25,
                     "axes.spines.top": False, "axes.spines.right": False})


def quiet(fn, *a, **kw):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        r = fn(*a, **kw)
    return r


# ----------------------------------------------------------------------
def fig1_midline(series, chain):
    ml = series["midlines"]
    nodes = series["nodes"]
    M = len(ml)
    fig, ax = plt.subplots(2, 1, figsize=(8.2, 5.4), sharex=True)
    cmap = plt.cm.viridis(np.linspace(0, 1, M))

    for i in range(M):
        ax[0].plot(ml[i][:, 0], ml[i][:, 1], color=cmap[i], lw=1.2)
        ax[1].plot(nodes[i][:, 0], nodes[i][:, 1], "-o", color=cmap[i],
                   lw=1.2, ms=2.6)

    ax[0].set_title(f"{SPECIES.replace('_', ' ')} midlines, "
                    f"clip {series['clip_id']} ({M} snapshots)")
    ax[0].set_ylabel("lateral [BL]")
    ax[1].set_title(f"Inextensible {chain.n_links}-link chain fitted by arc "
                    f"length (dots = joints / AprilTags)")
    ax[1].set_ylabel("lateral [BL]")
    ax[1].set_xlabel("along body [BL]")
    for a in ax:
        a.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(f"{OUT}/fig1_midline_fit.png")
    plt.close(fig)


def fig2_gait(series, fit, wave, chain):
    s = wave["joint_s"]
    fig, ax = plt.subplots(1, 3, figsize=(10.5, 3.2))

    ax[0].bar(np.arange(1, NJ + 1), np.degrees(fit["amp"][:, 0]),
              color="#3b6ea5")
    ax[0].set_xlabel("joint (head -> tail)")
    ax[0].set_ylabel("amplitude [deg]")
    ax[0].set_title("Joint amplitude envelope")

    psi = np.degrees(wave["psi"])
    ax[1].plot(s, psi, "o-", color="#b5484a")
    kfit = np.polyfit(s, psi, 1)
    ax[1].plot(s, np.polyval(kfit, s), "--", color="0.4",
               label=f"$\\lambda$ = {wave['wavelength_BL']:.2f} BL")
    ax[1].set_xlabel("joint position [BL]")
    ax[1].set_ylabel("phase [deg]")
    ax[1].set_title("Travelling wave: phase lag")
    ax[1].legend()

    gait = GaitGenerator(series, fit, freq_hz=1.0)
    t, th = gait.as_table(2.0, 0.005)
    vmax = float(np.ceil(np.abs(np.degrees(th)).max() / 5.0) * 5.0)
    im = ax[2].pcolormesh(t, np.arange(1, NJ + 1), np.degrees(th).T,
                          cmap="RdBu_r", shading="nearest",
                          vmin=-vmax, vmax=vmax)
    ax[2].set_xlabel("time [s] (at 1.0 Hz)")
    ax[2].set_ylabel("joint")
    ax[2].set_title("Reference $\\theta_k(t)$ [deg]")
    ax[2].grid(False)
    fig.colorbar(im, ax=ax[2])

    fig.tight_layout()
    fig.savefig(f"{OUT}/fig2_gait_params.png")
    plt.close(fig)


def fig3_tracking(dfs):
    joints = [1, 5, 8, 10]
    fig, ax = plt.subplots(len(joints), 1, figsize=(8.2, 7.0), sharex=True)
    for r, j in enumerate(joints):
        d = dfs["vision"]
        ax[r].plot(d.t, np.degrees(d[f"ref_{j}"]), "k", lw=1.6,
                   label="reference (biology)")
        ax[r].plot(d.t, np.degrees(d[f"true_{j}"]), color="#b5484a", lw=1.1,
                   label="vision-only PID")
        d2 = dfs["cascade"]
        ax[r].plot(d2.t, np.degrees(d2[f"true_{j}"]), color="#3b6ea5", lw=1.1,
                   label="cascade (encoder + camera)")
        ax[r].set_ylabel(f"joint {j}\n[deg]")
    ax[0].legend(ncol=3, fontsize=8, loc="upper right")
    ax[0].set_title("Closed-loop tracking, 1.5 Hz, 25 ms camera latency, "
                    "60 fps")
    ax[-1].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(f"{OUT}/fig3_tracking.png")
    plt.close(fig)


def fig4_latency(lat_ms, res):
    fig, ax = plt.subplots(figsize=(6.0, 3.6))
    ax.plot(lat_ms, res["vision"], "o-", color="#b5484a",
            label="vision-only PID (PC closes the loop)")
    ax.plot(lat_ms, res["cascade"], "s-", color="#3b6ea5",
            label="cascade: AS5600 @ ctrl rate + camera outer loop")
    ax.set_xlabel("camera + WiFi round-trip latency [ms]")
    ax.set_ylabel("RMS joint tracking error [deg]")
    ax.set_title("Latency is the binding constraint, not the PID gains")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(f"{OUT}/fig4_latency_study.png")
    plt.close(fig)


def fig5_tag_resolution(series, chain):
    """How big must a tag be, in pixels, to measure a 2 deg joint angle?"""
    fit = fit_harmonics(series["frames"], series["theta"], 1)
    gait = GaitGenerator(series, fit, freq_hz=1.5)
    fovs = [0.72, 0.80, 0.90, 1.05, 1.30, 1.60]
    px, err, miss = [], [], []
    for fov in fovs:
        cam = TopViewCamera.from_fov(fov, 1920, 1200, fx=1800,
                                     centre_m=(chain.total_length * 0.5, 0.0))
        est = TagAngleEstimator(chain.n_links, TAG)
        e, m = [], 0
        for t in np.linspace(0, 0.66, 12):
            th = gait(t)
            nodes, phi = chain.forward(th)
            img = cam.render(chain, nodes, phi, link_width_m=LINK_W)
            tm, pm, c, seen = est(img)
            m += (chain.n_links - seen.sum())
            if seen.all():
                e.append(np.degrees(np.abs(tm - th)))
        px.append(cam.tag_pixels(TAG))
        err.append(np.mean(e) if e else np.nan)
        miss.append(m)

    fig, ax = plt.subplots(figsize=(6.0, 3.6))
    ax.plot(px, err, "o-", color="#3b6ea5")
    for x, y, m, f in zip(px, err, miss, fovs):
        ax.annotate(f"FOV {f:.2f} m" + (f"\n{m} missed" if m else ""),
                    (x, y), textcoords="offset points", xytext=(6, 6),
                    fontsize=7)
    ax.axhline(2.0, ls="--", color="0.5")
    ax.set_ylim(0, 2.6)
    ax.text(px[-1], 2.08, "anterior joints only move ~2 deg", fontsize=7,
            color="0.35")
    ax.set_xlabel(f"AprilTag size in the image [px]  (physical tag "
                  f"{TAG*1000:.0f} mm)")
    ax.set_ylabel("mean joint-angle error [deg]")
    ax.set_title("Vision resolution vs field of view")
    fig.tight_layout()
    fig.savefig(f"{OUT}/fig5_tag_resolution.png")
    plt.close(fig)
    return px, err


# ----------------------------------------------------------------------
def fig6_ilc():
    """Does the once-per-cycle camera correction STAY converged?"""
    base = (f"--csv {CSV} --species {SPECIES} --clip {CLIP} "
            f"--duration 20 --freq 1.5 "
            f"--tag-size-mm {TAG*1000:.0f} --fov-width {FOV} "
            "--latency-ms 25 --cam-hz 60 --no-vision --arch cascade "
            "--out " + OUT)
    cases = [("--ilc-gain 0.0", "no camera correction", "0.55"),
             ("--ilc-gain 0.5 --ilc-qwin 1 --ilc-leak 1.0",
              "ILC, plain integral update", "#b5484a"),
             ("--ilc-gain 0.5 --ilc-qwin 5 --ilc-leak 0.97",
              "ILC + Q-filter + leak", "#3b6ea5")]
    fig, ax = plt.subplots(figsize=(6.4, 3.8))
    for extra, lab, col in cases:
        d = quiet(run_main, (base + " " + extra).split())[5]
        e = d[[c for c in d.columns if c.startswith("err_")]].to_numpy()
        cyc = (d.t.to_numpy() * 1.5).astype(int)
        r = [np.degrees(np.sqrt((e[cyc == c] ** 2).mean()))
             for c in range(cyc.max())]
        ax.plot(np.arange(len(r)), r, "o-", ms=3, color=col, label=lab)
    ax.set_xlabel("tail-beat cycle")
    ax.set_ylabel("RMS joint error [deg]")
    ax.set_title("Camera correction applied once per cycle")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(f"{OUT}/fig6_ilc_convergence.png")
    plt.close(fig)


def export_tables(series, fit, wave, chain):
    pd.DataFrame({
        "joint": np.arange(1, NJ + 1),
        "s_BL": wave["joint_s"],
        "offset_deg": np.degrees(fit["offset"]),
        "amplitude_deg": np.degrees(fit["amp"][:, 0]),
        "phase_deg": np.degrees(fit["phase"][:, 0]),
        "r2": fit["r2"],
    }).to_csv(f"{OUT}/gait_params.csv", index=False)

    # one normalised cycle, 64 steps -> straight into ESP32 flash
    gait = GaitGenerator(series, fit, freq_hz=1.0)
    ph = np.arange(64) / 64.0
    tbl = np.array([gait(p) for p in ph])
    df = pd.DataFrame(np.degrees(tbl),
                      columns=[f"joint_{k+1}_deg" for k in range(NJ)])
    df.insert(0, "phase", ph)
    df.to_csv(f"{OUT}/joint_reference_cycle.csv", index=False)

    with open(f"{OUT}/gait_table.h", "w") as f:
        f.write("// UNAGI joint reference, one tail-beat cycle, 64 steps\n"
                "// generated by simulation/make_report.py\n"
                f"// source: {SPECIES}, clip {CLIP}\n"
                f"#define UNAGI_N_JOINTS {NJ}\n"
                "#define UNAGI_N_PHASE   64\n"
                "const float unagi_gait_deg[UNAGI_N_PHASE]"
                "[UNAGI_N_JOINTS] = {\n")
        for row in np.degrees(tbl):
            f.write("  {" + ", ".join(f"{v:7.3f}" for v in row) + "},\n")
        f.write("};\n")


def make_video(series, fit, chain, seconds=3.0, fps=60):
    gait = GaitGenerator(series, fit, freq_hz=1.5)
    cam = TopViewCamera.from_fov(FOV, 1920, 1200, fx=1800,
                                 centre_m=(chain.total_length * 0.5, 0.0))
    est = TagAngleEstimator(chain.n_links, TAG)
    path = f"{OUT}/unagi_topview.mp4"
    vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps,
                         (1920, 1200))
    for i in range(int(seconds * fps)):
        t = i / fps
        th = gait(t)
        nodes, phi = chain.forward(th)
        img = cam.render(chain, nodes, phi, link_width_m=LINK_W)
        tm, pm, c, seen = est(img)
        cv2.putText(img, f"t={t:5.2f}s   tags {int(seen.sum())}/"
                         f"{chain.n_links}   max angle err "
                         f"{np.degrees(np.abs(tm-th)).max():.2f} deg",
                    (20, 44), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (20, 20, 20), 2, cv2.LINE_AA)
        for k, (cc, ss) in enumerate(zip(c, seen)):
            if ss:
                cv2.putText(img, str(k), tuple((cc + [26, -26]).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 90, 200), 2,
                            cv2.LINE_AA)
        vw.write(img)
    vw.release()
    return path


# ----------------------------------------------------------------------
def run(csv=None, clip=None, out=None, species=None):
    global CSV, CLIP, OUT, SPECIES
    CSV = csv or CSV
    CLIP = clip if clip is not None else CLIP
    OUT = out or OUT
    SPECIES = species or SPECIES
    files = sorted(glob.glob(CSV)) or [CSV]
    os.makedirs(OUT, exist_ok=True)
    chain = LinkChain.from_motor_spacing(
        HEAD_MM / 1000.0, PITCH_MM / 1000.0, NJ, TAIL_MM / 1000.0,
        tag_size_m=TAG)
    if CLIP is None:
        CLIP, tab = choose_clip(files, chain.fractions, species=SPECIES)
        tab.to_csv(f"{OUT}/clip_selection.csv", index=False)
    series = extract_joint_series(files, CLIP, chain.fractions,
                                  species=SPECIES)
    fit = fit_harmonics(series["frames"], series["theta"], 1)
    wave = wave_parameters(fit, series["link_fractions"])
    print(chain)
    sticker, fits = chain.tag_fits_on_link()
    print(f"tag sticker {sticker.max()*1000:.0f} mm, shortest link "
          f"{chain.L.min()*1000:.1f} mm -> "
          f"{'fits' if fits.all() else 'DOES NOT FIT'}")
    print("tag centres from snout [mm]:", chain.tag_positions_mm().round(1))
    print("largest tag per link [mm]:", (chain.max_tag_size()*1000).round(1))

    print("== gait extracted from biology ==")
    print(f"clip {series['clip_id']}, {len(series['frames'])} snapshots, "
          f"period {fit['period_frames']:.1f} video frames")
    print(f"body wavelength {wave['wavelength_BL']:.2f} BL, "
          f"head->tail phase lag {wave['total_lag_deg']:.0f} deg")

    fig1_midline(series, chain)
    fig2_gait(series, fit, wave, chain)
    export_tables(series, fit, wave, chain)

    # --- closed loop, real rendering + real AprilTag detection --------
    base = (f"--csv {CSV} --species {SPECIES} --clip {CLIP} "
            f"--duration 4 --freq 1.5 "
            f"--tag-size-mm {TAG*1000:.0f} --fov-width {FOV} "
            "--latency-ms 25 --cam-hz 60 --out " + OUT)
    dfs = {}
    for arch in ["vision", "cascade"]:
        out = quiet(run_main, (base + f" --arch {arch}").split())
        dfs[arch] = out[5]
        e = dfs[arch]
        e = e[e.t > 1.0][[c for c in e.columns if c.startswith("err_")]]
        print(f"[{arch:7s}] vision-in-the-loop RMS = "
              f"{np.degrees(np.sqrt((e.to_numpy()**2).mean())):.2f} deg")
    fig3_tracking(dfs)

    # --- latency sweep (fast surrogate) -------------------------------
    lat = [0, 5, 10, 20, 30, 45, 60, 80]
    res = {"vision": [], "cascade": []}
    for arch in res:
        for L in lat:
            out = quiet(run_main,
                        (base + f" --arch {arch} --no-vision "
                                f"--latency-ms {L}").split())
            d = out[5]
            d = d[d.t > 1.0][[c for c in d.columns if c.startswith("err_")]]
            res[arch].append(
                np.degrees(np.sqrt((d.to_numpy() ** 2).mean())))
    fig4_latency(lat, res)
    pd.DataFrame({"latency_ms": lat, **res}).to_csv(
        f"{OUT}/latency_study.csv", index=False)

    fig6_ilc()
    px, err = fig5_tag_resolution(series, chain)
    print("tag px:", np.round(px, 1), " angle err deg:", np.round(err, 2))

    vid = make_video(series, fit, chain)
    print("video:", vid)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=CSV, help="path or glob")
    ap.add_argument("--species", default=SPECIES)
    ap.add_argument("--clip", type=int, default=None,
                    help="clip id; omitted = choose automatically")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()
    run(a.csv, a.clip, a.out, a.species)
