"""
compare_species.py
==================

Same robot, six swimming modes.

Runs the extraction pipeline over every midline CSV in a folder and asks
the only question that matters for hardware: **which of these gaits can
this mechanism actually reproduce?**

    python -m simulation.compare_species --freq 1.5

Writes outputs/species_comparison.csv and outputs/fig7_species.png.
"""

from __future__ import annotations

import argparse
import glob
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from .midline_to_joints import (extract_joint_series, fit_harmonics,
                                wave_parameters, choose_clip, list_species,
                                norm_species)
from .link_chain import LinkChain, JointPlant

plt.rcParams.update({"figure.dpi": 130, "font.size": 9,
                     "axes.grid": True, "grid.alpha": 0.25,
                     "axes.spines.top": False, "axes.spines.right": False})


def pretty(name):
    return name.replace("_", " ")


def analyse(files, sp, chain, plant, freq_hz, joint_limit_deg):
    clip, tab = choose_clip(files, chain.fractions, verbose=False,
                            species=sp)
    s = extract_joint_series(files, clip, chain.fractions, species=sp)
    f = fit_harmonics(s["frames"], s["theta"], 1)
    w = wave_parameters(f, s["link_fractions"])

    amp = f["amp"][:, 0]
    peak = np.abs(f["offset"]) + amp             # worst-case excursion
    tau = plant.torque_required(amp, freq_hz)

    return dict(
        species=pretty(sp), clip=clip,
        n_clips=len(tab),
        period_frames=f["period_frames"],
        wavelength_BL=w["wavelength_BL"],
        lag_deg=w["total_lag_deg"],
        max_amp_deg=np.degrees(amp.max()),
        max_peak_deg=np.degrees(peak.max()),
        over_limit=int((np.degrees(peak) > joint_limit_deg).sum()),
        max_torque_mNm=tau.max() * 1000,
        stall_mNm=plant.Kt[0] * 1000,
        motor_limited=int((tau > plant.Kt).sum()),
        median_r2=float(np.median(f["r2"])),
    ), amp, f, w


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--pattern", default="Midline_*.csv",
                    help="the published files are NOT one species each; "
                         "they are read as one de-duplicated dataset")
    ap.add_argument("--species", nargs="*", default=None,
                    help="species to compare; omitted = the ones with the "
                         "most clips")
    ap.add_argument("--top", type=int, default=6)
    ap.add_argument("--list-species", action="store_true")
    ap.add_argument("--freq", type=float, default=1.5)
    ap.add_argument("--joints", type=int, default=10)
    ap.add_argument("--head-length-mm", type=float, default=149.85)
    ap.add_argument("--motor-pitch-mm", type=float, default=53.57)
    ap.add_argument("--tail-length-mm", type=float, default=50.0)
    ap.add_argument("--joint-limit", type=float, default=45.0)
    ap.add_argument("--stall-torque", type=float, default=0.045)
    ap.add_argument("--out", default="outputs")
    a = ap.parse_args(argv)

    os.makedirs(a.out, exist_ok=True)
    chain = LinkChain.from_motor_spacing(
        a.head_length_mm / 1000, a.motor_pitch_mm / 1000, a.joints,
        a.tail_length_mm / 1000)
    plant = JointPlant.from_chain(chain, stall_torque=a.stall_torque,
                                  limit_deg=a.joint_limit)

    files = sorted(glob.glob(a.pattern))
    if not files:
        raise SystemExit(f"no files match {a.pattern}")

    catalog = list_species(files)
    if a.list_species:
        print(catalog.to_string(index=False))
        return catalog

    if a.species:
        wanted = [norm_species(x) for x in a.species]
    else:
        wanted = catalog["species"].head(a.top).tolist()
    print(f"{len(catalog)} species available across {len(files)} files; "
          f"comparing {len(wanted)}\n")

    rows, amps, fits, waves, names = [], [], [], [], []
    for sp in wanted:
        try:
            r, amp, f, w = analyse(files, sp, chain, plant, a.freq,
                                   a.joint_limit)
        except Exception as e:
            print(f"[skip] {pretty(sp)}: {type(e).__name__}: {e}")
            continue
        rows.append(r)
        amps.append(np.degrees(amp))
        fits.append(f)
        waves.append(w)
        names.append(r["species"])
        print(f"{r['species']:26s} clip {r['clip']:4d}  "
              f"lambda {r['wavelength_BL']:4.2f} BL  "
              f"lag {r['lag_deg']:4.0f} deg  "
              f"peak {r['max_peak_deg']:5.1f} deg  "
              f"tau {r['max_torque_mNm']:5.0f} mN.m")

    df = pd.DataFrame(rows)
    df.to_csv(f"{a.out}/species_comparison.csv", index=False)

    # ---- figure ------------------------------------------------------
    fig, ax = plt.subplots(1, 3, figsize=(11.5, 3.6))
    cmap = plt.cm.turbo(np.linspace(0.08, 0.92, len(names)))
    j = np.arange(1, a.joints + 1)

    for i, n in enumerate(names):
        ax[0].plot(j, amps[i], "o-", ms=3.5, color=cmap[i], label=n)
    ax[0].axhline(a.joint_limit, ls="--", color="0.4", lw=1)
    ax[0].text(1, a.joint_limit * 1.02, f"joint limit {a.joint_limit:.0f} deg",
               fontsize=7, color="0.35")
    ax[0].set_xlabel("joint (head -> tail)")
    ax[0].set_ylabel("amplitude [deg]")
    ax[0].set_title("Joint amplitude envelope")
    ax[0].legend(fontsize=6.5)

    for i in range(len(names)):
        ax[1].plot(waves[i]["joint_s"], np.degrees(waves[i]["psi"]),
                   "o-", ms=3.5, color=cmap[i])
    ax[1].set_xlabel("joint position [BL]")
    ax[1].set_ylabel("phase [deg]")
    ax[1].set_title("Travelling wave")

    tau = df["max_torque_mNm"].to_numpy()
    ax[2].barh(np.arange(len(names)), tau, color=cmap)
    ax[2].axvline(df["stall_mNm"].iloc[0], ls="--", color="0.3")
    ax[2].text(df["stall_mNm"].iloc[0], -0.7,
               f" stall {df['stall_mNm'].iloc[0]:.0f}", fontsize=7,
               color="0.3")
    ax[2].set_yticks(np.arange(len(names)))
    ax[2].set_yticklabels(names, fontsize=7)
    ax[2].set_xlabel(f"peak joint torque at {a.freq} Hz [mN.m]")
    ax[2].set_title("Can the motors do it?")

    fig.tight_layout()
    fig.savefig(f"{a.out}/fig7_species.png")
    plt.close(fig)
    print(f"\nwrote {a.out}/species_comparison.csv and "
          f"{a.out}/fig7_species.png")
    return df


if __name__ == "__main__":
    main()
