"""
midline_to_joints.py
====================

Biological midline data  ->  UNAGI joint-angle reference trajectory.

Input
-----
Di Santo et al. (PNAS 2021) style midline CSV.
One row = one point on the body.  Columns used:

    clip_id   : which video clip
    midline   : index of the snapshot within the clip (1..M)
    frame     : video frame number of that snapshot
    location  : point index along the body (1..200, head -> tail)
    X, Y      : body-axis-aligned coordinates in body lengths (BL)
                X = along the mean swimming axis, Y = lateral

Pipeline
--------
1.  For each snapshot, resample the 200-point midline into an
    N_JOINT+1 link serial chain by ARC LENGTH (inextensible body).
2.  Link heading  phi_k = atan2(dy, dx)   (absolute)
    Joint angle   theta_k = phi_k - phi_{k-1}   (relative == what a motor does)
    Relative angles are invariant to the global heading of the fish,
    so head recoil / turning is automatically removed.
3.  Fit a shared-frequency harmonic model over the clip:

        theta_k(t) = c_k + A_k * sin(omega * t + psi_k)  [+ 2nd harmonic]

    A_k    -> amplitude envelope along the body
    psi_k  -> phase lag  ->  travelling wave speed / body wavelength
4.  Expose a GaitGenerator that can replay this at ANY frequency,
    amplitude scale, or with a spline instead of harmonics.

Author: for Ryo Kanno / UNAGI, Laboratory of Sustainability Robotics
"""

from __future__ import annotations

import os

import numpy as np
import pandas as pd


# ----------------------------------------------------------------------
# 1. Loading
# ----------------------------------------------------------------------
_READ_CACHE: dict = {}
_DATASET_CACHE: dict = {}
_WARNED: set = set()
_CACHE_MAX = 8


def norm_species(name):
    """'Thunnus albacares' and 'Thunnus_albacares' are the same fish.

    The dataset uses both spellings, sometimes inside one file.
    """
    return str(name).strip().replace(" ", "_")


def _read(csv_path):
    """Read the CSV and drop everything that cannot be used.

    Real digitised midline files are messy:
      * clip_id / midline / location arrive as floats, not ints
      * trailing or interleaved all-NaN rows (the mackerel file has 26132)
      * a text column mixed into a numeric one (pandas DtypeWarning)
      * midlines with different point counts within the same clip
    All of that is handled here so nothing downstream has to care.

    Results are cached on (path, mtime, size).  `choose_clip` scans every
    clip in the file, and the eel file has 135 clips over 300k rows --
    re-parsing it once per clip turns a 3 s scan into several minutes.
    """
    try:
        st = os.stat(csv_path)
        key = (os.path.abspath(csv_path), st.st_mtime_ns, st.st_size)
    except OSError:
        key = (os.path.abspath(csv_path), None, None)
    if key in _READ_CACHE:
        return _READ_CACHE[key]

    df = pd.read_csv(csv_path, low_memory=False)

    need = ["clip_id", "midline", "frame", "location", "X", "Y"]
    missing = [c for c in need if c not in df.columns]
    if missing:
        raise ValueError(f"{csv_path} is missing columns {missing}. "
                         f"found: {list(df.columns)}")

    for c in need:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    n0 = len(df)
    df = df.dropna(subset=need)
    if len(df) < n0 and key not in _WARNED:
        _WARNED.add(key)
        print(f"[load] dropped {n0 - len(df)} unusable rows from "
              f"{os.path.basename(csv_path)}")

    for c in ["clip_id", "midline", "frame", "location"]:
        df[c] = df[c].astype(int)

    if "species" in df.columns:
        df["species"] = df["species"].map(norm_species)

    while len(_READ_CACHE) >= _CACHE_MAX:
        _READ_CACHE.pop(next(iter(_READ_CACHE)))
    _READ_CACHE[key] = df
    return df


def load_dataset(csv_paths, species=None):
    """Read one or more CSVs as a single de-duplicated dataset.

    IMPORTANT: the published files are NOT one species each.  Despite the
    names, `Midline_Anguilla_rostrata.csv` holds 44 species and overlaps
    heavily with the others -- they are alphabetical suffixes of one
    dataset.  Selecting by filename silently gives you the wrong fish.
    Always select with `species=`.
    """
    if isinstance(csv_paths, str):
        csv_paths = [csv_paths]
    csv_paths = list(csv_paths)
    ckey = (tuple(csv_paths), None if species is None
            else norm_species(species))
    if ckey in _DATASET_CACHE:
        return _DATASET_CACHE[ckey]
    parts = [_read(p) for p in csv_paths]
    df = pd.concat(parts, ignore_index=True) if len(parts) > 1 else parts[0]
    if len(parts) > 1:
        df = df.drop_duplicates(subset=["species", "clip_id", "midline",
                                        "location"])
    if species is not None:
        want = norm_species(species)
        got = df[df["species"] == want]
        if len(got) == 0:
            avail = sorted(df["species"].dropna().unique().tolist())
            raise ValueError(f"species '{species}' not found. "
                             f"available: {avail}")
        df = got
    while len(_DATASET_CACHE) >= _CACHE_MAX:
        _DATASET_CACHE.pop(next(iter(_DATASET_CACHE)))
    _DATASET_CACHE[ckey] = df
    return df


def list_species(csv_paths):
    """Which fish are actually in these files, and how many clips each."""
    df = load_dataset(csv_paths)
    return (df.groupby("species")
              .agg(clips=("clip_id", "nunique"), rows=("X", "size"))
              .sort_values("clips", ascending=False)
              .reset_index())


def load_clip(csv_path, clip_id=None, min_points: int = 20,
              species=None):
    """Return (frames, midlines, clip_id) for one clip.

    frames   : (M,)   video frame number of each snapshot
    midlines : list of M arrays, each (P_i, 2), head -> tail, in body
               lengths.  A LIST, not an array: snapshots in the same clip
               often have different point counts, and every consumer
               resamples them onto the robot's links anyway.
    """
    df = load_dataset(csv_path, species=species)

    if clip_id is None:
        clip_id = int(df.groupby("clip_id")["midline"].nunique().idxmax())

    sub = df[df["clip_id"] == int(clip_id)]
    if len(sub) == 0:
        raise ValueError(
            f"clip_id {clip_id} not found"
            + (f" for species '{species}'" if species else "")
            + f". available: {sorted(df.clip_id.unique().tolist())}")

    frames, midlines = [], []
    for _, g in sub.groupby("midline"):
        g = g.sort_values("location")
        if len(g) < min_points:
            continue
        frames.append(int(g["frame"].iloc[0]))
        midlines.append(g[["X", "Y"]].to_numpy(dtype=float))

    if len(frames) < 4:
        raise ValueError(f"clip {clip_id} has only {len(frames)} usable "
                         f"snapshots (need >= 4)")

    order = np.argsort(frames)
    frames = np.asarray(frames)[order]
    midlines = [midlines[i] for i in order]
    return frames, midlines, int(clip_id)


def list_clips(csv_path: str) -> pd.DataFrame:
    df = _read(csv_path)
    return (df.groupby("clip_id")
              .agg(n_snapshots=("midline", "nunique"),
                   frame_span=("frame", lambda s: s.max() - s.min()),
                   n_points=("location", "max"))
              .reset_index())


# ----------------------------------------------------------------------
# 2. Midline -> serial chain
# ----------------------------------------------------------------------
def resample_chain(midline: np.ndarray, link_fractions: np.ndarray):
    """Fit an inextensible serial chain to one midline snapshot.

    Nodes are placed at fixed fractions of the TOTAL ARC LENGTH,
    so every link keeps a constant length -- exactly like the robot.

    Returns nodes (n_links+1, 2) in body lengths.
    """
    seg = np.diff(midline, axis=0)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))])
    total = s[-1]

    s_nodes = np.concatenate([[0.0], np.cumsum(link_fractions)]) * total
    s_nodes = np.clip(s_nodes, 0.0, total)

    x = np.interp(s_nodes, s, midline[:, 0])
    y = np.interp(s_nodes, s, midline[:, 1])
    nodes = np.column_stack([x, y]) / total          # normalise to 1 BL
    return nodes


def nodes_to_angles(nodes: np.ndarray):
    """nodes (n_links+1, 2) -> (headings, joint_angles) in radians.

    headings     : (n_links,)  absolute link angle  (what an AprilTag measures)
    joint_angles : (n_links-1,) relative angle      (what a motor commands)
    """
    d = np.diff(nodes, axis=0)
    phi = np.arctan2(d[:, 1], d[:, 0])
    phi = np.unwrap(phi)
    theta = np.diff(phi)
    return phi, theta


def extract_joint_series(csv_path, clip_id=None, link_fractions=None,
                         species=None):
    """Full pipeline: CSV -> theta(frame) for every joint.

    link_fractions : link lengths as fractions of the total, summing to 1.
                     Pass `chain.fractions` from your LinkChain -- the fit
                     follows the ROBOT's geometry, not an assumed fish
                     morphology.

    Returns dict with frames (M,), theta (M, n_joints), nodes (M, n_links+1, 2)
    """
    frames, midlines, clip_id = load_clip(csv_path, clip_id,
                                          species=species)
    if link_fractions is None:
        raise ValueError("link_fractions is required; pass chain.fractions")
    fr = np.asarray(link_fractions, dtype=float)
    fr = fr / fr.sum()

    nodes_all, theta_all, phi_all = [], [], []
    for ml in midlines:
        nodes = resample_chain(ml, fr)
        phi, theta = nodes_to_angles(nodes)
        nodes_all.append(nodes)
        theta_all.append(theta)
        phi_all.append(phi)

    return dict(
        clip_id=clip_id,
        species=species,
        frames=frames,
        midlines=midlines,   # list of (P_i, 2)
        nodes=np.array(nodes_all),
        headings=np.array(phi_all),
        theta=np.array(theta_all),                  # (M, n_joints)
        link_fractions=fr,
    )


# ----------------------------------------------------------------------
# 3. Shared-frequency harmonic fit
# ----------------------------------------------------------------------
def _design(t, omega, n_harm):
    cols = [np.ones_like(t)]
    for h in range(1, n_harm + 1):
        cols += [np.cos(h * omega * t), np.sin(h * omega * t)]
    return np.column_stack(cols)


def fit_harmonics(frames, theta, n_harm=1,
                  period_range=None, n_grid=800):
    """Fit theta_k(t) = c_k + sum_h A_kh sin(h*omega*t + psi_kh)
    with ONE omega shared by all joints (a single tail-beat frequency).

    frames : (M,)          in video frames
    theta  : (M, K)        joint angles [rad]

    period_range : (min, max) in video frames.  Default is derived from
                   the clip's own frame span, so this works on any
                   species / camera frame rate without hand-tuning.
                   Override it if a clip covers more than ~4 tail beats.

    Returns dict: period_frames, omega, offset (K,), amp (K,n_harm),
                  phase (K,n_harm), r2 (K,), coeffs
    """
    t = frames.astype(float)
    theta = np.asarray(theta, dtype=float)

    if period_range is None:
        span = float(t.max() - t.min())
        period_range = (max(4.0, 0.22 * span), 1.6 * span)

    periods = np.linspace(*period_range, n_grid)
    best = (np.inf, None, None)
    for T in periods:
        w = 2 * np.pi / T
        A = _design(t, w, n_harm)
        coef, *_ = np.linalg.lstsq(A, theta, rcond=None)
        res = float(np.sum((theta - A @ coef) ** 2))
        if res < best[0]:
            best = (res, T, coef)

    _, T, coef = best
    omega = 2 * np.pi / T
    A = _design(t, omega, n_harm)
    fit = A @ coef

    ss_res = np.sum((theta - fit) ** 2, axis=0)
    ss_tot = np.sum((theta - theta.mean(axis=0)) ** 2, axis=0)
    r2 = 1.0 - ss_res / np.maximum(ss_tot, 1e-12)

    K = theta.shape[1]
    offset = coef[0]
    amp = np.zeros((K, n_harm))
    phase = np.zeros((K, n_harm))
    for h in range(n_harm):
        a = coef[1 + 2 * h]      # cos coefficient
        b = coef[2 + 2 * h]      # sin coefficient
        amp[:, h] = np.hypot(a, b)
        phase[:, h] = np.arctan2(a, b)     # a*cos+b*sin = A*sin(x+psi)

    return dict(period_frames=T, omega=omega, offset=offset,
                amp=amp, phase=phase, r2=r2, coeffs=coef, n_harm=n_harm)


def wave_parameters(fit, link_fractions):
    """Phase lag along the body -> body wavelength and wave speed."""
    psi = np.unwrap(fit["phase"][:, 0])
    # arc-length position of each joint, in BL, measured from the snout
    s = np.cumsum(link_fractions)[:-1]
    # linear fit psi = psi0 - k*s   (k = spatial wavenumber, rad / BL)
    Amat = np.column_stack([np.ones_like(s), s])
    coef, *_ = np.linalg.lstsq(Amat, psi, rcond=None)
    k = -coef[1]
    lam = 2 * np.pi / k if abs(k) > 1e-9 else np.inf
    return dict(joint_s=s, psi=psi, wavenumber=k, wavelength_BL=lam,
                total_lag_deg=np.degrees(psi[0] - psi[-1]))


# ----------------------------------------------------------------------
# 4. Reference generator used by the controller
# ----------------------------------------------------------------------
class GaitGenerator:
    """Continuous, periodic joint-angle reference in real time.

    mode = 'harmonic' : analytic, smooth, CPG-ready
    mode = 'spline'   : periodic interpolation through the measured data
                        (more faithful, keeps 2nd-order body shape)
    """

    def __init__(self, series, fit, freq_hz=1.5, amp_scale=1.0,
                 mode="harmonic", clip_deg=45.0):
        self.series = series
        self.fit = fit
        self.freq_hz = float(freq_hz)
        self.amp_scale = float(amp_scale)
        self.mode = mode
        self.clip = np.radians(clip_deg)
        self.n_joints = series["theta"].shape[1]

        if mode == "spline":
            # normalised cycle phase of each measured snapshot
            t = series["frames"].astype(float)
            ph = ((t - t[0]) * fit["omega"]) % (2 * np.pi)
            order = np.argsort(ph)
            ph = ph[order]
            th = series["theta"][order]
            # wrap for periodicity
            self._ph = np.concatenate([ph - 2 * np.pi, ph, ph + 2 * np.pi])
            self._th = np.vstack([th, th, th])

    def __call__(self, t_sec: float) -> np.ndarray:
        """Joint angle reference [rad], shape (n_joints,)."""
        phase = 2 * np.pi * self.freq_hz * t_sec

        if self.mode == "harmonic":
            f = self.fit
            th = f["offset"].copy()
            for h in range(f["n_harm"]):
                th = th + f["amp"][:, h] * np.sin((h + 1) * phase
                                                  + f["phase"][:, h])
        else:
            ph = phase % (2 * np.pi)
            th = np.array([np.interp(ph, self._ph, self._th[:, k])
                           for k in range(self.n_joints)])

        th = f_center(th, self.fit["offset"], self.amp_scale)
        return np.clip(th, -self.clip, self.clip)

    def as_table(self, duration_s, dt):
        t = np.arange(0.0, duration_s, dt)
        return t, np.array([self(ti) for ti in t])


def f_center(theta, offset, scale):
    """Scale the oscillation about its own DC offset."""
    return offset + (theta - offset) * scale


# ----------------------------------------------------------------------
# 5. Species-agnostic clip selection
# ----------------------------------------------------------------------
def choose_clip(csv_path, link_fractions, min_snapshots=8, n_harm=1,
                verbose=True, species=None):
    """Pick the clip that gives the cleanest periodic gait.

    Clip IDs are file-specific, so hard-coding one stops working the
    moment you swap species.  This scores every clip by how well a single
    shared-frequency sinusoid explains its four largest-amplitude joints,
    and returns the winner.

    Score = median R^2 over the four highest-amplitude joints.  Anterior
    joints are excluded from the score on purpose: on a thunniform swimmer
    they move 1-2 deg and their R^2 is dominated by digitisation noise, so
    including them would just rank clips by how still the head was.
    """
    df = load_dataset(csv_path, species=species)
    rows = []
    for cid in sorted(df.clip_id.unique()):
        try:
            s = extract_joint_series(csv_path, int(cid), link_fractions,
                                     species=species)
        except Exception:
            continue
        if len(s["frames"]) < min_snapshots:
            continue
        f = fit_harmonics(s["frames"], s["theta"], n_harm, n_grid=300)
        amp = f["amp"][:, 0]
        top = np.argsort(amp)[-4:]
        span = float(s["frames"].max() - s["frames"].min())
        cycles = span / f["period_frames"]
        rows.append(dict(clip_id=int(cid),
                         n_snapshots=len(s["frames"]),
                         span_frames=int(span),
                         period_frames=f["period_frames"],
                         cycles=cycles,
                         per_cycle=len(s["frames"]) / max(cycles, 1e-6),
                         max_amp_deg=np.degrees(amp.max()),
                         score=float(np.median(f["r2"][top]))))
    if not rows:
        raise ValueError(f"no clip for species={species} has "
                         f">= {min_snapshots} snapshots")
    # tie-break on amplitude: when two clips fit equally well, prefer the
    # one with more signal in it
    tab = pd.DataFrame(rows)
    # Guard against aliasing: with a free period and ~10 samples, a clip
    # that covers only a fraction of a cycle can fit anything.  Require
    # roughly a full cycle, adequately sampled.
    ok = (tab.cycles > 0.7) & (tab.per_cycle >= 6)
    if ok.any():
        tab = tab[ok]
    tab = tab.sort_values(["score", "max_amp_deg"], ascending=False)
    if verbose:
        print(tab.to_string(index=False,
                            float_format=lambda v: f"{v:.2f}"))
    best = int(tab.iloc[0]["clip_id"])
    if verbose:
        print(f"-> using clip {best}")
    return best, tab
