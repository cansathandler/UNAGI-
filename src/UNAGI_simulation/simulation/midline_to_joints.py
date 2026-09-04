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

import numpy as np
import pandas as pd


# ----------------------------------------------------------------------
# 1. Loading
# ----------------------------------------------------------------------
def load_clip(csv_path: str, clip_id: int | None = None):
    """Return (frames, midlines) for one clip.

    frames   : (M,)      video frame number of each snapshot
    midlines : (M, P, 2) XY coordinates in body lengths, head -> tail
    """
    df = pd.read_csv(csv_path)

    if clip_id is None:
        # default: the clip with the most snapshots
        clip_id = int(df.groupby("clip_id")["midline"].nunique().idxmax())

    sub = df[df["clip_id"] == clip_id]
    if len(sub) == 0:
        raise ValueError(f"clip_id {clip_id} not found. "
                         f"available: {sorted(df.clip_id.unique())}")

    frames, midlines = [], []
    for _, g in sub.groupby("midline"):
        g = g.sort_values("location")
        frames.append(int(g["frame"].iloc[0]))
        midlines.append(g[["X", "Y"]].to_numpy(dtype=float))

    order = np.argsort(frames)
    frames = np.asarray(frames)[order]
    midlines = np.asarray(midlines)[order]
    return frames, midlines, clip_id


def list_clips(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    return (df.groupby("clip_id")
              .agg(species=("species", "first"),
                   n_snapshots=("midline", "nunique"),
                   frame_span=("frame", lambda s: s.max() - s.min()),
                   n_points=("location", "max"))
              .reset_index())


# ----------------------------------------------------------------------
# 2. Midline -> serial chain
# ----------------------------------------------------------------------
def default_link_fractions(n_joints: int = 10, head_fraction: float = 0.22):
    """Head link is longer (it holds the electronics); rest is uniform."""
    body = (1.0 - head_fraction) / n_joints
    return np.array([head_fraction] + [body] * n_joints)


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


def extract_joint_series(csv_path, clip_id=None, n_joints=10,
                         head_fraction=0.22):
    """Full pipeline: CSV -> theta(frame) for every joint.

    Returns dict with frames (M,), theta (M, n_joints), nodes (M, n_joints+2, 2)
    """
    frames, midlines, clip_id = load_clip(csv_path, clip_id)
    fr = default_link_fractions(n_joints, head_fraction)

    nodes_all, theta_all, phi_all = [], [], []
    for ml in midlines:
        nodes = resample_chain(ml, fr)
        phi, theta = nodes_to_angles(nodes)
        nodes_all.append(nodes)
        theta_all.append(theta)
        phi_all.append(phi)

    return dict(
        clip_id=clip_id,
        frames=frames,
        midlines=midlines,
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
                  period_range=(12.0, 90.0), n_grid=800):
    """Fit theta_k(t) = c_k + sum_h A_kh sin(h*omega*t + psi_kh)
    with ONE omega shared by all joints (a single tail-beat frequency).

    frames : (M,)          in video frames
    theta  : (M, K)        joint angles [rad]

    Returns dict: period_frames, omega, offset (K,), amp (K,n_harm),
                  phase (K,n_harm), r2 (K,), coeffs
    """
    t = frames.astype(float)
    theta = np.asarray(theta, dtype=float)

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
