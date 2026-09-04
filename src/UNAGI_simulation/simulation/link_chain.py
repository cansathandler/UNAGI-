"""
link_chain.py
=============

A plain planar serial chain with an AprilTag on every link.  No fish.

This replaces the earlier `fish_model.py`, which carried a tapered hull
and derived the joint loads from an assumed fish morphology (a Gaussian
width profile fitted by eye to a tuna).  None of that was measurable, so
none of it belonged in the model.

Everything here is either a dimension you take with a caliper or a mass
you take with a scale:

    link_lengths_m    length of each rigid link, snout to tail
    segment_mass_kg   mass of each link, in air
    segment_width_m   lateral projected width of each link
    tag_size_m        printed size of the black AprilTag square

The biology enters in exactly one place and one place only: the joint
angle *reference* produced by `midline_to_joints.py`.  The robot model
knows nothing about fish.

Units are SI throughout: metres, kilograms, radians, seconds.
"""

from __future__ import annotations

import numpy as np


# ----------------------------------------------------------------------
class LinkChain:
    """n_links rigid links, n_links-1 revolute joints, one tag per link.

        link 0  = head module          (tag id 0)
        link k  = body segment k       (tag id k)
        joint k = motor between link k-1 and link k,  k = 1..N
    """

    def __init__(self, link_lengths_m, tag_size_m=0.018):
        self.L = np.asarray(link_lengths_m, dtype=float)
        if self.L.ndim != 1 or self.L.size < 2:
            raise ValueError("link_lengths_m must be a 1-D list of >= 2 links")
        self.n_links = self.L.size
        self.n_joints = self.n_links - 1
        self.total_length = float(self.L.sum())
        self.fractions = self.L / self.total_length     # for the midline fit
        self.tag_size = float(tag_size_m)

    # ------------------------------------------------------------------
    @classmethod
    def uniform(cls, n_joints=10, head_length_m=0.088,
                segment_length_m=0.031, tag_size_m=0.018):
        """Convenience: one head link plus n_joints identical segments."""
        return cls([head_length_m] + [segment_length_m] * n_joints,
                   tag_size_m=tag_size_m)

    def __repr__(self):
        return (f"LinkChain({self.n_links} links, {self.n_joints} joints, "
                f"total {self.total_length*1000:.0f} mm, "
                f"tag {self.tag_size*1000:.0f} mm)")

    # ------------------------------------------------------------------
    def forward(self, theta, base_pos=(0.0, 0.0), base_heading=0.0):
        """Joint angles -> node positions and absolute link headings.

        theta : (n_joints,) relative angles [rad]
        returns nodes (n_links+1, 2) [m], phi (n_links,) [rad]
        """
        theta = np.asarray(theta, dtype=float)
        phi = base_heading + np.concatenate([[0.0], np.cumsum(theta)])
        nodes = np.zeros((self.n_links + 1, 2))
        nodes[0] = base_pos
        for k in range(self.n_links):
            nodes[k + 1] = nodes[k] + self.L[k] * np.array(
                [np.cos(phi[k]), np.sin(phi[k])])
        return nodes, phi

    def tag_poses(self, nodes, phi):
        """Centre and heading of the tag glued to the top of each link.

        The tag sits at the link midpoint and shares the link's heading,
        so a detected tag orientation IS the link orientation, and the
        difference between two neighbours IS the joint angle.
        """
        centres = 0.5 * (nodes[:-1] + nodes[1:])
        return centres, phi

    def joint_arclength(self):
        """Distance from the snout to each joint [m] (for plotting)."""
        return np.cumsum(self.L)[:-1]

    def tag_fits_on_link(self, quiet_modules=2):
        """The printed white square is larger than the black tag.

        36h11 is 8 modules across, so with a 2-module quiet zone on each
        side the sticker is 12/8 = 1.5x the nominal tag size.  Returns the
        required sticker size and a per-link boolean.
        """
        sticker = self.tag_size * (8 + 2 * quiet_modules) / 8.0
        return sticker, self.L >= sticker


# ----------------------------------------------------------------------
class JointPlant:
    """Per-joint second-order dynamics:

        J_k * ddtheta = Kt_k*u - c_k*dtheta - k_k*theta
                        - ch_k*dtheta*|dtheta|

    Every coefficient is either measured on the bench or computed from a
    measured dimension.  Nothing is inferred from an assumed body shape.

    Integrated with sub-steps: the quadratic drag term is stiff and
    explicit Euler at the control rate diverges.
    """

    def __init__(self, n_joints, J, Kt, c, k, ch,
                 limit_deg=45.0, vel_limit_dps=720.0, substeps=20):
        ones = np.ones(n_joints)
        self.n = n_joints
        self.J = np.asarray(J, float) * ones
        self.Kt = np.asarray(Kt, float) * ones
        self.c = np.asarray(c, float) * ones
        self.k = np.asarray(k, float) * ones
        self.ch = np.asarray(ch, float) * ones
        self.lim = np.radians(limit_deg)
        self.vlim = np.radians(vel_limit_dps)
        self.substeps = int(substeps)
        self.reset()

    # ------------------------------------------------------------------
    @classmethod
    def from_chain(cls, chain, segment_mass_kg=0.045, segment_width_m=0.040,
                   stall_torque=0.045, viscous=8e-4,
                   flexure_stiffness=0.030, added_mass_factor=1.6,
                   drag_coeff=1.1, water_density=1000.0,
                   load_decay_m=0.072, **kw):
        """Build the plant from measurable segment properties.

        segment_mass_kg / segment_width_m
            scalar, or one value per link (length n_links).  Weigh a
            printed segment and measure its lateral face -- that is all
            this needs.

        added_mass_factor
            water entrained by a laterally accelerating slab, ~1.5-2.
            This is what drags the wet resonance below the dry one.

        load_decay_m
            LUMPED APPROXIMATION.  Treating the whole trailing chain as a
            rigid stick about joint k hugely over-estimates the load,
            because every downstream joint is actuated too and relieves
            its neighbour.  Each trailing link is therefore weighted
            exp(-r / load_decay_m).  Identify this on the bench as soon as
            two segments exist; do not trust the default.

        flexure_stiffness
            TPU return spring at joint 1, tapering to 40% at the last
            joint (thinner wall aft).  Measure it: hang a known mass at a
            known lever arm and read the angle.
        """
        n_links = chain.n_links
        n = chain.n_joints
        m = np.asarray(segment_mass_kg, float) * np.ones(n_links)
        w = np.asarray(segment_width_m, float) * np.ones(n_links)
        L = chain.L
        node_s = np.concatenate([[0.0], np.cumsum(L)])

        J = np.zeros(n)
        ch = np.zeros(n)
        k = np.zeros(n)
        for j in range(n):                       # joint j drives links j+1..
            s0 = node_s[j + 1]
            for i in range(j + 1, n_links):
                r = 0.5 * (node_s[i] + node_s[i + 1]) - s0     # lever arm
                decay = np.exp(-r / load_decay_m)
                # slab about the joint: parallel axis + own inertia
                J[j] += decay * added_mass_factor * (
                    m[i] * r ** 2 + m[i] * L[i] ** 2 / 12.0)
                # quadratic drag of the slab sweeping at speed r*dtheta
                area = w[i] * L[i]
                ch[j] += decay * 0.5 * water_density * drag_coeff * \
                    area * r ** 3
            k[j] = flexure_stiffness * (1.0 - 0.6 * j / max(n - 1, 1))

        return cls(n, J, stall_torque, viscous, k, ch, **kw)

    # ------------------------------------------------------------------
    def natural_frequency_hz(self):
        """Wet resonance of each joint -- compare with your drive frequency."""
        return np.sqrt(self.k / self.J) / (2 * np.pi)

    def torque_required(self, amplitude_rad, freq_hz):
        """Peak torque to swing each joint at (amplitude, frequency).

        Compare against Kt to see which joints are motor-limited before
        you order anything.
        """
        w = 2 * np.pi * freq_hz
        a = np.asarray(amplitude_rad, float)
        return (self.J * a * w ** 2 + self.c * a * w
                + self.k * a + self.ch * (a * w) ** 2)

    def reset(self, theta0=None):
        self.theta = (np.zeros(self.n) if theta0 is None
                      else np.array(theta0, float))
        self.dtheta = np.zeros(self.n)

    def step(self, u, dt):
        """u : duty in [-1, 1] per joint."""
        u = np.clip(np.asarray(u, dtype=float), -1.0, 1.0)
        h = dt / self.substeps
        for _ in range(self.substeps):
            tau = (self.Kt * u
                   - self.c * self.dtheta
                   - self.k * self.theta
                   - self.ch * self.dtheta * np.abs(self.dtheta))
            self.dtheta = np.clip(self.dtheta + h * tau / self.J,
                                  -self.vlim, self.vlim)
            self.theta = self.theta + h * self.dtheta
            hit = np.abs(self.theta) > self.lim
            if hit.any():
                self.theta[hit] = np.clip(self.theta[hit], -self.lim, self.lim)
                self.dtheta[hit] = 0.0
        return self.theta.copy()
