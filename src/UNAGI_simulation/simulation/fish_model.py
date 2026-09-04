"""
fish_model.py
=============

Planar (top-view) serial-chain model of the UNAGI robot.

    link 0  = head module   (tag id 0)
    link k  = body segment  (tag id k)
    joint k = motor between link k-1 and link k   (k = 1..N)

Everything is in SI: metres, radians, seconds.

Two levels of fidelity:

  * FishKinematics -- pure geometry, used to generate the ground truth
    pose of every AprilTag from a set of joint angles.

  * JointPlant -- per-joint second-order dynamics
        J*ddtheta = Kt*u - c*dtheta - k_spring*theta - c_h*dtheta*|dtheta|
    representing rotor+link inertia, gearbox/bearing friction, the TPU
    flexure return spring, and quadratic hydrodynamic drag.
    This is what makes the PID gains you find here mean something.
"""

from __future__ import annotations

import numpy as np


# ----------------------------------------------------------------------
class FishKinematics:
    def __init__(self, link_fractions, body_length_m=0.681,
                 tag_size_m=0.017, half_width_frac=0.055):
        self.Lf = np.asarray(link_fractions, dtype=float)
        self.BL = float(body_length_m)
        self.L = self.Lf * self.BL              # link lengths [m]
        self.n_links = len(self.L)
        self.n_joints = self.n_links - 1
        self.tag_size = float(tag_size_m)
        self.half_width = half_width_frac * self.BL

    # ------------------------------------------------------------------
    def forward(self, theta, base_pos=(0.0, 0.0), base_heading=0.0):
        """theta (n_joints,) -> node positions and absolute link headings.

        Returns nodes (n_links+1, 2), phi (n_links,)
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
        """Centre + heading of the AprilTag glued on top of each link."""
        centres = 0.5 * (nodes[:-1] + nodes[1:])
        return centres, phi

    def body_outline(self, nodes, phi, taper=True):
        """Closed polygon for drawing the hull in the synthetic image."""
        s = np.concatenate([[0.0], np.cumsum(self.L)]) / self.BL
        if taper:
            # fat behind the head, thin at the peduncle, flare at the fluke
            w = self.half_width * (0.45 + 1.15 * np.exp(-((s - 0.28) / 0.30) ** 2))
            w = np.maximum(w, 0.12 * self.half_width)
        else:
            w = np.full_like(s, self.half_width)

        phin = np.concatenate([[phi[0]], phi])          # heading at each node
        nx = -np.sin(phin)
        ny = np.cos(phin)
        left = nodes + np.column_stack([nx, ny]) * w[:, None]
        right = nodes - np.column_stack([nx, ny]) * w[:, None]
        return np.vstack([left, right[::-1]])


# ----------------------------------------------------------------------
class JointPlant:
    """Second-order joint dynamics, per joint:

        J_k * ddtheta = Kt_k*u - c_k*dtheta - k_k*theta
                        - ch_k*dtheta*|dtheta|

    J_k   : inertia of everything BEHIND joint k, about joint k
            (so anterior joints are an order of magnitude heavier)
    Kt_k  : stall torque of the geared motor per unit duty
    c_k   : gearbox + bearing viscous friction
    k_k   : TPU flexure return spring
    ch_k  : quadratic hydrodynamic drag of the trailing body

    Integrated with sub-steps: the quadratic drag term is stiff and
    explicit Euler at the control rate blows up (been there).
    """

    def __init__(self, n_joints, J, Kt, c, k, ch,
                 limit_deg=45.0, vel_limit_dps=720.0, substeps=20):
        self.n = n_joints
        self.J = np.asarray(J, float) * np.ones(n_joints)
        self.Kt = np.asarray(Kt, float) * np.ones(n_joints)
        self.c = np.asarray(c, float) * np.ones(n_joints)
        self.k = np.asarray(k, float) * np.ones(n_joints)
        self.ch = np.asarray(ch, float) * np.ones(n_joints)
        self.lim = np.radians(limit_deg)
        self.vlim = np.radians(vel_limit_dps)
        self.substeps = int(substeps)
        self.reset()

    # ------------------------------------------------------------------
    @classmethod
    def from_kinematics(cls, kin, body_mass_kg=0.55, added_mass_factor=1.6,
                        stall_torque=0.045, viscous=8e-4,
                        flexure_stiffness=0.030, drag_coeff=1.1,
                        water_density=1000.0, load_decay_BL=0.18, **kw):
        """Build a physically scaled plant from the link geometry.

        added_mass_factor : water entrained by an accelerating body
                            (~1.5-2 for a slender body in the lateral
                            direction -- this is what pulls the wet
                            resonance well below the dry one)

        load_decay_BL     : LUMPED APPROXIMATION.  Treating the whole
                            trailing body as a rigid stick about joint k
                            hugely over-estimates the load, because in a
                            travelling wave every downstream joint is
                            actuated too and relieves its neighbour.
                            Each trailing segment is therefore weighted
                            exp(-r / (load_decay_BL * BL)).
                            Replace this with a proper multibody + fluid
                            model, or just identify it on the bench, once
                            the first segment exists.
        """
        n = kin.n_joints
        lam = body_mass_kg / kin.BL                  # kg per metre
        L = kin.L
        node_s = np.concatenate([[0.0], np.cumsum(L)])

        J, ch, k = np.zeros(n), np.zeros(n), np.zeros(n)
        for j in range(n):                            # joint j: links j+1..end
            s0 = node_s[j + 1]
            Jj = 0.0
            chj = 0.0
            for m in range(j + 1, kin.n_links):
                Lm = L[m]
                mm = lam * Lm
                r = 0.5 * (node_s[m] + node_s[m + 1]) - s0   # lever arm
                w = np.exp(-r / (load_decay_BL * kin.BL))
                Jj += w * added_mass_factor * (mm * r ** 2
                                               + mm * Lm ** 2 / 12.0)
                # drag of this segment: 0.5*rho*Cd*A*(r*w)^2 acting at r
                A = 2 * kin.half_width * Lm            # projected lateral area
                chj += w * 0.5 * water_density * drag_coeff * A * r ** 3
            J[j] = Jj
            ch[j] = chj
            # flexure gets softer towards the tail (thinner wall)
            k[j] = flexure_stiffness * (1.0 - 0.6 * j / max(n - 1, 1))

        return cls(n, J, stall_torque, viscous, k, ch, **kw)

    # ------------------------------------------------------------------
    def natural_frequency_hz(self):
        return np.sqrt(self.k / self.J) / (2 * np.pi)

    def reset(self, theta0=None):
        self.theta = np.zeros(self.n) if theta0 is None else np.array(theta0,
                                                                     float)
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
