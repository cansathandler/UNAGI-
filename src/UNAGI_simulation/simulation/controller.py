"""
controller.py
=============

Per-joint PID with the details that actually matter on hardware:

  * derivative on MEASUREMENT (no kick when the reference steps)
  * low-pass filtered derivative (AprilTag angle is noisy at ~0.3 deg)
  * conditional-integration anti-windup (duty saturates at +-1)
  * optional feed-forward from the reference velocity -- with a purely
    periodic reference this does most of the work and the PID only has
    to clean up, which is exactly the right split for UNAGI.

Also a LatencyBuffer, because the camera -> PC -> WiFi -> ESP32-C3 path
is the single biggest thing that will destabilise this loop.
"""

from __future__ import annotations

import collections
import numpy as np


class PIDArray:
    def __init__(self, n, kp=2.0, ki=0.5, kd=0.05,
                 d_tau=0.02, out_limit=1.0, i_limit=None, kff=0.0):
        self.n = n
        self.kp = np.asarray(kp, dtype=float) * np.ones(n)
        self.ki = np.asarray(ki, dtype=float) * np.ones(n)
        self.kd = np.asarray(kd, dtype=float) * np.ones(n)
        self.d_tau = float(d_tau)
        self.out_limit = float(out_limit)
        self.i_limit = i_limit
        self.kff = float(kff)
        self.reset()

    def reset(self):
        self.integral = np.zeros(self.n)
        self.prev_meas = np.zeros(self.n)
        self.dfilt = np.zeros(self.n)
        self._first = True

    def __call__(self, ref, meas, dt, ref_rate=None):
        err = ref - meas

        if self._first:
            self.prev_meas = meas.copy()
            self._first = False

        # filtered derivative of the measurement
        raw_d = (meas - self.prev_meas) / max(dt, 1e-9)
        alpha = dt / (self.d_tau + dt)
        self.dfilt += alpha * (raw_d - self.dfilt)
        self.prev_meas = meas.copy()

        p = self.kp * err
        d = -self.kd * self.dfilt
        ff = self.kff * (ref_rate if ref_rate is not None else 0.0)

        unsat = p + self.ki * self.integral + d + ff
        u = np.clip(unsat, -self.out_limit, self.out_limit)

        # conditional integration: stop winding up while saturated and
        # the error would push further into the stop
        can_int = ~((np.abs(unsat) > self.out_limit) & (np.sign(err) == np.sign(unsat)))
        self.integral += np.where(can_int, err * dt, 0.0)
        if self.i_limit is not None:
            self.integral = np.clip(self.integral, -self.i_limit, self.i_limit)

        return u, err


class LatencyBuffer:
    """Delays a signal by a fixed number of control steps."""

    def __init__(self, delay_steps, shape):
        self.n = int(max(0, delay_steps))
        self.buf = collections.deque([np.zeros(shape)] * (self.n + 1),
                                     maxlen=self.n + 1)

    def __call__(self, value):
        self.buf.append(np.asarray(value, dtype=float).copy())
        return self.buf[0].copy()


def computed_torque_gains(plant, bandwidth_hz=8.0, zeta=0.9,
                          integral_ratio=0.2):
    """Per-joint PID gains normalised by the plant.

    Joint 1 carries ~100x the inertia of joint 10, so a single scalar Kp
    cannot possibly suit both.  Scaling by J/Kt turns the tuning knob
    into something physical -- a closed-loop bandwidth in Hz -- and the
    same numbers then transfer to the real robot once you have measured
    J and Kt on the bench.

        u = (J/Kt) * [ wc^2 * e  +  2*zeta*wc * de  +  wi*wc^2 * int(e) ]
    """
    wc = 2 * np.pi * bandwidth_hz
    wi = integral_ratio * wc
    g = plant.J / plant.Kt
    return dict(kp=g * wc ** 2,
                kd=g * 2 * zeta * wc,
                ki=g * wi * wc ** 2)
