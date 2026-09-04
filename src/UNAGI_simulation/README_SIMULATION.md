# UNAGI virtual robot — closed-loop AprilTag control simulator

A virtual 10-motor robotic fish that carries real AprilTags, is watched by a
synthetic top-view camera, and is driven by a joint-angle reference extracted
from *Thunnus albacares* midline data.

The point is **not** to tune PID gains. It is to settle the control
architecture before any hardware exists.

---

## Install

```bash
pip install numpy pandas scipy matplotlib opencv-contrib-python
```

Drop `simulation/` next to `src/` in the `UNAGI-` repo, keep
`Midline_Thunnus_albacares.csv` at the repo root.

## Run

```bash
python -m simulation.make_report                        # everything
python -m simulation.run_sim --arch cascade --video     # one run + mp4
python -m simulation.run_sim --arch vision --no-vision --latency-ms 40
```

`--no-vision` skips rendering and injects equivalent angle noise — ~200x
faster, use it for parameter sweeps.

---

## What is where

| file | role |
|---|---|
| `midline_to_joints.py` | CSV → arc-length link fit → joint angles → shared-ω harmonic fit → `GaitGenerator` |
| `fish_model.py` | 11-link planar kinematics, tag frames, per-joint plant (inertia, added mass, quadratic drag, TPU flexure) |
| `virtual_camera.py` | pinhole top-view renderer with real 36h11 bitmaps; `TagAngleEstimator` turns detections into joint angles |
| `controller.py` | PID array, latency buffer, `computed_torque_gains` |
| `run_sim.py` | closed loop, two architectures |
| `make_report.py` | all figures, CSV/header exports, video |

**Nothing in the feedback path reads simulator state.** The loop goes
render → `cv2.aruco.detectMarkers` → tag corners → joint angles. Swap
`TopViewCamera` for the Basler and `TagAngleEstimator` for
`src/Detection_Pipeline/Tracker/april_tracker.py` and the controller is
unchanged.

---

## Results (clip 254, 10 joints, 1.5 Hz, 400 mm body)

**Gait extracted from biology**

| joint | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
|---|---|---|---|---|---|---|---|---|---|---|
| amplitude [deg] | 2.3 | 1.3 | 1.2 | 1.7 | 4.4 | 8.0 | 8.5 | 4.0 | 14.7 | 21.9 |
| phase [deg] | 85 | 165 | 155 | 100 | 81 | 79 | 70 | 20 | −56 | −53 |

Body wavelength **1.31 BL**, head→tail lag **137°** — consistent with
published thunniform kinematics.

Anterior R² is only 0.4–0.7, but that is because those joints move 1–2°;
the fit is not the problem, the signal-to-noise is.

**Latency, not gains, is the binding constraint**

| camera latency | vision-only PID | cascade + ILC |
|---|---|---|
| 0 ms | 2.9° | 1.3° |
| 10 ms | 6.4° | 1.6° |
| 25 ms | 11.5° | **2.1°** |
| 50 ms | 18.0° | 4.8° |

A grid over Kp ∈ [1,150], Ki, Kd changes vision-only RMS by less than 2°.
At 25 ms the loop enters a ~4 Hz limit cycle and the tail swings 45°
against a 22° reference. No gain fixes it.

**Recommended architecture**

- *inner*, on each XIAO ESP32-C3: AS5600 position loop at the control rate
  (0.088° quantisation, ~1 ms delay).
- *outer*, on the PC: phase-indexed **iterative learning control**, updated
  **once per tail beat**, not per frame. The correction is indexed by the
  phase at which the photo was taken, so the 25 ms round trip stops
  mattering.

**ILC needs a Q-filter.** A plain integral update converges to 2.0° by
cycle 4 and then diverges to 10° by cycle 28 — it learns the tag noise.
Circular low-pass across phase bins (`--ilc-qwin 5`) plus a leak
(`--ilc-leak 0.97`) holds 1.9–2.0° indefinitely. See
`fig6_ilc_convergence.png`.

**Camera sizing** (18 mm tag, 1920×1200)

| FOV width | tag [px] | mean angle error |
|---|---|---|
| 0.45 m | 77 | 0.21° |
| 0.60 m | 58 | 0.32° |
| 0.80 m | 43 | 0.45° |
| 1.05 m | 33 | 0.87° |
| 1.40 m | 25 | 1.05° |

Anterior joints move ~2°, so beyond ~1 m FOV the measurement noise exceeds
the signal there. Either stay under 0.6 m, or accept that the front of the
body is encoder-only.

Geometric constraint: at 400 mm body length with a 22% head, each segment is
**31 mm**, and an 18 mm tag plus quiet zone is **25 mm** square. Ten tags is
geometrically tight — consider tagging alternate segments and interpolating.

---

## Exports for firmware

- `gait_params.csv` — offset / amplitude / phase per joint (CPG table)
- `joint_reference_cycle.csv` — 64 phase steps × 10 joints, degrees
- `gait_table.h` — the same as a C array, ready for the ESP32

---

## Traps already hit here (they will recur on hardware)

1. **A mirrored tag is invisible.** World Y is up, image V is down; if the
   corner order flips, the detector finds 19 quads and decodes zero. Same
   thing happens on a real rig if a tag is applied from behind or viewed
   through a reflective surface.
2. **Quadratic drag is numerically stiff.** Explicit Euler at the control
   rate blows up; the plant integrates with 20 sub-steps.
3. **A scalar Kp cannot work.** Joint 1 carries ~100× the inertia of joint
   10. Gains are normalised by J/Kt so the single knob is a closed-loop
   bandwidth in Hz — measure J and Kt on the bench and the same numbers
   transfer.

---

## Model limitations — read before trusting a number

- The plant lumps trailing inertia with an exponential decay
  (`load_decay_BL`, default 0.18 BL) instead of solving the multibody
  problem. Rigid-stick inertia over-estimates the load badly because every
  downstream joint is actuated too. **Identify this on the bench.**
- No body-fluid coupling, no thrust, no free-swimming: the head is pinned.
  This simulator answers "can the controller reproduce the commanded
  shape", not "how fast does it swim".
- Added-mass factor 1.6 and Cd 1.1 are plausible defaults, not measurements.
- The camera is distortion-free by default and the tags are perfectly flat
  and unoccluded. Real water surface, refraction, and glare are not modelled.

## Next step

Measure the real camera → PC → WiFi → ESP32 round-trip latency (flash an
LED, timestamp both ends). Until that number exists you do not know which
row of the latency table you are on, and the architecture choice has no
evidence behind it.
