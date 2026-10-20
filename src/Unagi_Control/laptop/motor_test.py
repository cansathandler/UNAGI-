#!/usr/bin/env python3
"""
UNAGI motor bench test (in air)
===============================

Same path as the real run:  laptop --WiFi--> head --RS-485--> nodes.
No camera. Lets you drive each motor on its own and checks, from the head's
INA226 current, that each coil really draws current.

  python motor_test.py                  GUI
  python motor_test.py --auto           no GUI: run the automatic check once, print the table
  python motor_test.py --head 127.0.0.1 use with mock_head.py (no hardware)

GUI
---
  ARM / DISARM      motor power switch on the head (must ARM first)
  per motor:  + / -    pulse that motor at +duty / -duty for PULSE_S
              select   include it in the waveform (Start)
              invert   flip its sign (if a motor bends the wrong way)
  Start / Stop      hold / square / sine on all selected motors
  Auto test all     each motor: off -> +duty -> -duty, measures current rise -> PASS/FAIL
  SPACE             stop everything

In air the coils have no water cooling: keep duty moderate and runs short.
"Hold" mode switches itself off after HOLD_MAX_S.
"""
from __future__ import annotations

import argparse
import csv
import math
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

import unagi_control as uc
from unagi_control import HeadLink, N_JOINTS

SEND_HZ = 100             # duty packets per second (head watchdog is 200 ms)
PULSE_S = 0.5             # +/- button pulse length
HOLD_MAX_S = 2.0          # hold mode auto-off (coil heating in air)
DEFAULT_DUTY = 400        # per-mille
PASS_DELTA_A = 0.10       # auto test: current rise needed to count as "coil works"
AUTO_PHASES = (("base", 0, 0.4), ("pos", +1, 0.6), ("neg", -1, 0.6))  # name, sign, seconds
RESULT_DIR = Path(__file__).with_name("logs")


class MotorTester:
    """Owns the send loop (background thread). GUI / CLI only flip its fields."""

    def __init__(self, link: HeadLink):
        self.link = link
        self.lock = threading.Lock()
        self.selected = [False] * N_JOINTS
        self.invert = [False] * N_JOINTS
        self.mode = "square"           # hold | square | sine
        self.duty = DEFAULT_DUTY
        self.freq = 1.0
        self.wave_on = False
        self.wave_t0 = 0.0
        self.pulse = None              # (motor, sign, t_end)
        self.auto = None               # list of phases (motor, name, sign, t_end) still to run
        self.auto_samples = {}         # (motor, name) -> [current samples]
        self.results = {}              # motor -> dict
        self.armed = False
        self.flags = 0
        self.v = self.i = self.p = 0.0
        self.i_hist = deque(maxlen=SEND_HZ * 6)   # last 6 s of current
        self.v_hist = deque(maxlen=SEND_HZ * 6)   # last 6 s of motor-rail voltage
        self.last_tel = 0.0
        self.last_duty = np.zeros(N_JOINTS)
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)

    # ---------------- commands (thread-safe) ----------------
    def start(self):
        self.thread.start()

    def close(self):
        self.stop_all()
        time.sleep(0.05)
        self._stop.set()
        self.thread.join(timeout=1)
        self.link.stop()
        self.link.disarm()

    def arm(self):
        self.link.arm()

    def disarm(self):
        self.stop_all()
        self.link.disarm()

    def stop_all(self):
        with self.lock:
            self.wave_on = False
            self.pulse = None
            self.auto = None
        self.link.stop()

    def start_wave(self):
        with self.lock:
            self.auto, self.pulse = None, None
            self.wave_on, self.wave_t0 = True, time.perf_counter()

    def stop_wave(self):
        with self.lock:
            self.wave_on = False

    def pulse_motor(self, motor: int, sign: int):
        with self.lock:
            self.auto = None
            self.pulse = (motor, sign, time.perf_counter() + PULSE_S)

    def start_auto(self, motors=None):
        motors = range(N_JOINTS) if motors is None else motors
        with self.lock:
            self.wave_on, self.pulse = False, None
            self.auto_samples = {}
            for m in motors:
                self.results.pop(m, None)
            t = time.perf_counter() + 0.3
            phases = []
            for m in motors:
                for name, sign, dur in AUTO_PHASES:
                    t += dur
                    phases.append((m, name, sign, t))
            self.auto = phases

    @property
    def auto_running(self):
        return self.auto is not None

    # ---------------- duty computation ----------------
    def _sgn(self, m):
        return -1.0 if self.invert[m] else 1.0

    def _compute(self, t: float) -> np.ndarray:
        d = np.zeros(N_JOINTS)
        if self.auto is not None:
            while self.auto and t >= self.auto[0][3]:
                self.auto.pop(0)
            if not self.auto:
                self.auto = None
                self._finish_auto()
                return d
            m, name, sign, t_end = self.auto[0]
            d[m] = sign * self.duty * self._sgn(m)
            return d
        if self.pulse is not None:
            m, sign, t_end = self.pulse
            if t >= t_end:
                self.pulse = None
            else:
                d[m] = sign * self.duty * self._sgn(m)
        if self.wave_on:
            tt = t - self.wave_t0
            if self.mode == "hold":
                w = self.duty
                if tt > HOLD_MAX_S:
                    self.wave_on = False
                    w = 0.0
            elif self.mode == "square":
                w = self.duty if math.sin(2 * math.pi * self.freq * tt) >= 0 else -self.duty
            else:
                w = self.duty * math.sin(2 * math.pi * self.freq * tt)
            for m in range(N_JOINTS):
                if self.selected[m]:
                    d[m] += w * self._sgn(m)
        return d

    def _record_auto_sample(self, t: float, current: float):
        if not self.auto:
            return
        m, name, sign, t_end = self.auto[0]
        dur = dict((n, s) for n, _, s in AUTO_PHASES)[name]
        if t > t_end - 0.6 * dur:          # skip the first 40 % (coil + INA226 settling)
            self.auto_samples.setdefault((m, name), []).append(current)

    def _finish_auto(self):
        def avg(m, name):
            s = self.auto_samples.get((m, name), [])
            return float(np.mean(s)) if s else float("nan")
        motors = sorted({m for (m, _) in self.auto_samples})
        for m in motors:
            base, ip, ineg = avg(m, "base"), avg(m, "pos"), avg(m, "neg")
            dp, dn = ip - base, ineg - base
            ok = (dp > PASS_DELTA_A) and (dn > PASS_DELTA_A)
            self.results[m] = dict(base=base, i_pos=ip, i_neg=ineg, d_pos=dp, d_neg=dn,
                                   ok=bool(ok), duty=self.duty)
        self.link.stop()

    # ---------------- send loop ----------------
    def _loop(self):
        period = 1.0 / SEND_HZ
        next_t = time.perf_counter()
        n = 0
        while not self._stop.is_set():
            next_t += period
            t = time.perf_counter()
            with self.lock:
                d = self._compute(t)
                self.last_duty = d
                if self.armed:
                    self.link.send_duty(d)          # zeros when idle: keeps telemetry flowing
                elif n % 10 == 0:
                    self.link.ping()                # 10 Hz while disarmed
            n += 1
            tel = self.link.poll()
            if tel and self.link.rx_count:
                _, _, v, i, p, _, flags = tel
                with self.lock:
                    self.v, self.i, self.p, self.flags = v, i, p, flags
                    self.armed = bool(flags & 1)
                    self.last_tel = t
                    self.i_hist.append(i)
                    self.v_hist.append(v)
                    self._record_auto_sample(t, i)
                    if flags & 8:                   # head tripped overcurrent
                        self.wave_on, self.pulse, self.auto = False, None, None
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.perf_counter()

    # ---------------- results ----------------
    def results_table(self) -> str:
        lines = [f"{'motor':>5} {'base A':>8} {'+duty A':>8} {'-duty A':>8} {'dI+':>7} {'dI-':>7}  result"]
        for m in range(N_JOINTS):
            r = self.results.get(m)
            if r is None:
                lines.append(f"{m+1:>5}   (not tested)")
                continue
            lines.append(f"{m+1:>5} {r['base']:8.3f} {r['i_pos']:8.3f} {r['i_neg']:8.3f} "
                         f"{r['d_pos']:+7.3f} {r['d_neg']:+7.3f}  {'PASS' if r['ok'] else 'FAIL'}")
        return "\n".join(lines)

    def save_results(self) -> Path:
        RESULT_DIR.mkdir(exist_ok=True)
        path = RESULT_DIR / time.strftime("motor_test_%Y%m%d_%H%M%S.csv")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["motor", "duty", "base_A", "i_pos_A", "i_neg_A", "d_pos_A", "d_neg_A", "result"])
            for m in range(N_JOINTS):
                r = self.results.get(m)
                if r:
                    w.writerow([m + 1, r["duty"], f"{r['base']:.4f}", f"{r['i_pos']:.4f}", f"{r['i_neg']:.4f}",
                                f"{r['d_pos']:.4f}", f"{r['d_neg']:.4f}", "PASS" if r["ok"] else "FAIL"])
        return path


# =============================================================================
# CLI auto test
# =============================================================================
def run_cli_auto(tester: MotorTester, duty: int):
    tester.duty = duty
    tester.start()
    print("arming...")
    tester.arm()
    t_end = time.perf_counter() + 2.0
    while not tester.armed and time.perf_counter() < t_end:
        time.sleep(0.05)
    if not tester.armed:
        print("head did not report ARMED (check WiFi connection to UNAGI-HEAD). Abort.")
        tester.close()
        return
    time.sleep(0.3)
    if tester.v < 3.0:
        print(f"ARMED but motor rail is {tester.v:.2f} V: no power after the head's switch. "
              "Check the supply, then PIN_MOTOR_EN / MOTOR_EN_ACTIVE_HIGH (head console: raw 1 / raw 0).")
        tester.close()
        return
    print(f"supply {tester.v:.2f} V, idle {tester.i:.3f} A. Auto test at duty {duty} ...")
    tester.start_auto()
    total = len(tester.auto) if tester.auto else 0
    while tester.auto_running:
        left = len(tester.auto) if tester.auto else 0
        print(f"\r  motor {min(N_JOINTS, (total - left) // len(AUTO_PHASES) + 1):2d}/{N_JOINTS}   "
              f"I = {tester.i:6.3f} A", end="", flush=True)
        time.sleep(0.1)
    print("\n")
    print(tester.results_table())
    print(f"\nsaved {tester.save_results()}")
    tester.close()


# =============================================================================
# GUI
# =============================================================================
def run_gui(tester: MotorTester):
    import tkinter as tk
    from tkinter import ttk

    root = tk.Tk()
    root.title("UNAGI motor test")
    tester.start()

    # ---- top: power + status ----
    top = ttk.Frame(root, padding=6)
    top.grid(row=0, column=0, sticky="ew")
    ttk.Button(top, text="ARM", command=tester.arm).pack(side="left")
    ttk.Button(top, text="DISARM", command=tester.disarm).pack(side="left", padx=4)
    stop_btn = tk.Button(top, text="STOP ALL  (space)", bg="#c62828", fg="white",
                         command=tester.stop_all)
    stop_btn.pack(side="left", padx=8)
    status = ttk.Label(top, text="waiting for head...", font=("Consolas", 10))
    status.pack(side="left", padx=8)

    # ---- settings ----
    st = ttk.LabelFrame(root, text="Drive", padding=6)
    st.grid(row=1, column=0, sticky="ew", padx=6)
    duty_v = tk.IntVar(value=tester.duty)
    freq_v = tk.DoubleVar(value=tester.freq)
    mode_v = tk.StringVar(value=tester.mode)
    ttk.Label(st, text="duty ‰").grid(row=0, column=0)
    ttk.Scale(st, from_=0, to=uc.DUTY_LIMIT, variable=duty_v, length=260,
              command=lambda _: setattr(tester, "duty", int(duty_v.get()))).grid(row=0, column=1)
    duty_lbl = ttk.Label(st, width=5)
    duty_lbl.grid(row=0, column=2)
    ttk.Label(st, text="freq Hz").grid(row=1, column=0)
    ttk.Scale(st, from_=0.2, to=5.0, variable=freq_v, length=260,
              command=lambda _: setattr(tester, "freq", float(freq_v.get()))).grid(row=1, column=1)
    freq_lbl = ttk.Label(st, width=5)
    freq_lbl.grid(row=1, column=2)
    mf = ttk.Frame(st)
    mf.grid(row=0, column=3, rowspan=2, padx=10)
    for m in ("hold", "square", "sine"):
        ttk.Radiobutton(mf, text=m, value=m, variable=mode_v,
                        command=lambda: setattr(tester, "mode", mode_v.get())).pack(anchor="w")
    bf = ttk.Frame(st)
    bf.grid(row=0, column=4, rowspan=2)
    ttk.Button(bf, text="Start (selected)", command=tester.start_wave).pack(fill="x")
    ttk.Button(bf, text="Stop", command=tester.stop_wave).pack(fill="x", pady=2)

    # ---- per motor ----
    mt = ttk.LabelFrame(root, text="Motors", padding=6)
    mt.grid(row=2, column=0, sticky="ew", padx=6, pady=4)
    sel_v = [tk.BooleanVar() for _ in range(N_JOINTS)]
    inv_v = [tk.BooleanVar() for _ in range(N_JOINTS)]
    duty_bars, res_lbls = [], []
    hdr = ["", "pulse", "", "select", "invert", "duty now", "auto test result"]
    for c, h in enumerate(hdr):
        ttk.Label(mt, text=h, foreground="#666").grid(row=0, column=c, padx=3)
    for m in range(N_JOINTS):
        r = m + 1
        ttk.Label(mt, text=f"M{m+1}", width=4, font=("Consolas", 11, "bold")).grid(row=r, column=0)
        ttk.Button(mt, text="+", width=3, command=lambda m=m: tester.pulse_motor(m, +1)).grid(row=r, column=1)
        ttk.Button(mt, text="−", width=3, command=lambda m=m: tester.pulse_motor(m, -1)).grid(row=r, column=2)
        ttk.Checkbutton(mt, variable=sel_v[m],
                        command=lambda m=m: tester.selected.__setitem__(m, sel_v[m].get())).grid(row=r, column=3)
        ttk.Checkbutton(mt, variable=inv_v[m],
                        command=lambda m=m: tester.invert.__setitem__(m, inv_v[m].get())).grid(row=r, column=4)
        bar = tk.Canvas(mt, width=120, height=14, bg="#eee", highlightthickness=0)
        bar.grid(row=r, column=5, padx=4)
        duty_bars.append(bar)
        lbl = ttk.Label(mt, text="—", font=("Consolas", 10), width=34)
        lbl.grid(row=r, column=6, sticky="w")
        res_lbls.append(lbl)

    def set_all(val):
        for m in range(N_JOINTS):
            sel_v[m].set(val)
            tester.selected[m] = val
    af = ttk.Frame(mt)
    af.grid(row=N_JOINTS + 1, column=0, columnspan=7, pady=(6, 0), sticky="w")
    ttk.Button(af, text="select all", command=lambda: set_all(True)).pack(side="left")
    ttk.Button(af, text="select none", command=lambda: set_all(False)).pack(side="left", padx=4)
    ttk.Button(af, text="Auto test all", command=lambda: tester.start_auto()).pack(side="left", padx=12)
    save_lbl = ttk.Label(af, text="")
    ttk.Button(af, text="Save results",
               command=lambda: save_lbl.config(text=f"saved {tester.save_results().name}")).pack(side="left")
    save_lbl.pack(side="left", padx=6)

    # ---- current plot ----
    pf = ttk.LabelFrame(root, text="Head current (blue, left)  /  motor-rail voltage (orange, right)  — last 6 s", padding=4)
    pf.grid(row=3, column=0, sticky="ew", padx=6, pady=(0, 6))
    W, H = 640, 120
    plot = tk.Canvas(pf, width=W, height=H, bg="white", highlightthickness=0)
    plot.pack()

    def refresh():
        with tester.lock:
            age = time.perf_counter() - tester.last_tel
            v, i, p, fl, armed = tester.v, tester.i, tester.p, tester.flags, tester.armed
            hist = list(tester.i_hist)
            vhist = list(tester.v_hist)
            duty_now = tester.last_duty.copy()
            auto = tester.auto_running
        duty_lbl.config(text=str(tester.duty))
        freq_lbl.config(text=f"{tester.freq:.1f}")
        if tester.last_tel == 0 or age > 1.0:
            status.config(text="NO HEAD — connect laptop WiFi to UNAGI-HEAD", foreground="#c62828")
        elif armed and v < 3.0:
            status.config(text=f"ARMED but motor rail {v:.2f} V — no power after the switch "
                               f"(supply off? / PIN_MOTOR_EN pin or polarity?)", foreground="#c62828")
        else:
            status.config(text=f"{'ARMED' if armed else 'disarmed'}   {v:5.2f} V  {i:6.3f} A  {p:5.2f} W   "
                               f"{HeadLink.flag_str(fl)}{'   AUTO TEST…' if auto else ''}",
                          foreground="#2e7d32" if armed else "#555")
        for m in range(N_JOINTS):
            b = duty_bars[m]
            b.delete("all")
            b.create_line(60, 0, 60, 14, fill="#999")
            x = 60 + 60 * duty_now[m] / max(uc.DUTY_LIMIT, 1)
            if duty_now[m] != 0:
                b.create_rectangle(min(60, x), 2, max(60, x), 12,
                                   fill="#1565c0" if duty_now[m] > 0 else "#ef6c00", width=0)
            r = tester.results.get(m)
            if r:
                res_lbls[m].config(text=f"{'PASS' if r['ok'] else 'FAIL'}  dI+ {r['d_pos']:+.3f}  dI- {r['d_neg']:+.3f} A",
                                   foreground="#2e7d32" if r["ok"] else "#c62828")
        plot.delete("all")
        if len(hist) > 1:
            top_a = max(0.5, max(hist) * 1.2)
            pts = []
            for k, a in enumerate(hist):
                pts += [W * k / (tester.i_hist.maxlen - 1), H - H * a / top_a]
            plot.create_line(*pts, fill="#1565c0")
            plot.create_text(4, 4, anchor="nw", text=f"{top_a:.2f} A", fill="#666")
            plot.create_text(4, H - 4, anchor="sw", text="0", fill="#666")
        if len(vhist) > 1:                       # voltage on its own scale (right axis, orange)
            vmax = max(1.0, max(vhist) * 1.1)
            pts = []
            for k, a in enumerate(vhist):
                pts += [W * k / (tester.v_hist.maxlen - 1), H - H * a / vmax]
            plot.create_line(*pts, fill="#ef6c00")
            plot.create_text(W - 4, 4, anchor="ne", text=f"{vmax:.1f} V", fill="#ef6c00")
            plot.create_text(W - 4, 18, anchor="ne", text=f"min {min(vhist):.2f} V", fill="#ef6c00")
        root.after(100, refresh)

    def on_close():
        tester.close()
        root.destroy()

    root.bind("<space>", lambda e: tester.stop_all())
    root.protocol("WM_DELETE_WINDOW", on_close)
    refresh()
    root.mainloop()


def main():
    ap = argparse.ArgumentParser(description="UNAGI motor bench test")
    ap.add_argument("--head", default=None, help="head IP (default from unagi_control CONFIG)")
    ap.add_argument("--auto", action="store_true", help="no GUI: run automatic check once")
    ap.add_argument("--duty", type=int, default=DEFAULT_DUTY, help="test duty, per-mille")
    args = ap.parse_args()
    if args.head:
        uc.HEAD_IP = args.head
    tester = MotorTester(HeadLink())
    tester.duty = max(0, min(args.duty, uc.DUTY_LIMIT))
    if args.auto:
        run_cli_auto(tester, tester.duty)
    else:
        run_gui(tester)


if __name__ == "__main__":
    main()
