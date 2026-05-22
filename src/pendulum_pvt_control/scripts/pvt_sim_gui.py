#!/usr/bin/env python3
"""Offline visualiser for the PVT / MIT-mode (CiA-402 mode 5) impedance control.

  python3 pvt_sim_gui.py            # standalone, no ROS needed
  ros2 run pendulum_pvt_control pvt_sim_gui.py

A self-contained desktop tool that answers "if I command this, what should
happen?". It embeds the same quintic generator as pvt_goto.py, runs a forward
physics simulation of the pendulum under the drive's law

    tau = Kp*(q_d - q) + Kd*(qd_d - qd) + tau_ff

and plots the commanded setpoint against the predicted response so the control
behaviour can be validated before touching the real hardware.

The setpoint is generated at the chosen stream rate; the drive loop is modelled
at 1 kHz with the setpoint zero-order-held between stream updates — exactly the
real signal path, so a low stream rate visibly tracks worse.

No ROS, no hardware: pure numpy + matplotlib + tkinter.
"""

import math
import sys
import tkinter as tk
from tkinter import messagebox, ttk

import numpy as np

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import (  # noqa: E402
    FigureCanvasTkAgg,
    NavigationToolbar2Tk,
)
from matplotlib.figure import Figure  # noqa: E402

# Numeric defaults — gains/feedforward seeded from config/pvt_gains.yaml,
# plant physics from the pendulum_description URDF args, ceiling from the
# 0x6072 max_torque SDO (800 per-mille) x the 0.02693 Nm/per-mille effort factor.
DEFAULTS = {
    "q0": 0.0,         # start position (rad)
    "goal": 1.0,       # goal position (rad)
    "duration": 2.0,   # trajectory duration (s)
    "rate": 200.0,     # setpoint stream rate (Hz)
    "err_max": 0.05,   # pvt_goto hold fix: trajectory-pause lag limit (rad)
    "Kp": 22.95,       # stiffness (Nm/rad)
    "Kd": 2.14,        # damping (Nm.s/rad)
    "tau_limit": 20.0, # symmetric clamp on the feedforward (Nm); <=0 disables
    "mgl": 4.10,       # gravity torque amplitude (Nm)
    "J": 0.102,        # rotating inertia (kg.m^2)
    "Fv": 0.05,        # viscous friction coeff (Nm.s/rad)
    "B": 0.065,        # plant joint damping (Nm.s/rad)
    "friction": 0.33,  # plant Coulomb friction (Nm)
    "eps": 0.05,       # Coulomb friction smoothing velocity (rad/s)
    "ceiling": 21.5,   # hard drive torque ceiling (Nm)
    "block_start": 0.8,  # external-force block engages at this time (s)
    "block_dur": 0.6,    # external-force block holds this long, then releases (s)
}

FLAG_DEFAULTS = {
    "ff_gravity": True,
    "ff_inertia": True,
    "ff_viscous": False,
    "ceiling_enabled": True,
    "block_enabled": False,
}

DT_SIM = 0.001  # drive loop period (s) — the 0x60C2 interpolation time = 1 ms


def quintic_trajectory(q0, goal, duration, rate):
    """Streamed quintic setpoint — identical math to scripts/pvt_goto.py.

    Returns (pos, vel, acc) arrays of length steps+1, one per stream update.
    """
    dq = goal - q0
    dt_stream = 1.0 / rate
    steps = max(1, int(duration * rate))
    i = np.arange(steps + 1)
    t = np.minimum(i * dt_stream, duration)
    tau = t / duration
    s = 10.0 * tau ** 3 - 15.0 * tau ** 4 + 6.0 * tau ** 5
    sd = (30.0 * tau ** 2 - 60.0 * tau ** 3 + 30.0 * tau ** 4) / duration
    sdd = (60.0 * tau - 180.0 * tau ** 2 + 120.0 * tau ** 3) / (duration * duration)
    return q0 + dq * s, dq * sd, dq * sdd


def simulate(p):
    """Forward-simulate the pendulum under the drive's MIT control law.

    The streamed setpoint is zero-order-held; the plant is integrated at 1 kHz
    with semi-implicit (symplectic) Euler. Returns a dict of recorded arrays.
    """
    duration = p["duration"]
    rate = p["rate"]
    dt_stream = 1.0 / rate
    steps = max(1, int(duration * rate))
    sp_pos, sp_vel, sp_acc = quintic_trajectory(p["q0"], p["goal"], duration, rate)

    # External-force block window — pins the joint for [start, start+dur).
    block_on = p["block_enabled"]
    block_start = p["block_start"]
    block_end = block_start + p["block_dur"]

    # pvt_goto hold fix — a following-error governor: when on, the trajectory
    # clock stops advancing (and commands a zero-velocity hold) whenever the
    # joint lags by more than err_max, so the setpoint cannot run away during
    # a stall and there is no excessive catch-up velocity on release.
    mitigate = p.get("mitigate", False)
    err_max = p["err_max"]

    # Run past the trajectory end (and past any block release / governor
    # pause) so the steady-state hold / settling / catch-up stays visible.
    total = duration + max(1.0, 0.5 * duration)
    if block_on:
        total = max(total, block_end + 1.0, duration + p["block_dur"] + 1.0)
    n = int(total / DT_SIM) + 1

    t = np.arange(n) * DT_SIM
    q = np.empty(n)
    qd = np.empty(n)
    q_d = np.empty(n)
    qd_d = np.empty(n)
    qdd_d = np.empty(n)
    tau_kp = np.empty(n)
    tau_kd = np.empty(n)
    tau_ff = np.empty(n)
    tau_ff_raw = np.empty(n)
    tau_drive = np.empty(n)
    ff_clamped = np.zeros(n, dtype=bool)
    ceiling_hit = np.zeros(n, dtype=bool)

    kp, kd = p["Kp"], p["Kd"]
    tau_limit = p["tau_limit"]
    mgl, jj, fv = p["mgl"], p["J"], p["Fv"]
    bb, friction, eps = p["B"], p["friction"], p["eps"]
    ceiling = p["ceiling"]
    ff_g, ff_i, ff_v = p["ff_gravity"], p["ff_inertia"], p["ff_viscous"]
    use_ceiling = p["ceiling_enabled"] and ceiling > 0.0

    state_q = p["q0"]
    state_qd = 0.0
    traj_phase = 0.0        # governed trajectory time (s) — the hold-fix clock
    lost_t = 0.0            # trajectory time given up to the governor slowdown

    for k in range(n):
        # External force pinning the joint for a fixed window: the measured
        # velocity drops to 0 and the position stays frozen, while the drive
        # torque still computes against the growing position error.
        blocking = block_on and block_start <= k * DT_SIM < block_end
        if blocking:
            state_qd = 0.0

        # Zero-order-hold of the streamed quintic, sampled by the governed
        # trajectory clock traj_phase. The +1e-9 nudge absorbs float-division
        # rounding (an exact grid crossing — e.g. rate=1000 against the 1 kHz
        # sim — would otherwise flip int() a step early).
        #
        # pvt_goto hold fix: a following-error governor scales the clock by a
        # smooth speed factor f in [0, 1] — full speed while the joint keeps
        # up, ramping linearly to a standstill as the lag grows from err_max/2
        # to err_max. q_d then cannot run away during a stall and qd_d scales
        # down with it, so the trajectory eases back in on release instead of
        # snapping — and the factor is continuous, so the command never jumps.
        idx = min(int(traj_phase / dt_stream + 1e-9), steps)
        raw_q, raw_v, raw_a = sp_pos[idx], sp_vel[idx], sp_acc[idx]
        if mitigate:
            lag = abs(raw_q - state_q)
            f = (err_max - lag) / (0.5 * err_max)
            f = 0.0 if f < 0.0 else (1.0 if f > 1.0 else f)
        else:
            f = 1.0
        sq, sv, sa = raw_q, f * raw_v, f * raw_a
        traj_phase += f * DT_SIM
        lost_t += (1.0 - f) * DT_SIM

        # Host feedforward — uses live measured q/qd and the held commanded acc.
        g = mgl * math.sin(state_q) if ff_g else 0.0
        ji = jj * sa if ff_i else 0.0
        vi = fv * state_qd if ff_v else 0.0
        ff_raw = g + ji + vi
        if tau_limit > 0.0:
            ff = max(-tau_limit, min(tau_limit, ff_raw))
        else:
            ff = ff_raw

        # Drive MIT law — the Kp/Kd term is NOT bounded by tau_limit.
        pkp = kp * (sq - state_q)
        pkd = kd * (sv - state_qd)
        drive = pkp + pkd + ff
        if use_ceiling:
            clamped = max(-ceiling, min(ceiling, drive))
            if clamped != drive:
                ceiling_hit[k] = True
            drive = clamped

        # Plant: J*qdd = tau_drive - gravity - viscous - Coulomb friction.
        coulomb = friction * math.tanh(state_qd / eps)
        qdd = (drive - mgl * math.sin(state_q) - bb * state_qd - coulomb) / jj

        q[k] = state_q
        qd[k] = state_qd
        q_d[k] = sq
        qd_d[k] = sv
        qdd_d[k] = sa
        tau_kp[k] = pkp
        tau_kd[k] = pkd
        tau_ff[k] = ff
        tau_ff_raw[k] = ff_raw
        tau_drive[k] = drive
        ff_clamped[k] = ff != ff_raw

        # Semi-implicit (symplectic) Euler — skipped while the joint is pinned.
        if not blocking:
            state_qd = state_qd + qdd * DT_SIM
            state_q = state_q + state_qd * DT_SIM

    return {
        "t": t, "q": q, "qd": qd,
        "q_d": q_d, "qd_d": qd_d, "qdd_d": qdd_d,
        "tau_kp": tau_kp, "tau_kd": tau_kd,
        "tau_ff": tau_ff, "tau_ff_raw": tau_ff_raw, "tau_drive": tau_drive,
        "ff_clamped": ff_clamped, "ceiling_hit": ceiling_hit,
        "duration": duration, "dq": p["goal"] - p["q0"],
        "delayed_t": lost_t,
    }


def compute_metrics(res, p):
    """Validation numbers — does what should happen, happen?"""
    t, q, qd = res["t"], res["q"], res["qd"]
    q_d, qd_d = res["q_d"], res["qd_d"]
    duration, dq = res["duration"], res["dq"]
    lines = []

    peak_qd = float(np.max(np.abs(qd)))
    cmd_peak = float(np.max(np.abs(qd_d)))
    analytic = 1.875 * abs(dq) / duration
    lines.append(f"peak speed  sim {peak_qd:8.4f} | cmd {cmd_peak:8.4f} rad/s "
                 f"(quintic 1.875|dq|/T = {analytic:.4f})")
    if cmd_peak > 1e-9:
        lines.append(f"  sim/cmd speed ratio   : {peak_qd / cmd_peak:8.3f}")

    tail = t >= (t[-1] - 0.2)
    ss_err = float(np.mean(q_d[tail] - q[tail]))
    lines.append(f"steady-state pos error  : {ss_err:8.5f} rad")

    if abs(dq) > 1e-6:
        overshoot = max(0.0, float(np.max((q - p["goal"]) * np.sign(dq))) / abs(dq) * 100.0)
        lines.append(f"overshoot               : {overshoot:8.2f} %")
    else:
        lines.append("overshoot               :      n/a  (dq ~ 0)")

    band = max(0.02 * abs(dq), 0.01)
    outside = np.where(np.abs(q - p["goal"]) > band)[0]
    if len(outside) == 0:
        settle = "0.000 s"
    elif outside[-1] + 1 < len(t):
        settle = f"{t[outside[-1] + 1]:.3f} s"
    else:
        settle = "not settled"
    lines.append(f"settling time (+-{band:.3f})  : {settle}")

    peak_tau = float(np.max(np.abs(res["tau_drive"])))
    lines.append(f"peak |tau_drive|        : {peak_tau:8.4f} Nm")
    if p["ceiling_enabled"] and p["ceiling"] > 0.0:
        frac = float(np.mean(res["ceiling_hit"])) * 100.0
        hit = "YES" if res["ceiling_hit"].any() else "no"
        lines.append(f"  vs ceiling {p['ceiling']:6.2f} Nm  : {peak_tau / p['ceiling'] * 100:6.1f} %"
                     f"   hit={hit} ({frac:.1f}% of steps)")
    else:
        lines.append("  torque ceiling        : disabled")

    if p["tau_limit"] > 0.0:
        peak_raw = float(np.max(np.abs(res["tau_ff_raw"])))
        clamped = "YES" if res["ff_clamped"].any() else "no"
        lines.append(f"FF clamp (+-{p['tau_limit']:.1f} Nm)    : peak|tau_ff|={peak_raw:7.3f} Nm"
                     f"  clamped={clamped}")
    else:
        lines.append("FF clamp                : disabled (tau_limit <= 0)")

    traj = t <= duration
    rms = float(np.sqrt(np.mean((q_d[traj] - q[traj]) ** 2)))
    lines.append(f"RMS pos error (0..T)    : {rms:8.5f} rad")

    if p.get("mitigate", False):
        lines.append(f"pvt_goto hold fix       : ON  (lag<={p['err_max']:.3f} rad,"
                     f" traj delayed {res.get('delayed_t', 0.0):.2f} s)")
    else:
        lines.append("pvt_goto hold fix       : off")

    if p["block_enabled"]:
        b0 = p["block_start"]
        b1 = b0 + p["block_dur"]
        bmask = (t >= b0) & (t < b1)
        if bmask.any():
            blk_tau = float(np.max(np.abs(res["tau_drive"][bmask])))
            blk_lag = float(np.max(np.abs((q_d - q)[bmask])))
            lines.append(f"external block          : t={b0:.2f}..{b1:.2f} s")
            lines.append(f"  peak|tau| / pos lag    : {blk_tau:7.3f} Nm /{blk_lag:8.4f} rad")
        else:
            lines.append(f"external block          : t={b0:.2f}..{b1:.2f} s (outside sim)")
    return "\n".join(lines)


def envelope(t, y, target=3000):
    """Min/max envelope decimation for plotting.

    A long slow move produces a dense, fine ZOH sawtooth (the real 1 kHz-loop /
    stream-rate ripple). Drawn point-for-point across a wide axis it aliases
    into false jagged spikes. This collapses the signal to ~target points by
    emitting the per-bin min and max (in time order), so the ripple renders as
    an honest band. Signals already shorter than `target` are returned as-is.
    """
    n = len(t)
    if n <= target:
        return t, y
    bins = target // 2
    step = n // bins
    nb = n // step
    cut = nb * step
    tt = t[:cut].reshape(nb, step)
    yy = y[:cut].reshape(nb, step)
    rows = np.arange(nb)
    imin = yy.argmin(axis=1)
    imax = yy.argmax(axis=1)
    min_first = imin <= imax
    first_i = np.where(min_first, imin, imax)
    last_i = np.where(min_first, imax, imin)
    out_t = np.empty(nb * 2)
    out_y = np.empty(nb * 2)
    out_t[0::2] = tt[rows, first_i]
    out_t[1::2] = tt[rows, last_i]
    out_y[0::2] = yy[rows, first_i]
    out_y[1::2] = yy[rows, last_i]
    return out_t, out_y


# Parameter panel layout: (group title, [(key, label), ...]).
PARAM_GROUPS = [
    ("Trajectory", [
        ("q0", "start q0 (rad)"),
        ("goal", "goal (rad)"),
        ("duration", "duration (s)"),
        ("rate", "stream rate (Hz)"),
        ("err_max", "lag limit (rad)"),
    ]),
    ("Controller gains", [
        ("Kp", "Kp (Nm/rad)"),
        ("Kd", "Kd (Nm.s/rad)"),
        ("tau_limit", "tau_limit (Nm)"),
    ]),
    ("Feedforward model", [
        ("mgl", "mgl (Nm)"),
        ("J", "J (kg.m^2)"),
        ("Fv", "Fv (Nm.s/rad)"),
    ]),
    ("Plant physics", [
        ("B", "B damping (Nm.s/rad)"),
        ("friction", "Coulomb fric. (Nm)"),
        ("eps", "friction eps (rad/s)"),
        ("ceiling", "torque ceiling (Nm)"),
    ]),
    ("External block", [
        ("block_start", "block start (s)"),
        ("block_dur", "block duration (s)"),
    ]),
]

FLAG_LABELS = [
    ("ff_gravity", "gravity feedforward"),
    ("ff_inertia", "inertia feedforward"),
    ("ff_viscous", "viscous feedforward"),
    ("ceiling_enabled", "enforce torque ceiling"),
    ("block_enabled", "external position block"),
]


class PvtSimApp:
    """Tkinter app: parameter panel + embedded matplotlib + metrics."""

    def __init__(self, root):
        self.root = root
        root.title("PVT / MIT Mode Simulator")
        self.entries = {}
        self.flags = {}
        self.ghost = None
        self.last_res = None
        self.last_p = None
        self.mitigate = False  # pvt_goto hold fix — toggled by its button
        self._build_ui()
        self.run()  # initial render

    def _build_ui(self):
        left = ttk.Frame(self.root, padding=8)
        left.pack(side=tk.LEFT, fill=tk.Y)
        right = ttk.Frame(self.root)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        for title, fields in PARAM_GROUPS:
            grp = ttk.LabelFrame(left, text=title, padding=6)
            grp.pack(fill=tk.X, pady=3)
            for row, (key, label) in enumerate(fields):
                ttk.Label(grp, text=label).grid(row=row, column=0, sticky=tk.W,
                                                padx=2, pady=1)
                entry = ttk.Entry(grp, width=9)
                entry.insert(0, str(DEFAULTS[key]))
                entry.grid(row=row, column=1, padx=2, pady=1)
                entry.bind("<Return>", lambda _ev: self.run())
                self.entries[key] = entry

        toggles = ttk.LabelFrame(left, text="Toggles", padding=6)
        toggles.pack(fill=tk.X, pady=3)
        for key, label in FLAG_LABELS:
            var = tk.BooleanVar(value=FLAG_DEFAULTS[key])
            ttk.Checkbutton(toggles, text=label, variable=var).pack(anchor=tk.W)
            self.flags[key] = var
        self.ghost_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(toggles, text="keep previous run as ghost",
                        variable=self.ghost_var).pack(anchor=tk.W)

        btns = ttk.Frame(left)
        btns.pack(fill=tk.X, pady=(8, 2))
        ttk.Button(btns, text="Run / Compute", command=self.run).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=1)
        ttk.Button(btns, text="Reset", command=self.reset).pack(side=tk.LEFT, padx=1)
        ttk.Button(btns, text="Clear ghost", command=self.clear_ghost).pack(
            side=tk.LEFT, padx=1)

        # pvt_goto hold fix: toggles the following-error governor and stashes
        # the current run as a ghost, so the next Run overlays pre-fix vs post-fix.
        self.mitigate_btn = ttk.Button(left, text="pvt_goto hold fix:  OFF",
                                       command=self.toggle_mitigation)
        self.mitigate_btn.pack(fill=tk.X, pady=(0, 6))

        self.fig = Figure(figsize=(7.5, 8.0), dpi=100)
        self.axes = self.fig.subplots(4, 1, sharex=True)
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)

        self.metrics = tk.Text(right, height=13, font="TkFixedFont",
                               wrap=tk.NONE, state=tk.DISABLED, bg="#f4f4f4")
        self.metrics.pack(side=tk.BOTTOM, fill=tk.X)
        NavigationToolbar2Tk(self.canvas, right).update()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    def _read_params(self):
        """Read + validate the panel. Returns a params dict, or None on error."""
        p = {}
        for key, entry in self.entries.items():
            text = entry.get().strip()
            try:
                p[key] = float(text)
            except ValueError:
                messagebox.showerror("Invalid input",
                                     f"Field '{key}' is not a number: '{text}'")
                return None
        for key, var in self.flags.items():
            p[key] = bool(var.get())

        if p["duration"] <= 0.0:
            messagebox.showerror("Invalid input", "duration must be > 0")
            return None
        if p["rate"] <= 0.0:
            messagebox.showerror("Invalid input", "stream rate must be > 0")
            return None
        if p["J"] <= 0.0:
            messagebox.showerror("Invalid input", "J (inertia) must be > 0")
            return None
        if p["eps"] <= 0.0:
            messagebox.showerror("Invalid input", "friction eps must be > 0")
            return None
        if p["err_max"] <= 0.0:
            messagebox.showerror("Invalid input", "lag limit (err_max) must be > 0")
            return None
        p["mitigate"] = self.mitigate
        return p

    def run(self):
        params = self._read_params()
        if params is None:
            return
        if self.ghost_var.get() and self.last_res is not None:
            self.ghost = self.last_res
        res = simulate(params)
        self.last_res = res
        self.last_p = params
        self._plot(res, params)
        self._set_metrics(compute_metrics(res, params))

    def reset(self):
        for key, entry in self.entries.items():
            entry.delete(0, tk.END)
            entry.insert(0, str(DEFAULTS[key]))
        for key, var in self.flags.items():
            var.set(FLAG_DEFAULTS[key])
        self.mitigate = False
        self.mitigate_btn.config(text="pvt_goto hold fix:  OFF")
        self.run()

    def toggle_mitigation(self):
        """Toggle the pvt_goto hold fix, stash the current run as a ghost, and
        re-run — so the plot overlays pre-fix (ghost) against post-fix (latest).
        """
        self.mitigate = not self.mitigate
        self.mitigate_btn.config(
            text=f"pvt_goto hold fix:  {'ON' if self.mitigate else 'OFF'}")
        if self.last_res is not None:
            self.ghost = self.last_res
        self.run()

    def clear_ghost(self):
        self.ghost = None
        if self.last_res is not None:
            self._plot(self.last_res, self.last_p)

    def _set_metrics(self, text):
        self.metrics.config(state=tk.NORMAL)
        self.metrics.delete("1.0", tk.END)
        self.metrics.insert(tk.END, text)
        self.metrics.config(state=tk.DISABLED)

    def _plot(self, res, p):
        t = res["t"]
        ax_pos, ax_vel, ax_tau, ax_err = self.axes
        for ax in self.axes:
            ax.clear()

        ghost = self.ghost
        if ghost is not None:
            gt = ghost["t"]
            ax_pos.plot(*envelope(gt, ghost["q"]), color="0.6", lw=1.0,
                        alpha=0.5, label="q (ghost)")
            ax_vel.plot(*envelope(gt, ghost["qd"]), color="0.6", lw=1.0,
                        alpha=0.5, label="qd (ghost)")
            ax_tau.plot(*envelope(gt, ghost["tau_drive"]), color="0.6", lw=1.0,
                        alpha=0.5, label="tau_drive (ghost)")

        ax_pos.plot(*envelope(t, res["q_d"]), "b--", lw=1.2, label="q_d commanded")
        ax_pos.plot(*envelope(t, res["q"]), "b-", lw=1.6, label="q simulated")
        ax_pos.set_ylabel("position\n(rad)")

        ax_vel.plot(*envelope(t, res["qd_d"]), "g--", lw=1.2, label="qd_d commanded")
        ax_vel.plot(*envelope(t, res["qd"]), "g-", lw=1.6, label="qd simulated")
        ax_vel.set_ylabel("velocity\n(rad/s)")

        ax_tau.plot(*envelope(t, res["tau_kp"]), lw=1.0, color="tab:blue",
                    label="Kp term")
        ax_tau.plot(*envelope(t, res["tau_kd"]), lw=1.0, color="tab:green",
                    label="Kd term")
        ax_tau.plot(*envelope(t, res["tau_ff"]), lw=1.0, color="tab:orange",
                    label="tau_ff")
        ax_tau.plot(*envelope(t, res["tau_drive"]), "k-", lw=1.7,
                    label="tau_drive total")
        if p["tau_limit"] > 0.0:
            for sign in (1.0, -1.0):
                ax_tau.axhline(sign * p["tau_limit"], ls=":", color="orange",
                               lw=1.0, alpha=0.7)
        if p["ceiling_enabled"] and p["ceiling"] > 0.0:
            for sign in (1.0, -1.0):
                ax_tau.axhline(sign * p["ceiling"], ls="-.", color="red",
                               lw=1.0, alpha=0.7)
        ax_tau.set_ylabel("torque\n(Nm)")

        ax_err.plot(*envelope(t, res["q_d"] - res["q"]), "b-", lw=1.0,
                    label="q_d - q (rad)")
        ax_err.plot(*envelope(t, res["qd_d"] - res["qd"]), "g-", lw=1.0,
                    label="qd_d - qd (rad/s)")
        ax_err.axhline(0.0, color="0.7", lw=0.8)
        ax_err.set_ylabel("tracking\nerror")
        ax_err.set_xlabel("time (s)")

        block_on = p["block_enabled"]
        b0 = p["block_start"]
        b1 = b0 + p["block_dur"]
        for i, ax in enumerate(self.axes):
            ax.axvline(res["duration"], color="purple", ls=":", lw=1.0,
                       alpha=0.6)
            if block_on:
                ax.axvspan(b0, b1, color="red", alpha=0.10,
                           label="external block" if i == 0 else "_nolegend_")
            ax.grid(True, alpha=0.3)
            ax.legend(loc="best", fontsize=7, ncol=2)

        self.fig.tight_layout()
        self.canvas.draw()


def main():
    root = tk.Tk()
    root.geometry("1180x880")
    PvtSimApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
