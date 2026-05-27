#!/usr/bin/env python3
"""Pendulum PVT tuner — live gain tuning, mode toggle, e-stop, telemetry.

Discovers any active PVT controller by scanning the ROS service graph for nodes
that expose ``~/hold`` / ``~/free`` / ``~/damp`` Trigger services, lets the user
edit Kp / Kd / Kd_damp / mgl / ff_gravity, toggle HOLD / FREE / DAMP, drive the
supervisor e-stop & reset, and live-monitors bus voltage, motor / drive
temperature, following error, and the latched safety state.

Single-file by design — palette and ros2-CLI helpers are inlined.
"""

import os
import re
import subprocess
import sys
import threading
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64, Int8, String
from std_srvs.srv import Trigger

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

# ───────────────────────────────────────────────────────────
# CATPPUCCIN MOCHA PALETTE
# ───────────────────────────────────────────────────────────

C_BASE = "#1e1e2e"
C_SURFACE0 = "#313244"
C_SURFACE1 = "#45475a"
C_TEXT = "#cdd6f4"
C_SUBTEXT = "#6c7086"
C_GREEN = "#a6e3a1"
C_YELLOW = "#f9e2af"
C_RED = "#f38ba8"
C_BLUE = "#89b4fa"
C_PEACH = "#fab387"

GLOBAL_STYLESHEET = f"""
QMainWindow {{ background-color: {C_BASE}; color: {C_TEXT}; }}
QLabel {{ color: {C_TEXT}; }}
QGroupBox {{ color: {C_TEXT}; border: 1px solid {C_SURFACE1}; border-radius: 6px;
             margin-top: 10px; padding: 12px 10px 10px 10px; font-weight: bold; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 12px; padding: 0 4px;
                    color: {C_PEACH}; }}
QPushButton {{ background-color: {C_SURFACE0}; color: {C_TEXT};
               border: 1px solid {C_SURFACE1}; border-radius: 4px;
               padding: 6px 12px; font-weight: bold; }}
QPushButton:hover {{ background-color: {C_SURFACE1}; }}
QPushButton:pressed {{ background-color: #585b70; }}
QPushButton:disabled {{ color: {C_SUBTEXT}; }}
QDoubleSpinBox {{ background-color: {C_SURFACE0}; color: {C_TEXT};
                  border: 1px solid {C_SURFACE1}; border-radius: 4px;
                  padding: 4px 6px; font-family: monospace; min-width: 90px; }}
QComboBox {{ background-color: {C_SURFACE0}; color: {C_TEXT};
             border: 1px solid {C_SURFACE1}; border-radius: 4px;
             padding: 4px 8px; min-width: 180px; }}
QComboBox QAbstractItemView {{ background-color: {C_SURFACE0}; color: {C_TEXT};
                                selection-background-color: {C_SURFACE1}; }}
QCheckBox {{ color: {C_TEXT}; }}
"""

# ───────────────────────────────────────────────────────────
# ros2 CLI helpers (subprocess — matches action_position_sender style)
# ───────────────────────────────────────────────────────────


# ───────────────────────────────────────────────────────────
# Telemetry thresholds (match pendulum_safety defaults)
# ───────────────────────────────────────────────────────────

MOTOR_TEMP_WARN = 70.0
MOTOR_TEMP_ERROR = 90.0
DRIVE_TEMP_WARN = 70.0
DRIVE_TEMP_ERROR = 85.0
BUS_V_MIN = 40.0
BUS_V_MAX = 54.0

ESTOP_LABELS = {
    0: ("CLEAR", C_GREEN),
    1: ("E-STOP FREE", C_RED),
    2: ("E-STOP HOLD", C_RED),
}

SUPERVISOR_NODE = "/pendulum_safety_supervisor"


# ───────────────────────────────────────────────────────────
# Main window
# ───────────────────────────────────────────────────────────


class PvtTunerWindow(QMainWindow):

    # Cross-thread signals (ROS callbacks → GUI thread)
    _voltage_sig = pyqtSignal(float)
    _motor_temp_sig = pyqtSignal(float)
    _drive_temp_sig = pyqtSignal(float)
    _following_err_sig = pyqtSignal(float)
    _estop_state_sig = pyqtSignal(int)
    _breach_sig = pyqtSignal(str)
    _service_result_sig = pyqtSignal(str, bool, str)  # label, ok, message
    _discovery_sig = pyqtSignal(list, object, int, str)  # pvt, broadcaster, count, source
    _params_loaded_sig = pyqtSignal(str, object)  # ctrl, values dict
    _apply_results_sig = pyqtSignal(object)  # list of (name, ok, msg)
    _all_params_sig = pyqtSignal(str, object)  # ctrl, {name: value}
    _param_set_sig = pyqtSignal(str, bool, str)  # name, ok, message
    # Safety limits panel. Loaded values are {short_name: (supervisor_val,
    # controller_val)}; apply results are [(short_name, sup_ok, ctrl_ok, msg)].
    _safety_loaded_sig = pyqtSignal(object)
    _safety_applied_sig = pyqtSignal(object)

    def __init__(self, node: Node):
        super().__init__()
        self.setWindowTitle("Pendulum PVT Tuner")
        self.setMinimumSize(560, 600)
        self.setStyleSheet(GLOBAL_STYLESHEET)

        self._node = node
        self._controllers: list[str] = []
        self._current_ctrl: str | None = None
        self._joint_name: str = "—"

        # Per-signal cached subscriptions, keyed by topic
        self._telem_subs: dict[str, object] = {}
        self._broadcaster_ns: str | None = None

        # Build UI — two tabs: focused tuner + full parameter table
        self._tabs = QTabWidget()
        self.setCentralWidget(self._tabs)

        tuner_tab = QWidget()
        layout = QVBoxLayout(tuner_tab)
        layout.setSpacing(10)
        layout.setContentsMargins(12, 12, 12, 12)
        self._build_controller_box(layout)
        self._build_gains_box(layout)
        self._build_mode_box(layout)
        self._build_safety_box(layout)
        self._build_safety_limits_box(layout)
        self._build_telem_box(layout)
        layout.addStretch()
        self._tabs.addTab(tuner_tab, "Tuner")

        params_tab = QWidget()
        self._build_params_tab(params_tab)
        self._tabs.addTab(params_tab, "All parameters")

        # Wire cross-thread signals
        self._voltage_sig.connect(self._on_voltage)
        self._motor_temp_sig.connect(self._on_motor_temp)
        self._drive_temp_sig.connect(self._on_drive_temp)
        self._following_err_sig.connect(self._on_following_err)
        self._estop_state_sig.connect(self._on_estop_state)
        self._breach_sig.connect(self._on_breach)
        self._service_result_sig.connect(self._on_service_result)
        self._discovery_sig.connect(self._on_discovery_result)
        self._params_loaded_sig.connect(self._on_params_loaded)
        self._apply_results_sig.connect(self._on_apply_results)
        self._all_params_sig.connect(self._on_all_params_loaded)
        self._param_set_sig.connect(self._on_param_set)
        self._safety_loaded_sig.connect(self._on_safety_loaded)
        self._safety_applied_sig.connect(self._on_safety_applied)

        # Subscriptions that don't depend on controller discovery
        self._setup_safety_subs()

        # One-shot discovery at startup. Re-runs only on Refresh click —
        # cross-machine `ros2 control list_controllers` can take 10s+, so
        # polling would pile up parallel invocations and never finish.
        self._discovery_in_flight = False
        self._discover_async()

    # ─── UI BUILDERS ────────────────────────────────────────

    def _build_controller_box(self, parent):
        box = QGroupBox("Controller")
        v = QVBoxLayout(box)
        h = QHBoxLayout()
        h.setSpacing(10)
        self._ctrl_combo = QComboBox()
        self._ctrl_combo.currentTextChanged.connect(self._on_controller_changed)
        h.addWidget(self._ctrl_combo)
        self._joint_label = QLabel("joint: —")
        self._joint_label.setFont(QFont("monospace", 10))
        self._joint_label.setStyleSheet(f"color: {C_SUBTEXT};")
        h.addWidget(self._joint_label)
        h.addStretch()
        self._refresh_btn = QPushButton("Refresh")
        self._refresh_btn.setFixedWidth(80)
        self._refresh_btn.clicked.connect(self._discover_async)
        h.addWidget(self._refresh_btn)
        v.addLayout(h)
        self._discovery_status = QLabel("(discovering…)")
        self._discovery_status.setFont(QFont("monospace", 9))
        self._discovery_status.setStyleSheet(f"color: {C_SUBTEXT};")
        v.addWidget(self._discovery_status)
        parent.addWidget(box)

    def _build_gains_box(self, parent):
        box = QGroupBox("Gains && feedforward")
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)

        self._kp_spin = self._make_spin(0.0, 1000.0, 0.5, 3)
        self._kd_spin = self._make_spin(0.0, 100.0, 0.1, 3)
        self._kd_damp_spin = self._make_spin(0.0, 20.0, 0.1, 3)
        self._mgl_spin = self._make_spin(-100.0, 100.0, 0.01, 4)
        self._ff_grav_check = QCheckBox("ff_gravity")

        grid.addWidget(QLabel("Kp"), 0, 0)
        grid.addWidget(self._kp_spin, 0, 1)
        grid.addWidget(QLabel("Kd"), 0, 2)
        grid.addWidget(self._kd_spin, 0, 3)
        grid.addWidget(QLabel("mgl"), 0, 4)
        grid.addWidget(self._mgl_spin, 0, 5)

        grid.addWidget(self._ff_grav_check, 1, 0, 1, 2)
        grid.addWidget(QLabel("Kd_damp"), 1, 2)
        grid.addWidget(self._kd_damp_spin, 1, 3)

        self._read_btn = QPushButton("Read")
        self._read_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_BLUE}; "
            f"border: 1px solid {C_BLUE};"
        )
        self._read_btn.clicked.connect(self._on_read_gains)
        grid.addWidget(self._read_btn, 1, 4)

        self._apply_btn = QPushButton("Apply")
        self._apply_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_GREEN}; "
            f"border: 1px solid {C_GREEN};"
        )
        self._apply_btn.clicked.connect(self._on_apply_gains)
        grid.addWidget(self._apply_btn, 1, 5)

        self._gains_status = QLabel(" ")
        self._gains_status.setFont(QFont("monospace", 9))
        self._gains_status.setStyleSheet(f"color: {C_SUBTEXT};")
        grid.addWidget(self._gains_status, 2, 0, 1, 6)

        parent.addWidget(box)

    def _build_mode_box(self, parent):
        box = QGroupBox("Mode")
        v = QVBoxLayout(box)
        h = QHBoxLayout()
        self._hold_btn = QPushButton("HOLD (activate PVT)")
        self._hold_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_GREEN}; "
            f"border: 2px solid {C_GREEN};"
        )
        self._hold_btn.clicked.connect(lambda: self._call_controller_trigger("hold"))
        h.addWidget(self._hold_btn)

        self._free_btn = QPushButton("FREE (deactivate)")
        self._free_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_YELLOW}; "
            f"border: 2px solid {C_YELLOW};"
        )
        self._free_btn.clicked.connect(lambda: self._call_controller_trigger("free"))
        h.addWidget(self._free_btn)

        self._damp_btn = QPushButton("DAMP")
        self._damp_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_PEACH}; "
            f"border: 2px solid {C_PEACH};"
        )
        self._damp_btn.clicked.connect(lambda: self._call_controller_trigger("damp"))
        h.addWidget(self._damp_btn)
        v.addLayout(h)

        self._mode_label = QLabel("last action: —")
        self._mode_label.setFont(QFont("monospace", 10))
        self._mode_label.setStyleSheet(f"color: {C_SUBTEXT};")
        v.addWidget(self._mode_label)
        parent.addWidget(box)

    def _build_safety_box(self, parent):
        box = QGroupBox("Safety")
        v = QVBoxLayout(box)
        h = QHBoxLayout()
        self._estop_btn = QPushButton("E-STOP")
        self._estop_btn.setStyleSheet(
            f"background-color: {C_RED}; color: {C_BASE}; "
            f"border: 2px solid {C_RED}; font-size: 14px;"
        )
        self._estop_btn.clicked.connect(
            lambda: self._call_supervisor_trigger("estop")
        )
        h.addWidget(self._estop_btn)

        self._reset_btn = QPushButton("RESET")
        self._reset_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_PEACH}; "
            f"border: 2px solid {C_PEACH};"
        )
        self._reset_btn.clicked.connect(
            lambda: self._call_supervisor_trigger("reset")
        )
        h.addWidget(self._reset_btn)
        h.addStretch()
        v.addLayout(h)

        info = QGridLayout()
        info.addWidget(QLabel("state:"), 0, 0)
        self._estop_state_label = QLabel("(waiting)")
        self._estop_state_label.setFont(QFont("monospace", 11, QFont.Bold))
        info.addWidget(self._estop_state_label, 0, 1)
        info.addWidget(QLabel("breach:"), 1, 0)
        self._breach_label = QLabel("—")
        self._breach_label.setFont(QFont("monospace", 10))
        self._breach_label.setStyleSheet(f"color: {C_SUBTEXT};")
        info.addWidget(self._breach_label, 1, 1)
        info.setColumnStretch(1, 1)
        v.addLayout(info)
        parent.addWidget(box)

    # The numeric safety limits exposed in the Safety limits panel. Each lives
    # as a `safety.<key>` parameter on BOTH the supervisor (used for breach
    # detection) AND on every controller (used for command clamping). Apply
    # writes to both so the two stores stay in sync.
    _SAFETY_LIMIT_FIELDS = (
        # (key, label, lo, hi, step, decimals)
        ("position_min",      "pos_min",        -1000.0,  1000.0, 0.01, 4),
        ("position_max",      "pos_max",        -1000.0,  1000.0, 0.01, 4),
        ("velocity_limit",    "vel_limit",         0.0,    100.0, 0.1,  3),
        ("effort_limit",      "effort_limit",      0.0,   1000.0, 0.1,  3),
        ("slew_rate_limit",   "slew_rate",         0.0,    100.0, 0.1,  3),
        ("acceleration_limit","accel_limit",       0.0,   1000.0, 1.0,  2),
    )

    def _build_safety_limits_box(self, parent):
        box = QGroupBox("Safety limits  (writes to supervisor && active controller)")
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)

        # Two fields per row: (label, spin, label, spin) across 6 columns.
        self._safety_spins: dict[str, QDoubleSpinBox] = {}
        for idx, (key, label, lo, hi, step, decimals) in enumerate(
            self._SAFETY_LIMIT_FIELDS
        ):
            row = idx // 2
            col = (idx % 2) * 3
            spin = self._make_spin(lo, hi, step, decimals)
            self._safety_spins[key] = spin
            grid.addWidget(QLabel(label), row, col)
            grid.addWidget(spin, row, col + 1)

        btn_row = len(self._SAFETY_LIMIT_FIELDS) // 2 + 1
        self._safety_read_btn = QPushButton("Read")
        self._safety_read_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_BLUE}; "
            f"border: 1px solid {C_BLUE};"
        )
        self._safety_read_btn.clicked.connect(self._on_read_safety_limits)
        grid.addWidget(self._safety_read_btn, btn_row, 4)

        self._safety_apply_btn = QPushButton("Apply")
        self._safety_apply_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_GREEN}; "
            f"border: 1px solid {C_GREEN};"
        )
        self._safety_apply_btn.clicked.connect(self._on_apply_safety_limits)
        grid.addWidget(self._safety_apply_btn, btn_row, 5)

        self._safety_status = QLabel(" ")
        self._safety_status.setFont(QFont("monospace", 9))
        self._safety_status.setStyleSheet(f"color: {C_SUBTEXT};")
        grid.addWidget(self._safety_status, btn_row + 1, 0, 1, 6)

        parent.addWidget(box)

    def _on_read_safety_limits(self):
        ctrl = self._current_ctrl
        if not ctrl:
            self._safety_status.setText("no controller selected")
            self._safety_status.setStyleSheet(f"color: {C_YELLOW};")
            return
        self._safety_read_btn.setEnabled(False)
        keys = [k for k, *_ in self._SAFETY_LIMIT_FIELDS]
        names = [f"safety.{k}" for k in keys]

        def worker():
            sup_vals = self._rclpy_get_params(SUPERVISOR_NODE, names)
            ctrl_vals = self._rclpy_get_params(ctrl, names)
            merged = {
                k: (sup_vals.get(f"safety.{k}"), ctrl_vals.get(f"safety.{k}"))
                for k in keys
            }
            self._safety_loaded_sig.emit(merged)

        threading.Thread(target=worker, daemon=True).start()

    def _on_safety_loaded(self, merged: dict):
        self._safety_read_btn.setEnabled(True)
        mismatches = []
        any_value = False
        for key, (sup_v, ctrl_v) in merged.items():
            # Prefer supervisor's value (authoritative for breach detection).
            v = sup_v if isinstance(sup_v, (int, float)) else ctrl_v
            if isinstance(v, (int, float)):
                any_value = True
                self._safety_spins[key].setValue(float(v))
            if (isinstance(sup_v, (int, float)) and
                isinstance(ctrl_v, (int, float)) and
                abs(sup_v - ctrl_v) > 1e-9):
                mismatches.append(f"{key}: sup={sup_v:.4g} ctrl={ctrl_v:.4g}")
        if not any_value:
            self._safety_status.setText("no safety.* params returned")
            self._safety_status.setStyleSheet(f"color: {C_YELLOW};")
        elif mismatches:
            self._safety_status.setText(
                "mismatch (supervisor shown): " + " | ".join(mismatches)
            )
            self._safety_status.setStyleSheet(f"color: {C_YELLOW};")
        else:
            self._safety_status.setText("read OK — supervisor and controller agree")
            self._safety_status.setStyleSheet(f"color: {C_GREEN};")

    def _on_apply_safety_limits(self):
        ctrl = self._current_ctrl
        if not ctrl:
            self._safety_status.setText("no controller selected")
            self._safety_status.setStyleSheet(f"color: {C_YELLOW};")
            return
        self._safety_apply_btn.setEnabled(False)
        pairs = [
            (f"safety.{k}", float(self._safety_spins[k].value()))
            for k, *_ in self._SAFETY_LIMIT_FIELDS
        ]

        def worker():
            print(f"[pvt_tuner] safety apply -> supervisor + {ctrl}: {pairs}",
                  file=sys.stderr, flush=True)
            sup_res = self._rclpy_set_params(SUPERVISOR_NODE, pairs)
            ctrl_res = self._rclpy_set_params(ctrl, pairs)
            # Stitch into [(short_name, sup_ok, ctrl_ok, msg)].
            sup_by_name = {n: (ok, msg) for (n, ok, msg) in sup_res}
            ctrl_by_name = {n: (ok, msg) for (n, ok, msg) in ctrl_res}
            stitched = []
            for full, _ in pairs:
                short = full[len("safety."):]
                sup_ok, sup_msg = sup_by_name.get(full, (False, "no response"))
                ctrl_ok, ctrl_msg = ctrl_by_name.get(full, (False, "no response"))
                msg = ""
                if not sup_ok:
                    msg = f"sup: {sup_msg}"
                if not ctrl_ok:
                    msg = (msg + " | " if msg else "") + f"ctrl: {ctrl_msg}"
                stitched.append((short, sup_ok, ctrl_ok, msg))
            self._safety_applied_sig.emit(stitched)

        threading.Thread(target=worker, daemon=True).start()

    def _on_safety_applied(self, results: list):
        self._safety_apply_btn.setEnabled(True)
        all_ok = True
        fails = []
        for short, sup_ok, ctrl_ok, msg in results:
            if not (sup_ok and ctrl_ok):
                all_ok = False
                fails.append(f"{short} ({msg})")
        if all_ok:
            self._safety_status.setText(
                f"applied to supervisor + controller ({len(results)} fields)"
            )
            self._safety_status.setStyleSheet(f"color: {C_GREEN};")
        else:
            self._safety_status.setText("FAIL: " + " | ".join(fails))
            self._safety_status.setStyleSheet(f"color: {C_RED};")

    def _build_telem_box(self, parent):
        box = QGroupBox("Telemetry")
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)

        def mk_label():
            lab = QLabel("—")
            lab.setFont(QFont("monospace", 12, QFont.Bold))
            lab.setStyleSheet(f"color: {C_SUBTEXT};")
            return lab

        grid.addWidget(QLabel("bus voltage"), 0, 0)
        self._voltage_label = mk_label()
        grid.addWidget(self._voltage_label, 0, 1)

        grid.addWidget(QLabel("motor temp"), 1, 0)
        self._motor_temp_label = mk_label()
        grid.addWidget(self._motor_temp_label, 1, 1)

        grid.addWidget(QLabel("drive temp"), 2, 0)
        self._drive_temp_label = mk_label()
        grid.addWidget(self._drive_temp_label, 2, 1)

        grid.addWidget(QLabel("following error"), 3, 0)
        self._foll_err_label = mk_label()
        grid.addWidget(self._foll_err_label, 3, 1)

        self._broadcaster_label = QLabel("(searching for drive_status_broadcaster…)")
        self._broadcaster_label.setFont(QFont("monospace", 9))
        self._broadcaster_label.setStyleSheet(f"color: {C_SUBTEXT};")
        grid.addWidget(self._broadcaster_label, 4, 0, 1, 2)
        grid.setColumnStretch(1, 1)
        parent.addWidget(box)

    @staticmethod
    def _make_spin(lo, hi, step, decimals):
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setSingleStep(step)
        s.setDecimals(decimals)
        return s

    # ─── DISCOVERY ──────────────────────────────────────────

    def _discover_async(self):
        if self._discovery_in_flight:
            print("[pvt_tuner] discovery already in flight; ignoring",
                  file=sys.stderr, flush=True)
            return
        self._discovery_in_flight = True
        self._refresh_btn.setEnabled(False)
        self._refresh_btn.setText("…")
        print("[pvt_tuner] discovery cycle starting", file=sys.stderr, flush=True)
        threading.Thread(target=self._discover_worker_safe, daemon=True).start()

    def _discover_worker_safe(self):
        try:
            self._discover_worker()
        except Exception as e:
            print(
                f"[pvt_tuner] discovery thread crashed: {e!r}",
                file=sys.stderr, flush=True,
            )
            QTimer.singleShot(0, self._discovery_done)

    def _discovery_done(self):
        self._discovery_in_flight = False
        self._refresh_btn.setEnabled(True)
        self._refresh_btn.setText("Refresh")

    def _discover_worker(self):
        """Try rclpy /controller_manager/list_controllers first (uses the GUI's
        warm DDS cache → fast), fall back to `ros2 control list_controllers`
        subprocess (slow on cold cross-machine DDS, but always works)."""
        print("[pvt_tuner] trying rclpy ListControllers…",
              file=sys.stderr, flush=True)
        controllers = self._list_controllers_rclpy()
        source = "rclpy"
        if controllers is None:
            print("[pvt_tuner] rclpy path failed; falling back to subprocess "
                  "(may take 10–30s on first call cross-machine)…",
                  file=sys.stderr, flush=True)
            controllers = self._list_controllers_subprocess(timeout=30)
            source = "subprocess"

        pvt: list[str] = []
        broadcaster_ns: str | None = None
        for name, ctype, state in controllers:
            print(f"[pvt_tuner]   - {name!r}  type={ctype!r}  state={state!r}",
                  file=sys.stderr, flush=True)
            if state != "active":
                continue
            if ctype.endswith("PVTController"):
                pvt.append(f"/{name}")
            elif ctype.endswith("DriveStatusBroadcaster"):
                broadcaster_ns = f"/{name}"

        pvt.sort()
        print(f"[pvt_tuner] dispatching to GUI: pvt={pvt} broadcaster={broadcaster_ns}",
              file=sys.stderr, flush=True)
        self._discovery_sig.emit(pvt, broadcaster_ns, len(controllers), source)

    def _list_controllers_rclpy(self):
        """Call /controller_manager/list_controllers from the GUI's rclpy node.

        Returns a list of (name, type, state) on success, or None on failure.
        Verbose stderr logging so we can pin down which step stalls in
        cross-machine setups where DDS discovery is slow."""
        print("[pvt_tuner] rclpy: creating client", file=sys.stderr, flush=True)
        client = self._node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        try:
            print("[pvt_tuner] rclpy: waiting for service (≤30s)",
                  file=sys.stderr, flush=True)
            t0 = time.time()
            ok = client.wait_for_service(timeout_sec=30.0)
            print(f"[pvt_tuner] rclpy: wait_for_service={ok} "
                  f"after {time.time()-t0:.1f}s", file=sys.stderr, flush=True)
            if not ok:
                return None
            print("[pvt_tuner] rclpy: sending request", file=sys.stderr, flush=True)
            future = client.call_async(ListControllers.Request())
            deadline = time.time() + 30.0
            last_log = time.time()
            while not future.done() and time.time() < deadline:
                time.sleep(0.05)
                if time.time() - last_log > 2.0:
                    print(f"[pvt_tuner] rclpy: still waiting on response "
                          f"({time.time()-t0:.1f}s)",
                          file=sys.stderr, flush=True)
                    last_log = time.time()
            if not future.done():
                print("[pvt_tuner] rclpy: response timed out after 30s",
                      file=sys.stderr, flush=True)
                return None
            resp = future.result()
            print(f"[pvt_tuner] rclpy: response received with "
                  f"{len(resp.controller)} controller(s)",
                  file=sys.stderr, flush=True)
            return [(c.name, c.type, c.state) for c in resp.controller]
        finally:
            self._node.destroy_client(client)

    @staticmethod
    def _list_controllers_subprocess(timeout=10):
        """Parse `ros2 control list_controllers` output.

        Each row: ``name  pkg/Plugin  active|inactive|unconfigured``."""
        try:
            r = subprocess.run(
                ["ros2", "control", "list_controllers"],
                capture_output=True, text=True, timeout=timeout,
            )
            if r.returncode != 0:
                print(
                    f"[pvt_tuner] `ros2 control list_controllers` returned "
                    f"{r.returncode}: {r.stderr.strip()}",
                    file=sys.stderr, flush=True,
                )
                return []
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            print(f"[pvt_tuner] subprocess failed: {e!r}",
                  file=sys.stderr, flush=True)
            return []
        out = []
        ansi = re.compile(r"\x1b\[[0-9;]*[a-zA-Z]")
        for raw in r.stdout.splitlines():
            line = ansi.sub("", raw).strip()
            m = re.match(r"^(\S+)\s+(\S+/\S+)\s+(active|inactive|unconfigured)", line)
            if m:
                out.append((m.group(1), m.group(2), m.group(3)))
        return out

    def _on_discovery_result(self, pvt, broadcaster_ns, total_controllers, source):
        """pyqtSignal handler — runs on the GUI thread."""
        print("[pvt_tuner] _on_discovery_result fired on GUI thread",
              file=sys.stderr, flush=True)
        self._apply_discovery(pvt, broadcaster_ns, total_controllers, source)
        self._discovery_done()

    def _apply_discovery(
        self,
        pvt: list[str],
        broadcaster_ns: str | None,
        total_controllers: int,
        source: str,
    ):
        domain = os.environ.get("ROS_DOMAIN_ID", "0")
        if pvt:
            msg = (
                f"found {len(pvt)} PVT controller(s) via {source} "
                f"on domain {domain}: {', '.join(pvt)}"
            )
            self._discovery_status.setStyleSheet(f"color: {C_GREEN};")
        else:
            msg = (
                f"no PVT controller found via {source} on domain {domain} "
                f"({total_controllers} controllers listed). Is the "
                f"controller_manager reachable? Click Refresh."
            )
            self._discovery_status.setStyleSheet(f"color: {C_YELLOW};")
        self._discovery_status.setText(msg)
        print(f"[pvt_tuner] discovery: {msg}", file=sys.stderr, flush=True)
        if broadcaster_ns:
            print(
                f"[pvt_tuner] telemetry broadcaster: {broadcaster_ns}",
                file=sys.stderr, flush=True,
            )

        # Update controller dropdown without losing the current selection.
        if pvt != self._controllers:
            self._controllers = pvt
            current = self._ctrl_combo.currentText()
            self._ctrl_combo.blockSignals(True)
            self._ctrl_combo.clear()
            self._ctrl_combo.addItems(pvt)
            if current in pvt:
                self._ctrl_combo.setCurrentText(current)
            elif pvt:
                self._ctrl_combo.setCurrentIndex(0)
            self._ctrl_combo.blockSignals(False)
            self._on_controller_changed(self._ctrl_combo.currentText())

        if broadcaster_ns and broadcaster_ns != self._broadcaster_ns:
            self._broadcaster_ns = broadcaster_ns
            self._setup_telemetry_subs(broadcaster_ns)
            self._broadcaster_label.setText(f"broadcaster: {broadcaster_ns}")

    # ─── CONTROLLER PARAMS ──────────────────────────────────

    def _on_controller_changed(self, name: str):
        name = name.strip()
        if not name:
            self._current_ctrl = None
            return
        self._current_ctrl = name
        threading.Thread(
            target=self._load_controller_params, args=(name,), daemon=True
        ).start()
        # Auto-load the All Parameters tab too so it's never stale.
        self._refresh_all_params_async()
        # And the Safety limits panel — it reads from both the supervisor and
        # this controller, so it depends on which controller is selected.
        self._on_read_safety_limits()

    def _load_controller_params(self, ctrl: str):
        names = ["Kp", "Kd", "Kd_damp", "mgl", "ff_gravity", "joint"]
        print(f"[pvt_tuner] get_parameters({ctrl}, {names})",
              file=sys.stderr, flush=True)
        values = self._rclpy_get_params(ctrl, names)
        print(f"[pvt_tuner] got: {values}", file=sys.stderr, flush=True)
        self._params_loaded_sig.emit(ctrl, values)

    def _on_params_loaded(self, ctrl: str, values: dict):
        """pyqtSignal slot — runs on the GUI thread, safe to touch widgets."""
        if self._current_ctrl != ctrl:
            return
        kp = values.get("Kp")
        kd = values.get("Kd")
        kd_damp = values.get("Kd_damp")
        mgl = values.get("mgl")
        ff_g = values.get("ff_gravity")
        joint = values.get("joint")
        if isinstance(kp, (int, float)) and not isinstance(kp, bool):
            self._kp_spin.setValue(float(kp))
        if isinstance(kd, (int, float)) and not isinstance(kd, bool):
            self._kd_spin.setValue(float(kd))
        if isinstance(kd_damp, (int, float)) and not isinstance(kd_damp, bool):
            self._kd_damp_spin.setValue(float(kd_damp))
        if isinstance(mgl, (int, float)) and not isinstance(mgl, bool):
            self._mgl_spin.setValue(float(mgl))
        if isinstance(ff_g, bool):
            self._ff_grav_check.setChecked(ff_g)
        self._joint_name = joint if isinstance(joint, str) and joint else "—"
        self._joint_label.setText(f"joint: {self._joint_name}")
        self._read_btn.setEnabled(True)

    def _rclpy_get_params(self, ctrl: str, names: list[str], timeout: float = 5.0):
        """Read parameters from <ctrl> via rcl_interfaces/srv/GetParameters."""
        client = self._node.create_client(GetParameters, f"{ctrl}/get_parameters")
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                print(f"[pvt_tuner] get_parameters: {ctrl}/get_parameters "
                      f"unavailable after {timeout}s", file=sys.stderr, flush=True)
                return {}
            req = GetParameters.Request()
            req.names = names
            future = client.call_async(req)
            deadline = time.time() + timeout
            while not future.done() and time.time() < deadline:
                time.sleep(0.02)
            if not future.done():
                print("[pvt_tuner] get_parameters: response timed out",
                      file=sys.stderr, flush=True)
                return {}
            resp = future.result()
            out: dict[str, object] = {}
            for name, val in zip(names, resp.values):
                t = val.type
                if t == ParameterType.PARAMETER_BOOL:
                    out[name] = bool(val.bool_value)
                elif t == ParameterType.PARAMETER_INTEGER:
                    out[name] = int(val.integer_value)
                elif t == ParameterType.PARAMETER_DOUBLE:
                    out[name] = float(val.double_value)
                elif t == ParameterType.PARAMETER_STRING:
                    out[name] = str(val.string_value)
                # NOT_SET / other types are skipped — caller handles missing keys.
            return out
        finally:
            self._node.destroy_client(client)

    def _rclpy_set_params(
        self,
        ctrl: str,
        pairs: list[tuple[str, object]],
        timeout: float = 5.0,
    ):
        """Set parameters via rcl_interfaces/srv/SetParameters.

        Returns list of (name, ok, message)."""
        client = self._node.create_client(SetParameters, f"{ctrl}/set_parameters")
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return [(n, False, "service unavailable") for n, _ in pairs]
            req = SetParameters.Request()
            params = []
            for name, value in pairs:
                p = ParameterMsg()
                p.name = name
                pv = ParameterValue()
                # Order matters: bool is a subclass of int, so check bool first.
                if isinstance(value, bool):
                    pv.type = ParameterType.PARAMETER_BOOL
                    pv.bool_value = value
                elif isinstance(value, int):
                    pv.type = ParameterType.PARAMETER_INTEGER
                    pv.integer_value = value
                elif isinstance(value, float):
                    pv.type = ParameterType.PARAMETER_DOUBLE
                    pv.double_value = value
                elif isinstance(value, str):
                    pv.type = ParameterType.PARAMETER_STRING
                    pv.string_value = value
                else:
                    return [(n, False, f"unsupported type {type(value)}") for n, _ in pairs]
                p.value = pv
                params.append(p)
            req.parameters = params
            future = client.call_async(req)
            deadline = time.time() + timeout
            while not future.done() and time.time() < deadline:
                time.sleep(0.02)
            if not future.done():
                return [(n, False, "timeout") for n, _ in pairs]
            resp = future.result()
            return [
                (n, bool(r.successful), str(r.reason) if r.reason else "")
                for (n, _), r in zip(pairs, resp.results)
            ]
        finally:
            self._node.destroy_client(client)

    def _on_read_gains(self):
        ctrl = self._current_ctrl
        if not ctrl:
            self._gains_status.setText("no controller selected")
            self._gains_status.setStyleSheet(f"color: {C_YELLOW};")
            return
        self._read_btn.setEnabled(False)
        # _load_controller_params emits _params_loaded_sig; _on_params_loaded
        # re-enables the Read button.
        threading.Thread(
            target=self._load_controller_params, args=(ctrl,), daemon=True
        ).start()

    def _on_apply_gains(self):
        ctrl = self._current_ctrl
        if not ctrl:
            self._gains_status.setText("no controller selected")
            return
        self._apply_btn.setEnabled(False)
        targets: list[tuple[str, object]] = [
            ("Kp", float(self._kp_spin.value())),
            ("Kd", float(self._kd_spin.value())),
            ("Kd_damp", float(self._kd_damp_spin.value())),
            ("mgl", float(self._mgl_spin.value())),
            ("ff_gravity", bool(self._ff_grav_check.isChecked())),
        ]

        def worker():
            print(f"[pvt_tuner] set_parameters({ctrl}, {targets})",
                  file=sys.stderr, flush=True)
            results = self._rclpy_set_params(ctrl, targets)
            print(f"[pvt_tuner] set results: {results}",
                  file=sys.stderr, flush=True)
            self._apply_results_sig.emit(results)
            # Re-read so spinboxes reflect what the controller actually latched.
            self._load_controller_params(ctrl)

        threading.Thread(target=worker, daemon=True).start()

    def _on_apply_results(self, results: list):
        self._apply_btn.setEnabled(True)
        lines = []
        all_ok = True
        for name, ok, msg in results:
            if not ok:
                all_ok = False
                lines.append(f"{name} FAIL: {msg}")
            else:
                lines.append(name)
        self._gains_status.setText("set: " + " | ".join(lines))
        self._gains_status.setStyleSheet(
            f"color: {C_GREEN if all_ok else C_RED};"
        )

    # ─── SERVICE CALLS ──────────────────────────────────────

    def _call_controller_trigger(self, leaf: str):
        ctrl = self._current_ctrl
        if not ctrl:
            self._mode_label.setText("last action: (no controller selected)")
            return
        srv = f"{ctrl}/{leaf}"
        self._async_trigger(srv, label=f"{leaf}")

    def _call_supervisor_trigger(self, leaf: str):
        srv = f"{SUPERVISOR_NODE}/{leaf}"
        self._async_trigger(srv, label=f"supervisor/{leaf}")

    def _async_trigger(self, service: str, label: str):
        """Fire a Trigger service call from a transient client. Reports the
        result via _service_result_sig so the GUI thread can render it."""

        def worker():
            client = self._node.create_client(Trigger, service)
            try:
                if not client.wait_for_service(timeout_sec=2.0):
                    self._service_result_sig.emit(
                        label, False, f"service {service} unavailable"
                    )
                    return
                future = client.call_async(Trigger.Request())
                # Wait on the future — rclpy.spin runs on a separate thread,
                # so the future will complete here.
                deadline = time.time() + 5.0
                while not future.done() and time.time() < deadline:
                    time.sleep(0.02)
                if not future.done():
                    self._service_result_sig.emit(label, False, "timeout")
                    return
                resp = future.result()
                self._service_result_sig.emit(
                    label, bool(resp.success), resp.message or ""
                )
            finally:
                self._node.destroy_client(client)

        threading.Thread(target=worker, daemon=True).start()

    def _on_service_result(self, label: str, ok: bool, message: str):
        text = f"{label}: {'OK' if ok else 'FAIL'}"
        if message:
            text += f" — {message}"
        if label.startswith("supervisor/"):
            # Surface as a breach hint until the latched topic refreshes.
            self._breach_label.setText(text)
            self._breach_label.setStyleSheet(
                f"color: {C_GREEN if ok else C_RED};"
            )
        else:
            self._mode_label.setText(f"last action: {text}")
            self._mode_label.setStyleSheet(
                f"color: {C_GREEN if ok else C_RED};"
            )

    # ─── SAFETY SUBSCRIPTIONS (latched) ─────────────────────

    def _setup_safety_subs(self):
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._node.create_subscription(
            Int8, "/pendulum/safety/estop_state",
            lambda m: self._estop_state_sig.emit(int(m.data)), latched,
        )
        self._node.create_subscription(
            String, "/pendulum/safety/breach_reason",
            lambda m: self._breach_sig.emit(str(m.data)), latched,
        )

    def _on_estop_state(self, state: int):
        label, color = ESTOP_LABELS.get(state, (f"UNKNOWN ({state})", C_RED))
        self._estop_state_label.setText(label)
        self._estop_state_label.setStyleSheet(f"color: {color};")

    def _on_breach(self, reason: str):
        text = reason if reason else "—"
        self._breach_label.setText(text)
        color = C_SUBTEXT if reason in ("", "NONE", "none") else C_RED
        self._breach_label.setStyleSheet(f"color: {color};")

    # ─── TELEMETRY SUBSCRIPTIONS ────────────────────────────

    def _setup_telemetry_subs(self, ns: str):
        # Tear down any existing subs from a previous broadcaster.
        for topic, sub in list(self._telem_subs.items()):
            self._node.destroy_subscription(sub)
            del self._telem_subs[topic]

        def sub(name: str, sig):
            topic = f"{ns}/{name}"
            self._telem_subs[topic] = self._node.create_subscription(
                Float64, topic,
                lambda m, s=sig: s.emit(float(m.data)), 10,
            )

        sub("bus_voltage", self._voltage_sig)
        sub("motor_temperature", self._motor_temp_sig)
        sub("drive_temperature", self._drive_temp_sig)
        sub("following_error", self._following_err_sig)

    def _on_voltage(self, v: float):
        color = C_GREEN if BUS_V_MIN <= v <= BUS_V_MAX else C_RED
        self._voltage_label.setText(f"{v:7.2f} V")
        self._voltage_label.setStyleSheet(f"color: {color};")

    def _on_motor_temp(self, t: float):
        color = (
            C_RED if t >= MOTOR_TEMP_ERROR
            else C_YELLOW if t >= MOTOR_TEMP_WARN
            else C_GREEN
        )
        self._motor_temp_label.setText(f"{t:7.1f} °C")
        self._motor_temp_label.setStyleSheet(f"color: {color};")

    def _on_drive_temp(self, t: float):
        color = (
            C_RED if t >= DRIVE_TEMP_ERROR
            else C_YELLOW if t >= DRIVE_TEMP_WARN
            else C_GREEN
        )
        self._drive_temp_label.setText(f"{t:7.1f} °C")
        self._drive_temp_label.setStyleSheet(f"color: {color};")

    def _on_following_err(self, e: float):
        # No supervisor threshold for following error — colour only by magnitude.
        absv = abs(e)
        color = C_GREEN if absv < 0.05 else C_YELLOW if absv < 0.2 else C_RED
        self._foll_err_label.setText(f"{e:+8.4f} rad")
        self._foll_err_label.setStyleSheet(f"color: {color};")

    # ─── ALL-PARAMETERS TAB ─────────────────────────────────

    # System / read-only params not worth showing in the editor.
    _PARAMS_HIDDEN = frozenset({
        "use_sim_time",
        "qos_overrides",
        "start_type_description_service",
    })

    def _build_params_tab(self, root: QWidget):
        layout = QVBoxLayout(root)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        header = QHBoxLayout()
        self._params_tab_status = QLabel("(select a controller in the Tuner tab)")
        self._params_tab_status.setFont(QFont("monospace", 9))
        self._params_tab_status.setStyleSheet(f"color: {C_SUBTEXT};")
        header.addWidget(self._params_tab_status)
        header.addStretch()
        self._params_refresh_btn = QPushButton("Refresh")
        self._params_refresh_btn.setFixedWidth(90)
        self._params_refresh_btn.clicked.connect(self._refresh_all_params_async)
        header.addWidget(self._params_refresh_btn)
        layout.addLayout(header)

        # Scrollable grid: name | editor | Set
        self._params_grid_host = QWidget()
        self._params_grid = QGridLayout(self._params_grid_host)
        self._params_grid.setColumnStretch(1, 1)
        self._params_grid.setHorizontalSpacing(10)
        self._params_grid.setVerticalSpacing(4)

        scroll = QScrollArea()
        scroll.setWidget(self._params_grid_host)
        scroll.setWidgetResizable(True)
        layout.addWidget(scroll, 1)

        # name -> (editor, get_value_fn, set_btn, set_btn_label)
        self._params_editors: dict[str, tuple] = {}

    def _clear_params_grid(self):
        while self._params_grid.count():
            item = self._params_grid.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()
        self._params_editors.clear()

    def _refresh_all_params_async(self):
        ctrl = self._current_ctrl
        if not ctrl:
            self._params_tab_status.setText("(select a controller in the Tuner tab)")
            self._params_tab_status.setStyleSheet(f"color: {C_SUBTEXT};")
            return
        self._params_refresh_btn.setEnabled(False)
        self._params_tab_status.setText(f"loading {ctrl} …")
        self._params_tab_status.setStyleSheet(f"color: {C_BLUE};")
        threading.Thread(
            target=self._all_params_worker, args=(ctrl,), daemon=True
        ).start()

    def _all_params_worker(self, ctrl: str):
        names = self._rclpy_list_params(ctrl)
        names = sorted(
            n for n in names
            if n not in self._PARAMS_HIDDEN and not n.startswith("qos_overrides.")
        )
        print(f"[pvt_tuner] list_parameters({ctrl}) -> {len(names)} names",
              file=sys.stderr, flush=True)
        if not names:
            self._all_params_sig.emit(ctrl, {})
            return
        values = self._rclpy_get_params(ctrl, names, timeout=10.0)
        self._all_params_sig.emit(ctrl, values)

    def _on_all_params_loaded(self, ctrl: str, values: dict):
        """GUI-thread handler — rebuild the grid."""
        self._params_refresh_btn.setEnabled(True)
        if self._current_ctrl != ctrl:
            return  # stale response
        self._clear_params_grid()
        if not values:
            self._params_tab_status.setText(
                f"no parameters returned for {ctrl}"
            )
            self._params_tab_status.setStyleSheet(f"color: {C_YELLOW};")
            return

        self._params_tab_status.setText(
            f"{ctrl}  ({len(values)} parameters)"
        )
        self._params_tab_status.setStyleSheet(f"color: {C_GREEN};")

        for row, (name, value) in enumerate(values.items()):
            label = QLabel(name)
            label.setFont(QFont("monospace", 10))
            self._params_grid.addWidget(label, row, 0)

            editor, get_fn = self._make_param_editor(value)
            self._params_grid.addWidget(editor, row, 1)

            set_btn = QPushButton("Set")
            set_btn.setFixedWidth(60)
            set_btn.setStyleSheet(
                f"background-color: {C_SURFACE0}; color: {C_GREEN}; "
                f"border: 1px solid {C_GREEN};"
            )
            set_btn.clicked.connect(
                lambda _checked=False, n=name, g=get_fn: self._set_one_param(n, g())
            )
            self._params_grid.addWidget(set_btn, row, 2)

            self._params_editors[name] = (editor, get_fn, set_btn)

    @staticmethod
    def _make_param_editor(value):
        """Return (widget, get_value_callable) appropriate for ``value`` type."""
        if isinstance(value, bool):
            w = QCheckBox()
            w.setChecked(value)
            return w, w.isChecked
        if isinstance(value, int):
            w = QSpinBox()
            w.setRange(-2_147_483_648, 2_147_483_647)
            w.setValue(value)
            return w, w.value
        if isinstance(value, float):
            w = QDoubleSpinBox()
            w.setDecimals(6)
            w.setRange(-1e9, 1e9)
            w.setSingleStep(0.01)
            w.setValue(value)
            return w, w.value
        if isinstance(value, str):
            w = QLineEdit(value)
            return w, w.text
        # Fallback (arrays etc.): read-only label.
        w = QLineEdit(repr(value))
        w.setReadOnly(True)
        w.setStyleSheet(f"color: {C_SUBTEXT};")
        return w, lambda: None

    def _set_one_param(self, name: str, value):
        ctrl = self._current_ctrl
        if not ctrl or value is None:
            return
        editor_tuple = self._params_editors.get(name)
        if editor_tuple is not None:
            editor_tuple[2].setEnabled(False)
        threading.Thread(
            target=self._set_one_worker, args=(ctrl, name, value), daemon=True
        ).start()

    def _set_one_worker(self, ctrl, name, value):
        print(f"[pvt_tuner] set_one {name}={value!r}",
              file=sys.stderr, flush=True)
        results = self._rclpy_set_params(ctrl, [(name, value)])
        if results:
            n, ok, msg = results[0]
            self._param_set_sig.emit(n, ok, msg)
        else:
            self._param_set_sig.emit(name, False, "no response")

    def _on_param_set(self, name: str, ok: bool, msg: str):
        editor_tuple = self._params_editors.get(name)
        if editor_tuple is None:
            return
        editor, _, set_btn = editor_tuple
        set_btn.setEnabled(True)
        # Flash the Set button colour to give feedback.
        color = C_GREEN if ok else C_RED
        set_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {color}; "
            f"border: 1px solid {color};"
        )
        self._params_tab_status.setText(
            f"{name}: {'OK' if ok else 'FAIL'}" + (f" — {msg}" if msg else "")
        )
        self._params_tab_status.setStyleSheet(f"color: {color};")

    def _rclpy_list_params(self, ctrl: str, timeout: float = 5.0):
        client = self._node.create_client(ListParameters, f"{ctrl}/list_parameters")
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return []
            req = ListParameters.Request()
            req.depth = 0  # all
            future = client.call_async(req)
            deadline = time.time() + timeout
            while not future.done() and time.time() < deadline:
                time.sleep(0.02)
            if not future.done():
                return []
            resp = future.result()
            return list(resp.result.names)
        finally:
            self._node.destroy_client(client)


# ───────────────────────────────────────────────────────────
# main
# ───────────────────────────────────────────────────────────


def main():
    rclpy.init(args=sys.argv)
    node = Node("pvt_tuner_gui")

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    app = QApplication(sys.argv)
    win = PvtTunerWindow(node)
    win.show()
    exit_code = app.exec_()

    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
