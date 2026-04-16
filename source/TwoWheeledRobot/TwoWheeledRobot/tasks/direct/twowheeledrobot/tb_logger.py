"""
TensorBoard logger for the Two-Wheeled Robot simulation.

Logs all controller diagnostics (tilt, sliding surfaces, torques, event rates,
adaptive gains, etc.) as TensorBoard scalars so you can inspect training/tuning
runs in the browser.

Usage:
    tensorboard --logdir logs/tensorboard
"""

from __future__ import annotations

import os
from datetime import datetime

# Anchor log output to the TwoWheeledRobot project root regardless of where
# the launching script is called from.
# tb_logger.py lives at:
#   <project_root>/source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/
# so 6 parent dirs up == project root.
_HERE         = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_HERE, *[".."] * 6))
_DEFAULT_LOG_BASE = os.path.join(_PROJECT_ROOT, "logs", "tensorboard")


class TBLogger:
    """Lightweight wrapper around SummaryWriter.

    Call push(step, data) once per simulation step with the dict returned by
    controller.last.  Logging is throttled to log_every_n steps to avoid
    overwhelming the writer.

    Mirrors the RealtimePlotter API so both can be kept in sync easily.
    """

    def __init__(
        self,
        log_dir:      str | None = None,
        log_every_n:  int        = 10,
        enabled:      bool       = True,
    ) -> None:
        self.enabled = False
        if not enabled:
            return

        try:
            from torch.utils.tensorboard import SummaryWriter
        except ImportError:
            print("[TBLogger] tensorboard not found — logging disabled.")
            return

        if log_dir is None:
            stamp   = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            log_dir = os.path.join(_DEFAULT_LOG_BASE, stamp)

        os.makedirs(log_dir, exist_ok=True)
        self._writer       = SummaryWriter(log_dir=log_dir)
        self._log_every_n  = max(1, log_every_n)
        self._step_counter = 0
        self.enabled       = True
        print(f"[TBLogger] Writing to {os.path.abspath(log_dir)}")
        print(f"[TBLogger] View with:  tensorboard --logdir {os.path.abspath(log_dir)}")

    # ── Public API ─────────────────────────────────────────────────────────────

    def push(self, data: dict) -> None:
        """Log one simulation step.

        Expected keys (all optional):
            tilt, theta_des   [degrees]
            tilt_rate         [rad/s]
            yaw_rate          [rad/s]
            torque_L, torque_R [Nm]
            vel_cmd, vel_fwd  [m/s]
            s_bal, s_yaw      [rad]   — sliding surfaces
            trig_bal, trig_yaw [0/1]  — event trigger flags
            iet_bal, iet_yaw  [s]     — inter-event times
            k_bal             [A]     — adaptive balance gain
            x_odom            [m]     — wheel odometry
            u_bal, u_yaw      [A]     — held control outputs
        """
        if not self.enabled:
            return

        self._step_counter += 1
        if self._step_counter % self._log_every_n != 0:
            return

        g   = data.get
        s   = self._step_counter
        add = self._writer.add_scalar

        # ── Pose / state ────────────────────────────────────────────────────
        add("state/tilt_deg",       g("tilt",      0.0), s)
        add("state/tilt_rate",      g("tilt_rate", 0.0), s)
        add("state/yaw_rate",       g("yaw_rate",  0.0), s)
        add("state/x_odom_m",       g("x_odom",    0.0), s)

        # ── Velocity ────────────────────────────────────────────────────────
        add("velocity/cmd_mps",     g("vel_cmd",  0.0), s)
        add("velocity/fwd_mps",     g("vel_fwd",  0.0), s)
        add("velocity/error_mps",   g("vel_cmd",  0.0) - g("vel_fwd", 0.0), s)

        # ── Torques / currents ───────────────────────────────────────────────
        add("actuator/torque_L_Nm", g("torque_L", 0.0), s)
        add("actuator/torque_R_Nm", g("torque_R", 0.0), s)
        add("actuator/u_bal_A",     g("u_bal",    0.0), s)
        add("actuator/u_yaw_A",     g("u_yaw",    0.0), s)

        # ── PETASMC — sliding surfaces ───────────────────────────────────────
        add("smc/s_bal_rad",        g("s_bal",    0.0), s)
        add("smc/s_yaw_rad",        g("s_yaw",    0.0), s)

        # ── PETASMC — event diagnostics ─────────────────────────────────────
        add("event/trig_bal",       g("trig_bal", 0.0), s)
        add("event/trig_yaw",       g("trig_yaw", 0.0), s)
        add("event/iet_bal_s",      g("iet_bal",  0.0), s)
        add("event/iet_yaw_s",      g("iet_yaw",  0.0), s)

        # ── Adaptive gain ────────────────────────────────────────────────────
        add("gain/k_bal_A",         g("k_bal",    0.0), s)

    def close(self) -> None:
        if self.enabled:
            self._writer.flush()
            self._writer.close()
            self.enabled = False
