"""
Interactive diagnostic plotter — docked inside the Isaac Sim window.

Uses Isaac Lab's native LiveLinePlot widget (omni.ui).
No subprocess, no browser, no JSON files, no port conflicts.

Features (built into LiveLinePlot):
  - Pause / Play button
  - Autoscale or manual Y-axis range
  - Lowpass / Integrate / Derivative filter
  - Per-series legend with visibility toggles

Headless mode: omni.ui is unavailable → plotter silently disables itself.
"""

from __future__ import annotations

_WINDOW_TITLE = "Robot Diagnostics"


class RealtimePlotter:

    def __init__(
        self,
        enabled:    bool  = True,
        window_sec: float = 10.0,
        dt:         float = 0.02,
        update_hz:  float = 10.0,   # kept for API compatibility, unused
    ) -> None:
        self.enabled = False
        if not enabled:
            return

        try:
            import omni.ui as ui
            from isaaclab.ui.widgets import LiveLinePlot
        except ImportError:
            # Headless mode — omni.ui not available, silently disabled
            return

        n = max(1, int(window_sec / dt))

        self._window = ui.Window(_WINDOW_TITLE, width=520, height=1060)
        with self._window.frame:
            with ui.VStack(spacing=4):
                ui.Label(
                    f"Rolling window: {window_sec:.0f} s  "
                    f"({n} pts @ {dt * 1000:.0f} ms/step)",
                    height=18,
                    style={"color": 0xFF888888},
                )

                self._p_tilt = LiveLinePlot(
                    y_data=[[], []],
                    y_min=-60, y_max=60,
                    plot_height=120,
                    legends=["tilt (deg)", "theta_des (deg)"],
                    max_datapoints=n,
                )
                self._p_tilt_rate = LiveLinePlot(
                    y_data=[[]],
                    y_min=-5, y_max=5,
                    plot_height=100,
                    legends=["tilt_rate (rad/s)"],
                    max_datapoints=n,
                )
                self._p_yaw = LiveLinePlot(
                    y_data=[[]],
                    y_min=-5, y_max=5,
                    plot_height=100,
                    legends=["yaw_rate (rad/s)"],
                    max_datapoints=n,
                )
                self._p_torque = LiveLinePlot(
                    y_data=[[], []],
                    y_min=-5, y_max=5,
                    plot_height=120,
                    legends=["tau_L (Nm)", "tau_R (Nm)"],
                    max_datapoints=n,
                )
                self._p_vel = LiveLinePlot(
                    y_data=[[], []],
                    y_min=-2, y_max=2,
                    plot_height=100,
                    legends=["vel_cmd (m/s)", "vel_fwd (m/s)"],
                    max_datapoints=n,
                )
                # ── ETASMC diagnostics ────────────────────────────────────────
                self._p_surfaces = LiveLinePlot(
                    y_data=[[], []],
                    y_min=-1.5, y_max=1.5,
                    plot_height=120,
                    legends=["s_bal (rad)", "s_yaw (rad)"],
                    max_datapoints=n,
                )
                self._p_events = LiveLinePlot(
                    y_data=[[], []],
                    y_min=-0.1, y_max=1.1,
                    plot_height=80,
                    legends=["trig_bal", "trig_yaw"],
                    max_datapoints=n,
                )
                self._p_iet = LiveLinePlot(
                    y_data=[[], []],
                    y_min=0.0, y_max=1.0,
                    plot_height=100,
                    legends=["IET_bal (s)", "IET_yaw (s)"],
                    max_datapoints=n,
                )
                self._p_pos = LiveLinePlot(
                    y_data=[[]],
                    y_min=-2.0, y_max=2.0,
                    plot_height=80,
                    legends=["x_odom (m)"],
                    max_datapoints=n,
                )

        self.enabled = True

    # ── Public API ─────────────────────────────────────────────────────────────

    def push(self, data: dict) -> None:
        """Push one timestep of data. Call once per simulation step.

        Expected keys (all optional, default 0):
            tilt, theta_des   [degrees]
            tilt_rate         [rad/s]
            yaw_rate          [rad/s]
            torque_L, torque_R [Nm]
            vel_cmd, vel_fwd  [m/s]
            s_bal, s_yaw      [rad]      — sliding surfaces
            trig_bal, trig_yaw [0/1]     — event trigger flags
            iet_bal, iet_yaw  [s]        — inter-event times
            x_odom            [m]        — wheel odometry position
        """
        if not self.enabled:
            return
        g = data.get
        self._p_tilt.add_datapoint([g("tilt", 0.0),     g("theta_des", 0.0)])
        self._p_tilt_rate.add_datapoint([g("tilt_rate", 0.0)])
        self._p_yaw.add_datapoint([g("yaw_rate",  0.0)])
        self._p_torque.add_datapoint([g("torque_L", 0.0), g("torque_R", 0.0)])
        self._p_vel.add_datapoint([g("vel_cmd",   0.0),  g("vel_fwd",  0.0)])
        self._p_surfaces.add_datapoint([g("s_bal", 0.0), g("s_yaw", 0.0)])
        self._p_events.add_datapoint([g("trig_bal", 0.0), g("trig_yaw", 0.0)])
        self._p_iet.add_datapoint([g("iet_bal", 0.0), g("iet_yaw", 0.0)])
        self._p_pos.add_datapoint([g("x_odom", 0.0)])

    def close(self) -> None:
        if self.enabled:
            self._window.destroy()
            self.enabled = False
