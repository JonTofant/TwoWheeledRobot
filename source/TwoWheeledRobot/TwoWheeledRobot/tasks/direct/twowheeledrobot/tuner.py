"""
Real-time SMC parameter tuner — docked inside the Isaac Sim window.

Opens an omni.ui window with sliders for every tunable gain in the SMC
controller. Moving a slider takes effect immediately — no restart needed.

Headless mode: omni.ui is unavailable → tuner silently disables itself.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .control import RobotController

_WINDOW_TITLE = "SMC Parameter Tuner"

# (attr_name, label, min, max, step, display_format)
_BALANCE_PARAMS = [
    ("smc_alpha",      "α  (tilt-rate)",    0.000, 0.500, 0.001, "{:.3f}"),
    ("smc_beta",       "β  (vel-error)",    0.000, 3.000, 0.010, "{:.3f}"),
    ("smc_phi",        "φ  (boundary)",     0.010, 1.000, 0.010, "{:.3f}"),
    ("et_gamma",       "γ  (pos-hold)",     0.000, 0.500, 0.001, "{:.3f}"),
]

_HEADING_PARAMS = [
    ("smc_yaw_lambda", "λ  (yaw-rate)",     0.000, 1.000, 0.010, "{:.3f}"),
    ("smc_yaw_phi",    "φ_yaw (boundary)",  0.010, 1.000, 0.010, "{:.3f}"),
]

_GAIN_PARAMS = [
    ("et_k_max_bal",   "K_max_bal  [A]",    0.100, 5.000, 0.050, "{:.3f}"),
    ("et_k_min_bal",   "K_min_bal  [A]",    0.000, 3.000, 0.050, "{:.3f}"),
    ("et_k_sigma_bal", "σ_bal  [rad]",      0.010, 1.000, 0.010, "{:.3f}"),
    ("et_k_max_yaw",   "K_max_yaw  [A]",    0.000, 1.000, 0.010, "{:.3f}"),
    ("et_k_min_yaw",   "K_min_yaw  [A]",    0.000, 0.500, 0.005, "{:.3f}"),
    ("et_k_sigma_yaw", "σ_yaw  [rad]",      0.010, 1.000, 0.010, "{:.3f}"),
]

_TRIGGER_PARAMS = [
    ("et_eta_bal",     "η_bal  (relative)", 0.000, 1.000, 0.010, "{:.3f}"),
    ("et_eps_bal",     "ε_bal  [rad]",      0.000, 0.100, 0.001, "{:.4f}"),
    ("et_eta_yaw",     "η_yaw  (relative)", 0.000, 1.000, 0.010, "{:.3f}"),
    ("et_eps_yaw",     "ε_yaw  [rad]",      0.000, 0.100, 0.001, "{:.4f}"),
    ("et_min_iet_bal", "T_min_bal  [s]",    0.005, 0.100, 0.001, "{:.3f}"),
    ("et_min_iet_yaw", "T_min_yaw  [s]",    0.005, 0.100, 0.001, "{:.3f}"),
]

_ALL_SECTIONS = [
    ("Balance Surface",  _BALANCE_PARAMS),
    ("Heading Surface",  _HEADING_PARAMS),
    ("Adaptive Gains",   _GAIN_PARAMS),
    ("Event Triggers",   _TRIGGER_PARAMS),
]


class SMCTuner:

    def __init__(self, controller: "RobotController", enabled: bool = True) -> None:
        self.enabled = False
        if not enabled:
            return

        try:
            import omni.ui as ui
        except ImportError:
            # Headless mode — omni.ui unavailable, silently disabled
            return

        self._controller = controller
        self._sliders: dict[str, object] = {}   # attr → FloatSlider widget
        self._labels:  dict[str, object] = {}   # attr → Label widget (value read-back)

        self._window = ui.Window(
            _WINDOW_TITLE,
            width=440,
            height=700,
        )

        with self._window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=6):
                    ui.Spacer(height=4)

                    for section_title, params in _ALL_SECTIONS:
                        frame = ui.CollapsableFrame(
                            section_title,
                            collapsed=False,
                            style={"font_size": 14},
                        )
                        with frame:
                            with ui.VStack(spacing=2):
                                ui.Spacer(height=2)
                                for (attr, label, lo, hi, step, fmt) in params:
                                    self._add_row(ui, controller, attr, label, lo, hi, step, fmt)
                                ui.Spacer(height=4)

                    ui.Spacer(height=6)

                    # Reset to defaults button
                    btn = ui.Button(
                        "Reset to Defaults",
                        height=30,
                        style={"background_color": 0xFF444444},
                    )
                    btn.set_clicked_fn(self._reset_to_defaults)

                    ui.Spacer(height=4)

        self.enabled = True

    # ── Private helpers ────────────────────────────────────────────────────────

    def _add_row(self, ui, controller, attr, label, lo, hi, step, fmt) -> None:
        """Add a labelled slider row for one parameter."""
        current_val = getattr(controller, attr)

        with ui.HStack(height=22, spacing=4):
            ui.Label(label, width=150, style={"font_size": 12})

            slider = ui.FloatSlider(
                min=lo,
                max=hi,
                step=step,
            )
            slider.model.set_value(current_val)

            readback = ui.Label(
                fmt.format(current_val),
                width=60,
                style={"font_size": 12, "color": 0xFFCCCCCC},
            )

        # Capture loop variables explicitly
        def _on_change(model, a=attr, r=readback, f=fmt):
            val = model.get_value_as_float()
            setattr(controller, a, val)
            r.text = f.format(val)

        slider.model.add_value_changed_fn(_on_change)

        self._sliders[attr] = slider
        self._labels[attr]  = readback

    def _reset_to_defaults(self) -> None:
        """Restore every parameter to the module-level default and refresh sliders."""
        from .control import (
            SMC_ALPHA, SMC_BETA, SMC_PHI, ET_GAMMA,
            SMC_YAW_LAMBDA, SMC_YAW_PHI,
            ET_K_MAX_BAL, ET_K_MIN_BAL, ET_K_SIGMA_BAL,
            ET_K_MAX_YAW, ET_K_MIN_YAW, ET_K_SIGMA_YAW,
            ET_ETA_BAL, ET_EPS_BAL, ET_ETA_YAW, ET_EPS_YAW,
            ET_MIN_IET_BAL, ET_MIN_IET_YAW,
        )
        defaults = {
            "smc_alpha":      SMC_ALPHA,
            "smc_beta":       SMC_BETA,
            "smc_phi":        SMC_PHI,
            "et_gamma":       ET_GAMMA,
            "smc_yaw_lambda": SMC_YAW_LAMBDA,
            "smc_yaw_phi":    SMC_YAW_PHI,
            "et_k_max_bal":   ET_K_MAX_BAL,
            "et_k_min_bal":   ET_K_MIN_BAL,
            "et_k_sigma_bal": ET_K_SIGMA_BAL,
            "et_k_max_yaw":   ET_K_MAX_YAW,
            "et_k_min_yaw":   ET_K_MIN_YAW,
            "et_k_sigma_yaw": ET_K_SIGMA_YAW,
            "et_eta_bal":     ET_ETA_BAL,
            "et_eps_bal":     ET_EPS_BAL,
            "et_eta_yaw":     ET_ETA_YAW,
            "et_eps_yaw":     ET_EPS_YAW,
            "et_min_iet_bal": ET_MIN_IET_BAL,
            "et_min_iet_yaw": ET_MIN_IET_YAW,
        }
        for attr, val in defaults.items():
            setattr(self._controller, attr, val)
            if attr in self._sliders:
                self._sliders[attr].model.set_value(val)

    def close(self) -> None:
        if self.enabled:
            self._window.destroy()
            self.enabled = False
