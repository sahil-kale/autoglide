import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from simulator.constants import VisualizerParams
from simulator.utils.airplane_glyph import draw_airplane


class SingleThermalSimVisualizer:
    def __init__(self, params: VisualizerParams):
        if params.headless:
            matplotlib.use("Agg")
        self.fig = plt.figure(figsize=(16, 8))
        if params.headless:
            from matplotlib.backends.backend_agg import FigureCanvasAgg

            self.fig.canvas = FigureCanvasAgg(self.fig)
        gs = matplotlib.gridspec.GridSpec(
            6, 2, width_ratios=[2, 1], height_ratios=[1, 1, 1, 1, 1, 1]
        )
        self.ax = self.fig.add_subplot(gs[:, 0], projection="3d")
        self.scope_axes = []
        for i in range(len(params.scope_labels)):
            ax = self.fig.add_subplot(gs[i, 1])
            ax.set_ylabel(params.scope_labels[i])
            self.scope_axes.append(ax)
        self.scope_axes[-1].set_xlabel("Time (s)")
        self.fig.tight_layout()
        self.params = params
        self._frames = [] if params.headless and params.video_save_path else None
        self._step_count = 0
        # Store state for plotting
        self.xs = []
        self.ys = []
        self.hs = []
        self.times = []
        self.scope_data = {label: [] for label in params.scope_labels}

    def draw(self, loggedstate):
        # Update internal state with new data
        self.xs.append(loggedstate.glider_x)
        self.ys.append(loggedstate.glider_y)
        self.hs.append(loggedstate.glider_h)
        self.times.append(loggedstate.time)
        # For scope_data, update each label if present in loggedstate
        for label in self.scope_data:
            val = getattr(loggedstate, self._label_to_attr(label), None)
            if val is not None:
                self.scope_data[label].append(val)
            else:
                # Fallback: try to append NaN if not present
                self.scope_data[label].append(np.nan)

        SCOPE_UPDATE_EVERY = 10
        update_oscopes = self._step_count % SCOPE_UPDATE_EVERY == 0
        self.ax.clear()
        self.ax.plot(self.xs, self.ys, self.hs, linewidth=2, label="Path")
        # Plot actual thermal core (blue)
        if (
            hasattr(loggedstate, "actual_thermal_x")
            and hasattr(loggedstate, "actual_thermal_y")
            and hasattr(loggedstate, "actual_thermal_radius")
            and loggedstate.actual_thermal_radius is not None
        ):
            theta = np.linspace(0, 2 * np.pi, 100)
            actual_ring_x = (
                loggedstate.actual_thermal_x
                + loggedstate.actual_thermal_radius * np.cos(theta)
            )
            actual_ring_y = (
                loggedstate.actual_thermal_y
                + loggedstate.actual_thermal_radius * np.sin(theta)
            )
            actual_ring_z = np.full_like(theta, loggedstate.glider_h)
            self.ax.plot(
                actual_ring_x,
                actual_ring_y,
                actual_ring_z,
                "b--",
                linewidth=1.5,
                label="Thermal Core",
            )
            self.ax.scatter(
                loggedstate.actual_thermal_x,
                loggedstate.actual_thermal_y,
                loggedstate.glider_h,
                color="blue",
                marker="o",
                s=60,
                label="Core Center",
            )
        # Plot estimated thermal core (red)
        if self.params.plot_estimated_thermal_params:
            if (
                loggedstate.est_thermal_x is not None
                and loggedstate.est_thermal_y is not None
                and loggedstate.est_thermal_radius is not None
            ):
                theta = np.linspace(0, 2 * np.pi, 100)
                est_ring_x = (
                    loggedstate.est_thermal_x
                    + loggedstate.est_thermal_radius * np.cos(theta)
                )
                est_ring_y = (
                    loggedstate.est_thermal_y
                    + loggedstate.est_thermal_radius * np.sin(theta)
                )
                est_ring_z = np.full_like(theta, loggedstate.glider_h)
                self.ax.plot(
                    est_ring_x,
                    est_ring_y,
                    est_ring_z,
                    "r--",
                    linewidth=2,
                    label="Estimated Core",
                )
                self.ax.scatter(
                    loggedstate.est_thermal_x,
                    loggedstate.est_thermal_y,
                    loggedstate.glider_h,
                    color="red",
                    marker="x",
                    s=80,
                    label="Est Center",
                )
        draw_airplane(
            self.ax,
            x=loggedstate.glider_x,
            y=loggedstate.glider_y,
            z=loggedstate.glider_h,
            psi=loggedstate.glider_psi,
            phi=loggedstate.glider_phi,
            scale=self.params.airplane_scale,
        )
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Altitude (m)")
        self.ax.set_title(f"Glider Simulation (t={loggedstate.time:.1f}s)", y=0.9999)
        self.ax.legend()
        self.ax.set_xlim(
            loggedstate.glider_x - self.params.plot_span_xy,
            loggedstate.glider_x + self.params.plot_span_xy,
        )
        self.ax.set_ylim(
            loggedstate.glider_y - self.params.plot_span_xy,
            loggedstate.glider_y + self.params.plot_span_xy,
        )
        self.ax.set_zlim(
            loggedstate.glider_h - self.params.plot_span_z,
            loggedstate.glider_h + self.params.plot_span_z,
        )
        if update_oscopes:
            t_window = self.params.scope_window_sec
            t_now = loggedstate.time
            times_arr = np.array(self.times)
            idx_start = np.searchsorted(times_arr, t_now - t_window)
            for ax, label in zip(self.scope_axes, self.scope_data):
                ax.clear()
                ax.plot(
                    times_arr[idx_start:], np.array(self.scope_data[label])[idx_start:]
                )
                ax.set_ylabel(label)
            self.scope_axes[-1].set_xlabel("Time (s)")
        if self.params.headless:
            if self._frames is not None:
                self.fig.canvas.draw()
                buf = self.fig.canvas.buffer_rgba()
                frame = np.asarray(buf, dtype=np.uint8).reshape(
                    self.fig.canvas.get_width_height()[::-1] + (4,)
                )
                frame = frame[:, :, :3]
                self._frames.append(frame.copy())
        else:
            plt.pause(0.001)

        self._step_count += 1

    def _label_to_attr(self, label):
        # Map scope label to LoggedState attribute name
        # This mapping may need to be customized based on label names
        label_map = {
            "Altitude (m)": "glider_h",
            "Airspeed (m/s)": "glider_V",
            "Roll (deg)": "glider_phi",
            "Uplift Speed (m/s)": "disturbance_w",
            "Estimator Confidence": "estimator_confidence",
            "Guidance State": "guidance_state",
        }
        return label_map.get(label, label)

    def finalize_video(self):
        if self._frames is not None and self.params.video_save_path:
            import imageio

            imageio.mimsave(self.params.video_save_path, self._frames, fps=10)
