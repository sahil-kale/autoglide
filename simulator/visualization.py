import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from simulator.visualizer_constants import VisualizerParams
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

    def draw(self, loggedstate):
        SCOPE_UPDATE_EVERY = 10
        update_oscopes = self._step_count % SCOPE_UPDATE_EVERY == 0
        self.ax.clear()
        self.ax.plot(
            loggedstate.xs, loggedstate.ys, loggedstate.hs, linewidth=2, label="Path"
        )
        if loggedstate.thermal is not None:
            x_c, y_c = loggedstate.thermal.core_center_at_height(loggedstate.glider_h)
            r_th = loggedstate.thermal.r_th
            theta = np.linspace(0, 2 * np.pi, 100)
            ring_x = x_c + r_th * np.cos(theta)
            ring_y = y_c + r_th * np.sin(theta)
            ring_z = np.full_like(theta, loggedstate.glider_h)
            self.ax.plot(
                ring_x, ring_y, ring_z, "b--", linewidth=1.5, label="Thermal Core"
            )
            hs_drift = np.linspace(
                loggedstate.glider_h - 100, loggedstate.glider_h + 100, 20
            )
            drift_x = []
            drift_y = []
            for h in hs_drift:
                xc, yc = loggedstate.thermal.core_center_at_height(h)
                drift_x.append(xc)
                drift_y.append(yc)
            self.ax.plot(
                drift_x, drift_y, hs_drift, "b:", linewidth=1, label="Core Drift Path"
            )
        if (
            self.params.plot_estimated_thermal_params
            and loggedstate.thermal_estimator is not None
        ):
            thermal_estimate = loggedstate.thermal_estimator.get_estimate()
            est_core = thermal_estimate.get_location()
            est_Rth = thermal_estimate.get_radius()
            theta = np.linspace(0, 2 * np.pi, 100)
            est_ring_x = est_core.x + est_Rth * np.cos(theta)
            est_ring_y = est_core.y + est_Rth * np.sin(theta)
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
                est_core.x,
                est_core.y,
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
        self.ax.set_title(f"Glider Simulation (t={loggedstate.time:.1f}s)")
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
            times_arr = np.array(loggedstate.times)
            idx_start = np.searchsorted(times_arr, t_now - t_window)
            scope_items = list(loggedstate.scope_data.items())
            for ax, (label, data) in zip(self.scope_axes, scope_items):
                ax.clear()
                ax.plot(times_arr[idx_start:], np.array(data)[idx_start:])
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

    def finalize_video(self):
        if self._frames is not None and self.params.video_save_path:
            import imageio

            imageio.mimsave(self.params.video_save_path, self._frames, fps=10)
