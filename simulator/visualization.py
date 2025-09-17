import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np


class SingleThermalSimVisualizer:
    def __init__(
        self,
        scope_labels,
        airplane_scale,
        plot_span_xy,
        plot_span_z,
        plot_estimated_thermal_params,
        scope_window_sec,
        headless=False,
        video_save_path=None,
    ):
        # Use Agg backend for headless mode to ensure offscreen rendering
        if headless:
            matplotlib.use("Agg")
        self.fig = plt.figure(figsize=(16, 8))
        if headless:
            from matplotlib.backends.backend_agg import FigureCanvasAgg

            self.fig.canvas = FigureCanvasAgg(self.fig)
        gs = gridspec.GridSpec(
            6, 2, width_ratios=[2, 1], height_ratios=[1, 1, 1, 1, 1, 1]
        )
        self.ax = self.fig.add_subplot(gs[:, 0], projection="3d")
        self.scope_axes = []
        for i in range(len(scope_labels)):
            ax = self.fig.add_subplot(gs[i, 1])
            ax.set_ylabel(scope_labels[i])
            self.scope_axes.append(ax)
        self.scope_axes[-1].set_xlabel("Time (s)")
        self.fig.tight_layout()
        self.plot_span_xy = plot_span_xy
        self.plot_span_z = plot_span_z
        self.airplane_scale = airplane_scale
        self.plot_estimated_thermal_params = plot_estimated_thermal_params
        self.scope_window_sec = scope_window_sec
        self.headless = headless
        self.video_save_path = video_save_path
        self._frames = [] if headless and video_save_path else None

    def draw(
        self,
        xs,
        ys,
        hs,
        glider,
        scope_data,
        times,
        _time,
        _step_count,
        draw_airplane_func,
        thermal=None,
        thermal_estimator=None,
    ):
        SCOPE_UPDATE_EVERY = 10
        update_oscopes = _step_count % SCOPE_UPDATE_EVERY == 0

        # 3D plot always updates
        self.ax.clear()
        self.ax.plot(xs, ys, hs, linewidth=2, label="Path")

        # Draw thermal core ring at current altitude
        if thermal is not None:
            x_c, y_c = thermal.core_center_at_height(glider.h)
            r_th = thermal.r_th
            theta = np.linspace(0, 2 * np.pi, 100)
            ring_x = x_c + r_th * np.cos(theta)
            ring_y = y_c + r_th * np.sin(theta)
            ring_z = np.full_like(theta, glider.h)
            self.ax.plot(
                ring_x, ring_y, ring_z, "b--", linewidth=1.5, label="Thermal Core"
            )
            # Optionally, plot core drift path (projected)
            hs_drift = np.linspace(glider.h - 100, glider.h + 100, 20)
            drift_x = []
            drift_y = []
            for h in hs_drift:
                xc, yc = thermal.core_center_at_height(h)
                drift_x.append(xc)
                drift_y.append(yc)
            self.ax.plot(
                drift_x, drift_y, hs_drift, "b:", linewidth=1, label="Core Drift Path"
            )

        # Plot estimated thermal center and radius
        if self.plot_estimated_thermal_params and thermal_estimator is not None:
            thermal_estimate = thermal_estimator.get_estimate()
            est_core = thermal_estimate.get_location()
            est_Rth = thermal_estimate.get_radius()
            theta = np.linspace(0, 2 * np.pi, 100)
            est_ring_x = est_core.x + est_Rth * np.cos(theta)
            est_ring_y = est_core.y + est_Rth * np.sin(theta)
            est_ring_z = np.full_like(theta, glider.h)
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
                glider.h,
                color="red",
                marker="x",
                s=80,
                label="Est Center",
            )

        draw_airplane_func(
            self.ax,
            x=glider.x,
            y=glider.y,
            z=glider.h,
            psi=glider.psi,
            phi=glider.phi,
            scale=self.airplane_scale,
        )

        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Altitude (m)")
        self.ax.set_title(f"Glider Simulation (t={_time:.1f}s)")
        self.ax.legend()

        self.ax.set_xlim(glider.x - self.plot_span_xy, glider.x + self.plot_span_xy)
        self.ax.set_ylim(glider.y - self.plot_span_xy, glider.y + self.plot_span_xy)
        self.ax.set_zlim(glider.h - self.plot_span_z, glider.h + self.plot_span_z)

        if update_oscopes:
            # Only plot last scope_window_sec seconds
            t_window = self.scope_window_sec
            t_now = _time
            times_arr = np.array(times)
            idx_start = np.searchsorted(times_arr, t_now - t_window)
            scope_items = list(scope_data.items())
            for ax, (label, data) in zip(self.scope_axes, scope_items):
                ax.clear()
                ax.plot(times_arr[idx_start:], np.array(data)[idx_start:])
                ax.set_ylabel(label)
            self.scope_axes[-1].set_xlabel("Time (s)")

        if self.headless:
            if self._frames is not None:
                # Save current frame to buffer for video using Agg canvas
                self.fig.canvas.draw()
                buf = self.fig.canvas.buffer_rgba()
                frame = np.asarray(buf, dtype=np.uint8).reshape(
                    self.fig.canvas.get_width_height()[::-1] + (4,)
                )
                frame = frame[:, :, :3]  # drop alpha if you want RGB
                self._frames.append(frame.copy())
        else:
            plt.pause(0.001)

    def finalize_video(self):
        if self._frames is not None and self.video_save_path:
            import imageio

            imageio.mimsave(self.video_save_path, self._frames, fps=10)
