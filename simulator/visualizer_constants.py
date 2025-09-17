class VisualizerParams:
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
        self.scope_labels = scope_labels
        self.airplane_scale = airplane_scale
        self.plot_span_xy = plot_span_xy
        self.plot_span_z = plot_span_z
        self.plot_estimated_thermal_params = plot_estimated_thermal_params
        self.scope_window_sec = scope_window_sec
        self.headless = headless
        self.video_save_path = video_save_path
