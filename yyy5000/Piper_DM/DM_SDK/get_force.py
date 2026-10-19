#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
dmrobotics_live_wrench_dpg.py

Live visualization (Dear PyGui) for a 6D wrench:
  - Force:  Fx, Fy, Fz
  - Torque: Mx, My, Mz

UI behavior:
  1) The main window fills the whole viewport (no tiny-corner start).
  2) 2x3 grid auto-resizes with the viewport.
  3) Y-axis: minimum [-1, +1], auto expands with data.
  4) Reset: button + hotkey 'R' calls sensor.reset().
  5) Bottom status area is always visible (no scrolling needed).
"""

import time
from collections import deque

import numpy as np
import dearpygui.dearpygui as dpg

from dmrobotics import Sensor, SensorOptions, Mode


def _force_to_wrench(force_arr):
    """Normalize getForce() output into (fx, fy, fz, mx, my, mz)."""
    if isinstance(force_arr, (tuple, list)) and len(force_arr) == 2:
        force_arr = force_arr[1]
    f = np.asarray(force_arr, dtype=np.float32).reshape(-1)
    if f.size < 6:
        raise ValueError(f"wrench size < 6, got shape={np.asarray(force_arr).shape}")
    return tuple(map(float, f[:6]))


def _compute_ylim(ybuf, min_abs=1.0, margin_ratio=0.08, min_margin=1e-3):
    """Y-limits: at least [-1, 1], expand with data + margin."""
    if len(ybuf) == 0:
        return -min_abs, +min_abs

    y = np.asarray(ybuf, dtype=np.float64)
    y_min = float(np.min(y))
    y_max = float(np.max(y))

    span = max(y_max - y_min, min_margin)
    margin = max(span * margin_ratio, min_margin)

    lo = min(-min_abs, y_min - margin)
    hi = max(+min_abs, y_max + margin)

    # ensure minimum span
    if hi - lo < 2 * min_abs:
        lo = min(lo, -min_abs)
        hi = max(hi, +min_abs)

    return lo, hi


def main():
    # ====== Tune these parameters as needed ======
    dev_id = 2
    backend = "cuda"          # "cpu" / "cuda"
    max_fps = 120
    max_points = 1200
    update_hz = 120
    reset_cooldown_s = 0.8
    # ===========================================

    opt = SensorOptions(
        dev_id=dev_id,
        backend=backend,
        mode=Mode.STANDARD,
        show_fps=False,
        max_fps=max_fps,
        enable_raw=False,
        enable_deformation=False,
        enable_shear=False,
        enable_depth=False,
        enable_force=True,
    )
    sensor = Sensor(opt)

    channels = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
    tbuf = deque(maxlen=max_points)
    bufs = {k: deque(maxlen=max_points) for k in channels}

    # ---------- Dear PyGui setup ----------
    dpg.create_context()
    dpg.create_viewport(
        title="dmrobotics Live Wrench (Fx/Fy/Fz + Mx/My/Mz)",
        width=1280,
        height=860,
        resizable=True,
    )

    xaxis_ids = {}
    yaxis_ids = {}
    series_ids = {}

    last_reset_ts = 0.0
    pending_reset = False

    # --- UI items ---
    status_text = None
    fps_text = None
    last_text = None

    def do_reset(reason: str):
        nonlocal last_reset_ts, pending_reset
        now = time.time()
        if now - last_reset_ts < reset_cooldown_s:
            dpg.set_value(status_text, f"Reset skipped (cooldown {reset_cooldown_s:.1f}s).")
            return

        pending_reset = True
        try:
            sensor.reset()
            last_reset_ts = now
            dpg.set_value(status_text, f"Reset triggered ({reason}). Keep sensor surface free of contact.")
        except Exception as e:
            dpg.set_value(status_text, f"[WARN] Reset failed: {e}")
        finally:
            pending_reset = False

    def on_reset_button(sender, app_data, user_data):
        do_reset("button")

    def on_key_press(sender, app_data):
        # app_data is key code
        if app_data in (ord("r"), ord("R")):
            do_reset("hotkey")

    def build_plot(parent, name: str):
        with dpg.plot(label=name, height=-1, width=-1, parent=parent):
            xaxis = dpg.add_plot_axis(dpg.mvXAxis, label="t (s)")
            yaxis = dpg.add_plot_axis(dpg.mvYAxis, label=name)
            series = dpg.add_line_series([], [], label=name, parent=yaxis)
        return xaxis, yaxis, series

    # Main window (will be made primary so it fills viewport)
    with dpg.window(
        label="Wrench Plots",
        tag="MAIN_WINDOW",
        no_title_bar=True,
        no_move=True,
        no_resize=True,
        no_collapse=True,
    ):
        # Plots area (leave fixed height for bottom status)
        with dpg.child_window(tag="PLOTS_AREA", border=True, height=-170):
            with dpg.group(tag="GRID_ROOT"):
                with dpg.group(horizontal=True, tag="ROW0"):
                    with dpg.child_window(tag="CELL_00", border=True): pass
                    with dpg.child_window(tag="CELL_01", border=True): pass
                    with dpg.child_window(tag="CELL_02", border=True): pass
                with dpg.group(horizontal=True, tag="ROW1"):
                    with dpg.child_window(tag="CELL_10", border=True): pass
                    with dpg.child_window(tag="CELL_11", border=True): pass
                    with dpg.child_window(tag="CELL_12", border=True): pass

            cell_tags = ["CELL_00", "CELL_01", "CELL_02", "CELL_10", "CELL_11", "CELL_12"]
            for cell, ch in zip(cell_tags, channels):
                xaxis, yaxis, series = build_plot(cell, ch)
                xaxis_ids[ch] = xaxis
                yaxis_ids[ch] = yaxis
                series_ids[ch] = series

        # Bottom status area (always visible)
        with dpg.child_window(tag="STATUS_AREA", border=True, height=-1):
            status_text = dpg.add_text("Starting...")
            fps_text = dpg.add_text("UI FPS: -- | Data: --")
            last_text = dpg.add_text("Last: --")
            dpg.add_separator()
            dpg.add_button(label="Reset (R)", callback=on_reset_button)
            dpg.add_text(
                "Tips: Press 'R' to reset. "
                "Y-axis is at least [-1, 1] and auto-expands. Close window to exit.",
                color=(200, 200, 200),
            )

    # Key handler
    with dpg.handler_registry():
        dpg.add_key_press_handler(callback=on_key_press)

    # Disable vsync (may still be limited by compositor/driver)
    dpg.set_viewport_vsync(False)

    dpg.setup_dearpygui()
    dpg.show_viewport()

    # Make MAIN_WINDOW fill the viewport
    dpg.set_primary_window("MAIN_WINDOW", True)

    def layout_grid():
        """Resize 2x3 cell grid to fill PLOTS_AREA."""
        try:
            w, h = dpg.get_item_rect_size("PLOTS_AREA")
        except Exception:
            return
        if w <= 0 or h <= 0:
            return

        pad = 8
        cell_w = max(80, int((w - pad * 2) / 3))
        cell_h = max(80, int((h - pad * 1) / 2))

        for tag in ["CELL_00", "CELL_01", "CELL_02", "CELL_10", "CELL_11", "CELL_12"]:
            dpg.configure_item(tag, width=cell_w, height=cell_h)

    def fit_main_window_to_viewport():
        """Force MAIN_WINDOW to match the viewport client size."""
        vw = dpg.get_viewport_client_width()
        vh = dpg.get_viewport_client_height()
        dpg.configure_item("MAIN_WINDOW", pos=(0, 0), width=vw, height=vh)

    def on_viewport_resize(sender, app_data):
        # called by DPG when viewport size changes
        fit_main_window_to_viewport()
        layout_grid()

    dpg.set_viewport_resize_callback(on_viewport_resize)

    # Ensure correct sizing at startup (before first frames)
    fit_main_window_to_viewport()

    # ---------- Main loop ----------
    t0 = time.time()
    last_plot_update = 0.0

    ui_frame_cnt = 0
    ui_fps_t0 = time.time()
    ui_fps = 0.0

    data_cnt = 0
    data_fps_t0 = time.time()
    data_fps = 0.0

    # One-time layout after the first frame (rect sizes become valid)
    did_first_layout = False

    try:
        while dpg.is_dearpygui_running():
            now = time.time()

            # Read data
            w = sensor.getForce()
            fx, fy, fz, mx, my, mz = _force_to_wrench(w)

            t = now - t0
            tbuf.append(t)
            bufs["Fx"].append(fx)
            bufs["Fy"].append(fy)
            bufs["Fz"].append(fz)
            bufs["Mx"].append(mx)
            bufs["My"].append(my)
            bufs["Mz"].append(mz)

            data_cnt += 1
            if now - data_fps_t0 >= 1.0:
                data_fps = data_cnt / (now - data_fps_t0)
                data_cnt = 0
                data_fps_t0 = now

            # Update plots at update_hz
            if now - last_plot_update >= 1.0 / max(1, update_hz):
                xs = list(tbuf)

                for ch in channels:
                    ys = list(bufs[ch])
                    dpg.set_value(series_ids[ch], [xs, ys])

                    if len(xs) >= 2:
                        dpg.set_axis_limits(xaxis_ids[ch], xs[0], xs[-1])

                    lo, hi = _compute_ylim(bufs[ch], min_abs=1.0)
                    dpg.set_axis_limits(yaxis_ids[ch], lo, hi)

                dpg.set_value(
                    last_text,
                    f"Last  Fx={fx:+.3f}  Fy={fy:+.3f}  Fz={fz:+.3f}   "
                    f"Mx={mx:+.3f}  My={my:+.3f}  Mz={mz:+.3f}"
                )

                if not pending_reset and (now - last_reset_ts) > 2.0:
                    dpg.set_value(status_text, "Running...")

                last_plot_update = now

            # Render frame
            dpg.render_dearpygui_frame()

            # After first render, sizes become valid -> do a layout once
            if not did_first_layout:
                layout_grid()
                did_first_layout = True

            # UI fps display
            ui_frame_cnt += 1
            if now - ui_fps_t0 >= 1.0:
                ui_fps = ui_frame_cnt / (now - ui_fps_t0)
                ui_frame_cnt = 0
                ui_fps_t0 = now
                dpg.set_value(fps_text, f"UI FPS: {ui_fps:.1f} | Data rate: {data_fps:.1f} Hz")

            time.sleep(0.0005)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            sensor.disconnect()
        except Exception:
            pass
        dpg.destroy_context()


if __name__ == "__main__":
    main()
