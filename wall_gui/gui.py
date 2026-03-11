import os
import sys
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox

from wall_controller import WallController
from patterns import get_pattern_names


class WallGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Kinetic Wall Control")
        self.root.geometry("470x650")
        self.root.resizable(False, False)

        self.controller = WallController()

        self.palm_process = None
        self.depth_process = None

        self.port_var = tk.StringVar(value="/dev/ttyACM0")
        self.baud_var = tk.StringVar(value="9600")
        self.pattern_var = tk.StringVar(value="wave_lr")
        self.delay_var = tk.StringVar(value="5")
        self.repetitions_var = tk.StringVar(value="1")
        self.speed_var = tk.IntVar(value=50)
        self.status_var = tk.StringVar(value="Not connected")

        self.last_port = None
        self.last_baud = None

        self.auto_disconnected_for_external = False
        self.active_external_mode = None  # None, "palm", or "depth"

        self._build_ui()
        self._poll_status()

    def _build_ui(self):
        pad = 10

        main = ttk.Frame(self.root, padding=pad)
        main.pack(fill="both", expand=True)

        # Connection
        conn_frame = ttk.LabelFrame(main, text="Connection", padding=pad)
        conn_frame.pack(fill="x", pady=(0, 10))

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky="w")
        ttk.Entry(conn_frame, textvariable=self.port_var, width=18).grid(row=0, column=1, sticky="w", padx=5)

        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=2, sticky="w", padx=(10, 0))
        ttk.Entry(conn_frame, textvariable=self.baud_var, width=10).grid(row=0, column=3, sticky="w", padx=5)

        ttk.Button(conn_frame, text="Connect", command=self.connect).grid(
            row=1, column=0, columnspan=2, pady=(10, 0), sticky="ew"
        )
        ttk.Button(conn_frame, text="Disconnect", command=self.disconnect).grid(
            row=1, column=2, columnspan=2, pady=(10, 0), sticky="ew"
        )

        # Pattern control
        pattern_frame = ttk.LabelFrame(main, text="Pattern Control", padding=pad)
        pattern_frame.pack(fill="x", pady=(0, 10))

        ttk.Label(pattern_frame, text="Pattern:").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            pattern_frame,
            textvariable=self.pattern_var,
            values=get_pattern_names(),
            state="readonly",
            width=18
        ).grid(row=0, column=1, sticky="w", padx=5)

        ttk.Label(pattern_frame, text="Delay (s):").grid(row=1, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(pattern_frame, textvariable=self.delay_var, width=10).grid(
            row=1, column=1, sticky="w", padx=5, pady=(10, 0)
        )

        ttk.Label(pattern_frame, text="Repetitions:").grid(row=2, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(pattern_frame, textvariable=self.repetitions_var, width=10).grid(
            row=2, column=1, sticky="w", padx=5, pady=(10, 0)
        )

        ttk.Button(pattern_frame, text="Run Selected Pattern", command=self.run_pattern).grid(
            row=3, column=0, columnspan=2, sticky="ew", pady=(12, 0)
        )

        # Command ramp control
        ramp_frame = ttk.LabelFrame(main, text="Command Ramp Control", padding=pad)
        ramp_frame.pack(fill="x", pady=(0, 10))

        ttk.Label(ramp_frame, text="Command Ramp (0–300)").pack(anchor="w")

        ramp_slider = tk.Scale(
            ramp_frame,
            from_=0,
            to=300,
            orient="horizontal",
            variable=self.speed_var,
            length=350
        )
        ramp_slider.pack(anchor="w")

        ttk.Button(ramp_frame, text="Apply Ramp", command=self.apply_ramp).pack(anchor="w", pady=(8, 0))

        # Quick controls
        quick_frame = ttk.LabelFrame(main, text="Quick Controls", padding=pad)
        quick_frame.pack(fill="x", pady=(0, 10))

        ttk.Button(quick_frame, text="Home", command=self.home).grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        ttk.Button(quick_frame, text="All On", command=self.all_on).grid(row=0, column=1, sticky="ew", padx=5, pady=5)
        ttk.Button(quick_frame, text="Stop", command=self.stop).grid(row=0, column=2, sticky="ew", padx=5, pady=5)

        for i in range(3):
            quick_frame.columnconfigure(i, weight=1)

        # Palm tracker
        palm_frame = ttk.LabelFrame(main, text="Palm Tracker", padding=pad)
        palm_frame.pack(fill="x", pady=(0, 10))

        ttk.Button(
            palm_frame,
            text="Start Palm Tracker",
            command=self.start_palm_tracker
        ).grid(row=0, column=0, sticky="ew", padx=5, pady=5)

        ttk.Button(
            palm_frame,
            text="Stop Palm Tracker",
            command=self.stop_palm_tracker
        ).grid(row=0, column=1, sticky="ew", padx=5, pady=5)

        palm_frame.columnconfigure(0, weight=1)
        palm_frame.columnconfigure(1, weight=1)

        # Spatial depth
        depth_frame = ttk.LabelFrame(main, text="Spatial Depth", padding=pad)
        depth_frame.pack(fill="x", pady=(0, 10))

        ttk.Button(
            depth_frame,
            text="Start Spatial Depth",
            command=self.start_depth_tracker
        ).grid(row=0, column=0, sticky="ew", padx=5, pady=5)

        ttk.Button(
            depth_frame,
            text="Stop Spatial Depth",
            command=self.stop_depth_tracker
        ).grid(row=0, column=1, sticky="ew", padx=5, pady=5)

        depth_frame.columnconfigure(0, weight=1)
        depth_frame.columnconfigure(1, weight=1)

        # Status
        status_frame = ttk.LabelFrame(main, text="Status", padding=pad)
        status_frame.pack(fill="x")

        ttk.Label(status_frame, textvariable=self.status_var, wraplength=420).pack(anchor="w")

    def connect(self):
        try:
            if self._any_external_running():
                messagebox.showerror("Connection Error", "Stop Palm Tracker or Spatial Depth before connecting GUI serial.")
                return

            port = self.port_var.get().strip()
            baud = int(self.baud_var.get().strip())

            msg = self.controller.connect(port, baud)

            self.last_port = port
            self.last_baud = baud

            self.status_var.set(msg)
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def disconnect(self):
        try:
            msg = self.controller.disconnect()
            self.auto_disconnected_for_external = False
            self.status_var.set(msg)
        except Exception as e:
            messagebox.showerror("Disconnect Error", str(e))

    def home(self):
        try:
            self.controller.home()
            self.status_var.set("Sent home mask.")
        except Exception as e:
            messagebox.showerror("Home Error", str(e))

    def all_on(self):
        try:
            self.controller.all_on()
            self.status_var.set("Sent all-on mask.")
        except Exception as e:
            messagebox.showerror("All On Error", str(e))

    def stop(self):
        try:
            self.controller.stop()
            self.status_var.set("Stopped and sent home mask.")
        except Exception as e:
            messagebox.showerror("Stop Error", str(e))

    def apply_ramp(self):
        try:
            ramp_value = self.speed_var.get()
            msg = self.controller.set_ramp_value(ramp_value)
            self.status_var.set(msg)
        except Exception as e:
            messagebox.showerror("Ramp Error", str(e))

    def run_pattern(self):
        try:
            pattern = self.pattern_var.get().strip()
            delay = float(self.delay_var.get().strip())
            repetitions = int(self.repetitions_var.get().strip())

            self.controller.play_pattern(
                pattern_name=pattern,
                repetitions=repetitions,
                frame_delay=delay
            )
            self.status_var.set(f"Running pattern: {pattern}")
        except Exception as e:
            messagebox.showerror("Pattern Error", str(e))

    def _is_process_running(self, proc):
        return proc is not None and proc.poll() is None

    def _any_external_running(self):
        return self._is_process_running(self.palm_process) or self._is_process_running(self.depth_process)

    def _disconnect_gui_for_external(self):
        if self.controller.connected:
            self.controller.stop()
            self.controller.disconnect()
            self.auto_disconnected_for_external = True
        else:
            self.auto_disconnected_for_external = False

    def _reconnect_gui_serial_after_external(self, mode_name):
        if self.auto_disconnected_for_external and self.last_port is not None and self.last_baud is not None:
            try:
                msg = self.controller.connect(self.last_port, self.last_baud)
                self.auto_disconnected_for_external = False
                self.active_external_mode = None
                self.status_var.set(f"{mode_name} stopped. {msg}")
                return
            except Exception as e:
                self.auto_disconnected_for_external = False
                self.active_external_mode = None
                self.status_var.set(f"{mode_name} stopped. Reconnect failed: {e}")
                return

        self.active_external_mode = None
        self.status_var.set(f"{mode_name} stopped.")

    def start_palm_tracker(self):
        try:
            if self._is_process_running(self.depth_process):
                self.status_var.set("Spatial Depth is already running. Stop it first.")
                return

            if self._is_process_running(self.palm_process):
                self.status_var.set("Palm tracker is already running.")
                return

            self._disconnect_gui_for_external()

            script_path = os.path.join(os.path.dirname(__file__), "palm_test.py")

            self.palm_process = subprocess.Popen(
                [sys.executable, script_path],
                cwd=os.path.dirname(__file__)
            )

            self.active_external_mode = "palm"

            if self.auto_disconnected_for_external:
                self.status_var.set("Palm tracker started. GUI serial was disconnected automatically.")
            else:
                self.status_var.set("Palm tracker started.")

        except Exception as e:
            messagebox.showerror("Palm Tracker Error", str(e))

    def stop_palm_tracker(self):
        try:
            if self.palm_process is None or self.palm_process.poll() is not None:
                self.palm_process = None
                if self.active_external_mode == "palm" or self.auto_disconnected_for_external:
                    self._reconnect_gui_serial_after_external("Palm tracker")
                else:
                    self.status_var.set("Palm tracker is not running.")
                return

            self.palm_process.terminate()

            try:
                self.palm_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.palm_process.kill()
                self.palm_process.wait(timeout=2)

            self.palm_process = None
            self._reconnect_gui_serial_after_external("Palm tracker")

        except Exception as e:
            messagebox.showerror("Palm Tracker Stop Error", str(e))

    def start_depth_tracker(self):
        try:
            if self._is_process_running(self.palm_process):
                self.status_var.set("Palm tracker is already running. Stop it first.")
                return

            if self._is_process_running(self.depth_process):
                self.status_var.set("Spatial Depth is already running.")
                return

            self._disconnect_gui_for_external()

            script_path = os.path.join(os.path.dirname(__file__), "grid_depth.py")

            self.depth_process = subprocess.Popen(
                [sys.executable, script_path],
                cwd=os.path.dirname(__file__)
            )

            self.active_external_mode = "depth"

            if self.auto_disconnected_for_external:
                self.status_var.set("Spatial Depth started. GUI serial was disconnected automatically.")
            else:
                self.status_var.set("Spatial Depth started.")

        except Exception as e:
            messagebox.showerror("Spatial Depth Error", str(e))

    def stop_depth_tracker(self):
        try:
            if self.depth_process is None or self.depth_process.poll() is not None:
                self.depth_process = None
                if self.active_external_mode == "depth" or self.auto_disconnected_for_external:
                    self._reconnect_gui_serial_after_external("Spatial Depth")
                else:
                    self.status_var.set("Spatial Depth is not running.")
                return

            self.depth_process.terminate()

            try:
                self.depth_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.depth_process.kill()
                self.depth_process.wait(timeout=2)

            self.depth_process = None
            self._reconnect_gui_serial_after_external("Spatial Depth")

        except Exception as e:
            messagebox.showerror("Spatial Depth Stop Error", str(e))

    def on_close(self):
        try:
            if self._is_process_running(self.palm_process):
                self.palm_process.terminate()
                try:
                    self.palm_process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.palm_process.kill()
        except Exception:
            pass

        try:
            if self._is_process_running(self.depth_process):
                self.depth_process.terminate()
                try:
                    self.depth_process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.depth_process.kill()
        except Exception:
            pass

        try:
            self.controller.disconnect()
        except Exception:
            pass

        self.root.destroy()

    def _poll_status(self):
        try:
            # Auto-handle palm exiting on its own
            if self.palm_process is not None and self.palm_process.poll() is not None:
                self.palm_process = None
                if self.active_external_mode == "palm":
                    self._reconnect_gui_serial_after_external("Palm tracker")

            # Auto-handle depth exiting on its own
            if self.depth_process is not None and self.depth_process.poll() is not None:
                self.depth_process = None
                if self.active_external_mode == "depth":
                    self._reconnect_gui_serial_after_external("Spatial Depth")

            status = self.controller.get_status()
            if status["connected"]:
                self.status_var.set(
                    f"Connected | Mode: {status['mode']} | Busy: {status['busy']} | Ramp: {status['ramp_value']}"
                )
            else:
                if self._is_process_running(self.palm_process):
                    self.status_var.set("Palm tracker running | GUI serial disconnected")
                elif self._is_process_running(self.depth_process):
                    self.status_var.set("Spatial Depth running | GUI serial disconnected")
                else:
                    self.status_var.set("Not connected")
        except Exception:
            pass

        self.root.after(300, self._poll_status)


if __name__ == "__main__":
    root = tk.Tk()
    app = WallGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()