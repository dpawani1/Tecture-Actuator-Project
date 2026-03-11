import tkinter as tk
from tkinter import ttk, messagebox

from wall_controller import WallController
from patterns import get_pattern_names


class WallGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Kinetic Wall Control")
        self.root.geometry("460x460")
        self.root.resizable(False, False)

        self.controller = WallController()

        self.port_var = tk.StringVar(value="/dev/ttyACM0")
        self.baud_var = tk.StringVar(value="9600")
        self.pattern_var = tk.StringVar(value="wave_lr")
        self.delay_var = tk.StringVar(value="5")
        self.repetitions_var = tk.StringVar(value="1")
        self.speed_var = tk.IntVar(value=50)
        self.status_var = tk.StringVar(value="Not connected")

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

        # Speed control
        speed_frame = ttk.LabelFrame(main, text="Speed Control", padding=pad)
        speed_frame.pack(fill="x", pady=(0, 10))

        ttk.Label(speed_frame, text="Actuator Speed (%):").pack(anchor="w")

        speed_slider = tk.Scale(
            speed_frame,
            from_=0,
            to=100,
            orient="horizontal",
            variable=self.speed_var,
            length=350
        )
        speed_slider.pack(anchor="w")

        ttk.Button(speed_frame, text="Apply Speed", command=self.apply_speed).pack(anchor="w", pady=(8, 0))

        # Quick controls
        quick_frame = ttk.LabelFrame(main, text="Quick Controls", padding=pad)
        quick_frame.pack(fill="x", pady=(0, 10))

        ttk.Button(quick_frame, text="Home", command=self.home).grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        ttk.Button(quick_frame, text="All On", command=self.all_on).grid(row=0, column=1, sticky="ew", padx=5, pady=5)
        ttk.Button(quick_frame, text="Stop", command=self.stop).grid(row=0, column=2, sticky="ew", padx=5, pady=5)

        for i in range(3):
            quick_frame.columnconfigure(i, weight=1)

        # Status
        status_frame = ttk.LabelFrame(main, text="Status", padding=pad)
        status_frame.pack(fill="x")

        ttk.Label(status_frame, textvariable=self.status_var, wraplength=400).pack(anchor="w")

    def connect(self):
        try:
            port = self.port_var.get().strip()
            baud = int(self.baud_var.get().strip())
            msg = self.controller.connect(port, baud)
            self.status_var.set(msg)
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def disconnect(self):
        try:
            msg = self.controller.disconnect()
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

    def apply_speed(self):
        try:
            percent = self.speed_var.get()
            msg = self.controller.set_speed_percent(percent)
            self.status_var.set(msg)
        except Exception as e:
            messagebox.showerror("Speed Error", str(e))

    def run_pattern(self):
        try:
            pattern = self.pattern_var.get().strip()
            delay = float(self.delay_var.get().strip())
            repetitions = int(self.repetitions_var.get().strip())

            self.controller.play_pattern(pattern, repetitions=repetitions, frame_delay=delay)
            self.status_var.set(f"Running pattern: {pattern}")
        except Exception as e:
            messagebox.showerror("Pattern Error", str(e))

    def _poll_status(self):
        try:
            status = self.controller.get_status()
            if status["connected"]:
                self.status_var.set(
                    f"Connected | Mode: {status['mode']} | Busy: {status['busy']} | Speed: {status['speed_percent']}%"
                )
            else:
                self.status_var.set("Not connected")
        except Exception:
            pass

        self.root.after(300, self._poll_status)


if __name__ == "__main__":
    root = tk.Tk()
    app = WallGUI(root)
    root.mainloop()