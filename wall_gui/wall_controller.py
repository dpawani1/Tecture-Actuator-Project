import threading
import time
import serial
from patterns import get_pattern_frames, get_demo_sequence


class WallController:
    def __init__(self):
        self.ser = None
        self.connected = False

        self.worker_thread = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

        self.current_mode = "idle"

    def connect(self, port, baud=9600, startup_wait=10.5):
        if self.connected:
            return "Already connected."

        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2.0)  # allow serial to open/reset Arduino
        time.sleep(startup_wait)  # wait for Arduino homing window
        self.connected = True
        self.current_mode = "idle"
        return f"Connected to {port} at {baud} baud."

    def disconnect(self):
        self.stop()

        if self.ser is not None:
            try:
                self.send_mask(0)
                time.sleep(0.2)
            except Exception:
                pass

            try:
                self.ser.close()
            except Exception:
                pass

        self.ser = None
        self.connected = False
        self.current_mode = "disconnected"
        return "Disconnected."

    def send_mask(self, mask):
        if not self.connected or self.ser is None:
            raise RuntimeError("Not connected to serial port.")

        mask = max(0, min(511, int(mask)))
        with self.lock:
            self.ser.write(f"{mask}\n".encode("utf-8"))
            self.ser.flush()

    def home(self):
        self.stop()
        self.send_mask(0)
        self.current_mode = "home"

    def all_on(self):
        self.stop()
        self.send_mask(511)
        self.current_mode = "all_on"

    def stop(self):
        self.stop_event.set()

        if self.worker_thread is not None and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)

        self.worker_thread = None
        self.stop_event.clear()

        if self.connected:
            try:
                self.send_mask(0)
            except Exception:
                pass

        self.current_mode = "stopped"

    def is_busy(self):
        return self.worker_thread is not None and self.worker_thread.is_alive()

    def _run_frames(self, frames, loops, frame_delay, end_home=True):
        try:
            self.current_mode = "running"
            for _ in range(loops):
                if self.stop_event.is_set():
                    break
                for mask in frames:
                    if self.stop_event.is_set():
                        break
                    self.send_mask(mask)
                    time.sleep(frame_delay)

            if end_home and self.connected:
                self.send_mask(0)
                self.current_mode = "home"
            else:
                self.current_mode = "idle"

        except Exception as e:
            self.current_mode = f"error: {e}"

    def play_pattern(self, pattern_name, loops=5, frame_delay=0.35):
        if not self.connected:
            raise RuntimeError("Not connected.")

        self.stop()

        frames = get_pattern_frames(pattern_name)
        self.worker_thread = threading.Thread(
            target=self._run_frames,
            args=(frames, loops, frame_delay, True),
            daemon=True
        )
        self.worker_thread.start()
        self.current_mode = pattern_name

    def _run_demo(self):
        try:
            self.current_mode = "demo"
            sequence = get_demo_sequence()

            for pattern_name, loops, frame_delay in sequence:
                if self.stop_event.is_set():
                    break

                frames = get_pattern_frames(pattern_name)

                for _ in range(loops):
                    if self.stop_event.is_set():
                        break
                    for mask in frames:
                        if self.stop_event.is_set():
                            break
                        self.send_mask(mask)
                        time.sleep(frame_delay)

            if self.connected:
                self.send_mask(0)
                self.current_mode = "home"

        except Exception as e:
            self.current_mode = f"error: {e}"

    def play_demo(self):
        if not self.connected:
            raise RuntimeError("Not connected.")

        self.stop()

        self.worker_thread = threading.Thread(
            target=self._run_demo,
            daemon=True
        )
        self.worker_thread.start()

    def get_status(self):
        return {
            "connected": self.connected,
            "mode": self.current_mode,
            "busy": self.is_busy(),
        }