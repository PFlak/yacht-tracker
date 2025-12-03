import threading
import time


class AdjustableTimer:
    def __init__(self, callback, period_s: float):
        """
        callback: function to call periodically
        period_s: initial period in seconds (e.g. 0.1 for 10 Hz)
        """
        self._callback = callback
        self._period = period_s
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join()

    def set_period(self, period_s: float):
        """Change timer period on the fly."""

        with self._lock:
            self._period = period_s

    def _get_period(self):
        with self._lock:
            return self._period

    def _run(self):
        next_call = time.perf_counter()
        while not self._stop_event.is_set():
            period = self._get_period()
            next_call += period

            # Run the callback (handle your own exceptions inside callback if needed)
            self._callback()

            # Sleep until next tick (but be responsive to stop event)
            sleep_time = next_call - time.perf_counter()
            if sleep_time > 0:
                # wait() returns True if stop_event set, so we can break early
                if self._stop_event.wait(timeout=sleep_time):
                    break
