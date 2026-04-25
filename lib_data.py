import csv
import os
import time
import threading
import queue


class DataLogger:
    """
    Logger data dari STM32 ke CSV.

    - Data masuk lewat handle_sample(tuple)
    - Penulisan ke file dipisah di thread worker + queue,
      supaya aman di-rate ~20 ms (50 Hz).
    """

    def __init__(self, base_dir="logs"):
        self.base_dir = base_dir
        os.makedirs(self.base_dir, exist_ok=True)

        self._lock = threading.Lock()
        self._recording = False
        self._file = None
        self._writer = None
        self._queue = queue.Queue()
        self._row_count = 0

        self._worker_thread = threading.Thread(
            target=self._worker_loop,
            daemon=True
        )
        self._worker_thread.start()

    # ---------------- API ----------------

    def is_recording(self) -> bool:
        with self._lock:
            return self._recording

    def set_recording(self, enabled: bool):
        """
        Aktif/nonaktifkan rekam.
        Tiap kali ON akan buka file baru dengan timestamp.
        """
        with self._lock:
            if enabled == self._recording:
                return

            if enabled:
                self._open_new_file()
            else:
                self._close_file()

            self._recording = enabled

    def toggle_recording(self):
        with self._lock:
            new_state = not self._recording
        self.set_recording(new_state)

    def handle_sample(self, sample_tuple):
        """
        Dipanggil dari thread pembaca serial (lib_com.read_control_status).
        sample_tuple: (logtick, degree, cmX, setspeed, r1, r2, r3)
        """
        # push ke queue supaya tidak blocking thread serial
        self._queue.put(sample_tuple)

    # ---------------- internal ----------------

    def _open_new_file(self):
        if self._file is not None:
            self._close_file()

        ts = time.strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.base_dir, f"log_{ts}.csv")
        self._file = open(filename, "w", newline="")
        self._writer = csv.writer(self._file)
        self._row_count = 0

        # header
        self._writer.writerow(
            ["logtick", "degree", "cmX", "setspeed", "reserved1", "reserved2", "reserved3"]
        )
        print("Recording to:", filename)

    def _close_file(self):
        if self._file is not None:
            try:
                self._file.flush()
                self._file.close()
            except Exception:
                pass
        self._file = None
        self._writer = None
        self._row_count = 0

    def _worker_loop(self):
        """
        Loop penulis CSV. Selalu mengosongkan queue supaya
        tidak numpuk walau recording OFF.
        """
        while True:
            sample = self._queue.get()   # blocking
            with self._lock:
                if self._recording and self._writer is not None:
                    self._writer.writerow(sample)
                    self._row_count += 1
                    # flush per 50 baris (~1 detik @ 50 Hz)
                    if (self._row_count % 50) == 0:
                        self._file.flush()
