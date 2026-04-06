import threading
import numpy as np
from picamera2 import Picamera2
from logger import log


class Camera:
    def __init__(self, config):
        self._config = config
        self._cam = Picamera2()
        self._frame = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def start(self):
        """Configure the camera with fixed settings then launch capture thread."""
        w, h = self._config.CAMERA_RES
        cam_cfg = self._cam.create_video_configuration(
            main={"size": (w, h), "format": "BGR888"},
            controls={
                # Lock auto-exposure and white balance so lighting won't shift mid-run
                "AeEnable": False,
                "AwbEnable": False,
                "ExposureTime": 10000,   # 10 ms — tune for the venue lighting
                "AnalogueGain": 2.0,
            }
        )
        self._cam.configure(cam_cfg)
        self._cam.start()
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        log.info("Camera started at %dx%d @%d fps", w, h, self._config.CAMERA_FPS)

    def _capture_loop(self):
        """Continuously grab frames from picamera2 into the shared buffer."""
        while self._running:
            frame = self._cam.capture_array()  # returns BGR numpy array
            with self._lock:
                self._frame = frame

    def get_frame(self) -> np.ndarray | None:
        """Return the most recent frame (BGR). Returns None before first frame arrives."""
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def stop(self):
        """Stop capture thread and release camera."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        self._cam.stop()
        log.info("Camera stopped")
