import cv2
import numpy as np
from vision import ObjectType


class LaneDetector:
    def __init__(self, config):
        self._orange_lo = np.array(config.HSV_ORANGE_LOWER)
        self._orange_hi = np.array(config.HSV_ORANGE_UPPER)
        self._blue_lo   = np.array(config.HSV_BLUE_LOWER)
        self._blue_hi   = np.array(config.HSV_BLUE_UPPER)
        self._min_area  = config.MIN_CONTOUR_AREA

        # Keep recent history to decide direction once we've seen both colours
        # Reserved for temporal smoothing; current direction inference is frame-local.
        self._orange_cx_history = []
        self._blue_cx_history   = []

    def detect_lines(self, frame: np.ndarray) -> dict:
        """
        Detect orange and blue corner lines and infer driving direction.

        Returns:
            {
                'orange': bool,           # orange line visible this frame
                'blue':   bool,           # blue line visible this frame
                'orange_cx': int | None,  # horizontal centroid of orange blob
                'blue_cx':   int | None,
                'direction': 'cw' | 'ccw' | None,
            }
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        orange_cx = self._largest_blob_cx(hsv, self._orange_lo, self._orange_hi)
        blue_cx   = self._largest_blob_cx(hsv, self._blue_lo,   self._blue_hi)

        direction = None
        if orange_cx is not None and blue_cx is not None:
            # WRO track layout: orange is on the outer lane on the right side
            # when travelling CW; blue is on the inner lane.
            # If orange is to the RIGHT of blue → CW, else → CCW.
            direction = 'cw' if orange_cx > blue_cx else 'ccw'

        return {
            'orange':    orange_cx is not None,
            'blue':      blue_cx   is not None,
            'orange_cx': orange_cx,
            'blue_cx':   blue_cx,
            'direction': direction,
        }

    def _largest_blob_cx(self, hsv: np.ndarray, lo, hi) -> int | None:
        """Return the horizontal centroid of the largest blob matching the given HSV range."""
        mask = cv2.inRange(hsv, lo, hi)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        # A light open removes speckles without erasing the painted line blob.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self._min_area:
            return None

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None
        return int(M['m10'] / M['m00'])
