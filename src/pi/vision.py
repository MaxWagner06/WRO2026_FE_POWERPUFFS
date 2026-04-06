import cv2
import numpy as np
from dataclasses import dataclass
from enum import Enum
from typing import List


class ObjectType(Enum):
    RED_PILLAR   = "red"
    GREEN_PILLAR = "green"
    MAGENTA_WALL = "magenta"
    ORANGE_LINE  = "orange"
    BLUE_LINE    = "blue"


@dataclass
class DetectedObject:
    obj_type: ObjectType
    cx: int        # centroid x (pixels, 0=left edge)
    cy: int        # centroid y (pixels, 0=top edge)
    area: int      # contour area in pixels²
    bbox: tuple    # (x, y, w, h) bounding box


class Vision:
    def __init__(self, config):
        # Pull every HSV range out of config once so process_frame stays clean
        self._min_area = config.MIN_CONTOUR_AREA
        self._max_area = config.MAX_CONTOUR_AREA

        # Store ranges as numpy arrays for cv2.inRange
        self._ranges = {
            ObjectType.RED_PILLAR: [
                (np.array(config.HSV_RED_LOWER_1), np.array(config.HSV_RED_UPPER_1)),
                (np.array(config.HSV_RED_LOWER_2), np.array(config.HSV_RED_UPPER_2)),
            ],
            ObjectType.GREEN_PILLAR: [
                (np.array(config.HSV_GREEN_LOWER), np.array(config.HSV_GREEN_UPPER)),
            ],
            ObjectType.MAGENTA_WALL: [
                (np.array(config.HSV_MAGENTA_LOWER), np.array(config.HSV_MAGENTA_UPPER)),
            ],
            ObjectType.ORANGE_LINE: [
                (np.array(config.HSV_ORANGE_LOWER), np.array(config.HSV_ORANGE_UPPER)),
            ],
            ObjectType.BLUE_LINE: [
                (np.array(config.HSV_BLUE_LOWER), np.array(config.HSV_BLUE_UPPER)),
            ],
        }

    def process_frame(self, frame: np.ndarray) -> List[DetectedObject]:
        """Segment frame by color, find contours, return all detected objects."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []

        for obj_type, range_list in self._ranges.items():
            # Build combined mask — handles red's two-range wrap-around
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for (lo, hi) in range_list:
                mask |= cv2.inRange(hsv, lo, hi)

            # Small morphological cleanup to remove noise pixels
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if not (self._min_area <= area <= self._max_area):
                    continue  # too small (noise) or too large (field boundary)

                M = cv2.moments(cnt)
                if M['m00'] == 0:
                    continue
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                bbox = cv2.boundingRect(cnt)

                detections.append(DetectedObject(
                    obj_type=obj_type,
                    cx=cx, cy=cy,
                    area=int(area),
                    bbox=bbox,
                ))

        return detections
