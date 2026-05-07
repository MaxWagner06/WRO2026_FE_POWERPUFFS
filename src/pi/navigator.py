import time
from vision import DetectedObject, ObjectType


def _clamp(val, lo, hi):
    return max(lo, min(hi, val))


class Navigator:
    def __init__(self, config):
        self._cfg = config
        self._prev_error = 0.0
        self._last_time  = time.monotonic()

    def compute(
        self,
        telemetry: dict,
        detections: list[DetectedObject],
        mode: str,         # 'open' or 'obstacle'
        in_corner: bool = False,
    ) -> tuple[float, float]:
        """
        Compute steering angle (deg) and speed (RPM) for this decision cycle.

        Wall-follow with PD is always the base controller.
        When a pillar is visible in obstacle mode, blend in avoidance steering.
        """
        now = time.monotonic()
        dt  = max(now - self._last_time, 1e-3)   # avoid division by zero
        self._last_time = now

        # ── 1. Grab ToF readings (fall back to safe values if telemetry missing) ──
        tof_left  = telemetry.get('tof_left',  500) if telemetry else 500
        tof_right = telemetry.get('tof_right', 500) if telemetry else 500
        tof_front = telemetry.get('tof_front', 2000) if telemetry else 2000

        # ── 2. Select PD gains: stronger during corners ──
        # The FSM supplies in_corner so navigation can react before vision fully settles.
        kp = self._cfg.KP_CORNER   if in_corner else self._cfg.KP_STRAIGHT
        kd = self._cfg.KD_CORNER   if in_corner else self._cfg.KD_STRAIGHT

        # ── 3. PD wall-centering ──
        # Positive error → vehicle too far left → steer right
        error      = (tof_right - tof_left) - self._cfg.CENTER_OFFSET
        derivative = (error - self._prev_error) / dt
        wall_steer = kp * error + kd * derivative
        self._prev_error = error

        # ── 4. Corner pre-emption ──
        if tof_front < self._cfg.CORNER_THRESHOLD_MM:
            # Steering stays PD-based here; the lower speed gives the corner controller room.
            speed = float(self._cfg.SPEED_CORNER)
        else:
            speed = float(self._cfg.SPEED_STRAIGHT)

        # ── 5. Pillar avoidance (obstacle challenge only) ──
        final_steer = wall_steer
        if mode == 'obstacle':
            pillars = [d for d in detections if d.obj_type in (
                ObjectType.RED_PILLAR, ObjectType.GREEN_PILLAR
            )]
            if pillars:
                # Choose the closest (largest area) pillar
                pillar = max(pillars, key=lambda d: d.area)
                final_steer = self._blend_avoidance(wall_steer, pillar, tof_front)

        # ── 6. Emergency wall correction overrides everything ──
        if tof_left < self._cfg.WALL_DANGER_MM:
            final_steer = self._cfg.MAX_STEER_ANGLE   # hard steer right
        elif tof_right < self._cfg.WALL_DANGER_MM:
            final_steer = -self._cfg.MAX_STEER_ANGLE  # hard steer left

        return _clamp(final_steer, -self._cfg.MAX_STEER_ANGLE, self._cfg.MAX_STEER_ANGLE), speed

    def _blend_avoidance(self, wall_steer: float, pillar: DetectedObject, tof_front: float) -> float:
        """Blend wall-following with pillar avoidance. Closer pillar = stronger avoidance."""
        # Estimate distance from area: larger blob = closer (rough but fast)
        # Use tof_front as upper bound when pillar is ahead
        pillar_dist_est = max(100.0, self._cfg.BLEND_DISTANCE - (pillar.area / 50.0))

        if pillar.obj_type == ObjectType.RED_PILLAR:
            avoidance_steer = -self._cfg.AVOIDANCE_OFFSET   # pass LEFT of red
        else:
            avoidance_steer = +self._cfg.AVOIDANCE_OFFSET   # pass RIGHT of green

        # alpha=1 → trust wall-follow; alpha→0 → full avoidance
        alpha = _clamp(pillar_dist_est / self._cfg.BLEND_DISTANCE, 0.1, 1.0)
        return alpha * wall_steer + (1.0 - alpha) * avoidance_steer
