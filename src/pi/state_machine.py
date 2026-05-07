import time
from enum import Enum
from vision import DetectedObject, ObjectType
from logger import log


class State(Enum):
    IDLE            = "idle"
    DRIVING         = "driving"
    AVOIDING        = "avoiding"
    REVERSING       = "reversing"
    PARKING_SEARCH  = "parking_search"
    PARKING_EXECUTE = "parking_execute"
    STOPPING        = "stopping"
    STOPPED         = "stopped"


# How many laps trigger the end condition for each challenge
_LAPS_TO_FINISH = {'open': 3, 'obstacle': 3}


class StateMachine:
    def __init__(self, config, challenge_type: str):
        self._cfg            = config
        self._challenge      = challenge_type   # 'open' or 'obstacle'

        self.state           = State.IDLE
        self.lap_count       = 0
        self.section_count   = 0               # each corner = +1, each straight = +1

        self.last_pillar_color  = None         # 'red' or 'green', updated while AVOIDING
        self.total_pillars_seen = 0

        self._direction      = None            # 'cw' or 'ccw', locked in on first corner
        self._corner_entry_time = None         # monotonic time when gyro first spiked
        self._in_corner      = False

        self._reversal_done  = False           # obstacle: have we already reversed?
        self._state_entered  = time.monotonic()

        log.info("StateMachine init: challenge=%s", challenge_type)

    # ─── Public API ─────────────────────────────────────────────────────────────

    def notify_started(self):
        """Call this when the physical start button is pressed."""
        if self.state == State.IDLE:
            self._transition(State.DRIVING)

    def tick(self, telemetry: dict, detections: list, nav_output: tuple) -> dict:
        """
        Run one FSM cycle at 30 Hz.

        Returns a command dict: {'speed': int, 'steering': int, 'in_corner': bool}
        """
        steering, speed = nav_output

        # ── Update corner / lap tracking first ──
        self._update_corner_tracking(telemetry, detections)

        # ── Run current state handler ──
        handler = {
            State.IDLE:            self._handle_idle,
            State.DRIVING:         self._handle_driving,
            State.AVOIDING:        self._handle_avoiding,
            State.REVERSING:       self._handle_reversing,
            State.PARKING_SEARCH:  self._handle_parking_search,
            State.PARKING_EXECUTE: self._handle_parking_execute,
            State.STOPPING:        self._handle_stopping,
            State.STOPPED:         self._handle_stopped,
        }.get(self.state, self._handle_idle)

        command = handler(telemetry, detections, steering, speed)
        return command

    @property
    def in_corner(self) -> bool:
        # Exposed so Navigator can switch gains without owning lap-counting state.
        return self._in_corner

    # ─── State Handlers ─────────────────────────────────────────────────────────

    def _handle_idle(self, telemetry, detections, steering, speed):
        # Wait for start button (notify_started does the transition)
        return {'speed': 0, 'steering': 0, 'in_corner': False}

    def _handle_driving(self, telemetry, detections, steering, speed):
        # Check if we should switch to AVOIDING (obstacle challenge)
        if self._challenge == 'obstacle':
            pillars = [d for d in detections if d.obj_type in (
                ObjectType.RED_PILLAR, ObjectType.GREEN_PILLAR
            )]
            if pillars:
                self._transition(State.AVOIDING)
                return self._handle_avoiding(telemetry, detections, steering, speed)

        # Check lap completion
        laps_needed = _LAPS_TO_FINISH[self._challenge]
        if self.lap_count >= laps_needed:
            if self._challenge == 'obstacle':
                self._transition(State.PARKING_SEARCH)
            else:
                self._transition(State.STOPPING)
            return {'speed': 0, 'steering': 0, 'in_corner': self._in_corner}

        # Check reversal trigger (obstacle only, end of lap 2)
        if (self._challenge == 'obstacle'
                and self.lap_count == 2
                and not self._reversal_done
                and self.last_pillar_color == 'red'):
            self._transition(State.REVERSING)
            return {'speed': 0, 'steering': 0, 'in_corner': False}

        return {'speed': int(speed), 'steering': int(steering), 'in_corner': self._in_corner}

    def _handle_avoiding(self, telemetry, detections, steering, speed):
        # Track which colour pillar we're passing for lap-reversal logic
        pillars = [d for d in detections if d.obj_type in (
            ObjectType.RED_PILLAR, ObjectType.GREEN_PILLAR
        )]
        if pillars:
            closest = max(pillars, key=lambda d: d.area)
            color = 'red' if closest.obj_type == ObjectType.RED_PILLAR else 'green'
            if color != self.last_pillar_color:
                self.last_pillar_color = color
                self.total_pillars_seen += 1
                log.info("Pillar #%d detected: %s", self.total_pillars_seen, color)
        else:
            # Pillar cleared, back to DRIVING
            self._transition(State.DRIVING)

        return {'speed': int(speed), 'steering': int(steering), 'in_corner': self._in_corner}

    def _handle_reversing(self, telemetry, detections, steering, speed):
        """Execute 180° turn using IMU yaw, then resume DRIVING in opposite direction."""
        imu_yaw = telemetry.get('imu_yaw', 0) / 100.0 if telemetry else 0.0  # stored as deg*100
        elapsed = time.monotonic() - self._state_entered

        # Simplified: steer hard for ~2 s (tune with actual vehicle)
        if elapsed < 2.0:
            return {'speed': -self._cfg.SPEED_CORNER, 'steering': self._cfg.MAX_STEER_ANGLE, 'in_corner': True}

        # Reversal complete
        self._reversal_done = True
        # Flip direction
        self._direction = 'ccw' if self._direction == 'cw' else 'cw'
        self._transition(State.DRIVING)
        return {'speed': int(speed), 'steering': int(steering), 'in_corner': False}

    def _handle_parking_search(self, telemetry, detections, steering, speed):
        # Look for magenta parking wall
        magenta = [d for d in detections if d.obj_type == ObjectType.MAGENTA_WALL]
        if magenta:
            self._transition(State.PARKING_EXECUTE)
        return {'speed': int(self._cfg.SPEED_CORNER), 'steering': int(steering), 'in_corner': False}

    def _handle_parking_execute(self, telemetry, detections, steering, speed):
        """Simple two-phase park: overshoot past gap, reverse in."""
        elapsed = time.monotonic() - self._state_entered

        if elapsed < 1.0:
            # Phase 1: drive past the gap
            return {'speed': self._cfg.SPEED_CORNER, 'steering': 0, 'in_corner': False}
        elif elapsed < 2.5:
            # Phase 2: reverse into the spot
            return {'speed': -self._cfg.SPEED_CORNER, 'steering': self._cfg.PARKING_REVERSE_ANGLE, 'in_corner': False}

        # Parked — stop
        self._transition(State.STOPPED)
        return {'speed': 0, 'steering': 0, 'in_corner': False}

    def _handle_stopping(self, telemetry, detections, steering, speed):
        """Gradual slow-down then STOPPED."""
        elapsed = time.monotonic() - self._state_entered
        if elapsed > 1.0:
            self._transition(State.STOPPED)
        return {'speed': 0, 'steering': 0, 'in_corner': False}

    def _handle_stopped(self, telemetry, detections, steering, speed):
        return {'speed': 0, 'steering': 0, 'in_corner': False}

    # ─── Internal Helpers ────────────────────────────────────────────────────────

    def _transition(self, new_state: State):
        log.info("State: %s → %s  (lap=%d sec=%d)",
                 self.state.value, new_state.value,
                 self.lap_count, self.section_count)
        self.state = new_state
        self._state_entered = time.monotonic()

    def _update_corner_tracking(self, telemetry, detections):
        """Detect corners from gyro and camera lines; update lap/section counters."""
        if telemetry is None:
            return

        # IMU yaw rate in deg/s (stored as deg*100 in packet, but rate is direct from gyro)
        gyro_z = telemetry.get('gyro_z', 0.0)

        if abs(gyro_z) > self._cfg.GYRO_CORNER_THRESHOLD:
            if not self._in_corner:
                # Entering a corner
                self._corner_entry_time = time.monotonic()
                self._in_corner = True
        else:
            if self._in_corner:
                # Check we were cornering long enough (not just noise)
                duration_ms = (time.monotonic() - self._corner_entry_time) * 1000
                if duration_ms >= self._cfg.GYRO_CORNER_MIN_MS:
                    # The competition course alternates corner/straight sections.
                    self.section_count += 2   # corner + following straight = 2 sections
                    if self.section_count >= self._cfg.SECTIONS_PER_LAP:
                        self.section_count = 0
                        self.lap_count += 1
                        log.info("Lap %d completed", self.lap_count)
                self._in_corner = False
