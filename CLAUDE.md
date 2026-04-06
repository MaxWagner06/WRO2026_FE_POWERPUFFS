# CLAUDE.md — WRO Future Engineers 2026 Codebase

## Project Overview

This is an autonomous vehicle project for the WRO Future Engineers 2026 competition. The vehicle uses a **Raspberry Pi 5 (16GB)** for vision/decision and an **STM32F4 Nucleo** for real-time motor control. They communicate over UART.

Read `architecture.md` in the project root before making any changes. It contains the full system design, pin allocations, protocol definitions, and algorithm specifications.

## Tech Stack

- **Pi 5**: Python 3.11+, OpenCV 4.x, picamera2, pyserial
- **STM32F4**: C (STM32 HAL or bare-metal), compiled with arm-none-eabi-gcc or STM32CubeIDE
- **Communication**: UART 115200 baud, binary packets with CRC8

## Repository Layout

```
src/
├── pi/           # All Raspberry Pi Python code
│   ├── main.py
│   ├── camera.py
│   ├── vision.py
│   ├── lane.py
│   ├── navigator.py
│   ├── state_machine.py
│   ├── serial_comm.py
│   ├── config.py
│   └── logger.py
└── stm32/        # All STM32 C firmware
    └── Core/
        ├── Src/
        │   ├── main.c
        │   ├── motor_control.c
        │   ├── servo.c
        │   ├── sensors.c
        │   ├── comm.c
        │   ├── safety.c
        │   └── filters.c
        └── Inc/
            ├── motor_control.h
            ├── servo.h
            ├── sensors.h
            ├── comm.h
            ├── safety.h
            ├── filters.h
            └── protocol.h
```

## Implementation Instructions

When asked to implement or modify code, follow these rules:

### General Rules

1. **Always read `architecture.md` first** for system context before writing any module.
2. **All tunable constants go in `config.py`** (Pi side) or as `#define` in a single header (STM32 side). Never hardcode magic numbers in logic files.
3. **Every function must have a docstring** (Python) or a comment block (C) explaining purpose, inputs, outputs.
4. **Log everything useful** — state transitions, sensor readings, errors. Use `logger.py` on the Pi and UART debug prints (disabled in competition mode) on STM32.
5. **No wireless/network code** — the competition prohibits all RF/BT/Wi-Fi during rounds. Never import socket, requests, or any networking library in Pi code.

### Pi Python Code (`src/pi/`)

#### `config.py`
- Central configuration file. All PID gains, HSV ranges, thresholds, speeds, camera settings.
- Use typed constants with descriptive names (e.g., `KP_WALL_STRAIGHT = 0.5`).
- Group by category with comments.
- This file must be importable by all other modules.

#### `camera.py`
- Use `picamera2` library for Pi Camera Module 3.
- Run capture in a dedicated thread.
- Lock exposure and white balance at initialization (no auto during rounds).
- Publish frames to a shared buffer (use `threading.Lock`).
- Resolution: 640x480 at 30 FPS (from `config.py`).

```python
# Target API:
class Camera:
    def __init__(self, config):
        """Initialize Pi Camera Module 3 Wide with fixed settings."""
    def start(self):
        """Start capture thread."""
    def get_frame(self) -> np.ndarray:
        """Return latest frame (BGR numpy array). Thread-safe."""
    def stop(self):
        """Release camera resources."""
```

#### `vision.py`
- Receives BGR frames, returns detected objects.
- HSV color segmentation for: red, green, magenta, orange, blue.
- Contour detection with area filtering (`config.MIN_CONTOUR_AREA` to `config.MAX_CONTOUR_AREA`).
- Returns list of `DetectedObject` dataclass instances.

```python
from dataclasses import dataclass
from enum import Enum

class ObjectType(Enum):
    RED_PILLAR = "red"
    GREEN_PILLAR = "green"
    MAGENTA_WALL = "magenta"
    ORANGE_LINE = "orange"
    BLUE_LINE = "blue"

@dataclass
class DetectedObject:
    obj_type: ObjectType
    cx: int          # centroid x in frame
    cy: int          # centroid y in frame
    area: int        # contour area in pixels
    bbox: tuple      # (x, y, w, h) bounding box

class Vision:
    def __init__(self, config):
        """Load HSV ranges from config."""
    def process_frame(self, frame: np.ndarray) -> list[DetectedObject]:
        """Process one frame, return all detected objects."""
```

**Important**: Red wraps around in HSV (H=0 and H=180). Use two masks and OR them:
```python
mask_red = cv2.inRange(hsv, RED_LOWER_1, RED_UPPER_1) | cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2)
```

#### `lane.py`
- Detects orange and blue corner lines.
- Determines driving direction (CW vs CCW) from relative positions.
- Confirms section transitions.

```python
class LaneDetector:
    def __init__(self, config):
        pass
    def detect_lines(self, frame: np.ndarray) -> dict:
        """Return {'orange': bool, 'blue': bool, 'direction': 'cw'|'ccw'|None}"""
```

#### `navigator.py`
- Core control logic. Receives telemetry (ToF, IMU) + vision detections.
- Outputs steering angle (degrees) and speed (RPM).
- Implements PD wall-following as the base controller.
- Blends pillar avoidance when objects detected.

```python
class Navigator:
    def __init__(self, config):
        self.prev_error = 0.0
    def compute(self, telemetry: dict, detections: list[DetectedObject], mode: str) -> tuple[float, float]:
        """
        Returns (steering_angle_deg, speed_rpm).
        mode: 'open' or 'obstacle'
        """
```

Wall-following formula:
```
error = (tof_right - tof_left) - CENTER_OFFSET
steering = Kp * error + Kd * d(error)/dt
```

Corner pre-emption:
```
if tof_front < CORNER_THRESHOLD:
    switch to corner gains, reduce speed
```

Pillar avoidance blending:
```
alpha = clamp(pillar_distance / BLEND_DISTANCE, 0.1, 1.0)
final = alpha * wall_steer + (1-alpha) * avoidance_steer
```

#### `state_machine.py`
- Enum-based states: IDLE, DRIVING, AVOIDING, REVERSING, PARKING_SEARCH, PARKING_EXECUTE, STOPPING, STOPPED.
- `tick()` method called every decision cycle (30 Hz).
- Tracks: current_lap, current_section, last_pillar_color, total_pillars_passed.
- Log every state transition with timestamp.

```python
from enum import Enum

class State(Enum):
    IDLE = "idle"
    DRIVING = "driving"
    AVOIDING = "avoiding"
    REVERSING = "reversing"
    PARKING_SEARCH = "parking_search"
    PARKING_EXECUTE = "parking_execute"
    STOPPING = "stopping"
    STOPPED = "stopped"

class StateMachine:
    def __init__(self, config, challenge_type: str):
        """challenge_type: 'open' or 'obstacle'"""
        self.state = State.IDLE
        self.lap_count = 0
        self.section_count = 0
        self.last_pillar_color = None

    def tick(self, telemetry: dict, detections: list, nav_output: tuple) -> dict:
        """
        Run one FSM cycle.
        Returns dict with 'speed', 'steering', 'mode' for serial_comm.
        """
```

**Lap counting logic:**
- Corner detected = gyro angular rate > threshold for > 200ms
- Each corner increments section_count by 2 (corner + following straight)
- 4 corners = 8 sections = 1 lap
- Cross-validate with camera line detection (orange/blue at corners)

**Reversal trigger (Obstacle only):**
- Track every pillar color during lap 2
- When section_count crosses from lap 2 to lap 3: check last_pillar_color
- If RED: transition to REVERSING state
- REVERSING: execute 180-degree turn using gyro, then resume DRIVING in opposite direction

#### `serial_comm.py`
- Handles UART communication with STM32.
- Runs in dedicated thread.
- TX: sends command packets at 50 Hz.
- RX: receives telemetry packets at 100 Hz.
- CRC8 validation on all packets.

```python
class SerialComm:
    def __init__(self, port='/dev/ttyAMA0', baudrate=115200):
        pass
    def start(self):
        """Start rx/tx threads."""
    def send_command(self, msg_type: int, value: int):
        """Package and send command to STM32."""
    def get_telemetry(self) -> dict | None:
        """Return latest parsed telemetry. Thread-safe."""
```

Packet format (see `architecture.md` for full struct):
- Header: `[0xAA, 0x55]` for commands, `[0xBB, 0x66]` for telemetry
- CRC8-MAXIM over all bytes except the CRC itself

#### `main.py`
- Entry point.
- Parse command-line args: `--challenge open|obstacle`
- Initialize all modules with `config.py` values.
- Start threads: camera, serial_comm.
- Main loop at 30 Hz: get frame -> vision -> navigator -> state_machine -> send command.
- Handle Ctrl+C gracefully (stop motors, release camera).

```python
def main():
    config = load_config()
    camera = Camera(config)
    vision = Vision(config)
    lane = LaneDetector(config)
    serial = SerialComm(config.SERIAL_PORT, config.SERIAL_BAUD)
    navigator = Navigator(config)
    fsm = StateMachine(config, challenge_type=args.challenge)

    camera.start()
    serial.start()

    try:
        while fsm.state != State.STOPPED:
            frame = camera.get_frame()
            detections = vision.process_frame(frame)
            lines = lane.detect_lines(frame)
            telemetry = serial.get_telemetry()

            nav_steering, nav_speed = navigator.compute(telemetry, detections, args.challenge)
            command = fsm.tick(telemetry, detections, (nav_steering, nav_speed))

            serial.send_command(CMD_SPEED, command['speed'])
            serial.send_command(CMD_STEER, command['steering'])

            time.sleep(1/30)
    finally:
        serial.send_command(CMD_ESTOP, 0)
        camera.stop()
        serial.stop()
```

### STM32 C Firmware (`src/stm32/`)

#### `protocol.h`
- Shared packet definitions (must match Pi side exactly).
- Message type enums, struct definitions, CRC8 function.

```c
#define CMD_HEADER_0    0xAA
#define CMD_HEADER_1    0x55
#define TELEM_HEADER_0  0xBB
#define TELEM_HEADER_1  0x66

#define CMD_SPEED       0x01
#define CMD_STEER       0x02
#define CMD_MODE        0x03
#define CMD_ESTOP       0xFF

#define TELEM_SENSORS   0x10
```

#### `sensors.c`
- Poll 3x TFMini-S via their respective UARTs (USART2, USART3, UART4).
- Poll Adafruit 9-DoF IMU via I2C1.
- Read encoder via TIM3 in encoder mode.
- Apply moving average filter to ToF readings (window size 5).
- Apply complementary filter to IMU (gyro + magnetometer fusion for heading).
- Store all readings in a global `sensor_data` struct accessible by other modules.

**TFMini-S frame**: 9 bytes starting with `[0x59, 0x59]`. Parse distance from bytes 2-3 (little-endian, cm).

**Adafruit 9-DoF IMU**: Uses LSM6DSOX (accel+gyro) + LIS3MDL (magnetometer) or similar. Communicate via I2C at 400 kHz. Read raw registers and convert. Check the specific Adafruit breakout for register map and I2C addresses.

#### `motor_control.c`
- PID controller running at 200 Hz (TIM interrupt or SysTick).
- Input: target RPM from Pi command.
- Feedback: encoder tick count converted to RPM.
- Output: PWM duty cycle to motor driver.
- Anti-windup on integral term (clamp to configurable max).
- Direction pin control for forward/reverse.

```c
void motor_pid_update(void) {
    int32_t current_rpm = get_encoder_rpm();
    int32_t error = target_rpm - current_rpm;
    integral += error;
    integral = CLAMP(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
    int32_t derivative = error - prev_error;
    int32_t output = KP * error + KI * integral + KD * derivative;
    set_motor_pwm(CLAMP(output, -PWM_MAX, PWM_MAX));
    prev_error = error;
}
```

#### `servo.c`
- Map steering angle (degrees, from Pi command) to PWM pulse width.
- Servo center: ~1500 µs. Range: 1000-2000 µs.
- Clamp to mechanical limits of the Ackermann linkage.

```c
void set_steering_angle(int16_t angle_deg) {
    // angle_deg: -30 to +30
    angle_deg = CLAMP(angle_deg, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
    uint16_t pulse_us = SERVO_CENTER_US + (angle_deg * SERVO_US_PER_DEG);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
}
```

#### `comm.c`
- Parse incoming command packets from Pi (USART1 RX interrupt + ring buffer).
- Package telemetry struct and transmit at 100 Hz.
- CRC8 validation on received packets; discard invalid.
- Update `target_rpm` and `target_steering` globals on valid command receipt.

#### `safety.c`
- Watchdog: if no valid command received in 500 ms, set `target_rpm = 0` (emergency stop).
- Battery ADC: read every 1 second. If below `BATTERY_LOW_MV`, stop motors and set status LED.
- Start button: external interrupt on PC13. Sends "started" flag to main loop.

#### `main.c`
- HAL_Init, SystemClock_Config, peripheral initialization (all TIMs, UARTs, I2C, ADC, GPIOs).
- Super-loop or FreeRTOS with tasks:
  - Task 1 (200 Hz): sensor polling + PID control + servo update
  - Task 2 (100 Hz): telemetry transmission
  - Task 3 (continuous): command reception (interrupt-driven)
  - Task 4 (1 Hz): safety checks (battery, watchdog)

## Testing Strategy

When asked to write tests:

1. **Unit tests for vision.py**: Feed synthetic images with known colored rectangles. Assert correct detection and classification.
2. **Unit tests for state_machine.py**: Simulate telemetry sequences and verify state transitions, lap counting, reversal logic.
3. **Unit tests for navigator.py**: Feed mock ToF/vision data and verify steering output is within expected bounds.
4. **Integration test for serial_comm.py**: Loopback test — send a command packet and verify it parses correctly.
5. **STM32 tests**: Use UART debug output to verify sensor readings, PID response to step inputs.

## Common Tasks

### "Tune PID gains"
Edit values in `config.py` (Pi-side PD wall-following) or `motor_control.c` defines (STM32-side speed PID). Never change gains in logic code.

### "Add a new sensor"
1. Wire to STM32.
2. Add polling code in `sensors.c`.
3. Add field to `telemetry_packet_t` in `protocol.h`.
4. Update `serial_comm.py` parser on Pi side.
5. Update `navigator.py` to use the new data.

### "Change camera resolution"
Edit `config.CAMERA_RES` and `config.CAMERA_FPS`. Vision pipeline handles any resolution.

### "Add a new state"
1. Add to `State` enum in `state_machine.py`.
2. Add handler method.
3. Add transitions from/to existing states.
4. Log the transition.

## Do NOT

- Import any networking libraries (socket, requests, flask, etc.)
- Use `time.sleep()` in the STM32 firmware (use timer interrupts)
- Hardcode pin numbers outside of the init functions or defines
- Use floating point on STM32 without verifying FPU is enabled (STM32F4 has hardware FPU — enable it in compiler flags: `-mfloat-abi=hard -mfpu=fpv4-sp-d16`)
- Create any GUI — this runs headless
- Use multithreading on STM32 without FreeRTOS (use interrupts instead for bare-metal)
