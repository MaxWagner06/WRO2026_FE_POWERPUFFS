# WRO Future Engineers 2026 — Software Architecture

## System Overview

Dual-controller architecture: **Raspberry Pi 5 (16GB)** handles perception and decision-making; **STM32F4 Nucleo** handles real-time motor control and sensor I/O. They communicate over UART at 115200 baud using a binary packet protocol.

```
┌─────────────────────────────────────────────────────────┐
│                   RASPBERRY PI 5 (16GB)                 │
│                                                         │
│  ┌──────────┐  ┌───────────┐  ┌───────────────────────┐ │
│  │ camera.py│  │ vision.py │  │     lane.py           │ │
│  │ (capture)│─>│ (HSV seg) │  │ (orange/blue detect)  │ │
│  └──────────┘  └─────┬─────┘  └──────────┬────────────┘ │
│                      │                   │              │
│                      v                   v              │
│              ┌───────────────────────────────┐          │
│              │       navigator.py            │          │
│              │  (wall-follow + avoidance     │          │
│              │   path generation)            │          │
│              └──────────────┬────────────────┘          │
│                             │                           │
│                             v                           │
│              ┌───────────────────────────────┐          │
│              │     state_machine.py          │          │
│              │  (FSM: IDLE, DRIVING,         │          │
│              │   AVOIDING, REVERSING,        │          │
│              │   PARKING, STOPPING, STOPPED) │          │
│              └──────────────┬────────────────┘          │
│                             │                           │
│                             v                           │
│              ┌───────────────────────────────┐          │
│              │     serial_comm.py            │          │
│              │  (UART tx: commands           │          │
│              │   UART rx: telemetry)         │          │
│              └──────────────┬────────────────┘          │
│                             │                           │
└─────────────────────────────┼───────────────────────────┘
                              │ UART (115200 baud)
                              │ Binary packets + CRC8
┌─────────────────────────────┼───────────────────────────┐
│                   STM32F4 NUCLEO                        │
│                             │                           │
│              ┌──────────────┴────────────────┐          │
│              │        comm.c                 │          │
│              │  (parse commands,             │          │
│              │   package telemetry)          │          │
│              └──────────────┬────────────────┘          │
│                             │                           │
│           ┌─────────────────┼─────────────────┐         │
│           v                 v                 v         │
│  ┌──────────────┐ ┌──────────────┐ ┌─────────────────┐ │
│  │motor_control │ │  servo.c     │ │   sensors.c     │ │
│  │  .c          │ │  (steering   │ │ (3x TFMini-S    │ │
│  │ (PID speed   │ │   PWM)       │ │  via UART,      │ │
│  │  control)    │ │              │ │  Adafruit 9DoF  │ │
│  └──────┬───────┘ └──────┬───────┘ │  via I2C,       │ │
│         │                │         │  encoder via     │ │
│         v                v         │  timer capture)  │ │
│    DC Motor+Driver  Servo Motor    └─────────────────┘  │
│                                                         │
│              ┌───────────────────────────────┐          │
│              │       safety.c                │          │
│              │  (watchdog, battery ADC,      │          │
│              │   emergency stop)             │          │
│              └───────────────────────────────┘          │
└─────────────────────────────────────────────────────────┘
```

## Repository Structure

```
wro-fe-2026/
├── README.md                    # 5000+ char English (required by WRO)
├── src/
│   ├── pi/                      # Raspberry Pi 5 Python code
│   │   ├── main.py              # Entry point, thread orchestration
│   │   ├── camera.py            # Pi Camera Module 3 Wide capture
│   │   ├── vision.py            # HSV segmentation, contour detection
│   │   ├── lane.py              # Orange/blue line detection
│   │   ├── navigator.py         # Wall-follow + avoidance controller
│   │   ├── state_machine.py     # Finite state machine
│   │   ├── serial_comm.py       # UART protocol handler
│   │   ├── config.py            # All tunable parameters (HSV ranges, PID gains, etc.)
│   │   └── logger.py            # Telemetry logging for debugging
│   │
│   └── stm32/                   # STM32F4 C firmware
│       ├── Core/
│       │   ├── Src/
│       │   │   ├── main.c           # Init + super-loop / FreeRTOS tasks
│       │   │   ├── motor_control.c  # PID speed controller
│       │   │   ├── servo.c          # Ackermann steering PWM
│       │   │   ├── sensors.c        # TFMini-S + IMU + encoder polling
│       │   │   ├── comm.c           # UART protocol (Pi link)
│       │   │   ├── safety.c         # Watchdog + battery monitor
│       │   │   └── filters.c        # Moving average, complementary filter
│       │   └── Inc/
│       │       ├── motor_control.h
│       │       ├── servo.h
│       │       ├── sensors.h
│       │       ├── comm.h
│       │       ├── safety.h
│       │       ├── filters.h
│       │       └── protocol.h   # Shared packet definitions
│       └── Makefile / .ioc      # STM32CubeMX project or Makefile
│
├── models/                      # 3D print STL / CAD files
├── schemes/                     # Wiring diagrams (Fritzing / KiCad)
├── photos/                      # Vehicle photos (6 angles + team)
├── video/                       # YouTube links file
└── docs/
    ├── engineering_doc.pdf       # Printed engineering document
    └── strategy.pdf             # This strategy document
```

## Hardware Connections

### STM32F4 Nucleo Pin Allocation

| Peripheral | STM32 Resource | Pin(s)         | Connected To             |
|------------|---------------|----------------|--------------------------|
| USART1     | TX/RX         | PA9/PA10       | Raspberry Pi 5 (UART)    |
| USART2     | RX            | PA3            | TFMini-S #1 (Front)      |
| USART3     | RX            | PB11           | TFMini-S #2 (Left)       |
| UART4      | RX            | PA1            | TFMini-S #3 (Right)      |
| I2C1       | SCL/SDA       | PB6/PB7        | Adafruit 9-DoF IMU       |
| TIM1 CH1   | PWM           | PA8            | Servo motor (steering)   |
| TIM2 CH1   | PWM           | PA0            | Motor driver (speed PWM) |
| TIM3       | Encoder mode  | PA6/PA7        | Rotary encoder A/B       |
| GPIO       | Output        | PB0            | Motor driver DIR pin     |
| GPIO       | Input (EXTI)  | PC13           | Start button             |
| ADC1       | Analog in     | PA4            | Battery voltage divider  |
| GPIO       | Output        | PA5 (LED)      | Status LED               |

### UART Protocol Packets

**Pi → STM32 (Commands, 50 Hz):**

```c
// Header: 0xAA 0x55
// Packet structure:
typedef struct {
    uint8_t  header[2];     // 0xAA, 0x55
    uint8_t  msg_type;      // CMD_SPEED=0x01, CMD_STEER=0x02, CMD_MODE=0x03, CMD_ESTOP=0xFF
    uint8_t  length;        // payload length
    int16_t  value;         // speed: RPM (-500..500), steer: degrees (-30..30), mode: enum
    uint8_t  crc8;          // CRC-8/MAXIM
} command_packet_t;
```

**STM32 → Pi (Telemetry, 100 Hz):**

```c
typedef struct {
    uint8_t  header[2];     // 0xBB 0x66
    uint8_t  msg_type;      // TELEM_SENSORS=0x10
    uint8_t  length;
    uint16_t tof_front;     // mm
    uint16_t tof_left;      // mm
    uint16_t tof_right;     // mm
    int16_t  imu_yaw;       // degrees * 100
    int16_t  imu_pitch;     // degrees * 100
    int16_t  imu_roll;      // degrees * 100
    int32_t  encoder_ticks; // cumulative
    uint16_t battery_mv;    // millivolts
    uint8_t  crc8;
} telemetry_packet_t;
```

## State Machine

```
                         ┌──────────┐
                         │   IDLE   │
                         └────┬─────┘
                    button press │
                              v
                    ┌────────────────┐
          ┌────────>│    DRIVING     │<───────────────┐
          │         └───┬──────┬──┬──┘                │
          │    pillar   │      │  │ 3 laps done       │
          │   detected  │      │  │ (obstacle)        │
          │             v      │  v                   │
          │   ┌──────────┐   │  ┌──────────────┐     │
          │   │ AVOIDING │   │  │PARKING_SEARCH│     │
          │   └────┬─────┘   │  └──────┬───────┘     │
          │ pillar │         │  magenta│              │
          │ cleared│         │ detected│              │
          └────────┘         │         v              │
                             │  ┌──────────────┐     │
            3 laps (open)    │  │PARKING_EXEC  │     │
                             │  └──────┬───────┘     │
                             v         │              │
                    ┌──────────┐       │              │
                    │ STOPPING │       │              │
                    └────┬─────┘       │              │
                         │             │              │
                         v             v              │
                    ┌──────────────────────┐          │
                    │       STOPPED        │          │
                    └──────────────────────┘          │
                                                      │
                    ┌──────────┐    heading reversed   │
          red last  │REVERSING │──────────────────────┘
          sign at   └──────────┘
          lap 3
          (from DRIVING)
```

## Key Algorithms

### 1. PD Wall-Following (navigator.py)

```python
def compute_steering(tof_left, tof_right, tof_front, prev_error, dt):
    # Lane centering
    error = (tof_right - tof_left) - CENTER_OFFSET
    derivative = (error - prev_error) / dt

    steering = KP_STRAIGHT * error + KD_STRAIGHT * derivative

    # Corner pre-emption: if front sensor detects wall approach
    if tof_front < CORNER_THRESHOLD_MM:
        # Determine turn direction from current heading or line detection
        steering = CORNER_STEERING_ANGLE * turn_direction
        speed = CORNER_SPEED
    else:
        speed = STRAIGHT_SPEED

    return clamp(steering, -MAX_STEER, MAX_STEER), speed, error
```

### 2. Pillar Avoidance (navigator.py)

```python
def compute_avoidance(pillar, wall_steer, pillar_distance_estimate):
    # pillar.color: 'red' or 'green'
    # pillar.cx: horizontal centroid in frame (0=left, 640=right)

    if pillar.color == 'red':
        # Pass LEFT of red pillar
        avoidance_steer = -AVOIDANCE_OFFSET
    else:
        # Pass RIGHT of green pillar
        avoidance_steer = +AVOIDANCE_OFFSET

    # Blend: closer pillar = stronger avoidance
    alpha = max(0.1, min(1.0, pillar_distance_estimate / BLEND_DISTANCE))
    final_steer = alpha * wall_steer + (1 - alpha) * avoidance_steer

    return final_steer
```

### 3. IMU Complementary Filter (sensors.c)

```c
void update_heading(float gyro_z, float mag_heading, float dt) {
    // Integrate gyro
    heading_gyro += gyro_z * dt;

    // Complementary filter
    heading = ALPHA * heading_gyro + (1.0f - ALPHA) * mag_heading;
    // ALPHA = 0.98 (trust gyro short-term, mag long-term)
}
```

### 4. TFMini-S Parsing (sensors.c)

```c
// TFMini-S UART frame: 9 bytes
// [0x59] [0x59] [Dist_L] [Dist_H] [Strength_L] [Strength_H] [Temp_L] [Temp_H] [Checksum]
bool parse_tfmini(uint8_t *buf, uint16_t *distance_cm) {
    if (buf[0] != 0x59 || buf[1] != 0x59) return false;
    uint8_t checksum = 0;
    for (int i = 0; i < 8; i++) checksum += buf[i];
    if (checksum != buf[8]) return false;
    *distance_cm = buf[2] | (buf[3] << 8);
    return true;
}
```

## Configuration (config.py)

All tunable parameters in one file for rapid adjustment during practice:

```python
# ─── PID Gains ───
KP_STRAIGHT = 0.5
KD_STRAIGHT = 0.1
KP_CORNER = 1.2
KD_CORNER = 0.3
SPEED_STRAIGHT = 300   # RPM
SPEED_CORNER = 150     # RPM

# ─── ToF Thresholds ───
CORNER_THRESHOLD_MM = 800
CENTER_OFFSET = 0       # 0 = centered, positive = bias right
WALL_DANGER_MM = 80     # emergency correction threshold

# ─── HSV Color Ranges ───
HSV_RED_LOWER_1 = (0, 100, 80)
HSV_RED_UPPER_1 = (10, 255, 255)
HSV_RED_LOWER_2 = (170, 100, 80)
HSV_RED_UPPER_2 = (180, 255, 255)
HSV_GREEN_LOWER = (35, 100, 80)
HSV_GREEN_UPPER = (85, 255, 255)
HSV_MAGENTA_LOWER = (140, 100, 80)
HSV_MAGENTA_UPPER = (170, 255, 255)
HSV_ORANGE_LOWER = (10, 150, 150)
HSV_ORANGE_UPPER = (25, 255, 255)
HSV_BLUE_LOWER = (100, 150, 100)
HSV_BLUE_UPPER = (130, 255, 255)

# ─── Vision ───
CAMERA_RES = (640, 480)
CAMERA_FPS = 30
MIN_CONTOUR_AREA = 500
MAX_CONTOUR_AREA = 50000

# ─── Avoidance ───
AVOIDANCE_OFFSET = 15.0    # degrees
BLEND_DISTANCE = 1000.0    # mm — full avoidance when pillar closer than this

# ─── Parking ───
PARKING_OVERSHOOT_MM = 200
PARKING_REVERSE_ANGLE = -25  # degrees
PARKING_CLEARANCE_MIN = 20   # mm — abort if closer

# ─── Safety ───
WATCHDOG_TIMEOUT_MS = 500
BATTERY_LOW_MV = 6800       # 2S LiPo cutoff
```

## Threading Model (Pi 5)

```
main.py
  │
  ├── Thread 1: Camera + Vision (30 Hz)
  │     camera.py -> vision.py -> lane.py
  │     Writes to: shared_state.detected_objects, shared_state.lines
  │
  ├── Thread 2: Serial Comm (100 Hz rx, 50 Hz tx)
  │     serial_comm.py
  │     Reads from: shared_state.command
  │     Writes to: shared_state.telemetry
  │
  └── Main Thread: Decision Loop (30 Hz)
        navigator.py + state_machine.py
        Reads from: shared_state.detected_objects, shared_state.telemetry
        Writes to: shared_state.command
```

Shared state is a simple Python object protected by a threading.Lock. At 30 Hz with lightweight data structures, lock contention is negligible.
