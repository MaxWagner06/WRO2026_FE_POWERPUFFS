#!/usr/bin/env python3
"""
Entry point for WRO Future Engineers 2026 — PowerPuffs autonomous vehicle.
Run with:
    python main.py --challenge open
    python main.py --challenge obstacle
"""

import argparse
import time
import sys

import config
from camera      import Camera
from vision      import Vision
from lane        import LaneDetector
from navigator   import Navigator
from state_machine import StateMachine, State
from serial_comm import SerialComm, CMD_SPEED, CMD_STEER, CMD_ESTOP
from logger      import log


def parse_args():
    parser = argparse.ArgumentParser(description='WRO FE 2026 PowerPuffs')
    parser.add_argument('--challenge', choices=['open', 'obstacle'], required=True,
                        help='Which challenge to run')
    return parser.parse_args()


def main():
    args = parse_args()
    log.info("Starting — challenge: %s", args.challenge)

    # ── Module init ──────────────────────────────────────────────────────────────
    cam    = Camera(config)
    vision = Vision(config)
    lane   = LaneDetector(config)
    serial = SerialComm(config.SERIAL_PORT, config.SERIAL_BAUD)
    nav    = Navigator(config)
    fsm    = StateMachine(config, challenge_type=args.challenge)

    # ── Start background threads ─────────────────────────────────────────────────
    cam.start()
    serial.start()

    # Wait for camera to produce the first frame
    for _ in range(30):
        if cam.get_frame() is not None:
            break
        time.sleep(0.1)
    else:
        log.error("Camera never produced a frame — exiting")
        cam.stop()
        serial.stop()
        sys.exit(1)

    log.info("Camera ready. Waiting for start button...")

    # ── Main decision loop (30 Hz) ───────────────────────────────────────────────
    loop_interval = 1.0 / config.MAIN_LOOP_HZ

    try:
        while fsm.state != State.STOPPED:
            t0 = time.monotonic()

            frame     = cam.get_frame()
            telemetry = serial.get_telemetry()

            if frame is None:
                time.sleep(loop_interval)
                continue

            # Vision pipeline
            detections = vision.process_frame(frame)
            lines      = lane.detect_lines(frame)

            # Lock direction from first confirmed line reading
            # (pass lines into fsm if you want it to use camera-based direction)

            # Navigator output depends on whether we're in a corner
            nav_steering, nav_speed = nav.compute(
                telemetry, detections, args.challenge,
                in_corner=fsm.in_corner,
            )

            # FSM decides final command
            command = fsm.tick(telemetry, detections, (nav_steering, nav_speed))

            # Send to STM32
            serial.send_command(CMD_SPEED, command['speed'])
            serial.send_command(CMD_STEER, command['steering'])

            # Pace the loop
            elapsed = time.monotonic() - t0
            sleep_t = loop_interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        log.info("KeyboardInterrupt — stopping")

    finally:
        log.info("Sending emergency stop")
        serial.send_command(CMD_ESTOP, 0)
        time.sleep(0.1)   # allow the TX thread one last cycle
        cam.stop()
        serial.stop()
        log.info("Shutdown complete")


if __name__ == '__main__':
    main()
