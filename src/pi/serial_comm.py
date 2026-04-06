import threading
import struct
import serial
from logger import log

# ─── Packet constants (must match protocol.h) ───
CMD_HEADER   = bytes([0xAA, 0x55])
TELEM_HEADER = bytes([0xBB, 0x66])

CMD_SPEED  = 0x01
CMD_STEER  = 0x02
CMD_MODE   = 0x03
CMD_ESTOP  = 0xFF

TELEM_SENSORS = 0x10

# Telemetry packet payload size in bytes (everything between header and CRC)
# msg_type(1) + length(1) + tof_front(2) + tof_left(2) + tof_right(2)
# + imu_yaw(2) + imu_pitch(2) + imu_roll(2) + encoder_ticks(4) + battery_mv(2) = 20 bytes
_TELEM_PAYLOAD_LEN = 20
_TELEM_TOTAL_LEN   = 2 + _TELEM_PAYLOAD_LEN + 1  # header + payload + CRC


def _crc8_maxim(data: bytes) -> int:
    """CRC-8/MAXIM (Dallas/Maxim 1-wire CRC). Polynomial 0x31, init 0x00, reflect in/out."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc


class SerialComm:
    def __init__(self, port='/dev/ttyAMA0', baudrate=115200):
        self._port     = port
        self._baudrate = baudrate
        self._ser      = None
        self._running  = False

        self._telemetry      = None   # latest parsed telemetry dict
        self._telem_lock     = threading.Lock()

        self._cmd_queue      = []     # list of (msg_type, value) tuples
        self._cmd_lock       = threading.Lock()

        self._rx_thread = None
        self._tx_thread = None

    def start(self):
        """Open serial port and launch RX/TX threads."""
        self._ser = serial.Serial(self._port, self._baudrate, timeout=0.01)
        self._running = True

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True, name='serial-rx')
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True, name='serial-tx')
        self._rx_thread.start()
        self._tx_thread.start()
        log.info("SerialComm started on %s @%d baud", self._port, self._baudrate)

    def send_command(self, msg_type: int, value: int):
        """Queue a command for the TX thread to send."""
        with self._cmd_lock:
            self._cmd_queue.append((msg_type, value))

    def get_telemetry(self) -> dict | None:
        """Return the most recent telemetry dict. Thread-safe."""
        with self._telem_lock:
            return self._telemetry.copy() if self._telemetry else None

    def stop(self):
        self._running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=2)
        if self._tx_thread:
            self._tx_thread.join(timeout=2)
        if self._ser and self._ser.is_open:
            self._ser.close()
        log.info("SerialComm stopped")

    # ─── Internal threads ────────────────────────────────────────────────────────

    def _tx_loop(self):
        """Send queued commands at up to 50 Hz."""
        import time
        interval = 1.0 / 50
        while self._running:
            t0 = time.monotonic()
            with self._cmd_lock:
                cmds = self._cmd_queue[:]
                self._cmd_queue.clear()

            for (msg_type, value) in cmds:
                pkt = self._build_command(msg_type, value)
                try:
                    self._ser.write(pkt)
                except serial.SerialException as e:
                    log.error("TX error: %s", e)

            elapsed = time.monotonic() - t0
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def _rx_loop(self):
        """Continuously read bytes from STM32 and parse telemetry packets."""
        buf = bytearray()
        while self._running:
            try:
                chunk = self._ser.read(64)
            except serial.SerialException as e:
                log.error("RX error: %s", e)
                continue

            buf.extend(chunk)

            # Scan for header 0xBB 0x66
            while len(buf) >= _TELEM_TOTAL_LEN:
                idx = buf.find(b'\xBB\x66')
                if idx == -1:
                    buf.clear()
                    break
                if idx > 0:
                    del buf[:idx]   # discard garbage before header

                if len(buf) < _TELEM_TOTAL_LEN:
                    break   # wait for more bytes

                pkt = buf[:_TELEM_TOTAL_LEN]
                expected_crc = _crc8_maxim(pkt[:-1])
                if pkt[-1] == expected_crc:
                    telem = self._parse_telemetry(pkt)
                    with self._telem_lock:
                        self._telemetry = telem
                else:
                    log.warning("Telemetry CRC mismatch — discarding packet")

                del buf[:_TELEM_TOTAL_LEN]

    # ─── Packet construction / parsing ───────────────────────────────────────────

    def _build_command(self, msg_type: int, value: int) -> bytes:
        """Build a command_packet_t byte string with CRC."""
        # header(2) + msg_type(1) + length(1) + value(2) = 6 bytes before CRC
        payload = struct.pack('<BBBBh', 0xAA, 0x55, msg_type, 2, value)
        crc = _crc8_maxim(payload)
        return payload + bytes([crc])

    def _parse_telemetry(self, pkt: bytes) -> dict:
        """Unpack a telemetry_packet_t into a plain dict."""
        # Skip header(2), msg_type(1), length(1) — start reading payload at offset 4
        (tof_front, tof_left, tof_right,
         imu_yaw, imu_pitch, imu_roll,
         encoder_ticks,
         battery_mv) = struct.unpack_from('<HHHhhhiH', pkt, 4)

        return {
            'tof_front':     tof_front,      # mm
            'tof_left':      tof_left,       # mm
            'tof_right':     tof_right,      # mm
            'imu_yaw':       imu_yaw,        # deg * 100
            'imu_pitch':     imu_pitch,
            'imu_roll':      imu_roll,
            'gyro_z':        imu_yaw / 100.0,  # rough proxy — replace with actual gyro field if added
            'encoder_ticks': encoder_ticks,
            'battery_mv':    battery_mv,
        }
