# waveshare_st_client.py
import atexit
import logging
import time
from typing import Optional, Sequence, Tuple

import numpy as np

# STServo SDK (vendored inside the package)
from .STservo_sdk.port_handler import PortHandler
from .STservo_sdk.sts import sts as STS
from .STservo_sdk.group_sync_read import GroupSyncRead
from .STservo_sdk.group_sync_write import GroupSyncWrite

# ---- Control table (STS) ----
STS_TORQUE_ENABLE        = 40  # 1B
STS_GOAL_POSITION        = 42  # 2B
STS_GOAL_TIME            = 44  # 2B (unused for now)
STS_PRESENT_POSITION     = 56  # 2B
STS_PRESENT_SPEED        = 58  # 2B (signed)
STS_PRESENT_LOAD         = 60  # 2B (signed)
STS_PRESENT_TEMPERATURE  = 63  # 1B
STS_ACC                  = 41
STS_GOAL_SPEED           = 46 

LEN_GOAL_POSITION        = 2
LEN_PRESENT_POSITION     = 2
LEN_PRESENT_SPEED        = 2
LEN_PRESENT_LOAD         = 2
LEN_PRESENT_TEMP         = 1

# Scales to match your old Dynamixel client behavior
DEFAULT_POS_SCALE  = 2.0 * np.pi / 4096.0
DEFAULT_VEL_SCALE  = (2.0 * np.pi) / 4096.0  # rough; refine if desired
DEFAULT_LOAD_SCALE = 1.0

def _u_to_s(value: int, size_bytes: int) -> int:
    bits = 8 * size_bytes
    return value - (1 << bits) if (value & (1 << (bits - 1))) else value
def _decode_st_load(raw_16: int) -> float:
    """
    ST/Feetech present load:
      bits 0..9  : magnitude (0..1023)
      bit 10     : direction (1 = negative, 0 = positive)
      bits 11..15: unused
    Returns a normalized signed value in [-1.0, 1.0].
    """
    mag = raw_16 & 0x03FF
    sign = -1.0 if (raw_16 & 0x0400) else 1.0
    return sign * (mag / 1023.0)


class WaveshareStClient:
    """Drop-in replacement for DynamixelClient, using Waveshare/Feetech STS protocol."""

    OPEN_CLIENTS = set()

    def __init__(
        self,
        motor_ids: Sequence[int],
        port: str = "/dev/ttyACM0",
        baudrate: int = 1_000_000,
        lazy_connect: bool = False,
        pos_scale: Optional[float] = None,
        vel_scale: Optional[float] = None,
        load_scale: Optional[float] = None,
    ):
        self.motor_ids = list(motor_ids)
        self.port_name = port
        self.baudrate = baudrate
        self.lazy_connect = lazy_connect

        self.port_handler = PortHandler(self.port_name)
        self.proto = STS(self.port_handler)

        self._pos_scale = DEFAULT_POS_SCALE if pos_scale is None else pos_scale
        self._vel_scale = DEFAULT_VEL_SCALE if vel_scale is None else vel_scale
        self._load_scale = DEFAULT_LOAD_SCALE if load_scale is None else load_scale

        self._gread: Optional[GroupSyncRead] = None
        self._gw_pos: Optional[GroupSyncWrite] = None

        self._gw_speed = GroupSyncWrite(self.proto, STS_GOAL_SPEED, LEN_PRESENT_SPEED)  # 2 bytes
        self._gw_acc   = GroupSyncWrite(self.proto, STS_ACC, 1)                         # 1 byte

        self.OPEN_CLIENTS.add(self)

    # ---- lifecycle ----------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        # PortHandler in this SDK doesn't expose an "is_open" flag; be conservative.
        return hasattr(self.port_handler, "ser") and self.port_handler.ser is not None

    def connect(self):
        assert not self.is_connected, "Client is already connected."
        if not self.port_handler.openPort():
            raise OSError(f"Failed to open port {self.port_name}")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise OSError(f"Failed to set baudrate {self.baudrate}")
        logging.info("Opened %s @ %d", self.port_name, self.baudrate)

        # Optional: reboot motors (best effort)
        self.reboot_motors(self.motor_ids)
        time.sleep(0.5)

        # Enable torque
        self.set_torque_enabled(self.motor_ids, True)

        # Prepare group ops
        self._init_group_ops()

    def disconnect(self):
        if not self.is_connected:
            return
        try:
            self.set_torque_enabled(self.motor_ids, False)
        except Exception:
            pass
        self.port_handler.closePort()
        if self in self.OPEN_CLIENTS:
            self.OPEN_CLIENTS.remove(self)

    def __enter__(self):
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    def __del__(self):
        try:
            self.disconnect()
        except Exception:
            pass

    # ---- helpers ------------------------------------------------------------
    def _init_group_ops(self):
        # Correct signature: GroupSyncRead(proto, start_addr, data_len)
        # We'll read one contiguous block: position..temperature (56..64 inclusive => 9 bytes)
        start = STS_PRESENT_POSITION
        length = (STS_PRESENT_TEMPERATURE - STS_PRESENT_POSITION) + 1  # 64-56+1 = 9
        self._gread = GroupSyncRead(self.proto, start, length)
        for mid in self.motor_ids:
            self._gread.addParam(mid)

        # Correct signature: GroupSyncWrite(proto, start_addr, data_len)
        self._gw_pos = GroupSyncWrite(self.proto, STS_GOAL_POSITION, LEN_GOAL_POSITION)

    # ---- API expected by GripperController ---------------------------------
    def reboot_motors(self, motor_ids: Sequence[int]):
        for mid in motor_ids:
            try:
                self.proto.reboot(mid)
            except Exception as e:
                logging.debug("Reboot failed/unsupported on ID %d: %s", mid, e)

    def set_torque_enabled(
        self,
        motor_ids: Sequence[int],
        enabled: bool,
        retries: int = -1,
        retry_interval: float = 0.25,
    ):
        remaining = list(motor_ids)
        while remaining:
            failed = []
            for mid in remaining:
                try:
                    rc, _ = self.proto.write1ByteTxRx(mid, STS_TORQUE_ENABLE, 1 if enabled else 0)
                    if rc != 0:
                        failed.append(mid)
                except Exception:
                    failed.append(mid)
            if not failed:
                break
            if retries == 0:
                logging.error("Could not set torque %s for IDs: %s",
                              "enabled" if enabled else "disabled", failed)
                break
            retries -= 1
            time.sleep(retry_interval)
            remaining = failed

    def write_desired_pos(self, motor_ids: Sequence[int], positions: np.ndarray):
        assert len(motor_ids) == len(positions)
        ticks = (positions / self._pos_scale).astype(int)
        self._gw_pos.clearParam()
        for mid, val in zip(motor_ids, ticks):
            val = max(0, min(4095, int(val)))
            raw = int(val).to_bytes(LEN_GOAL_POSITION, "little", signed=False)
            ok = self._gw_pos.addParam(mid, raw)
            if not ok:
                logging.error("GroupSyncWrite addParam failed for ID %d", mid)
        self._gw_pos.txPacket()
        self._gw_pos.clearParam()
        return [time.monotonic()]

    def write_profile_velocity(self, motor_ids: Sequence[int], profile_velocity: np.ndarray, acc: int = None):
        # Write goal speed
        self._gw_speed.clearParam()
        for mid, spd in zip(motor_ids, profile_velocity):
            v = int(max(0, min(1023, spd)))  # STS speed range
            raw = v.to_bytes(2, "little", signed=False)
            ok = self._gw_speed.addParam(mid, raw)
            if not ok:
                logging.error("GroupSyncWrite(speed) addParam failed for ID %d", mid)
        self._gw_speed.txPacket()
        self._gw_speed.clearParam()

        # Optionally set acceleration
        if acc is not None:
            a = int(max(0, min(255, acc)))
            self._gw_acc.clearParam()
            for mid in motor_ids:
                ok = self._gw_acc.addParam(mid, bytes([a]))
                if not ok:
                    logging.error("GroupSyncWrite(acc) addParam failed for ID %d", mid)
            self._gw_acc.txPacket()
            self._gw_acc.clearParam()

    
    def read_pos_vel_load(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        pos = np.zeros(len(self.motor_ids), dtype=np.float32)
        vel = np.zeros(len(self.motor_ids), dtype=np.float32)
        load = np.zeros(len(self.motor_ids), dtype=np.float32)

        if self._gread:
            # Group read path
            self._gread.txRxPacket()
            for i, mid in enumerate(self.motor_ids):
                try:
                    # Availability checks; fall back if any fails
                    ok_pos = self._gread.isAvailable(mid, STS_PRESENT_POSITION, LEN_PRESENT_POSITION)
                    ok_vel = self._gread.isAvailable(mid, STS_PRESENT_SPEED, LEN_PRESENT_SPEED)
                    ok_load = self._gread.isAvailable(mid, STS_PRESENT_LOAD, LEN_PRESENT_LOAD)
                    if ok_pos:
                        p_raw = self._gread.getData(mid, STS_PRESENT_POSITION, LEN_PRESENT_POSITION)
                        pos[i] = float(_u_to_s(p_raw, 2)) * self._pos_scale
                    if ok_vel:
                        v_raw = self._gread.getData(mid, STS_PRESENT_SPEED, LEN_PRESENT_SPEED)
                        vel[i] = float(_u_to_s(v_raw, 2)) * self._vel_scale
                    if ok_load:
                        l_raw = self._gread.getData(mid, STS_PRESENT_LOAD, LEN_PRESENT_LOAD)
                        load[i] = _decode_st_load(int(l_raw)) * self._load_scale
                except Exception as e:
                    logging.debug("Group read fallback for ID %d due to %s", mid, e)
                    self._read_single_into(mid, i, pos, vel, load)
        else:
            # Per-ID path
            for i, mid in enumerate(self.motor_ids):
                self._read_single_into(mid, i, pos, vel, load)

        return pos, vel, load

    def read_pos_vel_load_temp(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        pos, vel, load = self.read_pos_vel_load()  # reuse the existing logic
        temps = np.zeros(len(self.motor_ids), dtype=np.float32)

        if self._gread:
            # We already txRx’d in read_pos_vel_load(); call again to keep this atomic if you prefer:
            self._gread.txRxPacket()
            for i, mid in enumerate(self.motor_ids):
                try:
                    ok_t = self._gread.isAvailable(mid, STS_PRESENT_TEMPERATURE, LEN_PRESENT_TEMP)
                    if ok_t:
                        t_raw = self._gread.getData(mid, STS_PRESENT_TEMPERATURE, LEN_PRESENT_TEMP)
                        temps[i] = float(t_raw)
                    else:
                        # fallback single read
                        t_raw, rc, _ = self.proto.read1ByteTxRx(mid, STS_PRESENT_TEMPERATURE)
                        if rc == 0:
                            temps[i] = float(t_raw)
                except Exception as e:
                    logging.error("read_pos_vel_load_temp: temp read failed for ID %d: %s", mid, e)
        else:
            # no group read prepared; per-ID
            for i, mid in enumerate(self.motor_ids):
                try:
                    t_raw, rc, _ = self.proto.read1ByteTxRx(mid, STS_PRESENT_TEMPERATURE)
                    if rc == 0:
                        temps[i] = float(t_raw)
                except Exception as e:
                    logging.error("read_pos_vel_load_temp: temp read failed for ID %d: %s", mid, e)

        return pos, vel, load, temps


    def _read_single_into(self, mid: int, i: int,
                          pos: np.ndarray, vel: np.ndarray, load: np.ndarray):
        try:
            p_raw, rc1, _ = self.proto.read2ByteTxRx(mid, STS_PRESENT_POSITION)
            v_raw, rc2, _ = self.proto.read2ByteTxRx(mid, STS_PRESENT_SPEED)
            l_raw, rc3, _ = self.proto.read2ByteTxRx(mid, STS_PRESENT_LOAD)
            if rc1 == 0:
                pos[i] = float(_u_to_s(p_raw, 2)) * self._pos_scale
            if rc2 == 0:
                vel[i] = float(_u_to_s(v_raw, 2)) * self._vel_scale
            if rc3 == 0:
                load[i] = _decode_st_load(int(l_raw)) * self._load_scale
        except Exception as e:
            logging.error("read_pos_vel_load (single) failed for ID %d: %s", mid, e)

    def read_temperature(self) -> np.ndarray:
        temps = np.zeros(len(self.motor_ids), dtype=np.float32)
        for i, mid in enumerate(self.motor_ids):
            try:
                t_raw, rc, _ = self.proto.read1ByteTxRx(mid, STS_PRESENT_TEMPERATURE)
                if rc == 0:
                    temps[i] = float(t_raw)
            except Exception as e:
                logging.error("read_temperature failed for ID %d: %s", mid, e)
        return temps



def _cleanup():
    for client in list(WaveshareStClient.OPEN_CLIENTS):
        try:
            client.disconnect()
        except Exception:
            pass

atexit.register(_cleanup)
