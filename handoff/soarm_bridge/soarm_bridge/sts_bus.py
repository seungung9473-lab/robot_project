"""
sts_bus.py — Feetech STS 서보 버스 드라이버 (ROS 없이도 동작)

패킷:  FF FF ID LEN INSTR PARAM... CHK      LEN = PARAM 수 + 2,  CHK = ~(ID+LEN+INSTR+PARAM합) & 0xFF
응답:  FF FF ID LEN ERR  DATA...  CHK
16비트 값은 little-endian (낮은 바이트 먼저)

레지스터 (우리가 쓰는 것만)
  5  ID (EEPROM)          40 토크 (0/1 만! 128 쓰지 말 것)    41 가속도 (x100 count/s²)
  42 목표 위치 (2B)        44 목표 시간 (2B, 안 씀 = 0)        46 목표 속도 (2B, count/s)
  55 EEPROM 잠금 (0=풀림, 1=잠금)
  56 현재 위치 (2B)        58 현재 속도 (2B)                  62 전압 (x0.1 V)   63 온도 (°C)
"""
import threading
import time

import serial

INST_PING, INST_READ, INST_WRITE, INST_SYNC_WRITE = 0x01, 0x02, 0x03, 0x83
BROADCAST = 0xFE

ADDR_ID, ADDR_TORQUE, ADDR_ACC, ADDR_GOAL_POS = 5, 40, 41, 42
ADDR_LOCK, ADDR_PRES_POS, ADDR_PRES_SPEED, ADDR_VOLT, ADDR_TEMP = 55, 56, 58, 62, 63


def _chk(body):
    return (~sum(body)) & 0xFF


def make_packet(sid, inst, params=()):
    body = [sid, len(params) + 2, inst, *params]
    return bytes([0xFF, 0xFF, *body, _chk(body)])


def _s16(v):
    """STS 위치·속도는 15번 비트가 방향(부호) 비트"""
    return -(v & 0x7FFF) if v & 0x8000 else v


class STSBus:
    def __init__(self, port='/dev/servo_bus', baud=1_000_000, timeout=0.02):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.lock = threading.Lock()

    def close(self):
        self.ser.close()

    # ---------------- 저수준
    def _txrx(self, pkt, sid, nread):
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write(pkt)
            if sid == BROADCAST or nread is None:
                return None
            head = self._read_exact(5)                 # FF FF ID LEN ERR
            if head is None or head[0] != 0xFF or head[1] != 0xFF or head[2] != sid:
                return None
            ln = head[3]
            rest = self._read_exact(ln - 1)            # DATA... CHK
            if rest is None:
                return None
            body = list(head[2:]) + list(rest[:-1])
            if _chk(body) != rest[-1]:
                return None
            return bytes(rest[:-1])                    # DATA

    def _read_exact(self, n):
        buf = b''
        t0 = time.monotonic()
        while len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if chunk:
                buf += chunk
            elif time.monotonic() - t0 > 0.05:
                return None
        return buf

    def ping(self, sid):
        return self._txrx(make_packet(sid, INST_PING), sid, 0) is not None

    def read(self, sid, addr, n):
        return self._txrx(make_packet(sid, INST_READ, (addr, n)), sid, n)

    def write(self, sid, addr, data):
        return self._txrx(make_packet(sid, INST_WRITE, (addr, *data)), sid, 0)

    # ---------------- 고수준
    def read_state(self, sid):
        """(위치 count, 속도, 전압 V, 온도 °C) — 실패하면 None"""
        d = self.read(sid, ADDR_PRES_POS, 8)           # 56..63
        if d is None or len(d) < 8:
            return None
        pos = d[0] | (d[1] << 8)
        spd = _s16(d[2] | (d[3] << 8))
        return pos, spd, d[6] / 10.0, d[7]

    def read_pos(self, sid):
        d = self.read(sid, ADDR_PRES_POS, 2)
        return None if d is None else d[0] | (d[1] << 8)

    def torque(self, sid, on):
        self.write(sid, ADDR_TORQUE, (1 if on else 0,))   # 0/1 만 씀 (128 금지)

    def sync_goal(self, goals, acc=30):
        """goals: {id: (pos_count, speed_count_per_s)} → 한 패킷으로 동시에 (41~47, 7바이트)"""
        params = [ADDR_ACC, 7]
        for sid, (pos, spd) in goals.items():
            pos = int(max(0, min(4095, pos)))
            spd = int(max(0, min(4000, spd)))
            params += [sid, acc & 0xFF, pos & 0xFF, pos >> 8, 0, 0, spd & 0xFF, spd >> 8]
        self._txrx(make_packet(BROADCAST, INST_SYNC_WRITE, params), BROADCAST, None)

    def set_id(self, old_id, new_id):
        """EEPROM 잠금 풀고 ID 변경 후 다시 잠금"""
        self.write(old_id, ADDR_LOCK, (0,))
        self.write(old_id, ADDR_ID, (new_id,))
        time.sleep(0.05)
        self.write(new_id, ADDR_LOCK, (1,))
