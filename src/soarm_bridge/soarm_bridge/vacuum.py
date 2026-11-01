"""
vacuum.py — ESP32 진공 보드 클라이언트 (ROS 없이도 동작)

ESP32 펌웨어 (esp32_servo_bridge_1 / _2) 명령: 글자 하나
  p / o  펌프 ON / OFF        v / c  밸브 ON(대기 개방 = 놓기) / OFF
  z      지금 압력을 0 으로     s      출력 ON/OFF 토글
200 ms 마다:  raw:#### diff:#### pump:#### valve:###
diff : 허공 ≈ 0,  물건 잡으면 크게 음수 (예: -900)
"""
import re
import threading
import time

import serial

LINE = re.compile(r'raw:\s*(-?\d+)\s+diff:\s*(-?\d+)')


class Vacuum:
    def __init__(self, port='/dev/esp32', baud=115200, hold_diff=-300, release_diff=-50, grip_diff=-400):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.hold_diff = hold_diff
        self.grip_diff = grip_diff
        self.release_diff = release_diff
        self.diff = None
        self.raw = None
        self.stamp = 0.0
        self.extra_lines = []           # TT ... 같은 다른 줄 (턴테이블 펌웨어 등)
        self._stop = False
        self._wlock = threading.Lock()
        self._th = threading.Thread(target=self._reader, daemon=True)
        self._th.start()
        time.sleep(2.0)                 # 포트 열 때 보드가 리셋될 수 있음
        self.send('o'); self.send('c')
        if not self.wait_fresh(1.0):    # 출력이 꺼져 있으면 켜기
            self.send('s')
        time.sleep(0.5)
        self.send('z')                  # 펌프 끈 상태에서 영점

    def _reader(self):
        buf = b''
        while not self._stop:
            try:
                buf += self.ser.read(256)
            except serial.SerialException:
                time.sleep(0.5)
                continue
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                s = line.decode(errors='ignore').strip()
                m = LINE.search(s)
                if m:
                    self.raw, self.diff = int(m.group(1)), int(m.group(2))
                    self.stamp = time.monotonic()
                elif s:
                    self.extra_lines = (self.extra_lines + [s])[-20:]

    def send(self, ch):
        with self._wlock:
            if self.ser.write(ch.encode()) != len(ch.encode()):raise IOError('ESP32 command write incomplete')

    def wait_fresh(self, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.stamp > t0:
                return True
            time.sleep(0.02)
        return False

    @property
    def online(self):
        return time.monotonic() - self.stamp < 1.0

    @property
    def holding(self):
        return self.diff is not None and self.online and self.diff <= self.hold_diff

    def grip(self, timeout=2.0):
        """밸브 닫고 펌프 ON → diff 가 grip_diff 이하로 떨어지면 성공"""
        self.send('c'); self.send('p')
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.diff is not None and self.online and self.diff <= self.grip_diff:
                return True, self.diff
            time.sleep(0.02)
        return False, self.diff

    def release(self, timeout=3.0):
        """펌프 OFF + 밸브 열기 → diff 가 release_diff 위로 올라오면 밸브 닫기"""
        self.send('o'); self.send('v')
        t0 = time.monotonic()
        ok = False
        while time.monotonic() - t0 < timeout:
            if self.diff is not None and self.diff > self.release_diff:
                ok = True
                break
            time.sleep(0.02)
        time.sleep(0.2)
        self.send('c')
        return ok, self.diff

    def grip_timed(self, seconds=1.0):
        """Open-loop command; success means written, never verified suction."""
        self.send('c'); self.send('p'); time.sleep(seconds)
        return True, '센서 미사용 · 흡착 명령 전송/대기 완료 (흡착 확인 없음)'

    def release_timed(self, seconds=1.0):
        self.send('o'); self.send('v')
        try:time.sleep(seconds)
        finally:self.send('c')
        return True, '센서 미사용 · 펌프 OFF/밸브 개방 후 닫힘 (압력 확인 없음)'

    def zero(self):
        self.send('z')

    def off(self):
        self.send('o'); self.send('c')

    def close(self):
        try:
            self.off()
        finally:
            self._stop = True
            time.sleep(0.15)
            self.ser.close()
