#!/usr/bin/env python3
"""가짜 서보 버스 + 가짜 ESP32 를 pty 로 만들어 STSBus / Vacuum 동작 검증 (하드웨어 없이)"""
import os, pty, sys, threading, time, tty
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from soarm_bridge.sts_bus import STSBus, make_packet, _chk
from soarm_bridge.vacuum import Vacuum

# ---------- 가짜 STS 서보 4개
class FakeServos:
    def __init__(self):
        self.reg = {i: bytearray(70) for i in (1, 2, 3, 4)}
        for i, p in zip((1, 2, 3, 4), (2066, 1886, 2042, 2258)):
            self.set16(i, 56, p); self.set16(i, 42, p); self.reg[i][62] = 121; self.reg[i][63] = 33; self.reg[i][5] = i
        self.m, s = pty.openpty(); tty.setraw(s); self.path = os.ttyname(s)
        threading.Thread(target=self.run, daemon=True).start()
    def set16(self, i, a, v): self.reg[i][a] = v & 0xFF; self.reg[i][a + 1] = v >> 8
    def get16(self, i, a): return self.reg[i][a] | (self.reg[i][a + 1] << 8)
    def reply(self, sid, data=b''):
        body = [sid, len(data) + 2, 0, *data]; os.write(self.m, bytes([0xFF, 0xFF, *body, _chk(body)]))
    def run(self):
        buf = b''
        while True:
            buf += os.read(self.m, 256)
            while len(buf) >= 6:
                k = buf.find(b'\xff\xff')
                if k < 0: buf = b''; break
                buf = buf[k:]
                if len(buf) < 4 or len(buf) < 4 + buf[3]: break
                sid, ln, ins = buf[2], buf[3], buf[4]; prm = buf[5:3 + ln]; buf = buf[4 + ln:]
                for i in self.reg:   # 목표 위치로 즉시 이동한 걸로 침
                    if self.reg[i][40] == 1: self.set16(i, 56, self.get16(i, 42))
                if ins == 0x01 and sid in self.reg: self.reply(sid)
                elif ins == 0x02 and sid in self.reg: self.reply(sid, bytes(self.reg[sid][prm[0]:prm[0] + prm[1]]))
                elif ins == 0x03 and sid in self.reg:
                    self.reg[sid][prm[0]:prm[0] + len(prm) - 1] = prm[1:]; self.reply(sid)
                elif ins == 0x83:
                    a, L = prm[0], prm[1]; d = prm[2:]
                    for j in range(0, len(d), L + 1):
                        i = d[j]
                        if i in self.reg: self.reg[i][a:a + L] = d[j + 1:j + 1 + L]

# ---------- 가짜 ESP32
class FakeESP:
    def __init__(self):
        self.pump = self.valve = 0; self.base = 2000; self.stream = True; self.load = False
        self.m, s = pty.openpty(); tty.setraw(s); self.path = os.ttyname(s)
        threading.Thread(target=self.rx, daemon=True).start(); threading.Thread(target=self.tx, daemon=True).start()
    def raw(self):
        if self.pump and self.load and not self.valve: return self.base - 900
        return self.base + (-15 if self.pump else 0)
    def rx(self):
        while True:
            for ch in os.read(self.m, 64).decode():
                if ch == 'p': self.pump = 1
                if ch == 'o': self.pump = 0
                if ch == 'v': self.valve = 1
                if ch == 'c': self.valve = 0
                if ch == 's': self.stream = not self.stream
                if ch == 'z': self.zero = self.raw()
    def tx(self):
        self.zero = self.base
        while True:
            time.sleep(0.2)
            if self.stream:
                r = self.raw()
                os.write(self.m, f'raw:{r} diff:{r - self.zero} pump:{self.pump*1000} valve:{self.valve*500}\n'.encode())

fs = FakeServos()
bus = STSBus(fs.path)
assert all(bus.ping(i) for i in (1, 2, 3, 4)) and not bus.ping(9), 'ping'
st = bus.read_state(2); assert st[0] == 1886 and abs(st[2] - 12.1) < 1e-6 and st[3] == 33, st
for i in (1, 2, 3, 4): bus.torque(i, True)
assert fs.reg[1][40] == 1
bus.sync_goal({1: (2500, 300), 2: (3000, 300), 3: (1000, 300), 4: (2047, 300)}, acc=30)
time.sleep(0.05)
assert [bus.read_pos(i) for i in (1, 2, 3, 4)] == [2500, 3000, 1000, 2047], [bus.read_pos(i) for i in (1,2,3,4)]
assert fs.reg[1][41] == 30 and fs.get16(1, 46) == 300
bus.set_id(4, 7); assert fs.reg[4][5] == 7
print('STSBus OK  (ping / read_state / torque / sync_goal / set_id)')

fe = FakeESP(); fe.stream = False      # 출력 꺼진 상태에서 시작해도 켜는지
vac = Vacuum(fe.path, hold_diff=-400)
assert vac.online and abs(vac.diff) < 30, vac.diff
ok, d = vac.grip(1.0); assert not ok, ('허공인데 잡힘?', d)
fe.load = True
ok, gd = vac.grip(2.0); assert ok and gd <= -400, gd
ok, d = vac.release(); time.sleep(0.1); assert ok and fe.valve == 0 and fe.pump == 0, (ok, d)
vac.close()
print(f'Vacuum OK  (출력 자동 켜기 / 허공 실패 판정 / 잡기 diff={gd} / 놓기)')
