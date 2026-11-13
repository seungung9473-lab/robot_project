#!/usr/bin/env python3
"""
motor_test.py — ROS 없이 서보 버스 확인 + 한 관절 살짝 움직여 보기

  python3 motor_test.py                 # ID 1~5 응답 / 위치 / 전압 / 온도만 읽음 (안 움직임)
  python3 motor_test.py --wiggle 1      # J1 을 지금 위치에서 +-100 count(약 9도) 천천히 왕복 후 제자리
  python3 motor_test.py --off           # 전체 토크 끄기 (팔이 떨어지니 손으로 받치고!)

처음엔 J1(좌우 회전)으로만 시험할 것. 12V 스위치에 손 올려두기.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from soarm_bridge.sts_bus import STSBus  # noqa: E402

LIMITS = {1: (900, 3432), 2: (1889, 4055), 3: (40, 2037), 4: (700, 3233), 5: (0, 4095)}  # 여유 포함

ap = argparse.ArgumentParser()
ap.add_argument('--port', default='/dev/servo_bus')
ap.add_argument('--wiggle', type=int, metavar='ID')
ap.add_argument('--off', action='store_true')
a = ap.parse_args()

bus = STSBus(a.port)
print(f'포트 {a.port} 열림')
for sid in range(1, 6):
    st = bus.read_state(sid)
    print(f'  ID {sid}: ' + ('응답 없음' if st is None else f'위치 {st[0]:4d}  전압 {st[2]:.1f} V  온도 {st[3]} C'))

if a.off:
    for sid in range(1, 6):
        bus.torque(sid, False)
    print('토크 OFF')

if a.wiggle:
    sid = a.wiggle
    p0 = bus.read_pos(sid)
    if p0 is None:
        sys.exit(f'ID {sid} 응답 없음')
    lo, hi = LIMITS[sid]
    bus.sync_goal({sid: (p0, 300)})          # 목표 = 현재 위치 먼저 (토크 켤 때 튀지 않게)
    bus.torque(sid, True)
    for tgt in (p0 + 100, p0 - 100, p0):
        tgt = max(lo, min(hi, tgt))
        bus.sync_goal({sid: (tgt, 300)}, acc=20)
        time.sleep(1.5)
        print(f'  목표 {tgt} -> 현재 {bus.read_pos(sid)}')
    print('끝 (토크는 켜진 상태로 둠. 끄려면 --off, 팔 받치고)')
bus.close()
