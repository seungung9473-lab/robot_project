#!/usr/bin/env python3
"""
새 STS 서보 ID 바꾸기 (예: 턴테이블 STS3215 → ID 5)
  ※ 버스에 바꿀 서보 '하나만' 연결하고 실행 (새 서보는 ID 1 이라 J1 이랑 겹침)
  python3 sts_set_id.py            → 연결된 서보 찾아서 보여주기만
  python3 sts_set_id.py 1 5        → ID 1 을 5 로 변경
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from soarm_bridge.sts_bus import STSBus

PORT = os.environ.get('SERVO_PORT', '/dev/servo_bus')
bus = STSBus(PORT)
found = [i for i in range(0, 20) if bus.ping(i)]
print('찾은 서보 ID:', found)
for i in found:
    st = bus.read_state(i)
    if st:
        print(f'  ID {i}: 위치 {st[0]}  전압 {st[2]:.1f}V  온도 {st[3]}°C')
if len(sys.argv) == 3:
    old, new = int(sys.argv[1]), int(sys.argv[2])
    if len(found) != 1:
        sys.exit('서보를 하나만 연결하고 다시 실행해 (지금 %d개)' % len(found))
    if old not in found:
        sys.exit(f'ID {old} 없음')
    print(f'변경 전 기록: ID={old}')
    bus.set_id(old, new)
    print('변경 후 확인:', 'OK' if bus.ping(new) else '실패', f'(ID {new})')
bus.close()
