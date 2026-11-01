"""Checks shared by stationary capture and stop-and-shoot scans."""
import math
from .controller import target_count

def preflight_angles(state,angles):
    if not state.get('zero_valid'):raise ValueError('먼저 turntable zero로 소프트웨어 영점을 저장하세요')
    if state.get('read_only',True):raise ValueError('회전 스캔은 read_only:=false 모드가 필요합니다')
    if not state.get('torque'):raise ValueError('회전 스캔 전에 턴테이블 토크 ON이 필요합니다')
    if state.get('operating_mode')!=3 or state.get('drive_mode',0)&4:raise ValueError('위치제어 모드3/속도 기반 프로필이 필요합니다')
    zero=state['zero_count']
    return [target_count(a,zero,state['min_position'],state['max_position']) for a in angles]

def stable_states(states,now,seconds=.5):
    recent=[(t,s) for t,s in states if now-t<=seconds+.25]
    if len(recent)<3 or now-recent[-1][0]>.35 or recent[-1][0]-recent[0][0]<seconds:return False
    if any(s.get('hardware_error',1) or s.get('moving',True) or abs(s.get('velocity_raw',999))>1 for _,s in recent):return False
    positions=[s['position_count'] for _,s in recent]
    return max(positions)-min(positions)<=2

def select_bundle(buffers,after_ns,now,max_skew_ns=100_000_000):
    # Choose a near-time bundle, not an arbitrary latest frame from each camera.
    candidates={k:[(received,msg) for received,msg in items if now-received<=.5 and msg.header.stamp.sec*10**9+msg.header.stamp.nanosec>=after_ns] for k,items in buffers.items()}
    if any(not values for values in candidates.values()):return None
    def stamp(item):return item[1].header.stamp.sec*10**9+item[1].header.stamp.nanosec
    for anchor in reversed(candidates['cam1_color']):
        bundle={key:min(values,key=lambda x:abs(stamp(x)-stamp(anchor))) for key,values in candidates.items()}
        stamps=[stamp(x) for x in bundle.values()]
        if max(stamps)-min(stamps)<=max_skew_ns:return bundle
    return None
