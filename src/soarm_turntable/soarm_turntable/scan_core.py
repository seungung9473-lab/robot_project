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

def stability_problem(states,now,seconds=.5):
    """None면 정지 안정. 아니면 ('sample'|'motion', 이유). sample=표본 부족/지연(움직임 증거 아님), motion=실제 움직임·오류 증거."""
    # 창 여유 .5s: 브리지가 10Hz로 읽는데 통신이 조금만 밀려도 .25s 여유로는 표본 간격이 seconds에 못 미침
    recent=[(t,s) for t,s in states if now-t<=seconds+.5]
    if len(recent)<3:return 'sample','상태 표본 부족 %d개'%len(recent)
    if now-recent[-1][0]>.35:return 'sample','상태 수신 지연 %.2fs'%(now-recent[-1][0])
    if recent[-1][0]-recent[0][0]<seconds:return 'sample','표본 구간 %.2fs < %.2fs'%(recent[-1][0]-recent[0][0],seconds)
    if any(s.get('hardware_error',1) for _,s in recent):return 'motion','하드웨어 오류 플래그'
    if any(s.get('moving',True) for _,s in recent):return 'motion','moving 플래그 ON'
    v=max(abs(s.get('velocity_raw',999)) for _,s in recent)
    if v>1:return 'motion','속도 raw %d (한계 1)'%v
    positions=[s['position_count'] for _,s in recent]
    if max(positions)-min(positions)>2:return 'motion','위치 변동 %d count (한계 2)'%(max(positions)-min(positions))
    return None

def stable_states(states,now,seconds=.5):
    return stability_problem(states,now,seconds) is None

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
