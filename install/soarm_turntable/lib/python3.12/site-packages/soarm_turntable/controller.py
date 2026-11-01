"""XL330-M288 controller. Construction and observation perform reads only."""
import math,threading,time

def signed(value,bits):return value-(1<<bits) if value&(1<<(bits-1)) else value

def target_count(degrees,zero,min_count,max_count):
    if isinstance(degrees,bool) or not isinstance(degrees,(float,int)) or not math.isfinite(degrees):raise ValueError('각도는 유한한 숫자여야 합니다')
    value=round(zero+degrees*4096/360)
    if not min_count<=value<=max_count:raise ValueError(f'단회전 범위 밖 목표: {value} count (허용 {min_count}~{max_count}); 360도 래핑하지 않습니다')
    return value

class Controller:
    def __init__(self,bus,read_only=True,profile_velocity=50,profile_acceleration=10):
        self.bus=bus;self.read_only=read_only;self.profile_velocity=profile_velocity;self.profile_acceleration=profile_acceleration;self.lock=threading.RLock();self.info=self.read_info()
        if self.info['model']!=1200:raise RuntimeError(f"XL330-M288 모델 번호 1200이 필요합니다: {self.info['model']}")
        if not 1<=profile_velocity<=32767 or not 1<=profile_acceleration<=32767:raise ValueError('프로필 0은 무제한이므로 허용하지 않습니다')
    def read_info(self):
        with self.lock:
            return {name:self.bus.read(addr,size) for name,addr,size in [('model',0,2),('firmware',6,1),('drive_mode',10,1),('operating_mode',11,1),('velocity_limit',44,4),('max_position',48,4),('min_position',52,4)]}
    def snapshot(self):
        with self.lock:
            data=self.bus.read_block(64,83)
            def get(addr,size):return int.from_bytes(data[addr-64:addr-64+size],'little')
            return {'stamp_monotonic':time.monotonic(),'torque':get(64,1),'hardware_error':get(70,1),'profile_acceleration':get(108,4),'profile_velocity':get(112,4),'goal_count':signed(get(116,4),32),'moving':bool(get(122,1)),'moving_status':get(123,1),'velocity_raw':signed(get(128,4),32),'position_count':signed(get(132,4),32),'voltage':get(144,2)/10,'temperature':get(146,1)}
    def check(self,state):
        if self.read_only:raise RuntimeError('읽기 전용 모드입니다')
        info=self.read_info()
        if info['operating_mode']!=3:raise RuntimeError('현재 버전은 위치제어 모드 3만 지원합니다. 모드는 자동 변경하지 않습니다')
        if info['drive_mode']&4:raise RuntimeError('시간 기반 프로필입니다. 속도 기반 프로필만 지원합니다')
        if state['hardware_error']:raise RuntimeError(f"모터 하드웨어 오류 {state['hardware_error']}")
        if not 3.7<=state['voltage']<=6.0:raise RuntimeError(f"XL330 전압 확인 필요: {state['voltage']}V")
        if self.profile_velocity>info['velocity_limit']:raise RuntimeError('프로필 속도가 모터 속도 제한보다 큽니다')
        return info
    def goto(self,degrees,zero):
        with self.lock:
            state=self.snapshot();info=self.check(state)
            if not state['torque']:raise RuntimeError('토크 OFF입니다. 턴테이블 torque on을 명시적으로 실행하세요')
            count=target_count(degrees,zero,info['min_position'],info['max_position'])
            self.bus.write(108,4,self.profile_acceleration);self.bus.write(112,4,self.profile_velocity);self.bus.write(116,4,count)
            return count
    def torque(self,on):
        with self.lock:
            if self.read_only:raise RuntimeError('읽기 전용 모드입니다')
            if not on:
                self.bus.write(64,1,0);return
            state=self.snapshot();info=self.check(state)
            if bool(state['torque'])==on:return
            if on:
                if state['moving']:raise RuntimeError('움직이는 중에는 토크를 켤 수 없습니다')
                if info['drive_mode']&8:raise RuntimeError('Goal Update 자동 토크 ON 설정입니다. 초기 목표 전송 전 수동 설정 확인이 필요합니다')
                if not info['min_position']<=state['position_count']<=info['max_position']:raise RuntimeError('현재 엔코더가 단회전 범위 밖입니다')
                self.bus.write(116,4,state['position_count'])
            self.bus.write(64,1,int(on))
            if bool(self.bus.read(64,1))!=on:raise RuntimeError('토크 쓰기 확인 실패')
    def stop(self):
        # Position hold, never torque OFF. Cannot guarantee stopping on lost USB.
        with self.lock:
            state=self.snapshot();self.check(state)
            if state['torque']:self.bus.write(116,4,state['position_count'])

class SDKBus:
    def __init__(self,port,baud=57600,motor_id=1):
        import subprocess
        from dynamixel_sdk import PortHandler,PacketHandler
        if subprocess.run(['fuser',port],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0:raise RuntimeError('OpenRB 포트를 다른 프로그램이 사용 중입니다. Wizard/중복 브리지를 종료하세요')
        self.port=PortHandler(port);self.packet=PacketHandler(2.0);self.motor_id=motor_id
        try:
            if not self.port.openPort() or not self.port.setBaudRate(baud):raise RuntimeError('OpenRB 포트 열기 실패')
            self.port.ser.exclusive=True
            time.sleep(2.0)  # OpenRB may reset when the USB CDC port opens.
        except Exception:
            if self.port.ser is not None:self.port.closePort()
            raise
    def read_block(self,addr,size):
        data,comm,error=self.packet.readTxRx(self.port,self.motor_id,addr,size)
        if comm!=0 or error or len(data)!=size:raise RuntimeError(f'DYNAMIXEL 읽기 실패 addr={addr}, comm={comm}, error={error}')
        return bytes(data)
    def read(self,addr,size):return int.from_bytes(self.read_block(addr,size),'little')
    def write(self,addr,size,value):
        fn=getattr(self.packet,f'write{size}ByteTxRx');comm,error=fn(self.port,self.motor_id,addr,value&((1<<(size*8))-1))
        if comm!=0 or error:raise RuntimeError(f'DYNAMIXEL 쓰기 실패 addr={addr}, comm={comm}, error={error}')
        if self.read(addr,size)!=value&((1<<(size*8))-1):raise RuntimeError(f'쓰기 검증 실패 addr={addr}')
    def close(self):self.port.closePort()
