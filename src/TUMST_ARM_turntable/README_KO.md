# XL330 + OpenRB 턴테이블

설정: XL330-M288-T(모델 번호 1200), ID1,57600bps, Protocol2.0, 직결,5V, profile_velocity50/profile_acceleration10. 전용 경로 /dev/turntable_bus. 기존 로봇·ESP32·카메라 프로세스와 분리합니다. 기동/읽기/종료 시 모터 쓰기 없음. 프로필은 명시적 goto에서만 RAM에 씁니다. ID/모드/방향/펌웨어/EEPROM은 바꾸지 않습니다.

## 현재 확인 상태
실제 XL330-M288(모델1200), 펌웨어53, 모드3, 속도 기반 프로필, 프로필50/10, 전압5.1V, 오류0을 읽어 확인했습니다. 실제 회전은 사용자가 Wizard에서 확인했습니다. 코드에 의한 실제 자동 회전은 아직 시험하지 않았습니다. 실제 두 카메라와 엔코더의 정지 촬영을 검증했으며 자세한 사용법은 SCAN_KO.md입니다.

## 읽기 전용
```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=73
ros2 launch soarm_turntable turntable.launch.py
```
다른 터미널에서 같은 환경을 불러온 뒤:
```bash
ros2 run soarm_turntable turntable read
ros2 topic echo /turntable/state
```
Wizard는 연결을 끊고 실행합니다. 동일 USB 포트에 두 프로그램을 동시에 연결하지 않습니다.

## 명시적 제어 (통신 확인 및 주변 정리 후)
읽기 전용 실행을 종료하고 `read_only:=false`로 실행합니다. 시작만으로 움직이지 않습니다.
```bash
ros2 launch soarm_turntable turntable.launch.py read_only:=false
ros2 run soarm_turntable turntable zero
ros2 run soarm_turntable turntable torque on
ros2 run soarm_turntable turntable goto 5 --timeout 15
ros2 run soarm_turntable turntable stop
```
각 명령은 별도 터미널에서 순서대로 실행합니다. 현재 위치를 소프트웨어 영점으로 저장하므로 하드웨어 영점은 바뀌지 않습니다. `goto` 각도 단위는 도이며 영점에 대한 상대 좌표입니다(매번 현재 위치에 더하는 상대 이동 아님). 정지 후 도착 오차6count(약0.53도), Moving=false,속도값<=1을 0.5초 확인하고 성공합니다. 정지는 현재 위치를 목표로 보내 토크를 유지합니다. USB 통신이 끊기면 소프트웨어 STOP으로 물리 정지를 보장하지 못합니다. 종료 전에 STOP하고 정지 여부를 확인합니다.

현재 버전은 위치제어 모드3, 속도 기반 프로필만 허용합니다. 0~4095 및 모터의 실제 min/max 범위 밖은 거부하며 360도를 0도로 래핑하지 않습니다. 처음부터 전체 360도 스캔을 명령하지 않습니다. 토크 ON 전 현재 위치로 목표를 맞춥니다. Goal Update 자동 토크 ON 설정(Drive Mode bit3)이 켜진 경우는 검토 없이 켜지 않도록 거부합니다.

소프트웨어 영점은 ~/.ros/soarm4/turntable_zero.json에 저장합니다. 모터 Homing Offset이나 회전 방향·기구 체결이 바뀌면 영점을 다시 설정해야 합니다. /turntable/joint_states는 별도 토픽이며 기존 /joint_states 및 로봇팔 모델에 끼워 넣지 않습니다. 실제 회전축 TF를 임의로 만들지 않습니다.

## 촬영/흡착 다음 단계
1. 카메라 두 대 USB3 연결 복구 및 프레임 안정성 확인.
2. 정지-촬영 방식으로 회전축·카메라·로봇 외부 좌표 보정.
3. 각도,카메라 이미지/깊이/내부행렬,시간을 함께 기록하고 테이블/배경을 제거.
4. 회전각으로 역변환한 물체 점군만 정합해 합성.
5. 지름25mm 컵의 여유 면적·평면도·표면 정상벡터·유효 깊이 및 로봇4축 접근/충돌 가능성 검사.
6. 선택된 면을 로봇이 접근할 수 있는 각도로 돌리고 정지한 뒤 집기.

이 패키지는 1단계 턴테이블 제어입니다. 아직 다각도 점군 합성·흡착면 검출·자동 집기·VLA 통합은 구현하지 않았습니다.
