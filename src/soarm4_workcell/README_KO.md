# SO-ARM4 실측 작업 환경

전장상자 아래 왼쪽 모서리 (1.300, 0.150, 0)m, 상자 17cm(X)×20cm(Y), 로봇이 -X 방향을 향하는 배치를 사용합니다. URDF 파일·관절·홈·TCP 수치는 수정하지 않습니다. `table → world` 고정 변환으로 기존 모델 전체를 배치합니다.

## 모터 없이 미리보기

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=99
ros2 launch soarm4_workcell preview.launch.py
```

별도 ROS 도메인 99의 `/soarm4_plan`에서 홈 상태를 표시합니다. 드라이버·ESP32·펌프·턴테이블·실행 액션을 시작하지 않습니다. RViz MotionPlanning에서 계획은 가능하고 실행은 비활성화됩니다. 카메라 프레임은 측정된 보정 프레임으로 표시하며 카메라의 실제 TF를 변경하지 않습니다.

## 기존 실물 MoveIt에 환경 추가

기존 `hardware_bringup.launch.py`가 실행되는 도메인에서 아래 노드만 하나 추가합니다. 이 명령 자체는 드라이버를 시작하거나 실행을 허용하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=73
ros2 launch soarm4_workcell scene.launch.py
```

기존 MoveIt의 계획 프레임은 `world`, 표시할 전체 배치의 기준은 `table`입니다. 센서가 TF를 발행하면 내부 optical↔link 변환을 읽어 `table→cam1_link`, `table→cam2_link`를 연결합니다. optical 프레임에 두 부모를 붙이거나 센서 내부 변환을 가정하지 않습니다. 필요한 TF가 없으면 해당 카메라 연결을 기다립니다. 동일 도메인에 미리보기의 가짜 joint_states와 실물 joint_states를 함께 실행하지 마세요.

## 등록되는 충돌 물체 4개

- workcell_table_guard: 180×75cm 상판 아래 20cm 금지 영역. 실제 상판 두께를 뜻하지 않습니다. 모델 바닥 지지 접촉 때문에 위쪽 경계를 상판보다 1mm 아래에 둡니다.
- workcell_turntable: 지름 15cm, 높이 5.9cm의 보수적 원통 외곽.
- workcell_pad1: ID6, 지름 10cm, 높이 5.9cm의 원통.
- workcell_pad2: ID5, 지름 10cm, 높이 5.5cm의 원통.

원통은 원판 아래 영역을 보수적으로 막습니다. 턴테이블 레일·모터 하우징·카메라 거치대·케이블·물체·잡은 물체는 치수/형상이 확인되지 않아 아직 등록하지 않았습니다. 이 환경만으로 실제 전체 경로의 충돌 안전이 검증되지는 않습니다. 카메라 외형은 시각 표시만 있습니다.

설치한 물체를 ApplyPlanningScene로 추가하고 GetPlanningScene로 실제 등록을 확인합니다. 기존 다른 장애물·AllowedCollisionMatrix를 덮어쓰지 않으며, 서비스 재시작 뒤 물체가 사라지면 다시 추가합니다.

## 현재 기준

- J1 중심: (1.3398, 0.2500, 0.1598)m.
- 턴테이블 축: (1.095, 0.235, 0.059)m를 지나 +Z 방향.
- pad1 중심: (1.097806, 0.384419, 0.059)m.
- pad2 중심: (1.098338, 0.083561, 0.055)m.
- 카메라: 2026-11-01 네 마커 보정 스냅샷.

배치 확인은 TCP/흡착점 실물 검증을 대체하지 않습니다. 카메라·패드 절대 정확도 미검증 상태는 config에 유지합니다. 로봇 모델이 바뀌면 체크섬 검증이 시작을 거부하므로 등록을 재검토해야 합니다.

## 실제 모터 제어 화면 (자동 이동 없음)

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=73
ros2 launch soarm4_workcell live.launch.py allow_execution:=true
```

기존 scene.launch.py 노드가 같은 도메인에 있어야 작업 환경이 적용됩니다. 12V 모터 전원과 피드백이 필요합니다. RViz Joints에서 목표를 조정하고 Plan 후 Execute합니다. 이 launch는 시작 시 홈 이동/토크 ON/목표 전송을 하지 않습니다. ESP32가 없는 상태에서도 팔 UI를 열 수 있도록 진공 브리지는 기본 OFF입니다. 펌프를 쓸 때는 연결 확인 후 `start_vacuum_bridge:=true`를 명시합니다. 이 명령과 기존 hardware_bringup을 동시에 실행하지 않습니다.
