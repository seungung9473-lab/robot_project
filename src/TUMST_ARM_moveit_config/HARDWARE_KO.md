# 실제 엔코더를 사용하는 MoveIt 실행 연결

이전 planning.launch.py는 계속 가상 상태 미리보기용입니다. 실제 로봇은 hardware.launch.py를 사용합니다. 이 launch에는 가상 관절 발행기나 모터 드라이버가 없고, 별도로 실행한 soarm_bridge의 실제 /joint_states, /tf, /arm_controller/follow_joint_trajectory를 사용합니다. 동일 포트를 여는 motor_test.py를 함께 실행하지 않습니다. 구형 soarm4_live 패키지는 제거했습니다.

## 읽기 전용으로 실제 상태에서 계획만 확인

기존 read_only bringup은 유지하고 다른 터미널에서:
```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=73
ros2 launch soarm4_moveit_config hardware.launch.py
```

실행하지 않고 실제 관절에서 계획하는 모드입니다. 이 화면에는 실제 움직임과 계획 미리보기가 함께 보이며 반투명 궤적은 실측값이 아닙니다.

## 한 번에 실행

기존 bringup/MoveIt를 Ctrl+C로 종료한 뒤 같은 터미널에서:
```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=73
ros2 launch soarm4_moveit_config hardware_bringup.launch.py allow_execution:=true
```

이 명령은 모터 제어기+진공+실측 상태+MoveIt RViz를 묶어 실행합니다. 기동만으로 이동하지 않으며 Execute를 눌러야 실제 이동합니다. allow_execution을 생략하면 읽기 전용으로 기동합니다. 기존 bringup과 동시에 실행하지 않습니다.

## 터미널을 나누어 첫 실물 실행

기존 읽기 전용 bringup과 위 MoveIt 창은 Ctrl+C로 종료합니다. 모터 드라이버는 하나만 실행합니다.

터미널1:
```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=73
ros2 launch soarm_bridge bringup.launch.py read_only:=false
```

터미널2:
```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=73
ros2 launch soarm4_moveit_config hardware.launch.py allow_execution:=true
```

기동만으로 토크를 켜거나 궤적을 실행하지 않습니다. 실제 이동은 Execute / Plan & Execute 명령에서 시작합니다. 첫 시험은 다음과 같이 합니다.

1. 실물과 화면이 홈 근처에서 일치하는지 확인합니다.
2. MotionPlanning → Planning에서 Start State=<current>, Goal State=<current>를 선택합니다.
3. Joints 탭에서 joint1만 현재 값보다 +1도로 변경합니다. 다른 관절 목표는 실측 현재 값을 유지합니다. 고정된 j1_test 이름을 선택하면 다른 관절도 정확한 홈으로 미세 이동할 수 있으므로 첫 J1 단독 시험에서는 이 이름을 사용하지 않습니다.
4. Velocity/Accel. Scaling을 각각0.05로 두고 Plan을 누릅니다.
5. 화면 경로가 의도대로인지 확인한 뒤 Execute를 누릅니다. 반복 재생은 꺼져 있습니다.

관절별 5° 초기 시험 제한을 제거했습니다. 실행 범위는 URDF 관절 리미트를 따릅니다. 충돌·피드백·시작 위치·속도 검사는 유지합니다.

## 드라이버 검증과 정지

- 모든 관절의 유한한 위치, URDF 리미트, 순증가 시간, 신선한 피드백, 시작 위치 오차를 검사합니다. 범위 밖 궤적은 잘라서 실행하지 않고 거부합니다.
- 토크를 켜기 전 현재 위치를 목표 레지스터에 저장하고 네 모터 모두 읽기 복귀값을 확인합니다. 이전 목표로 갑자기 이동하지 않게 합니다.
- 궤적을50Hz로 선형 보간하여 보내며, 속도 상한150count/s(약13.2도/s), 가속도 명령10을 초기 시험 설정으로 사용합니다. 이 설정은 부하별 성능을 보증하는 교정값이 아닙니다.
- 통신 유실/추종 오차/취소/정지 시 새 이동 목표 전송을 중단하고 마지막 측정 위치 유지 명령을 시도합니다. 통신 자체가 끊기면 유지 명령 전달도 보장할 수 없습니다. 필요 시 실물 전원 차단 수단을 사용합니다.
- Stop 버튼은 실행 취소를 전달합니다. 토크를 끄지 않으므로 팔이 풀리는 정지와 다릅니다. 종료 후에도 토크는 자동으로 풀지 않습니다.
- 별도의 중단 요청:
```bash
ros2 service call /servo/stop std_srvs/srv/Trigger '{}'
```

검증: 가짜 모터 ROS 액션에서 정상 실행, 시작 목표 읽기 불일치 시 토크 차단, 범위/시간/관절 이름 오류 거부, 취소, Stop 서비스, 통신 유실 중단을 확인했습니다. 시험용 RViz에서 Plan→Execute→공식 MoveItSimpleControllerManager→FollowJointTrajectory→엔코더 피드백 경로도 확인했습니다. 사용자는 별도로 5° 이하 실제 이동의 성공을 확인했습니다. HOME 버튼의 검증은 가짜 모터로 수행했으며 실제 홈 이동은 아직 시험하지 않았습니다.


## 홈 버튼과 명령 (0.5.0)

기존 실행을 Ctrl+C로 끝내고 hardware_bringup.launch.py를 allow_execution:=true로 다시 실행합니다. 시작만으로 움직이지 않습니다.

RViz 하단 **SO-ARM4 Home → 홈으로 이동 (HOME)** 을 누르거나, 같은 ROS 환경의 터미널에서 실행합니다.

```bash
ros2 run soarm4_moveit_config go_home.py
```

현재 엔코더를 확인하고 현재 자세를 유지하도록 토크를 켠 뒤, MoveIt 충돌 검사를 통과한 경로를 실행합니다. 홈은 네 모터의 엔코더 2047 기준입니다. HOME도 5° 거리 제한 없이 URDF 리미트와 충돌 검사를 따릅니다. 현재 자세나 계획 경로가 리미트 밖이면 거부합니다.

HOME 계획이나 이동은 같은 패널의 **정지 (STOP)** 으로 취소합니다. 종료/취소/계획 실패 후에도 토크는 자동으로 풀지 않습니다. 단, 통신 단절 시 위치 유지 명령의 전달은 보장되지 않습니다.

별도 가짜 모터와 RViz 창에서 HOME 버튼 → 충돌 검사 → 실행 → 엔코더 홈 도착을 확인했습니다(`validation_home_gui_sim.json`). 실제 포트는 열지 않았습니다.


## 토크 해제 버튼

팔을 받친 상태에서 SO-ARM4 Home 패널의 **토크 해제 (팔 받치고 누르기)** 를 누르면 네 관절의 위치 유지 토크를 끕니다. HOME 처리/궤적 실행 중에는 해제를 거부합니다. 먼저 STOP 후 정지가 완료되면 해제하세요. HOME 또는 다음 경로 실행은 토크를 다시 켭니다.

같은 버튼을 다시 누르면 현재 측정 위치로 목표를 먼저 설정하고 토크를 켜 현재 자세를 고정합니다. 토글 방향은 버튼 내부 추정이 아니라 네 모터의 실제 토크 레지스터를 읽어 결정합니다. 일부만 켜졌으면 먼저 모두 해제합니다. 통신 오류 시 전환하지 않습니다.


실행 범위 업데이트: 5° 거리 제한 제거. URDF 경계 및 경계 밖 거부, 5° 초과 정상 경로 허용, 속도 상한 유지 검증은 validation_urdf_execution_limits.json에 기록했습니다. 실제 전체 범위 이동 시험은 수행하지 않았습니다.
