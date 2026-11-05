# 현재 robot_ws에 맞춘 하드웨어 브리지

기존 soarm4_description과 MoveIt 패키지는 덮어쓰지 않습니다. 현재 URDF의 범위 안에 드는 정수 엔코더 값만 선택했습니다.

| 관절 | 최소 count | 최대 count |
|---|---:|---:|
| J1 | 974 | 3357 |
| J2 | 1994 | 4038 |
| J3 | 115 | 2047 |
| J4 | 774 | 3159 |

URDF의 소수점 각도와 count가 정확히 일치하지 않는 끝에서는 바깥으로 넘지 않도록 1count 안쪽으로 반올림했습니다. URDF는 변경하지 않았습니다. margin_lo/hi는0으로 설정했습니다. 이미 URDF 범위에 여유가 들어가 있으므로 다시 차감하지 않습니다.

처음 실행:
```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=73
ros2 launch soarm_bridge bringup.launch.py read_only:=true rviz:=true
```

이 실행에서는 서보 토크·궤적·턴테이블 명령을 거부합니다. 실제 모터 위치를 /joint_states로 읽고 로봇 상태 발행기와 RViz에 반영합니다. MoveIt 가상 자세와 혼합하지 않습니다. 진공 ESP32 브리지도 함께 시작하며 기존 코드 특성상 펌프 OFF/밸브 닫힘 및 압력 영점을 초기화합니다. 기동만으로 펌프를 켜지 않습니다. 진공 서비스는 기존대로 존재하므로 읽기 전용은 서보 명령에 대한 설정입니다.

다른 터미널 확인:
```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=73
ros2 topic echo /joint_states --once
ros2 topic echo /vacuum/diff --once
```

motor_test.py 및 별도의 soarm4_live는 함께 실행하지 않습니다. 이 브리지가 모터 포트를 단독으로 사용하면서 상태를 발행합니다. 하드웨어 제어를 켜기 전에는 현재 실제 자세의 계획 리미트 포함 여부, 모터 방향, 궤적 취소/정지 처리 등을 별도로 검증해야 합니다. 이 작업은 MoveIt Execute를 연결하거나 실행 모드의 전체 동작을 승인한 작업이 아닙니다.

필수 의존성 중 ros-jazzy-control-msgs가 없으면 빌드 후에도 실행 시 ModuleNotFoundError가 납니다.


속도 상한 업데이트: 사용자 요청으로 max_speed를 150에서 200 count/s로 조정했습니다. ACC 10, URDF 관절 리미트, 추종/최종 위치 오차 검사는 유지합니다. 이 변경 자체는 실제 이동 속도를 지정하거나 최종 위치 도달 실패 해결을 보증하지 않습니다.

추가 속도 상한 업데이트: 사용자 요청으로 200 → 250 count/s. ACC 10과 기존 위치·추종 오차 검사는 유지합니다. 실제 이동 시험은 수행하지 않았습니다.
