# 실제 로봇 실시간 표시

`/dev/servo_bus`에서 모터 ID1~4의 현재 엔코더·속도를 읽고 `/soarm4_live/joint_states`를 20Hz 목표로 발행합니다. 현재 soarm4_description의 URDF와 servo_map.yaml을 사용합니다. 별도 슬라이더/가상 고정 자세는 발행하지 않습니다.

```bash
bash "/home/seungung9473/문서/ChatGPT/라파 대회/soarm4_live/run_live.sh"
```

화면의 `LIVE - measured encoders`와 카운트 값이 현재 통신 상태입니다. 통신이 끊기면 새 관절 상태 발행을 중단하고 `LIVE FEEDBACK LOST`를 표시합니다. 이때 남은 로봇 그림은 마지막 수신 자세입니다.

이 노드는 READ 명령만 송신합니다. 토크·목표 위치·ID·영점·모드를 변경하지 않습니다. 따라서 켜기만 해서는 모터가 움직이지 않으며, 기존 토크 상태도 유지됩니다. 토크가 풀린 실물을 안전하게 지지해 자세를 바꾸면 화면이 그 측정값을 따라갑니다. 실물과 모델의 회전 방향/영점이 일치하는지는 실제 자세와 화면을 대조해 확인해야 합니다.

포트는 이 노드 하나가 사용합니다. 친구의 motor_test.py, servo_bridge, ID 변경 프로그램을 동시에 실행하면 통신이 충돌하므로 함께 실행하지 마세요. 현재 버전은 표시 전용이며, 모터에 이동 명령을 보내면서 실시간으로 표시하려면 동일한 단일 제어 노드에서 명령과 읽기를 함께 처리하도록 통합해야 합니다. 별도 프로그램 둘이 포트를 나눠 쓰는 방식은 사용하지 않습니다.

실측 각도가 계획 리미트 밖이면 `Outside planning limits`를 표시하지만 측정값 자체를 clamp하지 않습니다. 예: J2 count1886은 현재 변환에서194.15도로, 계획 상한184.75도 밖입니다. 읽기 전용 표시는 이 자세도 표시하지만 MoveIt 시작 상태로는 사용할 수 없습니다. 이 작업은 리미트/엔코더 변환을 변경하지 않습니다.

실시간 화면은 `/soarm4_live` namespace와 TF를 사용하며 MoveIt 미리보기(`/soarm4_plan`)와 분리되어 있습니다. 이 화면에 계획 경로 애니메이션은 나타나지 않습니다.

빌드:
```bash
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install --packages-select soarm4_live
source install/setup.bash
```
