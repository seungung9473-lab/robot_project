# SO-ARM 4DOF 진공 로봇 — 소프트웨어 인계 문서

Ubuntu 24.04 + ROS 2 Jazzy 노트북에서 로봇 팔 · 진공 · 카메라를 쓰기 위한 모든 것.
하드웨어 담당: 송완재 / 작성 2026-10-20

---

## 1. 폴더 구성

| 폴더 | 내용 |
|---|---|
| `soarm_bridge/` | **ROS 2 패키지 (새로 만듦)** — 서보 버스·ESP32 ↔ ROS. MoveIt 이 여기로 명령을 보냄 |
| `soarm4_description/` | ROS 2 패키지 — URDF (팔 모양, 관절 범위, 흡착컵 TCP) |
| `udev/99-soarm.rules` | USB 장치 이름 고정 (`/dev/servo_bus`, `/dev/esp32`) |
| `firmware/esp32_servo_bridge_1/` | ESP32 펌웨어 (지금 보드에 올라가 있는 것) |
| `vision/multiview_fusion.py` | 턴테이블 멀티뷰 → 점구름 합치기 → 석션 후보 점수 |
| `examples/simple_pick.py` | MoveIt 없이 이동·잡기·놓기 해보는 예제 |
| `docs/` | 퍼프보드 배선도, 진공 호스 연결도, 압력센서 연결 |

`teach_play.py` · `poses.json` (티칭 재생 스크립트) 는 라즈베리파이 `~/soarm` 에 있음.

---

## 2. 시스템 구조

```
노트북 (Ubuntu 24.04 / Jazzy)  ── USB ──┬─ URT-2  → 서보 버스 12V 1Mbps → STS 서보 ID 1~4 (+ 턴테이블 ID 5 예정)
  MoveIt / 비전 / VLM                   ├─ ESP32-S3 ("USB" 포트) → 펌프 · 밸브 · 압력센서
                                         └─ D415 x2 (USB 3)
```
- 11/12 전국대회: 위처럼 노트북에 직접 연결
- 12/22 라파 대회: 위 USB 장치를 라즈베리파이에 꽂고, 노트북과는 ROS 2 네트워크(같은 `ROS_DOMAIN_ID=73`)로 통신. 라파에도 같은 패키지를 깔면 됨

---

## 3. 노트북 설치 (처음 한 번)

```bash
# 1) 도구
sudo apt install -y python3-serial ros-jazzy-control-msgs ros-jazzy-robot-state-publisher \
                    ros-jazzy-joint-state-publisher-gui ros-jazzy-realsense2-camera

# 2) USB 장치 이름 고정 (서보 / ESP32 / RealSense)
sudo cp udev/99-soarm.rules /etc/udev/rules.d/
sudo curl -L -o /etc/udev/rules.d/99-realsense-libusb.rules \
  https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# 3) 워크스페이스에 두 패키지 넣고 빌드
mkdir -p ~/ros2_ws/src && cp -r soarm_bridge soarm4_description ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --symlink-install
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=73" >> ~/.bashrc
source ~/.bashrc
```

확인: 장치를 꽂고 `ls -l /dev/servo_bus /dev/esp32` → 두 줄 다 나와야 함.

---

## 4. 실행

```bash
ros2 launch soarm_bridge bringup.launch.py                    # 서보 + 진공 + robot_state_publisher
ros2 launch soarm_bridge bringup.launch.py use_cameras:=true  # + D415 두 대
```

RViz 로 팔 확인: `rviz2` → Fixed Frame `world` → RobotModel 추가 (Description Topic `/robot_description`)

---

## 5. ROS 인터페이스

| 종류 | 이름 | 타입 | 설명 |
|---|---|---|---|
| 토픽 | `/joint_states` | sensor_msgs/JointState | 관절 각도 rad, 30 Hz |
| 토픽 | `/servo/status` | std_msgs/Float32MultiArray | [전압 V, 온도 °C] × 4 |
| 액션 | `/arm_controller/follow_joint_trajectory` | control_msgs/FollowJointTrajectory | **MoveIt 이 궤적 보내는 곳** |
| 서비스 | `/servo/torque` | std_srvs/SetBool | true = 힘 줌 / false = 힘 뺌 (**팔 떨어짐, 받치고**) |
| 토픽 | `/vacuum/diff` | std_msgs/Int32 | 압력 변화. 허공 ≈ 0, 잡으면 약 -900 |
| 토픽 | `/vacuum/holding` | std_msgs/Bool | 잡고 있으면 true |
| 서비스 | `/vacuum/grip` | std_srvs/Trigger | 펌프 ON → 2초 안에 잡히면 success=true |
| 서비스 | `/vacuum/release` | std_srvs/Trigger | 펌프 OFF + 밸브 열어서 놓기 |
| 서비스 | `/vacuum/zero` | std_srvs/Trigger | 압력 영점 (펌프 끈 상태에서) |
| 서비스 | `/vacuum/off` | std_srvs/Trigger | 펌프·밸브 끄기 |
| 토픽 | `/turntable/goto` · `/turntable/angle` | std_msgs/Float64 (deg) | 턴테이블 (STS3215 달면 `turntable_id: 5`) |
| 토픽 | `/camera1/cam1/color/image_raw` 등 | sensor_msgs/Image | 카메라 (cam1 오른쪽, cam2 왼쪽) |

터미널에서 바로 시험:
```bash
ros2 service call /vacuum/grip std_srvs/srv/Trigger
ros2 service call /vacuum/release std_srvs/srv/Trigger
ros2 topic echo /joint_states
```

---

## 6. MoveIt 연결

MoveIt Setup Assistant 로 `soarm4_description/urdf/soarm4.urdf` 를 불러서 설정 만들고,
`moveit_controllers.yaml` 을 아래처럼:

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager
moveit_simple_controller_manager:
  controller_names: [arm_controller]
  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints: [joint1, joint2, joint3, joint4]
```
- 플래닝 그룹: `joint1`~`joint4`, 끝 링크 `cup_tip` (흡착컵 입구 중심, +X 가 컵 방향)
- 컨트롤러 이름을 바꾸면 `soarm_bridge/config/soarm.yaml` 의 `action_name` 도 같이 바꿀 것
- 4축이라 손목 방향을 마음대로 못 정함 → 목표 자세는 위치 + 접근 방향(위에서/옆에서) 위주로

---

## 7. 관절 정보 (실측)

| 관절 | 서보 ID | 모델 | count 범위 | URDF 범위 | 비고 |
|---|---|---|---|---|---|
| joint1 베이스 회전 | 1 | STS3250 | 860 ~ 3472 | −125° ~ +104° | + = 위에서 봤을 때 반시계 |
| joint2 어깨 | 2 | STS3250 | 1879 ~ 4095 | 0° ~ 195° | 0 = 앞으로 수평, 1879 = 대기(기구 끝) |
| joint3 팔꿈치 | 3 | STS3250 | 0 ~ 2047 | −180° ~ 0° | 2047 = L3 가 L2 위로 접힘(기구 끝) |
| joint4 손목 | 4 | STS3215 | 660 ~ 3273 | −108° ~ +122° | 0 기준은 아직 가정값 (RViz 로 확인 필요) |

- 변환: `rad = sign × (count − center) × 2π/4096` → `soarm_bridge/config/soarm.yaml`
- 링크: 베이스 높이 90, J1→J2 앞 30·위 100, L2 113, L3 135, J4→컵 끝 104 mm
- 대기 자세 count: J1 ≈2066, J2 1879, J3 2047, J4 ≈2258

---

## 8. 진공

- 동작: 잡기 = 밸브 닫고 펌프 ON, 놓기 = 펌프 OFF + 밸브 열기(대기 개방)
- 판정 기준 `hold_diff` (기본 -400): 허공 diff 와 잡았을 때 diff 의 **중간값**. 호스를 다시 꽂거나 세팅이 바뀌면 다시 재서 `config/soarm.yaml` 수정
- 압력센서 핀: 3 = GND, 4 = **5V**, 5 = OUT (분압 후 ESP32 G10)

---

## 9. 안전 (꼭 지킬 것)

1. **선 꽂고 빼기는 12V · USB 둘 다 끈 상태에서만** (이걸로 ESP32·TMC 한 번씩 날려 먹음)
2. 처음 움직일 때는 **손을 12V 스위치에** — 이상하면 바로 내리기
3. `/servo/torque false` 는 팔이 툭 떨어짐 → 손으로 받친 상태에서만
4. 서보 레지스터 40(토크)에는 0 / 1 만. 128 쓰지 말 것
5. 새 서보는 ID 가 1 → **혼자 연결해서** ID 바꾼 뒤 버스에 연결 (`soarm_bridge/tools/sts_set_id.py`)
6. 멀티미터로 잴 때 TMC·헤더처럼 촘촘한 핀은 전원 켠 채로 침 대지 말기 (퓨즈 나감)

---

## 10. 문제 해결

| 증상 | 확인 |
|---|---|
| `응답 없는 서보 ID` | 12V 켜졌는지, 서보 선, URT-2 USB |
| `/dev/esp32` 없음 | ESP32 를 **"USB"** 포트에 꽂았는지 (UART 포트 X) |
| ESP32 출력 안 들어옴 | 펌웨어 올릴 때 **USB CDC On Boot: Enabled** |
| diff 가 안 변함 | 압력센서 호스가 컵 쪽 진공 라인에 연결됐는지 |
| 카메라 한 대만 됨 | 둘 다 USB 3 (파란 포트), 시리얼 앞에 `_` 붙였는지 |
| 궤적 `GOAL_TOLERANCE_VIOLATED` | 팔이 걸렸거나 너무 빠름 → `max_speed` 낮추기, 전원 전압 확인 |
