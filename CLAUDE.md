# TUMST_ARM — Claude Code 작업 메모 (CLAUDE.md)

이 파일은 이 저장소에서 작업하는 Claude가 매번 먼저 읽는 프로젝트 요약이다.
작성: 2026-11-02 (하드웨어 담당 송완재 + Claude 대화 내용 정리). 모든 답변은 한국어로, 짧고 편하게.

## 1. 프로젝트 한 줄 요약
SO-ARM101 기반 4축 로봇팔 + 진공 흡착 + 회전 턴테이블 + D415 카메라 2대로,
처음 보는 물체를 스캔·인식(ER2 + SAM2 + 깊이)해서 크기/색별로 패드 2개에 분류하는 시스템.

- 팀: 송완재(하드웨어, 이 노트북 주인, GitHub SWJ-04), 장승웅(소프트웨어, GitHub seungung9473-lab, 저장소 주인)
- 대회: 지능로봇대회 EXCO 11/13~14 (세팅 11/12), 라즈베리파이 대회 12/22
- 11/1 밤: 빨간 블록(큰 것)→ID5 패드, 회색 블록(작은 것)→ID6 패드, 홈 복귀까지 실물 성공 (단일 성공, 반복 성공률 미측정)

## 2. 이 노트북 (송완재)
- ASUS Zenbook UM5401RA (Ryzen, Radeon 680M), 윈도우 듀얼부팅
- Ubuntu 24.04.5 (nvme0n1p6, ext4) + ROS 2 Jazzy, `~/.bashrc`에 `ROS_DOMAIN_ID=73`
- 작업 폴더 ~/robot_ws, **작업 브랜치 TUMST_test** (main은 친구 것, 건드리지 않음)
- 빌드: `colcon build --base-paths src --symlink-install` (handoff/ 안에 중복 패키지가 있어서 반드시 --base-paths src)
- rosdep: `rosdep install --from-paths src --ignore-src -y -r --skip-keys ament_python`
- 미리보기(로봇 없이): `ROS_DOMAIN_ID=99 ros2 launch soarm4_workcell preview.launch.py`
  (친구 로봇이 같은 네트워크에서 73번으로 돌 때 섞이지 않게 99 사용)

## 3. 절대 규칙
1. main 브랜치에 push 금지. 작업은 TUMST_test, 합칠 때는 PR.
2. build/ install/ log/ 커밋 금지 (.gitignore 있음).
3. 공개 저장소다. Gemini API 키, 비밀번호, 토큰은 코드·커밋·로그에 절대 넣지 않는다. 키는 환경변수 `GEMINI_API_KEY`로만.
4. 모터·펌프가 움직이는 명령(토크 ON, MoveIt Execute, HOME, turntable goto/scan, 펌프 ON)은 실행 전에 반드시 사용자에게 먼저 물어본다.
5. 친구가 고친 파일(특히 soarm_bridge의 servo_bridge 계열)은 통째로 덮어쓰지 말고 필요한 부분만 diff로 수정.
6. 서보 EEPROM 값은 바꾸기 전에 현재 값을 기록. 주소 40에 128 쓰기 금지.
7. 배선 변경·커넥터 탈착은 12V와 USB를 끈 상태에서만 (사용자가 직접). XL330(5V)에 12V 금지, 5V 라인에 XT60 금지.
8. 파일·브랜치·파티션 영구 삭제는 사용자가 직접 한다.

## 4. 하드웨어 핵심 값
### 팔 (서보 버스 /dev/servo_bus, URT-2, 1 Mbps, STS 프로토콜, 12V)
- J1~J3 STS3250, J4 STS3215, ID 1~4
- count 2047 = 0°, 11.3788 count/°
- 브리지 엔코더 제한 (최신, 11/1): min [974, 1934, 0, 774] / max [3357, 4095, 2047, 3159]
- 홈 = URDF J1 0°, J2 180°, J3 ≈ −179.91°, J4 0°. J1 영점(centers)은 11/3부터 2078 (원래 2047, 베이스 조인 뒤 컵 실측으로 +31)
- J1 PID 조정값: Dead Band 3, P 20 (헌팅 방지)
- 브리지 속도 상한 250 count/s, ACC 10

### 진공 (ESP32-S3, /dev/esp32)
- 펌프 GPIO12, 솔레노이드 밸브 GPIO9, 압력센서 OUT → 10k/20k 분압 → GPIO10
- 압력센서 XGZP6847A (정상 확인됨: 대기 2.5 V, 흡입 시 감소)
- diff 기준(원시값, kPa 아님): 흡착 시작·운반 유지 ≤ −100 (11/9 사용자 요청, 11/5 −200 · 11/3 −300 · 원래 −400), 해제 확인 > −50. 스크립트 값은 automation/reconstruction/vacuum_thresholds.py 한 곳
- 운반 중에는 펌프 계속 ON, 밸브로 해제
- 흡착 실패 시: 현재 엔코더 기준 컵 위치에서 면 쪽으로 1 mm 접근해 1회만 재시도
- T 분배 피팅은 11/2에 베이스 안으로 옮김 (팔 움직일 때 튜브 빠짐 방지)

### 턴테이블 (/dev/turntable_bus, OpenRB-150 + XL330-M288-T)
- ID 1, 57600 bps, Protocol 2.0, 5 V(전원함 LRS-35-5), Profile Velocity 50, Profile Acceleration 10
- Homing Offset −4070 (사용자 지정), 소프트웨어 영점 ~/.ros/soarm4/turntable_zero.json
- **turntable_bus udev 규칙이 저장소에 없음** → OpenRB 꽂고 `udevadm info`로 VID:PID 확인해서 추가 필요
- Dynamixel Wizard와 노드를 같은 포트에 동시에 연결하지 말 것

### 카메라
- Intel RealSense D415 ×2: cam1 017122060095, cam2 017122060597
- 실행: `ros2 launch soarm_bridge dual_d415.launch.py`

### 툴·TCP
- J4 축 → 컵 끝 104 mm (툴 v5, 25 mm 연장), 흡착컵 Ø25 (FGA-25)
- 현재 TCP = tool_link 기준 [0.1157, −0.0005, 0] m (실측 13 mm 높이 오차를 임시 보정한 값)
- 원인 미분리: TCP 길이 vs J2 영점 vs 처짐. 처짐 실측: 흡착 직후 약 1.2°, 들어 올린 뒤 약 3.5°

## 5. 작업공간 좌표 (테이블 좌표계)
- 원점 = 테이블 왼쪽 아래 모서리(위에서 봄), +X 오른쪽, +Y 안쪽, +Z 위. Z=0 = 합판 윗면(테이블+6.5 mm). 테이블 180 × 75 cm
- ArUco DICT_4X4_50
  - ID0 100 mm: 턴테이블
  - ID1~4 40 mm: 툴
  - ID5·ID6 60 mm (Ø100 원형 스티커): 패드
  - ID7·8·10·11 70 mm: 테이블 고정 마커
- **11/3 재배치 (카메라 180° 돌림, 프로파일 클램프)**. 값은 workcell.yaml이 기준, 11/1 값은 workcell.yaml.before_relocation_20261103
- 로봇 전장상자 왼쪽 아래 모서리 (142.8, 42.2), 정면 −X, J1 ≈ (146.78, 52.20, Z 16.08) (11/3 보정 다시 시작: XY 보정 0, 처짐 보정 3°만 유지, 브리지 도착 후 자동 보정)
- 고정 마커 (cm): ID11 (100.0, 70.0), ID10 (109.5, 61.0), ID8 (114.7, 43.2), ID7 (137.7, 50.7)
- 턴테이블 축 (125.1, 51.4), 상판 Z 5.9 cm, 원판 Ø150
- 패드 (좌표는 흰 스티커 중심): ID5 (125.1, 36.7, Z 5.9, 앞·큰 물체, 검은 마커는 흰 스티커 안에서 +X 1 mm), ID6 (124.6, 66.7, Z 5.75, 안쪽·작은 물체), Ø100
- 충돌 물체 power_box (220V 전원함): X 165~190, Y −15~40, Z 0~30 cm
- 카메라 보정: `python3 -m soarm4_workcell.camera_calib --capture DIR --fresh [--write]` (고정 마커 4개, ID0·5·6으로 확인)
- 카메라·로봇·턴테이블·패드 위치가 바뀌면 보정 다시

## 6. 패키지와 실행 (src/)
| 패키지 | 역할 |
|---|---|
| soarm4_description | URDF v5.7, 메시, 관절 한계, config/model.json·servo_map.yaml |
| soarm4_moveit_config | MoveIt 2 계획(OMPL), 세부 충돌 검사, 실물 실행, HOME/STOP/토크 패널, 픽앤플레이스 편집기 |
| soarm_bridge | 서보 브리지 + ESP32 진공 브리지, dual_d415 실행 |
| soarm_turntable | XL330 제어, 환경 세팅, 정지 촬영·회전 스캔 (scan.py) |
| soarm4_workcell | 테이블 좌표계 배치, 충돌 물체 등록 (preview/scene/live launch) |

- 읽기 전용 확인: `ros2 launch soarm_bridge bringup.launch.py read_only:=true rviz:=true`
- 실물 제어: `ros2 launch soarm4_moveit_config hardware_bringup.launch.py allow_execution:=true` (기동만으로는 안 움직임, Execute 눌러야 움직임)
- 포트 하나 = 프로그램 하나 (motor_test.py, 브리지 동시 실행 금지)
- 각 패키지의 README_KO.md / HARDWARE_KO.md / PICK_PLACE_KO.md / SCAN_KO.md 가 최신 문서. handoff/README_인계.md 는 10/20 옛날 문서.

## 7. 비전 파이프라인 (코드 일부는 저장소 밖, 친구 노트북 ~/문서/ChatGPT/라파 대회/ 의 reconstruction/, er2/)
1. 홈 이동 → 턴테이블 30° × 12방향 × 카메라 2대 = RGB-D 24장
2. Gemini Robotics-ER 2 (gemini-robotics-er-2-preview, 대체 gemini-3.8-flash, 오프라인 시 로컬 Qwen3-VL)가 물체 지정
3. SAM2 마스크 → 테이블 좌표 점군 복원 → 흡착면 선택(위를 향한 평면, Ø25 + 여유, 평탄도, 두 카메라 일치)
4. 큰 물체 → ID5, 작은 물체 → ID6 → 홈
- 원칙: VLM 좌표를 로봇 명령에 바로 쓰지 않는다. 잡는 위치는 깊이·3D·MoveIt이 결정.
- 실행 진입점: reconstruction/run_two_object_sort.py

## 8. 남은 문제
- 촬영 대기 중 턴테이블 '움직임 감지'로 스캔 중단
- 스캔 시작 시 /joint_states 미준비 중단 → scan.py에 최대 5초 대기 2줄 추가로 대응
- SAM2 두 물체 마스크 겹침으로 중단 (일부 실행)
- TCP 13 mm 원인 분리 (블록→컵 실측 30 mm인지, 컵 수평 자세 오차 방향)
- 반복 시험(같은 조건 10회)으로 성공률·사이클 시간 측정
- J3 혼자 회전한 적 있음 (원인 미확정)

## 9. 알려진 빌드·환경 이슈
- `Duplicate package names` → handoff/ 때문. `--base-paths src`
- `Cannot locate rosdep definition for [ament_python]` → `--skip-keys ament_python`
- RViz 종료(Ctrl+C) 때 move_group Segmentation fault → 종료 중에만 나는 것, 무시
- ros-jazzy-control-msgs 없으면 실행 시 ModuleNotFoundError

## 10. 기록
- 통합 개발일지(하드웨어 R1~R24, 소프트웨어 S1~S9, 트러블슈팅 총정리): Claude Docs "TUMST_ARM 통합 개발일지"
- 새로 한 작업은 커밋 메시지에 "무엇을, 왜"를 남기고, 큰 변경은 사용자에게 개발일지에 추가할지 물어본다.
