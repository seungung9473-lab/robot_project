# 두 D415 + XL330 정지 촬영

## 구현된 범위
- capture: 턴테이블을 움직이지 않고 두 카메라의 컬러 PNG, 정렬깊이 16bit PNG(16UC1이면 mm), CameraInfo 원문과 영상 시각, 실제 엔코더·모터 상태를 저장합니다.
- scan: 사용자가 지정한 각도 전체를 먼저 검사하고 각 위치로 이동→도착/정지 확인→새 영상 저장을 반복합니다. 정지 촬영 기준점(baseline)도 먼저 저장합니다.
- 카메라 여섯 메시지(두 대의 RGB/깊이/CameraInfo) 시각 차이는 최대100ms, 수신 지연 최대0.5초, 정지 전 영상 제외. 하드웨어 동기화는 아닙니다.
- Moving=false, 속도 레지스터 절댓값<=1, 최소0.5초 동안 엔코더 변화<=2count를 확인합니다. 도착 액션은 목표 오차6count도 검사합니다.
- 32FC1 깊이는 미터 단위 NPY로 보존합니다. 빈 깊이/잘못된 내부행렬/해상도 불일치/오래된 프레임을 거부합니다.
- scan은 신선한 /joint_states가 있어야 실행하며 로봇팔이 시작 위치에서 0.5도 넘게 움직이거나 피드백이1초 넘게 끊기면 턴테이블을 정지시키고 중단합니다. 이것은 실제 공간의 충돌 검사 대신이 아닙니다. 팔·케이블을 턴테이블 회전 영역에서 치워야 합니다.
- 범위 초과각, 통신 이상, 스캔 중 영점 변경, 외부 STOP을 감지하면 다음 시점으로 진행하지 않습니다. 자동 복귀/펌프/로봇팔 명령은 없습니다.

## 1. 카메라 터미널 (켜져 있으면 중복 실행하지 않음)
```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=73
ros2 launch soarm_bridge dual_d415.launch.py
```

## 2. 턴테이블 읽기 전용 터미널
Wizard 연결을 해제하고 실행합니다. 이 명령은 회전/토크/프로필을 쓰지 않습니다.
```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=73
ros2 launch soarm_turntable turntable.launch.py
```

## 3. 움직이지 않는 한 세트 촬영
별도 터미널에서 동일 ROS 환경을 불러온 뒤:
```bash
ros2 run soarm_turntable turntable_scan capture
```
기본 저장 위치는 ~/robot_ws/data/turntable_scans/YYYYMMDD_HHMMSS_ID/입니다. capture에는 영점 저장이나 토크 ON이 필요하지 않습니다. 영점이 없으면 angle_deg는 null, 실제 position_count는 보존합니다. 0도라고 임의로 표시하지 않습니다.

## 4. 회전 스캔 (주변 정리/로봇팔 정지 후 사용자가 명시적으로 실행)
읽기 전용 턴테이블만 Ctrl+C로 종료하고, 아래로 대체합니다. 다른 장치는 종료하지 않습니다.
```bash
ros2 launch soarm_turntable turntable.launch.py read_only:=false
```
다른 터미널에서:
```bash
ros2 run soarm_turntable turntable zero
ros2 run soarm_turntable turntable torque on
ros2 run soarm_turntable turntable_scan scan --angles 0 15 30
```
zero는 현재 위치를 기준으로 소프트웨어 원점을 저장합니다. 모터 Homing Offset/EEPROM은 변경하지 않습니다. 물체와 회전판이 흔들리지 않게 고정하고 주변을 치운 뒤 실행합니다. profile_velocity50/profile_acceleration10을 유지합니다. Robot /joint_states가 없으면 회전 스캔은 거부합니다. 모터 쓰기를 막는 기존 read-only 로봇 실행을 사용할 때도 ESP32 노드 기동 시 초기화가 있으므로 기존 실행 구성을 확인하세요.

정지:
```bash
ros2 run soarm_turntable turntable stop
```
또는 촬영 터미널 Ctrl+C. 통신 단절 중에는 소프트웨어 정지가 물리 정지를 보장하지 못하므로 전원을 안전하게 차단할 수 있어야 합니다. 완료 후 자동으로 0도로 돌아가지 않습니다.

## 저장 내용
session.json: 세션 상태(complete/failed/interrupted), 요청각도와 성공한 뷰 목록.
view_NNN/: cam1_color.png, cam1_depth_mm.png, cam2_color.png, cam2_depth_mm.png, metadata.json.
실패한 쓰기는 .pending-* 폴더에 남고 완료 뷰로 등록하지 않습니다. 원본은 임의 삭제/덮어쓰지 않습니다.

카메라-로봇 변환과 턴테이블 회전축 변환은 null입니다. 원본 수집 단계이며 보정되지 않은 여러 각도의 점군을 곧바로 겹치면 안 됩니다. 아직 평면 검출/흡착면 평가/자동 집기/VLA를 수행하지 않습니다.

## 확인 결과
실제 회전 없는 한 세트 촬영 성공: 640x480 RGB/16bit 깊이 두 대, 실제 턴테이블 엔코더2049, 프로필50/10, 전압5.1V, 모터오류0, 시각 차이약47ms. 당시는 영점을 저장하지 않아 상대각도는 null입니다. 프레임 전체 유효깊이 비율은 cam1 약60%,cam2 약87%로, 물체 ROI의 품질을 뜻하지 않습니다. 턴테이블 위에는 물체가 없었습니다.
가상 모터/카메라로 다각도 저장, 범위초과 사전차단, 영점·모드·토크 조건, 오래된/시각이 맞지 않는 영상 거부를 시험했습니다. 실제 자동 회전 촬영 시험은 아직 하지 않았습니다.
