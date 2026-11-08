# 2026-11-08 다른 PC 작업 인계

브랜치: `mybranch`. 친구 `TUMST_test`의 a475c74 파일을 CPU 노트북 구성에 통합한 스냅샷이다. Git merge 이력 대신 파일 통합으로 보존했다.

## 포함한 변경

- SAM2 CPU/FP32 실행과 이미지 인코딩 공유. CUDA 선택 코드는 참고용으로 남지만 기본 전체 실행은 CPU다.
- RViz 초기 velocity 0.30, acceleration 0.40. 이후 RViz에서 변경 가능.
- Pilz PTP 우선 홈 복귀, OMPL 대체 경로.
- 기존 모터 영점·변환·리미트 유지. J1 endpoint 미세 보정 최대 8count, 다른 관절 4count.
- J3 I 시험은 개선이 확인되지 않아 P32/D32/I0/적분 한도0으로 실물 복원. YAML은 기록이며 새 모터에 자동 설정하지 않는다.
- 토크 ON 이전 현재 위치 유지 목표를 읽어 확인하는 보호 추가.
- 새 전체 실행 전에 이전 턴테이블·패드 스캔 표시/임시 충돌 모델 정리. 고정 작업대 형상·보정·과거 파일 보존.
- RViz 작업대/복원 물체 MarkerArray 표시와 기존 제어 패널 유지.
- TCP/IK 엔코더 오차 측정, 홈 왕복/6지점 반복 시험 도구와 보수적 툴 간격 검사.

## 다른 PC에서 받기

기존 작업이 있으면 먼저 별도 백업한다. 새 폴더로 받는 예:
```bash
git clone --branch mybranch git@github.com:seungung9473-lab/robot_project.git ~/robot_ws
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys ament_python
colcon build --base-paths src --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=73
```

Ubuntu 24.04/ROS2 Jazzy 환경 기준. 이번 PC에는 `ros-jazzy-pilz-industrial-motion-planner`와 `ros-jazzy-moveit-kinematics`가 설치돼 있다.

## Git에 없는 로컬 준비물

- SAM2 Python 환경: `automation/reconstruction/.sam2_cpu`. CPU PyTorch 2.5.1+cpu로 확인했다. SAM2 소스 및 해당 Python 의존성을 별도 설치한다.
- SAM2 가중치: `automation/reconstruction/checkpoints/sam2.1_hiera_tiny.pt`.
- Gemini API 키: 새 PC에서 `python3 automation/er2/check_connection.py --setup-key`로 설정한다. 키 파일은 복사해 Git에 넣지 않는다.
- 직렬 포트 udev 규칙/접근 권한, RealSense 설치, 카메라 실물 고정 상태를 별도 확인한다.
- 턴테이블 소프트웨어 영점 `~/.ros/soarm4/turntable_zero.json`은 Git 외부다. 저장소의 영점 예제 count=1을 실제 원판 기준 확인 없이 적용하면 안 된다.
- 원본 시험 데이터, 스캔 영상, 가상환경, 보정 이력은 제외했다. 현재 작업대/카메라 행렬은 `src/soarm4_workcell/config/workcell.yaml`에 포함돼 있다. 카메라를 움직였다면 재보정한다.
- 보정 핀 시험은 `~/바탕화면/TUMST_ARM_cal_pin_v3_1/TUMST_ARM_cal_pin_v3.stl`을 참조한다. 다른 PC에서 이 실물 시험을 재개하려면 파일과 툴을 함께 준비한다.

## 경로 주의

기존 코드와 저장 보정 지문에는 `/home/seungung9473/robot_ws` 절대 경로가 일부 남아 있다. 다른 사용자명으로 실행하면 경로를 점검해야 한다. 특히 `named_poses.yaml`의 calibration_hashes는 파일 내용 검증을 유지한 채 새 경로로 옮긴다. 확인값을 임의로 없애거나 실패를 무시하면 안 된다. `configuration_revalidation.record` 등 과거 시험 경로는 이 PC 기록의 출처이며 새 PC에 없을 수 있다.

## 현재 검증 상태

- ROS 4개 패키지 빌드 성공. 최신 단위 테스트 101개 통과.
- CPU SAM2 저장 영상 1장/1프롬프트: 로딩·입출력 포함 약 3.42초. 전체 스캔 처리시간 아님.
- 실물 홈 이동, 30도 간격 12방향 정지 촬영 및 0도 복귀, ER2 인식, CPU SAM2 3개 물체 형상 복원/날짜 판독 확인.
- 실물 전체 집기/분류 완주는 아직 확인되지 않았다. 이전 시험은 패드5 배치 후보가 없어 집기 전에 중단됐다. 새 배치 시험도 있었으므로 과거 로그/스캔을 현재 장면으로 재사용하지 않는다.
- 이전 execution_safety_test.py는 현재 보정 영점/브리지 API와 맞지 않는 오래된 통합 모의 시험이다. 최초 hold 읽기 검증 누락은 실제 코드에 보완하고 별도 단위 테스트 3개로 확인했다. 오래된 모의 시험 전체 통과로 보고하지 않는다.

## 실행

기존 브리지·카메라·MoveIt·작업대 장면을 준비한다. 실물 주변을 정리하고 패드를 비운 상태에서 실행한다. 다른 컴퓨터에서 같은 하드웨어를 동시에 제어하지 않는다.
```bash
cd ~/robot_ws
export SOARM_SAM2_DEVICE=cpu
export SOARM_SAM2_PYTHON="$HOME/robot_ws/automation/reconstruction/.sam2_cpu/bin/python"
./run_sort.sh --execute
```

모든 새 전체 실행은 시험 기록기에 기록된다. 안내만 보려면 `./run_sort.sh`를 실행한다. 자연어 진입점은 `./run_tidy_demo.sh`다. Git에 올리는 작업에서는 모터·펌프를 새로 구동하지 않았다.

## 브랜치 정리

사용자 요청으로 최신 통합 스냅샷을 mybranch에 반영했다. 기존 mybranch 커밋 이력과 통합 커밋 c1082b8은 병합 이력에 보존했다. 임시 CPU 통합 브랜치는 삭제한다.
