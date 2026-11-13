# SO-ARM 전체 자동 실행

자동화 코드·SAM2 모델·가상환경·AI 연동·보정 참조·시험 자료를 `~/robot_ws` 안으로 통합했습니다.

## 실행

기존 로봇·진공·턴테이블·카메라 ROS 브리지가 정상 실행 중일 때:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=73
ros2 run soarm_pipeline expiry_sort --execute
```

동일한 실행을 하는 짧은 명령:

```bash
~/robot_ws/run_sort.sh --execute
```

`--execute` 없이 실행하면 안내만 표시하며 로봇을 움직이지 않습니다. 브리지를 자동으로 중복 기동하지 않습니다.

순서: 현재 장치·빈 컵 상태 확인 → 홈 → 30° 간격 12방향 정지 촬영 → 0° 복귀 → Gemini/SAM2 물체 분리·형상 복원 → 인쇄 날짜 우선·QR 보완 → 전체 경로 검사 → 집기·분류·홈. 기존 충돌·압력·피드백 검사는 유지됩니다. 날짜 인식이나 경로가 확정되지 않으면 실행을 중단할 수 있습니다.

## 실제 파일 위치

- `~/robot_ws/automation/reconstruction/`: 전체 실행·복원·집기·분류 코드, SAM2 가상환경·가중치, 촬영·실행 자료.
- `~/robot_ws/automation/er2/`: Gemini 연동, 모델 전환 정책, 인식 자료.
- `~/robot_ws/automation/camera_calibration/`: 보정 도구와 과거 보정 자료. 현재 로봇 배치값은 기존 `src/TUMST_ARM_workcell/config/workcell.yaml`을 사용합니다.
- `~/robot_ws/test_data/`: 자동 시험 기록, `index.csv`와 시험별 센서·결과 파일.
- `~/robot_ws/src/TUMST_ARM_pipeline/`: ROS 실행 진입점.

API 키는 개인 설정 `~/.config/soarm4/gemini.json` 또는 기존 환경변수를 그대로 사용하며 저장소에 복사하지 않습니다. ROS·시스템 Python은 기존 Ubuntu 설치를 사용합니다. 데스크탑 GPU 시연의 SAM2 실행용 Python은 `automation/reconstruction/.sam2_gpu` 환경을 사용합니다. 가상환경과 모델 가중치는 Git에 포함하지 않으므로 별도 설치해야 합니다.

이전 `문서/ChatGPT/라파 대회` 폴더의 `reconstruction`, `er2`, `camera_calibration`, `test_data`는 새 실제 위치를 가리키는 호환 링크입니다. 과거 결과 파일 속 절대 경로를 보존하기 위한 것으로, 새 전체 실행 명령은 이 링크가 없어도 시작할 수 있습니다. 과거 기록에 연결된 경로를 계속 열려면 링크를 유지하세요. 시험 자료와 모델·가상환경은 Git 제외 대상입니다.

## 빌드

```bash
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install --packages-select soarm_pipeline
source install/setup.bash
```

통합 검증에서는 구문·단위 테스트·ROS 진입점·SAM2 추론만 확인하며, 모터·펌프 시험이나 유료 API 호출을 자동 수행하지 않습니다.

데스크탑 통합본은 실행 전에 CUDA/FP32를 확인하고, ROS 진입점도 `.sam2_gpu` Python을 사용합니다. 기본 백엔드는 현재 작업공간이며 CPU로 자동 대체하지 않습니다. 최신 설치·통합 검증 설명은 저장소 루트 README.md를 확인하세요.
