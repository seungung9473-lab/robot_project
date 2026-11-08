# table_clear — SO-ARM 날짜 분류 시연

공유 브랜치: `TUMST_test`. 자연어 또는 고정 실행 명령으로 등록된 턴테이블 위 물체를 스캔하고, 날짜에 따라 폐기 위치·패드5·패드6으로 분류합니다. 음성 입력은 아직 포함되지 않습니다.

## 받기

새 작업공간을 만드는 경우:

```bash
git clone --branch TUMST_test --single-branch https://github.com/seungung9473-lab/robot_project.git ~/robot_ws
cd ~/robot_ws
```

이미 이 저장소를 사용하는 경우, 로컬 변경을 정리한 뒤 해당 작업공간에서 `git switch TUMST_test`와 `git pull --ff-only origin TUMST_test`로 갱신합니다. 현재 코드는 여러 경로에서 `~/robot_ws`를 사용하므로 이 디렉터리 이름을 유지하세요.

## 설치와 빌드

기본 환경은 Ubuntu24.04 / ROS2 Jazzy입니다. ROS, MoveIt, RealSense SDK/ROS 드라이버, colcon, rosdep가 필요합니다. ROS 저장소와 rosdep 초기 설정을 먼저 마친 환경에서:

```bash
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys ament_python
colcon build --base-paths src --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=73
```

### 별도 설치가 필요한 항목

Git에는 모델 가중치, Python 가상환경, API 키, 실험 원본을 넣지 않습니다. 저장소를 받기만 하면 GPU 추론까지 바로 준비되는 구성은 아닙니다.

- SAM2: 공식 설치 안내 https://github.com/facebookresearch/sam2 를 따라 CUDA 사용 가능한 PyTorch와 SAM2를 설치합니다. Python환경 위치는 `automation/reconstruction/.sam2_gpu/`, 가중치는 `automation/reconstruction/checkpoints/sam2.1_hiera_tiny.pt`입니다. SAM2.1 Hiera Tiny 설정을 사용합니다.
- 로봇용 시스템 Python에는 ROS Python 패키지와 `numpy`, `scipy`, `yaml`, `cv2`, `PIL`, `serial`, `zxingcpp`가 필요합니다. 가상환경에서 인식할 때도 해당 패키지를 사용할 수 있어야 합니다. `zxingcpp`의 pip 배포 이름은 `zxing-cpp`입니다. QR 모듈이 시스템 Python에 없으면 `run_sort.sh`의 시스템 Python 경로도 따로 준비해야 합니다.
- 데스크탑 시연은 `sam2_gpu_config.json`의 CUDA/FP32 설정을 유지합니다. CUDA가 없는 친구 장비에서 이 GPU 시연 경로가 동작한다고 보장하지 않습니다.
- Gemini 키는 환경변수 `GOOGLE_API_KEY`/`GEMINI_API_KEY` 또는 개인 설정 `~/.config/soarm4/gemini.json`을 사용합니다. 코드에 키를 적지 않습니다. `python3 automation/er2/check_connection.py --setup-key` 명령은 키를 가려서 입력받아 개인 설정에 저장합니다.

## 실행

### 안내만 확인 — 장치·API 실행 없음

```bash
cd ~/robot_ws
./run_sort.sh
SOARM_DEMO_BACKEND="$HOME/robot_ws" ./run_tidy_demo.sh "테이블 정리해"
```

자연어 진입점은 인접한 `robot_ws_mybranch`가 있으면 그 백엔드를 자동 선택할 수 있으므로, 공유 브랜치만 쓰려면 위처럼 `SOARM_DEMO_BACKEND`를 명시합니다.

### 실제 분류

서보·진공·턴테이블 제어 브리지, 두 카메라, MoveIt와 작업대 장면을 먼저 준비해야 합니다. 진입점은 드라이버를 자동 기동하지 않습니다. 장치 연결 및 보정 관련 준비법은 각 `src/` 패키지의 안내를 확인하세요.

```bash
cd ~/robot_ws
# 고정 날짜 분류: HOME, 회전 스캔, 인식, 경로 검증, 집기·분류
./run_sort.sh --execute

# 또는 자연어 지시 해석을 포함하는 시연
SOARM_DEMO_BACKEND="$HOME/robot_ws" ./run_tidy_demo.sh "테이블 정리해" --execute
```

`--execute`는 실제 모터·턴테이블·펌프 동작을 요청합니다. 두 명령을 동시에 실행하지 않습니다. 장치·압력·시작 자세가 확인되지 않거나 경로가 실패하면 중단합니다. 이 배포 정리에서는 모터를 작동시키거나 실제 완주를 확인하지 않았습니다.

## 현재 분류 규칙

- 한국 시간의 실제 오늘 날짜를 사용하며, 유통기한 당일까지는 유효합니다.
- 인쇄 날짜를 먼저 인식하고, 불확실하면 QR로 보완합니다. 관측이 확정되지 않은 물체는 이동하지 않습니다.
- 지난 날짜는 저장된 `trash_drop` 자세로 운반합니다.
- 남은 날짜 중 가까운 그룹은 ID5, 먼 그룹은 ID6입니다. 고정7일 기준이나 음성으로 임계값을 바꾸는 기능은 구현되어 있지 않습니다.
- ID5/6을 관객 기준 왼쪽/오른쪽으로 부르는 매핑은 실제 장치 배치와 확인해야 합니다.

## 파일과 기록

- `automation/reconstruction/`: 실행·인식·복원·경로·흡착 코드와 필요한 설정.
- `automation/er2/`: Gemini 연동.
- `src/`: ROS 패키지, 모델, 장치·작업대 설정.
- `automation/test_data/`: 새 실행의 로컬 기록. `test_data_recorder.py`는 실행 중 필요하므로 유지합니다.

기존 실험용 보정/마커 도구와 독립 카메라 미리보기, reconstruction 설명 문서·테스트는 실행 배포에서 제거했습니다. 원본 측정과 보정 재개 자료는 작성자 PC에 별도 보관합니다. 현재 보정 후보는 검증을 통과하지 않아 적용되지 않았습니다. 저장소의 작업대·카메라·툴·폐기 자세 설정이 친구의 실제 장치와 맞는지 확인해야 합니다.

재생성 가능한 `build/`, `install/`, `log/`, 캐시는 Git 제외 대상입니다. 원본 작업공간의 기존 build/install과 로봇 설정은 변경하지 않았습니다.
