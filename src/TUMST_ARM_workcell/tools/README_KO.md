# 스캔 → 복원 → 집기 → 옮기기 (실험용 도구)

11/3 송완재 노트북에서 블록 1개로 확인한 순서예요. 코드는 ROS 패키지 밖의 스크립트라서 `python3 파일.py`로 돌려요. **API 키는 파일에 쓰지 않고 환경변수로만** 넘겨요.

## 준비
- ROS 쪽: `live.launch.py allow_execution:=true`, `scene.launch.py`, `dual_d415.launch.py`, `turntable.launch.py read_only:=false`, 진공 브리지(esp32_bridge). `ROS_DOMAIN_ID`는 같게.
- 카메라 영상이 끊기면 `~/.ros/fastdds_shm.xml` (`soarm_bridge/config/fastdds_shm.xml`)와 `FASTRTPS_DEFAULT_PROFILES_FILE`.
- **SAM2 환경 `~/sam2_env`** (저장소 밖, 이 노트북에서 만든 것): Python 3.12 venv, torch 2.15(CPU), numpy, scipy, opencv-python-headless, pyyaml, hydra-core, iopath, pillow, SAM2.1 small 체크포인트(`~/sam2_env/checkpoints/sam2.1_hiera_small.pt`).
  정확한 설치 명령은 기록해 두지 않았어요. 공식 SAM2 저장소(facebookresearch/sam2)의 설치 안내를 따르고, `sam2_masks.py`가 위 체크포인트 경로를 읽으니 같은 위치에 두면 돼요. GPU 없이 CPU로 24장에 약 40초.
- Gemini 키: `export GEMINI_API_KEY=...` (실행하는 터미널에서만). 쓰고 나면 키를 폐기하고 새로 받는 걸 권장.

## 1. 스캔 (턴테이블이 돌아감: 먼저 주변 정리, 팔은 HOME)
```bash
ros2 run soarm_turntable turntable torque on
ros2 run soarm_turntable turntable_scan scan --angles 0 30 60 90 120 150 180 210 240 270 300 330
```
- **`turntable zero`는 쓰지 마세요.** 현재 위치를 새 영점으로 덮어써요 (영점은 `turntable_zero_20261103.json`, count 1).
- 토크 ON이 "엔코더가 단회전 범위 밖"이라며 실패하면 원판을 손으로 반시계 방향으로 조금 돌려서 count가 0 이상이 되게 하세요.
- 결과: `~/robot_ws/data/turntable_scans/<세션>/view_001..012` (view_000은 기준 촬영).

## 2~5. 복원 (모터 안 움직임)
```bash
S=~/robot_ws/data/turntable_scans/<세션>; A=~/robot_ws/data/analysis
python3 gemini_detect.py $S gemini-robotics-er-2-preview $A/boxes.json            # Gemini가 물체 박스를 찾음 (24장)
~/sam2_env/bin/python sam2_masks.py $S $A/boxes.json $A/sam2.npz $A/overlay        # 박스 → SAM2 마스크
~/sam2_env/bin/python recon.py $S $A/sam2.npz $A/cloud.npz                         # 12방향 점군 융합 (회전 방향·축 보정 확인)
~/sam2_env/bin/python pickup.py $S $A/cloud.npz $A/pickup.json gemini-robotics-er-2-preview   # 윗면 평면 → 후보 → Gemini 선택 → 흡착 위치
python3 -m soarm4_workcell.pickup_markers --cloud $A/cloud.npz --result $A/pickup.json       # RViz에 점군·흡착 위치 표시
```
- 원칙: **Gemini 좌표를 로봇 명령에 바로 쓰지 않아요.** Gemini는 물체 박스와 후보 선택만 하고, 최종 위치는 깊이·3D로 계산해요.
- 11/3 결과(빨간 블록, 윗면 4.7 cm, 높이 5.7 cm): 흡착 위치 (1.2498, 0.5136, z 0.1158), 윗면 평탄도 1.1 mm, 안에 들어가는 원 반지름 23 mm.

## 6. 집기·옮기기 한 사이클 (팔·펌프가 움직임: 실행 전 주변 확인)
```bash
python3 -m soarm4_workcell.pickplace_cycle --place pad_ID5 --scale 0.3 --block-h-mm 56.8 --log speed_corrected
```
- 블록 위치는 `block_measure`(빨간색 분할 + 깊이, 약 1초)로 매번 새로 재요. 블록이 빨간색이 아니면 색 기준을 고쳐야 해요.
- `--scale` 속도·가속도 배율(0.3에서 사이클 약 30초). 운반 중 압력이 −200보다 약해지면 `/servo/stop`.
- 놓는 위치 보정값은 `workcell.yaml`의 `place_correction_m`.
- 기록은 `experiment.py`(`~/robot_ws/data/experiments/<이름>/`). 요약: `python3 -m soarm4_workcell.experiment summary <폴더>`.

## 11/3 결과와 이어서 할 일
- 흡착 풀림 0/12, 사이클 평균 약 30초(배율 0.3).
- 놓인 위치: 평균 보정 전 ID5 Y +5.4 mm, 턴테이블 Y −8.3 mm. **보정 후 ID5 (−3.7, +0.7), 턴테이블 (+3.5, −3.9) mm** — Y는 줄었지만 X가 커졌고 시험이 2번씩이라 아직 확정 아님.
- **의심하는 원인**: 두 카메라가 같은 블록 윗면 중심을 8~14 mm 다르게 봐요 (`cam_gap_mm`). cam2(낮은 각도)는 윗면이 얇게 보여서 부정확할 수 있어요. 시험할 것: 윗면 중심을 cam1만으로, 또는 `minAreaRect` 중심으로 구해서 같은 시험을 반복 (`block_measure.py`).
- 보정 3°(처짐)·J1 영점 2078·`positive_gain 0.043` 등은 `TUMST_ARM.yaml`·`workcell.yaml`에 이미 들어 있어요. 값이 맞는지는 `EXPERIMENTS_KO.md`의 S1 시험(빈 컵 위치)부터 확인하세요.
- 아직 안 한 것: 회색 블록·두 물체 동시, ID6 패드 놓기 보정, 여러 모양, 10번 반복 성공률.
