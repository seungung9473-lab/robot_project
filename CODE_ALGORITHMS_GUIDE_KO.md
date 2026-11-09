# SO-ARM4 코드 구조·알고리즘 상세 설명서

> 2026-11-09 정리 반영: 최신 알고리즘별 요약은 CORE_ALGORITHMS_KO.md, 삭제·검증 내역은 CODE_CLEANUP_20261109_KO.md를 참고한다. 이 문서의 역사적 검증 설명은 당시 결과다.

작성일: 2026-11-08 (한국 시간)  
대상 작업공간: `/home/seungung9473/robot_ws`  
기준 브랜치: `mybranch`의 현재 작업 파일. 정리 작업의 미커밋 변경을 포함한다.

이 문서는 실제 파일을 읽어 정리한 코드 설명이다. 문서 작성 중 로봇·펌프·턴테이블을 구동하지 않았다. 파일에 기능이 구현되어 있다는 사실과 실제 로봇에서 그 기능의 전체 성공이 검증됐다는 사실은 구분한다.

## 1. 시스템 전체 역할

현재 시스템은 다음 구성요소가 협력하는 구조다.

1. Gemini: 물체의 의미·외형을 식별하고 인쇄된 날짜를 읽는다.
2. SAM2: 각 물체가 영상의 어느 픽셀에 해당하는지 분리한다. 현재 노트북에서는 CPU에서 실행한다.
3. 로컬 기하 계산: 깊이 영상을 3차원 점으로 바꾸고 형상·흡착면·좌표·관절 각도를 계산한다.
4. MoveIt 2: 로봇과 물체·작업대를 고려해 이동 경로를 계획하고 충돌을 검사한다.
5. 하드웨어 브리지: 관절 각도를 모터 엔코더 명령으로 변환하고 실물 피드백을 받는다.
6. 시험 기록기: 실행 시간·결과·오류·센서값·관련 파일 위치를 저장한다.

**SAM2도 AI다. ‘로컬에서 계산한다’는 말이 항상 ‘AI를 사용하지 않는다’는 뜻은 아니다.** Gemini는 원격 API 추론, SAM2는 로컬 AI 추론, 좌표 변환·IK·날짜 비교 등은 로컬 수치 계산이다.

현재 동작은 AI가 모터 명령을 직접 생성하는 구조가 아니다. AI의 관측 결과를 로컬 알고리즘과 검증 절차가 받아 최종 동작으로 연결한다.

## 2. 디렉터리와 단위

| 위치 | 내용 |
|---|---|
| `run_sort.sh` | 일반 전체 실행 명령 |
| `run_tidy_demo.sh` | 자연어 명령을 받는 전체 실행 명령 |
| `automation/reconstruction/` | 물체 분리·형상 복원·흡착면·날짜·경로·실행·기록 |
| `automation/reconstruction/motion/` | 공통 운동학과 이동 단계 관련 함수 |
| `automation/er2/` | Gemini 연결·모델 대체·물체 인식 |
| `src/soarm4_description/` | URDF, STL, 로봇 모델 설정 |
| `src/soarm4_moveit_config/` | MoveIt 설정·충돌 검증·홈·RViz 패널 |
| `src/soarm4_workcell/` | 테이블·카메라·패드·턴테이블 좌표와 측정 도구 |
| `src/soarm_bridge/` | STS 모터와 ESP32 진공 장치 제어 |
| `src/soarm_turntable/` | XL330 턴테이블 제어·정지 촬영 |
| `src/soarm_pipeline/` | 전체 실행을 ROS 명령으로 제공하는 얇은 진입점 |
| `test_data/`, `automation/test_data` | 시험 기록. 연결 경로는 실제 링크 설정에 따라 같다. |
| `build/`, `install/`, `log/` | 빌드 결과·설치 결과·ROS/빌드 로그 |

이하 파일 경로는 위 작업공간을 기준으로 표기한다.

- 3차원 위치: 대체로 m. 1 mm는 0.001 m.
- 관절 각도: ROS 내부에서는 rad. 화면 설명에는 °도 사용한다.
- 턴테이블 사용자 각도: 코드·JSON에 따라 °를 사용하고 액션 전송 시 rad로 바꾼다.
- 영상 좌표: 픽셀 `(u,v)` 또는 `(x,y)`.
- 모터 위치: 엔코더 count. 4096 count/회전이면 1 count는 약 0.08789°다.
- `diff=-200`: 현재 압력 신호의 판정 기준값이다. 센서의 물리 단위가 검증되지 않았다면 -200 Pa나 -200 kPa로 해석하면 안 된다.
- RViz 비율과 STS의 raw speed·acc는 서로 다른 값이다.

## 3. 실행은 어디서 시작되는가

### 3.1 일반 실행

파일: `run_sort.sh` → `automation/reconstruction/run_qr_expiry_cycle.py`

`run_sort.sh`는 ROS 환경과 작업공간 환경을 불러오고 `ROS_DOMAIN_ID=73`으로 전체 실행 파일을 호출한다. 자체적으로 물체를 인식하거나 IK를 계산하지 않는다.

`run_qr_expiry_cycle.py`가 실제 작업 순서를 관리한다.

```text
장치 제어 가능 여부·빈 컵 압력 확인
→ 다른 실행과 겹치지 않도록 잠금
→ 이전 턴테이블·패드 스캔 모델 초기화
→ 로봇 홈 이동
→ 30° 간격 12방향 정지 촬영
→ 턴테이블 0° 복귀
→ Gemini 물체 식별
→ CPU SAM2 분리와 형상 복원
→ 흡착면·관절 후보 생성
→ 인쇄 날짜 판독, 필요하면 QR 판독
→ 날짜에 따른 목적지 결정
→ 전체 물체 배치와 경로 검토
→ 검증된 집기·이동·놓기
```

`--execute`가 없으면 안내만 출력한다. 이 실행 파일은 브리지·카메라·MoveIt를 모두 자동 기동하는 프로그램은 아니다. 그 장치들이 먼저 준비되어 있어야 한다.

`src/soarm_pipeline/scripts/expiry_sort.py`는 같은 진입점을 찾아 실행한다. 그래서 `ros2 run soarm_pipeline expiry_sort --execute`와 셸 진입점이 같은 작업으로 이어진다.

### 3.2 자연어 실행

파일: `run_tidy_demo.sh` → `automation/reconstruction/tidy_demo.py`

“테이블 정리해” 같은 명령을 Gemini로 해석하고 지원되는 작업인지 검증한 다음 기존 인식·분류·계획 코드를 호출한다. 명령 해석 결과에 모터 위치를 임의로 넣도록 허용하지 않는다.

일반 실행과 자연어 실행은 처리 함수를 공유하지만 상위 제어 코드는 별개다. 자연어 실행에는 조건을 만족한 실행 전 계획 실패에 한해 최대 한 번 재스캔하는 기능도 있다. 일반 실행도 같은 자동 재스캔 정책이라고 가정하면 안 된다.

## 4. 이전 스캔 초기화와 동시 실행 방지

파일: `automation/reconstruction/scan_scene_reset.py`, `pad_occupancy.py`

이전 실험에서 물체를 치웠는데 RViz에 남아 있던 것은 실시간 물체 감지가 아니라 **저장된 점군을 계속 그리는 프로세스** 때문이었다.

새 전체 실행 시작 시 다음을 수행한다.

- 이전 `reconstructed_models` 표시 프로세스를 종료한다.
- `scanned_block`, `held_block`과 지정된 스캔·가상 배치·패드 배치 접두사의 충돌 모델을 제거한다.
- `/workcell_markers`에서 `reconstructed_pair_rgb` 네임스페이스의 점군 마커만 지운다.
- 작업대·패드 본체·턴테이블·카메라 표시·다른 장애물은 유지한다.
- 삭제 전후 장면과 정리 결과를 기록한다.
- 로봇에 부착된 물체가 있으면 초기화를 거부한다.

`scan_pick_cycle.lock`은 두 실행이 동시에 장면·턴테이블·로봇을 조작하지 못하게 하는 파일 잠금이다. 파일 자체를 지우는 것으로 실행 중 잠금을 해결하려 하면 안 된다.

이 초기화는 **물리적으로 놓인 물체를 감지해 치우는 기능이 아니다.** 새 실물 물체는 이후 촬영에서 새로 모델링한다. 과거 촬영·시험 파일도 삭제하지 않는다. 정리 도중 동작을 계속하지 않도록 새 전체 실행 경계에서만 호출한다.

## 5. 카메라와 턴테이블 촬영

| 파일 | 용도 |
|---|---|
| `src/soarm_bridge/launch/dual_d415.launch.py` | 시리얼 번호로 D415 두 대 실행. RGB·깊이·색상 정렬 깊이 발행 |
| `src/soarm_turntable/soarm_turntable/bridge.py` | XL330 상태·각도·액션·영점 서비스 제공 |
| `src/soarm_turntable/soarm_turntable/controller.py` | DYNAMIXEL SDK 읽기/쓰기, 토크·목표 위치·프로필 관리 |
| `src/soarm_turntable/soarm_turntable/scan.py` | 로봇 정지 확인, 회전, 안정화, 동기화 영상 저장 |
| `src/soarm_turntable/soarm_turntable/scan_core.py` | 정지 판정과 프레임 선택의 공통 로직 |
| `src/soarm_turntable/soarm_turntable/environment_core.py` | 영상 ROI·배경 깊이 차이 필터 |

### 5.1 각도 계산

턴테이블 각도는 대략 다음 관계를 사용한다.

`angle_deg = (현재 position_count - 저장 zero_count) × 360 / 4096`

소프트웨어 영점과 모터의 Homing Offset 레지스터는 서로 다른 개념이다. 홈을 2047로 임의 통일하지 않는다. 현재 `~/.ros/soarm4/turntable_zero.json`을 확인해야 한다.

### 5.2 움직이는 동안 촬영하지 않기

스캔은 목표 회전 완료만 믿지 않고 실제 피드백을 검사한다.

- `moving` 플래그, 실제 속도, 위치 변화, 상태 수신 시간을 검사한다.
- 정지 판정 후 추가 대기 0.5초를 적용한다.
- 대기 이전 프레임 버퍼를 비운다.
- 이후 촬영·수신된 RGB·정렬 깊이·CameraInfo만 선택한다.
- 캡처 중 위치가 2 count 넘게 변하면 중단한다.
- RGB와 정렬 깊이 해상도가 일치해야 한다.
- 스캔 중 로봇팔이 움직이거나 피드백이 오래되면 중단한다.

각 방향 폴더에 색상 이미지, 깊이 이미지, 카메라 내부 파라미터, 실제 엔코더 각도, 촬영 시간 조건을 저장한다. 계획 각도만 저장하는 방식이 아니다.

USB 대역폭이 부족하면 고해상도 프레임이 손상될 수 있다. 최근 실행에서는 두 카메라가 USB 2.1로 연결되어 640×480 설정으로 RGB와 정렬 깊이 수신을 확인했다. 1280 설정이 항상 가능한 상태는 아니다.

## 6. Gemini는 무엇을 계산하는가

| 파일 | 역할 | 처리 위치 |
|---|---|---|
| `automation/er2/check_connection.py` | 키 로드·연결 시험·API 통신 | 로컬 관리 + 원격 요청 |
| `automation/er2/select_object.py` | 영상 전처리, 물체 이름·점·상자 응답 검증 | 로컬 전처리 + Gemini |
| `automation/er2/model_router.py` | ER2 실패 유형에 따른 Flash 대체 | 로컬 분기 |
| `automation/er2/model_policy.json` | 주 모델·대체 모델·요청 횟수 정책 | 설정 |
| `automation/reconstruction/prepare_expiry_instances.py` | 실제 턴테이블 깊이 영역을 이용해 Gemini 입력 준비 | 로컬 + API 호출 연결 |
| `automation/reconstruction/read_numeric_dates.py` | 물체별 잘라낸 영상에서 인쇄 날짜 읽기 | Gemini + 로컬 응답 검증 |

현재 설정된 모델 식별자는 주 모델 `gemini-robotics-er-2-preview`, 대체 모델 `gemini-3.8-flash`다. 이는 현재 코드의 설정값이며 미래 API 제공 여부를 보장하는 문장이 아니다.

대체는 아무 오류에서나 발생하지 않는다. 설정상 일부 404·410·서버 오류·시간 초과·잘못된 출력에 한해 대체한다. 인증·결제·할당량 문제인 400/401/402/403/429에서는 자동 대체하지 않는다. 한 `generate` 호출의 모델 시도는 최대 2회다. 전체 작업에는 물체 식별과 물체별 날짜 판독 등 여러 호출이 있을 수 있다.

Gemini는 “초록색 블록이 이 위치에 보인다”, “라벨에 11.15가 보인다” 같은 관측을 반환한다. 3차원 흡착점·관절값·충돌 없는 경로를 확정하는 역할은 맡지 않는다. API 결과의 JSON 구조·좌표·근거 ID도 로컬에서 검사한다.

## 7. 물체를 서로 구분하고 SAM2 마스크 만들기

파일: `automation/reconstruction/process_three_color_objects.py`

이름에 `three_color`가 있지만 코드의 물체 수 조건은 1~8개다. 현재 날짜 분류 경로에서는 `--identity-only`로 사용하여 색상 기반 목적지 결정을 생략한다.

처리 순서:

1. 두 카메라의 깊이를 테이블 좌표로 변환한다.
2. 턴테이블 지지 영역과 높이 조건 등으로 물체 후보 점을 고른다. 일부 HSV 조건도 후보 생성에 사용한다.
3. Gemini가 지정한 픽셀 근처의 유효 깊이로 실제 3D 물체 씨앗점을 구한다.
4. 여러 물체의 점을 로컬 `kmeans2`로 군집화하여 물체별 중심을 대응시킨다.
5. 각 촬영 각도의 점을 역회전해 동일 물체가 어느 영역인지 연결한다.
6. 물체 안의 긍정 점, 다른 물체의 부정 점, 바운딩 박스를 SAM2 입력으로 만든다.

Gemini가 이름을 구분했다는 이유만으로 위치를 그대로 신뢰하지 않는다. 실제 깊이의 지지가 부족하거나 중심이 지나치게 가깝다면 중단한다.

### 7.1 SAM2 파일

| 파일 | 의미 |
|---|---|
| `apply_sam2.py` | SAM2.1 Hiera Tiny 로드, 장치 선택, 추론 실행 |
| `sam2_shared.py` | CPU에서 같은 영상의 특징을 한 번 계산하고 물체별 프롬프트에 재사용 |
| `sam2_gpu.py` | CUDA용 이미지 배치 추론. 현재 기본 전체 실행에서는 사용하지 않음 |
| `sam2_gpu_config.json` | 보존한 GPU FP32 설정 |

Hiera Tiny는 이미지 특징을 추출하는 작은 백본이다. SAM2 출력은 영상 크기의 마스크이며 물체 픽셀은 참, 나머지는 거짓으로 표현한다. SAM2가 물체의 길이나 로봇 좌표를 직접 측정하는 것은 아니다.

CPU 공유 방식은 **같은 영상을 물체 수만큼 다시 인코딩하는 중복**을 줄인다. 서로 다른 카메라·각도 영상을 하나의 동일 영상으로 취급하지는 않는다. 관측 불가능한 경우에는 마스크를 억지로 만들지 않고 가림으로 기록한다. 마스크 면적·중복·유효 관측 수 검사도 수행한다.

출력 예: `objects/object_1/sam2/view_001/cam1_mask.png`, 마스크 확인용 영상, 추론 목록.

## 8. 형상 복원: RGB-D를 점군으로 바꾸는 알고리즘

파일: `automation/reconstruction/reconstruct.py`, `placement_geometry.py`

### 8.1 깊이에서 3D 점으로

깊이 Z와 카메라 내부 파라미터를 이용한다. 왜곡을 무시한 기본식은 다음과 같다.

```text
Xcamera = (u - cx) × Z / fx
Ycamera = (v - cy) × Z / fy
Zcamera = 깊이
```

실제 구현은 `cv2.undistortPoints`로 왜곡 보정한 광선을 사용한다. SAM2 마스크에 포함되고 유효 깊이·작업 범위를 만족하는 픽셀만 고른다.

보정 행렬 `T_table_camera`를 적용해 카메라 좌표를 공통 테이블 좌표로 바꾼다. 그 후 실제 턴테이블 각도와 회전축을 이용해 모든 관측을 기준 각도로 역회전한다.

`p_reference = axis + Rz(-angle) × (p_table - axis)`

따라서 물체를 축 중심에 정확히 놓았다고 가정하지 않는다. 축에서 벗어난 물체도 축·각도·카메라 보정이 맞으면 기준 위치로 되돌릴 수 있다. 관측되지 않은 면까지 정확해지는 것은 아니다.

### 8.2 점군 정리

현재 `voxel()`의 주요 기본값:

- 복셀 크기 2 mm: 같은 작은 공간에 들어온 점과 RGB를 평균한다.
- 이웃 점까지의 평균 거리가 10 mm 이상인 고립 점을 제거한다. 질의 이웃 수는 자기 자신을 포함한 7개다.
- 8 mm 이내 점을 연결한 그래프에서 가장 큰 연결 성분을 남긴다.

큰 연결 성분만 남기는 처리는 잡음을 줄이지만, 얇거나 분리된 부품 일부도 제거할 수 있다. 모든 물체에 항상 무해한 필터는 아니다.

출력은 카메라별 `*_object.npz`, `*.ply`, 보고서와 선택적 미리보기다. `analysis/cloud.npz`에는 이후 계산에 쓰는 합친 점과 RGB가 들어간다.

**이 단계의 기본 결과는 관측 표면 점군이다. 물체 내부와 가려진 면을 채운 완전한 실제 CAD 메시가 아니다.**

## 9. 흡착 가능한 평면은 어떻게 찾는가

파일: `automation/reconstruction/inclined_geometry.py`, `plan_inclined_pick.py`, `shared_observations.py`

### 9.1 평면 검출

현재 기본은 모든 점의 곡률을 계산하는 방식이 아니라 **RANSAC 방식 평면 탐색 + SVD 평면 정밀화**다.

- 무작위 점 3개로 평면을 만든다.
- 평면에 가까운 점이 가장 많은 후보를 찾는다. 한 평면 탐색당 최대 600번 시도한다.
- 후보 점을 SVD로 다시 맞춰 법선과 평면식을 계산한다.
- 선택된 평면의 점을 제외하고 다음 면을 찾는다.
- 현재 평면 포함 거리 기준은 2.5 mm다.

평면식 `n·p + d = 0`에서 n은 면에 수직인 법선이다. 이 법선이 컵을 어느 방향으로 향하게 해야 하는지 계산하는 핵심이다.

### 9.2 컵 전체가 들어가는가

`patch()`는 평면 전체가 커 보여도 실제 컵 둘레가 지지되는지 별도로 검사한다.

| 항목 | 현재 기본값/조건 |
|---|---|
| 컵 반지름 | 12.5 mm, 지름 25 mm |
| 경계 여유 | 3 mm |
| 평면 후보 격자 | 1 mm |
| 원판 지지율 | 90% 초과 |
| 원판 샘플 근처 점 거리 | 3 mm 미만 |
| 평면 RMS 오차 | 2.5 mm 미만 |
| 기하학적 기울기 후보 | 최대 90° |

평면의 2D 볼록 경계를 구해 컵 반지름+여유가 들어가는지 검사한다. 최대 내접원 중심을 선형계획법으로 구해 좁은 유효 영역을 놓치지 않도록 보완한다. KD-tree로 원판 안의 지지 점과 틈을 확인하고, 표면 높이 잔차의 RMS를 구한다.

이 값은 **기하학적인 후보 조건**이다. 실제 고무 컵의 변형·누설·마찰·물체 무게를 완전히 예측하는 모델은 아니다. 실제 흡착 성공은 압력으로 확인한다.

### 9.3 두 카메라 교차 검증

한 카메라의 평면만으로 확정하지 않는다. 현재 코드에서는 두 면의 법선 차이 5° 이하와 평면·근접 점 거리 3 mm 이하 등을 검사하고, 공통 중심에서 두 카메라 모두 컵 원판 검사를 통과하는지 확인한다.

`shared_observations.py`는 동일한 깊이 프레임의 테이블 좌표 변환을 한 번 계산하고 물체별 마스크에 재사용한다. 결과를 바꾸기 위한 완화가 아니라 계산 중복 감소다.

## 10. 턴테이블 회전과 역기구학

파일: `inclined_geometry.py`, `automation/reconstruction/motion/kinematics.py`

### 10.1 정기구학 FK

관절값 `q1~q4`를 입력받아 링크의 원점 이동과 축 회전을 차례로 곱하고 마지막 TCP 오프셋을 적용한다. 출력은 TCP의 위치와 방향을 담은 4×4 행렬이다.

이 경로의 `fk()`는 `model.json`과 `workcell.yaml`을 읽는다. TCP 방향 계산에는 변환행렬의 +X 열을 사용하는 부분이 있으므로 “컵 방향은 언제나 Z축”이라고 가정하면 안 된다. 테이블의 +Z가 위쪽이라는 것과 툴의 로컬 축 정의는 별개다.

### 10.2 수치 IK

목표 TCP 위치와 툴 방향을 입력받아 `scipy.optimize.least_squares`로 관절값을 찾는다. 위치 오차와 방향 오차를 합친 잔차를 작게 만들며 관절 상·하한을 경계조건으로 건다. 여러 초기 관절값으로 시도하여 해가 있는지 탐색한다.

4자유도 로봇은 위치 3개와 방향 3개를 모두 독립적으로 제어할 수 없다. 법선을 따라 컵을 맞출 수 없는 후보는 탈락한다. 기하학적으로 90° 면을 허용해도 모든 옆면을 실제로 집을 수 있다는 뜻이 아니다.

### 10.3 턴테이블 정렬

`alignment_angles()`는 물체 면의 법선이 로봇의 방사 방향 피치 평면에 들어오는 턴테이블 각도를 찾는다. 각도를 구간별로 살피고 부호가 바뀌는 구간은 `brentq`로 근을 구한다.

`rotate_surface()`는 점과 법선을 함께 회전시킨다. 따라서 회전 전에 “물체를 돌리면 흡착면이 어디에 오는가”를 예측한다. 이 기울어진 면 집기 경로에서는 회전 후 접촉점을 영상으로 다시 맞추지 않는 `precomputed_prediction_only` 모드를 사용한다. 회전 전 관측을 저장하는 코드가 있는 것과 회전 후 재계산은 구분해야 한다.

### 10.4 처짐 보정

`sag_compensation.py`와 `compensated_axis()`는 로봇 방향·반경 등을 이용해 지령 방향을 보정한다. `plan_inclined_pick.py` 경로에는 기본 3° 방향 보정이 있다. 이는 질량·토크를 모두 푸는 물리 시뮬레이션이 아니라 실물 시험에서 얻은 경험 보정이다.

TCP 측정 도구의 `cup_goal.py`와 생산 경로의 JSON 기반 FK는 구현 경로가 다르다. URDF·JSON·실물 툴이 바뀌면 서로의 일치도도 다시 확인해야 한다.

## 11. 놓을 자리와 모든 물체의 실행 순서

| 파일 | 역할 |
|---|---|
| `pad_occupancy.py` | MoveIt의 기존 배치 물체를 읽고 점유 영역 계산 |
| `pad_packing.py` | 물체의 2D 외곽과 두 다각형의 분리 여부 계산 |
| `payload_placement.py` | 잡힌 물체와 TCP의 상대 자세를 고려한 놓기 IK |
| `tidy_pad_placement.py` | 패드 안에서 놓을 위치들을 탐색하고 공동 예약 |
| `plan_batch_pad.py` | 패드 예약 기능을 호출하는 진입점 |
| `plan_pick_batch.py` | 모든 물체의 후보·순서·놓기를 묶어 경로 검토 |
| `batch_search.py` | 물체 순서/후보를 바꾸며 제한된 백트래킹 탐색 |
| `planning_feedback.py` | 계산 진행 출력, 조건별 실패 결과 재사용 |

놓기 예약은 패드 지름, 물체 외곽, 기존 점유, 관절 IK를 함께 고려한다. 현재 기본은 2 mm 격자, 패드 가장자리 여유 2 mm, 물체 사이 여유 2 mm다. 쌓기를 기본으로 가정하지 않는다.

물체 외곽에는 관측 점군의 볼록 껍질 꼭짓점이 사용된다. 깊이 잡음이나 잘못 합쳐진 점이 멀리 있으면 외곽이 커져 배치 후보가 사라질 수 있다. **패드가 실제로 차 있다는 뜻과 계산에서 배치 후보를 못 찾았다는 뜻은 다르다.** 최근 실패도 점유 물체가 없는 `geometry` 경우가 있었다.

모든 물체를 집기 전에 앞선 물체가 놓였다고 가정한 가상 장애물까지 반영해 뒤 물체의 경로를 검토한다. 후보에 따라 물체 순서·회전 각도·놓을 위치를 바꾼다. 탐색 횟수·시간에 한도가 있으므로 “예산 내 경로 미확보”는 수학적으로 절대 불가능하다는 증명이 아니다.

실패 결과를 재사용할 때는 물체·점군 내용·작업대 장면·부착물·시작 자세·RViz 비율 등을 확인한다. 다른 배치의 실패를 무조건 가져다 쓰는 방식은 아니다.

## 12. 충돌 검사와 MoveIt 경로 계획

파일: `check_inclined_motion.py`, `collision_geometry.py`, `src/soarm4_moveit_config/`

### 12.1 형상 표시와 충돌 형상은 다르다

- 복원 표시: 실제 관측한 색상 점군을 RViz 마커로 그린다.
- 대상 물체 충돌 모델: 점군에서 `ConvexHull`을 만들어 삼각형 메시로 만든다.
- 주변 물체: 점군이 있으면 볼록 메시를 쓰는 경로가 있고, 없으면 상자 모델로 표현하는 경로도 있다.
- 로봇 본체: URDF의 visual·collision 정의를 사용한다.

볼록 껍질은 오목한 홈을 채울 수 있다. 따라서 화면의 관측 형상과 충돌 모델이 완전히 같지 않으며, 모든 충돌 모델이 실제 상세 메시라고 설명하면 틀리다. `convex_mesh()` 기본 padding은 0이지만 주변 물체 호출에서는 1 mm 등의 여유를 줄 수 있다. 물체 범위 계산에는 별도의 2 mm 확장도 있어, 전체 충돌 여유를 하나의 숫자로 표현하면 안 된다.

### 12.2 경로 계획

홈 복귀는 Pilz PTP를 먼저 시도하고 실패하면 OMPL RRT-Connect를 시도한다. 일반 계획도 해당 계획 파이프라인과 상태·경로 검사를 사용한다.

`src/soarm4_moveit_config/src/fine_collision_adapter.cpp`와 `scripts/fine_collision_service.py`는 모델에 따른 상세 경로 검토를 담당한다. 들고 있는 물체를 로봇에 부착한 상태와 놓은 후의 장애물도 단계에 맞춰 고려한다.

검사는 현재 장면에 등록된 장애물에 대해서만 유효하다. 임의로 놓은 추가 테이블·손·케이블이 장면에 없으면 충돌 검사를 통과해도 실제로 부딪힐 수 있다. 모델·좌표 보정의 오차도 남는다.

`batch_inputs.py`는 스캔·설정·실행 코드·날짜 결과 등의 SHA256 지문을 저장하여 계획 이후 입력이 바뀌었는지 검사한다. 계획과 실행 사이의 설정 변경을 조용히 무시하지 않는다.

## 13. 실제 집기·흡착·이동

| 파일 | 역할 |
|---|---|
| `execute_pick_batch.py` | 완성된 전체 계획의 단계별 실행, 완료 물체 기록 |
| `run_inclined_pick.py` | 선택한 물체·면·회전 각도로 실행용 세션 준비 |
| `verified_place_left.py` | 실제 접근·흡착·압력 감시·들기·이동·해제 처리 |
| `normal_contact_trajectory.py` | 면 법선 방향으로 연속 접근하는 궤적 생성 |
| `continuous_pre_suction.py` | 접촉 직전 이동 중 펌프 시작 조건 |
| `grip_retry.py` | 흡착 실패 후 제한된 추가 접근 목표 계산 |
| `execute_contact.py` | 짧은 접촉 이동·상대 접근 등의 별도 실행 도구 |

`verified_place_left.py`는 이름이 왼쪽이지만 `--pad 5/6`과 목적지 설정에 따라 다른 놓기 동작도 처리한다. 파일명이 전체 기능을 정확히 설명하지 않는 사례다.

### 13.1 연속 접근

`normal_contact_trajectory.py`는 TCP 위치를 접근점부터 접촉점까지 보간하고 각 중간 위치의 IK를 푼다. 기본 샘플은 81개다. 위치 보간에는 다음 5차 곡선을 사용한다.

`s(u) = 10u³ - 15u⁴ + 6u⁵`

시작·끝에서 속도와 가속도를 부드럽게 만들기 위한 것이다. 계산한 관절 속도·가속도가 한도를 넘지 않도록 전체 시간을 늘린다. 이 결과가 충돌 검사 자체를 대신하지는 않는다.

### 13.2 도착 1.5 mm 전 펌프 시작

`PreSuctionTrigger`는 실제 엔코더를 FK로 바꾼 TCP와 목표면 사이의 법선 방향 거리를 계산한다. 기본 1.5 mm 이내가 되면 한 번 펌프 명령을 보낸다. 이 트리거 자체는 압력을 읽어 발동하지 않고, 펌프를 켜기 위해 이동을 일부러 멈추지도 않는다.

이 거리는 별도 거리 센서 실측이 아니라 **엔코더와 모델에서 추정한 거리**다.

### 13.3 압력 확인과 재시도

현재 실제 집기 코드의 grip·hold 기준은 `diff=-200`이다. 들고 이동하는 동안 압력 피드백이 오래되거나 연속으로 흡착 유지가 깨지면 경로를 취소하는 로직이 있다.

`grip_retry.py`는 초기 접촉점에서 면 안쪽으로 누적 1·2·3 mm 목표를 생성한다. 현재 경로에서는 최대 3회 추가 접근을 준비하며, 매번 압력·동작 조건을 확인한다. 실패했다고 무한히 더 누르지 않는다. 재시도 종료 후 명시적으로 해제하는 분기도 있으므로 “모든 오류에서 펌프를 절대 끄지 않는다”고 일반화하면 안 된다. 자동 배치 실행의 다른 오류 처리에서는 자동 해제하지 않고 중단하는 경로도 있다.

## 14. 날짜는 AI가 읽고 로컬에서 비교한다

파일: `date_crops.py`, `read_numeric_dates.py`, `read_expiry_labels.py`, `expiry_policy.py`, `run_expiry_sort.py`

- `date_crops.py`: 물체 마스크에서 글자 판독에 사용할 영역을 잘라 모은다.
- `read_numeric_dates.py`: 잘라낸 영상의 인쇄 날짜를 Gemini에 읽게 하고 근거 ID·문자열·신뢰도 형식을 검증한다. 동일한 잘린 영상 내용이면 이전 응답을 검증해 재사용하는 코드가 있다.
- `read_expiry_labels.py`: 숫자 날짜 합의가 부족한 물체만 QR로 보완한다. QR 디코딩은 로컬 ZXing이고 Gemini가 QR 문자열을 추측하게 하지 않는다.
- QR이 어느 물체의 것인지 SAM2 마스크와 QR 영역의 겹침으로 대응시킨다. 현재 귀속 지지율 기준은 65%다.
- `expiry_policy.py`: 형식·날짜 유효성·여러 관측의 합의·실제 한국 날짜와의 차이를 계산한다.
- `run_expiry_sort.py`: 최종 날짜와 목적지를 묶고 전체 경로 검토로 넘긴다.

현재 규칙:

1. 날짜 당일까지는 유효하다.
2. 지난 날짜는 저장한 쓰레기통 자세 `trash_drop`으로 간다.
3. 남은 두 날짜 중 기준일에 가까운 날짜는 ID5, 더 먼 날짜는 ID6이다. 고정 7일 이내 규칙이 아니다.
4. 읽히지 않거나 상충하는 날짜는 옮기지 않는다.
5. QR은 첫 글자 뒤 YYMMDD 형식이다.
6. 인쇄된 MM.DD에 연도가 없으면 현재 실행 연도를 사용하는 로컬 정책이다. Gemini가 연도를 알아낸 사실로 해석하면 안 된다.
7. 기본 인쇄 날짜 합의에는 신뢰도 0.9 이상, 일치 관측 2개 이상 등의 조건을 사용한다.

날짜를 읽는 작업은 AI가 유리하지만, `11월 15일 - 오늘` 같은 달력 계산은 정확하고 빠른 로컬 코드가 맡는다.

## 15. 모터 명령과 피드백 보정

| 파일 | 역할 |
|---|---|
| `src/soarm_bridge/soarm_bridge/servo_bridge.py` | ROS 궤적→STS 명령, 엔코더→joint_states, 정지·토크 서비스 |
| `src/soarm_bridge/soarm_bridge/sts_bus.py` | STS 통신 패킷, 레지스터 읽기/쓰기, 동기 목표 전송 |
| `src/soarm_bridge/soarm_bridge/fine_limits.py` | 미세 보정 명령도 모터·모델 한도 안으로 제한 |
| `src/soarm_bridge/config/soarm.yaml` | 영점·부호·게인·속도·오차 허용·피드백 설정 |

이곳은 AI가 아니라 로컬 실시간 제어 코드다. ROS의 rad 목표를 중심 엔코더·회전 부호·게인으로 count로 변환한다. 반대로 실제 count를 rad로 바꿔 `/joint_states`에 발행한다.

현재 주요 설정:

| 설정 | 값 | 의미 |
|---|---:|---|
| `command_rate_hz` | 50 | 궤적 스트리밍 목표 주기 |
| `feedback_timeout` | 1.0초 | 브리지 피드백 시간 기준. 다른 모듈의 더 짧은 기준과 별개 |
| `max_speed` | 4000 | STS 명령 속도 상한. RViz 4000%가 아님 |
| `acc` | 50 | STS 레지스터 기반 가속 설정. RViz 비율과 다름 |
| `goal_tolerance_counts` | 15 | 브리지 도착 판정 관련 count 허용값 |
| `fine_correction_iterations` | 5 | 끝점 미세 보정 최대 반복 |
| `fine_tolerance_counts` | 1 | 미세 보정이 만족하려는 잔여 count 기준 |
| `fine_max_counts_per_joint` | [8,4,4,4] | J1~J4의 목표 주변 최대 추가 명령 범위 |

J1에는 기존 양의 방향 게인 0.043과 오늘 조정한 최대 8 count 미세 보정이 함께 있다. 미세 보정은 실제 엔코더 오차를 읽어 제한된 범위에서 명령을 보완한다. 한도에 막히거나 피드백이 없으면 무한 반복하지 않는다.

토크 ON 전에는 현재 위치를 유지하는 목표를 먼저 전송하고 각 모터의 목표 레지스터를 읽어 확인한다. 실패하면 그 경로에서 토크를 켜지 않는다. 이는 이전 목표로 갑자기 움직이는 위험을 줄이는 확인 절차다.

J3는 I=1 시험에서 홈 오차 개선이 확인되지 않아 원래 I=0·적분 한도0으로 복원했다. `config/j3_integral_trial.yaml`은 그 기록이며 실행 때 EEPROM을 자동 변경하는 설정이 아니다.

## 16. 진공 장치 제어

파일: `src/soarm_bridge/soarm_bridge/esp32_bridge.py`, `vacuum.py`

ESP32와 직렬 통신해 펌프·솔레노이드 밸브를 제어하고 압력 diff를 발행한다.

| ROS 기능 | 의미 |
|---|---|
| `/vacuum/pump_on` | 펌프 시작, 그 명령에서 압력 합의까지 대기하지 않음 |
| `/vacuum/grip` | 흡착 명령과 압력 기준 확인 |
| `/vacuum/release` | 펌프 OFF, 밸브 개방으로 해제, 이후 밸브 닫기 |
| `/vacuum/off` | 펌프·밸브 OFF. 해제 서비스와 같지 않음 |
| `/vacuum/diff` | 실제 수신한 압력 차 신호 |
| `/vacuum/holding` | 설정 기준에 따른 흡착 유지 상태 |

센서 없는 시간 기반 명령도 남아 있지만, 센서 확인 경로와 구분해야 한다. ‘펌프가 켜짐’과 ‘물체가 붙음’은 다른 상태다.

## 17. RViz·홈·표시 코드

| 파일 | 역할 |
|---|---|
| `src/soarm4_moveit_config/config/hardware_moveit.rviz` | MotionPlanning, 제어 패널, 작업대 MarkerArray 표시 설정 |
| `src/soarm4_moveit_config/src/home_panel.cpp` | HOME·STOP·토크·흡착·해제·수동 경로 편집 버튼 |
| `src/soarm4_moveit_config/scripts/home_manager.py` | 홈 요청·취소·상태 및 공통 관리 로직 |
| `src/soarm4_moveit_config/scripts/pick_place_manager.py` | 홈 관리 기능과 저장 경로·흡착/놓기 단계 관리 연결 |
| `src/soarm4_workcell/soarm4_workcell/scene.py` | 작업대 고정 충돌 모델·카메라/패드 마커·좌표 연결 |
| `src/soarm4_workcell/soarm4_workcell/reconstructed_models.py` | 저장 점군을 RGB 색상으로 표시 |

RViz는 제어 명령을 보내는 화면이지 카메라처럼 실물을 자동 갱신하는 화면이 아니다. `reconstructed_models`는 지정한 저장 스캔을 읽어 그린다. 실물 배치를 바꿔도 새 스캔을 연결하지 않으면 이전 모습이 남는다.

현재 점군 표시는 `/workcell_markers`의 MarkerArray를 구독하도록 구성했다. RViz만 수동으로 다시 실행할 때도 ROS와 `install/setup.bash`를 불러와야 사용자 패널·STL 패키지를 찾는다.

초기 속도/가속 비율은 0.30/0.40이다. 일부 실행 코드는 `/soarm4/rviz_scaling`을 통해 현재 UI 값을 읽는다. 이 서비스가 없으면 초기값을 임의로 추측해 실행하는 대신 실패하는 경로가 있다.

STOP은 소프트웨어 정지 요청이다. 독립 하드웨어 비상정지 장치와 같은 의미로 취급하면 안 된다.

## 18. 모델과 보정 설정 파일

| 파일 | 바꿀 때 생기는 영향 |
|---|---|
| `src/soarm4_description/urdf/soarm4.urdf` | 링크·관절·TCP·메시·리미트에 영향 |
| `src/soarm4_description/config/model.json` | 생산 경로 FK/IK의 모델·TCP 기준 |
| `src/soarm4_workcell/config/workcell.yaml` | 테이블↔로봇 변환, 카메라, 축, 패드, 보정 |
| `src/soarm4_workcell/config/named_poses.yaml` | 쓰레기통 등 저장 자세와 보정 파일 지문 |
| `src/soarm4_moveit_config/config/joint_limits.yaml` | MoveIt 속도·가속 한도 |
| `src/soarm_bridge/config/soarm.yaml` | 모터 변환·제어 설정 |

`trash_destination.py`는 저장 자세가 현재 보정 파일과 맞는지 SHA256으로 확인한다. 단순한 설명 파일로 보고 확인값을 삭제하면 안 된다. 오늘 추가한 끝점 보정 항목만 달라졌고 기존 좌표·변환·한도는 그대로였음을 비교한 후 지문을 갱신한 이력이 있다.

절대 경로 `/home/seungung9473/robot_ws`가 일부 남아 있어 다른 사용자 계정으로 옮기면 경로 검토가 필요하다. 보정 행렬이 Git에 있다고 새 카메라 배치에서도 그대로 유효한 것은 아니다.

## 19. 오차 측정·보정 시험용 코드

이 도구들은 일반 날짜 분류의 필수 실행 경로가 아니다. 파일이 남아 있다는 이유로 전체 실행 때 자동으로 수행되지 않는다.

| 위치 | 용도 |
|---|---|
| `src/soarm4_workcell/soarm4_workcell/cup_goal.py` | URDF 기반 FK/IK와 TCP 목표 도구 |
| `.../tcp_error_test.py` | 목표·실제 관절값 수집 공통 기능 |
| `.../tcp_error_metrics.py` | 관절 오차를 TCP 위치 오차로 환산 |
| `.../motor_limits.py` | 모터 count와 모델 각도의 공통 허용 범위 |
| `.../tool_clearance.py` | 툴/핀의 보수적 형상 간격 검사 |
| `.../random_tcp_plan.py` | 무작위 TCP/관절 후보와 이동 경로 사전 계획 |
| `.../random_tcp_execute.py` | 검토한 무작위 목표를 실제 실행·기록 |
| `.../cycle_error_test.py` | 턴테이블→왼쪽→홈→오른쪽→홈→쓰레기통 반복 측정 |
| `automation/reconstruction/run_tcp_50.py` | 50개 측정 작업 관련 실행 도구 |
| `automation/reconstruction/j3_integral_tune.py` | J3 I/적분 한도 읽기·제한된 시험·복원 |

`.../`는 이 표에서 `src/soarm4_workcell/soarm4_workcell/`을 뜻한다.

기본 측정 원리:

```text
관절 오차 = 실제 엔코더 변환 각도 - 목표 각도
TCP 추종 오차 = FK(실제 각도) - FK(목표 각도)
목표 위치 대비 총 모델 오차 = FK(실제 각도) - 요청 TCP 위치
```

Jacobian을 수치 미분하여 관절별 `J_i × 각도 오차_i`도 계산한다. 큰 각도 차이나 비선형 영역에서는 선형 근사 잔차가 생긴다.

**이 값은 엔코더와 로봇 모델로 계산한 TCP 오차다. 카메라·측정기 없이 실물 컵 중심의 절대 오차를 직접 측정한 값은 아니다.** 링크 휨·기어 유격·툴 길이 오차가 있으면 엔코더가 맞아도 실제 끝점은 어긋날 수 있다.

`tool_clearance.py`의 보정 핀 STL은 작업공간 밖의 바탕화면 경로를 참조한다. 다른 PC에서 그 시험을 재개하려면 해당 실물 툴·파일을 함께 준비해야 한다.

## 20. 작업 시간·센서·실패 기록

파일: `automation/reconstruction/test_data_recorder.py`

`start_trial()`은 새로운 시험 번호를 만들고 시작 시각·종류·실물 실행 여부를 저장한다. 이벤트와 센서 샘플은 JSONL로 남긴다. 종료 시 `summary.json`에 종료 시각·`duration_s`·결과·오류를 기록한다.

| 파일/값 | 뜻 |
|---|---|
| `summary.json` | 시험 시작·종료·성공/중단·총 시간 |
| `events.jsonl` | 단계별 사건과 시작 이후 경과 시간 |
| `telemetry.jsonl` | 저장하도록 연결된 센서 신호 |
| `index.csv` | 여러 시험을 찾기 위한 목록 |
| `planning_feedback.py`의 calculating() | 계산 중 기본 5초마다 경과 출력 |
| `tidy_demo.py`의 stages/total_seconds | 자연어 실행 단계별 시간과 전체 시간 |

일반 실행 `run_sort.sh`의 전체 시간은 상위 `expiry_cycle` 기록을 봐야 한다. 단순히 가장 최근 `summary.json`을 고르면 하위 인식·계획 시험 시간이 잡힐 수 있다. `duration_s`는 종료 시 확정되므로 실행 중에는 시작 시각 또는 이벤트의 `elapsed_s`로 경과를 계산해야 한다.

실행이 중단돼도 기록을 성공으로 바꾸지 않는다. 과거에 저장하지 않은 센서값은 추정해서 채우지 않는다.

## 21. 남아 있는 보조·이전 시험 도구는 왜 바로 지우지 않았는가

다음 파일들은 한 번의 전체 실행에서 모두 호출되는 것이 아니다. 다른 실험·복구·시각화 진입점이므로, 호출되지 않는다는 이유만으로 필요 없다고 확정하지 않았다.

| 묶음 | 대표 파일 | 용도 |
|---|---|---|
| 이전 고정 개수 물체 흐름 | `process_two_objects.py`, `process_same_color_pair.py`, `execute_two_object_sort.py`, `run_two_object_sort.py`, `finish_two_object_sort.py` | 과거 1~2개 물체 시험 흐름 |
| 고정 상태 물체 관측 | `detect_stationary_objects.py`, `find_stationary_alignments.py`, `prepare_stationary_grip.py` | 회전 없이 관측·정렬·집기 후보 준비 |
| 스캔→집기 보조 진입점 | `process_scan.py`, `process_current_scan.py`, `run_scan_to_pick.py`, `execute_scan_pick.py` | 이전 단계별 파이프라인 |
| 접촉·정렬 검증 | `check_cup_alignment.py`, `verify_horizontal_pick.py`, `refine_aligned_candidate.py`, `refine_contact_in_place.py`, `refine_reference_view.py`, `replan_contact_at_fixed_table.py`, `prepare_local_contact.py` | 접근 후보 확인·보정 시험 |
| 형상·흡착면 진단 | `audit_last_scan.py`, `measure_flat_faces.py`, `find_suction_surface.py`, `validate_offcenter.py` | 점군과 흡착면 진단 |
| 실패·들기 진단 | `recover_failed_pick.py`, `review_held_lift.py`, `record_suction_diagnosis.py`, `verified_suction_trial.py` | 상태 복구 검토·들기/압력 기록 |
| 점군 시각화 | `display_all_models.py`, `display_pair_clouds.py` | 저장 형상의 별도 표시 |
| 프롬프트 준비 | `prepare_sam2_prompts.py` | 별도 SAM2 프롬프트 생성 도구 |
| 궤적 재사용 | `trajectory_cache.py` | 기존 경로 데이터의 재사용 지원 |
| 홈 보정 후보 | `controller_homing_candidate.py` | 홈/제어기 보정 후보 분석 |

이 표는 각 파일의 책임을 찾기 위한 색인이다. 이 보조 도구 전부를 현재 실물에서 새로 검증했다는 뜻이 아니다. 실험 파일의 임의 실행을 권장하는 목록도 아니다.

## 22. 삭제한 항목과 보존한 항목

사용자 확인 후 삭제:

- `src/soarm4_live/`: 예전 별도 실시간 표시 패키지. 현재 servo_bridge와 동시에 같은 모터 포트를 열 수 있었다.
- 그 패키지의 `build/soarm4_live`, `install/soarm4_live` 잔여물.
- `src/soarm_bridge/test/execution_safety_test.py`: 현재 API·영점과 맞지 않는 구형 통합 모의 시험.
- 재생성 가능한 Python·pytest 캐시 일부.

보존:

- GPU SAM2 코드: 다른 GPU PC에서 활용할 수 있으므로 사용자 선택에 따라 유지.
- 모터 영점·부호·리미트·J1 보정·J3 복원 기록.
- 과거 시험 데이터·스캔·카메라 보정 자료.
- 사용 여부가 명확하지 않은 실험 도구.

삭제 파일은 `/home/seungung9473/robot_ws_cleanup_reports/` 아래 별도 백업에 보존했다. 최신 정리 후 6개 패키지 빌드와 101개 단위 테스트를 통과했다. 이 테스트 수는 실물 자동 분류 101회 성공을 뜻하지 않는다.

## 23. 현재 알고 있는 한계와 실제 검증 상태

- 실물 홈 이동, 12방향 정지 촬영과 0° 복귀, Gemini 인식, CPU SAM2 분리, 3개 물체 형상 복원·날짜 판독을 확인했다.
- 최근 자동 분류는 패드 배치 후보를 확보하지 못해 집기 전에 중단된 이력이 있다. 전체 실물 분류 성공으로 기록하지 않는다.
- 형상 복원은 보이는 표면의 점군이며 오목한 면이나 가려진 부분의 완전 복원이 아니다.
- 평면 검출과 실제 흡착 성공은 다르다. 압력 신호 확인이 필요하다.
- 충돌 모델·실물 배치·카메라/로봇 좌표가 어긋나면 계획이 통과해도 실제 충돌할 수 있다.
- CPU SAM2와 모든 기하·계획 단계의 시간을 합친 값이 실제 작업 시간이다. API 시간이나 GPU 한 단계 시간만으로 전체 속도를 설명하면 안 된다.
- 모터 설정값, MoveIt 비율, 모터 내부 PID는 서로 다른 제어 계층이다. 하나를 바꿨다고 다른 값이 자동으로 같은 값이 되지는 않는다.
- 이 문서는 현재 코드를 설명한다. 새로운 파일 변경·장치 교체·모델/보정 변경 시 다시 확인해야 한다.
