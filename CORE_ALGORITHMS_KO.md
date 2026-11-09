# SO-ARM4 핵심 알고리즘 설명서

작성일: 2026-11-09 · 대상: /home/seungung9473/robot_ws
기준: mybranch의 「필요없는 파일 제거」 정리 스냅샷(기반 c50090d). 친구 TUMST_test cc16f9c는 미적용.

이 문서는 현재 로컬 코드의 핵심 기능을 추출한 설명이다. 문서 작성 중 장치를 구동하거나 실제 동작을 검증하지 않았다. 아래 경로는 robot_ws 기준이다. 상세 설명은 CODE_ALGORITHMS_GUIDE_KO.md를 함께 참고한다.

## 1. 전체 처리 흐름

실행 잠금·기존 스캔 장면 정리 → 홈 → 30° 간격 정지 촬영 12방향 → 0° 복귀 → Gemini 물체 식별 → SAM2 물체 분리 → RGB-D 점군 복원 → 날짜 판독·분류 → 흡착면 탐색 → 턴테이블 정렬·IK → 모든 물체의 놓기·충돌 경로 검토 → 실제 집기·압력 확인·이동·해제 → 결과 기록.

주 진입점: run_sort.sh → automation/reconstruction/run_qr_expiry_cycle.py.
자연어 진입점: run_tidy_demo.sh. ROS 진입 패키지: src/soarm_pipeline.

## 2. AI와 로컬 계산의 역할 분담

| 처리 | 핵심 방법 | 담당 |
|---|---|---|
| 물체 이름·색상·대략적 영상 위치 | 영상 의미 추론 | Gemini 원격 API |
| 인쇄 날짜 | 영상 글자 판독 및 구조화 응답 | Gemini + 로컬 검증 |
| 물체 픽셀 경계 | 프롬프트 기반 분할 | 로컬 SAM2 AI |
| QR 문자열 | ZXing 디코딩 | 로컬 |
| 날짜 차이·목적지 | 달력 계산·정책 분기 | 로컬 |
| 3D 복원·평면·좌표·IK | 기하·수치 최적화 | 로컬 |
| 충돌·경로 | MoveIt/Pilz/OMPL | 로컬 |
| 압력·모터 피드백 | 상태 감시·제어 | 로컬 브리지 |

AI가 모터 엔코더 명령을 직접 생성하는 시스템은 아니다. 의미 인식 결과를 수치 계산과 검증 절차에 연결한 VLM 기반 로봇 자동화 구조다.

## 3. 정지 확인 후 촬영

코드: src/soarm_turntable/soarm_turntable/scan.py, scan_core.py, bridge.py, controller.py.

회전 액션 완료 후 실제 moving·속도·위치 변화·상태 수신 시간을 확인한다. 정지 확인 후 0.5초 더 기다리고, 그 이전 프레임을 제외한 RGB·정렬 깊이·CameraInfo를 저장한다. 촬영 중 위치 변화와 로봇팔 움직임도 확인한다.

각도는 저장 소프트웨어 영점과 실제 엔코더 차이로 계산한다.

    angle_deg = (position_count - zero_count) × 360 / 4096

Homing Offset 레지스터와 소프트웨어 영점은 별개다. 모든 모터의 홈을 2047로 통일하지 않는다. RGB/깊이 시간차와 해상도 일치도 검사한다.

## 4. 물체 대응과 SAM2 분할

코드: automation/er2/select_object.py, reconstruction/process_three_color_objects.py, apply_sam2.py, sam2_shared.py.
이 절 이후 reconstruction/ 경로는 automation/reconstruction/을 뜻한다.

Gemini가 제시한 영상 위치를 유효 깊이와 대응해 3D 씨앗점을 얻는다. kmeans2 군집화로 물체 중심을 연결하고, 각 물체의 긍정 점·다른 물체의 부정 점·상자를 SAM2 프롬프트로 만든다. 이름과 달리 process_three_color_objects.py는 1~8개 물체 조건을 처리한다.

SAM2.1 Hiera Tiny는 픽셀별 물체 마스크를 만든다. 현재 기본은 CPU이며 같은 영상의 특징을 여러 물체에 재사용해 중복 인코딩을 줄인다. SAM2 자체는 실제 길이·로봇 관절값을 계산하지 않는다.

## 5. RGB-D를 3차원으로 바꾸기

코드: reconstruction/reconstruct.py, placement_geometry.py, shared_observations.py.

카메라 내부 파라미터와 왜곡 보정 광선에 깊이를 곱해 카메라 좌표를 얻고, 보정 행렬로 테이블 좌표로 변환한다. 왜곡을 생략한 기본 관계는 다음과 같다.

    X = (u-cx) × Z/fx
    Y = (v-cy) × Z/fy
    p_table = T_table_camera × p_camera

여러 회전 각도에서 본 점을 공통 기준 자세로 되돌린다.

    p_reference = axis + Rz(-angle) × (p_table-axis)

물체가 턴테이블 중심에 있다고 가정하지 않는다. 회전축·카메라 보정·실제 각도가 정확해야 중심에서 벗어난 물체도 올바르게 합쳐진다.

점군 정리의 주요 기본값은 복셀 2mm 평균, 이웃 평균 거리 10mm 이상 고립점 제거, 8mm 연결 그래프에서 가장 큰 성분 유지다. shared_observations.py는 같은 프레임의 좌표 변환 결과를 물체별로 공유한다.

결과는 색상 점군이다. 가려진 면까지 완전히 복원한 CAD 메시라는 뜻은 아니다.

## 6. 흡착면 찾기 — RANSAC·SVD·원판 지지 검사

코드: reconstruction/inclined_geometry.py, plan_inclined_pick.py.

1. 무작위 3점으로 평면을 만들고 가까운 점의 수를 센다(RANSAC, 한 평면당 최대 600회).
2. 지지점이 많은 평면을 SVD로 정밀화해 법선과 평면식을 구한다.
3. 평면 점을 제거하며 다른 면도 찾는다.
4. 면 내부의 후보점에서 흡착컵 전체가 지지되는지 검사한다.

평면식: n·p+d=0. n은 면에 수직인 단위 법선이며 컵 방향 결정에 사용한다.
주요 기본 조건: 컵 지름 25mm, 경계 여유 3mm, 평면 포함 거리 2.5mm, RMS 2.5mm 미만, 원판 지지율 90% 초과.

볼록 경계, 최대 내접원 중심의 선형계획, KD-tree 근접 점 탐색으로 작은 면이나 가장자리의 불충분한 지지를 거른다. 두 카메라의 법선 차이 5° 이하·거리 3mm 이하 등의 교차 검증도 사용한다.

이는 모든 점의 곡률을 계산해 평면을 확정하는 방식이 아니다. 고무 컵의 변형과 누설을 완전히 예측하지 못하므로 실제 성공은 압력으로 확인한다.

## 7. 정기구학·역기구학·턴테이블 각도 탐색

코드: reconstruction/motion/kinematics.py, inclined_geometry.py, sag_compensation.py.

FK는 관절 원점 이동·축 회전·툴 오프셋의 4×4 변환행렬을 순서대로 곱해 TCP 위치와 방향을 계산한다. 모델은 model.json과 workcell.yaml을 사용한다.

IK는 scipy.optimize.least_squares로 위치·방향 잔차를 최소화한다. 관절 상하한을 경계로 설정하고 여러 초기값에서 탐색한다. 4자유도이므로 임의의 6자유도 자세를 모두 만족할 수 없다.

턴테이블 정렬은 면 법선과 로봇의 도달 방향 관계를 검사하고 brentq 근 찾기로 가능한 회전 각도를 계산한다. 점과 법선을 함께 회전하므로 회전 뒤의 흡착점을 미리 예측한다. 해당 집기 경로는 회전 후 영상을 통한 접촉점 재계산을 하지 않는 모드를 사용한다.

처짐 보정은 경험적으로 얻은 방향 보정이다. 강체·하중 전체를 푸는 물리 모델이 아니다. FK로 계산한 엔코더 위치 오차 역시 실측 TCP 오차와 구분해야 한다.

## 8. 패드 배치와 여러 물체의 순서

코드: reconstruction/pad_occupancy.py, pad_packing.py, tidy_pad_placement.py, payload_placement.py, batch_search.py, plan_pick_batch.py.

물체의 2D 외곽과 기존 점유를 계산하고, 패드 가장자리·물체 간 간격을 만족하는 위치를 격자로 탐색한다. 현재 기본은 2mm 격자, 가장자리 여유 2mm, 물체 사이 여유 2mm다. 잡은 물체와 TCP의 상대 자세를 반영해 놓기 IK도 계산한다.

제한된 백트래킹으로 물체 순서·흡착 후보·회전 각도·놓을 위치를 바꾼다. 앞 물체를 놓은 가상 장애물까지 반영해 전체 작업을 먼저 검토한다. 제한 시간 안에 후보를 못 찾은 것은 절대 불가능하다는 증명이 아니다.

## 9. 충돌 검사와 경로 계획

코드: reconstruction/collision_geometry.py, check_inclined_motion.py, src/soarm4_moveit_config.

대상 점군의 ConvexHull을 삼각형 충돌 메시로 만든다. 볼록 껍질은 오목한 홈을 채우므로 실제 표면과 차이가 있다. RViz 색상 점군 표시와 충돌 형상은 별개다. 일부 주변 형상은 상자로 표현한다.

홈 복귀는 Pilz PTP를 우선하고 실패하면 OMPL RRT-Connect를 시도한다. 상세 충돌 어댑터로 궤적을 추가 검토하며, 들고 있는 물체는 부착물로 처리한다.

계획 입력은 batch_inputs.py의 SHA256 지문으로 확인한다. 장면·설정·시작 자세 등이 달라진 계획을 무조건 재사용하지 않는다. 등록되지 않은 실제 장애물이나 잘못된 모델까지 검사가 보장하지는 않는다.

## 10. 부드러운 접촉과 흡착 상태 제어

코드: reconstruction/normal_contact_trajectory.py, continuous_pre_suction.py, grip_retry.py, verified_place_left.py, execute_pick_batch.py.

접근점에서 접촉점까지 TCP를 보간하고 각 점에서 IK를 푼다. 기본 81개 샘플과 5차 곡선을 사용한다.

    s(u)=10u³-15u⁴+6u⁵

시작·끝의 속도와 가속도를 부드럽게 하고, 관절 제한에 맞게 시간을 늘린다. 실제 엔코더 FK에서 계산한 목표면 거리가 1.5mm 이내이면 이동 중 펌프를 한 번 켠다. 별도 거리 센서로 잰 간격은 아니다.

현재 로컬 경로는 diff=-200으로 흡착·유지를 확인하고, 실패 시 초기 접촉점 기준 누적 1·2·3mm 추가 접근을 준비한다. 압력 피드백 지연이나 유지 실패 시 중단하는 로직이 있다. 펌프 ON은 흡착 성공과 다르다.

진공 장치 코드: src/soarm_bridge/soarm_bridge/esp32_bridge.py, vacuum.py.
/vacuum/release는 펌프 OFF→밸브 개방→밸브 닫기다. /vacuum/off는 펌프와 밸브 OFF이므로 같은 명령이 아니다. diff 수치는 검증 없이 Pa나 kPa로 해석하지 않는다.

## 11. 날짜·QR 기반 분류

코드: reconstruction/date_crops.py, read_numeric_dates.py, read_expiry_labels.py, expiry_policy.py, run_expiry_sort.py.

물체 마스크에서 날짜 영상을 잘라 Gemini로 먼저 판독한다. 판독이 불충분하면 로컬 ZXing QR을 사용한다. QR 뒤 6자리를 YYMMDD로 해석하고, QR 영역과 물체 마스크 겹침으로 귀속을 검사한다.

달력 유효성·관측 간 합의·한국 기준 날짜 비교는 로컬 계산이다. 지난 날짜는 쓰레기통, 유효한 두 날짜 중 가까운 날짜는 ID5, 먼 날짜는 ID6으로 분류한다. 날짜 당일까지는 유효하다. 연도 누락 등은 정책과 근거 검증을 거쳐야 한다.

모델 대체는 automation/er2/model_router.py와 model_policy.json에서 관리한다. 주 모델과 대체 모델은 설정값이며 인증·결제·할당량 오류까지 다른 모델로 자동 해결하는 구조는 아니다.

## 12. 모터 변환·피드백 미세 보정

코드: src/soarm_bridge/soarm_bridge/servo_bridge.py, config/soarm.yaml.

관절 rad를 영점·방향·보정이 포함된 엔코더 목표로 변환한다. 목표 궤적과 실제 엔코더를 비교해 시작 오차·추종 오차·도착 오차·피드백 유효 시간을 검사한다.

도착 시 제한된 반복 미세 보정이 있으며 현재 관절별 보정 상한은 J1 8count, 나머지 4count다. 이는 모터 내부 PID 변경과 별개다. J1 보정과 기존 모터 설정은 이번 파일 정리로 바꾸지 않았다.

관절 오차를 FK에 대입해 목표 대비 모델상 TCP 차이를 계산할 수 있으나, 링크 휨·백래시·툴 오프셋 오차까지 직접 측정한 값은 아니다.

## 13. 중복 계산 감소와 시험 기록

- sam2_shared.py: 같은 영상의 특징을 물체별로 공유.
- shared_observations.py: 깊이→테이블 변환 재사용.
- planning_feedback.py, trajectory_cache.py: 입력 조건이 맞는 계산 결과를 재사용.
- read_numeric_dates.py: 영상 내용과 검증 조건이 맞는 날짜 응답 재사용.
- scan_scene_reset.py: 새 전체 실행에서 이전 스캔 표시·임시 충돌 형상을 정리. 실제 물체나 과거 원본 자료를 지우는 기능은 아님.
- test_data_recorder.py: 시작·종료·오류·센서·원본 경로 기록. 실제 실행과 계획 검토를 구분.

## 14. 정리된 작업 폴더와 친구 수정본의 구분

현재 robot_ws에서 사용자 확인 후 제거한 파일은 다음 5개다.

- finish_two_object_sort.py
- process_current_scan.py
- process_same_color_pair.py
- prepare_stationary_grip.py
- controller_homing_candidate.py

정리 내역과 복구 백업은 CODE_CLEANUP_20261109_KO.md에 기록했다. 현재 실행 진입점·모터 설정·보정·과거 시험 자료는 유지했다.

친구 cc16f9c의 CUDA 필수화, 기본 속도 0.80, 5mm 전 미리 흡착, 5초 대기 후 5mm 재시도, 깊이 마스크 3px 침식, 10cm 후퇴 등은 이번 로컬 코드에 적용하지 않았다. 위 알고리즘 설명과 이 미적용 변경을 혼동하지 않는다.

## 15. 알고리즘별 찾아보기

| 알고리즘 | 입력 → 출력 | 핵심 코드 |
|---|---|---|
| 카메라 외부 보정·좌표 변환 | 마커 실측·영상 → 카메라/테이블 변환 | automation/camera_calibration/, src/soarm4_workcell/config/workcell.yaml |
| 영상 의미 인식 | RGB·사용자 조건 → 물체 이름·영상 위치 | automation/er2/select_object.py |
| 프롬프트 분할 | 영상·점·상자 → 물체 마스크 | reconstruction/apply_sam2.py, sam2_shared.py |
| 다각도 형상 복원 | 마스크·깊이·각도 → 기준 자세 색상 점군 | reconstruction/reconstruct.py |
| 평면 추정 | 점군 → 평면·법선·컵 지지 영역 | reconstruction/inclined_geometry.py |
| FK·IK | 관절 ↔ TCP 위치·축 방향 | reconstruction/motion/kinematics.py, inclined_geometry.py |
| 수평 목표 IK | 목표 XYZ → 아래 방향 관절 자세 | src/soarm4_workcell/soarm4_workcell/cup_goal.py |
| 턴테이블 정렬 | 흡착점·법선 → 회전 각도·회전 후 접촉점 | reconstruction/plan_inclined_pick.py |
| 평면 배치 탐색 | 물체 외곽·점유 → 패드 배치 예약 | reconstruction/tidy_pad_placement.py, pad_packing.py |
| 순서 탐색 | 전체 후보·장면 → 작업 순서·경로 | reconstruction/batch_search.py, plan_pick_batch.py |
| 충돌 경로 계획 | 관절 목표·충돌 모델 → 검증 궤적 | reconstruction/check_inclined_motion.py, src/soarm4_moveit_config/ |
| 접촉 보간·흡착 | 접근점·법선·피드백 → 연속 궤적·흡착 상태 | reconstruction/normal_contact_trajectory.py, verified_place_left.py |
| 날짜 분류 | 판독 날짜·한국 날짜 → 패드/쓰레기 목적지 | reconstruction/expiry_policy.py, run_expiry_sort.py |
| 엔코더 추종·보정 | 관절 궤적·실제 위치 → 모터 명령·오차 | src/soarm_bridge/soarm_bridge/servo_bridge.py |
| 실행 기록 | 실행·센서·오류 → 시험별 원본·결과 | reconstruction/test_data_recorder.py |

표의 reconstruction/은 automation/reconstruction/을 뜻한다. cup_goal.py의 solve()는 수평면용이며, 경사면 접촉은 법선 기반 solve_axis()와 사전 계산 관절값을 사용한다. 파일 공유가 곧 수평 자세 강제를 의미하지 않는다.
