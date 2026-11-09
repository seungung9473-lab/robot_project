# 코드 정리 결과 — 2026-11-09

현재 mybranch 작업 파일 기준. 친구 TUMST_test 수정본은 적용하지 않았다.

## 사용자 확인 후 제거한 파일

- `finish_two_object_sort.py`
- `process_current_scan.py`
- `process_same_color_pair.py`
- `prepare_stationary_grip.py`
- `controller_homing_candidate.py`

다섯 파일은 automation/reconstruction 아래에 있었다. 현재 추적된 실행 소스에서 파일명·모듈명 참조가 없는 것을 확인했다. 참조가 없다는 사실만으로 모든 수동 도구가 불필요하다고 판단하지 않고, 이번 5개는 사용자 승인 후 제거했다.

## 백업

- 백업: `/home/seungung9473/robot_ws_cleanup_reports/20261109_124844/removed_sources.tar.gz`
- 목록·SHA256: `/home/seungung9473/robot_ws_cleanup_reports/20261109_124844/manifest.json`
- 삭제 전 백업 내용의 SHA256 일치를 확인했다.

## 유지한 항목

- 현재 자동 분류 진입점, 형상 복원·IK·충돌·흡착 제어·기록 코드
- 수동 카메라 보정, 깊이 검증, 흡착 진단, 엔코더 오차 측정 도구
- 사용자가 보존을 요청한 GPU SAM2 코드
- 과거 시험·원본 스캔·보정값·백업, Python 환경과 가중치
- build/install/log: 현재 실행과 진단에 필요할 수 있어 유지

## 검증 범위

실행 소스의 문자열 참조 검사 및 백업 무결성 확인. 로봇·펌프·카메라 시험, 빌드, Git 커밋·업로드는 수행하지 않았다. 기존 미커밋 변경도 유지했다.

## 추가: 파일 내부 미사용 코드 정리

추적된 Python 파일 중 존재하는 305개를 pyflakes로 검사했다. 이 수에는 보존된 외부 라이브러리도 포함된다. 실제 수정은 프로젝트 소유 31개 파일의 미사용 import 37개다. 외부 라이브러리·vendor·과거 backups는 수정하지 않았다.

전체 파일·함수의 미참조만으로 삭제하지 않았다. 수동 실행 도구, ROS/CMake 등록, 외부 호출·공개 함수, import 부작용은 별도 고려가 필요하다. 이번 검사는 모든 코드가 불필요한 부분 없이 완전하다는 증명이 아니다.

### 검증

- 삭제 전 31개 원본을 백업했다.
- import 문을 제외한 변경 전후 AST가 모두 동일함을 확인했다. 계산·제어 로직 변경 없음.
- ROS 환경을 불러온 뒤 reconstruction 테스트, torque preload, workcell core/experiment, turntable 테스트: 82 passed.
- 첫 테스트 시도는 ROS 환경이 없어 모듈 import에서 중단했고, 환경을 로드한 뒤 재검사했다.
- 물리 장치 구동 없음. 빌드·Git 업로드 없음. 친구 브랜치 미적용.

### 추가 백업

/home/seungung9473/robot_ws_cleanup_reports/imports_20261109_125537/before_import_cleanup.tar.gz

### 수정 파일과 제거한 import

- `automation/camera_calibration/fit_active_axis.py`: `datetime`
- `automation/camera_calibration/measure_fixed_markers.py`: `os`
- `automation/camera_calibration/measure_pad5.py`: `os`
- `automation/camera_calibration/measure_pad6.py`: `os`
- `automation/camera_calibration/measure_turntable_marker.py`: `os`
- `automation/er2/select_object.py`: `numpy as np`
- `automation/er2/verify_flash_fallback.py`: `time`
- `automation/reconstruction/apply_sam2.py`: `time`
- `automation/reconstruction/audit_last_scan.py`: `sys`
- `automation/reconstruction/execute_contact.py`: `moveit_msgs.msg.DisplayTrajectory`
- `automation/reconstruction/execute_pick_batch.py`: `yaml`
- `automation/reconstruction/j3_integral_tune.py`: `sys`, `pathlib.Path`
- `automation/reconstruction/motion/approach.py`: `moveit_msgs.msg.DisplayTrajectory`
- `automation/reconstruction/motion/kinematics.py`: `scipy.optimize.least_squares`
- `automation/reconstruction/motion/lift.py`: `moveit_msgs.msg.DisplayTrajectory`
- `automation/reconstruction/motion/return.py`: `moveit_msgs.msg.DisplayTrajectory`
- `automation/reconstruction/read_numeric_dates.py`: `numpy as np`
- `automation/reconstruction/run_tcp_50.py`: `sys`
- `automation/reconstruction/sam2_gpu.py`: `hashlib`, `contextlib`
- `automation/reconstruction/scan_scene_reset.py`: `json`
- `automation/reconstruction/tests/test_expiry_sort.py`: `time`
- `src/soarm4_moveit_config/audit_tools/check_collisions.py`: `os`, `math`
- `src/soarm4_moveit_config/audit_tools/run_scan.py`: `sys`, `hashlib`
- `src/soarm4_moveit_config/scripts/plan_joint_goal.py`: `time`
- `src/soarm4_workcell/soarm4_workcell/pickplace_cycle.py`: `yaml`
- `src/soarm4_workcell/soarm4_workcell/random_tcp_plan.py`: `json`
- `src/soarm4_workcell/soarm4_workcell/reconstructed_models.py`: `moveit_msgs.msg.PlanningScene`
- `src/soarm4_workcell/soarm4_workcell/scene.py`: `json`, `pathlib.Path`
- `src/soarm4_workcell/test/test_experiment.py`: `json`
- `src/soarm_turntable/soarm_turntable/cli.py`: `json`, `time`
- `src/soarm_turntable/soarm_turntable/scan.py`: `tempfile`

### 판단이 필요한 항목은 유지

- 수동 보정·복구·진단·오차 측정 실행 파일: 다른 파일에서 호출되지 않아도 독립 실행할 수 있다.
- validate_runtime.py의 subs: 정적 분석에서 미사용이어도 ROS 구독 객체를 유지한다.
- 부작용이 있는 함수 호출의 미사용 반환값: 결과 변수만 미사용이라고 함수 호출까지 지우면 안 된다.
- 프로젝트 내부에서 가져온 이름: 다른 모듈에서 재노출해 사용할 수 있어 자동 제거에서 제외했다.
- C++/CMake/launch 파일과 외부 라이브러리: Python 정적 검사만으로 불필요함을 확정할 수 없다.
