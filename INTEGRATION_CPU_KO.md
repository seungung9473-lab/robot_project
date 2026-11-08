# 친구 수정본 CPU 통합 결과

원본: origin/TUMST_test a475c74. 현재 main 작업 폴더에 파일 수준 통합했고 Git commit/push는 하지 않음.

- SAM2 기본 CPU/FP32, 기존 .sam2_cpu 사용. 자연어 실행도 동일.
- RViz 초기 비율 velocity=0.30, acceleration=0.40 유지.
- 친구 PC 실행 경로를 현재 노트북 경로로 수정. 기존 named_poses 유지.
- J1 보정, servo_bridge.py, fine_limits.py, soarm.yaml, J3 복원 기록 파일의 SHA256이 통합 전과 동일.
- 기존 데이터·테스트·설명 문서는 삭제하지 않음.
- 백업: /home/seungung9473/robot_ws_merge_backups/20261108_203311/before.tar.gz

## 검증

4개 ROS 패키지 빌드 성공. 단위 테스트 95개 통과.
CPU SAM2 저장 영상 1장/물체 1개 프롬프트 실행 성공: 로딩·출력 포함 약 3.42초. 전체 스캔 처리 시간은 아님.
Pilz PTP/LIN/CIRC 등록과 MoveIt planning ready 확인. 별도 ROS domain202, 장치 구동 없음. Octomap 센서 미설정 경고 존재. 종료 시 MoveIt 종료 오류 발생; 시작 검증과 구분.

기존 execution_safety_test.py 모의 버스 시험은 최초 hold 목표 읽기 검증 기대에서 실패. domain201에서 재현. 해당 servo_bridge.py는 통합 전 SHA256과 동일하여 이번 통합의 변경은 아님. 통과한 것으로 처리하지 않음. 이 테스트의 기대 조건과 기존 토크 ON 로직은 별도 검토 필요.

기록:
- test_data/runs/20261108_203435_b87cf576 : 보호 파일·구문 검사
- test_data/runs/20261108_203453_d5e0a3da : CPU SAM2
- test_data/runs/20261108_203523_a1d89364 : 95개 테스트
- test_data/runs/20261108_203541_a346db85 : 기존 모의 시험 실패
- test_data/runs/20261108_203609_72814c98 : Pilz 시작 확인

## 실행

기존에 실행 중인 RViz/MoveIt 프로세스는 새 코드를 다시 시작해야 반영됨. 중복 bringup은 실행하지 말 것.

안내만 보기 (모터/API 실행 없음):
```bash
cd ~/robot_ws
./run_tidy_demo.sh
./run_sort.sh
```

실제 자동 분류는 기존 bringup과 작업대 장면·카메라가 준비된 상태에서 run_sort.sh --execute 또는 run_tidy_demo.sh '테이블 정리해' --execute 중 하나 사용. 이번 통합에서 실물 이동·펌프·촬영·API 호출 시험은 수행하지 않음.
