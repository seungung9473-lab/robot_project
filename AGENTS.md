# SO-ARM 자동화와 시험 기록

- 현재 전체 실행: `ros2 run soarm_pipeline expiry_sort --execute` 또는 `~/robot_ws/run_sort.sh --execute`.
- 실제 자동화 코드는 `automation/reconstruction`, Gemini 코드는 `automation/er2`입니다.
- 사용자 요청: 앞으로 하는 로봇·카메라·흡착·픽앤플레이스 시험은 모두 기록합니다. 기존 자동 진입점은 기록기를 사용합니다. 별도 시험은 `automation/reconstruction/test_data_recorder.py`의 `start_trial`로 등록하세요.
- 기록은 `test_data/index.csv` 및 `test_data/runs/`에 저장합니다. 실제 실행·계획 검증과 성공·중단을 구분하고, 과거에 없는 센서값을 만들어 넣지 마세요.
- API 키를 코드·Git·시험 데이터에 넣지 마세요. 기존 개인 설정을 사용합니다.
- 통합만 요청한 작업에서 모터·펌프를 시험 실행하지 마세요.
- 보정·관절 한계·압력 기준은 통합 때문에 바꾸지 않습니다. 기존 호환 링크는 과거 자료의 경로를 보존합니다.

## 이 노트북 작업 규칙

- 답변은 한국어로 한다. 작업·커밋·push는 TUMST_test에서 한다. main은 변경하지 않는다. 별도 요청 전에는 merge하지 않는다.
- 모터·펌프·토크 ON·HOME·회전 스캔·--execute 실행 전에는 사용자에게 확인받는다.
- 실험 코드는 검토 후 함께 공유하되 원본 데이터·모델·가상환경·키는 기본 업로드에서 제외한다.
