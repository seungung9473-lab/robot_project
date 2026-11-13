# SO-ARM4 MoveIt 2 경로 계획

Ubuntu 24.04 / ROS 2 Jazzy / MoveIt 2 2.12.4. 실제 모터를 제어하지 않는 계획·미리보기 패키지입니다.

## 실행

기존 관절 슬라이더/충돌 표시 창은 종료해도 됩니다. 이 패키지는 `/soarm4_plan`의 별도 관절 상태와 TF를 사용합니다.

```bash
bash "/home/seungung9473/문서/ChatGPT/라파 대회/soarm4_moveit_config/run_planning.sh"
```

RViz에서 기존 엔코더 2047 홈자세가 표시됩니다. 자동 데모는 기본으로 끕니다. MotionPlanning → Planning에서 Planning Group=arm, Start State=<current>로 두고 Goal State에서 home/left/right/reach/ready를 선택한 뒤 Plan을 누르세요. Joints 탭에서는 Goal State 관절 각도를 직접 조절할 수 있습니다. 계획이 성공하면 애니메이션으로 표시됩니다. Execute/Plan & Execute는 실물 제어에 사용하지 않습니다. 반투명 로봇은 경로 미리보기입니다. 실제 현재 상태는 고정된 홈자세로 남습니다. 모터는 움직이지 않습니다.

명령줄을 선호할 때만 다른 목표를 별도 터미널에서 다음처럼 지정할 수도 있습니다. 순서는 J1/J2/J3/J4, 단위는 URDF 관절각 도입니다. 엔코더값이나 모터 명령각이 아닙니다.

```bash
bash "/home/seungung9473/문서/ChatGPT/라파 대회/soarm4_moveit_config/plan_goal.sh" --goal-deg -30 150 -130 0
```

`Validated ... samples`가 표시되면 최종 검사 통과입니다. 결과는 `/tmp/soarm4_verified_plan.json`에 저장됩니다. `PLAN REJECTED`이면 현재 목표/경로를 사용하지 않습니다. 실패할 때 이전 결과 파일은 남아 있을 수 있으므로 이번 명령의 종료 코드와 로그를 기준으로 판단하세요.

## 구성과 시작 자세

- 실제 형상으로 보정한 네 모터 메시와 전장 상자를 포함한 현재 canonical URDF를 사용합니다. URDF/메시 해시가 달라지면 재검증 없이 실행되지 않습니다.
- 사용자 요청으로 J3 하한만 기존 홈(-179.912109375도, 엔코더2047)까지 변경했습니다. 반대쪽 J3 상한 및 다른 관절 리미트·방향·영점·베이스 위치는 유지합니다.
- MoveIt 시작은 `[0,180,-179.912109375,0]`도입니다. J3의 이전 홈 쪽 약10도 여유를 사용자 요청으로 없앴습니다. home이라는 이름의 목표도 추가했습니다. ready는 별도의 기존 준비 자세로 남습니다.
- 4자유도 팔이므로 우선 관절 목표를 사용합니다. 임의 6D 자세의 IK/드래그 목표는 설정하지 않았습니다. `No kinematics plugins defined` 경고는 이 구성에서 예상되는 메시지입니다.
- OMPL RRTConnect로 경로를 생성하고, 시간 보간 후 MoveIt 검증 및 최대 관절 변화 0.5도 간격의 추가 검사를 수행합니다. 시간 보간의 경로 편차 허용치는 0.001rad입니다.
- RViz 경로에도 z=0 아래로의 로봇 메시 통과를 세부 검사합니다. 명령줄 예제는 추가로 z=-1mm를 상면으로 하는 가상 테이블 상자를 장면에 등록합니다.

## 결합부 처리와 검증 범위

SRDF는 서로 고정된 부품 쌍과 네 모터/브래킷 결합 쌍만 제외합니다. 인접한 팔 전체의 충돌을 일괄 제외하지 않습니다. MoveIt의 결합 쌍 제외는 위치 조건이 없으므로, FineCollisionAdapter와 fine_collision_service.py가 모든 계획 응답에 기존 원통형 결합 영역을 적용한 FCL 세부 검사를 다시 수행합니다. 이 검사는 RViz Plan 버튼에도 적용됩니다. 영역 밖 접촉, 판정 한계 초과, 바닥 관통, 관절 한계 위반은 거부합니다. 환경 장애물도 MoveIt 상태 검사로 재확인합니다. 검사 서비스가 없거나 검사가 실패하면 RViz와 명령줄 모두 계획을 거부합니다. 통과한 경우에만 경로 표시를 발행합니다.

이 구성은 샘플링에 기반합니다. 연속 경로의 실물 안전 간극을 보증하지 않으며 케이블·호스·모델에 없는 주변 물체는 포함하지 않습니다. 새로운 환경 물체는 Planning Scene에 추가해야 합니다. 현재 카메라/센서 기반 장애물 갱신은 없으며 Octomap 센서 미설정 메시지는 예상됩니다.

하드웨어 드라이버, FollowJointTrajectory 제어기, ExecuteTrajectory 기능은 연결하지 않습니다. 별도 PreviewOnlyControllerManager는 제어기를 하나도 제공하지 않습니다. MoveIt 계획 결과를 실제 모터로 보내려면 실측 상태/엔코더 변환/제어기/실행 중 정지 기능을 별도 구현하고 검증해야 합니다.

## 확인 결과

- 리미트 안의 준비 자세에서 기본 관절 목표 계획 성공.
- 상판/로봇과 충돌하는 `[0,50,-169,0]`도 목표 거부.
- 기본 경로의 중간 지점에 25mm 가상 장애물을 두었을 때, 직선 중간 자세는 충돌하지만 OMPL 우회 경로가 생성됨. 시간 보간 후 420개 자세를 FCL 및 MoveIt 장면 검사로 확인.
- 실제 RViz 창에서 로봇·전장 상자·경로 애니메이션 표시 확인: moveit_preview.png.
- 결과는 validation_plan_*.json에 기록. 시험 장애물은 검사 뒤 제거했습니다.

## 설치/재빌드

```bash
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install --packages-select soarm4_moveit_config
source install/setup.bash
```

FCL/trimesh 런타임은 이 PC의 Python 3.12/x86_64용 파일을 runtime_deps에 포함했습니다. 다른 플랫폼은 별도 설치가 필요합니다. 가속도 제한 1rad/s²와 속도/가속도 배율 10%는 미리보기용 설정이며 실물 성능을 측정한 값이 아닙니다.

참고: [MoveIt URDF/SRDF](https://moveit.picknik.ai/main/doc/examples/urdf_srdf/urdf_srdf_tutorial.html), [Jazzy 기본 OMPL 어댑터](https://github.com/moveit/moveit2/blob/jazzy/moveit_configs_utils/default_configs/ompl_planning.yaml).

## RViz 조작 순서
1. MotionPlanning의 Planning 탭에서 Group을 arm, Start State를 <current>로 둡니다.
2. Goal State에서 home/home/left/right/reach/ready 중 하나를 고릅니다.
3. 임의 관절 목표는 Joints 탭의 Goal State를 선택한 뒤 joint1~joint4 값을 조절합니다. 현재 화면은 도(°) 단위로 표시됩니다. 파란 막대를 드래그하거나 값을 편집하면 됩니다.
4. Planning 탭으로 돌아와 Plan을 누릅니다. 성공하면 경로가 재생됩니다.
5. 실패하면 목표를 조절하고 Plan을 다시 누릅니다. 물리 로봇 제어는 아직 연결하지 않습니다.

자동 데모가 필요할 때만 run_planning.sh demo_plan:=true를 사용하세요.


0.5.0: 실제 제어 RViz에 HOME/STOP 패널과 `ros2 run soarm4_moveit_config go_home.py` 명령을 추가했습니다. 5° 초기 실행 제한은 제거했고 URDF 관절 리미트를 적용합니다. 사용법은 HARDWARE_KO.md를 참조하세요.
