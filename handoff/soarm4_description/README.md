# soarm4_description v2 (실제 메시 + 전장 박스)

v1은 박스/원통으로 대충 그린 거라 실물이랑 안 맞았음. v2는 **SO-101 공식 CAD 좌표 그대로** + 우리 진공툴 v5 + 전장 박스(상판/앞뒤벽/기둥4/하판)로 다시 만듦.
관절 각도 규칙은 v1이랑 똑같아서 `soarm_bridge` / `servo_map.yaml`은 안 바꿔도 됨.

## 좌표계
| 프레임 | 위치 | 방향 |
|---|---|---|
| `base_link` | 테이블 면, J1 축 바로 아래 | x=로봇 정면(박스 옆 열린 면 쪽), y=왼쪽, z=위 |
| `box_top` | 박스 상판 윗면 (z 95 mm) | base_link와 같음 |
| `box_cad` | 전장 박스 CAD 원점 (상판 윗면 중앙) | CAD 축 = base 축 (라즈베리파이 붙은 앞벽 = 로봇 오른쪽) |
| `link1` | J1 출력 (z 159.8) | 0자세에서 전부 base_link와 평행 |
| `link2` | J2 축 (앞 30.4, 높이 214.0) | |
| `link3` | J3 축 (link2에서 앞 112.6, 아래 28.0) | |
| `tool_link` | J4 축 (link3에서 앞 134.9, 위 5.2) | |
| `cup_tip` | 컵 입구 중심 (tool_link에서 앞 104) | x = 흡착 방향 |

## 관절 규칙 (v1과 동일)
0 rad = 팔을 앞으로 수평으로 쭉 편 자세 (윗팔·아랫팔·툴 부품이 전부 수평).
joint1 + = 위에서 봤을 때 반시계, joint2/3/4 + = 링크가 위로 들림.

| 관절 | 서보 count → 각도 | 확인 포인트 |
|---|---|---|
| joint1 | 2047 = 정면 | |
| joint2 | 4095 = 수평 앞, 3071 = 수직 | 윗팔 **부품**이 수직 (J3 축은 28 mm 앞으로 튀어나와 있는 게 정상) |
| joint3 | 0 = 일자, 2047 = 접힘 | |
| joint4 | 2047 = 툴이 아랫팔과 일직선 (**가정값**) | |

## 실행
```bash
cd ~/ros2_ws && colcon build --packages-select soarm4_description && source install/setup.bash
# 로봇 없이 슬라이더로 보기
ros2 launch soarm4_description display.launch.py
# 실제 로봇 따라 움직이기 (bringup이 robot_state_publisher도 띄움)
ros2 launch soarm_bridge bringup.launch.py
rviz2 -d $(ros2 pkg prefix soarm4_description)/share/soarm4_description/rviz/view.rviz
```

## 충돌검사 / MoveIt 쓸 때
- 충돌 형상: 링크별 convex hull (`meshes/collision`), 박스는 직육면체. 화면용은 `meshes/visual`.
- 붙어있는 링크끼리(base-link1, link1-link2, link2-link3, link3-tool_link)는 항상 닿아 있으니 SRDF에서 disable.
- 테이블 면 = base_link z=0. 턴테이블/분류통은 planning scene에 박스로 추가.
- 4축이라 6D 자세 IK는 안 풀림 → position-only IK 또는 해석적 IK (컵 수직 하강: joint2+joint3+joint4 = -90°).

## 실물이랑 3개만 확인해줘
1. **J1 축 중심 ↔ 박스 정면 쪽 모서리 = 82 mm, ↔ 파이 붙은 벽 바깥면 = 112.5 mm** (사진 보고 베이스가 상판 구멍 4개 위 가운데 있다고 가정). 다르면 `make_urdf_v2.py`의 `J1_ON_PLATE` 수정.
2. **테이블 ↔ J2 축 높이 = 214 mm**. 다르면 `BOX_H` 수정 (박스 밑에 고무발 있으면 그만큼 더하기).
3. **joint4 = 0 보냈을 때 툴이 아랫팔이랑 일직선**인지. 아니면 그때 J4 count 알려주면 `servo_map.yaml` center 수정.

## 검증
- SO-101 공식 URDF(so101_new_calib)의 FK와 랜덤 200 자세 비교: 오차 최대 0.004 mm
- 관성: 서보 55~60 g + 출력물(체적×밀도) 추정값 → 정확하지 않음
