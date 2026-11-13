# SO-ARM4 v5.7 — 체결 구멍 정렬

전장 상자는 고정하고 새 J1BASE가 포함된 로봇 베이스부터 팔 전체를 +Y로17.7mm 이동했습니다. 기존 X=-17.7mm, Z=0, yaw=+90°를 유지합니다.

robot_mount_fixed: xyz="-0.0177 0.0177 0", rpy="0 0 1.57079632679".

## 체결 검증

상판과 J1BASE STL을 단면으로 잘라 구멍4개의 중심을 추출했습니다. 상판 CAD 좌표(x,y)=(±35,10/45)mm와 베이스 CAD 좌표(x,z)=(±35,10/45)mm가 기존 조립 변환 때문에 Y로17.7mm 어긋나 있었습니다. 이번 이동 후4개 구멍의 X/Y 중심이 일치하며 STL 다각형 근사 오차는0.001mm 미만입니다.

validation_mount_holes.json에 좌표와 오차를 기록했고 mounting_holes_check.png에 위에서 본 정렬 도식을 넣었습니다. Z는 변경하지 않았습니다. 새 J1BASE의 CAD 바닥은 기존 CAD보다2.4mm 높게 잘려 있으며 체결부 등록 좌표를 유지했습니다.

## 적용·실행

기존 launch를 Ctrl+C로 종료한 뒤 이 폴더에서 실행하세요. 기존 패키지를 src 밖으로 백업한 뒤 전체 교체·빌드·실행합니다.

```bash
bash install_to_robot_ws.sh
```

이후 실행:

```bash
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch soarm4_description display.launch.py
```

기본 GUI는 엔코더2047 기준 홈자세에서 시작하며 Center도 홈으로 돌아갑니다. 기존 URDF 좌표계의 홈 각도는 J1=0°, J2=180°, J3≈−179.9121°, J4=0°입니다. 기하학적0자세와 엔코더 홈은 다릅니다. 정수 슬라이더의 한 눈금 내 반올림이 있습니다.

정적 홈 표시는 mode:=home, 원본 기하학적0자세는 mode:=zero, 원본 휴식자세는 mode:=rest로 실행할 수 있습니다. 각 모드는 하나씩 실행하세요. 표시 토픽과 TF는 /soarm4 아래입니다.

## 유지한 설정

- J1~J3 STS3250, J4 STS3215. 외형은 제공된 조립 GLB의 공통 서보 형상입니다.
- J2 axis=(0,-1,0). 엔코더 최대4038, 기계적 최대4095에서57count≈5.01°의 여유입니다. 반대쪽 최소1993은 유지했습니다.
- J2 기본/GUI URDF lower=0.0874369049rad, upper=3.2244rad. 360°로 확장하지 않았습니다.
- 다른 관절축, 원점, 리미트, 메시와 엔코더 참고값은 이전 버전 그대로입니다.
- 01 베이스 부분만 J1BASE.stl로 교체했고02 모터 홀더는 원래 형상을 보존했습니다.

본 패키지는 시각화/기구학용이며 실제 모터 통신 노드를 포함하지 않습니다. servo_map.yaml은 기존 변환 참고값으로 실기 영점/회전 방향 보정은 별도 확인이 필요합니다. J3의2047 홈은 canonical 각도 하한의 안전여유 밖이며 기존 GUI 표시 범위 안입니다. 하드웨어 리미트를 자동 변경하지 않습니다. 질량/관성은 임의 생성하지 않았습니다.

## 재생성·검증

config/model.json의 robot_base_xyz에 이번 Y 정렬이 저장되어 있습니다. python3 make_urdf.py로 다시 생성해도 유지됩니다. 베이스 메시 재생성은 python3 tools/rebuild_base_mesh.py입니다. meshes는 이미 m 단위이므로0.001 배율을 다시 적용하면 안 됩니다.

python3 tools/validate_geometry.py는 원본 조립본과 새 베이스 등록 좌표를 기준으로 형상을 비교합니다. numpy가 필요합니다. XML 파싱, 전체 메시 참조,4개 관절 설정 보존,상자 고정 및 Y 단독 이동을 확인했습니다. 기존 RViz 캡처와 runtime 결과는 이번 Y 이동 전의 참고 자료이며, 이번 배치는 STL 구멍 좌표와 FK로 검증했습니다.


## J3 홈 하한 변경 (사용자 요청)
J3 canonical 하한을 -3.1400586728019073rad (-179.912109375도)로 변경했습니다. 홈 엔코더2047과 일치하며 기존 홈 쪽 약10도 여유를 제거합니다. servo_map.yaml의 J3 max_count도2047로 맞췄습니다. 반대쪽 상한과 다른 관절 설정은 유지합니다. 이 YAML은 실물 드라이버가 아닙니다.
