# Storagy 자율주차 — 실제 로봇 통합 디버그 리포트
**날짜:** 2026-06-25
**대상 로봇:** `storagy@192.168.0.83` (Ubuntu 22.04, ROS 2 Humble)
**워크스페이스:** `~/Desktop/storagy_project_ws` (시뮬/주차, 활성) · `~/Desktop/storagy_ws` (구 실하드웨어 사본)
**목표:** Gazebo에서 개발한 자율주차 로직을 실제 로봇 하드웨어에서 동작
**결과:** ✅ **라이다 기반 자율주차 end-to-end 성공** (P2 도킹 오차 ~3cm)

---

## 0. 환경/접속 관련 핵심 메모 (재현 시 필수)
- 비대화형 SSH는 `~/.bashrc`를 안 읽으므로, 실행 중인 ROS 그래프를 보려면 **반드시** 두 환경변수를 export:
  - `export ROS_DOMAIN_ID=2`
  - `export ROS_LOCALHOST_ONLY=1`
  - 둘 중 하나라도 빠지면 디스커버리가 **조용히 빈 결과**를 반환 (노드/토픽 안 보임).
- 빌드: `colcon build --symlink-install`
  - launch/scripts/param/config 디렉토리는 **심볼릭 링크 설치** → src 수정이 즉시 반영(런치/스크립트/yaml은 재빌드 불필요).
  - `motor_driver2`(ament_python)는 수정 후 재빌드 필요.
- **좀비 프로세스 주의:** `sick_generic_caller`, `motor_driver2`, `nav2_amcl` 등이 **Ctrl-C로 안 죽고** 시리얼/라이다 TCP/USB를 점유 → 다음 launch가 충돌(SIGSEGV, "multiple access on port"). **재실행 전 반드시 정리** (`run_parking_real.sh`가 자동 처리).
- 텔레옵은 `/cmd_vel`이 아니라 **`/cmd_vel_nav`** 로 보내야 함 (Nav2 velocity_smoother가 `/cmd_vel` 소유):
  `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_nav`

---

## 1. 발생한 문제와 수정 (시간 순)

### 1-1. `bringup.launch.py` 실행 실패 — `hardware_bringup.launch.py` 없음
- **증상:** `No such file ... hardware_bringup.launch.py`
- **원인:** 해당 launch 파일이 `storagy_ws`에는 있으나 `storagy_project_ws`엔 누락.
- **수정:** `storagy_ws`에서 `hardware_bringup.launch.py` + `config/sick_scan_xd/` 복사 → `colcon build --packages-select storagy`.

### 1-2. nav2가 `Invalid frame ID "map"` 무한 반복
- **원인:** 이전 세션의 좀비 노드(특히 motor_driver2)가 `/dev/ttyUSB0` 점유 → 새 motor_driver2 즉사 → `/odom`·`odom→base` 없음 → AMCL이 `map→odom` 못 만듦 → `map` 프레임 부재.
- **수정:** 모든 잔존 ROS 노드 정리 후 재실행. (이때 좀비 문제 패턴 확인)

### 1-3. (시뮬 주차) `ros_gz_sim` 미설치 + ROS GPG 키 만료
- **증상:** `package 'ros_gz_sim' not found` → 설치 시도 시 모든 .deb **404** → `apt update`도 실패.
- **근본 원인:** ROS 저장소 **GPG 키가 2025-06-01 만료** → 색인 갱신 불가 → 옛 파일명 404.
- **수정:** 갱신 키로 교체 후 update/install
  ```bash
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  sudo apt update
  ```
  (신규 키 만료일 2030-06-01, InRelease 서명 검증 통과 확인)
- **참고:** `clean_and_run_parking.sh`의 하드코딩 경로 `/opt/storagy_project_ws` → `/home/storagy/Desktop/storagy_project_ws`로 수정(컨테이너용 스크립트였음).

### 1-4. 방향 전환: 시뮬이 아니라 실제 로봇 구동이 목표
- 실제 구동에는 Gazebo 불필요. **주차 앱 노드 3개를 실물 센서+Nav2 위에 배선**하는 통합으로 전환.
- 신규 `parking_real.launch.py` 작성 (bringup + slot_occupancy + line_detector + parking_manager).
- 카메라: line_detector가 컬러 포인트클라우드 필요 → `hardware_bringup.launch.py`에 `enable_colored_point_cloud:=true`, `depth_registration:=true` 추가.

### 1-5. line_detector 실물 컬러 클라우드 콜백 크래시
- standalone 기동은 정상, **실제 클라우드 수신 시 콜백에서 exit 1**.
- **대응(보류):** `use_line_detector:=false`로 라이다만 사용 (occupancy·도킹엔 충분). 카메라 횡센터링만 미사용.

### 1-6. ⭐ 근본 원인: 휠 오도메트리 펌웨어 캘리브레이션 오류
- **증상:** 로봇은 물리적으로 똑바로 가는데, RViz/오도메트리는 계속 우회전으로 인식 → 엉뚱한 방향 주행, 베이 측정 불일치, 맵 밖 이탈.
- **정량화:**
  - 직진 2m → odom yaw **−113°** (≈ −57°/m) 가짜 회전 (좌/우 휠 거리환산 비대칭)
  - 정지 중엔 안정(자이로 바이어스 아님), 360° 회전 → odom ~410°로 과다(트랙폭도 의심, 잠정)
- **수정 불가 영역:** 오도메트리는 **MCU 펌웨어**가 계산해 시리얼(`/dev/ttyS4`)로 중계. 소스/.bin/플래셔 없음, 캘리브 명령 없음, **IMU 없음**(sick TiM imu 토픽은 데이터 미발행). → ROS 파라미터로 못 고침.
- **해결(소프트웨어 우회): rf2o_laser_odometry (라이다 스캔매칭 오도메트리)**
  - MAPIRlab 소스 클론 → `colcon build` (사용자 승인 후).
  - `motor_driver2`의 `odom→base_footprint` TF 발행 비활성화(`sendTransform(t)` 라인).
  - rf2o가 `/scan`에서 `odom→base_footprint` 발행하도록 `bringup.launch.py`에 노드 추가
    (params: laser_scan_topic `/scan`, base_frame_id `base_footprint`, odom_frame_id `odom`, publish_tf true, freq 20).
  - **검증:** 직진 2m → yaw 변화 **~2°** (이전 113°), 횡편차 2cm. AMCL 추종 정상.

### 1-7. 베이 좌표를 실제 맵 프레임으로 재측정
- 기존 `parking_spaces.yaml` 좌표는 Gazebo 월드 기준 → 실제 SLAM 맵과 불일치.
- 오도메트리 정상화 후, 텔레옵으로 각 베이 주차 → `map→base_footprint` 측정.
  - P1 (−1.135, −0.059, 90°), P2 (−0.701, −0.059, 90°) **직접 측정**, 간격 0.434m, +X 한 줄·축정렬 확인.
  - P3 (−0.267), P4 (+0.167) **균일 외삽**. 모두 y≈−0.059, yaw 90°.
- `param/parking_spaces.yaml` 실제 맵 좌표로 교체.
- **검증:** 1·3·4 차량 배치, 2 비움 → `/parking/occupancy` = `P1=occupied,P2=free,P3=occupied,P4=occupied` (실제와 정확히 일치).

### 1-8. 접근 방향(입구 측) 뒤집힘 — 로봇이 제자리 좌회전만
- **증상:** 주차 시작 시 로봇이 진행 못 하고 제자리 좌회전, 타겟이 P2→P1(점유칸)로 튐.
- **원인:** 코드가 "입구 = −Y" 가정인데 **실제 입구는 +Y**(로봇이 +Y쪽 열린 공간에 있음) → 접근 포즈가 베이 뒤(도달 불가) → 주행 실패·재선택 thrash.
- **수정 (parking_manager_node.py):**
  - `_approach_pose`를 **일반식**으로: `x = cx − d·cos(yaw)`, `y = cy − d·sin(yaw)` (양쪽 입구 모두 지원).
  - 도킹 횡오차 부호를 방향에 맞게: `e_ct = (rx − center) · copysign(1, sin(approach_yaw))`.
  - 파라미터: `approach_yaw:=-1.5708`(−Y 향함, +Y에서 진입), `approach_offset:=0.70`(중심→접근 거리).

---

## 2. 최종 성공 로그
```
NAVIGATING  target=P2  approach=(-0.70, 0.64, -90°)   robot (-0.69, 0.63)
DOCKING     target=P2                                  robot (-0.69, 0.14) → (-0.69, -0.08)
DOCKED      target=P2                                  robot (-0.681, -0.086)
```
- P2 중심 (−0.701, −0.059) 대비 도킹 오차 **~2–3cm**.
- 파이프라인: rf2o 오도메트리 → AMCL 정합 → 라이다 점유 감지 → 빈칸 선택 → Nav2 접근 → 베이 도킹.

---

## 3. 실행 방법 (현재 동작 워크플로)
```bash
# 터미널 1 — 하드웨어 + rf2o + Nav2 + AMCL + RViz
cd ~/Desktop/storagy_project_ws && source install/setup.bash
ros2 launch storagy bringup.launch.py
#  → RViz에서 2D Pose Estimate로 정합 (라이다 스캔이 맵 벽에 겹치게)

# 터미널 2 — 주차 앱 노드 (정합 확인 후 실행)
cd ~/Desktop/storagy_project_ws && source install/setup.bash
ros2 launch storagy parking_nodes.launch.py use_line_detector:=false
#  관찰:  ros2 topic echo /parking/state   /   /parking/occupancy
```
- 정합 전 점유/베이 박스만 확인하려면: `... parking_nodes.launch.py manager_delay:=600` (로봇 안 움직임).
- 한 방 정리+실행: `~/Desktop/storagy_project_ws/src/storagy/scripts/run_parking_real.sh use_line_detector:=false` (단, bringup의 RViz 대신 parking.rviz 사용 — RViz 안 뜨면 위의 2-터미널 방식 권장).

---

## 4. 변경/생성 파일 (모두 .bak/.bak2 백업 존재) — 로봇의 `~/Desktop/storagy_project_ws/src/storagy/`
- `launch/bringup.launch.py` — rf2o 노드 추가
- `launch/hardware_bringup.launch.py` — storagy_ws에서 복사 + 컬러 포인트클라우드 활성화
- `launch/parking_real.launch.py` — (신규) 실물 주차 통합 (bringup 포함형)
- `launch/parking_nodes.launch.py` — (신규) 주차 노드만 (bringup 분리 실행용, 권장)
- `scripts/run_parking_real.sh` — (신규) 좀비 정리 후 실행
- `scripts/parking_manager_node.py` — _approach_pose 일반화 + 도킹 횡오차 부호 보정
- `param/parking_spaces.yaml` — 실제 맵 좌표로 교체
- `config/sick_scan_xd/new_sick_tim_5xx_front.yaml` — storagy_ws에서 복사
- `src/motor_driver2/motor_driver2/motor_driver2.py` — odom→base TF 발행 비활성화
- `src/rf2o_laser_odometry/` — (신규, MAPIRlab 소스) 라이다 오도메트리

---

## 5. 남은 과제 / 권장
1. **펌웨어 휠 오도메트리 캘리브레이션** (근본) — 제조사/펌웨어 팀.
   - 좌/우 휠 거리환산 상수 비대칭 제거(직진 시 yaw≈0), 이후 트랙폭 보정. 현재는 rf2o로 우회 중.
2. **line_detector 카메라 콜백 크래시** 수정 → 컬러 클라우드 기반 도킹 횡센터링(정밀도 향상). 현재 라이다만으로 ~3cm.
3. **SICK 라이다 노드 안정화** — 가끔 죽고 Ctrl-C로 안 죽음. (현재 재시작으로 대응)
4. **추가 점유 시나리오 테스트** (빈칸 P1/P3/P4 변경).
5. (선택) **rf2o 오도메트리로 맵 재생성** — 기존 맵은 틀어진 오도메트리로 만들어졌을 수 있음.

---
*작성: Claude Code 세션 (2026-06-25). 로봇 비밀번호 등 민감정보는 본 문서에 미포함.*
