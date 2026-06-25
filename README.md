# Storagy Project (Docker)

Storagy 로봇 ROS 2 프로젝트(주차장 시뮬레이션)를 **Docker 하나로** 실행하는 워크스페이스입니다.

- **GPU 불필요** — 소프트웨어 렌더링으로 동작
- **Windows / macOS(Apple Silicon 포함) / Ubuntu** 어디서나 동일하게 실행
- Gazebo/RViz 화면은 **웹 브라우저(noVNC)** 로 표시 — 호스트에 X 서버 설정 불필요
- `src/` 폴더가 **볼륨 마운트**되어 있어 코드를 직접 수정·실험 가능

## 구성

| 구성요소 | 내용 |
|---|---|
| 베이스 이미지 | `tiryoh/ros2-desktop-vnc:humble` (ROS 2 Humble + MATE 데스크탑 + noVNC) |
| 시뮬레이터 | Gazebo Harmonic (gz-sim8) + ros_gz |
| 내비게이션 | Nav2, slam_toolbox (apt 바이너리) |

## 빠른 시작

사전 준비: [Docker Desktop](https://www.docker.com/products/docker-desktop/)(Windows/Mac) 또는 Docker Engine + compose 플러그인(Ubuntu).

```bash
# 이미지 빌드 후 실행
docker compose up -d --build
```

> **아키텍처 안내**: ROS↔Gazebo Harmonic 브릿지(`ros_gz`)가 **arm64 apt 빌드가 없어**
> 이 이미지는 **amd64 전용**입니다(`docker-compose.yml` 의 `platform: linux/amd64`).
> - **Windows / Linux (amd64)** — 네이티브로 그대로 동작
> - **macOS (Apple Silicon, arm64)** — Docker Desktop 이 **Rosetta/QEMU 에뮬레이션**으로 실행
>   (동작은 하지만 Gazebo가 다소 느립니다. Docker Desktop → Settings → General →
>   "Use Rosetta for x86/amd64 emulation" 활성화 권장)

브라우저에서 **http://localhost:6080** 으로 접속하면 리눅스 데스크탑이 열립니다.
데스크탑 안의 터미널에서 시뮬레이션(Gazebo + 로봇 + RViz)을 실행합니다:

```bash
# 변경(월드/런치/맵/param)을 반영하려면 먼저 한 번 재빌드
rebuild_ws.sh

# 주차장 월드 + 로봇 (구조/주차칸 확인용)
ros2 launch storagy simulation_bringup.launch.py use_slam:=false use_nav2:=false

# 자율주행까지 (parkinglot 맵 + Nav2)
ros2 launch storagy simulation_bringup.launch.py use_slam:=false use_nav2:=true
```

## 자율 주차 데모 (카메라 + 라이다)

로봇이 주차장 월드에서 **빈 주차칸을 스스로 찾아 진입**하는 데모입니다. 카메라(주차선)와
라이다(점유/깊이)를 결합한 하이브리드 방식입니다.

```bash
# 전체 자율 주차 데모 (sim + 점유 + 선검출 + Nav2 + 주차 매니저 + 도킹)
scripts/clean_and_run_parking.sh
#   └ 헤드리스로: scripts/clean_and_run_parking.sh use_rviz2:=false use_gui:=false
#   └ Phase 0~2만(주행 없이 인지만): scripts/clean_and_run_parking.sh parking_demo.launch.py

# 상태 확인
ros2 topic echo /parking/occupancy   # P1=occupied,P2=free,...
ros2 topic echo /parking/lines       # 카메라가 측정한 주차선(divider) x
ros2 topic echo /parking/state       # IDLE→NAVIGATING→ARRIVED→DOCKING→DOCKED
```

`clean_and_run_parking.sh` 는 이전에 떠 있던 sim/ROS 스택(Nav2 서버 포함)을 먼저 모두
정리한 뒤 **딱 하나**만 실행합니다(스택 sim 방지). RViz 에는 점유/주차선 마커와
코스트맵이 함께 표시됩니다(`rviz/parking.rviz`).

**동작 흐름**

1. **점유 판정** `slot_occupancy_node` — `/scan` 을 map 프레임으로 변환해 칸별 점 개수로
   occupied/free 판정 (`param/parking_spaces.yaml` 의 칸 좌표 사용).
2. **주차선 검출** `line_detector_node` — `/camera/depth/points`(XYZ+RGB)에서 검은 테이프를
   골라 map 프레임의 divider x 를 측정.
3. **접근 주행** `parking_manager_node` — 빈 칸을 골라 Nav2(`NavigateToPose`)로 칸 입구 앞
   접근 포즈까지 주행. (AMCL 없이 ground-truth odom + 식별 `map→odom`,
   `param/navigation2/storagy_parking.yaml` 의 롤링 코스트맵 사용.)
4. **도킹** `parking_manager_node` — 카메라가 측정한 divider x 중점으로 측방을 정렬하고,
   라이다 후벽 거리로 깊이를 맞춰 칸 안 `parked_pose` 까지 정밀 진입.

> 메인 런치 파일은 `launch/parking_nav.launch.py` 입니다. 인지만 보려면
> `launch/parking_demo.launch.py` 를 쓰세요.

## 맵 불러오기

매핑은 이미 다른 곳에서 했다고 가정합니다. 기본 맵은 **`parkinglot`** 으로 이미 연결돼 있어,
`use_nav2:=true` 로 시뮬을 띄우면 자동으로 로드됩니다:

```bash
ros2 launch storagy simulation_bringup.launch.py use_slam:=false use_nav2:=true
```

- 맵 파일은 `src/storagy/map/parkinglot.yaml` (+ `.pgm`) 입니다. 다른 맵을 쓰려면 같은 폴더에
  넣고 `map:=<이름>.yaml` 런치 인자로 지정하거나, `simulation_bringup.launch.py` 의 기본 맵 경로를 바꾸세요.
- 맵 파일을 새로 추가/수정한 뒤에는 재빌드가 필요합니다: `docker compose exec storagy-project rebuild_ws.sh`

> 참고: `bringup.launch.py` 는 **실제 로봇**용 bringup(robot_state_publisher + 하드웨어)이며
> Gazebo 시뮬과는 무관합니다. 시뮬은 `simulation_bringup.launch.py` 를 사용하세요.

종료:

```bash
docker compose down
```

## Gazebo에서 주차장 월드 확인하기

주차장 환경은 `src/storagy/worlds/parkinglot.sdf` 에 들어 있습니다
(벽 + 검은 테이프 주차칸 4개 + 바닥/조명, 외부 메시 참조 없음).

```bash
docker compose up -d --build
docker compose exec storagy-project rebuild_ws.sh
# http://localhost:6080 접속 후 데스크탑 터미널에서:
ros2 launch storagy simulation_bringup.launch.py use_slam:=false use_nav2:=false
```

- 오른쪽 벽에 붙은 **검은 ㄷ자 주차선 4칸**(입구는 안쪽으로 개방)과 로봇이 보입니다.
- 로봇은 `(0, 0)` 에 **세로로 길게(+Y 방향)** 스폰되며, 오른쪽(+X)이 주차벽입니다.
- 각 주차칸 좌표·주차 목표 자세는 `src/storagy/param/parking_spaces.yaml` 에 정리돼 있습니다.

> 월드 파일만 빠르게 보고 싶다면(로봇 없이), Gazebo Harmonic 이 설치된 환경에서
> `gz sim src/storagy/worlds/parkinglot.sdf` 한 줄이면 됩니다. (아래 WSL 경로 참고)

## WSL에서 네이티브 실행 (Windows 팀원용)

Docker 없이 **WSL2 + Ubuntu 22.04** 에서 직접 빌드해 실행할 수도 있습니다.
WSLg 가 GUI를 그대로 띄워주므로 noVNC 없이 Gazebo/RViz 창이 바로 뜹니다.
(Windows 11 또는 최신 Windows 10 권장)

### 1. WSL2 + Ubuntu 22.04 설치 (PowerShell, 관리자)

```powershell
wsl --install -d Ubuntu-22.04
# 설치 후 Ubuntu 터미널을 열어 사용자 계정을 만든 뒤 아래는 모두 Ubuntu 안에서 실행합니다.
```

### 2. ROS 2 Humble + Gazebo Harmonic 설치 (Ubuntu 안)

```bash
# ROS 2 Humble
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install -y ros-humble-desktop python3-colcon-common-extensions

# Gazebo Harmonic + ros_gz + Nav2/SLAM (Dockerfile 과 동일한 패키지)
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
     -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
     | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update && sudo apt install -y \
     gz-harmonic ros-humble-ros-gzharmonic \
     ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
     ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
     ros-humble-rviz2
```

### 3. 워크스페이스 빌드 & 실행

```bash
git clone https://github.com/Aninnom/storagy-project-docker.git
cd storagy-project-docker

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# GPU 패스스루가 없으면 소프트웨어 렌더링으로
export LIBGL_ALWAYS_SOFTWARE=1

# 월드 파일만 빠르게
gz sim src/storagy/worlds/parkinglot.sdf
# 또는 로봇까지
ros2 launch storagy simulation_bringup.launch.py use_slam:=false use_nav2:=false
```

> 코드/월드/맵/param 을 고친 뒤에는 `colcon build` 를 다시 돌리고
> `source install/setup.bash` 로 새 환경을 적용하세요.

## 코드 수정하기 (볼륨 마운트)

호스트의 `./src` 폴더가 컨테이너의 `/opt/storagy_project_ws/src` 에 마운트되어 있어,
**호스트에서 파일을 수정하면 컨테이너에 즉시 반영**됩니다. 런치파일/월드/맵/URDF 등을
수정한 뒤에는 워크스페이스를 재빌드해야 합니다:

```bash
docker compose exec storagy-project rebuild_ws.sh
```

noVNC 데스크탑 안의 터미널에서 직접 `rebuild_ws.sh` / `run_sim.sh` 를 실행해도 됩니다.

## 폴더 구조

```
├── Dockerfile              # 이미지 정의 (Gazebo/Nav2/SLAM 의존성 + colcon build)
├── docker-compose.yml      # 포트, 볼륨 마운트
├── docker/
│   ├── run_sim.sh          # 데스크탑 세션에서 시뮬레이션 실행
│   └── rebuild_ws.sh       # src 수정 후 컨테이너 안에서 colcon 재빌드
└── src/
    └── storagy/            # 로봇 패키지 (URDF, 월드, 맵, 런치, param, scripts ...)
        ├── package.xml
        ├── CMakeLists.txt
        ├── launch/
        │   ├── simulation_bringup.launch.py  # Gazebo + 로봇 + RViz (+ SLAM/Nav2)
        │   └── ...
        ├── worlds/
        │   └── parkinglot.sdf      # 주차장 월드 (벽 + 검은 테이프 주차칸 4개)
        ├── map/
        │   ├── parkinglot.yaml     # parkinglot 맵 (+ .pgm)
        │   └── ...
        └── param/
            ├── parking_spaces.yaml          # 주차칸 좌표/주차 목표 자세
            └── navigation2/storagy.yaml     # Nav2 (풋프린트 0.4x0.3 등)
```
