# Storagy Project (Docker)

Storagy 로봇 ROS 2 프로젝트를 **Docker 하나로** 실행하기 위한 워크스페이스 골격입니다.

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

브라우저에서 **http://localhost:6080** 으로 접속하면 리눅스 데스크탑이 열립니다.
데스크탑 안의 터미널에서 실행합니다:

```bash
ros2 launch storagy bringup.launch.py
```

## 맵 불러오기

매핑은 이미 다른 곳에서 했다고 가정합니다. 완성된 맵 파일(`<이름>.yaml` + `<이름>.pgm`)을
`src/storagy/map/` 에 넣고, 런치 인자로 지정해 불러옵니다:

```bash
ros2 launch storagy bringup.launch.py map:=<이름>.yaml
```

- 기본 맵 이름은 `bringup.launch.py` 상단의 `DEFAULT_MAP` 에 정의돼 있습니다(현재 `map.yaml`). 맵이 준비되면 이 값을 바꾸세요.
- `bringup.launch.py` 는 맵 로딩 + Nav2(map_server·AMCL·내비게이션)만 연결돼 있습니다.
  AMCL이 위치추정을 하려면 `/scan` 과 TF를 발행하는 **로봇/Gazebo 시뮬**이 함께 떠 있어야 합니다 — 해당 부분은 프로젝트를 키우며 추가하세요.
- 맵 파일을 새로 추가/수정한 뒤에는 재빌드가 필요합니다: `docker compose exec storagy-project rebuild_ws.sh`

종료:

```bash
docker compose down
```

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
    └── storagy/            # 로봇 패키지 (런치 파일 등 — 여기에 살을 붙여 나갑니다)
        ├── package.xml
        ├── CMakeLists.txt
        ├── launch/
        │   └── bringup.launch.py   # 맵 로딩 + Nav2 연결
        └── map/                    # 완성된 맵(.yaml/.pgm)을 여기에 넣으세요
```
