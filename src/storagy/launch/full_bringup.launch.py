#!/usr/bin/env python3
"""전체 시스템 통합 실행 런치파일.

터미널 여러 개에서 따로 실행하던 노드들을 한 번에 실행한다:
  1. simulation_bringup.launch.py (Gazebo + RViz + Nav2, use_slam:=false use_nav2:=true)
  2. wander_nav_bt_node.py        (배회 BT 노드, Nav2 기동 대기를 위해 지연 실행)
  3. yolo_detector.py             (YOLO 사람 감지)
  4. storagy_llm agent_service    (LLM 에이전트 서비스)
  5. storagy_llm web_dashboard    (웹 대시보드)

agent_client 는 대화형 CLI 이므로 여기 포함하지 않는다. 별도 터미널에서:
  ros2 run storagy_llm agent_client

사용법:
  ros2 launch storagy full_bringup.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    storagy_share = get_package_share_directory('storagy')
    # install/storagy/share/storagy -> 워크스페이스 루트
    ws_root = os.path.abspath(
        os.path.join(storagy_share, '..', '..', '..', '..'))
    scripts_dir = os.path.join(ws_root, 'src', 'storagy', 'scripts')

    use_slam = LaunchConfiguration('use_slam')
    use_nav2 = LaunchConfiguration('use_nav2')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(storagy_share, 'launch',
                         'simulation_bringup.launch.py')),
        launch_arguments={
            'use_slam': use_slam,
            'use_nav2': use_nav2,
        }.items(),
    )

    wander_node = ExecuteProcess(
        cmd=['python3', os.path.join(scripts_dir, 'wander_nav_bt_node.py')],
        cwd=ws_root,
        output='screen',
    )

    # yolov8n.pt 가 워크스페이스 루트에 있어 cwd 를 맞춰준다
    yolo_detector = ExecuteProcess(
        cmd=['python3', os.path.join(scripts_dir, 'yolo_detector.py')],
        cwd=ws_root,
        output='screen',
    )

    # agent_service / web_dashboard 는 colcon 이 만든 console_script 래퍼다.
    # 래퍼 shebang 은 빌드 당시 인터프리터(시스템 /usr/bin/python3)로 고정돼,
    # venv 를 source 해도(=PATH 변경) 무시된다 → langchain/flask(venv 전용) 못 찾고 죽음.
    # prefix='python3' 를 붙이면 'python3 <래퍼>' 로 실행되어 yolo/wander 와 동일하게
    # "지금 활성화된 python3"(launch 전 venv activate → venv python)를 따른다.
    agent_service = Node(
        package='storagy_llm',
        executable='agent_service',
        prefix='python3',
        output='screen',
    )

    web_dashboard = Node(
        package='storagy_llm',
        executable='web_dashboard',
        prefix='python3',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_slam', default_value='false'),
        DeclareLaunchArgument('use_nav2', default_value='true'),
        simulation,
        # Gazebo/Nav2 가 어느 정도 올라온 뒤 나머지 노드 시작
        TimerAction(period=5.0, actions=[
            yolo_detector,
            agent_service,
            web_dashboard,
        ]),
        TimerAction(period=15.0, actions=[wander_node]),
    ])
