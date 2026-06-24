# Storagy ROS 2 project — runs on any OS (Mac/Windows/Ubuntu), no GPU needed.
#
# Base provides a full Linux desktop (MATE) reachable from a web browser via
# noVNC, so Gazebo and RViz windows show up in the browser. Works on amd64
# and arm64 (Apple Silicon).
FROM tiryoh/ros2-desktop-vnc:humble

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
# 1. System dependencies
#    - Gazebo Harmonic (gz-sim8) + ros_gz (Harmonic) from the OSRF apt repo
#    - Nav2 + slam_toolbox + simulation support packages from the ROS repo
#    Add/remove packages here as the project grows.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release ca-certificates \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-humble-ros-gzharmonic \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-slam-toolbox \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-rviz2 \
 && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 2. Build the ROS 2 workspace
#    Only the project packages are built; Nav2/SLAM come from apt above.
# ---------------------------------------------------------------------------
ENV WS=/opt/storagy_project_ws
WORKDIR ${WS}
COPY . ${WS}

RUN source /opt/ros/humble/setup.bash \
 && colcon build --symlink-install \
 && chmod -R a+rX ${WS}

# ---------------------------------------------------------------------------
# 3. Helper scripts + auto-source the workspace in every new terminal
#    rebuild_ws.sh re-runs colcon build — use it after editing files in the
#    volume-mounted ./src on the host.
# ---------------------------------------------------------------------------
RUN install -m 755 ${WS}/docker/run_sim.sh /usr/local/bin/run_sim.sh \
 && install -m 755 ${WS}/docker/rebuild_ws.sh /usr/local/bin/rebuild_ws.sh

RUN for HOME_DIR in /root /home/ubuntu; do \
        if [ -d "$HOME_DIR" ]; then \
            { \
              echo '# --- storagy project ws ---'; \
              echo 'source /opt/ros/humble/setup.bash'; \
              echo "source ${WS}/install/setup.bash 2>/dev/null"; \
              echo 'export LIBGL_ALWAYS_SOFTWARE=1'; \
              echo 'export DISPLAY=:1'; \
            } >> "$HOME_DIR/.bashrc"; \
        fi; \
    done

EXPOSE 80
