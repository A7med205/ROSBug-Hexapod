FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/hexapod_gait

# Resolve ROS dependencies before copying the rest of the source so that this
# layer remains cached when only implementation files change.
COPY ros2_ws/src/hexapod_sim/package.xml ros2_ws/src/hexapod_sim/package.xml
RUN rosdep update \
    && apt-get update \
    && rosdep install \
        --from-paths ros2_ws/src \
        --ignore-src \
        --rosdistro humble \
        -y \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt requirements.txt
RUN python3 -m pip install --no-cache-dir --requirement requirements.txt

COPY . .

WORKDIR /opt/hexapod_gait/ros2_ws
RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install --packages-select hexapod_sim

ENTRYPOINT ["/opt/hexapod_gait/docker/entrypoint.sh"]
