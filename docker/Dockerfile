FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-numpy \
    python3-colcon-common-extensions \
    curl \
    ros-jazzy-rclpy \
    ros-jazzy-sensor-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    ros-jazzy-ros2topic \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir --break-system-packages aiohttp \
    || pip3 install --no-cache-dir aiohttp

WORKDIR /project
CMD ["bash"]
