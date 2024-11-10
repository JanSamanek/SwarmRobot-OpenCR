FROM ros:humble

WORKDIR /root/ros2_ws

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["bash"]