FROM ros:humble

ARG ADDITIONAL_PACKAGES 
WORKDIR /root/ros2_ws

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-robot-state-publisher \
    ${ADDITIONAL_PACKAGES} \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/ros2_ws/src
COPY ros2_ws/src .

SHELL [ "/bin/bash", "-c" ]
RUN rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install
RUN source install/setup.bash

ENTRYPOINT ["ros2 ", "launch", "controller", "controller.launch.py"]
