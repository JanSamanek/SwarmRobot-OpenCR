FROM ros:humble

ARG ADDITIONAL_PACKAGES 
WORKDIR /root/ros2_ws

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-ament-package \
    ros-humble-robot-state-publisher \
    ${ADDITIONAL_PACKAGES} \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# COPY ros2_ws/src /root/ros2_ws

SHELL [ "/bin/bash", "-c" ]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# RUN rosdep install --from-paths src --ignore-src -r -y
# RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install
# RUN source install/setup.bash

ENTRYPOINT ["ros2 ", "launch", "controller", "controller.launch.py"]
