ARG ROS_VERSION=humble
FROM ghcr.io/aica-technology/ros2-control-libraries:${ROS_VERSION} AS pybullet

RUN sudo pip3 install pybullet torch==1.13.1


FROM pybullet AS network-interfaces

WORKDIR /tmp
RUN git clone -b v1.1.0 --depth 1 https://github.com/aica-technology/network-interfaces.git && \
    cd network-interfaces && sudo bash install.sh --auto --no-cpp
RUN sudo rm -rf /tmp/network-interfaces


FROM network-interfaces AS robot-descriptions

WORKDIR ${ROS2_WORKSPACE}
COPY --chown=${USER} ./robot_descriptions/ ./src/
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --symlink-install"


FROM robot-descriptions AS install-source

WORKDIR ${HOME}
COPY --chown=${USER} ./pybullet_simulation ./pybullet_simulation
RUN sudo pip3 install --editable ./pybullet_simulation

WORKDIR ${HOME}
COPY --chown=${USER} ./pybullet_zmq/ ./pybullet_zmq/
RUN sudo pip3 install --editable ./pybullet_zmq

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
