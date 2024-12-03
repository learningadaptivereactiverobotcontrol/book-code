ARG BASE_IMAGE=ghcr.io/aica-technology/ros2-modulo
ARG BASE_TAG=humble
FROM ${BASE_IMAGE}:${BASE_TAG} AS franka-lwi

WORKDIR /tmp
RUN sudo apt-get update && sudo apt-get install -y libsodium-dev libzmq3-dev
RUN wget https://github.com/zeromq/cppzmq/archive/v4.7.1.tar.gz -O cppzmq-4.7.1.tar.gz && tar -xzf cppzmq-4.7.1.tar.gz
RUN cd cppzmq-4.7.1 && mkdir build && cd build && cmake .. -DCPPZMQ_BUILD_TESTS=OFF && sudo make install
RUN rm -rf cppzmq*

RUN git clone -b v1.2.0 --depth 1 https://github.com/aica-technology/network-interfaces
RUN cd network-interfaces/cpp && mkdir build && cd build && cmake .. && sudo make install
RUN cd network-interfaces && bash install.sh --auto --no-cpp

RUN git clone -b develop --depth 1 https://github.com/aica-technology/franka_panda_description \
&& mv franka_panda_description ${ROS2_WORKSPACE}/src/franka_panda_description

RUN rm -rf /tmp/*


FROM franka-lwi AS dependencies

# upgrade ament_cmake_python
RUN sudo apt update && sudo apt install -y ros-${ROS_DISTRO}-ament-cmake-python && sudo rm -rf /var/lib/apt/lists/*
WORKDIR ${ROS2_WORKSPACE}


FROM dependencies AS build-sources

RUN sudo pip install pyzmq torch==1.13.1 sdf numpy==1.26.4

# Install tkinter
RUN sudo apt-get update && sudo apt-get install -y python3-tk

COPY --chown=${USER} ./matlab_bridge ./src/matlab_bridge
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --packages-select matlab_bridge franka_panda_description"

# clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
