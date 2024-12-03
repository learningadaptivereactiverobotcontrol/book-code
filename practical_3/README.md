# Practical 3 Simulation Setup

This folder contains all the code necessary to run the practical 3 in simulation. 

The matlab folder contains the matlab application generating a task velocity DS, and the matlab_bridge folder is a C++ application implementing the joint torque controller. The two applications communicate with each other with ROS2.

The dependencies folder contains a pybullet simulator set up to communicate with the torque controller via ZMQ, along with materials to easily install everything.

## Setup 

Follow the instructions in [setup.md](setup.md) to install and build the images needed for the practical.

Once installed, everything is already set-up to run with the simulation on any computer.

## Terminal Commands

We will now open four terminals. We assume each terminal starts here (laarc-course-code/matlab_exercises/practical_3/)

### Terminal #1 - Pybullet simulation
First navigate to simulator-backend, then start the container, then launch the simulator using the folowing commands :
```console
cd dependencies/simulator-backend/pybullet_zmq
bash run.sh
zmq-simulator
```
Note : You can move the camera around with Ctrl + drag (mouse).

### Terminal #2 - Matlab-bridge
```console
bash docker/build_docker.sh #(OPTIONAL - if changed anything in non matlab code)
bash docker/start_docker.sh
ros2 launch matlab_bridge matlab_bridge.launch.py
```

### Terminal #3 - Matlab-bridge connect
```console
bash docker/start_docker.sh -m connect
ros2 run matlab_bridge print_robot_state
```

### Terminal #4 - MATLAB
```console
matlab
```

## Command to activate null space
```console
ros2 param set /matlab_bridge activate_ns true
ros2 param get /matlab_bridge activate_ns 
```

## Command to change gain weights
```console
ros2 param set /matlab_bridge gain_weights 1.0
ros2 param get /matlab_bridge gain_weights 
```
## Optional commands for running matlab_bridge
```console
ros2 run matlab_bridge matlab_bridge
ros2 launch matlab_bridge matlab_bridge.launch.py
```


## Authors/Maintainers 

Maxime Gautier : maxime.gautier@epfl.ch
Tristan Bonatio : tristan.bonato@epfl.ch


## Dev Notes

Commands to push the images to ghcr, replace relevant variables 
```console
docker login ghcr.io -u USERNAME -p YOUR_TOKEN
docker build -f docker/Dockerfile -t epfl-lasa/larrc/practical_3_sim .
docker push ghcr.io/epfl-lasa/larrc/practical_3_sim:latest
```

TO push to docker hub
```console
docker login
docker tag epfl-lasa/larrc/practical_3_sim:latest maxime00/epfl-lasa-larrc-practical_3:bridge
docker push maxime00/epfl-lasa-larrc-practical_3:bridge
```