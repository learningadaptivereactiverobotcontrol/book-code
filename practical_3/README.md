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

## Authors/Maintainers 

Maxime Gautier : maxime.gautier@epfl.ch
Tristan Bonatio : tristan.bonato@epfl.ch


