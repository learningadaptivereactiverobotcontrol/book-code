# Practical 3 setup 

This file describes the dependencies and setup needed to run the practical_3 in simulation.

## Dependencies

The only dependency for this practical is the *simulator-backend*, available in the 'dependencies' folder.
It contains a docker image which can be run to start a pybullet simulation fo the Franka Emika Robot.

## Setup 

### Install Matlab

Assuming you do not already have MATLAB, you need to install it with the toolboxes listed in the top folder's README.

### Install docker

First, you need to install docker with sudo privileges. You can either do it yourself or run the docker/install_docker.sh script.

Note : Make sure everything is installed correctly by running the command :
```console
docker run hello-world
```

### Building images

Build the docker images for:
- simulator-end
- matlab-bridge

Follow these detailed steps : 

#### simulator-backend

Navigate to the simulator-backend folder and run the script :
```console
cd dependencies/simulator-backend/pybullet_zmq
bash docker/build-image.sh
```

### matlab-bridge
This is the container bridging matlab, ROS and the robot.

```console
bash docker/build_docker.sh
```

You can find more detailed information in each submodule's README.

## Authors/Maintainers 

Maxime Gautier : maxime.gautier@epfl.ch
Tristan Bonatio : tristan.bonato@epfl.ch
