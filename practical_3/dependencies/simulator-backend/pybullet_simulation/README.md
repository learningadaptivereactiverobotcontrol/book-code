# PyBullet Simulation

The `pybullet_simulation` package implements several helper classes that facilitate the setup of a PyBullet Simulator
for different network and robot interfaces (ROS, ROS2, ZMQ). The classes wrap the most important and useful methods from
the pybullet library and make them available in a more compact form.

The helper classes include:

- `RobotDescription` and `Robot`: Two classes that read the robot description from an URDF file, create the robot in the
  simulation, and allow for reading and writing the robot state.
- `Simulation`: This class creates the PyBullet server and GUI and steps the simulation.
- `FuncExecManager`: A function execution manager that keeps track of synchronous execution of multiple functions with
  deadline.

## Installation

The `pybullet_simulation` package contains a *setup.py* file and the installation is thus straight forward:

```bash
[sudo] pip3 install path/to/pybullet_simulation
```

Note that the package requires `Python>=3.8` and depends on `control-libraries>=3.1.0` which has to be installed
beforehand (see [this page](https://github.com/epfl-lasa/control-libraries/tree/main/python) for help).
