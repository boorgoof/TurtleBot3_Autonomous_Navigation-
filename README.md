To see the original repository: https://github.com/CaregnatoGianluca/group_number_4_assignment_1

**Authors:** Matteo Bino, Gianluca Caregnato, Federico Meneghetti  

## Project Overview

This repository contains the implementation for the **Assignment 1** (see the pdf).

We developed a robotic system capable of autonomous navigation and perception in a simulated environment. The main objectives of the project were:

- estimating the robot's final goal pose by detecting two fixed AprilTags viewed from a camera;
- implementing the standard navigation strategy (ROS2 nav2) to let the robot navigate to the goal position and eventually implement a custom controller (bonus points);
- detecting tables (cylindrical obstacles) from any available sensor and output their positions and radii;

## Installation and Build

Ensure you have a working ROS2 environment and the necessary dependencies installed (Nav2, AprilTag ROS).

- clone the repository for the simulation into your workspace:

    ```bash
    cd ~/ws_turtlebot_autonomus_navigation/src
    git clone https://github.com/PieroSimonet/ir_2526.git 
    ```

- build the package:
    ```bash
    cd ~/ws_turtlebot_autonomus_navigation
    colcon build
    ```

- source the setup file:
    ```bash
    source install/setup.bash
    ```

## Launch

There are two possible launch files located in the package **g4_launch**:

- **assignment_1.launch.xml**: launches the whole assignment using the autonomous navigation provided by nav2
- **assignment_1_v2.launch.xml**: launches the whole assignment using the custom navigation in the corridor, then switches back to the autonomous navigation provided by nav2 when the robot exits the corridor

You can launch the assignment by using the command:
```bash
ros2 launch g4_launch [launch_file_name]
```

## Fast commands:

To build and launch the program is possible to use the following commands:

`./build.sh`
`./run.sh` or `./run_2.sh`

## Node Diagram

<img src="./NodeDiagram.svg"/>

## Simulation Video

The simulation video can be downloaded [here](https://drive.google.com/file/d/1HAYpbHHGseoJaDzvPC-59G2zHD4fUUZC/view?usp=sharing).
