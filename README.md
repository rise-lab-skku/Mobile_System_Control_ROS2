# Mobile System Control (ROS 2)

This repository is for the ROS 2 version of the **Mobile System Control** lecture. For the ROS 1 version, please refer to [Mobile_System_Control](https://github.com/rise-lab-skku/Mobile_System_Control).

![MPC Controller Simulation Screen](https://github.com/rise-lab-skku/Mobile_System_Control/assets/80592399/8638328b-c6c6-4007-86b4-e025b3875c0c)

## Prerequisites

This repository has been tested on **Ubuntu 20.04 with ROS 2 Foxy**.  
Before building, please ensure your environment meets the prerequisites listed below.  

> Note: It was also successfully tested on **Ubuntu 22.04 with ROS 2 Humble** as of 2025-08-18.


1.  **ROS 2 Foxy Installation**
* Installation guide: [ROS 2 Foxy Official Installation Docs](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
* You must source the ROS 2 environment in every new terminal:
    ```sh
    source /opt/ros/foxy/setup.bash
    ```

## Launching CARLA
1. Download CARLA: [Mobile System Control package version of CARLA simulator](https://1drv.ms/u/c/c0946eca17387fd6/EctYFydgbM5JmR690gOO5AcByn2la_gwNoDY4BHIwHm-_A?e=pa897O)
2. Launch CARLA
    ```sh
    ./CarlaUE4.sh
    ```
## Set CARLA API
1. Check the directory of Python API in CARLA simulator
2. Add PYTHONPATH in ~/.bashrc or ~/.zshrc
* bash
    ```sh
    echo "PYTHONPATH=$PYTHONPATH:~/Downloads/CARLA_package/CARLA/PythonAPI/carla/dist/carla-0.9.15-py3.8-linux-x86_64.egg:~/Downloads/CARLA_package/CARLA/PythonAPI/carla/" >> ~/.bashrc
    ```

* zsh
    ```sh
    echo "PYTHONPATH=$PYTHONPATH:~/Downloads/CARLA_package/CARLA/PythonAPI/carla/dist/carla-0.9.15-py3.8-linux-x86_64.egg:~/Downloads/CARLA_package/CARLA/PythonAPI/carla/" >> ~/.zshrc
    ```

## Build ROS 2 Packages

###  Create a ROS 2 Workspace

1.  Create a ROS 2 workspace named `your workspace name`.
    ```sh
    mkdir -p ~/carla_ws_ros2/src
    cd ~/carla_ws_ros2/src
    ```
2. Clone this repository into the `src` folder.
    ```sh
    git clone https://github.com/rise-lab-skku/Mobile_System_Control_ROS2
    ```

### Install Dependencies

1. **Install External Libraries (e.g., QP Solver)**

    Run the provided `install.sh` script to install optimization libraries required for controllers like MPC.
    ```sh
    cd ~/carla_ws_ros2/src/Mobile_System_Control_ROS2
    sudo ./install.sh
    ```

### Build the Packages
1. Navigate to the workspace root directory and build all packages
    ```sh
    cd ~/carla_ws_ros2
    colcon build
    ```
2.  After the build is complete, you must source the setup file in every new terminal to use the packages.
- bash
    ```sh
    source ~/carla_ws_ros2/install/setup.bash
    ```
- zsh
    ```sh
    source ~/carla_ws_ros2/install/setup.zsh
    ```

## Launch Control Examples
1. Launch **carla_ros_bridge**
    ```sh
    ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
    ```
2. Launch **mobile_system_control**
    ```sh
    ros2 launch mobile_system_control mobile_system.launch.py
    ```
    ```sh
    ros2 launch mobile_system_control_vis mobile_system_control_vis.launch.py
    ```
3. Spawn vehicle and control window
    ```sh
    ros2 launch carla_spawn_objects carla_spawn_objects.launch.py object_name:=ego
    ```
    ```
    ros2 launch carla_manual_control carla_manual_control.launch.py role_name:=ego_vehicle
    ```
4. Launch control examples
 * PID controller
     ```
     ros2 launch pid_control_ex pid_control_ex.launch.py
     ```
 * Pure Pursuit controller
     ```
     ros2 launch purepursuit_control_ex purepursuit_control_ex.launch.py
     ```
 * Kanayama controller
     ```
     ros2 launch kanayama_control_ex kanayama_control_ex.launch.py
     ```
 * MPC controller
     ```
     ros2 launch mpc_control_ex mpc_control_ex.launch.py
     ```
## Download Link
1. [RoadRunner Map Files](https://1drv.ms/f/c/c0946eca17387fd6/Ekn4u42uQWhFsWUfN6Ae_8MBeV0yV4mr3dVWXNLXa3r6tQ?e=zc3rHF)
2. [ERP42 model (.dae)](https://1drv.ms/f/c/c0946eca17387fd6/EuThXux5BYZJltwDm1Geoq0BStqVOnv9tUcdESow4elpzg?e=IvXMAu)
3. [Blender Project File](https://1drv.ms/f/c/c0946eca17387fd6/El960lX0-r1BvrXtjOBeUZkBTbYzafYrxz1eFzJNy0-G_A?e=eJVt5j)
4. [ERP42 model from Blender (.fbx)](https://1drv.ms/f/c/c0946eca17387fd6/EqXsgeX6ZH5HlnJ_Vyb28WwBDQvXChfA8vYL10FLc-IWBg?e=OSJgdm)