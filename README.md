# Mobile System Control
This is the repository for **Mobile System Control** lecture

   ![mpc](https://github.com/rise-lab-skku/Mobile_System_Control/assets/80592399/8638328b-c6c6-4007-86b4-e025b3875c0c)

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


## Build Example Packages
1. Setup **catkin build**
* Change **catkin_make** workspace to **catkin build**
    ```sh
    pip3 install pip --upgrade
    pip3 install -U catkin_tools
    cd ~/catkin_ws/src
    rm CMakeLists.txt
    cd ..
    rm -rf build devel
    mkdir build devel install logs
    catkin **init**
    catkin build
    ```
2. Build Packages
* Install external library for QP solver and CARLA messages
    ```
    cd ~/catkin_ws/src
    git clone https://github.com/rise-lab-skku/Mobile_System_Control.git
    cd Mobile_System_Control
    sudo ./install.sh
    ```
* Build packages
    ```sh
    colcon build
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
        ros2 launch pure_pursuit_control_ex pure_pursuit_control_ex.launch.py
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