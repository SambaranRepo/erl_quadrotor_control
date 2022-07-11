# erl_quadrotor_control

This ROS package contains code for quadrotor control (using PX4 quadrotor simulator for now). It's tested for:
- Ubuntu 18.04, 20.04
- ROS Melodic, Noetic
- Gazebo 9,11
- PX4 (release version 1.10.0 for Ubuntu 18.04, release version 1.11 for Ubuntu 20.04)

# Setup in Ubuntu 18.04
Installation instructions for Ubuntu 18.04. Ubuntu 18.04 requires ROS Melodic, Gazebo9 and PX4 Firmware release 1.10.0
_________________________________________________________________


## First time setup:
1. Install ROS Melodic under Ubuntu 18.04 http://wiki.ros.org/melodic/Installation/Ubuntu
2. Run ubuntu_sim_ros_melodic.sh from https://dev.px4.io/v1.10/en/setup/dev_env_linux_ubuntu.html#rosgazebo
     - This installs additional tools (mavros, px4 sitl) necessary for integration with px4 and running/visualizing drones.
     ```
     wget https://raw.githubusercontent.com/PX4/Devguide/v1.10/build_scripts/ubuntu_sim_ros_melodic.sh
     chmod +x ubuntu_sim_ros_melodic.sh
     source ubuntu_sim_ros_melodic.sh
     ```
3. Dependencies
    ```dependencies
    sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
    ```
    - This installs python tools for the GUI
    ```
    sudo apt install libqt4-dev
    sudo apt install python-qt4 pyqt4-dev-tools
    sudo apt install python-pyside pyside-tools
    ```
4. Install PX4 firmware
    - Clone the PX4 firmware repository: https://github.com/PX4/Firmware
    - The default PX4 location for this package: ~/PX4/Firmware
    ```
    git clone --branch release/1.10 https://github.com/PX4/Firmware.git --recursive
    cd Firmware
    git submodule update --init --recursive
    bash ./Tools/setup/ubuntu.sh
    make px4_sitl_default gazebo
    ```
    - Upgrade ```sudo apt upgrade libignition-math2``` and ```sudo apt upgrade libsdformat6``` before ```bash ./Tools/setup/ubuntu.sh``` if running into symbol errors with gazebo
    - Change ```#define HAS_GYRO TRUE``` to ```#define HAS_GYRO true``` in this file ```Tools/sitl_gazebo/include/gazebo_opticalflow_plugin.h```
5. Clone this repo to to ~/catkin_ws/src
     ```
     git clone git@github.com:ExistentialRobotics/erl_quadrotor_control.git
     ```
6. To use geometric controller, clone this repo to ~/catkin_ws/src
     ```
     git clone git@github.com:KumarRobotics/kr_mav_control.git
     ```
    
    and in this file ```/interfaces/kr_mavros_interface/nodelet_plugin.xml```, change the line ```<library path="lib/libso3cmd_to_mavros_nodelet">``` to ```<library path="lib/libkr_mavros_interface">```.
    
7. Run ```catkin config -DCMAKE_BUILD_TYPE=Release``` and then ``` catkin build```.


# Setup in Ubuntu 20.04
Installation instructions for Ubuntu 20.04. Ubuntu 20.04 requires ROS Noetic, Gazebo11 and PX4 Firmware release 1.11
_________________________________________________________________

## First time setup:

1. Run the following scripts to install common dependency packages, ROS Noetic, Gazebo11, mavros, mavlink and mavros-extras
<pre>$chmod +x ubuntu_20.04_sim_common_deps.sh<
$chmod +x ubuntu_20.04_sim.sh
$./ubuntu_20.04_sim_common_deps.sh
$./ubuntu_20.04_sim.sh
</pre>

2. Install official mavros, mavros-extras 
<pre>sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras</pre>

3. Now try to launch gazebo in standalone method. 
<pre>$gazebo</pre>
If it launches without any symbolic errors, well and good. Otherwise perform the steps below : 

<pre>
$sudo apt upgrade libignition-math2
$sudo apt upgrade libsdformat6
$sudo apt update 
$sudo apt upgrade
</pre>

4. Now install the PX4 Firmware or PX4-Autopilot(Here we proceed with PX4-Autopilot, the same can be done for Firmware. Just replace PX4-Autopilot with FIrmware)
<pre>git clone --branch release/1.11 https://github.com/PX4/PX4-Autopilot.git --recursive
$cd PX4-Autopilot
$git submodule update --init --recursive
$bash ./Tools/setup/ubuntu.sh
</pre>

After succesful setup of the PX4 firmware, you will get a message to reboot or relogin before proceeding to the next step. 

<pre>$cd PX4/PX4-Autopilot
$make px4_sitl_default gazebo
</pre>

5. Clone this repo to to ~/catkin_ws/src
     ```
     git clone git@github.com:ExistentialRobotics/erl_quadrotor_control.git
     ```
6. To use geometric controller, clone this repo to ~/catkin_ws/src
     ```
     git clone git@github.com:KumarRobotics/kr_mav_control.git
     ```
    
    and in this file ```/interfaces/kr_mavros_interface/nodelet_plugin.xml```, change the line ```<library path="lib/libso3cmd_to_mavros_nodelet">``` to ```<library path="lib/libkr_mavros_interface">```.
    
7. Run ```catkin config -DCMAKE_BUILD_TYPE=Release``` and then ``` catkin build```.

8. You may encounter some error while building the above catkin workspace. Jut run ```catkin build``` until all packages are built succesfully.


# Running simulation:
1. Make sure the path "px4_dir" to your PX4 Firmware is updated in erl_quadrotor_control/scripts/launch-common.sh
It should look like this : ```px4_dir = ~/PX4/PX4-Autopilot``` if you cloned PX4/PX4-Autopilot or ```px4_dir = ~/PX4/Firmware``` if you cloned PX4/Firmware. 

2. Run ```launch_offb.sh``` to hover using PX4 controller
```
cd erl_quadrotor_control/scripts
chmod +x launch_offb.sh
./launch_offb.sh
```

You should see the drone takeoff, hover for sometime and then return to landing position. 

If you see a warning messages like : 
<pre>
WARN  [PreFlightCheck] Preflight Fail: Accel #0 uncalibrated
WARN  [PreFlightCheck] Preflight Fail: Accel #1 uncalibrated
WARN  [PreFlightCheck] Preflight Fail: Accel #2 uncalibrated
WARN  [PreFlightCheck] Preflight Fail: Gyro #0 uncalibrated
WARN  [PreFlightCheck] Preflight Fail: Gyro #1 uncalibrated
WARN  [PreFlightCheck] Preflight Fail: Gyro #2 uncalibrated
WARN  [PreFlightCheck] system power unavailable
</pre>

and the drone does not arm, then close the simulation, and perform the following step : 
<pre>rm -rf ~/.ros/eeprom/parameters*</pre>

and then relaunch again. It should work after this. 

- The quadrotor is originally placed at location (x,y,z) = (0,0,0) with orientation (yaw, pitch, roll) = (0,0,0) in local frame. Its job is to go to location (x,y,z) = (0,0,5) with orientation (yaw, pitch, roll) = (pi, 0, 0).

 ![output](/figs/quad_sim.gif)
 
 3. Run ```hover-offb-geoctl.sh``` to hover using geometric controller
```
cd erl_quadrotor_control/scripts
chmod +x hover-offb-geoctl.sh
./hover-offb-geoctl.sh
```
 - The quadrotor is originally placed at location (x,y,z) = (0,0,0) with orientation (yaw, pitch, roll) = (0,0,0) in local frame. Its job is to go to location (x,y,z) = (2,2,5) with orientation (yaw, pitch, roll) = (pi/2, 0, 0).

 ![output](/figs/geoctl_demo_hovering.gif)
 
  4. Run ```tracking-offb-geoctl.sh``` to track a circle using geometric controller
```
cd erl_quadrotor_control/scripts
chmod +x tracking-offb-geoctl.sh
./tracking-offb-geoctl.sh
```
 - The quadrotor is originally placed at location (x,y,z) = (0,0,0) with orientation (yaw, pitch, roll) = (0,0,0) in local frame. Its job is to track a circle of radius 2 meters, centered at (-2, 0, 5) with orientation (yaw, pitch, roll) = (pi/2, 0, 0).

 ![output](/figs/geoctl_demo_tracking_16.gif)

 5. Run ```launch-offb-panel.sh``` to launch the control panel 
 ```
 cd erl_quadrotor_control/scripts
 chmod +x launch-offb-panel.sh
 ./launch-offb-panel.sh
 ```
 ![output](/figs/control_panel_gui.gif)
 
- When exiting, ctrl+c the gazebo terminal on top right and wait for it to clean up before closing terminal
______________________________________________________________________________
# ROS Topics and Services used in this package.

When running simulation, check out ROS topics to see what is happening in the system:
```
rostopic list
rostopic echo <topic_name>
(ctrl+c to exit)
```

ROS Topics [http://wiki.ros.org/mavros]
1. mavros/state - tells current flight mode of the drone [https://dev.px4.io/en/concept/flight_modes.html]
2. mavros/setpoint_position/local - set a target pose in local frame
3. mavros/setpoint_velocity/cmd_vel - set a twist (linear and angular velocity).
4. mavros/setpoint_raw/attitude -  set a attitude target

Services
1. mavros/cmd/arming - arm the drone
2. mavros/set_mode - set the current flight mode
______________________________________________________________________________
# Description of PanelCmd Message.
## Components
| Name | Type | Value |
| --- | --- | --- |
| arm | bool | true (Takeoff), false (Land)
| algorithm | int8 | 0 (None), 1 (PX4), 2 (Geometric)
| trajectory_mode | int8 | 0 (None), 1 (Hover), 2 (Circle), 3 (Setpoint)
| x | float64 | x-axis position [m]
| y | float64 | y-axis position [m]
| z | float64 | z-axis position [m]
| yaw | float64 | yaw angle [deg]




 
