# capstone_welding_analysis
Capstone Project 2023

This repository contains the code for the capstone project of the 2023 class for DMS Research Group at the University of Turku. The project is a multi-disciplinary project that aims to provide a solution for weld monitoring where a weld bead measurment is retrived using a laser and a SICK Ranger E55 camera. The objective is to find welding errors and measure them.

# Project Structure
The code is divided in two parts: the ROS2 node that controls the SICK Ranger E55 camera and the laser and the code that processes the data.

The directory `ros2_ws` contains the ROS2 packages:
- `ranger`: contains the code for the ROS2 node that controls the SICK Ranger E55 camera and the laser, it publishes two topics `/range` and `/intensity` which are of type `interfaces/msg/Scan`.
- `interfaces`: contains the message definition for the ROS2 topics.


# To recreate the environment:
The necessary documentation and/or executables are provided in Seafile under our UTU account. Due to compatibility issues, it is recommended to use a Windows 10 machine. The following steps are required to recreate the environment:
1. Install Visual Studio 2019/2022 Community Edition (how_to_install_vs_studio_2022.pdf).
2. Install Visual Studio 2010 (some dll files are required that are only available with this version) (how_to_install_vs_studio_2010.pdf).
3. Install ROS2 Foxy Fitzroy
    - Follow the instructions using this tutorial from MS IOT - ROS on Windows: https://ms-iot.github.io/ROSOnWindows/GettingStarted/SetupRos2.html
    - **NB:** The tutorial also includes instructions on how to install Visual Studio 2019/2022 Community Edition.
4. Install SICK Ranger Studio 5.1 and eBus Driver (how_to_install_sick_api.pdf & how_to_connect_to_camera.pdf).

# To run the Scan Node:
1. Open a _Developer Command Prompt for VS 2019_ terminal and load the ROS environment
```bash
call c:\opt\ros\foxy\x64\setup.bat
```
2. Set the ROS Domain ID, a number between 0-100 (all terminals must have the same Domain ID)
```bash
set ROS_DOMAIN_ID=##
```
3. Setup the RMW Implementation
```bash
set RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
4. Navigate to the ros2_ws directory where you cloned the repository.
```bash
cd <path_to_ros2_ws>
```
5. Compile the workspace using:
```bash
colcon build --symlink-install --merge-install
```
4. Source the workspace using:
``` bash
call install\local_setup.bat
```
5. Run the node using:
```bash
ros2 run ranger scan <ip_address> <file_path> <type>
```
- Where:
    - `<ip_address>` is the IP address of the SICK Ranger Camera. For example, "192.168.0.92".
    - `<file_path>` is the full path of the PRM file to be used for the scan. For example, "C:\\\Users\\\user\\\Documents\\\RangerEHi3D.prm"
    - `<type>` is the type of scan to be performed. Options are `Measurement` or `Image`.

  **Note:** For now, the scan node only supports the `Measurement` scan type.