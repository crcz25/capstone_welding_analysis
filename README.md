# capstone_welding_analysis

Capstone Project 2023

This repository contains the code for the capstone project of the 2023 class for DMS Research Group at the University of Turku. The project is a multi-disciplinary project that aims to provide a solution for weld monitoring where a weld bead measurment is retrived using a laser and a SICK Ranger E55 camera. The objective is to find welding errors and measure them.

# Project Structure

The code is divided in two parts: the ROS2 node that controls the SICK Ranger E55 camera and the laser and the code that processes the data.

The directory `ros2_ws` contains the ROS2 packages:

- `ranger`: contains the code for the ROS2 node that controls the SICK Ranger E55 camera and the laser, it publishes two topics `/range` and `/intensity` which are of type `interfaces/msg/Scan`.
- `interfaces`: contains the message definition for the ROS2 topics.

# To recreate the environment:

The necessary documentation and/or executables are provided in Seafile under our UTU account. Due to compatibility issues, it is recommended to use a Windows 10/11 machine. The complete list of instructions are provided in the document *how_to_recreate_env.pdf*. Below is a summary of the steps required to recreate the environment.

1. Install Visual Studio 2019 Community Edition (how_to_install_vs_studio_2019.pdf).
2. Install Visual Studio 2010 (some dll files are required that are only available with this version) (how_to_install_vs_studio_2010.pdf).
3. Install ROS2 Foxy Fitzroy
   - Follow the instructions using this tutorial from MS IOT - ROS on Windows: https://ms-iot.github.io/ROSOnWindows/GettingStarted/SetupRos2.html
4. Install SICK Ranger Studio 5.1 and eBus Driver (how_to_install_sick_api.pdf & how_to_connect_to_camera.pdf).
5. Install and configure python 3.8.3 (how_to_install_python_3.8.3.pdf)
6. Now you can build your workspace and run the software.

## Build the ROS environment:

1. Open a _Developer Command Prompt for VS 2019_ terminal and load the environment using the script `setup.bat`. This script will load the ROS2 environment, the local setup of the workspace, set the RMW Implementation, set the ROS Domain ID and add the site-packages directory of the virtual environment to the PYTHONPATH.
2. Navigate to the ros2_ws directory where you cloned the repository.
    ```bash
    cd <path_to_ros2_ws>
    ```
3. Compile the workspace using:
    ```bash
    colcon build --symlink-install --merge-install
    ```

# To run the client (GUI):

1. Open a _Developer Command Prompt for VS 2019_ terminal and load the environment using the script `setup.bat`. This script will load the ROS2 environment, the local setup of the workspace, set the RMW Implementation, set the ROS Domain ID and add the site-packages directory of the virtual environment to the PYTHONPATH.

- For Windows:
  ```bash
  call setup.bat
  ```

1. Run the client using:

    ```bash
    python python_gui_py/App.py
    ```

# To run the Scan Node:

This node is used to control the SICK Ranger E55 camera and the laser. It publishes two topics `/range` and `/intensity` which are of type `interfaces/msg/Scan`. It only works when the device is connected to the camera via ethernet.

1. Open a _Developer Command Prompt for VS 2019_ terminal, load the ROS2 Environment, and set the RMW Implementation and the ROS Domain ID using the following commands:
    ```bash
    call c:\opt\ros\foxy\x64\setup.bat
    set ROS_DOMAIN_ID=20
    set "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    ```
2. Load the workspace using the following command:
    ```bash
    call ros2_ws\install\setup.bat
    ```
3. Test the connection to the camera and run the node using:

  ```bash
  ros2 run ranger scan <ip_address> <file_path> <type>
  ```

- Where:

  - `<ip_address>` is the IP address of the SICK Ranger Camera. For example, "192.168.0.92".
  - `<file_path>` is the full path of the PRM file to be used for the scan. For example, "C:\\\Users\\\user\\\Documents\\\RangerEHi3D.prm"
  - `<type>` is the type of scan to be performed. Options are `Measurement` or `Image`.

  **Note:** For now, the scan node only supports the `Measurement` scan type.

# To save a rosbag:

We recommend recording a rosbag to test the code without the need of the camera. The rosbag can then be played back to test the GUI and the processing code.

1. Open a _Developer Command Prompt for VS 2019_ terminal, load the ROS2 Environment, and set the RMW Implementation and the ROS Domain ID using the following commands:
    ```bash
    call c:\opt\ros\foxy\x64\setup.bat
    set ROS_DOMAIN_ID=20
    set "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    ```
2. Load the workspace using the following command:
    ```bash
    call ros2_ws\install\setup.bat
    ```
3. Run the following command to save the rosbag:

    ```bash
    ros2 bag record -a
    ```

3. To stop the recording, press `Ctrl + C` in the terminal where the rosbag is being recorded.

# To play a rosbag:

1. Open a _Developer Command Prompt for VS 2019_ terminal, load the ROS2 Environment, and set the RMW Implementation and the ROS Domain ID using the following commands:
    ```bash
    call c:\opt\ros\foxy\x64\setup.bat
    set ROS_DOMAIN_ID=20
    set "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    ```
2. Load the workspace using the following command:
    ```bash
    call call ros2_ws\install\setup.bat
    ```
3. Run the following command to play the rosbag:

    ```bash
    ros2 bag play <path_to_rosbag> -l
    ```

- The flag `-l` is used to loop the rosbag indefinitely.

4. To stop the playback, press `Ctrl + C` in the terminal where the rosbag is being played.

# Common Issues:

- Verify that the paths set in the `setup.bat` and `setup.sh` scripts are correct and correspond to the paths in your system.
- If you get the error `ImportError: Matplotlib requires numpy>=1.20; you have 1.19.2'`, load only the ROS2 environment and issue the next command to upgrade numpy:

```bash
python -m pip install --upgrade numpy==1.24.4
```
