# capstone_welding_analysis
Capstone Project 2023

# To run the Scan Node:
1. Open a terminal and load the ROS environment
2. Navigate to the ros2_ws directory
3. Compile the workspace using:
```bash
colcon build --symlink-install --merge-install
```
4. Source the workspace using:
``` bash
call install\local_setup.bat
```
5. Run the node using:
```bash
ros2 run ranger scan <file_path>
```
- Where:
    - `<file_path>` is the directory of the PRM file to be used for the scan.