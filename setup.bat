:: Load the ROS environment
call c:\opt\ros\foxy\x64\setup.bat
:: Load local env
call ros2_ws\install\setup.bat
:: Source the virtual python environment
call venv\Scripts\activate.bat
:: Set the ROS_DOMAIN_ID
set ROS_DOMAIN_ID=20
:: Set the RMW_IMPLEMENTATION
set "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
