:: Load the ROS environment
call c:\opt\ros\foxy\x64\setup.bat
:: Load local env
call <full_path_to_local_setup.bat>
:: Set the ROS_DOMAIN_ID
set ROS_DOMAIN_ID=20
:: Set the RMW_IMPLEMENTATION
set "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
:: Source the virtual python environment
call <full_path_to_activate.bat>
:: Add the site-packages from the virtual environment to the PYTHONPATH
set PYTHONPATH=%PYTHONPATH%;<full_path_to_virtual_env_site-packages>
