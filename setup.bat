:: Load the ROS environment
call c:\opt\ros\foxy\x64\setup.bat
:: Load local env
call C:\Users\magok\source\repos\crcz25\capstone_welding_analysis\ros2_ws\install\local_setup.bat
:: Set the ROS_DOMAIN_ID
set ROS_DOMAIN_ID=20
:: Set the RMW_IMPLEMENTATION
set RMW_IMPLEMENTATION "rmw_cyclonedds_cpp"
:: Source the virtual python environment
call ./env/Scripts/activate.bat
:: Add the site-packages to the PYTHONPATH
set PYTHONPATH=%PYTHONPATH%;%CD%\env\Lib\site-packages
