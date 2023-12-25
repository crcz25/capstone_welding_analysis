:: Load the ROS environment
call c:\opt\ros\foxy\x64\setup.bat
:: Load local env
call ros2_ws\install\setup.bat
:: Set the site-packages path to the PYTHONPATH
set Path=C:\Python38\Scripts\;C:\Python38\;%Path%
set PYTHONPATH=venv\Lib\site-packages;%PYTHONPATH%
:: Source the virtual python environment
call venv\Scripts\activate.bat
:: Set the ROS_DOMAIN_ID
set ROS_DOMAIN_ID=20
:: Set the RMW_IMPLEMENTATION
set "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"