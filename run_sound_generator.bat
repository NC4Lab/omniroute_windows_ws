@echo off

:: Run catkin_make BAT file to build the ROS workspace
call run_cmake.bat

:: Run the specific ROS node
echo Running sound_generator 
rosrun projection_operation sound_generator.py