@echo off

:: Run catkin_make BAT file to build the ROS workspace
call run_cmake.bat

:: Run the projection_calibration ROS node
echo Running Package EXE
rosrun projection_operation projection_calibration_node