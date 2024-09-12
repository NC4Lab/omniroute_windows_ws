@echo off

:: Run catkin_make BAT file to build the ROS workspace
call run_cmake.bat

:: Run the projection_display ROS node in the background
echo Running projection_display EXE
start /B rosrun projection_operation projection_display_node

:: Run the projection_sender ROS node in the background
echo Running Python projection_sender node
start /B rosrun projection_operation projection_sender.py

:: Prevent terminal from closing immediately
cmd /K

