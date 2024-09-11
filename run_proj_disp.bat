@echo off

:: Run catkin_make BAT file to build the ROS workspace
call run_cmake.bat

:: Run the projection_display ROS node
echo Running projection_display EXE
rosrun projection_operation projection_display_node

:: Run the projection_sender ROS node
echo Running Python projection_sender node
rosrun projection_operation projection_sender.py