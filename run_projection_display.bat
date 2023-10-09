@echo off

:: Run catkin_make BAT file to build the ROS workspace
call run_cmake.bat

:: Run the specific ROS node
echo Running projection_display EXE
rosrun projection_operation projection_display_node