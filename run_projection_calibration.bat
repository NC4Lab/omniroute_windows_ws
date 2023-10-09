@echo off

:: Run catkin_make BAT file to build the ROS workspace
call run_cmake.bat

:: Run the specific ROS node
echo Running Package EXE
::rosrun goodbuy_world_pkg goodbuy_world_node
::rosrun proj_pkg_test proj_pkg_test_node
rosrun projection_operation projection_calibration_node