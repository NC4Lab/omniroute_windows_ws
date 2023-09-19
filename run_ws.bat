@echo off  :: Turn off command echoing to clean up the output

:: Open a new terminal and run roscore
::start cmd /k "call C:\opt\ros\noetic\x64\setup.bat && roscore"

:: Pause to give roscore time to start up
::timeout /t 5

:: Source the ROS setup file
::call C:\opt\ros\noetic\x64\setup.bat

:: Navigate to your ROS workspace
cd C:\nc4_code_base\omniroute_windows_ws

:: Source the workspace to make sure the new build is recognized
devel\setup.bat

:: Build the ROS workspace using catkin_make
call catkin_make

:: Run the specific ROS node
call rosrun goodbuy_world_pkg goodbuy_world_pkg_node

