@echo off  :: Turn off command echoing to clean up the output

@REM :: Open a new terminal and run roscore
@REM start cmd /k "call C:\opt\ros\noetic\x64\setup.bat && roscore"

@REM :: Pause to give roscore time to start up
@REM timeout /t 5

@REM :: Source the ROS setup file
@REM call C:\opt\ros\noetic\x64\setup.bat

:: Navigate to your ROS workspace
cd C:\nc4_code_base\omniroute_windows_ws

:: Build the ROS workspace
catkin_make

:: Source the workspace to make sure the new build is recognized
call devel\setup.bat

:: Run the specific ROS node
::rosrun goodbuy_world_pkg goodbuy_world_node
rosrun projection_calibration projection_calibration_node

