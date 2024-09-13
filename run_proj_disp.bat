@echo off

:: Run catkin_make BAT file to build the ROS workspace
call run_cmake.bat

:: Run the projection_display ROS node 
echo Running projection_display EXE
rosrun projection_operation projection_display_node

@REM :: Run the projection_display ROS node in the background
@REM echo Running projection_display EXE
@REM start /B rosrun projection_operation projection_display_node

@REM :: Run the projection_sender ROS node in the background
@REM echo Running Python projection_sender node
@REM start /B rosrun projection_operation projection_sender.py

@REM :: Prevent terminal from closing immediately
@REM cmd /K