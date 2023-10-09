@echo off  :: Command Echoing Off

@echo off  :: Command Echoing Off

@REM :: Open a new terminal and run roscore
@REM start cmd /k "call C:\opt\ros\noetic\x64\setup.bat && roscore"

@REM :: Pause to give roscore time to start up
@REM timeout /t 5

@REM :: Source the ROS setup file
@REM call C:\opt\ros\noetic\x64\setup.bat

:: Source the ROS setup file
call C:\opt\ros\noetic\x64\setup.bat

:: Run catkin_make to build the ROS workspace
echo Running catkin_make
catkin_make

:: Source the workspace to make sure the new build is recognized
echo Sourcing ROS workspace.
call devel\setup.bat