@echo off  :: Command Echoing Off

:: Source the ROS setup file
call C:\opt\ros\noetic\x64\setup.bat

:: Check if 'devel' directory exists
if not exist "%cd%\devel" (
    :: Run catkin_make
    echo Running catkin_make
    catkin_make
)

:: Source the workspace to make sure the new build is recognized
echo Sourcing ROS workspace.
call devel\setup.bat

:: Check if the user has sourced the ROS workspace
if not defined ROS_PACKAGE_PATH (
    echo ERROR: Sourcing ROS workspace failed.
    call devel\setup.bat
)