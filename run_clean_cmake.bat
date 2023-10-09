@echo off  :: Command Echoing Off

:: Source the ROS setup file
echo Sourcing ROS setup file
call C:\opt\ros\noetic\x64\setup.bat

:: Delete the build and devel folders
echo Deleting build and devel folders
rmdir /s /q build
rmdir /s /q devel

:: Run catkin_make to build the ROS workspace
echo Running catkin_make
catkin_make

:: Source the workspace to make sure the new build is recognized
echo Sourcing ROS workspace.
call devel\setup.bat