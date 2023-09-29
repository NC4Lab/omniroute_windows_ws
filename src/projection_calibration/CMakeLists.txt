cmake_minimum_required(VERSION 3.0.2)
project(projection_calibration)

# Compile as C++11
# add_compile_options(-std=c++11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_generation
  cv_bridge  # Added for OpenCV support
)

# Find other dependencies
find_package(OpenCV REQUIRED)  # Added for OpenCV
find_package(glfw3 REQUIRED)  # Added for GLFW

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES projection_calibration
  CATKIN_DEPENDS roscpp roslib std_msgs cv_bridge  # Added cv_bridge for OpenCV
  DEPENDS OpenCV glfw3 Boost  # Added OpenCV, GLFW, and Boost
)

###########
## Build ##
###########

# TODO - README.md should ask user to install these libraries and create environment variables pointing to them.
set(glfw3_DIR "C:/Program Files (x86)/GLFW/lib/cmake/glfw3/")
set(IL_INCLUDE_DIR "C:/Program Files (x86)/IL/include")
set(IL_LIB_DIR "C:/Program Files (x86)/IL/lib/x64/Release/")
set(glm_DIR "C:/Program Files (x86)/glm/cmake/glm/")

## Declare a C++ library
# Need to explicetly add GLAD library for linking
add_library(GLAD lib/glad/glad.c)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/lib
  ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/lib/glad                # Added for GLAD
  ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/lib/glm                 # Added for GLM
  ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/lib/pugixml             # Added for PugiXML
  ${catkin_INCLUDE_DIRS}  # Added for ROS
  ${OpenCV_INCLUDE_DIRS}  # Added for OpenCV
  ${GLFW_INCLUDE_DIRS}    # Added for GLFW
  ${glfw3_INCLUDE_DIRS}   # Added for GLFW
  ${IL_INCLUDE_DIR}       # Added for DevIL
)

## Using local DLL for DevIL library
find_library(ILUT_LIBRARY NAMES ILUT.lib PATHS "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/dll")
find_library(IL_LIBRARY NAMES DevIL ILU ILUT.lib PATHS "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/dll")
find_library(ILU_LIBRARY NAMES ILU PATHS "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/dll")

## Check if DevIL libraries
if(IL_LIBRARY)
  message(STATUS "[LIBRARY]: Found IL library: ${IL_LIBRARY}")
else()
  message(FATAL_ERROR "[LIBRARY]: IL library not found")
endif()
if(ILU_LIBRARY)
  message(STATUS "[LIBRARY]: Found ILU library: ${ILU_LIBRARY}")
else()
  message(FATAL_ERROR "[LIBRARY]: ILU library not found")
endif()

## Print out the values of the environment variables
message("[ENV]: PROJECT_NAME set to: ${PROJECT_NAME}")
message("[ENV]: CMAKE_SOURCE_DIR set to: ${CMAKE_SOURCE_DIR}")
message("[ENV]: glfw3_DIR set to: ${glfw3_DIR}")
message("[ENV]: IL_INCLUDE_DIR set to: $ENV{IL_INCLUDE_DIR}") 
message("[ENV]: IL_LIB_DIR set to: $ENV{IL_LIB_DIR}")
message("[ENV]: glm_DIR set to: ${glm_DIR}")
message("[ENV]: IL_LIBRARY set to: ${IL_LIBRARY}")
message("[ENV]: ILU_LIBRARY set to: ${ILU_LIBRARY}")
message("[ENV]: ILUT_LIBRARY set to: ${ILUT_LIBRARY}")
message("[ENV]: OpenCV_INCLUDE_DIRS set to: ${OpenCV_INCLUDE_DIRS}")
message("[ENV]: OpenCV_LIBRARIES set to: ${OpenCV_LIBRARIES}")
message("[ENV]: OpenCV_LIBS set to: ${OpenCV_LIBS}")
message("[ENV]: OpenCV_DIR set to: ${OpenCV_DIR}")

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node 
  src/projection_calibration.cpp
  lib/glad/glad.c
	lib/glad/glad.h
  lib/glm/glm.hpp
  lib/pugixml/pugixml.cpp
  lib/pugixml/pugixml.hpp
)

## Manually copy DLLs to the executable's directory post-build
# Copy DevIL.dll 
add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different
                   ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/dll/DevIL.dll
                   $<TARGET_FILE_DIR:${PROJECT_NAME}_node>)
# Copy ILU.dll
add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different
                   ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/dll/ILU.dll
                   $<TARGET_FILE_DIR:${PROJECT_NAME}_node>)
# Copy ILUT.dll
add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different
                   ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/dll/ILUT.dll
                   $<TARGET_FILE_DIR:${PROJECT_NAME}_node>)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# Specify libraries to link against
target_link_libraries(${PROJECT_NAME}_node 
  GLAD
  glfw 
  ${catkin_LIBRARIES}
  ${IL_LIBRARY} ${ILU_LIBRARY} ${ILUT_LIBRARY}	
  ${OpenCV_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}_node 
  DESTINATION ${PROJECT_BINARY_DIR}/lib
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_projection_calibration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)