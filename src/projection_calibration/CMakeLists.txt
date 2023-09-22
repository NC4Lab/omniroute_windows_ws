cmake_minimum_required(VERSION 3.0.2)
project(projection_calibration)

# Compile as C++11
add_compile_options(-std=c++11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_generation
  cv_bridge
)

# Find other dependencies
find_package(OpenCV REQUIRED)

# Catkin specific configuration
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs cv_bridge
  DEPENDS OpenCV
)

# Set path to top loevel library directory
set(EXT_LIB_DIR "${CMAKE_SOURCE_DIR}/../lib")

# Build
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  
  # Include local library headers
  ${EXT_LIB_DIR}/glad/include
  ${EXT_LIB_DIR}/glfw/include
  ${EXT_LIB_DIR}/DevIL/include
  ${EXT_LIB_DIR}/pugixml
  ${EXT_LIB_DIR}/glm  
)

# Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/projection_calibration.cpp)

# Add local libraries
add_library(GLAD STATIC ${EXT_LIB_DIR}/glad/src/glad.c)
target_link_libraries(GLAD ${CMAKE_DL_LIBS})

# Link against local and system libraries
target_link_libraries(${PROJECT_NAME}_node
  GLAD
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EXT_LIB_DIR}/glfw/lib/glfw3.lib
  ${EXT_LIB_DIR}/DevIL/lib/DevIL.lib
  ${EXT_LIB_DIR}/DevIL/lib/ILU.lib
  ${EXT_LIB_DIR}/DevIL/lib/ILUT.lib
)

# Copy local glfw3 DLLs to output directory
add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${EXT_LIB_DIR}/glfw/bin/glfw3.dll
  $<TARGET_FILE_DIR:${PROJECT_NAME}_node>)

# Copy local DevIL DLLs to output directory
add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${EXT_LIB_DIR}/DevIL/bin/*.dll
  $<TARGET_FILE_DIR:${PROJECT_NAME}_node>)


## Mark executables for installation
install(TARGETS ${PROJECT_NAME}_node 
  DESTINATION ${PROJECT_BINARY_DIR}/lib
)