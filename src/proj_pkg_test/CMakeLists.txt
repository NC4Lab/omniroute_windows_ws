cmake_minimum_required(VERSION 3.0.2)
project(proj_pkg_test)

# Compile as C++11
add_compile_options(-std=c++11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# Find GLAD directories using the GLAD_DIR environment variable
find_path(GLAD_INCLUDE_DIR NAMES glad/glad.h HINTS $ENV{GLAD_DIR}/include)
find_path(GLAD_SRC_DIR NAMES glad.c HINTS $ENV{GLAD_DIR}/src)

# Find GLFW directories using the GLFW_DIR environment variable
find_path(GLFW_INCLUDE_DIR NAMES GLFW/glfw3.h HINTS $ENV{GLFW_DIR}/include)
find_library(GLFW_LIBRARY NAMES glfw3.lib HINTS $ENV{GLFW_DIR}/lib)

## Check if GLFW libraries
if(GLFW_LIBRARY)
  message(STATUS "[LIBRARY]: Found GLFW_LIBRARY: ${GLFW_LIBRARY}")
else()
  message(FATAL_ERROR "[LIBRARY]: GLFW_LIBRARY library not found")
endif()

# Include GLAD and GLFW directories
include_directories(
  ${GLAD_INCLUDE_DIR}
  ${GLFW_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/proj_pkg_test.cpp ${GLAD_SRC_DIR}/glad.c)

# Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${GLFW_LIBRARY}
  ${catkin_LIBRARIES}
)

## Print out the values of the environment variables
message("[ENV]: PROJECT_NAME set to: ${PROJECT_NAME}")
#message("[ENV]: CMAKE_SOURCE_DIR set to: ${CMAKE_SOURCE_DIR}")
message("[ENV]: GLAD_DIR set to: ${GLAD_DIR}")
message("[ENV]: GLFW_DIR set to: ${GLFW_DIR}")
message("[ENV]: OpenCV_INCLUDE_DIRS set to: ${OpenCV_INCLUDE_DIRS}")
message("[ENV]: OpenCV_LIBRARIES set to: ${OpenCV_LIBRARIES}")
message("[ENV]: OpenCV_LIBS set to: ${OpenCV_LIBS}")
message("[ENV]: OpenCV_DIR set to: ${OpenCV_DIR}")