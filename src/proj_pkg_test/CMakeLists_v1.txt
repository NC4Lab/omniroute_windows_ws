
cmake_minimum_required(VERSION 3.0.2)
project(proj_pkg_test)

# Compile as C++11
add_compile_options(-std=c++11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# Catkin specific configuration
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

# Set the GLAD directory from environment variables
set(GLAD_DIR $ENV{GLAD_DIR})
if(EXISTS ${GLAD_DIR})
  message(STATUS "[ENV]: GLAD_DIR set to: ${GLAD_DIR}")
else()
  message(FATAL_ERROR "[ENV]: GLAD_DIR not set")
endif()

# Set the GLFW directory from environment variables
set(GLFW_DIR $ENV{GLFW_DIR})
if(EXISTS ${GLFW_DIR})
  message(STATUS "[ENV]: GLFW_DIR set to: ${GLFW_DIR}")
else()
  message(FATAL_ERROR "[ENV]: GLFW_DIR not set")
endif()

# Add the GLAD and GLFW include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GLAD_DIR}/include
  ${GLFW_DIR}/include
)

# Add the GLAD source files
set(GLAD_SRC ${GLAD_DIR}/src/glad.c)
if(EXISTS ${GLAD_SRC})
  message(STATUS "[ENV]: GLAD_SRC set to: ${GLAD_SRC}")
else()
  message(FATAL_ERROR "[ENV]: GLAD_SRC not set")
endif()

# Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/proj_pkg_test.cpp ${GLAD_SRC})

# Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${GLFW_DIR}/lib-vc2019/glfw3.lib
)

# Install targets (this is just an example, adjust paths as needed)
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Print out the values of the environment variables
message("[ENV]: PROJECT_NAME set to: ${PROJECT_NAME}")
message("[ENV]: CATKIN_PACKAGE_LIB_DESTINATION set to: ${CATKIN_PACKAGE_LIB_DESTINATION}")
message("[ENV]: GLAD_DIR set to: ${GLAD_DIR}")
message("[ENV]: GLAD_SRC set to: $ENV{GLAD_SRC}") 
message("[ENV]: GLFW_DIR set to: ${GLFW_DIR}")
message("[ENV]: catkin_LIBRARIES set to: $ENV{catkin_LIBRARIES}")