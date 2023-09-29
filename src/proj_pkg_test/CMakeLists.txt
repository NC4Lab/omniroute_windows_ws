
cmake_minimum_required(VERSION 3.0.2)
project(proj_pkg_test)

# Compile as C++11 for Windows
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  cv_bridge  # Added for OpenCV support
)

# Find OpenCV
find_package(OpenCV REQUIRED) 

# Find pugixml
find_package(pugixml REQUIRED)

# Catkin specific configuration
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs cv_bridge 
  DEPENDS OpenCV  # Added for OpenCV
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

# Set the DevIL directory from environment variables
set(DevIL_DIR $ENV{DevIL_DIR})
if(EXISTS ${DevIL_DIR})
  message(STATUS "[ENV]: DevIL_DIR set to: ${DevIL_DIR}")
else()
  message(FATAL_ERROR "[ENV]: GLFWDevIL_DIR_DIR not set")
endif()

# Add the include directories
include_directories(
  include                 # Project include directory
  ${catkin_INCLUDE_DIRS}  # Catkin include directory
  ${GLAD_DIR}/include     # GLAD include directory
  ${GLFW_DIR}/include     # GLFW include directory
  ${DevIL_DIR}/include    # DevIL include directory
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
  ${OpenCV_LIBRARIES}
  ${PUGIXML_LIBRARIES}
  ${GLFW_DIR}/lib-vc2019/glfw3.lib
  ${DevIL_DIR}/lib/x64/Release/DevIL.lib
  ${DevIL_DIR}/lib/x64/Release/ILU.lib
  ${DevIL_DIR}/lib/x64/Release/ILUT.lib
)

# Install targets (this is just an example, adjust paths as needed)
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Copy DevIL DLLs to the executable directory so they are accessible at runtime
add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different
                   "${DevIL_DIR}/lib/x64/Release/DevIL.dll"
                   "${DevIL_DIR}/lib/x64/Release/ILU.dll"
                   "${DevIL_DIR}/lib/x64/Release/ILUT.dll"
                   $<TARGET_FILE_DIR:${PROJECT_NAME}_node>)

# Print out the values of the environment variables
message("[ENV]: PROJECT_NAME set to: ${PROJECT_NAME}")
message("[ENV]: CMAKE_SOURCE_DIR set to: ${CMAKE_SOURCE_DIR}")
message("[ENV]: CATKIN_PACKAGE_LIB_DESTINATION set to: ${CATKIN_PACKAGE_LIB_DESTINATION}")
message("[ENV]: GLAD_DIR set to: ${GLAD_DIR}")
message("[ENV]: GLAD_SRC set to: $ENV{GLAD_SRC}") 
message("[ENV]: GLFW_DIR set to: ${GLFW_DIR}")
message("[ENV]: DevIL_DIR set to: ${DevIL_DIR}")
message("[ENV]: catkin_LIBRARIES set to: $ENV{catkin_LIBRARIES}")