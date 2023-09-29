
cmake_minimum_required(VERSION 3.0.2)
project(proj_pkg_test)
message("[ENV]: PROJECT_NAME set to: ${PROJECT_NAME}")
message("[ENV]: CMAKE_SOURCE_DIR set to: ${CMAKE_SOURCE_DIR}")

# # Compile as C++11 for Windows
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  std_msgs
  cv_bridge  # Added for OpenCV support
)
if(catkin_FOUND)
  message(STATUS "[ENV]: catkin found")
else()
  message(FATAL_ERROR "[ENV]: catkin not found")
endif()

# Find OpenCV
find_package(OpenCV REQUIRED) 
if(OpenCV_FOUND)
  message(STATUS "[ENV]: OpenCV found")
else()
  message(FATAL_ERROR "[ENV]: OpenCV not found")
endif()

# Catkin specific configuration
catkin_package(
  CATKIN_DEPENDS roslib roscpp std_msgs cv_bridge
  DEPENDS OpenCV  # Added for OpenCV
)

# Set the GLAD directory from environment variables
if(EXISTS $ENV{GLAD_DIR})
  message(STATUS "[ENV]: GLAD_DIR set to: $ENV{GLAD_DIR}")
else()
  message(FATAL_ERROR "[ENV]: GLAD_DIR not set")
endif()

# Set the GLFW directory from environment variables
if(EXISTS $ENV{GLFW_DIR})
  message(STATUS "[ENV]: GLFW_DIR set to: $ENV{GLFW_DIR}")
else()
  message(FATAL_ERROR "[ENV]: GLFW_DIR not set")
endif()

# Set the DevIL directory from environment variables
if(EXISTS $ENV{DevIL_DIR})
  message(STATUS "[ENV]: DevIL_DIR set to: $ENV{DevIL_DIR}")
else()
  message(FATAL_ERROR "[ENV]: GLFWDevIL_DIR_DIR not set")
endif()

# Add the include directories
include_directories(
  include                   # Project include directory
  ${catkin_INCLUDE_DIRS}    # Catkin include directory
  $ENV{GLAD_DIR}/include       # GLAD include directory
  $ENV{GLFW_DIR}/include       # GLFW include directory
  $ENV{DevIL_DIR}/include      # DevIL include directory
  $ENV{PUGIXML_DIR}/include # PugiXML include directory
)

# Add the GLAD source files
set(GLAD_SRC $ENV{GLAD_DIR}/src/glad.c)
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
  $ENV{GLFW_DIR}/lib-vc2019/glfw3.lib
  $ENV{DevIL_DIR}/lib/x64/Release/DevIL.lib
  $ENV{DevIL_DIR}/lib/x64/Release/ILU.lib
  $ENV{DevIL_DIR}/lib/x64/Release/ILUT.lib
  $ENV{PUGIXML_DIR}/Lib/pugixml.lib
)

# Install targets 
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Copy DevIL DLLs to the executable directory so they are accessible at runtime
add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different
                   "$ENV{DevIL_DIR}/lib/x64/Release/DevIL.dll"
                   "$ENV{DevIL_DIR}/lib/x64/Release/ILU.dll"
                   "$ENV{DevIL_DIR}/lib/x64/Release/ILUT.dll"
                   $<TARGET_FILE_DIR:${PROJECT_NAME}_node>)

