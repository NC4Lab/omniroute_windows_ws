
cmake_minimum_required(VERSION 3.0.2)
project(proj_pkg_test)
message("[ENV]: PROJECT_NAME set to: ${PROJECT_NAME}")
message("[ENV]: CMAKE_SOURCE_DIR set to: ${CMAKE_SOURCE_DIR}")

# Compile as C++11 for Windows
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# # Find OpenGL
# find_package(OpenGL REQUIRED)
# if(OpenGL_FOUND)
#   message(STATUS "[ENV]: OpenGL found")
#   message(STATUS "[ENV]: OpenGL libraries are in: ${OPENGL_LIBRARIES}") 
# else()
#   message(FATAL_ERROR "[ENV]: OpenGL not found")
# endif()

# # Get the OpenGL include directory
# get_target_property(OpenGL_INCLUDE_DIR OpenGL::GL INTERFACE_INCLUDE_DIRECTORIES)
# message(STATUS "[ENV]: OpenGL include directory is: ${OpenGL_INCLUDE_DIR}")
# find_library(OPENGL32_LIBRARY NAMES opengl32)
# find_library(GLU32_LIBRARY NAMES glu32)
# message(STATUS "[ENV]: opengl32 library path: ${OPENGL32_LIBRARY}")
# message(STATUS "[ENV]: glu32 library path: ${GLU32_LIBRARY}")

# Catkin specific configuration
catkin_package(
  CATKIN_DEPENDS roslib roscpp std_msgs cv_bridge
  DEPENDS OpenCV  # Added for OpenCV
)

# Check for GLAD directory environment variable
if(EXISTS $ENV{GLAD_DIR})
  message(STATUS "[ENV]: GLAD_DIR set to: $ENV{GLAD_DIR}")
else()
  message(FATAL_ERROR "[ENV]: GLAD_DIR not set")
endif()

# Check for GLFW directory environment variable
if(EXISTS $ENV{GLFW_DIR})
  message(STATUS "[ENV]: GLFW_DIR set to: $ENV{GLFW_DIR}")
else()
  message(FATAL_ERROR "[ENV]: GLFW_DIR not set")
endif()

# Check for DevIL directory environment variable
if(EXISTS $ENV{DevIL_DIR})
  message(STATUS "[ENV]: DevIL_DIR set to: $ENV{DevIL_DIR}")
else()
  message(FATAL_ERROR "[ENV]: GLFWDevIL_DIR_DIR not set")
endif()

# Check for ROS NOETIC directory environment variable
if(EXISTS $ENV{ROS_NOETIC_DIR})
  message(STATUS "[ENV]: ROS_NOETIC_DIR set to: $ENV{ROS_NOETIC_DIR}")
else()
  message(FATAL_ERROR "[ENV]: ROS_NOETIC_DIR not set")
endif()

# Manually set OpenGL paths based on ROS_NOETIC_DIR
set(OpenGL_LIBRARY "$ENV{ROS_NOETIC_DIR}/Lib/OpenGL32.Lib")
set(OpenGL_INCLUDE_DIR "$ENV{ROS_NOETIC_DIR}/include")

# Check if the library exists
if(EXISTS ${OpenGL_LIBRARY})
  message(STATUS "[ENV]: OpenGL library found at ${OpenGL_LIBRARY}")
else()
  message(FATAL_ERROR "[ENV]: OpenGL library not found at ${OpenGL_LIBRARY}")
endif()

# Check if the include directory exists
if(EXISTS ${OpenGL_INCLUDE_DIR})
  message(STATUS "[ENV]: OpenGL include directory found at ${OpenGL_INCLUDE_DIR}")
else()
  message(FATAL_ERROR "[ENV]: OpenGL include directory not found at ${OpenGL_INCLUDE_DIR}")
endif()

# Add the include directories
include_directories(
  include                     # Project include directory
  ${catkin_INCLUDE_DIRS}      # Catkin include directory
  $ENV{GLAD_DIR}/include      # GLAD include directory
  $ENV{GLFW_DIR}/include      # GLFW include directory
  $ENV{DevIL_DIR}/include     # DevIL include directory
  $ENV{ROS_NOETIC_DIR}/include   # PugiXML include directory
  ${OpenGL_INCLUDE_DIR}
)

# Add the GLAD source files
set(GLAD_SRC $ENV{GLAD_DIR}/src/glad.c)
if(EXISTS ${GLAD_SRC})
  message(STATUS "[ENV]: GLAD_SRC set to: ${GLAD_SRC}")
else()
  message(FATAL_ERROR "[ENV]: GLAD_SRC not set")
endif()

# Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cpp ${GLAD_SRC})

# Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OPENGL_LIBRARIES}
  $ENV{GLFW_DIR}/lib-vc2019/glfw3.lib
  $ENV{DevIL_DIR}/lib/x64/Release/DevIL.lib
  $ENV{DevIL_DIR}/lib/x64/Release/ILU.lib
  $ENV{DevIL_DIR}/lib/x64/Release/ILUT.lib
  $ENV{ROS_NOETIC_DIR}/Lib/pugixml.lib
  ${OpenGL_LIBRARY}
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