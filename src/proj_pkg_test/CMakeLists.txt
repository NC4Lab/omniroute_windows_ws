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
# Check and log the catkin variables
if(catkin_FOUND)
  message(STATUS "[ENV]: catkin found")
else()
  message(FATAL_ERROR "[ENV]: catkin not found")
endif()
if(NOT catkin_LIBRARIES)
  message(FATAL_ERROR "[ENV]: catkin_LIBRARIES is not populated")
else()
  message(STATUS "[ENV]: catkin_LIBRARIES is populated")
endif()
if(NOT catkin_INCLUDE_DIRS)
  message(FATAL_ERROR "[ENV]: catkin_INCLUDE_DIRS is not populated")
else()
  message(STATUS "[ENV]: catkin_INCLUDE_DIRS is populated")
endif()

# Find OpenCV
find_package(OpenCV REQUIRED) 
# Check and log the OpenCV variables
if(OpenCV_FOUND)
  message(STATUS "[ENV]: OpenCV found")
else()
  message(FATAL_ERROR "[ENV]: OpenCV not found")
endif()
if(NOT OpenCV_LIBRARIES)
  message(FATAL_ERROR "[ENV]: OpenCV_LIBRARIES is not populated")
else()
  message(STATUS "[ENV]: OpenCV_LIBRARIES is populated")
endif()

# Catkin specific configuration
catkin_package(
  CATKIN_DEPENDS roslib roscpp std_msgs cv_bridge
  DEPENDS OpenCV  # Added for OpenCV
)

# Explicitly set set variables for GLAD
set(GLAD_INCLUDE_DIR "$ENV{GLAD_DIR}/include")
# Check and log the GLAD variables
if(EXISTS $ENV{GLAD_DIR})
  message(STATUS "[ENV]: GLAD_DIR set to: $ENV{GLAD_DIR}")
else()
  message(FATAL_ERROR "[ENV]: GLAD_DIR not set")
endif()
if(EXISTS ${GLAD_INCLUDE_DIR})
  message(STATUS "[ENV]: GLAD_INCLUDE_DIR found at: ${GLAD_INCLUDE_DIR}")
else()
  message(FATAL_ERROR "[ENV]: GLAD_INCLUDE_DIR not found")
endif()

# Explicitly set set variables for GLFW
set(GLFW_LIBRARY "$ENV{GLFW_DIR}/lib-vc2019/glfw3.lib")
set(GLFW_INCLUDE_DIR "$ENV{GLFW_DIR}/include")
# Check and log the GLFW variables
if(EXISTS ${GLFW_LIBRARY})
  message(STATUS "[ENV]: GLFW_LIBRARY found at: ${GLFW_LIBRARY}")
else()
  message(FATAL_ERROR "[ENV]: GLFW_LIBRARY not found")
endif()
if(EXISTS ${GLFW_INCLUDE_DIR})
  message(STATUS "[ENV]: GLFW_INCLUDE_DIR found at: ${GLFW_INCLUDE_DIR}")
else()
  message(FATAL_ERROR "[ENV]: GLFW_INCLUDE_DIR not found")
endif()

# Explicitly set set variables for DevIL
set(DevIL_LIBRARY "$ENV{DevIL_DIR}/lib/x64/Release/DevIL.lib")
set(ILU_LIBRARY "$ENV{DevIL_DIR}/lib/x64/Release/ILU.lib")
set(ILUT_LIBRARY "$ENV{DevIL_DIR}/lib/x64/Release/ILUT.lib")
set(DevIL_INCLUDE_DIR "$ENV{DevIL_DIR}/include")
# Check and log the DevIL variables
if(EXISTS ${DevIL_LIBRARY})
  message(STATUS "[ENV]: DevIL_LIBRARY found at: ${DevIL_LIBRARY}")
else()
  message(FATAL_ERROR "[ENV]: DevIL_LIBRARY not found")
endif()
if(EXISTS ${ILU_LIBRARY})
  message(STATUS "[ENV]: ILU_LIBRARY found at: ${ILU_LIBRARY}")
else()
  message(FATAL_ERROR "[ENV]: ILU_LIBRARY not found")
endif()
if(EXISTS ${ILUT_LIBRARY})
  message(STATUS "[ENV]: ILUT_LIBRARY found at: ${ILUT_LIBRARY}")
else()
  message(FATAL_ERROR "[ENV]: ILUT_LIBRARY not found")
endif()
if(EXISTS ${DevIL_INCLUDE_DIR})
  message(STATUS "[ENV]: DevIL_INCLUDE_DIR found at: ${DevIL_INCLUDE_DIR}")
else()
  message(FATAL_ERROR "[ENV]: DevIL_INCLUDE_DIR not found")
endif()

# Explicitly set set variables for Ros Noetic
set(ROS_NOETIC_INCLUDE_DIR "$ENV{ROS_NOETIC_DIR}/include")
# Check and log the Ros Noetic variables
if(EXISTS $ENV{ROS_NOETIC_DIR})
  message(STATUS "[ENV]: ROS_NOETIC_DIR set to: $ENV{ROS_NOETIC_DIR}")
else()
  message(FATAL_ERROR "[ENV]: ROS_NOETIC_DIR not set")
endif()
if(EXISTS ${ROS_NOETIC_INCLUDE_DIR})
  message(STATUS "[ENV]: Ros Noetic include directory found at: ${ROS_NOETIC_INCLUDE_DIR}")
else()
  message(FATAL_ERROR "[ENV]: Ros Noetic include directory not found")
endif()

# Explicitly set set variables for PugiXML
set(PugiXML_LIBRARY "$ENV{ROS_NOETIC_DIR}/Lib/pugixml.lib")
# Check and log the PugiXML variables
if(EXISTS ${PugiXML_LIBRARY})
  message(STATUS "[ENV]: PugiXML_LIBRARY found at: ${PugiXML_LIBRARY}")
else()
  message(FATAL_ERROR "[ENV]: PugiXML_LIBRARY not found")
endif()

# Explicitly set set variables for OpenGL
set(OpenGL_LIBRARY "$ENV{ROS_NOETIC_DIR}/Lib/OpenGL32.Lib")
if(EXISTS ${OpenGL_LIBRARY})
  message(STATUS "[ENV]: OpenGL_LIBRARY found at: ${OpenGL_LIBRARY}")
else()
  message(FATAL_ERROR "[ENV]: OpenGL_LIBRARY not found")
endif()

# Add the include directories
include_directories(
  ${catkin_INCLUDE_DIRS}  
  ${ROS_NOETIC_INCLUDE_DIR}
  ${GLAD_INCLUDE_DIR}
  ${GLFW_INCLUDE_DIR}
  ${DevIL_INCLUDE_DIR}
  ${PugiXML_INCLUDE_DIR}
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
  ${GLAD_LIBRARY}
  ${GLFW_LIBRARY}
  ${DevIL_LIBRARY}
  ${ILU_LIBRARY}
  ${ILUT_LIBRARY}
  ${PugiXML_LIBRARY}
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