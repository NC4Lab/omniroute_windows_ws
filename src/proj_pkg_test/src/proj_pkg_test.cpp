#include "proj_pkg_test.h"

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "proj_pkg_test_node");

  // Initialize GLAD and GLFW
  if (!initialize_libraries()) {
    ROS_ERROR("Failed to initialize GLAD and GLFW.");
    return 1;
  }

  ROS_INFO("GLAD and GLFW initialized successfully.");
  return 0;
}

bool initialize_libraries() {
  // Initialize GLFW
  if (!glfwInit()) {
    ROS_ERROR("Failed to initialize GLFW.");
    return false;
  }

  // Initialize GLAD
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    ROS_ERROR("Failed to initialize GLAD.");
    return false;
  }

  return true;
}
