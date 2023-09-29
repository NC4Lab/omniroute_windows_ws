#include "proj_pkg_test.h"

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "proj_pkg_test_node");

  // Initialize GLAD and GLFW
  if (!initialize_libraries()) {
    ROS_ERROR("Failed to initialize GLAD and GLFW.");
    return 1;
  }

  // Initialize DevIL
  if (!initialize_devil()) {
    ROS_ERROR("Failed to initialize DevIL.");
    return 1;
  }

  ROS_INFO("GLAD, GLFW, and DevIL initialized successfully.");
  return 0;
}

bool initialize_libraries() {
  // Initialize GLFW
  if (!glfwInit()) {
    ROS_ERROR("Failed to initialize GLFW.");
    return false;
  }

  // Create a GLFW windowed mode window and its OpenGL context
  GLFWwindow* window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
  if (!window) {
    ROS_ERROR("Failed to create GLFW window.");
    return false;
  }

  // Make the window's context current
  glfwMakeContextCurrent(window);

  // Initialize GLAD
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    ROS_ERROR("Failed to initialize GLAD.");
    return false;
  }

  return true;
}

bool initialize_devil() {
  // Initialize DevIL
  ilInit();
  iluInit();
  ilutInit();

  // Test DevIL
  ILuint image;
  ilGenImages(1, &image);
  ilBindImage(image);

  if (ilLoadImage("test_image.jpg")) {
    ROS_INFO("DevIL successfully loaded an image.");
  } else {
    ROS_ERROR("DevIL failed to load an image.");
    return false;
  }

  return true;
}
