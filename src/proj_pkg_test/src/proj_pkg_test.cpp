#include "proj_pkg_test.h"

int main(int argc, char **argv) {

  // Initialize ROS
  ros::init(argc, argv, "proj_pkg_test_node");

  // Test OpenCV
  if (!initialize_opencv()) {
    ROS_ERROR("Failed to initialize OpenCV.");
    return 1;
  }

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

  ROS_INFO("OpenCV, GLAD, GLFW, and DevIL initialized successfully.");
  return 0;
}

bool initialize_opencv() {
  // Get image path
  std::string packagePath = ros::package::getPath("proj_pkg_test");
  std::string imagePath = packagePath + "/img/mmPirate.png";  

  // Load the image
  cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
  if (image.empty()) {
    ROS_ERROR("OpenCV failed to load an image.");
    return false;
  }

  // If we reach this point, OpenCV is initialized and the image is loaded
  return true;
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

  // Get image path
  std::string packagePath = ros::package::getPath("proj_pkg_test");
  std::string imagePath = packagePath + "/img/mmPirate.png";  

  // Load the image
  ILuint imageID;
  ilGenImages(1, &imageID);
  ilBindImage(imageID);

  if (!ilLoadImage(imagePath.c_str())) {
    ROS_ERROR("DevIL failed to load an image.");
    return false;
  }

  // If we reach this point, DevIL is initialized and the image is loaded
  return true;
}