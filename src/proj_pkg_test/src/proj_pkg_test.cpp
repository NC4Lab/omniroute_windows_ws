#include "proj_pkg_test.h"

int main(int argc, char **argv)
{

  // Initialize ROS
  ros::init(argc, argv, "proj_pkg_test_node");

  // Test OpenCV
  if (!initialize_opencv())
  {
    ROS_ERROR("Failed to initialize OpenCV.");
    return 1;
  }

  // Initialize GLAD and GLFW
  if (!initialize_libraries())
  {
    ROS_ERROR("Failed to initialize GLAD and GLFW.");
    return 1;
  }

  // Initialize DevIL
  if (!initialize_devil())
  {
    ROS_ERROR("Failed to initialize DevIL.");
    return 1;
  }

  // Test PugiXML
  if (!test_pugixml())
  {
    ROS_ERROR("Failed to initialize PugiXML.");
    return 1;
  }

  // Test OpenGL Mathematics (GLM)
  if (!test_glm())
  {
    ROS_ERROR("Failed to initialize GLM.");
    return 1;
  }

  ROS_INFO("OpenCV, GLAD, GLFW, DevIL, PugiXML and GLM setup successfully.");
  return 0;
}

bool initialize_opencv()
{
  // Get image path
  std::string packagePath = ros::package::getPath("proj_pkg_test");
  std::string imagePath = packagePath + "/img/mmPirate.png";

  // Load the image
  cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
  if (image.empty())
  {
    ROS_ERROR("OpenCV failed to load an image.");
    return false;
  }

  // If we reach this point, OpenCV is initialized and the image is loaded
  return true;
}

bool initialize_libraries()
{
  // Initialize GLFW
  if (!glfwInit())
  {
    ROS_ERROR("Failed to initialize GLFW.");
    return false;
  }

  // Create a GLFW windowed mode window and its OpenGL context
  GLFWwindow *window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
  if (!window)
  {
    ROS_ERROR("Failed to create GLFW window.");
    return false;
  }

  // Make the window's context current
  glfwMakeContextCurrent(window);

  // Initialize GLAD
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
  {
    ROS_ERROR("Failed to initialize GLAD.");
    return false;
  }

  return true;
}

bool initialize_devil()
{
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

  if (!ilLoadImage(imagePath.c_str()))
  {
    ROS_ERROR("DevIL failed to load an image.");
    return false;
  }

  // If we reach this point, DevIL is initialized and the image is loaded
  return true;
}

bool test_pugixml()
{
  pugi::xml_document doc;
  if (doc.load_string("<root><child>text</child></root>"))
  {
    pugi::xml_node root = doc.child("root");
    std::string text = root.child("child").text().as_string();
    ROS_INFO("Pugixml test passed: %s", text.c_str());
    return true;
  }
  else
  {
    ROS_ERROR("Pugixml test failed.");
    return false;
  }
}

bool test_glm()
{
  // Initialize GLFW
  if (!glfwInit())
  {
    ROS_ERROR("Failed to initialize GLFW");
    return false;
  }

  // Create a windowed mode window and its OpenGL context
  GLFWwindow *window = glfwCreateWindow(640, 480, "Test Window", NULL, NULL);
  if (!window)
  {
    ROS_ERROR("Failed to create GLFW window");
    glfwTerminate();
    return false;
  }

  // Make the window's context current
  glfwMakeContextCurrent(window);

  // Test OpenGL functions
  glBegin(GL_POINTS);
  glVertex2f(0.0f, 0.0f);
  glEnd();

  glBegin(GL_TRIANGLES);
  glVertex2f(0.0f, 0.0f);
  glVertex2f(1.0f, 0.0f);
  glVertex2f(0.0f, 1.0f);
  glEnd();

  // Destroy the window and terminate GLFW
  glfwDestroyWindow(window);
  glfwTerminate();

  ROS_INFO("GLM test passed");
  return true;
}