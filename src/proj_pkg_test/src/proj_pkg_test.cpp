#include "proj_pkg_test.h"

ProjPkgTest::ProjPkgTest() : window(nullptr) {}

ProjPkgTest::~ProjPkgTest() {
    if (window) {
        glfwDestroyWindow(window);
    }
    glfwTerminate();
}

bool ProjPkgTest::initializeGL() {
    // Initialize GLFW
    if (!glfwInit()) {
        ROS_ERROR("Failed to initialize GLFW");
        return false;
    }

    // Create a windowed mode window and its OpenGL context
    window = glfwCreateWindow(640, 480, "GLAD Test", NULL, NULL);
    if (!window) {
        ROS_ERROR("Failed to create GLFW window");
        glfwTerminate();
        return false;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

    // Initialize GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        ROS_ERROR("Failed to initialize GLAD");
        return false;
    }

    ROS_INFO("GLAD initialized successfully");
    return true;
}

void ProjPkgTest::run() {
    if (initializeGL()) {
        ROS_INFO("GLAD and GLFW are set up correctly.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "proj_pkg_test_node");
    ProjPkgTest test;
    test.run();
    return 0;
}
