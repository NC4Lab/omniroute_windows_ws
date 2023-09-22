#ifndef PROJ_PKG_TEST_H
#define PROJ_PKG_TEST_H

// ROS headers
#include <ros/ros.h>

// GLAD and GLFW headers
#include <glad/glad.h>
#include <GLFW/glfw3.h>

class ProjPkgTest {
public:
    ProjPkgTest();
    ~ProjPkgTest();
    void run();
private:
    bool initializeGL();
    GLFWwindow* window;
};

#endif // PROJ_PKG_TEST_H
