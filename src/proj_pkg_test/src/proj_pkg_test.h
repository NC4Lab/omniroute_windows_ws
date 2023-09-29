#ifndef PROJ_PKG_TEST_H
#define PROJ_PKG_TEST_H

// ROS headers
#include <ros/ros.h>

// GLAD and GLFW headers
#include <glad/glad.h>
#include <GLFW/glfw3.h>

// DevIL headers
#include <IL/il.h>
#include <IL/ilu.h>
#include <IL/ilut.h>

// Function to initialize GLAD and GLFW
bool initialize_libraries();

// Function to initialize DevIL
bool initialize_devil();

#endif // PROJ_PKG_TEST_H
