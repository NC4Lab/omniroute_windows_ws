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

// ROS for robot operating system functionalities
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <XmlRpcValue.h>

// Standard Library for various utilities
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>

// PugiXML for XML parsing
#include <pugixml.hpp>

// OpenCV for computer vision tasks
#include <opencv2/opencv.hpp>

// // OpenGL for mathematics
// #include <GL/gl.h>

// Function to test OpenCV
bool initialize_opencv();

// Function to initialize GLAD and GLFW
bool initialize_libraries();

// Function to initialize DevIL
bool initialize_devil();

// Function to test PugiXML
bool test_pugixml();

// Function to test GLM
bool test_glm();


#endif // PROJ_PKG_TEST_H
