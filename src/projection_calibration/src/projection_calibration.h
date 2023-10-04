// ######################################

// ====== projection_calibration.h ======

// ######################################
#ifndef _PROJECTION_CALIBRATION_H
#define _PROJECTION_CALIBRATION_H

//============= INCLUDE ================

// OpenGL (GLAD and GLFW) for graphics and windowing
#define GLAPIENTRY APIENTRY
#include "glad/glad.h"
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

// DevIL for image loading and manipulation
#include <IL/il.h>
#include <IL/ilu.h>
#include <IL/ilut.h>
#include <IL/devil_cpp_wrapper.hpp>

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
#include "pugixml.hpp"

// OpenCV for computer vision tasks
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/hal/interface.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

// ============= METHODS =============

std::vector<cv::Point2f> createRectPoints(float, float, float, float, float);

// Function to load coordinates from an XML file
void loadCoordinatesXML();

// Function to save coordinates to an XML file
void saveCoordinatesXML();

// Function to compute homography matrix
void computeHomography();

// Callback function for handling keyboard input
void callbackKeyBinding(GLFWwindow *, int, int, int, int);

// Callback function for handling window resize events
void callbackFrameBufferSize(GLFWwindow *, int, int);

// Callback function for handling GLFW errors
static void callbackError(int, const char *);

// Function to draw a given control point marker
void drawControlPoint(float, float, float, float);

// Function to draw a rectangle with given corners
void drawWall(std::vector<cv::Point2f>, int);

// Function to draw multiple wall images
void drawWallsAll();

// The main function of the program
int main(int, char **);

// ============= VARIABLES =============

// Constants
const int MAZE_SIZE = 3;

// Variables related to control point positions and transformation
float cpPositions[4][5] = {
    {-0.8f, 0.8f, 0.02f, 0.02f, 0.0f}, // top-left control point
    {0.8f, 0.8f, 0.02f, 0.02f, 0.0f},  // top-right control point
    {0.8f, -0.8f, 0.02f, 0.02f, 0.0f}, // bottom-right control point
    {-0.8f, -0.8f, 0.02f, 0.02f, 0.0f} // bottom-left control point
};
float shearValues[MAZE_SIZE][MAZE_SIZE];
float sizeValues[MAZE_SIZE][MAZE_SIZE];
float configurationValues[3][3][3];
cv::Mat H = cv::Mat::eye(3, 3, CV_32F);
int cpSelected = 0;

// Variables related to wall properties
float wallWidth = 0.02f;
float wallHeight = 0.02f;
float wallSep = 0.05f;
std::string cpModMode = "pos";
float shearAmount = 0.0f;
std::vector<cv::Point2f> wallCorners = createRectPoints(0.0f, 0.0f, wallWidth, wallHeight, 0);

// Variables related to image and file paths
std::string windowName;
std::string packagePath = ros::package::getPath("projection_calibration");
std::string workspacePath = packagePath.substr(0, packagePath.rfind("/src"));
std::string imgPath = workspacePath + "/data/img";
std::string configPath = workspacePath + "/data/proj_cfg/proj_cfg.xml";

// List of image file paths
int imageInd = 0; // Index of the image to be loaded
std::vector<std::string> imagePaths = {
    imgPath + "/tj.bmp",
    imgPath + "/mmCarribean.png",
    // Add more image file paths as needed
};

// Container to hold the loaded images
std::vector<ILuint> imageIDs;

// Variables related to window and OpenGL
int winWidth = 3840;
int winHeight = 2160;
GLFWwindow *window;
GLuint fbo;
GLuint fboTexture;
GLFWmonitor *monitor = NULL;
int monitorNumber = 0;
GLFWmonitor **monitors;
int monitor_count;

ILint texWidth;
ILint texHeight;

#endif