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

// OpenGL Utility Toolkit (GLUT) for drawing 2D and 3D objects
#include <GL/glut.h> 

// ============= METHODS =============

// Callback function for handling keyboard input
void callbackKeyBinding(GLFWwindow *, int, int, int, int);

// Callback function for handling window resize events
void callbackFrameBufferSize(GLFWwindow *, int, int);

// Callback function for handling GLFW errors
static void callbackError(int, const char *);

// Function to draw a given control point marker
void drawControlPoint(float, float, float, float, std::vector<float> rgb);

// Function to draw a rectangle with given corners
void drawWall(std::vector<cv::Point2f>);

// Function to draw multiple wall images
void drawWallsAll();

// Function to display number as text using OpenGL's bitmap font
void drawNumber(int);

// Function to change the window monitor and mode (full-screen or windowed)
void changeWindowMonMode();

// Function to computer wall vertices
std::vector<cv::Point2f> computeWallVertices(float, float, float, float, float);

// Function to compute homography matrix
void computeHomography();

// Function to load coordinates from an XML file
void loadCoordinatesXML();

// Function to save coordinates to an XML file
void saveCoordinatesXML();

// The main function of the program
int main(int, char **);

// ============= VARIABLES =============

// Number of rows and columns in the maze
const int MAZE_SIZE = 3;

// Spricify window resolution: 4K resolution (3840x2160)
int winWidth = 3840;
int winHeight = 2160;
float winAspectRatio = (float)winWidth / (float)winHeight;

/**
 * @brief Array to hold the position and transformation parameters for control points in normalized coordinates [-1, 1].
 *
 * Each row corresponds to a specific control point on the screen, and each column holds a different attribute
 * of that control point.
 *
 * - Rows:
 *   - [0]: Top-left control point
 *   - [1]: Top-right control point
 *   - [2]: Bottom-right control point
 *   - [3]: Bottom-left control point
 *
 * - Columns:
 *   - [0]: X-distance from center [-1,1]
 *   - [1]: Y-distance from center [-1,1]
 *   - [2]: Width parameter
 *   - [3]: Height parameter
 *   - [4]: Shearing factor
 */

// Variables related to control points
const float cpSize = 0.015f;
const float xy_lim = 0.5f;
const float ht_scale = winAspectRatio * 1.8 * 0.75;
float cpPositions[4][5] = {
    {-xy_lim, xy_lim, cpSize, cpSize *ht_scale, 0.0f}, // top-left control point
    {xy_lim, xy_lim, cpSize, cpSize *ht_scale, 0.0f},  // top-right control point
    {xy_lim, -xy_lim, cpSize, cpSize *ht_scale, 0.0f}, // bottom-right control point
    {-xy_lim, -xy_lim, cpSize, cpSize *ht_scale, 0.0f} // bottom-left control point
};
cv::Mat H = cv::Mat::eye(3, 3, CV_32F);
std::string cpModMode = "position";
int cpSelected = 0;
std::vector<float> cpInactiveRGB = {0.0f, 0.0f, 1.0f};  // Selected control point color
std::vector<float> cpActiveRGB = {1.0f, 0.0f, 0.0f};  // Unselected control point color


// Wall image size and spacing
const float wallWidth = 0.02f;
const float wallSpace = 2.5f * wallWidth;
// const float wallWidth = cpSize;
// const float wallSpace = 0.05f;

// Variables related to image and file paths
std::string windowName = "Projection Calibration";
std::string packagePath = ros::package::getPath("projection_calibration");
std::string workspacePath = packagePath.substr(0, packagePath.rfind("/src"));
std::string imgPath = workspacePath + "/data/img";
std::string configPath = workspacePath + "/data/proj_cfg/proj_cfg.xml";

// List of image file paths
int imageInd = 0; // Index of the image to be loaded
std::vector<std::string> imagePaths = {
    imgPath + "/1_test_pattern.bmp",
    imgPath + "/2_manu_pirate.bmp",
    imgPath + "/3_earthlings.bmp",
    imgPath + "/4_tj.bmp",
};

// Container to hold the loaded images
std::vector<ILuint> imageIDs;

// Variables related to window and OpenGL
GLFWwindow *window;
GLuint fbo;
GLuint fboTexture;
GLFWmonitor *monitor = NULL;
GLFWmonitor **monitors;
int monitorCount;          // Number of monitors connected to the system
int monitorInd = 0;        // Index of the monitor to be used [0, 1]
bool isFullScreen = false; // Flag to indicate if the window is in full screen mode

// Variables related to mouse input (UNUSED)
ILint textureImgWidth;
ILint textureImgHeight;

#endif