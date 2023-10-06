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

// Callback function for handling keyboard input
void callbackKeyBinding(GLFWwindow *, int, int, int, int);

// Callback function for handling window resize events
void callbackFrameBufferSize(GLFWwindow *, int, int);

// Callback function for handling GLFW errors
static void callbackError(int, const char *);

// Function to draw a given control point marker
void drawControlPoint(float, float, float, std::vector<float>);

// Function to draw a rectangle with given corners
void drawWall(std::vector<cv::Point2f>);

// Function to draw multiple wall images
void drawWallsAll();

// Function to load number textures
void loadImgTextures(std::vector<ILuint> &, std::vector<std::string> &);

// Function to merge two ILuint images
ILuint mergeImages(ILuint, ILuint);

// Function to change the window monitor and mode (e.g., full-screen or windowed)
void updateWindowMonMode();

// Function to computer wall vertices
std::vector<cv::Point2f> computeWallVertices(float, float, float, float, float);

// Function to compute homography matrix
void computeHomography();

// Funciton to reset control point parameter list
void resetParamCP();

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
int winWidthPxl = 3840;
int winHeightPxl = 2160;
float winAspectRatio = (float)winWidthPxl / (float)winHeightPxl;

// Specify the window name
std::string windowName = "Projection Calibration";

// Variables related to control points
const float cpSize = 0.015f;
const float xy_lim = 0.5f;
const float ht_scale = winAspectRatio * 1.8 * 0.75;
cv::Mat H = cv::Mat::eye(3, 3, CV_32F);
int cpSelected = 0;
std::string cpModMode = "position";
std::vector<float> cpActiveRGB = {1.0f, 0.0f, 0.0f};   // Active control point marker color
std::vector<float> cpInactiveRGB = {0.0f, 0.0f, 1.0f}; // Inactive control point marker color
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
float cpParamDefault[4][5] = {
    {-xy_lim, xy_lim, cpSize, cpSize *ht_scale, 0.0f}, // top-left control point
    {xy_lim, xy_lim, cpSize, cpSize *ht_scale, 0.0f},  // top-right control point
    {xy_lim, -xy_lim, cpSize, cpSize *ht_scale, 0.0f}, // bottom-right control point
    {-xy_lim, -xy_lim, cpSize, cpSize *ht_scale, 0.0f} // bottom-left control point
};
float cpParam[4][5];

// Wall image size and spacing (pixels)
int wallWidthPxl = 300;
int heightHeightPxl = 540;

// Wall image size and spacing (normalized?)
/// @todo: Figure out how these values are used
const float wallWidth = 0.02; 
const float wallSpace = 2.5 * wallWidth;

// Directory paths
std::string packagePath = ros::package::getPath("projection_calibration");
std::string workspacePath = packagePath.substr(0, packagePath.rfind("/src"));
std::string configDirPath = workspacePath + "/data/proj_cfg";
std::string imgTestPath = workspacePath + "/data/img/test_patterns";
std::string imgStatePath = workspacePath + "/data/img/ui_state_images";

// Test image variables
std::vector<ILuint> imgTestIDs; // Container to hold the loaded images
std::vector<std::string> imgTestPaths = {
    // List of test image file paths
    imgTestPath + "/1_test_pattern.bmp",
    imgTestPath + "/2_manu_pirate.bmp",
    imgTestPath + "/3_earthlings.bmp",
};
int imgTestInd = 0; // Index of the image to be loaded
size_t nTestImg = imgTestPaths.size(); // Number of test images

// Monitor variables
std::vector<ILuint> imgMonIDs; // Container to hold the loaded images for ui
std::vector<std::string> imgMonPaths = {
    // List of monitor number image file paths
    imgStatePath + "/m0.bmp",
    imgStatePath + "/m1.bmp",
    imgStatePath + "/m2.bmp",
    imgStatePath + "/m3.bmp",
    imgStatePath + "/m4.bmp",
    imgStatePath + "/m5.bmp",
};
int imgMonInd = 0; // Index of the image to be loaded
int nMonitors;          // Number of monitors connected to the system
bool isFullScreen = false; // Flag to indicate if the window is in full screen mode

// Control point parameter image variables for ui
std::vector<ILuint> imgParamIDs; // Container to hold the loaded images for ui
std::vector<std::string> imgParamPaths = {
    // List of cp parameter image file paths
    imgStatePath + "/p.bmp",
    imgStatePath + "/d.bmp",
    imgStatePath + "/s.bmp",
};
int imgParamInd = 0; // Index of the image to be loaded

// Callibration image variables for ui
std::vector<ILuint> imgCalIDs; // Container to hold the loaded images for ui
std::vector<std::string> imgCalPaths = {
    // List of mode image file paths
    imgStatePath + "/c-wm.bmp",
    imgStatePath + "/c-wl.bmp",
    imgStatePath + "/c-wr.bmp",
    imgStatePath + "/c-f.bmp",
    imgStatePath + "/c-d.bmp",
};
int imgCalInd = 0; // Index of the image to be loaded
size_t nCalModes = imgCalPaths.size(); // Number of calibration modes

// Variables related to window and OpenGL
GLFWwindow *window;
GLuint fbo;
GLuint fboTexture;
GLFWmonitor *monitor = NULL;
GLFWmonitor **monitors;

// Variables related to mouse input (UNUSED)
ILint textureImgWidth;
ILint textureImgHeight;

#endif