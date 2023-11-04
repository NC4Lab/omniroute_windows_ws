// ######################################################################################################

// ======================================== projection_display.h ========================================

// ######################################################################################################

#ifndef _PROJECTION_DISPLY_H
#define _PROJECTION_DISPLY_H

// ================================================== INCLUDE ==================================================

// Local custom libraries
#include "projection_utils.h"

// ================================================== VARIABLES ==================================================

// // Directory paths
// std::string image_wall_dir_path = IMAGE_TOP_DIR_PATH + "/runtime_images/shapes_no_outline";

// // Maze image container
// std::vector<ILuint> imgMazeIDVec(4);

// // Wall image file variables
// std::vector<ILuint> wallImgMatVec; // Container to hold the loaded images for ui
// std::vector<std::string> imgWallPathVec = {
//     // List of image file paths
//     image_wall_dir_path + "/blank.bmp",    // [0] Blank image
//     image_wall_dir_path + "/square.bmp",   // [1] Square image
//     image_wall_dir_path + "/circle.bmp",   // [2] Circle image
//     image_wall_dir_path + "/triangle.bmp", // [3] Triangle image
//     image_wall_dir_path + "/star.bmp",     // [4] Star image
//     image_wall_dir_path + "/pentagon.bmp", // [5] Pentagon image
// };

// // Default monitor index for all windows
// int winMonIndDefault = 3; // Default monitor index for the window

// // Monitor and projector variables
// const int nProjectors = 2; // Number of projectors  (hardcoded)
// std::vector<int> projMonIndArr = {
//     // Index of the monitor associeted to each projector (hardcoded)
//     1,
//     2,
// };
// bool F.setFullscreen = false; // Flag to indicate if the window is in full screen mode
// bool isWinOnProj = false;  // Flag to indicate if the window is on the projector

// // Number of monitors (autopopulated)
// int N.monitors;             

// // Window for OpenGL
// GLFWwindow *p_windowIDVec[nProjectors];

// // FBO variables for OpenGL
// std::vector<GLuint> fboIDVec(nProjectors);
// std::vector<GLuint> fboTextureIDVec(nProjectors);

// // Monitor variable for OpenGL
// GLFWmonitor *p_monitorID = nullptr;
// GLFWmonitor **pp_monitorIDVec = nullptr;

// ================================================== FUNCTIONS ==================================================
/**
 * @file Circle.h
 * @author Adam Lester
 * @date Summer 2023
 * @brief Class for representing and manipulating a circle in OpenGL context.
 *
 * @section DESCRIPTION
 * The Circle class encapsulates the properties and behavior of a circle
 * that can be rendered using OpenGL. It includes methods for setting the
 * position, radius, color, and transformations such as rotation and scaling.
 * The class is designed to work with the OpenGL rendering pipeline and
 * expects the user to manage the shader program and uniform locations externally.
 *
 * @section USAGE
 * - Create an instance of the Circle class by providing initial parameters.
 * - Use member functions to modify properties or apply transformations.
 * - Call the draw() method within the OpenGL rendering loop to render the circle.
 *
 * @code
 * Circle circle(cv::Point2f(0.0f, 0.0f), 1.0f, cv::Scalar(1.0f, 0.0f, 0.0f), 36, 1.0f);
 * circle.setPosition(cv::Point2f(0.5f, -0.5f));
 * circle.setRadius(0.25f);
 * circle.setColor(cv::Scalar(0.0f, 1.0f, 0.0f));
 * circle.draw(shaderProgram, colorLocation, transformLocation);
 * @endcode
 *
 * @section DEPENDENCIES
 * - OpenGL for rendering the circle.
 * - OpenCV for matrix operations and handling transformations.
 *
 * @section LICENSE
 * @todo Define the license for the code.
 *
 * @details This class is part of the Omniroute project, which aims to
 * create an automated rodent maze apparatus. The Circle class helps in rendering
 * elements in the maze where a circular representation is required.
 */
/**
 * @brief  Entry point for the projection_display ROS node.
 *
 * This program initializes ROS, DevIL, and GLFW, and then enters a main loop
 * to handle image projection and calibration tasks.
 *
 * @param  argc  Number of command-line arguments.
 * @param  argv  Array of command-line arguments.
 *
 * @return 0 on successful execution, -1 on failure.
 */
int main(int, char **);

#endif
