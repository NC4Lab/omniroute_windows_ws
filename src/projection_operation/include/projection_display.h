// ######################################################################################################

// ======================================== projection_display.h ========================================

// ######################################################################################################

#ifndef _PROJECTION_DISPLY_H
#define _PROJECTION_DISPLY_H

// ================================================== INCLUDE ==================================================

// Local custom libraries
#include "projection_utils.h"

// ================================================== VARIABLES ==================================================

// Control point parameter array
float cpParam[4][5];

// The homography matrix used to warp perspective.
cv::Mat H = cv::Mat::eye(3, 3, CV_32F);

// Directory paths
std::string image_runtime_dir_path = IMAGE_TOP_DIR_PATH + "/runtime_images/shapes";

// Image file paths
std::vector<ILuint> imgWallIDVec; // Container to hold the loaded images for ui
std::vector<std::string> imgWallPathVec = {
    // List of image file paths
    image_runtime_dir_path + "/circle.bmp",
    image_runtime_dir_path + "/pentagon.bmp",
    image_runtime_dir_path + "/square.bmp",
    image_runtime_dir_path + "/star.bmp",
    image_runtime_dir_path + "/triangle.bmp"};

// Projector and monitor variables
int nMonitors; // Number of monitors connected to the system

// Projector variables
const int nProjectors = 2;               // Number of projectors
int indProjector = 0;                    // Index of the active projector
int indProjectorMonitorArr[nProjectors]; // Index of the monitor for each projector

// Specify the window names for the projectors
std::vector<std::string> windowNameVec = {
    "Projector 0",
    "Projector 1",
    "Projector 2",
    "Projector 3",
};

// Window for OpenGL
GLFWwindow *p_windowIDVec[nProjectors];

// FBO variables for OpenGL
std::vector<GLuint> fboIDVec(nProjectors);
std::vector<GLuint> fboTextureIDVec(nProjectors);

// Monitor variable for OpenGL
GLFWmonitor *p_monitorID = NULL;
GLFWmonitor **p_monitorIDVec;

// Projector variable for OpenGL
GLFWmonitor *p_projectorID;
GLFWmonitor *p_projectorIDVec[nProjectors];

// ================================================== FUNCTIONS ==================================================

/**
 * @brief GLFW key callback function to handle key events and execute corresponding actions.
 *
 * This function is set as the GLFW key callback and gets called whenever a key event occurs.
 * It handles various key events for control points, monitor handling, XML operations, and more.
 *
 * ## Keybindings:
 *
 * @param window Pointer to the GLFW window that received the event.
 * @param key The keyboard key that was pressed or released.
 * @param scancode The system-specific scancode of the key.
 * @param action GLFW_PRESS, GLFW_RELEASE or GLFW_REPEAT.
 * @param mods Bit field describing which modifier keys were held down.
 */
void callbackKeyBinding(GLFWwindow *, int, int, int, int);

/**
 * @brief Callback function for handling framebuffer size changes.
 *
 * This function is called whenever the framebuffer size changes,
 * and it updates the OpenGL viewport to match the new dimensions.
 *
 * @param window Pointer to the GLFW window.
 * @param width The new width of the framebuffer.
 * @param height The new height of the framebuffer.
 */
void callbackFrameBufferSizeGLFW(GLFWwindow *, int, int);

/**
 * @brief Callback function for handling errors.
 *
 * This function is called whenever an error occurs in the GLFW context.
 * It logs the error message using ROS_ERROR.
 *
 * @param error The error code.
 * @param description The error description.
 */
static void callbackErrorGLFW(int, const char *);

/**
 * @brief Check and print OpenGL errors.
 *
 * @param msg_str String to include in the message.
 */
void checkErrorGL(std::string);

/**
 * @brief Set up a GLFW window and its associated Framebuffer Object (FBO) and texture.
 *
 * This function creates a GLFW window, sets its OpenGL context and callbacks, and initializes
 * an FBO and texture to be used for offscreen rendering.
 *
 * @param pp_window_id GLFWwindow pointer array, where each pointer corresponds to a projector window.
 * @param win_ind Index of the window for which the setup is to be done.
 * @param pp_ref_monitor_id Reference to the GLFWmonitor pointer array.
 * @param mon_ind Index of the monitor to move the window to.
 * @param ref_window_name Reference to the name to be assigned to the GLFW window.
 * @param ref_fbo_id Reference to the GLuint variable where the generated FBO ID will be stored.
 * @param ref_fbo_texture_id Reference to the GLuint variable where the generated FBO texture ID will be stored.
 *
 * @return Void. The function will terminate the GLFW context and log an error if window creation fails.
 */
void setupProjGLFW(GLFWwindow **, int, GLFWmonitor **&, int, const std::string &, GLuint &, GLuint &);

/**
 * @brief Draws a textured rectangle using OpenGL.
 *
 * @param rect_vertices_vec Vector of vertex/corner points for a rectangular image.
 */
void drawRectImage(std::vector<cv::Point2f>);

/**
 * @brief Draws all walls in the maze grid with texture mapping and perspective warping.
 *
 * This function iterates through the maze grid to draw each wall. It uses the DevIL library
 * to handle image loading and OpenGL for rendering. The function also performs perspective
 * warping based on the homography matrix and shear and height values extracted from control points.
 */
void drawWallsAll();

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
