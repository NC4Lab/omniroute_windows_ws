// ######################################################################################################

// ======================================== projection_display.h ========================================

// ######################################################################################################

#ifndef _PROJECTION_DISPLY_H
#define _PROJECTION_DISPLY_H

// ================================================== INCLUDE ==================================================

// Local custom libraries
#include "projection_utils.h"

// ================================================== VARIABLES ==================================================

// Directory paths
std::string image_wall_dir_path = IMAGE_TOP_DIR_PATH + "/runtime_images/shapes_no_outline";

// Maze image container
std::vector<ILuint> imgMazeIDVec(4);

// Wall image file variables
std::vector<ILuint> imgWallIDVec; // Container to hold the loaded images for ui
std::vector<std::string> imgWallPathVec = {
    // List of image file paths
    image_wall_dir_path + "/blank.bmp",    // [0] Blank image
    image_wall_dir_path + "/square.bmp",   // [1] Square image
    image_wall_dir_path + "/circle.bmp",   // [2] Circle image
    image_wall_dir_path + "/triangle.bmp", // [3] Triangle image
    image_wall_dir_path + "/star.bmp",     // [4] Star image
    image_wall_dir_path + "/pentagon.bmp", // [5] Pentagon image
};

// Default monitor index for all windows
int winMonIndDefault = 3; // Default monitor index for the window

// Monitor and projector variables
const int nProjectors = 2; // Number of projectors  (hardcoded)
std::vector<int> projMonIndArr = {
    // Index of the monitor associeted to each projector (hardcoded)
    1,
    2,
};
bool isFullScreen = false; // Flag to indicate if the window is in full screen mode
bool isWinOnProj = false;  // Flag to indicate if the window is on the projector

// Number of monitors (autopopulated)
int nMonitors;             

// Window for OpenGL
GLFWwindow *p_windowIDVec[nProjectors];

// FBO variables for OpenGL
std::vector<GLuint> fboIDVec(nProjectors);
std::vector<GLuint> fboTextureIDVec(nProjectors);

// Monitor variable for OpenGL
GLFWmonitor *p_monitorID = nullptr;
GLFWmonitor **pp_monitorIDVec = nullptr;

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
 * @brief Checks for OpenGL errors and logs them.
 * Should be called after OpenGL API calls.
 *
 * @example checkErrorGL(__LINE__, __FILE__);
 *
 * @param line Line number where the function is called.
 * @param file_str File name where the function is called.
 * @param msg_str Optional message to provide additional context (default to nullptr).
 *
 * @return 0 if no errors, -1 if error.
 */
int checkErrorGL(int, const char *, const char * = nullptr);

/**
 * @brief Checks for GLFW errors and logs them.
 * Should be called after GLFW API calls.
 *
 * @example checkErrorGLFW(__LINE__, __FILE__);
 *
 * @param line Line number where the function is called.
 * @param file_str File name where the function is called.
 * @param msg_str Optional message to provide additional context (default to nullptr).
 *
 * @return 0 if no errors, -1 if error.
 */
int checkErrorGLFW(int, const char *, const char * = nullptr);

/**
 * @brief Set up a GLFW window and its associated Framebuffer Object (FBO) and texture.
 *
 * This function creates a GLFW window, sets its OpenGL context and callbacks, and initializes
 * an FBO and texture to be used for offscreen rendering.
 *
 * @param pp_window_id GLFWwindow pointer array, where each pointer corresponds to a projector window.
 * @param win_ind Index of the window for which the setup is to be done.
 * @param pp_r_monitor_id Reference to the GLFWmonitor pointer array.
 * @param mon_id_ind Index of the monitor to move the window to.
 * @param r_fbo_id Reference to the GLuint variable where the generated FBO ID will be stored.
 * @param r_fbo_texture_id Reference to the GLuint variable where the generated FBO texture ID will be stored.
 *
 * @return 0 on successful execution, -1 on failure.
 */
int setupProjGLFW(GLFWwindow **, int, GLFWmonitor **&, int, GLuint &, GLuint &);

/**
 * @brief Changes the display mode and monitor of the application window.
 *
 * This function switches the application window between full-screen and windowed modes
 * and moves it to the monitor specified by the global variable imgMonNumInd.
 *
 * In full-screen mode, the window is resized to match the dimensions of the selected monitor.
 * In windowed mode, the window is resized to a default size and positioned near the top-left
 * corner of the selected monitor.
 *
 * @note The global variables monitor, monitors, imgMonNumInd, window, and isFullScreen are
 *       used to control the behavior of this function.
 *       Will only exicute if monotor parameters have changed.
 *
 * @param p_window_id Pointer to the GLFWwindow pointer that will be updated.
 * @param win_ind Index of the window for which the setup is to be done.
 * @param pp_r_monitor_id Reference to the GLFWmonitor pointer array.
 * @param mon_id_ind Index of the monitor to move the window to.
 * @param is_fullscreen Boolean flag indicating whether the window should be set to full-screen mode.
 *
 * @return 0 on successful execution, -1 on failure.
 */
int updateWindowMonMode(GLFWwindow *, int, GLFWmonitor **&, int, bool);

/**
 * @brief Draws a textured rectangle using OpenGL.
 *
 * @param quad_vertices_vec Vector of vertex/corner points for a rectangular image.
 *
 * @return 0 if no errors, -1 if error.
 */
int drawQuadImage(std::vector<cv::Point2f>);

/**
 * @brief Draws walls on the OpenGL window.
 *
 * This function is responsible for drawing the walls on the OpenGL window.
 * It iterates through each calibration mode and each wall in the maze to
 * draw the corresponding image.
 *
 * @param proj_ind Index of the projector being used.
 * @param mon_iind Index of the projector monitor being used.
 * @param p_window_id Pointer to the GLFW window.
 * @param fbo_texture_id Framebuffer Object's texture ID.
 * @param r_image_id_vec Reference to the vector containing image IDs.
 *
 * @return Returns 0 on success, -1 otherwise.
 */
int drawWalls(int, int, GLFWwindow *, GLuint, std::vector<ILuint> &);

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
