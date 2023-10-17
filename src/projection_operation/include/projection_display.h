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
float calParam[4][5];

// The homography matrix used to warp perspective.
cv::Mat H = cv::Mat::eye(3, 3, CV_32F);

// Directory paths
std::string image_wall_dir_path = IMAGE_TOP_DIR_PATH + "/runtime_images/shapes";

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

// Specify the window names for the projectors
std::vector<std::string> windowNameVec = {
    "Projector 0",
    "Projector 1",
    "Projector 2",
    "Projector 3",
};

// Monitor and projector variables
int nMonitors;             // Number of monitors (autopopulated)
const int nProjectors = 2; // Number of projectors  (autopopulated)
int indProjMonCalArr[nProjectors] = {
    // Index of the monitor for each projector (hardcoded)
    1,
    2,
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
 * @return 0 on successful execution, -1 on failure.
 */
int setupProjGLFW(GLFWwindow **, int, GLFWmonitor **&, int, const std::string &, GLuint &, GLuint &);

/**
 * @brief Draws a textured rectangle using OpenGL.
 *
 * @param rect_vertices_vec Vector of vertex/corner points for a rectangular image.
 */
void drawRectImage(std::vector<cv::Point2f>);

/**
 * @brief Draws walls on the OpenGL window.
 *
 * This function is responsible for drawing the walls on the OpenGL window.
 * It iterates through each calibration mode and each wall in the maze to
 * draw the corresponding image.
 *
 * @param ref_H Reference to the Homography Matrix.
 * @param cp_param Array containing control point parameters.
 * @param proj_i Index of the projector being used.
 * @param p_window_id Pointer to the GLFW window.
 * @param fbo_texture_id Framebuffer Object's texture ID.
 * @param ref_image_ids_vec Reference to the vector containing image IDs.
 *
 * @return Returns 0 on success, -1 otherwise.
 */
int drawWalls(cv::Mat&, float[4][5], int, GLFWwindow*, GLuint, std::vector<ILuint>&);


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
 * @param pp_ref_monitor_id Reference to the GLFWmonitor pointer array.
 * @param mon_ind Index of the monitor to move the window to.
 * @param do_fullscreen Boolean flag indicating whether the window should be set to full-screen mode.
 *
 * @return 0 on successful execution, -1 on failure.
 */
int updateWindowMonMode(GLFWwindow *, GLFWmonitor **&, int, bool);

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
