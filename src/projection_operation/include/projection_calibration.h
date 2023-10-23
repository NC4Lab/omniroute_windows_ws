// ##########################################################################################################

// ======================================== projection_calibration.h ========================================

// ##########################################################################################################

#ifndef _PROJECTION_CALIBRATION_H
#define _PROJECTION_CALIBRATION_H

// ================================================== INCLUDE ==================================================

// Local custom libraries
#include "projection_utils.h"

// ================================================== VARIABLES ==================================================

// Specify the window name
std::string windowName = "Projection Calibration";

// Dynamic control point parameter arrays
float ctrlPointParams[4][5]; 

// Other variables related to control points
int cpSelectedInd = 0;
std::vector<float> cpActiveRGBVec = {0.0f, 1.0f, 0.0f}; // Active control point marker color (green)
std::vector<float> cpInactiveRGBVec = {1.0f, 0.0f, 0.0f}; // Inactive control point marker color (red)
std::string calModeStr = "position"; // Parmeter being modified [position, dimension, shear]

// The 3x3 homography matrix of 32-bit floating-point numbers used to warp perspective.
cv::Mat homMat = cv::Mat::eye(3, 3, CV_32F);

// Directory paths
std::string image_wall_dir_path = IMAGE_TOP_DIR_PATH + "/calibration_images";
std::string image_state_dir_path = IMAGE_TOP_DIR_PATH + "/ui_state_images";

// Test image variables
std::vector<ILuint> imgWallIDVec; // Container to hold the loaded images
std::vector<std::string> imgWallPathVec = {
    // List of image file paths
    image_wall_dir_path + "/1_test_pattern.bmp",
    image_wall_dir_path + "/2_all_white.bmp",
    image_wall_dir_path + "/3_manu_pirate.bmp",
    image_wall_dir_path + "/4_earthlings.bmp",
};
int imgWallInd = 0;                      // Index of the image to be loaded

// Monitor variables
std::vector<ILuint> imgMonIDVec; // Container to hold the loaded images for ui
std::vector<std::string> imgMonPathVec = {
    // List of monitor number image file paths
    image_state_dir_path + "/m0.bmp",
    image_state_dir_path + "/m1.bmp",
    image_state_dir_path + "/m2.bmp",
    image_state_dir_path + "/m3.bmp",
    image_state_dir_path + "/m4.bmp",
    image_state_dir_path + "/m5.bmp",
};
int winMonInd = 0;         // Index of the image to be loaded
int nMonitors;             // Number of monitors connected to the system
bool isFullScreen = false; // Flag to indicate if the window is in full screen mode

// Control point parameter image variables for ui
std::vector<ILuint> imgParamIDVec; // Container to hold the loaded images for ui
std::vector<std::string> imgParamPathVec = {
    // List of calibration parameter image file paths
    image_state_dir_path + "/p.bmp",
    image_state_dir_path + "/d.bmp",
    image_state_dir_path + "/s.bmp",
};
int imgParamInd = 0; // Index of the image to be loaded

// Callibration image variables for ui
std::vector<ILuint> imgCalIDVec; // Container to hold the loaded images for ui
std::vector<std::string> imgCalPathVec = {
    // List of mode image file paths
    image_state_dir_path + "/c-wl.bmp", // left walls
    image_state_dir_path + "/c-wm.bmp", // middle walls
    image_state_dir_path + "/c-wr.bmp", // right walls
    // image_state_dir_path + "/c-f.bmp",  // maze floor
    // image_state_dir_path + "/c-d.bmp",  // distal cues
};
int calModeInd = 1;                      // Index of the image to be loaded
size_t nCalModes = imgCalPathVec.size(); // Number of calibration modes

// Variables related to window and OpenGL
GLFWwindow *p_windowID = nullptr;
GLFWmonitor **pp_monitorIDVec = nullptr;

// ================================================== FUNCTIONS ==================================================

/**
 * @brief GLFW key callback function to handle key events and execute corresponding actions.
 *
 * This function is set as the GLFW key callback and gets called whenever a key event occurs.
 * It handles various key events for control points, monitor handling, XML operations, and more.
 *
 * ## Keybindings:
 * @see README.md
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
 * @return 0 if no errors, -1 if error.
 */
int updateWindowMonMode(GLFWwindow *, int, GLFWmonitor **&, int, bool);

/**
 * @brief Draws a control point as a quadrilateral using OpenGL.
 *
 * This function uses OpenGL to draw a quadrilateral that represents a control point.
 * The control point is drawn as a colored circle.
 *
 * @param x The control point x-coordinate.
 * @param y The control point y-coordinate.
 * @param radius The radius of the control point.
 * @param rgb_vec Vector of rgb values to color the marker.
 *
 * @return 0 if no errors, -1 if error.
 */
int drawControlPoint(float, float, float, std::vector<float>);

/**
 * @brief Draws a textured rectangle using OpenGL.
 *
 * @param quad_vertices_vec Vector of vertex/corner points for a rectangular image.
 *
 * @return 0 if no errors, -1 if error.
 */
int drawQuadImage(std::vector<cv::Point2f>);

/**
 * @brief Renders a 2D maze grid by drawing each cell (e.g., wall) with texture mapping and perspective warping.
 *
 * This function is a core part of the maze visualization pipeline. It utilizes the OpenGL graphics library for rendering,
 * and the DevIL library for image handling. The function performs several key operations:
 * 1. Texture mapping of the wall images.
 * 2. Perspective warping based on a precomputed homography matrix.
 * 3. Shear and height adjustments based on control point calibration.
 * 4. Optional overlay of status images for cells corresponding to selected control points.
 * 
 * @section Control Point and Grid Correspondence
 * 
 * This function employs a set of control points for calibration. The table below describes 
 * the correspondence between the displayed image, Normalized Device Coordinates (NDC), 
 * control points, and grid indices:
 * 
 * - Top-Left:      NDC (-1, 1),    Control Point [0],  Grid Index [0][s-1]
 * - Top-Right:     NDC (1, 1),     Control Point [1],  Grid Index [s-1][s-1]
 * - Bottom-Right:  NDC (1, -1),    Control Point [2],  Grid Index [s-1][0]
 * - Bottom-Left:   NDC (-1, -1),   Control Point [3],  Grid Index [0][0]
 * 
 * @param r_hom_mat The 3x3 homography matrix used for perspective warping of the walls.
 * @param ctrl_point_params A 4x5 array containing control point parameters (x, y, width, height, shear).
 * @param fbo_texture_id OpenGL framebuffer object's texture ID.
 * @param img_wall_id DevIL image ID for the base wall image.
 * @param img_mode_mon_id DevIL image ID for the monitor mode image.
 * @param img_mode_param_id DevIL image ID for the parameter visualization image.
 * @param img_mode_cal_id DevIL image ID for the calibration image.
 *
 * @return Integer status code: 0 if successful, -1 if an error occurred.
 */
int drawWalls(cv::Mat &, float[4][5], GLuint, ILuint, ILuint, ILuint, ILuint);

/**
 * @brief  Entry point for the projection_calibration ROS node.
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