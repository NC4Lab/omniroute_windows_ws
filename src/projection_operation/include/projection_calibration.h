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
float cpParam[4][5]; // Dynamic array to hold the control point parameters

// Other variables related to control points
int cpSelected = 0;
std::string cpModMode = "position";
std::vector<float> cpActiveRGBVec = {1.0f, 0.0f, 0.0f};   // Active control point marker color
std::vector<float> cpInactiveRGBVec = {0.0f, 0.0f, 1.0f}; // Inactive control point marker color

// The 3x3 homography matrix of 32-bit floating-point numbers used to warp perspective.
cv::Mat H = cv::Mat::eye(3, 3, CV_32F);

// Directory paths
std::string image_wall_dir_path = IMAGE_TOP_DIR_PATH + "/calibration_images";
std::string image_state_dir_path = IMAGE_TOP_DIR_PATH + "/ui_state_images";

// Test image variables
std::vector<ILuint> imgWallIDVec; // Container to hold the loaded images
std::vector<std::string> imgWallPathVec = {
    // List of image file paths
    image_wall_dir_path + "/1_test_pattern.bmp",
    image_wall_dir_path + "/2_manu_pirate.bmp",
    image_wall_dir_path + "/3_earthlings.bmp",
};
int imgWallInd = 0;                      // Index of the image to be loaded
size_t nWallImg = imgWallPathVec.size(); // Number of test images

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
    // List of cp parameter image file paths
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
int calModeInd = 0;                      // Index of the image to be loaded
size_t nCalModes = imgCalPathVec.size(); // Number of calibration modes

// Variables related to window and OpenGL
GLFWwindow *p_windowID;
GLFWmonitor **p_monitorIDVec;

// ================================================== FUNCTIONS ==================================================

/**
 * @brief GLFW key callback function to handle key events and execute corresponding actions.
 *
 * This function is set as the GLFW key callback and gets called whenever a key event occurs.
 * It handles various key events for control points, monitor handling, XML operations, and more.
 *
 * ## Keybindings:
 * - **Any Key Release Action:**
 *   - `R`: Reset control point parameters.
 *   - `F1-F4`: Select a control point (Top-Left, Top-Right, Bottom-Right, Bottom-Left).
 *   - `A, D, S`: Change calibration point parameters.
 *   - `F`: Toggle fullscreen mode.
 *   - `M`: Move window to another monitor.
 *   - `0-5`: Select monitor index.
 *   - `Enter`: Save coordinates to XML.
 *   - `L`: Load coordinates from XML.
 *
 * - **Any Key Press or Repeat Action:**
 *   - `Alt + Left/Right`: Change calibration mode.
 *   - `Ctrl + Left/Right`: Change displayed image.
 *   - `Shift or no modifier + Arrow keys`: Adjust control point position, dimension, or shear.
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
 * @brief Draws a control point as a quadrilateral using OpenGL.
 *
 * This function uses OpenGL to draw a quadrilateral that represents a control point.
 * The control point is drawn in a clockwise direction, starting from the bottom-left corner.
 *
 * @param x The x-coordinate of the bottom-left corner of the control point.
 * @param y The y-coordinate of the bottom-left corner of the control point.
 * @param radius The radius of the control point.
 * @param rgb_vec Vector of rgb values to color the marker.
 */
void drawControlPoint(float, float, float, std::vector<float>);

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
 *
 * @param ref_H The homography matrix used to warp perspective.
 * @param cp_param The array of control point parameters.
 * @param fbo_texture The OpenGL texture ID of the framebuffer object.
 * @param img_base_id The DevIL image ID of the base image.
 * @param img_mon_id The DevIL image ID of the monitor image.
 * @param img_param_id The DevIL image ID of the parameter image.
 * @param img_cal_id The DevIL image ID of the calibration image.
 */
void drawWalls(cv::Mat &, float[4][5], GLuint, ILuint, ILuint, ILuint, ILuint);

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
 * @param is_fullscreen Boolean flag indicating whether the window should be set to full-screen mode.
 */
void updateWindowMonMode(GLFWwindow *, GLFWmonitor **&, int, bool);

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