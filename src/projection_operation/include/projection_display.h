// ######################################################################################################

// ======================================== projection_display.h ========================================

// ######################################################################################################

#ifndef _PROJECTION_DISPLY_H
#define _PROJECTION_DISPLY_H

// ================================================== INCLUDE ==================================================

// Local custom libraries
#include "projection_utils.h"

// ================================================== VARIABLES ==================================================

/**
 * @brief Struct for holding the 3x3 arrays of homography matrix for each calibratrion mode.
 */
struct HomographyStruct
{
    std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> wallGridLeft;
    std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> wallGridMiddle;
    std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> wallGridRight;
    std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> mazeFloor;
};

/**
 * @brief  4x3x3 data contianer for storing the 3x3 homography matrix for each wall image
 */
std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> HMAT_GRID_ARR;

/**
 * @brief  Array of OpenGL context objects.
 */
std::array<MazeRenderContext, 4> PROJ_GL_ARR;

/**
 * @brief Struct for global flags.
 */
static struct FlagStruct
{
    bool loadXML = false;           // Flag to indicate if the XML file needs to be loaded
    bool saveXML = false;           // Flag to indicate if the XML file needs to be saved
    bool switchWindowMode = false;  // Flag to indicate if the window mode needs to be updated
    bool updateWallTextures = false; // Flag to indicate if wall vertices, homography and texture need to be updated
    bool fullscreenMode = false;    // Flag to indicate if the window needs to be set to full screen mode
} F;

// Struct for global counts
static struct CountStruct
{
    int monitors;       // Number of monitors connected to the system
    int projectors = 2; // Number of projectors  (hardcoded)
    int wallImages = 6; // Number of wall images
} N;

// Specify the montior index for each prjector
std::vector<int> projMonIndArr = {
    // Index of the monitor associeted to each projector (hardcoded)
    1,
    2,
};

// Sub-directory paths
std::string wall_image_path = IMAGE_TOP_DIR_PATH + "/calibration";

std::vector<std::string> wallImgPathVec = {
    // List of image file paths
    wall_image_path + "/blank.bmp",    // [0] Blank image
    wall_image_path + "/square.bmp",   // [1] Square image
    wall_image_path + "/circle.bmp",   // [2] Circle image
    wall_image_path + "/triangle.bmp", // [3] Triangle image
    wall_image_path + "/star.bmp",     // [4] Star image
    wall_image_path + "/pentagon.bmp", // [5] Pentagon image
};

// Vectors to store the loaded images in cv::Mat format
std::vector<cv::Mat> wallImgMatVec; // Vector of wall image texture matrices

// ================================================== FUNCTIONS ==================================================

/**
 * @brief GLFW key callback function to handle key events and execute corresponding actions.
 *
 * @param window Pointer to the GLFW window that received the event.
 * @param key The keyboard key that was pressed or released.
 * @param scancode The system-specific scancode of the key.
 * @param action GLFW_PRESS, GLFW_RELEASE or GLFW_REPEAT.
 * @param mods Bit field describing which modifier keys were held down.
 */
void callbackKeyBinding(GLFWwindow *, int, int, int, int);

/**
 * @brief Loads the images and homography matices array for the application.
 * 
 * This function uses OpenCV to load wall images into memory.
 * It uses loadHMATxml() to load the homography matrices from XML files.
 * 
 * @throws std::runtime_error if image or xml loading fails.
 */
void appLoadData();

/**
 * @brief Initializes OpenGL settings and creates shader programs.
 * 
 * This function sets up the graphics libraries, initializes the rendering
 * context, and creates shader programs for wall image and control point rendering.
 * 
 * @throws std::runtime_error if OpenGL initialization fails.
 */
void appInitializeOpenGL();

/**
 * @brief The main loop of the application.
 * 
 * Handles the application's main loop, including checking keyboard callbacks,
 * updating window mode, and rendering frames. Exits on window close, escape key,
 * or when an error occurs.
 * 
 * @throws std::runtime_error if an error occurs during execution.
 */
void appMainLoop();

/**
 * @brief Cleans up resources upon application shutdown.
 * 
 * This function deletes the CircleRenderer class shader program, cleans up
 * OpenGL wall image objects, and terminates the graphics library.
 */
void appCleanup();

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
