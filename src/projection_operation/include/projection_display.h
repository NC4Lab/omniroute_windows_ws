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
 * @brief Struct for global flags.
 *
 * @details Flag update_textures is initialized as true to force the
 * initial update of the displayed texture.
 */
static struct FlagStruct
{
    bool change_window_mode = false;  // Flag to indicate if all window modes needs to be updated
    bool windows_set_to_proj = false; // Flag to indicate if the windows are set to their respective projectors
    bool fullscreen_mode = false;     // Flag to indicate if the window is in full screen mode
    bool update_textures = true;      // Flag to indicate if wall vertices, homography and texture need to be updated
} F;

/**
 * @brief Struct for global indices.
 */
static struct IndStruct
{
    const int starting_monitor = 0; // Default starting monitor index for the windows (hardcoded)
    const std::vector<int> proj_mon_vec = {
        1,
        2,
    }; // Vector of indeces of the monitor associeted to each projector (hardcoded)

} I;

/**
 * @brief Struct for global counts
 */
static struct CountStruct
{
    int monitor;              // Number of monitors connected to the system
    const int projector = static_cast<int>(I.proj_mon_vec.size()); // Number of projectors
    const int wall_image = 6;                                      // Number of wall images
} N;

/**
 * @brief  Struct for rat mask tracking and graphics.
 */
static struct RatTracker
{
    cv::Point2f marker_position = cv::Point2f(0.0f, 0.0f);      // Marker center (cm)
    const GLfloat marker_radius = 5.0f;                         // Marker default circle radius (cm)
    const cv::Scalar marker_rgb = cv::Scalar(1.0f, 0.0f, 0.0f); // Marker color (black)
    const int marker_segments = 36;                             // Number of segments used to approximate the circle geometry
} RT;

/**
 * @brief A n_projectors sized element veoctor containing a 3x3x3 data contianer for storing 3x3 homography matrices (UGLY!)
 */
std::array<std::array<std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>, N_CAL_MODES>, 4> HMAT_ARR;

/**
 * @brief Array of homography matrices for warping the rat mask marker from maze cm to ndc space for each projector.
 */
std::array<cv::Mat, 4> HMAT_CM_TO_NDC_ARR;

/**
 * @brief  Array of marker for masking rat for each projector.
 */
std::array<CircleRenderer, 4> RM_CIRCREND_ARR;

/**
 * @brief  Array of OpenGL context objects.
 */
std::vector<MazeRenderContext> PROJ_CTX_VEC(N.projector);

/**
 * @brief Offset for the window position
 */
float winOffsetDefualt = 500.0f;

/**
 * @brief Offset for the window position
 */
std::vector<cv::Point> winOffsetVec;

/**
 * @brief Image file sub-directory path
 */
std::string runtime_wall_image_path = IMAGE_TOP_DIR_PATH + "/runtime";

/**
 * @brief List of wall image file paths
 */
std::vector<std::string> fiImgPathWallVec = {
    runtime_wall_image_path + "/w_blank.png",    // [0] Blank image
    runtime_wall_image_path + "/w_square.png",   // [1] Square image
    runtime_wall_image_path + "/w_circle.png",   // [2] Circle image
    runtime_wall_image_path + "/w_triangle.png", // [3] Triangle image
    runtime_wall_image_path + "/w_star.png",     // [4] Star image
    runtime_wall_image_path + "/w_pentagon.png", // [5] Pentagon image
};
/**
 * @brief List of floor image file paths
 */
std::vector<std::string> fiImgPathFloorVec = {
    runtime_wall_image_path + "/f_blank.png",
    runtime_wall_image_path + "/f_white.png",
    runtime_wall_image_path + "/f_chambers.png",
};

// Vectors to store the loaded images in cv::Mat format
std::vector<cv::Mat> wallImgMatVec;  // Vector of wall image texture matrices
std::vector<cv::Mat> floorImgMatVec; // Vector of floor image texture matrices

// ================================================== FUNCTIONS ==================================================

/**
 * @brief GLFW key callback function to handle key events and execute corresponding actions.
 *
 * @param window Pointer to the GLFW window that received the event.
 * @param key The keyboard key that was pressed or released.
 * @param scancode The system-specific scancode of the key.
 * @param action GLFW_PRESS, GLFW_RELEASE or GLFW_REPEAT.
 * @param mods Bit field describing which modifier keys were held down.
 *
 * @details
 * This function is set as the GLFW key callback and gets called whenever a key event occurs.
 * It handles various key events for control points, monitor handling, XML operations, and more.
 *
 * ## Keybindings:
 * @see README.md
 */
void callbackKeyBinding(
    GLFWwindow *window,
    int key,
    int scancode,
    int action,
    int mods);

/**
 * @brief Simulates rat movement.
 *
 * @param move_step Distance to move in cm.
 * @param max_turn_angle Maximum angle to turn in degrees.
 * @param[out] out_RT RatTracker struct object to be updated.
 */
void simulateRatMovement(
    float move_step,
    float max_turn_angle,
    RatTracker &out_RT);

/**
 * @brief Applies the homography matrices to warp wall image textures and combine them.
 *
 * @param _proj_mon_ind Index of the monitor associated to the projector.
 * @param _wallImgMatVec Vectors containing the loaded wall images in cv::Mat format
 * @param _floorImgMatVec Vectors containing the loaded floor images in cv::Mat format
 * @param _HMAT_ARR Big ass ugly array of arrays of matrices of shit!
 * @param[out] out_projCtx MazeRenderContext OpenGL context handler.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int updateTexture(
    int proj_mon_ind,
    const std::vector<cv::Mat> &_wallImgMatVec,
    const std::vector<cv::Mat> &_floorImgMatVec,
    const std::array<std::array<std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>, N_CAL_MODES>, 4> &_HMAT_ARR,
    MazeRenderContext &out_projCtx);

/**
 * @brief Draws control points associated with each corner wall.
 *
 * @param _RT RatTracker struct object.
 * @param[out] out_rmCircRend CircleRenderer objects used to draw the control points.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int drawRatMask(
    const RatTracker &_RT,
    CircleRenderer &out_rmCircRend);

/**
 * @brief Initializes the variables for the application.
 *
 * This function uses OpenCV to load wall images into memory.
 * It also loads and computes various parameters used in the library
 *
 * @throws std::runtime_error.
 */
void appInitVariables();

/**
 * @brief Initializes OpenGL settings and creates shader programs.
 *
 * This function sets up the graphics libraries, initializes the rendering
 * context, and creates shader programs for wall image and control point rendering.
 *
 * @throws std::runtime_error if OpenGL initialization fails.
 */
void appInitOpenGL();

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
