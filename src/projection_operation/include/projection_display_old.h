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
 */
static struct FlagStruct
{
    bool change_window_mode = false;  // Flag to indicate if all window modes needs to be updated
    bool update_textures = false;     // Flag to indicate if wall vertices, homography and texture need to be updated
    bool windows_set_to_proj = false; // Flag to indicate if the windows are set to their respective projectors
    bool fullscreen_mode = false;     // Flag to indicate if the window is in full screen mode
} F;

/**
 * @brief Struct for global indices.
 */
static struct IndStruct
{
    int starting_monitor = 0; // Default starting monitor index for the windows
    std::vector<int> proj_mon_vec = {
        0,
        1,
    }; // Index of the monitor associeted to each projector (hardcoded)

} I;

/**
 * @brief Struct for global counts
 */
static struct CountStruct
{
    int monitors;             // Number of monitors connected to the system
    const int projectors = 2; // Number of projectors  (hardcoded)
    const int wall_image = 6; // Number of wall images
} N;

/**
 * @brief A n_projectors sized element veoctor containing a 3x3x3 data contianer for storing 3x3 homography matrices (UGLY!)
 */
std::vector<std::array<std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>, N_CAL_MODES>> HMAT_ARR_VEC(N.projectors);

/**
 * @brief  Array of OpenGL context objects.
 */
std::vector<MazeRenderContext> PROJ_CTX_VEC(N.projectors);

/**
 * @brief  Marker for masking rat.
 */
CircleRenderer ratMaskCircRend;

// Rat mask graphics parameters
cv::Point2f rmPosition = cv::Point2f(0.0f, 0.0f);      // Marker center
const GLfloat rmMakerRadius = 0.0025f;                 // Default control point rendered circle radius
const cv::Scalar rmRGB = cv::Scalar(0.0f, 0.0f, 0.0f); // Marker color (black)
const int rmRenderSegments = 36;                       // Number of segments used to approximate the circle geometry

/**
 * @brief Image file sub-directory path
 */
std::string runtime_wall_image_path = IMAGE_TOP_DIR_PATH + "/runtime";

/**
 * @brief Offset for the window position
 */
std::vector<cv::Point> winOffsetVec;

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
    runtime_wall_image_path + "/f_chambers.png",
};

// Vectors to store the loaded images in cv::Mat format
std::vector<cv::Mat> wallImgMatVec; // Vector of wall image texture matrices
std::vector<cv::Mat> floorImgMatVec; // Vector of floor image texture matrices


// ================================================== FUNCTIONS ==================================================

/**
 * @brief Use MazeRenderContext::checkKeyInput to get and process
 * key press events for all windows.
 *
 * @return Integer status code [-1:error, 0:no change, 1:new input].
 */
int procKeyPress();

/**
 * @brief Converts an OpenCV Mat image into an OpenGL texture and returns the texture ID.
 *
 * @param img_mat The cv::Mat image that needs to be converted.
 * @param texture_id Reference to the GLuint ID of the generated texture.
 *
 * @return Integer status code [-1:error, 0:successful].
 *
 * @details
 * This function takes an OpenCV Mat image as input and converts it into an OpenGL texture.
 * The OpenCV image is first converted from BGR to RGB format. Then, a new OpenGL texture is
 * generated and the converted image data is stored in this texture.
 *
 * The texture parameters for minification and magnification filters are set to GL_LINEAR.
 *
 * @note This function assumes that the input image is of type CV_8UC3 and has no alpha channel.
 */
int loadTexture(
    cv::Mat img_mat,
    GLuint &texture_id);
    
/**
 * @brief Applies the homography matrices to warp wall image textures and combine them.
 *
 * @param _proj_mon_ind Index of the monitor associated to the projector.
 * @param _wallImgMatVec Vectors containing the loaded images in cv::Mat format
 * @param _HMAT_ARR_VEC Big ass ugly vector of arrays of matrices of shit!
 * @param[out] out_progGL MazeRenderContext OpenGL context handler.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int updateTexture(
    int proj_mon_ind,
    const std::vector<cv::Mat> &_wallImgMatVec,
    const std::vector<std::array<std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>, N_CAL_MODES>> &_HMAT_ARR_VEC,
    MazeRenderContext &out_progGL);

/**
 * @brief Initializes the variables for the application.
 *
 * Just some shit.
 *
 * @throws std::runtime_error.
 */
void appInitVariables();

/**
 * @brief Loads the images and homography matices array for the application.
 *
 * This function uses OpenCV to load wall images into memory.
 * It uses xmlLoadHMAT() to load the homography matrices from XML files.
 *
 * @throws std::runtime_error if image or xml loading fails.
 */
void appLoadAssets();

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
