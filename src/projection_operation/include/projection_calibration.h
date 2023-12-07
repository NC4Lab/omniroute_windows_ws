// ##########################################################################################################

// ======================================== projection_calibration.h ========================================

// ##########################################################################################################

#ifndef _PROJECTION_CALIBRATION_H
#define _PROJECTION_CALIBRATION_H

// ================================================== INCLUDE ==================================================

// Local custom libraries
#include "projection_utils.h"

// ================================================== VARIABLES ==================================================

// Control point graphics
const int cpRenderSegments = 36;                                       // Number of segments used to approximate the circle geometry
const cv::Scalar cpWallVertSelectedRGB = cv::Scalar(1.0f, 0.0f, 0.0f); // Select control point marker color (red)
const cv::Scalar cpMazeVertSelectedRGB = cv::Scalar(0.0f, 1.0f, 0.0f); // Selected control point wall color (green)
const cv::Scalar cpDefaultRGB = cv::Scalar(0.0f, 0.0f, 1.0f);          // Default control point marker color (blue)
const GLfloat cpDefualtMakerRadius = 0.0025f;                          // Default control point rendered circle radius
const GLfloat cpSelectedMakerRadius = 0.005f;                          // Selected control point rendered circle radius

/**
 * @brief  4x4 data container for tracking the vertex coordinates for the corner wall images which are used as control point.
 *
 * The array stores the corner wall vertex as OpenGL's Normalized Device Coordinates (NDC), which range from [-1, 1]
 * with the origin at the center of the screen.
 *
 * @details
 *[4][4] = [conrol point][vertex]
 *
 * - Dimension 1: Control Point [0, 1, 2, 3]
 * - Index with respect to the overall maze verteces
 *
 * - Dimension 2: Vertex(x, y) [0, 1, 2, 3]
 * - Index with respect to the wall image verteces
 *
 * - Note the mind-fucking vertex indices please:
 *      - 0: OpenGL: Top-left       OpenCV: Bottom-left
 *      - 1: OpenGL: Top-right      OpenCV: Bottom-right
 *      - 2: OpenGL: Bottom-right   OpenCV: Top-right
 *      - 3: OpenGL: Bottom-left    OpenCV: Top-left
 */
std::array<std::array<cv::Point2f, 4>, 4> CP_GRID_ARR;

/**
 * @brief 3x3x3 data contianer for storing wall homography matrices for each wall image and each calibration mode.
 * 
 * @note We remove calibration mode dimension.
 */ 
std::array<std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>, N_CAL_MODES> WALL_HMAT_ARR; // Wall homography matrix array

/**
 * @brief  OpenGL context objects.
 */
MazeRenderContext projCtx;

/**
 * @brief  4x4 array of the CircleRenderer class objects.
 */
std::array<std::array<CircleRenderer, 4>, 4> CP_GLOBJ_ARR;

/**
 * @brief Struct for global flags.
 */
static struct FlagStruct
{
    bool db_run = false;                  // Flag to indicate if something should be run for debugging
    bool xml_load_hmat = false;           // Flag to indicate if the XML file needs to be loaded
    bool xml_save_hmat = false;           // Flag to indicate if the XML file needs to be saved
    bool change_window_mode = false;      // Flag to indicate if the window mode needs to be updated
    bool init_control_points = false;     // Flag to indicate if the control point markers need to be reinitialized
    bool update_wall_textures = false;    // Flag to indicate if wall vertices, homography and texture need to be updated
    bool update_wall_homographys = false; // Flag to indicate if wall vertices, homography and texture need to be updated
    bool fullscreen_mode = false;         // Flag to indicate if the window is in full screen mode
} F;

/**
 * @brief Struct for global counts.
 */
static struct CountStruct
{
    int monitors;              // Number of monitors connected to the system
    const int wall_images = 4; // Number of wall images
} N;

/**
 * @brief Struct for global indices.
 */
static struct IndStruct
{
    int wall_image = 0; // Index of the image to be loaded
    int cal_mode = 1;   // Index of the current calibration mode walls[0: left, 1: middle, 2: right]
    int monitor = 0;    // Index of the active monitor to be loaded
    /**
     * @brief cpRowColMap maps a given row cpRowColMap[0] and column cpRowColMap[1] index to the 1D vector.
     * To make things more complicated, this needs to account for the y axis being flipped.
     *
     * @details
     * - Maze/wall vertex indices
     *      - 0: OpenGL: Top-left       OpenCV: Bottom-left
     *      - 1: OpenGL: Top-right      OpenCV: Bottom-right
     *      - 2: OpenGL: Bottom-right   OpenCV: Top-right
     *      - 3: OpenGL: Bottom-left    OpenCV: Top-left
     */
    std::array<std::array<int, 2>, 2> cp_map = {{{0, 1},
                                                 {3, 2}}};
    std::array<int, 2> cp_maze_vert_selected = {0, 0}; // Selected maze vertex [row, col]
    std::array<int, 2> cp_wall_vert_selected = {1, 0}; // Selected wall vertex [row, col]
    const int cp_wall_origin_vertex = 3;               // Vertex index of the wall image origin (bottom-left)
    // std::array<int, 2> cpSelected = {0, 0};
} I;

// Sub-directory paths
std::string calib_image_path = IMAGE_TOP_DIR_PATH + "/calibration";

// Image file paths
std::vector<std::string> fiImgPathWallVec = {
    // List of image file paths
    calib_image_path + "/0_tp_wall.png",
    calib_image_path + "/1_tp_wall.png",
    calib_image_path + "/2_tp_wall.png",
    calib_image_path + "/3_tp_wall.png",
};
std::vector<std::string> fiImgPathMonVec = {
    // List of monitor number image file paths
    calib_image_path + "/m0.png",
    calib_image_path + "/m1.png",
    calib_image_path + "/m2.png",
    calib_image_path + "/m3.png",
    calib_image_path + "/m4.png",
    calib_image_path + "/m5.png",
};
std::vector<std::string> fiImgPathCalVec = {
    // List of mode image file paths
    calib_image_path + "/cwl.png", // left walls
    calib_image_path + "/cwm.png", // middle walls
    calib_image_path + "/cwr.png", // right walls
};

// Vectors to store the loaded images in cv::Mat format
std::vector<cv::Mat> wallImgMatVec; // Vector of wall image texture matrices
std::vector<cv::Mat> monImgMatVec;  // Vector of monitor mode image texture matrices
std::vector<cv::Mat> calImgMatVec;  // Vector of calibration mode image texture matrices

// Scalar to store the floor image in cv::Mat format
cv::Mat floorImgMat; // Floor image texture matrix

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
void callbackKeyBinding(GLFWwindow *, int, int, int, int);

/**
 * @brief Initializes values for the verteces of the coner walls which will be used as calibraton control points.
 *
 * @param cal_ind Index of the active calibration mode.
 * @param[out] out_CP_GRID_ARR Reference to the 4x4 array containing the coordinates of the corner wall's vertices.
 *
 * @details
 * Control point x and y coordinates are specified in Normalized Device Coordinates (NDC) [-1, 1].
 * The vertices for the entire projected image are calculated based on the dimensions that enclose
 * all control points (i.e., boundary dimensions in the control point plane).
 */
void initControlPoints(int, std::array<std::array<cv::Point2f, 4>, 4> &);

/**
 * @brief Initialize OpenGL resources for CircleRenderer objects.
 *
 * @param _CP_GRID_ARR The 4x4 array containing the coordinates of the corner wall's vertices.
 * @param[out] out_CP_RENDERERS The 4x4 array of CircleRenderer objects used to draw the control points.
 *
 * @return Integer status code [-1:error, 0:successful].
 *
 * @details
 * Initializes the vertex buffer, shader program, and default values
 * for the control point markers.
 */
int initCircleRendererObjects(const std::array<std::array<cv::Point2f, 4>, 4> &, std::array<std::array<CircleRenderer, 4>, 4> &);

/**
 * @brief Draws control points associated with each corner wall.
 *
 * @param _CP_GRID_ARR The control point coordinates used to warp the wall image.
 * @param[out] out_CP_RENDERERS The 4x4 array of CircleRenderer objects used to draw the control points.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int renderControlPoints(const std::array<std::array<cv::Point2f, 4>, 4> &, std::array<std::array<CircleRenderer, 4>, 4> &);

/**
 * @brief Computes updated homography matrices for all walls.
 *
 * @param cal_ind Index of the active calibration mode.
 * @param _CP_GRID_ARR The control point coordinates used to warp the wall image.
 * @param[out] out_WALL_HMAT_ARR Reference to array to store calibration matrices.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int updateWallHomographys(
    int cal_ind,
    const std::array<std::array<cv::Point2f, 4>, 4> &_CP_GRID_ARR,
    std::array<std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>, N_CAL_MODES> &out_WALL_HMAT_ARR);

/**
 * @brief Applies the homography matrices to warp wall image textures and combine them.
 *
 * @param cal_ind Index of the active calibration mode.
 * @param img_wall_mat cv::Mat image matrix for the base wall image.
 * @param img_mon_mat cv::Mat image matrix for the monitor mode image.
 * @param img_cal_mat cv::Mat image matrix for the calibration image.
 * @param _WALL_HMAT_ARR Array containing calibration matrices for the maze.
 * @param[out] out_wallTexture OpenGL texture ID for the wall image.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int updateWallTextures(
    int cal_ind,
    cv::Mat img_wall_mat, cv::Mat img_mon_mat, cv::Mat img_cal_mat,
    const std::array<std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>, N_CAL_MODES> &_WALL_HMAT_ARR,
    GLuint &out_WALL_TEXTURE_ID);

/**
 * @brief Initializes the datasets for the application.
 *
 * This function logs the setup parameters, initializes control
 * point coordinates, and computes wall homography matrices.
 *
 * @throws std::runtime_error if initialization fails.
 */
void appInitVariables();

/**
 * @brief Loads the necessary images for the application.
 *
 * This function uses OpenCV to load wall images, monitor number images,
 * and calibration mode images into memory.
 *
 * @throws std::runtime_error if image loading fails.
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
 * @brief Initalizes a new homography matrix xml file with
 * identity matrices for all elements if none exists.
 *
 * @throws std::runtime_error if image loading fails.
 */
void appInitFileXML();

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
 * @brief  Entry point for the projection_calibration ROS node.
 *
 *
 * @param  argc  Number of command-line arguments.
 * @param  argv  Array of command-line arguments.
 *
 * @return Integer status code [-1:error, 0:successful].
 *
 * @details
 * This program initializes ROS, DevIL, and GLFW, and then enters a main loop
 * to handle image projection and calibration tasks.
 */
int main(int, char **);

#endif