// ##########################################################################################################

// ======================================== projection_calibration.h ========================================

// ##########################################################################################################

#ifndef _PROJECTION_CALIBRATION_H
#define _PROJECTION_CALIBRATION_H

// ================================================== INCLUDE ==================================================

// Local custom libraries
#include "projection_utils.h"

// ================================================== VARIABLES ==================================================

// Data structure to hold stuff for each projector, row, column, and calibration mode
template <typename T>
using ProjectionMap = std::map<Projector, 
    std::map<Row, 
    std::map<Column,
    std::map<CalibrationMode,
    T>>>>;

// Map to store the indices of wall images
ProjectionMap<int> indexMap;

// Map to store all the homography matrices
ProjectionMap<cv::Mat> hMatMap;

/**
 * @brief  4x4 data container for tracking the vertex coordinates for the corner wall images which are used as control point.
 *
 * The array stores the corner wall vertex as OpenGL's Normalized Device Coordinates (NDC), which range from [-1, 1]
 * with the origin at the center of the screen.
 *
 * @details
 *[4][4] = [control point][vertex]
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
 * @brief 3x3x4 data contianer for storing default wall vertices for each wall in NDC.
 */
std::array<std::array<std::array<cv::Point2f, 4>, GLB_MAZE_SIZE>, GLB_MAZE_SIZE> WALL_GRID_ARR_DEFAULT;

/**
 * @brief 3x3x4 data contianer for storing wall homography matrices for each wall image (3x3)
 * and floor image (1x1) and each calibration mode (4).
 *
 * @details
 * [4][3][3] = [calibration modes][grid rows][grid columns]
 *
 * - Dimension 1: Calibration Mode [left walls, middle walls, right walls, floor]
 *
 * - Dimension 2: Grid Rows [0, 1, 2]
 * - Index with respect to the overall the 3 chambers
 *
 * - Dimension 3: Grid Columns [0, 1, 2]
 * - Index with respect to the overall the 3 chambers
 *
 * The overall structure stores homography matrices for all wall and floor images, across different calibration modes.
 * The inner-most element is a cv::Mat (3x3 matrix), which contains the homography data for a given wall image or the floor image.
 */
std::array<std::array<std::array<cv::Mat, GLB_MAZE_SIZE>, GLB_MAZE_SIZE>, N_CAL_MODES> HMAT_ARR;

/**
 * @brief  OpenGL context objects.
 */
MazeRenderContext projCtx;

/**
 * @brief  4x4 array of the CircleRenderer class objects.
 */
std::array<std::array<CircleRenderer, 4>, 4> CP_CIRCREND_ARR;

/**
 * @brief  Enum for tracking the current calibration mode
 */
CalibrationMode CAL_MODE = WALLS_MIDDLE;

// Control point graphics parameters
const GLfloat cpMakerRadius = 0.0025f;                                 // Control point rendered circle radius
const cv::Scalar cpWallVertSelectedRGB = cv::Scalar(1.0f, 0.0f, 0.0f); // Select control point marker color (red)
const cv::Scalar cpMazeVertSelectedRGB = cv::Scalar(0.0f, 1.0f, 0.0f); // Selected control point wall color (green)
const cv::Scalar cpDefaultRGB = cv::Scalar(0.0f, 0.0f, 1.0f);          // Default control point marker color (blue)
const int cpRenderSegments = 36;                                       // Number of segments used to approximate the circle geometry

/**
 * @brief Struct for flaging state changes.
 *
 * @details Flags update_mode_img and update_textures are initialized as true
 * to force the initial update of the mode image and displayed texture.
 */
static struct FlagStateStruct {
    bool xml_load_hmat = false;      // Flag to indicate if the XML file needs to be loaded
    bool xml_save_hmat = false;      // Flag to indicate if the XML file needs to be saved
    bool change_window_mode = false; // Flag to indicate if the window mode needs to be updated
    bool init_control_points = true; // Flag to indicate if the control point markers need to be reinitialized
    bool fullscreen_mode = false;    // Flag to indicate if the window is in full screen mode
    bool update_mode_img = true;     // Flag to indicate if the monitor and calibration mode image needs to be updated
    bool update_textures = true;     // Flag to indicate if image vertices, homography and texture need to be updated
    bool update_homographys = true;  // Flag to indicate if image vertices, homography and texture need to be updated
} F;

/**
 * @brief Struct for global indices.
 *
 * @details CP_MAP
 * - Maze/wall vertex indices
 *      - 0: OpenGL: Top-left       OpenCV: Bottom-left
 *      - 1: OpenGL: Top-right      OpenCV: Bottom-right
 *      - 2: OpenGL: Bottom-right   OpenCV: Top-right
 *      - 3: OpenGL: Bottom-left    OpenCV: Top-left
 * - Also accounts for the y axis being flipped
 */
static struct IndStruct {
    int monitor = 0;     // Index of the active monitor to be loaded
    int projector = -1;  // Index of the projector associated with the active monitor to be loaded
    int wall_image = 0;  // Index of the test wall image to be loaded
    int floor_image = 0; // Index of the test wall image to be loaded
    std::array<std::array<int, 2>, 2> CP_MAP = {{{0, 1},
                                                 {3, 2}}}; // Maps a given row and column cpRowColMap[1] index to the 1D control points vector.
    std::array<int, 2> cp_maze_vert_selected = {0, 0};     // Selected maze vertex [row, col]
    std::array<int, 2> cp_wall_vert_selected = {1, 0};     // Selected wall vertex [row, col]
    const int cp_wall_origin_vertex = 3;                   // Vertex index of the wall image origin (bottom-left)
    // std::array<int, 2> cpSelected = {0, 0};
} I;

// Vectors to store the loaded images in cv::Mat format
std::vector<cv::Mat> calibTestWallMats;     // Vector of wall test image texture matrices
std::vector<cv::Mat> calibTestFloorMats;    // Vector of floor test image texture matrices
std::vector<cv::Mat> calibMonWallMats;  // Vector of monitor mode image texture matrices for wall calibration
std::vector<cv::Mat> calibMonFloorMats; // Vector of monitor mode image texture matrices for floor calibration
std::vector<cv::Mat> calibModeMats;      // Vector of calibration mode image texture matrices

CalibrationXML calXML; // XML file handler

/**
 * @brief Matrix to store the mode image displaying the current calibration mode and monitor
 */
cv::Mat modeMat;

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
 * @brief Initializes values for the verteces of the coner walls which will be used as calibraton control points.
 *
 * @param _CAL_MODE Enum of type CalibrationMode for the active calibration mode.
 * @param[out] out_CP_GRID_ARR Reference to the 4x4 array containing the coordinates of the corner wall's vertices.
 * @param[out] out_WALL_GRID_ARR_DEFAULT Reference to 3x3x4 data contianer for storing default wall vertices for each wall.
 *
 * @details
 * Control point x and y coordinates are specified in Normalized Device Coordinates (NDC) [-1, 1].
 * The vertices for the entire projected image are calculated based on the dimensions that enclose
 * all control points (i.e., boundary dimensions in the control point plane).
 */
void initVertexCoordinates(
    CalibrationMode _CAL_MODE,
    std::array<std::array<cv::Point2f, 4>, 4> &out_CP_GRID_ARR,
    std::array<std::array<std::array<cv::Point2f, 4>, GLB_MAZE_SIZE>, GLB_MAZE_SIZE> &out_WALL_GRID_ARR_DEFAULT);

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
int initCircleRendererObjects(
    const std::array<std::array<cv::Point2f, 4>, 4> &_CP_GRID_ARR,
    std::array<std::array<CircleRenderer, 4>, 4> &out_CP_RENDERERS);

/**
 * @brief Draws control points associated with each corner wall.
 *
 * @param _CAL_MODE Enum of type CalibrationMode for the active calibration mode.
 * @param _CP_GRID_ARR The control point coordinates used to warp the wall image.
 * @param[out] out_CP_RENDERERS The 4x4 array of CircleRenderer objects used to draw the control points.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int drawControlPoints(
    CalibrationMode _CAL_MODE,
    const std::array<std::array<cv::Point2f, 4>, 4> &_CP_GRID_ARR,
    std::array<std::array<CircleRenderer, 4>, 4> &out_CP_RENDERERS);

/**
 * @brief Computes updated homography matrices for all walls.
 *
 * @param _CAL_MODE Enum of type CalibrationMode for the active calibration mode.
 * @param _CP_GRID_ARR The control point coordinates used to warp the wall image.
 * @param _WALL_GRID_ARR_DEFAULT The 3x3x4 data contianer of default wall vertices for each wall.
 * @param[out] out_HMAT_ARR Reference to array to store calibration matrices.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int updateWallHomographys(
    CalibrationMode _CAL_MODE,
    const std::array<std::array<cv::Point2f, 4>, 4> &_CP_GRID_ARR,
    const std::array<std::array<std::array<cv::Point2f, 4>, GLB_MAZE_SIZE>, GLB_MAZE_SIZE> &_WALL_GRID_ARR_DEFAULT,
    std::array<std::array<std::array<cv::Mat, GLB_MAZE_SIZE>, GLB_MAZE_SIZE>, N_CAL_MODES> &out_HMAT_ARR);

/**
 * @brief Computes updated homography matrices for the floor image.
 *
 * @param _CP_ARR The control point coordinates used to warp the floor image.
 * @param[out] out_H The output homography matrix.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int updateFloorHomography(
    const std::array<cv::Point2f, 4> &_CP_ARR,
    cv::Mat &out_H);

/**
 * @brief Update the mode image.
 *
 * @param img_base_mat cv::Mat image matrix for the base image.
 * @param img_mon_mat cv::Mat image matrix for the monitor mode image.
 * @param img_cal_mat cv::Mat image matrix for the calibration image.
 * @param[out] out_img_mode_mat Reference to cv::Mat image to store the merged mode image.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int updateModeImage(cv::Mat img_base_mat, cv::Mat img_mon_mat, cv::Mat img_cal_mat, cv::Mat &out_img_mode_mat);

/**
 * @brief Applies the homography matrices to warp image textures and combine them.
 *
 * @param img_base_mat cv::Mat image matrix for the base image.
 * @param img_mode_mat cv::Mat image matrix for the monitor mode image.
 * @param _CAL_MODE Enum of type CalibrationMode for the active calibration mode.
 * @param _HMAT_ARR Array containing calibration matrices for the maze.
 * @param[out] out_projCtx MazeRenderContext OpenGL context handler.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int updateTexture(
    cv::Mat img_base_mat,
    cv::Mat img_mode_mat,
    CalibrationMode _CAL_MODE,
    const std::array<std::array<std::array<cv::Mat, GLB_MAZE_SIZE>, GLB_MAZE_SIZE>, > &_HMAT_ARR,
    MazeRenderContext &out_projCtx);

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
 * @note Files need to be manually deleted if they need to be reinitialized.
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
int main(int argc, char **argv);

#endif