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

// Create an associateed 4x4 array of the CircleRenderer class objects
std::array<std::array<CircleRenderer, 4>, 4> CP_RENDERERS;

/**
 * @brief  3x3 data contianer for storing the 3x3 homography matrix for each wall image
 */
std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> HMAT_GRID_ARR;

// Struct for global flags
static struct FlagStruct
{
    bool dbRun = false;                   // Flag to indicate if something should be run for debugging
    bool loadXML = false;                 // Flag to indicate if the XML file needs to be loaded
    bool saveXML = false;                 // Flag to indicate if the XML file needs to be saved
    bool switchWindowMode = false;        // Flag to indicate if the window mode needs to be updated
    bool initControlPointMarkers = false; // Flag to indicate if the control point markers need to be reinitialized
    bool updateWallTexture = false;       // Flag to indicate if wall vertices, homography and texture need to be updated
    bool setFullscreen = false;           // Flag to indicate if the window needs to be set to full screen mode
} F;

// Struct for global counts
static struct CountStruct
{
    int monitors;       // Number of monitors connected to the system
    int wallImages = 4; // Number of wall images
    int calModes = 3;   // Number of calibration modes
} N;

// Struct for global indices
static struct IndStruct
{
    int wallImage = 0; // Index of the image to be loaded
    int calMode = 1;   // Index of the current calibration mode walls[0: left, 1: middle, 2: right]
    int winMon = 0;    // Index of the active monitor to be loaded
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
    std::array<std::array<int, 2>, 2> cpMap = {{{0, 1},
                                                {3, 2}}};
    std::array<int, 2> cpMazeVertSel = {0, 0}; // Selected maze vertex [row, col]
    std::array<int, 2> cpWallVertSel = {0, 0}; // Selected wall vertex [row, col]
    const int cpWVOrigin = 3;                  // Vertex index of the wall image origin (bottom-left)
    // std::array<int, 2> cpSelected = {0, 0};
} I;

// Sub-directory paths
std::string calib_image_path = IMAGE_TOP_DIR_PATH + "/calibration";

// Image file paths
std::vector<std::string> wallImgPathVec = {
    // List of image file paths
    calib_image_path + "/1_test_pattern.png",
    calib_image_path + "/2_manu_pirate.png",
    calib_image_path + "/3_earthlings.png",
    calib_image_path + "/4_all_white.png",
};
std::vector<std::string> monImgPathVec = {
    // List of monitor number image file paths
    calib_image_path + "/m0.png",
    calib_image_path + "/m1.png",
    calib_image_path + "/m2.png",
    calib_image_path + "/m3.png",
    calib_image_path + "/m4.png",
    calib_image_path + "/m5.png",
};
std::vector<std::string> calImgPathVec = {
    // List of mode image file paths
    calib_image_path + "/cwl.png", // left walls
    calib_image_path + "/cwm.png", // middle walls
    calib_image_path + "/cwr.png", // right walls
};

// Vectors to store the loaded images in cv::Mat format
std::vector<cv::Mat> wallImgMatVec; // Vector of wall image texture matrices
std::vector<cv::Mat> monImgMatVec;  // Vector of monitor mode image texture matrices
std::vector<cv::Mat> calImgMatVec;  // Vector of calibration mode image texture matrices

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
 * @param[out] out_CP_GRID_ARR Reference to the 4x4 array containing the coordinates of the corner wall's vertices.
 *
 * @details
 * Control point x and y coordinates are specified in Normalized Device Coordinates (NDC) [-1, 1].
 * The vertices for the entire projected image are calculated based on the dimensions that enclose
 * all control points (i.e., boundary dimensions in the control point plane).
 */
void initCtrlPtCoords(std::array<std::array<cv::Point2f, 4>, 4> &);

/**
 * @brief Loads PNG images with alpha channel from specified file paths and stores them in a vector as cv::Mat objects.
 *
 * @param img_paths_vec A vector of file paths to the images to be loaded.
 * @param[out] out_img_mat_vec Reference to a vector of cv::Mat where the loaded images will be stored.
 *
 * @return Integer status code [0:successful, -1:error].
 *
 * @details
 * This function takes a vector of file paths and iteratively loads each PNG image
 * with an alpha channel using OpenCV's imread function. It checks that the image
 * has the correct dimensions and the presence of an alpha channel. The function
 * returns 0 if all images are loaded successfully and -1 if any error occurs.
 *
 * Note on exporting from Adobe Illustrator:
 * - Select the Export As function and select PNG format.
 * - Tick the 'Use Arboard' option and click 'Export'.
 * - Set the 'Resolution' to 'Screen (72 ppi)' or 'Other' and then input '72 PPI'
 * - Set the 'Anti-aliasing' option ', choose 'Art Optimized (Supersampling)'.
 * - Tick the 'Transparency' checkbox in the PNG export options to include the alpha channel.
 * - Confirm the PNG is exported with a bit depth that supports alpha (typically PNG-24).
 */
int loadImgMat(const std::vector<std::string> &img_paths_vec, std::vector<cv::Mat> &out_img_mat_vec);

/**
 * @brief Merges a mask image over a base image using the alpha channel and stores the result.
 *
 * @param base_img_path Output cv::Mat containing the base image.
 * @param[out] out_base_img cv::Mat containing the base image for merging.
 *
 * @return Integer status code [0:successful, -1:error].
 *
 * @details
 * This function overlays the mask image on top of the base image using the alpha channel
 * of the mask image. Pixels from the mask image are copied over to the base image based on
 * the alpha value - if the alpha value is not fully transparent (0), the pixel is copied.
 */
int mergeImgMat(const cv::Mat &mask_img, cv::Mat &out_base_img);

/**
 * @brief Initialize OpenGL resources for CircleRenderer objects.
 *
 * @param _CP_GRID_ARR The 4x4 array containing the coordinates of the corner wall's vertices.
 * @param[out] out_CP_RENDERERS The 4x4 array of CircleRenderer objects used to draw the control points.
 *
 * @return Integer status code [0:successful, -1:error].
 *
 * @details
 * Initializes the vertex buffer, shader program, and default values
 * for the control point markers.
 */
int initCircleRendererObjects(const std::array<std::array<cv::Point2f, 4>, 4> &, std::array<std::array<CircleRenderer, 4>, 4> &);

/**
 * @brief Updates the stored warped wall image vertices based on the control point array.
 *
 * @param img_wall_mat cv::Mat image matrix for the base wall image.
 * @param img_mode_mon_mat cv::Mat image matrix for the monitor mode image.
 * @param img_mode_cal_mat cv::Mat image matrix for the calibration image.
 * @param _HMAT_GRID_ARR 3x3 array of Homography matrices used to warp the wall image.
 * @param[out] out_wallTexture OpenGL texture ID for the wall image.
 *
 * @return Integer status code [0:successful, -1:error].
 */
int updateWallTexture(
    cv::Mat, cv::Mat, cv::Mat,
    std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> &,
    GLuint &);

/**
 * @brief Converts an OpenCV Mat image into an OpenGL texture and returns the texture ID.
 *
 * @param image The cv::Mat image that needs to be converted.
 *
 * @return GLuint ID of the generated texture.
 *
 * @details
 * This function takes an OpenCV Mat image as input and converts it into an OpenGL texture.
 * The OpenCV image is first converted from BGR to RGB format. Then, a new OpenGL texture is
 * generated and the converted image data is stored in this texture.
 *
 * The texture parameters for minification and magnification filters are set to GL_LINEAR.
 *
 * Note: This function assumes that the input image is of type CV_8UC3 and has no alpha channel.
 */

GLuint loadTexture(cv::Mat);

/**
 * @brief Renders a all wall images from the computed texture2D maze grid by drawing each cell (e.g., wall) with texture mapping and perspective warping.
 *
 * @param renCtx Reference to an instance of the out_renCtx class.
 *
 * @return Integer status code  [0:successful, -1:error].
 */
int renderWallImage(const MazeRenderContext &renCtx);

/**
 * @brief Draws control points associated with each corner wall.
 *
 * @param _CP_GRID_ARR The control point coordinates used to warp the wall image.
 * @param[out] out_CP_RENDERERS The 4x4 array of CircleRenderer objects used to draw the control points.
 *
 * @return Integer status code [0:successful, -1:error].
 */
int renderControlPoints(const std::array<std::array<cv::Point2f, 4>, 4> &, std::array<std::array<CircleRenderer, 4>, 4> &);

/**
 * @brief  Entry point for the projection_calibration ROS node.
 *
 *
 * @param  argc  Number of command-line arguments.
 * @param  argv  Array of command-line arguments.
 *
 * @return Integer status code [0:successful, -1:error].
 *
 * @details
 * This program initializes ROS, DevIL, and GLFW, and then enters a main loop
 * to handle image projection and calibration tasks.
 */
int main(int, char **);

#endif