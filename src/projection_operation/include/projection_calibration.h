// ##########################################################################################################

// ======================================== projection_calibration.h ========================================

// ##########################################################################################################

#ifndef _PROJECTION_CALIBRATION_H
#define _PROJECTION_CALIBRATION_H

// ================================================== INCLUDE ==================================================

// Local custom libraries
#include "projection_utils.h"

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
void callbackFrameBufferSize(GLFWwindow *, int, int);

/**
 * @brief Callback function for handling errors.
 *
 * This function is called whenever an error occurs in the GLFW context.
 * It logs the error message using ROS_ERROR.
 *
 * @param error The error code.
 * @param description The error description.
 */
static void callbackError(int, const char *);

/**
 * @brief Draws a control point as a quadrilateral using OpenGL.
 *
 * This function uses OpenGL to draw a quadrilateral that represents a control point.
 * The control point is drawn in a clockwise direction, starting from the bottom-left corner.
 *
 * @param x The x-coordinate of the bottom-left corner of the control point.
 * @param y The y-coordinate of the bottom-left corner of the control point.
 * @param radius The radius of the control point.
 * @param rgb Vector of rgb values to color the marker.
 */
void drawControlPoint(float, float, float, std::vector<float>);

/**
 * @brief Draws a textured wall using OpenGL.
 *
 * @param img_vertices Vector of vertex/corner points for the wall.
 */
void drawWall(std::vector<cv::Point2f>);

/**
 * @brief Draws all walls in the maze grid with texture mapping and perspective warping.
 *
 * This function iterates through the maze grid to draw each wall. It uses the DevIL library
 * to handle image loading and OpenGL for rendering. The function also performs perspective 
 * warping based on the homography matrix and shear and height values extracted from control points.
 *
 * The function performs the following operations:
 * - Enables OpenGL texture mapping.
 * - Iterates through each cell in the maze grid.
 *   - Merges various images based on conditions and binds them.
 *   - Sets the texture image.
 *   - Calculates shear and height for the current wall.
 *   - Warps perspective based on the homography matrix.
 *   - Draws the wall.
 * - Disables OpenGL texture mapping.
 */
void drawWallsAll();

/**
 * @brief Loads images from specified file paths and stores their IDs in a reference vector.
 *
 * This function takes a vector of file paths (`ref_img_paths`) and iteratively loads each image
 * using the DevIL library. The function then stores the ILuint IDs of successfully loaded images
 * in a reference vector (`ref_image_ids`).
 *
 * @param ref_image_ids A reference to a vector of ILuint where the IDs of the loaded images will be stored.
 * @param ref_img_paths A reference to a vector of file paths to the images to be loaded.
 *
 * @note Utilizes the DevIL image library for image loading operations.
 */
void loadImgTextures(std::vector<ILuint> &, std::vector<std::string> &);

/**
 * @brief Merges two images by overlaying non-white pixels from the second image onto the first.
 *
 * This function takes two images, img1 and img2, represented as ILuint IDs. It overlays img2 onto img1,
 * replacing pixels in img1 with corresponding non-white pixels from img2. The resulting merged image is
 * returned as a new ILuint ID.
 *
 * @param img1 The ILuint ID of the baseline image.
 * @param img2 The ILuint ID of the mask image.
 * @return ILuint ID of the merged image. Returns 0 if an error occurs.
 *
 * @warning The dimensions of img1 and img2 must match.
 */
ILuint mergeImages(ILuint, ILuint);

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
 */
void updateWindowMonMode();

/**
 * @brief Creates a vector of points representing a rectangle with shear for each wall.
 *
 * This function generates a rectangle's corner points starting from the top-left corner
 * and going clockwise. The rectangle is defined by its top-left corner (x0, y0),
 * width, height, and a shear amount.
 *
 * @param x0 The x-coordinate of the top-left corner of the rectangle.
 * @param y0 The y-coordinate of the top-left corner of the rectangle.
 * @param width The width of the rectangle.
 * @param height The height of the rectangle.
 * @param shear_amount The amount of shear to apply to the rectangle.
 *
 * @return std::vector<cv::Point2f> A vector of 4 points representing the corners of the rectangle.
 */
std::vector<cv::Point2f> computeWallVertices(float, float, float, float, float);

/**
 * @brief Computes the homography matrix based on control points and wall image vertices.
 *
 * This function calculates the homography matrix that maps points from the source image (wall images)
 * to the destination image (control points). The homography matrix is stored in the global variable H.
 *
 * Control points are specified in normalized coordinates and are fetched from the global variable cpParam.
 * Wall image vertices are calculated based on the dimensions and spacing of the maze walls.
 *
 * @note This function uses the OpenCV library to compute the homography matrix.
 * @note The global variables cpParam, MAZE_SIZE, and wallSpace are used to control the behavior of this function.
 */
void computeHomography();

/**
 * @brief Used to reset control point parameter list.
 */
void resetParamCP();

/**
 * @brief Loads homography matrix from an XML file.
 *
 * This function is a wrapper that calls the 3-argument version with a dummy 2D array.
 *
 * @param ref_H Homography matrix to populate.
 * @param full_path Path to the XML file.
 */
void loadCoordinatesXML(cv::Mat& ref_H, std::string full_path);

/**
 * @brief Loads control points and homography matrix from an XML file.
 *
 * This is the primary function containing the implementation. It reads an XML file 
 * to populate the `ref_H` and `ref_cp_param` matrices.
 *
 * @note Uses pugiXML for XML parsing.
 *
 * @param ref_H Homography matrix to populate.
 * @param ref_cp_param 2D array for control points in normalized coordinates [-1, 1].
 * @param full_path Path to the XML file.
 */
void loadCoordinatesXML(cv::Mat& ref_H, float (&ref_cp_param)[4][5], std::string full_path);

/**
 * @brief Saves the control point positions and homography matrix to an XML file.
 *
 * This function uses the pugixml library to create an XML document and populate it with
 * the control point positions and homography matrix. The control point positions are stored in a 2D array
 * and the homography matrix is stored in a cv::Mat object. Both are saved under their respective
 * XML nodes.
 *
 * @note The XML file is saved to the path specified by the global variable 'configPath'.
 *
 * Example XML structure:
 * @code
 * <config>
 *   <cpParam>
 *     <Row>
 *       <Cell>value</Cell>
 *       ...
 *     </Row>
 *     ...
 *   </cpParam>
 *   <H>
 *     <Row>
 *       <Cell>value</Cell>
 *       ...
 *     </Row>
 *     ...
 *   </H>
 * </config>
 * @endcode
 *
 * @return void
 */
void saveCoordinatesXML();
//void loadCoordinatesXML(float [4][5], std::string);

/**
 * @brief  Entry point for the projection_calibration_node ROS node.
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

// ================================================== VARIABLES ==================================================

// Number of rows and columns in the maze
const int MAZE_SIZE = 3;

// Spricify window resolution: 4K resolution (3840x2160)
int winWidthPxl = 3840;
int winHeightPxl = 2160;
float winAspectRatio = (float)winWidthPxl / (float)winHeightPxl;

// Specify the window name
std::string windowName = "Projection Calibration";

// Variables related to control points
const float cpSize = 0.015f;
const float xy_lim = 0.5f;
const float ht_scale = winAspectRatio * 1.8 * 0.75;
cv::Mat H = cv::Mat::eye(3, 3, CV_32F);
int cpSelected = 0;
std::string cpModMode = "position";
std::vector<float> cpActiveRGB = {1.0f, 0.0f, 0.0f};   // Active control point marker color
std::vector<float> cpInactiveRGB = {0.0f, 0.0f, 1.0f}; // Inactive control point marker color
/**
 * @brief Array to hold the position and transformation parameters for control points in normalized coordinates [-1, 1].
 *
 * Each row corresponds to a specific control point on the screen, and each column holds a different attribute
 * of that control point.
 *
 * - Rows:
 *   - [0]: Top-left control point
 *   - [1]: Top-right control point
 *   - [2]: Bottom-right control point
 *   - [3]: Bottom-left control point
 *
 * - Columns:
 *   - [0]: X-distance from center [-1,1]
 *   - [1]: Y-distance from center [-1,1]
 *   - [2]: Width parameter
 *   - [3]: Height parameter
 *   - [4]: Shearing factor
 */
float cpParam_default[4][5] = {
    {-xy_lim, xy_lim, cpSize, cpSize *ht_scale, 0.0f}, // top-left control point
    {xy_lim, xy_lim, cpSize, cpSize *ht_scale, 0.0f},  // top-right control point
    {xy_lim, -xy_lim, cpSize, cpSize *ht_scale, 0.0f}, // bottom-right control point
    {-xy_lim, -xy_lim, cpSize, cpSize *ht_scale, 0.0f} // bottom-left control point
};
float cpParam[4][5];

// Wall image size and spacing (pixels)
int wallWidthPxl = 300;
int heightHeightPxl = 540;

// Wall image size and spacing (normalized?)
/// @todo: Figure out how these values are used
const float wallWidth = 0.02;
const float wallSpace = 2.5 * wallWidth;

// Directory paths
std::string packagePath = ros::package::getPath("projection_operation");
std::string workspacePath = packagePath.substr(0, packagePath.rfind("/src"));
std::string configDirPath = workspacePath + "/data/proj_cfg";
std::string imgTestPath = workspacePath + "/data/img/test_patterns";
std::string imgStatePath = workspacePath + "/data/img/ui_state_images";

// Test image variables
std::vector<ILuint> imgTestIDs; // Container to hold the loaded images
std::vector<std::string> imgTestPaths = {
    // List of test image file paths
    imgTestPath + "/1_test_pattern.bmp",
    imgTestPath + "/2_manu_pirate.bmp",
    imgTestPath + "/3_earthlings.bmp",
};
int imgTestInd = 0;                    // Index of the image to be loaded
size_t nTestImg = imgTestPaths.size(); // Number of test images

// Monitor variables
std::vector<ILuint> imgMonIDs; // Container to hold the loaded images for ui
std::vector<std::string> imgMonPaths = {
    // List of monitor number image file paths
    imgStatePath + "/m0.bmp",
    imgStatePath + "/m1.bmp",
    imgStatePath + "/m2.bmp",
    imgStatePath + "/m3.bmp",
    imgStatePath + "/m4.bmp",
    imgStatePath + "/m5.bmp",
};
int imgMonInd = 0;         // Index of the image to be loaded
int nMonitors;             // Number of monitors connected to the system
bool isFullScreen = false; // Flag to indicate if the window is in full screen mode

// Control point parameter image variables for ui
std::vector<ILuint> imgParamIDs; // Container to hold the loaded images for ui
std::vector<std::string> imgParamPaths = {
    // List of cp parameter image file paths
    imgStatePath + "/p.bmp",
    imgStatePath + "/d.bmp",
    imgStatePath + "/s.bmp",
};
int imgParamInd = 0; // Index of the image to be loaded

// Callibration image variables for ui
std::vector<ILuint> imgCalIDs; // Container to hold the loaded images for ui
std::vector<std::string> imgCalPaths = {
    // List of mode image file paths
    imgStatePath + "/c-wm.bmp",
    imgStatePath + "/c-wl.bmp",
    imgStatePath + "/c-wr.bmp",
    imgStatePath + "/c-f.bmp",
    imgStatePath + "/c-d.bmp",
};
int imgCalInd = 0;                     // Index of the image to be loaded
size_t nCalModes = imgCalPaths.size(); // Number of calibration modes

// Variables related to window and OpenGL
GLFWwindow *window;
GLuint fbo;
GLuint fboTexture;
GLFWmonitor *monitor = NULL;
GLFWmonitor **monitors;

#endif