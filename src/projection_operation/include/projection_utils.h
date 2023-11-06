// ####################################################################################################

// ======================================== projection_utils.h ========================================

// ####################################################################################################

/**
 * @file projection_utils.h
 * @author Adam Lester
 * @date   Summer 2023
 * @brief @todo
 *
 * @section dependencies Dependencies
 * This library depends on the following libraries:
 *
 * - OpenGL for rendering graphics
 *
 * - OpenCV for image processing
 *
 * - DevIL for image loading and manipulation
 *      - The Developer's Image Library (DevIL) is a cross-platform image processing library that provides a simple and
 *           easy-to-use API for developers. It is written in C and aims to facilitate various image file operations, including loading,
 *          saving, conversion, and manipulation.
 *
 * @section introduction Introduction
 * Detailed library description.
 *
 * @section usage Usage
 * How to use this library.
 *
 * @section license License
 * Licensing information.
 *
 * @details Omniroute Chamber and Wall Image Layout
 *
 * - The 3x3 verstion of the maze consitsts of 9 octogonal 'Chambers' arranged in a 3x3 grid.
 *
 * - Each chamber is a is composed of 8 movable wall pannels measuring [W, H] [17cm, 10cm].
 *
 * - There are 4 projecors arrayed around the outside of the maze.
 *      - Mi 4K Laser Projector 150" Specifications
 *          - Display Technology: 0.47" DMD
 *          - Light Source: ALPD
 *          - Resolution: 4K (3840 × 2160)
 *          - Throw Ratio: 0.233
 *          - Screen Size: 80" to 150"
 *          - Frame Rate: 60 fps @todo check this
 *
 *
 *                                          ______
 *                                         |  p1  |
 *                                         |______|
 *                                           /__\
 *
 *                       ________          ________          ________
 *                     /    w2    \      /          \      /          \
 *                   / w1        w3 \  /              \  /              \
 *                  |                ||                ||                |
 *                  |w0    [c0]    w4||      [c1]      ||      [c2]      |
 *                  |                ||                ||                |
 *                   \  w7       w5 /  \              /  \              /
 *                     \ ___w6___ /      \ ________ /      \ ________ /
 *                     /          \      /          \      /          \
 *   ____            /              \  /              \  /              \            ____
 *  |    |/|        |                ||                ||                |        |\|    |
 *  | p0 | |        |      [c3]      ||      [c4]      ||      [c5]      |        | | p2  |
 *  |____|\|        |                ||                ||                |        |/|____|
 *                   \              /  \              /  \              /
 *                     \ ________ /      \ ________ /      \ ________ /
 *                     /          \      /          \      /          \
 *                   /              \  /              \  /              \
 *                  |                |                 ||                |
 *                  |      [c6]      ||      [c7]      ||      [c8]      |
 *                  |                ||                ||                |
 *                   \              /  \              /  \              /
 *                     \ ________ /      \ ________ /      \ ________ /
 *
 *                                            __
 *                                          _\__/_
 *                                         |  p3  |
 *                                         |______|
 *
 *
 *
 *  @details Omniroute Wall Projection Geometry
 *
 * - The walls requre 3 seperatate callibrations for each projector:
 *      - Left wall calibration (Lc)
 *      - Middle wall calibration (Mc)
 *      - Right wall calibration (Rc)

 * - For each calibration, 9 wall images are created.
 *
 *
 *
 *                  ________          ___Mc___          ________
 *                /          \   Lc /          \ Rc   /          \
 *               /   (0, 0)   \    /   (0, 1)   \    /   (0, 2)   \
 *
 *
 *                  ________          ________          ________
 *                /          \      /          \      /          \
 *               /   (1, 0)   \    /   (1, 1)   \    /   (1, 2)   \
 *
 *
 *                   ________          ________         ________
 *                 /          \      /          \     /          \
 *                /   (2, 0)   \    /   (2, 1)   \   /   (2, 2)   \
 *
 *
 *
 *                                        __
 *                                      _\__/_
 *                                     |      |
 *                                     |______|
 *
 *
 *
 *  @details Omniroute Wall Image Processing Parameters
 *
 * - Wall Layout:
 *     - 3x3 grid represented by 3 rows (R) and 3 columns (C)
 *
 * - Wall Vertices:
 *      - Indexed clockwise for the top left [0,1,2,3].
 *
 * - Calibration points:
 *      - The vertices of the corner walls (CW) act as the Calibration Points (CP).
 *
 *
 *                   C(0)           C(1)          C(2)
 *
 *                 0-----1        0-----1        0-----1
 *       R(0)      | CW0 |        |     |        | CW1 |
 *                 |     |        |     |        |     |
 *                 3-----2        3-----2        3-----2
 *
 *                 0-----1        0-----1        0-----1
 *       R(1)      |     |        |     |        |     |
 *                 |     |        |     |        |     |
 *                 3-----2        3-----2        3-----2
 *
 *                 0-----1        0-----1        0-----1
 *       R(2)      | CW3 |        |     |        | CW2 |
 *                 |     |        |     |        |     |
 *                 3-----2        3-----2        3-----2
 *
 *  - Calibration Procedure:
 *      - For the calibration opperatin, only one GLFWwindow window is used, but it can be moved between monitors.
 *      - A test pattern image with the same aspect ration as the walls is read in (currently using DevIL).
 *      - This image is tesselated  uniformly over a 3x3 grid in the in the graphics window
 *      - The window is moved to the desired projector and set to fullscreen.
 *      - Corner wall vertices are visible in the projected image displayed during calibration.
 *      - Each corner wall vertex acts as a 'control points'.
 *      - These control point vertices are independently positioned to the physical corners using a keyboard.
 *      - This process continues until all four vertices from all four corrner wall have been positioned
 *      - These values are then used to interpolate all other non-corner wall vertices.
 *      - All 3x3x4 warped wall vertices are tehn saved.
 *      - This continues until all 3 calibrations have been performed for all 4 projectors.
 *
 *  - Example of warping applied to the first row of walls:
 *
 *                   C(0)           C(1)          C(2)
 *
 *              0-----   1     0-----------1     0--------1
 *       R(0)    \   CW0 |      \         /      | CW1   /
 *                \      |       \       /       |      /
 *                 3-----2        3-----2        3-----2
 *
 */

#ifndef _PROJECTION_UTILS_H
#define _PROJECTION_UTILS_H

// ================================================== INCLUDE ==================================================

// Check if APIENTRY is already defined and undefine it
#ifdef APIENTRY
#undef APIENTRY
#endif

// OpenGL (GLAD and GLFW) for graphics and windowing
#include "glad/glad.h"
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

// Undefine APIENTRY after GLFW and GLAD headers
#ifdef APIENTRY
#undef APIENTRY
#endif

// DevIL for image loading and manipulation
#include <IL/il.h>
#include <IL/ilu.h>
#include <IL/ilut.h>
#include <IL/devil_cpp_wrapper.hpp>

// Define BOOST_BIND_GLOBAL_PLACEHOLDERS to suppress deprecation warnings related to Boost's bind placeholders
#define BOOST_BIND_GLOBAL_PLACEHOLDERS

// ROS for robot operating system functionalities
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <XmlRpcValue.h>

// Standard Library for various utilities
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <limits>
#include <array>
#include <vector>
#include <string>
#include <cstring>
#include <cmath>

// PugiXML for XML parsing
#include "pugixml.hpp"

// OpenCV for computer vision tasks
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/hal/interface.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

// ================================================== VARIABLES ==================================================

// Get top-level package path
extern const std::string package_path = ros::package::getPath("projection_operation");
extern const std::string workspace_path = package_path.substr(0, package_path.rfind("/src"));

// Directory paths for configuration files
extern const std::string CONFIG_DIR_PATH = workspace_path + "/data/proj_calibration_parameters";

// Directory paths for configuration images
extern const std::string IMAGE_TOP_DIR_PATH = workspace_path + "/data/proj_images";

/**
 * @brief 4D array of hardcoded image indices to display.
 *
 * This array is used to map specific image indices to a combination of
 * projector, chamber row, chamber column, calibration mode, and wall position.
 *
 * The array dimensions are as follows:
 * - Projector: 0 to 3
 * - Maze Chamber Row: 0 to 2
 * - Maze Chamber Column: 0 to 2
 * - Calibration Mode: 0 to 2 (represents l_wall, m_wall, r_wall)
 *
 * Format: array[4][3][3][3] = array[Projector][Chamber Row][Chamber Column][Calibration Mode{Left, Center, Right}]
 */

// Template of 4D array for hardcoded image indices to display
int TEMPLATE[4][3][3][3] = {
    // Projector 0: East
    {
        // Chamber Row: Top, Column: Left, Center, Right
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Middle
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Bottom
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
    },
    // Projector 1: North
    {
        // Chamber Row: Top, Column: Left, Center, Right
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Middle
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Bottom
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
    },
    // Projector 2: West
    {
        // Chamber Row: Top, Column: Left, Center, Right
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Middle
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Bottom
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
    },
    // Projector 3: South
    {
        // Chamber Row: Top, Column: Left, Center, Right
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Middle
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Bottom
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
    },
};

// Actual hardcoded image indices used to display
int IMG_PROJ_MAP[4][3][3][3] = {
    // Projector 0: East
    {
        // Chamber Row: Top, Column: Left, Center, Right
        {{0, 0, 0}, {1, 1, 1}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Middle
        {{0, 0, 0}, {0, 0, 4}, {0, 2, 2}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Bottom
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
    },
    // Projector 1: North
    {
        // Chamber Row: Top, Column: Left, Center, Right
        {{0, 0, 0}, {0, 2, 2}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Middle
        {{0, 1, 1}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Bottom
        {{0, 0, 0}, {0, 0, 0}, {0, 3, 3}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
    },
    // Projector 2: West
    {
        // Chamber Row: Top, Column: Left, Center, Right
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Middle
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Bottom
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
    },
    // Projector 3: South
    {
        // Chamber Row: Top, Column: Left, Center, Right
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Middle
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Bottom
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
    },
};

// // Actual hardcoded image indices used to display
// int IMG_PROJ_MAP[4][3][3][3] = {
//     // Projector 0: East
//     {
//         // Chamber Row: Top, Column: Left, Center, Right
//         {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
//         // Chamber Row: Middle
//         {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
//         // Chamber Row: Bottom
//         {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
//     },
//     // Projector 1: North
//     {
//         // Chamber Row: Top, Column: Left, Center, Right
//         {{0, 0, 0}, {1, 2, 3}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
//         // Chamber Row: Middle
//         {{1, 2, 3}, {0, 0, 0}, {3, 2, 1}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
//         // Chamber Row: Bottom
//         {{0, 0, 0}, {3, 2, 1}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
//     },
//     // Projector 2: West
//     {
//         // Chamber Row: Top, Column: Left, Center, Right
//         {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
//         // Chamber Row: Middle
//         {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
//         // Chamber Row: Bottom
//         {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
//     },
//     // Projector 3: South
//     {
//         // Chamber Row: Top, Column: Left, Center, Right
//         {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
//         // Chamber Row: Middle
//         {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
//         // Chamber Row: Bottom
//         {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
//     },
// };

// Number of rows and columns in the maze
extern const int MAZE_SIZE = 3;

// Sprecify window resolution: 4K resolution (3840x2160)
extern const int WINDOW_WIDTH_PXL = 3840;
extern const int WINDOW_HEIGHT_PXL = 2160;
extern const float WINDOW_ASPECT_RATIO = (float)WINDOW_WIDTH_PXL / (float)WINDOW_HEIGHT_PXL;

// Specify the maze width and height (NDC)
const float MAZE_WIDTH_NDC = 0.3f;
const float MAZE_HEIGHT_NDC = 0.6f;

// Wall image size (pixels)
extern const int WALL_IMAGE_WIDTH_PXL = 300;
extern const int WALL_IMAGE_HEIGHT_PXL = 540;

// Default wall width and height (NDC)
extern const float WALL_IMAGE_WIDTH_NDC = (MAZE_WIDTH_NDC / (float(MAZE_SIZE) - 1)) / (1 + std::sqrt(2));   // Wall width based on octogonal geometry in NDC
extern const float WALL_IMAGE_HEIGHT_NDC = (MAZE_HEIGHT_NDC / (float(MAZE_SIZE) - 1)) / (1 + std::sqrt(2)); // Wall height based on octogonal geometry in NDC

// ================================================== FUNCTIONS ==================================================

/**
 * @brief Formats the file name for the XML file based on the active calibration mode and monitor.
 *
 * Format:
 * - `cfg_m<number>_c<number>.xml`
 *
 * @param mon_id_ind Index of the active or desired monitor.
 * @param mode_cal_ind Index of the active or desired calibration mode.
 * @param config_dir_path Path to the directory where the XML file will be loaded/saved.
 *
 */
std::string formatCoordinatesFilePathXML(int, int, std::string);

/**
 * @brief Loads control point parameters and homography matrix from an XML file.
 *
 * @param full_path Path to the XML file.
 * @param verbose_level Level of verbosity for printing loaded data (0:nothing, 1:file name, 2:control point parameters, 3:homography matrix).
 * @param[out] out_HMAT Reference to the homography matrix to populate.
 * @param[out] out_ctrl_point_params Reference to a 4x6 array containing control point parameters (x, y, width, height, shear x, shear y).
 *
 * @return Integer status code [0:successful, -1:error].
 *
 * @details
 * This is the primary function containing the implementation. It reads an XML file
 * to populate the `r_h_mat` and `r_ctrl_point_params` matrices.
 *
 * @note Uses pugiXML for XML parsing.
 */
// int loadCoordinatesXML(std::string, int, cv::Mat &, std::array<std::array<float, 6>, 4> &);

/**
 * @brief Saves the calibration parameter coordinates and homography matrix to an XML file.
 *
 * @param h_mat The homography matrix used to warp perspective.
 * @param ctrl_point_params A 4x6 array containing control point parameters (x, y, width, height, shear x, shear y).
 * @param full_path Path to the XML file.
 *
 * @details
 * This function uses the pugixml library to create an XML document and populate it with
 * the calibration parameter coordinates and homography matrix. The calibration parameter are stored
 * in a 2D array and the homography matrix is stored in a cv::Mat object. Both are saved under their
 * respective XML nodes.
 *
 * @note The XML file is saved to the path specified by the global variable 'configPath'.
 *
 * Example XML structure:
 * @code
 * <config>
 *   <ctrl_point_params>
 *     <row>
 *       <cell>value</cell>
 *       ...
 *     </row>
 *     ...
 *   </ctrl_point_params>
 *   <h_mat>
 *     <row>
 *       <cell>value</cell>
 *       ...
 *     </row>
 *     ...
 *   </h_mat>
 * </config>
 * @endcode
 */
// void saveCoordinatesXML(cv::Mat, std::array<std::array<float, 6>, 4>, std::string);

/**
 * @brief Checks if a given set of vertices defines a valid quadrilateral.
 *
 * @param quad_vertices std:arr or std:vector of the four vertices defining a quadrilateral.
 *
 * @return Integer status code [-1:invalid wrong size, -2:invalid wrong shape, 0:valid].
 */
int checkQuadVertices(const std::vector<cv::Point2f> &);

/**
 * @brief Converts the units of the quadrilateral from NDC to pixels.
 *
 * @param quad_vertices_ndc The quadrilateral vertices in NDC
 * @param window_width_pxl The width of the window in pixels
 * @param window_height_pxl The height of the window in pixels
 *
 * @return Vector of quadrilateral vertices in pixels
 *
 * @details
 * Convert from NDC [-1, 1] to pixel [0, width or height] and
 * inverts the y to match OpenCV's top-left origin
 */
std::vector<cv::Point2f> quadVertNdc2Pxl(const std::vector<cv::Point2f> &, int, int);

/**
 * @brief Performs bilinear interpolation.
 *
 * @param a The value at the bottom-left corner.
 * @param b The value at the bottom-right corner.
 * @param c The value at the top-left corner.
 * @param d The value at the top-right corner.
 * @param grid_row_i The row index in the grid.
 * @param grid_col_i The column index in the grid.
 * @param grid_size The size of the grid.
 *
 * @details
 * This function performs bilinear interpolation based on a point's position (grid_row_i, grid_col_i)
 * within a 2D grid based on the vertex coordinates of the corner walls.
 *
 * The corner values correspond to the following positions within a unit square:
 * - a: Value at the bottom-left corner  (x, y) = (0, 0)
 * - b: Value at the bottom-right corner (x, y) = (1, 0)
 * - c: Value at the top-left corner     (x, y) = (0, 1)
 * - d: Value at the top-right corner    (x, y) = (1, 1)
 *
 * @return The interpolated value at the specified grid point.
 */
float bilinearInterpolation(float, float, float, float, int, int, int);

/**
 * @brief Computes updated Homography matrices for all walls.
 *
 * @param _CP_COORDS The control point coordinates used to warp the wall image.
 * @param[out] out_WALL_HMAT_DATA updated 3x3 array of Homography matrices used to warp the wall image.
 *
 * @return Integer status code [0:successful, -1:error].
 */
int updateHomography(
    const std::array<std::array<cv::Point2f, 4>, 4> &,
    std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> &);

/**
 * @brief Manages timed delays within a loop.
 *
 * @param dt_wait Latency in milliseconds for the delay; used only on the initial call.
 *
 * @details
 * Maintains a static timestamp to enforce a delay relative to `ros::Time::now()`.
 * Returns false if the delay has not elapsed, true otherwise, resetting the timer.
 *
 * @return True if the delay has elapsed, otherwise false.
 */
bool dbRunDT(int);

/**
 * @brief Track and print the elapsed time between calls.
 *
 * @details
 * Call this function with `do_reset` set to true to start timing,
 * and call it again with `do_reset` set to false to print the elapsed time.
 *
 * @param do_reset If true, resets the start time. If false, prints the elapsed time.
 * @param line Line number where the function is called.
 * @param file_path File path where the function is called.
 * 
  * @example dbLogDT(true, __LINE__, __FILE__);
 */
void dbLogDT(bool = false, int = 0, const char * = nullptr);

/**
 * @brief Pauses program and waits for any keypress.
 */
void dbWaitForInput();

/**
 * @brief Prints the coordinates of a quadrilateral's vertices.
 *
 * @param quad_vertices The quadrilateral's vertices.
 *
 * @note Use this regular expression to find the ros info and time stamp:
 *   \[\s*([A-Z]+)\]\s*\[([\d\.]+)\]:
 */
void dbLogQuadVertices(const std::vector<cv::Point2f> &);

/**
 * @brief Prints the coordinates of all entries in the control point array.
 */
void dbLogCtrlPointCoordinates(const std::array<std::array<cv::Point2f, 4>, 4> &);

/**
 * @brief Prints the coordinates of all entries in the warped wall vertices array.
 *
 * @param _WALL_WARP_COORDS Reference to the warped wall vertices array.
 */
void dbLogWallVerticesCoordinates(const std::array<std::array<std::array<cv::Point2f, 4>, MAZE_SIZE>, MAZE_SIZE> &);

/**
 * @brief Prints the coordinates of all entries in the homography matrix.
 *
 * @param _HMAT The homography matrix.
 */
void dbLogHomMat(const cv::Mat &);

#endif