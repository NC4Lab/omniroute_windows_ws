// ####################################################################################################

// ======================================== projection_utils.h ========================================

// ####################################################################################################

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

// Spricify window resolution: 4K resolution (3840x2160)
extern const int PROJ_WIN_WIDTH_PXL = 3840;
extern const int PROJ_WIN_HEIGHT_PXL = 2160;
extern const float PROJ_WIN_ASPECT_RATIO = (float)PROJ_WIN_WIDTH_PXL / (float)PROJ_WIN_HEIGHT_PXL;

// Directory paths for configuration files
extern const std::string package_path = ros::package::getPath("projection_operation");
extern const std::string workspace_path = package_path.substr(0, package_path.rfind("/src"));
extern const std::string CONFIG_DIR_PATH = workspace_path + "/data/proj_cfg";

// Directory paths for configuration images
extern const std::string IMAGE_TOP_DIR_PATH = workspace_path + "/data/proj_img";

// Number of rows and columns in the maze
extern const int MAZE_SIZE = 3;

// Wall image size and spacing (pixels)
extern const int WALL_WIDTH_PXL = 300;
extern const int WALL_HEIGHT_PXL = 540;

// Specify the origin plane width and height (NDC)
const float originPlaneWidth = 0.3f;
const float originPlaneHeight = 0.6f;

// Wall spacing (NDC)
extern const float WALL_SPACE_X = originPlaneWidth / (float(MAZE_SIZE) - 1);  // Wall spacing on X axis NDC
extern const float WALL_SPACE_Y = originPlaneHeight / (float(MAZE_SIZE) - 1); // Wall spacing on Y axis NDC

// Default wall width and height (NDC)
const float wall_width_ndc = WALL_SPACE_X / (1 + std::sqrt(2));  // Wall width based on octogonal geometry in NDC
const float wall_height_ndc = WALL_SPACE_Y / (1 + std::sqrt(2)); // Wall height based on octogonal geometry in NDC

/**
 * @brief Control Point Calibration Parameter Array
 *
 * This array stores the position and transformation parameters for each control point used in calibration.
 * The units for the x, y, width, and height parameters are in OpenGL's Normalized Device Coordinates (NDC),
 * which range from [-1, 1] with the origin at the center of the screen.
 *
 * @note Control point at index 1 only allows for positional adjustments and does not contribute to scaling or shearing.
 *
 * Each row in the array corresponds to a specific control point on the screen, while each column represents a different attribute
 * of that control point.
 *
 * @details
 * - Rows:
 *   - [0]: Top-left control point,     corresponding to grid point [0][s-1]
 *   - [1]: Top-right control point,    corresponding to grid point [s-1][s-1]
 *   - [2]: Bottom-right control point, corresponding to grid point [s-1][0]
 *   - [3]: Bottom-left control point,  corresponding to grid point [0][0]
 *
 * - Columns:
 *   - [0]: X-coordinate in NDC
 *   - [1]: Y-coordinate in NDC
 *   - [2]: Wall width in NDC
 *   - [3]: Wall height in NDC
 *   - [4]: Horizontal shearing factor
 *   - [5]: Vertical shearing factor
 *
 * @note
 * - Control Point Parameters (Example):
 *    CP  |  X-Dist  |  Y-Dist  |    Width  |   Height  |  Shear X  |  Shear Y  |
 *    ---------------------------------------------------------
 *    [0] |   -0.15  |    0.30  |    0.06   |    0.12   |    0.00
 *    [1] |    0.15  |    0.30  |    0.06   |    0.12   |    0.00
 *    [2] |    0.15  |   -0.30  |    0.06   |    0.12   |    0.00
 *    [3] |   -0.15  |   -0.30  |    0.06   |    0.12   |    0.00
 *    ---------------------------------------------------------
 *
 * @note
 * Control Point Indices to Grid Point Mapping
 *
 *         Ctrl Points    Grid Points
 *        =============  =============
 *        | -1  | +1  |  |  0  | s-1 |
 *    =================  =================
 *     +1 | [0] | [1] |  | [0] | [1] | s-1
 *    -----------------  -----------------
 *     -1 | [3] | [2] |  | [3] | [2] |  0
 *    =================  =================
 *
 * @section NDC Origin
 * In OpenGL's NDC, the origin (0,0) is located at the center of the screen.
 * The X-axis extends from -1 (left) to 1 (right).
 * The Y-axis extends from -1 (bottom) to 1 (top).
 */
const float cp_x = originPlaneWidth / 2;  // starting X-coordinate in NDC coordinates
const float cp_y = originPlaneHeight / 2; // starting Y-coordinate in NDC coordinates
const std::array<std::array<float, 6>, 4> CTRL_POINT_PARAMS = {{
    {{-cp_x, +cp_y, wall_width_ndc, wall_height_ndc, 0.0f, 0.0f}}, // top-left control point
    {{+cp_x, +cp_y, wall_width_ndc, wall_height_ndc, 0.0f, 0.0f}}, // top-right control point
    {{+cp_x, -cp_y, wall_width_ndc, wall_height_ndc, 0.0f, 0.0f}}, // bottom-right control point
    {{-cp_x, -cp_y, wall_width_ndc, wall_height_ndc, 0.0f, 0.0f}}  // bottom-left control point
}};

// Default wall width and height (NDC)
extern const float W_WD_DEF = WALL_SPACE_X / (1 + std::sqrt(2)); // Wall width based on octogonal geometry in NDC
extern const float W_HT_DEF = WALL_SPACE_Y / (1 + std::sqrt(2)); // Wall height based on octogonal geometry in NDC

// Defualt x and y coordinates for control points (NDC)
extern const float CP_X_DEF = originPlaneWidth / 2;  // starting X-coordinate in NDC coordinates
extern const float CP_Y_DEF = originPlaneHeight / 2; // starting Y-coordinate in NDC coordinates

/**
 * @brief  4x4 data container for tracking control point coordinates in Normalized Device Coordinates (NDC)
 *         [4][4] = [conrol point][vertex]
 *
 * @details
 *
 * - Dimension 1: Control Point [0, 1, 2, 3]
 *  - 0: Top-left (image vertex)
 *  - 1: Top-right (image vertex)
 *  - 2: Bottom-left (image vertex)
 *  - 3: Bottom-right (image vertex)
 *
 * - Dimension 2: Vertex(x, y) [0, 1, 2, 3]
 *  - 0: Top-left  (quadrilateral vertex)
 *  - 1: Top-right  (quadrilateral vertex)
 *  - 2: Bottom-left  (quadrilateral vertex)
 *  - 3: Bottom-right  (quadrilateral vertex)
 */
std::array<std::array<cv::Point2f, 4>, 4> CONTROL_POINT_COORDINATES;

/**
 * @brief  3x3x4 data container for tracking storing warped wall vertices coordinates in Normalized Device Coordinates (NDC)
 *         [3][3][4] = [row][col][vertex]
 *
 * @details
 *
 * - Dimension 1: Rows [0-MAZE_SIZE-1]:
 *  - Top to Bottom
 *
 * - Dimension 2: Column [0-MAZE_SIZE-1]:
 *  - Left to Right
 *
 * - Dimension 3: Vertex(x, y) [0, 1, 2, 3]
 *  - 0: Top-left  (quadrilateral vertex)
 *  - 1: Top-right  (quadrilateral vertex)
 *  - 2: Bottom-left  (quadrilateral vertex)
 *  - 3: Bottom-right  (quadrilateral vertex)
 */
std::array<std::array<std::array<cv::Point2f, 4>, MAZE_SIZE>, MAZE_SIZE> WARPED_WALL_COORDINATES;

// The 3x3 homography matrix of 32-bit floating-point numbers used to warp perspective.
cv::Mat H_MAT = cv::Mat::eye(3, 3, CV_32F);


// ================================================== FUNCTIONS ==================================================

/**
 * @brief Checks for DevIL errors and logs them.
 * Should be called after DevIL API calls.
 *
 * @example checkErrorDevIL(__LINE__, __FILE__);
 *
 * @param line Line number where the function is called.
 * @param file_str File name where the function is called.
 * @param msg_str Optional message to provide additional context (default to nullptr).
 *
 * @return 0 if no errors, -1 if error.
 */
int checkErrorDevIL(int, const char *, const char * = nullptr);

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
 * This is the primary function containing the implementation. It reads an XML file
 * to populate the `r_hom_mat` and `r_ctrl_point_params` matrices.
 *
 * @note Uses pugiXML for XML parsing.
 *
 * @param[out] r_hom_mat Reference to the homography matrix to populate.
 * @param[out] r_ctrl_point_params Reference to a 4x6 array containing control point parameters (x, y, width, height, shear x, shear y).
 * @param full_path Path to the XML file.
 * @param verbose_level Level of verbosity for printing loaded data (0:nothing, 1:file name, 2:control point parameters, 3:homography matrix).
 *
 * @return 0 on successful execution, -1 on failure.
 */
int loadCoordinatesXML(cv::Mat &, std::array<std::array<float, 6>, 4> &, std::string, int = 0);

/**
 * @brief Saves the calibration parameter coordinates and homography matrix to an XML file.
 *
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
 *   <hom_mat>
 *     <row>
 *       <cell>value</cell>
 *       ...
 *     </row>
 *     ...
 *   </hom_mat>
 * </config>
 * @endcode
 *
 * @param hom_mat The homography matrix used to warp perspective.
 * @param ctrl_point_params A 4x6 array containing control point parameters (x, y, width, height, shear x, shear y).
 * @param full_path Path to the XML file.
 */
void saveCoordinatesXML(cv::Mat, std::array<std::array<float, 6>, 4>, std::string);

/**
 * @brief Loads images from specified file paths and stores their IDs in a reference vector.
 *
 * This function takes a vector of file paths and iteratively loads each image
 * using the DevIL library. The function then stores the ILuint IDs of successfully loaded images
 * in a reference vector.
 *
 * @param img_paths_vec A vector of file paths to the images to be loaded.
 * @param[out] r_image_id_vec A reference to a vector of ILuint where the IDs of the loaded images will be stored.
 *
 * @return DevIL status: 0 on successful execution, -1 on failure.
 */
int loadImgTextures(std::vector<std::string>, std::vector<ILuint> &);

/**
 * @brief Deletes DevIL images from a given vector of image IDs.
 *
 * @param r_image_id_vec Vector containing DevIL image IDs.
 *
 * @return DevIL status: 0 on successful execution, -1 on failure.
 */
int deleteImgTextures(std::vector<ILuint> &);

/**
 * @brief Merges two images by overlaying non-white pixels from the second image onto the first.
 *
 * This function takes two images, img1 and img2, represented as ILuint IDs. It overlays img2 onto img1,
 * replacing pixels in img1 with corresponding non-white pixels from img2. The resulting merged image is
 * returned as a new ILuint ID.
 *
 * @param img1_id The ILuint ID of the baseline image.
 * @param img2_id The ILuint ID of the mask image.
 * @param[out] r_img_merge_id Reference to an ILuint where the ID of the merged image will be stored.
 *
 * @return DevIL status: 0 on successful execution, -1 on failure.
 *
 * @warning The dimensions of img1 and img2 must match.
 */
int mergeImages(ILuint, ILuint, ILuint &);

/**
 * @brief Calculates an interpolated value using bilinear interpolation on a 2D grid.
 *
 * This function performs bilinear interpolation based on a point's position (grid_row_i, grid_col_i)
 * within a 2D grid. The grid corners are defined by a set of 4x6 control point parameters.
 *
 * @param ctrl_point_params A 4x6 array containing control point parameters (x, y, width, height, shear x, shear y).
 * @param ctrl_point_params_ind Index of the specific control point parameter to interpolate [0, 4].
 * @param grid_row_i Index of the point along the first axis (rows) within the grid.
 * @param grid_col_i Index of the point along the second axis (columns) within the grid.
 * @param grid_size Number of cells along one axis in the grid.
 * @param do_offset Flag to indicate whether to offset the interpolated value by the control point parameter at the origin.
 *
 * @return float The interpolated value calculated based on the specified control point parameters and grid position.
 */
float bilinearInterpolation(std::array<std::array<float, 6>, 4>, int, int, int, int, bool);

/**
 * @brief Calculates an interpolated value using bilinear interpolation on a 2D grid.
 *
 * This function performs bilinear interpolation based on a point's position (grid_row_i, grid_col_i)
 * within a 2D grid. The grid corners are defined by a set of 4x6 control point parameters.
 *
 * @param ctrl_point_params 4x6 array of control point parameters. Each row corresponds to one control point
 *                          and contains five parameters for that point (x, y, width, height, shear x, shear y).
 * @param ctrl_point_params_ind Index of the specific control point parameter to interpolate [0, 4].
 * @param grid_row_i Index of the point along the first axis (rows) within the grid.
 * @param grid_col_i Index of the point along the second axis (columns) within the grid.
 * @param grid_size Number of cells along one axis in the grid.
 *
 * @return float The interpolated value calculated based on the specified control point parameters and grid position.
 */
float bilinearInterpolationFull(std::array<std::array<float, 6>, 4>, int, int, int, int);

/**
 * @brief Creates a vector of points representing a quadrilateral with shear.
 *
 * This function generates a quadrilateral's corner points starting from the top-left corner
 * and going clockwise. The quadrilateral is defined by its top-left corner (x0, y0),
 * width, height, and a shear amount. The units for x0, y0, width, and height are in
 * OpenGL's Normalized Device Coordinates (NDC) [-1, 1].
 *
 * @param x0 The x-coordinate of the top-left corner of the quadrilateral in NDC.
 * @param y0 The y-coordinate of the top-left corner of the quadrilateral in NDC.
 * @param width The width of the quadrilateral in NDC.
 * @param height The height of the quadrilateral in NDC.
 * @param shear_x The amount of horizontal shear to apply to the quadrilateral.
 * @param shear_y The amount of vertical shear to apply to the quadrilateral.
 *
 * @return std::vector<cv::Point2f> A vector of 4 points representing the corners of the rectangle, in NDC.
 */
std::vector<cv::Point2f> computeQuadVertices(float, float, float, float, float, float);

/**
 * @brief Computes the global homography matrix based on overall control point parameters.
 *
 * This function calculates the global homography matrix that maps points from the source plane
 * (representing the entire projected image) to the destination plane (defined by control points).
 * Unlike per-wall transformations, this matrix is intended for the complete image, providing a
 * unified perspective warp.
 *
 * Control point x and y coordinates are specified in Normalized Device Coordinates (NDC) [-1, 1].
 * The vertices for the entire projected image are calculated based on the dimensions that enclose
 * all control points (i.e., boundary dimensions in the control point plane).
 *
 * @note This function utilizes the OpenCV library to compute the homography matrix.
 *
 * @param r_hom_mat Reference to the cv::Mat object where the computed homography matrix will be stored.
 * @param ctrl_point_params A 4x6 array containing control point parameters (x, y, width, height, shear x, shear y).
 */
void computeHomography(cv::Mat &, std::array<std::array<float, 6>, 4>);

/**
 * @brief Computes the perspective warp of a given set of quadrilateral vertices using a homography matrix.
 *
 * This function takes a set of quadrilateral vertices and applies a projective transformation to each vertex.
 * The transformation is governed by a given homography matrix. Before the warp, an optional translation is
 * applied to each vertex.
 *
 * @param quad_vertices_vec A vector containing the original Cartesian coordinates of the quadrilateral's vertices.
 *                          The vertices are processed in-place.
 *
 * @param r_hom_mat Reference to the homography matrix. This 3x3 matrix is used to perform the projective
 *                  transformation on each vertex.
 *
 * @details
 * 1. Each vertex undergoes a translation operation defined by `x_translate` and `y_translate`.
 *
 * 2. The function converts the translated vertices to homogeneous coordinates, which allows for a unified
 *    representation that can undergo linear transformations including projective warps.
 *
 * 3. The homography matrix `r_hom_mat` is then applied to these homogeneous coordinates to produce the
 *    new coordinates of each vertex in the warped perspective.
 *
 * 4. Finally, the function converts these new homogeneous coordinates back to Cartesian coordinates.
 *
 * @note: Homogeneous Coordinates: are a mathematical trick used to simplify
 *        complex transformations like translation, rotation, and shearing.
 *        Cartesian coordinates (x, y) are converted to [x, y, 1] for the
 *        transformation. The third value (ptMat.at<float>(2)) may change
 *        post-transformation so we divide the new x and y by this value
 *        (ptMat /= ptMat.at<float>(2)) to get back to cartisian x, y coordinates.
 *
 * @return std::vector<cv::Point2f> A vector containing the new Cartesian coordinates of the warped vertices.
 */
std::vector<cv::Point2f> computePerspectiveWarp(std::vector<cv::Point2f>, cv::Mat &);

/**
 * @brief Used to reset control point parameter list.
 *
 * @param r_ctrl_point_params Reference to the 4x6 array of control point parameters.
 * @param mode_cal_ind Index of the active calibration mode.
 */
void updateCalParams(std::array<std::array<float, 6>, 4> &, int);

/**
 * @brief Prints the control point parameters to the ROS log.
 *
 * @param ctrl_point_params 4x6 array of control point parameters.
 */
void dbLogCtrlPointParams(std::array<std::array<float, 6>, 4>);

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
 * The corner values correspond to the following positions within a unit square:
 * - a: Value at the bottom-left corner  (x, y) = (0, 0)
 * - b: Value at the bottom-right corner (x, y) = (1, 0)
 * - c: Value at the top-left corner     (x, y) = (0, 1)
 * - d: Value at the top-right corner    (x, y) = (1, 1)
 *
 * @return The interpolated value at the specified grid point.
 */
float bilinearInterpolationFullV2(float a, float b, float c, float d, int grid_row_i, int grid_col_i, int grid_size);

std::array<std::array<cv::Point2f, 4>, 4> initControlPointCoordinates();

cv::Mat computeHomographyV2(const std::array<std::array<cv::Point2f, 4>, 4> &);

cv::Point2f perspectiveWarpPoint(cv::Point2f p_unwarped, const cv::Mat &H_MAT);

std::array<std::array<std::array<cv::Point2f, 4>, MAZE_SIZE>, MAZE_SIZE> updateWarpedWallVertices(const std::array<std::array<cv::Point2f, 4>, 4> &);

void dbLogQuadVertices(const std::vector<cv::Point2f> &);

void dbLogQuadVerticesArr(const std::array<cv::Point2f, 4> &);

void dbLogCtrlPointCoordinates(const std::array<std::array<cv::Point2f, 4>, 4>&);

void dbLogWallVerticesCoordinates(const std::array<std::array<std::array<cv::Point2f, 4>, MAZE_SIZE>, MAZE_SIZE> &);

#endif