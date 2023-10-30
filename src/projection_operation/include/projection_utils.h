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

// Directory paths for configuration files
extern const std::string package_path = ros::package::getPath("projection_operation");
extern const std::string workspace_path = package_path.substr(0, package_path.rfind("/src"));
extern const std::string CONFIG_DIR_PATH = workspace_path + "/data/proj_cfg";

// Directory paths for configuration images
extern const std::string IMAGE_TOP_DIR_PATH = workspace_path + "/data/proj_img";

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
extern const int PROJ_WIN_WIDTH_PXL = 3840;
extern const int PROJ_WIN_HEIGHT_PXL = 2160;
extern const float PROJ_WIN_ASPECT_RATIO = (float)PROJ_WIN_WIDTH_PXL / (float)PROJ_WIN_HEIGHT_PXL;

// Wall image size (pixels)
extern const int WALL_WIDTH_PXL = 300;
extern const int WALL_HEIGHT_PXL = 540;

// // Wall image size (NDC)
// extern const float WALL_WIDTH_NDC = (static_cast<float>(WALL_WIDTH_PXL) / static_cast<float>(PROJ_WIN_WIDTH_PXL)) * 2;
// extern const float WALL_HEIGHT_NDC = (static_cast<float>(WALL_HEIGHT_PXL) / static_cast<float>(PROJ_WIN_HEIGHT_PXL)) * 2;

// // Specify the origin plane width and height (NDC)
// const float ORIGIN_PLANE_WIDTH_NDC = 0.3f;
// const float ORIGIN_PLANE_HEIGHT_NDC = 0.6f;

// // Wall spacing (NDC)
// extern const float WALL_SPACE_HORZ_NDC = ORIGIN_PLANE_WIDTH_NDC / (float(MAZE_SIZE) - 1);  // Wall spacing on X axis NDC
// extern const float WALL_SPACE_VERT_NDC = ORIGIN_PLANE_HEIGHT_NDC / (float(MAZE_SIZE) - 1); // Wall spacing on Y axis NDC

// // Default wall width and height (NDC)
// extern const float WALL_WIDTH_NDC = WALL_SPACE_HORZ_NDC / (1 + std::sqrt(2)); // Wall width based on octogonal geometry in NDC
// extern const float WALL_HEIGHT_NDC = WALL_SPACE_VERT_NDC / (1 + std::sqrt(2)); // Wall height based on octogonal geometry in NDC

// TEMP
const float ORIGIN_PLANE_WIDTH_NDC = 0.5;
const float ORIGIN_PLANE_HEIGHT_NDC = 1.0f;
extern const float WALL_SPACE_HORZ_NDC = ORIGIN_PLANE_WIDTH_NDC / (float(MAZE_SIZE) - 1);  // Wall spacing on X axis NDC
extern const float WALL_SPACE_VERT_NDC = ORIGIN_PLANE_HEIGHT_NDC / (float(MAZE_SIZE) - 1); // Wall spacing on Y axis NDC
extern const float WALL_WIDTH_NDC = 2 * WALL_SPACE_HORZ_NDC / (1 + std::sqrt(2));          // Wall width based on octogonal geometry in NDC
extern const float WALL_HEIGHT_NDC = 2 * WALL_SPACE_VERT_NDC / (1 + std::sqrt(2));         // Wall height based on octogonal geometry in NDC

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
 * @return Integer status code [0:successful, -1:error].
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
 * to populate the `r_h_mat` and `r_ctrl_point_params` matrices.
 *
 * @note Uses pugiXML for XML parsing.
 *
 * @param full_path Path to the XML file.
 * @param verbose_level Level of verbosity for printing loaded data (0:nothing, 1:file name, 2:control point parameters, 3:homography matrix).
 * @param[out] out_HMAT Reference to the homography matrix to populate.
 * @param[out] out_ctrl_point_params Reference to a 4x6 array containing control point parameters (x, y, width, height, shear x, shear y).
 *
 * @return Integer status code [0:successful, -1:error].
 */
int loadCoordinatesXML(std::string, int, cv::Mat &, std::array<std::array<float, 6>, 4> &);

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
 *   <h_mat>
 *     <row>
 *       <cell>value</cell>
 *       ...
 *     </row>
 *     ...
 *   </h_mat>
 * </config>
 * @endcode
 *
 * @param h_mat The homography matrix used to warp perspective.
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
 * @param[out] out_image_id_vec Reference to a vector of ILuint where the IDs of the loaded images will be stored.
 *
 * @return Integer status code [0:successful, -1:error].
 */
int loadImgTextures(std::vector<std::string>, std::vector<ILuint> &);

/**
 * @brief Deletes DevIL images from a given vector of image IDs.
 *
 * @param r_image_id_vec Reference to the cector containing DevIL image IDs.
 *
 * @return Integer status code [0:successful, -1:error].
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
 * @param[out] out_img_merge_id Reference to an ILuint where the ID of the merged image will be stored.
 *
 * @return Integer status code [0:successful, -1:error].
 *
 * @warning The dimensions of img1 and img2 must match.
 */
int generateMergedTexture(ILuint, ILuint, ILuint &);

/**
 * @brief Converts quadrilateral vertices vector of cv::Point2f to an array of cv::Point2f
 *
 * @param quad_vert_vec
 *
 * @return quadrilateral vertices array of cv::Point2f
 */
std::array<cv::Point2f, 4> quadVec2Arr(const std::vector<cv::Point2f> &);

/**
 * @brief Converts quadrilateral vertices array of cv::Point2f to a vector of cv::Point2f
 *
 * @param quad_vert_arr
 *
 * @return quadrilateral vertices vector of cv::Point2f
 */
std::vector<cv::Point2f> quadArr2Vec(const std::array<cv::Point2f, 4> &);

/**
 * @brief Checks if a given set of vertices defines a valid quadrilateral.
 *
 * @param quad_vertices std:arr or std:vector of the four vertices defining a quadrilateral.
 *
 * @return Integer status code [0:valid, 1:wrong size, 2:wrong shape].
 */
int checkQuadVertices(const std::array<cv::Point2f, 4> &);
int checkQuadVertices(const std::vector<cv::Point2f> &);

/**
 * @brief Calculates the bounding dimensions based on a given set of vertices.
 *
 * @param quad_vertices std:arr of four vertices defining a quadrilateral.
 *
 * @return Size object containing the maximum dimensions.
 */
cv::Size getBoundaryDims(std::array<cv::Point2f, 4>);

/**
 * @brief Performs bilinear interpolation.
 *
 * This function performs bilinear interpolation based on a point's position (grid_row_i, grid_col_i)
 * within a 2D grid based on the vertex coordinates of the corner walls.
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
float bilinearInterpolation(float, float, float, float, int, int, int);

/**
 * @brief Computes the homography matrix based on target plane vertices.
 *
 * @param origin_width Width of the origin plane.
 * @param origin_height Height of the origin plane.
 * @param target_plane_vertices Four vertices defining the target plane.
 * @param[out] out_HMAT Reference to the homography matrix.
 *
 * @return Integer status code [0:successful, -1:error].
 */
int computeHomography(float, float, std::array<cv::Point2f, 4>, cv::Mat &);
int computeHomography(float, float, std::vector<cv::Point2f>, cv::Mat &);

/**
 * @brief Applies a given homography matrix to a texture image and returns the transformed texture.
 *
 * @param source_texture_id DevIL image ID of the source texture.
 * @param _HMAT Homography matrix.
 * @param target_plane_vertices Vertices of the target plane.
 * @param[out] out_warped_texture_id Reference to the DevIL image ID of the outputed warped texture.
 *
 * @return Integer status code [0:successful, -1:error].
 */
int generateWarpedTexture(ILuint, cv::Mat, std::array<cv::Point2f, 4>, ILuint &);

/**
 * @brief Computes the perspective warp of a given coordinate point using a homography matrix.
 *
 * This function takes a cv::Point2f point vertices and applies a projective transformation to each vertex.
 * The transformation is governed by a given homography matrix.
 *
 *
 * @details
 *
 * - The function converts the translated piont to homogeneous coordinates, which allows for a unified
 *    representation that can undergo linear transformations including projective warps.
 *
 * - The homography matrix is then applied to these homogeneous coordinates to produce the
 *    new coordinates of the point in the warped perspective.
 *
 * - Finally, the function converts these new homogeneous coordinates back to Cartesian coordinates.
 *
 * @note: Homogeneous Coordinates: are a mathematical trick used to simplify
 *        complex transformations like translation, rotation, and shearing.
 *        Cartesian coordinates (x, y) are converted to [x, y, 1] for the
 *        transformation. The third value (ptMat.at<float>(2)) may change
 *        post-transformation so we divide the new x and y by this value
 *        (ptMat /= ptMat.at<float>(2)) to get back to cartisian x, y coordinates.
 *
 * @param p_unwarped The unwarped cv::Point2f values.
 * @param _HMAT The 3x3 homography matrix used to perform the projective transformation.
 * @param[out] out_p_warped Reference to cv::Point2f containing the new Cartesian coordinates of the warped vertex position.
 *
 * @return Integer status code [0:successful, -1:error].
 */
int perspectiveWarpPoint(cv::Point2f, cv::Mat, cv::Point2f &);

/**
 * @brief Initializes values for the verteces of the coner walls which will be used as calibraton control points.
 *
 *
 * Control point x and y coordinates are specified in Normalized Device Coordinates (NDC) [-1, 1].
 * The vertices for the entire projected image are calculated based on the dimensions that enclose
 * all control points (i.e., boundary dimensions in the control point plane).
 *
 * @param[out] out_CTRL_PNT_DATA Reference to the 4x4 array containing the coordinates of the corner wall's vertices.
 */
void initControlPointCoordinates(std::array<std::array<cv::Point2f, 4>, 4> &);

/**
 * @brief Updates the stored warped wall image vertices based on the control point array.
 *
 * @param _CTRL_PNT_DATA The control point coordinates used to warp the wall image.
 * @param[out] out_WALL_VERT_DATA updated 3x3x4 warped wall image vertices array.
 *
 * @return Integer status code [0:successful, -1:error].
 */
int updateWallVertices(
    const std::array<std::array<cv::Point2f, 4>, 4> &,
    std::array<std::array<std::array<cv::Point2f, 4>, MAZE_SIZE>, MAZE_SIZE> &);

/**
* @brief Recomputes the homography matrix for each wall.
 *
 * @param _CTRL_PNT_DATA The control point coordinates used to warp the wall image.
 * @param _WALL_VERT_DATA The 3x3x4 warped wall image vertices array.
 * @param[out] out_WALL_HMAT_DATA updated 3x3 array of Homography matrices used to warp the wall image.
 *
 * @return Integer status code [0:successful, -1:error].
 */
int updateWallHomography(
    const std::array<std::array<cv::Point2f, 4>, 4> &,
     const std::array<std::array<std::array<cv::Point2f, 4>, MAZE_SIZE>, MAZE_SIZE> &,
    std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> &);

/**
 * @brief Prints the coordinates of a quadrilateral's vertices.
 *
 * @param quad_vertices The quadrilateral's vertices.
 */
void dbLogQuadVertices(const std::vector<cv::Point2f> &);
void dbLogQuadVertices(const std::array<cv::Point2f, 4> &);

/**
 * @brief Prints the coordinates of all entries in the control point array.
 */
void dbLogCtrlPointCoordinates(const std::array<std::array<cv::Point2f, 4>, 4> &);

/**
 * @brief Prints the coordinates of all entries in the warped wall vertices array.
 *
 * @param _WALL_VERT_DATA Reference to the warped wall vertices array.
 */
void dbLogWallVerticesCoordinates(const std::array<std::array<std::array<cv::Point2f, 4>, MAZE_SIZE>, MAZE_SIZE> &);

/**
 * @brief Prints the coordinates of all entries in the homography matrix.
 *
 * @param _HMAT The homography matrix.
 */
void dbLogHomMat(const cv::Mat &);

#endif