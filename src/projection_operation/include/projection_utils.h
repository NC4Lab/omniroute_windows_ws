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
        {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Middle
        {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Bottom
        {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
    },
    // Projector 1: North
    {
        // Chamber Row: Top, Column: Left, Center, Right
        {{0, 0, 0}, {1, 2, 3}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Middle
        {{1, 2, 3}, {0, 0, 0}, {3, 2, 1}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Bottom
        {{0, 0, 0}, {3, 2, 1}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
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

// Control point image radius
extern const float CP_RADIUS_NDC = 0.005f;

// Defualt offset of control points from the center of the screen
const float cal_offset_x = 0.15f; // X offset from center of screen for control points
const float cal_offset_y = 0.3f;  // X offset from center of screen for control points

// Wall image size and spacing in OpenGL's Normalized Device Coordinates (NDC) [-1, 1]
const float wall_width_ndc = ((cal_offset_x * 2) / (float(MAZE_SIZE) - 1)) / (1 + std::sqrt(2));
const float wall_height_ndc = ((cal_offset_y * 2) / (float(MAZE_SIZE) - 1)) / (1 + std::sqrt(2));
extern const float WALL_SPACE = 2.0f * wall_width_ndc;

// Control point calibration parameter array
/**
 * @brief Array to hold the position and transformation parameters for control points in Normalized Device Coordinates (NDC) [-1, 1].
 *
 * @note The second control point (index 1) can only be used to adjust the wall position.
 *
 * Each row corresponds to a specific control point on the screen, and each column holds a different attribute
 * of that control point.
 *
 * - Rows:
 *   - [0]: Top-left control point (grid point [0][0])
 *   - [1]: Top-right control point (grid point [0][1])
 *   - [2]: Bottom-right control point (grid point [1][0])
 *   - [3]: Bottom-left control point (grid point [1][1])
 *
 * - Columns:
 *   - [0]: X-distance from center [-1,1]
 *   - [1]: Y-distance from center [-1,1]
 *   - [2]: Wall width parameter [-1,1]
 *   - [3]: Wall height parameter [-1,1]
 *   - [4]: Shearing factor
 */
extern const float CONT_POINT_PARAMS[4][5] = {
    // Default control point parameters
    {-cal_offset_x, cal_offset_y, wall_width_ndc, wall_height_ndc, 0.0f}, // top-left control point
    {cal_offset_x, cal_offset_y, wall_width_ndc, wall_height_ndc, 0.0f},  // top-right control point
    {cal_offset_x, -cal_offset_y, wall_width_ndc, wall_height_ndc, 0.0f}, // bottom-right control point
    {-cal_offset_x, -cal_offset_y, wall_width_ndc, wall_height_ndc, 0.0f} // bottom-left control point
};

// ================================================== FUNCTIONS ==================================================

void interpOffsetTEMP(float cont_point_params[4][5], float &x_offset, float &y_offset, float wall_row_i, float wall_col_i);

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
 * @brief Loads calibration parameters and homography matrix from an XML file.
 *
 * This is the primary function containing the implementation. It reads an XML file
 * to populate the `r_hom_mat` and `r_cont_point_params` matrices.
 *
 * @note Uses pugiXML for XML parsing.
 *
 * @param[out] r_hom_mat Reference to the homography matrix to populate.
 * @param[out] r_cont_point_params Reference to the 4x5 array of calibration parameters.
 * @param full_path Path to the XML file.
 * @param verbose_level Level of verbosity for printing loaded data (0:nothing, 1:file name, 2:calibration parameters, 3:homography matrix).
 *
 * @return 0 on successful execution, -1 on failure.
 */
int loadCoordinatesXML(cv::Mat &, float (&)[4][5], std::string, int = 0);

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
 *   <cont_point_params>
 *     <row>
 *       <cell>value</cell>
 *       ...
 *     </row>
 *     ...
 *   </cont_point_params>
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
 * @param cont_point_params The 4x5 array of calibration parameters.
 * @param full_path Path to the XML file.
 */
void saveCoordinatesXML(cv::Mat, float[4][5], std::string);

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


std::vector<float> getArrColumn(float [4][5], int);

/**
 * @brief Calculate control point distance based on control point coordinates.
 *
 * @param cont_point_params The 4x5 array of calibration parameters.
 * @param [out] r_control_point_distances 4x2 array to store calculated distances spacings in x and y direction.
 */
void calculateControlPointDistances(float[4][5], float (&)[4][5]);

/**
 * @brief Calculates an interpolated value using bilinear interpolation on a 2D grid.
 *
 * This function performs bilinear interpolation based on the position of a point
 * within a 2D grid (grid_row_i, grid_col_i) and predefined values at the grid corners.
 *
 * @param cont_point_params The 4x5 array of calibration parameters.
 * @param cont_point_params_ind The index of the calibration parameters (3:height, 4:sheer).
 * @param grid_row_i The index of the point along the first axis within the grid.
 * @param grid_col_i The index of the point along the second axis within the grid.
 * @param grid_size The size of the 2D grid.
 *
 * @return float The calculated interpolated value.
 */
float interpolateWallParam(float[4][5], int, int, int, int, bool);

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
 * @param shear_amount The amount of shear to apply to the quadrilateral.
 *
 * @return std::vector<cv::Point2f> A vector of 4 points representing the corners of the rectangle, in NDC.
 */
std::vector<cv::Point2f> computeQuadVertices(float, float, float, float, float);

/**
 * @brief Computes the perspective warp for a given set of points.
 *
 * @param quad_vertices_vec The vector of points representing the rectangle.
 * @param r_hom_mat The homography matrix used to warp perspective.
 * @param x_offset The x-offset to apply to the vertices.
 * @param y_offset The y-offset to apply to the vertices.
 *
 * @return std::vector<cv::Point2f> The warped vertices.
 */
std::vector<cv::Point2f> computePerspectiveWarp(std::vector<cv::Point2f>, cv::Mat &, float, float);

/**
 * @brief Computes the homography matrix based on calibration parameterss and wall image vertices.
 *
 * This function calculates the homography matrix that maps points from the source image (wall images)
 * to the destination image (control points). The homography matrix is stored in the global variable homMat.
 *
 * Control points are specified in normalized coordinates based on the calibration paramiters array.
 * Wall image vertices are calculated based on the dimensions and spacing of the maze walls.
 *
 * @note This function uses the OpenCV library to compute the homography matrix.
 *
 * @param r_hom_mat The homography matrix used to warp perspective.
 * @param cont_point_params The 4x5 array of calibration parameters.
 */
void computeHomography(cv::Mat &, float[4][5]);

/**
 * @brief Used to reset control point parameter list.
 *
 * @param r_cont_point_params Reference to the 4x5 array of calibration parameters.
 * @param mode_cal_ind Index of the active calibration mode.
 */
void updateCalParams(float (&)[4][5], int);

#endif