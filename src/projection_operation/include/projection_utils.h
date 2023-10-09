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

// Number of rows and columns in the maze
extern const int MAZE_SIZE = 3;

// Wall image size and spacing (pixels)
extern const int WALL_WIDTH_PXL = 300;
extern const int WALL_HEIGHT_PXL = 540;

// Wall image size and spacingOpenGL's Normalized Device Coordinates (NDC) [-1, 1]
/// @todo: Figure out how these values are used and what units they are in
extern const float WALL_WIDTH = 0.02f;
extern const float WALL_SPACE = 2.5f * WALL_WIDTH;

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

// ================================================== FUNCTIONS ==================================================

// void initializeGL(GLFWwindow *&, int, int, std::string, GLuint &);

/**
 * @brief Formats the file name for the XML file based on the active calibration mode and monitor.
 *
 * Format:
 * - `cfg_m<number>_c<number>.xml`
 *
 * @param mon_ind Index of the active or desired monitor.
 * @param cal_ind Index of the active or desired calibration mode.
 * @param config_dir_path Path to the directory where the XML file will be loaded/saved.
 *
 */
std::string formatCoordinatesFilePathXML(int, int, std::string);

/**
 * @brief Loads homography matrix from an XML file.
 *
 * This function is a wrapper that calls the 3-argument version with a dummy 2D array.
 *
 * @param ref_H Homography matrix to populate.
 * @param full_path Path to the XML file.
 */
void loadCoordinatesXML(cv::Mat &ref_H, std::string full_path);

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
void loadCoordinatesXML(cv::Mat &ref_H, float (&ref_cp_param)[4][5], std::string full_path);

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
 * @param cp_param 2D array of control point positions.
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
 * @param ref_image_ids_vec A reference to a vector of ILuint where the IDs of the loaded images will be stored.
 * @param img_paths_vec A vector of file paths to the images to be loaded.
 *
 * @note Utilizes the DevIL image library for image loading operations.
 */
void loadImgTextures(std::vector<ILuint> &, std::vector<std::string>);

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
 * @brief Calculates an interpolated value using bilinear interpolation on a 2D grid.
 *
 * This function performs bilinear interpolation based on the position of a point
 * within a 2D grid (grid_i_ind, grid_j_ind) and predefined values at the grid corners.
 *
 * @param cp_param The array of control point parameters.
 * @param cp_ind The index of the control point parameter (3:height, 4:sheer).
 * @param grid_i_ind The index of the point along the first axis within the grid.
 * @param grid_j_ind The index of the point along the second axis within the grid.
 * @param GRID_SIZE The size of the 2D grid.
 *
 * @return float The calculated interpolated value.
 */
float calculateInterpolatedValue(float[4][5], int, int, int, int);

/**
 * @brief Creates a vector of points representing a rectangle with shear for each wall.
 *
 * This function generates a rectangle's corner points starting from the top-left corner
 * and going clockwise. The rectangle is defined by its top-left corner (x0, y0),
 * width, height, and a shear amount. The units for x0, y0, width, and height are in
 * OpenGL's Normalized Device Coordinates (NDC) [-1, 1].
 *
 * @param x0 The x-coordinate of the top-left corner of the rectangle in NDC.
 * @param y0 The y-coordinate of the top-left corner of the rectangle in NDC.
 * @param width The width of the rectangle in NDC.
 * @param height The height of the rectangle in NDC.
 * @param shear_amount The amount of shear to apply to the rectangle.
 *
 * @return std::vector<cv::Point2f> A vector of 4 points representing the corners of the rectangle, in NDC.
 */
std::vector<cv::Point2f> computeRectVertices(float, float, float, float, float);

/** 
 * @brief Computes the perspective warp for a given set of points.
 * 
 * @param rect_vertices_vec The vector of points representing the rectangle.
 * @param ref_H The homography matrix used to warp perspective.
 * @param x_offset The x-offset to apply to the vertices.
 * @param y_offset The y-offset to apply to the vertices.
 * 
 * @return std::vector<cv::Point2f> The warped vertices.
*/
std::vector<cv::Point2f> computePerspectiveWarp(std::vector<cv::Point2f>, cv::Mat &, float, float);

#endif