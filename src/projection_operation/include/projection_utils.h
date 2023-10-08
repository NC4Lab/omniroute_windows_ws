// ####################################################################################################

// ======================================== projection_utils.h ========================================

// ####################################################################################################

#ifndef _PROJECTION_UTILS_H
#define _PROJECTION_UTILS_H

// ================================================== INCLUDE ==================================================

// OpenGL (GLAD and GLFW) for graphics and windowing
#define GLAPIENTRY APIENTRY
#include "glad/glad.h"
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

// DevIL for image loading and manipulation
#include <IL/il.h>
#include <IL/ilu.h>
#include <IL/ilut.h>
#include <IL/devil_cpp_wrapper.hpp>

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

// ================================================== FUNCTIONS ==================================================

/**
 * @brief Formats the file name for the XML file based on the active calibration mode and monitor.
 *
 * Format:
 * - `cfg_c<number>_m<number>.xml`
 *
 * @param cal_ind Index of the active or desired calibration mode.
 * @param mon_ind Index of the active or desired monitor.
 * @param config_dir_path Path to the directory where the XML file will be loaded/saved.
 * 
 */
std::string formatCoordinatesFilePathXML(int cal_ind, int mon_ind, std::string);

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
 * @param cp_param 2D array of control point positions.
 * @param full_path Path to the XML file.
 */
void saveCoordinatesXML(cv::Mat, float[4][5], std::string);

#endif