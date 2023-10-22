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
 * - Chamber Row: 0 to 2
 * - Chamber Column: 0 to 2
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

// // Wall image size and spacingOpenGL's Normalized Device Coordinates (NDC) [-1, 1]
// /// @todo: Figure out how these values are used and what units they are in
// const float wall_width_ndc = 0.015f;
// const float wall_height_ndc = 0.015f;
// extern const float WALL_SPACE = 2.0f * wall_width_ndc;

// // Variables related to control point parameters
// extern const float CP_RADIUS_NDC = 0.005f;
// const float cal_offset_x = 0.15f; // X offset from center of screen for control points
// const float cal_offset_y = 0.3f; // X offset from center of screen for control points

// Control point image radius
extern const float CP_RADIUS_NDC = 0.005f;

// Defualt offset of control points from the center of the screen
const float cal_offset_x = 0.15f; // X offset from center of screen for control points
const float cal_offset_y = 0.3f;  // X offset from center of screen for control points

// Wall image size and spacingOpenGL's Normalized Device Coordinates (NDC) [-1, 1]
/// @todo: Figure out how these values are used and what units they are in
const float wall_width_ndc = ((cal_offset_x * 2) / (float(MAZE_SIZE) - 1)) / (1 + std::sqrt(2));
const float wall_height_ndc = ((cal_offset_y * 2) / (float(MAZE_SIZE) - 1)) / (1 + std::sqrt(2));
extern const float WALL_SPACE = 2.0f * wall_width_ndc;

// Calibration parameter array
/**
 * @brief Array to hold the position and transformation parameters for control points in normalized coordinates [-1, 1].
 *
 * @note The second control point (index 1) can only be used to adjust the wall position.
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
 *   - [2]: Wall width parameter [-1,1]
 *   - [3]: Wall height parameter [-1,1]
 *   - [4]: Shearing factor
 */
extern const float CAL_PARAM_DEFAULT[4][5] = {
    // Default control point parameters
    {-cal_offset_x, cal_offset_y, wall_width_ndc, wall_height_ndc, 0.0f}, // top-left control point
    {cal_offset_x, cal_offset_y, wall_width_ndc, wall_height_ndc, 0.0f},  // top-right control point
    {cal_offset_x, -cal_offset_y, wall_width_ndc, wall_height_ndc, 0.0f}, // bottom-right control point
    {-cal_offset_x, -cal_offset_y, wall_width_ndc, wall_height_ndc, 0.0f} // bottom-left control point
};

// ================================================== CLASSES ==================================================

/**
 * @class GLFWWrapper
 * @brief A wrapper class for managing GLFW resources.
 *
 * This class encapsulates the GLFWwindow and GLFWmonitor pointers,
 * and provides utility methods for creating windows, getting monitors,
 * and cleaning up resources. It adheres to the RAII principles.
 */
class GLFWWrapper {
private:
    GLFWwindow *p_windowID;         ///< Pointer to GLFW window.
    GLFWmonitor **pp_monitorIDVec;  ///< Pointer to array of GLFW monitors.

    /**
     * @brief Cleans up the GLFW resources.
     * 
     * Destroys the GLFW window and terminates GLFW.
     */
    void cleanUp();

public:
    int nMonitors;               ///< Number of monitors.
    bool initStatus;             ///< Status of GLFW initialization.
    
    /**
     * @brief Constructor that initializes GLFW.
     */
    GLFWWrapper();

    /**
     * @brief Destructor that releases GLFW resources.
     */
    ~GLFWWrapper();

    /**
     * @brief Deleted copy constructor.
     */
    GLFWWrapper(const GLFWWrapper&) = delete;

    /**
     * @brief Deleted copy assignment operator.
     */
    GLFWWrapper& operator=(const GLFWWrapper&) = delete;

    /**
     * @brief Get the window ID.
     * 
     * @return Pointer to GLFW window.
     */
    GLFWwindow* getWindowID() const;

    /**
     * @brief Get the monitor ID vector.
     * 
     * @return Pointer to array of GLFW monitors.
     */
    GLFWmonitor** getMonitorIDVec() const;

    /**
     * @brief Creates a GLFW window.
     * 
     * @param width Width of the window.
     * @param height Height of the window.
     * @param title Title of the window.
     * @param monitor Monitor to use.
     * @param share Window whose context to share.
     * @return True if window creation is successful, false otherwise.
     */
    bool createWindow(int width, int height, const char* title, GLFWmonitor* monitor = nullptr, GLFWwindow* share = nullptr);
};

// ================================================== FUNCTIONS ==================================================

/**
 * @brief Checks for DevIL errors and logs them.
 * Should be called after DevIL API calls.
 *
 * @example checkErrorDevIL(__LINE__, __FILE__);
 *
 * @param[in] int Line number where the function is called.
 * @param[in] const char* File name where the function is called.
 * @param[in] const char* Optional message to provide additional context (default to nullptr).
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
 * @param mon_ind Index of the active or desired monitor.
 * @param cal_ind Index of the active or desired calibration mode.
 * @param config_dir_path Path to the directory where the XML file will be loaded/saved.
 *
 */
std::string formatCoordinatesFilePathXML(int, int, std::string);

/**
 * @brief Loads control points and homography matrix from an XML file.
 *
 * This is the primary function containing the implementation. It reads an XML file
 * to populate the `ref_H` and `ref_cal_param_arr` matrices.
 *
 * @note Uses pugiXML for XML parsing.
 *
 * @param ref_H Homography matrix to populate.
 * @param ref_cal_param_arr 2D array for control points in normalized coordinates [-1, 1].
 * @param full_path Path to the XML file.
 * @param verbose_level Level of verbosity for printing loaded data (0:nothing, 1:file name, 2:control points, 3:homography matrix).
 *
 * @return 0 on successful execution, -1 on failure.
 */
int loadCoordinatesXML(cv::Mat &, float (&)[4][5], std::string, int = 0);

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
 *   <calParam>
 *     <Row>
 *       <Cell>value</Cell>
 *       ...
 *     </Row>
 *     ...
 *   </calParam>
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
 * @param cal_param_arr 2D array of control point positions.
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
 * @return DevIL status: 0 on successful execution, -1 on failure.
 */
int loadImgTextures(std::vector<ILuint> &, std::vector<std::string>);

/**
 * @brief Deletes DevIL images from a given vector of image IDs.
 *
 * @param ref_image_ids_vec Vector containing DevIL image IDs.
 * 
  * @return DevIL status: 0 on successful execution, -1 on failure.
 */
int deleteImgTextures(std::vector<ILuint>&);

/**
 * @brief Merges two images by overlaying non-white pixels from the second image onto the first.
 *
 * This function takes two images, img1 and img2, represented as ILuint IDs. It overlays img2 onto img1,
 * replacing pixels in img1 with corresponding non-white pixels from img2. The resulting merged image is
 * returned as a new ILuint ID.
 *
 * @param img1 The ILuint ID of the baseline image.
 * @param img2 The ILuint ID of the mask image.
 * @param[out] ref_img_merge_id Reference to an ILuint where the ID of the merged image will be stored.
 * 
 * @return DevIL status: 0 on successful execution, -1 on failure.
 *
 * @warning The dimensions of img1 and img2 must match.
 */
int mergeImages(ILuint, ILuint, ILuint &);

/**
 * @brief Calculate corner spacings based on calibration parameters.
 *
 * This function computes the wall spacings at each of the four corners of the maze
 * based on the given calibration parameters.
 *
 * @param cal_param_arr 4x5 array of calibration parameters.
 * @param [out] corner_spacings_x 2x2 array to store calculated corner spacings in x-direction.
 * @param [out] corner_spacings_y 2x2 array to store calculated corner spacings in y-direction.
 * @param maze_size Number of cells in the maze along one dimension.
 */
void calculateCornerSpacing(float[4][5], float (&)[2][2], float (&)[2][2], int);

/**
 * @brief Calculate interpolated wall spacing for either x or y direction.
 *
 * This function uses the corner spacings and the wall's position within the grid
 * to compute the interpolated wall spacing in either x or y direction.
 *
 * @param corner_spacings 2x2 array containing the spacings at each corner.
 * @param wall_i The row index of the wall within the maze grid.
 * @param wall_j The column index of the wall within the maze grid.
 * @param maze_size The size of the maze grid.
 * @return The interpolated wall spacing.
 */
float calculateInterpolatedWallSpacing(float[2][2], float, float, int);

/**
 * @brief Calculates an interpolated value using bilinear interpolation on a 2D grid.
 *
 * This function performs bilinear interpolation based on the position of a point
 * within a 2D grid (grid_ind_i, grid_ind_j) and predefined values at the grid corners.
 *
 * @param cal_param_arr The array of control point parameters.
 * @param cal_param_ind The index of the control point parameter (3:height, 4:sheer).
 * @param grid_ind_i The index of the point along the first axis within the grid.
 * @param grid_ind_j The index of the point along the second axis within the grid.
 * @param grid_size The size of the 2D grid.
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

/**
 * @brief Computes the homography matrix based on control points and wall image vertices.
 *
 * This function calculates the homography matrix that maps points from the source image (wall images)
 * to the destination image (control points). The homography matrix is stored in the global variable H.
 *
 * Control points are specified in normalized coordinates based on the calibration paramiters array.
 * Wall image vertices are calculated based on the dimensions and spacing of the maze walls.
 *
 * @note This function uses the OpenCV library to compute the homography matrix.
 *
 * @param ref_H The homography matrix used to warp perspective.
 * @param cal_param_arr The array of control point parameters.
 */
void computeHomography(cv::Mat &, float[4][5]);

/**
 * @brief Used to reset control point parameter list.
 *
 * @param ref_cal_param Reference to 2D calibration parameter array.
 * @param cal_ind Index of the active calibration mode.
 */
void updateCalParams(float (&)[4][5], int);

#endif