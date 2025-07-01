// ##########################################################################################################

// =================================================== projection_config.h ===================================

// ##########################################################################################################

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
 *  | p0 | |        |      [c3]      ||      [c4]      ||      [c5]      |        | | p2 |
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
 *      - For the calibration operation, only one GLFWwindow window is used, but it can be moved between monitors.
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

#ifndef _PROJECTION_CONFIG_H
#define _PROJECTION_CONFIG_H

// ================================================== INCLUDE ==================================================

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
#include <tuple>
#include <memory>
#include <unordered_map>

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
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h> 
#include <geometry_msgs/PoseStamped.h>
#include <XmlRpcValue.h>
#include <tf/tf.h>

// PugiXML for XML parsing
#include "pugixml.hpp"

// OpenCV for computer vision tasks
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/hal/interface.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

// ================================================== ENUMERATIONS ==================================================
enum ChamberEnum {
    CHAM_0 = 0, 
    CHAM_1,
    CHAM_2,
    CHAM_3,   
    CHAM_4,
    CHAM_5,
    CHAM_6,
    CHAM_7,
    CHAM_8 
};
const int N_CHAMBERS = 9; // Number of chambers in the maze
const std::array<ChamberEnum, N_CHAMBERS> CHAMBERS = {CHAM_0, CHAM_1, CHAM_2, CHAM_3, CHAM_4, CHAM_5, CHAM_6, CHAM_7, CHAM_8};

enum SurfaceEnum {
    WALL_0 = 0,
    WALL_1,
    WALL_2,
    WALL_3,
    WALL_4,
    WALL_5,
    WALL_6,
    WALL_7,
    FLOOR,
};
const int N_SURF = 9; // Number of surfaces in each chamber (8 walls + 1 floor)
const std::array<SurfaceEnum, N_SURF> SURFACES = {WALL_0, WALL_1, WALL_2, WALL_3, WALL_4, WALL_5, WALL_6, WALL_7, FLOOR};

enum ProjectorEnum {
    PROJ_0 = 0, // Projector 0 (East)
    PROJ_1,     // Projector 1 (North)
    PROJ_2,     // Projector 2 (West)
    PROJ_3      // Projector 3 (South)
};
const int N_PROJ = 4; // Number of projectors
const std::array<ProjectorEnum, N_PROJ> PROJECTORS = {PROJ_0, PROJ_1, PROJ_2, PROJ_3};

enum RowEnum {
    ROW_0 = 0, // Top row
    ROW_1,     // Middle row
    ROW_2      // Bottom row
};
const int N_ROWS = 3; // Number of rows in the chamber grid
const std::array<RowEnum, N_ROWS> ROWS = {ROW_0, ROW_1, ROW_2};

enum ColumnEnum {
    COL_0 = 0, // Left column
    COL_1,     // Center column
    COL_2      // Right column
};
const int N_COLS = 3; // Number of columns in the chamber grid
const std::array<ColumnEnum, N_COLS> COLS = {COL_0, COL_1, COL_2};

// Enum for tracking the current calibration mode
enum CalibrationMode {
    MODE_WALLS_LEFT = 0,
    MODE_WALLS_MIDDLE,
    MODE_WALLS_RIGHT,
    MODE_FLOOR
};
const int N_CAL_MODES = 4; // Number of calibration modes
const std::array<CalibrationMode, N_CAL_MODES> CAL_MODES = {MODE_WALLS_LEFT, MODE_WALLS_MIDDLE, MODE_WALLS_RIGHT, MODE_FLOOR};
const std::array<CalibrationMode, N_CAL_MODES-1> WALL_CAL_MODES = {MODE_WALLS_LEFT, MODE_WALLS_MIDDLE, MODE_WALLS_RIGHT};


// ================================================== VARIABLES ==================================================
/**
 * @brief ROS loop rate in Hertz.
 */
const int GLB_ROS_LOOP_RATE = 100;

/**
 * @brief Global variable for verbose logging.
 */
const bool GLB_DO_VERBOSE_DEBUG = false;

/**
 * @brief Global variable to set the OpenGL debug level.
 *
 * @details  [0: None, 1: >=Default 2: >=Low, 3: >=Medium, 4: High]
 */
const int GLB_DEBUG_LEVEL_GL = 2;

/**
 * @brief Pi
 */
static constexpr float GLOB_PI = 3.14159265358979323846f;

// Get top-level package path
extern const std::string PACKAGE_PATH = ros::package::getPath("projection_operation");
extern const std::string WORKSPACE_PATH = PACKAGE_PATH.substr(0, PACKAGE_PATH.rfind("/src"));

// Directory paths for calibration image files
extern const std::string CONFIG_DIR_PATH = WORKSPACE_PATH + "/data/projection/params";

// Directory paths for runtime image files
extern const std::string IMAGE_DIR_PATH = WORKSPACE_PATH + "/data/projection/images";

/**
 * @brief Image file sub-directory path
 */
const std::string RUNTIME_IMAGE_PATH = IMAGE_DIR_PATH + "/runtime";
const std::string CALIB_IMAGE_PATH = IMAGE_DIR_PATH + "/calibration";

//TODO: All of these paths should be moved to a config file.
/**
 * @brief File names for available calibration test wall images
 * 
 */
std::vector<std::string> CALIB_TEST_WALL_IMAGES = {
    "0_test_wall.png",
    "1_test_wall.png",
    "2_test_wall.png",
    "3_test_wall.png",
};
size_t N_CALIB_TEST_WALL_IMAGES = CALIB_TEST_WALL_IMAGES.size(); // Number of available calibration test wall images

/**
 * @brief File names for available calibration floor images
 * 
 */
std::vector<std::string> CALIB_TEST_FLOOR_IMAGES = {
    "0_test_floor.png",
    "1_test_floor.png"
};
size_t N_CALIB_TEST_FLOOR_IMAGES = CALIB_TEST_FLOOR_IMAGES.size(); // Number of available calibration test floor images

/**
 * @brief File names for available calibration monitor wall images
 * 
 */
std::vector<std::string> CALIB_MON_WALL_IMAGES = {
    "w_m0.png",
    "w_m1.png",
    "w_m2.png",
    "w_m3.png",
    "w_m4.png",
    "w_m5.png"
};
size_t N_CALIB_MON_WALL_IMAGES = CALIB_MON_WALL_IMAGES.size(); // Number of available calibration monitor wall images

/**
 * @brief File names for available calibration monitor floor images 
 * 
 */
std::vector<std::string> CALIB_MON_FLOOR_IMAGES = {
    "f_m0.png",
    "f_m1.png",
    "f_m2.png",
    "f_m3.png",
    "f_m4.png",
    "f_m5.png"
};
size_t N_CALIB_MON_FLOOR_IMAGES = CALIB_MON_FLOOR_IMAGES.size(); // Number of available calibration monitor floor images

/**
 * @brief List of available calibration mode images
 *
 */
std::vector<std::string> CALIB_MODE_IMAGES = {
    "w_c0.png", // left walls
    "w_c1.png", // middle walls
    "w_c2.png", // right walls
    "f_c0.png", // maze floor
};
size_t N_CALIB_MODE_IMAGES = CALIB_MODE_IMAGES.size(); // Number of available calibration mode images

/**
 * @brief File names for available runtime wall images
 * 
 * @note This list needs to match that used in:
 * omniroute_ubuntu_ws\src\omniroute_operation\src\shared_utils\projection_operation.py
 */
std::vector<std::string> RUNTIME_WALL_IMAGES = {
    "w_black.png",
    "w_square.png",
    "w_square.png",
    "w_triangle.png",
    "w_star.png",
    "w_pentagon.png",
    "w_rm_blue_left.png",
    "w_rm_blue_middle.png",
    "w_rm_blue_right.png",
    "w_rm_green_left.png",
    "w_rm_green_middle.png",
    "w_rm_green_right.png",
    "w_rm_teal_left.png",
    "w_rm_teal_middle.png",
    "w_rm_teal_right.png"
};
size_t N_RUNTIME_WALL_IMAGES = RUNTIME_WALL_IMAGES.size(); // Number of available runtime wall images

/**
 * @brief File names for available runtime floor images
 * 
 * @note This list needs to match that used in:
 * omniroute_ubuntu_ws\src\omniroute_operation\src\shared_utils\projection_operation.py
 */
std::vector<std::string> RUNTIME_FLOOR_IMAGES = {
    "f_black.png",
    "f_green.png",
    "f_pattern_0.png",
    "f_pattern_1.png",
    "f_pattern_2.png",
    "f_white.png"
};
size_t N_RUNTIME_FLOOR_IMAGES = RUNTIME_FLOOR_IMAGES.size(); // Number of available runtime floor images


#endif