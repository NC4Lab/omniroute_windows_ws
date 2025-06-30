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
const std::array<ChamberEnum, N_CHAMBERS> Chambers = {CHAM_0, CHAM_1, CHAM_2, CHAM_3, CHAM_4, CHAM_5, CHAM_6, CHAM_7, CHAM_8};

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
const std::array<SurfaceEnum, N_SURF> Surfaces = {WALL_0, WALL_1, WALL_2, WALL_3, WALL_4, WALL_5, WALL_6, WALL_7, FLOOR};

enum ProjectorEnum {
    PROJ_0 = 0, // Projector 0 (East)
    PROJ_1,     // Projector 1 (North)
    PROJ_2,     // Projector 2 (West)
    PROJ_3      // Projector 3 (South)
};
const int N_PROJ = 4; // Number of projectors
const std::array<ProjectorEnum, N_PROJ> Projectors = {PROJ_0, PROJ_1, PROJ_2, PROJ_3};

enum RowEnum {
    ROW_0 = 0, // Top row
    ROW_1,     // Middle row
    ROW_2      // Bottom row
};
const int N_ROWS = 3; // Number of rows in the chamber grid
const std::array<RowEnum, N_ROWS> Rows = {ROW_0, ROW_1, ROW_2};

enum ColumnEnum {
    COL_0 = 0, // Left column
    COL_1,     // Center column
    COL_2      // Right column
};
const int N_COLS = 3; // Number of columns in the chamber grid
const std::array<ColumnEnum, N_COLS> Columns = {COL_0, COL_1, COL_2};

// Enum for tracking the current calibration mode
enum CalibrationMode {
    MODE_WALLS_LEFT = 0,
    MODE_WALLS_MIDDLE,
    MODE_WALLS_RIGHT,
    MODE_FLOOR
};
const int N_CAL_MODES = 4; // Number of calibration modes
const std::array<CalibrationMode, N_CAL_MODES> CalibrationModes = {MODE_WALLS_LEFT, MODE_WALLS_MIDDLE, MODE_WALLS_RIGHT, MODE_FLOOR};
const std::array<CalibrationMode, N_CAL_MODES-1> WallCalibrationModes = {MODE_WALLS_LEFT, MODE_WALLS_MIDDLE, MODE_WALLS_RIGHT};


// ================================================== VARIABLES ==================================================
/**
 * @brief ROS loop rate in Hertz.
 */
const int GLB_ROS_LOOP_RATE = 100;

/**
 * @brief Global variable for verbose logging.
 */
const bool GLB_DO_VERBOSE_DEBUG = true;

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

/**
 * @brief 4D array of hardcoded image indices to display.
 *
 * @details
 * This array is used to map specific wall image indices to a combination of
 * projector, chamber row, chamber column, calibration mode, and wall position.
 *
 * [4][3][3][3] = [number of projectors][grid rows][grid columns][calibration modes]
 *
 * - Dimension 1: Projectors [0, 1, 2, 3]
 *
 * - Dimension 2: Calibration Mode [0: left walls, 1: middle walls, 2: right walls]
 *
 * - Dimension 3: Grid Rows [0, 1, 2]
 *   - Index with respect to the grid rows in the chamber.
 *
 * - Dimension 4: Grid Columns [0, 1, 2]
 *   - Index with respect to the grid columns in the chamber.
 *
 * Image file mapping for shapes:
 * [0] Black
 * [1] Square
 * [2] Circle
 * [3] Triangle
 * [4] Star
 * [5] Pentagon
 *
 * Element mapping:
 *
 *    {{// Projector 0: East
 *      {{
 *          // Chamber Row: Top, Column: Left, Center, Right
 *          {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}
 *                                               // Chamber Row: Middle
 *          {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}
 *                                               // Chamber Row: Bottom
 *          {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}  // {Calibration Mode: Left, Center, Right}, {...}, {...}
 *      }},
 *      // Projector 1: North
 *      {{
 *          // Chamber Row: Top, Column: Left, Center, Right
 *          {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}
 *                                               // Chamber Row: Middle
 *          {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}
 *                                               // Chamber Row: Bottom
 *          {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}  // {Calibration Mode: Left, Center, Right}, {...}, {...}
 *      }},
 *      // Projector 2: West
 *      {{
 *          // Chamber Row: Top, Column: Left, Center, Right
 *          {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}
 *                                               // Chamber Row: Middle
 *          {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}
 *                                               // Chamber Row: Bottom
 *          {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}  // {Calibration Mode: Left, Center, Right}, {...}, {...}
 *      }},
 *      // Projector 3: South
 *      {{
 *          // Chamber Row: Top, Column: Left, Center, Right
 *          {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}
 *                                               // Chamber Row: Middle
 *          {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}
 *                                               // Chamber Row: Bottom
 *          {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}  // {Calibration Mode: Left, Center, Right}, {...}, {...}
 *      }}}};
 */
using ProjWallConfigIndices4D = std::array<std::array<std::array<std::array<int, 3>, 3>, 3>, 4>;

#endif