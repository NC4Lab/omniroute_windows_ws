// ##########################################################################################################

// =================================================== projection_config.h ===================================

// ##########################################################################################################

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

// ================================================== VARIABLES ==================================================

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
 * @brief 4D array of hardcoded image indices to display.
 *
 * @details
 * This array is used to map specific wall image indices to a combination of
 * projector, chamber row, chamber column, calibration mode, and wall position.
 *
 * The array dimensions are as follows:
 * - Projector: 0 to 3
 * - Maze Chamber Row: 0 to 2
 * - Maze Chamber Column: 0 to 2
 * - Calibration Mode: 0 to 2 (represents l_wall, m_wall, r_wall)
 *
 * Image file mapping for shapes:
 * [0] Blank
 * [1] Square
 * [2] Circle
 * [3] Triangle
 * [4] Star
 * [5] Pentagon
 * 
 * Format: array[4][3][3][3] = array[Projector][Chamber Row][Chamber Column][Calibration Mode{Left, Center, Right}]
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
using ProjWallImageCfg4D = std::array<std::array<std::array<std::array<int, 3>, 3>, 3>, 4>;

/**
 * @brief 1D array for hardcoded floor image indices to display
 *
 * @details
 * This array is used to map specific floor image indices to a combination a
 * given projector.
 *
 * Image file mapping for grayscale:
 * [0] Black
 * [1] Gray (20%)
 * [2] Gray (40%)
 * [3] Gray (60%)
 * [4] Gray (80%)
 * [5] White
 * [6] Pattern
 * 
 * Element mapping:
 * {
 *    0, // Projector 0: West
 *    0, // Projector 1: North
 *    0, // Projector 2: East
 *    0, // Projector 3: South
 * };
 */
using ProjFloorImageCfg1D = std::array<int, 4>;

// std::array<std::array<std::array<std::array<int, 3>, 3>, 3>, 4> GLB_WALL_IMG_PROJ_MAP =
//     {{// Projector 0: West Projector (Facing East)
//       {{
//           // Chamber Row: Top, Column: Left, Center, Right
//           {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}
//                                                // Chamber Row: Middle
//           {{{0, 0, 0}, {A, 0, B}, {0, 0, 0}}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}
//                                                // Chamber Row: Bottom
//           {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}  // {Calibration Mode: Left, Center, Right}, {...}, {...}
//       }},
//       // Projector 1: North Projector (Facing South)
//       {{
//           // Chamber Row: Top, Column: Left, Center, Right
//           {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}
//                                                // Chamber Row: Middle
//           {{{0, 0, 0}, {B, 0, C}, {0, 0, 0}}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}
//                                                // Chamber Row: Bottom
//           {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}  // {Calibration Mode: Left, Center, Right}, {...}, {...}
//       }},
//       // Projector 2: Ease Projector (Facing West)
//       {{
//           // Chamber Row: Top, Column: Left, Center, Right
//           {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}
//                                                // Chamber Row: Middle
//           {{{0, 0, 0}, {C, 0, D}, {0, 0, 0}}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}
//                                                // Chamber Row: Bottom
//           {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}  // {Calibration Mode: Left, Center, Right}, {...}, {...}
//       }},
//       // Projector 3: South Projector (Facing North)
//       {{
//           // Chamber Row: Top, Column: Left, Center, Right
//           {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}
//                                                // Chamber Row: Middle
//           {{{0, 0, 0}, {D, 0, A}, {0, 0, 0}}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}
//                                                // Chamber Row: Bottom
//           {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}}  // {Calibration Mode: Left, Center, Right}, {...}, {...}
//       }}}};

#endif