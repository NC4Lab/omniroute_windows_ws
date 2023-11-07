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

#pragma once

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

// ================================================== CLASS: MazeRenderContext ==================================================

#ifndef MAZE_RENDER_CONTEXT_CALLBACKS_H
#define MAZE_RENDER_CONTEXT_CALLBACKS_H

// Callback function type for key events
using KeyCallbackFunc = void (*)(GLFWwindow *, int, int, int, int);

#endif // MAZE_RENDER_CONTEXT_CALLBACKS_H

#ifndef MAZE_RENDER_CONTEXT_H
#define MAZE_RENDER_CONTEXT_H

class MazeRenderContext
{
public:
    GLuint shaderProgram;             // Shader program for wall rendering
    const char *vertexShaderSource;   // Vertex shader source code for wall rendering
    const char *fragmentShaderSource; // Fragment shader source code for wall rendering
    GLuint vao;                       // Vertex Array Object
    GLuint vbo;                       // Vertex Buffer Object
    GLuint ebo;                       // Element Buffer Object
    GLuint textureID;                 // Texture for the wall
    GLFWwindow *windowID;             // The window associated with this context
    GLFWmonitor *monitorID;           // The monitor associated with this context
    float *verticesArr;               // Vertex data for the wall's geometry
    unsigned int *indicesArr;         // Index data for rendering the wall using triangles
    int windowInd;                    // Index of the window associated with this context
    int monitorInd;                   // Index of the monitor associated with this context
private:
    static GLFWmonitor **_PP_Monitor;
    static int _NumMonitors;

public:
    // Constructor
    MazeRenderContext();

    // Destructor
    ~MazeRenderContext();

    // Copy constructor and copy assignment are deleted to avoid accidental copying
    MazeRenderContext(const MazeRenderContext &) = delete;
    MazeRenderContext &operator=(const MazeRenderContext &) = delete;

    // Move constructor and move assignment for efficient transfer of resources
    MazeRenderContext(MazeRenderContext &&other) noexcept;
    MazeRenderContext &operator=(MazeRenderContext &&other) noexcept;

    /**
     * @brief Callback function for handling framebuffer size changes.
     *
     * @param window Pointer to the GLFW window.
     * @param width The new width of the framebuffer.
     * @param height The new height of the framebuffer.
     *
     * @details
     * This function is called whenever the framebuffer size changes,
     * and it updates the OpenGL viewport to match the new dimensions.
     */
    static void CallbackFrameBufferSizeGLFW(GLFWwindow *window, int width, int height);

    /**
     * @brief Callback function for handling OpenGL errors.
     *
     * @param source The source of the error.
     * @param type The type of the error.
     * @param id The error ID.
     * @param severity The severity of the error.
     * @param length The length of the error message.
     * @param message The error message.
     * @param userParam User-defined parameter.
     *
     * @details
     * This function is called whenever an error occurs in the OpenGL context.
     * It logs the error message using ROS_ERROR.
     */
    static void CallbackDebugOpenGL(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message, const void *userParam);

    /**
     * @brief Callback function for handling errors.
     *
     * @param error The error code.
     * @param description The error description.
     *
     * @details
     * This function is called whenever an error occurs in the GLFW context.
     * It logs the error message using ROS_ERROR.
     */
    static void CallbackErrorGLFW(int error, const char *description);

    /**
     * @brief Checks for OpenGL errors and logs them.
     * Should be called after OpenGL API calls.
     *
     * @param line Line number where the function is called.
     * @param file_str File name where the function is called.
     * @param msg_str Optional message to provide additional context (default to nullptr).
     *
     * @return Integer status code  [0:successful, -1:error].
     *
     *
     * @example CheckErrorOpenGL(__LINE__, __FILE__);
     */
    static int CheckErrorOpenGL(int, const char *, const char * = nullptr);

    /**
     * @brief Checks for GLFW errors and logs them.
     * Should be called after GLFW API calls.
     *
     * @param line Line number where the function is called.
     * @param file_str File name where the function is called.
     * @param msg_str Optional message to provide additional context (default to nullptr).
     *
     * @return Integer status code  [0:successful, -1:error].
     *
     * @example CheckErrorGLFW(__LINE__, __FILE__);
     */
    static int CheckErrorGLFW(int, const char *, const char * = nullptr);

    /**
     * @brief Initializes GLFW and discovers monitors.
     *
     * Sets up GLFW, including setting an error callback and initializing the library.
     * It also discovers and stores the available monitors, reporting the total count.
     *
     * @param[out] out_n_mon Reference to an integer to store the number of monitors found.
     * @return int Status of setup (0 for success, -1 for failure).
     */
    int static SetupGraphicsLibraries(int &n_mon);

    /**
     * @brief Cleans up GLFW and resets monitor info.
     *
     * Terminates GLFW to clean up all resources and resets monitor pointers and count.
     * Logs the result of the cleanup process.
     *
     * @return int Status of cleanup (0 for success, -1 for failure indicated by GLFW errors).
     */
    static int CleanupGraphicsLibraries();

    /**
     * @brief Initializes a new rendering context.
     *
     * Checks and sets a new monitor based on the monitor index, creates a new GLFW window,
     * and sets up the OpenGL context with necessary callbacks and extensions.
     *
     * @param win_ind Index of the window to initialize.
     * @param mon_ind Index of the monitor to use for the window.
     * @param key_callback Optional key callback function to set for the window (default to nullptr).
     * @return int Status of context initialization (0 for success, -1 for failure).
     */
    int initContext(int win_ind, int mon_ind, KeyCallbackFunc key_callback = nullptr);

    /**
     * @brief Cleans up all OpenGL resources.
     *
     * This method deletes the shader program, textures, VAO, VBO, EBO, and destroys
     * the GLFW window if they have been created. It also logs the cleanup process if
     * logging is enabled.
     *
     * @param log_errors If set to true, the method logs the status of each resource
     *                   cleanup operation.
     * @return An integer status code. If all resources are cleaned up without any
     *         OpenGL errors, it returns 0. If there are any OpenGL errors during
     *         cleanup, it returns -1.
     */
    int cleanupContext(bool log_errors = true);

    /**
     * @brief Compiles and links shaders for a given class instance.
     *
     * @param vertex_source Source code for the vertex shader stored as a C++ raw string literal.
     * @param fragment_source Source code for the fragment shader stored as a C++ raw string literal.
     * @return Integer status code  [0:successful, -1:error].
     *
     * @details
     * This function encapsulates the process of creating, compiling, and linking an OpenGL shader program.
     * The OpenGL shader program is part of the OpenGL graphics pipeline and is essential for rendering graphics.
     *
     * The function compiles the vertex and fragment shaders from their source code.
     * It then links them into a shader program, which can be activated with glUseProgram().
     *
     * - Vertex Shader: Takes attributes like position, color, texture coordinates, normals, etc.,
     *   and computes processed values to be used in later pipeline stages.
     *
     * - Fragment Shader: Takes interpolated attributes from the rasterization stage and computes
     *   the final color of a pixel. This is the stage where things like texture mapping,
     *   lighting calculations, etc., would typically be performed.
     *
     * Once the shader program is created and linked successfully, it returns the GLuint ID of the shader program.
     * This ID is used to activate the shader program for rendering.
     */
    int compileAndLinkShaders(const GLchar *vertex_source, const GLchar *fragment_source);

    /**
     * @brief Cleans up shader objects.
     * @return Integer status code  [0:successful, -1:error].
     *
     * @details
     * This method deletes the shader program associated with
     * the CircleRenderer class. It should be called once when
     * the application is terminating, to ensure proper cleanup
     * of OpenGL resources.
     */
    int cleanupShaderObjects();

    /**
     * @brief Changes the display mode and monitor of the application window.
     *
     *
     * @param mon_id_ind Index of the monitor to move the window to.
     * @param is_fullscreen Boolean flag indicating whether the window should be set to full-screen mode.
     * @return Integer status code  [0:successful, -1:error].
     *
     * @details
     * This function switches the application window between full-screen and windowed modes
     * and moves it to the monitor specified by the global variable imgMonNumInd.
     */
    int switchWindowMode(int, bool);

    /**
     * @brief Sets the GLFW window to always be on top or not.
     *
     * This function uses the GLFW library to set the given window's
     * floating attribute, which determines whether it should always
     * stay on top of other windows.
     *
     * @note I don't think the glfwSetWindowAttrib() call does anything
     *
     * @param is_top_always A boolean flag to set or unset the window as always on top.
     * @return Integer status code  [0:successful, -1:error].
     */
    int setWindowStackOrder(bool is_top_always);

    /**
     * @brief Function to set the background color and redraw the window
     *
     * @param color The color to set the background to.
     * @param duration The duration to flash the background for (ms).
     *
     * @example flashBackgroundColor(window, cv::Scalar(0.1f, 0.0f, 0.0f), 0.5f);
     */
    void flashBackgroundColor(const cv::Scalar &color, int duration);

private:
    /**
     * @brief Private helper methods to check shader compilation.
     *
     * @param shader The shader to check.
     * @param shader_type The type of shader to check.
     */
    bool _checkShaderCompilation(GLuint shader, const std::string &shader_type);

    /**
     * @brief Private helper methods to check shader linking.
     *
     * @param program The shader program to check.
     */
    bool _checkProgramLinking(GLuint program);

    // Private method to check monitor index and id is valid
    int _checkMonitor(int mon_ind);

    // Simple test of OpenGL and GLFW callbacks
    void _testCallbacks(GLFWwindow *win);
};

#endif // MAZE_RENDER_CONTEXT_H

// ================================================== CLASS: CircleRenderer ==================================================

#ifndef CIRCLE_RENDERER_H
#define CIRCLE_RENDERER_H

/**
 * @class CircleRenderer
 * @brief Encapsulates a circle's properties and its rendering.
 *
 * This class is designed to represent a circle with various attributes such as
 * position, color, and scale. It also provides functionality to compute vertex
 * data for rendering the circle in OpenGL and update its transformation matrix.
 */
class CircleRenderer
{
public:
    /**
     * @brief Vertex shader source code with aspect ratio correction.
     *
     * @note The shader program will reverse the y axis to match image coordinate systems used by OpenCV.
     *
     * @details
     * This GLSL vertex shader code corrects for the aspect ratio of the display
     * when rendering circle vertices.
     *
     * - `#version 330 core`: Specifies that we're using GLSL version 3.30 with the core profile.
     * - `layout (location = 0) in vec2 aPos;`: Defines the input vertex attribute at location 0.
     * - `uniform mat4 transform;`: A uniform for the transformation matrix.
     * - `uniform float aspectRatio;`: A uniform for the aspect ratio correction. Also flips the y-coordinate.
     * - `void main() { ... }`: The main function that applies transformation and aspect ratio correction.
     *
     * Use this shader source by compiling it into a vertex shader object and linking it into a shader program.
     * Set the `aspectRatio` uniform before drawing to correct the y-coordinate of vertices based on the display aspect ratio.
     */
    static constexpr const char *vertexShaderSource = R"glsl(
    #version 330 core
    layout (location = 0) in vec2 aPos;
    uniform mat4 transform;
    uniform float aspectRatio; // Uniform for the aspect ratio

    void main() {
        vec2 correctedPos = aPos;
        correctedPos.y *= -aspectRatio; // Flip the y-coordinate and correct the aspect ratio
        gl_Position = transform * vec4(correctedPos, 0.0, 1.0);
    }
    )glsl";
    /**
     * @brief Fragment shader source code for circle coloring.
     *
     * @details
     * This is a GLSL fragment shader source code stored as a C++ raw string literal.
     * - `#version 330 core`: Specifies that the GLSL version is 3.30 and we're using the core profile.
     * - `out vec4 FragColor;`: Declares an output variable for the final fragment color.
     * - `uniform vec4 color;`: Declares a uniform variable for the color, which can be set dynamically via OpenGL calls.
     * - `void main() { ... }`: Main function of the fragment shader, sets the fragment color to the uniform color value.
     */
    static constexpr const char *fragmentShaderSource = R"glsl(
        #version 330 core
        out vec4 FragColor;
        uniform vec4 color;

        void main() {
            FragColor = color;
        }
    )glsl";

public:
    int circID;                     // Index of the circle, used for identification.
    cv::Scalar circColor;           // Color of the circle.
    cv::Point2f circPosition;       // Position of the circle in 2D space.
    float cirRadius;                // Radius of the circle.
    float circRotationAngle;        // Rotation angle of the circle in degrees.
    cv::Point2f circScalingFactors; // Scaling factors for the circle's x and y dimensions.
    unsigned int circSegments;      // Number of segments used to approximate the circle geometry.

private:
    std::vector<float> _circVertices;                     // Vertex data for the circle's geometry.
    GLuint _vao;                                          // Vertex Array Object for the circle.
    GLuint _vbo;                                          // Vertex Buffer Object for the circle's vertices.
    cv::Mat _transformationMatrix;                        // Transformation matrix for the circle's vertices.
    static constexpr float _PI = 3.14159265358979323846f; // Pi
    static int _CircCnt;                                  // Static index counter for the CircleRenderer class objects
    static GLuint _ShaderProgram;                         // Shader program for rendering
    static GLint _ColorLocation;                          // Location of color uniform in shader
    static GLint _TransformLocation;                      // Location of transform uniform in shader
    static GLint _AspectRatioLocation;                    // Location of aspect ratio uniform in shader
    static float _AspectRatioUniform;                     // Aspect ratio uniform for the shader program

public:
    /**
     * @brief Defualt construct for new CircleRenderer object.
     */
    CircleRenderer();

    /**
     * @brief Destructor for the CircleRenderer object.
     * Cleans up the OpenGL objects associated with the circle.
     */
    ~CircleRenderer();
    /**
     * @brief Initializes the CircleRenderer with specified attributes.
     *
     * @param pos Position of the circle's center.
     * @param rad Radius of the circle.
     * @param col Color of the circle.
     * @param segments Number of segments for the circle approximation.
     */
    void initializeCircleAttributes(cv::Point2f pos, float rad, cv::Scalar col, unsigned int segments);

    /**
     * @brief Sets the position of the circle.
     * @param pos New position of the circle's center.
     */
    void setPosition(cv::Point2f pos);

    /**
     * @brief Sets the radius of the circle.
     * @param rad New radius of the circle.
     */
    void setRadius(float rad);

    /**
     * @brief Sets the rotation angle of the circle.
     * @param angle New rotation angle in degrees.
     */
    void setRotationAngle(float angle);

    /**
     * @brief Sets the scaling factors of the circle.
     * @param scaling_factors New scaling factors for the x and y axes.
     */
    void setScaling(cv::Point2f scaling_factors);

    /**
     * @brief Sets the color of the circle.
     * @param col New color of the circle.
     */
    void setColor(cv::Scalar col);

    /**
     * @brief Recomputes the circle parameters and updates the OpenGL buffer.
     */
    void recomputeParameters();

    /**
     * @brief Draws the circle using the stored shader program and uniforms.
     */
    void draw();

    /**
     * @brief Compiles, links shaders, and gets uniform locations.
     * @param aspect_ratio The aspect ratio to be set for the shader.
     * @return Integer status code  [0:successful, -1:error].
     *
     * @details
     * This static method compiles the vertex and fragment shaders,
     * links them into a shader program, and retrieves the uniform
     * locations. It should be called once during initialization.
     */
    static int CompileAndLinkCircleShaders(float aspect_ratio);

    /**
     * @brief Cleans up shader objects and other shared resources.
     * @return Integer status code  [0:successful, -1:error].
     *
     * @details
     * This function should be called when the application is terminating
     * or when you're sure that all instances of CircleRenderer are done
     * with the shader program and related resources.
     */
    static int CleanupClassResources();

    /**
     * @brief Sets up the shader for drawing.
     *
     * @details
     * This method should be called before drawing each frame.
     * It sets the current shader program and updates the uniform
     * variables such as aspect ratio.
     */
    static void SetupShader();

    /**
     * @brief Unsets the shader after drawing.
     *
     * @details
     * This method should be called after drawing each frame.
     */
    static void UnsetShader();

private:
    /**
     * @brief Private helper methods to check shader compilation.
     *
     * @param shader The shader to check.
     * @param shader_type The type of shader to check.
     */
    static bool _CheckShaderCompilation(GLuint shader, const std::string &shader_type);

    /**
     * @brief Private helper methods to check shader linking.
     *
     * @param program The shader program to check.
     */
    static bool _CheckProgramLinking(GLuint program);

    /**
     * @brief Sets up the OpenGL Vertex Array Object and Vertex Buffer Object.
     */
    void _setupRenderBuffers();

    /**
     * @brief Converts an OpenCV Mat to an array suitable for OpenGL transformations.
     * @param mat The OpenCV Mat to convert.
     * @return An array of floats representing the matrix data.
     */
    std::array<float, 16> _cvMatToGlArray(const cv::Mat &mat);

    /**
     * @brief Computes the transformation matrix based on the circle's parameters.
     */
    void _computeTransformation();

    /**
     * @brief Computes the vertices for the circle approximation.
     * @param position Position of the circle's center.
     * @param radius Radius of the circle.
     * @param circSegments Number of segments for the circle approximation.
     * @param circVertices Reference to the vertex array to store the computed vertices.
     */
    void _computeVertices(cv::Point2f position, float radius, unsigned int circSegments, std::vector<float> &circVertices);
};

#endif // CIRCLE_RENDERER_H

// ================================================== VARIABLES ==================================================

/**
 * @brief Vertex data for rendering the textured rectangle.
 *
 * The array contains 4 vertices, each with 4 floats. The first two floats
 * represent the vertex coordinates, and the last two represent the texture
 * coordinates. The vertices are defined in normalized device coordinates (NDC).
 *
 * @details
 * | Index  | Description      | NDC X  | NDC Y  | Tex X  | Tex Y  |
 * |--------|------------------|--------|--------|--------|--------|
 * | 1      | Top-left vertex  | -1.0f  |  1.0f  |  0.0f  |  0.0f  |
 * | 0      | Top-right vertex |  1.0f  |  1.0f  |  1.0f  |  0.0f  |
 * | 2      | Bottom-right     |  1.0f  | -1.0f  |  1.0f  |  1.0f  |
 * | 3      | Bottom-left      | -1.0f  | -1.0f  |  0.0f  |  1.0f  |
 *
 * The texture coordinates are flipped vertically to align with OpenCV's top-left origin.
 */
float WALL_GL_VERTICES[] = {
    -1.0f, 1.0f, 0.0f, 0.0f, // Top-left
    1.0f, 1.0f, 1.0f, 0.0f,  // Top-right
    1.0f, -1.0f, 1.0f, 1.0f, // Bottom-right
    -1.0f, -1.0f, 0.0f, 1.0f // Bottom-left
};

/**
 * @brief Index data for rendering the textured rectangle using triangles.
 *
 * @details
 *
 *   Vertices        Triangles
 *   0-----1          0-----1
 *   |     |          | \   |
 *   |     |  ====>   |  \  |
 *   |     |          |   \ |
 *   3-----2          3-----2
 *
 * This array uses index buffering to specify which vertices from the `vertices`
 * array form each of the two triangles that make up the rectangle. This technique
 * allows for the re-use of vertices, thus reducing the amount of data sent to the GPU.
 */
unsigned int WALL_GL_INDICES[] = {
    0, 1, 2, // First Triangle
    0, 2, 3  // Second Triangle
};

/**
 * @brief Vertex shader source code for wall images.
 *
 * @details
 * This is a GLSL (OpenGL Shading Language) vertex shader source code stored as a C++ raw string literal.
 * - `#version 330 core`: Specifies that the GLSL version is 3.30 and we're using the core profile.
 * - `in vec2 position;`: Declares an input vertex attribute called `position`. Receives vertex coordinates from the application.
 * - `in vec2 texcoord;`: Declares another input vertex attribute called `texcoord`. Receives texture coordinates from the application.
 * - `out vec2 Texcoord;`: Declares an output variable that will be passed to the fragment shader.
 * - `void main() { ... }`: Main function where the vertex shader performs its work.
 */
const char *wallVertexSource = R"glsl(
    #version 330 core
    in vec2 position;
    in vec2 texcoord;
    out vec2 Texcoord;

    void main() {
        Texcoord = texcoord;
        gl_Position = vec4(position, 0.0, 1.0);
    }
)glsl";

/**
 * @brief Fragment shader source code for wall images.
 *
 * @details
 * This is a GLSL (OpenGL Shading Language) fragment shader source code also stored as a C++ raw string literal.
 * - `#version 330 core`: Specifies that the GLSL version is 3.30 and we're using the core profile.
 * - `in vec2 Texcoord;`: Receives the texture coordinates from the vertex shader.
 * - `out vec4 outColor;`: Declares an output variable for storing the color to be used for the fragment.
 * - `uniform sampler2D tex;`: Declares a uniform variable representing a 2D texture.
 * - `void main() { ... }`: Main function of the fragment shader, samples the texture at the given coordinates and sets the output color.
 */
const char *wallFragmentSource = R"glsl(
    #version 330 core
    in vec2 Texcoord;
    out vec4 outColor;
    uniform sampler2D tex;

    void main() {
        outColor = texture(tex, Texcoord);
    }
)glsl";

// Get top-level package path
extern const std::string package_path = ros::package::getPath("projection_operation");
extern const std::string workspace_path = package_path.substr(0, package_path.rfind("/src"));

// Directory paths for configuration files
extern const std::string CONFIG_DIR_PATH = workspace_path + "/data/projection/params";

// Directory paths for configuration images
extern const std::string IMAGE_TOP_DIR_PATH = workspace_path + "/data/projection/images";

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

// Global variable to set the OpenGL debug level.
int DEBUG_LEVEL_GL = 3; // [0: None, 1: >=Default 2: >=Low, 3: >=Medium, 4: High]

// ================================================== FUNCTIONS ==================================================

int initWallRenderObjects(MazeRenderContext &out_renCtx,
                          float *vertices, size_t verticesSize,
                          unsigned int *indices, size_t indicesSize)
{
    int status = 0;

    // Generate and bind an Element Buffer Object (EBO)
    glGenBuffers(1, &out_renCtx.ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, out_renCtx.ebo);
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    // Initialize the EBO with index data
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indicesSize, indices, GL_DYNAMIC_DRAW);
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    // Generate and bind a Vertex Array Object (VAO)
    glGenVertexArrays(1, &out_renCtx.vao);
    glBindVertexArray(out_renCtx.vao);
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    // Generate and bind a Vertex Buffer Object (VBO)
    glGenBuffers(1, &out_renCtx.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, out_renCtx.vbo);
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    // Initialize the VBO with vertex data
    glBufferData(GL_ARRAY_BUFFER, verticesSize, vertices, GL_STATIC_DRAW);
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    // Specify the format of the vertex data for the position attribute
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0); // Enable the position attribute
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    // Specify the format of the vertex data for the texture coordinate attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));
    glEnableVertexAttribArray(1); // Enable the texture coordinate attribute
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    // Unbind the VAO to prevent accidental modification
    glBindVertexArray(0);
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    // Return GL status
    return status;
}

/**
 * @brief Formats the file name for the XML file based on the active calibration mode and monitor.
 *
 * Format:
 * - `cfg_m<number>_c<number>_<data_type>.xml`
 *
 * @param d_type Type of data being saved  [0: homography matrix, 1: for control points].
 * @param mon_id_ind Index of the active or desired monitor.
 * @param mode_cal_ind Index of the active or desired calibration mode.
 * @param config_dir_path Path to the directory where the XML file will be loaded/saved.
 *
 */
std::string frmtFilePathxml(int, int, int, std::string);

/**
 * @brief Prompts the user for a single digit input or an option to quit.
 *
 * @return A std::string containing the single digit entered by the user.
 *         If the user enters 'q' or 'Q', an empty string is returned.
 *
 * @note To handle a cancellation, the calling code should check if the
 *       returned string is empty.
 *
 * Example:
 * @code
 * std::string digit = promptForSingleDigitOrQuit();
 * if (digit.empty()) {
 *     // Handle the cancellation
 * } else {
 *     // Proceed with the operation using the digit
 * }
 * @endcode
 */
std::string promptForProjectorNumber();

/**
 * @brief Saves the homography matrix array to an XML file.
 *
 * @param full_path Path to the XML file.
 * @param _HMAT_GRID_ARR Reference to the 3x3 array of Homography matrices to be saved.
 *
 * @return Integer status code [0:successful, -1:error].
 *
 * @details
 * This function uses the pugixml library to create an XML document and populate it with
 * the homography matrix for each wall in a 3x3 grid.
 */
int saveHMATxml(std::string, const std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> &);

/**
 * @brief Loads the homography matrix array from an XML file.
 *
 * @param full_path Path to the XML file.
 * @param[out] out_HMAT_GRID_ARR Reference to 3x3 array of Homography matrices to be initialized with loaded values.
 *
 * @return Integer status code [0:successful, -1:error].
 *
 * @note Uses pugiXML for XML parsing.
 */
int loadHMATxml(std::string, std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> &);

/**
 * @brief Saves the control point array to an XML file.
 *
 * @param full_path The full path to the XML file.
 * @param _CP_GRID_ARR The control point array to save.
 * @return int Returns 0 on success, -1 on failure.
 */
int saveCPxml(const std::string &full_path, const std::array<std::array<cv::Point2f, 4>, 4> &_CP_GRID_ARR);

/**
 * @brief Load the control point array from the XML file.
 *
 * @param full_path The full path to the XML file.
 * @param[out] Reference to the control point array to be initialized with loaded values.
 *
 * @return int Returns 0 on success, -1 on failure.
 */
int loadCPxml(const std::string &full_path, std::array<std::array<cv::Point2f, 4>, 4> &out_CP_GRID_ARR);

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
 * @param _CP_GRID_ARR The control point coordinates used to warp the wall image.
 * @param[out] out_HMAT_GRID_ARR updated 3x3 array of Homography matrices used to warp the wall image.
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