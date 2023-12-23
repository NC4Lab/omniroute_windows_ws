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
#include <tuple>

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
    GLuint textureID;          // Texture for the wall
    GLFWwindow *windowID;      // The window associated with this context
    GLFWmonitor *monitorID;    // The monitor associated with this context
    int windowInd;             // Enum of type CalibrationMode for the window associated with this context
    int monitorInd;            // Enum of type CalibrationMode for the monitor associated with this context
    bool isContextInitialized; // Flag indicating whether there context has been initialized
    bool isFullScreen;         // Flag indicating whether the window is in full screen mode
private:
    int _windowWidthPxl;              // Width of the window
    int _windowHeightPxl;             // Height of the window
    GLuint _shaderProgram;            // Shader program for wall rendering
    GLuint _vao;                      // Vertex Array Object
    GLuint _vbo;                      // Vertex Buffer Object
    GLuint _ebo;                      // Element Buffer Object
    static GLFWmonitor **_PP_Monitor; // Pointer to the pointer to the GLFW monitors
    static int _NumMonitors;          // Number of monitors connected to the system

public:
    /**
     * @brief Constructor to initialize context members.
     *
     * Initializes all member variables to default states,
     * deferring the setup until resources are available.
     */
    MazeRenderContext::MazeRenderContext();

    /**
     * @brief Destructor to clean up resources.
     *
     * Cleans up OpenGL and GLFW resources if the context
     * has been initialized to ensure proper resource management.
     */
    MazeRenderContext::~MazeRenderContext();

    /**
     * @brief Deleted copy constructor.
     *
     * Copy constructor is deleted to avoid unintentional copying,
     * ensuring unique ownership of context resources.
     */
    MazeRenderContext(const MazeRenderContext &) = delete;

    /**
     * @brief Deleted copy assignment operator.
     *
     * Copy assignment is deleted to avoid unintentional copying,
     * ensuring unique ownership of context resources.
     */
    MazeRenderContext &operator=(const MazeRenderContext &) = delete;

    /**
     * @brief Move constructor for resource transfer.
     *
     * Transfers ownership of context resources from another instance
     * and resets that instance to prevent double deletion.
     */
    MazeRenderContext(MazeRenderContext &&other) noexcept;

    /**
     * @brief Move assignment operator for resource transfer.
     *
     * Cleans up the current resources and transfers ownership
     * of context resources from another instance.
     */
    MazeRenderContext &operator=(MazeRenderContext &&other) noexcept;

    /**
     * @brief Callback function for handling framebuffer size changes.
     *
     * This function is called whenever the framebuffer size changes,
     * and it updates the OpenGL viewport to match the new dimensions.
     *
     * @param window Pointer to the GLFW window.
     * @param width The new width of the framebuffer.
     * @param height The new height of the framebuffer.
     */
    static void CallbackFrameBufferSizeGLFW(
        GLFWwindow *window,
        int width,
        int height);

    /**
     * @brief Callback function for handling OpenGL errors.
     *
     * This function is called whenever an error occurs in the OpenGL context.
     * It logs the error message using ROS_ERROR.
     *
     * @param source The source of the error.
     * @param type The type of the error.
     * @param id The error ID.
     * @param severity The severity of the error.
     * @param length The length of the error message.
     * @param message The error message.
     * @param userParam User-defined parameter.
     */
    static void CallbackDebugOpenGL(
        GLenum source,
        GLenum type,
        GLuint id,
        GLenum severity,
        GLsizei length,
        const GLchar *message,
        const void *userParam);

    /**
     * @brief Callback function for handling errors.
     *
     * This function is called whenever an error occurs in the GLFW context.
     * It logs the error message using ROS_ERROR.
     *
     * @param error The error code.
     * @param description The error description.
     */
    static void CallbackErrorGLFW(
        int error,
        const char *description);

    /**
     * @brief Checks for OpenGL errors and logs them.
     * Should be called after OpenGL API calls.
     *
     * @param line Line number where the function is called.
     * @param file_str File name where the function is called.
     * @param msg_str Optional message to provide additional context (default to nullptr).
     * @return Integer status code [-1:error, 0:successful].
     *
     * @example CheckErrorOpenGL(__LINE__, __FILE__);
     */
    static int CheckErrorOpenGL(
        int line,
        const char *file_str,
        const char *msg_str = nullptr);

    /**
     * @brief Checks for GLFW errors and logs them.
     * Should be called after GLFW API calls.
     *
     * @param line Line number where the function is called.
     * @param file_str File name where the function is called.
     * @param msg_str Optional message to provide additional context (default to nullptr).
     * @return Integer status code [-1:error, 0:successful].
     *
     * @example CheckErrorGLFW(__LINE__, __FILE__);
     */
    static int CheckErrorGLFW(
        int line,
        const char *file_str,
        const char *msg_str = nullptr);

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
     * @brief Compiles and links shaders for a given class instance.
     *
     * @param vertex_source Source code for the vertex shader stored as a C++ raw string literal.
     * @param fragment_source Source code for the fragment shader stored as a C++ raw string literal.
     * @return Integer status code [-1:error, 0:successful].
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
    int compileAndLinkShaders(
        const GLchar *vertex_source,
        const GLchar *fragment_source);

    /**
     * @brief This method checks the validation status of the shader program
     * and logs any errors if the validation fails.
     *
     * Make sure to call this method after your shader program has been linked.
     *
     * @return Integer status code [-1:error, 0:successful].
     */
    int MazeRenderContext::checkShaderProgram();

    /**
     * @brief Initializes a new rendering context.
     *
     * Checks and sets a new monitor based on the monitor index, creates a new GLFW window,
     * and sets up the OpenGL context with necessary callbacks and extensions.
     *
     * @param win_ind Enum of type CalibrationMode for the window to initialize.
     * @param mon_ind Enum of type CalibrationMode for the monitor to use for the window.
     * @param win_width Width of the window to create.
     * @param win_height Height of the window to create.
     * @param key_callback Optional key callback function to set for the window (default to nullptr).
     * @return int Status of context initialization (0 for success, -1 for failure).
     */
    int initWindowContext(
        int win_ind,
        int mon_ind,
        int win_width,
        int win_height,
        KeyCallbackFunc key_callback = nullptr);

    /**
     * @brief Initialize OpenGL resources for wall image render objects.
     *
     * @param vertices Pointer to the vertex data array for rendering.
     * @param vertices_size Size of the vertex data array.
     * @param indices Pointer to the index data array for rendering.
     * @param indices_size Size of the index data array.
     *
     * @return Integer status code [-1:error, 0:successful].
     *
     * @details
     * Initializes the Vertex Array Object (VAO), Vertex Buffer Object (VBO) and Element Buffer Object (EBO).
     */
    int initRenderObjects(
        float *vertices,
        size_t vertices_size,
        unsigned int *indices,
        size_t indices_size);

    /**
     * @brief Sets up the system for a new rendering.
     *
     * Makes this the current GL context and clears the color
     * buffers of the back buffer for a new frame.
     *
     * @return Integer status code [-1:error, 0:successful].
     */
    int initWindowForDrawing();

    /**
     * @brief Converts an OpenCV Mat image into an OpenGL texture for this context.
     *
     * @param img_mat The cv::Mat image that needs to be converted.
     *
     * @return Integer status code [-1:error, 0:successful].
     *
     * @details
     * This function takes an OpenCV Mat image as input and converts it into an OpenGL texture.
     * The OpenCV image is first converted from BGR to RGB format. Then, a new OpenGL texture is
     * generated and the converted image data is stored in this texture.
     *
     * @note This function assumes that the input image is of type CV_8UC3 and has no alpha channel.
     */
    int MazeRenderContext::loadMatTexture(cv::Mat img_mat);

    /**
     * @brief Renders the current texture stored in a given class instance.
     *
     * @return Integer status code [-1:error, 0:successful].
     */
    int MazeRenderContext::drawTexture();

    /**
     * @brief Swap and poll the buffer.
     *
     * Basically a wrapper for glfwSwapBuffers() and glfwPollEvents() which
     * swaps the front and back buffers of the window and processes events.
     *
     * @return Integer status code [-1:error, 0:successful].
     */
    int bufferSwapPoll();

    /**
     * @brief Changes the display mode and monitor of the application window.
     *
     * This function switches the application window between full-screen and windowed modes
     * and moves it to the monitor specified by the global variable imgMonNumInd.
     *
     * @param mon_ind Enum of type CalibrationMode for the monitor to move the window to.
     * @param is_fullscreen Boolean flag indicating whether the window should be set to full-screen mode.
     * @param offset_xy Optional offset to apply to window position (default to (0, 0)).
     *
     * @return Integer status code [-1:error, 0:successful].
     */
    int changeWindowDisplayMode(
        int mon_ind,
        bool is_fullscreen,
        cv::Point offset_xy = cv::Point(0.0f, 0.0f));

    /**
     * @brief Sets the GLFW window to always be on top if it is in fullscreen mode.
     *
     * @note This needs to be called continually in a main loop.
     *
     * @return Integer status code [-1:error, 0:successful].
     */
    int forceWindowStackOrder();

    /**
     * @brief Function to set the background color and redraw the window
     *
     * @param color The color to set the background to.
     * @param duration The duration to flash the background for (ms).
     *
     * @return Integer status code [-1:error, 0:successful].
     *
     * @example flashBackgroundColor(window, cv::Scalar(0.1f, 0.0f, 0.0f), 0.5f);
     */
    int flashBackgroundColor(
        const cv::Scalar &color,
        int duration);

    /**
     * @brief Check if the user has requested to exit the program.
     *
     * This function checks if the user has pressed the escape key
     * or if the window has been closed.
     *
     * @return Integer status code  [-1:error, 0:no change, 1:window closed, 2:esc key pressed].
     */
    int checkExitRequest();

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
    int cleanupContext(bool log_errors);

private:
    /**
     * @brief Helper function for resetting members.
     *
     * This is useful for both the move constructor and the move assignment operator.
     */
    void _resetMembers();

    /**
     * @brief Private helper methods to check shader compilation.
     *
     * @param __shaderProgram The shader to check.
     * @param shader_type The type of shader to check.
     *
     * @return Integer status code [-1:error, 0:successful].
     */
    int _checkShaderCompilation(
        GLuint __shaderProgram,
        const std::string &shader_type);

    /**
     * @brief Private helper methods to check shader linking.
     * @param __shaderProgram The shader program to check.
     *
     * @return Integer status code [-1:error, 0:successful].
     */
    int _checkProgramLinking(GLuint __shaderProgram);

    /**
     * @brief  Private method to check monitor index and id is valid.
     *
     * @param mon_ind Index of monitor ID to check.
     * @param out_monitorInd Refernce to global just to make it clear the value is being set.
     * @param out_monitorID Refernce to global just to make it clear the value is being set.
     *
     * @return Integer status code [-1:error, 0:successful].
     */
    int _setMonitor(
        int mon_ind,
        GLFWmonitor *&out_monitorID,
        int &out_monitorInd);

    /**
     * @brief  Simple test of OpenGL and GLFW callbacks.
     *
     * @return Integer status code [-1:error, 0:successful].
     */
    int _testCallbacks();
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
    int circID;                      // Enum of type CalibrationMode for the circle, used for identification.
    std::vector<float> circVertices; // Vertex data for the circle's geometry.
    cv::Point2f circPosition;        // Position of the circle in 2D space.
    float cirRadius;                 // Radius of the circle.
    cv::Scalar circColor;            // Color of the circle.
    unsigned int circSegments;       // Number of segments used to approximate the circle geometry.
    cv::Mat circHomMatNDC;           // Homography matrix to convert position from cm to NDC space (default to identity).

private:
    static GLuint _ShaderProgram;                // Shader program for rendering
    GLuint _vao;                                 // Vertex Array Object for the circle.
    GLuint _vbo;                                 // Vertex Buffer Object for the circle's vertices.
    cv::Mat _transformationMatrix;               // Transformation matrix for the circle's vertices.
    static int _CircCnt;                         // Static index counter for the CircleRenderer class objects
    static GLint _ColorLocation;                 // Location of color uniform in shader
    static GLint _TransformLocation;             // Location of transform uniform in shader
    static GLint _AspectRatioLocation;           // Location of aspect ratio uniform in shader
    static float _AspectRatioUniform;            // Aspect ratio uniform for the shader program
    static int CircleRenderer::_WindowWidthPxl;  // Width of the window in pixels
    static int CircleRenderer::_WindowHeightPxl; // Height of the window in pixels
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
    static constexpr const char *_circVertexShaderSource = R"glsl(
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
    static constexpr const char *_circFragmentShaderSource = R"glsl(
        #version 330 core
        out vec4 FragColor;
        uniform vec4 color;

        void main() {
            FragColor = color;
        }
    )glsl";

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
     * Sets up the OpenGL Vertex Array Object and Vertex Buffer Object.
     *
     * @param _circPosition Position of the circle's center.
     * @param _cirRadius Radius of the circle.
     * @param _circColor Color of the circle.
     * @param _circSegments Number of segments for the circle approximation.
     * @param _circHomMatNDC Optional homography matrix to convert position from cm to NDC space (default to identity).
     */
    int initializeCircleObject(
        cv::Point2f _circPosition,
        float _cirRadius,
        cv::Scalar _circColor,
        unsigned int _circSegments,
        cv::Mat _circHomMatNDC = cv::Mat::eye(3, 3, CV_64F));

    /**
     * @brief Compiles, links shaders, and gets uniform locations.
     *
     * @details
     * This static method compiles the vertex and fragment shaders,
     * links them into a shader program, and retrieves the uniform
     * locations. It should be called once during initialization.
     *
     * @param __AspectRatioUniform Optional aspect ratio uniform for the shader program.
     *
     * @return Integer status code [-1:error, 0:successful].
     */
    static int CompileAndLinkCircleShaders(float __AspectRatioUniform);

    /**
     * @brief Sets up the shader for drawing.
     *
     * @details
     * This method should be called before drawing each frame.
     * It sets the current shader program and updates the uniform
     * variables such as aspect ratio.
     *
     * @return Integer status code [-1:error, 0:successful].
     */
    static int SetupShader();

    /**
     * @brief Unsets the shader after drawing.
     *
     * @details
     * This method should be called after drawing each frame.
     *
     * @return Integer status code [-1:error, 0:successful].
     */
    static int UnsetShader();

    /**
     * @brief Sets the position of the circle.
     *
     * @param pos New position of the circle's center.
     */
    void setPosition(cv::Point2f _circPosition);

    /**
     * @brief Sets the radius of the circle.
     *
     * @param _cirRadius New radius of the circle.
     */
    void setRadius(float _cirRadius);

    /**
     * @brief Sets the color of the circle.
     *
     * @param _circColor New color of the circle.
     */
    void setColor(cv::Scalar _circColor);

    /**
     * @brief Recomputes the circle parameters and updates the OpenGL buffer.
     *
     * @param do_coord_warp Option to convert circle coordinates based on circHomMatNDC.
     *
     * @return Integer status code [-1:error, 0:successful].
     */
    int updateCircleObject(bool do_coord_warp = false);

    /**
     * @brief Draws the circle using the stored shader program and uniforms.
     *
     * @return Integer status code [-1:error, 0:successful].
     */
    int draw();

    /**
     * @brief Cleans up shader objects and other shared resources.
     *
     * @details
     * This function should be called when the application is terminating
     * or when you're sure that all instances of CircleRenderer are done
     * with the shader program and related resources.
     *
     * @return Integer status code [-1:error, 0:successful].
     */
    static int CleanupClassResources();

private:
    /**
     * @brief Private helper methods to check shader compilation.
     *
     * @param ___ShaderProgram The shader to check.
     * @param shader_type The type of shader to check.
     *
     * @return Integer status code [-1:error, 0:successful].
     */
    static int _CheckShaderCompilation(
        GLuint ___ShaderProgram,
        const std::string &shader_type);

    /**
     * @brief Private helper methods to check shader linking.
     *
     * @param ___ShaderProgram The shader program to check.
     *
     * @return Integer status code [-1:error, 0:successful].
     */
    static int _CheckProgramLinking(GLuint ___ShaderProgram);

    /**
     * @brief Converts an OpenCV Mat to an array suitable for OpenGL transformations.
     *
     * @param[out] out_transformationMatrix The OpenCV Mat to convert.
     *
     * @return An array of floats representing the matrix data.
     */
    std::array<float, 16> _cvMatToGlArray(const cv::Mat &out_transformationMatrix);

    /**
     * @brief Computes the vertices for the circle approximation.
     *
     * @param[out] out_circVertices Reference to the vertex array to store the computed vertices.
     */
    void _computeVertices(std::vector<float> &out_circVertices);

    /**
     * @brief Warp/transforms circle coordinates using the stored homography matrix.
     *
     * This method takes a vector of circle vertices (as float values representing
     * x and y coordinates) and transforms them using the provided homography matrix.
     * The transformed coordinates are then stored back into the same vector.
     * The method assumes the input vector contains an even number of elements,
     * representing pairs of (x, y) coordinates.
     *
     * @note Assumes out_circVertices is a flat list of coordinates: [x1, y1, x2, y2, ..., xn, yn]
     *
     * @param[out] out_circVertices The vector of circle vertices in CM, to be transformed.
     */
    void _convertToNDC(std::vector<float> &out_circVertices);
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
float QUAD_GL_VERTICES[] = {
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
unsigned int QUAD_GL_INDICES[] = {
    0, 1, 2, // First Triangle
    0, 2, 3  // Second Triangle
};

/**
 * @brief Vertex shader source code for rendered images.
 *
 * @details
 * This is a GLSL (OpenGL Shading Language) vertex shader source code stored as a C++ raw string literal.
 * - `#version 330 core`: Specifies that the GLSL version is 3.30 and we're using the core profile.
 * - `in vec2 position;`: Declares an input vertex attribute called `position`. Receives vertex coordinates from the application.
 * - `in vec2 texcoord;`: Declares another input vertex attribute called `texcoord`. Receives texture coordinates from the application.
 * - `out vec2 Texcoord;`: Declares an output variable that will be passed to the fragment shader.
 * - `void main() { ... }`: Main function where the vertex shader performs its work.
 */
const char *QUAD_GL_VERTEX_SOURCE = R"glsl(
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
 * @brief Fragment shader source code for rendered images.
 *
 * @details
 * This is a GLSL (OpenGL Shading Language) fragment shader source code also stored as a C++ raw string literal.
 * - `#version 330 core`: Specifies that the GLSL version is 3.30 and we're using the core profile.
 * - `in vec2 Texcoord;`: Receives the texture coordinates from the vertex shader.
 * - `out vec4 outColor;`: Declares an output variable for storing the color to be used for the fragment.
 * - `uniform sampler2D tex;`: Declares a uniform variable representing a 2D texture.
 * - `void main() { ... }`: Main function of the fragment shader, samples the texture at the given coordinates and sets the output color.
 */
const char *QUAD_GL_FRAGMENT_SOURCE = R"glsl(
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
 * @todo: Change these to std::array
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
int WALL_IMG_PROJ_MAP_TEMPLATE[4][3][3][3] = {
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

// // Actual hardcoded wall image indices used to display
// /// @note Used for RTI grant 2023
// int WALL_IMG_PROJ_MAP[4][3][3][3] = {
//     // Projector 0: East
//     {
//         // Chamber Row: Top, Column: Left, Center, Right
//         {{0, 0, 0}, {1, 1, 1}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
//         // Chamber Row: Middle
//         {{0, 0, 0}, {0, 0, 4}, {0, 2, 2}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
//         // Chamber Row: Bottom
//         {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
//     },
//     // Projector 1: North
//     {
//         // Chamber Row: Top, Column: Left, Center, Right
//         {{0, 0, 0}, {0, 2, 2}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
//         // Chamber Row: Middle
//         {{0, 1, 1}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
//         // Chamber Row: Bottom
//         {{0, 0, 0}, {0, 0, 0}, {0, 3, 3}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
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

// Actual hardcoded wall image indices used to display
int WALL_IMG_PROJ_MAP[4][3][3][3] = {
    // Projector 0: East
    {
        // Chamber Row: Top, Column: Left, Center, Right
        {{1, 2, 3}, {0, 0, 0}, {3, 2, 1}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Middle
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Bottom
        {{3, 4, 5}, {0, 0, 0}, {5, 4, 3}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
    },
    // Projector 1: North
    {
        // Chamber Row: Top, Column: Left, Center, Right
        {{0, 0, 0}, {1, 2, 3}, {0, 0, 0}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
        // Chamber Row: Middle
        {{1, 2, 3}, {4, 4, 4}, {3, 2, 1}}, // {Calibration Mode: Left, Center, Right}, {...}, {...}}
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

// Template of 1D array for hardcoded floor image indices to display
int MAZE_IMG_PROJ_MAP[4] = {
    1, // Projector 0: East
    1, // Projector 1: North
    1, // Projector 2: West
    1, // Projector 3: South
};

// Enum for tracking the current calibration mode
enum CalibrationMode
{
    WALLS_LEFT = 0,
    WALLS_MIDDLE = 1,
    WALLS_RIGHT = 2,
    FLOOR = 3,
    N_CAL_MODES
};
CalibrationMode CAL_MODE = WALLS_MIDDLE;

// Calibration mode strings
std::vector<std::string> CAL_MADE_STR_VEC = {"cwl", "cwm", "cwr", "cmf"};

// Number of rows and columns in the maze
extern const int MAZE_SIZE = 3;

// Sprecify window resolution: 4K resolution (3840x2160)
extern const int WINDOW_WIDTH_PXL = 3840;
extern const int WINDOW_HEIGHT_PXL = 2160;

// Specify window aspect ratio
extern const float WINDOW_AP = (float)WINDOW_WIDTH_PXL / WINDOW_HEIGHT_PXL;

// Maze width and height (pixels)
const int MAZE_IMAGE_WIDTH_PXL = 1800;
const int MAZE_IMAGE_HEIGHT_PXL = 1800;

// Maze width and height (NDC)
const float MAZE_WIDTH_NDC = 0.3f;
const float MAZE_HEIGHT_NDC = MAZE_WIDTH_NDC * WINDOW_AP;

// Maze width and height (cm)
const float MAZE_WIDTH_HEIGHT_CM = 90.0f;

// Wall image size (pixels)
extern const int WALL_IMAGE_WIDTH_PXL = 300;
extern const int WALL_IMAGE_HEIGHT_PXL = 540;

// Default wall width and height (NDC)
extern const float WALL_IMAGE_WIDTH_NDC = (MAZE_WIDTH_NDC / (float(MAZE_SIZE) - 1)) / (1 + std::sqrt(2));   // Wall width based on octogonal geometry in NDC
extern const float WALL_IMAGE_HEIGHT_NDC = (MAZE_HEIGHT_NDC / (float(MAZE_SIZE) - 1)) / (1 + std::sqrt(2)); // Wall height based on octogonal geometry in NDC

// Global variable to set the OpenGL debug level.
const int DEBUG_LEVEL_GL = 2; // [0: None, 1: >=Default 2: >=Low, 3: >=Medium, 4: High]

// Pi
static constexpr float PI = 3.14159265358979323846f;

// ================================================== FUNCTIONS ==================================================

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
bool dbRunDT(int dt_wait);

/**
 * @brief Track and print the elapsed time between calls with line and function info.
 *
 * @details
 * Call this function with `do_reset` set to true to start timing,
 * and call it again with `do_reset` set to false to print the elapsed time.
 *
 * @param do_reset If true, resets the start time. If false, prints the elapsed time.
 * @param line Line number where the function is called.
 * @param file_path File path where the function is called.
 *
 * @example dbTraceCalls(true, __LINE__, __FILE__);
 */
void dbTraceCalls(
    bool do_reset = false,
    int line = 0,
    const char *file_path = nullptr);

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
void dbLogQuadVertices(const std::vector<cv::Point2f> &quad_vertices);

/**
 * @brief Prints the coordinates of all entries in the control point array.
 */
void dbLogCtrlPointCoordinates(const std::array<std::array<cv::Point2f, 4>, 4> &);

/**
 * @brief Prints the coordinates of all entries in the warped wall vertices array.
 *
 * @param _WALL_WARP_COORDS Reference to the warped wall vertices array.
 */
void dbLogWallVerticesCoordinates(const std::array<std::array<std::array<cv::Point2f, 4>, MAZE_SIZE>, MAZE_SIZE> &_WALL_WARP_COORDS);

/**
 * @brief Prints the coordinates of all entries in the homography matrix.
 *
 * @param _HMAT The homography matrix.
 */
void dbLogHomMat(const cv::Mat &_HMAT);

/**
 * @brief Displays a warped image in a window.
 *
 * @details
 * This function creates a window with the name "Warped Image Display" and
 * displays the given image. The function waits for a key press before
 * closing the window and returning.
 *
 * @param img_mat The image matrix to display.
 */
void dbDispImgMat(const cv::Mat &img_mat);

/**
 * @brief Prompts the user for a single digit input or an option to quit.
 *
 * @return A int containing the single digit entered by the user.
 *         If the user enters 'q' or 'Q', an -1 is returned.
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
int promptForProjectorNumber();

/**
 * @brief Formats the file name for the XML file for homography matrices.
 *
 * @details
 * Format:
 * - `hmats_m<number>.xml`
 * - `hmats_m0.xml`
 *
 * @param proj_ind Index/number of the projector to load data for.
 * @param[out] out_path Reference to string that will store the path to the XML file.
 */
void xmlFrmtFileStringsHmat(
    int proj_ind,
    std::string &out_path);

/**
 * @brief Formats the file name for the XML file for maze vertices matrices.
 *
 * @details
 * Format:
 * - `maze_vertices.xml`
 *
 * @param[out] out_path Reference to string that will store the path to the XML file.
 */
void xmlFrmtFileStringsVertices(std::string &out_path);

/**
 * @brief Save a single cv::Mat homography matrix to an XML file.
 *
 * @details
 * This function uses the pugixml library to create an XML document and populate it with
 * the homography matrix for each wall in a 3x3 grid.
 *
 * @param _H The homography matrix to save.
 * @param proj_ind Index/number of the projector to load data for.
 * @param _CAL_MODE Enum of type CalibrationMode for the active or desired calibration mode.
 * @param grid_row Row index for the array of homography matrices to save.
 * @param grid_col Column index for the array of homography matrices save.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int xmlSaveHMAT(const cv::Mat &_H,
                int proj_ind,
                CalibrationMode _CAL_MODE,
                int grid_row,
                int grid_col);

/**
 * @brief Load a single cv::Mat homography matrix from an XML file.
 *
 * @note Input proj_ind should be -1 if projector index needs to be inputed manually.
 *
 * @param proj_ind Index/number of the projector to load data for.
 * @param _CAL_MODE Enum of type CalibrationMode for the active or desired calibration mode.
 * @param grid_row Row index for the array of homography matrices to load.
 * @param grid_col Column index for the array of homography matrices to load.
 * @param[out] out_H The output homography matrix.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int xmlLoadHMAT(
    int proj_ind,
    CalibrationMode _CAL_MODE,
    int grid_row,
    int grid_col,
    cv::Mat &out_H);

/**
 * @brief Save a vector of four vertices (cv::Point2f) to an XML file.
 *
 * @param quad_vertices_ndc Vector of cv::Point2f representing the vertices.
 * @param proj_ind Index/number of the projector to load data for.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int xmlSaveVertices(const std::vector<cv::Point2f> &quad_vertices_ndc, int proj_ind);

/**
 * @brief Load a vector of four vertices (cv::Point2f) from an XML file.
 *
 * @param proj_ind Index/number of the projector to load data for.
 * @param[out] out_quad_vertices_ndc Output vector of cv::Point2f for the vertices.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int xmlLoadVertices(int proj_ind, std::vector<cv::Point2f> &out_quad_vertices_ndc);

/**
 * @brief Checks for size and signulararity issues in a homography matrix.
 *
 * @param _H The homography matrix to check.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int checkHMAT(const cv::Mat &_H);

/**
 * @brief Checks if a given set of vertices defines a valid quadrilateral.
 *
 * @param quad_vertices std:vector of the four vertices defining a quadrilateral.
 *
 * @return Integer status code [-2:invalid wrong shape, -1:invalid wrong size, 0:valid].
 */
int checkQuadVertices(const std::vector<cv::Point2f> &quad_vertices);

/**
 * @brief Converts the units of the quadrilateral from NDC to pixels.
 *
 * @details
 * Convert from NDC [-1, 1] to pixel [0, width or height] and
 * inverts the y to match OpenCV's top-left origin
 *
 * @param quad_vertices_ndc The quadrilateral vertices in NDC
 * @param window_width_pxl The width of the window in pixels
 * @param window_height_pxl The height of the window in pixels
 *
 * @return Vector of quadrilateral vertices in pixels
 */
std::vector<cv::Point2f> quadVertNdc2Pxl(
    const std::vector<cv::Point2f> &quad_vertices_ndc,
    int window_width_pxl,
    int window_height_pxl);

/**
 * @brief Converts the units of the quadrilateral from pixels to NDC.
 *
 * @details
 * Converts from pixel coordinates [0, width or height] to NDC [-1, 1]
 * while maintaining the y-axis direction the same as NDC, with origin at the bottom.
 * This assumes that the input pixel coordinates are based on OpenCV's top-left origin.
 *
 * @param quad_vertices_pxl The quadrilateral vertices in pixel coordinates
 * @param window_width_pxl The width of the window in pixels
 * @param window_height_pxl The height of the window in pixels
 *
 * @return Vector of quadrilateral vertices in NDC
 */
std::vector<cv::Point2f> quadVertPxl2Ndc(
    const std::vector<cv::Point2f> &quad_vertices_pxl,
    int window_width_pxl,
    int window_height_pxl);

/**
 * @brief Computes homography matrices for a geven set of vertices.
 *
 * @param source_vertices_pxl std:vector of the four vertices defining the source plane in pixels.
 * @param target_vertices_ndc std:vector of the four vertices defining the target plane in NDC.
 * @param[out] out_H The output homography matrix.
 *
 * @return Integer status code [-1:invalid wrong size, -2:invalid wrong shape, 0:valid].
 */
int computeHomographyMatrix(
    const std::vector<cv::Point2f> &source_vertices_pxl,
    const std::vector<cv::Point2f> &target_vertices_ndc,
    cv::Mat &out_H);

/**
 * @brief Loads PNG images with alpha channel from specified file paths and stores them in a vector as cv::Mat objects.
 *
 * @param img_paths_vec A vector of file paths to the images to be loaded.
 * @param[out] out_img_mat_vec Reference to a vector of cv::Mat where the loaded images will be stored.
 *
 * @return Integer status code [-1:error, 0:successful].
 *
 * @details
 * This function takes a vector of file paths and iteratively loads each PNG image
 * with an alpha channel using OpenCV's imread function. It checks that the image
 * has the correct dimensions and the presence of an alpha channel. The function
 * returns 0 if all images are loaded successfully and -1 if any error occurs.
 *
 * Note on exporting from Adobe Illustrator:
 * - Select the Export As function and select PNG format.
 * - Tick the 'Use Arboard' option and click 'Export'.
 * - Set the 'Resolution' to 'Screen (72 ppi)' or 'Other' and then input '72 PPI'
 * - Set the 'Anti-aliasing' option ', choose 'Art Optimized (Supersampling)'.
 * - Tick the 'Transparency' checkbox in the PNG export options to include the alpha channel.
 * - Confirm the PNG is exported with a bit depth that supports alpha (typically PNG-24).
 */
int loadImgMat(
    const std::vector<std::string> &img_paths_vec,
    std::vector<cv::Mat> &out_img_mat_vec);

/**
 * @brief Merges a mask image over a base image using the alpha channel and stores the result.
 *
 * @param base_img_path Output cv::Mat containing the base image.
 * @param[out] out_base_img cv::Mat containing the base image for merging.
 *
 * @return Integer status code [-1:error, 0:successful].
 *
 * @details
 * This function overlays the mask image on top of the base image using the alpha channel
 * of the mask image. Pixels from the mask image are copied over to the base image based on
 * the alpha value - if the alpha value is not fully transparent (0), the pixel is copied.
 */
int mergeImgMat(
    const cv::Mat &mask_img,
    cv::Mat &out_base_img);

/**
 * @brief Warp OpenCV image using homography matrix.
 *
 * @param img_mat The cv::Mat image that needs to be warped.
 * @param _H The homography matrix used for warping.
 * @param[out] out_img_mat Reference to the outputed warped image.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int warpImgMat(
    cv::Mat img_mat,
    cv::Mat _H,
    cv::Mat &out_img_mat);

#endif