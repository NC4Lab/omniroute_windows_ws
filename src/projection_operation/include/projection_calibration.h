// ##########################################################################################################

// ======================================== projection_calibration.h ========================================

// ##########################################################################################################

#ifndef _PROJECTION_CALIBRATION_H
#define _PROJECTION_CALIBRATION_H

// ================================================== INCLUDE ==================================================

// Local custom libraries
#include "projection_utils.h"

// ================================================== CLASS: MazeRenderContext ==================================================

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
     * @example checkErrorGLFW(__LINE__, __FILE__);
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
     * @return int Status of context initialization (0 for success, -1 for failure).
     */
    int initContext(int win_ind, int mon_ind);

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

// Control point graphics
const int cpRenderSegments = 36;                                       // Number of segments used to approximate the circle geometry
const cv::Scalar cpWallVertSelectedRGB = cv::Scalar(1.0f, 0.0f, 0.0f); // Select control point marker color (red)
const cv::Scalar cpMazeVertSelectedRGB = cv::Scalar(0.0f, 1.0f, 0.0f); // Selected control point wall color (green)
const cv::Scalar cpDefaultRGB = cv::Scalar(0.0f, 0.0f, 1.0f);          // Default control point marker color (blue)
const GLfloat cpDefualtMakerRadius = 0.0025f;                          // Default control point rendered circle radius
const GLfloat cpSelectedMakerRadius = 0.005f;                          // Selected control point rendered circle radius

/**
 * @brief  4x4 data container for tracking the vertex coordinates for the corner wall images which are used as control point.
 *
 * The array stores the corner wall vertex as OpenGL's Normalized Device Coordinates (NDC), which range from [-1, 1]
 * with the origin at the center of the screen.
 *
 * @details
 *[4][4] = [conrol point][vertex]
 *
 * - Dimension 1: Control Point [0, 1, 2, 3]
 * - Index with respect to the overall maze verteces
 *
 * - Dimension 2: Vertex(x, y) [0, 1, 2, 3]
 * - Index with respect to the wall image verteces
 *
 * - Note the mind-fucking vertex indices please:
 *      - 0: OpenGL: Top-left       OpenCV: Bottom-left
 *      - 1: OpenGL: Top-right      OpenCV: Bottom-right
 *      - 2: OpenGL: Bottom-right   OpenCV: Top-right
 *      - 3: OpenGL: Bottom-left    OpenCV: Top-left
 */
std::array<std::array<cv::Point2f, 4>, 4> CP_GRID_ARR;

// Create an associateed 4x4 array of the CircleRenderer class objects
std::array<std::array<CircleRenderer, 4>, 4> CP_RENDERERS;

/**
 * @brief  3x3 data contianer for storing the 3x3 homography matrix for each wall image
 */
std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> HMAT_GRID_ARR;

// Global variable to set the OpenGL debug level.
int DEBUG_LEVEL_GL = 3; // [0: None, 1: >=Default 2: >=Low, 3: >=Medium, 4: High]

// Struct for global flags
static struct FlagStruct
{
    bool dbRun = false;                   // Flag to indicate if something should be run for debugging
    bool loadXML = false;                 // Flag to indicate if the XML file needs to be loaded
    bool saveXML = false;                 // Flag to indicate if the XML file needs to be saved
    bool switchWindowMode = false;        // Flag to indicate if the window mode needs to be updated
    bool initControlPointMarkers = false; // Flag to indicate if the control point markers need to be reinitialized
    bool updateWallTexture = false;       // Flag to indicate if wall vertices, homography and texture need to be updated
    bool setFullscreen = false;           // Flag to indicate if the window needs to be set to full screen mode
} F;

// Struct for global counts
static struct CountStruct
{
    int monitors;       // Number of monitors connected to the system
    int wallImages = 4; // Number of wall images
    int calModes = 3;   // Number of calibration modes
} N;

// Struct for global indices
static struct IndStruct
{
    int wallImage = 0; // Index of the image to be loaded
    int calMode = 1;   // Index of the current calibration mode walls[0: left, 1: middle, 2: right]
    int winMon = 0;    // Index of the active monitor to be loaded
    /**
     * @brief cpRowColMap maps a given row cpRowColMap[0] and column cpRowColMap[1] index to the 1D vector.
     * To make things more complicated, this needs to account for the y axis being flipped.
     *
     * @details
     * - Maze/wall vertex indices
     *      - 0: OpenGL: Top-left       OpenCV: Bottom-left
     *      - 1: OpenGL: Top-right      OpenCV: Bottom-right
     *      - 2: OpenGL: Bottom-right   OpenCV: Top-right
     *      - 3: OpenGL: Bottom-left    OpenCV: Top-left
     */
    std::array<std::array<int, 2>, 2> cpMap = {{{0, 1},
                                                {3, 2}}};
    std::array<int, 2> cpMazeVertSel = {0, 0}; // Selected maze vertex [row, col]
    std::array<int, 2> cpWallVertSel = {0, 0}; // Selected wall vertex [row, col]
    const int cpWVOrigin = 3;                  // Vertex index of the wall image origin (bottom-left)
    // std::array<int, 2> cpSelected = {0, 0};
} I;

// Sub-directory paths
std::string calib_image_path = IMAGE_TOP_DIR_PATH + "/calibration";

// Image file paths
std::vector<std::string> wallImgPathVec = {
    // List of image file paths
    calib_image_path + "/1_test_pattern.png",
    calib_image_path + "/2_manu_pirate.png",
    calib_image_path + "/3_earthlings.png",
    calib_image_path + "/4_all_white.png",
};
std::vector<std::string> monImgPathVec = {
    // List of monitor number image file paths
    calib_image_path + "/m0.png",
    calib_image_path + "/m1.png",
    calib_image_path + "/m2.png",
    calib_image_path + "/m3.png",
    calib_image_path + "/m4.png",
    calib_image_path + "/m5.png",
};
std::vector<std::string> calImgPathVec = {
    // List of mode image file paths
    calib_image_path + "/cwl.png", // left walls
    calib_image_path + "/cwm.png", // middle walls
    calib_image_path + "/cwr.png", // right walls
};

// Vectors to store the loaded images in cv::Mat format
std::vector<cv::Mat> wallImgMatVec; // Vector of wall image texture matrices
std::vector<cv::Mat> monImgMatVec;  // Vector of monitor mode image texture matrices
std::vector<cv::Mat> calImgMatVec;  // Vector of calibration mode image texture matrices

// ================================================== FUNCTIONS ==================================================

/**
 * @brief GLFW key callback function to handle key events and execute corresponding actions.
 *
 * @param window Pointer to the GLFW window that received the event.
 * @param key The keyboard key that was pressed or released.
 * @param scancode The system-specific scancode of the key.
 * @param action GLFW_PRESS, GLFW_RELEASE or GLFW_REPEAT.
 * @param mods Bit field describing which modifier keys were held down.
 *
 * @details
 * This function is set as the GLFW key callback and gets called whenever a key event occurs.
 * It handles various key events for control points, monitor handling, XML operations, and more.
 *
 * ## Keybindings:
 * @see README.md
 */
void callbackKeyBinding(GLFWwindow *, int, int, int, int);

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
 * @example checkErrorOpenGL(__LINE__, __FILE__);
 */
int checkErrorOpenGL(int, const char *, const char * = nullptr);

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
 * @example checkErrorGLFW(__LINE__, __FILE__);
 */
int checkErrorGLFW(int, const char *, const char * = nullptr);

/**
 * @brief Initializes values for the verteces of the coner walls which will be used as calibraton control points.
 *
 * @param[out] out_CP_GRID_ARR Reference to the 4x4 array containing the coordinates of the corner wall's vertices.
 *
 * @details
 * Control point x and y coordinates are specified in Normalized Device Coordinates (NDC) [-1, 1].
 * The vertices for the entire projected image are calculated based on the dimensions that enclose
 * all control points (i.e., boundary dimensions in the control point plane).
 */
void initCtrlPtCoords(std::array<std::array<cv::Point2f, 4>, 4> &);

/**
 * @brief Loads PNG images with alpha channel from specified file paths and stores them in a vector as cv::Mat objects.
 *
 * @param img_paths_vec A vector of file paths to the images to be loaded.
 * @param[out] out_img_mat_vec Reference to a vector of cv::Mat where the loaded images will be stored.
 *
 * @return Integer status code [0:successful, -1:error].
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
int loadImgMat(const std::vector<std::string> &img_paths_vec, std::vector<cv::Mat> &out_img_mat_vec);

/**
 * @brief Merges a mask image over a base image using the alpha channel and stores the result.
 *
 * @param base_img_path Output cv::Mat containing the base image.
 * @param[out] out_base_img cv::Mat containing the base image for merging.
 *
 * @return Integer status code [0:successful, -1:error].
 *
 * @details
 * This function overlays the mask image on top of the base image using the alpha channel
 * of the mask image. Pixels from the mask image are copied over to the base image based on
 * the alpha value - if the alpha value is not fully transparent (0), the pixel is copied.
 */
int mergeImgMat(const cv::Mat &mask_img, cv::Mat &out_base_img);

/**
 * @brief Initialize OpenGL resources for wall image render objects.
 *
 * @param[out] out_renCtx Reference to an instance of the out_renCtx class.
 *
 * @return Integer status code [0:successful, -1:error].
 *
 * @details
 * Initializes the Vertex Array Object (VAO), Vertex Buffer Object (VBO) and Element Buffer Object (EBO).
 */
int initWallRenderObjects(MazeRenderContext &out_renCtx, float *vertices, size_t verticesSize, unsigned int *indices, size_t indicesSize);

/**
 * @brief Initialize OpenGL resources for CircleRenderer objects.
 *
 * @param _CP_GRID_ARR The 4x4 array containing the coordinates of the corner wall's vertices.
 * @param[out] out_CP_RENDERERS The 4x4 array of CircleRenderer objects used to draw the control points.
 *
 * @return Integer status code [0:successful, -1:error].
 *
 * @details
 * Initializes the vertex buffer, shader program, and default values
 * for the control point markers.
 */
int initCircleRendererObjects(const std::array<std::array<cv::Point2f, 4>, 4> &, std::array<std::array<CircleRenderer, 4>, 4> &);

/**
 * @brief Updates the stored warped wall image vertices based on the control point array.
 *
 * @param img_wall_mat cv::Mat image matrix for the base wall image.
 * @param img_mode_mon_mat cv::Mat image matrix for the monitor mode image.
 * @param img_mode_cal_mat cv::Mat image matrix for the calibration image.
 * @param _HMAT_GRID_ARR 3x3 array of Homography matrices used to warp the wall image.
 * @param[out] out_wallTexture OpenGL texture ID for the wall image.
 *
 * @return Integer status code [0:successful, -1:error].
 */
int updateWallTexture(
    cv::Mat, cv::Mat, cv::Mat,
    std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> &,
    GLuint &);

/**
 * @brief Converts an OpenCV Mat image into an OpenGL texture and returns the texture ID.
 *
 * @param image The cv::Mat image that needs to be converted.
 *
 * @return GLuint ID of the generated texture.
 *
 * @details
 * This function takes an OpenCV Mat image as input and converts it into an OpenGL texture.
 * The OpenCV image is first converted from BGR to RGB format. Then, a new OpenGL texture is
 * generated and the converted image data is stored in this texture.
 *
 * The texture parameters for minification and magnification filters are set to GL_LINEAR.
 *
 * Note: This function assumes that the input image is of type CV_8UC3 and has no alpha channel.
 */

GLuint loadTexture(cv::Mat);

/**
 * @brief Renders a all wall images from the computed texture2D maze grid by drawing each cell (e.g., wall) with texture mapping and perspective warping.
 *
 * @param renCtx Reference to an instance of the out_renCtx class.
 *
 * @return Integer status code  [0:successful, -1:error].
 */
int renderWallImage(const MazeRenderContext &renCtx);

/**
 * @brief Draws control points associated with each corner wall.
 *
 * @param _CP_GRID_ARR The control point coordinates used to warp the wall image.
 * @param[out] out_CP_RENDERERS The 4x4 array of CircleRenderer objects used to draw the control points.
 *
 * @return Integer status code [0:successful, -1:error].
 */
int renderControlPoints(const std::array<std::array<cv::Point2f, 4>, 4> &, std::array<std::array<CircleRenderer, 4>, 4> &);

/**
 * @brief  Entry point for the projection_calibration ROS node.
 *
 *
 * @param  argc  Number of command-line arguments.
 * @param  argv  Array of command-line arguments.
 *
 * @return Integer status code [0:successful, -1:error].
 *
 * @details
 * This program initializes ROS, DevIL, and GLFW, and then enters a main loop
 * to handle image projection and calibration tasks.
 */
int main(int, char **);

#endif