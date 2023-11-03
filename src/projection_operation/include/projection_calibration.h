// ##########################################################################################################

// ======================================== projection_calibration.h ========================================

// ##########################################################################################################

#ifndef _PROJECTION_CALIBRATION_H
#define _PROJECTION_CALIBRATION_H

// ================================================== INCLUDE ==================================================

// Local custom libraries
#include "projection_utils.h"

// ================================================== VARIABLES ==================================================

/**
 * @brief  4x4 data container for tracking the vertex coordinates for the corner wall images which are used as control point
 *
 * The array stores the corner wall vertex as OpenGL's Normalized Device Coordinates (NDC), which range from [-1, 1]
 * with the origin at the center of the screen.
 *
 * @details
 *[4][4] = [conrol point][vertex]
 *
 * - Dimension 1: Control Point [0, 1, 2, 3]
 * - Index with respect to the overall maze verteces
 *      - 0: Top-left
 *      - 1: Top-right
 *      - 2: Bottom-right
 *      - 3: Bottom-left
 *
 * - Dimension 2: Vertex(x, y) [0, 1, 2, 3]
 * - Index with respect to the wall image verteces
 *      - 0: Top-left
 *      - 1: Top-right
 *      - 2: Bottom-right
 *      - 3: Bottom-left
 */
std::array<std::array<cv::Point2f, 4>, 4> CP_COORDS;

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
 * @brief Global Element Buffer Object (EBO) for wall images and control point markers.
 */
GLuint WALL_EBO;
GLuint CP_EBO;

/**
 * @brief Global Vertex Array Object (VAO) for wall image and control point markers.
 */
GLuint WALL_VAO; // Vertex Array Object (VAO) for wall image.
GLuint CP_VAO;   // Vertex Array Object (VAO) for control point markers.

/**
 * @brief  Global Vertex Buffer Object (VBO) for rendering the wall image.
 */
GLuint WALL_VBO;

/**
 * @brief  Global Vertex Buffer Object (VBO) arrays for rendering the control point markers.
 */
std::array<std::array<GLuint, 4>, 4> CP_VBO_POS_ARR;
std::array<std::array<GLuint, 4>, 4> CP_VBO_RGB_ARR;
std::array<std::array<GLuint, 4>, 4> CP_VBO_RAD_ARR;

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

/**
 * @brief Vertex shader source code for control point rendering.
 *
 * @note The shader program will reverse the y axis to match image coordinate systems used by OpenCV.
 *
 * @details
 * This GLSL vertex shader source code is stored as a C++ raw string literal.
 * - `#version 330 core`: Specifies that the GLSL version is 3.30 and we're using the core profile.
 * - `layout (location = 0) in vec2 position;`: Declares an input vertex attribute for position, received from the application.
 *      The y-axis will be inverted to match image coordinate systems such as those used in OpenCV.
 * - `layout (location = 1) in vec3 color;`: Declares an input vertex attribute for color, received from the application.
 * - `layout (location = 2) in float size;`: Declares an input vertex attribute for size, received from the application.
 * - `out vec3 fragColor;`: Passes the color to the fragment shader.
 * - `void main() { ... }`: Main function of the shader, where it calculates the final position of the vertex with an inverted y-axis and sets the point size for rendering.
 */
const GLchar *ctrlPtVertexSource = R"glsl(
    #version 330 core
    layout (location = 0) in vec2 position;
    layout (location = 1) in vec3 color;
    layout (location = 2) in float size;
    out vec3 fragColor;
    void main() {
        // Invert the y coordinate to match image coordinate systems
        gl_Position = vec4(position.x, -position.y, 0.0, 1.0);
        gl_PointSize = size;
        fragColor = color;
    }
)glsl";

/**
 * @brief Fragment shader source code for control point rendering.
 *
 * @details
 * This GLSL fragment shader source code is stored as a C++ raw string literal.
 * - `#version 330 core`: Specifies that the GLSL version is 3.30 and we're using the core profile.
 * - `in vec3 fragColor;`: Receives the color from the vertex shader.
 * - `out vec4 color;`: Declares an output variable to output the color of the fragment.
 * - `void main() { ... }`: Main function of the shader, which outputs the color of the control point.
 */
const GLchar *ctrlPtFragmentSource = R"glsl(
    #version 330 core
    in vec3 fragColor;
    out vec4 color;
    void main() {
        color = vec4(fragColor, 1.0);
    }
)glsl";

/**
 * @brief Shader program IDs for wall image and control point marker rendering.
 *
 * @details
 * - `WALL_SHADER`: Shader program used for rendering the warped wall images.
 * - `CP_SHADER`: Shader program used for rendering the control point markers as circles.
 */
GLuint WALL_SHADER;
GLuint CP_SHADER;

/**
 * @brief  OpenGL textures associated with the current wall texture.
 */
GLuint WALL_TEXTURE_ID;

/**
 * @brief  3x3 data contianer for storing the 3x3 homography matrix for each wall image
 */
std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> WALL_HMAT_DATA;

// Global variable to set the OpenGL debug level.
int DEBUG_LEVEL_GL = 3; // [0: None, 1: >=Default 2: >=Low, 3: >=Medium, 4: High]

// Control point graphics
const std::array<GLfloat, 3> cpVertSelectedRGB = {0.0f, 1.0f, 0.0f}; // Select control point marker color (green)
const std::array<GLfloat, 3> cpWallSelectedRGB = {1.0f, 0.0f, 0.0f}; // Selected control point wall color (red)
const std::array<GLfloat, 3> cpDefaultRGB = {0.0f, 0.0f, 1.0f};      // Default control point marker color (blue)
const GLfloat cpDefualtMakerRadius = 0.0025f;                        // Control point image radius
const GLfloat cpSelectedMakerRadius = 0.005f;                        // Control point image radius

// Struct for flags set/used in the keyboard callback
static struct FlagStruct
{
    bool dbRun = false;                   // Flag to indicate if something should be run for debugging
    bool loadXML = false;                 // Flag to indicate if the XML file needs to be loaded
    bool saveXML = false;                 // Flag to indicate if the XML file needs to be saved
    bool updateWindowMonMode = false;     // Flag to indicate if the window mode needs to be updated
    bool initControlPointMarkers = false; // Flag to indicate if the control point markers need to be reinitialized
    bool updateWallDatasets = false;      // Flag to indicate if the warped wall vertices need to be updated
    bool setFullscreen = false;           // Flag to indicate if the window needs to be set to full screen mode
} F;

// Struct for indices set/used in the keyboard callback
static struct IndStruct
{
    std::array<int, 2> cpSelected = {0, 0}; // Index: maze_corner[0,1] wall_vertex[0,1,2,3]
    int wallImage = 0;                      // Index of the image to be loaded
    int calMode = 1;                        // Index of the image to be loaded
    int winMon = 0;                         // Index of the active monitor to be loaded
} I;

// Struct for counts used in the keyboard callback
static struct CountStruct
{
    int monitors;       // Number of monitors connected to the system
    int wallImages = 4; // Number of wall images
    int calModes = 3;   // Number of calibration modes
} N;

// Sub-directory paths
std::string image_wall_dir_path = IMAGE_TOP_DIR_PATH + "/calibration_images";
std::string image_state_dir_path = IMAGE_TOP_DIR_PATH + "/ui_mode_images";

// Image file paths
std::vector<std::string> wallImgPathVec = {
    // List of image file paths
    image_wall_dir_path + "/1_test_pattern.png",
    image_wall_dir_path + "/2_manu_pirate.png",
    image_wall_dir_path + "/3_earthlings.png",
    image_wall_dir_path + "/4_all_white.png",
};
std::vector<std::string> monImgPathVec = {
    // List of monitor number image file paths
    image_state_dir_path + "/m0.png",
    image_state_dir_path + "/m1.png",
    image_state_dir_path + "/m2.png",
    image_state_dir_path + "/m3.png",
    image_state_dir_path + "/m4.png",
    image_state_dir_path + "/m5.png",
};
std::vector<std::string> calImgPathVec = {
    // List of mode image file paths
    image_state_dir_path + "/cwl.png", // left walls
    image_state_dir_path + "/cwm.png", // middle walls
    image_state_dir_path + "/cwr.png", // right walls
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
void callbackFrameBufferSizeGLFW(GLFWwindow *, int, int);

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
 *
 * @details
 * This function is called whenever an error occurs in the OpenGL context.
 * It logs the error message using ROS_ERROR.
 */
static void APIENTRY callbackDebugOpenGL(GLenum, GLenum, GLuint, GLenum, GLsizei, const GLchar *, const void *);

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
static void callbackErrorGLFW(int, const char *);

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
 * @example checkErrorGL(__LINE__, __FILE__);
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
 * @brief Changes the display mode and monitor of the application window.
 *
 *
 * @param p_window_id Pointer to the GLFWwindow pointer that will be updated.
 * @param win_ind Index of the window for which the setup is to be done.
 * @param pp_r_monitor_id Reference to the GLFWmonitor pointer array.
 * @param mon_id_ind Index of the monitor to move the window to.
 * @param is_fullscreen Boolean flag indicating whether the window should be set to full-screen mode.
 *
 * @return Integer status code  [0:successful, -1:error].
 *
 * @details
 * This function switches the application window between full-screen and windowed modes
 * and moves it to the monitor specified by the global variable imgMonNumInd.
 *
 * In full-screen mode, the window is resized to match the dimensions of the selected monitor.
 * In windowed mode, the window is resized to a default size and positioned near the top-left
 * corner of the selected monitor.
 *
 * @note The global variables monitor, monitors, imgMonNumInd, window, and F.setFullscreen are
 *       used to control the behavior of this function.
 *       Will only exicute if monotor parameters have changed.
 */
int updateWindowMonMode(GLFWwindow *, int, GLFWmonitor **&, int, bool);

/**
 * @brief Renders a all wall images from the computed texture2D maze grid by drawing each cell (e.g., wall) with texture mapping and perspective warping.
 *
 * @param _WALL_TEXTURE_ID OpenGL texture ID for the wall image.
 *
 * @return Integer status code  [0:successful, -1:error].
 */
int renderWallImage(const GLuint &);

/**
 * @brief Draws control points associated with each corner wall.
 *
 * @param _CP_COORDS The control point coordinates used to warp the wall image.
 *
 * @return Integer status code [0:successful, -1:error].
 */
int renderControlPoints(const std::array<std::array<cv::Point2f, 4>, 4> &);

/**
 * @brief Initialize OpenGL resources for wall image render objects.
 *
 * @return Integer status code [0:successful, -1:error].
 *
 * @details
 * Initializes the vertex buffer, shader program, and default values
 * for the wall image render and control point markers.
 */
int initializeWallObjects();

/**
 * @brief Initialize OpenGL resources for control point marker objects.
 *
 * @return Integer status code [0:successful, -1:error].
 *
 * @details
 * Initializes the vertex buffer, shader program, and default values
 * for the control point markers.
 */
int initializeControlPointObjects();

/**
 * @brief Creates an OpenGL shader program from vertex and fragment shader source code.
 *
 * @param vertex_source Source code for the vertex shader.
 * @param fragment_source Source code for the fragment shader.
 * @param geometry_source Source code for the geometry shader.
 *
 * @return GLuint ID of the generated shader program.
 *
 * @details
 * This function encapsulates the process of creating, compiling, and linking an OpenGL shader program.
 * The OpenGL shader program is part of the OpenGL graphics pipeline and is essential for rendering graphics.
 *
 * The pipeline can be broken down into the following stages:
 * 1. Vertex Processing: Vertex shaders manipulate the attributes of vertices.
 *    This can include things like transforming the vertex position, normal, texture coordinate, etc.
 * 2. Primitive Assembly: Vertices are grouped into geometric primitives (points, lines, and triangles).
 * 3. Rasterization: The primitives are converted into a set of fragments.
 * 4. Fragment Processing: Fragment shaders manipulate the attributes of fragments,
 *    which are essentially potential pixels. Here you might apply textures, calculate lighting, etc.
 * 5. Output Merging: Fragments are converted into actual framebuffer pixels.
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
GLuint compileAndLinkShaders(const GLchar *, const GLchar *, const GLchar *);

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
 * @brief Updates the stored warped wall image vertices based on the control point array.
 *
 * @param img_wall_mat cv::Mat image matrix for the base wall image.
 * @param img_mode_mon_mat cv::Mat image matrix for the monitor mode image.
 * @param img_mode_cal_mat cv::Mat image matrix for the calibration image.
 * @param _WALL_HMAT_DATA 3x3 array of Homography matrices used to warp the wall image.
 * @param[out] out_WALL_TEXTURE_ID OpenGL texture ID for the wall image.
 *
 * @return Integer status code [0:successful, -1:error].
 */
int updateWallTexture(
    cv::Mat, cv::Mat, cv::Mat,
    std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> &,
    GLuint &);

bool initializeGLFWandGLAD();
void testRenderSinglePoint();

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