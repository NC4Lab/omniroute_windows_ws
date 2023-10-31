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
* @var const GLchar* vertexSource
* @brief Vertex shader source code.

* @details
* This is a GLSL (OpenGL Shading Language) vertex shader source code stored as a C++ raw string literal.
* - `#version 330 core`: Specifies that the GLSL version is 3.30 and we're using the core profile.
* - `in vec2 position;`: Declares an input vertex attribute called `position`. Receives vertex coordinates from the application.
* - `in vec2 texcoord;`: Declares another input vertex attribute called `texcoord`. Receives texture coordinates from the application.
* - `out vec2 Texcoord;`: Declares an output variable that will be passed to the fragment shader.
* - `void main() { ... }`: Main function where the vertex shader performs its work.
*/
extern const char* vertexSource;

/**
* @var const GLchar* fragmentSource
* @brief Fragment shader source code.
*
* @details
* This is a GLSL (OpenGL Shading Language) fragment shader source code also stored as a C++ raw string literal.
* - `#version 330 core`: Specifies that the GLSL version is 3.30 and we're using the core profile.
* - `in vec2 Texcoord;`: Receives the texture coordinates from the vertex shader.
* - `out vec4 outColor;`: Declares an output variable for storing the color to be used for the fragment.
* - `uniform sampler2D tex;`: Declares a uniform variable representing a 2D texture.
* - `void main() { ... }`: Main function of the fragment shader, samples the texture at the given coordinates and sets the output color.
*/
extern const char* fragmentSource;

/**
 * @brief Shader program IDs for wall image and circle rendering.
 *
 * @details
 * - `wallShaderProgram`: Shader program used for rendering the warped wall images.
 * - `circleShaderProgram`: Shader program used for rendering the control point markers as circles.
 */
GLuint wallShaderProgram, circleShaderProgram;

/**
 * @brief Global Vertex Buffer Object (VBO) for wall images and control point markers.
 * 
 * @details 
 * - 'wallVertexBuffer': This VBO contains the vertex data for rendering and transforming
 *      the wall images. It is initialized at the beginning of the program and
 *      updated as needed.
 * - 'circleVertexBuffer': This VBO contains the vertex data for rendering the control
 *      point markers (circles). Like wallVertexBuffer, it is initialized at the start
 *      and updated as necessary.
 */
GLuint wallVertexBuffer, circleVertexBuffer;

/**
 * @brief  4x4 data container for tracking the paramaters related to the plotted control point markers  asscicated with the 
 * vertex coordinates for the corner wall images 
 *
 * @details
 *[4][4] = [conrol point][vertex]
 *
 * - Dimension 1: Control Point [0, 1, 2, 3]
 * - Vetices are with respect to the overall viewport
 *      - 0: Top-left
 *      - 1: Top-right
 *      - 2: Bottom-left
 *      - 3: Bottom-right
 *
 * - Dimension 2: Vertex(x, y) [0, 1, 2, 3]
 * - Vetices are with respect to the wall image vertex
 *      - 0: Top-left 
 *      - 1: Top-right
 *      - 2: Bottom-right 
 *      - 3: Bottom-left
 * 
 * @param point: The x and y coordinates of the vertex
 */
struct CpMarkers {
    cv::Point2f point; // The x and y coordinates of the vertex
    float radius; // The radius of the control point
    std::array<float, 3> color; // The RGB color values for the control point
};
std::array<std::array<CpMarkers, 4>, 4> CPm;


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
 *  - 0: Top-left (image vertex)
 *  - 1: Top-right (image vertex)
 *  - 2: Bottom-left (image vertex)
 *  - 3: Bottom-right (image vertex)
 *
 * - Dimension 2: Vertex(x, y) [0, 1, 2, 3]
 *  - 0: Top-left  (quadrilateral vertex)
 *  - 1: Top-right  (quadrilateral vertex)
 *  - 2: Bottom-left  (quadrilateral vertex)
 *  - 3: Bottom-right  (quadrilateral vertex)
 */
std::array<std::array<cv::Point2f, 4>, 4> CTRL_PNT_DATA;

/**
 * @brief  MAZE_SIZExMAZE_SIZEx4 data container for storing the final warped wall vertex coordinates in Normalized Device Coordinates (NDC)
 *
 * The array stores the all wall verteces as OpenGL's Normalized Device Coordinates (NDC), which range from [-1, 1]
 * with the origin at the center of the screen.
 *
 * @details
 * [MAZE_SIZE][MAZE_SIZE][4] = [row][col][vertex]
 *
 * - Dimension 1: Rows [0-MAZE_SIZE-1]:
 *  - Top to Bottom
 *
 * - Dimension 2: Column [0-MAZE_SIZE-1]:
 *  - Left to Right
 *
 * - Dimension 3: Vertex(x, y) [0, 1, 2, 3]
 *  - 0: Top-left  (quadrilateral vertex)
 *  - 1: Top-right  (quadrilateral vertex)
 *  - 2: Bottom-left  (quadrilateral vertex)
 *  - 3: Bottom-right  (quadrilateral vertex)
 */
std::array<std::array<std::array<cv::Point2f, 4>, MAZE_SIZE>, MAZE_SIZE> WALL_VERT_DATA;

/**
 * @brief  3x3 data contianer for storing the 3x3 homography matrix for each wall image
 */
std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> WALL_HMAT_DATA;


// The 3x3 homography matrix of 32-bit floating-point numbers used to warp perspective.
cv::Mat HMAT = cv::Mat::eye(3, 3, CV_32F);

// Struct primarely used for tracking flags set in the keyboard callback
static struct FlagStruct
{
    bool dbRun = false;                    // Flag to indicate if something should be run for debugging
    bool loadXML = false;                  // Flag to indicate if the XML file needs to be loaded
    bool saveXML = false;                  // Flag to indicate if the XML file needs to be saved
    bool updateWindowMonMode = false;      // Flag to indicate if the window mode needs to be updated
    bool initControlPointMarkers = false;  // Flag to indicate if the control point markers need to be reinitialized
    bool updateWallDatasets = false; // Flag to indicate if the warped wall vertices need to be updated
} F;

// Specify the window name
std::string windowName = "Projection Calibration";

// Dynamic control point parameter arrays
std::array<std::array<float, 6>, 4> ctrlPointParams;
int cpSelectedInd = 0;
std::string calModeStr = "position"; // Parmeter being modified [position, dimension, shear]

// Control point graphics
std::array<float, 3> cpVertSelectedRGB = {0.0f, 1.0f, 0.0f}; // Select control point marker color (green)
std::array<float, 3> cpWallSelectedRGB = {1.0f, 0.0f, 0.0f}; // Selected control point wall color (red)
std::array<float, 3> cpUnelectedRGB = {0.0f, 0.0f, 1.0f};    // Inactive control point marker color (blue)

// Contorl point user interface
int cpWallSelectedInd = 0; // Selected control point wall index [0,1,2,3]
int cpVertSelectedInd = 2; // Selected control point wall vertex index [0,1,2,3]

// Control point image radius
const std::array<float, 2> cpMakerRadius = {0.0025f, 0.005f};

// The 3x3 homography matrix of 32-bit floating-point numbers used to warp perspective.
cv::Mat homMat = cv::Mat::eye(3, 3, CV_32F);

// Directory paths
std::string image_wall_dir_path = IMAGE_TOP_DIR_PATH + "/calibration_images";
std::string image_state_dir_path = IMAGE_TOP_DIR_PATH + "/ui_state_images";

// Test image variables
std::vector<ILuint> texWallIDVec; // Container to hold the loaded images
std::vector<std::string> imgWallPathVec = {
    // List of image file paths
    image_wall_dir_path + "/1_test_pattern.bmp",
    image_wall_dir_path + "/2_manu_pirate.bmp",
    image_wall_dir_path + "/3_earthlings.bmp",
    image_wall_dir_path + "/4_all_white.bmp",
};
int imgWallInd = 0; // Index of the image to be loaded

// Monitor variables
std::vector<ILuint> texMonIDVec; // Container to hold the loaded images for ui
std::vector<std::string> imgMonPathVec = {
    // List of monitor number image file paths
    image_state_dir_path + "/m0.bmp",
    image_state_dir_path + "/m1.bmp",
    image_state_dir_path + "/m2.bmp",
    image_state_dir_path + "/m3.bmp",
    image_state_dir_path + "/m4.bmp",
    image_state_dir_path + "/m5.bmp",
};
int winMonInd = 0;         // Index of the image to be loaded
int nMonitors;             // Number of monitors connected to the system
bool isFullScreen = false; // Flag to indicate if the window is in full screen mode

// Callibration image variables for ui
std::vector<ILuint> texCalIDVec; // Container to hold the loaded images for ui
std::vector<std::string> imgCalPathVec = {
    // List of mode image file paths
    image_state_dir_path + "/c-wl.bmp", // left walls
    image_state_dir_path + "/c-wm.bmp", // middle walls
    image_state_dir_path + "/c-wr.bmp", // right walls
    // image_state_dir_path + "/c-f.bmp",  // maze floor
    // image_state_dir_path + "/c-d.bmp",  // distal cues
};
int calModeInd = 1;                      // Index of the image to be loaded
size_t nCalModes = imgCalPathVec.size(); // Number of calibration modes

// Variables related to window and OpenGL
GLFWwindow *p_windowID = nullptr;
GLFWmonitor **pp_monitorIDVec = nullptr;

// ================================================== FUNCTIONS ==================================================

/**
 * @brief GLFW key callback function to handle key events and execute corresponding actions.
 *
 * This function is set as the GLFW key callback and gets called whenever a key event occurs.
 * It handles various key events for control points, monitor handling, XML operations, and more.
 *
 * ## Keybindings:
 * @see README.md
 *
 * @param window Pointer to the GLFW window that received the event.
 * @param key The keyboard key that was pressed or released.
 * @param scancode The system-specific scancode of the key.
 * @param action GLFW_PRESS, GLFW_RELEASE or GLFW_REPEAT.
 * @param mods Bit field describing which modifier keys were held down.
 */
void callbackKeyBinding(GLFWwindow *, int, int, int, int);

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
void callbackFrameBufferSizeGLFW(GLFWwindow *, int, int);

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
static void callbackErrorOpenGL(GLenum, GLenum, GLuint, GLenum, GLsizei, const GLchar *, const void *);

/**
 * @brief Callback function for handling errors.
 *
 * This function is called whenever an error occurs in the GLFW context.
 * It logs the error message using ROS_ERROR.
 *
 * @param error The error code.
 * @param description The error description.
 */
static void callbackErrorGLFW(int, const char *);

/**
 * @brief Checks for OpenGL errors and logs them.
 * Should be called after OpenGL API calls.
 *
 * @example checkErrorGL(__LINE__, __FILE__);
 *
 * @param line Line number where the function is called.
 * @param file_str File name where the function is called.
 * @param msg_str Optional message to provide additional context (default to nullptr).
 *
 * @return Integer status code  [0:successful, -1:error].
 */
int checkErrorOpenGL(int, const char *, const char * = nullptr);

/**
 * @brief Checks for GLFW errors and logs them.
 * Should be called after GLFW API calls.
 *
 * @example checkErrorGLFW(__LINE__, __FILE__);
 *
 * @param line Line number where the function is called.
 * @param file_str File name where the function is called.
 * @param msg_str Optional message to provide additional context (default to nullptr).
 *
 * @return Integer status code  [0:successful, -1:error].
 */
int checkErrorGLFW(int, const char *, const char * = nullptr);

/**
 * @brief Changes the display mode and monitor of the application window.
 *
 * This function switches the application window between full-screen and windowed modes
 * and moves it to the monitor specified by the global variable imgMonNumInd.
 *
 * In full-screen mode, the window is resized to match the dimensions of the selected monitor.
 * In windowed mode, the window is resized to a default size and positioned near the top-left
 * corner of the selected monitor.
 *
 * @note The global variables monitor, monitors, imgMonNumInd, window, and isFullScreen are
 *       used to control the behavior of this function.
 *       Will only exicute if monotor parameters have changed.
 *
 * @param p_window_id Pointer to the GLFWwindow pointer that will be updated.
 * @param win_ind Index of the window for which the setup is to be done.
 * @param pp_r_monitor_id Reference to the GLFWmonitor pointer array.
 * @param mon_id_ind Index of the monitor to move the window to.
 * @param is_fullscreen Boolean flag indicating whether the window should be set to full-screen mode.
 *
 * @return Integer status code  [0:successful, -1:error].
 */
int updateWindowMonMode(GLFWwindow *, int, GLFWmonitor **&, int, bool);

/**
 * @brief Draws a control point as a quadrilateral using OpenGL.
 *
 * This function uses OpenGL to draw a quadrilateral that represents a control point.
 * The control point is drawn as a colored circle.
 *
 * @param x The control point x-coordinate.
 * @param y The control point y-coordinate.
 * @param radius The radius of the control point.
 * @param rgb_arr Array of rgb values to color the marker.
 *
 * @return Integer status code  [0:successful, -1:error].
 */
int drawColoredCircle(float, float, float, std::array<float, 3>);

/**
 * @brief Draws control points associated with each corner wall.
 *
 */
int updateControlPointMarkers();

/**
 * @brief Draws a textured quadrilateral using OpenGL.
 *
 * @param quad_vertices_vec Array of vertex/corner points for a quadrilateral image.
 *
 * @return Integer status code  [0:successful, -1:error].
 */
int drawQuadImage(std::array<cv::Point2f, 4>);

/**
 * @brief Renders a 2D maze grid by drawing each cell (e.g., wall) with texture mapping and perspective warping.
 *
 * This function is a core part of the maze visualization pipeline. It utilizes the OpenGL graphics library for rendering,
 * and the DevIL library for image handling. The function performs several key operations:
 * 1. Texture mapping of the wall images.
 * 2. Perspective warping based on a precomputed homography matrix.
 * 3. Shear and height adjustments based on control point calibration.
 * 4. Optional overlay of status images for cells corresponding to selected control points.
 *
 * @param fbo_texture_id OpenGL framebuffer object's (FBO) texture ID.
 * @param tex_wall_id DevIL image ID for the base wall image.
 * @param tex_mode_mon_id DevIL image ID for the monitor mode image.
 * @param tex_mode_cal_id DevIL image ID for the calibration image.
 *
 * @return Integer status code  [0:successful, -1:error].
 */
int drawWallImages(GLuint, ILuint, ILuint, ILuint);

void setupOpenGL();

/**
 * @brief Creates an OpenGL shader program from vertex and fragment shader source code.
 *
 * @param vertexSource Source code for the vertex shader.
 * @param fragmentSource Source code for the fragment shader.
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
GLuint createShaderProgram(const GLchar *vertexSource, const GLchar *fragmentSource);

/**
 * @brief Initialize OpenGL resources for control point markers.
 *
 * @details
 * Initializes the vertex buffer, shader program, and default values
 * for control point markers.
 *
 * @return 0 on success, -1 on failure.
 */
int initializeControlPointMarkers();

GLuint loadTexture(cv::Mat image);

bool textureMerge(const std::string &base_img_path, const std::string &mask_img_path, const std::string &output_img_path, cv::Mat &out_merg_img);

int warpRenderWallImages();


/**
 * @brief  Entry point for the projection_calibration ROS node.
 *
 * This program initializes ROS, DevIL, and GLFW, and then enters a main loop
 * to handle image projection and calibration tasks.
 *
 * @param  argc  Number of command-line arguments.
 * @param  argv  Array of command-line arguments.
 *
 * @return 0 on successful execution, -1 on failure.
 */
int main(int, char **);

#endif