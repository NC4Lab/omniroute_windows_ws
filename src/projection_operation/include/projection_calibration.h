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
 * @brief Shader program IDs for wall image and control point marker rendering.
 *
 * @details
 * - `WALL_SHADER`: Shader program used for rendering the warped wall images.
 * - `CTRL_PT_SHADER`: Shader program used for rendering the control point markers as circles.
 */
GLuint WALL_SHADER;
GLuint CTRL_PT_SHADER;

/**
 * @brief Global Element Buffer Object (EBO) for wall images and control point markers.
 *
 * @details
 * - 'WALL_EBO': Element Buffer Object (EBO) containing the index data for rendering the wall images.
 *      Initialized at the beginning of the program and remains constant.
 * - 'CTRL_PT_EBO': Element Buffer Object (EBO) containing the index data for rendering the control point markers.
 *      Initialized at the beginning of the program and remains constant.
 */
GLuint WALL_EBO;
GLuint CTRL_PT_EBO;

/**
 * @brief Global Vertex Array Object (VAO) arrays for wall images and control point markers.
 *
 * @details
 * - 'WALL_VAO_ARR': 2D array of Vertex Array Objects (VAOs) that manage vertex attributes for each wall image.
 *      Each VAO is initialized at the beginning of the program.
 * - 'CTRL_PT_VAO_ARR': 2D array of Vertex Array Objects (VAOs) that manage vertex attributes for each control point marker.
 *      Each VAO is initialized at the beginning of the program.
 */
std::array<std::array<GLuint, MAZE_SIZE>, MAZE_SIZE> WALL_VAO_ARR;
std::array<std::array<GLuint, 4>, 4> CTRL_PT_VAO_ARR;

/**
 * @brief Global Vertex Buffer Object (VBO) arrays for wall images and control point markers.
 *
 * @details
 * - 'WALL_VBO_ARR': Array of VBO that contain the vertex data for rendering and transforming
 *      the wall images. It is initialized at the beginning of the program and
 *      updated as needed.
 * - 'CTRL_PT_VBO_ARR':  Array of VBO that contain the vertex data for rendering the control
 *      point markers (circles). Like WALL_VBO_ARR, it is initialized at the start
 *      and updated as necessary.
 */
std::array<std::array<GLuint, MAZE_SIZE>, MAZE_SIZE> WALL_VBO_ARR;
std::array<std::array<GLuint, 4>, 4> CTRL_PT_VBO_ARR;

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
 *
 * @details
 * This is a GLSL (OpenGL Shading Language) vertex shader source code stored as a C++ raw string literal.
 * - `#version 330 core`: Specifies that the GLSL version is 3.30 and we're using the core profile.
 * - `in vec2 position;`: Declares an input vertex attribute called `position`. Receives vertex coordinates from the application.
 * - `in vec2 texcoord;`: Declares another input vertex attribute called `texcoord`. Receives texture coordinates from the application.
 * - `out vec2 Texcoord;`: Declares an output variable that will be passed to the fragment shader.
 * - `void main() { ... }`: Main function where the vertex shader performs its work.
 */
const char *vertexSource = R"glsl(
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
const char *fragmentSource = R"glsl(
    #version 330 core
    in vec2 Texcoord;
    out vec4 outColor;
    uniform sampler2D tex;
    void main() {
        outColor = texture(tex, Texcoord);
    }
)glsl";

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
std::array<std::array<cv::Point2f, 4>, 4> CTRL_PT_COORDS;

/**
 * @brief  MAZE_SIZExMAZE_SIZEx4 data container for storing the final warped wall vertex coordinates in Normalized Device Coordinates (NDC)
 *
 * The array stores the all wall verteces as OpenGL's Normalized Device Coordinates (NDC), which range from [-1, 1]
 * with the origin at the center of the screen.
 *
 * @details
 * [MAZE_SIZE][MAZE_SIZE][4] = [row][col][vertex]
 *
 * - Dimension 1: Row in maze grid [0-MAZE_SIZE-1]:
 *  - Top to Bottom
 *
 * - Dimension 2: Column in maze grid [0-MAZE_SIZE-1]:
 *  - Left to Right
 *
 * - Dimension 3: Vertex(x, y) [0, 1, 2, 3]
 *  - 0: Top-left  (quadrilateral vertex)
 *  - 1: Top-right  (quadrilateral vertex)
 *  - 2: Bottom-right  (quadrilateral vertex)
 *  - 3: Bottom-left  (quadrilateral vertex)
 */
std::array<std::array<std::array<cv::Point2f, 4>, MAZE_SIZE>, MAZE_SIZE> WALL_WARP_COORDS;

/**
 * @brief  3x3 data contianer for storing the 3x3 homography matrix for each wall image
 */
std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> WALL_HMAT_DATA;

// Global variable to set the OpenGL debug level.
int DEBUG_LEVEL_GL = 3;  // [0: None, 1: >=Default 2: >=Low, 3: >=Medium, 4: High]

// Struct primarely used for tracking flags set in the keyboard callback
static struct FlagStruct
{
    bool dbRun = false;                   // Flag to indicate if something should be run for debugging
    bool loadXML = false;                 // Flag to indicate if the XML file needs to be loaded
    bool saveXML = false;                 // Flag to indicate if the XML file needs to be saved
    bool updateWindowMonMode = false;     // Flag to indicate if the window mode needs to be updated
    bool initControlPointMarkers = false; // Flag to indicate if the control point markers need to be reinitialized
    bool updateWallDatasets = false;      // Flag to indicate if the warped wall vertices need to be updated
} F;

// Selected control point
std::array<int, 2> cpSelectedInd = {0, 0}; // Index: maze_corner[0,1] wall_vertex[0,1,2,3]

// Control point graphics
std::array<float, 3> cpVertSelectedRGB = {0.0f, 1.0f, 0.0f}; // Select control point marker color (green)
std::array<float, 3> cpWallSelectedRGB = {1.0f, 0.0f, 0.0f}; // Selected control point wall color (red)
std::array<float, 3> cpInactiveRGB = {0.0f, 0.0f, 1.0f};    // Inactive control point marker color (blue)

// Control point image radius
const std::array<float, 2> cpMakerRadius = {0.0025f, 0.005f};

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
 * *
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
 * @note The global variables monitor, monitors, imgMonNumInd, window, and isFullScreen are
 *       used to control the behavior of this function.
 *       Will only exicute if monotor parameters have changed.
 */
int updateWindowMonMode(GLFWwindow *, int, GLFWmonitor **&, int, bool);

/**
 * @brief Draws a control point as a quadrilateral using OpenGL.
 *
 * @details
 * This function uses OpenGL to draw a quadrilateral that represents a control point.
 * The control point is drawn as a colored control point marker.
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
 * @param fbo_texture_id OpenGL framebuffer object's (FBO) texture ID.
 * @param tex_wall_id DevIL image ID for the base wall image.
 * @param tex_mode_mon_id DevIL image ID for the monitor mode image.
 * @param tex_mode_cal_id DevIL image ID for the calibration image.
 *
 * @return Integer status code  [0:successful, -1:error].
 *
 * @details
 * This function is a core part of the maze visualization pipeline. It utilizes the OpenGL graphics library for rendering,
 * and the DevIL library for image handling. The function performs several key operations:
 * 1. Texture mapping of the wall images.
 * 2. Perspective warping based on a precomputed homography matrix.
 * 3. Shear and height adjustments based on control point calibration.
 * 4. Optional overlay of status images for cells corresponding to selected control points.
 */
int drawWallImages(GLuint, ILuint, ILuint, ILuint);

/**
 * @brief Initialize OpenGL resources for render objects (wall images and control point markers).
 *
 * @return Integer status code [0:successful, -1:error].
 *
 * @details
 * Initializes the vertex buffer, shader program, and default values
 * for the wall images and control point markers.
 */
int initializeOpenGLObjects();

/**
 * @brief Creates an OpenGL shader program from vertex and fragment shader source code.
 *
 * @param vertex_source Source code for the vertex shader.
 * @param fragment_source Source code for the fragment shader.
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
GLuint createShaderProgram(const GLchar *, const GLchar *);

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
 * @brief Merges a mask image over a base image and stores the result in an output image.
 *
 * @param base_img_path Path to the base image.
 * @param mask_img_path Path to the mask image.
 * @param output_img_path Path where the merged image will be saved.
 * @param out_merg_img Output cv::Mat containing the merged image.
 *
 * @return true on successful merge, false otherwise.
 *
 * @details
 * This function merges a mask image over a base image. Both images are read from
 * their respective paths. The function then overlays the mask image on top of the
 * base image. If the mask pixel is not white, it is overlaid onto the base image.
 * The merged image is stored in an output cv::Mat. The function returns true if
 * the merge operation is successful and false otherwise.
 */

bool textureMerge(const std::string &, const std::string &, const std::string &, cv::Mat &);

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