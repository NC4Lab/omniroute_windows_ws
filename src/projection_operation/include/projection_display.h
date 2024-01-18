// ######################################################################################################

// ======================================== projection_display.h ========================================

// ######################################################################################################

#ifndef _PROJECTION_DISPLY_H
#define _PROJECTION_DISPLY_H

// ================================================== INCLUDE ==================================================

// Local custom libraries
#include "projection_utils.h"

// ================================================== VARIABLES ==================================================

/**
 * @brief Struct for flaging state changes.
 *
 * @details Flag update_textures is initialized as true to force the
 * initial update of the displayed texture.
 */
static struct FlagStateStruct
{
    bool update_textures = true;      // Flag to indicate if wall vertices, homography and texture need to be updated
    bool change_window_mode = false;  // Flag to indicate if all window modes needs to be updated
    bool windows_set_to_proj = false; // Flag to indicate if the windows are set to their respective projectors
    bool fullscreen_mode = false;     // Flag to indicate if the window is in full screen mode
    bool force_window_focus = false;  // Flag to indicate if the window should be forced into focus
} F;

/**
 * @brief Struct for global indices.
 */
static struct IndStruct
{
    const int starting_monitor = 0; // Default starting monitor index for the windows (hardcoded)
    const std::vector<int> proj_mon_vec = {
        2,                  // Projector 0
        1,                  // Projector 1
        4,                  // Projector 2
        3,                  // Projector 3
    };                      // Vector of indeces of the monitor associeted to each projector (hardcoded)
    int wall_image_cfg = 0; // Index of the curren wall image configuration
} I;

/**
 * @brief Struct for global counts
 */
static struct CountStruct
{
    int monitor;                                                   // Number of monitors connected to the system
    const int projector = static_cast<int>(I.proj_mon_vec.size()); // Number of projectors
    const int wall_image = 6;                                      // Number of wall images
} N;

/**
 * @brief  Struct for rat mask tracking and graphics.
 */
static struct RatTracker
{
    cv::Point2f marker_position = cv::Point2f(0.0f, 0.0f); // Marker center (cm)
    const GLfloat marker_radius = 5.0f;                    // Marker default circle radius (cm)
    cv::Scalar marker_rgb = cv::Scalar(1.0f, 0.0f, 0.0f);  // Marker color (black)
    const int marker_segments = 36;                        // Number of segments used to approximate the circle geometry
} RT;

/**
 * @brief Struct for ROS communication.
 */
struct ROSComm
{
    std::unique_ptr<ros::NodeHandle> node_handle; // Smart pointer to ROS node handler
    std::unique_ptr<ros::Rate> loop_rate;         // Smart pointer to ros::Rate
    ros::Subscriber projection_cmd_sub;           // ROS subscriber
    ros::Subscriber harness_pose_sub;              // ROS subscriber
    int last_projection_cmd = -1;                 // Variable to store the last command received, initialize with an invalid value
    geometry_msgs::PoseStamped last_harness_pose; // Variable to store the last harness pose received
    bool is_message_received = false;             // Flag to indicate if a message has been received
} RC;

/**
 * @brief A n_projectors vector containing a 4x3x3x3 array contianer for storring different wall image configurations
 */
std::vector<ProjWallImageCfg4D> PROJ_WALL_IMAGE_CFG_4D_VEC;

/**
 * @brief A n_projectors array contianer for storring different floor image configurations
 */
ProjFloorImageCfg1D PROJ_FLOOR_IMAGE_CFG_1D = {
    1, // Projector 0: West
    1, // Projector 1: North
    1, // Projector 2: East
    1, // Projector 3: South
};

/**
 * @brief A n_projectors sized element veoctor containing a 3x3x3 data contianer for storing 3x3 homography matrices (UGLY!)
 */
std::array<std::array<std::array<std::array<cv::Mat, GLB_MAZE_SIZE>, GLB_MAZE_SIZE>, N_CAL_MODES>, 4> HMAT_ARR;

/**
 * @brief Array of homography matrices for warping the rat mask marker from maze cm to ndc space for each projector.
 */
std::array<cv::Mat, 4> HMAT_CM_TO_NDC_ARR;

/**
 * @brief  Array of marker for masking rat for each projector.
 */
std::array<CircleRenderer, 4> RM_CIRCREND_ARR;

/**
 * @brief  Array of OpenGL context objects.
 */
std::vector<MazeRenderContext> PROJ_CTX_VEC(N.projector);

/**
 * @brief Offset for the window position
 */
std::vector<cv::Point> winOffsetVec;

/**
 * @brief Image file sub-directory path
 */
std::string runtime_wall_image_path = GLB_IMAGE_TOP_DIR_PATH + "/runtime";

/**
 * @brief List of wall image file paths
 */
std::vector<std::string> fiImgPathWallVec = {
    runtime_wall_image_path + "/w_blank.png",    // [0] Blank shape
    runtime_wall_image_path + "/w_square.png",   // [1] Square shape
    runtime_wall_image_path + "/w_circle.png",   // [2] Circle shape
    runtime_wall_image_path + "/w_triangle.png", // [3] Triangle shape
    runtime_wall_image_path + "/w_star.png",     // [4] Star shape
    runtime_wall_image_path + "/w_pentagon.png", // [5] Pentagon shape
};
/**
 * @brief List of floor image file paths
 */
std::vector<std::string> fiImgPathFloorVec = {
    runtime_wall_image_path + "/f_black.png",  // [0] Black
    runtime_wall_image_path + "/f_gray_0.png", // [1] Gray (20%)
    runtime_wall_image_path + "/f_gray_1.png", // [2] Gray (40%)
    runtime_wall_image_path + "/f_gray_2.png", // [3] Gray (60%)
    runtime_wall_image_path + "/f_gray_3.png", // [4] Gray (80%)
    runtime_wall_image_path + "/f_white.png",  // [5] White
};

// Vectors to store the loaded images in cv::Mat format
std::vector<cv::Mat> wallImgMatVec;  // Vector of wall image texture matrices
std::vector<cv::Mat> floorImgMatVec; // Vector of floor image texture matrices

// ================================================== FUNCTIONS ==================================================

/**
 * @brief GLFW key callback function to handle key events and execute corresponding actions.
 *
 * @details
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
void callbackKeyBinding(
    GLFWwindow *window,
    int key,
    int scancode,
    int action,
    int mods);

/**
 * @brief Callback function for the "projection_cmd" topic subscription.
 *
 * @details
 * This function is called whenever a new message is received on the "projection_cmd" topic.
 * It stores the received data and flags the new message.
 *
 * @param msg Const pointer to the received message.
 * @param out_RC Pointer to the ROSComm struct where the last command and message received flag are updated.
 */
void callbackCmdROS(const std_msgs::Int32::ConstPtr &msg, ROSComm *out_RC);

/**
 * @brief Callback function for the "harness_pose" topic subscription.
 *
 * @details
 * This function is called whenever a new message is received on the "harness_pose_in_maze" topic.
 * It stores the received data and flags the new message.
 *
 * @param msg Const pointer to the received message.
 * @param out_RC Pointer to the ROSComm struct where the last command and message received flag are updated.
 */
void callbackHarnessPoseROS(const geometry_msgs::PoseStamped::ConstPtr &msg, ROSComm *out_RC);

/**
 * @brief Initializes the ROS subscriber for the "projection_cmd" topic within the given ROSComm structure.
 *
 * @details
 * This function sets up a subscriber to the "projection_cmd" topic, which receives Int32 messages.
 * The received command updates the last_projection_cmd field in the ROSComm struct and sets
 * a flag indicating a message has been received.
 *
 * @param out_RC Reference to the ROSComm struct that holds the ROS node handle and subscriber.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int initSubscriberROS(ROSComm &out_RC);

/**
 * @brief Processes commands received from the "projection_cmd" topic.
 *
 * @details
 * This function checks if a new message has been received adn logs the command and resets the flag.
 *
 * @param out_RC Reference to the ROSComm struct containing the last received command and message flag.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int procCmdROS(ROSComm &out_RC);

/**
 * @brief Simulates rat movement.
 *
 * @param move_step Distance to move in cm.
 * @param max_turn_angle Maximum angle to turn in degrees.
 * @param[out] out_RT RatTracker struct object to be updated.
 */
void simulateRatMovement(
    float move_step,
    float max_turn_angle,
    RatTracker &out_RT);

/**
 * @brief 
 * 
 * @param harness_pose Position and orientation of the rat harness in the maze (in m). 
 * @param out_RT Rat tracker struct object to be updated.
 */
void placeRatTracker(
    geometry_msgs::PoseStamped harness_pose,
    RatTracker &out_RT);

/**
 * @brief Add a new projector image configuration to the vector.
 *
 * @details
 * This function adds a new 4D array configuration to a vector of existing configurations based on the specified projector direction.
 * It initializes a new configuration with all elements set to zero, then sets specific 'left' and 'right' values in the center cell of the
 * middle row for the primary and secondary projectors. The primary and secondary projectors are determined by the direction parameter.
 * The new configuration is then added to the provided vector.
 *
 * @param direction The direction the rat is facing at the choice point ("east", "north", "west", or "south").
 * @param left_shape_ind The index of the image to use the 'left' wall of the center cell of the middle row.
 * @param right_shape_ind The index of the image to use the 'right' wall of the center cell of the middle row.
 * @param[out] out_PROJ_WALL_IMAGE_CFG_4D_VEC Vector of ProjWallImageCfg4D projector image configurations.
 */
void addImageConfiguration(const std::string &direction,
                           int left_shape_ind,
                           int right_shape_ind,
                           std::vector<ProjWallImageCfg4D> &out_PROJ_WALL_IMAGE_CFG_4D_VEC);

/**
 * @brief Get the vertices cooresponding to the maze boundaries in centimeters.
 *
 * @details
 * This function returns the vertices of the maze boundaries in centimeters for use
 * with the NCD to centimeter transform. Vertices are rotated based on the orientation
 * of a given projector.
 *
 * @param proj_ind Index of the projector.
 * @param[out] maze_vert_cm_vec Vector of maze vertices.
 */
void populateMazeVertNdcVec(int proj_ind, std::vector<cv::Point2f> &maze_vert_cm_vec);

/**
 * @brief Applies the homography matrices to warp wall image textures and combine them.
 *
 * @param _wallImgMatVec Vectors containing the loaded wall images in cv::Mat format
 * @param _floorImgMatVec Vectors containing the loaded floor images in cv::Mat format
 * @param _PROJ_WALL_IMAGE_CFG_3D 3D array of image indices for each projector, wall, and calibration mode.
 * @param _PROJ_FLOOR_IMAGE_CFG_1D 1D array of floor image indices for each projector.
 * @param _HMAT_ARR Big ass ugly array of arrays of matrices of shit!
 * @param[out] out_projCtx MazeRenderContext OpenGL context handler.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int updateTexture(
    const std::vector<cv::Mat> &_wallImgMatVec,
    const std::vector<cv::Mat> &_floorImgMatVec,
    const ProjWallImageCfg4D &_PROJ_WALL_IMAGE_CFG_3D,
    const ProjFloorImageCfg1D &_PROJ_FLOOR_IMAGE_CFG_1D,
    const std::array<std::array<std::array<std::array<cv::Mat, GLB_MAZE_SIZE>, GLB_MAZE_SIZE>, N_CAL_MODES>, 4> &_HMAT_ARR,
    MazeRenderContext &out_projCtx);

/**
 * @brief Draws control points associated with each corner wall.
 *
 * @param _RT RatTracker struct object.
 * @param[out] out_rmCircRend CircleRenderer objects used to draw the control points.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int drawRatMask(
    const RatTracker &_RT,
    CircleRenderer &out_rmCircRend);

/**
 * @brief Initializes the ROS node and sets up the subscriber for the "projection_cmd" topic.
 *
 * @details
 * This function initializes the ROS node, creates a node handle and private node handle, then calls
 * initSubscriberROS to set up the subscriber.
 *
 * @param argc The argc argument from the main function (number of command-line arguments).
 * @param argv The argv argument from the main function (array of command-line argument strings).
 * @param out_RC Reference to the ROSComm struct to be used for storing the node handle and subscriber.
 *
 * @throws std::runtime_error.
 */
void appInitROS(int argc, char **argv, ROSComm &out_RC);

/**
 * @brief Loads the necessary images and data for the application.
 *
 * This function uses OpenCV to load wall images, homography matrices,
 * control point vertices and floor vertices.
 *
 * @throws std::runtime_error if image loading fails.
 */
void appLoadAssets();

/**
 * @brief Initializes the variables for the application.
 *
 * @details
 * This function initializes several veriables including wall configuration
 * arrays.
 *
 * @throws std::runtime_error.
 */
void appInitVariables();

/**
 * @brief Initializes OpenGL settings and creates shader programs.
 *
 * @details
 * This function sets up the graphics libraries, initializes the rendering
 * context, and creates shader programs for wall image and control point rendering.
 *
 * @throws std::runtime_error if OpenGL initialization fails.
 */
void appInitOpenGL();

/**
 * @brief The main loop of the application.
 *
 * @details
 * Handles the application's main loop, including checking keyboard callbacks,
 * updating window mode, and rendering frames. Exits on window close, escape key,
 * or when an error occurs.
 *
 * @throws std::runtime_error if an error occurs during execution.
 */
void appMainLoop();

/**
 * @brief Cleans up resources upon application shutdown.
 *
 * @details
 * This function deletes the CircleRenderer class shader program, cleans up
 * OpenGL wall image objects, and terminates the graphics library.
 */
void appCleanup();

/**
 * @brief  Entry point for the projection_display ROS node.
 *
 * @details
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
