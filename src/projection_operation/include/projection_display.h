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

    std::vector<int> proj_mon_vec = {1, 2, 3, 4}; // Vector of indeces of the monitor associated with each projector
    /*
    MSM: 2024-03-20
    Is not hardcoded anymore.
    = {
        2, // Projector 0
        1, // Projector 1
        4, // Projector 2
        3, // Projector 3
    };
    */

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
    const GLfloat marker_radius = 7.5f;                    // Marker default circle radius (cm)
    cv::Scalar marker_rgb = cv::Scalar(0.0f, 0.0f, 0.0f);  // Marker color (black)
    const int marker_segments = 36;                        // Number of segments used to approximate the circle geometry
    double offset_distance = 5.0f;                         // Translational offset from harness (cm)
    double offset_angle = 75.0f;                           // Rotational offset from harness (degree)
} RT;

/**
 * @brief Struct for ROS communication.
 */
struct ROSComm
{
    std::unique_ptr<ros::NodeHandle> node_handle; // Smart pointer to ROS node handler
    std::unique_ptr<ros::Rate> loop_rate;         // Smart pointer to ros::Rate
    ros::Subscriber proj_cmd_sub;                 // ROS subscriber for projection commands
    int last_proj_cmd = -1;                       // Variable to store the last command received, initialized with an invalid value
    int last_proj_img[10][10];                  // Variable to store the last image configuration received
    ros::Subscriber proj_img_sub;                 // ROS subscriber for projection commands
    bool is_proj_cmd_message_received = false;    // Flag to indicate if a projection command message has been received
    bool is_proj_img_message_received = false;  // Flag to indicate if a projection image message has been received
    ros::Subscriber track_pos_sub;                // ROS subscriber for tracking rat position
    geometry_msgs::PoseStamped last_track_pos;    // Variable to store the last tracking rat pose received
    bool is_track_message_received = false;       // Flag to indicate if a track position message has been received

    // Constructor to initialize last_proj_img to -1
    ROSComm()
    {
        for (int i = 0; i < 10; ++i)
            for (int j = 0; j < 10; ++j)
                last_proj_img[i][j] = -1;
    }
} RC;

/**
 * @brief A n_projectors vector containing a 4x3x3x3 array contianer for storring different wall image configurations
 */
std::vector<ProjWallConfigIndices4D> PROJ_WALL_CONFIG_INDICES_4D_VEC;

/**
 * @brief A n_projectors array contianer for storring different floor image configurations
 */
ProjFloorConfigIndices1D PROJ_FLOOR_CONFIG_INDICES_1D = {
    1, // Projector 0: West
    1, // Projector 1: North
    1, // Projector 2: East
    1, // Projector 3: South
};

/**
 * @brief A vector of size n_projectors, where each element contains a 3x3 homography matrices for
 * the floor image transformations.
 */
std::array<cv::Mat, 4> FLOOR_HMAT_ARR;

/**
 * @brief A vector of size n_projectors, where each element contains a 3x3x3 data container for storing 3x3 homography matrices
 * for the wall image transformations.
 *
 * @details
 * [4][3][3][3] = [number of projectors][grid rows][grid columns][calibration modes]
 *
 * - Dimension 1: Projectors [0, 1, 2, 3]
 *   - Represents different projectors, where each projector has its own set of homography matrices.
 *
 * - Dimension 2: Calibration Mode [0: left walls, 1: middle walls, 2: right walls]
 *   - Represents different calibration modes corresponding to various parts of the environment (walls and floor).
 *
 * - Dimension 3: Grid Rows [0, 1, 2]
 *   - Index with respect to the grid rows in the chamber.
 *
 * - Dimension 4: Grid Columns [0, 1, 2]
 *   - Index with respect to the grid columns in the chamber.
 *
 * The overall structure stores homography matrices (3x3 matrices) for all wall and floor images across different calibration modes
 * for each projector. Each innermost element is a cv::Mat (3x3 matrix) containing homography data for the given wall or floor image.
 */
std::array<std::array<std::array<std::array<cv::Mat, GLB_MAZE_SIZE>, GLB_MAZE_SIZE>, N_CAL_MODES - 1>, 4> WALL_HMAT_ARR;

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
    runtime_wall_image_path + "/w_black.png",    // [0] Black fill
    runtime_wall_image_path + "/w_square.png",   // [1] Square shape
    runtime_wall_image_path + "/w_circle.png",   // [2] Circle shape
    runtime_wall_image_path + "/w_triangle.png", // [3] Triangle shape
    runtime_wall_image_path + "/w_star.png",     // [4] Star shape
    runtime_wall_image_path + "/w_pentagon.png", // [5] Pentagon shape
    runtime_wall_image_path + "/w_circle.png",   // [6] Circle shape
};
/**
 * @brief List of floor image file paths
 */
std::vector<std::string> fiImgPathFloorVec = {
    runtime_wall_image_path + "/f_black.png",     // [0] Black fill
    runtime_wall_image_path + "/f_pattern_0.png", // [1] Pattern 0
    runtime_wall_image_path + "/f_pattern_1.png", // [2] Pattern 1
    runtime_wall_image_path + "/f_pattern_2.png", // [3] Pattern 2
    runtime_wall_image_path + "/f_white.png",     // [4] White
};

// Vectors to store the raw loaded images in cv::Mat format
std::vector<cv::Mat> wallRawImgMatVec;  // Vector of indevidual wall image texture matrices
std::vector<cv::Mat> floorRawImgMatVec; // Vector of indevidual floor image texture matrices

/**
 * @brief Array to store the image of all blank walls to use as the
 * baseline image.
 */
std::array<cv::Mat, 4> wallBlankImgMatArr;

/**
 * @brief Array of vectors to store the rotated floor images in cv::Mat format
 * for each projector.
 *
 * @details
 * [4][N] = [number of projectors][number of images]
 *
 * - Dimension 1: Projectors [0, 1, 2, 3]
 *
 * - Dimension 2: Image N
 */
std::array<std::vector<cv::Mat>, 4> floorRotatedImgMatVecArr;

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
void callbackProjCmdROS(const std_msgs::Int32::ConstPtr &msg, ROSComm *out_RC);

/**
 * @brief Callback function for the "projection_image" topic subscription.
 *
 * @details
 * This function is called whenever a new message is received on the "projection_image" topic.
 * It stores the received data and flags the new message.
 *
 * @param msg Const pointer to the received message.
 * @param out_RC Pointer to the ROSComm struct where the last command and message received flag are updated.
 */
void callbackProjImageROS(const std_msgs::Int32MultiArray::ConstPtr &msg, ROSComm *out_RC);

/**
 * @brief Callback function for the "track_pose" topic subscription.
 *
 * @details
 * This function is called whenever a new message is received on the "harness_pose_in_maze" topic.
 * It stores the received data and flags the new message.
 *
 * @param msg Const pointer to the received message.
 * @param out_RC Pointer to the ROSComm struct where the last command and message received flag are updated.
 */
void callbackTrackPosROS(const geometry_msgs::PoseStamped::ConstPtr &msg, ROSComm *out_RC);

/**
 * @brief Initializes the ROS subscriber for the "projection_cmd" topic within the given ROSComm structure.
 *
 * @details
 * This function sets up a subscriber to the "projection_cmd" topic, which receives Int32 messages.
 * The received command updates the last_proj_cmd field in the ROSComm struct and sets
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
 * This function checks if a new projection message has been received and processes the message.
 *
 * @param[out] out_RC Reference to the ROSComm struct containing the ROS coms data.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int procProjMsgROS(ROSComm &out_RC);

/**
 * @brief Processes commands received from the "harness_pose_in_maze" topic.
 *
 * @details
 * This function checks if a new position tracking message has been received and processes the message.
 *
 * @param[out] out_RC Reference to the ROSComm struct containing the ROS coms data.
 * @param[out] out_RT Rat tracker struct object to be updated.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int procTrackMsgROS(ROSComm &out_RC, RatTracker &out_RT);

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
 * @brief Sets the wall image configuration for a projector array.
 *
 * This function updates the out_PROJ_WALL_CONFIG_INDICES_4D_VEC array to set images on specified walls
 * in a chamber, considering each projector's orientation. It supports setting images for multiple walls.
 *
 * @param image_ind Index of the image to be projected.
 * @param chamber_ind Index of the target chamber.
 * @param wall_ind Index of the wall for single-wall overload.
 * @param walls_ind Vector of wall indices for multiple-wall overload.
 * @param out_PROJ_WALL_CONFIG_INDICES_4D_VEC Reference to the 4D projector configuration array.
 */

// Overload for setting an image on a single wall
void configWallImageIndex(int image_ind, int chamber_ind, int wall_ind, ProjWallConfigIndices4D &out_PROJ_WALL_CONFIG_INDICES_4D_VEC);

// Overload for setting images on multiple walls
void configWallImageIndex(int image_ind, int chamber_ind, const std::vector<int> &walls_ind, ProjWallConfigIndices4D &out_PROJ_WALL_CONFIG_INDICES_4D_VEC);

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
void computeMazeVertCm(int proj_ind, std::vector<cv::Point2f> &maze_vert_cm_vec);

/**
 * @brief Rotate the floor image texture for a given projector.
 *
 * @param img_rot_deg Rotation angle in degrees which must be in incriments of 90.
 * @param in_img_mat Input image matrix.
 * @param[out] out_img_mat_vec Reference to a vector of cv::Mat where rotated images will be stored.
 */
void rotateFloorImage(
    int img_rot_deg,
    const cv::Mat &in_img_mat,
    std::vector<cv::Mat> &out_img_mat_vec);

/**
 * @brief Applies the homography matrices to warp floor image textures.
 *
 * @param proj_ind Index of the projector associated with the given image.
 * @param _floorRawImgMatVec Vectors containing the loaded floor images in cv::Mat format
 * @param _wallBlankImgMat Blank walls image in cv::Mat format
 * @param _PROJ_FLOOR_CONFIG_INDICES_1D 1D array of floor image indices for each projector.
 * @param _FLOOR_HMAT_ARR Array of homography matrices for the floor image transformations.
 * @param[out] out_img_mat Reference to store the new cv::Mat image.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int updateFloorTexture(
    int proj_ind,
    const std::vector<cv::Mat> &_floorRawImgMatVec,
    const cv::Mat _wallBlankImgMat,
    const ProjFloorConfigIndices1D &_PROJ_FLOOR_CONFIG_INDICES_1D,
    std::array<cv::Mat, 4> &_FLOOR_HMAT_ARR,
    cv::Mat &out_img_mat);

/**
 * @brief Applies the homography matrices to warp wall image textures and combine them into a new image.
 *
 * @param proj_ind Index of the projector associated with the given image.
 * @param _wallRawImgMatVec Vectors containing the loaded wall images in cv::Mat format
 * @param _PROJ_WALL_CONFIG_INDICES_4D Multidimensional array of walll image indices.
 * @param _WALL_HMAT_ARR Big ass ugly array of arrays of arrays of matrices!
 * @param do_ignore_blank_img Bool to handle blank/black imgages [true: skip; false: include]
 * @param[out] out_img_mat Reference to store the new cv::Mat image.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int updateWallTexture(
    int proj_ind,
    const std::vector<cv::Mat> &_wallRawImgMatVec,
    const ProjWallConfigIndices4D &_PROJ_WALL_CONFIG_INDICES_4D,
    const std::array<std::array<std::array<std::array<cv::Mat, GLB_MAZE_SIZE>, GLB_MAZE_SIZE>, N_CAL_MODES - 1>, 4> &_WALL_HMAT_ARR,
    bool do_ignore_blank_img,
    cv::Mat &out_img_mat);

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
