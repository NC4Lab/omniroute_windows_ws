// ######################################################################################################

// ======================================== projection_display.h ========================================

// ######################################################################################################

#ifndef _PROJECTION_DISPLAY_H
#define _PROJECTION_DISPLAY_H

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
static struct FlagStateStruct {
    bool update_textures = true;      // Flag to indicate if wall vertices, homography and texture need to be updated
    bool change_window_mode = false;  // Flag to indicate if all window modes needs to be updated
    bool windows_set_to_proj = false; // Flag to indicate if the windows are set to their respective projectors
    bool fullscreen_mode = false;     // Flag to indicate if the window is in full screen mode
    bool force_window_focus = false;  // Flag to indicate if the window should be forced into focus
} F;


/**
 * @brief  Struct for rat mask tracking and graphics.
 */
static struct RatTracker {
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
static struct ROSComm {
    std::unique_ptr<ros::NodeHandle> node_handle; // Smart pointer to ROS node handler
    std::unique_ptr<ros::Rate> loop_rate;         // Smart pointer to ros::Rate
    ros::Subscriber proj_cmd_sub;                 // ROS subscriber for projection commands
    ros::Subscriber proj_img_sub;                 // ROS subscriber for projection image data
    ros::Subscriber track_pos_sub;                // ROS subscriber for tracking rat position
} RC;

/**
 * @brief A 4x3x3x3 array container for storing the wall image configuration indices
 */
ProjWallConfigIndices4D PROJ_WALL_CONFIG_INDICES_4D;

/**
 * @brief Floor image configurations
 */
int projFloorConfigIndex = 0;

/**
 * @brief A vector of size n_projectors, where each element contains a 3x3 homography matrices for
 * the floor image transformations.
 */
std::array<cv::Mat, N_PROJ> FLOOR_HMAT_ARR;

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
std::array<std::array<std::array<std::array<cv::Mat, GLB_MAZE_SIZE>, GLB_MAZE_SIZE>, N_CAL_MODES - 1>, N_PROJ> WALL_HMAT_ARR;

/**
 * @brief Array of homography matrices for warping the rat mask marker from maze cm to ndc space for each projector.
 */
std::array<cv::Mat, N_PROJ> HMAT_CM_TO_NDC_ARR;

/**
 * @brief  Array of marker for masking rat for each projector.
 */
std::array<CircleRenderer, N_PROJ> RM_CIRCREND_ARR;

/**
 * @brief  Array of OpenGL context objects.
 */
std::vector<MazeRenderContext> PROJ_CTX_VEC(N_PROJ);

/**
 * @brief Offset for the window position
 */
std::vector<cv::Point> winOffsetVec;

// Vectors to store the raw loaded images in cv::Mat format
std::vector<cv::Mat> runtimeWallMats;  // Vector of individual wall image texture matrices
std::vector<cv::Mat> runtimeFloorMats; // Vector of individual floor image texture matrices

/**
 * @brief Blank baseline image.
 */
cv::Mat blankMat = cv::Mat::zeros(GLB_MONITOR_HEIGHT_PXL, GLB_MONITOR_WIDTH_PXL, CV_8UC4);

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
std::array<std::vector<cv::Mat>, N_PROJ> rotatedRuntimeFloorMats;

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
void callbackKeyBinding(GLFWwindow *window, int key,
    int scancode, int action, int mods);

/**
 * @brief Callback function for the "projection_cmd" topic subscription.
 *
 * @details
 * This function is called whenever a new message is received on the "projection_cmd" topic.
 * It stores the received data and flags the new message.
 *
 * @param msg Const pointer to the received message.
 */
void callbackProjCmdROS(const std_msgs::Int32::ConstPtr &msg);

/**
 * @brief Callback function for the "projection_image" topic subscription.
 *
 * @details
 * This function is called whenever a new message is received on the "projection_image" topic.
 * It stores the received data and flags the new message.
 *
 * @param msg Const pointer to the received message.
 */
void callbackProjImgROS(const std_msgs::Int32MultiArray::ConstPtr &msg);

/**
 * @brief Callback function for the "track_pose" topic subscription.
 *
 * @details
 * This function is called whenever a new message is received on the "harness_pose_in_maze" topic.
 * It stores the received data and flags the new message.
 *
 * @param msg Const pointer to the received message.
 */
void callbackTrackPosROS(const geometry_msgs::PoseStamped::ConstPtr &msg);

/**
 * @brief Simulates rat movement.
 *
 * @param move_step Distance to move in cm.
 * @param max_turn_angle Maximum angle to turn in degrees.
 */
void simulateRatMovement(
    float move_step,
    float max_turn_angle);

/**
 * @brief Sets the wall image configuration for a projector array.
 *
 * This function updates the out_PROJ_WALL_CONFIG_INDICES_4D array to set images on specified walls
 * in a chamber, considering each projector's orientation. It supports setting images for multiple walls.
 *
 * @param image_ind Index of the image to be projected.
 * @param chamber_ind Index of the target chamber.
 * @param wall_ind Index of the wall for single-wall overload.
 * @param walls_ind Vector of wall indices for multiple-wall overload.
 * @param out_PROJ_WALL_CONFIG_INDICES_4D Reference to the 4D projector configuration array.
 */

// Overload for setting an image on a single wall
void configWallImageIndex(int image_ind, int chamber_ind, int wall_ind, ProjWallConfigIndices4D &out_PROJ_WALL_CONFIG_INDICES_4D);

// Overload for setting images on multiple walls
void configWallImageIndex(int image_ind, int chamber_ind, const std::vector<int> &walls_ind, ProjWallConfigIndices4D &out_PROJ_WALL_CONFIG_INDICES_4D);

/**
 * @brief Get the vertices cooresponding to the maze boundaries in centimeters.
 *
 * @details
 * This function returns the vertices of the maze boundaries in centimeters for use
 * with the NDC to centimeter transform. Vertices are rotated based on the orientation
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
 * @param _floorMats Floor image in cv::Mat format
 * @param[out] out_img_mat Reference to store the new cv::Mat image.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int updateFloorTexture(
    int proj_ind,
    cv::Mat &_floorMats,
    cv::Mat &out_img_mat);

/**
 * @brief Applies the homography matrices to warp wall image textures and combine them into a new image.
 *
 * @param proj_ind Index of the projector associated with the given image.
 * @param do_ignore_blank_img Bool to handle blank/black imgages [true: skip; false: include]
 * @param[out] out_img_mat Reference to store the new cv::Mat image.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int updateWallTexture(
    int proj_ind,
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

class TimingData{
    public:
        ros::Time currentTime, lastTime;
        ros::Duration deltaTime, minDeltaTime, maxDeltaTime;
        std::string name;

        void setName(const std::string &_name) {
            name = _name;
        }

        void start() {
            currentTime = ros::Time::now();
            lastTime = currentTime;
        }

        void reset() {
            deltaTime = ros::Duration(0);
            minDeltaTime = ros::Duration(0.0);
            maxDeltaTime = ros::Duration(0.0);
            start();
        }
        
        void update(bool print = false) {
            currentTime = ros::Time::now();
            deltaTime = currentTime - lastTime;
            lastTime = currentTime;

            // Update min and max delta time
            if (minDeltaTime.isZero() || deltaTime < minDeltaTime)
                minDeltaTime = deltaTime;
            if (deltaTime > maxDeltaTime)
                maxDeltaTime = deltaTime;
            
            // Print timing data if requested
            if (print) printTimingData();
        }

        void printTimingData() {
            ROS_INFO("[Timer: %s] Duration: %f, Min: %f, Max: %f",
                     name.c_str(), deltaTime.toSec(), minDeltaTime.toSec(), maxDeltaTime.toSec());
        }

};
TimingData displayTimer[10];

/**
 * @brief Initializes the ROS node and sets up the subscriber for the "projection_cmd" topic.
 *
 * @details
 * This function initializes the ROS node, creates a node handle and private node handle, then sets up subscribers
 *
 * @param argc The argc argument from the main function (number of command-line arguments).
 * @param argv The argv argument from the main function (array of command-line argument strings).
 *
 * @throws std::runtime_error.
 */
void appInitROS(int argc, char **argv);

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
