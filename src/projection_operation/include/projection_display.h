// ######################################################################################################

// ======================================== projection_display.h ========================================

// ######################################################################################################

#ifndef _PROJECTION_DISPLY_H
#define _PROJECTION_DISPLY_H

// ================================================== INCLUDE ==================================================

// Local custom libraries
#include "projection_utils.h"

// ================================================== VARIABLES ==================================================
// Initialize blank wall image mat
const cv::Mat WALL_BLANK_IMG_MAT = cv::Mat::zeros(GLB_MONITOR_HEIGHT_PXL, GLB_MONITOR_WIDTH_PXL, CV_8UC4); // Initialize cv::Mat

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
    geometry_msgs::PoseStamped track_pos_data;    // Variable to store the last tracking rat pose received
} RC;

/**
 * @brief Array of homography matrices for warping the rat mask marker from maze cm to ndc space for each projector.
 */
std::array<cv::Mat, N_PROJ> HMAT_CM_TO_NDC_ARR;

/**
 * @brief  Array of OpenGL context objects.
 */
std::array<MazeRenderContext, N_PROJ> PROJ_CTX_VEC;

/**
 * @brief Offset for the window position
 */
std::vector<cv::Point> winOffsetVec;

// Vectors to store the raw loaded images in cv::Mat format
std::vector<cv::Mat> runtimeWallMats;  // Vector of individual wall image texture matrices
std::vector<cv::Mat> runtimeFloorMats; // Vector of individual floor image texture matrices

// Vectors to store precomputed warped images
const int N_WARPED_WALL_IMAGES = 3;
std::array<ProjectionMap<cv::Mat>, N_WARPED_WALL_IMAGES> WARPED_RUNTIME_WALL_MATS;  // Vector of warped wall image texture matrices

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
 * @brief Initializes the ROS subscriber for the "projection_cmd" topic within the given ROSComm structure.
 *
 * @details
 * This function sets up a subscriber to the "projection_cmd" topic, which receives Int32 messages.
 * The received command updates the proj_cmd_data field in the ROSComm struct and sets
 * a flag indicating a message has been received.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int initSubscriberROS();

/**
 * @brief Simulates rat movement.
 *
 * @param move_step Distance to move in cm.
 * @param max_turn_angle Maximum angle to turn in degrees.
 */
void simulateRatMovement(float move_step, float max_turn_angle);

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
 * @brief Applies the homography matrices to warp floor image textures.
 *
 * @param proj_ind Index of the projector associated with the given image.
 * @param do_ignore_blank_img Bool to handle blank/black imgages [true: skip; false: include]
 * @param[out] out_img_mat Reference to store the new cv::Mat image.
 *
 */
void updateFloorTexture(int proj_ind, bool do_ignore_blank_img, bool update_all, cv::Mat &out_img_mat);

/**
 * @brief Applies the homography matrices to warp wall image textures and combine them into a new image.
 *
 * @param proj_ind Index of the projector associated with the given image.
 * @param do_ignore_blank_img Bool to handle blank/black imgages [true: skip; false: include]
 * @param[out] out_img_mat Reference to store the new cv::Mat image.
 *
 */
void updateWallTexture(int proj_ind, bool do_ignore_blank_img, bool udpate_all, cv::Mat &out_img_mat);

/**
 * @brief Draws control points associated with each corner wall.
 *
 * @param[out] out_rmCircRend Pointer to CircleRenderer object
 *
 */
void drawRatMask(CircleRenderer *out_rmCircRend);

bool TIMING_ENABLED = true;
class TimingData {
    public:
        ros::Time currentTime, lastTime;
        ros::Duration deltaTime, minDeltaTime, maxDeltaTime;
        std::string name;

        void reset() {
            if (!TIMING_ENABLED) return;
            deltaTime = ros::Duration(0);
            minDeltaTime = ros::Duration(0.0);
            maxDeltaTime = ros::Duration(0.0);
            name = "";
            start();
        }

        void start() {
            currentTime = ros::Time::now();
            lastTime = currentTime;
        }
        
        void update(bool print = false) {
            if (!TIMING_ENABLED) return;
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
            if (!TIMING_ENABLED) return;
            ROS_INFO("[%s] Delta: %f, Min Delta: %f, Max Delta: %f",
                     name.c_str(), deltaTime.toSec(), minDeltaTime.toSec(), maxDeltaTime.toSec());
        }

};
TimingData displayTimer[10]; // Array of TimingData objects for different parts of the application

/**
 * @brief Initializes the ROS node and sets up the subscriber for the "projection_cmd" topic.
 *
 * @details
 * This function initializes the ROS node, creates a node handle and private node handle, then calls
 * initSubscriberROS to set up the subscriber.
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
 * @return Integer status code [<0:error, 0:successful, >0:exit request].
 */
int appMainLoop();

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
