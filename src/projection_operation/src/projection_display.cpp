// ########################################################################################################

// ======================================== projection_display.cpp ========================================

// ########################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_display.h"

// ================================================== FUNCTIONS ==================================================

void callbackKeyBinding(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    // Set the current OpenGL context to the window
    glfwMakeContextCurrent(window);

    // _______________ ANY KEY RELEASE ACTION _______________

    if (action == GLFW_RELEASE)
    {

        // ---------- Set/unset Fullscreen [F] ----------

        if (key == GLFW_KEY_F)
        {
            F.fullscreen_mode = !F.fullscreen_mode;
            F.change_window_mode = true;
        }

        // ---------- Check for change window monitor command [M] ----------

        if (key == GLFW_KEY_M)
        {
            F.windows_set_to_proj = !F.windows_set_to_proj;
            F.change_window_mode = true;
        }

        // ---------- Force Window to Top of UI Stack [T] ----------

        if (key == GLFW_KEY_T)
        {
            F.force_window_focus = true;
        }
    }

    // _______________ ANY KEY PRESS OR REPEAT ACTION _______________
    else if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        static int wall_img_ind_last = -1;
        static int floor_img_ind_last = -1;

        // ---------- Change wall configuration [SHIFT [0-8]] ----------
        if (mods & GLFW_MOD_SHIFT)
        {
            int wall_img_ind = wall_img_ind_last;
            if (key == GLFW_KEY_0)
            {
                wall_img_ind = 0;
            }
            else if (key == GLFW_KEY_1 && wallRawImgMatVec.size() > 1)
            {
                wall_img_ind = 1;
            }
            else if (key == GLFW_KEY_2 && wallRawImgMatVec.size() > 2)
            {
                wall_img_ind = 2;
            }
            else if (key == GLFW_KEY_3 && wallRawImgMatVec.size() > 3)
            {
                wall_img_ind = 3;
            }
            else if (key == GLFW_KEY_4 && wallRawImgMatVec.size() > 4)
            {
                wall_img_ind = 4;
            }
            else if (key == GLFW_KEY_5 && wallRawImgMatVec.size() > 5)
            {
                wall_img_ind = 5;
            }
            // Check for configuration change
            if (wall_img_ind != wall_img_ind_last)
            {
                ROS_INFO("[callbackKeyBinding] Initiated change wall image configuration from %d to %d", wall_img_ind_last, wall_img_ind);
                
                // Set the flag to update the textures
                F.update_textures = true;

                // Loop through the data and set all entries to a given wall ind
                for (int i = 0; i < 4; ++i)
                {
                    for (int j = 0; j < 3; ++j)
                    {
                        for (int k = 0; k < 3; ++k)
                        {
                            for (int l = 0; l < 3; ++l)
                            {
                                PROJ_WALL_CONFIG_INDICES_4D[i][j][k][l] = wall_img_ind; // Set each element to 1
                            }
                        }
                    }
                }
                wall_img_ind_last = wall_img_ind;
            }
        }

        // ---------- Change floor configuration [CTRL [0-5]] ----------
        else if (mods & GLFW_MOD_CONTROL)
        {
            int floor_img_ind = floor_img_ind_last;
            if (key == GLFW_KEY_0)
            {
                floor_img_ind = 0;
            }
            else if (key == GLFW_KEY_1 && floorRotatedImgMatVecArr.size() > 1)
            {
                floor_img_ind = 1;
            }
            else if (key == GLFW_KEY_2 && floorRotatedImgMatVecArr.size() > 2)
            {
                floor_img_ind = 2;
            }
            else if (key == GLFW_KEY_3 && floorRotatedImgMatVecArr.size() > 3)
            {
                floor_img_ind = 3;
            }
            else if (key == GLFW_KEY_4 && floorRotatedImgMatVecArr.size() > 4)
            {
                floor_img_ind = 4;
            }
            else if (key == GLFW_KEY_5 && floorRotatedImgMatVecArr.size() > 5)
            {
                floor_img_ind = 5;
            }

            // Check for configuration change
            if (floor_img_ind != floor_img_ind_last)
            {
                ROS_INFO("[callbackKeyBinding] Initiated change floor image configuration from %d to %d", floor_img_ind_last, floor_img_ind);
                // Set the flag to update the textures
                F.update_textures = true;
                // Update index
                projFloorConfigIndex = floor_img_ind;
                floor_img_ind_last = floor_img_ind;
            }
        }
    }
}

void callbackProjCmdROS(const std_msgs::Int32::ConstPtr &msg, ROSComm *out_RC)
{
    // Update the last received command
    out_RC->proj_cmd_data = msg->data;
    out_RC->is_proj_cmd_message_received = true;

    // Log projection command
    if (GLB_DO_VERBOSE_DEBUG)
        ROS_INFO("[callbackProjCmdROS] Received projection command: %d", out_RC->proj_cmd_data);
}

void callbackProjImgROS(const std_msgs::Int32MultiArray::ConstPtr &msg, ROSComm *out_RC)
{
    // Check that the received data has the correct size for a 10x8 array (80 elements)
    if (msg->data.size() != 80)
    {
        ROS_ERROR("[callbackProjImgROS] Received incorrect array size. Expected 80 elements, but got %zu", msg->data.size());
        return;
    }

    // Store the 10x8 data in proj_img_data
    for (int cham_ind = 0; cham_ind < 10; ++cham_ind)
    {
        for (int wall_ind = 0; wall_ind < 8; ++wall_ind)
        {
            out_RC->proj_img_data[cham_ind][wall_ind] = msg->data[cham_ind * 8 + wall_ind];
        }
    }

    // Flag that a projection image message has been received
    out_RC->is_proj_img_message_received = true;

    // Log the entire 2D array
    if (GLB_DO_VERBOSE_DEBUG)
    {
        ROS_INFO("[callbackProjImgROS] Stored ROS projection image data:");
        for (int cham_ind = 0; cham_ind < 10; ++cham_ind)
        {
            std::stringstream row_stream;
            row_stream << "Walls[" << cham_ind << "] = [";
            for (int wall_ind = 0; wall_ind < 8; ++wall_ind)
            {
                row_stream << out_RC->proj_img_data[cham_ind][wall_ind];
                if (wall_ind < 7) // Add a comma between elements, but not after the last one
                {
                    row_stream << ", ";
                }
            }
            row_stream << "]";
            ROS_INFO("%s", row_stream.str().c_str());
        }
    }
}

void callbackTrackPosROS(const geometry_msgs::PoseStamped::ConstPtr &msg, ROSComm *out_RC)
{
    // Update the last received command
    out_RC->track_pos_data = *msg;
    out_RC->is_track_pos_message_received = true;

    // Log position data
    if (GLB_DO_VERBOSE_DEBUG)
        ROS_INFO("[callbackHarnessPosROS] Received tracking: position x[%f] y[%f] z[%f], orientation x[%f] y[%f] z[%f]",
                 out_RC->track_pos_data.pose.position.x,
                 out_RC->track_pos_data.pose.position.y,
                 out_RC->track_pos_data.pose.position.z,
                 out_RC->track_pos_data.pose.orientation.x,
                 out_RC->track_pos_data.pose.orientation.y,
                 out_RC->track_pos_data.pose.orientation.z);
}

int initSubscriberROS(ROSComm &out_RC)
{
    // Check if node handle is initialized
    if (!out_RC.node_handle)
    {
        ROS_ERROR("[initSubscriberROS]Node handle is not initialized!");
        return -1;
    }

    // Initialize the "projection_cmd" subscriber using boost::bind
    out_RC.proj_cmd_sub = out_RC.node_handle->subscribe<std_msgs::Int32>(
        "projection_cmd", 10, boost::bind(&callbackProjCmdROS, _1, &out_RC));
    if (!out_RC.proj_cmd_sub)
    {
        ROS_ERROR("[initSubscriberROS]Failed to subscribe to 'projection_cmd' topic!");
        return -1;
    }

    // Initialize the "projection_image" subscriber
    out_RC.proj_img_sub = out_RC.node_handle->subscribe<std_msgs::Int32MultiArray>(
        "projection_image", 10, boost::bind(&callbackProjImgROS, _1, &out_RC));
    if (!out_RC.proj_img_sub)
    {
        ROS_ERROR("[initSubscriberROS]Failed to subscribe to 'projection_image' topic!");
        return -1;
    }

    // Initialize the "harness_pose_in_maze" subscriber
    out_RC.track_pos_sub = out_RC.node_handle->subscribe<geometry_msgs::PoseStamped>(
        "harness_pose_in_maze", 10, boost::bind(&callbackTrackPosROS, _1, &out_RC));
    if (!out_RC.track_pos_sub)
    {
        ROS_ERROR("[initSubscriberROS]Failed to subscribe to 'harness_pose_in_maze' topic!");
        return -1;
    }

    return 0;
}

int procProjCmdROS(ROSComm &out_RC)
{
    // Check if node handle is initialized
    if (!ros::ok())
    {
        ROS_ERROR("[procProjCmdROS]ROS is no longer running!");
        return -1;
    }

    // Bail if no message received
    if (!out_RC.is_proj_cmd_message_received)
        return 0;

    // Reset the flag
    out_RC.is_proj_cmd_message_received = false;

    // ---------- Monitor Mode Change Commmands ----------

    // Move monitor command [-1]
    if (out_RC.proj_cmd_data == -1)
    {
        F.windows_set_to_proj = !F.windows_set_to_proj;
        F.change_window_mode = true;
    }

    // Set/unset Fullscreen [-2] ----------
    else if (out_RC.proj_cmd_data == -2)
    {
        F.fullscreen_mode = !F.fullscreen_mode;
        F.change_window_mode = true;
    }

    // Force window to top [-3] ----------
    else if (out_RC.proj_cmd_data == -3)
    {
        F.force_window_focus = true;
    }

    else
    {
        ROS_WARN("[procProjCmdROS] Received invalid projection command: %d", out_RC.proj_cmd_data);
    }

    return 0;
}

int procProjImgROS(ROSComm &out_RC)
{
    // Check if node handle is initialized
    if (!ros::ok())
    {
        ROS_ERROR("[procProjImgROS] ROS is no longer running!");
        return -1;
    }

    // Bail if no message received
    if (!out_RC.is_proj_img_message_received)
        return 0;

    // Reset the flag
    out_RC.is_proj_img_message_received = false;

    // ---------- Update image index data ----------

    for (int cham_ind = 0; cham_ind < 10; ++cham_ind)
    {
        for (int wall_ind = 0; wall_ind < 8; ++wall_ind)
        {
            // Store image index
            int img_ind = out_RC.proj_img_data[cham_ind][wall_ind];

            // Update the wall index array
            if (cham_ind < 9)
                configWallImageIndex(img_ind, cham_ind, wall_ind, PROJ_WALL_CONFIG_INDICES_4D);
            // Update the floor image index
            else if (wall_ind == 0) // Only store the first entry
                projFloorConfigIndex = img_ind;
            // Skip unused floor entries
            else
                continue;
        }
    }

    // Set the flag to update the textures
    F.update_textures = true;

    return 0;
}

int procTrackMsgROS(ROSComm &out_RC, RatTracker &out_RT)
{
    // Check if node handle is initialized
    if (!ros::ok())
    {
        ROS_ERROR("[procTrackMsgROS]ROS is no longer running!");
        return -1;
    }

    // Bail if no message received
    if (!out_RC.is_track_pos_message_received)
        return 0;

    // Reset the flag
    out_RC.is_track_pos_message_received = false;

    // Convert the track pose to centimeters
    geometry_msgs::Point position_cm;
    position_cm.x = out_RC.track_pos_data.pose.position.x * 100.0f;
    position_cm.y = out_RC.track_pos_data.pose.position.y * 100.0f;

    // Extract the quaternion
    tf::Quaternion q(
        out_RC.track_pos_data.pose.orientation.x,
        out_RC.track_pos_data.pose.orientation.y,
        out_RC.track_pos_data.pose.orientation.z,
        out_RC.track_pos_data.pose.orientation.w);

    // Convert quaternion to RPY (roll, pitch, yaw)
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Convert offset angle from degrees to radians
    double offset_angle_rad = out_RT.offset_angle * GLOB_PI / 180.0f;

    // Adjust the yaw by the offset angle
    double adjusted_yaw = yaw + offset_angle_rad;

    // Calculate the offset in global frame
    double offset_x = out_RT.offset_distance * cos(adjusted_yaw);
    double offset_y = out_RT.offset_distance * sin(adjusted_yaw);

    // Apply the offset to the original position
    out_RT.marker_position.x = position_cm.x + offset_x;
    out_RT.marker_position.y = position_cm.y + offset_y;

    return 0;
}

// Function to convert quaternion to yaw
double getYawFromQuaternion(const geometry_msgs::Quaternion &quat)
{
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

// Function to calculate the new position with offset
geometry_msgs::Point getOffsetPosition(const geometry_msgs::Pose &pose, double offset_distance)
{
    double yaw = getYawFromQuaternion(pose.orientation);

    // Calculate the offset in global frame
    double offset_x = offset_distance * cos(yaw);
    double offset_y = offset_distance * sin(yaw);

    // Apply the offset to the original position
    geometry_msgs::Point new_position;
    new_position.x = pose.position.x + offset_x;
    new_position.y = pose.position.y + offset_y;
    new_position.z = pose.position.z; // Assuming no change in z-axis

    return new_position;
}

void simulateRatMovement(float move_step, float max_turn_angle, RatTracker &out_RT)
{
    // Track marker angle
    static float marker_angle = 0.0f;

    // Lambda function to keep the rat within the enclosure and turn when hitting the wall
    auto keepWithinBoundsAndTurn = [&](cv::Point2f &position)
    {
        cv::Point2f original_position = position;
        position.x = std::min(std::max(position.x, 0.0f), GLB_MAZE_WIDTH_HEIGHT_CM);
        position.y = std::min(std::max(position.y, 0.0f), GLB_MAZE_WIDTH_HEIGHT_CM);

        // If position changed (rat hits the wall), rotate randomly up to 180 degrees
        if (position != original_position)
        {
            marker_angle += rand() % 181 - 90; // Random turn between -90 and +90 degrees
        }
    };

    // Randomly decide to change direction
    if (rand() % 10 == 0)
    { // 10% chance to change direction
        marker_angle += (rand() % static_cast<int>(2 * max_turn_angle)) - max_turn_angle;
    }

    // Calculate new position
    float radian_angle = marker_angle * GLOB_PI / 180.0f;
    out_RT.marker_position.x += move_step * cos(radian_angle);
    out_RT.marker_position.y += move_step * sin(radian_angle);

    // Keep the rat within the enclosure and turn if hitting the wall
    keepWithinBoundsAndTurn(out_RT.marker_position);
}

void configWallImageIndex(int image_ind, int chamber_ind, int wall_ind, ProjWallConfigIndices4D &out_PROJ_WALL_CONFIG_INDICES_4D)
{
    std::vector<int> walls_ind = {wall_ind};
    configWallImageIndex(image_ind, chamber_ind, walls_ind, out_PROJ_WALL_CONFIG_INDICES_4D);
}

void configWallImageIndex(int image_ind, int chamber_ind, const std::vector<int> &walls_ind, ProjWallConfigIndices4D &out_PROJ_WALL_CONFIG_INDICES_4D)
{
    // Determine the row and column based on chamber index
    int row = chamber_ind / 3;
    int col = chamber_ind % 3;

    // Wall to array index mapping for each projector
    std::unordered_map<int, std::unordered_map<int, int>> wallToIndexMapping = {
        {0, {{3, 0}, {4, 1}, {5, 2}}}, // Projector 0 (West) - East walls
        {1, {{5, 0}, {6, 1}, {7, 2}}}, // Projector 1 (North) - South walls
        {2, {{7, 0}, {0, 1}, {1, 2}}}, // Projector 2 (East) - West walls
        {3, {{1, 0}, {2, 1}, {3, 2}}}  // Projector 3 (South) - North walls
    };

    // Iterate through each projector
    for (int proj = 0; proj < 4; ++proj)
    {
        // Calculate adjusted row and column for each projector
        int adjusted_row = row;
        int adjusted_col = col;

        // Adjust the indices based on the projector orientation
        switch (proj)
        {
        case 0: // Adjustments for Projector 0 (West)
            adjusted_row = 2 - col;
            adjusted_col = row;
            break;
        case 1: // Adjustments for Projector 1 (North)
            adjusted_row = 2 - row;
            adjusted_col = 2 - col;
            break;
        case 2: // Adjustments for Projector 2 (East)
            adjusted_row = col;
            adjusted_col = 2 - row;
            break;
        case 3: // No adjustments needed for Projector 3 (South)
            break;
        }

        // Iterate through each wall index
        for (int wall : walls_ind)
        {
            // Check if wall index is valid and has a mapping
            auto wallMapping = wallToIndexMapping[proj].find(wall);
            if (wallMapping != wallToIndexMapping[proj].end())
            {
                // Set the image index for the corresponding wall
                out_PROJ_WALL_CONFIG_INDICES_4D[proj][adjusted_row][adjusted_col][wallMapping->second] = image_ind;
            }
        }
    }
}

void computeMazeVertCm(int proj_ind, std::vector<cv::Point2f> &maze_vert_cm_vec)
{
    // Lambda function for circular shift
    auto circShift = [](const std::vector<cv::Point2f> &vec, int shift) -> std::vector<cv::Point2f>
    {
        std::vector<cv::Point2f> shifted_vec(vec.size());
        int n = static_cast<int>(vec.size());
        for (int v_i = 0; v_i < n; ++v_i)
        {
            shifted_vec[v_i] = vec[(v_i + shift + n) % n];
        }
        return shifted_vec;
    };

    // Template vertices
    const std::vector<cv::Point2f> template_maze_vert_cm_vec = {
        cv::Point2f(0, GLB_MAZE_WIDTH_HEIGHT_CM),
        cv::Point2f(GLB_MAZE_WIDTH_HEIGHT_CM, GLB_MAZE_WIDTH_HEIGHT_CM),
        cv::Point2f(GLB_MAZE_WIDTH_HEIGHT_CM, 0.0),
        cv::Point2f(0.0, 0.0)};

    // Apply circular shift based on the given projector
    switch (proj_ind)
    {
    case 0: // Circular shift left by 1
        maze_vert_cm_vec = circShift(template_maze_vert_cm_vec, 1);
        break;
    case 1: // Circular shift left by 2
        maze_vert_cm_vec = circShift(template_maze_vert_cm_vec, 2);
        break;
    case 2: // Circular shift right by 1
        maze_vert_cm_vec = circShift(template_maze_vert_cm_vec, -1);
        break;
    case 3: // No shift
        maze_vert_cm_vec = circShift(template_maze_vert_cm_vec, 0);
        break;
    default:
        break;
    }
}

void rotateFloorImage(int img_rot_deg, const cv::Mat &in_img_mat, std::vector<cv::Mat> &out_img_mat_vec)
{
    // Normalize rotation to be within 0-270 degrees using modulo
    int normalized_rot_deg = (img_rot_deg % 360 + 360) % 360;

    cv::Mat rotated_img;

    // Rotate based on the normalized rotation angle
    switch (normalized_rot_deg)
    {
    case 90:
        cv::rotate(in_img_mat, rotated_img, cv::ROTATE_90_CLOCKWISE);
        break;
    case 180:
        cv::rotate(in_img_mat, rotated_img, cv::ROTATE_180);
        break;
    case 270:
        cv::rotate(in_img_mat, rotated_img, cv::ROTATE_90_COUNTERCLOCKWISE);
        break;
    case 0:
    default:
        // No rotation needed, just copy the input image
        rotated_img = in_img_mat.clone();
        break;
    }

    // Store the rotated image in the output vector
    out_img_mat_vec.push_back(rotated_img);
}

int updateFloorTexture(
    int proj_ind,
    cv::Mat &_floorImgMat,
    const cv::Mat _wallBlankImgMat,
    std::array<cv::Mat, 4> &_FLOOR_HMAT_ARR,
    cv::Mat &out_img_mat)
{

    // Copy the floor image to be used
    cv::Mat img_copy;
    _floorImgMat.copyTo(img_copy);

    // Get homography matrix for this wall
    cv::Mat H = _FLOOR_HMAT_ARR[proj_ind];

    // Warp Perspective
    cv::Mat img_warp;
    if (warpImgMat(img_copy, H, img_warp) < 0)
    {
        ROS_ERROR("[updateFloorTexture] Warp image error: Projector[%d]", proj_ind);
        return -1;
    }

    // Merge the warped image with the final image
    if (mergeImgMat(img_warp, out_img_mat) < 0)
        return -1;

    // Merge the blank wall image with the final image
    if (mergeImgMat(_wallBlankImgMat, out_img_mat) < 0)
        return -1;

    return 0;
}

int updateWallTexture(
    int proj_ind,
    const std::vector<cv::Mat> &_wallRawImgMatVec,
    const ProjWallConfigIndices4D &_PROJ_WALL_CONFIG_INDICES_4D,
    const std::array<std::array<std::array<std::array<cv::Mat, GLB_MAZE_SIZE>, GLB_MAZE_SIZE>, N_CAL_MODES - 1>, 4> &_WALL_HMAT_ARR,
    bool do_ignore_blank_img,
    cv::Mat &out_img_mat)
{
    // Iterate through through calibration modes (left walls, middle walls, right walls)
    for (int cal_i = 0; cal_i < N_CAL_MODES - 1; cal_i++)
    {
        CalibrationMode _CAL_MODE = static_cast<CalibrationMode>(cal_i);

        // Iterate through the maze grid rows
        for (int gr_i = 0; gr_i < GLB_MAZE_SIZE; gr_i++) // image bottom to top
        {
            // Iterate through each column in the maze row
            for (int gc_i = 0; gc_i < GLB_MAZE_SIZE; gc_i++) // image left to right
            {
                // Get the index of the wall image to be used
                int img_ind = _PROJ_WALL_CONFIG_INDICES_4D[proj_ind][gr_i][gc_i][_CAL_MODE];
                if (_wallRawImgMatVec[img_ind].empty())
                {
                    ROS_ERROR("[updateWallTexture] Stored OpenCV wall image is empty: Projector[%d] Wall[%d][%d] Calibration[%d] Image[%d]",
                              proj_ind, gr_i, gc_i, _CAL_MODE, img_ind);
                    return -1;
                }

                // Skip empty images
                if (img_ind == 0 && do_ignore_blank_img)
                    continue;

                // Copy the wall image to be used
                cv::Mat img_copy;
                _wallRawImgMatVec[img_ind].copyTo(img_copy);

                // Get homography matrix for this wall
                cv::Mat H = _WALL_HMAT_ARR[proj_ind][_CAL_MODE][gr_i][gc_i];

                // Warp Perspective
                cv::Mat img_warp;
                if (warpImgMat(img_copy, H, img_warp) < 0)
                {
                    ROS_ERROR("[updateWallTexture] Warp image error: Projector[%d] Wall[%d][%d] Calibration[%d]",
                              proj_ind, gr_i, gc_i, _CAL_MODE);
                    return -1;
                }

                // Merge the warped image with the final image
                if (mergeImgMat(img_warp, out_img_mat) < 0)
                    return -1;
            }
        }
    }

    return 0;
}

int drawRatMask(
    const RatTracker &_RT,
    CircleRenderer &out_rmCircRend)
{
    // Setup the CircleRenderer class shaders
    if (CircleRenderer::SetupShader() < 0)
        return -1;

    // Set the marker position
    out_rmCircRend.setPosition(_RT.marker_position);

    // Recompute the marker parameters
    if (out_rmCircRend.updateCircleObject(true) < 0)
        return -1;

    // Draw the marker
    if (out_rmCircRend.draw() < 0)
        return -1;

    // Unset the shader program
    if (CircleRenderer::UnsetShader() < 0)
        return -1;

    // Return GL status
    return 0;
}

void appInitROS(int argc, char **argv, ROSComm &out_RC)
{
    ROS_INFO("[projection_display:appInitROS] STARTING PROJECTION DISPLAY NODE");

    // Initialize ROS
    ros::init(argc, argv, "projection_display", ros::init_options::AnonymousName);
    if (!ros::master::check())
        throw std::runtime_error("[appInitROS] Failed initialzie ROS: ROS master is not running");

    // Initialize NodeHandle inside RC
    RC.node_handle = std::make_unique<ros::NodeHandle>();

    // Initialize the ros::Rate object with a specific rate
    out_RC.loop_rate = std::make_unique<ros::Rate>(GLB_ROS_LOOP_RATE);

    // Initialize the subscriber
    if (initSubscriberROS(out_RC) < 0)
        throw std::runtime_error("[appInitROS] Failed to initialize ROS subscriber");

    ROS_INFO("[projection_display:appInitROS] Finished initializing ROS successfully");
}

void appLoadAssets()
{
    // ---------- Load Images with OpenCV ----------

    // Get the wall images
    std::vector<std::string> fi_img_path_wall_vec;                                        // declare the vector to store the paths
    size_t n_wall_img = sizeof(WALL_IMAGE_FILE_NAMES) / sizeof(WALL_IMAGE_FILE_NAMES[0]); // calculate the number of images
    for (size_t i = 0; i < n_wall_img; ++i)
    {
        fi_img_path_wall_vec.push_back(RUNTIME_IMAGE_PATH + "/" + WALL_IMAGE_FILE_NAMES[i] + ".png");
    }
    if (loadImgMat(fi_img_path_wall_vec, wallRawImgMatVec) < 0)
        throw std::runtime_error("[appLoadAssets] Failed to load OpentCV wall images");

    // Get the floor images
    std::vector<std::string> fi_img_path_floor_vec;                                          // declare the vector to store the paths
    size_t n_floor_img = sizeof(FLOOR_IMAGE_FILE_NAMES) / sizeof(FLOOR_IMAGE_FILE_NAMES[0]); // calculate the number of images
    for (size_t i = 0; i < n_floor_img; ++i)
    {
        fi_img_path_floor_vec.push_back(RUNTIME_IMAGE_PATH + "/" + FLOOR_IMAGE_FILE_NAMES[i] + ".png");
    }
    if (loadImgMat(fi_img_path_floor_vec, floorRawImgMatVec) < 0)
        throw std::runtime_error("[appLoadAssets] Failed to load OpentCV floor images");

    // ---------- Load Wall and Floor Homography Matrices from XML ----------
    for (int proj_ind = 0; proj_ind < N.projector; ++proj_ind) // for each projector
    {
        for (int cal_i = 0; cal_i < N_CAL_MODES; ++cal_i)
        {
            CalibrationMode _CAL_MODE = static_cast<CalibrationMode>(cal_i);

            // Store wall homography matrices
            if (_CAL_MODE == WALLS_LEFT || _CAL_MODE == WALLS_MIDDLE || _CAL_MODE == WALLS_RIGHT)
            {
                // Iterate through the maze grid rows
                for (int gr_i = 0; gr_i < GLB_MAZE_SIZE; ++gr_i)
                {
                    for (int gc_i = 0; gc_i < GLB_MAZE_SIZE; ++gc_i)
                    {
                        // Load the homography matrix from XML
                        if (xmlLoadHMAT(proj_ind, _CAL_MODE, gr_i, gc_i, WALL_HMAT_ARR[proj_ind][_CAL_MODE][gr_i][gc_i]) < 0)
                            throw std::runtime_error("[appLoadAssets] Error returned from: xmlLoadHMAT");
                    }
                }
            }
            // Store floor homography matrices
            else
            {
                // Load the homography matrix from XML
                if (xmlLoadHMAT(proj_ind, _CAL_MODE, 0, 0, FLOOR_HMAT_ARR[proj_ind]) < 0)
                    throw std::runtime_error("[appLoadAssets] Error returned from: xmlLoadHMAT");
            }
        }
    }

    // ---------- Load Maze Boundary Vertices ----------
    for (int proj_ind = 0; proj_ind < N.projector; ++proj_ind) // for each projector
    {
        std::vector<cv::Point2f> maze_vert_ndc_vec(4);
        std::vector<cv::Point2f> maze_vert_cm_vec(4);

        // Load the maze vertices from XML
        if (xmlLoadVertices(proj_ind, maze_vert_ndc_vec) < 0)
            throw std::runtime_error("[appLoadAssets] Error returned from: xmlLoadVertices");

        // Compute the rotated maze vertices in centimeter units
        computeMazeVertCm(proj_ind, maze_vert_cm_vec);

        // Compute the homography matrix for warping the rat mask marker from maze cm to ndc space for each projector
        cv::Mat H;
        if (computeHomographyMatrix(maze_vert_cm_vec, maze_vert_ndc_vec, H))
            throw std::runtime_error("[appLoadAssets] Projector[" + std::to_string(proj_ind) + "]: Invalid homography matrix for rat mask image");

        // Store the homography matrix
        HMAT_CM_TO_NDC_ARR[proj_ind] = H;
    }

    ROS_INFO("[projection_display:appLoadAssets] Finished loading variables successfully");
}

void appInitVariables()
{
    // ---------- Intialize the Window Offset Vector ---------
    winOffsetVec.clear();              // Clear any existing elements
    winOffsetVec.reserve(N.projector); // Reserve memory for efficiency
    for (int mon_ind = 0; mon_ind < N.projector; ++mon_ind)
    {
        // Calculate x and y offsets based on the monitor resolution
        int x_offset = mon_ind * (GLB_MONITOR_WIDTH_PXL / N.projector) * 0.9f;
        int y_offset = mon_ind * (GLB_MONITOR_HEIGHT_PXL / N.projector) * 0.9f;
        winOffsetVec.emplace_back(x_offset, y_offset);
    }

    // ---------- Convert and store rotated floor images ---------

    // Loop through floorRawImgMatVec images and store a new entry for each projector in floorRotatedImgMatVecArr
    for (size_t i = 0; i < floorRawImgMatVec.size(); ++i)
    {
        rotateFloorImage(270, floorRawImgMatVec[i], floorRotatedImgMatVecArr[0]); // Projector 0
        rotateFloorImage(180, floorRawImgMatVec[i], floorRotatedImgMatVecArr[1]); // Projector 1
        rotateFloorImage(90, floorRawImgMatVec[i], floorRotatedImgMatVecArr[2]);  // Projector 2
        rotateFloorImage(0, floorRawImgMatVec[i], floorRotatedImgMatVecArr[3]);   // Projector 3
    }

    ROS_INFO("[projection_display:appInitVariables] Finished initializing variables successfully");
}

void appInitOpenGL()
{

    // Initialize GLFW and OpenGL settings and get number of monitors on the system
    if (MazeRenderContext::SetupGraphicsLibraries(N.monitor, I.proj_mon_vec) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to initialize graphics");
    ROS_INFO("[projection_display:appInitOpenGL] OpenGL initialized: Projector monitor indices: %d, %d, %d, %d", I.proj_mon_vec[0], I.proj_mon_vec[1], I.proj_mon_vec[2], I.proj_mon_vec[3]);

    // // Check if expected monitors exceed available monitors
    // if (I.proj_mon_vec.back() >= N.monitor) // compare last entry
    //     throw std::runtime_error("[appInitOpenGL] Monitor index exceeds available monitors");

    // Initialize OpenGL for each projector
    for (int proj_ind = 0; proj_ind < N.projector; ++proj_ind)
    {
        // Start on the default screen
        int mon_ind = I.starting_monitor;

        // Initialze render context for each projector
        if (PROJ_CTX_VEC[proj_ind].initWindowContext(proj_ind, mon_ind, GLB_MONITOR_WIDTH_PXL, GLB_MONITOR_HEIGHT_PXL, callbackKeyBinding) < 0)
            throw std::runtime_error("[appInitOpenGL] Failed to initialize render context");

        // Initialize OpenGL wall image objects
        if (PROJ_CTX_VEC[proj_ind].initRenderObjects(GLB_QUAD_GL_VERTICES, sizeof(GLB_QUAD_GL_VERTICES), GLB_QUAD_GL_INDICES, sizeof(GLB_QUAD_GL_INDICES)) < 0)
            throw std::runtime_error("[appInitOpenGL] Failed to initialize opengl wall image objects");

        // Create the shader program for wall image rendering
        if (PROJ_CTX_VEC[proj_ind].compileAndLinkShaders(GLB_QUAD_GL_VERTEX_SOURCE, GLB_QUAD_GL_FRAGMENT_SOURCE) < 0)
            throw std::runtime_error("[appInitOpenGL] Window[" + std::to_string(PROJ_CTX_VEC[proj_ind].windowInd) + "]: Failed to compile and link wall shader");

        // Create the shader program for CircleRenderer class rat mask rendering
        if (CircleRenderer::CompileAndLinkCircleShaders(1.0) < 0)
            throw std::runtime_error("[appInitOpenGL] Failed to compile and link circlerenderer class shader");

        // Set all projectors to the starting monitor and include xy offset
        if (PROJ_CTX_VEC[proj_ind].changeWindowDisplayMode(mon_ind, F.fullscreen_mode, winOffsetVec[PROJ_CTX_VEC[proj_ind].windowInd]) < 0)
            throw std::runtime_error("[appInitOpenGL] Window[" + std::to_string(PROJ_CTX_VEC[proj_ind].windowInd) + "]: Failed Initial update of window monitor mode");

        // Initialize the CircleRenderer class object for rat masking
        if (RM_CIRCREND_ARR[proj_ind].initializeCircleObject(
                RT.marker_position,          // position
                RT.marker_radius,            // radius
                RT.marker_rgb,               // color
                RT.marker_segments,          // segments
                HMAT_CM_TO_NDC_ARR[proj_ind] // homography matrix
                ) < 0)
            throw std::runtime_error("[appInitOpenGL] Failed to initialize CircleRenderer class object");

        // Initialize blank wall image mat
        ProjWallConfigIndices4D proj_wall_cfg_indices_blank = {};                                              // Initialize to all zeros for blank images
        wallBlankImgMatArr[proj_ind] = cv::Mat::zeros(GLB_MONITOR_HEIGHT_PXL, GLB_MONITOR_WIDTH_PXL, CV_8UC4); // Initialize cv::Mat
        if (updateWallTexture(proj_ind,
                              wallRawImgMatVec,
                              proj_wall_cfg_indices_blank,
                              WALL_HMAT_ARR,
                              false,
                              wallBlankImgMatArr[proj_ind]))
            throw std::runtime_error("[appInitOpenGL] Window[" + std::to_string(proj_ind) + "]: Failed to update wall texture");

        ROS_INFO("[projection_display:appInitOpenGL] OpenGL initialized: Projector[%d] Window[%d] Monitor[%d]", proj_ind, PROJ_CTX_VEC[proj_ind].windowInd, PROJ_CTX_VEC[proj_ind].monitorInd);
    }

    ROS_INFO("[projection_display:appInitOpenGL] OpenGL contexts and objects Initialized succesfully");
}


void appMainLoop()
{
    // Initialize the timing data
    mainLoopTD.reset();

    int status = 0;
    ROS_INFO("i[appMainLoop] Starting");

    while (status == 0)
    {

        // --------------- Check State Flags ---------------

        if (F.change_window_mode)
        {
            for (auto &projCtx : PROJ_CTX_VEC)
            {
                int mon_ind = F.windows_set_to_proj ? I.proj_mon_vec[projCtx.windowInd] : I.starting_monitor;
                if (projCtx.changeWindowDisplayMode(mon_ind, F.fullscreen_mode, winOffsetVec[projCtx.windowInd]) < 0)
                    throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from: MazeRenderContext::changeWindowDisplayMode");
            }
        }

        // Force to windows so thay are on top
        if (F.force_window_focus)
        {
            for (auto &projCtx : PROJ_CTX_VEC)
            {
                if (projCtx.forceWindowFocus() < 0)
                    throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from: MazeRenderContext::forceWindowFocus");
            }
        }

        // TEMP Simulate rat movement for testing
        // simulateRatMovement(0.5f, 45.0f, RT);

        // Recompute wall parameters and update wall image texture
        if (F.update_textures)
        {
            for (auto &projCtx : PROJ_CTX_VEC)
            {
                cv::Mat img_mat = cv::Mat::zeros(GLB_MONITOR_HEIGHT_PXL, GLB_MONITOR_WIDTH_PXL, CV_8UC4);

                // Update floor image texture
                if (updateFloorTexture(projCtx.windowInd,
                                       floorRotatedImgMatVecArr[projCtx.windowInd][projFloorConfigIndex],
                                       wallBlankImgMatArr[projCtx.windowInd],
                                       FLOOR_HMAT_ARR,
                                       img_mat))
                    throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Failed to update wall texture");

                // Update wall image texture
                if (updateWallTexture(projCtx.windowInd,
                                      wallRawImgMatVec,
                                      PROJ_WALL_CONFIG_INDICES_4D,
                                      WALL_HMAT_ARR,
                                      true,
                                      img_mat))
                    throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Failed to update wall texture");

                // Load the new texture
                if (projCtx.loadMatTexture(img_mat) < 0)
                    throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Failed to load wall texture");
            }
        }

        // Reset keybinding flags
        F.change_window_mode = false;
        F.update_textures = false;
        F.force_window_focus = false;

        // --------------- Handle Image Processing for Next Frame ---------------
        for (auto &projCtx : PROJ_CTX_VEC)
        {

            // Prepare the frame for rendering (clear the back buffer)
            if (projCtx.initWindowForDrawing() < 0)
                throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from: MazeRenderContext::initWindowForDrawing");

            // Draw/update wall images
            if (projCtx.drawTexture() < 0)
                throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from: drawTexture");

            // Draw/update rat mask marker
            if (drawRatMask(RT, RM_CIRCREND_ARR[projCtx.windowInd]) < 0)
                throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from: drawRatMask");

            // Swap buffers and poll events
            if (projCtx.bufferSwapPoll() < 0)
                throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from: MazeRenderContext::bufferSwapPoll");

            // Check if ROS shutdown
            if (!ros::ok())
                throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Unexpected ROS shutdown");

            // Get exit request status
            status = status == 0 ? projCtx.checkExitRequest() : status;
            if (status < 0)
                throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from: MazeRenderContext::checkExitRequest");
        }

        // Check for exit
        if (status > 0)
            break;

        // --------------- Handle ROS Messages and Operations ---------------

        // Process a single round of callbacks for ROS messages
        ros::spinOnce();

        // Process ROS projection command messages
        if (procProjCmdROS(RC) < 0)
            throw std::runtime_error("[appMainLoop] Error returned from: procProjCmdROS");

        // Process ROS projection image messages
        if (procProjImgROS(RC) < 0)
            throw std::runtime_error("[appMainLoop] Error returned from: procProjImgROS");

        // Process ROS tracking position messages
        if (procTrackMsgROS(RC, RT) < 0)
            throw std::runtime_error("[appMainLoop] Error returned from: procTrackMsgROS");

        // Measure the time taken for the loop iteration
        // mainLoopTD.addDeltaTime(true); // Add delta time and print timing data

        // Sleep to maintain the loop rate
        RC.loop_rate->sleep();
    }

    // Check which condition caused the loop to exit
    if (status == 1)
        ROS_INFO("[projection_display:appMainLoop] Loop Terminated:  GLFW window should close");
    else if (status == 2)
        ROS_INFO("[projection_display:appMainLoop] Loop Terminated:  Escape key was pressed");
    else
        ROS_INFO("[projection_display:appMainLoop] Loop Terminated:  Reason unknown");
}

void appCleanup()
{
    ROS_INFO("SHUTTING DOWN");

    // Clean up OpenGL wall image objects for each window
    for (int proj_ind = 0; proj_ind < N.projector; ++proj_ind)
    {
        if (PROJ_CTX_VEC[proj_ind].cleanupContext(true) != 0)
            ROS_WARN("[appCleanup] Error during cleanup of MazeRenderContext: Projector[%d] Window[%d] Monitor[%d]",
                     proj_ind, PROJ_CTX_VEC[proj_ind].windowInd, PROJ_CTX_VEC[proj_ind].monitorInd);
        else
            ROS_INFO("[projection_display:appCleanup] MazeRenderContext instance cleaned up successfully: Projector[%d] Window[%d] Monitor[%d]",
                     proj_ind, PROJ_CTX_VEC[proj_ind].windowInd, PROJ_CTX_VEC[proj_ind].monitorInd);
    }

    // Terminate graphics
    if (MazeRenderContext::CleanupGraphicsLibraries() < 0)
        ROS_WARN("[appCleanup] Failed to terminate GLFW library");
    else
        ROS_INFO("[projection_display:appCleanup] GLFW library terminated successfully");
}

int main(int argc, char **argv)
{
    try
    {
        appInitROS(argc, argv, RC);
        appLoadAssets();
        appInitVariables();
        appInitOpenGL();
        appMainLoop();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("!!EXCEPTION CAUGHT!!: %s", e.what());
        void appCleanup();
        return -1;
    }
    return 0;
}
