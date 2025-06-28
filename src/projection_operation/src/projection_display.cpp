// ########################################################################################################

// ======================================== projection_display.cpp ========================================

// ########################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_display.h"

// ================================================== FUNCTIONS ==================================================

void callbackKeyBinding(GLFWwindow *window, int key, int scancode, int action, int mods) {
    // Set the current OpenGL context to the window
    glfwMakeContextCurrent(window);

    // _______________ ANY KEY RELEASE ACTION _______________

    if (action == GLFW_RELEASE) {

        // ---------- Set/unset Fullscreen [F] ----------
        if (key == GLFW_KEY_F) {
            FLAG_FULLSCREEN_MODE = !FLAG_FULLSCREEN_MODE;
            FLAG_CHANGE_WINDOW_MODE = true;
        }

        // ---------- Check for change window monitor command [M] ----------
        if (key == GLFW_KEY_M) {
            FLAG_WINDOWS_SET_TO_PROJ = !FLAG_WINDOWS_SET_TO_PROJ;
            FLAG_CHANGE_WINDOW_MODE = true;
        }

        // ---------- Force Window to Top of UI Stack [T] ----------
        if (key == GLFW_KEY_T) 
            FLAG_FORCE_WINDOW_FOCUS = true;
        
    }
}

void callbackProjCmdROS(const std_msgs::Int32::ConstPtr &msg) {
    // Log projection command
    if (GLB_DO_VERBOSE_DEBUG)
        ROS_INFO("[callbackProjCmdROS] Received projection command: %d", msg->data);

    // ---------- Monitor Mode Change Commmands ----------
    // Move monitor command [-1]
    if (msg->data == -1) {
        FLAG_WINDOWS_SET_TO_PROJ = !FLAG_WINDOWS_SET_TO_PROJ;
        FLAG_CHANGE_WINDOW_MODE = true;
    }
    // Set/unset Fullscreen [-2] ----------
    else if (msg->data == -2) {
        FLAG_FULLSCREEN_MODE = !FLAG_FULLSCREEN_MODE;
        FLAG_CHANGE_WINDOW_MODE = true;
    }
    // Force window to top [-3] ----------
    else if (msg->data == -3)
        FLAG_FORCE_WINDOW_FOCUS = true;
    else
        ROS_WARN("[callbackProjCmdROS] Received invalid projection command: %d", msg->data);
}

void callbackProjImgROS(const std_msgs::Int32MultiArray::ConstPtr &msg) {
    // Expects a N_CHAMBERS x N_SURF array of integers
    // where N_CHAMBERS = 9 and N_SURF = 9 (8 walls + 1 floor)
    ROS_INFO("[callbackProjImgROS] Received projection image data with size: %zu", msg->data.size());

    // Check that the received data has the correct size for a 9x9 array (81 elements)
    if (msg->data.size() != N_CHAMBERS * N_SURF) {
        ROS_ERROR("[callbackProjImgROS] Received incorrect array size. Expected %d elements, but got %zu", 
            N_CHAMBERS * N_SURF, msg->data.size());
        return;
    }

    // Store the maze image data in the global maze image map
    for (auto &cham : Chambers)
        for (auto &surf : Surfaces)
            MAZE_IMAGE_MAP[cham][surf] = msg->data[cham * N_SURF + surf];
    
    // Convert the received data to a projection map
    mazeToProjectionMap(MAZE_IMAGE_MAP, PROJECTION_IMAGE_MAP);

    // Set the flag to update the textures
    FLAG_UPDATE_TEXTURES = true;
}

void callbackTrackPosROS(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // Log position data
    if (GLB_DO_VERBOSE_DEBUG)
        ROS_INFO("[callbackHarnessPosROS] Received tracking: position x[%f] y[%f] z[%f], orientation x[%f] y[%f] z[%f] w[%f]",
                 msg->pose.position.x, msg->pose.position.y,  msg->pose.position.z,
                 msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

    // Convert the track pose to centimeters
    geometry_msgs::Point position_cm;
    position_cm.x = msg->pose.position.x * 100.0f;
    position_cm.y = msg->pose.position.y * 100.0f;

    // Extract the quaternion
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w);
    // Convert quaternion to RPY (roll, pitch, yaw)
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Convert offset angle from degrees to radians
    double offset_angle_rad = RT.offset_angle * M_PI / 180.0f;

    // Adjust the yaw by the offset angle
    double adjusted_yaw = yaw + offset_angle_rad;

    // Calculate the offset in global frame
    double offset_x = RT.offset_distance * cos(adjusted_yaw);
    double offset_y = RT.offset_distance * sin(adjusted_yaw);

    // Apply the offset to the original position
    RT.marker_position.x = position_cm.x + offset_x;
    RT.marker_position.y = position_cm.y + offset_y;
}

// Function to convert quaternion to yaw
double getYawFromQuaternion(const geometry_msgs::Quaternion &quat) {
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

// Function to calculate the new position with offset
geometry_msgs::Point getOffsetPosition(const geometry_msgs::Pose &pose, double offset_distance) {
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

void simulateRatMovement(float move_step, float max_turn_angle) {
    // Track marker angle
    static float marker_angle = 0.0f;

    // Lambda function to keep the rat within the enclosure and turn when hitting the wall
    auto keepWithinBoundsAndTurn = [&](cv::Point2f &position) {
        cv::Point2f original_position = position;
        position.x = std::min(std::max(position.x, 0.0f), GLB_MAZE_WIDTH_HEIGHT_CM);
        position.y = std::min(std::max(position.y, 0.0f), GLB_MAZE_WIDTH_HEIGHT_CM);

        // If position changed (rat hits the wall), rotate randomly up to 180 degrees
        if (position != original_position)
            marker_angle += rand() % 181 - 90; // Random turn between -90 and +90 degrees //TODO: Why 181?
    };

    // Randomly decide to change direction
    if (rand() % 10 == 0) // 10% chance to change direction
        marker_angle += (rand() % static_cast<int>(2 * max_turn_angle)) - max_turn_angle;

    // Calculate new position
    float radian_angle = marker_angle * M_PI / 180.0f;
    RT.marker_position.x += move_step * cos(radian_angle);
    RT.marker_position.y += move_step * sin(radian_angle);

    // Keep the rat within the enclosure and turn if hitting the wall
    keepWithinBoundsAndTurn(RT.marker_position);
}

// void configWallImageIndex(int image_ind, int chamber_ind, int wall_ind, ProjectionMap<int> &out_PROJ_WALL_CONFIG_INDICES_4D) {
//     std::vector<int> walls_ind = {wall_ind};
//     configWallImageIndex(image_ind, chamber_ind, walls_ind, out_PROJ_WALL_CONFIG_INDICES_4D);
// }

// void configWallImageIndex(int image_ind, int chamber_ind, const std::vector<int> &walls_ind, ProjectionMap<int> &out_PROJ_WALL_CONFIG_INDICES_4D) {
//     // Determine the row and column based on chamber index
//     int row = chamber_ind / 3;
//     int col = chamber_ind % 3;

//     // Iterate through each projector
//     for (auto proj : Projectors) {
//         // Calculate adjusted row and column for each projector
//         int adjusted_row = row;
//         int adjusted_col = col;

//         // Adjust the indices based on the projector orientation
//         switch (proj) {
//         case 0: // Adjustments for Projector 0 (West)
//             adjusted_row = 2 - col;
//             adjusted_col = row;
//             break;
//         case 1: // Adjustments for Projector 1 (North)
//             adjusted_row = 2 - row;
//             adjusted_col = 2 - col;
//             break;
//         case 2: // Adjustments for Projector 2 (East)
//             adjusted_row = col;
//             adjusted_col = 2 - row;
//             break;
//         case 3: // No adjustments needed for Projector 3 (South)
//             break;
//         }

//         // Wall to array index mapping for each projector
//         std::unordered_map<int, std::unordered_map<int, int>> wallToIndexMapping = {
//             {0, {{3, 0}, {4, 1}, {5, 2}}}, // Projector 0 (West) - East walls
//             {1, {{5, 0}, {6, 1}, {7, 2}}}, // Projector 1 (North) - South walls
//             {2, {{7, 0}, {0, 1}, {1, 2}}}, // Projector 2 (East) - West walls
//             {3, {{1, 0}, {2, 1}, {3, 2}}}  // Projector 3 (South) - North walls
//         };

//         // Iterate through each wall index
//         for (int wall : walls_ind) {
//             // Check if wall index is valid and has a mapping
//             auto wallMapping = wallToIndexMapping[proj].find(wall);
//             if (wallMapping != wallToIndexMapping[proj].end()) // Set the image index for the corresponding wall
//                 out_PROJ_WALL_CONFIG_INDICES_4D[proj][adjusted_row][adjusted_col][wallMapping->second] = image_ind;
//         }
//     }
// }

void computeMazeVertCm(int proj_ind, std::vector<cv::Point2f> &maze_vert_cm_vec) {
    // Lambda function for circular shift
    auto circShift = [](const std::vector<cv::Point2f> &vec, int shift) -> std::vector<cv::Point2f> {
        std::vector<cv::Point2f> shifted_vec(vec.size());
        int n = static_cast<int>(vec.size());
        for (int v_i = 0; v_i < n; ++v_i)
            shifted_vec[v_i] = vec[(v_i + shift + n) % n];

        return shifted_vec;
    };

    // Template vertices
    const std::vector<cv::Point2f> template_maze_vert_cm_vec = {
        cv::Point2f(0, GLB_MAZE_WIDTH_HEIGHT_CM),
        cv::Point2f(GLB_MAZE_WIDTH_HEIGHT_CM, GLB_MAZE_WIDTH_HEIGHT_CM),
        cv::Point2f(GLB_MAZE_WIDTH_HEIGHT_CM, 0.0),
        cv::Point2f(0.0, 0.0)};

    // Apply circular shift based on the given projector
    switch (proj_ind) {
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

cv::Mat rotateImage(int proj_ind, const cv::Mat &in_img_mat) {
    cv::Mat rotated_img = in_img_mat.clone(); // Initialize with the input image

    if (proj_ind == 0) cv::rotate(in_img_mat, rotated_img, cv::ROTATE_90_COUNTERCLOCKWISE);
    else if (proj_ind == 1) cv::rotate(in_img_mat, rotated_img, cv::ROTATE_180);
    else if (proj_ind == 2) cv::rotate(in_img_mat, rotated_img, cv::ROTATE_90_CLOCKWISE);

    return rotated_img;
}


int updateFloorTexture(int proj_ind, cv::Mat &out_img_mat) {

    displayTimer[4].start();

    // Get homography matrix for this wall
    cv::Mat H = FLOOR_HMAT_ARR[proj_ind];

    // Get image to be used for the floor texture
    cv::Mat floorMat = runtimeFloorMats[floorImageIndex];
    floorMat = rotateImage(proj_ind, floorMat); // Rotate the image based on the projector index
    
    // Warp Perspective
    cv::Mat img_warp;
    if (warpImgMat(floorMat, H, img_warp) < 0) {
        ROS_ERROR("[updateFloorTexture] Warp image error: Projector[%d]", proj_ind);
        return -1;
    }

    // Merge the warped image with the final image
    if (mergeImgMat(img_warp, out_img_mat) < 0) // TODO: Error messages here
        return -1;

    // Merge the blank wall image with the final image
    if (mergeImgMat(BLANK_PXL_MAT, out_img_mat) < 0)
        return -1;
    
    displayTimer[4].update(true);

    return 0;
}

int updateWallTexture(int proj_ind, cv::Mat &out_img_mat) {
    displayTimer[5].start();
    int img_ind = 0; // Initialize image index
    cv::Mat img_warp;
    // Iterate through through calibration modes (left walls, middle walls, right walls)
    for (auto &mode : {MODE_WALLS_LEFT, MODE_WALLS_MIDDLE, MODE_WALLS_RIGHT}) {
        for (auto &proj : Projectors) {
            for (auto &row : Rows) {
                for (auto &col : Columns) {
                    img_ind = PROJECTION_IMAGE_MAP[proj][row][col][mode];
                    // Check if the image index is valid
                    if (img_ind < 0 || img_ind >= N_RUNTIME_WALL_IMAGES) img_ind = 0; // Reset to default image index if invalid

                    // cv::warpPerspective(runtimeWallMats[img_ind], img_warp, 
                    //     WALL_HMAT_ARR[proj_ind][mode][row][col], 
                    //     cv::Size(GLB_MONITOR_WIDTH_PXL, GLB_MONITOR_HEIGHT_PXL));

                    // displayTimer[6].start();
                    if (warpImgMat(runtimeWallMats[img_ind], WALL_HMAT_ARR[proj_ind][mode][row][col], img_warp) < 0) {
                        ROS_ERROR("[updateWallTexture] Warp image error: Projector[%d] Wall[%d][%d] Calibration[%d] Image[%d]",
                                  proj_ind, row, col, mode, img_ind);
                        return -1;
                    }
                    // displayTimer[6].update(true);

                    // displayTimer[7].start();
                    // Merge the warped image with the final image
                    if (mergeImgMat(img_warp, out_img_mat) < 0) {
                        ROS_ERROR("[updateWallTexture] Merge image error: Projector[%d] Wall[%d][%d] Calibration[%d] Image[%d]",
                                  proj_ind, row, col, mode, img_ind);
                        return -1;
                    }
                    // displayTimer[7].update(true);
                }
            }
        }
    }
    displayTimer[5].update(true);
    return 0;
}

int drawRatMask(const RatTracker &_RT, CircleRenderer &out_rmCircRend) {
    // Setup the CircleRenderer class shaders
    if (CircleRenderer::SetupShader() < 0) return -1;

    // Set the marker position
    out_rmCircRend.setPosition(_RT.marker_position);

    // Recompute the marker parameters
    if (out_rmCircRend.updateCircleObject(true) < 0) return -1;

    // Draw the marker
    if (out_rmCircRend.draw() < 0) return -1;

    // Unset the shader program
    if (CircleRenderer::UnsetShader() < 0) return -1;

    // Return GL status
    return 0;
}

void appInitROS(int argc, char **argv) {
    ROS_INFO("[projection_display:appInitROS] STARTING PROJECTION DISPLAY NODE");

    // Initialize ROS
    ros::init(argc, argv, "projection_display", ros::init_options::AnonymousName);
    if (!ros::master::check())
        throw std::runtime_error("[appInitROS] Failed to initialize ROS: ROS master is not running");

    // Initialize NodeHandle inside RC
    RC.node_handle = std::make_unique<ros::NodeHandle>();

    // Check if node handle is initialized
    if (!RC.node_handle) ROS_ERROR("[appInitROS] Node handle is not initialized!");

    // Initialize the ros::Rate object with a specific rate
    RC.loop_rate = std::make_unique<ros::Rate>(GLB_ROS_LOOP_RATE);

    // Initialize the "projection_cmd" subscriber
    RC.proj_cmd_sub = RC.node_handle->subscribe<std_msgs::Int32>("projection_cmd", 10, callbackProjCmdROS);
    if (!RC.proj_cmd_sub) ROS_ERROR("[appInitROS] Failed to subscribe to 'projection_cmd' topic!");

    // Initialize the "projection_image" subscriber
    RC.proj_img_sub = RC.node_handle->subscribe<std_msgs::Int32MultiArray>("projection_image", 10, callbackProjImgROS);
    if (!RC.proj_img_sub) ROS_ERROR("[appInitROS] Failed to subscribe to 'projection_image' topic!");

    // Initialize the "harness_pose_in_maze" subscriber
    RC.track_pos_sub = RC.node_handle->subscribe<geometry_msgs::PoseStamped>("harness_pose_in_maze", 1, callbackTrackPosROS);
    if (!RC.track_pos_sub) ROS_ERROR("[appInitROS] Failed to subscribe to 'harness_pose_in_maze' topic!");

    ROS_INFO("[projection_display:appInitROS] Finished initializing ROS successfully");
}

void appLoadAssets() {
    // ---------- Load Images with OpenCV ----------
    // Get the configured image paths
    std::vector<std::string> runtimeWallImages, runtimeFloorImages; // declare the vector to store the paths
    for (auto &fileName : RUNTIME_WALL_IMAGES)
        runtimeWallImages.push_back(RUNTIME_IMAGE_PATH + "/" + fileName);
    for (auto &filename : RUNTIME_FLOOR_IMAGES) // iterate through the file names
        runtimeFloorImages.push_back(RUNTIME_IMAGE_PATH + "/" + filename);

    //TODO: Move to dynamic loading - this is a static list that is memory inefficient
    if (loadImgMat(runtimeWallImages, runtimeWallMats) < 0)
        throw std::runtime_error("[appLoadAssets] Failed to load OpenCV wall images");

    if (loadImgMat(runtimeFloorImages, runtimeFloorMats) < 0)
        throw std::runtime_error("[appLoadAssets] Failed to load OpenCV floor images");

    // ---------- Load Wall and Floor Homography Matrices from XML ----------
    CalibrationXML calXML;
    for (int proj_ind = 0; proj_ind < N_PROJ; ++proj_ind) { // for each projector
        for (int cal_i = 0; cal_i < N_CAL_MODES; ++cal_i) {
            CalibrationMode _CAL_MODE = static_cast<CalibrationMode>(cal_i);

            // Store wall homography matrices
            if (_CAL_MODE == MODE_WALLS_LEFT || _CAL_MODE == MODE_WALLS_MIDDLE || _CAL_MODE == MODE_WALLS_RIGHT) {
                // Iterate through the maze grid rows
                for (int gr_i = 0; gr_i < GLB_MAZE_SIZE; ++gr_i) {
                    for (int gc_i = 0; gc_i < GLB_MAZE_SIZE; ++gc_i) {
                        // Load the homography matrix from XML
                        if (calXML.loadHMat(proj_ind, _CAL_MODE, gr_i, gc_i, WALL_HMAT_ARR[proj_ind][_CAL_MODE][gr_i][gc_i]) < 0)
                            throw std::runtime_error("[appLoadAssets] Error returned from:loadHMat");
                    }
                }
            }
            // Store floor homography matrices
            else if (calXML.loadHMat(proj_ind, _CAL_MODE, 0, 0, FLOOR_HMAT_ARR[proj_ind]) < 0) // Load the homography matrix from XML
                throw std::runtime_error("[appLoadAssets] Error returned from: loadHMat");
        }
    }

    // ---------- Load Maze Boundary Vertices ----------
    for (int proj_ind = 0; proj_ind < N_PROJ; ++proj_ind) { // for each projector
        std::vector<cv::Point2f> maze_vert_ndc_vec(N_PROJ);
        std::vector<cv::Point2f> maze_vert_cm_vec(N_PROJ);

        // Load the maze vertices from XML
        if (calXML.loadVertices(proj_ind, maze_vert_ndc_vec) < 0)
            throw std::runtime_error("[appLoadAssets] Error returned from:loadVertices");

        // Compute the rotated maze vertices in centimeter units
        computeMazeVertCm(proj_ind, maze_vert_cm_vec);

        // Compute the homography matrix for warping the rat mask marker from maze cm to ndc space for each projector
        cv::Mat H;
        if (computeHomographyMatrix(maze_vert_cm_vec, maze_vert_ndc_vec, H))
            throw std::runtime_error("[appLoadAssets] Projector[" + std::to_string(proj_ind) + "]: Invalid homography matrix for rat mask image");

        // Store the homography matrix
        HMAT_CM_TO_NDC_ARR[proj_ind] = H;
    }

    // Blank image maps as default
    blankMazeMap(MAZE_BLANK_MAP);
    blankMazeMap(MAZE_IMAGE_MAP);
    blankProjectionMap(PROJECTION_BLANK_MAP);
    blankProjectionMap(PROJECTION_IMAGE_MAP);

    ROS_INFO("[projection_display:appLoadAssets] Finished loading variables successfully");
}

void appInitVariables() {
    // ---------- Intialize the Window Offset Vector ---------
    winOffsetVec.clear();              // Clear any existing elements
    winOffsetVec.reserve(N_PROJ); // Reserve memory for efficiency
    for (int mon_ind = 0; mon_ind < N_PROJ; ++mon_ind) {
        // Calculate x and y offsets based on the monitor resolution
        int x_offset = mon_ind * (GLB_MONITOR_WIDTH_PXL / N_PROJ) * 0.9f;
        int y_offset = mon_ind * (GLB_MONITOR_HEIGHT_PXL / N_PROJ) * 0.9f;
        winOffsetVec.emplace_back(x_offset, y_offset);
    }

    ROS_INFO("[projection_display:appInitVariables] Finished initializing variables successfully");
}

void appInitOpenGL() {
    // Initialize GLFW and OpenGL settings and get number of monitors on the system
    if (SetupGraphicsLibraries() < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to initialize graphics");
    ROS_INFO("[appInitOpenGL] OpenGL initialized");

    // Initialize OpenGL for each projector
    for (int proj_ind = 0; proj_ind < N_PROJ; ++proj_ind) {
        // Start on the default screen
        int mon_ind = 0;
        ROS_INFO("[appInitOpenGL] Initializing OpenGL for Projector[%d] on Monitor[%d]", proj_ind, mon_ind);

        // Initialze render context for each projector
        if (PROJ_CTX_ARR[proj_ind].initWindowContext(proj_ind, mon_ind, GLB_MONITOR_WIDTH_PXL, GLB_MONITOR_HEIGHT_PXL, callbackKeyBinding) < 0)
            throw std::runtime_error("[appInitOpenGL] Failed to initialize render context");
        
        // Initialize OpenGL wall image objects
        if (PROJ_CTX_ARR[proj_ind].initRenderObjects(GLB_QUAD_GL_VERTICES, sizeof(GLB_QUAD_GL_VERTICES), GLB_QUAD_GL_INDICES, sizeof(GLB_QUAD_GL_INDICES)) < 0)
            throw std::runtime_error("[appInitOpenGL] Failed to initialize opengl wall image objects");

        // Create the shader program for wall image rendering
        if (PROJ_CTX_ARR[proj_ind].compileAndLinkShaders(GLB_QUAD_GL_VERTEX_SOURCE, GLB_QUAD_GL_FRAGMENT_SOURCE) < 0)
            throw std::runtime_error("[appInitOpenGL] Window[" + std::to_string(PROJ_CTX_ARR[proj_ind].windowInd) + "]: Failed to compile and link wall shader");

        // Create the shader program for CircleRenderer class rat mask rendering
        if (CircleRenderer::CompileAndLinkCircleShaders(1.0) < 0)
            throw std::runtime_error("[appInitOpenGL] Failed to compile and link circlerenderer class shader");

        // Set all projectors to the starting monitor and include xy offset
        if (PROJ_CTX_ARR[proj_ind].changeWindowDisplayMode(mon_ind, FLAG_FULLSCREEN_MODE, winOffsetVec[PROJ_CTX_ARR[proj_ind].windowInd]) < 0)
            throw std::runtime_error("[appInitOpenGL] Window[" + std::to_string(PROJ_CTX_ARR[proj_ind].windowInd) + "]: Failed Initial update of window monitor mode");

        // Initialize the CircleRenderer class object for rat masking
        if (RM_CIRCREND_ARR[proj_ind].initializeCircleObject(
                RT.marker_position,          // position
                RT.marker_radius,            // radius
                RT.marker_rgb,               // color
                RT.marker_segments,          // segments
                HMAT_CM_TO_NDC_ARR[proj_ind] // homography matrix
                ) < 0)
            throw std::runtime_error("[appInitOpenGL] Failed to initialize CircleRenderer class object");

        ROS_INFO("[appInitOpenGL] OpenGL initialized: Projector[%d] Window[%d] Monitor[%d]", proj_ind, PROJ_CTX_ARR[proj_ind].windowInd, PROJ_CTX_ARR[proj_ind].monitorInd);
    }
    ROS_INFO("[appInitOpenGL] OpenGL contexts and objects Initialized succesfully");
}


void appMainLoop()
{
    displayTimer[0].start();

    // --------------- Check State Flags ---------------
    // Check if the window mode needs to be changed
    if (FLAG_CHANGE_WINDOW_MODE) {
        for (auto &projCtx : PROJ_CTX_ARR) {
            int mon_ind = FLAG_WINDOWS_SET_TO_PROJ ? PROJ_MON_VEC[projCtx.windowInd] : 0;
            if (projCtx.changeWindowDisplayMode(mon_ind, FLAG_FULLSCREEN_MODE, winOffsetVec[projCtx.windowInd]) < 0)
                throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from: changeWindowDisplayMode");
        }
    }

    // Force to windows so thay are on top
    if (FLAG_FORCE_WINDOW_FOCUS) {
        for (auto &projCtx : PROJ_CTX_ARR) {
            if (projCtx.forceWindowFocus() < 0)
                throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from: forceWindowFocus");
        }
    }

    displayTimer[0].update(true); // Update the timing data for the first displayTimer
    displayTimer[1].start(); // Start the second displayTimer for updating textures

    // TEST Simulate rat movement for testing
    // simulateRatMovement(0.5f, 45.0f, RT);

    // Recompute wall parameters and update wall image texture
    if (FLAG_UPDATE_TEXTURES) {
        for (auto &projCtx : PROJ_CTX_ARR) {
            cv::Mat imgMat = BLANK_PXL_MAT.clone();

            // Update floor image texture
            if (updateFloorTexture(projCtx.windowInd, imgMat))
                throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Failed to update wall texture");

            // Update wall image texture
            if (updateWallTexture(projCtx.windowInd, imgMat))
                throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Failed to update wall texture");

            // Load the new texture
            if (projCtx.loadMatTexture(imgMat) < 0)
                throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Failed to load wall texture");
        }
    }

    // Reset keybinding flags
    FLAG_CHANGE_WINDOW_MODE = false;
    FLAG_UPDATE_TEXTURES = false;
    FLAG_FORCE_WINDOW_FOCUS = false;

    displayTimer[1].update(true); // Update the timing data for the second displayTimer
    displayTimer[2].start(); // Start the third displayTimer for image processing

    // --------------- Handle image processing for each projector ---------------
    for (auto &projCtx : PROJ_CTX_ARR) {
        // Prepare the frame for rendering (clear the back buffer)
        if (projCtx.initWindowForDrawing() < 0)
            throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from: initWindowForDrawing");

        // Draw/update wall images
        if (projCtx.drawTexture() < 0)
            throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from: drawTexture");

        // Draw/update rat mask marker
        if (drawRatMask(RT, RM_CIRCREND_ARR[projCtx.windowInd]) < 0)
            throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from: drawRatMask");

        // Swap buffers and poll events
        if (projCtx.bufferSwapPoll() < 0)
            throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from: bufferSwapPoll");

        // Check if ROS shutdown
        if (!ros::ok())
            throw std::runtime_error("[appMainLoop] ROS is no longer running!");

        // Get exit request status
        if (!projCtx.checkExitRequest()) continue;
    }

    displayTimer[2].update(true); // Update the timing data for the third displayTimer
    displayTimer[3].start(); // Start the fourth displayTimer for ROS operations

    // --------------- Handle ROS Messages and Operations ---------------
    // Process a single round of callbacks for ROS messages
    ros::spinOnce();

    displayTimer[3].update(true); // Update the timing data for the fourth timer

    // Sleep to maintain the loop rate
    RC.loop_rate->sleep();
}

void appCleanup() {
    ROS_INFO("SHUTTING DOWN");

    // Clean up OpenGL wall image objects for each window
    for (int proj_ind = 0; proj_ind < N_PROJ; ++proj_ind) {
        if (PROJ_CTX_ARR[proj_ind].cleanupContext(true) != 0)
            ROS_WARN("[appCleanup] Error during cleanup of MazeRenderContext: Projector[%d] Window[%d] Monitor[%d]",
                     proj_ind, PROJ_CTX_ARR[proj_ind].windowInd, PROJ_CTX_ARR[proj_ind].monitorInd);
        else
            ROS_INFO("[projection_display:appCleanup] MazeRenderContext instance cleaned up successfully: Projector[%d] Window[%d] Monitor[%d]",
                     proj_ind, PROJ_CTX_ARR[proj_ind].windowInd, PROJ_CTX_ARR[proj_ind].monitorInd);
    }

    // Terminate graphics
    if (CleanupGraphicsLibraries() < 0)
        ROS_WARN("[appCleanup] Failed to terminate GLFW library");
    else
        ROS_INFO("[projection_display:appCleanup] GLFW library terminated successfully");
}

int main(int argc, char **argv) {
    // try {
        appInitROS(argc, argv); // Initialize ROS node and subscribers
        appLoadAssets();
        appInitVariables();
        appInitOpenGL();

        TIMING_ENABLED = false; // Disable timing for now
        for (auto &timer : displayTimer) {
            timer.reset();
        }
        displayTimer[0].name = "Window_operations";
        displayTimer[1].name = "Update_textures";
        displayTimer[2].name = "Image_processing";
        displayTimer[3].name = "ROS_operations";
        displayTimer[4].name = "Floor_texture_update";
        displayTimer[5].name = "Wall_texture_update";
        displayTimer[6].name = "Warp";
        displayTimer[7].name = "Merge";

        while(ros::ok()) appMainLoop(); // Main loop for rendering and processing
    // }
    // catch (const std::exception &e) {
    //     ROS_ERROR("!!EXCEPTION CAUGHT!!: %s", e.what());
    //     void appCleanup();
    //     return -1;
    // }
    return 0;
}