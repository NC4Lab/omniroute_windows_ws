// ########################################################################################################

// ======================================== projection_display.cpp ========================================

// ########################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_display.h"

// ================================================== FUNCTIONS ==================================================

int KEY_WALL_IMG_IND = 0; // Global variable to track the wall image index for key binding
int KEY_FLOOR_IMG_IND = 0; // Global variable to track the floor image index for key binding

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

    // _______________ ANY KEY PRESS OR REPEAT ACTION _______________
    else if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        // ---------- Change wall configuration [SHIFT [L-R]] ----------
        if (mods & GLFW_MOD_SHIFT) {
            if (key == GLFW_KEY_LEFT)
                KEY_WALL_IMG_IND = (KEY_WALL_IMG_IND - 1) % N_RUNTIME_WALL_IMAGES;
            else if (key == GLFW_KEY_RIGHT)
                KEY_WALL_IMG_IND = (KEY_WALL_IMG_IND + 1) % N_RUNTIME_WALL_IMAGES;
            else
                return; // Ignore other keys

            ROS_INFO("[callbackKeyBinding] Initiated change wall image configuration to %d", KEY_WALL_IMG_IND);

            // Loop through the data and set all entries to a given wall ind
            for (auto &proj : PROJECTORS)
                for (auto &row : ROWS)
                    for (auto &col : COLS)
                        for (auto &mode : WALL_CAL_MODES)
                            PROJECTION_IMAGE_MAP[proj][row][col][mode] = KEY_WALL_IMG_IND; // Set each element to the new wall image index

            // Set the flag to update the textures
            FLAG_UPDATE_TEXTURES = true;
        }

        // ---------- Change floor configuration [CTRL [L-R]] ----------
        else if (mods & GLFW_MOD_CONTROL) {
            if (key == GLFW_KEY_LEFT)
                KEY_FLOOR_IMG_IND = (KEY_FLOOR_IMG_IND - 1) % N_RUNTIME_FLOOR_IMAGES;
            else if (key == GLFW_KEY_RIGHT)
                KEY_FLOOR_IMG_IND = (KEY_FLOOR_IMG_IND + 1) % N_RUNTIME_FLOOR_IMAGES;
            else
                return; // Ignore other keys

            ROS_INFO("[callbackKeyBinding] Initiated change floor image configuration to %d", KEY_FLOOR_IMG_IND);

            for (auto &proj : PROJECTORS)
                PROJECTION_IMAGE_MAP[proj][0][0][MODE_FLOOR] = KEY_FLOOR_IMG_IND; // Set element to the new floor image index

            // Set the flag to update the textures
            FLAG_UPDATE_TEXTURES = true;
        }
    }
}

void callbackProjCmdROS(const std_msgs::Int32::ConstPtr &msg) {
    // Log projection command
    if (GLB_DO_VERBOSE_DEBUG)
        ROS_INFO("[callbackProjCmdROS] Received projection command: %d", msg->data);

    // ------- Monitor Mode Change Commmands ----------
    if (msg->data == -1) {  // Move monitor command [-1]
        FLAG_WINDOWS_SET_TO_PROJ = !FLAG_WINDOWS_SET_TO_PROJ;
        FLAG_CHANGE_WINDOW_MODE = true;
    }
    else if (msg->data == -2) { // Set/unset Fullscreen [-2] ----------
        FLAG_FULLSCREEN_MODE = !FLAG_FULLSCREEN_MODE;
        FLAG_CHANGE_WINDOW_MODE = true;
    }
    else if (msg->data == -3) // Force window to top [-3] ----------
        FLAG_FORCE_WINDOW_FOCUS = true;
    else ROS_WARN("[procProjCmdROS] Received invalid projection command: %d", msg->data);
}

void callbackProjImgROS(const std_msgs::Int32MultiArray::ConstPtr &msg) {
    // Expects a N_CHAMBERS x N_SURF array of integers
    // where N_CHAMBERS = 9 and N_SURF = 9 (8 walls + 1 floor)
    if (msg->data.size() != N_CHAMBERS * N_SURF) {
        ROS_ERROR("[callbackProjImgROS] Received incorrect array size. Expected %d elements, but got %zu", 
            N_CHAMBERS * N_SURF, msg->data.size());
        return;
    }

    // Store the maze image data in the global maze image map
    for (auto &cham : CHAMBERS)
        for (auto &surf : SURFACES)
            MAZE_IMAGE_MAP[cham][surf] = msg->data[cham * N_SURF + surf];
    
    // Convert the received data to a projection map
    mazeToProjectionMap(MAZE_IMAGE_MAP, PROJECTION_IMAGE_MAP);

    if (GLB_DO_VERBOSE_DEBUG) {
        // Log the entire maze image map
        ROS_INFO("[callbackProjImgROS] Maze image map:");
        for (auto &cham : CHAMBERS) {
            std::stringstream row_stream;
            row_stream << "Chamber[" << cham << "] = [";
            for (auto &surf : SURFACES) {
                row_stream << MAZE_IMAGE_MAP[cham][surf];
                if (surf != SURFACES.back()) // Add a comma between elements, but not after the last one
                    row_stream << ", ";
            }
            row_stream << "]";
            ROS_INFO("%s", row_stream.str().c_str());
        }
    }
    // Set the flag to update the textures
    FLAG_UPDATE_TEXTURES = true;
}

void callbackTrackPosROS(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // Update the last received command
    RC.track_pos_data = *msg;

    // Log position data
    if (GLB_DO_VERBOSE_DEBUG)
        ROS_INFO("[callbackHarnessPosROS] Received tracking: position x[%f] y[%f] z[%f], orientation x[%f] y[%f] z[%f]",
                 RC.track_pos_data.pose.position.x,
                 RC.track_pos_data.pose.position.y,
                 RC.track_pos_data.pose.position.z,
                 RC.track_pos_data.pose.orientation.x,
                 RC.track_pos_data.pose.orientation.y,
                 RC.track_pos_data.pose.orientation.z);
    
    // Convert the track pose to centimeters
    geometry_msgs::Point position_cm;
    position_cm.x = RC.track_pos_data.pose.position.x * 100.0f;
    position_cm.y = RC.track_pos_data.pose.position.y * 100.0f;

    // Extract the quaternion
    tf::Quaternion q(
        RC.track_pos_data.pose.orientation.x,
        RC.track_pos_data.pose.orientation.y,
        RC.track_pos_data.pose.orientation.z,
        RC.track_pos_data.pose.orientation.w);

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

//TODO: Integrate into others
int initSubscriberROS() {
    // Check if node handle is initialized
    if (!RC.node_handle) {
        ROS_ERROR("[initSubscriberROS]Node handle is not initialized!");
        return -1;
    }

    // Initialize the "projection_cmd" subscriber using boost::bind
    RC.proj_cmd_sub = RC.node_handle->subscribe<std_msgs::Int32>(
        "projection_cmd", 10, boost::bind(&callbackProjCmdROS, _1));
    if (!RC.proj_cmd_sub) {
        ROS_ERROR("[initSubscriberROS]Failed to subscribe to 'projection_cmd' topic!");
        return -1;
    }

    // Initialize the "projection_image" subscriber
    RC.proj_img_sub = RC.node_handle->subscribe<std_msgs::Int32MultiArray>(
        "projection_image", 10, boost::bind(&callbackProjImgROS, _1));
    if (!RC.proj_img_sub) {
        ROS_ERROR("[initSubscriberROS]Failed to subscribe to 'projection_image' topic!");
        return -1;
    }

    // Initialize the "harness_pose_in_maze" subscriber
    RC.track_pos_sub = RC.node_handle->subscribe<geometry_msgs::PoseStamped>(
        "harness_pose_in_maze", 10, boost::bind(&callbackTrackPosROS, _1));
    if (!RC.track_pos_sub) {
        ROS_ERROR("[initSubscriberROS]Failed to subscribe to 'harness_pose_in_maze' topic!");
        return -1;
    }
    return 0;
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
            marker_angle += rand() % 181 - 90; // Random turn between -90 and +90 degrees
    };

    // Randomly decide to change direction
    if (rand() % 10 == 0) { // 10% chance to change direction
        marker_angle += (rand() % static_cast<int>(2 * max_turn_angle)) - max_turn_angle;
    }

    // Calculate new position
    float radian_angle = marker_angle * M_PI / 180.0f;
    RT.marker_position.x += move_step * cos(radian_angle);
    RT.marker_position.y += move_step * sin(radian_angle);

    // Keep the rat within the enclosure and turn if hitting the wall
    keepWithinBoundsAndTurn(RT.marker_position);
}

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
    if (proj_ind == 0) maze_vert_cm_vec = circShift(template_maze_vert_cm_vec, 1);
    else if (proj_ind == 1) maze_vert_cm_vec = circShift(template_maze_vert_cm_vec, 2);
    else if (proj_ind == 2) maze_vert_cm_vec = circShift(template_maze_vert_cm_vec, -1);
    else if (proj_ind == 3) maze_vert_cm_vec = circShift(template_maze_vert_cm_vec, 0);
}

int updateFloorTexture(int proj_ind, bool do_ignore_blank_img, cv::Mat &out_img_mat) {
    int img_ind = PROJECTION_IMAGE_MAP[proj_ind][0][0][MODE_FLOOR];

    // Skip empty images
    if (img_ind == 0 && do_ignore_blank_img) return 0;

    // Warp Perspective
    cv::Mat img_warp;
    cv::warpPerspective(runtimeFloorMats[img_ind], img_warp, HMAT_MAP[proj_ind][0][0][MODE_FLOOR], 
        cv::Size(GLB_MONITOR_WIDTH_PXL, GLB_MONITOR_HEIGHT_PXL));

    // Merge the warped image with the final image
    mergeImgMat(img_warp, out_img_mat);
    return 0;
}

int updateWallTexture(int proj_ind, bool do_ignore_blank_img, cv::Mat &out_img_mat) {
    // Iterate through through calibration modes (left walls, middle walls, right walls)
    cv::Mat img_warp;
    for (auto cal_mode : WALL_CAL_MODES) {
        for (auto row : ROWS) {
            for (auto col : COLS) {
                //TODO: Update only new indices
                int img_ind = PROJECTION_IMAGE_MAP[proj_ind][row][col][cal_mode];

                // Skip empty images
                if (img_ind == 0 && do_ignore_blank_img) continue;

                if (img_ind < N_WARPED_WALL_IMAGES)
                    img_warp = WARPED_RUNTIME_WALL_MATS[img_ind][proj_ind][row][col][cal_mode]; // Use precomputed warped image
                else
                    cv::warpPerspective(runtimeWallMats[img_ind], img_warp, HMAT_MAP[proj_ind][row][col][cal_mode],
                        cv::Size(GLB_MONITOR_WIDTH_PXL, GLB_MONITOR_HEIGHT_PXL));

                // Merge the warped image with the final image
                mergeImgMat(img_warp, out_img_mat);
            }
        }
    }
    return 0;
}

void drawRatMask(CircleRenderer &out_rmCircRend) {
    // Setup the CircleRenderer class shaders
    CircleRenderer::SetupShader();

    // Set the marker position
    out_rmCircRend.setPosition(RT.marker_position);

    // Recompute the marker parameters
    out_rmCircRend.updateCircleObject(true);

    // Draw the marker
    out_rmCircRend.draw();

    // Unset the shader program
    CircleRenderer::UnsetShader();
}

void appInitROS(int argc, char **argv) {
    ROS_INFO("[projection_display:appInitROS] STARTING PROJECTION DISPLAY NODE");

    // Initialize ROS
    ros::init(argc, argv, "projection_display", ros::init_options::AnonymousName);
    if (!ros::master::check())
        throw std::runtime_error("[appInitROS] Failed to initialize ROS: ROS master is not running");

    // Initialize NodeHandle inside RC
    RC.node_handle = std::make_unique<ros::NodeHandle>();

    // Initialize the ros::Rate object with a specific rate
    RC.loop_rate = std::make_unique<ros::Rate>(GLB_ROS_LOOP_RATE);

    // Initialize the subscribers
    if (initSubscriberROS() < 0)
        throw std::runtime_error("[appInitROS] Failed to initialize ROS subscriber");

    ROS_INFO("[projection_display:appInitROS] Finished initializing ROS successfully");
}

void prewarpWallImages() {
    cv::Mat img_warp;
    for (int image_id = 0; image_id < N_WARPED_WALL_IMAGES; ++image_id) {
        ROS_INFO("[prewarpWallImages] Prewarping wall image %d of %d", image_id + 1, N_WARPED_WALL_IMAGES);
        for (auto &proj : PROJECTORS) {
            for (auto &row : ROWS) {
                for (auto &col : COLS) {
                    for (auto &cal_mode : WALL_CAL_MODES) {

                        // Warp Perspective
                        cv::warpPerspective(runtimeWallMats[image_id], img_warp, HMAT_MAP[proj][row][col][cal_mode],
                            cv::Size(GLB_MONITOR_WIDTH_PXL, GLB_MONITOR_HEIGHT_PXL));

                        // Store the warped image
                        WARPED_RUNTIME_WALL_MATS[image_id][proj][row][col][cal_mode] = img_warp.clone();
                    }
                }
            }
        }
    }
}

void appLoadAssets()
{
    // Make wall and floor images available to the ROS parameter server
    ros::param::set("runtime_wall_images", RUNTIME_WALL_IMAGES);
    ros::param::set("runtime_floor_images", RUNTIME_FLOOR_IMAGES);

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
    // Load wall homography matrices
    for (auto &proj : PROJECTORS) { // for each projector
        for (auto &row : ROWS)
            for (auto &col : COLS)
                for (auto &cal_mode : WALL_CAL_MODES)
                    if (xmlLoadHMAT(proj, cal_mode, row, col, HMAT_MAP[proj][row][col][cal_mode]) < 0)
                        throw std::runtime_error("[appLoadAssets] Error returned from: xmlLoadHMAT");

        // Load floor homography matrices (one for each projector, for now)
        if (xmlLoadHMAT(proj, MODE_FLOOR, 0, 0, HMAT_MAP[proj][0][0][MODE_FLOOR]) < 0)
                throw std::runtime_error("[appLoadAssets] Error returned from: xmlLoadHMAT");
        
        double floor_rot_deg = 270.0 - proj * 90.0; // Default rotation for each projector
        // Incorporate rotation into the floor homography matrix
        cv::Mat rotation_mat = cv::getRotationMatrix2D(cv::Point2f(GLB_MAZE_IMAGE_WIDTH_PXL / 2, GLB_MAZE_IMAGE_HEIGHT_PXL / 2), floor_rot_deg, 1.0);
        // Add [0, 0, 1] to the rotation matrix to make it a full homography matrix
        rotation_mat.push_back(cv::Mat::zeros(1, 3, CV_64F));
        rotation_mat.at<double>(2, 2) = 1.0; // Set the last element to 1 to make it a valid homography matrix
        // Apply the rotation to the floor homography matrix
        HMAT_MAP[proj][0][0][MODE_FLOOR] = HMAT_MAP[proj][0][0][MODE_FLOOR] * rotation_mat;
    }
    // Prewarp the images 
    prewarpWallImages();

    // ---------- Load Maze Boundary Vertices ---------
    for (auto proj : PROJECTORS) { // for each projector
        std::vector<cv::Point2f> maze_vert_ndc_vec(4);
        std::vector<cv::Point2f> maze_vert_cm_vec(4);

        // Load the maze vertices from XML
        if (xmlLoadVertices(proj, maze_vert_ndc_vec) < 0)
            throw std::runtime_error("[appLoadAssets] Error returned from: xmlLoadVertices");

        // Compute the rotated maze vertices in centimeter units
        computeMazeVertCm(proj, maze_vert_cm_vec);

        // Compute the homography matrix for warping the rat mask marker from maze cm to ndc space for each projector
        cv::Mat H;
        if (computeHomographyMatrix(maze_vert_cm_vec, maze_vert_ndc_vec, H))
            throw std::runtime_error("[appLoadAssets] Projector[" + std::to_string(proj) + "]: Invalid homography matrix for rat mask image");

        // Store the homography matrix
        HMAT_CM_TO_NDC_ARR[proj] = H;
    }

    ROS_INFO("[projection_display:appLoadAssets] Finished loading variables successfully");
}

void appInitVariables() {
    // ---------- Intialize the Window Offset Vector ---------
    winOffsetVec.clear();              // Clear any existing elements
    winOffsetVec.reserve(N_PROJ); // Reserve memory for efficiency
    for (auto proj : PROJECTORS) {
        // Calculate x and y offsets based on the monitor resolution
        int x_offset = proj * (GLB_MONITOR_WIDTH_PXL / N_PROJ) * 0.9f;
        int y_offset = proj * (GLB_MONITOR_HEIGHT_PXL / N_PROJ) * 0.9f;
        winOffsetVec.emplace_back(x_offset, y_offset);
    }

    ROS_INFO("[projection_display:appInitVariables] Finished initializing variables successfully");
}

void appInitOpenGL() {
    // Initialize GLFW and OpenGL settings and get number of monitors on the system
    if (SetupGraphicsLibraries() < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to initialize graphics");
    ROS_INFO("[projection_display:appInitOpenGL] OpenGL initialized: Projector monitor indices: %d, %d, %d, %d", PROJ_MON_VEC[0], PROJ_MON_VEC[1], PROJ_MON_VEC[2], PROJ_MON_VEC[3]);

    // Initialize OpenGL for each projector
    for (auto proj : PROJECTORS)
    {
        // Start on the default screen
        int mon_ind = STARTING_MONITOR;

        // Initialze render context for each projector
        if (PROJ_CTX_VEC[proj].initWindowContext(proj, mon_ind, GLB_MONITOR_WIDTH_PXL, GLB_MONITOR_HEIGHT_PXL, callbackKeyBinding) < 0)
            throw std::runtime_error("[appInitOpenGL] Failed to initialize render context");

        // Initialize OpenGL wall image objects
        if (PROJ_CTX_VEC[proj].initRenderObjects(GLB_QUAD_GL_VERTICES, sizeof(GLB_QUAD_GL_VERTICES), GLB_QUAD_GL_INDICES, sizeof(GLB_QUAD_GL_INDICES)) < 0)
            throw std::runtime_error("[appInitOpenGL] Failed to initialize opengl wall image objects");

        // Create the shader program for wall image rendering
        if (PROJ_CTX_VEC[proj].compileAndLinkShaders(GLB_QUAD_GL_VERTEX_SOURCE, GLB_QUAD_GL_FRAGMENT_SOURCE) < 0)
            throw std::runtime_error("[appInitOpenGL] Window[" + std::to_string(PROJ_CTX_VEC[proj].windowInd) + "]: Failed to compile and link wall shader");

        // Create the shader program for CircleRenderer class rat mask rendering
        if (CircleRenderer::CompileAndLinkCircleShaders(1.0) < 0)
            throw std::runtime_error("[appInitOpenGL] Failed to compile and link circlerenderer class shader");

        // Set all projectors to the starting monitor and include xy offset
        PROJ_CTX_VEC[proj].changeWindowDisplayMode(mon_ind, FLAG_FULLSCREEN_MODE, winOffsetVec[PROJ_CTX_VEC[proj].windowInd]);

        // Initialize the CircleRenderer class object for rat masking
        if (RM_CIRCREND_ARR[proj].initializeCircleObject(
                RT.marker_position,          // position
                RT.marker_radius,            // radius
                RT.marker_rgb,               // color
                RT.marker_segments,          // segments
                HMAT_CM_TO_NDC_ARR[proj] // homography matrix
                ) < 0)
            throw std::runtime_error("[appInitOpenGL] Failed to initialize CircleRenderer class object");

        ROS_INFO("[projection_display:appInitOpenGL] OpenGL initialized: Projector[%d] Window[%d] Monitor[%d]", proj, PROJ_CTX_VEC[proj].windowInd, PROJ_CTX_VEC[proj].monitorInd);
    }

    ROS_INFO("[projection_display:appInitOpenGL] OpenGL contexts and objects Initialized succesfully");
}


void projectorLoop(MazeRenderContext &projCtx) {
    projCtx.makeContextCurrent(); // Make the OpenGL context current for this thread
    
    // --------------- Check State Flags ---------------
    if (FLAG_CHANGE_WINDOW_MODE) {
        int mon_ind = FLAG_WINDOWS_SET_TO_PROJ ? PROJ_MON_VEC[projCtx.windowInd] : STARTING_MONITOR;
        projCtx.changeWindowDisplayMode(mon_ind, FLAG_FULLSCREEN_MODE, winOffsetVec[projCtx.windowInd]);
    }

    if (FLAG_FORCE_WINDOW_FOCUS) projCtx.forceWindowFocus();

    // Recompute wall parameters and update wall image texture
    if (FLAG_UPDATE_TEXTURES) {
        cv::Mat img_mat = WALL_BLANK_IMG_MAT.clone(); // Initialize the image matrix to blank

        //TODO: Only update if different
        if (updateFloorTexture(projCtx.windowInd, false, img_mat))
            throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Failed to update floor texture");

        // Update wall image texture
        if (updateWallTexture(projCtx.windowInd, false, img_mat))
            throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Failed to update wall texture");

        // Load the new texture
        projCtx.loadMatTexture(img_mat);
    }
    // --------------- Handle Image Processing for Next Frame ---------------
    // Prepare the frame for rendering (clear the back buffer)
    projCtx.initWindowForDrawing();
    // Draw/update wall images
    projCtx.drawTexture();
    // Draw/update rat mask marker
    //TODO: This should be in the MazeRenderContext class
    drawRatMask(RM_CIRCREND_ARR[projCtx.windowInd]);
    // Swap buffers and poll events
    projCtx.bufferSwapPoll();
}

// std::array<std::thread, N_PROJ> projThreads; // Array to hold projector threads

int appMainLoop() {
        int status = 0; // Initialize status to 0 (no error)
        displayTimer[0].start(); // Start the timer for the loop

        // TEMP Simulate rat movement for testing
        // simulateRatMovement(0.5f, 45.0f);

        // Run projector loop
        for (auto proj : PROJECTORS)
            projectorLoop(PROJ_CTX_VEC[proj]); // Call the projector loop for each context

        // Reset keybinding flags
        FLAG_CHANGE_WINDOW_MODE = false;
        FLAG_UPDATE_TEXTURES = false;
        FLAG_FORCE_WINDOW_FOCUS = false;

        // --------------- Handle ROS Messages and Operations ---------------
        // Process a single round of callbacks for ROS messages
        ros::spinOnce();

        displayTimer[0].update(true);
        // Sleep to maintain the loop rate
        RC.loop_rate->sleep();

        return status; // Return the status of the main loop
}

void appCleanup() {
    ROS_INFO("SHUTTING DOWN");

    // Clean up OpenGL wall image objects for each window
    for (auto proj : PROJECTORS) {
        if (PROJ_CTX_VEC[proj].cleanupContext(true) != 0)
            ROS_WARN("[appCleanup] Error during cleanup of MazeRenderContext: Projector[%d] Window[%d] Monitor[%d]",
                     proj, PROJ_CTX_VEC[proj].windowInd, PROJ_CTX_VEC[proj].monitorInd);
        else
            ROS_INFO("[projection_display:appCleanup] MazeRenderContext instance cleaned up successfully: Projector[%d] Window[%d] Monitor[%d]",
                     proj, PROJ_CTX_VEC[proj].windowInd, PROJ_CTX_VEC[proj].monitorInd);
    }

    // Terminate graphics
    if (CleanupGraphicsLibraries() < 0)
        ROS_WARN("[appCleanup] Failed to terminate GLFW library");
    else
        ROS_INFO("[projection_display:appCleanup] GLFW library terminated successfully");
}

int main(int argc, char **argv) {
    try {
        appInitROS(argc, argv);
        appLoadAssets();
        appInitVariables();
        appInitOpenGL();

        // Setup timers
        for (auto &t: displayTimer) t.reset();

        displayTimer[0].name = "Main Loop";
        displayTimer[1].name = "Update Floor Texture";
        displayTimer[2].name = "Update Wall Texture";
        displayTimer[3].name = "Load Texture";
    
        int mainLoopStatus = 0;
        while (ros::ok() && !mainLoopStatus) mainLoopStatus = appMainLoop();
    }
    catch (const std::exception &e) {
        ROS_ERROR("!!EXCEPTION CAUGHT!!: %s", e.what());
        appCleanup();
        return -1;
    }
    return 0;
}