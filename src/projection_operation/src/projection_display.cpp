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

        // ----------Set/unset Fullscreen [F] ----------

        if (key == GLFW_KEY_F)
        {
            F.fullscreen_mode = !F.fullscreen_mode;
            F.change_window_mode = true;
        }

        // ----------Check for move monitor command [M] ----------

        if (key == GLFW_KEY_M)
        {
            F.windows_set_to_proj = !F.windows_set_to_proj;
            F.change_window_mode = true;
        }
    }
}

void simulateRatMovement(float move_step, float max_turn_angle, RatTracker &out_RT)
{
    // Track marker angle
    static float marker_angle = 0.0f;

    // Lambda function to keep the rat within the enclosure and turn when hitting the wall
    auto keepWithinBoundsAndTurn = [&](cv::Point2f &position)
    {
        cv::Point2f original_position = position;
        position.x = std::min(std::max(position.x, 0.0f), MAZE_WIDTH_HEIGHT_CM);
        position.y = std::min(std::max(position.y, 0.0f), MAZE_WIDTH_HEIGHT_CM);

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
    float radian_angle = marker_angle * PI / 180.0f;
    out_RT.marker_position.x += move_step * cos(radian_angle);
    out_RT.marker_position.y += move_step * sin(radian_angle);

    // Keep the rat within the enclosure and turn if hitting the wall
    keepWithinBoundsAndTurn(out_RT.marker_position);
}

void populateMazeVertNdcVec(int proj_mon_ind, std::vector<cv::Point2f> &out_rotated_vec)
{
    // Lambda function for circular shift
    auto circShift = [](const std::vector<cv::Point2f> &vec, int shift) -> std::vector<cv::Point2f>
    {
        std::vector<cv::Point2f> shifted_vec(vec.size());
        int n = vec.size();
        for (int i = 0; i < n; ++i)
        {
            shifted_vec[i] = vec[(i + shift + n) % n];
        }
        return shifted_vec;
    };

    // Template vertices
    const std::vector<cv::Point2f> template_maze_vert_cm_vec = {
        cv::Point2f(0, MAZE_WIDTH_HEIGHT_CM),
        cv::Point2f(MAZE_WIDTH_HEIGHT_CM, MAZE_WIDTH_HEIGHT_CM),
        cv::Point2f(MAZE_WIDTH_HEIGHT_CM, 0.0),
        cv::Point2f(0.0, 0.0)};

    // Apply circular shift based on proj_mon_ind
    switch (proj_mon_ind)
    {
    case 0: // Circular shift left by 1
        out_rotated_vec = circShift(template_maze_vert_cm_vec, -1);
        break;
    case 1: // No shift
        out_rotated_vec = template_maze_vert_cm_vec;
        break;
    case 2: // Circular shift right by 1
        out_rotated_vec = circShift(template_maze_vert_cm_vec, 1);
        break;
    case 3: // Circular shift right by 2
        out_rotated_vec = circShift(template_maze_vert_cm_vec, 2);
        break;
    default:
        break;
    }
}

int updateTexture(
    int proj_mon_ind,
    const std::vector<cv::Mat> &_wallImgMatVec,
    const std::vector<cv::Mat> &_floorImgMatVec,
    const std::array<std::array<std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>, N_CAL_MODES>, 4> &_HMAT_ARR,
    MazeRenderContext &out_projCtx)
{
    // Initialize the image to be used as the texture
    cv::Mat img_merge = cv::Mat::zeros(WINDOW_HEIGHT_PXL, WINDOW_WIDTH_PXL, CV_8UC4);

    // Iterate through through calibration modes in descending order so floor is drawn first
    for (int cal_i = N_CAL_MODES - 1; cal_i >= 0; --cal_i)
    {
        CalibrationMode _CAL_MODE = static_cast<CalibrationMode>(cal_i);

        // TEMP
        if (_CAL_MODE != FLOOR)
            continue;

        // Specify number of rows/cols to loop through
        int grid_size = (_CAL_MODE == WALLS_LEFT || _CAL_MODE == WALLS_MIDDLE || _CAL_MODE == WALLS_RIGHT) ? MAZE_SIZE : 1;

        // Iterate through the maze grid rows
        for (int gr_i = 0; gr_i < grid_size; gr_i++) // image bottom to top
        {
            // Iterate through each column in the maze row
            for (int gc_i = 0; gc_i < grid_size; gc_i++) // image left to right
            {
                // Get the wall image to be used
                cv::Mat img_copy;
                if (_CAL_MODE == WALLS_LEFT || _CAL_MODE == WALLS_MIDDLE || _CAL_MODE == WALLS_RIGHT)
                {
                    int img_ind = WALL_IMG_PROJ_MAP[proj_mon_ind][gr_i][gc_i][_CAL_MODE];
                    if (_wallImgMatVec[img_ind].empty())
                    {
                        ROS_ERROR("[updateTexture] Stored OpenCV wall image is empty: Window[%d] Monitor[%d] Wall[%d][%d] Calibration[%d] Image[%d]",
                                  out_projCtx.windowInd, proj_mon_ind, gr_i, gc_i, _CAL_MODE, img_ind);
                        return -1;
                    }
                    else
                    {
                        _wallImgMatVec[img_ind].copyTo(img_copy);
                    }
                }
                else
                {
                    int img_ind = MAZE_IMG_PROJ_MAP[proj_mon_ind];
                    if (_floorImgMatVec[img_ind].empty())
                    {
                        ROS_ERROR("[updateTexture] Stored OpenCV floor image is empty: Window[%d] Monitor[%d] Wall[%d][%d] Calibration[%d] Image[%d]",
                                  out_projCtx.windowInd, proj_mon_ind, gr_i, gc_i, _CAL_MODE, img_ind);
                        return -1;
                    }
                    else
                    {
                        _floorImgMatVec[img_ind].copyTo(img_copy);
                    }
                }

                // Get homography matrix for this wall
                cv::Mat H = _HMAT_ARR[proj_mon_ind][_CAL_MODE][gr_i][gc_i];

                // Warp Perspective
                cv::Mat img_warp;
                if (warpImgMat(img_copy, H, img_warp) < 0)
                {
                    ROS_ERROR("[updateTexture] Warp image error: Window[%d] Monitor[%d] Wall[%d][%d] Calibration[%d]",
                              out_projCtx.windowInd, proj_mon_ind, gr_i, gc_i, _CAL_MODE);
                    return -1;
                }

                // Merge the warped image with the final image
                if (mergeImgMat(img_warp, img_merge) < 0)
                    return -1;
            }
        }
    }

    // Load the new texture and return status
    if (out_projCtx.loadMatTexture(img_merge) < 0)
    {
        ROS_ERROR("[updateTexture] Failed to load texture");
        return -1;
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

void appInitVariables()
{
    // Load images using OpenCV
    if (loadImgMat(fiImgPathWallVec, wallImgMatVec) < 0)
        throw std::runtime_error("[appInitVariables] Failed to load OpentCV wall images");
    if (loadImgMat(fiImgPathFloorVec, floorImgMatVec) < 0)
        throw std::runtime_error("[appInitVariables] Failed to load OpentCV wall images");

    // Load homography matrix XML file data
    for (const int &mon_ind : I.proj_mon_vec) // for each monitor index
    {
        for (int cal_i = 0; cal_i < N_CAL_MODES; ++cal_i)
        {
            CalibrationMode _CAL_MODE = static_cast<CalibrationMode>(cal_i);

            // Specify number of rows/cols to loop through based on active calibration mode
            int grid_size = (_CAL_MODE == WALLS_LEFT || _CAL_MODE == WALLS_MIDDLE || _CAL_MODE == WALLS_RIGHT) ? MAZE_SIZE : 1;

            // Iterate through the maze grid rows
            for (int gr_i = 0; gr_i < grid_size; ++gr_i)
            {
                for (int gc_i = 0; gc_i < grid_size; ++gc_i)
                {
                    // Load the homography matrix from XML
                    if (xmlLoadHMAT(mon_ind, _CAL_MODE, gr_i, gc_i, HMAT_ARR[mon_ind][_CAL_MODE][gr_i][gc_i]) < 0)
                        throw std::runtime_error("[appInitVariables] Error returned from xmlLoadHMAT");
                }
            }
        }
    }

    // Load maze boundary vertices in NDC units for each projector
    for (const int &mon_ind : I.proj_mon_vec) // for each monitor index
    {
        std::vector<cv::Point2f> maze_vert_ndc_vec(4);
        std::vector<cv::Point2f> maze_vert_cm_vec(4);

        // Load the maze vertices from XML
        if (xmlLoadVertices(mon_ind, maze_vert_ndc_vec) < 0)
            throw std::runtime_error("[appInitVariables] Error returned from xmlLoadVertices");

        // Compute the rotated maze vertices in centimeter units
        populateMazeVertNdcVec(mon_ind, maze_vert_cm_vec);

        // const std::vector<cv::Point2f> maze_vert_cm_vec = {
        //     cv::Point2f(0, MAZE_WIDTH_HEIGHT_CM),
        //     cv::Point2f(MAZE_WIDTH_HEIGHT_CM, MAZE_WIDTH_HEIGHT_CM),
        //     cv::Point2f(MAZE_WIDTH_HEIGHT_CM, 0.0),
        //     cv::Point2f(0.0, 0.0)};

        // Compute the homography matrix for warping the rat mask marker from maze cm to ndc space for each projector
        cv::Mat H;
        if (computeHomographyMatrix(maze_vert_cm_vec, maze_vert_ndc_vec, H))
            throw std::runtime_error("[appInitVariables] Monitor[" + std::to_string(mon_ind) + "]: Invalid homography matrix for rat mask image");

        // Store the homography matrix
        HMAT_CM_TO_NDC_ARR[mon_ind] = H;
    }

    // Intialize the window offset vector
    winOffsetVec.clear();              // Clear any existing elements
    winOffsetVec.reserve(N.projector); // Reserve memory for efficiency
    for (int mon_ind = 0; mon_ind < N.projector; ++mon_ind)
    {
        int offset = static_cast<int>(winOffsetDefualt * (mon_ind + 0.05f));
        winOffsetVec.emplace_back(offset, offset);
    }

    ROS_INFO("[appInitVariables] Finished loading and initializing variables successfully");
}

void appInitOpenGL()
{

    // Initialize GLFW and OpenGL settings
    if (MazeRenderContext::SetupGraphicsLibraries(N.monitor) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to initialize graphics");

    // Check if expected monitors exceed available monitors
    if (I.proj_mon_vec.back() >= N.monitor) // compare last entry
        throw std::runtime_error("[appInitOpenGL] Monitor index exceeds available monitors");

    // Initialize OpenGL for each projector
    int cnt = 0;
    for (auto &projCtx : PROJ_CTX_VEC)
    {
        int mon_ind = I.proj_mon_vec[cnt++];

        // Initialze render context for each projector
        if (projCtx.initWindowContext(mon_ind, I.starting_monitor, WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL, callbackKeyBinding) < 0)
            throw std::runtime_error("[appInitOpenGL] Failed to initialize render context");

        // Initialize OpenGL wall image objects
        if (projCtx.initRenderObjects(QUAD_GL_VERTICES, sizeof(QUAD_GL_VERTICES), QUAD_GL_INDICES, sizeof(QUAD_GL_INDICES)) < 0)
            throw std::runtime_error("[appInitOpenGL] Failed to initialize opengl wall image objects");

        // Set all projectors to the starting monitor and include xy offset
        if (projCtx.changeWindowDisplayMode(I.starting_monitor, F.fullscreen_mode, winOffsetVec[projCtx.windowInd]) < 0)
            throw std::runtime_error("[appInitOpenGL] Window[" + std::to_string(projCtx.windowInd) + "]: Failed Initial update of window monitor mode");

        // Create the shader program for wall image rendering
        if (projCtx.compileAndLinkShaders(QUAD_GL_VERTEX_SOURCE, QUAD_GL_FRAGMENT_SOURCE) < 0)
            throw std::runtime_error("[appInitOpenGL] Window[" + std::to_string(projCtx.windowInd) + "]: Failed to compile and link wall shader");

        // Create the shader program for CircleRenderer class rat mask rendering
        if (CircleRenderer::CompileAndLinkCircleShaders(1.0) < 0)
            throw std::runtime_error("[appInitOpenGL] Failed to compile and link circlerenderer class shader");

        // Initialize the CircleRenderer class object for rat masking
        if (RM_CIRCREND_ARR[mon_ind].initializeCircleObject(
                RT.marker_position,         // position
                RT.marker_radius,           // radius
                RT.marker_rgb,              // color
                RT.marker_segments,         // segments
                HMAT_CM_TO_NDC_ARR[mon_ind] // homography matrix
                ) < 0)
            throw std::runtime_error("[appInitOpenGL] Failed to initialize CircleRenderer class object");

        ROS_INFO("[appInitOpenGL] OpenGL initialized: Window[%d] Monitor[%d]", projCtx.windowInd, projCtx.monitorInd);
    }

    ROS_INFO("[appInitOpenGL] OpenGL contexts and objects Initialized succesfully");
}

void appMainLoop()
{
    int status = 0;
    while (status == 0)
    {
        // --------------- Check Kayboard Callback Flags ---------------

        // Update the window monitor and mode
        if (F.change_window_mode)
        {
            for (auto &projCtx : PROJ_CTX_VEC)
            {
                int mon_ind = F.windows_set_to_proj ? I.proj_mon_vec[projCtx.windowInd] : I.starting_monitor;
                if (projCtx.changeWindowDisplayMode(mon_ind, F.fullscreen_mode) < 0)
                    throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from MazeRenderContext::changeWindowDisplayMode");
            }
        }

        // Recompute wall parameters and update wall image texture
        if (F.update_textures)
        {
            for (auto &projCtx : PROJ_CTX_VEC)
            {
                // Initialize wall image texture
                if (updateTexture(I.proj_mon_vec[projCtx.windowInd], wallImgMatVec, floorImgMatVec, HMAT_ARR, projCtx))
                    throw std::runtime_error("[appInitOpenGL] Window[" + std::to_string(projCtx.windowInd) + "]: Failed to initialize wall texture");
            }
        }

        // Reset keybinding flags
        F.change_window_mode = false;
        F.update_textures = false;

        // --------------- Handle Image Processing for Next Frame ---------------

        for (auto &projCtx : PROJ_CTX_VEC)
        {
            // Prepare the frame for rendering (clear the back buffer)
            if (projCtx.initWindowForDrawing() < 0)
                throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from MazeRenderContext::initWindowForDrawing");

            // Make sure winsow always stays on top in fullscreen mode
            if (projCtx.forceWindowStackOrder(F.fullscreen_mode) < 0)
                throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from MazeRenderContext::forceWindowStackOrder");

            // Draw/update wall images
            if (projCtx.drawTexture() < 0)
                throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from drawTexture");

            // TEMP
            simulateRatMovement(0.5f, 45.0f, RT);

            // Draw/update rat mask marker
            if (drawRatMask(RT, RM_CIRCREND_ARR[projCtx.windowInd]) < 0)
                throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from drawRatMask");

            // Swap buffers and poll events
            if (projCtx.bufferSwapPoll() < 0)
                throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from MazeRenderContext::bufferSwapPoll");

            // Check if ROS shutdown
            if (!ros::ok())
                throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Unextpected ROS shutdown");

            // Check for exit
            status = projCtx.checkExitRequest();
            if (status > 0)
                break;
            else if (status < 0)
                throw std::runtime_error("[appMainLoop] Window[" + std::to_string(projCtx.windowInd) + "]: Error returned from MazeRenderContext::checkExitRequest");
        }
    }

    return;

    // Check which condition caused the loop to exit
    if (status == 1)
        ROS_INFO("[appMainLoop] Loop Terminated:  GLFW window should close");
    else if (status == 2)
        ROS_INFO("[appMainLoop] Loop Terminated:  Escape key was pressed");
    else
        ROS_INFO("[appMainLoop] Loop Terminated:  Reason unknown");
}

void appCleanup()
{
    ROS_INFO("SHUTTING DOWN");

    // Clean up OpenGL wall image objects for each window
    for (int proj_i = 0; proj_i < N.projector; ++proj_i)
    {
        if (PROJ_CTX_VEC[proj_i].cleanupContext(true) != 0)
            ROS_WARN("[appCleanup] Error during cleanup of MazeRenderContext: Window[%d] Monitor[%d]",
                     proj_i, PROJ_CTX_VEC[proj_i].monitorInd);
        else
            ROS_INFO("[appCleanup] MazeRenderContext instance cleaned up successfully: Window[%d] Monitor[%d]",
                     proj_i, PROJ_CTX_VEC[proj_i].monitorInd);
    }

    // Terminate graphics
    if (MazeRenderContext::CleanupGraphicsLibraries() < 0)
        ROS_WARN("[appCleanup] Failed to terminate GLFW library");
    else
        ROS_INFO("[appCleanup] GLFW library terminated successfully");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "projection_calibration", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    try
    {
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
