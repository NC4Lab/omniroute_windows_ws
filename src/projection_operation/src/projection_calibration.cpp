// ############################################################################################################

// ======================================== projection_calibration.cpp ========================================

// ############################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_calibration.h"

// ================================================== FUNCTIONS ==================================================

void callbackKeyBinding(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    // Set the current OpenGL context to the window
    glfwMakeContextCurrent(window);

    // _______________ ANY KEY RELEASE ACTION _______________

    if (action == GLFW_RELEASE)
    {

        // ---------- Monitor handling [F] ----------

        // Set/unset Fullscreen
        if (key == GLFW_KEY_F)
        {
            F.fullscreenMode = !F.fullscreenMode;
            F.switchWindowMode = true;
        }

        // Move the window to another monitor
        else if (key == GLFW_KEY_0)
        {
            I.winMon = 0;
            F.switchWindowMode = true;
        }
        else if (key == GLFW_KEY_1 && N.monitors > 1)
        {
            I.winMon = 1;
            F.switchWindowMode = true;
        }
        else if (key == GLFW_KEY_2 && N.monitors > 2)
        {
            I.winMon = 2;
            F.switchWindowMode = true;
        }
        else if (key == GLFW_KEY_3 && N.monitors > 3)
        {
            I.winMon = 3;
            F.switchWindowMode = true;
        }
        else if (key == GLFW_KEY_4 && N.monitors > 4)
        {
            I.winMon = 4;
            F.switchWindowMode = true;
        }
        else if (key == GLFW_KEY_5 && N.monitors > 5)
        {
            I.winMon = 5;
            F.switchWindowMode = true;
        }

        // ---------- XML Handling [ENTER, L] ----------

        // Save coordinates to XML
        else if (key == GLFW_KEY_S)
        {
            F.saveXML = true;
        }

        // Load coordinates from XML
        else if (key == GLFW_KEY_L)
        {
            F.loadXML = true;
        }

        // ---------- Image selector keys [F1-F4] ----------

        else if (key == GLFW_KEY_F1)
        {
            I.wallImage = N.wallImages > 0 ? 0 : I.wallImage;
            F.updateWallTextures = true;
        }
        else if (key == GLFW_KEY_F2)
        {
            I.wallImage = N.wallImages > 1 ? 1 : I.wallImage;
            F.updateWallTextures = true;
        }
        else if (key == GLFW_KEY_F3)
        {
            I.wallImage = N.wallImages > 2 ? 2 : I.wallImage;
            F.updateWallTextures = true;
        }
        else if (key == GLFW_KEY_F4)
        {
            I.wallImage = N.wallImages > 3 ? 3 : I.wallImage;
            F.updateWallTextures = true;
        }

        // ---------- Control Point Reset [R] ----------

        else if (key == GLFW_KEY_R)
        {
            F.initControlPoints = true;
        }
    }

    // _______________ ANY KEY PRESS OR REPEAT ACTION _______________
    else if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {

        // ---------- Calibration mode [CTRL + SHIFT [LEFT, RIGHT]] ----------

        if ((mods & GLFW_MOD_CONTROL) && (mods & GLFW_MOD_SHIFT))
        {

            // Listen for arrow key input to switch through calibration modes
            if (key == GLFW_KEY_LEFT)
            {
                I.calMode = (I.calMode > 0) ? I.calMode - 1 : N_CAL_MODES - 1;
                F.initControlPoints = true;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                I.calMode = (I.calMode < N_CAL_MODES - 1) ? I.calMode + 1 : 0;
                F.initControlPoints = true;
            }
        }

        // ---------- Contol point maze vertex selector keys [CTRL [LEFT, RIGHT, UP, DOWN]] ----------

        else if (mods & GLFW_MOD_CONTROL)
        {
            bool is_cp_maze_vert_changed = false;

            if (key == GLFW_KEY_UP)
            {
                // Set row index to top row
                I.cpMazeVertSel[0] = 0;
                is_cp_maze_vert_changed = true;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                // Set row index to bottom row
                I.cpMazeVertSel[0] = 1;
                is_cp_maze_vert_changed = true;
            }
            else if (key == GLFW_KEY_LEFT)
            {
                // Set column index to first column
                I.cpMazeVertSel[1] = 0;
                is_cp_maze_vert_changed = true;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                // Set column index to second column
                I.cpMazeVertSel[1] = 1;
                is_cp_maze_vert_changed = true;
            }

            if (is_cp_maze_vert_changed)
            {
                // Set flag to update the wall texture if the maze vertex was changed
                /// @note this is ensures the mode images are udated to the new
                F.updateWallTextures = true;

                // Set the wall vertex to the ortin if the maze vertex is changed
                for (int i = 0; i < 2; ++i)
                {
                    for (int j = 0; j < 2; ++j)
                    {
                        if (I.cpMap[i][j] == I.cpWVOrigin)
                        {
                            // Set the wall vertex to the origin
                            I.cpWallVertSel[0] = i;
                            I.cpWallVertSel[1] = j;
                        }
                    }
                }
            }
        }

        // ---------- Control point wall vertex selector keys [ALT [LEFT, RIGHT, UP, DOWN]] ----------

        else if (mods & GLFW_MOD_ALT)
        {
            if (key == GLFW_KEY_UP)
            {
                // Set row index to top row
                I.cpWallVertSel[0] = 0;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                // Set row index to bottom row
                I.cpWallVertSel[0] = 1;
            }
            else if (key == GLFW_KEY_LEFT)
            {
                // Set column index to first column
                I.cpWallVertSel[1] = 0;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                // Set column index to second column
                I.cpWallVertSel[1] = 1;
            }
        }

        // ---------- Control point translate [SHIFT or no modifier] ----------
        else
        {
            // Set the position increment based on whether the shift key is pressed
            float pos_inc = (mods & GLFW_MOD_SHIFT) ? 0.01f : 0.0005f;

            // Get the maze and wall vertex indices cooresponding to the selected control point
            int mv_ind = I.cpMap[I.cpMazeVertSel[0]][I.cpMazeVertSel[1]];
            int wv_ind = I.cpMap[I.cpWallVertSel[0]][I.cpWallVertSel[1]];

            // Store current origin
            cv::Point2f cp_origin_save = CP_GRID_ARR[mv_ind][I.cpWVOrigin];

            // Listen for arrow key input to move selected control point
            if (key == GLFW_KEY_LEFT)
            {
                CP_GRID_ARR[mv_ind][wv_ind].x -= pos_inc; // Move left
                F.updateWallTextures = true;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                CP_GRID_ARR[mv_ind][wv_ind].x += pos_inc; // Move right
                F.updateWallTextures = true;
            }
            else if (key == GLFW_KEY_UP)
            {
                CP_GRID_ARR[mv_ind][wv_ind].y -= pos_inc; // Move up
                F.updateWallTextures = true;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                CP_GRID_ARR[mv_ind][wv_ind].y += pos_inc; // Move down
                F.updateWallTextures = true;
            }

            // Shift all control points if origin moved
            cv::Point2f cp_origin_new = CP_GRID_ARR[mv_ind][I.cpWVOrigin];

            // Calculate the change in x and y for the origin
            float delta_x = cp_origin_new.x - cp_origin_save.x;
            float delta_y = cp_origin_new.y - cp_origin_save.y;

            // Check if the origin vertex of the wall was moved (e.g., bottom-left)
            if (wv_ind == I.cpWVOrigin && (abs(delta_x) > 0.0f || abs(delta_y) > 0.0f))
            {
                // Update all other vertices based on the change in the origin
                for (int i = 0; i < 4; ++i) // Assuming there are 4 vertices
                {
                    if (i != I.cpWVOrigin) // Skip the origin vertex itself
                    {
                        CP_GRID_ARR[mv_ind][i].x += delta_x;
                        CP_GRID_ARR[mv_ind][i].y += delta_y;
                    }
                }
            }
        }
    }
}

void initControlPoints(int cal_ind, std::array<std::array<cv::Point2f, 4>, 4> &out_CP_GRID_ARR)
{
    // Specify the control point limits
    float cp_x = MAZE_WIDTH_NDC / 2;  // starting X-coordinate in NDC coordinates
    float cp_y = MAZE_HEIGHT_NDC / 2; // starting Y-coordinate in NDC coordinates                                        

    // Iterate through control point outer array (maze vertices)
    for (float mv_i = 0; mv_i < 4; mv_i++) // image bottom to top
    {
        cv::Point2f p_org;

        // 0: image top-left
        if (mv_i == 0)
            p_org = cv::Point2f(-cp_x, -cp_y);

        // 1: image top-right
        else if (mv_i == 1)
            p_org = cv::Point2f(+cp_x, -cp_y);

        // 2: image bottom-right
        else if (mv_i == 2)
            p_org = cv::Point2f(+cp_x, +cp_y);

        // 3: image bottom-left
        else if (mv_i == 3)
            p_org = cv::Point2f(-cp_x, +cp_y);

        // Set x y values for each wall vertex
        out_CP_GRID_ARR[mv_i] = {
            cv::Point2f(p_org.x, p_org.y),                                                // top left
            cv::Point2f(p_org.x + WALL_IMAGE_WIDTH_NDC, p_org.y),                         // top right
            cv::Point2f(p_org.x + WALL_IMAGE_WIDTH_NDC, p_org.y + WALL_IMAGE_HEIGHT_NDC), // bottom right
            cv::Point2f(p_org.x, p_org.y + WALL_IMAGE_HEIGHT_NDC),                        // bottom left
        };
    }
}

int initCircleRendererObjects(const std::array<std::array<cv::Point2f, 4>, 4> &_CP_GRID_ARR,
                              std::array<std::array<CircleRenderer, 4>, 4> &out_CP_RENDERERS)
{
    // Iterate through control point outer array (maze vertices)
    for (float mv_i = 0; mv_i < 4; mv_i++) // image bottom to top
    {
        // Iterate through control point inner array (wall vertices)
        for (int wv_i = 0; wv_i < 4; ++wv_i)
        {
            out_CP_RENDERERS[mv_i][wv_i].initializeCircleAttributes(
                _CP_GRID_ARR[mv_i][wv_i], // position
                cpDefualtMakerRadius,     // radius
                cpDefaultRGB,             // color
                cpRenderSegments          // segments
            );
        }
    }

    // Return GL status
    return MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__, "initCircleRendererObjects");
}

int renderControlPoints(const std::array<std::array<cv::Point2f, 4>, 4> &_CP_GRID_ARR,
                        std::array<std::array<CircleRenderer, 4>, 4> &out_CP_RENDERERS)
{
    // Setup the CircleRenderer class shaders
    CircleRenderer::SetupShader();

    // Loop through the control points and draw them
    for (int mv_i = 0; mv_i < 4; ++mv_i)
    {
        for (int wv_i = 0; wv_i < 4; ++wv_i)
        {
            // Get the maze and wall vertex indices cooresponding to the selected control point
            int mv_ind = I.cpMap[I.cpMazeVertSel[0]][I.cpMazeVertSel[1]];
            int wv_ind = I.cpMap[I.cpWallVertSel[0]][I.cpWallVertSel[1]];

            // Define the marker color
            cv::Scalar col = (mv_i == mv_ind && wv_i == wv_ind) ? cpWallVertSelectedRGB : (mv_i == mv_ind) ? cpMazeVertSelectedRGB
                                                                                                           : cpDefaultRGB;

            // Define the marker radius
            GLfloat rad = wv_i == 3 ? cpSelectedMakerRadius : cpDefualtMakerRadius;

            // Set the marker parameters
            out_CP_RENDERERS[mv_i][wv_i].setPosition(_CP_GRID_ARR[mv_i][wv_i]);
            out_CP_RENDERERS[mv_i][wv_i].setRadius(rad);
            out_CP_RENDERERS[mv_i][wv_i].setColor(col);

            // Recompute the marker parameters
            out_CP_RENDERERS[mv_i][wv_i].recomputeParameters();

            // Draw the marker
            out_CP_RENDERERS[mv_i][wv_i].draw();

            // Check for errors
            if (MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__) < 0)
            {
                ROS_ERROR("[renderControlPoints] Error Thrown for Control Point[%d][%d]", mv_i, wv_i);
                return -1;
            }
        }
    }

    // Unset the shader program
    CircleRenderer::UnsetShader();

    // Return GL status
    return MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__, "renderControlPoints");
}

int updateWallHomographys(
    int cal_ind,
    const std::array<std::array<cv::Point2f, 4>, 4> &_CP_GRID_ARR,
    std::array<std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>, N_CAL_MODES> &out_WALL_HMAT_ARR)
{
    // Define origin plane vertices
    std::vector<cv::Point2f> source_vertices_pxl = {
        cv::Point2f(0.0f, 0.0f),                                  // Top-left
        cv::Point2f(WALL_IMAGE_WIDTH_PXL, 0.0f),                  // Top-right
        cv::Point2f(WALL_IMAGE_WIDTH_PXL, WALL_IMAGE_HEIGHT_PXL), // Bottom-right
        cv::Point2f(0.0f, WALL_IMAGE_HEIGHT_PXL)};                // Bottom-left

    // Iterate trough grid/wall rows
    for (float gr_i = 0; gr_i < MAZE_SIZE; gr_i++) // image bottom to top
    {
        // Iterate trough grid/wall columns
        for (float gc_i = 0; gc_i < MAZE_SIZE; gc_i++) // image left to right
        {
            std::vector<cv::Point2f> target_vertices_ndc(4);
            for (int p_i = 0; p_i < 4; p_i++)
            {
                // Get the wall vertex values for each maze corner for the interpolation function
                ///@note that y values must be flipped to account for the image origin being in the top-left corner
                cv::Point2f p_a = _CP_GRID_ARR[0][p_i]; // bottom-left interp == top left NDC
                cv::Point2f p_b = _CP_GRID_ARR[1][p_i]; // bottom-right interp == top right NDC
                cv::Point2f p_c = _CP_GRID_ARR[3][p_i]; // top-left interp == bottom left NDC
                cv::Point2f p_d = _CP_GRID_ARR[2][p_i]; // top-right interp == bottom right NDC

                // Get the interpolated vertex x-coordinate
                cv::Point2f p_interp(
                    bilinearInterpolation(p_a.x, p_b.x, p_c.x, p_d.x, gr_i, gc_i, MAZE_SIZE),  // x
                    bilinearInterpolation(p_a.y, p_b.y, p_c.y, p_d.y, gr_i, gc_i, MAZE_SIZE)); // y

                // Store the warped vertex coordinates
                target_vertices_ndc[p_i] = p_interp;
            }

            // Convert to pixel coordinates for OpenCV's findHomography function
            std::vector<cv::Point2f> target_vertices_pxl = quadVertNdc2Pxl(target_vertices_ndc, WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL);

            // Check that the target plane vertices are valid
            int resp = checkQuadVertices(target_vertices_pxl);
            if (resp < 0)
            {
                ROS_WARN("[updateWallHomographys] Target Plane Vertices Invalid: Reason[%s]",
                         resp == -1 ? "Wrong Number of Vertices" : "Vertices are Collinear");
                return -1;
            }

            // Use OpenCV's findHomography function to compute the homography matrix
            cv::Mat H = cv::findHomography(source_vertices_pxl, target_vertices_pxl);

            // Check for valid homography matrix
            if (H.empty())
            {
                ROS_ERROR("[updateWallHomographys] Failed to Compute Homography Matrix");
                return -1;
            }

            // Store the homography matrix
            out_WALL_HMAT_ARR[cal_ind][gr_i][gc_i] = H;
        }
    }

    // TEMP
    // dbLogCtrlPointCoordinates(_CP_GRID_ARR);
    // TEMP
    // return -1;

    // Return success
    return 0;
}

int updateWallTextures(
    int cal_ind,
    cv::Mat img_wall_mat, cv::Mat img_mon_mat, cv::Mat img_cal_mat,
    const std::array<std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>, N_CAL_MODES> &_WALL_HMAT_ARR,
    GLuint &out_WALL_TEXTURE_ID)
{
    // Initialize the image to be used as the texture
    cv::Mat im_wall_merge = cv::Mat::zeros(WINDOW_HEIGHT_PXL, WINDOW_WIDTH_PXL, CV_8UC4);

    // Iterate through the maze grid rows
    for (float gr_i = 0; gr_i < MAZE_SIZE; gr_i++) // image bottom to top
    {
        // Iterate through each column in the maze row
        for (float gc_i = 0; gc_i < MAZE_SIZE; gc_i++) // image left to right
        {
            // Copy wall image
            cv::Mat img_copy;
            img_wall_mat.copyTo(img_copy);

            // Get the maze vertex indice cooresponding to the selected control point
            int mv_ind = I.cpMap[I.cpMazeVertSel[0]][I.cpMazeVertSel[1]];

            //  Create merged image for the wall corresponding to the selected control point
            if (
                (mv_ind == 0 && gr_i == 0 && gc_i == 0) ||
                (mv_ind == 1 && gr_i == 0 && gc_i == MAZE_SIZE - 1) ||
                (mv_ind == 3 && gr_i == MAZE_SIZE - 1 && gc_i == 0) ||
                (mv_ind == 2 && gr_i == MAZE_SIZE - 1 && gc_i == MAZE_SIZE - 1))
            {
                // Merge test pattern and active monitor image
                if (mergeImgMat(img_mon_mat, img_copy) < 0)
                    return -1;

                // Merge previous image and active calibration image
                if (mergeImgMat(img_cal_mat, img_copy) < 0)
                    return -1;
            }

            // Get homography matrix for this wall
            cv::Mat H = _WALL_HMAT_ARR[cal_ind][gr_i][gc_i];

            // Warp Perspective
            cv::Mat im_warp;
            cv::warpPerspective(img_copy, im_warp, H, cv::Size(WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL));

            // Merge the warped image with the final image
            if (mergeImgMat(im_warp, im_wall_merge) < 0)
                return -1;

            // // TEMP
            // cv::namedWindow("Warped Image Display", cv::WINDOW_AUTOSIZE);
            // cv::imshow("Warped Image Display", im_warp);
            // cv::waitKey(0);
            // cv::destroyWindow("Warped Image Display");
            // break;
        }
    }

    // Make the new texture and return status
    return loadTexture(im_wall_merge, out_WALL_TEXTURE_ID);
}

void appInitDatasets()
{
    // Log setup parameters
    ROS_INFO("[appInitDatasets] Config XML Path: %s", CONFIG_DIR_PATH.c_str());
    ROS_INFO("[appInitDatasets] Display: Width[%d] Height[%d] AR[%0.2f]", WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL, WINDOW_ASPECT_RATIO);
    ROS_INFO("[appInitDatasets] Wall (Pxl): Width[%d] Height[%d]", WALL_IMAGE_WIDTH_PXL, WALL_IMAGE_HEIGHT_PXL);
    ROS_INFO("[appInitDatasets] Wall (NDC): Width[%0.2f] Height[%0.2f] Space Horz[%0.2f] Space Vert[%0.2f]", WALL_IMAGE_WIDTH_NDC, WALL_IMAGE_HEIGHT_NDC);
    ROS_INFO("[appInitDatasets] Origin Plane (NDC): Width[%0.2f] Height[%0.2f]", WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL);

    // Initialize control point coordinate dataset
    initControlPoints(I.calMode, CP_GRID_ARR);

    // Initialize wall homography matrices array
    if (updateWallHomographys(I.calMode, CP_GRID_ARR, WALL_HMAT_ARR) < 0)
        throw std::runtime_error("[appInitDatasets] Failed to initialize wall parameters");

    ROS_INFO("SETUP: Data Structures Initialized");
}

void appLoadImages()
{
    // Load images using OpenCV
    if (loadImgMat(wallImgPathVec, wallImgMatVec) < 0)
        throw std::runtime_error("[appLoadImages] Failed to load wall images");
    if (loadImgMat(monImgPathVec, monImgMatVec) < 0)
        throw std::runtime_error("[appLoadImages] Failed to load monitor number images");
    if (loadImgMat(calImgPathVec, calImgMatVec) < 0)
        throw std::runtime_error("[appLoadImages] Failed to load calibration mode images");

    ROS_INFO("SETUP: Images Loaded");
}

void appInitOpenGL()
{
    // Initialize GLFW and OpenGL settings
    if (MazeRenderContext::SetupGraphicsLibraries(N.monitors) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to initialize graphics");

    // Initialze render context
    if (PROJ_GL[0].initContext(0, 0, callbackKeyBinding) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to initialize render context");

    // Initialize OpenGL wall image objects
    if (initWallRenderObjects(PROJ_GL[0],
                              WALL_GL_VERTICES, sizeof(WALL_GL_VERTICES),
                              WALL_GL_INDICES, sizeof(WALL_GL_INDICES)) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to initialize opengl wall image objects");

    // Update monitor and window mode settings
    if (PROJ_GL[0].switchWindowMode(I.winMon, F.fullscreenMode) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed Initial update of window monitor mode");

    // Create the shader program for wall image rendering
    if (PROJ_GL[0].compileAndLinkShaders(WALL_VERTEX_SOURCE, WALL_FRAGMENT_SOURCE) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to compile and link wall shader");

    // Create the shader program for CircleRenderer class control point rendering
    if (CircleRenderer::CompileAndLinkCircleShaders(WINDOW_ASPECT_RATIO) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to compile and link circlerenderer class shader");

    // Initialize the CircleRenderer class objects array
    if (initCircleRendererObjects(CP_GRID_ARR, CP_CIRCREN_ARR) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to initialize control point variables");

    // Initialize wall image texture
    if (updateWallTextures(I.calMode, wallImgMatVec[I.wallImage], monImgMatVec[I.winMon], calImgMatVec[I.calMode], WALL_HMAT_ARR, PROJ_GL[0].textureID) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to initialize wall texture");

    ROS_INFO("SETUP: OpenGL Initialized");
}

void appInitFileXML()
{
    for (int mon_ind = 0; mon_ind < N.monitors; ++mon_ind)
    {
        for (int cal_ind = 0; cal_ind < N_CAL_MODES; ++cal_ind)
        {
            for (int gr_i = 0; gr_i < MAZE_SIZE; ++gr_i)
            {
                for (int gc_i = 0; gc_i < MAZE_SIZE; ++gc_i)
                {
                    if (xmlSaveHMAT(WALL_HMAT_ARR[I.calMode][gr_i][gc_i], I.winMon, I.calMode, gr_i, gc_i) < 0)
                        throw std::runtime_error("[appInitFileXML] Error returned from xmlSaveHMAT");
                }
            }
        }
        // TEMP
        return;
    }
}

void appMainLoop()
{
    int status = 0;
    while (status == 0)
    {

        // --------------- Check Kayboard Callback Flags ---------------

        // Load/save XML file
        if (F.loadXML || F.saveXML)
        {
            for (int gr_i = 0; gr_i < MAZE_SIZE; ++gr_i)
            {
                for (int gc_i = 0; gc_i < MAZE_SIZE; ++gc_i)
                {
                    // Save XML file
                    if (F.saveXML)
                    {
                        if (xmlSaveHMAT(WALL_HMAT_ARR[I.calMode][gr_i][gc_i], I.winMon, I.calMode, gr_i, gc_i) < 0)
                            throw std::runtime_error("[appMainLoop] Error returned from xmlSaveHMAT");
                        F.saveXML = false;
                    }
                    // Load XML file
                    if (F.loadXML)
                    {
                        if (xmlLoadHMAT(I.winMon, I.calMode, gr_i, gc_i, WALL_HMAT_ARR[I.calMode][gr_i][gc_i]) < 0)
                            throw std::runtime_error("[appMainLoop] Error returned from xmlLoadHMAT");
                        F.loadXML = false;
                    }
                }
            }

            // Flash the background to indicate the file was loaded/saved
            PROJ_GL[0].flashBackgroundColor(cv::Scalar(0.0f, 0.25f, 0.0f), 500);
        }

        // Update the window monitor and mode
        if (F.switchWindowMode)
        {
            if (PROJ_GL[0].switchWindowMode(I.winMon, F.fullscreenMode) < 0)
                throw std::runtime_error("[appMainLoop] Error returned from switchWindowMode");

            F.switchWindowMode = false;
        }

        // Initialize/reinitialize control point coordinate dataset
        if (F.initControlPoints)
        {
            initControlPoints(I.calMode, CP_GRID_ARR);
            F.initControlPoints = false;

            // Need to update the homography mats and wall textures
            F.updateWallTextures = true;
        }

        // Recompute wall parameters and update wall image texture
        if (F.updateWallTextures)
        {
            // Update wall homography matrices array
            if (updateWallHomographys(I.calMode, CP_GRID_ARR, WALL_HMAT_ARR) < 0)
                throw std::runtime_error("[appMainLoop] Error returned from updateWallHomographys");

            // Update wall image texture
            if (updateWallTextures(I.calMode, wallImgMatVec[I.wallImage], monImgMatVec[I.winMon], calImgMatVec[I.calMode], WALL_HMAT_ARR, PROJ_GL[0].textureID) < 0)
                throw std::runtime_error("[appMainLoop] Error returned from updateWallTextures");

            F.updateWallTextures = false;
        }

        // --------------- Handle Image Processing for Next Frame ---------------

        // Prepare the frame for rendering (make context clear the back buffer)
        if (PROJ_GL[0].initWindow() < 0)
            throw std::runtime_error("[appMainLoop] Error returned from MazeRenderContext::initWindow");

        // Make sure winsow always stays on top in fullscreen mode
        if (PROJ_GL[0].setWindowStackOrder(F.fullscreenMode) < 0)
            throw std::runtime_error("[appMainLoop] Error returned from MazeRenderContext::setWindowStackOrder");

        // Draw/update wall images
        if (renderWallImage(PROJ_GL[0]) < 0)
            throw std::runtime_error("[appMainLoop] Error returned from renderWallImage");

        // Draw/update control point markers
        if (renderControlPoints(CP_GRID_ARR, CP_CIRCREN_ARR) < 0)
            throw std::runtime_error("[appMainLoop] Error returned from renderControlPoints");

        // Swap buffers and poll events
        if (PROJ_GL[0].bufferSwapPoll() < 0)
            throw std::runtime_error("[appMainLoop] Error returned from MazeRenderContext::bufferSwapPoll");

        // Check if ROS shutdown
        if (!ros::ok())
            throw std::runtime_error("[appMainLoop] Unextpected ROS shutdown");

        // Check for exit
        status = PROJ_GL[0].checkExitRequest();
        if (status < 0)
            throw std::runtime_error("[appMainLoop] Error returned from MazeRenderContext::checkExitRequest");
    }

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
    ROS_INFO("[appCleanup] Shutting Down Projection Calibration Node...");

    // Delete CircleRenderer class shader program
    if (CircleRenderer::CleanupClassResources() < 0)
        ROS_WARN("[appCleanup] Failed to delete CircleRenderer shader program");
    else
        ROS_INFO("[appCleanup] CircleRenderer shader program deleted successfully");

    // Clean up OpenGL wall image objects
    if (PROJ_GL[0].cleanupContext() != 0)
        ROS_WARN("[appCleanup] Error during cleanup of MazeRenderContext instance");
    else
        ROS_INFO("[appCleanup] MazeRenderContext instance cleaned up successfully");

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

    // // TEMP
    // std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> HMAT_GRID_ARR;
    // for (int gr_i = 0; gr_i < MAZE_SIZE; ++gr_i)
    // {
    //     for (int gc_i = 0; gc_i < MAZE_SIZE; ++gc_i)
    //     {
    //         HMAT_GRID_ARR[gr_i][gc_i] = cv::Mat::eye(3, 3, CV_64F);
    //     }
    // }
    // std::string out_tag;
    // std::string out_path;
    // xmlFrmtFileStrings(9, 0, out_path, out_tag);
    // saveHMATxml(out_path, HMAT_GRID_ARR);
    // return 0;

    try
    {
        appInitDatasets();
        appLoadImages();
        appInitOpenGL();
        appInitFileXML();
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
