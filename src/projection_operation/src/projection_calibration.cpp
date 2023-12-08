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
            F.fullscreen_mode = !F.fullscreen_mode;
            F.change_window_mode = true;
        }

        // Move the window to another monitor
        int mon_ind = I.monitor;
        if (key == GLFW_KEY_0)
        {
            mon_ind = 0;
        }
        else if (key == GLFW_KEY_1 && N.monitors > 1)
        {
            mon_ind = 1;
        }
        else if (key == GLFW_KEY_2 && N.monitors > 2)
        {
            mon_ind = 2;
        }
        else if (key == GLFW_KEY_3 && N.monitors > 3)
        {
            mon_ind = 3;
        }
        else if (key == GLFW_KEY_4 && N.monitors > 4)
        {
            mon_ind = 4;
        }
        else if (key == GLFW_KEY_5 && N.monitors > 5)
        {
            mon_ind = 5;
        }
        // Check for monitor change
        if (mon_ind != I.monitor)
        {
            // Update the monitor index
            I.monitor = mon_ind;
            // Set the window update flag
            F.change_window_mode = true;
            // Set the reinstalize control points flag
            F.init_control_points = true;
        }

        // ---------- XML Handling [ENTER, L] ----------

        // Save coordinates to XML
        if (key == GLFW_KEY_S)
        {
            F.xml_save_hmat = true;
            F.update_textures = true;
        }

        // Load coordinates from XML
        if (key == GLFW_KEY_L)
        {
            F.xml_load_hmat = true;
            F.update_textures = true;
        }

        // ---------- Image selector keys [F1-F4] ----------

        // Update the image index
        int img_ind = (I.cal_mode < 3) ? I.wall_image : I.floor_image;
        if (key == GLFW_KEY_F1)
            img_ind = 0;
        else if (key == GLFW_KEY_F2)
            img_ind = 1;
        else if (key == GLFW_KEY_F3)
            img_ind = 2;
        else if (key == GLFW_KEY_F4)
            img_ind = 3;

        // Check for image change
        if ((img_ind != (I.cal_mode < 3) ? I.wall_image : I.floor_image) &&
            (img_ind < (I.cal_mode < 3) ? N.wall_images : N.floor_images))
        {
            // Update the image index and set flag
            if (I.cal_mode < 3)
                I.wall_image = img_ind;
            else if (I.cal_mode == 3)
                I.floor_image = img_ind;

            // Set the update texture flag
            F.update_textures = true;
        }

        // ---------- Control Point Reset [R] ----------

        if (key == GLFW_KEY_R)
        {
            F.init_control_points = true;
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
                I.cal_mode = (I.cal_mode > 0) ? I.cal_mode - 1 : 0;
                F.init_control_points = true;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                I.cal_mode = (I.cal_mode < N_CAL_MODES - 1) ? I.cal_mode + 1 : N_CAL_MODES - 1;
                F.init_control_points = true;
            }
        }

        // ---------- Contol point maze vertex selector keys [CTRL [LEFT, RIGHT, UP, DOWN]] ----------

        else if (mods & GLFW_MOD_CONTROL)
        {
            bool is_vert_changed = false;

            if (key == GLFW_KEY_UP)
            {
                // Set row index to top row
                I.cp_maze_vert_selected[0] = 0;
                is_vert_changed = true;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                // Set row index to bottom row
                I.cp_maze_vert_selected[0] = 1;
                is_vert_changed = true;
            }
            else if (key == GLFW_KEY_LEFT)
            {
                // Set column index to first column
                I.cp_maze_vert_selected[1] = 0;
                is_vert_changed = true;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                // Set column index to second column
                I.cp_maze_vert_selected[1] = 1;
                is_vert_changed = true;
            }

            if (is_vert_changed)
            {
                // Set flag to update the wall texture if the maze vertex was changed
                /// @note this is ensures the mode images are udated to the new
                F.update_homographys = true;

                // Set the wall vertex to the ortin if the maze vertex is changed
                for (int i = 0; i < 2; ++i)
                {
                    for (int j = 0; j < 2; ++j)
                    {
                        if (I.CP_MAP[i][j] == I.cp_wall_origin_vertex)
                        {
                            // Set the wall vertex to the origin
                            I.cp_wall_vert_selected[0] = i;
                            I.cp_wall_vert_selected[1] = j;
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
                I.cp_wall_vert_selected[0] = 0;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                // Set row index to bottom row
                I.cp_wall_vert_selected[0] = 1;
            }
            else if (key == GLFW_KEY_LEFT)
            {
                // Set column index to first column
                I.cp_wall_vert_selected[1] = 0;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                // Set column index to second column
                I.cp_wall_vert_selected[1] = 1;
            }
        }

        // ---------- Control point translate [SHIFT or no modifier] ----------
        else
        {
            // Set the position increment based on whether the shift key is pressed
            float pos_inc = (mods & GLFW_MOD_SHIFT) ? 0.01f : 0.0005f;

            // Get the maze and wall vertex indices cooresponding to the selected control point
            int mv_ind = I.CP_MAP[I.cp_maze_vert_selected[0]][I.cp_maze_vert_selected[1]];
            int wv_ind = I.CP_MAP[I.cp_wall_vert_selected[0]][I.cp_wall_vert_selected[1]];

            // Store current origin
            cv::Point2f cp_origin_save = CP_GRID_ARR[mv_ind][I.cp_wall_origin_vertex];

            // Listen for arrow key input to move selected control point
            if (key == GLFW_KEY_LEFT)
            {
                CP_GRID_ARR[mv_ind][wv_ind].x -= pos_inc; // Move left
                F.update_homographys = true;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                CP_GRID_ARR[mv_ind][wv_ind].x += pos_inc; // Move right
                F.update_homographys = true;
            }
            else if (key == GLFW_KEY_UP)
            {
                CP_GRID_ARR[mv_ind][wv_ind].y -= pos_inc; // Move up
                F.update_homographys = true;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                CP_GRID_ARR[mv_ind][wv_ind].y += pos_inc; // Move down
                F.update_homographys = true;
            }

            // Shift all control points if origin moved
            cv::Point2f cp_origin_new = CP_GRID_ARR[mv_ind][I.cp_wall_origin_vertex];

            // Calculate the change in x and y for the origin
            float delta_x = cp_origin_new.x - cp_origin_save.x;
            float delta_y = cp_origin_new.y - cp_origin_save.y;

            // Check if the origin vertex of the wall was moved (e.g., bottom-left)
            if (wv_ind == I.cp_wall_origin_vertex && (abs(delta_x) > 0.0f || abs(delta_y) > 0.0f))
            {
                // Update all other vertices based on the change in the origin
                for (int i = 0; i < 4; ++i) // Assuming there are 4 vertices
                {
                    if (i != I.cp_wall_origin_vertex) // Skip the origin vertex itself
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
    float cp_x = FLOOR_WIDTH_NDC / 2;  // starting X-coordinate in NDC coordinates
    float cp_y = FLOOR_HEIGHT_NDC / 2; // starting Y-coordinate in NDC coordinates

    // Add an x offset based on the calibration mode by shifting the origin to the left or right
    float offset_x = 1.0f * static_cast<float>(WALL_IMAGE_WIDTH_NDC);
    offset_x *= (cal_ind == 0) ? -1 : (cal_ind == 2) ? 1
                                                     : 0;

    // Specify number of control point groups to loop through
    // 4 groups for wall calibration and 1 group for floor calibration
    int n_cp_groups = (cal_ind < 3) ? 4 : 1;

    // Sprcify with width and height spacing
    // Set the spacing to be the wall image size for wall calibration and the maze size for floor calibration
    float cp_spacing_width = (cal_ind < 3) ? WALL_IMAGE_WIDTH_NDC : FLOOR_WIDTH_NDC;
    float cp_spacing_height = (cal_ind < 3) ? WALL_IMAGE_HEIGHT_NDC : FLOOR_HEIGHT_NDC;

    // Iterate through control point outer array (maze vertices)
    for (int cp_i = 0; cp_i < 4; cp_i++) // image bottom to top
    {
        cv::Point2f p_org;

        // 0: image top-left
        if (cp_i == 0)
            p_org = cv::Point2f(-cp_x + offset_x, -cp_y);

        // 1: image top-right
        else if (cp_i == 1)
            p_org = cv::Point2f(+cp_x + offset_x, -cp_y);

        // 2: image bottom-right
        else if (cp_i == 2)
            p_org = cv::Point2f(+cp_x + offset_x, +cp_y);

        // 3: image bottom-left
        else if (cp_i == 3)
            p_org = cv::Point2f(-cp_x + offset_x, +cp_y);

        // Set x y values for each wall vertex
        out_CP_GRID_ARR[cp_i] = {
            cv::Point2f(p_org.x, p_org.y),                                        // top left
            cv::Point2f(p_org.x + cp_spacing_width, p_org.y),                     // top right
            cv::Point2f(p_org.x + cp_spacing_width, p_org.y + cp_spacing_height), // bottom right
            cv::Point2f(p_org.x, p_org.y + cp_spacing_height),                    // bottom left
        };
    }
}

int initCircleRendererObjects(const std::array<std::array<cv::Point2f, 4>, 4> &_CP_GRID_ARR,
                              std::array<std::array<CircleRenderer, 4>, 4> &out_CP_RENDERERS)
{
    // Iterate through control point outer array (maze vertices)
    for (int cp_i = 0; cp_i < 4; cp_i++) // image bottom to top
    {
        // Iterate through control point inner array (wall vertices)
        for (int cp_j = 0; cp_j < 4; ++cp_j)
        {
            out_CP_RENDERERS[cp_i][cp_j].initializeCircleAttributes(
                _CP_GRID_ARR[cp_i][cp_j], // position
                cpDefualtMakerRadius,     // radius
                cpDefaultRGB,             // color
                cpRenderSegments          // segments
            );
        }
    }

    // Return GL status
    return MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__, "initCircleRendererObjects");
}

int renderControlPoints(int cal_ind,
                        const std::array<std::array<cv::Point2f, 4>, 4> &_CP_GRID_ARR,
                        std::array<std::array<CircleRenderer, 4>, 4> &out_CP_RENDERERS)
{
    // Setup the CircleRenderer class shaders
    CircleRenderer::SetupShader();

    // Specify number of control point groups to loop through
    // 4 groups for wall calibration and 1 group for floor calibration
    int n_cp_groups = (cal_ind < 3) ? 4 : 1;

    // Loop through the control points and draw them
    for (int cp_i = 0; cp_i < n_cp_groups; ++cp_i)
    {
        for (int cp_j = 0; cp_j < 4; ++cp_j)
        {
            // Get the maze and wall vertex indices cooresponding to the selected control point
            int mv_ind = I.CP_MAP[I.cp_maze_vert_selected[0]][I.cp_maze_vert_selected[1]];
            int wv_ind = I.CP_MAP[I.cp_wall_vert_selected[0]][I.cp_wall_vert_selected[1]];

            // Define the marker color
            cv::Scalar col = (cp_i == mv_ind && cp_j == wv_ind) ? cpWallVertSelectedRGB : (cp_i == mv_ind) ? cpMazeVertSelectedRGB
                                                                                                           : cpDefaultRGB;

            // Define the marker radius
            GLfloat rad = cp_j == 3 ? cpSelectedMakerRadius : cpDefualtMakerRadius;

            // Set the marker parameters
            out_CP_RENDERERS[cp_i][cp_j].setPosition(_CP_GRID_ARR[cp_i][cp_j]);
            out_CP_RENDERERS[cp_i][cp_j].setRadius(rad);
            out_CP_RENDERERS[cp_i][cp_j].setColor(col);

            // Recompute the marker parameters
            out_CP_RENDERERS[cp_i][cp_j].recomputeParameters();

            // Draw the marker
            out_CP_RENDERERS[cp_i][cp_j].draw();

            // Check for errors
            if (MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__) < 0)
            {
                ROS_ERROR("[renderControlPoints] Error Thrown for Control Point[%d][%d]", cp_i, cp_j);
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
    std::array<std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>, N_CAL_MODES> &out_HMAT_ARR)
{
    // Define source plane vertices
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

            // Compute and store the homography matrix
            if (computeHomographyMatrix(source_vertices_pxl, target_vertices_ndc, out_HMAT_ARR[cal_ind][gr_i][gc_i]) < 0)
                return -1;
        }
    }

    // Return success
    return 0;
}

int updateFloorHomography(const std::array<cv::Point2f, 4> &_CP_ARR, cv::Mat &out_H)
{
    // Define source plane vertices
    std::vector<cv::Point2f> source_vertices_pxl = {
        cv::Point2f(0.0f, 0.0f),                                    // Top-left
        cv::Point2f(FLOOR_IMAGE_WIDTH_PXL, 0.0f),                   // Top-right
        cv::Point2f(FLOOR_IMAGE_WIDTH_PXL, FLOOR_IMAGE_HEIGHT_PXL), // Bottom-right
        cv::Point2f(0.0f, FLOOR_IMAGE_HEIGHT_PXL)};                 // Bottom-left

    // Convert _CP_ARR to vector
    std::vector<cv::Point2f> target_vertices_ndc(_CP_ARR.begin(), _CP_ARR.end());

    // Compute and store the homography matrix
    if (computeHomographyMatrix(source_vertices_pxl, target_vertices_ndc, out_H) < 0)
        return -1;

    // Return success
    return 0;
}

int updateTextures(
    int cal_ind,
    cv::Mat img_wall_mat, cv::Mat img_mon_mat, cv::Mat img_cal_mat,
    const std::array<std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>, N_CAL_MODES> &_HMAT_ARR,
    GLuint &out_texture_id)
{
    // Initialize the image to be used as the texture
    cv::Mat im_wall_merge = cv::Mat::zeros(WINDOW_HEIGHT_PXL, WINDOW_WIDTH_PXL, CV_8UC4);

    // Specify number of rows/cols to loop through
    int grid_size = (cal_ind < 3) ? MAZE_SIZE : 1;

    // Iterate through the maze grid rows
    for (int gr_i = 0; gr_i < grid_size; gr_i++) // image bottom to top
    {
        // Iterate through each column in the maze row
        for (int gc_i = 0; gc_i < grid_size; gc_i++) // image left to right
        {
            // Copy wall image
            cv::Mat img_copy;
            img_wall_mat.copyTo(img_copy);

            // Get the maze vertex indice cooresponding to the selected control point
            int mv_ind = I.CP_MAP[I.cp_maze_vert_selected[0]][I.cp_maze_vert_selected[1]];

            //  Create merged image for the wall corresponding to the selected control point
            if (
                (mv_ind == 0 && gr_i == 0 && gc_i == 0) ||
                (mv_ind == 1 && gr_i == 0 && gc_i == grid_size - 1) ||
                (mv_ind == 3 && gr_i == grid_size - 1 && gc_i == 0) ||
                (mv_ind == 2 && gr_i == grid_size - 1 && gc_i == grid_size - 1))
            {
                // Merge test pattern and active monitor image
                if (mergeImgMat(img_mon_mat, img_copy) < 0)
                    return -1;

                // Merge previous image and active calibration image
                if (mergeImgMat(img_cal_mat, img_copy) < 0)
                    return -1;
            }

            // Get homography matrix for this wall
            cv::Mat H = _HMAT_ARR[cal_ind][gr_i][gc_i];

            // Warp Perspective
            cv::Mat im_warp;
            cv::warpPerspective(img_copy, im_warp, H, cv::Size(WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL));

            // Merge the warped image with the final image
            if (mergeImgMat(im_warp, im_wall_merge) < 0)
                return -1;
        }
    }

    // Make the new texture and return status
    return loadTexture(im_wall_merge, out_texture_id);
}

void appInitVariables()
{
    // Log setup parameters
    ROS_INFO("[appInitVariables] Config XML Path: %s", CONFIG_DIR_PATH.c_str());
    ROS_INFO("[appInitVariables] Display: Width[%d] Height[%d] AR[%0.2f]", WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL, WINDOW_ASPECT_RATIO);
    ROS_INFO("[appInitVariables] Floor (Pxl): Width[%d] Height[%d]", FLOOR_IMAGE_WIDTH_PXL, FLOOR_IMAGE_HEIGHT_PXL);
    ROS_INFO("[appInitVariables] Floor (NDC): Width[%0.2f] Height[%0.2f] Space Horz[%0.2f] Space Vert[%0.2f]", FLOOR_WIDTH_NDC, FLOOR_HEIGHT_NDC);
    ROS_INFO("[appInitVariables] Wall (Pxl): Width[%d] Height[%d]", WALL_IMAGE_WIDTH_PXL, WALL_IMAGE_HEIGHT_PXL);
    ROS_INFO("[appInitVariables] Wall (NDC): Width[%0.2f] Height[%0.2f] Space Horz[%0.2f] Space Vert[%0.2f]", WALL_IMAGE_WIDTH_NDC, WALL_IMAGE_HEIGHT_NDC);
    ROS_INFO("[appInitVariables] Origin Plane (NDC): Width[%0.2f] Height[%0.2f]", WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL);

    // Initialzie the hology arrays for all calibration modes and save to XML

    // Initialize control point coordinate dataset
    initControlPoints(I.cal_mode, CP_GRID_ARR);

    // Update wall homographys
    if (I.cal_mode < 3)
    {
        if (updateWallHomographys(I.cal_mode, CP_GRID_ARR, HMAT_ARR) < 0)
            throw std::runtime_error("[appInitVariables] Failed to initialize wall parameters");
    }
    // Update floor homography
    else if (I.cal_mode == 3)
    {
        if (updateFloorHomography(CP_GRID_ARR[0], HMAT_ARR[I.cal_mode][0][0]) < 0)
            throw std::runtime_error("[appInitVariables] Failed to initialize floor parameters");
    }

    ROS_INFO("[appInitVariables] Data Structures CP_GRID_ARR and HMAT_ARR initialized succesfully");
}

void appLoadAssets()
{
    // Load images using OpenCV
    if (loadImgMat(fiImgPathWallVec, testWallImgMatVec) < 0)
        throw std::runtime_error("[appLoadAssets] Failed to load OpentCV wall test images");
    if (loadImgMat(fiImgPathFloorVec, testFloorImgMatVec) < 0)
        throw std::runtime_error("[appLoadAssets] Failed to load OpentCV floor test images");
    if (loadImgMat(fiImgPathCalVec, calImgMatVec) < 0)
        throw std::runtime_error("[appLoadAssets] Failed to load OpentCV calibration mode images");
    if (loadImgMat(fiImgPathMonWallVec, monWallImgMatVec) < 0)
        throw std::runtime_error("[appLoadAssets] Failed to load OpentCV monitor number wall images");
    if (loadImgMat(fiImgPathMonFloorVec, monFloorImgMatVec) < 0)
        throw std::runtime_error("[appLoadAssets] Failed to load OpentCV onitor number floor images");

    ROS_INFO("[appLoadAssets] OpentCV mat images loaded succesfully");
}

void appInitOpenGL()
{
    // Initialize GLFW and OpenGL settings
    if (MazeRenderContext::SetupGraphicsLibraries(N.monitors) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to initialize graphics");

    // Initialze render context
    if (projCtx.initWindowContext(0, 0, WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL, callbackKeyBinding) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to initialize render context");

    // Initialize OpenGL wall image objects
    if (initWallRenderObjects(projCtx,
                              WALL_GL_VERTICES, sizeof(WALL_GL_VERTICES),
                              WALL_GL_INDICES, sizeof(WALL_GL_INDICES)) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to initialize opengl wall image objects");

    // Update monitor and window mode settings
    if (projCtx.changeWindowDisplayMode(I.monitor, F.fullscreen_mode) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed Initial update of window monitor mode");

    // Create the shader program for wall image rendering
    if (projCtx.compileAndLinkShaders(WALL_VERTEX_SOURCE, WALL_FRAGMENT_SOURCE) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to compile and link wall shader");

    // Create the shader program for CircleRenderer class control point rendering
    if (CircleRenderer::CompileAndLinkCircleShaders(WINDOW_ASPECT_RATIO) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to compile and link circlerenderer class shader");

    // Initialize the CircleRenderer class objects array
    if (initCircleRendererObjects(CP_GRID_ARR, CP_GLOBJ_ARR) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to initialize control point variables");

    // Initialize wall textures
    if (I.cal_mode < 3)
    {
        if (updateTextures(I.cal_mode, testWallImgMatVec[I.wall_image], monWallImgMatVec[I.monitor], calImgMatVec[I.cal_mode], HMAT_ARR, projCtx.textureID) < 0)
            throw std::runtime_error("[appMainLoop] Failed to initialize wall texture");
    }
    // Initialize floor texture
    else if (I.cal_mode == 3)
    {
        if (updateTextures(I.cal_mode, testFloorImgMatVec[I.wall_image], monFloorImgMatVec[I.monitor], calImgMatVec[I.cal_mode], HMAT_ARR, projCtx.textureID) < 0)
            throw std::runtime_error("[appMainLoop] Failed to initialize floor texture");
    }

    ROS_INFO("[appInitOpenGL] OpenGL context and objects initialized succesfully");
}

void appInitFileXML()
{
    auto fileExists = [](const std::string &_file_path) -> bool
    {
        std::ifstream file(_file_path);
        return file.good();
    };
    std::vector<int> mon_missing_vec;
    std::string file_path;
    std::string cal_mode;
    int n_mon_max = 6; // want to have files for up to 6 monitors

    // Check for non-initialized XML files and initialize them
    for (int mon_i = 0; mon_i < n_mon_max; ++mon_i)
    {
        bool isMonMissing = false;

        for (int cal_ind = 0; cal_ind < N_CAL_MODES && !isMonMissing; ++cal_ind)
        {
            // 4 groups for wall calibration and 1 group for floor calibration
            int grid_size = (cal_ind < 3) ? MAZE_SIZE : 1;
            for (int gr_i = 0; gr_i < grid_size && !isMonMissing; ++gr_i)
            {
                for (int gc_i = 0; gc_i < grid_size && !isMonMissing; ++gc_i)
                {
                    // Check if the file exists
                    xmlFrmtFileStrings(mon_i, cal_ind, file_path);
                    if (!fileExists(file_path))
                    {
                        // Check if mon_i is already in mon_missing_vec
                        if (std::find(mon_missing_vec.begin(), mon_missing_vec.end(), mon_i) == mon_missing_vec.end())
                        {
                            mon_missing_vec.push_back(mon_i);
                            isMonMissing = true; // Break out of the nested loops
                            ROS_WARN("[appInitFileXML] Initilizing XML file for Monitor[%d]: %s", mon_i, file_path.c_str());
                        }
                    }
                }
            }
        }
    }

    // Hack: make temp variables for control point grid and wall homography matrices
    std::array<std::array<cv::Point2f, 4>, 4> _CP_GRID_ARR;
    std::array<std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>, N_CAL_MODES> _HMAT_ARR;

    // Loop through the missing projectors
    for (auto &mon_i : mon_missing_vec)
    {
        for (int cal_ind = 0; cal_ind < N_CAL_MODES; ++cal_ind)
        {
            // Initalize temp control points and homography matrices
            initControlPoints(cal_ind, _CP_GRID_ARR);
            if (cal_ind < 3)
            {
                if (updateWallHomographys(cal_ind, _CP_GRID_ARR, _HMAT_ARR) < 0)
                    throw std::runtime_error("[appInitFileXML] Failed to initialize wall parameters");
            }
            else if (cal_ind == 3)
            {
                if (updateFloorHomography(_CP_GRID_ARR[0], _HMAT_ARR[cal_ind][0][0]) < 0)
                    throw std::runtime_error("[appInitFileXML] Failed to initialize floor parameters");
            }

            // Save each homography matrix to XML
            int grid_size = (cal_ind < 3) ? MAZE_SIZE : 1; // 4 groups for wall calibration and 1 group for floor calibration
            for (int gr_i = 0; gr_i < grid_size; ++gr_i)
            {
                for (int gc_i = 0; gc_i < grid_size; ++gc_i)
                {
                    if (xmlSaveHMAT(_HMAT_ARR[cal_ind][gr_i][gc_i], mon_i, cal_ind, gr_i, gc_i) < 0)
                        throw std::runtime_error("[appInitFileXML] Error returned from xmlSaveHMAT");
                }
            }
        }
    }
}

void appMainLoop()
{
    int status = 0;
    while (status == 0)
    {
        // --------------- Check Kayboard Callback Flags ---------------

        // Load/save XML file
        if (F.xml_load_hmat || F.xml_save_hmat)
        {
            int grid_size = (I.cal_mode < 3) ? MAZE_SIZE : 1;
            for (int gr_i = 0; gr_i < grid_size; ++gr_i)
            {
                for (int gc_i = 0; gc_i < grid_size; ++gc_i)
                {
                    // Save XML file
                    if (F.xml_save_hmat)
                    {
                        if (xmlSaveHMAT(HMAT_ARR[I.cal_mode][gr_i][gc_i], I.monitor, I.cal_mode, gr_i, gc_i) < 0)
                            throw std::runtime_error("[appMainLoop] Error returned from xmlSaveHMAT");
                    }
                    // Load XML file
                    if (F.xml_load_hmat)
                    {
                        if (xmlLoadHMAT(I.monitor, I.cal_mode, gr_i, gc_i, HMAT_ARR[I.cal_mode][gr_i][gc_i]) < 0)
                            throw std::runtime_error("[appMainLoop] Error returned from xmlLoadHMAT");
                    }
                }
            }

            // Flash the background to indicate the file was loaded/saved
            projCtx.flashBackgroundColor(cv::Scalar(0.0f, 0.25f, 0.0f), 10);
        }

        // Update the window monitor and mode
        if (F.change_window_mode)
        {
            if (projCtx.changeWindowDisplayMode(I.monitor, F.fullscreen_mode) < 0)
                throw std::runtime_error("[appMainLoop] Error returned from changeWindowDisplayMode");
        }

        // Initialize/reinitialize control point coordinate dataset
        if (F.init_control_points)
        {
            initControlPoints(I.cal_mode, CP_GRID_ARR);
        }

        // Update homography matrices array
        if (F.update_homographys ||
            F.init_control_points)
        {
            // Update wall homographys
            if (I.cal_mode < 3)
            {
                if (updateWallHomographys(I.cal_mode, CP_GRID_ARR, HMAT_ARR) < 0)
                    throw std::runtime_error("[appMainLoop] Error returned from updateWallHomographys");
            }
            // Update floor homography
            else if (I.cal_mode == 3)
            {
                if (updateFloorHomography(CP_GRID_ARR[0], HMAT_ARR[I.cal_mode][0][0]) < 0)
                    throw std::runtime_error("[appMainLoop] Error returned from updateFloorHomography");
            }
        }

        // Update image texture
        if (F.update_textures ||
            F.update_homographys ||
            F.init_control_points)
        {
            // Update wall textures
            if (I.cal_mode < 3)
            {
                if (updateTextures(I.cal_mode, testWallImgMatVec[I.wall_image], monWallImgMatVec[I.monitor], calImgMatVec[I.cal_mode], HMAT_ARR, projCtx.textureID) < 0)
                    throw std::runtime_error("[appMainLoop] Error returned from updateTextures for wall images");
            }
            // Update floor texture
            else if (I.cal_mode == 3)
            {
                if (updateTextures(I.cal_mode, testFloorImgMatVec[I.wall_image], monFloorImgMatVec[I.monitor], calImgMatVec[I.cal_mode], HMAT_ARR, projCtx.textureID) < 0)
                    throw std::runtime_error("[appMainLoop] Error returned from updateTextures for floor images");
            }
        }

        // Reset keybinding flags
        F.xml_load_hmat = false;
        F.xml_save_hmat = false;
        F.change_window_mode = false;
        F.init_control_points = false;
        F.update_textures = false;
        F.update_homographys = false;

        // --------------- Handle Rendering for Next Frame ---------------

        // Prepare the frame for rendering (make context clear the back buffer)
        if (projCtx.initWindow() < 0)
            throw std::runtime_error("[appMainLoop] Error returned from MazeRenderContext::initWindow");

        // Make sure winsow always stays on top in fullscreen mode
        if (projCtx.forceWindowStackOrder(F.fullscreen_mode) < 0)
            throw std::runtime_error("[appMainLoop] Error returned from MazeRenderContext::forceWindowStackOrder");

        // Draw/update texture
        if (projCtx.drawTexture() < 0)
            throw std::runtime_error("[appMainLoop] Error returned from drawTexture");

        // Draw/update control point markers
        if (renderControlPoints(I.cal_mode, CP_GRID_ARR, CP_GLOBJ_ARR) < 0)
            throw std::runtime_error("[appMainLoop] Error returned from renderControlPoints");

        // Swap buffers and poll events
        if (projCtx.bufferSwapPoll() < 0)
            throw std::runtime_error("[appMainLoop] Error returned from MazeRenderContext::bufferSwapPoll");

        // Check if ROS shutdown
        if (!ros::ok())
            throw std::runtime_error("[appMainLoop] Unexpected ROS shutdown");

        // Check for exit
        status = projCtx.checkExitRequest();
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
    if (projCtx.cleanupContext() != 0)
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

    try
    {
        appInitVariables();
        appLoadAssets();
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
