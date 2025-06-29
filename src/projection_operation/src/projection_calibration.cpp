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

        // ----------Set/unset Fullscreen [F] ----------

        if (key == GLFW_KEY_F)
        {
            FLAG_FULLSCREEN_MODE = !FLAG_FULLSCREEN_MODE;
            FLAG_CHANGE_WINDOW_MODE = true;
        }

        // ----------Move the window to another monitor [0-5] ----------

        // Check number keys and update the monitor index
        int mon_ind = I.monitor;
        if (key == GLFW_KEY_0)
        {
            mon_ind = 0;
        }
        else if (key == GLFW_KEY_1 && N.monitor > 1)
        {
            mon_ind = 1;
        }
        else if (key == GLFW_KEY_2 && N.monitor > 2)
        {
            mon_ind = 2;
        }
        else if (key == GLFW_KEY_3 && N.monitor > 3)
        {
            mon_ind = 3;
        }
        else if (key == GLFW_KEY_4 && N.monitor > 4)
        {
            mon_ind = 4;
        }
        else if (key == GLFW_KEY_5 && N.monitor > 5)
        {
            mon_ind = 5;
        }
        // Check for monitor change
        if (mon_ind != I.monitor)
        {
            // Prompt for projector number if not specified
            I.projector = promptForProjectorNumber();
            ROS_INFO("[callbackKeyBinding] Initiated monitor change: Projector[%d], Monitor[%d]", I.projector, mon_ind);
            // Update the monitor index
            I.monitor = mon_ind;
            // Set the window update flag
            FLAG_CHANGE_WINDOW_MODE = true;
            // Set the reinstalize control points flag
            FLAG_INIT_CONTROL_POINTS = true;
            // Set flag to update mode image
            FLAG_UPDATE_MODE_IMG = true;
        }

        // ---------- Image selector keys [F1-F4] ----------

        // Update the image index
        int img_ind = (CAL_MODE == WALLS_LEFT || CAL_MODE == WALLS_MIDDLE || CAL_MODE == WALLS_RIGHT) ? I.wall_image : I.floor_image;
        if (key == GLFW_KEY_F1)
            img_ind = 0;
        else if (key == GLFW_KEY_F2)
            img_ind = 1;
        else if (key == GLFW_KEY_F3)
            img_ind = 2;
        else if (key == GLFW_KEY_F4)
            img_ind = 3;

        // Check for image change
        if ((CAL_MODE == WALLS_LEFT || CAL_MODE == WALLS_MIDDLE || CAL_MODE == WALLS_RIGHT) &&
            (img_ind != I.wall_image) &&
            (img_ind < N.wall_image))
        {
            ROS_INFO("[callbackKeyBinding] Initiated change image from %d to %d", I.wall_image, img_ind);
            // Update the image index
            I.wall_image = img_ind;
            // Set the update texture flag
            FLAG_UPDATE_HOMOGRAPHYS = true;
            // Set the update mode image flag
            FLAG_UPDATE_MODE_IMG = true;
        }
        if ((CAL_MODE == FLOOR) &&
            (img_ind != I.floor_image) &&
            (img_ind < N.floor_image))
        {
            ROS_INFO("[callbackKeyBinding] Initiated change image from %d to %d", I.floor_image, img_ind);
            // Update the image index
            I.floor_image = img_ind;
            // Set the update texture flag
            FLAG_UPDATE_HOMOGRAPHYS = true;
            // Set the update mode image flag
            FLAG_UPDATE_MODE_IMG = true;
        }

        // ---------- XML Handling [ENTER, L] ----------

        // Save coordinates to XML
        if (key == GLFW_KEY_S)
        {
            ROS_INFO("[callbackKeyBinding] Initiated save XML");
            FLAG_XML_SAVE_HMAT = true;
            FLAG_UPDATE_TEXTURES = true;
        }

        // Load coordinates from XML
        if (key == GLFW_KEY_L)
        {
            ROS_INFO("[callbackKeyBinding] Initiated load XML");
            FLAG_XML_LOAD_HMAT = true;
            FLAG_UPDATE_TEXTURES = true;
        }

        // ---------- Control Point Reset [R] ----------

        if (key == GLFW_KEY_R)
        {
            ROS_INFO("[callbackKeyBinding] Initiated control point reset");
            FLAG_INIT_CONTROL_POINTS = true;
        }
    }

    // _______________ ANY KEY PRESS OR REPEAT ACTION _______________
    else if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {

        // ---------- Calibration mode [CTRL + SHIFT [LEFT, RIGHT]] ----------

        if ((mods & GLFW_MOD_CONTROL) && (mods & GLFW_MOD_SHIFT))
        {
            bool is_cal_mode_changed = false;

            // Listen for arrow key input to switch through calibration modes
            if (key == GLFW_KEY_LEFT)
            {
                CAL_MODE = (CAL_MODE > 0) ? static_cast<CalibrationMode>(CAL_MODE - 1) : static_cast<CalibrationMode>(0);
                is_cal_mode_changed = true;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                CAL_MODE = (CAL_MODE < N_CAL_MODES - 1) ? static_cast<CalibrationMode>(CAL_MODE + 1) : static_cast<CalibrationMode>(N_CAL_MODES - 1);
                is_cal_mode_changed = true;
            }

            // Set flags
            if (is_cal_mode_changed)
            {
                // Set flag to update the control points
                FLAG_INIT_CONTROL_POINTS = true;
                // Set flag to update mode image
                FLAG_UPDATE_MODE_IMG = true;
            }
        }

        // ---------- Contol point maze vertex selector keys [CTRL [LEFT, RIGHT, UP, DOWN]] ----------

        else if (mods & GLFW_MOD_CONTROL && CAL_MODE < 3)
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
                // Set flag to update the wall homography matrix when the vertex is changed
                FLAG_UPDATE_HOMOGRAPHYS = true;

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

            // Get the maze and wall vertex indices cooresponding to the row and column of the selected control point
            int mv_ind = I.CP_MAP[I.cp_maze_vert_selected[0]][I.cp_maze_vert_selected[1]];
            int wv_ind = I.CP_MAP[I.cp_wall_vert_selected[0]][I.cp_wall_vert_selected[1]];

            // Store current origin
            cv::Point2f cp_origin_save = CP_GRID_ARR[mv_ind][I.cp_wall_origin_vertex];

            // Listen for arrow key input to move selected control point
            if (key == GLFW_KEY_LEFT)
            {
                CP_GRID_ARR[mv_ind][wv_ind].x -= pos_inc; // Move left
                FLAG_UPDATE_HOMOGRAPHYS = true;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                CP_GRID_ARR[mv_ind][wv_ind].x += pos_inc; // Move right
                FLAG_UPDATE_HOMOGRAPHYS = true;
            }
            else if (key == GLFW_KEY_UP)
            {
                CP_GRID_ARR[mv_ind][wv_ind].y -= pos_inc; // Move up
                FLAG_UPDATE_HOMOGRAPHYS = true;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                CP_GRID_ARR[mv_ind][wv_ind].y += pos_inc; // Move down
                FLAG_UPDATE_HOMOGRAPHYS = true;
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

void initVertexCoordinates(
    CalibrationMode _CAL_MODE,
    std::array<std::array<cv::Point2f, 4>, 4> &out_CP_GRID_ARR,
    std::array<std::array<std::array<cv::Point2f, 4>, GLB_MAZE_SIZE>, GLB_MAZE_SIZE> &out_WALL_GRID_ARR_DEFAULT)
{
    // Spacing between vertices for the x and y axis
    // Set the spacing to be the wall image size for wall calibration and the maze size for floor calibration
    float vert_space_x = (_CAL_MODE == WALLS_LEFT || _CAL_MODE == WALLS_MIDDLE || _CAL_MODE == WALLS_RIGHT) ? GLB_WALL_IMAGE_WIDTH_NDC : GLB_MAZE_WIDTH_NDC;
    float vert_space_y = (_CAL_MODE == WALLS_LEFT || _CAL_MODE == WALLS_MIDDLE || _CAL_MODE == WALLS_RIGHT) ? GLB_WALL_IMAGE_HEIGHT_NDC : GLB_MAZE_HEIGHT_NDC;

    // Starting and ending points for x and y
    cv::Point2f start_point(-(GLB_MAZE_WIDTH_NDC / 2), -(GLB_MAZE_HEIGHT_NDC / 2));
    cv::Point2f end_point(+(GLB_MAZE_WIDTH_NDC / 2), +(GLB_MAZE_HEIGHT_NDC / 2));

    // Specify number of rows/cols to loop through
    int grid_size = (_CAL_MODE == WALLS_LEFT || _CAL_MODE == WALLS_MIDDLE || _CAL_MODE == WALLS_RIGHT) ? GLB_MAZE_SIZE : 1;

    // Iterate through the maze grid rows
    for (int gr_i = 0; gr_i < grid_size; gr_i++) // image bottom to top
    {
        // Iterate through each column in the maze row
        for (int gc_i = 0; gc_i < grid_size; gc_i++) // image left to right
        {
            // Calculate x and y
            double p_org_x = start_point.x;
            double p_org_y = start_point.y;
            if (_CAL_MODE == WALLS_LEFT || _CAL_MODE == WALLS_MIDDLE || _CAL_MODE == WALLS_RIGHT)
            {
                // Calculate wall specific x and y
                p_org_x = start_point.x + ((end_point.x - start_point.x) * gc_i) / (grid_size - 1);
                p_org_y = start_point.y + ((end_point.y - start_point.y) * gr_i) / (grid_size - 1);
            }

            // Set x y values for vertex of the wall
            out_WALL_GRID_ARR_DEFAULT[gr_i][gc_i] = {
                cv::Point2f(p_org_x, p_org_y),                               // Top left wall vertext
                cv::Point2f(p_org_x + vert_space_x, p_org_y),                // Top right wall vertext
                cv::Point2f(p_org_x + vert_space_x, p_org_y + vert_space_y), // Bottom right wall vertext
                cv::Point2f(p_org_x, p_org_y + vert_space_y),                // Bottom left wall vertext
            };
        }
    }

    // Reinitialize the control points
    out_CP_GRID_ARR[0] = out_WALL_GRID_ARR_DEFAULT[0][0];                         // Top left maze corner
    out_CP_GRID_ARR[1] = out_WALL_GRID_ARR_DEFAULT[0][GLB_MAZE_SIZE - 1];             // Top right maze corner
    out_CP_GRID_ARR[2] = out_WALL_GRID_ARR_DEFAULT[GLB_MAZE_SIZE - 1][GLB_MAZE_SIZE - 1]; // Bottom right maze corner
    out_CP_GRID_ARR[3] = out_WALL_GRID_ARR_DEFAULT[GLB_MAZE_SIZE - 1][0];             // Bottom left maze corner

    // Reset the selected control points
    I.cp_maze_vert_selected = {0, 0};
    I.cp_wall_vert_selected = {1, 0};
}

int initCircleRendererObjects(const std::array<std::array<cv::Point2f, 4>, 4> &_CP_GRID_ARR,
                              std::array<std::array<CircleRenderer, 4>, 4> &out_CP_RENDERERS)
{
    // Iterate through control point outer array (maze vertices)
    for (int mv_i = 0; mv_i < 4; mv_i++)
    {
        // Iterate through control point inner array (wall vertices)
        for (int wv_i = 0; wv_i < 4; ++wv_i)
        {
            out_CP_RENDERERS[mv_i][wv_i].initializeCircleObject(
                _CP_GRID_ARR[mv_i][wv_i], // position
                cpMakerRadius,            // radius
                cpDefaultRGB,             // color
                cpRenderSegments          // segments
            );
        }
    }

    // Return GL status
    return MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__, "initCircleRendererObjects");
}

int drawControlPoints(CalibrationMode _CAL_MODE,
                      const std::array<std::array<cv::Point2f, 4>, 4> &_CP_GRID_ARR,
                      std::array<std::array<CircleRenderer, 4>, 4> &out_CP_RENDERERS)
{
    // Setup the CircleRenderer class shaders
    CircleRenderer::SetupShader();

    // Specify number of control point groups to loop through
    int n_cp_groups = (_CAL_MODE == WALLS_LEFT || _CAL_MODE == WALLS_MIDDLE || _CAL_MODE == WALLS_RIGHT) ? 4 : 1;

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

            // Set the marker parameters
            out_CP_RENDERERS[cp_i][cp_j].setPosition(_CP_GRID_ARR[cp_i][cp_j]);
            out_CP_RENDERERS[cp_i][cp_j].setColor(col);

            // Recompute the marker parameters
            out_CP_RENDERERS[cp_i][cp_j].updateCircleObject();

            // Draw the marker
            out_CP_RENDERERS[cp_i][cp_j].draw();

            // Check for errors
            if (MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__) < 0)
            {
                ROS_ERROR("[drawControlPoints] Error Thrown for Control Point[%d][%d]", cp_i, cp_j);
                return -1;
            }
        }
    }

    // Unset the shader program
    CircleRenderer::UnsetShader();

    // Return GL status
    return MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__, "drawControlPoints");
}

int updateWallHomographys(
    CalibrationMode _CAL_MODE,
    const std::array<std::array<cv::Point2f, 4>, 4> &_CP_GRID_ARR,
    const std::array<std::array<std::array<cv::Point2f, 4>, GLB_MAZE_SIZE>, GLB_MAZE_SIZE> &_WALL_GRID_ARR_DEFAULT,
    std::array<std::array<std::array<cv::Mat, GLB_MAZE_SIZE>, GLB_MAZE_SIZE>, N_CAL_MODES> &out_HMAT_ARR)
{
    // Define source plane vertices
    std::vector<cv::Point2f> source_vertices_pxl = {
        cv::Point2f(0.0f, 0.0f),                                  // Top-left
        cv::Point2f(GLB_WALL_IMAGE_WIDTH_PXL, 0.0f),                  // Top-right
        cv::Point2f(GLB_WALL_IMAGE_WIDTH_PXL, GLB_WALL_IMAGE_HEIGHT_PXL), // Bottom-right
        cv::Point2f(0.0f, GLB_WALL_IMAGE_HEIGHT_PXL)};                // Bottom-left

    // Iterate trough grid/wall rows
    for (float gr_i = 0; gr_i < GLB_MAZE_SIZE; gr_i++) // image bottom to top
    {
        // Iterate trough grid/wall columns
        for (float gc_i = 0; gc_i < GLB_MAZE_SIZE; gc_i++) // image left to right
        {
            std::vector<cv::Point2f> target_vertices_ndc(4);

            // Loop through the wall vertices
            for (int wv_i = 0; wv_i < 4; wv_i++)
            {
                // Get the source vertices based on the default wall vertex coordinates
                std::vector<cv::Point2f> s_vert =
                    {
                        _WALL_GRID_ARR_DEFAULT[0][0][wv_i],                         // Top left maze corner
                        _WALL_GRID_ARR_DEFAULT[0][GLB_MAZE_SIZE - 1][wv_i],             // Top right maze corner
                        _WALL_GRID_ARR_DEFAULT[GLB_MAZE_SIZE - 1][GLB_MAZE_SIZE - 1][wv_i], // Bottom right maze corner
                        _WALL_GRID_ARR_DEFAULT[GLB_MAZE_SIZE - 1][0][wv_i],             // Bottom left maze corner
                    };

                // Get the target vertices corresponding to this wall vertex
                std::vector<cv::Point2f> t_vert =
                    {
                        _CP_GRID_ARR[0][wv_i], // Top-left maze corner
                        _CP_GRID_ARR[1][wv_i], // Top-right maze corner
                        _CP_GRID_ARR[2][wv_i], // Bottom-right maze corner
                        _CP_GRID_ARR[3][wv_i]  // Bottom-left maze corner
                    };

                // Compute the homography matrix for this this point
                cv::Mat H;
                if (computeHomographyMatrix(s_vert, t_vert, H) < 0)
                    return -1;

                // Create a vector for the original and warped point
                std::vector<cv::Point2f> s_pnt = {_WALL_GRID_ARR_DEFAULT[gr_i][gc_i][wv_i]};
                std::vector<cv::Point2f> t_pnt(1);

                // Perspective warp the source vertex to the target plane
                cv::perspectiveTransform(s_pnt, t_pnt, H);

                // Store the transformed point
                target_vertices_ndc[wv_i] = t_pnt[0];
            }

            // Convert NDC coordinates to pixels
            std::vector<cv::Point2f> target_vertices_pxl = quadVertNdc2Pxl(target_vertices_ndc, GLB_MONITOR_WIDTH_PXL, GLB_MONITOR_HEIGHT_PXL);

            // Compute and store the homography matrix for this wall image
            if (computeHomographyMatrix(source_vertices_pxl, target_vertices_pxl, out_HMAT_ARR[_CAL_MODE][gr_i][gc_i]) < 0)
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
        cv::Point2f(0.0f, 0.0f),                                  // Top-left
        cv::Point2f(GLB_MAZE_IMAGE_WIDTH_PXL, 0.0f),                  // Top-right
        cv::Point2f(GLB_MAZE_IMAGE_WIDTH_PXL, GLB_MAZE_IMAGE_HEIGHT_PXL), // Bottom-right
        cv::Point2f(0.0f, GLB_MAZE_IMAGE_HEIGHT_PXL)};                // Bottom-left

    // Convert _CP_ARR to vector
    std::vector<cv::Point2f> target_vertices_ndc(_CP_ARR.begin(), _CP_ARR.end());

    // Convert NDC coordinates to pixels
    std::vector<cv::Point2f> target_vertices_pxl = quadVertNdc2Pxl(target_vertices_ndc, GLB_MONITOR_WIDTH_PXL, GLB_MONITOR_HEIGHT_PXL);

    // Compute and store the homography matrix
    if (computeHomographyMatrix(source_vertices_pxl, target_vertices_pxl, out_H) < 0)
        return -1;

    // Return success
    return 0;
}

int updateModeImage(cv::Mat img_main_mat, cv::Mat img_mon_mat, cv::Mat img_cal_mat, cv::Mat &out_img_mode_mat)
{
    // Copy main image to output image
    img_main_mat.copyTo(out_img_mode_mat);

    // Merge test pattern and active monitor image
    if (mergeImgMat(img_mon_mat, out_img_mode_mat) < 0)
        return -1;

    // Merge previous image and active calibration image
    if (mergeImgMat(img_cal_mat, out_img_mode_mat) < 0)
        return -1;

    return 0;
}

int updateTexture(
    cv::Mat img_base_mat, cv::Mat img_mode_mat, CalibrationMode _CAL_MODE,
    const std::array<std::array<std::array<cv::Mat, GLB_MAZE_SIZE>, GLB_MAZE_SIZE>, N_CAL_MODES> &_HMAT_ARR,
    MazeRenderContext &out_projCtx) {
    // Initialize the image to be used as the texture
    cv::Mat img_merge = cv::Mat::zeros(GLB_MONITOR_HEIGHT_PXL, GLB_MONITOR_WIDTH_PXL, CV_8UC4);

    // Specify number of rows/cols to loop through
    int grid_size = (_CAL_MODE == WALLS_LEFT || _CAL_MODE == WALLS_MIDDLE || _CAL_MODE == WALLS_RIGHT) ? GLB_MAZE_SIZE : 1;

    // Iterate through the maze grid rows
    for (int gr_i = 0; gr_i < grid_size; gr_i++) { // image bottom to top
        // Iterate through each column in the maze row
        for (int gc_i = 0; gc_i < grid_size; gc_i++) { // image left to right
            cv::Mat img_copy;

            // Get the maze vertex indice cooresponding to the selected control point
            int mv_ind = I.CP_MAP[I.cp_maze_vert_selected[0]][I.cp_maze_vert_selected[1]];

            //  Check if mode image should be used
            if ((mv_ind == 0 && gr_i == 0 && gc_i == 0) ||
                (mv_ind == 1 && gr_i == 0 && gc_i == grid_size - 1) ||
                (mv_ind == 3 && gr_i == grid_size - 1 && gc_i == 0) ||
                (mv_ind == 2 && gr_i == grid_size - 1 && gc_i == grid_size - 1))
                img_mode_mat.copyTo(img_copy); // Use mode image
            else
                img_base_mat.copyTo(img_copy); // Use standard image

            // Get homography matrix for this wall
            cv::Mat H = _HMAT_ARR[_CAL_MODE][gr_i][gc_i];

            // Warp the image
            cv::Mat img_warp;
            cv::warpPerspective(img_copy, img_warp, H, cv::Size(GLB_MONITOR_WIDTH_PXL, GLB_MONITOR_HEIGHT_PXL));

            // Merge the warped image with the final image
            if (mergeImgMat(img_warp, img_merge) < 0) return -1;
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

void appLoadAssets()
{
    // Log setup parameters
    ROS_INFO("[appLoadAssets] Config XML Path: %s", GLB_CONFIG_DIR_PATH.c_str());
    ROS_INFO("[appLoadAssets] Display: Width[%d] Height[%d]", GLB_MONITOR_WIDTH_PXL, GLB_MONITOR_HEIGHT_PXL);
    ROS_INFO("[appLoadAssets] Floor (Pxl): Width[%d] Height[%d]", GLB_MAZE_IMAGE_WIDTH_PXL, GLB_MAZE_IMAGE_HEIGHT_PXL);
    ROS_INFO("[appLoadAssets] Floor (NDC): Width[%0.2f] Height[%0.2f] Space Horz[%0.2f] Space Vert[%0.2f]", GLB_MAZE_WIDTH_NDC, GLB_MAZE_HEIGHT_NDC);
    ROS_INFO("[appLoadAssets] Wall (Pxl): Width[%d] Height[%d]", GLB_WALL_IMAGE_WIDTH_PXL, GLB_WALL_IMAGE_HEIGHT_PXL);
    ROS_INFO("[appLoadAssets] Wall (NDC): Width[%0.2f] Height[%0.2f] Space Horz[%0.2f] Space Vert[%0.2f]", GLB_WALL_IMAGE_WIDTH_NDC, GLB_WALL_IMAGE_HEIGHT_NDC);
    ROS_INFO("[appLoadAssets] Origin Plane (NDC): Width[%0.2f] Height[%0.2f]", GLB_MONITOR_WIDTH_PXL, GLB_MONITOR_HEIGHT_PXL);

    // Load images using OpenCV
    if (loadImgMat(fiImgPathWallVec, wallImgMatVec) < 0)
        throw std::runtime_error("[appLoadAssets] Failed to load OpentCV wall test images");
    if (loadImgMat(fiImgPathFloorVec, floorImgMatVec) < 0)
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
    std::vector<int> proj_mon_vec = {1,2,3,4};
    if (MazeRenderContext::SetupGraphicsLibraries(N.monitor, proj_mon_vec) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to initialize graphics");

    // Initialze render context
    if (projCtx.initWindowContext(0, 0, GLB_MONITOR_WIDTH_PXL, GLB_MONITOR_HEIGHT_PXL, callbackKeyBinding) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to initialize render context");

    // Initialize OpenGL wall image objects
    if (projCtx.initRenderObjects(GLB_QUAD_GL_VERTICES, sizeof(GLB_QUAD_GL_VERTICES), GLB_QUAD_GL_INDICES, sizeof(GLB_QUAD_GL_INDICES)) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to initialize opengl wall image objects");

    // Update monitor and window mode settings
    if (projCtx.changeWindowDisplayMode(I.monitor, FLAG_FULLSCREEN_MODE, cv::Point(0.0f, 0.0f)) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed Initial update of window monitor mode");

    // Create the shader program for wall image rendering
    if (projCtx.compileAndLinkShaders(GLB_QUAD_GL_VERTEX_SOURCE, GLB_QUAD_GL_FRAGMENT_SOURCE) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to compile and link wall shader");

    // Create the shader program for CircleRenderer class control point rendering
    if (CircleRenderer::CompileAndLinkCircleShaders(GLB_MONITOR_AR) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to compile and link circlerenderer class shader");

    // Initialize the CircleRenderer class objects array
    if (initCircleRendererObjects(CP_GRID_ARR, CP_CIRCREND_ARR) < 0)
        throw std::runtime_error("[appInitOpenGL] Failed to initialize control point variables");

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

    // Check for non-initialized XML files and initialize them
    for (int proj_i = 0; proj_i < 4; ++proj_i)
    {
        bool isMonMissing = false;

        for (int cal_i = 0; cal_i < N_CAL_MODES && !isMonMissing; ++cal_i)
        {
            CalibrationMode _CAL_MODE = static_cast<CalibrationMode>(cal_i);

            // Specify number of rows/cols to loop through based on active calibration mode
            int grid_size = (_CAL_MODE == WALLS_LEFT || _CAL_MODE == WALLS_MIDDLE || _CAL_MODE == WALLS_RIGHT) ? GLB_MAZE_SIZE : 1;

            // Iterate through the maze grid rows
            for (int gr_i = 0; gr_i < grid_size && !isMonMissing; ++gr_i)
            {
                for (int gc_i = 0; gc_i < grid_size && !isMonMissing; ++gc_i)
                {
                    // Check if the file exists
                    xmlFrmtFileStringsHmat(proj_i, file_path);
                    if (!fileExists(file_path))
                    {
                        // Check if mon_i is already in mon_missing_vec
                        if (std::find(mon_missing_vec.begin(), mon_missing_vec.end(), proj_i) == mon_missing_vec.end())
                        {
                            mon_missing_vec.push_back(proj_i);
                            isMonMissing = true; // Break out of the nested loops
                            ROS_WARN("[appInitFileXML] Initilizing XML file for Projector[%d]: %s", proj_i, file_path.c_str());
                        }
                    }
                }
            }
        }
    }

    // Hack: make temp variables for control point grid and wall homography matrices
    std::array<std::array<cv::Point2f, 4>, 4> _CP_GRID_ARR;
    std::array<std::array<std::array<cv::Point2f, 4>, GLB_MAZE_SIZE>, GLB_MAZE_SIZE> _WALL_GRID_ARR_DEFAULT;
    std::array<std::array<std::array<cv::Mat, GLB_MAZE_SIZE>, GLB_MAZE_SIZE>, N_CAL_MODES> _HMAT_ARR;

    // Loop through the missing projectors
    for (auto &proj_i : mon_missing_vec)
    {
        for (int cal_i = 0; cal_i < N_CAL_MODES; ++cal_i)
        {
            CalibrationMode _CAL_MODE = static_cast<CalibrationMode>(cal_i);

            // Initalize temp control points and homography matrices
            initVertexCoordinates(_CAL_MODE, _CP_GRID_ARR, _WALL_GRID_ARR_DEFAULT);
            if (_CAL_MODE == WALLS_LEFT || _CAL_MODE == WALLS_MIDDLE || _CAL_MODE == WALLS_RIGHT)
            {
                if (updateWallHomographys(_CAL_MODE, _CP_GRID_ARR, _WALL_GRID_ARR_DEFAULT, _HMAT_ARR) < 0)
                    throw std::runtime_error("[appInitFileXML] Failed to initialize wall parameters");
            }
            else if (_CAL_MODE == FLOOR)
            {
                if (updateFloorHomography(_CP_GRID_ARR[0], _HMAT_ARR[_CAL_MODE][0][0]) < 0)
                    throw std::runtime_error("[appInitFileXML] Failed to initialize floor parameters");
            }

            // Specify number of rows/cols to loop through based on active calibration mode
            int grid_size = (_CAL_MODE == WALLS_LEFT || _CAL_MODE == WALLS_MIDDLE || _CAL_MODE == WALLS_RIGHT) ? GLB_MAZE_SIZE : 1;

            // Save each homography matrix to XML
            for (int gr_i = 0; gr_i < grid_size; ++gr_i)
            {
                for (int gc_i = 0; gc_i < grid_size; ++gc_i)
                {
                    if (xmlSaveHMAT(_HMAT_ARR[_CAL_MODE][gr_i][gc_i], proj_i, _CAL_MODE, gr_i, gc_i) < 0)
                        throw std::runtime_error("[appInitFileXML] Error returned from: xmlSaveHMAT");
                    if (_CAL_MODE == FLOOR)
                    {
                        std::vector<cv::Point2f> vert_vec(_CP_GRID_ARR[0].begin(), _CP_GRID_ARR[0].end());
                        if (xmlSaveVertices(vert_vec, proj_i) < 0)
                            throw std::runtime_error("[appInitFileXML] Error returned from: xmlSaveVertices");
                    }
                }
            }

            // Specify number of control point groups to loop through based on active calibration mode
            int cp_group_size = (CAL_MODE == WALLS_LEFT || CAL_MODE == WALLS_MIDDLE || CAL_MODE == WALLS_RIGHT) ? 4 : 1;

            // Save the control points to XML
            for (int cp_i = 0; cp_i < cp_group_size; ++cp_i)
            {
                if (xmlSaveControlPoints(_CP_GRID_ARR[cp_i], proj_i, _CAL_MODE, cp_i) < 0)
                    throw std::runtime_error("[appInitFileXML] Error returned from: xmlSaveControlPoints");
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
        if (FLAG_XML_LOAD_HMAT || FLAG_XML_SAVE_HMAT)
        {
            // Prompt for projector number if not specified
            if (I.projector < 0)
                I.projector = promptForProjectorNumber();

            // Specify number of rows/cols to loop through based on active calibration mode
            int grid_size = (CAL_MODE == WALLS_LEFT || CAL_MODE == WALLS_MIDDLE || CAL_MODE == WALLS_RIGHT) ? GLB_MAZE_SIZE : 1;

            // Loop through the maze grid rows
            for (int gr_i = 0; gr_i < grid_size; ++gr_i)
            {
                for (int gc_i = 0; gc_i < grid_size; ++gc_i)
                {
                    // Save XML file
                    if (FLAG_XML_SAVE_HMAT)
                    {
                        // Save the homography matrix to XML
                        if (xmlSaveHMAT(HMAT_ARR[CAL_MODE][gr_i][gc_i], I.projector, CAL_MODE, gr_i, gc_i) < 0)
                            throw std::runtime_error("[appMainLoop] Error returned from: xmlSaveHMAT");

                        // Save the maze vertices to XML
                        if (CAL_MODE == FLOOR)
                        {
                            std::vector<cv::Point2f> vert_vec(CP_GRID_ARR[0].begin(), CP_GRID_ARR[0].end());
                            if (xmlSaveVertices(vert_vec, I.projector) < 0)
                                throw std::runtime_error("[appMainLoop] Error returned from: xmlSaveVertices");
                        }
                    }
                    // Load XML file
                    if (FLAG_XML_LOAD_HMAT)
                    {
                        // Load the homography matrix from XML
                        if (xmlLoadHMAT(I.projector, CAL_MODE, gr_i, gc_i, HMAT_ARR[CAL_MODE][gr_i][gc_i]) < 0)
                            throw std::runtime_error("[appMainLoop] Error returned from: xmlLoadHMAT");
                    }
                }
            }

            // Specify number of control point groups to loop through based on active calibration mode
            int cp_group_size = (CAL_MODE == WALLS_LEFT || CAL_MODE == WALLS_MIDDLE || CAL_MODE == WALLS_RIGHT) ? 4 : 1;

            // Save/load the control points to XML
            for (int cp_i = 0; cp_i < cp_group_size; ++cp_i)
            {
                if (FLAG_XML_SAVE_HMAT)
                {
                    if (xmlSaveControlPoints(CP_GRID_ARR[cp_i], I.projector, CAL_MODE, cp_i) < 0)
                        throw std::runtime_error("[appMainLoop] Error returned from: xmlSaveControlPoints");
                }
                if (FLAG_XML_LOAD_HMAT)
                {
                    if (xmlLoadControlPoints(I.projector, CAL_MODE, cp_i, CP_GRID_ARR[cp_i]) < 0)
                        throw std::runtime_error("[appMainLoop] Error returned from: xmlSaveControlPoints");
                }
            }

            // Flash the background to indicate the file was loaded/saved
            projCtx.flashBackgroundColor(cv::Scalar(0.0f, 0.25f, 0.0f), 10);
        }

        // Update the window monitor and mode
        if (FLAG_CHANGE_WINDOW_MODE)
        {
            if (projCtx.changeWindowDisplayMode(I.monitor, FLAG_FULLSCREEN_MODE) < 0)
                throw std::runtime_error("[appMainLoop] Error returned from: changeWindowDisplayMode");
        }

        // Initialize/reinitialize control point coordinate dataset
        if (FLAG_INIT_CONTROL_POINTS)
        {
            initVertexCoordinates(CAL_MODE, CP_GRID_ARR, WALL_GRID_ARR_DEFAULT);
        }

        // Update homography matrices array
        if (FLAG_UPDATE_HOMOGRAPHYS ||
            FLAG_INIT_CONTROL_POINTS)
        {
            // Update wall homographys
            if (CAL_MODE == WALLS_LEFT || CAL_MODE == WALLS_MIDDLE || CAL_MODE == WALLS_RIGHT)
            {
                if (updateWallHomographys(CAL_MODE, CP_GRID_ARR, WALL_GRID_ARR_DEFAULT, HMAT_ARR) < 0)
                    throw std::runtime_error("[appMainLoop] Error returned from: updateWallHomographys");
            }
            // Update floor homography
            else if (CAL_MODE == FLOOR)
            {
                if (updateFloorHomography(CP_GRID_ARR[0], HMAT_ARR[CAL_MODE][0][0]) < 0)
                    throw std::runtime_error("[appMainLoop] Error returned from: updateFloorHomography");
            }
        }

        // Update the caliration and monitor mode image
        if (FLAG_UPDATE_MODE_IMG)
        {
            // Update wall textures
            if (CAL_MODE == WALLS_LEFT || CAL_MODE == WALLS_MIDDLE || CAL_MODE == WALLS_RIGHT)
            {
                if (updateModeImage(wallImgMatVec[I.wall_image], monWallImgMatVec[I.monitor], calImgMatVec[CAL_MODE], modeImgMat) < 0)
                    throw std::runtime_error("[appMainLoop] Error returned from: updateModeImage for wall image");
            }
            // Update floor texture
            else if (CAL_MODE == FLOOR)
            {
                if (updateModeImage(floorImgMatVec[I.floor_image], monFloorImgMatVec[I.monitor], calImgMatVec[CAL_MODE], modeImgMat) < 0)
                    throw std::runtime_error("[appMainLoop] Error returned from: updateModeImage for floor image");
            }
        }

        // Update image texture
        if (FLAG_UPDATE_TEXTURES ||
            FLAG_UPDATE_HOMOGRAPHYS ||
            FLAG_INIT_CONTROL_POINTS)
        {
            // Update wall textures
            if (CAL_MODE == WALLS_LEFT || CAL_MODE == WALLS_MIDDLE || CAL_MODE == WALLS_RIGHT)
            {
                if (updateTexture(wallImgMatVec[I.wall_image], modeImgMat, CAL_MODE, HMAT_ARR, projCtx) < 0)
                    throw std::runtime_error("[appMainLoop] Error returned from: updateTexture for wall images");
            }
            // Update floor texture
            else if (CAL_MODE == FLOOR)
            {
                if (updateTexture(floorImgMatVec[I.floor_image], modeImgMat, CAL_MODE, HMAT_ARR, projCtx) < 0)
                    throw std::runtime_error("[appMainLoop] Error returned from: updateTexture for floor images");
            }
        }

        // Reset keybinding flags
        FLAG_XML_LOAD_HMAT = false;
        FLAG_XML_SAVE_HMAT = false;
        FLAG_CHANGE_WINDOW_MODE = false;
        FLAG_INIT_CONTROL_POINTS = false;
        FLAG_UPDATE_TEXTURES = false;
        FLAG_UPDATE_HOMOGRAPHYS = false;
        FLAG_UPDATE_MODE_IMG = false;

        // --------------- Handle Rendering for Next Frame ---------------

        // Prepare the frame for rendering (make context clear the back buffer)
        if (projCtx.initWindowForDrawing() < 0)
            throw std::runtime_error("[appMainLoop] Error returned from: MazeRenderContext::initWindowForDrawing");

        // Make sure winsow always stays on top in fullscreen mode
        if (projCtx.forceWindowFocus() < 0)
            throw std::runtime_error("[appMainLoop] Error returned from: MazeRenderContext::forceWindowFocus");

        // Draw/update texture
        if (projCtx.drawTexture() < 0)
            throw std::runtime_error("[appMainLoop] Error returned from: drawTexture");

        // Draw/update control point markers
        if (drawControlPoints(CAL_MODE, CP_GRID_ARR, CP_CIRCREND_ARR) < 0)
            throw std::runtime_error("[appMainLoop] Error returned from: drawControlPoints");

        // Swap buffers and poll events
        if (projCtx.bufferSwapPoll() < 0)
            throw std::runtime_error("[appMainLoop] Error returned from: MazeRenderContext::bufferSwapPoll");

        // Check if ROS shutdown
        if (!ros::ok())
            throw std::runtime_error("[appMainLoop] Unexpected ROS shutdown");

        // Check for exit
        status = projCtx.checkExitRequest();
        if (status < 0)
            throw std::runtime_error("[appMainLoop] Error returned from: MazeRenderContext::checkExitRequest");
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
    if (projCtx.cleanupContext(false) != 0)
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
