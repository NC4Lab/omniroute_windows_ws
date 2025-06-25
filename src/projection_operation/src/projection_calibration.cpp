// ############################################################################################################

// ======================================== projection_calibration.cpp ========================================

// ############################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_calibration.h"

// ================================================== FUNCTIONS ==================================================

void callbackKeyBinding(GLFWwindow *window, int key, int scancode, int action, int mods) {
    // Set the current OpenGL context to the window
    glfwMakeContextCurrent(window);

    // _______________ ANY KEY RELEASE ACTION _______________
    if (action == GLFW_RELEASE) {

        // ----------Set/unset Fullscreen [F] ----------

        if (key == GLFW_KEY_F) {
            F.fullscreen_mode = !F.fullscreen_mode;
            F.change_window_mode = true;
        }

        // ----------Move the window to another monitor [0-5] ----------

        // Check number keys and update the monitor index
        int mon_ind = I.monitor;
        if (key == GLFW_KEY_0)                          mon_ind = 0;
        else if (key == GLFW_KEY_1 && N.monitor > 1)    mon_ind = 1;
        else if (key == GLFW_KEY_2 && N.monitor > 2)    mon_ind = 2;
        else if (key == GLFW_KEY_3 && N.monitor > 3)    mon_ind = 3;
        else if (key == GLFW_KEY_4 && N.monitor > 4)    mon_ind = 4;
        else if (key == GLFW_KEY_5 && N.monitor > 5)    mon_ind = 5;

        // Check for monitor change
        if (mon_ind != I.monitor) {
            I.projector = promptForProjectorNumber(); // Prompt for projector number if not specified
            ROS_INFO("[callbackKeyBinding] Initiated monitor change: Projector[%d], Monitor[%d]", I.projector, mon_ind);

            I.monitor = mon_ind;            // Update the monitor index
            F.change_window_mode = true;    // Set the window update flag
            F.init_control_points = true;   // Set the reinstalize control points flag
            F.update_mode_img = true;       // Set flag to update mode image
        }

        // ---------- Image selector keys [F1-F4] ----------
        // Update the image index
        int img_ind = (CAL_MODE == WALLS_LEFT || CAL_MODE == WALLS_MIDDLE || CAL_MODE == WALLS_RIGHT) ? I.wall_image : I.floor_image;
        if (key == GLFW_KEY_F1)         img_ind = 0;
        else if (key == GLFW_KEY_F2)    img_ind = 1;
        else if (key == GLFW_KEY_F3)    img_ind = 2;
        else if (key == GLFW_KEY_F4)    img_ind = 3;

        // Check for image change
        if ((CAL_MODE == WALLS_LEFT || CAL_MODE == WALLS_MIDDLE || CAL_MODE == WALLS_RIGHT) &&
            (img_ind != I.wall_image) &&
            (img_ind < N.wall_image)) {
            ROS_INFO("[callbackKeyBinding] Initiated change image from %d to %d", I.wall_image, img_ind);
            I.wall_image = img_ind;         // Update the image index
            F.update_homographys = true;    // Set the update texture flag
            F.update_mode_img = true;       // Set the update mode image flag
        }

        if ((CAL_MODE == FLOOR) && (img_ind != I.floor_image) && (img_ind < N.floor_image)) {
            ROS_INFO("[callbackKeyBinding] Initiated change image from %d to %d", I.floor_image, img_ind);
            I.floor_image = img_ind;         // Update the image index
            F.update_homographys = true;     // Set the update texture flag
            F.update_mode_img = true;        // Set the update mode image flag
        }

        // ---------- XML Handling [ENTER, L] ----------
        // Save coordinates to XML
        if (key == GLFW_KEY_S) {
            ROS_INFO("[callbackKeyBinding] Initiated save XML");
            F.xml_save_hmat = true;
            F.update_textures = true;
        }

        // Load coordinates from XML
        if (key == GLFW_KEY_L) {
            ROS_INFO("[callbackKeyBinding] Initiated load XML");
            F.xml_load_hmat = true;
            F.update_textures = true;
        }

        // ---------- Control Point Reset [R] ----------
        if (key == GLFW_KEY_R) {
            ROS_INFO("[callbackKeyBinding] Initiated control point reset");
            F.init_control_points = true;
        }
    }

    // _______________ ANY KEY PRESS OR REPEAT ACTION _______________
    else if (action == GLFW_PRESS || action == GLFW_REPEAT) {

        // ---------- Calibration mode [CTRL + SHIFT [LEFT, RIGHT]] ----------
        if ((mods & GLFW_MOD_CONTROL) && (mods & GLFW_MOD_SHIFT)) {
            // Listen for arrow key input to switch through calibration modes
            bool is_cal_mode_changed = true; // Assume a valid key was pressed
            if (key == GLFW_KEY_LEFT)       CAL_MODE = (CAL_MODE > 0) ? static_cast<CalibrationMode>(CAL_MODE - 1) : static_cast<CalibrationMode>(0);
            else if (key == GLFW_KEY_RIGHT) CAL_MODE = (CAL_MODE < N_CAL_MODES - 1) ? static_cast<CalibrationMode>(CAL_MODE + 1) : static_cast<CalibrationMode>(N_CAL_MODES - 1);
            else                            is_cal_mode_changed = false;           // No valid key pressed, do not update calibration mode

            // Set flags
            if (is_cal_mode_changed) {
                F.init_control_points = true;   // Set flag to update the control points
                F.update_mode_img = true;       // Set flag to update mode image
            }
        }

        // ---------- Contol point maze vertex selector keys [CTRL [LEFT, RIGHT, UP, DOWN]] ----------

        else if (mods & GLFW_MOD_CONTROL && CAL_MODE < 3) {
            bool is_vert_changed = true;
            if (key == GLFW_KEY_UP)         I.cp_maze_vert_selected[0] = 0; // Set row index to top row
            else if (key == GLFW_KEY_DOWN)  I.cp_maze_vert_selected[0] = 1; // Set row index to bottom row
            else if (key == GLFW_KEY_LEFT)  I.cp_maze_vert_selected[1] = 0; // Set column index to first column
            else if (key == GLFW_KEY_RIGHT) I.cp_maze_vert_selected[1] = 1; // Set column index to second column
            else                            is_vert_changed = false;        // No valid key pressed, do not update vertex

            if (is_vert_changed) {
                // Set flag to update the wall homography matrix when the vertex is changed
                F.update_homographys = true;

                // Set the wall vertex to the ortin if the maze vertex is changed
                for (int i = 0; i < 2; ++i) {
                    for (int j = 0; j < 2; ++j) {
                        if (I.CP_MAP[i][j] == I.cp_wall_origin_vertex) {
                            // Set the wall vertex to the origin
                            I.cp_wall_vert_selected[0] = i;
                            I.cp_wall_vert_selected[1] = j;
                        }
                    }
                }
            }
        }

        // ---------- Control point wall vertex selector keys [ALT [LEFT, RIGHT, UP, DOWN]] ----------
        else if (mods & GLFW_MOD_ALT) {
            if (key == GLFW_KEY_UP)             I.cp_wall_vert_selected[0] = 0; // Set row index to top row
            else if (key == GLFW_KEY_DOWN)      I.cp_wall_vert_selected[0] = 1; // Set row index to bottom row
            else if (key == GLFW_KEY_LEFT)      I.cp_wall_vert_selected[1] = 0; // Set column index to first column
            else if (key == GLFW_KEY_RIGHT)     I.cp_wall_vert_selected[1] = 1; // Set column index to second column
        }

        // ---------- Control point translate [SHIFT or no modifier] ----------
        else {
            // Set the position increment based on whether the shift key is pressed
            float pos_inc = (mods & GLFW_MOD_SHIFT) ? 0.01f : 0.0005f;

            // Get the maze and wall vertex indices cooresponding to the row and column of the selected control point
            int mv_ind = I.CP_MAP[I.cp_maze_vert_selected[0]][I.cp_maze_vert_selected[1]];
            int wv_ind = I.CP_MAP[I.cp_wall_vert_selected[0]][I.cp_wall_vert_selected[1]];

            // Store current origin
            cv::Point2f cp_origin_save = CP_GRID_ARR[mv_ind][I.cp_wall_origin_vertex];

            // Listen for arrow key input to move selected control point
            bool update_homographys = true;
            if (key == GLFW_KEY_LEFT)       CP_GRID_ARR[mv_ind][wv_ind].x -= pos_inc; // Move left
            else if (key == GLFW_KEY_RIGHT) CP_GRID_ARR[mv_ind][wv_ind].x += pos_inc; // Move right
            else if (key == GLFW_KEY_UP)    CP_GRID_ARR[mv_ind][wv_ind].y -= pos_inc; // Move up
            else if (key == GLFW_KEY_DOWN)  CP_GRID_ARR[mv_ind][wv_ind].y += pos_inc; // Move down
            else update_homographys = false; // No valid key pressed, do not update homography
            F.update_homographys = update_homographys;

            // Shift all control points if origin moved
            cv::Point2f cp_origin_new = CP_GRID_ARR[mv_ind][I.cp_wall_origin_vertex];

            // Calculate the change in x and y for the origin
            float delta_x = cp_origin_new.x - cp_origin_save.x;
            float delta_y = cp_origin_new.y - cp_origin_save.y;

            // Check if the origin vertex of the wall was moved (e.g., bottom-left)
            if (wv_ind == I.cp_wall_origin_vertex && (abs(delta_x) > 0.0f || abs(delta_y) > 0.0f)) {
                // Update all other vertices based on the change in the origin
                for (int i = 0; i < 4; ++i) { // Assuming there are 4 vertices
                    if (i != I.cp_wall_origin_vertex) { // Skip the origin vertex itself
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
    std::array<std::array<std::array<cv::Point2f, 4>, GLB_MAZE_SIZE>, GLB_MAZE_SIZE> &out_WALL_GRID_ARR_DEFAULT) {
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
    for (int gr_i = 0; gr_i < grid_size; gr_i++) { // image bottom to top
        // Iterate through each column in the maze row
        for (int gc_i = 0; gc_i < grid_size; gc_i++) { // image left to right
            // Calculate x and y
            double p_org_x = start_point.x;
            double p_org_y = start_point.y;
            if (_CAL_MODE == WALLS_LEFT || _CAL_MODE == WALLS_MIDDLE || _CAL_MODE == WALLS_RIGHT) {
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
                              std::array<std::array<CircleRenderer, 4>, 4> &out_CP_RENDERERS) {
    // Iterate through control point outer array (maze vertices)
    for (int mv_i = 0; mv_i < 4; mv_i++) {
        // Iterate through control point inner array (wall vertices)
        for (int wv_i = 0; wv_i < 4; ++wv_i) {
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
                      std::array<std::array<CircleRenderer, 4>, 4> &out_CP_RENDERERS) {
    // Setup the CircleRenderer class shaders
    CircleRenderer::SetupShader();

    // Specify number of control point groups to loop through
    int n_cp_groups = (_CAL_MODE == WALLS_LEFT || _CAL_MODE == WALLS_MIDDLE || _CAL_MODE == WALLS_RIGHT) ? 4 : 1;

    // Loop through the control points and draw them
    for (int cp_i = 0; cp_i < n_cp_groups; ++cp_i) {
        for (int cp_j = 0; cp_j < 4; ++cp_j) {
            // Get the maze and wall vertex indices cooresponding to the selected control point
            int mv_ind = I.CP_MAP[I.cp_maze_vert_selected[0]][I.cp_maze_vert_selected[1]];
            int wv_ind = I.CP_MAP[I.cp_wall_vert_selected[0]][I.cp_wall_vert_selected[1]];

            // Define the marker color
            cv::Scalar col = (cp_i == mv_ind && cp_j == wv_ind) ? 
            cpWallVertSelectedRGB : (cp_i == mv_ind) ? 
            cpMazeVertSelectedRGB : cpDefaultRGB;

            // Set the marker parameters
            out_CP_RENDERERS[cp_i][cp_j].setPosition(_CP_GRID_ARR[cp_i][cp_j]);
            out_CP_RENDERERS[cp_i][cp_j].setColor(col);

            // Recompute the marker parameters
            out_CP_RENDERERS[cp_i][cp_j].updateCircleObject();

            // Draw the marker
            out_CP_RENDERERS[cp_i][cp_j].draw();

            // Check for errors
            if (MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__) < 0) {
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
    std::array<std::array<std::array<cv::Mat, GLB_MAZE_SIZE>, GLB_MAZE_SIZE>, N_CAL_MODES> &out_HMAT_ARR) {
    // Define source plane vertices
    std::vector<cv::Point2f> source_vertices_pxl = {
        cv::Point2f(0.0f, 0.0f),                                            // Top-left
        cv::Point2f(GLB_WALL_IMAGE_WIDTH_PXL, 0.0f),                        // Top-right
        cv::Point2f(GLB_WALL_IMAGE_WIDTH_PXL, GLB_WALL_IMAGE_HEIGHT_PXL),   // Bottom-right
        cv::Point2f(0.0f, GLB_WALL_IMAGE_HEIGHT_PXL)};                      // Bottom-left

    // Iterate trough grid/wall rows
    for (float gr_i = 0; gr_i < GLB_MAZE_SIZE; gr_i++) { // image bottom to top
        // Iterate trough grid/wall columns
        for (float gc_i = 0; gc_i < GLB_MAZE_SIZE; gc_i++) { // image left to right
            std::vector<cv::Point2f> target_vertices_ndc(4);

            // Loop through the wall vertices
            for (int wv_i = 0; wv_i < 4; wv_i++) {
                // Get the source vertices based on the default wall vertex coordinates
                std::vector<cv::Point2f> s_vert = {
                        _WALL_GRID_ARR_DEFAULT[0][0][wv_i],                         // Top left maze corner
                        _WALL_GRID_ARR_DEFAULT[0][GLB_MAZE_SIZE - 1][wv_i],             // Top right maze corner
                        _WALL_GRID_ARR_DEFAULT[GLB_MAZE_SIZE - 1][GLB_MAZE_SIZE - 1][wv_i], // Bottom right maze corner
                        _WALL_GRID_ARR_DEFAULT[GLB_MAZE_SIZE - 1][0][wv_i],             // Bottom left maze corner
                    };

                // Get the target vertices corresponding to this wall vertex
                std::vector<cv::Point2f> t_vert = {
                        _CP_GRID_ARR[0][wv_i], // Top-left maze corner
                        _CP_GRID_ARR[1][wv_i], // Top-right maze corner
                        _CP_GRID_ARR[2][wv_i], // Bottom-right maze corner
                        _CP_GRID_ARR[3][wv_i]  // Bottom-left maze corner
                    };

                // Compute the homography matrix for this this point
                cv::Mat H;
                if (computeHomographyMatrix(s_vert, t_vert, H) < 0) return -1;

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

    return 0;
}

int updateFloorHomography(const std::array<cv::Point2f, 4> &_CP_ARR, cv::Mat &out_H) {
    // Define source plane vertices
    std::vector<cv::Point2f> source_vertices_pxl = {
        cv::Point2f(0.0f, 0.0f),                                            // Top-left
        cv::Point2f(GLB_MAZE_IMAGE_WIDTH_PXL, 0.0f),                        // Top-right
        cv::Point2f(GLB_MAZE_IMAGE_WIDTH_PXL, GLB_MAZE_IMAGE_HEIGHT_PXL),   // Bottom-right
        cv::Point2f(0.0f, GLB_MAZE_IMAGE_HEIGHT_PXL)                        // Bottom-left
    }; 

    // Convert _CP_ARR to vector
    std::vector<cv::Point2f> target_vertices_ndc(_CP_ARR.begin(), _CP_ARR.end());

    // Convert NDC coordinates to pixels
    std::vector<cv::Point2f> target_vertices_pxl = quadVertNdc2Pxl(target_vertices_ndc, GLB_MONITOR_WIDTH_PXL, GLB_MONITOR_HEIGHT_PXL);

    // Compute and store the homography matrix
    if (computeHomographyMatrix(source_vertices_pxl, target_vertices_pxl, out_H) < 0) return -1;

    return 0;
}

int updateModeImage(cv::Mat img_main_mat, cv::Mat img_mon_mat, cv::Mat img_cal_mat, cv::Mat &out_img_mode_mat) {
    // Copy main image to output image
    img_main_mat.copyTo(out_img_mode_mat);

    // Merge test pattern and active monitor image
    if (mergeImgMat(img_mon_mat, out_img_mode_mat) < 0) return -1;

    // Merge previous image and active calibration image
    if (mergeImgMat(img_cal_mat, out_img_mode_mat) < 0) return -1;

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
            if (warpImgMat(img_copy, H, img_warp) < 0) {
                ROS_ERROR("[updateTexture] Warp image error: Wall[%d][%d] Calibration[%d]", gr_i, gc_i, _CAL_MODE);
                return -1;
            }

            // Merge the warped image with the final image
            if (mergeImgMat(img_warp, img_merge) < 0)
                return -1;
        }
    }

    // Load the new texture and return status
    if (out_projCtx.loadMatTexture(img_merge) < 0) {
        ROS_ERROR("[updateTexture] Failed to load texture");
        return -1;
    }

    return 0;
}

void appLoadAssets() {
    // Log setup parameters
    ROS_INFO("[appLoadAssets] Config XML Path: %s", CONFIG_DIR_PATH.c_str());
    ROS_INFO("[appLoadAssets] Display: Width[%d] Height[%d]", GLB_MONITOR_WIDTH_PXL, GLB_MONITOR_HEIGHT_PXL);
    ROS_INFO("[appLoadAssets] Floor (Pxl): Width[%d] Height[%d]", GLB_MAZE_IMAGE_WIDTH_PXL, GLB_MAZE_IMAGE_HEIGHT_PXL);
    ROS_INFO("[appLoadAssets] Floor (NDC): Width[%0.2f] Height[%0.2f] Space Horz[%0.2f] Space Vert[%0.2f]", GLB_MAZE_WIDTH_NDC, GLB_MAZE_HEIGHT_NDC);
    ROS_INFO("[appLoadAssets] Wall (Pxl): Width[%d] Height[%d]", GLB_WALL_IMAGE_WIDTH_PXL, GLB_WALL_IMAGE_HEIGHT_PXL);
    ROS_INFO("[appLoadAssets] Wall (NDC): Width[%0.2f] Height[%0.2f] Space Horz[%0.2f] Space Vert[%0.2f]", GLB_WALL_IMAGE_WIDTH_NDC, GLB_WALL_IMAGE_HEIGHT_NDC);
    ROS_INFO("[appLoadAssets] Origin Plane (NDC): Width[%0.2f] Height[%0.2f]", GLB_MONITOR_WIDTH_PXL, GLB_MONITOR_HEIGHT_PXL);

    std::vector<std::string> calibTestWallImages, calibTestFloorImages, calibMonWallImages, calibMonFloorImages, calibModeImages;
    for (auto &filename : CALIB_TEST_WALL_IMAGES)   calibTestWallImages.push_back(CALIB_IMAGE_PATH + "/" + filename);
    for (auto &filename : CALIB_TEST_FLOOR_IMAGES)  calibTestFloorImages.push_back(CALIB_IMAGE_PATH + "/" + filename);
    for (auto &filename : CALIB_MON_WALL_IMAGES)    calibMonWallImages.push_back(CALIB_IMAGE_PATH + "/" + filename);
    for (auto &filename : CALIB_MON_FLOOR_IMAGES)   calibMonFloorImages.push_back(CALIB_IMAGE_PATH + "/" + filename);
    for (auto &filename : CALIB_MODE_IMAGES)        calibModeImages.push_back(CALIB_IMAGE_PATH + "/" + filename);

    // Load images using OpenCV
    if (loadImgMat(calibTestWallImages, calibTestWallMats) < 0)
        throw std::runtime_error("[appLoadAssets] Failed to load OpenCV wall test images");
    if (loadImgMat(calibTestFloorImages, calibTestFloorMats) < 0)
        throw std::runtime_error("[appLoadAssets] Failed to load OpenCV floor test images");
    if (loadImgMat(calibMonWallImages, calibMonWallMats) < 0)
        throw std::runtime_error("[appLoadAssets] Failed to load OpenCV monitor number wall images");
    if (loadImgMat(calibMonFloorImages, calibMonFloorMats) < 0)
        throw std::runtime_error("[appLoadAssets] Failed to load OpenCV monitor number floor images");
    if (loadImgMat(calibModeImages, calibModeMats) < 0)
        throw std::runtime_error("[appLoadAssets] Failed to load OpenCV calibration mode images");

    ROS_INFO("[appLoadAssets] OpenCV mat images loaded succesfully");
}

void appInitOpenGL() {
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
    if (projCtx.changeWindowDisplayMode(I.monitor, F.fullscreen_mode, cv::Point(0.0f, 0.0f)) < 0)
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

void appInitFileXML() {
    std::vector<int> mon_missing_vec;

    // Check for non-initialized XML files and initialize them
    for (int proj_i = 0; proj_i < 4; ++proj_i) {
        bool isMonMissing = false;

        for (int cal_i = 0; cal_i < N_CAL_MODES && !isMonMissing; ++cal_i) {
            CalibrationMode _CAL_MODE = static_cast<CalibrationMode>(cal_i);

            // Specify number of rows/cols to loop through based on active calibration mode
            int grid_size = (_CAL_MODE == WALLS_LEFT || _CAL_MODE == WALLS_MIDDLE || _CAL_MODE == WALLS_RIGHT) ? GLB_MAZE_SIZE : 1;

            // Iterate through the maze grid rows
            for (int gr_i = 0; gr_i < grid_size && !isMonMissing; ++gr_i) {
                for (int gc_i = 0; gc_i < grid_size && !isMonMissing; ++gc_i) {
                    // Check if the file exists
                    if (!fileExists(calXML.fileNameHMat[proj_i])) {
                        // Check if mon_i is already in mon_missing_vec
                        if (std::find(mon_missing_vec.begin(), mon_missing_vec.end(), proj_i) == mon_missing_vec.end()) {
                            mon_missing_vec.push_back(proj_i);
                            isMonMissing = true; // Break out of the nested loops
                            ROS_WARN("[appInitFileXML] Initilizing XML file for Projector[%d]: %s", proj_i, calXML.fileNameHMat[proj_i].c_str());
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
    for (auto &proj_i : mon_missing_vec) {
        for (int cal_i = 0; cal_i < N_CAL_MODES; ++cal_i) {
            CalibrationMode _CAL_MODE = static_cast<CalibrationMode>(cal_i);

            // Initalize temp control points and homography matrices
            initVertexCoordinates(_CAL_MODE, _CP_GRID_ARR, _WALL_GRID_ARR_DEFAULT);
            if (_CAL_MODE == WALLS_LEFT || _CAL_MODE == WALLS_MIDDLE || _CAL_MODE == WALLS_RIGHT)
                if (updateWallHomographys(_CAL_MODE, _CP_GRID_ARR, _WALL_GRID_ARR_DEFAULT, _HMAT_ARR) < 0)
                    throw std::runtime_error("[appInitFileXML] Failed to initialize wall parameters");
            else if (_CAL_MODE == FLOOR)
                if (updateFloorHomography(_CP_GRID_ARR[0], _HMAT_ARR[_CAL_MODE][0][0]) < 0)
                    throw std::runtime_error("[appInitFileXML] Failed to initialize floor parameters");

            // Specify number of rows/cols to loop through based on active calibration mode
            int grid_size = (_CAL_MODE == WALLS_LEFT || _CAL_MODE == WALLS_MIDDLE || _CAL_MODE == WALLS_RIGHT) ? GLB_MAZE_SIZE : 1;

            // Save each homography matrix to XML
            for (int gr_i = 0; gr_i < grid_size; ++gr_i) {
                for (int gc_i = 0; gc_i < grid_size; ++gc_i) {
                    if (calXML.saveHMat(proj_i, _CAL_MODE, gr_i, gc_i, _HMAT_ARR[_CAL_MODE][gr_i][gc_i]) < 0)
                        throw std::runtime_error("[appInitFileXML] Error returned from: calXML.saveHMat");
                    if (_CAL_MODE == FLOOR) {
                        std::vector<cv::Point2f> vert_vec(_CP_GRID_ARR[0].begin(), _CP_GRID_ARR[0].end());
                        if (calXML.saveVertices(proj_i, vert_vec) < 0)
                            throw std::runtime_error("[appInitFileXML] Error returned from: calXML.saveVertices");
                    }
                }
            }

            // Specify number of control point groups to loop through based on active calibration mode
            int cp_group_size = (CAL_MODE == WALLS_LEFT || CAL_MODE == WALLS_MIDDLE || CAL_MODE == WALLS_RIGHT) ? 4 : 1;

            // Save the control points to XML
            for (int cp_i = 0; cp_i < cp_group_size; ++cp_i) {
                if (calXML.saveControlPoints(proj_i, _CAL_MODE, cp_i, _CP_GRID_ARR[cp_i]) < 0)
                    throw std::runtime_error("[appInitFileXML] Error returned from: calXML.saveControlPoints");
            }
        }
    }
}

void appMainLoop() {
    int status = 0;
    while (status == 0) {
        // --------------- Check Kayboard Callback Flags ---------------

        // Load/save XML file
        if (F.xml_load_hmat || F.xml_save_hmat) {
            // Prompt for projector number if not specified
            if (I.projector < 0) I.projector = promptForProjectorNumber();

            // Specify number of rows/cols to loop through based on active calibration mode
            int grid_size = (CAL_MODE == WALLS_LEFT || CAL_MODE == WALLS_MIDDLE || CAL_MODE == WALLS_RIGHT) ? GLB_MAZE_SIZE : 1;

            // Loop through the maze grid rows
            for (int gr_i = 0; gr_i < grid_size; ++gr_i) {
                for (int gc_i = 0; gc_i < grid_size; ++gc_i) {
                    // Save XML file
                    if (F.xml_save_hmat) {
                        // Save the homography matrix to XML
                        if (calXML.saveHMat(I.projector, CAL_MODE, gr_i, gc_i, HMAT_ARR[CAL_MODE][gr_i][gc_i]) < 0)
                            throw std::runtime_error("[appMainLoop] Error returned from: calXML.saveHMat");

                        // Save the maze vertices to XML
                        if (CAL_MODE == FLOOR) {
                            std::vector<cv::Point2f> vert_vec(CP_GRID_ARR[0].begin(), CP_GRID_ARR[0].end());
                            if (calXML.saveVertices(I.projector, vert_vec) < 0)
                                throw std::runtime_error("[appMainLoop] Error returned from: calXML.saveVertices");
                        }
                    }
                    // Load XML file
                    if (F.xml_load_hmat) {
                        // Load the homography matrix from XML
                        if (calXML.loadHMat(I.projector, CAL_MODE, gr_i, gc_i, HMAT_ARR[CAL_MODE][gr_i][gc_i]) < 0)
                            throw std::runtime_error("[appMainLoop] Error returned from: loadHMat");
                    }
                }
            }

            // Specify number of control point groups to loop through based on active calibration mode
            int cp_group_size = (CAL_MODE == WALLS_LEFT || CAL_MODE == WALLS_MIDDLE || CAL_MODE == WALLS_RIGHT) ? 4 : 1;

            // Save/load the control points to XML
            for (int cp_i = 0; cp_i < cp_group_size; ++cp_i) {
                if (F.xml_save_hmat) {
                    if (calXML.saveControlPoints(I.projector, CAL_MODE, cp_i, CP_GRID_ARR[cp_i]) < 0)
                        throw std::runtime_error("[appMainLoop] Error returned from: calXML.saveControlPoints");
                }
                if (F.xml_load_hmat) {
                    if (calXML.loadControlPoints(I.projector, CAL_MODE, cp_i, CP_GRID_ARR[cp_i]) < 0)
                        throw std::runtime_error("[appMainLoop] Error returned from: calXML.saveControlPoints");
                }
            }

            // Flash the background to indicate the file was loaded/saved
            projCtx.flashBackgroundColor(cv::Scalar(0.0f, 0.25f, 0.0f), 10);
        }

        // Update the window monitor and mode
        if (F.change_window_mode)
            if (projCtx.changeWindowDisplayMode(I.monitor, F.fullscreen_mode) < 0)
                throw std::runtime_error("[appMainLoop] Error returned from: changeWindowDisplayMode");

        // Initialize/reinitialize control point coordinate dataset
        if (F.init_control_points)
            initVertexCoordinates(CAL_MODE, CP_GRID_ARR, WALL_GRID_ARR_DEFAULT);

        // Update homography matrices array
        if (F.update_homographys || F.init_control_points) {
            // Update wall homographys
            if (CAL_MODE == WALLS_LEFT || CAL_MODE == WALLS_MIDDLE || CAL_MODE == WALLS_RIGHT)
                if (updateWallHomographys(CAL_MODE, CP_GRID_ARR, WALL_GRID_ARR_DEFAULT, HMAT_ARR) < 0)
                    throw std::runtime_error("[appMainLoop] Error returned from: updateWallHomographys");
            // Update floor homography
            else if (CAL_MODE == FLOOR)
                if (updateFloorHomography(CP_GRID_ARR[0], HMAT_ARR[CAL_MODE][0][0]) < 0)
                    throw std::runtime_error("[appMainLoop] Error returned from: updateFloorHomography");
        }

        // Update the caliration and monitor mode image
        if (F.update_mode_img) {
            // Update wall textures
            if (CAL_MODE == WALLS_LEFT || CAL_MODE == WALLS_MIDDLE || CAL_MODE == WALLS_RIGHT)
                if (updateModeImage(calibTestWallMats[I.wall_image], calibMonWallMats[I.monitor], calibModeMats[CAL_MODE], modeMat) < 0)
                    throw std::runtime_error("[appMainLoop] Error returned from: updateModeImage for wall image");
            // Update floor texture
            else if (CAL_MODE == FLOOR)
                if (updateModeImage(calibTestFloorMats[I.floor_image], calibMonFloorMats[I.monitor], calibModeMats[CAL_MODE], modeMat) < 0)
                    throw std::runtime_error("[appMainLoop] Error returned from: updateModeImage for floor image");
        }

        // Update image texture
        if (F.update_textures || F.update_homographys || F.init_control_points) {
            // Update wall textures
            if (CAL_MODE == WALLS_LEFT || CAL_MODE == WALLS_MIDDLE || CAL_MODE == WALLS_RIGHT)
                if (updateTexture(calibTestWallMats[I.wall_image], modeMat, CAL_MODE, HMAT_ARR, projCtx) < 0)
                    throw std::runtime_error("[appMainLoop] Error returned from: updateTexture for wall images");
            // Update floor texture
            else if (CAL_MODE == FLOOR)
                if (updateTexture(calibTestFloorMats[I.floor_image], modeMat, CAL_MODE, HMAT_ARR, projCtx) < 0)
                    throw std::runtime_error("[appMainLoop] Error returned from: updateTexture for floor images");
        }

        // Reset keybinding flags
        F.xml_load_hmat = false;
        F.xml_save_hmat = false;
        F.change_window_mode = false;
        F.init_control_points = false;
        F.update_textures = false;
        F.update_homographys = false;
        F.update_mode_img = false;

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

void appCleanup() {
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "projection_calibration", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    try {
        appLoadAssets();
        appInitOpenGL();
        appInitFileXML();
        appMainLoop();
    }
    catch (const std::exception &e) {
        ROS_ERROR("!!EXCEPTION CAUGHT!!: %s", e.what());
        void appCleanup();
        return -1;
    }
    return 0;
}