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
            isFullScreen = !isFullScreen;
        }

        // Move the window to another monitor
        else if (key == GLFW_KEY_0)
        {
            winMonInd = 0;
        }
        else if (key == GLFW_KEY_1 && nMonitors > 1)
        {
            winMonInd = 1;
        }
        else if (key == GLFW_KEY_2 && nMonitors > 2)
        {
            winMonInd = 2;
        }
        else if (key == GLFW_KEY_3 && nMonitors > 3)
        {
            winMonInd = 3;
        }
        else if (key == GLFW_KEY_4 && nMonitors > 4)
        {
            winMonInd = 4;
        }
        else if (key == GLFW_KEY_5 && nMonitors > 5)
        {
            winMonInd = 5;
        }

        // ---------- XML Handling [ENTER, L] ----------

        // Save coordinates to XML
        else if (key == GLFW_KEY_ENTER)
        {
            // Get the path to the config directory and format the save file name
            std::string file_path = formatCoordinatesFilePathXML(winMonInd, calModeInd, CONFIG_DIR_PATH);

            // Save the coordinates to the XML file
            saveCoordinatesXML(homMat, contPointParams, file_path);
        }

        // Load coordinates from XML
        else if (key == GLFW_KEY_L)
        {
            // Get the path to the config directory and format the load file name
            std::string file_path = formatCoordinatesFilePathXML(winMonInd, calModeInd, CONFIG_DIR_PATH);

            // Load the coordinates from the XML file
            loadCoordinatesXML(homMat, contPointParams, file_path);
        }

        // ---------- Control Point Reset [R] ----------

        else if (key == GLFW_KEY_R)
        {
            updateCalParams(contPointParams, calModeInd);
        }

        // ---------- Target selector keys [F1-F4] ----------

        // Top-left control point
        else if (key == GLFW_KEY_F1)
        {
            cpSelectedInd = 0;
        }

        // Top-right control point
        else if (key == GLFW_KEY_F2)
        {
            cpSelectedInd = 1;
        }

        // Bottom-right control point
        else if (key == GLFW_KEY_F3)
        {
            cpSelectedInd = 2;
        }

        // Bottom-left control point
        else if (key == GLFW_KEY_F4)
        {
            cpSelectedInd = 3;
        }

        // ---------- Change calibration point parameter keys [A, D, S] ----------

        // Control point position
        else if (key == GLFW_KEY_A)
        {
            calModeStr = "position";
            imgParamInd = 0;
        }

        // Wall dimension calibration
        else if (key == GLFW_KEY_D)
        {
            calModeStr = "dimension";
            imgParamInd = 1;
        }

        // Wall shear calibration
        else if (key == GLFW_KEY_S)
        {
            calModeStr = "shear";
            imgParamInd = 2;
        }
    }

    // _______________ ANY KEY PRESS OR REPEAT ACTION _______________
    else if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {

        // ---------- Calibration mode [CTRL + [LEFT, RIGHT]] ----------

        if (mods & GLFW_MOD_CONTROL)
        {
            // Listen for arrow key input to switch through calibration modes
            if (key == GLFW_KEY_LEFT)
            {
                calModeInd = (calModeInd > 0) ? calModeInd - 1 : (int)nCalModes - 1;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                calModeInd = (calModeInd < nCalModes - 1) ? calModeInd + 1 : 0;
            }
            // Reset a subset of control point parameters when switching calibration modes
            if (key == GLFW_KEY_LEFT || key == GLFW_KEY_RIGHT)
            {
                updateCalParams(contPointParams, calModeInd);
            }
        }

        // ---------- Image change [ALT + [LEFT, RIGHT]] ----------

        else if (mods & GLFW_MOD_ALT)
        {
            // Listen for arrow key input to switch through images
            if (key == GLFW_KEY_LEFT)
            {
                imgWallInd = (imgWallInd > 0) ? imgWallInd - 1 : (int)imgWallPathVec.size() - 1;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                imgWallInd = (imgWallInd < imgWallPathVec.size() - 1) ? imgWallInd + 1 : 0;
            }
        }

        // ---------- Control point adjustments [SHIFT or no modifier] ----------
        else
        {
            // ---------- Control point position change [LEFT, RIGHT, UP, DOWN] ----------
            if (calModeStr == "position")
            {
                // Set the position increment based on whether the shift key is pressed
                float pos_inc = (mods & GLFW_MOD_SHIFT) ? 0.01f : 0.0005f;

                // Listen for arrow key input to move selected control point
                if (key == GLFW_KEY_LEFT)
                {
                    contPointParams[cpSelectedInd][0] -= pos_inc; // Move left
                }
                else if (key == GLFW_KEY_RIGHT)
                {
                    contPointParams[cpSelectedInd][0] += pos_inc; // Move right
                }
                else if (key == GLFW_KEY_UP)
                {
                    contPointParams[cpSelectedInd][1] += pos_inc; // Move up
                }
                else if (key == GLFW_KEY_DOWN)
                {
                    contPointParams[cpSelectedInd][1] -= pos_inc; // Move down
                }
            }

            // ---------- Wall dimension calibration change [LEFT, RIGHT, UP, DOWN] ----------
            if (calModeStr == "dimension")
            {
                // Set the width and height dimension increment based on whether the shift key is pressed
                float wd_inc = (mods & GLFW_MOD_SHIFT) ? 0.001f : 0.0001f;
                float ht_inc = (mods & GLFW_MOD_SHIFT) ? 0.05f : 0.005f;

                // Listen for arrow key input to adjust dimension/height
                if (key == GLFW_KEY_LEFT)
                {
                    contPointParams[cpSelectedInd][2] -= wd_inc; // Decrease width
                }
                else if (key == GLFW_KEY_RIGHT)
                {
                    contPointParams[cpSelectedInd][2] += wd_inc; // Increase width
                }
                else if (key == GLFW_KEY_UP)
                {
                    contPointParams[cpSelectedInd][3] += ht_inc; // Increase height
                }
                else if (key == GLFW_KEY_DOWN)
                {
                    contPointParams[cpSelectedInd][3] -= ht_inc; // Decrease height
                }
            }

            // ---------- Wall shear calibration change [LEFT, RIGHT] ----------
            if (calModeStr == "shear")
            {
                // Set the shear increment based on whether the shift key is pressed
                float shr_inc = (mods & GLFW_MOD_SHIFT) ? 0.01f : 0.001f;

                // Listen for arrow key input to adjust shear
                if (key == GLFW_KEY_LEFT)
                {
                    contPointParams[cpSelectedInd][4] -= shr_inc; // Skew left
                }
                else if (key == GLFW_KEY_RIGHT)
                {
                    contPointParams[cpSelectedInd][4] += shr_inc; // Skew right
                }
            }
        }
    }

    // _______________ Update _______________

    // Recompute homography matrix
    computeHomography(homMat, contPointParams);

    // Update the window monitor and mode
    updateWindowMonMode(p_windowID, 0, pp_monitorIDVec, winMonInd, isFullScreen);
}

void callbackFrameBufferSizeGLFW(GLFWwindow *window, int width, int height)
{
    glViewport(0, 0, width, height);
    checkErrorGL(__LINE__, __FILE__);
}

static void callbackErrorGLFW(int error, const char *description)
{
    ROS_ERROR("[GLFW] Error Callback: Error[%d] Description[%s]", error, description);
}

int checkErrorGL(int line, const char *file_str, const char *msg_str)
{
    GLenum gl_err;
    while ((gl_err = glGetError()) != GL_NO_ERROR)
    {
        if (msg_str)
            ROS_INFO("[OpenGL] Error Flagged: Message[%s] Error Number[%u] File[%s] Line[%d]", msg_str, gl_err, file_str, line);
        else
            ROS_INFO("[OpenGL] Error Flagged: Error Number[%u] File[%s] Line[%d]", gl_err, file_str, line);
        return -1;
    }
    return 0;
}

int checkErrorGLFW(int line, const char *file_str, const char *msg_str)
{
    const char *description;
    int glfw_err = glfwGetError(&description);
    if (glfw_err != GLFW_NO_ERROR)
    {
        if (msg_str)
            ROS_ERROR("[GLFW] Error Flagged: Message[%s] Description[%s] File[%s] Line[%d]", msg_str, description, file_str, line);
        else
            ROS_ERROR("[GLFW] Error Flagged: Description[%s] File[%s] Line[%d]", description, file_str, line);
        return -1;
    }
    return 0;
}

int drawControlPoint(float x, float y, float radius, std::vector<float> rgb_vec)
{
    const int segments = 100; // Number of segments to approximate a circle

    // Begin drawing a filled circle
    glBegin(GL_TRIANGLE_FAN);

    // Set the color to green
    glColor3f(rgb_vec[0], rgb_vec[1], rgb_vec[2]);

    // Center of the circle
    glVertex2f(x, y);

    // Calculate and draw the vertices of the circle
    for (int i = 0; i <= segments; i++)
    {
        float theta = 2.0f * 3.1415926f * float(i) / float(segments);
        float px = x + radius * cosf(theta);
        float py = y + (radius * PROJ_WIN_ASPECT_RATIO) * sinf(theta);
        glVertex2f(px, py);
    }

    // End drawing
    glEnd();

    // Return GL status
    return checkErrorGL(__LINE__, __FILE__);
}

int drawQuadImage(std::vector<cv::Point2f> quad_vertices_vec)
{

    // Start drawing a quadrilateral
    glBegin(GL_QUADS);

    // Set the color to white (for texture mapping)
    /// @note: this is nececary when drawing the control points
    glColor3f(1.0f, 1.0f, 1.0f);

    // Set texture and vertex coordinates for each corner

    // Bottom-left corner
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(quad_vertices_vec[0].x, quad_vertices_vec[0].y);

    // Bottom-right corner
    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(quad_vertices_vec[1].x, quad_vertices_vec[1].y);

    // Top-right corner
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(quad_vertices_vec[2].x, quad_vertices_vec[2].y);

    // Top-left corner
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(quad_vertices_vec[3].x, quad_vertices_vec[3].y);

    // End drawing
    glEnd();

    // Check and return GL status
    return checkErrorGL(__LINE__, __FILE__);
}

int drawWalls(
    cv::Mat &r_hom_mat,
    float cont_point_params[4][5],
    GLuint fbo_texture_id,
    ILuint img_wall_id,
    ILuint img_mode_mon_id,
    ILuint img_mode_param_id,
    ILuint img_mode_cal_id)
{
    // Enable OpenGL texture mapping
    glEnable(GL_TEXTURE_2D);

    // Calculate the a 2x2 array of control point distances
    float control_point_distances[4][5];
    calculateControlPointDistances(cont_point_params, control_point_distances);

    // // TEMP
    // ROS_INFO("cs_x[0][0][%0.2f] cs_x[0][1][%0.2f] cs_x[1][0][%0.2f] cs_x[1][1][%0.2f]    cs_y[0][0][% 0.2f] cs_y[0][1][% 0.2f] cs_y[1][0][% 0.2f] cs_y[1][1][% 0.2f]",
    //          corner_space_arr_x[0][0], corner_space_arr_x[0][1], corner_space_arr_x[1][0], corner_space_arr_x[1][1],
    //          corner_space_arr_y[0][0], corner_space_arr_y[0][1], corner_space_arr_y[1][0], corner_space_arr_y[1][1]);

    // TEMP
    ROS_INFO("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
 
    /**
     * @note The correspondence between grid indeces and control points is as follows:
     * 
     * 1: top-left:
     *      control point: 0
     *      grid index: [0, last]
     * 2: top-right:
     *      control point: 1
     *      grid index: [0, 0]
     * 3: bottom-right:
     *      control point: 2
     *      grid index: [last, 0]
     * 4: bottom-left:
     *      control point: 3
     *      grid index: [last, last]
     */ 

    // Iterate through the maze grid rows
    for (float wall_row_i = 0; wall_row_i < MAZE_SIZE; wall_row_i++)
    {
        // Iterate through each column in the maze row
        for (float wall_col_i = 0; wall_col_i < MAZE_SIZE; wall_col_i++)
        {
            //  Create merged image for the wall corresponding to the selected control point
            if (
                (cpSelectedInd == 0 && wall_row_i == 0 && wall_col_i == MAZE_SIZE - 1) ||
                (cpSelectedInd == 1 && wall_row_i == MAZE_SIZE - 1 && wall_col_i == MAZE_SIZE - 1) ||
                (cpSelectedInd == 2 && wall_row_i == MAZE_SIZE - 1 && wall_col_i == 0) ||
                (cpSelectedInd == 3 && wall_row_i == 0 && wall_col_i == 0))
            {
                // if (
                //     (wall_row_i == 0 && wall_col_i == MAZE_SIZE - 1))
                // {
                ILuint img_merge1_id;
                ILuint img_merge2_id;
                ILuint img_merge3_id;

                // Merge test pattern and active monitor image
                if (mergeImages(img_wall_id, img_mode_mon_id, img_merge1_id) != 0)
                    return -1;

                // Merge previous image and active cp parameter image
                if (mergeImages(img_merge1_id, img_mode_param_id, img_merge2_id) != 0)
                    return -1;

                // Merge previous image and active calibration image
                if (mergeImages(img_merge2_id, img_mode_cal_id, img_merge3_id) != 0)
                    return -1;

                ilBindImage(img_merge3_id);
            }
            else
            {
                ilBindImage(img_wall_id); // show test pattern
            }
            if (checkErrorDevIL(__LINE__, __FILE__) != 0)
                return -1;

            // Calculate width, height and shear for the current wall
            float width_val = interpolateWallParam(cont_point_params, 2, wall_row_i, wall_col_i, MAZE_SIZE, true);
            float height_val = interpolateWallParam(cont_point_params, 3, wall_row_i, wall_col_i, MAZE_SIZE, true);
            float shear_val = interpolateWallParam(cont_point_params, 4, wall_row_i, wall_col_i, MAZE_SIZE, true);

            // Create wall vertices
            std::vector<cv::Point2f> quad_vertices_vec = computeQuadVertices(0.0f, 0.0f, width_val, height_val, shear_val);

            // // Calculate the interpolated wall spacings for this grid cell
            // float x_offset = interpolateWallParam(control_point_distances, 0, wall_row_i, wall_col_i, MAZE_SIZE);
            // float y_offset = interpolateWallParam(control_point_distances, 1, wall_row_i, wall_col_i, MAZE_SIZE);

            // Calculate the interpolated wall spacings for this grid cell
            float x_offset = interpolateWallParam(cont_point_params, 0, wall_row_i, wall_col_i, MAZE_SIZE, false);
            float y_offset = interpolateWallParam(cont_point_params, 1, wall_row_i, wall_col_i, MAZE_SIZE, false);

            // float x_offset;
            // float y_offset;
            // interpOffsetTEMP(cont_point_params, x_offset, y_offset, wall_row_i, wall_col_i);

            // TEMP
            ROS_INFO("Offset: i[%0.2f] j[%0.2f] x[%0.2f] y[%0.2f]", wall_row_i, wall_col_i, x_offset, y_offset);

            // Apply perspective warping to vertices
            std::vector<cv::Point2f> quad_vertices_warped = computePerspectiveWarp(quad_vertices_vec, r_hom_mat, x_offset, y_offset);

            // Set texture image
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
                         ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB,
                         GL_UNSIGNED_BYTE, ilGetData());

            // Bind texture to framebuffer object
            glBindTexture(GL_TEXTURE_2D, fbo_texture_id);

            // Draw the wall
            if (drawQuadImage(quad_vertices_warped) != 0)
                return -1;
        }
    }

    // TEMP
    ROS_INFO("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");

    // Disable OpenGL texture mapping
    glDisable(GL_TEXTURE_2D);

    // Return GL status
    return checkErrorGL(__LINE__, __FILE__);
}

int updateWindowMonMode(GLFWwindow *p_window_id, int win_ind, GLFWmonitor **&pp_r_monitor_id, int mon_id_ind, bool is_fullscreen)
{
    static int imp_mon_id_ind_last = mon_id_ind;
    static bool is_fullscreen_last = !is_fullscreen;

    // Check if monitor or fullscreen mode has changed
    if (imp_mon_id_ind_last == mon_id_ind && is_fullscreen_last == is_fullscreen)
    {
        return 0;
    }

    // Get GLFWmonitor for active monitor
    GLFWmonitor *p_monitor_id = pp_r_monitor_id[mon_id_ind];

    // Update window size and position
    if (p_monitor_id)
    {
        // Get the video mode of the selected monitor
        const GLFWvidmode *mode = glfwGetVideoMode(p_monitor_id);
        if (!mode)
        {
            ROS_ERROR("[WIN MODE] Failed to Get Video Mode: Monitor[%d]", mon_id_ind);
            return -1;
        }

        // Set the window to full-screen mode on the current monitor
        glfwSetWindowMonitor(p_window_id, p_monitor_id, 0, 0, mode->width, mode->height, mode->refreshRate);
        if (!p_monitor_id)
        {
            ROS_ERROR("[WIN MODE] Invalid Monitor Pointer: Monitor[%d]", mon_id_ind);
            return -1;
        }

        if (!is_fullscreen)
        {
            // Get the position of the current monitor
            int monitor_x, monitor_y;
            glfwGetMonitorPos(p_monitor_id, &monitor_x, &monitor_y);

            // Validate monitor position
            if (monitor_x < 0 || monitor_y < 0)
            {
                ROS_WARN("[WIN MODE] Invalid Monitor Position: Monitor[%d] X[%d] Y[%d]", mon_id_ind, monitor_x, monitor_y);
                return 0;
            }

            // Set the window to windowed mode and position it on the current monitor
            glfwSetWindowMonitor(p_window_id, NULL, monitor_x + 100, monitor_y + 100, (int)(500.0f * PROJ_WIN_ASPECT_RATIO), 500, 0);
        }

        // Update window title
        std::string new_title = "Window[" + std::to_string(win_ind) + "] Monitor[" + std::to_string(mon_id_ind) + "]";
        glfwSetWindowTitle(p_window_id, new_title.c_str());

        ROS_INFO("[WIN MODE] Move Window: Monitor[%d] Format[%s]", mon_id_ind, is_fullscreen ? "fullscreen" : "windowed");
    }
    else
    {
        ROS_WARN("[WIN MODE] Failed Move Window: Monitor[%d] Format[%s]", mon_id_ind, is_fullscreen ? "fullscreen" : "windowed");
        return 0;
    }

    // Update last monitor and fullscreen mode
    imp_mon_id_ind_last = mon_id_ind;
    is_fullscreen_last = is_fullscreen;

    return 0;
}

int main(int argc, char **argv)
{
    //  _______________ SETUP _______________

    // ROS Initialization
    ros::init(argc, argv, "projection_calibration", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    ROS_INFO("RUNNING MAIN");

    // Log paths for debugging
    ROS_INFO("[SETUP] Config XML Path: %s", CONFIG_DIR_PATH.c_str());
    ROS_INFO("[SETUP] Display: Width=%d Height=%d AR=%0.2f", PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, PROJ_WIN_ASPECT_RATIO);
    ROS_INFO("[SETUP] Wall (Pxl): Width=%d Space=%d", WALL_WIDTH_PXL, WALL_HEIGHT_PXL);

    // Initialize control point parameters
    updateCalParams(contPointParams, calModeInd);

    // Do initial computations of homography matrix
    computeHomography(homMat, contPointParams);

    // --------------- OpenGL SETUP ---------------

    // Initialize GLFW
    glfwSetErrorCallback(callbackErrorGLFW);
    if (!glfwInit())
    {
        checkErrorGLFW(__LINE__, __FILE__);
        ROS_ERROR("[GLFW] Initialization Failed");
        return -1;
    }

    // Get the list of available monitors and their count
    pp_monitorIDVec = glfwGetMonitors(&nMonitors);
    ROS_INFO("[GLFW] Found %d monitors", nMonitors);

    // Create GLFW window
    p_windowID = glfwCreateWindow(PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, "", NULL, NULL);
    checkErrorGLFW(__LINE__, __FILE__);
    if (!p_windowID)
    {
        glfwTerminate();
        ROS_ERROR("[GLFW] Create Window Failed");
        return -1;
    }

    // Set OpenGL context and callbacks
    glfwMakeContextCurrent(p_windowID);
    gladLoadGL();
    glfwSetKeyCallback(p_windowID, callbackKeyBinding);
    glfwSetFramebufferSizeCallback(p_windowID, callbackFrameBufferSizeGLFW);

    // Initialize FBO and texture
    GLuint fbo_id;
    GLuint fbo_texture_id;

    // Generate and set up the FBO
    glGenFramebuffers(1, &fbo_id);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_id);
    checkErrorGL(__LINE__, __FILE__);

    // Generate and set up the texture
    glGenTextures(1, &fbo_texture_id);
    glBindTexture(GL_TEXTURE_2D, fbo_texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    checkErrorGL(__LINE__, __FILE__);

    // Attach the texture to the FBO
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_texture_id, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    checkErrorGL(__LINE__, __FILE__);

    // Update the window monitor and mode
    updateWindowMonMode(p_windowID, 0, pp_monitorIDVec, winMonInd, isFullScreen);

    // Get OpenGL version
    const GLubyte *opengl_version = glGetString(GL_VERSION);
    ROS_INFO("[OpenGL] Intitalized: Version [%s]", opengl_version);

    // Get GLFW version
    int glfw_major, glfw_minor, glfw_rev;
    glfwGetVersion(&glfw_major, &glfw_minor, &glfw_rev);
    ROS_INFO("[GLFW] Intitalized: Version: %d.%d.%d", glfw_major, glfw_minor, glfw_rev);

    // --------------- DevIL SETUP ---------------

    // Initialize DevIL library
    ilInit();
    if (checkErrorDevIL(__LINE__, __FILE__) != 0)
        return -1;
    ILint version = ilGetInteger(IL_VERSION_NUM);
    ROS_INFO("[DevIL] Intitalized: Version[%d]", version);

    // Load images
    if (loadImgTextures(imgWallPathVec, imgWallIDVec) != 0)
    {
        ROS_ERROR("[DevIL] Failed to load wall images");
        return -1;
    }
    if (loadImgTextures(imgMonPathVec, imgMonIDVec) != 0)
    {
        ROS_ERROR("[DevIL] Failed to load monitor images");
        return -1;
    }
    if (loadImgTextures(imgParamPathVec, imgParamIDVec) != 0)
    {
        ROS_ERROR("[DevIL] Failed to load parameter images");
        return -1;
    }
    if (loadImgTextures(imgCalPathVec, imgCalIDVec) != 0)
    {
        ROS_ERROR("[DevIL] Failed to load calibration images");
        return -1;
    }

    // _______________ MAIN LOOP _______________

    while (!glfwWindowShouldClose(p_windowID) && ros::ok())
    {

        // Clear back buffer for new frame
        glClear(GL_COLOR_BUFFER_BIT);
        if (checkErrorGL(__LINE__, __FILE__))
            break;

        // Draw/update wall images
        drawWalls(homMat, contPointParams, fbo_texture_id, imgWallIDVec[imgWallInd], imgMonIDVec[winMonInd], imgParamIDVec[imgParamInd], imgCalIDVec[calModeInd]);
        if (checkErrorGL(__LINE__, __FILE__))
            break;

        // Draw/update control points
        for (int i = 0; i < 4; i++)
        {
            // Get control point color based on cp selection and mode
            std::vector<float> cp_col =
                (cpSelectedInd != i) ? cpInactiveRGBVec : (cpSelectedInd == 1 && (calModeStr != "position")) ? cpDisabledRGBVec
                                                                                                             : cpActiveRGBVec;

            // Draw the control point
            drawControlPoint(contPointParams[i][0], contPointParams[i][1], CP_RADIUS_NDC, cp_col);
            if (checkErrorGL(__LINE__, __FILE__))
                break;
        }

        // Swap buffers and poll events
        glfwSwapBuffers(p_windowID);
        checkErrorGLFW(__LINE__, __FILE__);
        if (checkErrorGL(__LINE__, __FILE__))
            break;

        // Poll events
        glfwPollEvents();
        checkErrorGLFW(__LINE__, __FILE__);

        // Exit condition
        if (glfwGetKey(p_windowID, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(p_windowID))
            break;
    }

    // _______________ CLEANUP _______________
    ROS_INFO("SHUTTING DOWN");

    // Check which condition caused the loop to exit
    if (!ros::ok())
    {
        ROS_INFO("[LOOP TERMINATION] ROS Node is no Longer in a Good State");
    }
    else if (glfwWindowShouldClose(p_windowID))
    {
        ROS_INFO("[LOOP TERMINATION] GLFW Window Should Close");
    }
    else if (glfwGetKey(p_windowID, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    {
        ROS_INFO("[LOOP TERMINATION] Escape Key was Pressed");
    }

    // Destroy GLFW window and DevIL images
    if (p_windowID)
    {
        glfwDestroyWindow(p_windowID);
        p_windowID = nullptr;
    }
    checkErrorGLFW(__LINE__, __FILE__);
    ROS_INFO("[SHUTDOWN] Detroyd GLFW windows");

    // Delete DevIL images
    deleteImgTextures(imgWallIDVec);
    deleteImgTextures(imgMonIDVec);
    deleteImgTextures(imgParamIDVec);
    deleteImgTextures(imgCalIDVec);
    ROS_INFO("[SHUTDOWN] Deleted DevIL images");

    // Delete FBO and textures
    glDeleteFramebuffers(1, &fbo_id);
    checkErrorGL(__LINE__, __FILE__);

    // Delete FBO and textures
    glDeleteTextures(1, &fbo_texture_id);
    checkErrorGL(__LINE__, __FILE__);
    ROS_INFO("[SHUTDOWN] Deleted FBO and textures");

    // Shutdown DevIL
    ilShutDown();
    checkErrorDevIL(__LINE__, __FILE__);
    ROS_INFO("[SHUTDOWN] Shutdown DevIL");

    // Terminate GLFW
    glfwTerminate();
    checkErrorGLFW(__LINE__, __FILE__);
    ROS_INFO("[SHUTDOWN] Terminated GLFW");

    // Return success
    return 0;
}
