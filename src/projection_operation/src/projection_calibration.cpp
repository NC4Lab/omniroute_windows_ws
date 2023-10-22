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
        else if (key == GLFW_KEY_1 && glfwWrapper.nMonitors > 1)
        {
            winMonInd = 1;
        }
        else if (key == GLFW_KEY_2 && glfwWrapper.nMonitors > 2)
        {
            winMonInd = 2;
        }
        else if (key == GLFW_KEY_3 && glfwWrapper.nMonitors > 3)
        {
            winMonInd = 3;
        }
        else if (key == GLFW_KEY_4 && glfwWrapper.nMonitors > 4)
        {
            winMonInd = 4;
        }
        else if (key == GLFW_KEY_5 && glfwWrapper.nMonitors > 5)
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
            saveCoordinatesXML(H, calParam, file_path);
        }

        // Load coordinates from XML
        else if (key == GLFW_KEY_L)
        {
            // Get the path to the config directory and format the load file name
            std::string file_path = formatCoordinatesFilePathXML(winMonInd, calModeInd, CONFIG_DIR_PATH);

            // Load the coordinates from the XML file
            loadCoordinatesXML(H, calParam, file_path);
        }

        // ---------- Control Point Reset [R] ----------

        else if (key == GLFW_KEY_R)
        {
            updateCalParams(calParam, calModeInd);
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
            calParamMode = "position";
            imgParamInd = 0;
        }

        // Wall dimension calibration
        else if (key == GLFW_KEY_D)
        {
            calParamMode = "dimension";
            imgParamInd = 1;
        }

        // Wall shear calibration
        else if (key == GLFW_KEY_S)
        {
            calParamMode = "shear";
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
                updateCalParams(calParam, calModeInd);
            }
        }

        // ---------- Image change [ALT + [LEFT, RIGHT]] ----------

        else if (mods & GLFW_MOD_ALT)
        {
            // Listen for arrow key input to switch through images
            if (key == GLFW_KEY_LEFT)
            {
                imgWallInd = (imgWallInd > 0) ? imgWallInd - 1 : (int)nWallImg - 1;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                imgWallInd = (imgWallInd < nWallImg - 1) ? imgWallInd + 1 : 0;
            }
        }

        // ---------- Control point adjustments [SHIFT or no modifier] ----------
        else
        {
            // ---------- Control point position change [LEFT, RIGHT, UP, DOWN] ----------
            if (calParamMode == "position")
            {
                // Set the position increment based on whether the shift key is pressed
                float pos_inc = (mods & GLFW_MOD_SHIFT) ? 0.01f : 0.0005f;

                // Listen for arrow key input to move selected control point
                if (key == GLFW_KEY_LEFT)
                {
                    calParam[cpSelectedInd][0] -= pos_inc; // Move left
                }
                else if (key == GLFW_KEY_RIGHT)
                {
                    calParam[cpSelectedInd][0] += pos_inc; // Move right
                }
                else if (key == GLFW_KEY_UP)
                {
                    calParam[cpSelectedInd][1] += pos_inc; // Move up
                }
                else if (key == GLFW_KEY_DOWN)
                {
                    calParam[cpSelectedInd][1] -= pos_inc; // Move down
                }
            }

            // ---------- Wall dimension calibration change [LEFT, RIGHT, UP, DOWN] ----------
            if (calParamMode == "dimension")
            {
                // Set the width and height dimension increment based on whether the shift key is pressed
                float wd_inc = (mods & GLFW_MOD_SHIFT) ? 0.001f : 0.0001f;
                float ht_inc = (mods & GLFW_MOD_SHIFT) ? 0.05f : 0.005f;

                // Listen for arrow key input to adjust dimension/height
                if (key == GLFW_KEY_LEFT)
                {
                    calParam[cpSelectedInd][2] -= wd_inc; // Decrease width
                }
                else if (key == GLFW_KEY_RIGHT)
                {
                    calParam[cpSelectedInd][2] += wd_inc; // Increase width
                }
                else if (key == GLFW_KEY_UP)
                {
                    calParam[cpSelectedInd][3] += ht_inc; // Increase height
                }
                else if (key == GLFW_KEY_DOWN)
                {
                    calParam[cpSelectedInd][3] -= ht_inc; // Decrease height
                }
            }

            // ---------- Wall shear calibration change [LEFT, RIGHT] ----------
            if (calParamMode == "shear")
            {
                // Set the shear increment based on whether the shift key is pressed
                float shr_inc = (mods & GLFW_MOD_SHIFT) ? 0.01f : 0.001f;

                // Listen for arrow key input to adjust shear
                if (key == GLFW_KEY_LEFT)
                {
                    calParam[cpSelectedInd][4] -= shr_inc; // Skew left
                }
                else if (key == GLFW_KEY_RIGHT)
                {
                    calParam[cpSelectedInd][4] += shr_inc; // Skew right
                }
            }
        }
    }

    // _______________ Update _______________

    // Recompute homography matrix
    computeHomography(H, calParam);

    // Update the window monitor and mode
    updateWindowMonMode(glfwWrapper, 0, winMonInd, isFullScreen);
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

    // // TEMP
    // glColor3f(0.5f, 0.5f, 0.5f);

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

int drawRectImage(std::vector<cv::Point2f> rect_vertices_vec)
{

    // Start drawing a quadrilateral
    glBegin(GL_QUADS);

    // Set the color to white (for texture mapping)
    /// @note: this is nececary when drawing the control points
    glColor3f(1.0f, 1.0f, 1.0f);

    // Set texture and vertex coordinates for each corner

    // Bottom-left corner
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(rect_vertices_vec[0].x, rect_vertices_vec[0].y);

    // Bottom-right corner
    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(rect_vertices_vec[1].x, rect_vertices_vec[1].y);

    // Top-right corner
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(rect_vertices_vec[2].x, rect_vertices_vec[2].y);

    // Top-left corner
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(rect_vertices_vec[3].x, rect_vertices_vec[3].y);

    // End drawing
    glEnd();

    // Check and return GL status
    return checkErrorGL(__LINE__, __FILE__);
}

int drawWalls(
    cv::Mat &ref_H,
    float cal_param_arr[4][5],
    GLuint fbo_texture_id,
    ILuint img_base_id,
    ILuint img_mon_id,
    ILuint img_param_id,
    ILuint img_cal_id)
{
    // Enable OpenGL texture mapping
    glEnable(GL_TEXTURE_2D);

    // Inside your drawWalls() function
    float corner_spacings_x[2][2], corner_spacings_y[2][2];
    calculateCornerSpacing(cal_param_arr, corner_spacings_x, corner_spacings_y, MAZE_SIZE);

    // // TEMP
    // ROS_INFO("cs_x[0][0][%0.2f] cs_x[0][1][%0.2f] cs_x[1][0][%0.2f] cs_x[1][1][%0.2f]    cs_y[0][0][% 0.2f] cs_y[0][1][% 0.2f] cs_y[1][0][% 0.2f] cs_y[1][1][% 0.2f]",
    //          corner_spacings_x[0][0], corner_spacings_x[0][1], corner_spacings_x[1][0], corner_spacings_x[1][1],
    //          corner_spacings_y[0][0], corner_spacings_y[0][1], corner_spacings_y[1][0], corner_spacings_y[1][1]);

    // Iterate through the maze grid rows
    for (float wall_i = 0; wall_i < MAZE_SIZE; wall_i++)
    {
        // Iterate through each cell/column in the maze row
        for (float wall_j = 0; wall_j < MAZE_SIZE; wall_j++)
        {
            // Create merged image for active control point wall
            if (
                (cpSelectedInd == 0 && wall_i == 0 && wall_j == MAZE_SIZE - 1) ||
                (cpSelectedInd == 1 && wall_i == MAZE_SIZE - 1 && wall_j == MAZE_SIZE - 1) ||
                (cpSelectedInd == 2 && wall_i == MAZE_SIZE - 1 && wall_j == 0) ||
                (cpSelectedInd == 3 && wall_i == 0 && wall_j == 0))
            {
                ILuint merge_images_1;
                ILuint merge_images_2;
                ILuint merge_images_3;

                // Merge test pattern and active monitor image
                if (mergeImages(img_base_id, img_mon_id, merge_images_1) != 0)
                    return -1;

                // Merge previous image and active cp parameter image
                if (mergeImages(merge_images_1, img_param_id, merge_images_2) != 0)
                    return -1;

                // Merge previous image and active calibration image
                if (mergeImages(merge_images_2, img_cal_id, merge_images_3) != 0)
                    return -1;

                ilBindImage(merge_images_3);
            }
            else
            {
                ilBindImage(img_base_id); // show test pattern
            }
            if (checkErrorDevIL(__LINE__, __FILE__) != 0)
                return -1;

            // Calculate width, height and shear for the current wall
            float width_val = calculateInterpolatedValue(cal_param_arr, 2, wall_i, wall_j, MAZE_SIZE);
            float height_val = calculateInterpolatedValue(cal_param_arr, 3, wall_i, wall_j, MAZE_SIZE);
            float shear_val = calculateInterpolatedValue(cal_param_arr, 4, wall_i, wall_j, MAZE_SIZE);

            // Create wall vertices
            std::vector<cv::Point2f> rect_vertices_vec = computeRectVertices(0.0f, 0.0f, width_val, height_val, shear_val);

            // Calculate the interpolated wall spacings for this grid cell
            float wall_space_x = calculateInterpolatedWallSpacing(corner_spacings_x, wall_i, wall_j, MAZE_SIZE);
            float wall_space_y = calculateInterpolatedWallSpacing(corner_spacings_y, wall_i, wall_j, MAZE_SIZE);

            // Calculate the wall offset
            float x_offset = wall_i * wall_space_x;
            float y_offset = wall_j * wall_space_y;

            // Apply perspective warping to vertices
            std::vector<cv::Point2f> rect_vertices_warped = computePerspectiveWarp(rect_vertices_vec, ref_H, x_offset, y_offset);

            // Set texture image
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
                         ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB,
                         GL_UNSIGNED_BYTE, ilGetData());

            // Bind texture to framebuffer object
            glBindTexture(GL_TEXTURE_2D, fbo_texture_id);

            // Draw the wall
            if (drawRectImage(rect_vertices_warped) != 0)
                return -1;
        }
    }

    // Disable OpenGL texture mapping
    glDisable(GL_TEXTURE_2D);

    // Return GL status
    return checkErrorGL(__LINE__, __FILE__);
}

int updateWindowMonMode(GLFWWrapper &glfw_wrapper, int win_ind, int mon_ind, bool is_fullscreen)
{
    static int imp_mon_ind_last = mon_ind;
    static bool is_fullscreen_last = !is_fullscreen;

    // Check if monitor or fullscreen mode has changed
    if (imp_mon_ind_last == mon_ind && is_fullscreen_last == is_fullscreen)
    {
        return 0;
    }

    // Get pointers to the GLFWwindow and GLFWmonitor
    GLFWwindow *p_window_id = glfwWrapper.getWindowID();
    GLFWmonitor **pp_ref_monitor_id = glfw_wrapper.getMonitorIDVec();

    // Get GLFWmonitor for active monitor
    GLFWmonitor *p_monitor_id = pp_ref_monitor_id[mon_ind];

    // Update window size and position
    if (p_monitor_id)
    {
        // Get the video mode of the selected monitor
        const GLFWvidmode *mode = glfwGetVideoMode(p_monitor_id);
        if (!mode)
        {
            ROS_ERROR("[WIN MODE] Failed to Get Video Mode: Monitor[%d]", mon_ind);
            return -1;
        }

        // Set the window to full-screen mode on the current monitor
        glfwSetWindowMonitor(p_window_id, p_monitor_id, 0, 0, mode->width, mode->height, mode->refreshRate);
        if (!p_monitor_id)
        {
            ROS_ERROR("[WIN MODE] Invalid Monitor Pointer: Monitor[%d]", mon_ind);
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
                ROS_WARN("[WIN MODE] Invalid Monitor Position: Monitor[%d] X[%d] Y[%d]", mon_ind, monitor_x, monitor_y);
                return 0;
            }

            // Set the window to windowed mode and position it on the current monitor
            glfwSetWindowMonitor(p_window_id, NULL, monitor_x + 100, monitor_y + 100, (int)(500.0f * PROJ_WIN_ASPECT_RATIO), 500, 0);
        }

        // Update window title
        std::string new_title = "Window[" + std::to_string(win_ind) + "] Monitor[" + std::to_string(mon_ind) + "]";
        glfwSetWindowTitle(p_window_id, new_title.c_str());

        ROS_INFO("[WIN MODE] Move Window: Monitor[%d] Format[%s]", mon_ind, is_fullscreen ? "fullscreen" : "windowed");
    }
    else
    {
        ROS_WARN("[WIN MODE] Failed Move Window: Monitor[%d] Format[%s]", mon_ind, is_fullscreen ? "fullscreen" : "windowed");
        return 0;
    }

    // Update last monitor and fullscreen mode
    imp_mon_ind_last = mon_ind;
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
    updateCalParams(calParam, calModeInd);

    // Do initial computations of homography matrix
    computeHomography(H, calParam);

    // --------------- OpenGL SETUP ---------------

    // Check GLFW initialization
    if (!glfwWrapper.initStatus)
    {
        checkErrorGLFW(__LINE__, __FILE__);
        ROS_ERROR("[GLFW] Initialization Failed");
        return -1;
    }

    // Setup callback
    glfwSetErrorCallback(callbackErrorGLFW);

    // Create window
    if (!glfwWrapper.createWindow(PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, "", NULL, NULL))
    {
        glfwTerminate();
        ROS_ERROR("[GLFW] Create Window Failed");
        return -1;
    }

    // Get pointers to the GLFWwindow and GLFWmonitor
    GLFWwindow *p_window_id = glfwWrapper.getWindowID();
    GLFWmonitor **pp_monitorIDVec = glfwWrapper.getMonitorIDVec();
    ROS_INFO("[GLFW] Found %d monitors", glfwWrapper.nMonitors);

    // Set OpenGL context and callbacks
    glfwMakeContextCurrent(p_window_id);
    gladLoadGL();
    glfwSetKeyCallback(p_window_id, callbackKeyBinding);
    glfwSetFramebufferSizeCallback(p_window_id, callbackFrameBufferSizeGLFW);

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
    updateWindowMonMode(glfwWrapper, 0, winMonInd, isFullScreen);

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
    if (loadImgTextures(imgWallIDVec, imgWallPathVec) != 0)
    {
        ROS_ERROR("[DevIL] Failed to load wall images");
        return -1;
    }
    if (loadImgTextures(imgMonIDVec, imgMonPathVec) != 0)
    {
        ROS_ERROR("[DevIL] Failed to load monitor images");
        return -1;
    }
    if (loadImgTextures(imgParamIDVec, imgParamPathVec) != 0)
    {
        ROS_ERROR("[DevIL] Failed to load parameter images");
        return -1;
    }
    if (loadImgTextures(imgCalIDVec, imgCalPathVec) != 0)
    {
        ROS_ERROR("[DevIL] Failed to load calibration images");
        return -1;
    }

    // _______________ MAIN LOOP _______________

    while (!glfwWindowShouldClose(p_window_id) && ros::ok())
    {

        // Clear back buffer for new frame
        glClear(GL_COLOR_BUFFER_BIT);
        if (checkErrorGL(__LINE__, __FILE__))
            break;

        // Draw/update wall images
        drawWalls(H, calParam, fbo_texture_id, imgWallIDVec[imgWallInd], imgMonIDVec[winMonInd], imgParamIDVec[imgParamInd], imgCalIDVec[calModeInd]);
        if (checkErrorGL(__LINE__, __FILE__))
            break;

        // Draw/update control points
        for (int i = 0; i < 4; i++)
        {
            // Get control point color based on cp selection and mode
            std::vector<float> cp_col =
                (cpSelectedInd != i) ? cpInactiveRGBVec : (cpSelectedInd == 1 && (calParamMode != "position")) ? cpDisabledRGBVec
                                                                                                               : cpActiveRGBVec;

            // Draw the control point
            drawControlPoint(calParam[i][0], calParam[i][1], CP_RADIUS_NDC, cp_col);
            if (checkErrorGL(__LINE__, __FILE__))
                break;
        }

        // Swap buffers and poll events
        glfwSwapBuffers(p_window_id);
        checkErrorGLFW(__LINE__, __FILE__);
        if (checkErrorGL(__LINE__, __FILE__))
            break;

        // Poll events
        glfwPollEvents();
        checkErrorGLFW(__LINE__, __FILE__);

        // Exit condition
        if (glfwGetKey(p_window_id, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(p_window_id))
            break;
    }

    // _______________ CLEANUP _______________
    ROS_INFO("SHUTTING DOWN");

    // Check which condition caused the loop to exit
    if (!ros::ok())
    {
        ROS_INFO("[LOOP TERMINATION] ROS Node is no Longer in a Good State");
    }
    else if (glfwWindowShouldClose(p_window_id))
    {
        ROS_INFO("[LOOP TERMINATION] GLFW Window Should Close");
    }
    else if (glfwGetKey(p_window_id, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    {
        ROS_INFO("[LOOP TERMINATION] Escape Key was Pressed");
    }

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
