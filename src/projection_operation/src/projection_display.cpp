// ########################################################################################################

// ======================================== projection_display.cpp ========================================

// ########################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_display.h"

// ================================================== FUNCTIONS ==================================================

void callbackKeyBinding(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    bool do_window_update = false;

    // _______________ ANY KEY RELEASE ACTION _______________

    if (action == GLFW_RELEASE)
    {

        // ---------- Monitor handling [F, M] ----------

        // Set/unset all windows to fullscreen [F]
        if (key == GLFW_KEY_F)
        {
            isFullScreen = !isFullScreen;
            do_window_update = true;
        }

        // Move window between projector and default monitor [M]
        if (key == GLFW_KEY_M)
        {
            isWinOnProj = !isWinOnProj;
            do_window_update = true;
        }
    }

    // _______________ ANY KEY PRESS OR REPEAT ACTION _______________
    else if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
    }

    // _______________ Update _______________

    // Update all windows
    if (do_window_update)
    {
        for (int proj_i = 0; proj_i < nProjectors; ++proj_i)
        {
            int mon_id_ind = isWinOnProj ? projMonIndArr[proj_i] : winMonIndDefault; // Show image on default or projector monitor
            updateWindowMonMode(p_windowIDVec[proj_i], proj_i, pp_monitorIDVec, mon_id_ind, isFullScreen);
        }
    }
}

void callbackFrameBufferSizeGLFW(GLFWwindow *window, int width, int height)
{
    glViewport(0, 0, width, height);
    checkErrorGL(__LINE__, __FILE__);
}

static void callbackErrorGLFW(int error, const char *description)
{
    ROS_ERROR("[GLFW] Error Flagged: Error[%d] Description[%s]", error, description);
}

void checkErrorGL(int line, const char *file_str)
{
    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR)
    {
        // Log or print the error code
        ROS_INFO("[OpenGL] Error Flagged: Line[%d] File[%s] Error Number[%s]: ", line, file_str, err);
    }
}

int setupProjGLFW(
    GLFWwindow **pp_window_id,
    int win_ind,
    GLFWmonitor **&pp_r_monitor_id,
    int mon_id_ind,
    const std::string &r_window_name,
    GLuint &r_fbo_id,
    GLuint &r_fbo_texture_id)
{
    // Create GLFW window
    pp_window_id[win_ind] = glfwCreateWindow(PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, r_window_name.c_str(), NULL, NULL);
    if (!pp_window_id[win_ind])
    {
        glfwTerminate();
        ROS_ERROR("[GLFW] Create Window Failed");
        return -1;
    }

    // Set OpenGL context and callbacks
    glfwMakeContextCurrent(pp_window_id[win_ind]);
    gladLoadGL();
    glfwSetKeyCallback(pp_window_id[win_ind], callbackKeyBinding);
    glfwSetFramebufferSizeCallback(pp_window_id[win_ind], callbackFrameBufferSizeGLFW);

    // Generate and set up the FBO
    glGenFramebuffers(1, &r_fbo_id);
    glBindFramebuffer(GL_FRAMEBUFFER, r_fbo_id);

    // Generate and set up the texture
    glGenTextures(1, &r_fbo_texture_id);
    glBindTexture(GL_TEXTURE_2D, r_fbo_texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Attach the texture to the FBO
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, r_fbo_texture_id, 0);

    // Check for FBO completeness
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE)
    {
        ROS_ERROR("[GLFW] Framebuffer is Not Complete");
        return -1;
    }

    // Unbind the FBO
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Set window to wondowed mode on the second monitor
    if (updateWindowMonMode(pp_window_id[win_ind], win_ind, pp_r_monitor_id, mon_id_ind, isFullScreen) != 0)
    {
        ROS_ERROR("[GLFW] Failed to Update Window[%d] Monitor[%d] Mode", win_ind, mon_id_ind);
        return -1;
    }
    else
    {
        ROS_INFO("[GLFW] Setup Window[%d] On Monitor[%d]", win_ind, mon_id_ind);
    }

    // Check for GL errors
    checkErrorGL(__LINE__, __FILE__);

    return 0;
}

void drawRectImage(std::vector<cv::Point2f> rect_vertices_vec)
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

    // Check for GL errors
    checkErrorGL(__LINE__, __FILE__);
}

int drawWalls(
    cv::Mat &r_hom_mat,
    float cont_point_params[4][5],
    int proj_i,
    GLFWwindow *p_window_id,
    GLuint fbo_texture_id,
    std::vector<ILuint> &r_image_id_vec)
{
    // Enable OpenGL texture mapping
    glEnable(GL_TEXTURE_2D);

    // Draw wall images for each calibration mode wall [left, middle, right]
    for (int cal_i = 0; cal_i < 3; cal_i++)
    {
        // Load the image transform coordinates from the XML file
        std::string file_path = formatCoordinatesFilePathXML(projMonIndArr[proj_i], cal_i, CONFIG_DIR_PATH);
        if (loadCoordinatesXML(homMat, contPointParams, file_path, 0) != 0)
        {
            ROS_ERROR("XML: Missing XML File[%s]", file_path.c_str());
            return -1;
        }

        // Iterate through the maze grid
        for (float wall_i = 0; wall_i < MAZE_SIZE; wall_i++)
        {
            // Iterate through each cell in the maze row
            for (float wall_j = 0; wall_j < MAZE_SIZE; wall_j++)
            {
                // Get the image index for the current wall
                int wall_row = MAZE_SIZE - 1 - (int)wall_j;
                int wall_col = (int)wall_i;
                int img_ind = IMG_PROJ_MAP[proj_i][wall_row][wall_col][cal_i];

                // Bind image
                ilBindImage(r_image_id_vec[img_ind]); // show test pattern

                // Calculate shear and height for the current wall
                float width_val = calculateInterpolatedValue(cont_point_params, 2, wall_i, wall_j, MAZE_SIZE);
                float height_val = calculateInterpolatedValue(cont_point_params, 3, wall_i, wall_j, MAZE_SIZE);
                float shear_val = calculateInterpolatedValue(cont_point_params, 4, wall_i, wall_j, MAZE_SIZE);

                // Create wall vertices
                std::vector<cv::Point2f> rect_vertices_vec = computeRectVertices(0.0f, 0.0f, width_val, height_val, shear_val);

                // Apply perspective warping to vertices
                float x_offset = wall_i * WALL_SPACE;
                float y_offset = wall_j * WALL_SPACE;
                std::vector<cv::Point2f> rect_vertices_warped = computePerspectiveWarp(rect_vertices_vec, r_hom_mat, x_offset, y_offset);

                // Set texture image
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
                             ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB,
                             GL_UNSIGNED_BYTE, ilGetData());

                // Bind texture to framebuffer object
                glBindTexture(GL_TEXTURE_2D, fbo_texture_id);
                if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
                {
                    ROS_ERROR("Failed to Bind GL Frame Buffer Opbject for window[%d]", proj_i);
                    return -1;
                }

                // Draw the wall
                drawRectImage(rect_vertices_warped);
            }
        }
    }

    // Disable OpenGL texture mapping
    glDisable(GL_TEXTURE_2D);

    // Check for GL errors
    checkErrorGL(__LINE__, __FILE__);

    return 0;
}

int updateWindowMonMode(GLFWwindow *p_window_id, int win_ind, GLFWmonitor **&pp_r_monitor_id, int mon_id_ind, bool is_fullscreen)
{
    int x_pos, y_pos;

    // Set the current OpenGL context to the window
    glfwMakeContextCurrent(p_window_id);

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
        x_pos = 0;
        y_pos = 0;
        glfwSetWindowMonitor(p_window_id, p_monitor_id, x_pos, y_pos, mode->width, mode->height, mode->refreshRate);
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

            // Specify offset from the top-left corner of the monitor
            x_pos = monitor_x + (int)(500.0f * ((float)win_ind + 0.1f));
            y_pos = monitor_y + (int)(500.0f * ((float)win_ind + 0.1f));

            // Set the window to windowed mode and position it on the current monitor
            glfwSetWindowMonitor(p_window_id, NULL, x_pos, y_pos, (int)(500.0f * PROJ_WIN_ASPECT_RATIO), 500, 0);
        }

        // Update window title
        std::string new_title = "Window[" + std::to_string(win_ind) + "] Monitor[" + std::to_string(mon_id_ind) + "]";
        glfwSetWindowTitle(p_window_id, new_title.c_str());

        ROS_INFO("RAN: Update Window: Monitor[%d] Format[%s] X[%d] Y[%d]", mon_id_ind, is_fullscreen ? "fullscreen" : "windowed", x_pos, y_pos);
    }
    else
    {
        ROS_ERROR("[GLFW] Monitor[%d] Not Found", mon_id_ind);
        return -1;
    }

    return 0;
}

int main(int argc, char **argv)
{
    //  _______________ SETUP _______________

    // ROS Initialization
    ros::init(argc, argv, "projection_display", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    ROS_INFO("RUNNING MAIN");

    // Log paths for debugging
    ROS_INFO("[SETUP] Config XML Path: %s", CONFIG_DIR_PATH.c_str());
    ROS_INFO("[SETUP] Display: Width=%d Height=%d AR=%0.2f", PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, PROJ_WIN_ASPECT_RATIO);
    ROS_INFO("[SETUP] Wall (Norm): Width=%0.2f Space=%0.2f", wall_width_ndc, WALL_SPACE);

    // Initialize control point parameters
    updateCalParams(contPointParams, 0);

    // Do initial computations of homography matrix
    computeHomography(homMat, contPointParams);

    // --------------- OpenGL SETUP V2 ---------------

    // Initialize GLFW
    glfwSetErrorCallback(callbackErrorGLFW);
    if (!glfwInit())
    {
        ROS_ERROR("[GLFW] Initialization Failed");
        return -1;
    }

    // Get the list of available monitors and their count
    pp_monitorIDVec = glfwGetMonitors(&nMonitors);
    ROS_INFO("[GLFW] Monitors Found [%d] Projectors Sepcified[%d]", nMonitors, nProjectors);

    // Make sure nProj is not greater than the total number of monitors found
    if (nProjectors > nMonitors)
    {
        ROS_ERROR("[GLFW] Error Fewer Monitors[%d] Found than Expected Projectors[%d]", nMonitors, nProjectors);
        return -1;
    }

    // Copy the monitor ind associated with each projector
    for (int proj_i = 0; proj_i < nProjectors; ++proj_i)
    {
        p_projectorIDVec[proj_i] = pp_monitorIDVec[projMonIndArr[proj_i]];
    }

    // Create GLFW window
    for (int proj_i = 0; proj_i < nProjectors; ++proj_i)
    {
        int mon_id_ind = winMonIndDefault; // Show image on default monitor
        // int mon_id_ind =  projMonIndArr[proj_i]; // Show image on projector monitor

        if (setupProjGLFW(p_windowIDVec, proj_i, pp_monitorIDVec, mon_id_ind, windowNameVec[proj_i], fboIDVec[proj_i], fboTextureIDVec[proj_i]) != 0)
        {
            ROS_ERROR("[GLFW] Setup Failed for Window[%d]", proj_i);
            return -1;
        };
    }

    // --------------- DevIL SETUP ---------------

    // Initialize DevIL library
    ilInit();

    // Load images
    loadImgTextures(imgWallPathVec, imgWallIDVec);

    // _______________ MAIN LOOP _______________

    // Initialize a variable to check if all windows should close
    bool shouldClose = false;

    while (!shouldClose)
    {
        // Initialize a variable to check if all windows should close
        shouldClose = true;

        // Update the window contents and process events for each projectors window
        for (int proj_i = 0; proj_i < nProjectors; ++proj_i)
        {
            // Get the GLFW objects for this projector
            GLFWwindow *p_window_id = p_windowIDVec[proj_i];
            GLuint fbo_texture_id = fboTextureIDVec[proj_i];

            if (!glfwWindowShouldClose(p_window_id))
            {
                shouldClose = false; // At least one window is still open

                // Make the window's context current
                glfwMakeContextCurrent(p_window_id);
                if (glfwGetCurrentContext() != p_window_id)
                {
                    ROS_ERROR("[MAIN] Failed to Set GLFW Context for Window[%d]", proj_i);
                    return -1;
                }

                // Clear back buffer for new frame
                glClear(GL_COLOR_BUFFER_BIT);

                // Draw the walls
                if (drawWalls(homMat, contPointParams, proj_i, p_windowIDVec[proj_i], fboTextureIDVec[proj_i], imgWallIDVec) != 0)
                {
                    ROS_ERROR("[MAIN] Failed to Draw Walls for Window[%d]", proj_i);
                    return -1;
                }

                // Unbind the texture
                glBindTexture(GL_TEXTURE_2D, 0);

                // Unbind the FBO
                glBindFramebuffer(GL_FRAMEBUFFER, 0);

                // Swap buffers
                glfwSwapBuffers(p_window_id);

                // Check for GL errors
                checkErrorGL(__LINE__, __FILE__);
            }

            // Exit condition
            if (glfwGetKey(p_window_id, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(p_window_id))
            {
                shouldClose = true;
                break;
            }
        }

        // Poll and process events for all windows
        glfwPollEvents();

        // Check for GL errors
        checkErrorGL(__LINE__, __FILE__);
    }

    // _______________ CLEANUP _______________
    ROS_INFO("[SHUTDOWN] Started");

    // Destroy GL objects
    for (int proj_i = 0; proj_i < nProjectors; ++proj_i)
    {
        // Destroy GLFW window
        glfwDestroyWindow(p_windowIDVec[proj_i]);

        //  Delete each FBO and texture
        glDeleteFramebuffers(1, &fboIDVec[proj_i]);
        glDeleteTextures(1, &fboTextureIDVec[proj_i]);
    }
    ROS_INFO("[SHUTDOWN] Detroyd GLFW window and DevIL images");

    // Destroy DevIL images
    for (ILuint image_id : imgWallIDVec)
    {
        ilDeleteImages(1, &image_id);
    }
    ROS_INFO("[SHUTDOWN] Deleted DevIL images");

    // Shutdown DevIL
    ilShutDown();
    ROS_INFO("[SHUTDOWN] Shutdown DevIL");

    // Terminate GLFW
    glfwTerminate();
    ROS_INFO("[SHUTDOWN] Terminated GLFW");

    return 0;
}
