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

int setupProjGLFW(
    GLFWwindow **pp_window_id,
    int win_ind,
    GLFWmonitor **&pp_r_monitor_id,
    int mon_id_ind,
    GLuint &r_fbo_id,
    GLuint &r_fbo_texture_id)
{
    // Create GLFW window
    pp_window_id[win_ind] = glfwCreateWindow(PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, "", NULL, NULL);
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

int drawQuadImage(std::vector<cv::Point2f> quad_vertices_vec)
{

    // Start drawing a quadrilateral
    glBegin(GL_QUADS);

    // Set the color to white (for texture mapping)
    /// @note: this is nececary when drawing the control points
    glColor3f(1.0f, 1.0f, 1.0f);

    // Set texture and vertex coordinates for each corner

    // Top-left corner of texture
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(quad_vertices_vec[0].x, quad_vertices_vec[0].y);

    // Top-right corner of texture
    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(quad_vertices_vec[1].x, quad_vertices_vec[1].y);

    // Bottom-right corner of texture
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(quad_vertices_vec[2].x, quad_vertices_vec[2].y);

    // Bottom-left corner of texture
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(quad_vertices_vec[3].x, quad_vertices_vec[3].y);

    // End drawing
    glEnd();

    // Check and return GL status
    return checkErrorGL(__LINE__, __FILE__);
}

int drawWalls(
    int proj_ind,
    int mon_id_ind,
    GLFWwindow *p_window_id,
    GLuint fbo_texture_id,
    std::vector<ILuint> &r_image_id_vec)
{

    // Enable OpenGL texture mapping
    glEnable(GL_TEXTURE_2D);

    // Draw wall images for each calibration mode wall [left, middle, right]
    for (int cal_i = 0; cal_i < 3; cal_i++)
    {
        // Initialize control point parameter array and homography matrix
        std::array<std::array<float, 6>, 4> ctrl_point_params;
        cv::Mat hom_mat = cv::Mat::eye(3, 3, CV_32F);

        // Load the image transform coordinates from the XML file
        std::string file_path = frmtFilePathxml(mon_id_ind, cal_i, CONFIG_DIR_PATH);
        if (loadHMATxml(hom_mat, ctrl_point_params, file_path, 0) != 0)
        {
            ROS_ERROR("XML: Missing XML File[%s]", file_path.c_str());
            return -1;
        }

        // TEMP 
        computeHomography(hom_mat, ctrl_point_params);

        // Iterate through the maze grid
        for (float grid_row_i = 0; grid_row_i < MAZE_SIZE; grid_row_i++)
        {
            // Iterate through each cell in the maze row
            for (float grid_col_i = 0; grid_col_i < MAZE_SIZE; grid_col_i++)
            {
                // Get the image index for the current wall
                int wall_row = MAZE_SIZE - 1 - (int)grid_row_i;
                int wall_col = (int)grid_col_i;
                int img_ind = IMG_PROJ_MAP[proj_ind][wall_row][wall_col][cal_i];

                // Bind image
                ilBindImage(r_image_id_vec[img_ind]); // show test pattern

                // Calculate width, height and shear for the current wall
                float width = bilinearInterpolationFull(ctrl_point_params, 2, grid_row_i, grid_col_i, MAZE_SIZE);   // wall width
                float height = bilinearInterpolationFull(ctrl_point_params, 3, grid_row_i, grid_col_i, MAZE_SIZE);  // wall height
                float shear_x = bilinearInterpolationFull(ctrl_point_params, 4, grid_row_i, grid_col_i, MAZE_SIZE); // wall x shear
                float shear_y = bilinearInterpolationFull(ctrl_point_params, 5, grid_row_i, grid_col_i, MAZE_SIZE); // wall x shear

                // Get origin coordinates of wall
                float x_origin = grid_col_i * WALL_SPACE_X;
                float y_origin = grid_row_i * WALL_SPACE_Y;

                // Create wall vertices
                std::vector<cv::Point2f> quad_vertices_raw = computeQuadVertices(x_origin, y_origin, width, height, shear_x, shear_y);

                // Apply perspective warping to vertices
                std::vector<cv::Point2f> quad_vertices_warped = computePerspectiveWarp(quad_vertices_raw, hom_mat);

                // Set texture image
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
                             ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB,
                             GL_UNSIGNED_BYTE, ilGetData());

                // Bind texture to framebuffer object
                glBindTexture(GL_TEXTURE_2D, fbo_texture_id);
                if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
                {
                    ROS_ERROR("Failed to Bind GL Frame Buffer Opbject for window[%d]", proj_ind);
                    return -1;
                }

                // Draw the wall
                if (drawQuadImage(quad_vertices_warped) != 0)
                    return -1;
            }
        }
    }

    // Disable OpenGL texture mapping
    glDisable(GL_TEXTURE_2D);

    // Check for GL errors
    checkErrorGL(__LINE__, __FILE__);

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

    // Create GLFW window
    for (int proj_i = 0; proj_i < nProjectors; ++proj_i)
    {
        int mon_id_ind = winMonIndDefault; // Show image on default monitor
        // int mon_id_ind =  projMonIndArr[proj_i]; // Show image on projector monitor

        if (setupProjGLFW(p_windowIDVec, proj_i, pp_monitorIDVec, mon_id_ind, fboIDVec[proj_i], fboTextureIDVec[proj_i]) != 0)
        {
            ROS_ERROR("[GLFW] Setup Failed for Window[%d]", proj_i);
            return -1;
        };
    }

    // --------------- DevIL SETUP ---------------

    // Initialize DevIL library
    ilInit();

    // Load images
    if (loadImgTextures(imgWallPathVec, imgWallIDVec) != 0)
    {
        ROS_ERROR("[DevIL] Failed to load wall images");
        return -1;
    }

    // _______________ MAIN LOOP _______________

    // Initialize a variable to check for errors and windows closed
    bool is_win_closed = false;
    bool is_err_thrown = false;

    while (!is_err_thrown && !is_win_closed && ros::ok())
    {
        is_win_closed = true;

        // Update the window contents and process events for each projectors window
        for (int proj_i = 0; proj_i < nProjectors; ++proj_i)
        {
            // Get the GLFW objects for this projector
            GLFWwindow *p_window_id = p_windowIDVec[proj_i];
            GLuint fbo_texture_id = fboTextureIDVec[proj_i];

            if (!glfwWindowShouldClose(p_window_id))
            {
                is_win_closed = false; // At least one window is still open

                // Make the window's context current
                glfwMakeContextCurrent(p_window_id);
                if (glfwGetCurrentContext() != p_window_id)
                {
                    ROS_ERROR("[MAIN] Failed to Set GLFW Context for Window[%d]", proj_i);
                    is_err_thrown = true;
                    break;
                }

                // Clear back buffer for new frame
                glClear(GL_COLOR_BUFFER_BIT);

                // Draw the walls
                if (drawWalls(proj_i, projMonIndArr[proj_i], p_windowIDVec[proj_i], fboTextureIDVec[proj_i], imgWallIDVec) != 0)
                {
                    ROS_ERROR("[MAIN] Failed to Draw Walls for Window[%d]", proj_i);
                    is_err_thrown = true;
                    break;
                }

                // Unbind the texture
                glBindTexture(GL_TEXTURE_2D, 0);

                // Unbind the FBO
                glBindFramebuffer(GL_FRAMEBUFFER, 0);

                // Swap buffers
                glfwSwapBuffers(p_window_id);
                if (checkErrorGLFW(__LINE__, __FILE__) ||
                    checkErrorGL(__LINE__, __FILE__))
                {
                    is_err_thrown = true;
                    break;
                }
            }

            // Exit condition
            if (glfwGetKey(p_window_id, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(p_window_id))
            {
                is_win_closed = true;
                break;
            }
        }

        // Poll and process events for all windows
        glfwPollEvents();
        if (checkErrorGLFW(__LINE__, __FILE__))
            break;
    }

    // _______________ CLEANUP _______________
    ROS_INFO("SHUTTING DOWN");

    // Check which condition caused the loop to exit
    if (!ros::ok())
        ROS_INFO("[LOOP TERMINATION] ROS Node is no Longer in a Good State");
    else if (is_win_closed)
        ROS_INFO("[LOOP TERMINATION] GLFW Window Should Close");
    else if (is_err_thrown)
        ROS_INFO("[LOOP TERMINATION] An Error Was Thrown");
    else
        ROS_INFO("[LOOP TERMINATION] Reason Unknown");

    // Delete FBO and textures
    for (int proj_i = 0; proj_i < nProjectors; ++proj_i)
    {
        glDeleteFramebuffers(1, &fboIDVec[proj_i]);
        checkErrorGL(__LINE__, __FILE__);
        glDeleteTextures(1, &fboTextureIDVec[proj_i]);
        checkErrorGL(__LINE__, __FILE__);
    }
    ROS_INFO("[SHUTDOWN] Deleted FBO and textures");

    // Delete DevIL images
    deleteImgTextures(imgWallIDVec);
    ROS_INFO("[SHUTDOWN] Deleted DevIL images");

    // Destroy GL objects
    for (int proj_i = 0; proj_i < nProjectors; ++proj_i)
    {
        glfwDestroyWindow(p_windowIDVec[proj_i]);
        p_windowIDVec[proj_i] = nullptr;
        checkErrorGLFW(__LINE__, __FILE__);
    }
    ROS_INFO("[SHUTDOWN] Detroyd GLFW windows");

    // Shutdown DevIL
    ilShutDown();
    checkErrorDevIL(__LINE__, __FILE__);
    ROS_INFO("[SHUTDOWN] Shutdown DevIL");

    // Terminate GLFW
    glfwTerminate();
    checkErrorGLFW(__LINE__, __FILE__);
    ROS_INFO("[SHUTDOWN] Terminated GLFW");

    return 0;
}
