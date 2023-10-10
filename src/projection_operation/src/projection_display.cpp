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
    }

    // _______________ ANY KEY PRESS OR REPEAT ACTION _______________
    else if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
    }
}

void callbackFrameBufferSizeGLFW(GLFWwindow *window, int width, int height)
{
    glViewport(0, 0, width, height);
}

static void callbackErrorGLFW(int error, const char *description)
{
    ROS_ERROR("[GLFW] Error Flagged: %s", description);
}

void checkErrorGL(std::string msg_str)
{
    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR)
    {
        // Log or print the error code
        ROS_INFO("[OpenGL] Error Flagged [%s]: Error Number[%s]: ", msg_str, err);
    }
}

int setupProjGLFW(
    GLFWwindow **pp_window_id,
    int win_ind,
    GLFWmonitor **&pp_ref_monitor_id,
    int mon_ind,
    const std::string &ref_window_name,
    GLuint &ref_fbo_id,
    GLuint &ref_fbo_texture_id)
{
    // Create GLFW window
    pp_window_id[win_ind] = glfwCreateWindow(PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, ref_window_name.c_str(), NULL, NULL);
    if (!pp_window_id[win_ind])
    {
        glfwTerminate();
        ROS_ERROR("GLFW: Create Window Failed");
        return -1;
    }

    // Set OpenGL context and callbacks
    glfwMakeContextCurrent(pp_window_id[win_ind]);
    gladLoadGL();
    glfwSetKeyCallback(pp_window_id[win_ind], callbackKeyBinding);
    glfwSetFramebufferSizeCallback(pp_window_id[win_ind], callbackFrameBufferSizeGLFW);

    // Generate and set up the FBO
    glGenFramebuffers(1, &ref_fbo_id);
    glBindFramebuffer(GL_FRAMEBUFFER, ref_fbo_id);

    // Generate and set up the texture
    glGenTextures(1, &ref_fbo_texture_id);
    glBindTexture(GL_TEXTURE_2D, ref_fbo_texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Attach the texture to the FBO
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, ref_fbo_texture_id, 0);

    // Check for FBO completeness
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE)
    {
        ROS_ERROR("GLFW: Framebuffer is Not Complete");
        return -1;
    }

    // Unbind the FBO
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Tet window to wondowed mode
    if (updateWindowMonMode(pp_window_id[win_ind], pp_ref_monitor_id, mon_ind, false) != 0)
    {
        ROS_ERROR("GLFW: Failed to Update Window[%d] Monitor[%d] Mode", win_ind, mon_ind);
        return -1;
    }
    else
    {
        ROS_INFO("GLFW: Setup Window[%d] On Monitor[%d]", win_ind, mon_ind);
    }

    // Check for errors
    checkErrorGL("setupProjGLFW");

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
}

int drawWalls(
    cv::Mat &ref_H,
    float cp_param[4][5],
    int proj_i,
    GLFWwindow *p_window_id,
    GLuint fbo_texture_id,
    std::vector<ILuint> &ref_image_ids_vec)
{
    // Enable OpenGL texture mapping
    glEnable(GL_TEXTURE_2D);

    // Draw wall images for each calibration mode wall [left, middle, right]
    for (int cal_i = 0; cal_i < 3; cal_i++)
    {
        // Load the image transform coordinates from the XML file
        std::string file_path = formatCoordinatesFilePathXML(indProjMonCalArr[proj_i], cal_i, CONFIG_DIR_PATH);
        if (loadCoordinatesXML(H, cpParam, file_path, 0) != 0)
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
                ilBindImage(ref_image_ids_vec[img_ind]); // show test pattern

                // Calculate shear and height for the current wall
                float height_val = calculateInterpolatedValue(cp_param, 3, wall_i, wall_j, MAZE_SIZE);
                float shear_val = calculateInterpolatedValue(cp_param, 4, wall_i, wall_j, MAZE_SIZE);

                // Create wall vertices
                std::vector<cv::Point2f> rect_vertices_vec = computeRectVertices(0.0f, 0.0f, WALL_WIDTH, height_val, shear_val);

                // Apply perspective warping to vertices
                float x_offset = wall_i * WALL_SPACE;
                float y_offset = wall_j * WALL_SPACE;
                std::vector<cv::Point2f> rect_vertices_warped = computePerspectiveWarp(rect_vertices_vec, ref_H, x_offset, y_offset);

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

    return 0;
}

int updateWindowMonMode(GLFWwindow *p_window_id, GLFWmonitor **&pp_ref_monitor_id, int mon_ind, bool do_fullscreen)
{
    // Get GLFWmonitor for active monitor
    GLFWmonitor *p_monitor_id = pp_ref_monitor_id[mon_ind];

    // Update window size and position
    if (p_monitor_id)
    {
        // Get the video mode of the selected monitor
        const GLFWvidmode *mode = glfwGetVideoMode(p_monitor_id);

        // Set the window to full-screen mode on the current monitor
        glfwSetWindowMonitor(p_window_id, p_monitor_id, 0, 0, mode->width, mode->height, mode->refreshRate);

        if (!do_fullscreen)
        {
            // Get the position of the current monitor
            int monitor_x, monitor_y;
            glfwGetMonitorPos(p_monitor_id, &monitor_x, &monitor_y);

            // Specify offset from the top-left corner of the monitor
            int x_offset = 500 * (mon_ind);
            int y_offset = 500 * (mon_ind);

            // Set the window to windowed mode and position it on the current monitor
            glfwSetWindowMonitor(p_window_id, NULL, monitor_x + x_offset, monitor_y + y_offset, (int)(500.0f * PROJ_WIN_ASPECT_RATIO), 500, 0);
        }
    }
    else
    {
        ROS_ERROR("GLFW: Monitor[%d] Not Found", mon_ind);
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
    ROS_INFO("SETTINGS: Config XML Path: %s", CONFIG_DIR_PATH.c_str());
    ROS_INFO("SETTINGS: Display: Width=%d Height=%d AR=%0.2f", PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, PROJ_WIN_ASPECT_RATIO);
    ROS_INFO("SETTINGS: Wall (Norm): Width=%0.2f Space=%0.2f", WALL_WIDTH, WALL_SPACE);
    ROS_INFO("SETTINGS: Wall (Pxl): Width=%d Space=%d", (int)(WALL_WIDTH * (float)PROJ_WIN_WIDTH_PXL), (int)(WALL_SPACE * (float)PROJ_WIN_WIDTH_PXL));

    // Initialize control point parameters
    resetParamCP(cpParam, 0);

    // Do initial computations of homography matrix
    computeHomography(H, cpParam);

    // --------------- OpenGL SETUP V2 ---------------

    // Initialize GLFW
    glfwSetErrorCallback(callbackErrorGLFW);
    if (!glfwInit())
    {
        ROS_ERROR("GLFW: Initialization Failed");
        return -1;
    }

    // Get the list of available monitors and their count
    p_monitorIDVec = glfwGetMonitors(&nMonitors);
    ROS_INFO("GLFW: Found %d monitors", nMonitors);

    // Make sure nProj is not greater than the total number of monitors found
    if (nProjectors > nMonitors)
    {
        ROS_ERROR("GLFW: Error Fewer Monitors[%d] Found than Expected Projectors[%d]", nMonitors, nProjectors);
        return -1;
    }

    // Copy the monitor ind associated with each projector
    for (int i = 0; i < nProjectors; ++i)
    {
        p_projectorIDVec[i] = p_monitorIDVec[indProjMonCalArr[i]];
    }

    // Create GLFW window
    for (int i = 0; i < nProjectors; ++i)
    {
        if (setupProjGLFW(p_windowIDVec, i, p_monitorIDVec, indProjMonCalArr[i], windowNameVec[i], fboIDVec[i], fboTextureIDVec[i]) != 0)
        {
            ROS_ERROR("GLFW: Setup Failed for Window[%d]", i);
            return -1;
        };
    }

    // --------------- DevIL SETUP ---------------

    // Initialize DevIL library
    ilInit();

    // Load images
    loadImgTextures(imgWallIDVec, imgWallPathVec);

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
                    ROS_ERROR("Failed to Set GLFW Context for Window[%d]", proj_i);
                    return -1;
                }

                // Clear back buffer for new frame
                glClear(GL_COLOR_BUFFER_BIT);

                // Draw the walls
                drawWalls(H, cpParam, proj_i, p_windowIDVec[proj_i], fboTextureIDVec[proj_i], imgWallIDVec);

                // Unbind the texture
                glBindTexture(GL_TEXTURE_2D, 0);

                // Unbind the FBO
                glBindFramebuffer(GL_FRAMEBUFFER, 0);

                // Swap buffers
                glfwSwapBuffers(p_window_id);
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

        // Check for errors
        checkErrorGL("Main Loop");
    }

    // _______________ CLEANUP _______________

    // Destroy GL objects
    for (int proj_i = 0; proj_i < nProjectors; ++proj_i)
    {
        // Destroy GLFW window
        glfwDestroyWindow(p_windowIDVec[proj_i]);

        //  Delete each FBO and texture
        glDeleteFramebuffers(1, &fboIDVec[proj_i]);
        glDeleteTextures(1, &fboTextureIDVec[proj_i]);
    }

    // Destroy DevIL images
    for (ILuint image_id : imgWallIDVec)
    {
        ilDeleteImages(1, &image_id);
    }

    // Shutdown DevIL
    ilShutDown();

    // Terminate GLFW
    glfwTerminate();

    return 0;
}
