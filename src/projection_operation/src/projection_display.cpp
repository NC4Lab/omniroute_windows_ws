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

void setupProjGLFW(
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
        return;
    }

    // Set OpenGL context and callbacks
    glfwMakeContextCurrent(pp_window_id[win_ind]);
    gladLoadGL();
    glfwSetKeyCallback(pp_window_id[win_ind], callbackKeyBinding);
    glfwSetFramebufferSizeCallback(pp_window_id[win_ind], callbackFrameBufferSizeGLFW);

    // Initialize FBO and attach texture to it
    glGenFramebuffers(1, &ref_fbo_id);
    glBindFramebuffer(GL_FRAMEBUFFER, ref_fbo_id);
    glGenTextures(1, &ref_fbo_texture_id);
    glBindTexture(GL_TEXTURE_2D, ref_fbo_texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, ref_fbo_texture_id, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Get GLFWmonitor for active monitor
    GLFWmonitor *p_ref_monitor_id = pp_ref_monitor_id[mon_ind];

    // Update window size and position
    if (pp_ref_monitor_id)
    {
        // Get the video mode of the selected monitor
        const GLFWvidmode *mode = glfwGetVideoMode(p_ref_monitor_id);

        // Set the window to full-screen mode on the specified monitor
        glfwSetWindowMonitor(pp_window_id[win_ind], p_ref_monitor_id, 0, 0, mode->width, mode->height, mode->refreshRate);

        ROS_INFO("GLFW: Setup Window[%d] On Monitor[%d]", win_ind, mon_ind);
    }
    else
    {
        ROS_ERROR("GLFW: Monitor[%d] Not Found", mon_ind);
    }

    // TEMP Minimize the window
    glfwIconifyWindow(pp_window_id[win_ind]);

    // Check for errors
    checkErrorGL("setupProjGLFW");
}

void drawRectImage(std::vector<cv::Point2f> rect_vertices_vec)
{

    // Start drawing a quadrilateral
    glBegin(GL_QUADS);

    // Set the color to white
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

void drawWallsAll()
{
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

    // --------------- OpenGL SETUP ---------------

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

    // TEMP: hardcoding for now
    nMonitors = 3;

    // Make sure nProj is not greater than the total number of monitors found
    if (nProjectors > nMonitors)
    {
        ROS_ERROR("GLFW: Error Fewer Monitors[%d] Found than Expected Projectors[%d]", nMonitors, nProjectors);
        return -1;
    }

    // Copy the last n_projectors entries from GLFWmonitor monitor pointer array
    int startIndex = nMonitors - nProjectors;
    for (int i = 0; i < nProjectors; ++i)
    {
        // Get the monotor indices for the projectors
        indProjectorMonitorArr[i] = startIndex + i;

        // Copy the GLFWmonitor pointer
        p_projectorIDVec[i] = p_monitorIDVec[indProjectorMonitorArr[i]];
    }

    // Create GLFW window
    for (int i = 0; i < nProjectors; ++i)
    {
        setupProjGLFW(p_windowIDVec, i, p_monitorIDVec, indProjectorMonitorArr[i], windowNameVec[i], fboIDVec[i], fboTextureIDVec[i]);
    }

    // --------------- DevIL SETUP ---------------

    // Initialize DevIL library
    ilInit();

    // Load images
    loadImgTextures(imgWallIDVec, imgWallPathVec);

    // _______________ MAIN LOOP _______________

    bool shouldClose = false;

    while (!shouldClose)
    {
        // Initialize a variable to check if all windows should close
        shouldClose = true;

        // Update the window contents and process events for each projectors window
        for (int i = 0; i < nProjectors; ++i)
        {
            GLFWwindow *p_window_id = p_windowIDVec[i];

            if (!glfwWindowShouldClose(p_window_id))
            {
                shouldClose = false; // At least one window is still open

                // Make the window's context current
                glfwMakeContextCurrent(p_window_id);

                // Clear back buffer for new frame
                glClear(GL_COLOR_BUFFER_BIT);

                // Bind the FBO
                glBindFramebuffer(GL_FRAMEBUFFER, fboIDVec[i]);

                // Enable OpenGL texture mapping
                glEnable(GL_TEXTURE_2D);

                // Loop through the calibration modes [left, middle, right]
                for (int j = 0; j < 3; j++)
                {
                    // Load the image transform coordinates from the XML file
                    std::string file_path = formatCoordinatesFilePathXML(indProjectorMonitorArr[i], j, CONFIG_DIR_PATH);
                    loadCoordinatesXML(H, cpParam, file_path);

                    // Bind texture to framebuffer object
                    glBindTexture(GL_TEXTURE_2D, fboTextureIDVec[i]);

                    // Iterate through the maze grid
                    for (float i_wall = 0; i_wall < MAZE_SIZE; i_wall++)
                    {
                        // Iterate through each cell in the maze row
                        for (float j_wall = 0; j_wall < MAZE_SIZE; j_wall++)
                        {
                            // Bind image
                            ilBindImage(imgWallIDVec[0]);

                            // Set texture image
                            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
                                         ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB,
                                         GL_UNSIGNED_BYTE, ilGetData());

                            // Calculate shear and height for the current wall
                            float height_val = calculateInterpolatedValue(cpParam, 3, i_wall, j_wall, MAZE_SIZE);
                            float shear_val = calculateInterpolatedValue(cpParam, 4, i_wall, j_wall, MAZE_SIZE);

                            // Create wall vertices
                            std::vector<cv::Point2f> rect_vertices = computeRectVertices(0.0f, 0.0f, WALL_WIDTH, height_val, shear_val);

                            // Apply perspective warping to vertices
                            float x_offset = i_wall * WALL_SPACE;
                            float y_offset = j_wall * WALL_SPACE;
                            std::vector<cv::Point2f> rect_vertices_warped = computePerspectiveWarp(rect_vertices, H, x_offset, y_offset);

                            // Draw the wall
                            drawRectImage(rect_vertices_warped);
                        }
                    }
                }

                // Unbind the FBO
                glBindFramebuffer(GL_FRAMEBUFFER, 0);

                // Disable OpenGL texture mapping
                glDisable(GL_TEXTURE_2D);

                // Swap buffers and poll events
                glfwSwapBuffers(p_window_id);
            }

            // Exit condition
            if (glfwGetKey(p_window_id, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(p_window_id))
            {
                shouldClose = true;
                break;
            }
        }

        // Poll for and process events for all windows
        glfwPollEvents();

        // Check for errors
        checkErrorGL("Main Loop");
    }

    // _______________ CLEANUP _______________

    // Destroy GLFW window
    for (int i = 0; i < nProjectors; ++i)
    {
        glfwDestroyWindow(p_windowIDVec[i]);
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
