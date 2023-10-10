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

        // ---------- Control Point Reset [R] ----------

        if (key == GLFW_KEY_R)
        {
            resetParamCP(cpParam, calModeInd);
        }

        // ---------- Target selector keys [F1-F4] ----------

        // Top-left control point
        else if (key == GLFW_KEY_F1)
        {
            cpSelected = 0;
        }

        // Top-right control point
        else if (key == GLFW_KEY_F2)
        {
            cpSelected = 1;
        }

        // Bottom-right control point
        else if (key == GLFW_KEY_F3)
        {
            cpSelected = 2;
        }

        // Bottom-left control point
        else if (key == GLFW_KEY_F4)
        {
            cpSelected = 3;
        }

        // ---------- Change calibration point parameter keys [A, D, S] ----------

        // Control point position [up, down, left, right]
        else if (key == GLFW_KEY_A)
        {
            cpModMode = "position";
            imgParamInd = 0;
        }

        // Control point height [up, down]
        else if (key == GLFW_KEY_D)
        {
            cpModMode = "dimension";
            imgParamInd = 1;
        }

        // Control point shear [up, down]
        else if (key == GLFW_KEY_S)
        {
            cpModMode = "shear";
            imgParamInd = 2;
        }

        // ---------- Monitor handling [F, M] ----------

        // Set/unset Fullscreen
        else if (key == GLFW_KEY_F)
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
            saveCoordinatesXML(H, cpParam, file_path);
        }

        // Load coordinates from XML
        else if (key == GLFW_KEY_L)
        {
            // Get the path to the config directory and format the load file name
            std::string file_path = formatCoordinatesFilePathXML(winMonInd, calModeInd, CONFIG_DIR_PATH);

            // Load the coordinates from the XML file
            loadCoordinatesXML(H, cpParam, file_path);
        }
    }

    // _______________ ANY KEY PRESS OR REPEAT ACTION _______________
    else if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {

        // ---------- Calibration mode [ALT + [LEFT, RIGHT]] ----------

        if (mods & GLFW_MOD_ALT)
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
            // Reset control point parameters when switching calibration modes
            if (key == GLFW_KEY_LEFT || key == GLFW_KEY_RIGHT)
            {
                resetParamCP(cpParam, calModeInd);
            }
        }

        // ---------- Image change [ALT + [LEFT, RIGHT]] ----------

        else if (mods & GLFW_MOD_CONTROL)
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
            if (cpModMode == "position")
            {
                // Set the position increment based on whether the shift key is pressed
                float pos_inc = (mods & GLFW_MOD_SHIFT) ? 0.05f : 0.01f;

                // Listen for arrow key input to move selected control point
                if (key == GLFW_KEY_LEFT)
                {
                    cpParam[cpSelected][0] -= pos_inc;
                }
                else if (key == GLFW_KEY_RIGHT)
                {
                    cpParam[cpSelected][0] += pos_inc;
                }
                else if (key == GLFW_KEY_UP)
                {
                    cpParam[cpSelected][1] += pos_inc;
                }
                else if (key == GLFW_KEY_DOWN)
                {
                    cpParam[cpSelected][1] -= pos_inc;
                }
            }

            // ---------- Control point dimension/hight change [UP, DOWN] ----------
            if (cpModMode == "dimension")
            {
                // Set the dimension increment based on whether the shift key is pressed
                float dim_inc = (mods & GLFW_MOD_SHIFT) ? 0.0025f : 0.0005f;

                // Listen for arrow key input to adjust dimension/height
                if (key == GLFW_KEY_UP)
                {
                    cpParam[cpSelected][3] += dim_inc;
                }
                else if (key == GLFW_KEY_DOWN)
                {
                    cpParam[cpSelected][3] -= dim_inc;
                }
            }

            // ---------- Control point shear change [UP, DOWN] ----------
            if (cpModMode == "shear")
            {
                // Set the shear increment based on whether the shift key is pressed
                float shr_inc = (mods & GLFW_MOD_SHIFT) ? 0.025f : 0.005f;

                // Listen for arrow key input to adjust shear
                if (key == GLFW_KEY_UP)
                {
                    cpParam[cpSelected][4] += shr_inc;
                }
                else if (key == GLFW_KEY_DOWN)
                {
                    cpParam[cpSelected][4] -= shr_inc;
                }
            }
        }
    }

    // ---------- Recompute homography matrix ----------

    // Recompute homography matrix
    computeHomography(H, cpParam);

    // Update the window monitor and mode
    updateWindowMonMode(p_windowID, p_monitorIDVec, winMonInd, isFullScreen);
}

void callbackFrameBufferSizeGLFW(GLFWwindow *window, int width, int height)
{
    glViewport(0, 0, width, height);
}

static void callbackErrorGLFW(int error, const char *description)
{
    ROS_ERROR("[GLFW] Error Flagged: %s", description);
}

void drawControlPoint(float x, float y, float radius, std::vector<float> rgb_vec)
{
    int segments = 100; // Number of segments to approximate a circle

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

void drawWalls(
    cv::Mat &ref_H,
    float cp_param[4][5],
    GLuint fbo_texture_id,
    ILuint img_base_id,
    ILuint img_mon_id,
    ILuint img_param_id,
    ILuint img_cal_id)
{
    // Enable OpenGL texture mapping
    glEnable(GL_TEXTURE_2D);

    // Iterate through the maze grid
    for (float i_wall = 0; i_wall < MAZE_SIZE; i_wall++)
    {
        // Iterate through each cell in the maze row
        for (float j_wall = 0; j_wall < MAZE_SIZE; j_wall++)
        {
            // Bind image
            if (i_wall == 1 && j_wall == 1)
            {
                // Merge images
                ILuint merge_images_1 = mergeImages(img_base_id, img_mon_id);      // merge test pattern and active monitor image
                ILuint merge_images_2 = mergeImages(merge_images_1, img_param_id); // merge previous image and active cp parameter image
                ILuint merge_images_3 = mergeImages(merge_images_2, img_cal_id);   // merge previous image and active calibration image
                ilBindImage(merge_images_3);
            }
            else
            {
                ilBindImage(img_base_id); // show test pattern
            }

            // Calculate shear and height for the current wall
            float height_val = calculateInterpolatedValue(cp_param, 3, i_wall, j_wall, MAZE_SIZE);
            float shear_val = calculateInterpolatedValue(cp_param, 4, i_wall, j_wall, MAZE_SIZE);

            // Create wall vertices
            std::vector<cv::Point2f> rect_vertices_vec = computeRectVertices(0.0f, 0.0f, WALL_WIDTH, height_val, shear_val);

            // Apply perspective warping to vertices
            float x_offset = i_wall * WALL_SPACE;
            float y_offset = j_wall * WALL_SPACE;
            std::vector<cv::Point2f> rect_vertices_warped = computePerspectiveWarp(rect_vertices_vec, ref_H, x_offset, y_offset);

            // Set texture image
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
                         ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB,
                         GL_UNSIGNED_BYTE, ilGetData());

            // Bind texture to framebuffer object
            glBindTexture(GL_TEXTURE_2D, fbo_texture_id);

            // Draw the wall
            drawRectImage(rect_vertices_warped);
        }
    }

    // Disable OpenGL texture mapping
    glDisable(GL_TEXTURE_2D);
}

void updateWindowMonMode(GLFWwindow *p_window_id, GLFWmonitor **&pp_ref_monitor_id, int mon_ind, bool is_fullscreen)
{
    static int imp_mon_ind_last = mon_ind;
    static bool is_fullscreen_last = !is_fullscreen;

    // Check if monitor or fullscreen mode has changed
    if (imp_mon_ind_last == mon_ind && is_fullscreen_last == is_fullscreen)
    {
        return;
    }

    // Get GLFWmonitor for active monitor
    GLFWmonitor *p_monitor_id = pp_ref_monitor_id[mon_ind];

    // Update window size and position
    if (p_monitor_id)
    {
        // Get the video mode of the selected monitor
        const GLFWvidmode *mode = glfwGetVideoMode(p_monitor_id);

        // Set the window to full-screen mode on the current monitor
        glfwSetWindowMonitor(p_window_id, p_monitor_id, 0, 0, mode->width, mode->height, mode->refreshRate);

        if (!is_fullscreen)
        {
            // Get the position of the current monitor
            int monitor_x, monitor_y;
            glfwGetMonitorPos(p_monitor_id, &monitor_x, &monitor_y);

            // Set the window to windowed mode and position it on the current monitor
            glfwSetWindowMonitor(p_window_id, NULL, monitor_x + 100, monitor_y + 100, (int)(500.0f * PROJ_WIN_ASPECT_RATIO), 500, 0);
        }
        ROS_INFO("RAN: Move window to monitor %d and set to %s", mon_ind, is_fullscreen ? "fullscreen" : "windowed");
    }
    else
    {
        ROS_WARN("FAILED: Move window to monitor %d and set to %s", mon_ind, is_fullscreen ? "fullscreen" : "windowed");
        return;
    }

    // Update last monitor and fullscreen mode
    imp_mon_ind_last = mon_ind;
    is_fullscreen_last = is_fullscreen;
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
    ROS_INFO("SETTINGS: Config XML Path: %s", CONFIG_DIR_PATH.c_str());
    ROS_INFO("SETTINGS: Display: Width=%d Height=%d AR=%0.2f", PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, PROJ_WIN_ASPECT_RATIO);
    ROS_INFO("SETTINGS: Wall (Norm): Width=%0.2f Space=%0.2f", WALL_WIDTH, WALL_SPACE);
    ROS_INFO("SETTINGS: Wall (Pxl): Width=%d Space=%d", (int)(WALL_WIDTH * (float)PROJ_WIN_WIDTH_PXL), (int)(WALL_SPACE * (float)PROJ_WIN_WIDTH_PXL));

    // Initialize control point parameters
    resetParamCP(cpParam, calModeInd);

    // Do initial computations of homography matrix
    computeHomography(H, cpParam);

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

    // Create GLFW window
    p_windowID = glfwCreateWindow(PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, windowName.c_str(), NULL, NULL);
    if (!p_windowID)
    {
        glfwTerminate();
        ROS_ERROR("GLFW: Create Window Failed");
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

    // Generate and set up the texture
    glGenTextures(1, &fbo_texture_id);
    glBindTexture(GL_TEXTURE_2D, fbo_texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Attach the texture to the FBO
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_texture_id, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Update the window monitor and mode
    updateWindowMonMode(p_windowID, p_monitorIDVec, winMonInd, isFullScreen);

    // --------------- DevIL SETUP ---------------

    // Initialize DevIL library
    ilInit();

    // Load images
    loadImgTextures(imgWallIDVec, imgWallPathVec);
    loadImgTextures(imgMonIDVec, imgMonPathVec);
    loadImgTextures(imgParamIDVec, imgParamPathVec);
    loadImgTextures(imgCalIDVec, imgCalPathVec);

    // // TEMP
    // glClear(GL_COLOR_BUFFER_BIT);
    // drawWalls(H, cpParam, fbo_texture_id, imgWallIDVec[imgWallInd], imgMonIDVec[winMonInd], imgParamIDVec[imgParamInd], imgCalIDVec[calModeInd]);
    // glfwSwapBuffers(p_windowID);
    // ros::Duration(5.0).sleep(); // Sleeps for 1 second

    // _______________ MAIN LOOP _______________

    while (!glfwWindowShouldClose(p_windowID))
    {
        // Clear back buffer for new frame
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw/update wall images
        drawWalls(H, cpParam, fbo_texture_id, imgWallIDVec[imgWallInd], imgMonIDVec[winMonInd], imgParamIDVec[imgParamInd], imgCalIDVec[calModeInd]);

        // Draw/update control points
        for (int i = 0; i < 4; i++)
        {
            drawControlPoint(cpParam[i][0], cpParam[i][1], cpParam[i][2], cpSelected == i ? cpActiveRGBVec : cpInactiveRGBVec);
        }

        // Swap buffers and poll events
        glfwSwapBuffers(p_windowID);
        glfwPollEvents();

        // Exit condition
        if (glfwGetKey(p_windowID, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(p_windowID))
            break;
    }

    // _______________ CLEANUP _______________

    // Destroy GLFW window and DevIL images
    glfwDestroyWindow(p_windowID);
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
