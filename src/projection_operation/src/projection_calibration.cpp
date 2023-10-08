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
            resetParamCP();
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
            imgMonInd = 0;
        }
        else if (key == GLFW_KEY_1 && nMonitors > 1)
        {
            imgMonInd = 1;
        }
        else if (key == GLFW_KEY_2 && nMonitors > 2)
        {
            imgMonInd = 2;
        }
        else if (key == GLFW_KEY_3 && nMonitors > 3)
        {
            imgMonInd = 3;
        }
        else if (key == GLFW_KEY_4 && nMonitors > 4)
        {
            imgMonInd = 4;
        }
        else if (key == GLFW_KEY_5 && nMonitors > 5)
        {
            imgMonInd = 5;
        }

        // ---------- XML Handling [ENTER, L] ----------

        // Save coordinates to XML
        else if (key == GLFW_KEY_ENTER)
        {
            // Get the path to the config directory and format the save file name
            std::string file_path = formatCoordinatesFilePathXML(imgCalInd, imgMonInd, configDirPath);

            // Save the coordinates to the XML file
            saveCoordinatesXML(H, cpParam, file_path);
        }

        // Load coordinates from XML
        else if (key == GLFW_KEY_L)
        {
            // Get the path to the config directory and format the load file name
            std::string file_path = formatCoordinatesFilePathXML(imgCalInd, imgMonInd, configDirPath);

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
                imgCalInd = (imgCalInd > 0) ? imgCalInd - 1 : nCalModes - 1;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                imgCalInd = (imgCalInd < nCalModes - 1) ? imgCalInd + 1 : 0;
            }
            // Reset control point parameters when switching calibration modes
            if (key == GLFW_KEY_LEFT || key == GLFW_KEY_RIGHT)
            {
                resetParamCP();
            }
        }

        // ---------- Image change [ALT + [LEFT, RIGHT]] ----------

        else if (mods & GLFW_MOD_CONTROL)
        {
            // Listen for arrow key input to switch through images
            if (key == GLFW_KEY_LEFT)
            {
                imgTestInd = (imgTestInd > 0) ? imgTestInd - 1 : nTestImg - 1;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                imgTestInd = (imgTestInd < nTestImg - 1) ? imgTestInd + 1 : 0;
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
    computeHomography();

    // Update the window monitor and mode
    updateWindowMonMode();
}

void callbackFrameBufferSizeGLFW(GLFWwindow *window, int width, int height)
{
    glViewport(0, 0, width, height);
}

static void callbackErrorGLFW(int error, const char *description)
{
    ROS_ERROR("Error: %s\n", description);
}

void drawControlPoint(float x, float y, float radius, std::vector<float> rgb)
{
    int segments = 100; // Number of segments to approximate a circle

    // Begin drawing a filled circle
    glBegin(GL_TRIANGLE_FAN);

    // Set the color to green
    glColor3f(rgb[0], rgb[1], rgb[2]);

    // Center of the circle
    glVertex2f(x, y);

    // Calculate and draw the vertices of the circle
    for (int i = 0; i <= segments; i++)
    {
        float theta = 2.0f * 3.1415926f * float(i) / float(segments);
        float px = x + radius * cosf(theta);
        float py = y + (radius * winAspectRatio) * sinf(theta);
        glVertex2f(px, py);
    }

    // End drawing
    glEnd();
}

void drawRectImage(std::vector<cv::Point2f> img_vertices)
{

    // Start drawing a quadrilateral
    glBegin(GL_QUADS);

    // Set the color to white
    glColor3f(1.0f, 1.0f, 1.0f);

    // Set texture and vertex coordinates for each corner

    // Bottom-left corner
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(img_vertices[0].x, img_vertices[0].y);

    // Bottom-right corner
    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(img_vertices[1].x, img_vertices[1].y);

    // Top-right corner
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(img_vertices[2].x, img_vertices[2].y);

    // Top-left corner
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(img_vertices[3].x, img_vertices[3].y);

    // End drawing
    glEnd();
}

void drawWallsAll(cv::Mat &ref_H, float cp_param[4][5], int maze_size, float wall_width, float wall_space, GLuint fbo_texture, ILuint img_base_id, ILuint img_mon_id, ILuint img_param_id, ILuint img_cal_id)
{
    // Extract shear and height values from control points
    float height1 = cp_param[0][3];
    float height3 = cp_param[2][3];
    float height4 = cp_param[3][3];
    float shear1 = cp_param[0][4];
    float shear3 = cp_param[2][4];
    float shear4 = cp_param[3][4];

    // Enable OpenGL texture mapping
    glEnable(GL_TEXTURE_2D);

    // Iterate through the maze grid
    for (float i_wall = 0; i_wall < maze_size; i_wall++)
    {

        // Iterate through each cell in the maze row
        for (float j_wall = 0; j_wall < maze_size; j_wall++)
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
                ilBindImage(imgTestIDs[imgTestInd]); // show test pattern
            }

            // Set texture image
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
                         ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB,
                         GL_UNSIGNED_BYTE, ilGetData());

            // Bind texture to framebuffer object
            glBindTexture(GL_TEXTURE_2D, fbo_texture);

            // Calculate shear and height for the current wall
            float shear_val = shear4 + (i_wall / (maze_size - 1)) * (shear3 - shear4) +
                              (j_wall / (maze_size - 1)) * (shear1 - shear4);
            float height_val = height4 + (i_wall / (maze_size - 1)) * (height3 - height4) +
                               (j_wall / (maze_size - 1)) * (height1 - height4);

            // Create wall vertices
            std::vector<cv::Point2f> img_vertices = computeRectVertices(0.0f, 0.0f, wall_width, height_val, shear_val);

            // Apply perspective warping to vertices
            for (auto &p : img_vertices)
            {
                // Update vertex positions based on shear and height
                p.x += i_wall * wall_space;
                p.y += j_wall * wall_space;

                // Apply homography matrix to warp perspective
                float data[] = {p.x, p.y, 1};
                cv::Mat ptMat(3, 1, CV_32F, data);
                ref_H.convertTo(ref_H, ptMat.type());
                ptMat = ref_H * ptMat;
                ptMat /= ptMat.at<float>(2);

                // Update vertex coordinates
                p.x = ptMat.at<float>(0, 0);
                p.y = ptMat.at<float>(0, 1);
            }

            // Draw the wall
            drawRectImage(img_vertices);
        }
    }

    // Disable OpenGL texture mapping
    glDisable(GL_TEXTURE_2D);
}

void updateWindowMonMode()
{
    static int imp_mon_ind_last = imgMonInd;
    static int is_fullscreen_last = !isFullScreen;

    // Check if monitor or fullscreen mode has changed
    if (imp_mon_ind_last == imgMonInd && is_fullscreen_last == isFullScreen)
    {
        return;
    }

    // Get GLFWmonitor for active monitor
    monitor = monitors[imgMonInd];

    // Update window size and position
    if (monitor)
    {
        // Get the video mode of the selected monitor
        const GLFWvidmode *mode = glfwGetVideoMode(monitor);

        // Set the window to full-screen mode on the current monitor
        glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);

        if (!isFullScreen)
        {
            // Get the position of the current monitor
            int monitor_x, monitor_y;
            glfwGetMonitorPos(monitor, &monitor_x, &monitor_y);

            // Set the window to windowed mode and position it on the current monitor
            glfwSetWindowMonitor(window, NULL, monitor_x + 100, monitor_y + 100, (int)(500.0f * winAspectRatio), 500, 0);
        }
        ROS_INFO("RAN: Move window to monitor %d and set to %s", imgMonInd, isFullScreen ? "fullscreen" : "windowed");
    }
    else
    {
        ROS_WARN("FAILED: Move window to monitor %d and set to %s", imgMonInd, isFullScreen ? "fullscreen" : "windowed");
    }

    // Update last monitor and fullscreen mode
    imp_mon_ind_last = imgMonInd;
    is_fullscreen_last = isFullScreen;
}

void computeHomography()
{
    // Get the corner/vertex values for each of the control points
    std::vector<cv::Point2f> cp_vertices;
    cp_vertices.push_back(cv::Point2f(cpParam[0][0], cpParam[0][1]));
    cp_vertices.push_back(cv::Point2f(cpParam[1][0], cpParam[1][1]));
    cp_vertices.push_back(cv::Point2f(cpParam[2][0], cpParam[2][1]));
    cp_vertices.push_back(cv::Point2f(cpParam[3][0], cpParam[3][1]));

    // Get the corner/vertex values for each of the wall images
    std::vector<cv::Point2f> img_vertices;
    img_vertices = computeRectVertices(0.0f, 0.0f, (float(MAZE_SIZE) - 1) * wallSpace, (float(MAZE_SIZE) - 1) * wallSpace, 0);

    // Compute the homography matrix
    H = findHomography(img_vertices, cp_vertices);
}

void resetParamCP()
{
    // Copy the default array to the dynamic one
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 5; ++j)
        {
            cpParam[i][j] = cpParam_default[i][j];
        }
    }

    // Add an offset when calibrating left or right wall images
    float horz_offset = 0.2f;
    if (imgCalInd == 1) // left wall
    {
        cpParam[0][0] -= horz_offset;
        cpParam[1][0] -= horz_offset;
        cpParam[2][0] -= horz_offset;
        cpParam[3][0] -= horz_offset;
    }
    else if (imgCalInd == 2) // right wall
    {
        cpParam[0][0] += horz_offset;
        cpParam[1][0] += horz_offset;
        cpParam[2][0] += horz_offset;
        cpParam[3][0] += horz_offset;
    }
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
    ROS_INFO("SETTINGS: Package Path: %s", packagePath.c_str());
    ROS_INFO("SETTINGS: Config XML Path: %s", configDirPath.c_str());
    ROS_INFO("SETTINGS: Display: XYLim=[%0.2f,%0.2f] Width=%d Height=%d AR=%0.2f", xy_lim, xy_lim, winWidthPxl, winHeightPxl, winAspectRatio);
    ROS_INFO("SETTINGS: Wall (Norm): Width=%0.2f Space=%0.2f", wallWidth, wallSpace);
    ROS_INFO("SETTINGS: Wall (Pxl): Width=%d Space=%d", (int)(wallWidth * (float)winWidthPxl), (int)(wallSpace * (float)winWidthPxl));

    // Initialize control point parameters
    resetParamCP();

    // Initialize DevIL library
    ilInit();

    // Load images
    loadImgTextures(imgTestIDs, imgTestPaths);
    loadImgTextures(imgMonIDs, imgMonPaths);
    loadImgTextures(imgParamIDs, imgParamPaths);
    loadImgTextures(imgCalIDs, imgCalPaths);

    // Initialize GLFW
    glfwSetErrorCallback(callbackErrorGLFW);
    if (!glfwInit())
    {
        ROS_ERROR("GLFW: Initialization Failed");
        return -1;
    }

    // Create GLFW window
    window = glfwCreateWindow(winWidthPxl, winHeightPxl, windowName.c_str(), NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        ROS_ERROR("GLFW: Create Window Failed");
        return -1;
    }

    // Set OpenGL context and callbacks
    glfwMakeContextCurrent(window);
    gladLoadGL();
    glfwSetKeyCallback(window, callbackKeyBinding);
    glfwSetFramebufferSizeCallback(window, callbackFrameBufferSizeGLFW);

    // Initialize FBO and attach texture to it
    GLuint fbo;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glGenTextures(1, &fboTexture);
    glBindTexture(GL_TEXTURE_2D, fboTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, winWidthPxl, winHeightPxl, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fboTexture, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // // Call to initializeGL to set up OpenGL context and GLFW
    // initializeGL(window, winWidthPxl, winHeightPxl, windowName, fboTexture);

    // Get the list of available monitors and their count
    monitors = glfwGetMonitors(&nMonitors);
    // TEMP: hardcoding for now
    nMonitors = 2;

    // Do initial computations of homography matrix
    computeHomography();

    // Update the window monitor and mode
    updateWindowMonMode();

    // _______________ MAIN LOOP _______________

    while (!glfwWindowShouldClose(window))
    {
        // Clear back buffer for new frame
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw/update wall images
        drawWallsAll(H, cpParam, MAZE_SIZE, wallWidth, wallSpace, fboTexture, imgTestIDs[imgTestInd], imgMonIDs[imgMonInd], imgParamIDs[imgParamInd], imgCalIDs[imgCalInd]);

        // Draw/update control points
        for (int i = 0; i < 4; i++)
        {
            drawControlPoint(cpParam[i][0], cpParam[i][1], cpParam[i][2], cpSelected == i ? cpActiveRGB : cpInactiveRGB);
        }

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();

        // Exit condition
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(window))
            break;
    }

    // _______________ CLEANUP _______________

    // Destroy GLFW window and DevIL images
    glfwDestroyWindow(window);
    for (ILuint imageID : imgTestIDs)
    {
        ilDeleteImages(1, &imageID);
    }

    // Shutdown DevIL
    ilShutDown();

    // Terminate GLFW
    glfwTerminate();

    return 0;
}
