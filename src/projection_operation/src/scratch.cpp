// void drawWallsAll(
//     cv::Mat &ref_H,
//     float cp_param[4][5],
//     GLuint fbo_texture_id,
//     ILuint img_base_id)
// {
//     // Enable OpenGL texture mapping
//     glEnable(GL_TEXTURE_2D);

//     // Iterate through the maze grid
//     for (float i_wall = 0; i_wall < MAZE_SIZE; i_wall++)
//     {
//         // Iterate through each cell in the maze row
//         for (float j_wall = 0; j_wall < MAZE_SIZE; j_wall++)
//         {
//             // Bind image
//             ilBindImage(imgWallIDVec[img_base_id]); // show test pattern

//             // Calculate shear and height for the current wall
//             float height_val = calculateInterpolatedValue(cp_param, 3, i_wall, j_wall, MAZE_SIZE);
//             float shear_val = calculateInterpolatedValue(cp_param, 4, i_wall, j_wall, MAZE_SIZE);

//             // Create wall vertices
//             std::vector<cv::Point2f> rect_vertices_vec = computeRectVertices(0.0f, 0.0f, WALL_WIDTH, height_val, shear_val);

//             // Apply perspective warping to vertices
//             float x_offset = i_wall * WALL_SPACE;
//             float y_offset = j_wall * WALL_SPACE;
//             std::vector<cv::Point2f> rect_vertices_warped = computePerspectiveWarp(rect_vertices_vec, ref_H, x_offset, y_offset);

//             // Set texture image
//             glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
//                          ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB,
//                          GL_UNSIGNED_BYTE, ilGetData());

//             // Bind texture to framebuffer object
//             glBindTexture(GL_TEXTURE_2D, fbo_texture_id);

//             // Draw the wall
//             drawRectImage(rect_vertices_warped);
//         }
//     }

//     // Disable OpenGL texture mapping
//     glDisable(GL_TEXTURE_2D);
// }

// //  _______________ SETUP _______________

//     // ROS Initialization
//     ros::init(argc, argv, "projection_display", ros::init_options::AnonymousName);
//     ros::NodeHandle n;
//     ros::NodeHandle nh("~");
//     ROS_INFO("RUNNING MAIN");

//     // Log paths for debugging
//     ROS_INFO("SETTINGS: Config XML Path: %s", CONFIG_DIR_PATH.c_str());
//     ROS_INFO("SETTINGS: Display: Width=%d Height=%d AR=%0.2f", PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, PROJ_WIN_ASPECT_RATIO);
//     ROS_INFO("SETTINGS: Wall (Norm): Width=%0.2f Space=%0.2f", WALL_WIDTH, WALL_SPACE);
//     ROS_INFO("SETTINGS: Wall (Pxl): Width=%d Space=%d", (int)(WALL_WIDTH * (float)PROJ_WIN_WIDTH_PXL), (int)(WALL_SPACE * (float)PROJ_WIN_WIDTH_PXL));

//     // --------------- OpenGL SETUP ---------------

//     std::string windowName = "TEMP";
//     GLFWwindow *p_windowID;

//     // Initialize GLFW
//     glfwSetErrorCallback(callbackErrorGLFW);
//     if (!glfwInit())
//     {
//         ROS_ERROR("GLFW: Initialization Failed");
//         return -1;
//     }

//     // Get the list of available monitors and their count
//     p_monitorIDVec = glfwGetMonitors(&nMonitors);
//     ROS_INFO("GLFW: Found %d monitors", nMonitors);

//     // Create GLFW window
//     p_windowID = glfwCreateWindow(PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, windowName.c_str(), NULL, NULL);
//     if (!p_windowID)
//     {
//         glfwTerminate();
//         ROS_ERROR("GLFW: Create Window Failed");
//         return -1;
//     }

//     // Set OpenGL context and callbacks
//     glfwMakeContextCurrent(p_windowID);
//     gladLoadGL();
//     glfwSetKeyCallback(p_windowID, callbackKeyBinding);
//     glfwSetFramebufferSizeCallback(p_windowID, callbackFrameBufferSizeGLFW);

//     // Initialize FBO and texture
//     GLuint fbo_id;
//     GLuint fbo_texture_id;

//     // Generate and set up the FBO
//     glGenFramebuffers(1, &fbo_id);
//     glBindFramebuffer(GL_FRAMEBUFFER, fbo_id);

//     // Generate and set up the texture
//     glGenTextures(1, &fbo_texture_id);
//     glBindTexture(GL_TEXTURE_2D, fbo_texture_id);
//     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
//     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

//     // Attach the texture to the FBO
//     glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_texture_id, 0);
//     glBindFramebuffer(GL_FRAMEBUFFER, 0);

//     // Initialize control point parameters
//     resetParamCP(cpParam, 0);

//     // --------------- DevIL SETUP ---------------

//     // Initialize DevIL library
//     ilInit();

//     // Load images
//     loadImgTextures(imgWallIDVec, imgWallPathVec);

//     // _______________ MAIN LOOP _______________

//     bool shouldClose = false;

//     // TEMP
//     glClear(GL_COLOR_BUFFER_BIT);
//     drawWallsAll(H, cpParam, fbo_texture_id, imgWallIDVec[0]);
//     glfwSwapBuffers(p_windowID);
//     glfwPollEvents();
//     ros::Duration(1.0).sleep(); // Sleeps for 1 second
//     shouldClose = true;






// --------------- TEMP ---------------

std::string windowName = "TEMP";
GLFWwindow *p_windowID;
int mon_ind = 1;
int win_ind = 0;

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

// Get GLFWmonitor for active monitor
GLFWmonitor *p_ref_monitor_id = p_monitorIDVec[mon_ind];

// Update window size and position
if (p_ref_monitor_id)
{
    // Get the video mode of the selected monitor
    const GLFWvidmode *mode = glfwGetVideoMode(p_ref_monitor_id);

    // Set the window to full-screen mode on the specified monitor
    glfwSetWindowMonitor(p_windowID, p_ref_monitor_id, 0, 0, mode->width, mode->height, mode->refreshRate);

    ROS_INFO("GLFW: Setup Window[%d] On Monitor[%d]", win_ind, mon_ind);
}
else
{
    ROS_ERROR("GLFW: Monitor[%d] Not Found", mon_ind);
    return -1;
}

TEMP Minimize the window
glfwIconifyWindow(p_windowID);

// Check for errors
checkErrorGL("setupProjGLFW");