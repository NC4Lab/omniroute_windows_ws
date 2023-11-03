
int main(int argc, char **argv)
{

    //  _______________ SETUP _______________

    // ROS Initialization
    ros::init(argc, argv, "projection_calibration", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    ROS_INFO("RUNNING MAIN");

    // Log setup parameters
    ROS_INFO("[SETUP] Config XML Path: %s", CONFIG_DIR_PATH.c_str());
    ROS_INFO("[SETUP] Display: Width[%d] Height[%d] AR[%0.2f]", PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, PROJ_WIN_ASPECT_RATIO);
    ROS_INFO("[SETUP] Wall (Pxl): Width[%d] Height[%d]", WALL_WIDTH_PXL, WALL_HEIGHT_PXL);
    ROS_INFO("[SETUP] Wall (NDC): Width[%0.2f] Height[%0.2f] Space Horz[%0.2f] Space Vert[%0.2f]", WALL_WIDTH_NDC, WALL_HEIGHT_NDC, WALL_SPACE_HORZ_NDC, WALL_SPACE_VERT_NDC);
    ROS_INFO("[SETUP] Origin Plane (NDC): Width[%0.2f] Height[%0.2f]", PROJ_WIN_WIDTH_PXL, MAZE_HEIGHT_NDC);

    // --------------- VARIABLE SETUP ---------------

    // Specify window resolution size
    const int win_wd_pxl = 1000;
    const int win_ht_pxl = 1000;

    // Image size (pixels)
    const int im_wd_pxl = 300;
    const int im_ht_pxl = 540;

    // Image size (NDC)
    const float im_wd_ndc = (static_cast<float>(im_wd_pxl) / static_cast<float>(win_wd_pxl)) * 2;
    const float im_ht_ndc = (static_cast<float>(im_ht_pxl) / static_cast<float>(win_ht_pxl)) * 2;

    // --------------- OpenGL SETUP ---------------

    // Declare GLFW variables
    GLFWwindow *p_window_id = nullptr;
    GLFWmonitor **pp_monitor_id_Vec = nullptr;

    // Initialize GLFW and set error callback
    glfwSetErrorCallback(callbackErrorGLFW);
    if (!glfwInit())
    {
        ROS_ERROR("[SETUP] GLFW Initialization Failed");
        return -1;
    }

    // Discover available monitors
    pp_monitor_id_Vec = glfwGetMonitors(&N.monitors);
    if (!pp_monitor_id_Vec || N.monitors == 0)
    {
        ROS_ERROR("[SETUP] Monitors Found: None");
        return -1;
    }
    ROS_INFO("[SETUP] Monitors Found: %d", N.monitors);

    // Create a new GLFW window
    p_window_id = glfwCreateWindow(PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, "", NULL, NULL);
    //p_window_id = glfwCreateWindow(win_wd_pxl, win_ht_pxl, "OpenGL", NULL, NULL);
    if (!p_window_id || checkErrorGLFW(__LINE__, __FILE__))
    {
        glfwTerminate();
        ROS_ERROR("[SETUP] GLFW Failed to Create Window");
        return -1;
    }

    // Set the GLFW window as the current OpenGL context
    glfwMakeContextCurrent(p_window_id);

    // Load OpenGL extensions using GLAD
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

    // Load OpenGL extensions using GLAD
    if (!gladLoadGL()) // Added this check
    {
        ROS_ERROR("[SETUP] GLAD Failed to Load OpenGL");
        return -1;
    }

    // Set GLFW callbacks for keyboard and framebuffer size events
    glfwSetKeyCallback(p_window_id, callbackKeyBinding);
    glfwSetFramebufferSizeCallback(p_window_id, callbackFrameBufferSizeGLFW);

    // Enable OpenGL debugging context and associate callback
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(callbackDebugOpenGL, 0);

    // Initialize OpenGL objects
    if (initializeWallObjects() != 0)
    {
        ROS_ERROR("[SETUP] Failed to Initialize OpenGL Objects");
        return -1;
    }

    // Log OpenGL versions
    const GLubyte *opengl_version = glGetString(GL_VERSION);
    ROS_INFO("[OpenGL] Initialized: Version [%s]", opengl_version);

    // Log GLFW versions
    int glfw_major, glfw_minor, glfw_rev;
    glfwGetVersion(&glfw_major, &glfw_minor, &glfw_rev);
    ROS_INFO("[GLFW] Initialized: Version: %d.%d.%d", glfw_major, glfw_minor, glfw_rev);

    // Update monitor and window mode settings
    updateWindowMonMode(p_window_id, 0, pp_monitor_id_Vec, I.winMon, F.setFullscreen);

    // --------------- TEST IMAGE SETUP ---------------

    // // Test load and merg images
    // std::string base_img_path = "C:/Users/lester/MeDocuments/Research/MadhavLab/CodeBase/omniroute_windows_ws/data/proj_img/calibration_images/1_test_pattern.bmp";
    // std::string mask_img_path = "C:/Users/lester/MeDocuments/Research/MadhavLab/CodeBase/omniroute_windows_ws/data/proj_img/ui_state_images/m0.bmp";
    // std::string output_img_path = "C:/Users/lester/MeDocuments/Research/MadhavLab/CodeBase/omniroute_windows_ws/data/assets/temp/output_image.bmp";
    // cv::Mat im_wall;

    // // Merge the images
    // if (textureMerge(base_img_path, mask_img_path, output_img_path, im_wall))
    // {
    //     ROS_INFO("Successfully merged images.");
    // }
    // else
    // {
    //     std::cout << "Failed to merge images." << std::endl;
    // }

        // Load image using OpenCV
    std::string img_path = "C:/Users/lester/MeDocuments/Research/MadhavLab/CodeBase/omniroute_windows_ws/data/proj_img/calibration_images/1_test_pattern.bmp";
    // std::string img_path = "C:/Users/lester/MeDocuments/Research/MadhavLab/CodeBase/omniroute_windows_ws/data/proj_img/calibration_images/2_manu_pirate.bmp";
    cv::Mat im_wall = cv::imread(img_path.c_str());
    if (im_wall.empty())
    {
        ROS_ERROR("Failed to load image");
        return -1;
    }

    // Populate the source correspondence points
    /// @note Assumes Y-axis points down
    std::vector<cv::Point2f> srcPoints = {
        cv::Point2f(0, 0),                 // Top-left (0,0)
        cv::Point2f(im_wd_pxl, 0),         // Top-right (1,0)
        cv::Point2f(im_wd_pxl, im_ht_pxl), // Bottom-right (1,1)
        cv::Point2f(0, im_ht_pxl)};        // Bottom-left (0,1)

    // Populate the destination correspondence points
    std::vector<cv::Point2f> dstPoints = {
        cv::Point2f(375, 230),
        cv::Point2f(675, 205),
        cv::Point2f(600, 695),
        cv::Point2f(350, 770)};

    // Find Homography
    cv::Mat H = cv::findHomography(srcPoints, dstPoints);

    // TEMP
    // H = cv::Mat::eye(3, 3, CV_32F);

    // Warp Perspective
    cv::Mat im_warp;
    cv::warpPerspective(im_wall, im_warp, H, cv::Size(win_wd_pxl, win_ht_pxl));

    // // TEMP
    // im_warp = im_wall.clone();

    // Load warpedImage as a texture
    GLuint texture = loadTexture(im_warp);

    // // Display image directly through OpenCV
    // cv::namedWindow("Warped Image Display", cv::WINDOW_AUTOSIZE);
    // cv::imshow("Warped Image Display", im_warp);
    // cv::waitKey(0);
    // cv::destroyWindow("Warped Image Display");

    // Print params
    ROS_INFO("IMAGE DIMS: Rows[%d] Cols[%d]", im_wall.rows, im_wall.cols);
    ROS_INFO("srcPoints:");
    dbLogQuadVertices(srcPoints);
    // ROS_INFO("dstPoints:");
    // dbLogQuadVertices(dstPoints);
    dbLogHomMat(H);

    // Main loop (unchanged)
    while (!glfwWindowShouldClose(p_window_id))
    {

        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT);

        // Use the shader program for wall rendering
        glUseProgram(WALL_SHADER);

        // Bind the texture for the walls
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture);

        // Loop through all the wall images
        for (int gr_i = 0; gr_i < 1; gr_i++)
        {
            for (int gc_i = 0; gc_i < 1; gc_i++)
            {
                // Bind the Vertex Array Object(VAO) specific to the current wall
                glBindVertexArray(WALL_VAO_ARR[gr_i][gr_i]);

                // Bind the common Element Buffer Object (EBO)
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, WALL_EBO);

                // Draw the rectangle (2 triangles) for the current wall
                glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

                // Unbind the VAO to prevent accidental modification
                glBindVertexArray(0);
            }
        }

        // Swap the front and back buffers
        glfwSwapBuffers(p_window_id);

        // Poll for events like keyboard input or window closing
        glfwPollEvents();
    }

    // Cleanup
    glfwDestroyWindow(p_window_id);
    glfwTerminate();
    return 0;
}

