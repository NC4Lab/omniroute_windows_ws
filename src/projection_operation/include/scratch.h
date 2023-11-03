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
    ROS_INFO("[SETUP] Display: Width[%d] Height[%d] AR[%0.2f]", WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL, WINDOW_ASPECT_RATIO);
    ROS_INFO("[SETUP] Wall (Pxl): Width[%d] Height[%d]", WALL_IMAGE_WIDTH_PXL, WALL_IMAGE_HEIGHT_PXL);
    ROS_INFO("[SETUP] Wall (NDC): Width[%0.2f] Height[%0.2f] Space Horz[%0.2f] Space Vert[%0.2f]", WALL_IMAGE_WIDTH_NDC, WALL_IMAGE_HEIGHT_NDC, WALL_SPACE_HORZ_NDC, WALL_SPACE_VERT_NDC);
    ROS_INFO("[SETUP] Origin Plane (NDC): Width[%0.2f] Height[%0.2f]", WINDOW_WIDTH_PXL, MAZE_HEIGHT_NDC);

    // TEMP
    warpRenderWallImages();
    return 0;

    // --------------- VARIABLE SETUP ---------------

    // Initialze control points
    initControlPointCoordinates(CP_COORDS);

    // Initialize wall parameter datasets
    if (updateWallVertices(CP_COORDS, WALL_WARP_COORDS) != 0)
    {
        ROS_ERROR("[SETUP] Failed to Initalize the Wall Vertices Dataset");
        return -1;
    }

    // Initialize homography matrix dataset
    if (updateWallHomography(CP_COORDS, WALL_WARP_COORDS, WALL_HMAT_DATA) != 0)
    {
        ROS_ERROR("[SETUP] Failed to Initalize the Wall Homography Dataset");
        return -1;
    }

    // --------------- OpenGL SETUP ---------------

    // Initialize GLFW and set error callback
    glfwSetErrorCallback(callbackErrorGLFW);
    if (!glfwInit())
    {
        checkErrorGLFW(__LINE__, __FILE__);
        ROS_ERROR("[GLFW] Initialization Failed");
        return -1;
    }

    // Discover available monitors
    pp_monitorIDVec = glfwGetMonitors(&N.monitors);
    if (!pp_monitorIDVec || N.monitors == 0) // Added this check
    {
        ROS_ERROR("[GLFW] No monitors found");
        return -1;
    }
    ROS_INFO("[GLFW] Found %d monitors", N.monitors);

    // Create a new GLFW window
    p_windowID = glfwCreateWindow(WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL, "", NULL, NULL);
    checkErrorGLFW(__LINE__, __FILE__);
    if (!p_windowID)
    {
        glfwTerminate();
        ROS_ERROR("[GLFW] Create Window Failed");
        return -1;
    }

    // Set the GLFW window as the current OpenGL context
    glfwMakeContextCurrent(p_windowID);

    // Load OpenGL extensions using GLAD
    if (!gladLoadGL()) // Added this check
    {
        ROS_ERROR("[GLAD] Failed to initialize GLAD");
        return -1;
    }

    // Enable OpenGL debugging context
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(callbackErrorOpenGL, 0);

    // Set GLFW callbacks for keyboard and framebuffer size events
    glfwSetKeyCallback(p_windowID, callbackKeyBinding);
    glfwSetFramebufferSizeCallback(p_windowID, callbackFrameBufferSizeGLFW);

    // Initialize Framebuffer Object (FBO) and its texture
    GLuint fbo_id = 0;
    GLuint fbo_texture_id = 0;

    // Generate an FBO and bind it
    glGenFramebuffers(1, &fbo_id);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_id);
    if (checkErrorOpenGL(__LINE__, __FILE__) != 0)
    {
        ROS_ERROR("[OpenGL] Failed to Generate FBO");
        return -1;
    }

    // Generate a texture for the FBO
    glGenTextures(1, &fbo_texture_id);
    glBindTexture(GL_TEXTURE_2D, fbo_texture_id);
    if (checkErrorOpenGL(__LINE__, __FILE__) != 0)
    {
        ROS_ERROR("[OpenGL] Failed to Generate FBO Texture");
        return -1;
    }

    // Allocate storage for the texture on the GPU
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);

    // Set the texture's MIN and MAG filter to linear interpolation.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); // Handles sampling when the texture is scaled down
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); // Handles sampling when the texture is scaled up
    if (checkErrorOpenGL(__LINE__, __FILE__) != 0)
    {
        ROS_ERROR("[OpenGL] Failed to Set FBO Texture Parameters");
        return -1;
    }

    // Attach the texture to the FBO
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_texture_id, 0);

    // Check FBO completeness
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE)
    {
        // Handle incomplete FBO, possibly log an error or exit
        ROS_ERROR("[OpenGL] FBO is not complete");
        return -1;
    }

    // Unbind the FBO (bind to default framebuffer)
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (checkErrorOpenGL(__LINE__, __FILE__) != 0)
    {
        ROS_ERROR("[OpenGL] Failed to Unbind FBO");
        return -1;
    }

    // Update monitor and window mode settings
    updateWindowMonMode(p_windowID, 0, pp_monitorIDVec, I.winMon, F.setFullscreen);

    // Log OpenGL versions
    const GLubyte *opengl_version = glGetString(GL_VERSION);
    ROS_INFO("[OpenGL] Initialized: Version [%s]", opengl_version);

    // Log GLFW versions
    int glfw_major, glfw_minor, glfw_rev;
    glfwGetVersion(&glfw_major, &glfw_minor, &glfw_rev);
    ROS_INFO("[GLFW] Initialized: Version: %d.%d.%d", glfw_major, glfw_minor, glfw_rev);

    // --------------- DevIL SETUP ---------------

    // Initialize DevIL library
    ilInit();
    if (checkErrorDevIL(__LINE__, __FILE__) != 0)
        return -1;

    // Log the DevIL version
    ILint version = ilGetInteger(IL_VERSION_NUM);
    ROS_INFO("[DevIL] Intitalized: Version[%d]", version);

    // Load images
    if (loadImgTextures(imgWallPathVec, wallImgMatVec) != 0)
    {
        ROS_ERROR("[DevIL] Failed to load wall images");
        return -1;
    }
    if (loadImgTextures(imgMonPathVec, monImgMatVec) != 0)
    {
        ROS_ERROR("[DevIL] Failed to load monitor images");
        return -1;
    }
    if (loadImgTextures(imgCalPathVec, calImgMatVec) != 0)
    {
        ROS_ERROR("[DevIL] Failed to load calibration images");
        return -1;
    }

    // _______________ MAIN LOOP _______________

    bool is_error = false;
    while (!glfwWindowShouldClose(p_windowID) && ros::ok())
    {

        // --------------- Check Kayboard Callback Flags ---------------

        // Load XML file
        if (F.loadXML)
        {
            std::string file_path = formatCoordinatesFilePathXML(I.winMon, I.calMode, CONFIG_DIR_PATH);
            /// @todo Ad save xml back in
            F.loadXML = false;
        }

        // Save XML file
        if (F.saveXML)
        {
            std::string file_path = formatCoordinatesFilePathXML(I.winMon, I.calMode, CONFIG_DIR_PATH);
            /// @todo Ad save xml back in
            F.saveXML = false;
        }

        // Update the window monitor and mode
        if (F.updateWindowMonMode)
        {
            if (updateWindowMonMode(p_windowID, 0, pp_monitorIDVec, I.winMon, F.setFullscreen) != 0)
            {
                ROS_ERROR("[MAIN] Update Window Monitor Mode Threw Error");
                is_error = true;
                break;
            }
            F.updateWindowMonMode = false;
        }

        // Initialize/reinitialize control point coordinate dataset
        if (F.initControlPointMarkers)
        {
            initControlPointCoordinates(CP_COORDS);
            F.initControlPointMarkers = false;
        }

        // Recompute wall vertices and homography matrices
        if (F.updateWallDatasets)
        {
            // Initialize wall parameter datasets
            if (updateWallVertices(CP_COORDS, WALL_WARP_COORDS) != 0)
            {
                ROS_ERROR("[MAIN] Update of Wall Vertices Datasets Failed");
                return -1;
            }

            // Initialize homography matrix dataset
            if (updateWallHomography(CP_COORDS, WALL_WARP_COORDS, WALL_HMAT_DATA) != 0)
            {
                ROS_ERROR("[MAIN] Update of Wall Homography Datasets Failed");
                return -1;
            }
            F.updateWallDatasets = false;
        }

        // --------------- Handle Image Processing for Next Frame ---------------

        // Clear back buffer for new frame
        glClear(GL_COLOR_BUFFER_BIT);
        if (checkErrorOpenGL(__LINE__, __FILE__))
        {
            is_error = true;
            break;
        }

        // Draw/update wall images
        if (drawWallImages(fbo_texture_id, wallImgMatVec[I.wallImage], monImgMatVec[I.winMon], calImgMatVec[I.calMode]) != 0)
        {
            ROS_ERROR("[MAIN] Draw Walls Threw Error");
            is_error = true;
            break;
        }

        // Draw/update control point markers
        if (updateControlPointMarkers() != 0)
        {
            ROS_ERROR("[MAIN] Draw Control Point Threw Error");
            is_error = true;
            break;
        }

        // Swap buffers and poll events
        glfwSwapBuffers(p_windowID);
        if (
            checkErrorGLFW(__LINE__, __FILE__) ||
            checkErrorOpenGL(__LINE__, __FILE__))
        {
            is_error = true;
            break;
        }

        // Poll events
        glfwPollEvents();

        // Exit condition
        if (glfwGetKey(p_windowID, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(p_windowID))
            break;
    }

    // _______________ CLEANUP _______________
    ROS_INFO("SHUTTING DOWN");

    // Check which condition caused the loop to exit
    if (!ros::ok())
        ROS_INFO("[LOOP TERMINATION] ROS Node is no Longer in a Good State");
    else if (glfwWindowShouldClose(p_windowID))
        ROS_INFO("[LOOP TERMINATION] GLFW Window Should Close");
    else if (glfwGetKey(p_windowID, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        ROS_INFO("[LOOP TERMINATION] Escape Key was Pressed");
    else if (is_error)
        ROS_INFO("[LOOP TERMINATION] Error Thrown");
    else
        ROS_INFO("[LOOP TERMINATION] Reason Unknown");

    // Delete FBO
    if (fbo_id != 0)
    {
        glDeleteFramebuffers(1, &fbo_id);
        if (checkErrorOpenGL(__LINE__, __FILE__) != 0)
            ROS_WARN("[SHUTDOWN] Failed to Delete FBO");
        else
            ROS_INFO("[SHUTDOWN] Deleted FBO");
    }
    else
        ROS_WARN("[SHUTDOWN] No FBO to Delete");

    // Delete FBO texture
    if (fbo_texture_id != 0)
    {
        glDeleteTextures(1, &fbo_texture_id);
        if (checkErrorOpenGL(__LINE__, __FILE__) != 0)
            ROS_WARN("[SHUTDOWN] Failed to Delete FBO Texture");
        else
            ROS_INFO("[SHUTDOWN] Deleted FBO Texture");
    }
    else
        ROS_WARN("[SHUTDOWN] No FBO Texture to Delete");

    // Delete DevIL images
    if (deleteImgTextures(wallImgMatVec) == 0)
        ROS_INFO("[SHUTDOWN] Deleted DevIL Wall Images");
    if (deleteImgTextures(monImgMatVec) == 0)
        ROS_INFO("[SHUTDOWN] Deleted DevIL Monitor Images");
    if (deleteImgTextures(calImgMatVec) == 0)
        ROS_INFO("[SHUTDOWN] Deleted DevIL Calibration Images");

    // Destroy GLFW window
    if (p_windowID)
    {
        glfwDestroyWindow(p_windowID);
        p_windowID = nullptr;
        if (checkErrorGLFW(__LINE__, __FILE__) != 0)
            ROS_WARN("[SHUTDOWN] Failed to Destroy GLFW Window");
        else
            ROS_INFO("[SHUTDOWN] Destroyed GLFW Window");
    }
    else
    {
        ROS_WARN("[SHUTDOWN] No GLFW window to destroy");
    }

    // Shutdown DevIL
    ilShutDown();
    checkErrorDevIL(__LINE__, __FILE__);
    ROS_INFO("[SHUTDOWN] Shutdown DevIL");

    // Terminate GLFW
    glfwTerminate();
    checkErrorGLFW(__LINE__, __FILE__);
    ROS_INFO("[SHUTDOWN] Terminated GLFW");

    // Return success
    return is_error ? -1 : 0;
}
