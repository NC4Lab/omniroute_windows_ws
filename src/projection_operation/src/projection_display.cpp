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
}

void appLoadData()
{
    // Load images using OpenCV
    if (loadImgMat(wallImgPathVec, wallImgMatVec) < 0)
        throw std::runtime_error("[appLoadData] Failed to load wall images");
}

void appInitializeOpenGL()
{
    for (int proj_i = 0; proj_i < N.projectors; ++proj_i)
    {
        // Initialize GLFW and OpenGL settings
        if (MazeRenderContext::SetupGraphicsLibraries(N.monitors) < 0)
            throw std::runtime_error("[appInitializeOpenGL] Failed to initialize graphics");

        // Initialze render context for each projector
        if (PROJ_GL_ARR[proj_i].initContext(0, 0, callbackKeyBinding) < 0)
            throw std::runtime_error("[appInitializeOpenGL] Failed to initialize render context");

        // Initialize OpenGL wall image objects
        if (initWallRenderObjects(PROJ_GL_ARR[proj_i],
                                  WALL_GL_VERTICES, sizeof(WALL_GL_VERTICES),
                                  WALL_GL_INDICES, sizeof(WALL_GL_INDICES)) < 0)
            throw std::runtime_error("[appInitializeOpenGL] Failed to initialize opengl wall image objects");

        // Update monitor and window mode settings
        if (PROJ_GL_ARR[proj_i].switchWindowMode(projMonIndArr[proj_i], F.fullscreenMode) < 0)
            throw std::runtime_error("[appInitializeOpenGL] Failed Initial update of window monitor mode");

        // Create the shader program for wall image rendering
        if (PROJ_GL_ARR[proj_i].compileAndLinkShaders(WALL_VERTEX_SOURCE, WALL_FRAGMENT_SOURCE) < 0)
            throw std::runtime_error("[appInitializeOpenGL] Failed to compile and link wall shader");
    }

    ROS_INFO("SETUP: OpenGL Initialized");
}

void appMainLoop()
{
    // int status = 0;
    // while (status == 0)
    // {

    //     // --------------- Check Kayboard Callback Flags ---------------

    //     // Load/save XML file
    //     if (F.loadXML || F.saveXML)
    //     {
    //         std::string file_path_hmat = frmtFilePathXML(0, I.winMon, I.calMode, CONFIG_DIR_PATH);
    //         std::string file_path_cp = frmtFilePathXML(1, I.winMon, I.calMode, CONFIG_DIR_PATH);

    //         // Save XML file
    //         if (F.saveXML)
    //         {
    //             status = saveHMATxml(file_path_hmat, HMAT_GRID_ARR);
    //             status = saveCPxml(file_path_cp, CP_GRID_ARR);
    //         }
    //         F.saveXML = false;

    //         // Load XML file
    //         if (F.loadXML)
    //         {
    //             status = loadHMATxml(file_path_hmat, HMAT_GRID_ARR);
    //             status = loadCPxml(file_path_cp, CP_GRID_ARR);
    //             F.updateWallTextures = true;
    //         }
    //         F.loadXML = false;

    //         // Flash the background to indicate the file was loaded/saved
    //         if (status >= 0)
    //             PROJ_GL_ARR[0].flashBackgroundColor(cv::Scalar(0.0f, 0.25f, 0.0f), 500);
    //         else
    //         {
    //             PROJ_GL_ARR[0].flashBackgroundColor(cv::Scalar(0.25f, 0.0f, 0.0f), 500);
    //             throw std::runtime_error("[appMainLoop] Error return from load/save XML function");
    //             break;
    //         }
    //     }

    //     // Update the window monitor and mode
    //     if (F.switchWindowMode)
    //     {
    //         if (PROJ_GL_ARR[0].switchWindowMode(I.winMon, F.fullscreenMode) < 0)
    //             throw std::runtime_error("[appMainLoop] Error returned from updateWallHomographys");

    //         F.switchWindowMode = false;
    //     }

    //     // Recompute wall parameters and update wall image texture
    //     if (F.updateWallTextures)
    //     {
    //         // Update wall homography matrices array
    //         if (updateWallHomographys(CP_GRID_ARR, HMAT_GRID_ARR) < 0)
    //             throw std::runtime_error("[appMainLoop] Error returned from updateWallHomographys");

    //         // Update wall image texture
    //         if (updateWallTextures(wallImgMatVec[I.wallImage], monImgMatVec[I.winMon], calImgMatVec[I.calMode], HMAT_GRID_ARR, PROJ_GL_ARR[0].textureID) < 0)
    //             throw std::runtime_error("[appMainLoop] Error returned from updateWallTextures");

    //         F.updateWallTextures = false;
    //     }

    //     // --------------- Handle Image Processing for Next Frame ---------------

    //     // Prepare the frame for rendering (clear the back buffer)
    //     if (PROJ_GL_ARR[0].initWindow() < 0)
    //         throw std::runtime_error("[appMainLoop] Error returned from MazeRenderContext::initWindow");

    //     // Make sure winsow always stays on top in fullscreen mode
    //     PROJ_GL_ARR[0].setWindowStackOrder(F.fullscreenMode);

    //     // Draw/update wall images
    //     if (renderWallImage(PROJ_GL_ARR[0]) < 0)
    //         throw std::runtime_error("[appMainLoop] Error returned from renderWallImage");

    //     // Swap buffers and poll events
    //     if (PROJ_GL_ARR[0].bufferSwapPoll() < 0)
    //         throw std::runtime_error("[appMainLoop] Error returned from MazeRenderContext::bufferSwapPoll");

    //     // Check if ROS shutdown
    //     if (!ros::ok())
    //         throw std::runtime_error("[appMainLoop] Unextpected ROS shutdown");

    //     // Check for exit
    //     status = PROJ_GL_ARR[0].checkExitRequest();
    //     if (status < 0)
    //         throw std::runtime_error("[appMainLoop] Error returned from MazeRenderContext::checkExitRequest");
    // }

    // // Check which condition caused the loop to exit
    // if (status == 1)
    //     ROS_INFO("[appMainLoop] Loop Terminated:  GLFW window should close");
    // else if (status == 2)
    //     ROS_INFO("[appMainLoop] Loop Terminated:  Escape key was pressed");
    // else
    //     ROS_INFO("[appMainLoop] Loop Terminated:  Reason unknown");
}

void appCleanup()
{
    ROS_INFO("SHUTTING DOWN");

    // Delete CircleRenderer class shader program
    if (CircleRenderer::CleanupClassResources() < 0)
        ROS_WARN("[appCleanup] Failed to delete CircleRenderer shader program");
    else
        ROS_INFO("[appCleanup] CircleRenderer shader program deleted successfully");

    // Clean up OpenGL wall image objects
    if (PROJ_GL_ARR[0].cleanupContext() != 0)
        ROS_WARN("[appCleanup] Error during cleanup of MazeRenderContext instance");
    else
        ROS_INFO("[appCleanup] MazeRenderContext instance cleaned up successfully");

    // Terminate graphics
    if (MazeRenderContext::CleanupGraphicsLibraries() < 0)
        ROS_WARN("[appCleanup] Failed to terminate GLFW library");
    else
        ROS_INFO("[appCleanup] GLFW library terminated successfully");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "projection_calibration", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    try
    {
        appLoadData();
        appInitializeOpenGL();
        appMainLoop();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("!!EXCEPTION CAUGHT!!: %s", e.what());
        void appCleanup();
        return -1;
    }
    return 0;
}
