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
    ROS_ERROR("Error: %s\n", description);
}

int main(int argc, char **argv)
{
    // //  _______________ SETUP _______________

    // // ROS Initialization
    // ros::init(argc, argv, "projection_display", ros::init_options::AnonymousName);
    // ros::NodeHandle n;
    // ros::NodeHandle nh("~");
    // ROS_INFO("RUNNING MAIN");

    // // Log paths for debugging
    // ROS_INFO("SETTINGS: Config XML Path: %s", configDirPath.c_str());
    // ROS_INFO("SETTINGS: Display: XYLim=[%0.2f,%0.2f] Width=%d Height=%d AR=%0.2f", cp_xy_lim, cp_xy_lim, PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, PROJ_WIN_ASPECT_RATIO);
    // ROS_INFO("SETTINGS: Wall (Norm): Width=%0.2f Space=%0.2f", WALL_WIDTH, WALL_SPACE);
    // ROS_INFO("SETTINGS: Wall (Pxl): Width=%d Space=%d", (int)(WALL_WIDTH * (float)PROJ_WIN_WIDTH_PXL), (int)(WALL_SPACE * (float)PROJ_WIN_WIDTH_PXL));

    // // Initialize DevIL library
    // ilInit();

    // // Load images
    // // loadImgTextures(imgTestIDs, imgTestPaths);

    // // Initialize GLFW
    // glfwSetErrorCallback(callbackErrorGLFW);
    // if (!glfwInit())
    // {
    //     ROS_ERROR("GLFW: Initialization Failed");
    //     return -1;
    // }

    // // Create GLFW window
    // window = glfwCreateWindow(PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, windowName.c_str(), NULL, NULL);
    // if (!window)
    // {
    //     glfwTerminate();
    //     ROS_ERROR("GLFW: Create Window Failed");
    //     return -1;
    // }

    // // Set OpenGL context and callbacks
    // glfwMakeContextCurrent(window);
    // gladLoadGL();
    // glfwSetKeyCallback(window, callbackKeyBinding);
    // glfwSetFramebufferSizeCallback(window, callbackFrameBufferSizeGLFW);

    // // Initialize FBO and attach texture to it
    // GLuint fbo;
    // glGenFramebuffers(1, &fbo);
    // glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    // glGenTextures(1, &fboTexture);
    // glBindTexture(GL_TEXTURE_2D, fboTexture);
    // glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fboTexture, 0);
    // glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // // // Call to initializeGL to set up OpenGL context and GLFW
    // // initializeGL(window, winWidthPxl, winHeightPxl, windowName, fboTexture);

    // // Get the list of available monitors and their count
    // monitors = glfwGetMonitors(&nMonitors);
    // // TEMP: hardcoding for now
    // nMonitors = 2;

    // // _______________ MAIN LOOP _______________

    // while (!glfwWindowShouldClose(window))
    // {
    //     // Clear back buffer for new frame
    //     glClear(GL_COLOR_BUFFER_BIT);

    //     // Draw/update wall images
    //     drawWallsAll(H, cpParam, fboTexture, imgTestIDs[imgTestInd], imgMonIDs[imgMonInd], imgParamIDs[imgParamInd], imgCalIDs[imgCalInd]);

    //     // Draw/update control points
    //     for (int i = 0; i < 4; i++)
    //     {
    //         drawControlPoint(cpParam[i][0], cpParam[i][1], cpParam[i][2], cpSelected == i ? cpActiveRGB : cpInactiveRGB);
    //     }

    //     // Swap buffers and poll events
    //     glfwSwapBuffers(window);
    //     glfwPollEvents();

    //     // Exit condition
    //     if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(window))
    //         break;
    // }

    // // _______________ CLEANUP _______________

    // // Destroy GLFW window and DevIL images
    // glfwDestroyWindow(window);
    // for (ILuint imageID : imgTestIDs)
    // {
    //     ilDeleteImages(1, &imageID);
    // }

    // // Shutdown DevIL
    // ilShutDown();

    // // Terminate GLFW
    // glfwTerminate();

    return 0;
}
