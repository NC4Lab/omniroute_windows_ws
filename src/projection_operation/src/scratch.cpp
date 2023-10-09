// ############################################################################################################

// ======================================== projection_calibration.cpp ========================================

// ############################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_calibration.h"

// ================================================== VARIABLES ==================================================

// Specify the window name
std::string windowName = "Projection Calibration";

// Variables related to control point parameters
const float cp_size = 0.015f;
const float cp_xy_lim = 0.5f;
const float cp_height = cp_size * PROJ_WIN_ASPECT_RATIO * 1.8 * 0.75;

// Control point parameter arrays
float cpParam_default[4][5] = {
    {-cp_xy_lim, cp_xy_lim, cp_size, cp_height, 0.0f},
    {cp_xy_lim, cp_xy_lim, cp_size, cp_height, 0.0f},
    {cp_xy_lim, -cp_xy_lim, cp_size, cp_height, 0.0f},
    {-cp_xy_lim, -cp_xy_lim, cp_size, cp_height, 0.0f}
};
float cpParam[4][5];

// Other variables related to control points
int cpSelected = 0;
std::string cpModMode = "position";
std::vector<float> cpActiveRGBVec = {1.0f, 0.0f, 0.0f};
std::vector<float> cpInactiveRGBVec = {0.0f, 0.0f, 1.0f};

// The 3x3 homography matrix
cv::Mat H = cv::Mat::eye(3, 3, CV_32F);

// Directory paths
std::string image_test_dir_path = IMAGE_TOP_DIR_PATH + "/calibration_images";
std::string image_state_dir_path = IMAGE_TOP_DIR_PATH + "/ui_state_images";

// Test image variables
std::vector<ILuint> imgTestIDVec;
std::vector<std::string> imgTestPathVec;
int imgTestInd = 0;

// Monitor variables
std::vector<ILuint> imgMonIDVec;
std::vector<std::string> imgMonPathVec;
int winMonInd = 0;
int nMonitors;
bool isFullScreen = false;

// Control point parameter image variables for ui
std::vector<ILuint> imgParamIDVec;
std::vector<std::string> imgParamPathVec;
int imgParamInd = 0;

// Calibration image variables for ui
std::vector<ILuint> imgCalIDVec;
std::vector<std::string> imgCalPathVec;
int calModeInd = 0;

// Variables related to window and OpenGL
GLFWwindow *p_windowID;
GLFWmonitor **p_monitorIDVec;

// ================================================== FUNCTIONS ==================================================

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

void drawWallsAll(
    cv::Mat &ref_H,
    float cp_param[4][5],
    GLuint fbo_texture,
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
                ilBindImage(imgTestIDVec[imgTestInd]); // show test pattern
            }

            // Set texture image
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
                         ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB,
                         GL_UNSIGNED_BYTE, ilGetData());

            // Bind texture to framebuffer object
            glBindTexture(GL_TEXTURE_2D, fbo_texture);

            // Calculate shear and height for the current wall
            float height_val = calculateInterpolatedValue(cp_param, 3, i_wall, j_wall, MAZE_SIZE);
            float shear_val = calculateInterpolatedValue(cp_param, 4, i_wall, j_wall, MAZE_SIZE);

            // Create wall vertices
            std::vector<cv::Point2f> rect_vertices = computeRectVertices(0.0f, 0.0f, WALL_WIDTH, height_val, shear_val);

            // Apply perspective warping to vertices
            float x_offset = i_wall * WALL_SPACE;
            float y_offset = j_wall * WALL_SPACE;
            std::vector<cv::Point2f> rect_vertices_warped = computePerspectiveWarp(rect_vertices, ref_H, x_offset, y_offset);

            // Draw the wall
            drawRectImage(rect_vertices_warped);
        }
    }

    // Disable OpenGL texture mapping
    glDisable(GL_TEXTURE_2D);
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

    // Attach texture to FBO
    glGenFramebuffers(1, &fbo_id);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_id);
    glGenTextures(1, &fbo_texture_id);
    glBindTexture(GL_TEXTURE_2D, fbo_texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_texture_id, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // --------------- DevIL SETUP ---------------

    // Initialize DevIL library
    ilInit();

    // Load images
    loadImgTextures(imgTestIDVec, imgTestPathVec);
    loadImgTextures(imgMonIDVec, imgMonPathVec);
    loadImgTextures(imgParamIDVec, imgParamPathVec);
    loadImgTextures(imgCalIDVec, imgCalPathVec);

    // --------------- Runtime SETUP ---------------

    // Initialize control point parameters
    resetParamCP(cpParam, cpParam_default, calModeInd);

    // Do initial computations of homography matrix
    computeHomography(H, cpParam);

    // Update the window monitor and mode
    updateWindowMonMode(p_windowID, p_monitorIDVec, winMonInd, isFullScreen);

    // _______________ MAIN LOOP _______________

    while (!glfwWindowShouldClose(p_windowID))
    {
        // Clear back buffer for new frame
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw/update wall images
        drawWallsAll(H, cpParam, fbo_texture_id, imgTestIDVec[imgTestInd], imgMonIDVec[winMonInd], imgParamIDVec[imgParamInd], imgCalIDVec[calModeInd]);

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
    for (ILuint image_id : imgTestIDVec)
    {
        ilDeleteImages(1, &image_id);
    }

    // Shutdown DevIL
    ilShutDown();

    // Terminate GLFW
    glfwTerminate();

    return 0;
}
