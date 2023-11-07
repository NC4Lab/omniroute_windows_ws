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

        // ---------- Monitor handling [F] ----------

        // Set/unset Fullscreen
        if (key == GLFW_KEY_F)
        {
            F.setFullscreen = !F.setFullscreen;
            F.switchWindowMode = true;
        }

        // Move the window to another monitor
        else if (key == GLFW_KEY_0)
        {
            I.winMon = 0;
            F.switchWindowMode = true;
        }
        else if (key == GLFW_KEY_1 && N.monitors > 1)
        {
            I.winMon = 1;
            F.switchWindowMode = true;
        }
        else if (key == GLFW_KEY_2 && N.monitors > 2)
        {
            I.winMon = 2;
            F.switchWindowMode = true;
        }
        else if (key == GLFW_KEY_3 && N.monitors > 3)
        {
            I.winMon = 3;
            F.switchWindowMode = true;
        }
        else if (key == GLFW_KEY_4 && N.monitors > 4)
        {
            I.winMon = 4;
            F.switchWindowMode = true;
        }
        else if (key == GLFW_KEY_5 && N.monitors > 5)
        {
            I.winMon = 5;
            F.switchWindowMode = true;
        }

        // ---------- XML Handling [ENTER, L] ----------

        // Save coordinates to XML
        else if (key == GLFW_KEY_S)
        {
            F.saveXML = true;
        }

        // Load coordinates from XML
        else if (key == GLFW_KEY_L)
        {
            F.loadXML = true;
        }

        // ---------- Image selector keys [F1-F4] ----------

        else if (key == GLFW_KEY_F1)
        {
            I.wallImage = N.wallImages > 0 ? 0 : I.wallImage;
            F.updateWallTexture = true;
        }
        else if (key == GLFW_KEY_F2)
        {
            I.wallImage = N.wallImages > 1 ? 1 : I.wallImage;
            F.updateWallTexture = true;
        }
        else if (key == GLFW_KEY_F3)
        {
            I.wallImage = N.wallImages > 2 ? 2 : I.wallImage;
            F.updateWallTexture = true;
        }
        else if (key == GLFW_KEY_F4)
        {
            I.wallImage = N.wallImages > 3 ? 3 : I.wallImage;
            F.updateWallTexture = true;
        }

        // ---------- Control Point Reset [R] ----------

        else if (key == GLFW_KEY_R)
        {
            F.initControlPointMarkers = true;
        }
    }

    // _______________ ANY KEY PRESS OR REPEAT ACTION _______________
    else if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {

        // ---------- Calibration mode [CTRL + SHIFT [LEFT, RIGHT]] ----------

        if ((mods & GLFW_MOD_CONTROL) && (mods & GLFW_MOD_SHIFT))
        {

            // Listen for arrow key input to switch through calibration modes
            if (key == GLFW_KEY_LEFT)
            {
                I.calMode = (I.calMode > 0) ? I.calMode - 1 : N.calModes - 1;
                F.initControlPointMarkers = true;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                I.calMode = (I.calMode < N.calModes - 1) ? I.calMode + 1 : 0;
                F.initControlPointMarkers = true;
            }
        }

        // ---------- Contol point maze vertex selector keys [CTRL [LEFT, RIGHT, UP, DOWN]] ----------

        else if (mods & GLFW_MOD_CONTROL)
        {
            bool is_cp_maze_vert_changed = false;

            if (key == GLFW_KEY_UP)
            {
                // Set row index to top row
                I.cpMazeVertSel[0] = 0;
                is_cp_maze_vert_changed = true;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                // Set row index to bottom row
                I.cpMazeVertSel[0] = 1;
                is_cp_maze_vert_changed = true;
            }
            else if (key == GLFW_KEY_LEFT)
            {
                // Set column index to first column
                I.cpMazeVertSel[1] = 0;
                is_cp_maze_vert_changed = true;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                // Set column index to second column
                I.cpMazeVertSel[1] = 1;
                is_cp_maze_vert_changed = true;
            }

            if (is_cp_maze_vert_changed)
            {
                // Set flag to update the wall texture if the maze vertex was changed
                /// @note this is ensures the mode images are udated to the new
                F.updateWallTexture = true;

                // Set the wall vertex to the ortin if the maze vertex is changed
                for (int i = 0; i < 2; ++i)
                {
                    for (int j = 0; j < 2; ++j)
                    {
                        if (I.cpMap[i][j] == I.cpWVOrigin)
                        {
                            // Set the wall vertex to the origin
                            I.cpWallVertSel[0] = i;
                            I.cpWallVertSel[1] = j;
                        }
                    }
                }
            }
        }

        // ---------- Control point wall vertex selector keys [ALT [LEFT, RIGHT, UP, DOWN]] ----------

        else if (mods & GLFW_MOD_ALT)
        {
            if (key == GLFW_KEY_UP)
            {
                // Set row index to top row
                I.cpWallVertSel[0] = 0;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                // Set row index to bottom row
                I.cpWallVertSel[0] = 1;
            }
            else if (key == GLFW_KEY_LEFT)
            {
                // Set column index to first column
                I.cpWallVertSel[1] = 0;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                // Set column index to second column
                I.cpWallVertSel[1] = 1;
            }
        }

        // ---------- Control point translate [SHIFT or no modifier] ----------
        else
        {
            // Set the position increment based on whether the shift key is pressed
            float pos_inc = (mods & GLFW_MOD_SHIFT) ? 0.01f : 0.0005f;

            // Get the maze and wall vertex indices cooresponding to the selected control point
            int mv_ind = I.cpMap[I.cpMazeVertSel[0]][I.cpMazeVertSel[1]];
            int wv_ind = I.cpMap[I.cpWallVertSel[0]][I.cpWallVertSel[1]];

            // Store current origin
            cv::Point2f cp_origin_save = CP_GRID_ARR[mv_ind][I.cpWVOrigin];

            // Listen for arrow key input to move selected control point
            if (key == GLFW_KEY_LEFT)
            {
                CP_GRID_ARR[mv_ind][wv_ind].x -= pos_inc; // Move left
                F.updateWallTexture = true;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                CP_GRID_ARR[mv_ind][wv_ind].x += pos_inc; // Move right
                F.updateWallTexture = true;
            }
            else if (key == GLFW_KEY_UP)
            {
                CP_GRID_ARR[mv_ind][wv_ind].y -= pos_inc; // Move up
                F.updateWallTexture = true;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                CP_GRID_ARR[mv_ind][wv_ind].y += pos_inc; // Move down
                F.updateWallTexture = true;
            }

            // Shift all control points if origin moved
            cv::Point2f cp_origin_new = CP_GRID_ARR[mv_ind][I.cpWVOrigin];

            // Calculate the change in x and y for the origin
            float delta_x = cp_origin_new.x - cp_origin_save.x;
            float delta_y = cp_origin_new.y - cp_origin_save.y;

            // Check if the origin vertex of the wall was moved (e.g., bottom-left)
            if (wv_ind == I.cpWVOrigin && (delta_x > 0.0f || delta_y > 0.0f))
            {
                // Update all other vertices based on the change in the origin
                for (int i = 0; i < 4; ++i) // Assuming there are 4 vertices
                {
                    if (i != I.cpWVOrigin) // Skip the origin vertex itself
                    {
                        CP_GRID_ARR[mv_ind][i].x += delta_x;
                        CP_GRID_ARR[mv_ind][i].y += delta_y;
                    }
                }
            }
        }
    }
}

void initCtrlPtCoords(std::array<std::array<cv::Point2f, 4>, 4> &out_CP_GRID_ARR)
{
    // Specify the control point limits
    float cp_x = MAZE_WIDTH_NDC / 2;        // starting X-coordinate in NDC coordinates
    const float cp_y = MAZE_HEIGHT_NDC / 2; // starting Y-coordinate in NDC coordinates

    // Add an x offset based on the calibration mode
    float offset_x = 0.5f * static_cast<float>(MAZE_WIDTH_NDC);
    cp_x += (I.calMode == 0) ? -offset_x : (I.calMode == 2) ? offset_x
                                                            : 0;

    // Iterate through control point outer array (maze vertices)
    for (float mv_i = 0; mv_i < 4; mv_i++) // image bottom to top
    {
        cv::Point2f p_org;

        // 0: image top-left
        if (mv_i == 0)
            p_org = cv::Point2f(-cp_x, -cp_y);

        // 1: image top-right
        else if (mv_i == 1)
            p_org = cv::Point2f(+cp_x, -cp_y);

        // 2: image bottom-right
        else if (mv_i == 2)
            p_org = cv::Point2f(+cp_x, +cp_y);

        // 3: image bottom-left
        else if (mv_i == 3)
            p_org = cv::Point2f(-cp_x, +cp_y);

        // Set x y values for each wall vertex
        out_CP_GRID_ARR[mv_i] = {
            cv::Point2f(p_org.x, p_org.y),                                                // top left
            cv::Point2f(p_org.x + WALL_IMAGE_WIDTH_NDC, p_org.y),                         // top right
            cv::Point2f(p_org.x + WALL_IMAGE_WIDTH_NDC, p_org.y + WALL_IMAGE_HEIGHT_NDC), // bottom right
            cv::Point2f(p_org.x, p_org.y + WALL_IMAGE_HEIGHT_NDC),                        // bottom left
        };
    }
}

int loadImgMat(const std::vector<std::string> &img_paths_vec, std::vector<cv::Mat> &out_img_mat_vec)
{
    out_img_mat_vec.clear(); // Ensure the output vector is empty before starting

    for (const std::string &img_path : img_paths_vec)
    {
        // Load image using OpenCV
        cv::Mat img = cv::imread(img_path, cv::IMREAD_UNCHANGED);

        // Check if image is loaded successfully
        if (img.empty())
        {
            ROS_ERROR("[loadImgMat] Failed to load image from path: %s", img_path.c_str());
            return -1;
        }

        // Check if image has an alpha channel
        if (img.channels() != 4)
        {
            ROS_ERROR("[loadImgMat] Image does not have an alpha channel: %s", img_path.c_str());
            return -1;
        }

        // Check if image dimensions are as expected
        if (img.cols != WALL_IMAGE_WIDTH_PXL || img.rows != WALL_IMAGE_HEIGHT_PXL)
        {
            ROS_ERROR("[loadImgMat] Image dimensions do not match expected size: %s", img_path.c_str());
            return -1;
        }

        // Determine depth
        std::string depth_str;
        switch (img.depth())
        {
        case CV_8U:
            depth_str = "CV_8U";
            break;
        case CV_8S:
            depth_str = "CV_8S";
            break;
        case CV_16U:
            depth_str = "CV_16U";
            break;
        case CV_16S:
            depth_str = "CV_16S";
            break;
        case CV_32S:
            depth_str = "CV_32S";
            break;
        case CV_32F:
            depth_str = "CV_32F";
            break;
        case CV_64F:
            depth_str = "CV_64F";
            break;
        default:
            depth_str = "Unknown";
            break;
        }

        // Store the loaded image in the output vector
        out_img_mat_vec.push_back(img);

        // Log the image information
        ROS_INFO("[loadImgMat] Successfully loaded image: Channels[%d] Depth[%s] Path[%s]",
                 img.channels(), depth_str.c_str(), img_path.c_str());
    }

    // Return success
    return 0;
}

int mergeImgMat(const cv::Mat &mask_img, cv::Mat &out_base_img)
{
    // Check if images are loaded successfully
    if (out_base_img.empty() || mask_img.empty())
    {
        ROS_ERROR("[mergeImgMat] Error: Could not read one or both images.");
        return -1;
    }

    // Check dimensions
    if (out_base_img.size() != mask_img.size())
    {
        ROS_ERROR("[mergeImgMat] Error: Image dimensions do not match. "
                  "Base image(%d, %d), Mask image(%d, %d)",
                  out_base_img.cols, out_base_img.rows,
                  mask_img.cols, mask_img.rows);
        return -1;
    }

    // Loop through each pixel
    for (int y = 0; y < out_base_img.rows; ++y)
    {
        for (int x = 0; x < out_base_img.cols; ++x)
        {
            const cv::Vec4b &base_pixel = out_base_img.at<cv::Vec4b>(y, x);
            const cv::Vec4b &mask_pixel = mask_img.at<cv::Vec4b>(y, x);

            // If the alpha channel of the mask pixel is not fully transparent, overlay it
            if (mask_pixel[3] != 0)
            {
                out_base_img.at<cv::Vec4b>(y, x) = mask_pixel;
            }
        }
    }

    return 0;
}

int initCircleRendererObjects(const std::array<std::array<cv::Point2f, 4>, 4> &_CP_GRID_ARR,
                              std::array<std::array<CircleRenderer, 4>, 4> &out_CP_RENDERERS)
{
    // Iterate through control point outer array (maze vertices)
    for (float mv_i = 0; mv_i < 4; mv_i++) // image bottom to top
    {
        // Iterate through control point inner array (wall vertices)
        for (int wv_i = 0; wv_i < 4; ++wv_i)
        {
            out_CP_RENDERERS[mv_i][wv_i].initializeCircleAttributes(
                _CP_GRID_ARR[mv_i][wv_i], // position
                cpDefualtMakerRadius,     // radius
                cpDefaultRGB,             // color
                cpRenderSegments          // segments
            );
        }
    }

    // Return GL status
    return MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__, "initCircleRendererObjects");
}

int renderControlPoints(const std::array<std::array<cv::Point2f, 4>, 4> &_CP_GRID_ARR,
                        std::array<std::array<CircleRenderer, 4>, 4> &out_CP_RENDERERS)
{
    // Setup the CircleRenderer class shaders
    CircleRenderer::SetupShader();

    // Loop through the control points and draw them
    for (int mv_i = 0; mv_i < 4; ++mv_i)
    {
        for (int wv_i = 0; wv_i < 4; ++wv_i)
        {
            // Get the maze and wall vertex indices cooresponding to the selected control point
            int mv_ind = I.cpMap[I.cpMazeVertSel[0]][I.cpMazeVertSel[1]];
            int wv_ind = I.cpMap[I.cpWallVertSel[0]][I.cpWallVertSel[1]];

            // Define the marker color
            cv::Scalar col = (mv_i == mv_ind && wv_i == wv_ind) ? cpWallVertSelectedRGB : (mv_i == mv_ind) ? cpMazeVertSelectedRGB
                                                                                                           : cpDefaultRGB;

            // Define the marker radius
            GLfloat rad = wv_i == 3 ? cpSelectedMakerRadius : cpDefualtMakerRadius;

            // Set the marker parameters
            out_CP_RENDERERS[mv_i][wv_i].setPosition(_CP_GRID_ARR[mv_i][wv_i]);
            out_CP_RENDERERS[mv_i][wv_i].setRadius(rad);
            out_CP_RENDERERS[mv_i][wv_i].setColor(col);

            // Recompute the marker parameters
            out_CP_RENDERERS[mv_i][wv_i].recomputeParameters();

            // Draw the marker
            out_CP_RENDERERS[mv_i][wv_i].draw();

            // Check for errors
            if (MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__) < 0)
            {
                ROS_ERROR("[renderControlPoints] Error Thrown for Control Point[%d][%d]", mv_i, wv_i);
                return -1;
            }
        }
    }

    // Unset the shader program
    CircleRenderer::UnsetShader();

    // Return GL status
    return MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__, "renderControlPoints");
}

int main(int argc, char **argv)
{

    //  _______________ SETUP _______________

    // ROS Initialization
    ros::init(argc, argv, "projection_calibration", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    ROS_INFO("RUNNING MAIN");

    // Log setup parameters
    ROS_INFO("[main] Config XML Path: %s", CONFIG_DIR_PATH.c_str());
    ROS_INFO("[main] Display: Width[%d] Height[%d] AR[%0.2f]", WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL, WINDOW_ASPECT_RATIO);
    ROS_INFO("[main] Wall (Pxl): Width[%d] Height[%d]", WALL_IMAGE_WIDTH_PXL, WALL_IMAGE_HEIGHT_PXL);
    ROS_INFO("[main] Wall (NDC): Width[%0.2f] Height[%0.2f] Space Horz[%0.2f] Space Vert[%0.2f]", WALL_IMAGE_WIDTH_NDC, WALL_IMAGE_HEIGHT_NDC);
    ROS_INFO("[main] Origin Plane (NDC): Width[%0.2f] Height[%0.2f]", WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL);

    // --------------- VARIABLE SETUP ---------------

    // Initialize control point coordinate dataset
    initCtrlPtCoords(CP_GRID_ARR);

    // Initialize wall homography matrices array
    if (updateHomography(CP_GRID_ARR, HMAT_GRID_ARR) < 0)
    {
        ROS_ERROR("[main] Failed to initialize wall parameters");
        return -1;
    }

    // --------------- OpenGL SETUP ---------------

    // Declare MazeRenderContext instance
    std::array<MazeRenderContext, 1> RenContx;

    // Initialize GLFW and OpenGL settings
    if (MazeRenderContext::SetupGraphicsLibraries(N.monitors) < 0)
    {
        ROS_ERROR("[main] Failed to initialize graphics");
        return -1;
    }

    // Initialze render context
    /// @note this is where we would loop through our windows if there were more
    if (RenContx[0].initContext(0, 0, callbackKeyBinding) < 0)
    {
        ROS_ERROR("[main] Failed to initialize render context");
        return -1;
    }

    // Initialize OpenGL wall image objects
    if (initWallRenderObjects(RenContx[0],
                              WALL_GL_VERTICES, sizeof(WALL_GL_VERTICES),
                              WALL_GL_INDICES, sizeof(WALL_GL_INDICES)) < 0)
    {
        ROS_ERROR("[main] Failed to initialize opengl wall image objects");
        return -1;
    }

    // Update monitor and window mode settings
    /// @note consider moving into class setup
    if (RenContx[0].switchWindowMode(I.winMon, F.setFullscreen) < 0)
    {
        ROS_ERROR("[main] Failed Initial update of window monitor mode");
        return -1;
    }

    // Create the shader program for wall image rendering
    if (RenContx[0].compileAndLinkShaders(wallVertexSource, wallFragmentSource) < 0)
    {
        ROS_ERROR("[main] Failed to compile and link wall shader");
        return -1;
    }

    // Create the shader program for CircleRenderer class control point rendering
    if (CircleRenderer::CompileAndLinkCircleShaders(WINDOW_ASPECT_RATIO) < 0)
    {
        ROS_ERROR("[main] Failed to compile and link circlerenderer class shader");
        return -1;
    }
    // Initialize the CircleRenderer class objects array
    if (initCircleRendererObjects(CP_GRID_ARR, CP_RENDERERS) < 0)
    {
        ROS_ERROR("[main] Failed to initialize control point variables");
        return -1;
    }

    // --------------- RENDER IMAGE LOADING  ---------------

    // Load images using OpenCV
    if (loadImgMat(wallImgPathVec, wallImgMatVec) < 0)
    {
        ROS_ERROR("[main] Failed to load wall images");
        return -1;
    }
    if (loadImgMat(monImgPathVec, monImgMatVec) < 0)
    {
        ROS_ERROR("[main] Failed to load monitor number images");
        return -1;
    }
    if (loadImgMat(calImgPathVec, calImgMatVec) < 0)
    {
        ROS_ERROR("[main] Failed to load calibration mode images");
        return -1;
    }

    // Initialize wall image texture
    if (updateWallTexture(wallImgMatVec[I.wallImage], monImgMatVec[I.winMon], calImgMatVec[I.calMode], HMAT_GRID_ARR, RenContx[0].textureID) < 0)
    {
        ROS_ERROR("[main] Failed to initialize wall texture");
        return -1;
    }

    // _______________ MAIN LOOP _______________

    int status = 0;
    while (!glfwWindowShouldClose(RenContx[0].windowID) && ros::ok())
    {

        // --------------- Check Kayboard Callback Flags ---------------

        // Load/save XML file
        if (F.loadXML || F.saveXML)
        {
            std::string file_path_hmat = frmtFilePathxml(0, I.winMon, I.calMode, CONFIG_DIR_PATH);
            std::string file_path_cp = frmtFilePathxml(1, I.winMon, I.calMode, CONFIG_DIR_PATH);

            // Save XML file
            if (F.saveXML)
            {
                status = saveHMATxml(file_path_hmat, HMAT_GRID_ARR);
                status = saveCPxml(file_path_cp, CP_GRID_ARR);
            }
            F.saveXML = false;

            // Load XML file
            if (F.loadXML)
            {
                status = loadHMATxml(file_path_hmat, HMAT_GRID_ARR);
                status = loadCPxml(file_path_cp, CP_GRID_ARR);
                F.updateWallTexture = true;
            }
            F.loadXML = false;

            // Flash the background to indicate the file was loaded/saved
            if (status >= 0)
                RenContx[0].flashBackgroundColor(cv::Scalar(0.0f, 0.25f, 0.0f), 500);
            else
            {
                RenContx[0].flashBackgroundColor(cv::Scalar(0.25f, 0.0f, 0.0f), 500);
                ROS_ERROR("[main] Failed to load/save XML file");
                break;
            }
        }

        // Update the window monitor and mode
        if (F.switchWindowMode)
        {
            if (RenContx[0].switchWindowMode(I.winMon, F.setFullscreen) < 0)
            {
                ROS_ERROR("[main] Update window monitor mode threw an error");
                status = -1;
                break;
            }
            F.switchWindowMode = false;
        }

        // Initialize/reinitialize control point coordinate dataset
        if (F.initControlPointMarkers)
        {
            initCtrlPtCoords(CP_GRID_ARR);
            F.initControlPointMarkers = false;
            F.updateWallTexture = true;
        }

        // Recompute wall parameters and update wall image texture
        if (F.updateWallTexture)
        {
            // Update wall homography matrices array
            if (updateHomography(CP_GRID_ARR, HMAT_GRID_ARR) < 0)
            {
                ROS_ERROR("[main] Update of wall vertices datasets failed");
                status = -1;
                break;
            }

            // Update wall image texture
            if (updateWallTexture(wallImgMatVec[I.wallImage], monImgMatVec[I.winMon], calImgMatVec[I.calMode], HMAT_GRID_ARR, RenContx[0].textureID) < 0)
            {
                ROS_ERROR("[main] Update of wall homography datasets failed");
                status = -1;
                break;
            }
            F.updateWallTexture = false;
        }

        // --------------- Handle Image Processing for Next Frame ---------------

        // Clear back buffer for new frame
        glClear(GL_COLOR_BUFFER_BIT);
        if (MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__, "[main] Error flagged following glclear()"))
        {
            status = -1;
            break;
        }

        // Make sure winsow always stays on top in fullscreen mode
        RenContx[0].setWindowStackOrder(F.setFullscreen);

        // Draw/update wall images
        if (renderWallImage(RenContx[0]) < 0)
        {
            ROS_ERROR("[main] Draw walls threw an error");
            status = -1;
            break;
        }

        // Draw/update control point markers
        if (renderControlPoints(CP_GRID_ARR, CP_RENDERERS) < 0)
        {
            ROS_ERROR("[main] Draw control point threw an error");
            status = -1;
            break;
        }

        // Swap buffers and poll events
        glfwSwapBuffers(RenContx[0].windowID);
        if (MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__, "[main] Error flagged following glfwSwapBuffers()") < 0)
        {
            status = -1;
            break;
        }

        // Poll events
        glfwPollEvents();

        // Exit condition
        if (glfwGetKey(RenContx[0].windowID, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(RenContx[0].windowID))
            break;
    }

    // _______________ CLEANUP _______________
    ROS_INFO("SHUTTING DOWN");

    // Check which condition caused the loop to exit
    if (!ros::ok())
        ROS_INFO("[main] Loop Terminated:  ROS node is no longer in a good state");
    else if (glfwWindowShouldClose(RenContx[0].windowID))
        ROS_INFO("[main] Loop Terminated:  GLFW window should close");
    else if (glfwGetKey(RenContx[0].windowID, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        ROS_INFO("[main] Loop Terminated:  Escape key was pressed");
    else if (status < 0)
        ROS_INFO("[main] Loop Terminated:  Error thrown");
    else
        ROS_INFO("[main] Loop Terminated:  Reason unknown");

    // Delete CircleRenderer class shader program
    if (CircleRenderer::CleanupClassResources() < 0)
        ROS_WARN("[main] Failed to delete CircleRenderer shader program");
    else
        ROS_INFO("[main] CircleRenderer shader program deleted successfully");

    // Clean up OpenGL wall image objects
    if (RenContx[0].cleanupContext() != 0)
        ROS_WARN("[main] Error during cleanup of MazeRenderContext instance");
    else
        ROS_INFO("[main] MazeRenderContext instance cleaned up successfully");

    // Terminate graphics
    if (MazeRenderContext::CleanupGraphicsLibraries() < 0)
        ROS_WARN("[main] Failed to terminate GLFW library");
    else
        ROS_INFO("[main] GLFW library terminated successfully");

    // Return success
    return status;
}
