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
            isFullScreen = !isFullScreen;
            F.updateWindowMonMode = true;
        }

        // Move the window to another monitor
        else if (key == GLFW_KEY_0)
        {
            winMonInd = 0;
            F.updateWindowMonMode = true;
        }
        else if (key == GLFW_KEY_1 && nMonitors > 1)
        {
            winMonInd = 1;
            F.updateWindowMonMode = true;
        }
        else if (key == GLFW_KEY_2 && nMonitors > 2)
        {
            winMonInd = 2;
            F.updateWindowMonMode = true;
        }
        else if (key == GLFW_KEY_3 && nMonitors > 3)
        {
            winMonInd = 3;
            F.updateWindowMonMode = true;
        }
        else if (key == GLFW_KEY_4 && nMonitors > 4)
        {
            winMonInd = 4;
            F.updateWindowMonMode = true;
        }
        else if (key == GLFW_KEY_5 && nMonitors > 5)
        {
            winMonInd = 5;
            F.updateWindowMonMode = true;
        }

        // ---------- XML Handling [ENTER, L] ----------

        // Save coordinates to XML
        else if (key == GLFW_KEY_ENTER)
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
            imgWallInd = (int)imgWallPathVec.size() > 0 ? 0 : imgWallInd;
        }
        else if (key == GLFW_KEY_F2)
        {
            imgWallInd = (int)imgWallPathVec.size() > 1 ? 1 : imgWallInd;
        }
        else if (key == GLFW_KEY_F3)
        {
            imgWallInd = (int)imgWallPathVec.size() > 2 ? 2 : imgWallInd;
        }
        else if (key == GLFW_KEY_F4)
        {
            imgWallInd = (int)imgWallPathVec.size() > 3 ? 3 : imgWallInd;
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
                calModeInd = (calModeInd > 0) ? calModeInd - 1 : (int)nCalModes - 1;
                F.initControlPointMarkers = true;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                calModeInd = (calModeInd < nCalModes - 1) ? calModeInd + 1 : 0;
                F.initControlPointMarkers = true;
            }
        }

        // ---------- Contol point wall selector keys [CTRL [LEFT, RIGHT, UP, DOWN]] ----------

        else if (mods & GLFW_MOD_CONTROL)
        {
            if (key == GLFW_KEY_UP)
            {
                // Move to the top row, keeping the horizontal position
                cpWallSelectedInd = (cpWallSelectedInd % 2); // Result will be 0 or 1
            }
            else if (key == GLFW_KEY_DOWN)
            {
                // Move to the bottom row, keeping the horizontal position
                cpWallSelectedInd = 2 + (cpWallSelectedInd % 2); // Result will be 2 or 3
            }
            else if (key == GLFW_KEY_LEFT)
            {
                // Move to the left column, keeping the vertical position
                cpWallSelectedInd = (cpWallSelectedInd >= 2) ? 2 : 0; // Result will be 0 or 2
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                // Move to the right column, keeping the vertical position
                cpWallSelectedInd = (cpWallSelectedInd >= 2) ? 3 : 1; // Result will be 1 or 3
            }
        }

        // ---------- Contol point vertex selector keys [ALT [LEFT, RIGHT, UP, DOWN]] ----------

        else if (mods & GLFW_MOD_ALT)
        {
            if (key == GLFW_KEY_UP)
            {
                // Move to the top row, keeping the horizontal position
                cpVertSelectedInd = (cpVertSelectedInd % 2); // Result will be 0 or 1
            }
            else if (key == GLFW_KEY_DOWN)
            {
                // Move to the bottom row, keeping the horizontal position
                cpVertSelectedInd = 2 + (cpVertSelectedInd % 2); // Result will be 2 or 3
            }
            else if (key == GLFW_KEY_LEFT)
            {
                // Move to the left column, keeping the vertical position
                cpVertSelectedInd = (cpVertSelectedInd >= 2) ? 2 : 0; // Result will be 0 or 2
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                // Move to the right column, keeping the vertical position
                cpVertSelectedInd = (cpVertSelectedInd >= 2) ? 3 : 1; // Result will be 1 or 3
            }
        }

        // ---------- Control point translate [SHIFT or no modifier] ----------
        else
        {
            // Set the position increment based on whether the shift key is pressed
            float pos_inc = (mods & GLFW_MOD_SHIFT) ? 0.01f : 0.0005f;

            // Store current origin
            cv::Point2f cp_origin_save = CTRL_PNT_DATA[cpWallSelectedInd][2];

            // Listen for arrow key input to move selected control point
            if (key == GLFW_KEY_LEFT)
            {
                CTRL_PNT_DATA[cpWallSelectedInd][cpVertSelectedInd].x -= pos_inc; // Move left
                F.updateWallDatasets = true;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                CTRL_PNT_DATA[cpWallSelectedInd][cpVertSelectedInd].x += pos_inc; // Move right
                F.updateWallDatasets = true;
            }
            else if (key == GLFW_KEY_UP)
            {
                CTRL_PNT_DATA[cpWallSelectedInd][cpVertSelectedInd].y += pos_inc; // Move up
                F.updateWallDatasets = true;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                CTRL_PNT_DATA[cpWallSelectedInd][cpVertSelectedInd].y -= pos_inc; // Move down
                F.updateWallDatasets = true;
            }

            // Shift all control points if origin moved
            cv::Point2f cp_origin_new = CTRL_PNT_DATA[cpWallSelectedInd][2];

            // Calculate the change in x and y for the origin
            float delta_x = cp_origin_new.x - cp_origin_save.x;
            float delta_y = cp_origin_new.y - cp_origin_save.y;

            // Check if the origin vertex was moved
            if (cpVertSelectedInd == 2)
            {
                // Update all other vertices based on the change in the origin
                for (int i = 0; i < 4; ++i) // Assuming there are 4 vertices
                {
                    if (i != 2) // Skip the origin vertex itself
                    {
                        CTRL_PNT_DATA[cpWallSelectedInd][i].x += delta_x;
                        CTRL_PNT_DATA[cpWallSelectedInd][i].y += delta_y;
                    }
                }
            }
        }
    }
}

void callbackFrameBufferSizeGLFW(GLFWwindow *window, int width, int height)
{
    glViewport(0, 0, width, height);
    checkErrorOpenGL(__LINE__, __FILE__);
}

static void callbackErrorOpenGL(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message, const void *userParam)
{
    ROS_ERROR("[OpenGL ERROR CALLBACK] Type[0x%x] ID[%d] Severity[0x%x] Message[%s]", type, id, severity, message);
}

static void callbackErrorGLFW(int error, const char *description)
{
    ROS_ERROR("[GLFW ERROR CALLBACK] Error[%d] Description[%s]", error, description);
}

int checkErrorOpenGL(int line, const char *file_str, const char *msg_str)
{
    GLenum gl_err;
    while ((gl_err = glGetError()) != GL_NO_ERROR)
    {
        if (msg_str)
            ROS_INFO("[OpenGL ERROR CHECK] Message[%s] Error Number[%u] File[%s] Line[%d]", msg_str, gl_err, file_str, line);
        else
            ROS_INFO("[OpenGL ERROR CHECK] Error Number[%u] File[%s] Line[%d]", gl_err, file_str, line);
        return -1;
    }
    return 0;
}

int checkErrorGLFW(int line, const char *file_str, const char *msg_str)
{
    const char *description;
    int glfw_err = glfwGetError(&description);
    if (glfw_err != GLFW_NO_ERROR)
    {
        if (msg_str)
            ROS_ERROR("[GLFW ERROR CHECK] Message[%s] Description[%s] File[%s] Line[%d]", msg_str, description, file_str, line);
        else
            ROS_ERROR("[GLFW ERROR CHECK] Description[%s] File[%s] Line[%d]", description, file_str, line);
        return -1;
    }
    return 0;
}

int updateWindowMonMode(GLFWwindow *p_window_id, int win_ind, GLFWmonitor **&pp_r_monitor_id, int mon_id_ind, bool is_fullscreen)
{
    static int imp_mon_id_ind_last = mon_id_ind;
    static bool is_fullscreen_last = !is_fullscreen;

    // Check if monitor or fullscreen mode has changed
    if (imp_mon_id_ind_last == mon_id_ind && is_fullscreen_last == is_fullscreen)
    {
        return 0;
    }

    // Get GLFWmonitor for active monitor
    GLFWmonitor *p_monitor_id = pp_r_monitor_id[mon_id_ind];

    // Update window size and position
    if (p_monitor_id)
    {
        // Get the video mode of the selected monitor
        const GLFWvidmode *mode = glfwGetVideoMode(p_monitor_id);
        if (!mode)
        {
            ROS_ERROR("[WIN MODE] Failed to Get Video Mode: Monitor[%d]", mon_id_ind);
            return -1;
        }

        // Set the window to full-screen mode on the current monitor
        glfwSetWindowMonitor(p_window_id, p_monitor_id, 0, 0, mode->width, mode->height, mode->refreshRate);
        if (!p_monitor_id)
        {
            ROS_ERROR("[WIN MODE] Invalid Monitor Pointer: Monitor[%d]", mon_id_ind);
            return -1;
        }

        if (!is_fullscreen)
        {
            // Get the position of the current monitor
            int monitor_x, monitor_y;
            glfwGetMonitorPos(p_monitor_id, &monitor_x, &monitor_y);

            // Validate monitor position
            if (monitor_x < 0 || monitor_y < 0)
            {
                ROS_WARN("[WIN MODE] Invalid Monitor Position: Monitor[%d] X[%d] Y[%d]", mon_id_ind, monitor_x, monitor_y);
                return 0;
            }

            // Set the window to windowed mode and position it on the current monitor
            glfwSetWindowMonitor(p_window_id, NULL, monitor_x + 100, monitor_y + 100, (int)(500.0f * PROJ_WIN_ASPECT_RATIO), 500, 0);
        }

        // Update window title
        std::string new_title = "Window[" + std::to_string(win_ind) + "] Monitor[" + std::to_string(mon_id_ind) + "]";
        glfwSetWindowTitle(p_window_id, new_title.c_str());

        ROS_INFO("[WIN MODE] Move Window: Monitor[%d] Format[%s]", mon_id_ind, is_fullscreen ? "fullscreen" : "windowed");
    }
    else
    {
        ROS_WARN("[WIN MODE] Failed Move Window: Monitor[%d] Format[%s]", mon_id_ind, is_fullscreen ? "fullscreen" : "windowed");
        return 0;
    }

    // Update last monitor and fullscreen mode
    imp_mon_id_ind_last = mon_id_ind;
    is_fullscreen_last = is_fullscreen;

    return 0;
}

int drawColoredCircle(float x, float y, float radius, std::array<float, 3> rgb_arr)
{
    const int segments = 100; // Number of segments to approximate a circle

    // Begin drawing a filled circle
    glBegin(GL_TRIANGLE_FAN);

    // Set the color to green
    glColor3f(rgb_arr[0], rgb_arr[1], rgb_arr[2]);

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

    // Return GL status
    return checkErrorOpenGL(__LINE__, __FILE__);
}

int updateControlPointMarkers()
{

    // Itterate through control points
    for (int cp_i = 0; cp_i < 4; cp_i++)
    {
        // Itterate through verteces
        for (int v_i = 0; v_i < 4; v_i++)
        {
            float cp_rad = cpMakerRadius[0];
            std::array<float, 3> cp_col = cpUnelectedRGB;

            // Set color based on cp selected
            if (cp_i == cpWallSelectedInd)
            {
                if (cpVertSelectedInd == v_i)
                {
                    cp_col = cpVertSelectedRGB;
                }
                else
                    cp_col = cpWallSelectedRGB;
            }

            // Make marker size larger for control point origin/anchor
            if (v_i == 2)
                cp_rad = cpMakerRadius[1];

            // Get the control point coordinates
            cv::Point2f p_cp = CTRL_PNT_DATA[cp_i][v_i];

            // Draw the control point
            if (drawColoredCircle(p_cp.x, p_cp.y, cp_rad, cp_col) != 0)
            {
                ROS_ERROR("[MAIN] Draw Control Point Threw Error");
                return -1;
            }
        }
    }
    return 0;
}

int drawQuadImage(std::array<cv::Point2f, 4> quad_vertices_arr)
{
    // Start drawing a quadrilateral
    glBegin(GL_QUADS);

    // Set the color to white (for texture mapping)
    /// @note: this is necessary when drawing the control points
    glColor3f(1.0f, 1.0f, 1.0f);

    // Top-left corner of texture
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(quad_vertices_arr[0].x, quad_vertices_arr[0].y);

    // Top-right corner of texture
    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(quad_vertices_arr[1].x, quad_vertices_arr[1].y);

    // Bottom-right corner of texture
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(quad_vertices_arr[3].x, quad_vertices_arr[3].y);

    // Bottom-left corner of texture
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(quad_vertices_arr[2].x, quad_vertices_arr[2].y);

    // End drawing
    glEnd();

    // Check and return GL status
    return checkErrorOpenGL(__LINE__, __FILE__);
}

int drawBarycentricImage(std::array<cv::Point2f, 4> quad_vertices_arr)
{
    // Divide the quadrilateral into two triangles
    // Triangle 1: [0, 1, 2]
    // Triangle 2: [2, 3, 0]

    // Start drawing the triangles
    glBegin(GL_TRIANGLES);

    // Set the color to white for texture mapping
    glColor3f(1.0f, 1.0f, 1.0f);

    // --------------- First Triangle -----------------
    // Triangle formed by vertices [0, 1, 2]

    // Top-left corner of texture (Vertex 0)
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(quad_vertices_arr[0].x, quad_vertices_arr[0].y);

    // Top-right corner of texture (Vertex 1)
    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(quad_vertices_arr[1].x, quad_vertices_arr[1].y);

    // Bottom-left corner of texture (Vertex 2)
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(quad_vertices_arr[2].x, quad_vertices_arr[2].y);

    // --------------- Second Triangle -----------------
    // Triangle formed by vertices [2, 1, 3]

    // Bottom-left corner of texture (Vertex 2)
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(quad_vertices_arr[2].x, quad_vertices_arr[2].y);

    // Top-right corner of texture (Vertex 1)
    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(quad_vertices_arr[1].x, quad_vertices_arr[1].y);

    // Bottom-right corner of texture (Vertex 3)
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(quad_vertices_arr[3].x, quad_vertices_arr[3].y);

    // End drawing
    glEnd();

    // Check and return GL status
    return checkErrorOpenGL(__LINE__, __FILE__);
}

int drawWallImages(GLuint fbo_texture_id, ILuint tex_wall_id, ILuint tex_mode_mon_id, ILuint tex_mode_cal_id)
{
    // Enable OpenGL texture mapping
    glEnable(GL_TEXTURE_2D);

    // Iterate through the maze grid rows
    for (float grow_i = 0; grow_i < MAZE_SIZE; grow_i++) // image bottom to top
    {
        // Iterate through each column in the maze row
        for (float gcol_i = 0; gcol_i < MAZE_SIZE; gcol_i++) // image left to right
        {
            // Create a copy of the wall image
            ILuint copy_tex_wall_id;
            ilBindImage(tex_wall_id);
            ilGenImages(1, &copy_tex_wall_id);
            ilBindImage(copy_tex_wall_id);
            ilCopyImage(tex_wall_id);

            //  Create merged image for the wall corresponding to the selected control point
            if (
                (cpWallSelectedInd == 0 && grow_i == 0 && gcol_i == 0) ||
                (cpWallSelectedInd == 1 && grow_i == 0 && gcol_i == MAZE_SIZE - 1) ||
                (cpWallSelectedInd == 2 && grow_i == MAZE_SIZE - 1 && gcol_i == 0) ||
                (cpWallSelectedInd == 3 && grow_i == MAZE_SIZE - 1 && gcol_i == MAZE_SIZE - 1))
            {
                // // Merge test pattern and active monitor image
                // if (textureMerge(tex_mode_mon_id, copy_tex_wall_id) != 0)
                //     return -1;

                // // Merge previous image and active calibration image
                // if (textureMerge(tex_mode_cal_id, copy_tex_wall_id) != 0)
                //     return -1;
            }
            if (checkErrorDevIL(__LINE__, __FILE__) != 0)
            {
                ilDeleteImages(1, &copy_tex_wall_id);
                return -1;
            }

            // Get warped vertices for this wall
            std::array<cv::Point2f, 4> quad_vertices_warped = WALL_VERT_DATA[grow_i][gcol_i];

            // Get homography matrix for this wall's texture
            cv::Mat _HMAT = WALL_HMAT_DATA[grow_i][gcol_i];

            // Warp the texture
            if (textureWarp(_HMAT, quad_vertices_warped, copy_tex_wall_id) != 0)
                return -1;

            // Bind the image
            ilBindImage(copy_tex_wall_id); // show test pattern
            if (checkErrorDevIL(__LINE__, __FILE__) != 0)
            {
                ilDeleteImages(1, &copy_tex_wall_id);
                return -1;
            }

            // Set texture image
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
                         ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB,
                         GL_UNSIGNED_BYTE, ilGetData());

            // Bind texture to framebuffer object
            glBindTexture(GL_TEXTURE_2D, fbo_texture_id);

            // Delete the texture
            ilDeleteImages(1, &copy_tex_wall_id);

            // TEMP
            if (gcol_i != 0 || grow_i != 0)
                continue;

            // Draw the wall
            if (drawQuadImage(quad_vertices_warped) != 0)
                return -1;
        }
    }

    // Disable OpenGL texture mapping
    glDisable(GL_TEXTURE_2D);

    // Return GL status
    return checkErrorOpenGL(__LINE__, __FILE__);
}

void testFindHomography()
{
    // Initialize ROS node
    ros::NodeHandle nh;

    // 1. Test with a more substantial transformation
    ROS_INFO("Test 1: More substantial transformation");

    std::vector<cv::Point2f> origin_vertices1 = {
        cv::Point2f(0.0f, 0.2f),
        cv::Point2f(0.1f, 0.2f),
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(0.1f, 0.0f)};

    std::vector<cv::Point2f> target_vertices1;
    for (const auto &point : origin_vertices1)
    {
        target_vertices1.push_back(cv::Point2f(point.x + 0.05f, point.y + 0.05f));
    }

    cv::Mat H1 = cv::findHomography(origin_vertices1, target_vertices1);

    ROS_INFO("[Test 1] Homography Matrix:");
    ROS_INFO("[%.2f, %.2f, %.2f]", H1.at<double>(0, 0), H1.at<double>(0, 1), H1.at<double>(0, 2));
    ROS_INFO("[%.2f, %.2f, %.2f]", H1.at<double>(1, 0), H1.at<double>(1, 1), H1.at<double>(1, 2));
    ROS_INFO("[%.2f, %.2f, %.2f]", H1.at<double>(2, 0), H1.at<double>(2, 1), H1.at<double>(2, 2));

    // 2. Test with a synthetic example
    ROS_INFO("Test 2: Synthetic example with known homography");

    std::vector<cv::Point2f> origin_vertices2 = {
        cv::Point2f(0.0f, 0.2f),
        cv::Point2f(0.1f, 0.2f),
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(0.1f, 0.0f)};

    // Apply a known homography (identity with a translation of 0.1 in x)
    std::vector<cv::Point2f> target_vertices2;
    for (const auto &point : origin_vertices2)
    {
        target_vertices2.push_back(cv::Point2f(point.x + 0.1f, point.y));
    }

    cv::Mat H2 = cv::findHomography(origin_vertices2, target_vertices2);

    ROS_INFO("[Test 2] Homography Matrix:");
    ROS_INFO("[%.2f, %.2f, %.2f]", H2.at<double>(0, 0), H2.at<double>(0, 1), H2.at<double>(0, 2));
    ROS_INFO("[%.2f, %.2f, %.2f]", H2.at<double>(1, 0), H2.at<double>(1, 1), H2.at<double>(1, 2));
    ROS_INFO("[%.2f, %.2f, %.2f]", H2.at<double>(2, 0), H2.at<double>(2, 1), H2.at<double>(2, 2));
}

// int main(int argc, char **argv)
// {

//     //  _______________ SETUP _______________

//     // ROS Initialization
//     ros::init(argc, argv, "projection_calibration", ros::init_options::AnonymousName);
//     ros::NodeHandle n;
//     ros::NodeHandle nh("~");

//     // --------------- VARIABLE SETUP ---------------

//     // Sprecify window resolution: 4K resolution (3840x2160)
//     const int win_height_pxl = 1000;
//     const int win_width_pxl = 1000;
//     const float win_aspect_ratio = (float)win_width_pxl / (float)win_height_pxl;

//     // Image size (pixels)
//     const int img_width_pxl = 300;
//     const int img_height_pxl = 540;

//     // Image size (NDC)
//     const float img_width_ndc = (static_cast<float>(img_width_pxl) / static_cast<float>(win_width_pxl)) * 2;
//     const float img_height_ndc = (static_cast<float>(img_height_pxl) / static_cast<float>(win_height_pxl)) * 2;

//     // --------------- Specify Transform ---------------

//     // Define origin plane vertices
//     std::vector<cv::Point2f> origin_vertices = {
//         cv::Point2f(0.0f, img_height_ndc),          // Top-left
//         cv::Point2f(img_width_ndc, img_height_ndc), // Top-right
//         cv::Point2f(0.0f, 0.0f),                    // Bottom-left
//         cv::Point2f(img_width_ndc, 0.0f)};          // Bottom-right

//     // Center the image
//     for (int i = 0; i < 4; ++i)
//     {
//         origin_vertices[i].x -= img_width_ndc / 2.0f;
//         origin_vertices[i].y -= img_height_ndc / 2.0f;
//     }

//     // Copy the origin plane vertices
//     std::vector<cv::Point2f> target_vertices;
//     for (const auto &point : origin_vertices)
//     {
//         target_vertices.push_back(cv::Point2f(point.x, point.y));
//     }

//     // Warp the target in some way
//     target_vertices[1].x += 0.1f;
//     target_vertices[3].y -= 0.1f;
//     target_vertices[3].x -= 0.1f;

//     // --------------- Convert to pixel coordinates ---------------

//     // Function to convert NDC to Pixel Coordinates
//     auto convertToPixelCoords = [](const cv::Point2f &ndc_point, int img_width, int img_height)
//     {
//         float pixel_x = (ndc_point.x + 1.0f) / 2.0f * img_width;
//         float pixel_y = (ndc_point.y + 1.0f) / 2.0f * img_height;
//         return cv::Point2f(pixel_x, pixel_y);
//     };

//     // Convert origin_vertices to pixel coordinates
//     std::vector<cv::Point2f> origin_vertices_pixel;
//     for (const auto &point : origin_vertices)
//     {
//         origin_vertices_pixel.push_back(convertToPixelCoords(point, img_width_pxl, img_height_pxl));
//     }

//     // Convert target_vertices to pixel coordinates
//     std::vector<cv::Point2f> target_vertices_pixel;
//     for (const auto &point : target_vertices)
//     {
//         target_vertices_pixel.push_back(convertToPixelCoords(point, img_width_pxl, img_height_pxl));
//     }

//     // --------------- OpenGL SETUP ---------------

//     // Initialize GLFW
//     glfwInit();

//     // Create a new GLFW window
//     GLFWwindow *p_window_id = glfwCreateWindow(win_width_pxl, win_height_pxl, "", NULL, NULL);

//     // Set the GLFW window as the current OpenGL context
//     glfwMakeContextCurrent(p_window_id);

//     // Load OpenGL extensions using GLAD
//     gladLoadGL();

//     // Initialize Framebuffer Object (FBO) and its texture
//     GLuint fbo_id = 0;
//     GLuint fbo_texture_id = 0;

//     // Generate an FBO and bind it
//     glGenFramebuffers(1, &fbo_id);
//     glBindFramebuffer(GL_FRAMEBUFFER, fbo_id);

//     // Generate a texture for the FBO
//     glGenTextures(1, &fbo_texture_id);
//     glBindTexture(GL_TEXTURE_2D, fbo_texture_id);

//     // Allocate storage for the texture on the GPU
//     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, win_width_pxl, win_height_pxl, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);

//     // Set the texture's MIN and MAG filter to linear interpolation.
//     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

//     // Attach the texture to the FBO
//     glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_texture_id, 0);

//     // Unbind the FBO (bind to default framebuffer)
//     glBindFramebuffer(GL_FRAMEBUFFER, 0);

//     // Minimize window
//     GLFWmonitor **pp_monitor_id_vec = nullptr;
//     int n_mon;
//     pp_monitor_id_vec = glfwGetMonitors(&n_mon);
//     GLFWmonitor *p_monitor_id = pp_monitor_id_vec[0];

//     // --------------- DevIL SETUP ---------------

//     ilInit();

//     // Load DevIL image
//     ILuint img_id;
//     std::string img_path = "C:/Users/lester/MeDocuments/Research/MadhavLab/CodeBase/omniroute_windows_ws/data/proj_img/calibration_images/1_test_pattern.bmp";
//     ilGenImages(1, &img_id);
//     ilBindImage(img_id);
//     ILboolean success = ilLoadImage(img_path.c_str());
//     if (success == IL_TRUE)
//     {
//         ilConvertImage(IL_RGB, IL_UNSIGNED_BYTE);
//     }
//     else
//     {
//         ROS_ERROR("[MAIN] Failed to Load Image: Path[%s]", img_path.c_str());
//         return -1;
//     }

//     // --------------- Warp the Image ---------------

//     // Find the homography matrix
//     cv::Mat _HMAT = cv::findHomography(origin_vertices_pixel, target_vertices_pixel);

//     // Find the target boundary width and height
//     float max_width = 0.0;
//     float max_height = 0.0;
//     for (int i = 0; i < 4; ++i)
//     {
//         for (int j = i + 1; j < 4; ++j)
//         {
//             float dx = target_vertices[i].x - target_vertices[j].x;
//             float dy = target_vertices[i].y - target_vertices[j].y;
//             max_width = std::max(max_width, std::abs(dx));
//             max_height = std::max(max_height, std::abs(dy));
//         }
//     }
//     int bound_width = static_cast<int>(max_width);
//     int bound_height = static_cast<int>(max_height);
//     cv::Size warped_texture_size(bound_width, bound_height);

//     // Bind the DevIL image
//     ilBindImage(img_id);

//     // Get the dimensions and data of the texture image
//     int source_width = ilGetInteger(IL_IMAGE_WIDTH);
//     int source_height = ilGetInteger(IL_IMAGE_HEIGHT);
//     ILubyte *data = ilGetData();

//     // Create a cv::Mat from the ILuint image
//     cv::Mat source_texture_mat(source_height, source_width, CV_8UC3, data);

//     // Create a cv::Mat to hold the warped texture
//     cv::Mat warped_texture_mat;

//     // Use OpenCV's warpPerspective function to apply the homography matrix to the texture image
//     cv::warpPerspective(
//         source_texture_mat,
//         warped_texture_mat,
//         _HMAT,
//         warped_texture_size);

//     // Update the original image data with the new warped data
//     ilTexImage(
//         warped_texture_mat.cols,
//         warped_texture_mat.rows,
//         1, 3,
//         IL_RGB, IL_UNSIGNED_BYTE,
//         warped_texture_mat.data);
//     if (checkErrorDevIL(__LINE__, __FILE__) != 0)
//     {
//         ROS_ERROR("[WARP TEX] Error Updating Warped Texture: ID[%u]", img_id);
//         return -1;
//     }

//     // Unbind the image before exiting
//     ilBindImage(0);

//     // --------------- Draw the image ---------------

//     // Show the image
//     while (!glfwWindowShouldClose(p_window_id) && ros::ok())
//     {
//         glClear(GL_COLOR_BUFFER_BIT);

//         // Enable OpenGL texture mapping
//         glEnable(GL_TEXTURE_2D);

//         // Bind the image
//         ilBindImage(img_id);

//         // Set texture image
//         glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
//                      ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB,
//                      GL_UNSIGNED_BYTE, ilGetData());

//         // Bind texture to framebuffer object
//         glBindTexture(GL_TEXTURE_2D, fbo_texture_id);

//         // Start drawing a quadrilateral
//         glBegin(GL_QUADS);

//         // Set the color to white (for texture mapping)
//         glColor3f(1.0f, 1.0f, 1.0f);

//         // Top-left corner of texture
//         glTexCoord2f(0.0f, 1.0f);
//         glVertex2f(target_vertices[0].x, target_vertices[0].y);

//         // Top-right corner of texture
//         glTexCoord2f(1.0f, 1.0f);
//         glVertex2f(target_vertices[1].x, target_vertices[1].y);

//         // Bottom-right corner of texture
//         glTexCoord2f(1.0f, 0.0f);
//         glVertex2f(target_vertices[3].x, target_vertices[3].y);

//         // Bottom-left corner of texture
//         glTexCoord2f(0.0f, 0.0f);
//         glVertex2f(target_vertices[2].x, target_vertices[2].y);

//         // End drawing
//         glEnd();

//         // Disable OpenGL texture mapping
//         glDisable(GL_TEXTURE_2D);

//         glfwSwapBuffers(p_window_id);
//         glfwPollEvents();

//         // Exit condition
//         if (glfwGetKey(p_window_id, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(p_window_id))
//             break;
//     }

//     // _______________ CLEANUP _______________

//     ilDeleteImages(1, &img_id);
//     glDeleteFramebuffers(1, &fbo_id);
//     glDeleteTextures(1, &fbo_texture_id);
//     glfwDestroyWindow(p_window_id);
//     p_window_id = nullptr;
//     ilShutDown();
//     glfwTerminate();

//     return 0;
// }

int main(int argc, char **argv)
{
    // ROS Initialization
    ros::init(argc, argv, "projection_calibration");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // --------------- VARIABLE SETUP ---------------

    // Sprecify window resolution: 4K resolution (3840x2160)
    const int win_height_pxl = 1000;
    const int win_width_pxl = 1000;
    const float win_aspect_ratio = (float)win_width_pxl / (float)win_height_pxl;

    // Image size (pixels)
    const int img_width_pxl = 300;
    const int img_height_pxl = 540;

    // Image size (NDC)
    const float img_width_ndc = (static_cast<float>(img_width_pxl) / static_cast<float>(win_width_pxl)) * 2;
    const float img_height_ndc = (static_cast<float>(img_height_pxl) / static_cast<float>(win_height_pxl)) * 2;

    // --------------- Specify Transform ---------------

    // Define origin plane vertices
    std::vector<cv::Point2f> origin_vertices_ndc = {
        cv::Point2f(0.0f, img_height_ndc),          // Top-left
        cv::Point2f(img_width_ndc, img_height_ndc), // Top-right
        cv::Point2f(0.0f, 0.0f),                    // Bottom-left
        cv::Point2f(img_width_ndc, 0.0f)};          // Bottom-right

    // Center the image
    for (int i = 0; i < 4; ++i)
    {
        origin_vertices_ndc[i].x -= img_width_ndc / 2.0f;
        origin_vertices_ndc[i].y -= img_height_ndc / 2.0f;
    }

    // Copy the origin plane vertices
    std::vector<cv::Point2f> target_vertices_ndc;
    for (const auto &point : origin_vertices_ndc)
    {
        target_vertices_ndc.push_back(cv::Point2f(point.x, point.y));
    }

    // Warp the target in some way
    target_vertices_ndc[1].x += 0.1f;
    target_vertices_ndc[3].y -= 0.1f;
    target_vertices_ndc[3].x -= 0.1f;
    target_vertices_ndc[2].x -= 0.1f;
    target_vertices_ndc[0].y += 0.1f;

    // --------------- OpenGL SETUP ---------------

    // Initialize GLFW
    if (!glfwInit())
    {
        ROS_ERROR("Failed to initialize GLFW");
        return -1;
    }

    // Create a GLFW window
    GLFWwindow *window = glfwCreateWindow(win_height_pxl, win_width_pxl, "Projection Calibration", NULL, NULL);
    if (window == NULL)
    {
        ROS_ERROR("Failed to create GLFW window");
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Load OpenGL function pointers using GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        ROS_ERROR("Failed to initialize GLAD");
        return -1;
    }

    // OpenGL texture for the warped image
    GLuint warped_texture_id;
    glGenTextures(1, &warped_texture_id);

    // OpenGL setup
    glViewport(0, 0, win_width_pxl, win_height_pxl);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-1, 1, -1, 1, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // --------------- Image Setup ---------------

    // Load an image using OpenCV
    std::string img_path = "C:/Users/lester/MeDocuments/Research/MadhavLab/CodeBase/omniroute_windows_ws/data/proj_img/calibration_images/1_test_pattern.bmp";
    // std::string img_path = "C:/Users/lester/MeDocuments/Research/MadhavLab/CodeBase/omniroute_windows_ws/data/proj_img/calibration_images/2_manu_pirate.bmp";
    cv::Mat img = cv::imread(img_path.c_str());
    if (img.empty())
    {
        ROS_ERROR("Failed to load image");
        return -1;
    }

    // Convert BGR to RGB
    cv::Mat img_rgb;
    cv::cvtColor(img, img_rgb, cv::COLOR_BGR2RGB);

    // --------------- Convert to pixel coordinates ---------------

    // Function to convert NDC to Pixel Coordinates
    // auto convertToPixelCoords = [](std::vector<cv::Point2f> &quad_vertices, int width_pxl, int height_pxl, float width_ndc, float height_ndc)
    // {
    //     float min_x = std::numeric_limits<float>::max();
    //     float min_y = std::numeric_limits<float>::max();

    //     // First pass: Convert from NDC [-1, 1] to pixel [0, width or height] and find minimum x and y
    //     for (auto &point : quad_vertices)
    //     {
    //         point.x = ((point.x / width_ndc) + 0.5f) * width_pxl;
    //         point.y = ((-point.y / height_ndc) + 0.5f) * height_pxl; // Invert y to match OpenCV's top-left origin

    //         min_x = std::min(min_x, point.x);
    //         min_y = std::min(-min_y, point.y);
    //     }

    //     // Calculate the shifts needed to make all coordinates non-negative
    //     float x_shift = (min_x < 0) ? -min_x : 0;
    //     float y_shift = (min_y < 0) ? -min_y : 0;

    //     // Second pass: Shift all points by the calculated shifts
    //     for (auto &point : quad_vertices)
    //     {
    //         point.x += x_shift;
    //         point.y += y_shift;
    //     }
    // };

        // Function to convert NDC to Pixel Coordinates
    auto convertToPixelCoords = [](std::vector<cv::Point2f> &quad_vertices, int width_pxl, int height_pxl, float width_ndc, float height_ndc)
    {
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();

        // First pass: Convert from NDC [-1, 1] to pixel [0, width or height] and find minimum x and y
        for (const auto &point : quad_vertices)
        {
            float x_pxl = ((point.x / width_ndc) + 0.5f) * width_pxl;
            float y_pxl = ((-point.y / height_ndc) + 0.5f) * height_pxl; // Invert y to match OpenCV's top-left origin

            min_x = std::min(min_x, x_pxl);
            min_y = std::min(min_y, y_pxl);
        }

        // Calculate the shifts needed to make all coordinates non-negative
        float x_shift = (min_x < 0) ? -min_x : 0;
        float y_shift = (min_y < 0) ? -min_y : 0;

        // Second pass: Shift all points by the calculated shifts
        for (auto &point : quad_vertices)
        {
            point.x = ((point.x / width_ndc) + 0.5f) * width_pxl + x_shift;
            point.y = ((-point.y / height_ndc) + 0.5f) * height_pxl + y_shift;
        }
    };

    auto scaleQuadVert2UnitSquare = [](std::vector<cv::Point2f> &quad_vertices)
    {
        // Initialze max to type min and min to type max
        cv::Point2f p_min(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        cv::Point2f p_max(std::numeric_limits<float>::min(), std::numeric_limits<float>::min());

        // Find the min/max values
        for (const auto &point : quad_vertices)
        {
            p_min.x = std::min(p_min.x, point.x);
            p_max.x = std::max(p_max.x, point.x);
            p_min.y = std::min(p_min.y, point.y);
            p_max.y = std::max(p_max.y, point.y);
        }

        // Specify range
        float s_min = 0.0f;
        float s_max = 1.0f;

        // Scale the values to the new range
        for (auto &point : quad_vertices)
        {
            point.x = s_min + (point.x - p_min.x) * (s_max - s_min) / (p_max.x - p_min.x);
            point.y = s_min + (point.y - p_min.y) * (s_max - s_min) / (p_max.y - p_min.y);
        }
    };

    // std::vector<cv::Point2f> convertOpenGLToOpenCV(const std::vector<cv::Point2f> &vertices, int imageHeight)
    // {
    //     std::vector<cv::Point2f> convertedVertices;

    //     for (const cv::Point2f &vertex : vertices)
    //     {
    //         // Flip the y-coordinate to change from OpenGL to OpenCV
    //         cv::Point2f convertedVertex(vertex.x, imageHeight - vertex.y);

    //         convertedVertices.push_back(convertedVertex);
    //     }

    //     return convertedVertices;
    // };

     // --------------- Old Pixel Conversion ---------------

    // Convert origin_vertices to pixel coordinates
    std::vector<cv::Point2f> origin_vertices_pixel = origin_vertices_ndc;
    convertToPixelCoords(origin_vertices_pixel, img_width_pxl, img_height_pxl, img_width_ndc, img_height_ndc);

    // Convert target_vertices to pixel coordinates
    std::vector<cv::Point2f> target_vertices_pixel = target_vertices_ndc;
    convertToPixelCoords(target_vertices_pixel, img_width_pxl, img_height_pxl, img_width_ndc, img_height_ndc);

    // Find the bounding rectangle for the target vertices
    cv::Rect targ_bounding_rec = cv::boundingRect(target_vertices_pixel);
    cv::Size targ_img_size(targ_bounding_rec.width, targ_bounding_rec.height);

    // --------------- Warp the Image ---------------

    // Find Homography and Warp Image
    cv::Mat img_warped;
    cv::Mat H = cv::findHomography(origin_vertices_pixel, target_vertices_pixel);
    cv::warpPerspective(img_rgb, img_warped, H, targ_img_size);

    // Convert target vertices to texture coordinates
    std::vector<cv::Point2f> target_vertices_uv = target_vertices_ndc;

    // --------------- Debugging ---------------

    cv::namedWindow("Warped Image Display", cv::WINDOW_AUTOSIZE);
    cv::imshow("Warped Image Display", img_warped);
    // cv::waitKey(0);
    // cv::destroyWindow("Warped Image Display");

    // Log the homography matrix and vertices
    ROS_INFO("[COMPUTE HOMOGRAPHY] target_vertices_pixel:");
    dbLogQuadVertices(target_vertices_pixel);
    // ROS_INFO("[COMPUTE HOMOGRAPHY] target_vertices_ndc:");
    // dbLogQuadVertices(target_vertices_ndc);
    // ROS_INFO("[COMPUTE HOMOGRAPHY] target_vertices_uv:");
    // dbLogQuadVertices(target_vertices_uv);
    // dbLogHomMat(H);

    // Log targ_img_size
    ROS_INFO("PIXEL BOUNDING BOX: Width[%d] Height[%d]", targ_img_size.width, targ_img_size.height);

    // --------------- Draw Maker setup ---------------

    // Colors for markers
    std::array<float, 3> rgb_green = {0.0f, 1.0f, 0.0f};
    std::array<float, 3> rgb_red = {1.0f, 0.0f, 0.0f};
    std::array<float, 3> rgb_blue = {0.0f, 0.0f, 1.0f};
    std::array<float, 3> rgb_cyan = {0.0f, 1.0f, 1.0f};
    float cp_rad = 0.05f;

    auto drawMaker = [](float x, float y, float radius, std::array<float, 3> rgb_arr)
    {
        const int segments = 100; // Number of segments to approximate a circle

        // Begin drawing a filled circle
        glBegin(GL_TRIANGLE_FAN);

        // Set the color to green
        glColor3f(rgb_arr[0], rgb_arr[1], rgb_arr[2]);

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

        // Return GL status
        return checkErrorOpenGL(__LINE__, __FILE__);
    };

    // --------------- Draw the wall image ---------------

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        // Rendering commands here
        glClear(GL_COLOR_BUFFER_BIT);

        // Enable OpenGL texture mapping
        glEnable(GL_TEXTURE_2D);

        // Bind the texture and upload the warped image
        glBindTexture(GL_TEXTURE_2D, warped_texture_id);

        // Upload the warped RGB image
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_warped.cols, img_warped.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, img_warped.data);

        // Upload the RGB image
        // glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_rgb.cols, img_rgb.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, img_rgb.data); // Modified Line

        // Texture Parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // Start drawing a quadrilateral
        glBegin(GL_QUADS);

        // Set the color to white (for texture mapping)
        glColor3f(1.0f, 1.0f, 1.0f);

        // Top-left corner of texture
        glTexCoord2f(target_vertices_uv[0].x, target_vertices_uv[0].y);
        glVertex2f(target_vertices_ndc[0].x, target_vertices_ndc[0].y);

        // Top-right corner of texture
        glTexCoord2f(target_vertices_uv[1].x, target_vertices_uv[1].y);
        glVertex2f(target_vertices_ndc[1].x, target_vertices_ndc[1].y);

        // Bottom-right corner of texture
        glTexCoord2f(target_vertices_uv[3].x, target_vertices_uv[3].y);
        glVertex2f(target_vertices_ndc[3].x, target_vertices_ndc[3].y);

        // Bottom-left corner of texture
        glTexCoord2f(target_vertices_uv[2].x, target_vertices_uv[2].y);
        glVertex2f(target_vertices_ndc[2].x, target_vertices_ndc[2].y);

        // End drawing
        glEnd();

        // // Draw markers
        // drawMaker(-0.5f, 0.5f, cp_rad, rgb_green); // Top-left
        // drawMaker(0.5f, 0.5f, cp_rad, rgb_red);    // Top-right
        // drawMaker(-0.5f, -0.5f, cp_rad, rgb_blue); // Bottom-left
        // drawMaker(0.5f, -0.5f, cp_rad, rgb_cyan);  // Bottom-right

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // _______________ CLEANUP _______________

    // Cleanup and exit
    glDisable(GL_TEXTURE_2D);
    glDeleteTextures(1, &warped_texture_id);
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}

// int main(int argc, char **argv)
// {

//     //  _______________ SETUP _______________

//     // ROS Initialization
//     ros::init(argc, argv, "projection_calibration", ros::init_options::AnonymousName);
//     ros::NodeHandle n;
//     ros::NodeHandle nh("~");
//     ROS_INFO("RUNNING MAIN");

//     // Log setup parameters
//     ROS_INFO("[SETUP] Config XML Path: %s", CONFIG_DIR_PATH.c_str());
//     ROS_INFO("[SETUP] Display: Width[%d] Height[%d] AR[%0.2f]", PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, PROJ_WIN_ASPECT_RATIO);
//     ROS_INFO("[SETUP] Wall (Pxl): Width[%d] Height[%d]", WALL_WIDTH_PXL, WALL_HEIGHT_PXL);
//     ROS_INFO("[SETUP] Wall (NDC): Width[%0.2f] Height[%0.2f] Space Horz[%0.2f] Space Vert[%0.2f]", WALL_WIDTH_NDC, WALL_HEIGHT_NDC, WALL_SPACE_HORZ_NDC, WALL_SPACE_VERT_NDC);
//     ROS_INFO("[SETUP] Origin Plane (NDC): Width[%0.2f] Height[%0.2f]", PROJ_WIN_WIDTH_PXL, MAZE_HEIGHT_NDC);

//     // --------------- VARIABLE SETUP ---------------

//     // Initialze control points
//     initControlPointCoordinates(CTRL_PNT_DATA);

//     // Initialize wall parameter datasets
//     if (updateWallVertices(CTRL_PNT_DATA, WALL_VERT_DATA) != 0)
//     {
//         ROS_ERROR("[SETUP] Failed to Initalize the Wall Vertices Dataset");
//         return -1;
//     }

//     // Initialize homography matrix dataset
//     if (updateWallHomography(CTRL_PNT_DATA, WALL_VERT_DATA, WALL_HMAT_DATA) != 0)
//     {
//         ROS_ERROR("[SETUP] Failed to Initalize the Wall Homography Dataset");
//         return -1;
//     }

//     // --------------- OpenGL SETUP ---------------

//     // Initialize GLFW and set error callback
//     glfwSetErrorCallback(callbackErrorGLFW);
//     if (!glfwInit())
//     {
//         checkErrorGLFW(__LINE__, __FILE__);
//         ROS_ERROR("[GLFW] Initialization Failed");
//         return -1;
//     }

//     // Discover available monitors
//     pp_monitorIDVec = glfwGetMonitors(&nMonitors);
//     if (!pp_monitorIDVec || nMonitors == 0) // Added this check
//     {
//         ROS_ERROR("[GLFW] No monitors found");
//         return -1;
//     }
//     ROS_INFO("[GLFW] Found %d monitors", nMonitors);

//     // Create a new GLFW window
//     p_windowID = glfwCreateWindow(PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, "", NULL, NULL);
//     checkErrorGLFW(__LINE__, __FILE__);
//     if (!p_windowID)
//     {
//         glfwTerminate();
//         ROS_ERROR("[GLFW] Create Window Failed");
//         return -1;
//     }

//     // Set the GLFW window as the current OpenGL context
//     glfwMakeContextCurrent(p_windowID);

//     // Load OpenGL extensions using GLAD
//     if (!gladLoadGL()) // Added this check
//     {
//         ROS_ERROR("[GLAD] Failed to initialize GLAD");
//         return -1;
//     }

//     // Enable OpenGL debugging context
//     glEnable(GL_DEBUG_OUTPUT);
//     glDebugMessageCallback(callbackErrorOpenGL, 0);

//     // Set GLFW callbacks for keyboard and framebuffer size events
//     glfwSetKeyCallback(p_windowID, callbackKeyBinding);
//     glfwSetFramebufferSizeCallback(p_windowID, callbackFrameBufferSizeGLFW);

//     // Initialize Framebuffer Object (FBO) and its texture
//     GLuint fbo_id = 0;
//     GLuint fbo_texture_id = 0;

//     // Generate an FBO and bind it
//     glGenFramebuffers(1, &fbo_id);
//     glBindFramebuffer(GL_FRAMEBUFFER, fbo_id);
//     if (checkErrorOpenGL(__LINE__, __FILE__) != 0)
//     {
//         ROS_ERROR("[OpenGL] Failed to Generate FBO");
//         return -1;
//     }

//     // Generate a texture for the FBO
//     glGenTextures(1, &fbo_texture_id);
//     glBindTexture(GL_TEXTURE_2D, fbo_texture_id);
//     if (checkErrorOpenGL(__LINE__, __FILE__) != 0)
//     {
//         ROS_ERROR("[OpenGL] Failed to Generate FBO Texture");
//         return -1;
//     }

//     // Allocate storage for the texture on the GPU
//     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);

//     // Set the texture's MIN and MAG filter to linear interpolation.
//     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); // Handles sampling when the texture is scaled down
//     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); // Handles sampling when the texture is scaled up
//     if (checkErrorOpenGL(__LINE__, __FILE__) != 0)
//     {
//         ROS_ERROR("[OpenGL] Failed to Set FBO Texture Parameters");
//         return -1;
//     }

//     // Attach the texture to the FBO
//     glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_texture_id, 0);

//     // Check FBO completeness
//     GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
//     if (status != GL_FRAMEBUFFER_COMPLETE)
//     {
//         // Handle incomplete FBO, possibly log an error or exit
//         ROS_ERROR("[OpenGL] FBO is not complete");
//         return -1;
//     }

//     // Unbind the FBO (bind to default framebuffer)
//     glBindFramebuffer(GL_FRAMEBUFFER, 0);
//     if (checkErrorOpenGL(__LINE__, __FILE__) != 0)
//     {
//         ROS_ERROR("[OpenGL] Failed to Unbind FBO");
//         return -1;
//     }

//     // Update monitor and window mode settings
//     updateWindowMonMode(p_windowID, 0, pp_monitorIDVec, winMonInd, isFullScreen);

//     // Log OpenGL versions
//     const GLubyte *opengl_version = glGetString(GL_VERSION);
//     ROS_INFO("[OpenGL] Initialized: Version [%s]", opengl_version);

//     // Log GLFW versions
//     int glfw_major, glfw_minor, glfw_rev;
//     glfwGetVersion(&glfw_major, &glfw_minor, &glfw_rev);
//     ROS_INFO("[GLFW] Initialized: Version: %d.%d.%d", glfw_major, glfw_minor, glfw_rev);

//     // --------------- DevIL SETUP ---------------

//     // Initialize DevIL library
//     ilInit();
//     if (checkErrorDevIL(__LINE__, __FILE__) != 0)
//         return -1;

//     // Log the DevIL version
//     ILint version = ilGetInteger(IL_VERSION_NUM);
//     ROS_INFO("[DevIL] Intitalized: Version[%d]", version);

//     // Load images
//     if (loadImgTextures(imgWallPathVec, texWallIDVec) != 0)
//     {
//         ROS_ERROR("[DevIL] Failed to load wall images");
//         return -1;
//     }
//     if (loadImgTextures(imgMonPathVec, texMonIDVec) != 0)
//     {
//         ROS_ERROR("[DevIL] Failed to load monitor images");
//         return -1;
//     }
//     if (loadImgTextures(imgCalPathVec, texCalIDVec) != 0)
//     {
//         ROS_ERROR("[DevIL] Failed to load calibration images");
//         return -1;
//     }

//     // _______________ MAIN LOOP _______________

//     bool is_error = false;
//     while (!glfwWindowShouldClose(p_windowID) && ros::ok())
//     {

//         // --------------- Check Kayboard Callback Flags ---------------

//         // Load XML file
//         if (F.loadXML)
//         {
//             std::string file_path = formatCoordinatesFilePathXML(winMonInd, calModeInd, CONFIG_DIR_PATH);
//             loadCoordinatesXML(file_path, 3, homMat, ctrlPointParams);
//             F.loadXML = false;
//         }

//         // Save XML file
//         if (F.saveXML)
//         {
//             std::string file_path = formatCoordinatesFilePathXML(winMonInd, calModeInd, CONFIG_DIR_PATH);
//             saveCoordinatesXML(homMat, ctrlPointParams, file_path);
//             F.saveXML = false;
//         }

//         // Update the window monitor and mode
//         if (F.updateWindowMonMode)
//         {
//             if (updateWindowMonMode(p_windowID, 0, pp_monitorIDVec, winMonInd, isFullScreen) != 0)
//             {
//                 ROS_ERROR("[MAIN] Update Window Monitor Mode Threw Error");
//                 is_error = true;
//                 break;
//             }
//             F.updateWindowMonMode = false;
//         }

//         // Initialize/reinitialize control point coordinate dataset
//         if (F.initControlPointMarkers)
//         {
//             initControlPointCoordinates(CTRL_PNT_DATA);
//             F.initControlPointMarkers = false;
//         }

//         // Recompute wall vertices and homography matrices
//         if (F.updateWallDatasets)
//         {
//             // Initialize wall parameter datasets
//             if (updateWallVertices(CTRL_PNT_DATA, WALL_VERT_DATA) != 0)
//             {
//                 ROS_ERROR("[MAIN] Update of Wall Vertices Datasets Failed");
//                 return -1;
//             }

//             // Initialize homography matrix dataset
//             if (updateWallHomography(CTRL_PNT_DATA, WALL_VERT_DATA, WALL_HMAT_DATA) != 0)
//             {
//                 ROS_ERROR("[MAIN] Update of Wall Homography Datasets Failed");
//                 return -1;
//             }
//             F.updateWallDatasets = false;
//         }

//         // --------------- Handle Image Processing for Next Frame ---------------

//         // Clear back buffer for new frame
//         glClear(GL_COLOR_BUFFER_BIT);
//         if (checkErrorOpenGL(__LINE__, __FILE__))
//         {
//             is_error = true;
//             break;
//         }

//         // Draw/update wall images
//         if (drawWallImages(fbo_texture_id, texWallIDVec[imgWallInd], texMonIDVec[winMonInd], texCalIDVec[calModeInd]) != 0)
//         {
//             ROS_ERROR("[MAIN] Draw Walls Threw Error");
//             is_error = true;
//             break;
//         }

//         // Draw/update control point markers
//         if (updateControlPointMarkers() != 0)
//         {
//             ROS_ERROR("[MAIN] Draw Control Point Threw Error");
//             is_error = true;
//             break;
//         }

//         // Swap buffers and poll events
//         glfwSwapBuffers(p_windowID);
//         if (
//             checkErrorGLFW(__LINE__, __FILE__) ||
//             checkErrorOpenGL(__LINE__, __FILE__))
//         {
//             is_error = true;
//             break;
//         }

//         // Poll events
//         glfwPollEvents();

//         // Exit condition
//         if (glfwGetKey(p_windowID, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(p_windowID))
//             break;
//     }

//     // _______________ CLEANUP _______________
//     ROS_INFO("SHUTTING DOWN");

//     // Check which condition caused the loop to exit
//     if (!ros::ok())
//         ROS_INFO("[LOOP TERMINATION] ROS Node is no Longer in a Good State");
//     else if (glfwWindowShouldClose(p_windowID))
//         ROS_INFO("[LOOP TERMINATION] GLFW Window Should Close");
//     else if (glfwGetKey(p_windowID, GLFW_KEY_ESCAPE) == GLFW_PRESS)
//         ROS_INFO("[LOOP TERMINATION] Escape Key was Pressed");
//     else if (is_error)
//         ROS_INFO("[LOOP TERMINATION] Error Thrown");
//     else
//         ROS_INFO("[LOOP TERMINATION] Reason Unknown");

//     // Delete FBO
//     if (fbo_id != 0)
//     {
//         glDeleteFramebuffers(1, &fbo_id);
//         if (checkErrorOpenGL(__LINE__, __FILE__) != 0)
//             ROS_WARN("[SHUTDOWN] Failed to Delete FBO");
//         else
//             ROS_INFO("[SHUTDOWN] Deleted FBO");
//     }
//     else
//         ROS_WARN("[SHUTDOWN] No FBO to Delete");

//     // Delete FBO texture
//     if (fbo_texture_id != 0)
//     {
//         glDeleteTextures(1, &fbo_texture_id);
//         if (checkErrorOpenGL(__LINE__, __FILE__) != 0)
//             ROS_WARN("[SHUTDOWN] Failed to Delete FBO Texture");
//         else
//             ROS_INFO("[SHUTDOWN] Deleted FBO Texture");
//     }
//     else
//         ROS_WARN("[SHUTDOWN] No FBO Texture to Delete");

//     // Delete DevIL images
//     if (deleteImgTextures(texWallIDVec) == 0)
//         ROS_INFO("[SHUTDOWN] Deleted DevIL Wall Images");
//     if (deleteImgTextures(texMonIDVec) == 0)
//         ROS_INFO("[SHUTDOWN] Deleted DevIL Monitor Images");
//     if (deleteImgTextures(texCalIDVec) == 0)
//         ROS_INFO("[SHUTDOWN] Deleted DevIL Calibration Images");

//     // Destroy GLFW window
//     if (p_windowID)
//     {
//         glfwDestroyWindow(p_windowID);
//         p_windowID = nullptr;
//         if (checkErrorGLFW(__LINE__, __FILE__) != 0)
//             ROS_WARN("[SHUTDOWN] Failed to Destroy GLFW Window");
//         else
//             ROS_INFO("[SHUTDOWN] Destroyed GLFW Window");
//     }
//     else
//     {
//         ROS_WARN("[SHUTDOWN] No GLFW window to destroy");
//     }

//     // Shutdown DevIL
//     ilShutDown();
//     checkErrorDevIL(__LINE__, __FILE__);
//     ROS_INFO("[SHUTDOWN] Shutdown DevIL");

//     // Terminate GLFW
//     glfwTerminate();
//     checkErrorGLFW(__LINE__, __FILE__);
//     ROS_INFO("[SHUTDOWN] Terminated GLFW");

//     // Return success
//     return is_error ? -1 : 0;
// }
