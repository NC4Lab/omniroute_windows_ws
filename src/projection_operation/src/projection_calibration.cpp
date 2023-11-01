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
                cpSelectedInd[0] = (cpSelectedInd[0] % 2); // Result will be 0 or 1
            }
            else if (key == GLFW_KEY_DOWN)
            {
                // Move to the bottom row, keeping the horizontal position
                cpSelectedInd[0] = 2 + (cpSelectedInd[0] % 2); // Result will be 2 or 3
            }
            else if (key == GLFW_KEY_LEFT)
            {
                // Move to the left column, keeping the vertical position
                cpSelectedInd[0] = (cpSelectedInd[0] >= 2) ? 2 : 0; // Result will be 0 or 2
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                // Move to the right column, keeping the vertical position
                cpSelectedInd[0] = (cpSelectedInd[0] >= 2) ? 3 : 1; // Result will be 1 or 3
            }
        }

        // ---------- Contol point vertex selector keys [ALT [LEFT, RIGHT, UP, DOWN]] ----------

        else if (mods & GLFW_MOD_ALT)
        {
            if (key == GLFW_KEY_UP)
            {
                // Move to the top row, keeping the horizontal position
                cpSelectedInd[1] = (cpSelectedInd[1] % 2); // Result will be 0 or 1
            }
            else if (key == GLFW_KEY_DOWN)
            {
                // Move to the bottom row, keeping the horizontal position
                cpSelectedInd[1] = 2 + (cpSelectedInd[1] % 2); // Result will be 2 or 3
            }
            else if (key == GLFW_KEY_LEFT)
            {
                // Move to the left column, keeping the vertical position
                cpSelectedInd[1] = (cpSelectedInd[1] >= 2) ? 2 : 0; // Result will be 0 or 2
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                // Move to the right column, keeping the vertical position
                cpSelectedInd[1] = (cpSelectedInd[1] >= 2) ? 3 : 1; // Result will be 1 or 3
            }
        }

        // ---------- Control point translate [SHIFT or no modifier] ----------
        else
        {
            // Set the position increment based on whether the shift key is pressed
            float pos_inc = (mods & GLFW_MOD_SHIFT) ? 0.01f : 0.0005f;

            // Store current origin
            cv::Point2f cp_origin_save = CTRL_PT_COORDS[cpSelectedInd[0]][2];

            // Listen for arrow key input to move selected control point
            if (key == GLFW_KEY_LEFT)
            {
                CTRL_PT_COORDS[cpSelectedInd[0]][cpSelectedInd[1]].x -= pos_inc; // Move left
                F.updateWallDatasets = true;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                CTRL_PT_COORDS[cpSelectedInd[0]][cpSelectedInd[1]].x += pos_inc; // Move right
                F.updateWallDatasets = true;
            }
            else if (key == GLFW_KEY_UP)
            {
                CTRL_PT_COORDS[cpSelectedInd[0]][cpSelectedInd[1]].y += pos_inc; // Move up
                F.updateWallDatasets = true;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                CTRL_PT_COORDS[cpSelectedInd[0]][cpSelectedInd[1]].y -= pos_inc; // Move down
                F.updateWallDatasets = true;
            }

            // Shift all control points if origin moved
            cv::Point2f cp_origin_new = CTRL_PT_COORDS[cpSelectedInd[0]][2];

            // Calculate the change in x and y for the origin
            float delta_x = cp_origin_new.x - cp_origin_save.x;
            float delta_y = cp_origin_new.y - cp_origin_save.y;

            // Check if the origin vertex was moved
            if (cpSelectedInd[1] == 2)
            {
                // Update all other vertices based on the change in the origin
                for (int i = 0; i < 4; ++i) // Assuming there are 4 vertices
                {
                    if (i != 2) // Skip the origin vertex itself
                    {
                        CTRL_PT_COORDS[cpSelectedInd[0]][i].x += delta_x;
                        CTRL_PT_COORDS[cpSelectedInd[0]][i].y += delta_y;
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

    // Itterate through maze cornerss
    for (int c_i = 0; c_i < 4; c_i++)
    {
        // Itterate through verteces
        for (int v_i = 0; v_i < 4; v_i++)
        {
            float cp_rad = cpMakerRadius[0];
            std::array<float, 3> cp_col = cpUnelectedRGB;

            // Set color based on cp selected
            if (c_i == cpSelectedInd[0])
            {
                if (cpSelectedInd[1] == v_i)
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
            cv::Point2f p_cp = CTRL_PT_COORDS[c_i][v_i];

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

int drawWallImages(GLuint fbo_texture_id, ILuint tex_wall_id, ILuint tex_mode_mon_id, ILuint tex_mode_cal_id)
{
    // Enable OpenGL texture mapping
    glEnable(GL_TEXTURE_2D);

    // Iterate through the wall grid rows and columns
    for (float grow_i = 0; grow_i < MAZE_SIZE; grow_i++) // image bottom to top
    {
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
                (cpSelectedInd[0] == 0 && grow_i == 0 && gcol_i == 0) ||
                (cpSelectedInd[0] == 1 && grow_i == 0 && gcol_i == MAZE_SIZE - 1) ||
                (cpSelectedInd[0] == 2 && grow_i == MAZE_SIZE - 1 && gcol_i == 0) ||
                (cpSelectedInd[0] == 3 && grow_i == MAZE_SIZE - 1 && gcol_i == MAZE_SIZE - 1))
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
            std::array<cv::Point2f, 4> quad_vertices_warped = WALL_WARP_COORDS[grow_i][gcol_i];

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

int initializeOpenGLObjects()
{
    // -------- LAMBDA FUNCTION FOR INITIALIZING VAO AND VBO BY ELEMENT -------
    auto initializeByElement = [](GLuint &vao, GLuint &vbo, float *vertices, size_t size)
    {
        // Generate and bind a Vertex Array Object (VAO)
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);

        // Generate and bind a Vertex Buffer Object (VBO)
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);

        // Initialize the VBO with vertex data
        glBufferData(GL_ARRAY_BUFFER, size, vertices, GL_STATIC_DRAW);

        // Specify the format of the vertex data for the position attribute
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
        glEnableVertexAttribArray(0); // Enable the position attribute

        // Specify the format of the vertex data for the texture coordinate attribute
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));
        glEnableVertexAttribArray(1); // Enable the texture coordinate attribute

        // Unbind the VAO to prevent accidental modification
        glBindVertexArray(0);
    };

    // --------------- SETUP FOR WALL IMAGE RENDERING ---------------

    // Generate and bind an Element Buffer Object (EBO)
    glGenBuffers(1, &WALL_EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, WALL_EBO);

    // Initialize the EBO with index data
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(WALL_GL_INDICES), WALL_GL_INDICES, GL_DYNAMIC_DRAW);

    // Iterate through the wall grid to initialize each element's VAO and VBO
    for (int gr_i = 0; gr_i < MAZE_SIZE; gr_i++)
    {
        for (int gc_i = 0; gc_i < MAZE_SIZE; gc_i++)
        {
            initializeByElement(WALL_VAO_ARR[gr_i][gc_i], WALL_VBO_ARR[gr_i][gc_i], WALL_GL_VERTICES, sizeof(WALL_GL_VERTICES));
        }
    }

    // Create the shader program for wall image rendering
    WALL_SHADER = createShaderProgram(vertexSource, fragmentSource);

    // --------------- SETUP FOR CONTROL POINT RENDERING ---------------

    // Iterate through the maze corners to initialize each control point's VAO and VBO
    for (int c_i = 0; c_i < 4; c_i++)
    {
        for (int v_i = 0; v_i < 4; v_i++)
        {
            initializeByElement(CTRL_PT_VAO_ARR[c_i][v_i], CTRL_PT_VBO_ARR[c_i][v_i], NULL, sizeof(cv::Point2f));
        }
    }

    // Create the shader program for control point rendering
    CTRL_PT_SHADER = createShaderProgram(vertexSource, fragmentSource);

    // Return success
    return 0;
}

GLuint createShaderProgram(const GLchar *vertex_source, const GLchar *fragment_source)
{
    // Create and compile the vertex shader
    GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_source, NULL);
    glCompileShader(vertex_shader);

    // Create and compile the fragment shader
    GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_source, NULL);
    glCompileShader(fragment_shader);

    // Link the vertex and fragment shader into a shader program
    GLuint shader_program = glCreateProgram();
    glAttachShader(shader_program, vertex_shader);
    glAttachShader(shader_program, fragment_shader);
    glLinkProgram(shader_program);

    // Delete the vertex and fragment shaders as they're now linked into the program
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    return shader_program;
}

GLuint loadTexture(cv::Mat image)
{
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    // Convert image from BGR to RGB
    cv::Mat image_rgb;
    cv::cvtColor(image, image_rgb, cv::COLOR_BGR2RGB);

    // Handle alignment
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    // Create texture
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_rgb.cols,
                 image_rgb.rows, 0, GL_RGB, GL_UNSIGNED_BYTE,
                 image_rgb.data);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    return textureID;
}

bool textureMerge(const std::string &base_img_path, const std::string &mask_img_path, const std::string &output_img_path, cv::Mat &out_merg_img)
{
    // Read the base and mask images
    cv::Mat base_img = cv::imread(base_img_path, cv::IMREAD_UNCHANGED);
    cv::Mat mask_img = cv::imread(mask_img_path, cv::IMREAD_UNCHANGED);

    // Check if images are loaded successfully
    if (base_img.empty() || mask_img.empty())
    {
        ROS_ERROR("Error: Could not read one or both images.");
        return false;
    }

    // Check dimensions
    if (base_img.size() != mask_img.size())
    {
        ROS_ERROR("Error: Image dimensions do not match.");
        return false;
    }

    // Copy the base image to the output image
    out_merg_img = base_img.clone();

    // Loop through each pixel
    for (int y = 0; y < base_img.rows; ++y)
    {
        for (int x = 0; x < base_img.cols; ++x)
        {
            cv::Vec4b &base_pixel = base_img.at<cv::Vec4b>(y, x);
            cv::Vec4b &mask_pixel = mask_img.at<cv::Vec4b>(y, x);

            // If the mask pixel is not white, overlay it onto the base image
            if (mask_pixel[0] != 255 || mask_pixel[1] != 255 || mask_pixel[2] != 255)
            {
                out_merg_img.at<cv::Vec4b>(y, x) = mask_pixel;
            }
        }
    }

    // // Save the merged image
    // if (!cv::imwrite(output_img_path, out_merg_img)) {
    //     ROS_ERROR("Error: Could not save merged image.");
    //     return false;
    // }

    return true;
}

int warpRenderWallImages()
{

    // Specify window resolution size
    const int win_wd_pxl = 1000;
    const int win_ht_pxl = 1000;

    // Image size (pixels)
    const int im_wd_pxl = 300;
    const int im_ht_pxl = 540;

    // Image size (NDC)
    const float im_wd_ndc = (static_cast<float>(im_wd_pxl) / static_cast<float>(win_wd_pxl)) * 2;
    const float im_ht_ndc = (static_cast<float>(im_ht_pxl) / static_cast<float>(win_ht_pxl)) * 2;

    // Initialize GLFW
    if (!glfwInit())
    {
        ROS_ERROR("Failed to initialize GLFW");
        return -1;
    }
    GLFWwindow *window = glfwCreateWindow(win_wd_pxl, win_ht_pxl, "OpenGL", NULL, NULL);
    if (!window)
    {
        ROS_ERROR("Failed to create window");
        return -1;
    }
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

    // Initialize OpenGL objects
    if (initializeOpenGLObjects() != 0)
    {
        ROS_ERROR("Failed to initialize OpenGL objects");
        return -1;
    }

    // // Load image using OpenCV
    // std::string img_path = "C:/Users/lester/MeDocuments/Research/MadhavLab/CodeBase/omniroute_windows_ws/data/proj_img/calibration_images/1_test_pattern.bmp";
    // // std::string img_path = "C:/Users/lester/MeDocuments/Research/MadhavLab/CodeBase/omniroute_windows_ws/data/proj_img/calibration_images/2_manu_pirate.bmp";
    // cv::Mat im_wall = cv::imread(img_path.c_str());
    // if (im_wall.empty())
    // {
    //     ROS_ERROR("Failed to load image);
    //     return -1;
    // }

    // Test load and merg images
    std::string base_img_path = "C:/Users/lester/MeDocuments/Research/MadhavLab/CodeBase/omniroute_windows_ws/data/proj_img/calibration_images/1_test_pattern.bmp";
    std::string mask_img_path = "C:/Users/lester/MeDocuments/Research/MadhavLab/CodeBase/omniroute_windows_ws/data/proj_img/ui_state_images/m0.bmp";
    std::string output_img_path = "C:/Users/lester/MeDocuments/Research/MadhavLab/CodeBase/omniroute_windows_ws/data/assets/temp/output_image.bmp";
    cv::Mat im_wall;

    // Merge the images
    if (textureMerge(base_img_path, mask_img_path, output_img_path, im_wall))
    {
        ROS_INFO("Successfully merged images.");
    }
    else
    {
        std::cout << "Failed to merge images." << std::endl;
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

    // // Main loop (unchanged)
    // while (!glfwWindowShouldClose(window))
    // {
    //     // Clear the screen
    //     glClear(GL_COLOR_BUFFER_BIT);

    //     // Use the shader program for wall rendering
    //     glUseProgram(WALL_SHADER);

    //     // Bind the texture for the wall
    //     glActiveTexture(GL_TEXTURE0);
    //     glBindTexture(GL_TEXTURE_2D, texture);

    //     // Bind the VAO for the first wall
    //     glBindVertexArray(WALL_VAO_ARR[0][0]);

    //     // Bind the EBO for the first wall (if you're using the same EBO for all walls, this step may not be necessary every frame)
    //     glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, WALL_EBO);

    //     // Draw the rectangle (2 triangles) for the first wall
    //     glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    //     // Unbind the VAO to prevent accidental modification
    //     glBindVertexArray(0);

    //     // Swap the front and back buffers
    //     glfwSwapBuffers(window);

    //     // Poll for events like keyboard input or window closing
    //     glfwPollEvents();
    // }

    // Main loop (unchanged)
    while (!glfwWindowShouldClose(window))
    {

        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT);

        // Use the shader program for wall rendering
        glUseProgram(WALL_SHADER);

        // Bind the common Element Buffer Object (EBO)
        // (assuming you are using the same EBO for all walls)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, WALL_EBO);

        // Bind the texture for the walls
        // (assuming you are using the same texture for all walls)
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture);

        // Loop through all the wall images
        for (int gr_i = 0; gr_i < 1; gr_i++)
        {
            for (int gc_i = 0; gc_i < 1; gc_i++)
            {
                // Bind the Vertex Array Object (VAO) specific to the current wall
                glBindVertexArray(WALL_VAO_ARR[gr_i][gc_i]);

                // Draw the rectangle (2 triangles) for the current wall
                glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

                // Unbind the VAO to prevent accidental modification
                glBindVertexArray(0);
            }
        }
    }

    // Swap the front and back buffers
    glfwSwapBuffers(window);

    // Poll for events like keyboard input or window closing
    glfwPollEvents();

    // Cleanup
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
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
    ROS_INFO("[SETUP] Config XML Path: %s", CONFIG_DIR_PATH.c_str());
    ROS_INFO("[SETUP] Display: Width[%d] Height[%d] AR[%0.2f]", PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, PROJ_WIN_ASPECT_RATIO);
    ROS_INFO("[SETUP] Wall (Pxl): Width[%d] Height[%d]", WALL_WIDTH_PXL, WALL_HEIGHT_PXL);
    ROS_INFO("[SETUP] Wall (NDC): Width[%0.2f] Height[%0.2f] Space Horz[%0.2f] Space Vert[%0.2f]", WALL_WIDTH_NDC, WALL_HEIGHT_NDC, WALL_SPACE_HORZ_NDC, WALL_SPACE_VERT_NDC);
    ROS_INFO("[SETUP] Origin Plane (NDC): Width[%0.2f] Height[%0.2f]", PROJ_WIN_WIDTH_PXL, MAZE_HEIGHT_NDC);

    // TEMP
    warpRenderWallImages();
    return 0;

    // --------------- VARIABLE SETUP ---------------

    // Initialze control points
    initControlPointCoordinates(CTRL_PT_COORDS);

    // Initialize wall parameter datasets
    if (updateWallVertices(CTRL_PT_COORDS, WALL_WARP_COORDS) != 0)
    {
        ROS_ERROR("[SETUP] Failed to Initalize the Wall Vertices Dataset");
        return -1;
    }

    // Initialize homography matrix dataset
    if (updateWallHomography(CTRL_PT_COORDS, WALL_WARP_COORDS, WALL_HMAT_DATA) != 0)
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
    pp_monitorIDVec = glfwGetMonitors(&nMonitors);
    if (!pp_monitorIDVec || nMonitors == 0) // Added this check
    {
        ROS_ERROR("[GLFW] No monitors found");
        return -1;
    }
    ROS_INFO("[GLFW] Found %d monitors", nMonitors);

    // Create a new GLFW window
    p_windowID = glfwCreateWindow(PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, "", NULL, NULL);
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
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);

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
    updateWindowMonMode(p_windowID, 0, pp_monitorIDVec, winMonInd, isFullScreen);

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
    if (loadImgTextures(imgWallPathVec, texWallIDVec) != 0)
    {
        ROS_ERROR("[DevIL] Failed to load wall images");
        return -1;
    }
    if (loadImgTextures(imgMonPathVec, texMonIDVec) != 0)
    {
        ROS_ERROR("[DevIL] Failed to load monitor images");
        return -1;
    }
    if (loadImgTextures(imgCalPathVec, texCalIDVec) != 0)
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
            std::string file_path = formatCoordinatesFilePathXML(winMonInd, calModeInd, CONFIG_DIR_PATH);
            /// @todo Ad save xml back in
            F.loadXML = false;
        }

        // Save XML file
        if (F.saveXML)
        {
            std::string file_path = formatCoordinatesFilePathXML(winMonInd, calModeInd, CONFIG_DIR_PATH);
            /// @todo Ad save xml back in
            F.saveXML = false;
        }

        // Update the window monitor and mode
        if (F.updateWindowMonMode)
        {
            if (updateWindowMonMode(p_windowID, 0, pp_monitorIDVec, winMonInd, isFullScreen) != 0)
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
            initControlPointCoordinates(CTRL_PT_COORDS);
            F.initControlPointMarkers = false;
        }

        // Recompute wall vertices and homography matrices
        if (F.updateWallDatasets)
        {
            // Initialize wall parameter datasets
            if (updateWallVertices(CTRL_PT_COORDS, WALL_WARP_COORDS) != 0)
            {
                ROS_ERROR("[MAIN] Update of Wall Vertices Datasets Failed");
                return -1;
            }

            // Initialize homography matrix dataset
            if (updateWallHomography(CTRL_PT_COORDS, WALL_WARP_COORDS, WALL_HMAT_DATA) != 0)
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
        if (drawWallImages(fbo_texture_id, texWallIDVec[imgWallInd], texMonIDVec[winMonInd], texCalIDVec[calModeInd]) != 0)
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
    if (deleteImgTextures(texWallIDVec) == 0)
        ROS_INFO("[SHUTDOWN] Deleted DevIL Wall Images");
    if (deleteImgTextures(texMonIDVec) == 0)
        ROS_INFO("[SHUTDOWN] Deleted DevIL Monitor Images");
    if (deleteImgTextures(texCalIDVec) == 0)
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
