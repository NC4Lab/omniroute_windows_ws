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
            F.updateWindowMonMode = true;
        }

        // Move the window to another monitor
        else if (key == GLFW_KEY_0)
        {
            I.winMon = 0;
            F.updateWindowMonMode = true;
        }
        else if (key == GLFW_KEY_1 && N.monitors > 1)
        {
            I.winMon = 1;
            F.updateWindowMonMode = true;
        }
        else if (key == GLFW_KEY_2 && N.monitors > 2)
        {
            I.winMon = 2;
            F.updateWindowMonMode = true;
        }
        else if (key == GLFW_KEY_3 && N.monitors > 3)
        {
            I.winMon = 3;
            F.updateWindowMonMode = true;
        }
        else if (key == GLFW_KEY_4 && N.monitors > 4)
        {
            I.winMon = 4;
            F.updateWindowMonMode = true;
        }
        else if (key == GLFW_KEY_5 && N.monitors > 5)
        {
            I.winMon = 5;
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
            I.wallImage = N.wallImages > 0 ? 0 : I.wallImage;
        }
        else if (key == GLFW_KEY_F2)
        {
            I.wallImage = N.wallImages > 1 ? 1 : I.wallImage;
        }
        else if (key == GLFW_KEY_F3)
        {
            I.wallImage = N.wallImages > 2 ? 2 : I.wallImage;
        }
        else if (key == GLFW_KEY_F4)
        {
            I.wallImage = N.wallImages > 3 ? 3 : I.wallImage;
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

        // ---------- Contol point wall selector keys [CTRL [LEFT, RIGHT, UP, DOWN]] ----------

        else if (mods & GLFW_MOD_CONTROL)
        {
            if (key == GLFW_KEY_UP)
            {
                // Move to the top row, keeping the horizontal position
                I.cpSelected[0] = (I.cpSelected[0] % 2); // Result will be 0 or 1
            }
            else if (key == GLFW_KEY_DOWN)
            {
                // Move to the bottom row, keeping the horizontal position
                I.cpSelected[0] = 2 + (I.cpSelected[0] % 2); // Result will be 2 or 3
            }
            else if (key == GLFW_KEY_LEFT)
            {
                // Move to the left column, keeping the vertical position
                I.cpSelected[0] = (I.cpSelected[0] >= 2) ? 2 : 0; // Result will be 0 or 2
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                // Move to the right column, keeping the vertical position
                I.cpSelected[0] = (I.cpSelected[0] >= 2) ? 3 : 1; // Result will be 1 or 3
            }
        }

        // ---------- Contol point vertex selector keys [ALT [LEFT, RIGHT, UP, DOWN]] ----------

        else if (mods & GLFW_MOD_ALT)
        {
            if (key == GLFW_KEY_UP)
            {
                // Move to the top row, keeping the horizontal position
                I.cpSelected[1] = (I.cpSelected[1] % 2); // Result will be 0 or 1
            }
            else if (key == GLFW_KEY_DOWN)
            {
                // Move to the bottom row, keeping the horizontal position
                I.cpSelected[1] = 2 + (I.cpSelected[1] % 2); // Result will be 2 or 3
            }
            else if (key == GLFW_KEY_LEFT)
            {
                // Move to the left column, keeping the vertical position
                I.cpSelected[1] = (I.cpSelected[1] >= 2) ? 2 : 0; // Result will be 0 or 2
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                // Move to the right column, keeping the vertical position
                I.cpSelected[1] = (I.cpSelected[1] >= 2) ? 3 : 1; // Result will be 1 or 3
            }
        }

        // ---------- Control point translate [SHIFT or no modifier] ----------
        else
        {
            // Set the position increment based on whether the shift key is pressed
            float pos_inc = (mods & GLFW_MOD_SHIFT) ? 0.01f : 0.0005f;

            // Store current origin
            cv::Point2f cp_origin_save = CP_COORDS[I.cpSelected[0]][2];

            // Listen for arrow key input to move selected control point
            if (key == GLFW_KEY_LEFT)
            {
                CP_COORDS[I.cpSelected[0]][I.cpSelected[1]].x -= pos_inc; // Move left
                F.updateWallDatasets = true;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                CP_COORDS[I.cpSelected[0]][I.cpSelected[1]].x += pos_inc; // Move right
                F.updateWallDatasets = true;
            }
            else if (key == GLFW_KEY_UP)
            {
                CP_COORDS[I.cpSelected[0]][I.cpSelected[1]].y += pos_inc; // Move up
                F.updateWallDatasets = true;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                CP_COORDS[I.cpSelected[0]][I.cpSelected[1]].y -= pos_inc; // Move down
                F.updateWallDatasets = true;
            }

            // Shift all control points if origin moved
            cv::Point2f cp_origin_new = CP_COORDS[I.cpSelected[0]][2];

            // Calculate the change in x and y for the origin
            float delta_x = cp_origin_new.x - cp_origin_save.x;
            float delta_y = cp_origin_new.y - cp_origin_save.y;

            // Check if the origin vertex was moved
            if (I.cpSelected[1] == 2)
            {
                // Update all other vertices based on the change in the origin
                for (int i = 0; i < 4; ++i) // Assuming there are 4 vertices
                {
                    if (i != 2) // Skip the origin vertex itself
                    {
                        CP_COORDS[I.cpSelected[0]][i].x += delta_x;
                        CP_COORDS[I.cpSelected[0]][i].y += delta_y;
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

static void APIENTRY callbackDebugOpenGL(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message, const void *userParam)
{
    // Get level of severity
    int s_level = (severity == GL_DEBUG_SEVERITY_NOTIFICATION) ? 1 : (severity == GL_DEBUG_SEVERITY_LOW)  ? 2
                                                                 : (severity == GL_DEBUG_SEVERITY_MEDIUM) ? 3
                                                                 : (severity == GL_DEBUG_SEVERITY_HIGH)   ? 4
                                                                                                          : 0;

    // Check if the message is below the specified debug level
    if (s_level < DEBUG_LEVEL_GL)
    {
        return;
    }

    // Log the message
    ROS_ERROR("[OPENGL DEBUG CALLBACK] Type[0x%x] ID[%d] Severity[0x%x] Message[%s]", type, id, severity, message);
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

int initializeWallObjects()
{

    // Generate and bind an Element Buffer Object (EBO)
    glGenBuffers(1, &WALL_EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, WALL_EBO);

    // Initialize the EBO with index data
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(WALL_GL_INDICES), WALL_GL_INDICES, GL_DYNAMIC_DRAW);

    // Generate and bind a Vertex Array Object (VAO)
    glGenVertexArrays(1, &WALL_VAO);
    glBindVertexArray(WALL_VAO);

    // Generate and bind a Vertex Buffer Object (VBO)
    glGenBuffers(1, &WALL_VBO);
    glBindBuffer(GL_ARRAY_BUFFER, WALL_VBO);

    // Initialize the VBO with vertex data
    glBufferData(GL_ARRAY_BUFFER, sizeof(WALL_GL_VERTICES), WALL_GL_VERTICES, GL_STATIC_DRAW);

    // Specify the format of the vertex data for the position attribute
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0); // Enable the position attribute

    // Specify the format of the vertex data for the texture coordinate attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));
    glEnableVertexAttribArray(1); // Enable the texture coordinate attribute

    // Unbind the VAO to prevent accidental modification
    glBindVertexArray(0);

    // Create the shader program for wall image rendering
    WALL_SHADER = createShaderProgram(wallVertexSource, wallFragmentSource);

    // Return GL status
    return checkErrorOpenGL(__LINE__, __FILE__);
}

int initializeControlPointObjects()
{
    // Create shader program
    CP_SHADER = createShaderProgram(ctrlPtVertexSource, ctrlPtFragmentSource);

    // Generate and bind the VAO
    glGenVertexArrays(1, &CP_VAO);
    glBindVertexArray(CP_VAO);

    // Loop through all control points to create individual VBOs
    for (int c_i = 0; c_i < 4; ++c_i)
    {
        for (int v_i = 0; v_i < 4; ++v_i)
        {
            // Generate VBO for position
            glGenBuffers(1, &CP_VBO_POS_ARR[c_i][v_i]);
            glBindBuffer(GL_ARRAY_BUFFER, CP_VBO_POS_ARR[c_i][v_i]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(cv::Point2f), nullptr, GL_DYNAMIC_DRAW);

            // Generate VBO for color
            glGenBuffers(1, &CP_VBO_RGB_ARR[c_i][v_i]);
            glBindBuffer(GL_ARRAY_BUFFER, CP_VBO_RGB_ARR[c_i][v_i]);
            glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(GLfloat), cpDefaultRGB.data(), GL_STATIC_DRAW);

            // Generate VBO for size
            glGenBuffers(1, &CP_VBO_RAD_ARR[c_i][v_i]);
            glBindBuffer(GL_ARRAY_BUFFER, CP_VBO_RAD_ARR[c_i][v_i]);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat), &cpDefualtMakerRadius, GL_STATIC_DRAW);
        }
    }

    // Unbind the VAO
    glBindVertexArray(0);

    // Check for errors
    return checkErrorOpenGL(__LINE__, __FILE__);
}

int updateWallTexture(
    cv::Mat img_wall_mat, cv::Mat img_mode_mon_mat, cv::Mat img_mode_cal_mat,
    std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> &_WALL_HMAT_DATA,
    GLuint &out_WALL_TEXTURE_ID)
{
    // Initializ the image to be used as the texture
    cv::Mat im_wall_merge = cv::Mat::zeros(WALL_HEIGHT_PXL, WALL_WIDTH_PXL, CV_8UC3); // Bottom-right

    // Iterate through the maze grid rows
    for (float grow_i = 0; grow_i < MAZE_SIZE; grow_i++) // image bottom to top
    {
        // Iterate through each column in the maze row
        for (float gcol_i = 0; gcol_i < MAZE_SIZE; gcol_i++) // image left to right
        {
            //  Create merged image for the wall corresponding to the selected control point
            if (
                (I.cpSelected[0] == 0 && grow_i == 0 && gcol_i == 0) ||
                (I.cpSelected[0] == 1 && grow_i == 0 && gcol_i == MAZE_SIZE - 1) ||
                (I.cpSelected[0] == 2 && grow_i == MAZE_SIZE - 1 && gcol_i == 0) ||
                (I.cpSelected[0] == 3 && grow_i == MAZE_SIZE - 1 && gcol_i == MAZE_SIZE - 1))
            {
                // // Merge test pattern and active monitor image
                // if (textureMerge(tex_mode_mon_id, copy_tex_wall_id) != 0)
                //     return -1;

                // // Merge previous image and active calibration image
                // if (textureMerge(tex_mode_cal_id, copy_tex_wall_id) != 0)
                //     return -1;
            }
        }
    }

    // Make the new texture
    WALL_TEXTURE_ID = loadTexture(im_wall_merge);

    return 0;
}

int renderWallImage(const GLuint &_WALL_TEXTURE_ID)
{
    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT);

    // Use the shader program for wall rendering
    glUseProgram(WALL_SHADER);

    // Bind the texture for the walls
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, WALL_TEXTURE_ID);

    // Bind the Vertex Array Object(VAO) specific to the current wall
    glBindVertexArray(WALL_VAO);

    // Bind the common Element Buffer Object (EBO)
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, WALL_EBO);

    // Draw the rectangle (2 triangles) for the current wall
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    // Unbind the VAO to prevent accidental modification
    glBindVertexArray(0);

    // Return GL status
    return checkErrorOpenGL(__LINE__, __FILE__);
}

int renderControlPoints(const std::array<std::array<cv::Point2f, 4>, 4> &_CP_COORDS)
{
    // Use the shader program for control point rendering
    glUseProgram(CP_SHADER);

    // Bind the VAO
    glBindVertexArray(CP_VAO);

    // Loop through the control points and draw them
    for (int c_i = 0; c_i < 4; ++c_i)
    {
        for (int v_i = 0; v_i < 4; ++v_i)
        {
            // Define the marker color
            std::array<GLfloat, 3> cp_rgb;
            if (c_i == I.cpSelected[0])
            {
                if (I.cpSelected[1] == v_i)
                    cp_rgb = cpVertSelectedRGB;
                else
                    cp_rgb = cpWallSelectedRGB;
            }
            else
                cp_rgb = cpDefaultRGB;

            // Define the marker radius
            GLfloat cp_rad = v_i == 3 ? cpSelectedMakerRadius : cpDefualtMakerRadius;

            // Convert cv::Point2f to GLfloat
            GLfloat cp_coord[] = {
                static_cast<GLfloat>(_CP_COORDS[c_i][v_i].x),
                static_cast<GLfloat>(_CP_COORDS[c_i][v_i].y)};

            // Bind position VBO and update data
            glBindBuffer(GL_ARRAY_BUFFER, CP_VBO_POS_ARR[c_i][v_i]);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(cp_coord), cp_coord);

            // Bind color VBO and update data
            glBindBuffer(GL_ARRAY_BUFFER, CP_VBO_RGB_ARR[c_i][v_i]);
            glBufferSubData(GL_ARRAY_BUFFER, 0, 3 * sizeof(GLfloat), cp_rgb.data());

            // Bind size VBO and update data
            glBindBuffer(GL_ARRAY_BUFFER, CP_VBO_RAD_ARR[c_i][v_i]);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(GLfloat), &cp_rad);

            // Draw the control point
            glDrawArrays(GL_POINTS, 0, 1);
        }
    }

    // Unbind the VAO
    glBindVertexArray(0);

    // Return GL status
    return checkErrorOpenGL(__LINE__, __FILE__);
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
            ROS_ERROR("Failed to load image from path: %s", img_path.c_str());
            return -1;
        }

        // Check if image has an alpha channel
        if (img.channels() != 4)
        {
            ROS_ERROR("Image does not have an alpha channel: %s", img_path.c_str());
            return -1;
        }

        // Check if image dimensions are as expected
        if (img.cols != WALL_WIDTH_PXL || img.rows != WALL_HEIGHT_PXL)
        {
            ROS_ERROR("Image dimensions do not match expected size: %s", img_path.c_str());
            return -1;
        }

        // Store the loaded image in the output vector
        out_img_mat_vec.push_back(img);
        ROS_INFO("Successfully loaded image with alpha channel from path: %s", img_path.c_str());
    }

    // Return success
    return 0;
}

int mergeImgMat(const cv::Mat &mask_img, cv::Mat &out_base_img)
{
    // Check if images are loaded successfully
    if (out_base_img.empty() || mask_img.empty())
    {
        ROS_ERROR("Error: Could not read one or both images.");
        return -1;
    }

    // Check dimensions
    if (out_base_img.size() != mask_img.size())
    {
        ROS_ERROR("Error: Image dimensions do not match.");
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

    // Initialize OpenGL wall image objects
    if (initializeWallObjects() != 0)
    {
        ROS_ERROR("[SETUP] Failed to Initialize OpenGL Wall Image Objects");
        return -1;
    }

    // Initialize OpenGL control point marker objects
    if (initializeControlPointObjects() != 0)
    {
        ROS_ERROR("[SETUP] Failed to Initialize OpenGL COntrol Point Marker Objects");
        return -1;
    }

    // Log OpenGL versions
    const GLubyte *opengl_version = glGetString(GL_VERSION);
    ROS_INFO("[SETUP] OpenGL Initialized: Version [%s]", opengl_version);

    // Log GLFW versions
    int glfw_major, glfw_minor, glfw_rev;
    glfwGetVersion(&glfw_major, &glfw_minor, &glfw_rev);
    ROS_INFO("[SETUP] GLFW Initialized: Version: %d.%d.%d", glfw_major, glfw_minor, glfw_rev);

    // Update monitor and window mode settings
    updateWindowMonMode(p_window_id, 0, pp_monitor_id_Vec, I.winMon, F.setFullscreen);

    // --------------- VARIABLE SETUP ---------------

    // Load images using OpenCV
    if (loadImgMat(wallImgPathVec, wallImgMatVec) != 0)
    {
        ROS_ERROR("[SETUP] Failed to Load Wall Images");
        return -1;
    }
    if (loadImgMat(monImgPathVec, monImgMatVec) != 0)
    {
        ROS_ERROR("[SETUP] Failed to Load Monitor Number Images");
        return -1;
    }
    if (loadImgMat(calImgPathVec, calImgMatVec) != 0)
    {
        ROS_ERROR("[SETUP] Failed to Load Calibration Mode Images");
        return -1;
    }

    // --------------- VARIABLE SETUP ---------------

    // _______________ MAIN LOOP _______________

    // bool is_error = false;
    // while (!glfwWindowShouldClose(p_window_id) && ros::ok())
    // {

    //     // --------------- Check Kayboard Callback Flags ---------------

    //     // Load XML file
    //     if (F.loadXML)
    //     {
    //         std::string file_path = formatCoordinatesFilePathXML(I.winMon, I.calMode, CONFIG_DIR_PATH);
    //         /// @todo Ad save xml back in
    //         F.loadXML = false;
    //     }

    //     // Save XML file
    //     if (F.saveXML)
    //     {
    //         std::string file_path = formatCoordinatesFilePathXML(I.winMon, I.calMode, CONFIG_DIR_PATH);
    //         /// @todo Ad save xml back in
    //         F.saveXML = false;
    //     }

    //     // Update the window monitor and mode
    //     if (F.updateWindowMonMode)
    //     {
    //         if (updateWindowMonMode(p_window_id, 0, pp_monitor_id_Vec, I.winMon, F.setFullscreen) != 0)
    //         {
    //             ROS_ERROR("[MAIN] Update Window Monitor Mode Threw Error");
    //             is_error = true;
    //             break;
    //         }
    //         F.updateWindowMonMode = false;
    //     }

    //     // Initialize/reinitialize control point coordinate dataset
    //     if (F.initControlPointMarkers)
    //     {
    //         initControlPointCoordinates(CP_COORDS);
    //         F.initControlPointMarkers = false;
    //     }

    //     // Recompute wall vertices and homography matrices
    //     if (F.updateWallDatasets)
    //     {
    //         // Initialize wall parameter datasets
    //         if (updateWallParameters(CP_COORDS, WALL_WARP_COORDS, WALL_HMAT_DATA) != 0)
    //         {
    //             ROS_ERROR("[MAIN] Update of Wall Vertices Datasets Failed");
    //             return -1;
    //         }

    //         // Initialize homography matrix dataset
    //         if (drawWallImages(fbo_texture_id, wallImgMatVec[I.wallImage], monImgMatVec[I.winMon], calImgMatVec[I.calMode]) != 0)
    //             if (updateWallHomography(CP_COORDS, WALL_WARP_COORDS, WALL_HMAT_DATA) != 0)
    //             {
    //                 ROS_ERROR("[MAIN] Update of Wall Homography Datasets Failed");
    //                 return -1;
    //             }
    //         F.updateWallDatasets = false;
    //     }

    //     // --------------- Handle Image Processing for Next Frame ---------------

    //     // Clear back buffer for new frame
    //     glClear(GL_COLOR_BUFFER_BIT);
    //     if (checkErrorOpenGL(__LINE__, __FILE__))
    //     {
    //         is_error = true;
    //         break;
    //     }

    //     // Draw/update wall images
    //     if (renderWallImage(WALL_TEXTURE_ID) != 0)
    //     {
    //         ROS_ERROR("[MAIN] Draw Walls Threw Error");
    //         is_error = true;
    //         break;
    //     }

    //     // Draw/update control point markers
    //     if (renderControlPoints(CP_COORDS) != 0)
    //     {
    //         ROS_ERROR("[MAIN] Draw Control Point Threw Error");
    //         is_error = true;
    //         break;
    //     }

    //     // Swap buffers and poll events
    //     glfwSwapBuffers(p_window_id);
    //     if (
    //         checkErrorGLFW(__LINE__, __FILE__) ||
    //         checkErrorOpenGL(__LINE__, __FILE__))
    //     {
    //         is_error = true;
    //         break;
    //     }

    //     // Poll events
    //     glfwPollEvents();

    //     // Exit condition
    //     if (glfwGetKey(p_window_id, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(p_window_id))
    //         break;
    // }

    // // _______________ CLEANUP _______________
    // ROS_INFO("SHUTTING DOWN");

    // // Check which condition caused the loop to exit
    // if (!ros::ok())
    //     ROS_INFO("[LOOP TERMINATION] ROS Node is no Longer in a Good State");
    // else if (glfwWindowShouldClose(p_window_id))
    //     ROS_INFO("[LOOP TERMINATION] GLFW Window Should Close");
    // else if (glfwGetKey(p_window_id, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    //     ROS_INFO("[LOOP TERMINATION] Escape Key was Pressed");
    // else if (is_error)
    //     ROS_INFO("[LOOP TERMINATION] Error Thrown");
    // else
    //     ROS_INFO("[LOOP TERMINATION] Reason Unknown");

    // // Delete wall texture
    // if (WALL_TEXTURE_ID != 0)
    // {
    //     glDeleteFramebuffers(1, &WALL_TEXTURE_ID);
    //     if (checkErrorOpenGL(__LINE__, __FILE__) != 0)
    //         ROS_WARN("[SHUTDOWN] Failed to Delete Wall Texture");
    //     else
    //         ROS_INFO("[SHUTDOWN] Deleted Wall Texture");
    // }
    // else
    //     ROS_WARN("[SHUTDOWN] No Wall Texture to Delete");

    // // Destroy GLFW window
    // if (p_window_id)
    // {
    //     glfwDestroyWindow(p_window_id);
    //     p_window_id = nullptr;
    //     if (checkErrorGLFW(__LINE__, __FILE__) != 0)
    //         ROS_WARN("[SHUTDOWN] Failed to Destroy GLFW Window");
    //     else
    //         ROS_INFO("[SHUTDOWN] Destroyed GLFW Window");
    // }
    // else
    // {
    //     ROS_WARN("[SHUTDOWN] No GLFW window to destroy");
    // }

    // // Terminate GLFW
    // glfwTerminate();
    // checkErrorGLFW(__LINE__, __FILE__);
    // ROS_INFO("[SHUTDOWN] Terminated GLFW");

    // // Return success
    // return is_error ? -1 : 0;

    // --------------- TEST IMAGE SETUP ---------------

    // Use first wall image
    cv::Mat im_wall = wallImgMatVec[0];
    cv::Mat im_wall_mask = monImgMatVec[0];

    // Test merge image
    mergeImgMat(im_wall_mask, im_wall);

    // Populate the source correspondence points
    /// @note Assumes Y-axis points down
    std::vector<cv::Point2f> srcPoints = {
        cv::Point2f(0, 0),                            // Top-left (0,0)
        cv::Point2f(WALL_WIDTH_PXL, 0),               // Top-right (1,0)
        cv::Point2f(WALL_WIDTH_PXL, WALL_HEIGHT_PXL), // Bottom-right (1,1)
        cv::Point2f(0, WALL_HEIGHT_PXL)};             // Bottom-left (0,1)

    // Populate the destination correspondence points
    std::vector<cv::Point2f> dstPoints = {
        cv::Point2f(375, 230),
        cv::Point2f(675, 205),
        cv::Point2f(600, 695),
        cv::Point2f(350, 770)};

    // Find Homography
    cv::Mat H1 = cv::findHomography(srcPoints, dstPoints);
    // Warp Perspective
    cv::Mat im_warp1;
    cv::warpPerspective(im_wall, im_warp1, H1, cv::Size(PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL));

    // Test second image

    // Loop through dstPoints
    for (int i = 0; i < dstPoints.size(); i++)
    {
        // Update srcPoints
        dstPoints[i].x += WALL_WIDTH_PXL + 50;
    }

    // Find Homography
    cv::Mat H2 = cv::findHomography(srcPoints, dstPoints);
    // Warp Perspective
    cv::Mat im_warp2;
    cv::warpPerspective(im_wall, im_warp2, H2, cv::Size(PROJ_WIN_WIDTH_PXL, PROJ_WIN_HEIGHT_PXL));

    // Merge images
    mergeImgMat(im_warp1, im_warp2);

    // Make texture
    WALL_TEXTURE_ID = loadTexture(im_warp2);

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
    dbLogHomMat(H1);

    // Main loop (unchanged)
    while (!glfwWindowShouldClose(p_window_id))
    {

        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT);

        // Use the shader program for wall rendering
        glUseProgram(WALL_SHADER);

        // Bind the texture for the walls
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, WALL_TEXTURE_ID);

        // Bind the Vertex Array Object(VAO) specific to the current wall
        glBindVertexArray(WALL_VAO);

        // Bind the common Element Buffer Object (EBO)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, WALL_EBO);

        // Draw the rectangle (2 triangles) for the current wall
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

        // Unbind the VAO to prevent accidental modification
        glBindVertexArray(0);

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
