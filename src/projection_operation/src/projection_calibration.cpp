// ############################################################################################################

// ======================================== projection_calibration.cpp ========================================

// ############################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_calibration.h"

// ================================================== CLASSES ==================================================

CircleRenderer::CircleRenderer()
    : circPosition(cv::Point2f(0.0f, 0.0f)), cirRadius(1.0f), circColor(cv::Scalar(1.0, 1.0, 1.0)),
      circSegments(32), circRotationAngle(0.0f), circScalingFactors(cv::Point2f(1.0f, 1.0f))
{
    // Define instance count and itterate static IDX
    circID = IDX++;

    // Initialize the transformation matrix as an identity matrix
    _transformationMatrix = cv::Mat::eye(4, 4, CV_32F);
}

CircleRenderer::~CircleRenderer()
{
    glDeleteVertexArrays(1, &_VAO);
    glDeleteBuffers(1, &_VBO);
}

void CircleRenderer::initializeCircleRenderer(cv::Point2f pos, float rad, cv::Scalar col, unsigned int segments)
{
    // Set variables
    circPosition = pos;
    cirRadius = rad;
    circColor = col;
    circSegments = segments;

    // Initialize variables
    circScalingFactors = cv::Point2f(1.0f, 1.0f);
    circRotationAngle = 0.0f;

    // Run initial vertex computation
    _computeVertices(circPosition, cirRadius, circSegments, _circVertices);

    // Setup OpenGL buffers
    _setupOpenGL();

    // // TEMP
    // ROS_INFO("[CircleRenderer] Initialized Instance[%d]: Position[%0.2f, %0.2f] Radius[%0.4f] Color[%.2f, %.2f, %.2f] Segments[%d]",
    //          circID, circPosition.x, circPosition.y, cirRadius, circColor[0], circColor[1], circColor[2], circSegments);
}

void CircleRenderer::setPosition(cv::Point2f pos)
{
    // Modify the y position based on the aspect ratio
    pos.y /= _ASPECT_RATIO_UNIFORM;
    circPosition = pos;
}

void CircleRenderer::setRadius(float rad)
{
    cirRadius = rad;
}

void CircleRenderer::setRotationAngle(float angle)
{
    circRotationAngle = angle;
}

void CircleRenderer::setScaling(cv::Point2f scaling_factors)
{
    circScalingFactors = scaling_factors;
}

void CircleRenderer::setColor(cv::Scalar col)
{
    circColor = col;
}

void CircleRenderer::recomputeParameters()
{
    // Update the transformation matrix
    _computeTransformation();

    // Generate the new vertices based on the current position, radius, and circSegments
    _computeVertices(circPosition, cirRadius, circSegments, _circVertices);

    // Bind the VBO to the GL_ARRAY_BUFFER target
    glBindBuffer(GL_ARRAY_BUFFER, _VBO);

    // Update the VBO's data with the new vertices. This call will reallocate the buffer if necessary
    // or simply update the data store's contents if the buffer is large enough.
    glBufferData(GL_ARRAY_BUFFER, _circVertices.size() * sizeof(float), _circVertices.data(), GL_DYNAMIC_DRAW);

    // Unbind the buffer
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // // TEMP
    // ROS_INFO("[CircleRenderer] Initialized Instance[%d]: Position[%0.2f, %0.2f] Radius[%0.4f] Color[%.2f, %.2f, %.2f] Segments[%d]",
    //          circID, circPosition.x, circPosition.y, cirRadius, circColor[0], circColor[1], circColor[2], circSegments);
}

void CircleRenderer::draw()
{
    // Set color
    glUniform4f(_COLOR_LOCATION, circColor[0], circColor[1], circColor[2], 1.0f);

    // Use the updated transformation matrix instead of an identity matrix
    auto transformArray = _cvMatToGlArray(_transformationMatrix);
    glUniformMatrix4fv(_TRANSFORM_LOCATION, 1, GL_FALSE, transformArray.data());

    // Bind the VAO and draw the circle using GL_TRIANGLE_FAN
    glBindVertexArray(_VAO);
    glDrawArrays(GL_TRIANGLE_FAN, 0, static_cast<GLsizei>(_circVertices.size() / 2));

    // Unbind the VAO to leave a clean state
    glBindVertexArray(0);
}

// Static method to setup shader program and get uniform locations
int CircleRenderer::compileAndLinkCircleShaders(float aspect_ratio)
{
    // Set the aspect ratio
    _ASPECT_RATIO_UNIFORM = aspect_ratio;

    // Compile vertex shader
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
    glCompileShader(vertexShader);
    if (!_checkShaderCompilation(vertexShader))
    {
        ROS_ERROR("[CircleRenderer] Vertex shader compilation failed.");
        return -1;
    }

    // Compile fragment shader
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
    glCompileShader(fragmentShader);
    if (!_checkShaderCompilation(fragmentShader))
    {
        ROS_ERROR("[CircleRenderer] Fragment shader compilation failed.");
        return -1;
    }

    // Link shaders into a program
    _SHADER_PROGRAM = glCreateProgram();
    glAttachShader(_SHADER_PROGRAM, vertexShader);
    glAttachShader(_SHADER_PROGRAM, fragmentShader);
    glLinkProgram(_SHADER_PROGRAM);
    if (!_checkProgramLinking(_SHADER_PROGRAM))
    {
        ROS_ERROR("[CircleRenderer] Shader program linking failed.");
        return -1;
    }

    // After linking, shaders can be deleted
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // Get uniform locations
    _COLOR_LOCATION = glGetUniformLocation(_SHADER_PROGRAM, "color");
    _TRANSFORM_LOCATION = glGetUniformLocation(_SHADER_PROGRAM, "transform");
    _ASPECT_RATIO_LOCATION = glGetUniformLocation(_SHADER_PROGRAM, "aspectRatio");

    // Check for errors in getting uniforms
    if (_COLOR_LOCATION == -1 || _TRANSFORM_LOCATION == -1 || _ASPECT_RATIO_LOCATION == -1)
    {
        ROS_ERROR("[CircleRenderer] Error getting uniform locations.");
        return -1;
    }

    return 0; // Success
}

int CircleRenderer::cleanupShaderObjects()
{
    // Use OpenGL calls to delete shader program and shaders
    if (_SHADER_PROGRAM != 0)
    {
        glDeleteProgram(_SHADER_PROGRAM);
        _SHADER_PROGRAM = 0;
    }
    else
    {
        return -1;
    }
}

void CircleRenderer::setupShaderForDrawing()
{
    // Use the shader program
    glUseProgram(_SHADER_PROGRAM);

    // Set the aspect ratio uniform
    glUniform1f(_ASPECT_RATIO_LOCATION, _ASPECT_RATIO_UNIFORM);
}

void CircleRenderer::unsetShaderForDrawing()
{
    // Unset the shader program
    glUseProgram(0);
}

bool CircleRenderer::_checkShaderCompilation(GLuint shader)
{
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        // Get and log the error message
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        ROS_ERROR("[CircleRenderer] Shader compilation error: %s", infoLog);
        return false;
    }
    return true;
}

bool CircleRenderer::_checkProgramLinking(GLuint program)
{
    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success)
    {
        // Get and log the error message
        char infoLog[512];
        glGetProgramInfoLog(program, 512, nullptr, infoLog);
        ROS_ERROR("[CircleRenderer] Program linking error: %s", infoLog);
        return false;
    }
    return true;
}

void CircleRenderer::_setupOpenGL()
{
    // Generate a new Vertex Array Object (VAO) and store the ID
    glGenVertexArrays(1, &_VAO);
    // Generate a new Vertex Buffer Object (VBO) and store the ID
    glGenBuffers(1, &_VBO);

    // Bind the VAO to set it up
    glBindVertexArray(_VAO);

    // Bind the VBO to the GL_ARRAY_BUFFER target
    glBindBuffer(GL_ARRAY_BUFFER, _VBO);
    // Copy vertex data into the buffer's memory (GL_DYNAMIC_DRAW hints that the data might change often)
    glBufferData(GL_ARRAY_BUFFER, _circVertices.size() * sizeof(float), _circVertices.data(), GL_DYNAMIC_DRAW);

    // Define an array of generic vertex attribute data. Arguments:
    // 0: the index of the vertex attribute to be modified.
    // 2: the number of components per generic vertex attribute. Since we're using 2D points, this is 2.
    // GL_FLOAT: the data type of each component in the array.
    // GL_FALSE: specifies whether fixed-point data values should be normalized or not.
    // 2 * sizeof(float): the byte offset between consecutive vertex attributes.
    // (void*)0: offset of the first component of the first generic vertex attribute in the array.
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);

    // Enable the vertex attribute array for the VAO at index 0
    glEnableVertexAttribArray(0);

    // Unbind the VAO to prevent further modification.
    glBindVertexArray(0);
}

void CircleRenderer::_computeVertices(cv::Point2f position, float radius, unsigned int circSegments, std::vector<float> &circVertices)
{
    // Clear the vertex vector to prepare for new vertices
    circVertices.clear();

    // Loop to generate vertices for the circle
    for (unsigned int i = 0; i <= circSegments; ++i)
    {
        // Calculate the angle for the current segment
        float angle = 2.0f * std::acos(-1.0) * i / circSegments;
        // Determine the x position of the vertex on the circle
        float baseX = position.x + (radius * std::cos(angle));
        // Determine the y position of the vertex on the circle
        float baseY = position.y + (radius * std::sin(angle));

        // Create a 4x1 matrix (homogeneous coordinates) for the vertex
        cv::Mat vertex = (cv::Mat_<float>(4, 1) << baseX, baseY, 0, 1);
        // Apply the transformation matrix to the vertex
        cv::Mat transformedVertex = _transformationMatrix * vertex;
        // Extract the transformed x coordinate and add to the vertices list
        circVertices.push_back(transformedVertex.at<float>(0, 0));
        // Extract the transformed y coordinate and add to the vertices list
        circVertices.push_back(transformedVertex.at<float>(1, 0));
    }
}

void CircleRenderer::_computeTransformation()
{
    // Initialize transformation matrix as identity if not done
    if (_transformationMatrix.empty())
    {
        _transformationMatrix = cv::Mat::eye(4, 4, CV_32F);
    }

    // (1) Translate to the origin
    cv::Mat translationToOrigin = cv::Mat::eye(4, 4, CV_32F);
    translationToOrigin.at<float>(0, 3) = -circPosition.x;
    translationToOrigin.at<float>(1, 3) = -circPosition.y;

    // (2) Rotate around the origin (create a 3x3 rotation matrix first)
    cv::Mat rotation2D = cv::getRotationMatrix2D(cv::Point2f(0, 0), circRotationAngle, 1.0);
    cv::Mat rotation = cv::Mat::eye(4, 4, CV_32F); // Convert to 4x4 matrix
    rotation2D.copyTo(rotation.rowRange(0, 2).colRange(0, 3));

    // (3) Scale the circle by the scaling factors
    cv::Mat scaling = cv::Mat::eye(4, 4, CV_32F);
    scaling.at<float>(0, 0) = circScalingFactors.x;
    scaling.at<float>(1, 1) = circScalingFactors.y;

    // (4) Translate back to the original position
    cv::Mat translationBack = cv::Mat::eye(4, 4, CV_32F);
    translationBack.at<float>(0, 3) = circPosition.x;
    translationBack.at<float>(1, 3) = circPosition.y;

    // The multiplication order here is important
    _transformationMatrix = translationBack * scaling * rotation * translationToOrigin;
}

// Helper function to convert OpenCV Mat to an array for OpenGL
std::array<float, 16> CircleRenderer::_cvMatToGlArray(const cv::Mat &mat)
{
    assert(mat.cols == 4 && mat.rows == 4 && mat.type() == CV_32F);
    std::array<float, 16> glArray;
    std::copy(mat.begin<float>(), mat.end<float>(), glArray.begin());
    return glArray;
}

// Initialize static members
int CircleRenderer::IDX = 0;
GLuint CircleRenderer::_SHADER_PROGRAM = 0;
GLint CircleRenderer::_COLOR_LOCATION = -1;
GLint CircleRenderer::_TRANSFORM_LOCATION = -1;
GLint CircleRenderer::_ASPECT_RATIO_LOCATION = -1;
float CircleRenderer::_ASPECT_RATIO_UNIFORM = 1.0f;

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
    ROS_DEBUG("[callbackDebugOpenGL] Type[0x%x] ID[%d] Severity[0x%x] Message[%s]", type, id, severity, message);
}

static void callbackErrorGLFW(int error, const char *description)
{
    ROS_ERROR("[callbackErrorGLFW] Error[%d] Description[%s]", error, description);
}

int checkErrorOpenGL(int line, const char *file_str, const char *msg_str)
{
    GLenum gl_err;
    while ((gl_err = glGetError()) != GL_NO_ERROR)
    {
        ROS_ERROR("[checkErrorOpenGL] Message[%s] Error Number[%u] File[%s] Line[%d]",
                  msg_str ? msg_str : "No additional info", gl_err, file_str, line);
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
            ROS_ERROR("[checkErrorGLFW] Message[%s] Description[%s] File[%s] Line[%d]", msg_str, description, file_str, line);
        else
            ROS_ERROR("[checkErrorGLFW] Description[%s] File[%s] Line[%d]", description, file_str, line);
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
            ROS_ERROR("[updateWindowMonMode] Failed to Get Video Mode: Monitor[%d]", mon_id_ind);
            return -1;
        }

        // Set the window to full-screen mode on the current monitor
        glfwSetWindowMonitor(p_window_id, p_monitor_id, 0, 0, mode->width, mode->height, mode->refreshRate);
        if (!p_monitor_id)
        {
            ROS_ERROR("[updateWindowMonMode] Invalid Monitor Pointer: Monitor[%d]", mon_id_ind);
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
                ROS_WARN("[updateWindowMonMode] Invalid Monitor Position: Monitor[%d] X[%d] Y[%d]", mon_id_ind, monitor_x, monitor_y);
                return 0;
            }

            // Set the window to windowed mode and position it on the current monitor
            glfwSetWindowMonitor(p_window_id, NULL, monitor_x + 100, monitor_y + 100, (int)(500.0f * WINDOW_ASPECT_RATIO), 500, 0);
        }

        // Update window title
        std::string new_title = "Window[" + std::to_string(win_ind) + "] Monitor[" + std::to_string(mon_id_ind) + "]";
        glfwSetWindowTitle(p_window_id, new_title.c_str());

        ROS_INFO("[updateWindowMonMode] Move Window: Monitor[%d] Format[%s]", mon_id_ind, is_fullscreen ? "fullscreen" : "windowed");
    }
    else
    {
        ROS_WARN("[updateWindowMonMode] Failed Move Window: Monitor[%d] Format[%s]", mon_id_ind, is_fullscreen ? "fullscreen" : "windowed");
        return 0;
    }

    // Update last monitor and fullscreen mode
    imp_mon_id_ind_last = mon_id_ind;
    is_fullscreen_last = is_fullscreen;

    return 0;
}

void initControlPointCoordinates(std::array<std::array<cv::Point2f, 4>, 4> &out_CP_COORDS)
{
    // Specify the control point limits
    const float cp_x = MAZE_WIDTH_NDC / 2;  // starting X-coordinate in NDC coordinates
    const float cp_y = MAZE_HEIGHT_NDC / 2; // starting Y-coordinate in NDC coordinates

    // Iterate through control point outer array (corners)
    for (float cp_i = 0; cp_i < 4; cp_i++) // image bottom to top
    {
        cv::Point2f p_org;

        // 0: image top-left
        if (cp_i == 0)
            p_org = cv::Point2f(-cp_x, -cp_y);

        // 1: image top-right
        else if (cp_i == 1)
            p_org = cv::Point2f(+cp_x, -cp_y);

        // 2: image bottom-right
        else if (cp_i == 2)
            p_org = cv::Point2f(+cp_x, +cp_y);

        // 3: image bottom-left
        else if (cp_i == 3)
            p_org = cv::Point2f(-cp_x, +cp_y);

        // Set x y values for each vertex
        out_CP_COORDS[cp_i] = {
            cv::Point2f(p_org.x, p_org.y),                                                // top left
            cv::Point2f(p_org.x + WALL_IMAGE_WIDTH_NDC, p_org.y),                         // top right
            cv::Point2f(p_org.x + WALL_IMAGE_WIDTH_NDC, p_org.y + WALL_IMAGE_HEIGHT_NDC), // bottom right
            cv::Point2f(p_org.x, p_org.y + WALL_IMAGE_HEIGHT_NDC),                        // bottom left
        };
    }
}

int renderWallImage(const GLuint &_WALL_TEXTURE_ID, const GLuint &_WALL_SHADER, const GLuint &_WALL_VAO, const GLuint &_WALL_EBO)
{
    // Use the shader program for wall rendering
    glUseProgram(_WALL_SHADER);

    // Bind the texture for the walls
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, _WALL_TEXTURE_ID);

    // Bind the Vertex Array Object(VAO) specific to the current wall
    glBindVertexArray(_WALL_VAO);

    // Bind the common Element Buffer Object (EBO)
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _WALL_EBO);

    // Draw the rectangle (2 triangles) for the current wall
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    // Unbind the VAO to prevent accidental modification
    glBindVertexArray(0);

    // Unset the shader program
    glUseProgram(0);

    // Return GL status
    return checkErrorOpenGL(__LINE__, __FILE__);
}

int renderControlPoints(const std::array<std::array<cv::Point2f, 4>, 4> &_CP_COORDS,
                        std::array<std::array<CircleRenderer, 4>, 4> &out_CP_RENDERERS)
{
    // Setup the CircleRenderer class shaders
    CircleRenderer::setupShaderForDrawing();

    // Loop through the control points and draw them
    for (int c_i = 0; c_i < 4; ++c_i)
    {
        for (int v_i = 0; v_i < 4; ++v_i)
        {
            // Define the marker color
            cv::Scalar col;
            // Selected conrnor and wall vertex
            if (c_i == I.cpSelected[0] && v_i == I.cpSelected[1])
                col = cpVertSelectedRGB;
            // Selected conrnor but not wall vertex
            else if (c_i == I.cpSelected[0] && v_i != I.cpSelected[1])
                col = cpCornerSelectedRGB;
            // Unselected
            else
                col = cpDefaultRGB;

            // Define the marker radius
            GLfloat rad = v_i == 3 ? cpSelectedMakerRadius : cpDefualtMakerRadius;

            // Set the marker parameters
            out_CP_RENDERERS[c_i][v_i].setPosition(_CP_COORDS[c_i][v_i]);
            out_CP_RENDERERS[c_i][v_i].setRadius(rad);
            out_CP_RENDERERS[c_i][v_i].setColor(col);

            // Recompute the marker parameters
            out_CP_RENDERERS[c_i][v_i].recomputeParameters();

            // Draw the marker
            out_CP_RENDERERS[c_i][v_i].draw();

            // Check for errors
            if (checkErrorOpenGL(__LINE__, __FILE__) < 0)
            {
                ROS_ERROR("[renderControlPoints] Error Thrown for Control Point[%d][%d]", c_i, v_i);
                return -1;
            }
        }
    }

    // Unset the shader program
    CircleRenderer::unsetShaderForDrawing();

    // // TEMP
    // return -1;

    // Return GL status
    return checkErrorOpenGL(__LINE__, __FILE__);
}

// int renderControlPoints(const std::array<std::array<cv::Point2f, 4>, 4> &_CP_COORDS,
//                         std::array<std::array<CircleRenderer, 4>, 4> &out_CP_RENDERERS)
// {
//     // Setup the CircleRenderer class shaders
//     CircleRenderer::setupShaderForDrawing();

//     out_CP_RENDERERS[0][0].setPosition(cv::Point2f(-0.5f, 0.5f));

//     out_CP_RENDERERS[0][0].setRadius(0.05f);

//     //out_CP_RENDERERS[0][0].setColor(cv::Scalar(0.0f, 0.0f, 1.0f));
//     out_CP_RENDERERS[0][0].setColor(cpDefaultRGB);

//     // Recompute the marker parameters
//     out_CP_RENDERERS[0][0].recomputeParameters();

//     // Draw the marker
//     out_CP_RENDERERS[0][0].draw();

//     // Check for errors
//     if (checkErrorOpenGL(__LINE__, __FILE__) < 0)
//     {
//         ROS_ERROR("[renderControlPoints] Error Thrown for Control Point");
//         return -1;
//     }

//     // Unset the shader program
//     CircleRenderer::unsetShaderForDrawing();

//     // Return GL status
//     return checkErrorOpenGL(__LINE__, __FILE__);
// }

int initializeCircleRendererObjects(const std::array<std::array<cv::Point2f, 4>, 4> &_CP_COORDS,
                                    std::array<std::array<CircleRenderer, 4>, 4> &out_CP_RENDERERS)
{
    // Iterate through control point outer array (corners)
    for (float cp_i = 0; cp_i < 4; cp_i++) // image bottom to top
    {
        // Iterate through control point inner array to initialize the CircleRenderer class objects array
        for (int cv_i = 0; cv_i < 4; ++cv_i)
        {
            out_CP_RENDERERS[cp_i][cv_i].initializeCircleRenderer(
                _CP_COORDS[cp_i][cv_i], // position
                cpDefualtMakerRadius,   // radius
                cpDefaultRGB,           // color
                cpRenderSegments        // segments
            );
        }
    }

    // Return GL status
    return checkErrorOpenGL(__LINE__, __FILE__);
}

int initializeWallRenderObjects(GLuint &_WALL_VAO, GLuint &_WALL_VBO, GLuint &_WALL_EBO)
{

    // Generate and bind an Element Buffer Object (EBO)
    glGenBuffers(1, &_WALL_EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _WALL_EBO);

    // Initialize the EBO with index data
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(WALL_GL_INDICES), WALL_GL_INDICES, GL_DYNAMIC_DRAW);

    // Generate and bind a Vertex Array Object (VAO)
    glGenVertexArrays(1, &_WALL_VAO);
    glBindVertexArray(_WALL_VAO);

    // Generate and bind a Vertex Buffer Object (VBO)
    glGenBuffers(1, &_WALL_VBO);
    glBindBuffer(GL_ARRAY_BUFFER, _WALL_VBO);

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

    // Return GL status
    return checkErrorOpenGL(__LINE__, __FILE__);
}

int compileAndLinkShaders(const GLchar *vertex_source, const GLchar *fragment_source, GLuint &shader_program_out)
{
    auto checkCompileErrors = [](GLuint shader, const std::string &type) -> bool
    {
        GLint success;
        GLchar infoLog[1024];
        if (type != "PROGRAM")
        {
            glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
            if (!success)
            {
                glGetShaderInfoLog(shader, 1024, NULL, infoLog);
                ROS_ERROR("[Shader Compilation Error] Type: %s\n%s", type.c_str(), infoLog);
                return false;
            }
        }
        else
        {
            glGetProgramiv(shader, GL_LINK_STATUS, &success);
            if (!success)
            {
                glGetProgramInfoLog(shader, 1024, NULL, infoLog);
                ROS_ERROR("[Shader Linking Error] Type: %s\n%s", type.c_str(), infoLog);
                return false;
            }
        }
        return true;
    };

    GLuint shader_program = glCreateProgram();
    if (!shader_program)
    {
        ROS_ERROR("[Shader Program Error] Unable to create shader program.");
        return -1;
    }

    GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    if (!vertex_shader)
    {
        ROS_ERROR("[Shader Program Error] Unable to create vertex shader.");
        glDeleteProgram(shader_program);
        return -1;
    }

    GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    if (!fragment_shader)
    {
        ROS_ERROR("[Shader Program Error] Unable to create fragment shader.");
        glDeleteShader(vertex_shader);
        glDeleteProgram(shader_program);
        return -1;
    }

    // Vertex Shader
    glShaderSource(vertex_shader, 1, &vertex_source, NULL);
    glCompileShader(vertex_shader);
    if (!checkCompileErrors(vertex_shader, "VERTEX"))
    {
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);
        glDeleteProgram(shader_program);
        return -1;
    }
    glAttachShader(shader_program, vertex_shader);

    // Fragment Shader
    glShaderSource(fragment_shader, 1, &fragment_source, NULL);
    glCompileShader(fragment_shader);
    if (!checkCompileErrors(fragment_shader, "FRAGMENT"))
    {
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);
        glDeleteProgram(shader_program);
        return -1;
    }
    glAttachShader(shader_program, fragment_shader);

    // Link the shader program
    glLinkProgram(shader_program);
    if (!checkCompileErrors(shader_program, "PROGRAM"))
    {
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);
        glDeleteProgram(shader_program);
        return -1;
    }

    shader_program_out = shader_program;

    // Cleanup: detach and delete shaders
    glDetachShader(shader_program, vertex_shader);
    glDeleteShader(vertex_shader);
    glDetachShader(shader_program, fragment_shader);
    glDeleteShader(fragment_shader);

    // No need to delete the program as it's being used outside this function

    // Return GL status
    return checkErrorOpenGL(__LINE__, __FILE__);
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

int updateWallTexture(
    cv::Mat img_wall_mat, cv::Mat img_mode_mon_mat, cv::Mat img_mode_cal_mat,
    std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> &_WALL_HMAT_DATA,
    GLuint &out_WALL_TEXTURE_ID)
{
    // Initializ the image to be used as the texture
    cv::Mat im_wall_merge = cv::Mat::zeros(WINDOW_HEIGHT_PXL, WINDOW_WIDTH_PXL, CV_8UC4);

    // Iterate through the maze grid rows
    for (float grow_i = 0; grow_i < MAZE_SIZE; grow_i++) // image bottom to top
    {
        // Iterate through each column in the maze row
        for (float gcol_i = 0; gcol_i < MAZE_SIZE; gcol_i++) // image left to right
        {
            // Copy wall image
            cv::Mat img_copy;
            img_wall_mat.copyTo(img_copy);

            //  Create merged image for the wall corresponding to the selected control point
            if (
                (I.cpSelected[0] == 0 && grow_i == 0 && gcol_i == 0) ||
                (I.cpSelected[0] == 1 && grow_i == 0 && gcol_i == MAZE_SIZE - 1) ||
                (I.cpSelected[0] == 2 && grow_i == MAZE_SIZE - 1 && gcol_i == 0) ||
                (I.cpSelected[0] == 3 && grow_i == MAZE_SIZE - 1 && gcol_i == MAZE_SIZE - 1))
            {
                ;
                // Merge test pattern and active monitor image
                if (mergeImgMat(img_mode_mon_mat, img_copy) < 0)
                    return -1;

                // Merge previous image and active calibration image
                if (mergeImgMat(img_mode_cal_mat, img_copy) < 0)
                    return -1;
            }

            // Get homography matrix for this wall
            cv::Mat H = _WALL_HMAT_DATA[grow_i][gcol_i];

            // Warp Perspective
            cv::Mat im_warp;
            cv::warpPerspective(img_copy, im_warp, H, cv::Size(WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL));

            // Merge the warped image with the final image
            if (mergeImgMat(im_warp, im_wall_merge) < 0)
                return -1;

            // // TEMP
            // cv::namedWindow("Warped Image Display", cv::WINDOW_AUTOSIZE);
            // cv::imshow("Warped Image Display", im_warp);
            // cv::waitKey(0);
            // cv::destroyWindow("Warped Image Display");
            // break;
        }
    }

    // Make the new texture
    out_WALL_TEXTURE_ID = loadTexture(im_wall_merge);

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
    ROS_INFO("[SETUP] Display: Width[%d] Height[%d] AR[%0.2f]", WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL, WINDOW_ASPECT_RATIO);
    ROS_INFO("[SETUP] Wall (Pxl): Width[%d] Height[%d]", WALL_IMAGE_WIDTH_PXL, WALL_IMAGE_HEIGHT_PXL);
    ROS_INFO("[SETUP] Wall (NDC): Width[%0.2f] Height[%0.2f] Space Horz[%0.2f] Space Vert[%0.2f]", WALL_IMAGE_WIDTH_NDC, WALL_IMAGE_HEIGHT_NDC);
    ROS_INFO("[SETUP] Origin Plane (NDC): Width[%0.2f] Height[%0.2f]", WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL);

    // --------------- VARIABLE SETUP ---------------

    // Initialize control point coordinate dataset
    initControlPointCoordinates(CP_COORDS);

    // Initialize wall homography matrices array
    if (updateHomographyMatrices(CP_COORDS, WALL_HMAT_DATA) < 0)
    {
        ROS_ERROR("[SETUP] Failed to Initialize Wall Parameters");
        return -1;
    }

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
    p_window_id = glfwCreateWindow(WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL, "", NULL, NULL);
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
    if (initializeWallRenderObjects(WALL_VAO, WALL_VBO, WALL_EBO) < 0)
    {
        ROS_ERROR("[SETUP] Failed to Initialize OpenGL Wall Image Objects");
        return -1;
    }

    // Update monitor and window mode settings
    updateWindowMonMode(p_window_id, 0, pp_monitor_id_Vec, I.winMon, F.setFullscreen);

    // Create the shader program for wall image rendering
    if (compileAndLinkShaders(wallVertexSource, wallFragmentSource, WALL_SHADER) < 0)
    {
        ROS_ERROR("[SETUP] Failed to Compile and Link Wall Shader");
        return -1;
    }

    // Create the shader program for CircleRenderer class control point rendering
    if (CircleRenderer::compileAndLinkCircleShaders(WINDOW_ASPECT_RATIO) < 0)
    {
        ROS_ERROR("[SETUP] Failed to Compile and Link CircleRenderer Class Shader");
        return -1;
    }

    // Initialize the CircleRenderer class objects array
    if (initializeCircleRendererObjects(CP_COORDS, CP_RENDERERS) < 0)
    {
        ROS_ERROR("[SETUP] Failed to Initialize Control Point Variables");
        return -1;
    }

    if (checkErrorOpenGL(__LINE__, __FILE__) < 0)
    {
        ROS_ERROR("[SETUP] Error Thrown During OpenGL Setup");
        return -1;
    }

    // Log OpenGL versions
    const GLubyte *opengl_version = glGetString(GL_VERSION);
    ROS_INFO("[SETUP] OpenGL Initialized: Version [%s]", opengl_version);

    // Log GLFW versions
    int glfw_major, glfw_minor, glfw_rev;
    glfwGetVersion(&glfw_major, &glfw_minor, &glfw_rev);
    ROS_INFO("[SETUP] GLFW Initialized: Version: %d.%d.%d", glfw_major, glfw_minor, glfw_rev);

    // --------------- RENDER IMAGE LOADING  ---------------

    // Load images using OpenCV
    if (loadImgMat(wallImgPathVec, wallImgMatVec) < 0)
    {
        ROS_ERROR("[SETUP] Failed to Load Wall Images");
        return -1;
    }
    if (loadImgMat(monImgPathVec, monImgMatVec) < 0)
    {
        ROS_ERROR("[SETUP] Failed to Load Monitor Number Images");
        return -1;
    }
    if (loadImgMat(calImgPathVec, calImgMatVec) < 0)
    {
        ROS_ERROR("[SETUP] Failed to Load Calibration Mode Images");
        return -1;
    }

    // Initialize wall image texture
    if (updateWallTexture(wallImgMatVec[I.wallImage], monImgMatVec[I.winMon], calImgMatVec[I.calMode], WALL_HMAT_DATA, WALL_TEXTURE_ID) < 0)
    {
        ROS_ERROR("[SETUP] Failed to Initialize Wall Texture");
        return -1;
    }

    // _______________ MAIN LOOP _______________

    bool is_error = false;
    while (!glfwWindowShouldClose(p_window_id) && ros::ok())
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
            if (updateWindowMonMode(p_window_id, 0, pp_monitor_id_Vec, I.winMon, F.setFullscreen) < 0)
            {
                ROS_ERROR("[MAIN] Update Window Monitor Mode Threw an Error");
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

        // Recompute wall parameters and update wall image texture
        if (F.updateWallDatasets)
        {
            // Update wall homography matrices array
            if (updateHomographyMatrices(CP_COORDS, WALL_HMAT_DATA) < 0)
            {
                ROS_ERROR("[MAIN] Update of Wall Vertices Datasets Failed");
                is_error = true;
                break;
            }

            // Update wall image texture
            if (updateWallTexture(wallImgMatVec[I.wallImage], monImgMatVec[I.winMon], calImgMatVec[I.calMode], WALL_HMAT_DATA, WALL_TEXTURE_ID) < 0)
            {
                ROS_ERROR("[MAIN] Update of Wall Homography Datasets Failed");
                is_error = true;
                break;
            }
            F.updateWallDatasets = false;
        }

        // --------------- Handle Image Processing for Next Frame ---------------

        // Clear back buffer for new frame
        glClear(GL_COLOR_BUFFER_BIT);
        if (checkErrorOpenGL(__LINE__, __FILE__))
        {
            ROS_ERROR("[MAIN] Error Following glClear() Call Threw an Error");
            is_error = true;
            break;
        }

        // Draw/update wall images
        if (renderWallImage(WALL_TEXTURE_ID, WALL_SHADER, WALL_VAO, WALL_EBO) < 0)
        {
            ROS_ERROR("[MAIN] Draw Walls Threw an Error");
            is_error = true;
            break;
        }

        // Draw/update control point markers
        if (renderControlPoints(CP_COORDS, CP_RENDERERS) < 0)
        {
            ROS_ERROR("[MAIN] Draw Control Point Threw an Error");
            is_error = true;
            break;
        }

        // Swap buffers and poll events
        glfwSwapBuffers(p_window_id);
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
        if (glfwGetKey(p_window_id, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(p_window_id))
            break;
    }

    // _______________ CLEANUP _______________
    ROS_INFO("SHUTTING DOWN");

    // Check which condition caused the loop to exit
    if (!ros::ok())
        ROS_INFO("[LOOP TERMINATION] ROS Node is no Longer in a Good State");
    else if (glfwWindowShouldClose(p_window_id))
        ROS_INFO("[LOOP TERMINATION] GLFW Window Should Close");
    else if (glfwGetKey(p_window_id, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        ROS_INFO("[LOOP TERMINATION] Escape Key was Pressed");
    else if (is_error)
        ROS_INFO("[LOOP TERMINATION] Error Thrown");
    else
        ROS_INFO("[LOOP TERMINATION] Reason Unknown");

    // Delete wall shader program
    if (WALL_SHADER != 0)
    {
        glDeleteProgram(WALL_SHADER);
        ROS_INFO("[SHUTDOWN] Deleted WALL_SHADER program");
    }
    else
    {
        ROS_WARN("[SHUTDOWN] No WALL_SHADER program to delete");
    }

    // Delete CircleRenderer class shader program
    if (CircleRenderer::cleanupShaderObjects())
    {
        ROS_INFO("[SHUTDOWN] Deleted CircleRenderer program");
    }
    else
    {
        ROS_WARN("[SHUTDOWN] No CircleRenderer program to delete");
    }

    // Delete wall texture
    if (WALL_TEXTURE_ID != 0)
    {
        glDeleteFramebuffers(1, &WALL_TEXTURE_ID);
        if (checkErrorOpenGL(__LINE__, __FILE__) < 0)
            ROS_WARN("[SHUTDOWN] Failed to Delete Wall Texture");
        else
            ROS_INFO("[SHUTDOWN] Deleted Wall Texture");
    }
    else
        ROS_WARN("[SHUTDOWN] No Wall Texture to Delete");

    // Destroy GLFW window
    if (p_window_id)
    {
        glfwDestroyWindow(p_window_id);
        p_window_id = nullptr;
        if (checkErrorGLFW(__LINE__, __FILE__) < 0)
            ROS_WARN("[SHUTDOWN] Failed to Destroy GLFW Window");
        else
            ROS_INFO("[SHUTDOWN] Destroyed GLFW Window");
    }
    else
    {
        ROS_WARN("[SHUTDOWN] No GLFW window to destroy");
    }

    // Terminate GLFW
    glfwTerminate();
    checkErrorGLFW(__LINE__, __FILE__);
    ROS_INFO("[SHUTDOWN] Terminated GLFW");

    // Return success
    return is_error ? -1 : 0;

    // --------------- TEST IMAGE SETUP ---------------

    // Use first wall image
    cv::Mat im_wall = wallImgMatVec[0];
    cv::Mat im_wall_mask_1 = monImgMatVec[0];
    cv::Mat im_wall_mask_2 = calImgMatVec[0];

    // Test merge image
    mergeImgMat(im_wall_mask_1, im_wall);
    mergeImgMat(im_wall_mask_2, im_wall);

    // Populate the source correspondence points
    /// @note Assumes Y-axis points down
    std::vector<cv::Point2f> srcPoints = {
        cv::Point2f(0, 0),                                        // Top-left (0,0)
        cv::Point2f(WALL_IMAGE_WIDTH_PXL, 0),                     // Top-right (1,0)
        cv::Point2f(WALL_IMAGE_WIDTH_PXL, WALL_IMAGE_HEIGHT_PXL), // Bottom-right (1,1)
        cv::Point2f(0, WALL_IMAGE_HEIGHT_PXL)};                   // Bottom-left (0,1)

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
    cv::warpPerspective(im_wall, im_warp1, H1, cv::Size(WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL));

    // Test second image

    // Loop through dstPoints
    for (int i = 0; i < dstPoints.size(); i++)
    {
        // Update srcPoints
        dstPoints[i].x += WALL_IMAGE_WIDTH_PXL + 50;
    }

    // Find Homography
    cv::Mat H2 = cv::findHomography(srcPoints, dstPoints);
    // Warp Perspective
    cv::Mat im_warp2;
    cv::warpPerspective(im_wall, im_warp2, H2, cv::Size(WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL));

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
