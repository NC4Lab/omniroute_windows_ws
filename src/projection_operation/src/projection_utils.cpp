// ######################################################################################################

// ======================================== projection_utils.cpp ========================================

// ######################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_utils.h"

// ================================================== CLASS: MazeRenderContext ==================================================

// Initialize MazeRenderContext static members
GLFWmonitor **MazeRenderContext::_PP_Monitor = nullptr;
int MazeRenderContext::_NumMonitors = 0;

// Constructor
MazeRenderContext::MazeRenderContext()
    : shaderProgram(0), vao(0), vbo(0), ebo(0), textureID(0),
      windowID(nullptr), monitorID(nullptr), windowInd(-1), monitorInd(-1)
{
    // Members are initialized to default values, setup is deferred
}

// DESTRUCTOR
MazeRenderContext::~MazeRenderContext()
{
    // Clean up resources without logging
    if (isContextInitialized)
    {
        cleanupContext(false);
    }
}

// COPY CONSTRUCTOR
MazeRenderContext::MazeRenderContext(MazeRenderContext &&other) noexcept
    : shaderProgram(other.shaderProgram), vao(other.vao), vbo(other.vbo),
      ebo(other.ebo), textureID(other.textureID),
      windowID(other.windowID), monitorID(other.monitorID),
      windowInd(other.windowInd), monitorInd(other.monitorInd),
      isContextInitialized(other.isContextInitialized)
{
    // Reset the other's members to prevent double deletion
    other._resetMembers(); // Use a member function for resetting
}

// MOVE CONSTRUCTOR
MazeRenderContext &MazeRenderContext::operator=(MazeRenderContext &&other) noexcept
{
    if (this != &other)
    {
        // Clean up existing resources if initialized
        if (isContextInitialized)
        {
            cleanupContext(false);
        }

        // Transfer ownership of resources from other to this
        shaderProgram = other.shaderProgram;
        vao = other.vao;
        vbo = other.vbo;
        ebo = other.ebo;
        textureID = other.textureID;
        windowID = other.windowID;
        monitorID = other.monitorID;
        windowInd = other.windowInd;
        monitorInd = other.monitorInd;
        isContextInitialized = other.isContextInitialized;

        // Reset the other's members to default values
        other._resetMembers();
    }
    return *this;
}

void MazeRenderContext::CallbackFrameBufferSizeGLFW(GLFWwindow *window, int width, int height)
{
    glViewport(0, 0, width, height);
    CheckErrorGLFW(__LINE__, __FILE__, "CallbackFrameBufferSizeGLFW");
}

void MazeRenderContext::CallbackDebugOpenGL(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message, const void *userParam)
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
    ROS_INFO("[callbackDebugOpenGL] Type[0x%x] ID[%d] Severity[0x%x] Message[%s]", type, id, severity, message);
}

void MazeRenderContext::CallbackErrorGLFW(int error, const char *description)
{
    ROS_ERROR("[CallbackErrorGLFW] Error[%d] Description[%s]", error, description);
}

int MazeRenderContext::CheckErrorOpenGL(int line, const char *file_str, const char *msg_str)
{
    // ROS_INFO("        [CheckErrorOpenGL] DEBUG: File[%s] Line[%d]", file_str, line); // TEMP
    GLenum gl_err;
    while ((gl_err = glGetError()) != GL_NO_ERROR)
    {
        ROS_ERROR("[CheckErrorOpenGL] Message[%s] Error Number[%u] File[%s] Line[%d]",
                  msg_str ? msg_str : "No additional info", gl_err, file_str, line);
        return -1;
    }
    return 0;
}

int MazeRenderContext::CheckErrorGLFW(int line, const char *file_str, const char *msg_str)
{
    // ROS_INFO("        [CheckErrorGLFW] DEBUG: File[%s] Line[%d]", file_str, line); // TEMP
    const char *description;
    int glfw_err = glfwGetError(&description);
    if (glfw_err != GLFW_NO_ERROR)
    {
        ROS_ERROR("[CheckErrorGLFW] Message[%s] Description[%s] File[%s] Line[%d]",
                  msg_str ? msg_str : "No additional info", description, file_str, line);
        return -1;
    }
    return 0;
}

int MazeRenderContext::SetupGraphicsLibraries(int &out_n_mon)
{
    int status = 0;

    // Initialize GLFW and set error callback
    glfwSetErrorCallback(CallbackErrorGLFW);
    if (!glfwInit())
    {
        ROS_ERROR("[MazeRenderContext::SetupGraphicsLibraries] GLFW Initialization Failed");
        return -1;
    }
    status = CheckErrorGLFW(__LINE__, __FILE__);

    // Request a debug context for future windows (all windows can share the same context)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);                 // Specify OpenGL version if needed
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);                 // Specify OpenGL version if needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // Request core profile
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);            // Request debug context
    status = CheckErrorGLFW(__LINE__, __FILE__);

    // Discover available monitors
    _PP_Monitor = glfwGetMonitors(&_NumMonitors);
    status = CheckErrorGLFW(__LINE__, __FILE__);

    // Check for errors in monitor discovery
    if (!_PP_Monitor || _NumMonitors == 0)
    {
        ROS_ERROR("[MazeRenderContext::SetupGraphicsLibraries] No Monitors Found");
        return -1;
    }
    ROS_INFO("[MazeRenderContext::SetupGraphicsLibraries] Monitors Found: %d", _NumMonitors);
    out_n_mon = _NumMonitors;

    return 0;
}

int MazeRenderContext::initContext(int win_ind, int mon_ind, KeyCallbackFunc key_callback)
{
    int status = 0;

    // Check/set the new monitor id
    if (_checkMonitor(mon_ind) < 0)
    {
        ROS_ERROR("[MazeRenderContext::initContext] Context Setup Failed: Window[%d] Monitor[%d]", win_ind, mon_ind);
        return -1;
    }

    // Create a new GLFW window
    windowID = glfwCreateWindow(WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL, "", monitorID, NULL); // Use the monitor in window creation
    if (!windowID)
    {
        glfwTerminate();
        ROS_ERROR("[MazeRenderContext::initContext] GLFW Failed to Create Window");
        return -1;
    }
    status = CheckErrorGLFW(__LINE__, __FILE__);
    windowInd = win_ind; // Store the window index

    // Set the GLFW window as the current OpenGL context
    glfwMakeContextCurrent(windowID);
    status = CheckErrorGLFW(__LINE__, __FILE__);

    // Very imporant flag!
    isContextInitialized = true;

    // Load OpenGL extensions using GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress) || !gladLoadGL())
    {
        glfwDestroyWindow(windowID);
        ROS_ERROR("[MazeRenderContext::initContext] Failed to load GLAD");
        return -1;
    }

    // Set GLFW callbacks for keyboard events using the KeyCallbackFunc function pointer
    glfwSetKeyCallback(windowID, key_callback);
    status = CheckErrorGLFW(__LINE__, __FILE__);

    // Set GLFW callbacks for framebuffer size events
    glfwSetFramebufferSizeCallback(windowID, CallbackFrameBufferSizeGLFW);
    status = CheckErrorGLFW(__LINE__, __FILE__);

    // Enable OpenGL debugging context and associate callback
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(CallbackDebugOpenGL, nullptr);
    status = CheckErrorOpenGL(__LINE__, __FILE__);

    // Print OpenGL version information once when the first window is created
    if (windowInd == 0)
    {
        // Log OpenGL versions
        const GLubyte *opengl_version = glGetString(GL_VERSION);
        ROS_INFO("[main] OpenGL initialized: Version[%s]", opengl_version);

        // Log GLFW versions
        int glfw_major, glfw_minor, glfw_rev;
        glfwGetVersion(&glfw_major, &glfw_minor, &glfw_rev);
        ROS_INFO("[main] GLFW initialized: Version[%d.%d.%d]", glfw_major, glfw_minor, glfw_rev);
        status = CheckErrorGLFW(__LINE__, __FILE__);
    }

    return status;
}

int MazeRenderContext::compileAndLinkShaders(const GLchar *vertex_source, const GLchar *fragment_source)
{
    // Check that context is initialized
    if (!isContextInitialized)
    {
        ROS_ERROR("[compileAndLinkShaders] Context Not Initialized");
        return -1;
    }

    // Compile vertex shader
    GLuint temp_vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(temp_vertex_shader, 1, &vertex_source, nullptr);
    glCompileShader(temp_vertex_shader);
    if (!_checkShaderCompilation(temp_vertex_shader, "VERTEX"))
    {
        glDeleteShader(temp_vertex_shader);
        ROS_ERROR("[compileAndLinkShaders] Vertex shader compilation failed.");
        return -1;
    }

    // Compile fragment shader
    GLuint temp_fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(temp_fragment_shader, 1, &fragment_source, nullptr);
    glCompileShader(temp_fragment_shader);
    if (!_checkShaderCompilation(temp_fragment_shader, "FRAGMENT"))
    {
        glDeleteShader(temp_vertex_shader);
        glDeleteShader(temp_fragment_shader);
        ROS_ERROR("[compileAndLinkShaders] Fragment shader compilation failed.");
        return -1;
    }

    // Link shaders into a program
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, temp_vertex_shader);
    glAttachShader(shaderProgram, temp_fragment_shader);
    glLinkProgram(shaderProgram);
    if (!_checkProgramLinking(shaderProgram))
    {
        glDeleteShader(temp_vertex_shader);
        glDeleteShader(temp_fragment_shader);
        glDeleteProgram(shaderProgram);
        ROS_ERROR("[compileAndLinkShaders] Shader program linking failed.");
        return -1;
    }

    // Cleanup: detach and delete shaders
    glDetachShader(shaderProgram, temp_vertex_shader);
    glDeleteShader(temp_vertex_shader);
    glDetachShader(shaderProgram, temp_fragment_shader);
    glDeleteShader(temp_fragment_shader);

    return CheckErrorOpenGL(__LINE__, __FILE__);
}

int MazeRenderContext::validateShaderProgram()
{
    if (shaderProgram == 0)
    {
        ROS_ERROR("[MazeRenderContext::validateShaderProgram] No shader program to validate.");
        return -1;
    }

    glValidateProgram(shaderProgram);
    GLint validationStatus;
    glGetProgramiv(shaderProgram, GL_VALIDATE_STATUS, &validationStatus);

    if (validationStatus == GL_FALSE)
    {
        GLint logLength;
        glGetProgramiv(shaderProgram, GL_INFO_LOG_LENGTH, &logLength);

        // The maxLength includes the NULL character
        std::vector<GLchar> errorLog(logLength);
        glGetProgramInfoLog(shaderProgram, logLength, &logLength, &errorLog[0]);

        // Log the error
        ROS_ERROR("[MazeRenderContext::validateShaderProgram] Shader program validation failed: %s", &errorLog[0]);

        // Optionally, delete the shader program if you won't need it after a failed validation
        // glDeleteProgram(shaderProgram);
        // shaderProgram = 0;

        return -1;
    }

    return 0;
}

int MazeRenderContext::cleanupContext(bool log_errors)
{
    int status = 0;

    if (shaderProgram != 0)
    {
        glDeleteProgram(shaderProgram);
        status |= CheckErrorOpenGL(__LINE__, __FILE__, "[MazeRenderContext::cleanupContext] glDeleteProgram error");
        if (log_errors)
            ROS_INFO("[MazeRenderContext::cleanupContext] %s", status ? "Failed to delete shader program" : "Shader program deleted successfully");
        shaderProgram = 0;
    }

    if (textureID != 0)
    {
        glDeleteTextures(1, &textureID);
        status |= CheckErrorOpenGL(__LINE__, __FILE__, "[MazeRenderContext::cleanupContext] glDeleteTextures error");
        if (log_errors)
            ROS_INFO("[MazeRenderContext::cleanupContext] %s", status ? "Failed to delete texture" : "Texture deleted successfully");
        textureID = 0;
    }

    if (vao != 0)
    {
        glDeleteVertexArrays(1, &vao);
        status |= CheckErrorOpenGL(__LINE__, __FILE__, "[MazeRenderContext::cleanupContext] glDeleteVertexArrays error");
        if (log_errors)
            ROS_INFO("[MazeRenderContext::cleanupContext] %s", status ? "Failed to delete VAO" : "VAO deleted successfully");
        vao = 0;
    }

    if (vbo != 0)
    {
        glDeleteBuffers(1, &vbo);
        status |= CheckErrorOpenGL(__LINE__, __FILE__, "[MazeRenderContext::cleanupContext] glDeleteBuffers error");
        if (log_errors)
            ROS_INFO("[MazeRenderContext::cleanupContext] %s", status ? "Failed to delete VBO" : "VBO deleted successfully");
        vbo = 0;
    }

    if (ebo != 0)
    {
        glDeleteBuffers(1, &ebo);
        status |= CheckErrorOpenGL(__LINE__, __FILE__, "[MazeRenderContext::cleanupContext] glDeleteBuffers error");
        if (log_errors)
            ROS_INFO("[MazeRenderContext::cleanupContext] %s", status ? "Failed to delete EBO" : "EBO deleted successfully");
        ebo = 0;
    }

    if (windowID != nullptr)
    {
        glfwDestroyWindow(windowID);
        status |= CheckErrorGLFW(__LINE__, __FILE__, "[MazeRenderContext::cleanupContext] glfwDestroyWindow error");
        if (log_errors)
            ROS_INFO("[MazeRenderContext::cleanupContext] %s", status ? "Failed to destroy GLFW window" : "GLFW window destroyed successfully");
        windowID = nullptr;
    }

    return status ? -1 : 0;
}

int MazeRenderContext::initWindow()
{
    int status = 0;

    // Set the GLFW window as the current OpenGL context
    glfwMakeContextCurrent(windowID);
    status = CheckErrorGLFW(__LINE__, __FILE__);

    // Clear the back buffer for a new frame
    glClear(GL_COLOR_BUFFER_BIT);
    status = CheckErrorOpenGL(__LINE__, __FILE__);

    return status;
}

int MazeRenderContext::bufferSwapPoll()
{
    // Swap buffers and poll events
    glfwSwapBuffers(windowID);

    // Poll events
    glfwPollEvents();

    return CheckErrorGLFW(__LINE__, __FILE__);
}

int MazeRenderContext::checkExitRequest()
{
    int status = 0;

    // Check if the window should close
    if (glfwWindowShouldClose(windowID))
    {
        ROS_INFO("[MazeRenderContext::checkExitRequest] Window[%d] Monitor[%d] Close Requested", windowInd, monitorInd);
        return 1;
    }

    // Check if the escape key is pressed
    if (glfwGetKey(windowID, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    {
        ROS_INFO("[MazeRenderContext::checkExitRequest] Window[%d] Monitor[%d] Escape Key Pressed", windowInd, monitorInd);
        return 2;
    }

    if (CheckErrorGLFW(__LINE__, __FILE__) < 0)
        return -1;

    return 0;
}

int MazeRenderContext::CleanupGraphicsLibraries()
{
    int status = 0;

    // Terminate GLFW, which cleans up all GLFW resources.
    glfwTerminate();
    status = CheckErrorGLFW(__LINE__, __FILE__);
    if (status < 0)
        ROS_WARN("[MazeRenderContext::CleanupGraphicsLibraries] Failed to Terminate GLFW Library");
    else
        ROS_INFO("[MazeRenderContext::CleanupGraphicsLibraries] Graphics libraries and shared resources cleaned up successfully.");

    // Reset monitor pointers and count
    _PP_Monitor = nullptr;
    _NumMonitors = 0;

    return status;
}

int MazeRenderContext::switchWindowMode(int mon_ind_new, bool do_fullscreen)
{
    // Check/set the new monitor id
    if (_checkMonitor(mon_ind_new) < 0)
    {
        ROS_ERROR("[MazeRenderContext::initContext] Context Setup Failed: Window[%d] Monitor[%d]", windowInd, mon_ind_new);
        return -1;
    }

    if (!monitorID)
    { // Early return if monitor pointer is invalid
        ROS_ERROR("[switchWindowMode] Invalid Monitor Pointer: Monitor[%d]", mon_ind_new);
        return -1;
    }

    // Get the video mode of the selected monitor
    const GLFWvidmode *mode = glfwGetVideoMode(monitorID);
    if (!mode)
    { // Validate video mode retrieval
        ROS_ERROR("[switchWindowMode] Failed to Get Video Mode: Monitor[%d]", mon_ind_new);
        return -1;
    }

    // Update window to full-screen or windowed mode based on 'do_fullscreen'
    if (do_fullscreen)
    {
        glfwSetWindowMonitor(windowID, monitorID, 0, 0, mode->width, mode->height, mode->refreshRate);
    }
    else
    {
        int monitor_x, monitor_y;
        glfwGetMonitorPos(monitorID, &monitor_x, &monitor_y); // Get monitor position
        if (monitor_x < 0 || monitor_y < 0)
        { // Validate monitor position
            ROS_WARN("[switchWindowMode] Invalid Monitor Position: Monitor[%d] X[%d] Y[%d]", mon_ind_new, monitor_x, monitor_y);
            return 0;
        }
        glfwSetWindowMonitor(windowID, NULL, monitor_x + 100, monitor_y + 100, (int)(500.0f * WINDOW_ASPECT_RATIO), 500, 0); // Set windowed mode position and size
    }

    // Make sure winsow always stays on top in fullscreen mode
    setWindowStackOrder(do_fullscreen);

    // Update window title with window and monitor indices
    std::string new_title = "Window[" + std::to_string(windowInd) + "] Monitor[" + std::to_string(mon_ind_new) + "]";
    glfwSetWindowTitle(windowID, new_title.c_str());
    ROS_INFO("[switchWindowMode] Move Window: Monitor[%d] Format[%s]", mon_ind_new, do_fullscreen ? "fullscreen" : "windowed");

    return CheckErrorGLFW(__LINE__, __FILE__, "MazeRenderContext::switchWindowMode");
}

int MazeRenderContext::setWindowStackOrder(bool is_top_always)
{
    // Set the GLFW_FLOATING attribute based on the alwaysOnTop boolean
    // glfwSetWindowAttrib(windowID, GLFW_FLOATING, is_top_always ? GLFW_TRUE : GLFW_FALSE);

    // Check if the window is minimized (iconified)
    if (glfwGetWindowAttrib(windowID, GLFW_ICONIFIED))
    {
        // If the window is minimized and is_top_always is true, restore the window
        if (is_top_always)
            glfwRestoreWindow(windowID);
    }

    // It may be useful to also call this here to process events such as restore
    glfwPollEvents();
    return CheckErrorGLFW(__LINE__, __FILE__, "MazeRenderContext::_setWindowStackOrder");
}

void MazeRenderContext::flashBackgroundColor(const cv::Scalar &color, int duration)
{
    // Set the new clear color using BGR values from cv::Scalar
    glClearColor(color[0], color[1], color[2], 1.0f); // RGBA

    // Clear the window with the new color
    glClear(GL_COLOR_BUFFER_BIT);
    glfwSwapBuffers(windowID);

    // Wait for the duration of the flash
    std::this_thread::sleep_for(std::chrono::milliseconds(duration));

    // Reset the clear color (assuming the default is black)
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
}

void MazeRenderContext::_resetMembers()
{
    shaderProgram = 0;
    vao = 0;
    vbo = 0;
    ebo = 0;
    textureID = 0;
    windowID = nullptr;
    monitorID = nullptr;
    windowInd = -1;
    monitorInd = -1;
    isContextInitialized = false;
}

bool MazeRenderContext::_checkShaderCompilation(GLuint shader, const std::string &shader_type)
{
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char infoLog[1024];
        glGetShaderInfoLog(shader, 1024, nullptr, infoLog);
        ROS_ERROR("[MazeRenderContext] %s shader compilation error: %s", shader_type.c_str(), infoLog);
        return false;
    }
    return true;
}

bool MazeRenderContext::_checkProgramLinking(GLuint program)
{
    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success)
    {
        char infoLog[1024];
        glGetProgramInfoLog(program, 1024, nullptr, infoLog);
        ROS_ERROR("[MazeRenderContext] Program linking error: %s", infoLog);
        return false;
    }
    return true;
}

int MazeRenderContext::_checkMonitor(int mon_ind)
{
    // Validate the inputs
    if (mon_ind >= _NumMonitors)
    {
        ROS_ERROR("[MazeRenderContext::_checkMonitor] Monitor Index[%d] Exceeds Available Monitors[%d]", mon_ind, _NumMonitors);
        return -1;
    }

    // Set the monitor for this instance
    monitorID = _PP_Monitor[mon_ind];
    if (!monitorID)
    {
        ROS_ERROR("[MazeRenderContext::initContext] Invalid monitor pointer for index %d", mon_ind);
        return -1;
    }
    monitorInd = mon_ind; // Store the monitor index

    return 0;
}

void MazeRenderContext::_testCallbacks()
{
    ROS_INFO("============== START: CALLBACK DEBUGGIN ==============");
    // Trigger buffer
    glfwSetWindowSize(windowID, 800, 600); // Change the size to something different

    // Insert a debug message manually
    glDebugMessageInsert(GL_DEBUG_SOURCE_APPLICATION, GL_DEBUG_TYPE_ERROR, 0, GL_DEBUG_SEVERITY_HIGH, -1, "Test debug message");
    CheckErrorOpenGL(__LINE__, __FILE__, "[main] Error Flagged Following glDebugMessageInsert()");

    // Try to create a window with invalid arguments
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 99); // Set an invalid version to trigger an error
    GLFWwindow *new_win = glfwCreateWindow(640, 480, "Bad Window!", nullptr, nullptr);
    ROS_INFO("============== END: CALLBACK DEBUGGIN ==============");
}

// ================================================== CLASS: CircleRenderer ==================================================

// Initialize CircleRenderer static members
int CircleRenderer::_CircCnt = 0;
GLuint CircleRenderer::_ShaderProgram = 0;
GLint CircleRenderer::_ColorLocation = -1;
GLint CircleRenderer::_TransformLocation = -1;
GLint CircleRenderer::_AspectRatioLocation = -1;
float CircleRenderer::_AspectRatioUniform = 1.0f;

CircleRenderer::CircleRenderer()
    : circPosition(cv::Point2f(0.0f, 0.0f)), cirRadius(1.0f), circColor(cv::Scalar(1.0, 1.0, 1.0)),
      circSegments(32), circRotationAngle(0.0f), circScalingFactors(cv::Point2f(1.0f, 1.0f))
{
    // Define instance count and itterate static _CircCnt
    circID = _CircCnt++;

    // Initialize the transformation matrix as an identity matrix
    _transformationMatrix = cv::Mat::eye(4, 4, CV_32F);
}

CircleRenderer::~CircleRenderer()
{
    // Cleanup instance-level OpenGL resources
    if (_vao != 0)
    {
        glDeleteVertexArrays(1, &_vao);
        _vao = 0;
    }
    if (_vbo != 0)
    {
        glDeleteBuffers(1, &_vbo);
        _vbo = 0;
    }
}

void CircleRenderer::initializeCircleAttributes(cv::Point2f pos, float rad, cv::Scalar col, unsigned int segments)
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
    _setupRenderBuffers();

    // // TEMP
    // ROS_INFO("[CircleRenderer] Initialized Instance[%d]: Position[%0.2f, %0.2f] Radius[%0.4f] Color[%.2f, %.2f, %.2f] Segments[%d]",
    //          circID, circPosition.x, circPosition.y, cirRadius, circColor[0], circColor[1], circColor[2], circSegments);
}

void CircleRenderer::setPosition(cv::Point2f pos)
{
    // Modify the y position based on the aspect ratio
    pos.y /= _AspectRatioUniform;
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
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);

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
    glUniform4f(_ColorLocation, circColor[0], circColor[1], circColor[2], 1.0f);

    // Use the updated transformation matrix instead of an identity matrix
    auto transformArray = _cvMatToGlArray(_transformationMatrix);
    glUniformMatrix4fv(_TransformLocation, 1, GL_FALSE, transformArray.data());

    // Bind the VAO and draw the circle using GL_TRIANGLE_FAN
    glBindVertexArray(_vao);
    glDrawArrays(GL_TRIANGLE_FAN, 0, static_cast<GLsizei>(_circVertices.size() / 2));

    // Unbind the VAO to leave a clean state
    glBindVertexArray(0);
}

int CircleRenderer::CompileAndLinkCircleShaders(float aspect_ratio)
{
    // Set the aspect ratio
    _AspectRatioUniform = aspect_ratio;

    // Compile vertex shader
    GLuint temp_vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(temp_vertex_shader, 1, &vertexShaderSource, nullptr);
    glCompileShader(temp_vertex_shader);
    if (!_CheckShaderCompilation(temp_vertex_shader, "VERTEX"))
    {
        glDeleteShader(temp_vertex_shader);
        ROS_ERROR("[CircleRenderer] Vertex shader compilation failed.");
        return -1;
    }

    // Compile fragment shader
    GLuint temp_fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(temp_fragment_shader, 1, &fragmentShaderSource, nullptr);
    glCompileShader(temp_fragment_shader);
    if (!_CheckShaderCompilation(temp_fragment_shader, "FRAGMENT"))
    {
        glDeleteShader(temp_vertex_shader);
        glDeleteShader(temp_fragment_shader);
        ROS_ERROR("[CircleRenderer] Fragment shader compilation failed.");
        return -1;
    }

    // Link shaders into a program
    _ShaderProgram = glCreateProgram();
    glAttachShader(_ShaderProgram, temp_vertex_shader);
    glAttachShader(_ShaderProgram, temp_fragment_shader);
    glLinkProgram(_ShaderProgram);
    if (!_CheckProgramLinking(_ShaderProgram))
    {
        glDeleteShader(temp_vertex_shader);
        glDeleteShader(temp_fragment_shader);
        glDeleteProgram(_ShaderProgram);
        ROS_ERROR("[CircleRenderer] Shader program linking failed.");
        return -1;
    }

    // Cleanup: detach and delete shaders
    glDetachShader(_ShaderProgram, temp_vertex_shader);
    glDeleteShader(temp_vertex_shader);
    glDetachShader(_ShaderProgram, temp_fragment_shader);
    glDeleteShader(temp_fragment_shader);

    // Get uniform locations
    _ColorLocation = glGetUniformLocation(_ShaderProgram, "color");
    _TransformLocation = glGetUniformLocation(_ShaderProgram, "transform");
    _AspectRatioLocation = glGetUniformLocation(_ShaderProgram, "aspectRatio");

    // Check for errors in getting uniforms
    if (_ColorLocation == -1 || _TransformLocation == -1 || _AspectRatioLocation == -1)
    {
        ROS_ERROR("[CircleRenderer] Error getting uniform locations.");
        return -1;
    }

    return 0; // Success
}

int CircleRenderer::CleanupClassResources()
{
    // Delete the shader program
    if (_ShaderProgram != 0)
    {
        glDeleteProgram(_ShaderProgram);
        _ShaderProgram = 0;
    }

    // Reset the static uniform locations to -1 indicating they are no longer valid
    _ColorLocation = -1;
    _TransformLocation = -1;
    _AspectRatioLocation = -1;

    // Reset the aspect ratio uniform
    _AspectRatioUniform = 0.0f;

    ///@todo add error check for open gl to class
    return 0;
}

void CircleRenderer::SetupShader()
{
    // Use the shader program
    glUseProgram(_ShaderProgram);

    // Set the aspect ratio uniform
    glUniform1f(_AspectRatioLocation, _AspectRatioUniform);
}

void CircleRenderer::UnsetShader()
{
    // Unset the shader program
    glUseProgram(0);
}

bool CircleRenderer::_CheckShaderCompilation(GLuint shader, const std::string &shader_type)
{
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        // Get and log the error message
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        ROS_ERROR("[CircleRenderer] %s shader compilation error: %s", shader_type.c_str(), infoLog);
        return false;
    }
    return true;
}

bool CircleRenderer::_CheckProgramLinking(GLuint program)
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

void CircleRenderer::_setupRenderBuffers()
{
    // Generate a new Vertex Array Object (VAO) and store the ID
    glGenVertexArrays(1, &_vao);
    // Generate a new Vertex Buffer Object (VBO) and store the ID
    glGenBuffers(1, &_vbo);

    // Bind the VAO to set it up
    glBindVertexArray(_vao);

    // Bind the VBO to the GL_ARRAY_BUFFER target
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);
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

std::array<float, 16> CircleRenderer::_cvMatToGlArray(const cv::Mat &mat)
{
    assert(mat.cols == 4 && mat.rows == 4 && mat.type() == CV_32F);
    std::array<float, 16> glArray;
    std::copy(mat.begin<float>(), mat.end<float>(), glArray.begin());
    return glArray;
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

// ================================================== FUNCTIONS ==================================================

bool dbRunDT(int dt_wait)
{
    // Initialize with 0 to return true on the first call
    static ros::Time ts_wait = ros::Time(0);
    ros::Time now = ros::Time::now();

    // Check if ts_wait is set to zero or the current time is past ts_wait
    if (ts_wait == ros::Time(0) || now > ts_wait)
    {
        // Set the next wait timestamp
        ts_wait = now + ros::Duration(0, dt_wait * 1000000); // Convert ms to ns
        return true;
    }

    return false;
}

void dbTraceCalls(bool do_reset, int line, const char *file_path)
{
    static int cnt_calls = 0;
    static ros::Time start_time;
    static int line_start = 0;
    static std::string file_name_start = "";

    // Function to extract file name from file path
    auto extractFileName = [](const char *path) -> std::string
    {
        if (path == nullptr)
        {
            return std::string("NULL");
        }
        const char *file_name = strrchr(path, '\\'); // Windows file path separator
        if (!file_name)
            file_name = strrchr(path, '/'); // In case of Unix-style paths
        if (file_name)
        {
            return std::string(file_name + 1); // Skip past the last separator
        }
        return std::string(path); // No separator found, return the whole string
    };

    // Reset start time
    if (do_reset)
    {
        cnt_calls = 0;
        line_start = line;
        file_name_start = extractFileName(file_path);
        start_time = ros::Time::now();
    }
    // Print elapsed time
    else
    {
        cnt_calls++;
        ros::Duration elapsed_time = ros::Time::now() - start_time;
        std::string file_name_current = extractFileName(file_path);
        if (line_start == 0 || line == 0)
        {
            ROS_INFO("Call[%d]: Elapsed Time: %f milliseconds", cnt_calls, elapsed_time.toNSec() / 1e6);
        }
        else
        {
            ROS_INFO("Call[%d]: Elapsed Time from %s[%d] to %s[%d] is %0.2f milliseconds",
                     cnt_calls, file_name_start.c_str(), line_start, file_name_current.c_str(), line,
                     elapsed_time.toNSec() / 1e6);
        }
    }
}

void dbWaitForInput()
{
    ROS_INFO("Paused for Debugging: Press Enter to Continue...");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

void dbLogQuadVertices(const std::vector<cv::Point2f> &quad_vertices)
{
    if (quad_vertices.size() != 4)
    {
        ROS_ERROR("[dbLogQuadVertices] Invalid number of vertices. Expected 4.");
        return;
    }

    ROS_INFO("         Quad Vertices             ");
    ROS_INFO("===================================");
    ROS_INFO("    |     Left     |     Right    |");

    if (std::any_of(quad_vertices.begin(), quad_vertices.end(), [](const cv::Point2f &point)
                    { return point.x > 10.0f || point.y > 10.0f; }))
    {
        ROS_INFO("-----------------------------------");
        ROS_INFO("    |V0  X , Y     |V1  X , Y     |");
        ROS_INFO("-----------------------------------");
        ROS_INFO("Top | %+5.0f, %+5.0f | %+5.0f, %+5.0f |",
                 quad_vertices[0].x, quad_vertices[0].y, quad_vertices[1].x, quad_vertices[1].y);
        ROS_INFO("-----------------------------------");
        ROS_INFO("    |V3  X , Y     |V2  X , Y     |");
        ROS_INFO("-----------------------------------");
        ROS_INFO("Btm | %+5.0f, %+5.0f | %+5.0f, %+5.0f |",
                 quad_vertices[3].x, quad_vertices[3].y, quad_vertices[2].x, quad_vertices[2].y);
    }
    else
    {
        ROS_INFO("-----------------------------------");
        ROS_INFO("    |V0  X , Y     |V1  X , Y     |");
        ROS_INFO("-----------------------------------");
        ROS_INFO("Top | %+5.2f, %+5.2f | %+5.2f, %+5.2f |",
                 quad_vertices[0].x, quad_vertices[0].y, quad_vertices[1].x, quad_vertices[1].y);
        ROS_INFO("-----------------------------------");
        ROS_INFO("    |V3  X , Y     |V2  X , Y     |");
        ROS_INFO("-----------------------------------");
        ROS_INFO("Btm | %+5.2f, %+5.2f | %+5.2f, %+5.2f |",
                 quad_vertices[3].x, quad_vertices[3].y, quad_vertices[2].x, quad_vertices[2].y);
    }

    ROS_INFO("===================================");
}

void dbLogCtrlPointCoordinates(const std::array<std::array<cv::Point2f, 4>, 4> &r_ctrl_pnt_coords)
{
    ROS_INFO("        Control Point Coordinates        ");
    ROS_INFO("=========================================");
    ROS_INFO("        |      Left     |     Right     |");

    // Loop through each control point
    for (int cp = 0; cp < 4; ++cp)
    {
        // Fetch the vertices for the current control point
        auto &quad_vertices = r_ctrl_pnt_coords[cp];

        // Print the top row coordinates
        ROS_INFO("=========================================");
        ROS_INFO("        |V0   X , Y     |V1   X , Y     ||");
        ROS_INFO("-----------------------------------------");
        ROS_INFO("[%d] Top | %+5.2f , %+5.2f | %+5.2f , %+5.2f |",
                 cp,
                 quad_vertices[0].x, quad_vertices[0].y, quad_vertices[1].x, quad_vertices[1].y);

        // Print the bottom row coordinates
        ROS_INFO("---------------------------------------");
        ROS_INFO("        |V3   X , Y     |V2   X , Y     |");
        ROS_INFO("---------------------------------------");
        ROS_INFO("[%d] Btm | %+5.2f , %+5.2f | %+5.2f , %+5.2f |",
                 cp,
                 quad_vertices[3].x, quad_vertices[3].y, quad_vertices[2].x, quad_vertices[2].y);
    }
    ROS_INFO("=========================================");
}

void dbLogWallVerticesCoordinates(const std::array<std::array<std::array<cv::Point2f, 4>, MAZE_SIZE>, MAZE_SIZE> &_WALL_WARP_COORDS)
{
    ROS_INFO("                                       Warped Wall Coordinates                                               ");
    ROS_INFO("=============================================================================================================");
    ROS_INFO("        ||   (0) Left    |   (0) Right   ||   (1) Left    |   (1) Right   ||   (2) Left    |   (2) Right   ||");
    ROS_INFO("-------------------------------------------------------------------------------------------------------------");

    // Loop through each row and column in the maze
    for (int row = 0; row < MAZE_SIZE; ++row)
    {
        ROS_INFO("        ||   X   ,   Y   |   X   ,   Y   ||   X   ,   Y   |   X   ,   Y   ||   X   ,   Y   |   X   ,   Y   ||");
        ROS_INFO("-------------------------------------------------------------------------------------------------------------");

        // Buffer to hold the formatted string for each row
        char buffer[256];

        // Format and print the Top row coordinates
        snprintf(buffer, sizeof(buffer), "(%d) Top ||", row);
        for (int col = 0; col < MAZE_SIZE; ++col)
        {
            // Fetch the quad vertices for the current [row][col]
            auto &quad_vertices = _WALL_WARP_COORDS[row][col];
            snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), " %+4.2f , %+4.2f | %+4.2f , %+4.2f ||",
                     quad_vertices[0].x, quad_vertices[0].y, quad_vertices[1].x, quad_vertices[1].y);
        }
        ROS_INFO("%s", buffer);

        // Format and print the Bottom row coordinates
        snprintf(buffer, sizeof(buffer), "(%d) Btm ||", row);
        for (int col = 0; col < MAZE_SIZE; ++col)
        {
            // Fetch the quad vertices for the current [row][col]
            auto &quad = _WALL_WARP_COORDS[row][col];
            snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), " %+4.2f , %+4.2f | %+4.2f , %+4.2f ||",
                     quad[3].x, quad[3].y, quad[2].x, quad[2].y);
        }
        ROS_INFO("%s", buffer);

        ROS_INFO("-------------------------------------------------------------------------------------------------------------");
    }
}

void dbLogHomMat(const cv::Mat &r_HMAT)
{
    // Check if the input matrix is 3x3
    if (r_HMAT.rows != 3 || r_HMAT.cols != 3)
    {
        ROS_WARN("The input matrix is not 3x3. Cannot print.");
        return;
    }

    ROS_INFO("         Homography Matrix        ");
    ROS_INFO("==================================");
    ROS_INFO("          |  C0   |  C1   |  C2   |");
    ROS_INFO("----------------------------------");

    for (int i = 0; i < 3; ++i)
    {
        ROS_INFO("R%d        | %+5.2f | %+5.2f | %+5.2f |", i,
                 r_HMAT.at<double>(i, 0),
                 r_HMAT.at<double>(i, 1),
                 r_HMAT.at<double>(i, 2));
    }

    // Separator line
    ROS_INFO("==================================");
}

std::string promptForProjectorNumber()
{
    std::string input;
    while (true)
    {
        std::cout << "Enter the number of the projector being calibrated [0,1,2,3], or press 'q' to quit: ";
        std::cin >> input;

        // Check for quit command
        if (input == "q" || input == "Q")
        {
            std::cout << "Operation cancelled by the user." << std::endl;
            return ""; // Return an empty string or a specific value to indicate cancellation
        }

        // Check if the input is a single digit
        if (input.length() == 1 && std::isdigit(input[0]))
        {
            return input;
        }
        else
        {
            std::cout << "Invalid input. Please enter a single numeric digit or 'q' to quit." << std::endl;
        }

        // Clear the input stream in case of invalid input (e.g., if more than one character was entered)
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}

/**
 * @brief Formats the file name for the XML file based on the active calibration mode and monitor.
 *
 * Format:
 * - `hmats_m<number>.xml`
 * - `hmats_m0.xml`
 *
 * @param mon_ind Index of the active or desired monitor.
 * @param cal_ind Index of the active or desired calibration mode.
 * @param[out] out_path Reference to string that will store the path to the XML file.
 * @param[out] out_tag Reference to string that will store the tag for the calibration mode.
 *
 * @return Integer status code [-1:error, 0:successful].
 */
int xmlFrmtFileStrings(int mon_ind, int cal_ind, std::string &out_path, std::string &out_tag)
{
    // Format the output tag
    out_path =
        CONFIG_DIR_PATH + "/" +
        "hmats" +
        "_m" + std::to_string(mon_ind) +
        ".xml";

    // Get the calibration mode xml tag
    switch (cal_ind)
    {
    case 0:
        out_tag = "cwl";
        break;
    case 1:
        out_tag = "cwm";
        break;
    case 2:
        out_tag = "cwr";
        break;
    case 3:
        out_tag = "cmf";
        break;
    default:
        // Handle the error or unexpected cal_ind value
        ROS_ERROR("[frmtFilePathXML] Invalid cal_ind[%d]", cal_ind);
        return -1;
    }

    return 0;
}

int xmlSaveHMAT(const cv::Mat &H, int mon_ind, int cal_ind, int grid_row, int grid_col)
{
    // Get the full file path and tag for the given calibration mode
    std::string file_path;
    std::string cal_tag;
    xmlFrmtFileStrings(mon_ind, cal_ind, file_path, cal_tag);

    // Create an XML document object
    pugi::xml_document doc;
    pugi::xml_node root_node;

    // Load the existing file or create a new one if it doesn't exist
    pugi::xml_parse_result result = doc.load_file(file_path.c_str());
    if (!result)
    {
        // Create a new root node if the file doesn't exist
        root_node = doc.append_child("Root");
    }
    else
    {
        // Get the root node
        root_node = doc.child("Root");
    }

    // Find or create the parent element under the root node
    pugi::xml_node parent_node = root_node.child(cal_tag.c_str());
    if (!parent_node)
    {
        // If the parent tag doesn't exist, create it
        parent_node = root_node.append_child(cal_tag.c_str());
    }

    // Create a child node for storing the homography matrix with row and col attributes
    pugi::xml_node hmat_node = parent_node.append_child("HMAT");
    hmat_node.append_attribute("grid_row") = grid_row;
    hmat_node.append_attribute("grid_col") = grid_col;

    // TEMP
    //ROS_INFO("!!!!! grid_row[%d] grid_col[%d]", grid_row, grid_col);

    // Iterate over the rows of the matrix
    for (int i = 0; i < H.rows; ++i)
    {
        // Create a row node
        pugi::xml_node row_node = hmat_node.append_child("row");

        // Iterate over the elements in the row
        for (int j = 0; j < H.cols; ++j)
        {
            // Get the value from the matrix
            double value = H.at<double>(i, j); // Ensure the matrix type is double

            // Create a cell node and set its value
            pugi::xml_node cell_node = row_node.append_child("cell");
            cell_node.append_child(pugi::node_pcdata).set_value(std::to_string(value).c_str());
        }
    }

    // Save the document to the provided file path and check if it was successful
    bool save_succeeded = doc.save_file(file_path.c_str());
    if (!save_succeeded)
    {
        ROS_ERROR("[xmlSaveHMAT] Failed to save homography matrix to XML File[%s]", file_path.c_str());
        return -1;
    }

    // TEMP
    // dbWaitForInput();

    return 0;
}

// TEMP
int saveHMATxml(const std::string full_path,
                const std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> &_HMAT_GRID_ARR)
{
    // Create an XML document
    pugi::xml_document doc;

    // Add root node for the homography matrix grid
    pugi::xml_node hmat_grid_node = doc.append_child("HMATGrid");

    // Iterate over each matrix in the grid
    for (int grid_row = 0; grid_row < MAZE_SIZE; ++grid_row)
    {
        for (int grid_col = 0; grid_col < MAZE_SIZE; ++grid_col)
        {
            // Get the current homography matrix from the array
            const cv::Mat &current_hmat = _HMAT_GRID_ARR[grid_row][grid_col];

            // Create a child node for storing the current homography matrix
            pugi::xml_node current_hmat_node = hmat_grid_node.append_child("HMAT");
            current_hmat_node.append_attribute("row") = grid_row;
            current_hmat_node.append_attribute("col") = grid_col;

            // Iterate over the rows of the matrix
            for (int i = 0; i < MAZE_SIZE; ++i)
            {
                // Create a row node
                pugi::xml_node row_node = current_hmat_node.append_child("row");

                // Iterate over the elements in the row
                for (int j = 0; j < MAZE_SIZE; ++j)
                {
                    // Get the value from the matrix
                    float value = current_hmat.at<float>(i, j);

                    // Create a cell node and set its value
                    pugi::xml_node cell_node = row_node.append_child("cell");
                    cell_node.append_child(pugi::node_pcdata).set_value(std::to_string(value).c_str());
                }
            }
        }
    }

    // Save the document to the provided file path and check if it was successful
    bool save_succeeded = doc.save_file(full_path.c_str());
    if (!save_succeeded)
    {
        ROS_ERROR("[saveHMATxml] Failed to save homography matrix to XML File[%s]", full_path.c_str());
        return -1;
    }

    ROS_INFO("[saveHMATxml] Saved the homography matrix to XML File[%s]", full_path.c_str());
    return 0;
}

int xmlLoadHMAT(int mon_ind, int cal_ind, int grid_row, int grid_col, cv::Mat &out_H)
{
    // Get the full file path amd tag for the given calibration mode
    std::string file_path;
    std::string cal_tag;
    xmlFrmtFileStrings(mon_ind, cal_ind, file_path, cal_tag);

    // Create an XML document
    pugi::xml_document doc;

    // Parse the XML file
    pugi::xml_parse_result result = doc.load_file(file_path.c_str());
    if (!result)
    {
        ROS_ERROR("[xmlLoadHMAT] Failed to load homography matrix from XML File[%s]", file_path.c_str());
        return -1;
    }

    // Get the root node
    pugi::xml_node root_node = doc.child("Root");
    if (root_node.empty())
    {
        ROS_ERROR("[xmlLoadHMAT] The 'Root' node is missing in the XML File[%s]", file_path.c_str());
        return -1;
    }

    // Find the parent element under the root node
    pugi::xml_node parent_node = root_node.child(cal_tag.c_str());
    if (parent_node.empty())
    {
        ROS_ERROR("[xmlLoadHMAT] The '%s' node is missing in the XML File[%s]", cal_tag.c_str(), file_path.c_str());
        return -1;
    }

    // Find the specific HMAT node with the given row and col attributes
    for (pugi::xml_node hmat_node = parent_node.child("HMAT");
         hmat_node;
         hmat_node = hmat_node.next_sibling("HMAT"))
    {
        if (hmat_node.attribute("grid_row").as_int() == grid_row &&
            hmat_node.attribute("grid_col").as_int() == grid_col)
        {
            // Initialize a temporary matrix to store the values
            cv::Mat H = cv::Mat::zeros(MAZE_SIZE, MAZE_SIZE, CV_64F);

            // Iterate over each row node within the HMAT node
            int i = 0; // Row counter
            for (pugi::xml_node row_node = hmat_node.child("row");
                 row_node && i < MAZE_SIZE;
                 row_node = row_node.next_sibling("row"), ++i)
            {
                int j = 0; // Column counter

                // Iterate over each cell node within the row
                for (pugi::xml_node cell_node = row_node.child("cell");
                     cell_node && j < MAZE_SIZE;
                     cell_node = cell_node.next_sibling("cell"), ++j)
                {
                    // Get the value from the cell and store it in the matrix
                    double value = std::stod(cell_node.child_value());
                    H.at<double>(i, j) = value;
                }
            }

            // Copy the temporary matrix to the output parameter
            out_H = H;
            ROS_INFO("[xmlLoadHMAT] Loaded the homography matrix from XML File[%s]", file_path.c_str());
            return 0;
        }
    }

    // If the specific HMAT node was not found, return an error
    ROS_ERROR("[xmlLoadHMAT] The specified HMAT node with grid_row %d and grid_col %d under '%s' is missing in the XML File[%s]",
              grid_row, grid_col, cal_tag.c_str(), file_path.c_str());
    return -1;
}

int xmlSaveTupleHMATs(
    const std::tuple<
        std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>,
        std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>,
        std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>,
        cv::Mat> &_HMAT_TUPLE,
    int mon_ind, int cal_ind)
{
    // Handle wall homography matrices
    if (cal_ind < N_CAL_MODES - 1)
    {
        for (int gr_i = 0; gr_i < MAZE_SIZE; ++gr_i)
        {
            for (int gc_i = 0; gc_i < MAZE_SIZE; ++gc_i)
            {
                cv::Mat H;
                if (getTupleHMAT(_HMAT_TUPLE, cal_ind, gr_i, gc_i, H) < 0)
                    return -1;
                if (xmlSaveHMAT(H, mon_ind, cal_ind, gr_i, gc_i) < 0)
                    return -1;
            }
        }
    }
    // Handle single homography matrix
    else
    {
        cv::Mat H;
        if (getTupleHMAT(_HMAT_TUPLE, cal_ind, 0, 0, H) < 0)
            return -1;
        if (xmlSaveHMAT(H, mon_ind, cal_ind, 0, 0) < 0)
            return -1;
    }
    
    return 0;
}

int xmlLoadTupleHMATs(
    int mon_ind, int cal_ind,
    std::tuple<
        std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>,
        std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>,
        std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>,
        cv::Mat> &_HMAT_TUPLE)
{
    // Handle wall homography matrices
    if (cal_ind < N_CAL_MODES - 4)
    {
        for (int gr_i = 0; gr_i < MAZE_SIZE; ++gr_i)
        {
            for (int gc_i = 0; gc_i < MAZE_SIZE; ++gc_i)
            {
                cv::Mat H;
                if (xmlLoadHMAT(mon_ind, cal_ind, gr_i, gc_i, H) < 0)
                    return -1;
                if (setTupleHMAT(H, cal_ind, gr_i, gc_i, _HMAT_TUPLE) < 0)
                    return -1;
            }
        }
    }
    // Handle single homography matrix
    else
    {
        cv::Mat H;
        if (xmlLoadHMAT(mon_ind, cal_ind, 0, 0, H) < 0)
            return -1;
        if (setTupleHMAT(H, cal_ind, 0, 0, _HMAT_TUPLE) < 0)
            return -1;
    }
    return 0;
}

int getTupleHMAT(
    const std::tuple<
        std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>,
        std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>,
        std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>,
        cv::Mat> &_HMAT_TUPLE,
    int cal_ind, int grid_row, int grid_col, cv::Mat &out_H)
{
    // Check if the input matrix is 3x3
    if (grid_row < 0 || grid_row >= MAZE_SIZE || grid_col < 0 || grid_col >= MAZE_SIZE)
    {
        ROS_ERROR("[getTupleHMAT] Invalid grid_row[%d] or grid_col[%d] value", grid_row, grid_col);
        return -1;
    }

    switch (cal_ind)
    {
    case 0:
        out_H = std::get<0>(_HMAT_TUPLE)[grid_row][grid_col];
        break;
    case 1:
        out_H = std::get<1>(_HMAT_TUPLE)[grid_row][grid_col];
        break;
    case 2:
        out_H = std::get<2>(_HMAT_TUPLE)[grid_row][grid_col];
        break;
    case 3:
        out_H = std::get<3>(_HMAT_TUPLE);
        break;
    default:
        // Handle the error or unexpected cal_ind value
        ROS_ERROR("[getTupleHMAT] Invalid cal_ind[%d]", cal_ind);
        return -1;
    }

    return 0;
}

int setTupleHMAT(
    const cv::Mat &H, int cal_ind, int grid_row, int grid_col,
    std::tuple<
        std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>,
        std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>,
        std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE>,
        cv::Mat> &out_HMAT_TUPLE)
{
    // Check if the input matrix is 3x3
    if (grid_row < 0 || grid_row >= MAZE_SIZE || grid_col < 0 || grid_col >= MAZE_SIZE)
    {
        ROS_ERROR("[setTupleHMAT] Invalid grid_row[%d] or grid_col[%d] value", grid_row, grid_col);
        return -1;
    }

    switch (cal_ind)
    {
    case 0:
        std::get<0>(out_HMAT_TUPLE)[grid_row][grid_col] = H;
        break;
    case 1:
        std::get<1>(out_HMAT_TUPLE)[grid_row][grid_col] = H;
        break;
    case 2:
        std::get<2>(out_HMAT_TUPLE)[grid_row][grid_col] = H;
        break;
    case 3:
        std::get<3>(out_HMAT_TUPLE) = H;
        break;
    default:
        // Handle the error or unexpected cal_ind value
        ROS_ERROR("[setTupleHMAT] Invalid cal_ind[%d]", cal_ind);
        return -1;
    }

    return 0;
}

int checkQuadVertices(const std::vector<cv::Point2f> &quad_vertices)
{

    // Check if the input vector has exactly 4 vertices
    if (quad_vertices.size() != 4)
        return -1;

    // Check if any three points are collinear; for a valid quadrilateral, no three points should be collinear
    for (int i = 0; i < 4; ++i)
    {
        cv::Point2f p1 = quad_vertices[i];
        cv::Point2f p2 = quad_vertices[(i + 1) % 4];
        cv::Point2f p3 = quad_vertices[(i + 2) % 4];
        float area = p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y);
        if (std::abs(area) < 1e-5)
            return -2; // The points are collinear
    }
    return 0;
}

std::vector<cv::Point2f> quadVertNdc2Pxl(const std::vector<cv::Point2f> &quad_vertices_ndc, int window_width_pxl, int window_height_pxl)
{

    std::vector<cv::Point2f> quad_vertices_pxl;
    quad_vertices_pxl.reserve(quad_vertices_ndc.size());

    for (const auto &vertex : quad_vertices_ndc)
    {
        // Convert x point values from NDC to pixel coordinates
        float x_pixel = (vertex.x + 1.0f) * (window_width_pxl / 2.0f);

        // Convet y points but keep y-axis direction the same as NDC, with origin at the bottom
        float y_pixel = (vertex.y + 1.0f) * (window_height_pxl / 2.0f);

        // // Y is inverted because pixel coordinates increase downwards
        // float y_pixel = (1.0f - vertex.y) * ((float)window_height_pxl / 2.0f);

        // Store values
        quad_vertices_pxl.emplace_back(x_pixel, y_pixel);
    }

    return quad_vertices_pxl;
}

float bilinearInterpolation(float a, float b, float c, float d, int grid_row_i, int grid_col_i, int grid_size)
{
    // Calculate the relative position within the grid.
    float x = static_cast<float>(grid_col_i) / (grid_size - 1);
    float y = static_cast<float>(grid_row_i) / (grid_size - 1);

    // Perform bilinear interpolation using the formula.
    float interp_val = (1 - x) * (1 - y) * a +
                       x * (1 - y) * b +
                       (1 - x) * y * c +
                       x * y * d;

    // Return the final interpolated value.
    return interp_val;
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

int loadTexture(cv::Mat image, GLuint &textureID)
{
    int status = 0;

    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    // Convert image from BGR to RGB
    cv::Mat image_rgb;
    cv::cvtColor(image, image_rgb, cv::COLOR_BGR2RGB);

    // Handle alignment
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    // Create texture
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_rgb.cols,
                 image_rgb.rows, 0, GL_RGB, GL_UNSIGNED_BYTE,
                 image_rgb.data);
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    return status;
}

int initWallRenderObjects(MazeRenderContext &out_renCtx,
                          float *vertices, size_t verticesSize,
                          unsigned int *indices, size_t indicesSize)
{
    int status = 0;

    glfwMakeContextCurrent(out_renCtx.windowID);
    status = MazeRenderContext::CheckErrorGLFW(__LINE__, __FILE__);

    // Generate and bind an Element Buffer Object (EBO)
    glGenBuffers(1, &out_renCtx.ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, out_renCtx.ebo);
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    // Initialize the EBO with index data
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indicesSize, indices, GL_DYNAMIC_DRAW);
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    // Generate and bind a Vertex Array Object (VAO)
    glGenVertexArrays(1, &out_renCtx.vao);
    glBindVertexArray(out_renCtx.vao);
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    // Generate and bind a Vertex Buffer Object (VBO)
    glGenBuffers(1, &out_renCtx.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, out_renCtx.vbo);
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    // Initialize the VBO with vertex data
    glBufferData(GL_ARRAY_BUFFER, verticesSize, vertices, GL_STATIC_DRAW);
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    // Specify the format of the vertex data for the position attribute
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0); // Enable the position attribute
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    // Specify the format of the vertex data for the texture coordinate attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));
    glEnableVertexAttribArray(1); // Enable the texture coordinate attribute
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    // Unbind the VAO to prevent accidental modification
    glBindVertexArray(0);
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    // Return GL status
    return status;
}

int renderWallImage(MazeRenderContext &_renCtx)
{
    int status = 0;

    // Check the shader program for errors
    if (_renCtx.validateShaderProgram())
    {
        ROS_ERROR("[renderWallImage] Shader program validation failed");
        return -1;
    }
    glfwMakeContextCurrent(_renCtx.windowID);
    status = MazeRenderContext::CheckErrorGLFW(__LINE__, __FILE__);

    // Use the shader program for wall rendering
    glUseProgram(_renCtx.shaderProgram);

    // Bind the texture for the walls
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, _renCtx.textureID);

    // Bind the Vertex Array Object(VAO) specific to the current wall
    glBindVertexArray(_renCtx.vao);

    // Bind the common Element Buffer Object (EBO)
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _renCtx.ebo);

    // Draw the rectangle (2 triangles) for the current wall
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    // Unbind the VAO to prevent accidental modification
    glBindVertexArray(0);

    // Unset the shader program
    glUseProgram(0);
    status = MazeRenderContext::CheckErrorOpenGL(__LINE__, __FILE__);

    return status;
}