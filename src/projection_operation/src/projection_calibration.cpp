// ############################################################################################################

// ======================================== projection_calibration.cpp ========================================

// ############################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_calibration.h"

// ================================================== CLASS: MazeRenderContext ==================================================

// Initialize MazeRenderContext static members
GLFWmonitor **MazeRenderContext::_PP_Monitor = nullptr;
int MazeRenderContext::_NumMonitors = 0;

MazeRenderContext::MazeRenderContext()
    : shaderProgram(0), vao(0), vbo(0), ebo(0), textureID(0),
      windowID(nullptr), monitorID(nullptr), windowInd(-1), monitorInd(-1)
{
    // Members are initialized to default values, setup is deferred
}

MazeRenderContext::~MazeRenderContext()
{
    // Clean up resources without logging
    cleanupContext();
}

MazeRenderContext::MazeRenderContext(MazeRenderContext &&other) noexcept
    : windowID(other.windowID), monitorID(other.monitorID),
      windowInd(other.windowInd), monitorInd(other.monitorInd)
{
    // Reset the other's members to default values to prevent double deletion
    other.shaderProgram = 0;
    other.vao = 0;
    other.vbo = 0;
    other.ebo = 0;
    other.textureID = 0;
    other.windowID = nullptr;
    other.monitorID = nullptr;
    other.windowInd = -1;
    other.monitorInd = -1;
}

MazeRenderContext &MazeRenderContext::operator=(MazeRenderContext &&other) noexcept
{
    if (this != &other) // Prevent self-assignment
    {
        // Clean up existing resources
        glDeleteProgram(shaderProgram);
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
        glDeleteBuffers(1, &ebo);
        glDeleteTextures(1, &textureID);

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

        // Reset the other's members to default values
        other.shaderProgram = 0;
        other.vao = 0;
        other.vbo = 0;
        other.ebo = 0;
        other.textureID = 0;
        other.windowID = nullptr;
        other.monitorID = nullptr;
        other.windowInd = -1;
        other.monitorInd = -1;
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
    const char *description;
    int glfw_err = glfwGetError(&description);
    if (glfw_err != GLFW_NO_ERROR)
    {
        ROS_ERROR("[checkErrorGLFW] Message[%s] Description[%s] File[%s] Line[%d]",
                  msg_str ? msg_str : "No additional info", description, file_str, line);
        return -1;
    }
    return 0;
}

int MazeRenderContext::SetupGraphicsLibraries(int &out_n_mon)
{
    // Initialize GLFW and set error callback
    glfwSetErrorCallback(CallbackErrorGLFW);
    if (!glfwInit())
    {
        ROS_ERROR("[MazeRenderContext::SetupGraphicsLibraries] GLFW Initialization Failed");
        return -1;
    }

    // Request a debug context for future windows
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);      // Specify OpenGL version if needed
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);      // Specify OpenGL version if needed
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE); // Request debug context

    // Discover available monitors
    _PP_Monitor = glfwGetMonitors(&_NumMonitors);
    if (!_PP_Monitor || _NumMonitors == 0)
    {
        ROS_ERROR("[MazeRenderContext::SetupGraphicsLibraries] No Monitors Found");
        return -1;
    }
    ROS_INFO("[MazeRenderContext::SetupGraphicsLibraries] Monitors Found: %d", _NumMonitors);

    // Set the number of monitors for the reference output argument
    out_n_mon = _NumMonitors;

    return 0;
}

int MazeRenderContext::CleanupGraphicsLibraries()
{
    int err_status = 0;

    // Terminate GLFW, which cleans up all GLFW resources.
    glfwTerminate();
    err_status = checkErrorGLFW(__LINE__, __FILE__, "[MazeRenderContext::CleanupGraphicsLibraries] Error Flagged Following glfwTerminate()");

    // Reset monitor pointers and count
    _PP_Monitor = nullptr;
    _NumMonitors = 0;

    if (err_status < 0)
        ROS_WARN("[MazeRenderContext::CleanupGraphicsLibraries] Failed to Terminate GLFW Library");
    else
        ROS_INFO("[MazeRenderContext::CleanupGraphicsLibraries] Graphics libraries and shared resources cleaned up successfully.");

    return err_status;
}

int MazeRenderContext::initContext(int win_ind, int mon_ind)
{

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
    windowInd = win_ind; // Store the window index

    // Set the GLFW window as the current OpenGL context
    glfwMakeContextCurrent(windowID);

    // Load OpenGL extensions using GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress) || !gladLoadGL())
    {
        glfwDestroyWindow(windowID);
        ROS_ERROR("[MazeRenderContext::initContext] Failed to load GLAD");
        return -1;
    }

    // Set GLFW callbacks for keyboard and framebuffer size events
    glfwSetKeyCallback(windowID, callbackKeyBinding);
    glfwSetFramebufferSizeCallback(windowID, CallbackFrameBufferSizeGLFW);

    // Enable OpenGL debugging context and associate callback
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(CallbackDebugOpenGL, nullptr);

    return checkErrorGLFW(__LINE__, __FILE__, "MazeRenderContext::switchWindowMode");
}

int MazeRenderContext::cleanupContext(bool log_errors)
{
    int err_status = 0;

    if (shaderProgram != 0)
    {
        glDeleteProgram(shaderProgram);
        err_status |= checkErrorOpenGL(__LINE__, __FILE__, "[cleanup] glDeleteProgram error");
        if (log_errors)
            ROS_INFO("[cleanup] %s", err_status ? "Failed to delete shader program" : "Shader program deleted successfully");
        shaderProgram = 0;
    }

    if (textureID != 0)
    {
        glDeleteTextures(1, &textureID);
        err_status |= checkErrorOpenGL(__LINE__, __FILE__, "[cleanup] glDeleteTextures error");
        if (log_errors)
            ROS_INFO("[cleanup] %s", err_status ? "Failed to delete texture" : "Texture deleted successfully");
        textureID = 0;
    }

    if (vao != 0)
    {
        glDeleteVertexArrays(1, &vao);
        err_status |= checkErrorOpenGL(__LINE__, __FILE__, "[cleanup] glDeleteVertexArrays error");
        if (log_errors)
            ROS_INFO("[cleanup] %s", err_status ? "Failed to delete VAO" : "VAO deleted successfully");
        vao = 0;
    }

    if (vbo != 0)
    {
        glDeleteBuffers(1, &vbo);
        err_status |= checkErrorOpenGL(__LINE__, __FILE__, "[cleanup] glDeleteBuffers error");
        if (log_errors)
            ROS_INFO("[cleanup] %s", err_status ? "Failed to delete VBO" : "VBO deleted successfully");
        vbo = 0;
    }

    if (ebo != 0)
    {
        glDeleteBuffers(1, &ebo);
        err_status |= checkErrorOpenGL(__LINE__, __FILE__, "[cleanup] glDeleteBuffers error");
        if (log_errors)
            ROS_INFO("[cleanup] %s", err_status ? "Failed to delete EBO" : "EBO deleted successfully");
        ebo = 0;
    }

    if (windowID != nullptr)
    {
        glfwDestroyWindow(windowID);
        err_status |= checkErrorGLFW(__LINE__, __FILE__, "[cleanup] glfwDestroyWindow error");
        if (log_errors)
            ROS_INFO("[cleanup] %s", err_status ? "Failed to destroy GLFW window" : "GLFW window destroyed successfully");
        windowID = nullptr;
    }

    return err_status ? -1 : 0;
}

int MazeRenderContext::compileAndLinkShaders(const GLchar *vertex_source, const GLchar *fragment_source)
{
    // Compile vertex shader
    GLuint temp_vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(temp_vertex_shader, 1, &vertex_source, nullptr);
    glCompileShader(temp_vertex_shader);
    if (!_checkShaderCompilation(temp_vertex_shader, "VERTEX"))
    {
        glDeleteShader(temp_vertex_shader);
        ROS_ERROR("[MazeRenderContext] Vertex shader compilation failed.");
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
        ROS_ERROR("[MazeRenderContext] Fragment shader compilation failed.");
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
        ROS_ERROR("[MazeRenderContext] Shader program linking failed.");
        return -1;
    }

    // Cleanup: detach and delete shaders
    glDetachShader(shaderProgram, temp_vertex_shader);
    glDeleteShader(temp_vertex_shader);
    glDetachShader(shaderProgram, temp_fragment_shader);
    glDeleteShader(temp_fragment_shader);

    return checkErrorOpenGL(__LINE__, __FILE__, "MazeRenderContext::compileAndLinkShaders");
}

int MazeRenderContext::cleanupShaderObjects()
{
    // Use OpenGL calls to delete shader program and shaders
    if (shaderProgram != 0)
    {
        glDeleteProgram(shaderProgram);
        shaderProgram = 0;
        return checkErrorOpenGL(__LINE__, __FILE__, "MazeRenderContext::cleanupShaderObjects");
    }
    else
        return -1;
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

    // Update window title with window and monitor indices
    std::string new_title = "Window[" + std::to_string(windowInd) + "] Monitor[" + std::to_string(mon_ind_new) + "]";
    glfwSetWindowTitle(windowID, new_title.c_str());
    ROS_INFO("[switchWindowMode] Move Window: Monitor[%d] Format[%s]", mon_ind_new, do_fullscreen ? "fullscreen" : "windowed");

    return checkErrorGLFW(__LINE__, __FILE__, "MazeRenderContext::switchWindowMode");
}

void MazeRenderContext::flashBackgroundColor(const cv::Scalar &color, float duration)
{
    // Set the new clear color using BGR values from cv::Scalar
    glClearColor(color[2], color[1], color[0], 1.0f); // RGBA

    // Clear the window with the new color
    glClear(GL_COLOR_BUFFER_BIT);
    glfwSwapBuffers(windowID);

    // Wait for the duration of the flash
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(duration * 1000)));

    // Reset the clear color (assuming the default is black)
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
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

void _testCallbacks(GLFWwindow *win)
{
    ROS_INFO("============== START: CALLBACK DEBUGGIN ==============");
    // Trigger buffer
    glfwSetWindowSize(win, 800, 600); // Change the size to something different

    // Insert a debug message manually
    glDebugMessageInsert(GL_DEBUG_SOURCE_APPLICATION, GL_DEBUG_TYPE_ERROR, 0, GL_DEBUG_SEVERITY_HIGH, -1, "Test debug message");
    checkErrorOpenGL(__LINE__, __FILE__, "[main] Error Flagged Following glDebugMessageInsert()");

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

void initCtrlPtCoords(std::array<std::array<cv::Point2f, 4>, 4> &out_CP_GRID_ARR)
{
    // Specify the control point limits
    const float cp_x = MAZE_WIDTH_NDC / 2;  // starting X-coordinate in NDC coordinates
    const float cp_y = MAZE_HEIGHT_NDC / 2; // starting Y-coordinate in NDC coordinates

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

    // Return GL status
    return checkErrorOpenGL(__LINE__, __FILE__, "compileAndLinkShaders");
}

int initWallRenderObjects(MazeRenderContext &out_renCtx,
                          float *vertices, size_t verticesSize,
                          unsigned int *indices, size_t indicesSize)
{
    // Generate and bind an Element Buffer Object (EBO)
    glGenBuffers(1, &out_renCtx.ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, out_renCtx.ebo);

    // Initialize the EBO with index data
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indicesSize, indices, GL_DYNAMIC_DRAW);

    // Generate and bind a Vertex Array Object (VAO)
    glGenVertexArrays(1, &out_renCtx.vao);
    glBindVertexArray(out_renCtx.vao);

    // Generate and bind a Vertex Buffer Object (VBO)
    glGenBuffers(1, &out_renCtx.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, out_renCtx.vbo);

    // Initialize the VBO with vertex data
    glBufferData(GL_ARRAY_BUFFER, verticesSize, vertices, GL_STATIC_DRAW);

    // Specify the format of the vertex data for the position attribute
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0); // Enable the position attribute

    // Specify the format of the vertex data for the texture coordinate attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));
    glEnableVertexAttribArray(1); // Enable the texture coordinate attribute

    // Unbind the VAO to prevent accidental modification
    glBindVertexArray(0);

    // Return GL status
    return checkErrorOpenGL(__LINE__, __FILE__, "initWallRenderObjects");
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
    return checkErrorOpenGL(__LINE__, __FILE__, "initCircleRendererObjects");
}

int updateWallTexture(
    cv::Mat img_wall_mat, cv::Mat img_mode_mon_mat, cv::Mat img_mode_cal_mat,
    std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> &_HMAT_GRID_ARR,
    GLuint &out_WALL_TEXTURE_ID)
{
    // Initialize the image to be used as the texture
    cv::Mat im_wall_merge = cv::Mat::zeros(WINDOW_HEIGHT_PXL, WINDOW_WIDTH_PXL, CV_8UC4);

    // Iterate through the maze grid rows
    for (float gr_i = 0; gr_i < MAZE_SIZE; gr_i++) // image bottom to top
    {
        // Iterate through each column in the maze row
        for (float gc_i = 0; gc_i < MAZE_SIZE; gc_i++) // image left to right
        {
            // Copy wall image
            cv::Mat img_copy;
            img_wall_mat.copyTo(img_copy);

            // Get the maze vertex indice cooresponding to the selected control point
            int mv_ind = I.cpMap[I.cpMazeVertSel[0]][I.cpMazeVertSel[1]];

            //  Create merged image for the wall corresponding to the selected control point
            if (
                (mv_ind == 0 && gr_i == 0 && gc_i == 0) ||
                (mv_ind == 1 && gr_i == 0 && gc_i == MAZE_SIZE - 1) ||
                (mv_ind == 3 && gr_i == MAZE_SIZE - 1 && gc_i == 0) ||
                (mv_ind == 2 && gr_i == MAZE_SIZE - 1 && gc_i == MAZE_SIZE - 1))
            {
                // Merge test pattern and active monitor image
                if (mergeImgMat(img_mode_mon_mat, img_copy) < 0)
                    return -1;

                // Merge previous image and active calibration image
                if (mergeImgMat(img_mode_cal_mat, img_copy) < 0)
                    return -1;
            }

            // Get homography matrix for this wall
            cv::Mat H = _HMAT_GRID_ARR[gr_i][gc_i];

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

int renderWallImage(const MazeRenderContext &mazeRenderContext)
{
    // Use the shader program for wall rendering
    glUseProgram(mazeRenderContext.shaderProgram);

    // Bind the texture for the walls
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, mazeRenderContext.textureID);

    // Bind the Vertex Array Object(VAO) specific to the current wall
    glBindVertexArray(mazeRenderContext.vao);

    // Bind the common Element Buffer Object (EBO)
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mazeRenderContext.ebo);

    // Draw the rectangle (2 triangles) for the current wall
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    // Unbind the VAO to prevent accidental modification
    glBindVertexArray(0);

    // Unset the shader program
    glUseProgram(0);

    // Return GL status
    return checkErrorOpenGL(__LINE__, __FILE__, "renderWallImage");
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
            if (checkErrorOpenGL(__LINE__, __FILE__) < 0)
            {
                ROS_ERROR("[renderControlPoints] Error Thrown for Control Point[%d][%d]", mv_i, wv_i);
                return -1;
            }
        }
    }

    // Unset the shader program
    CircleRenderer::UnsetShader();

    // // TEMP
    // return -1;

    // Return GL status
    return checkErrorOpenGL(__LINE__, __FILE__, "renderControlPoints");
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
    RenContx[0].initContext(0, 0);

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

    // Log OpenGL versions
    const GLubyte *opengl_version = glGetString(GL_VERSION);
    ROS_INFO("[main] OpenGL initialized: Version[%s]", opengl_version);

    // Log GLFW versions
    int glfw_major, glfw_minor, glfw_rev;
    glfwGetVersion(&glfw_major, &glfw_minor, &glfw_rev);
    ROS_INFO("[main] GLFW initialized: Version[%d.%d.%d]", glfw_major, glfw_minor, glfw_rev);

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

    bool is_error = false;
    while (!glfwWindowShouldClose(RenContx[0].windowID) && ros::ok())
    {

        // --------------- Check Kayboard Callback Flags ---------------

        // Load/save XML file
        if (F.loadXML || F.saveXML)
        {
            std::string file_path = frmtFilePathxml(I.winMon, I.calMode, CONFIG_DIR_PATH);
            if (F.saveXML) // Load XML file
                saveHMATxml(file_path, HMAT_GRID_ARR);
            else if (F.loadXML) // Save XML file
                loadHMATxml(file_path, HMAT_GRID_ARR);
            RenContx[0].flashBackgroundColor(cv::Scalar(0.1f, 0.0f, 0.0f), 0.5f);
            F.saveXML = false;
            F.loadXML = false;
        }

        // Update the window monitor and mode
        if (F.switchWindowMode)
        {
            if (RenContx[0].switchWindowMode(I.winMon, F.setFullscreen) < 0)
            {
                ROS_ERROR("[main] Update window monitor mode threw an error");
                is_error = true;
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
                is_error = true;
                break;
            }

            // Update wall image texture
            if (updateWallTexture(wallImgMatVec[I.wallImage], monImgMatVec[I.winMon], calImgMatVec[I.calMode], HMAT_GRID_ARR, RenContx[0].textureID) < 0)
            {
                ROS_ERROR("[main] Update of wall homography datasets failed");
                is_error = true;
                break;
            }
            F.updateWallTexture = false;
        }

        // --------------- Handle Image Processing for Next Frame ---------------

        // Clear back buffer for new frame
        glClear(GL_COLOR_BUFFER_BIT);
        if (checkErrorGLFW(__LINE__, __FILE__, "[main] Error flagged following glclear()"))
        {
            is_error = true;
            break;
        }

        // Draw/update wall images
        if (renderWallImage(RenContx[0]) < 0)
        {
            ROS_ERROR("[main] Draw walls threw an error");
            is_error = true;
            break;
        }

        // Draw/update control point markers
        if (renderControlPoints(CP_GRID_ARR, CP_RENDERERS) < 0)
        {
            ROS_ERROR("[main] Draw control point threw an error");
            is_error = true;
            break;
        }

        // Swap buffers and poll events
        glfwSwapBuffers(RenContx[0].windowID);
        if (checkErrorGLFW(__LINE__, __FILE__, "[main] Error flagged following glfwSwapBuffers()") < 0)
        {
            is_error = true;
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
    else if (is_error)
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
    return is_error ? -1 : 0;
}
