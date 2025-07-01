// ######################################################################################################

// ======================================== projection_utils.cpp ========================================

// ######################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_utils.h"

void CallbackFrameBufferSizeGLFW(GLFWwindow *window, int width, int height) {
    glViewport(0, 0, width, height);
    CheckErrorGLFW(__LINE__, __FILE__, "CallbackFrameBufferSizeGLFW");
}

void CallbackDebugOpenGL(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message, const void *userParam) {
    // Get level of severity
    int s_level = (severity == GL_DEBUG_SEVERITY_NOTIFICATION) ? 1 : (severity == GL_DEBUG_SEVERITY_LOW)  ? 2
                                                                 : (severity == GL_DEBUG_SEVERITY_MEDIUM) ? 3
                                                                 : (severity == GL_DEBUG_SEVERITY_HIGH)   ? 4
                                                                                                          : 0;
    // Check if the message is below the specified debug level
    if (s_level < GLB_DEBUG_LEVEL_GL) return;
    if (s_level == 1)
        ROS_INFO("[CallbackDebugOpenGL] Type[0x%x] ID[%d] Severity[%d:0x%x] Message[%s]", type, id, s_level, severity, message);
    else
        ROS_ERROR("[CallbackDebugOpenGL] Type[0x%x] ID[%d] Severity[%d:0x%x] Message[%s]", type, id, s_level, severity, message);
}

void CallbackErrorGLFW(int error, const char *description) {
    ROS_ERROR("[CallbackErrorGLFW] Error[%d] Description[%s]", error, description);
}

int CheckErrorOpenGL(int line, const char *file_str, const char *msg_str) {
    GLenum gl_err;
    while ((gl_err = glGetError()) != GL_NO_ERROR) {
        ROS_ERROR("[CheckErrorOpenGL] Message[%s] Error Number[%u] File[%s] Line[%d]",
                  msg_str ? msg_str : "No additional info", gl_err, file_str, line);
        return -1;
    }
    return 0;
}

int CheckErrorGLFW(int line, const char *file_str, const char *msg_str) {
    const char *description;
    int glfw_err = glfwGetError(&description);
    if (glfw_err != GLFW_NO_ERROR) {
        ROS_ERROR("[CheckErrorGLFW] Message[%s] Description[%s] File[%s] Line[%d]",
                  msg_str ? msg_str : "No additional info", description, file_str, line);
        return -1;
    }
    return 0;
}

int SetupGraphicsLibraries() {
    int status = 0;

    // Initialize GLFW and set error callback
    glfwSetErrorCallback(CallbackErrorGLFW);
    if (!glfwInit()) {
        ROS_ERROR("[SetupGraphicsLibraries] GLFW Initialization Failed");
        return -1;
    }
    status = CheckErrorGLFW(__LINE__, __FILE__) < 0 ? -1 : status;

    // Request a debug context for future windows (all windows can share the same context)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);                 // Specify OpenGL version if needed
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);                 // Specify OpenGL version if needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // Request core profile
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);            // Request debug context
    status = CheckErrorGLFW(__LINE__, __FILE__) < 0 ? -1 : status;

    // Discover available monitors
    MONITORS = glfwGetMonitors(&N_MONITORS);
    status = CheckErrorGLFW(__LINE__, __FILE__) < 0 ? -1 : status;

    // Check for errors in monitor discovery
    if (!MONITORS || N_MONITORS == 0) {
        ROS_ERROR("[SetupGraphicsLibraries] No Monitors Found");
        return -1;
    }

    // Sort the monitors by x
    int monitor_x, monitor_y, monitor_width, monitor_height;
    std::vector<std::pair<int, int>> monitor_x_ind;
    for (int i = 0; i < N_MONITORS; i++) {
        glfwGetMonitorWorkarea(MONITORS[i], &monitor_x, &monitor_y, &monitor_width, &monitor_height);
        monitor_x_ind.push_back(std::make_pair(monitor_x, i));
    }
    std::sort(monitor_x_ind.begin(), monitor_x_ind.end());

    // Store the monitor indices for projection
    PROJ_MON_VEC.clear();
    for (auto &pair : monitor_x_ind) PROJ_MON_VEC.push_back(pair.second);

    // Take out first monitor as the default projection monitor
    PROJ_MON_VEC.erase(PROJ_MON_VEC.begin());

    ROS_INFO("[SetupGraphicsLibraries] Monitors Found: %d", N_MONITORS);

    return 0;
}

int CleanupGraphicsLibraries() {
    int status = 0;

    // Terminate GLFW, which cleans up all GLFW resources.
    glfwTerminate();
    status = CheckErrorGLFW(__LINE__, __FILE__);
    if (status < 0) ROS_WARN("[CleanupGraphicsLibraries] Failed to Terminate GLFW Library");
    else ROS_INFO("[CleanupGraphicsLibraries] Graphics libraries and shared resources cleaned up successfully.");

    // Reset monitor pointers and count
    MONITORS = nullptr;
    N_MONITORS = 0;

    return status;
}


// ================================================== CLASS: MazeRenderContext ==================================================

// Constructor
MazeRenderContext::MazeRenderContext()
    : _shaderProgram(0), _vao(0), _vbo(0), _ebo(0), textureID(0),
      window(nullptr), monitor(nullptr),
      _windowWidthPxl(1024), _windowHeightPxl(576),
      windowInd(-1), monitorInd(-1)
{// Members are initialized to default values, setup is deferred
}

MazeRenderContext::~MazeRenderContext() {
    // Clean up resources without logging
    if (isContextInitialized) cleanupContext(false);
}

MazeRenderContext::MazeRenderContext(MazeRenderContext &&other) noexcept
    : textureID(other.textureID),
      window(other.window),
      monitor(other.monitor),
      windowInd(other.windowInd),
      monitorInd(other.monitorInd),
      isContextInitialized(other.isContextInitialized),
      isFullScreen(other.isFullScreen),
      _windowWidthPxl(other._windowWidthPxl),
      _windowHeightPxl(other._windowHeightPxl),
      _shaderProgram(other._shaderProgram),
      _vao(other._vao),
      _vbo(other._vbo),
      _ebo(other._ebo)
{   // Reset the other's members to prevent double deletion
    other._resetMembers(); // Use a member function for resetting
}

MazeRenderContext &MazeRenderContext::operator=(MazeRenderContext &&other) noexcept {
    if (this != &other) {
        // Clean up existing resources if initialized
        if (isContextInitialized) cleanupContext(false);

        // Transfer ownership of resources from other to this
        textureID = other.textureID;
        window = other.window;
        monitor = other.monitor;
        windowInd = other.windowInd;
        monitorInd = other.monitorInd;
        isContextInitialized = other.isContextInitialized;
        isFullScreen = other.isFullScreen;
        _windowHeightPxl = other._windowHeightPxl;
        _windowWidthPxl = other._windowWidthPxl;
        _shaderProgram = other._shaderProgram;
        _vao = other._vao;
        _vbo = other._vbo;
        _ebo = other._ebo;

        // Reset the other's members to default values
        other._resetMembers();
    }
    return *this;
}

void MazeRenderContext::makeContextCurrent() {
    // Set the GLFW window as the current OpenGL context
    if (window == nullptr) {
        ROS_ERROR("[MazeRenderContext::makeContextCurrent] Window is not initialized.");
        return;
    }
    glfwMakeContextCurrent(window);
}

int MazeRenderContext::compileAndLinkShaders(const GLchar *vertex_source, const GLchar *fragment_source) {
    int status = 0;

    // Set the GLFW window as the current OpenGL context
    if (window == nullptr) return -1;
    glfwMakeContextCurrent(window);
    status = CheckErrorGLFW(__LINE__, __FILE__) < 0 ? -1 : status;

    // Check that context is initialized
    if (!isContextInitialized) {
        ROS_ERROR("[MazeRenderContext::compileAndLinkShaders] Context Not Initialized");
        return -1;
    }

    // Compile vertex shader
    GLuint temp_vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(temp_vertex_shader, 1, &vertex_source, nullptr);
    glCompileShader(temp_vertex_shader);
    if (_checkShaderCompilation(temp_vertex_shader, "VERTEX") < 0) {
        glDeleteShader(temp_vertex_shader);
        ROS_ERROR("[MazeRenderContext::compileAndLinkShaders] Vertex shader compilation failed.");
        return -1;
    }
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Compile fragment shader
    GLuint temp_fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(temp_fragment_shader, 1, &fragment_source, nullptr);
    glCompileShader(temp_fragment_shader);
    if (_checkShaderCompilation(temp_fragment_shader, "FRAGMENT") < 0) {
        glDeleteShader(temp_vertex_shader);
        glDeleteShader(temp_fragment_shader);
        ROS_ERROR("[MazeRenderContext::compileAndLinkShaders] Fragment shader compilation failed.");
        return -1;
    }
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Link shaders into a program
    _shaderProgram = glCreateProgram();
    glAttachShader(_shaderProgram, temp_vertex_shader);
    glAttachShader(_shaderProgram, temp_fragment_shader);
    glLinkProgram(_shaderProgram);
    if (_checkProgramLinking(_shaderProgram) < 0) {
        glDeleteShader(temp_vertex_shader);
        glDeleteShader(temp_fragment_shader);
        glDeleteProgram(_shaderProgram);
        ROS_ERROR("[MazeRenderContext::compileAndLinkShaders] Shader program linking failed.");
        return -1;
    }
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Cleanup: detach and delete shaders
    glDetachShader(_shaderProgram, temp_vertex_shader);
    glDeleteShader(temp_vertex_shader);
    glDetachShader(_shaderProgram, temp_fragment_shader);
    glDeleteShader(temp_fragment_shader);
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    return status;
}

int MazeRenderContext::checkShaderProgram() {
    if (_shaderProgram == 0) {
        ROS_ERROR("[MazeRenderContext::checkShaderProgram] No shader program to validate.");
        return -1;
    }

    glValidateProgram(_shaderProgram);
    GLint validation_status;
    glGetProgramiv(_shaderProgram, GL_VALIDATE_STATUS, &validation_status);

    if (validation_status == GL_FALSE) {
        GLint log_length;
        glGetProgramiv(_shaderProgram, GL_INFO_LOG_LENGTH, &log_length);

        // The max length includes the NULL character
        std::vector<GLchar> error_log(log_length);
        glGetProgramInfoLog(_shaderProgram, log_length, &log_length, &error_log[0]);

        // Log the error
        ROS_ERROR("[MazeRenderContext::checkShaderProgram] Shader program validation failed: %s", &error_log[0]);

        // Delete the shader program if you won't need it after a failed validation
        glDeleteProgram(_shaderProgram);
        _shaderProgram = 0;

        return -1;
    }
    return 0;
}

int MazeRenderContext::initWindowContext(int win_ind, int mon_ind, int win_width, int win_height, KeyCallbackFunc key_callback) {
    int status = 0;

    // Check/set the new monitor id
    _setMonitor(mon_ind);

    // Store the window size
    _windowWidthPxl = win_width;
    _windowHeightPxl = win_height;

    // Create a new GLFW window
    window = glfwCreateWindow(_windowWidthPxl, _windowHeightPxl, "", monitor, NULL); // Use the monitor in window creation
    if (!window) {
        glfwTerminate();
        ROS_ERROR("[MazeRenderContext::initWindowContext] GLFW Failed to Create Window");
        return -1;
    }
    status = CheckErrorGLFW(__LINE__, __FILE__) < 0 ? -1 : status;
    windowInd = win_ind; // Store the window index

    // Set the GLFW window as the current OpenGL context
    glfwMakeContextCurrent(window);
    status = CheckErrorGLFW(__LINE__, __FILE__) < 0 ? -1 : status;

    // Very imporant flag!
    isContextInitialized = true;

    // Load OpenGL extensions using GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress) || !gladLoadGL()) {
        glfwDestroyWindow(window);
        ROS_ERROR("[MazeRenderContext::initWindowContext] Failed to load GLAD");
        return -1;
    }

    // Set GLFW callbacks for keyboard events using the KeyCallbackFunc function pointer
    if (key_callback != nullptr) glfwSetKeyCallback(window, key_callback);
    status = CheckErrorGLFW(__LINE__, __FILE__) < 0 ? -1 : status;

    // Set GLFW callbacks for framebuffer size events
    glfwSetFramebufferSizeCallback(window, CallbackFrameBufferSizeGLFW);
    status = CheckErrorGLFW(__LINE__, __FILE__) < 0 ? -1 : status;

    // Enable OpenGL debugging context and associate callback
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(CallbackDebugOpenGL, nullptr);
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Print OpenGL version information once when the first window is created
    if (windowInd == 0) {
        // Log OpenGL versions
        const GLubyte *opengl_version = glGetString(GL_VERSION);
        ROS_INFO("[MazeRenderContext::initWindowContext] OpenGL initialized: Version[%s]", opengl_version);

        // Log GLFW versions
        int glfw_major, glfw_minor, glfw_rev;
        glfwGetVersion(&glfw_major, &glfw_minor, &glfw_rev);
        ROS_INFO("[MazeRenderContext::initWindowContext]  GLFW initialized: Version[%d.%d.%d]", glfw_major, glfw_minor, glfw_rev);
        status = CheckErrorGLFW(__LINE__, __FILE__) < 0 ? -1 : status;
    }
    return status;
}

int MazeRenderContext::initRenderObjects(float *vertices, size_t vertices_size,
                                         unsigned int *indices, size_t indices_size) {
    int status = 0;

    // Set the GLFW window as the current OpenGL context
    if (window == nullptr) return -1;
    glfwMakeContextCurrent(window);
    status = CheckErrorGLFW(__LINE__, __FILE__) < 0 ? -1 : status;

    // Generate and bind an Element Buffer Object (EBO)
    glGenBuffers(1, &_ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);
    status = CheckErrorOpenGL(__LINE__, __FILE__);

    // Initialize the EBO with index data
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_size, indices, GL_DYNAMIC_DRAW);
    status = CheckErrorOpenGL(__LINE__, __FILE__);

    // Generate and bind a Vertex Array Object (VAO)
    glGenVertexArrays(1, &_vao);
    glBindVertexArray(_vao);
    status = CheckErrorOpenGL(__LINE__, __FILE__);

    // Generate and bind a Vertex Buffer Object (VBO)
    glGenBuffers(1, &_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);
    status = CheckErrorOpenGL(__LINE__, __FILE__);

    // Initialize the VBO with vertex data
    glBufferData(GL_ARRAY_BUFFER, vertices_size, vertices, GL_STATIC_DRAW);
    status = CheckErrorOpenGL(__LINE__, __FILE__);

    // Specify the format of the vertex data for the position attribute
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0); // Enable the position attribute
    status = CheckErrorOpenGL(__LINE__, __FILE__);

    // Specify the format of the vertex data for the texture coordinate attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));
    glEnableVertexAttribArray(1); // Enable the texture coordinate attribute
    status = CheckErrorOpenGL(__LINE__, __FILE__);

    // Unbind the VAO to prevent accidental modification
    glBindVertexArray(0);
    status = CheckErrorOpenGL(__LINE__, __FILE__);

    // Set the texture parameters for minification and magnification filters to GL_LINEAR.
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // status = CheckErrorOpenGL(__LINE__, __FILE__);

    // Return GL status
    return status;
}

void MazeRenderContext::initWindowForDrawing() {
    // Clear the back buffer for a new frame
    glClear(GL_COLOR_BUFFER_BIT);
}

void MazeRenderContext::loadMatTexture(cv::Mat img_mat) {
    // Generate and bind the texture
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    // Convert image from BGR to RGB
    cv::cvtColor(img_mat, img_mat, cv::COLOR_BGR2RGB);

    // Handle alignment
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    // Create texture
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_mat.cols,
                 img_mat.rows, 0, GL_RGB, GL_UNSIGNED_BYTE,
                 img_mat.data);

    // Set the texture parameters for minification and magnification filters to GL_LINEAR.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

void MazeRenderContext::drawTexture() {
    // Check the shader program for errors
    // checkShaderProgram();

    // Use the shader program for wall rendering
    glUseProgram(_shaderProgram);

    // Bind the texture for the image
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, textureID);

    // Bind the Vertex Array Object(VAO) specific to the current wall
    glBindVertexArray(_vao);

    // Bind the common Element Buffer Object (EBO)
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);

    // Draw the rectangle (2 triangles) for the current wall
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    // Unbind the VAO to prevent accidental modification
    glBindVertexArray(0);

    // Unset the shader program
    glUseProgram(0);
}

void MazeRenderContext::bufferSwapPoll() {
    // Swap buffers 
    glfwSwapBuffers(window);

    // Poll events
    glfwPollEvents();
}

void MazeRenderContext::changeWindowDisplayMode(int mon_ind_new, bool do_fullscreen, cv::Point offset_xy) {
    int win_x, win_y;
    int win_width, win_height;

    // Update global
    isFullScreen = do_fullscreen;

    // Check/set the new monitor id
    _setMonitor(mon_ind_new);

    // Get the video mode of the selected monitor
    const GLFWvidmode *mode = glfwGetVideoMode(monitor);
    if (!mode) { // Validate video mode retrieval
        ROS_ERROR("[MazeRenderContext::changeWindowDisplayMode] Failed to Get Video Mode: Monitor[%d]", mon_ind_new);
        return;
    }

    // Set position to match fullscreeen
    win_width = mode->width;
    win_height = mode->height;
    win_x = 0;
    win_y = 0;

    // Update window to full-screen or windowed mode based on 'isFullScreen'
    if (isFullScreen)
        glfwSetWindowMonitor(window, monitor, win_x, win_x, win_width, win_height, mode->refreshRate);
    else {
        // Get monitor position
        int monitor_x = 0, monitor_y = 0; // Initialize to default values
        glfwGetMonitorPos(monitor, &monitor_x, &monitor_y);

        if (monitor_x < 0 || monitor_y < 0) // Validate monitor position
            ROS_WARN("[MazeRenderContext::changeWindowDisplayMode] Invalid Monitor Position: Monitor[%d] X[%d] Y[%d]", mon_ind_new, monitor_x, monitor_y);

        // Calculate window size based on aspect ratio of the inialized window dimensions
        win_width = static_cast<int>(576.0f * (_windowWidthPxl / static_cast<float>(_windowHeightPxl)));
        win_height = 576.0f;

        // Compute window position based on monitor position and offset with a minimum offset of 100 pixels
        win_x = monitor_x + offset_xy.x + 100;
        win_y = monitor_y + offset_xy.y + 100;

        // Set windowed mode position and size and include offset
        glfwSetWindowMonitor(window, NULL, win_x, win_y, win_width, win_height, 0);
    }

    // Update window title with window and monitor indices
    std::string new_title = "Window[" + std::to_string(windowInd) + "] Monitor[" + std::to_string(mon_ind_new) + "]";
    glfwSetWindowTitle(window, new_title.c_str());

    // Log the new window display mode if verbose logging is enabled
    if (GLB_DO_VERBOSE_DEBUG)
        ROS_INFO("[MazeRenderContext::changeWindowDisplayMode] Modified Window[%d]: Monitor[%d] Position[%d,%d,%d,%d] Format[%s]",
                 windowInd, monitorInd, win_x, win_y, win_width, win_height, isFullScreen ? "fullscreen" : "windowed");
}

void MazeRenderContext::forceWindowFocus() {
    // Check if the window is minimized (iconified)
    if (glfwGetWindowAttrib(window, GLFW_ICONIFIED))
        glfwRestoreWindow(window); // If the window is minimized and in fullscreen mode, restore the window
}

int MazeRenderContext::flashBackgroundColor(const cv::Scalar &color, int duration) {
    int status = 0;

    // Set the GLFW window as the current OpenGL context
    if (window == nullptr) return -1;
    glfwMakeContextCurrent(window);
    status = CheckErrorGLFW(__LINE__, __FILE__) < 0 ? -1 : status;

    // Set the new clear color using BGR values from cv::Scalar
    glClearColor(color[0], color[1], color[2], 1.0f); // RGBA

    // Clear the window with the new color
    glClear(GL_COLOR_BUFFER_BIT);
    glfwSwapBuffers(window);

    // Wait for the duration of the flash
    std::this_thread::sleep_for(std::chrono::milliseconds(duration));

    // Reset the clear color (assuming the default is black)
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    status = CheckErrorGLFW(__LINE__, __FILE__) < 0 ? -1 : status;
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;
    return status;
}

int MazeRenderContext::cleanupContext(bool log_errors) {
    int status = 0;

    // Set the GLFW window as the current OpenGL context
    if (window != nullptr) {
        glfwMakeContextCurrent(window);
        status = CheckErrorGLFW(__LINE__, __FILE__) < 0 ? -1 : status;
    }

    // Delete the shader program
    if (_shaderProgram != 0) {
        glDeleteProgram(_shaderProgram);
        status = CheckErrorOpenGL(__LINE__, __FILE__, "[MazeRenderContext::cleanupContext] glDeleteProgram error") < 0 ? -1 : status;
        if (log_errors)
            ROS_INFO("[MazeRenderContext::cleanupContext] %s", status ? "Failed to delete shader program" : "Shader program deleted successfully");
        _shaderProgram = 0;
        status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;
    }

    // Delete the texture
    if (textureID != 0) {
        glDeleteTextures(1, &textureID);
        status = CheckErrorOpenGL(__LINE__, __FILE__, "[MazeRenderContext::cleanupContext] glDeleteTextures error") < 0 ? -1 : status;
        if (log_errors)
            ROS_INFO("[MazeRenderContext::cleanupContext] %s", status ? "Failed to delete texture" : "Texture deleted successfully");
        textureID = 0;
        status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;
    }

    // Delete the VAO, VBO, and EBO
    if (_vao != 0) {
        glDeleteVertexArrays(1, &_vao);
        status = CheckErrorOpenGL(__LINE__, __FILE__, "[MazeRenderContext::cleanupContext] glDeleteVertexArrays error") < 0 ? -1 : status;
        if (log_errors)
            ROS_INFO("[MazeRenderContext::cleanupContext] %s", status ? "Failed to delete VAO" : "VAO deleted successfully");
        _vao = 0;
        status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;
    }
    if (_vbo != 0) {
        glDeleteBuffers(1, &_vbo);
        status = CheckErrorOpenGL(__LINE__, __FILE__, "[MazeRenderContext::cleanupContext] glDeleteBuffers error") < 0 ? -1 : status;
        if (log_errors)
            ROS_INFO("[MazeRenderContext::cleanupContext] %s", status ? "Failed to delete VBO" : "VBO deleted successfully");
        _vbo = 0;
        status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;
    }
    if (_ebo != 0) {
        glDeleteBuffers(1, &_ebo);
        status = CheckErrorOpenGL(__LINE__, __FILE__, "[MazeRenderContext::cleanupContext] glDeleteBuffers error") < 0 ? -1 : status;
        if (log_errors)
            ROS_INFO("[MazeRenderContext::cleanupContext] %s", status ? "Failed to delete EBO" : "EBO deleted successfully");
        _ebo = 0;
        status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;
    }

    // Destroy the GLFW window
    if (window != nullptr) {
        glfwDestroyWindow(window);
        status = CheckErrorGLFW(__LINE__, __FILE__, "[MazeRenderContext::cleanupContext] glfwDestroyWindow error") < 0 ? -1 : status;
        if (log_errors)
            ROS_INFO("[MazeRenderContext::cleanupContext] %s", status ? "Failed to destroy GLFW window" : "GLFW window destroyed successfully");
        window = nullptr;
        status = CheckErrorGLFW(__LINE__, __FILE__) < 0 ? -1 : status;
    }
    return status ? -1 : 0;
}

void MazeRenderContext::_resetMembers() {
    _shaderProgram = 0;
    _vao = 0;
    _vbo = 0;
    _ebo = 0;
    textureID = 0;
    window = nullptr;
    monitor = nullptr;
    windowInd = -1;
    monitorInd = -1;
    _windowWidthPxl = 800;
    _windowHeightPxl = 600;
    isContextInitialized = false;
    isFullScreen = false;
}

int MazeRenderContext::_checkShaderCompilation(GLuint __shaderProgram, const std::string &shader_type) {
    GLint success;
    glGetShaderiv(__shaderProgram, GL_COMPILE_STATUS, &success);
    if (!success) {
        char info_log[1024];
        glGetShaderInfoLog(__shaderProgram, 1024, nullptr, info_log);
        ROS_ERROR("[MazeRenderContext::_checkShaderCompilation] %s shader compilation error: %s", shader_type.c_str(), info_log);
        return -1;
    }
    return 0;
}

int MazeRenderContext::_checkProgramLinking(GLuint __shaderProgram) {
    GLint success;
    glGetProgramiv(__shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        char info_log[1024];
        glGetProgramInfoLog(__shaderProgram, 1024, nullptr, info_log);
        ROS_ERROR("[MazeRenderContext::_checkProgramLinking] Program linking error: %s", info_log);
        return -1;
    }
    return 0;
}

void MazeRenderContext::_setMonitor(int mon_ind) {
    // Validate the inputs
    if (mon_ind >= N_MONITORS) {
        ROS_ERROR("[MazeRenderContext::_setMonitor] Monitor Index[%d] Exceeds Available Monitors[%d]", mon_ind, N_MONITORS);
        return;
    }

    // Check that the monitor pointer is valid
    monitor = MONITORS[mon_ind];

    // Set the references to the class intance monitor id and index
    monitorInd = mon_ind;
}

int MazeRenderContext::_testCallbacks() {
    // Check that the window pointer is valid
    if (window == nullptr) return -1;

    ROS_INFO("============== START: CALLBACK DEBUGGIN ==============");
    // Trigger buffer
    glfwSetWindowSize(window, 800, 600); // Change the size to something different

    // Insert a debug message manually
    glDebugMessageInsert(GL_DEBUG_SOURCE_APPLICATION, GL_DEBUG_TYPE_ERROR, 0, GL_DEBUG_SEVERITY_HIGH, -1, "Test debug message");
    CheckErrorOpenGL(__LINE__, __FILE__, "[main] Error Flagged Following glDebugMessageInsert()");

    // Try to create a window with invalid arguments
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 99); // Set an invalid version to trigger an error
    GLFWwindow *new_win = glfwCreateWindow(640, 480, "Bad Window!", nullptr, nullptr);
    ROS_INFO("============== END: CALLBACK DEBUGGIN ==============");

    return 0;
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
    : circPosition(cv::Point2f(0.0f, 0.0f)),
      cirRadius(1.0f),
      circColor(cv::Scalar(1.0, 1.0, 1.0)),
      circSegments(32),
      circHomMatNDC(cv::Mat::eye(3, 3, CV_64F)) {
    // Define instance count and itterate static _CircCnt
    circID = _CircCnt++;

    // Initialize the transformation matrix as an identity matrix
    _transformationMatrix = cv::Mat::eye(4, 4, CV_32F);
}

CircleRenderer::~CircleRenderer() {
    // Cleanup instance-level OpenGL resources
    if (_vao != 0) {
        glDeleteVertexArrays(1, &_vao);
        _vao = 0;
    }
    if (_vbo != 0) {
        glDeleteBuffers(1, &_vbo);
        _vbo = 0;
    }
}

int CircleRenderer::initializeCircleObject(cv::Point2f _circPosition, float _cirRadius, cv::Scalar _circColor,
                                           unsigned int _circSegments, cv::Mat _circHomMatNDC) {
    int status = 0;

    // Set variables
    circPosition = _circPosition;
    cirRadius = _cirRadius;
    circColor = _circColor;
    circSegments = _circSegments;
    circHomMatNDC = _circHomMatNDC;

    // Run initial vertex computation
    _computeVertices(circVertices);

    // Generate a new Vertex Array Object (VAO) and store the ID
    glGenVertexArrays(1, &_vao);
    // Generate a new Vertex Buffer Object (VBO) and store the ID
    glGenBuffers(1, &_vbo);
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Bind the VAO to set it up
    glBindVertexArray(_vao);
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Bind the VBO to the GL_ARRAY_BUFFER target
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);
    // Copy vertex data into the buffer's memory (GL_DYNAMIC_DRAW hints that the data might change often)
    glBufferData(GL_ARRAY_BUFFER, circVertices.size() * sizeof(float), circVertices.data(), GL_DYNAMIC_DRAW);
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Define an array of generic vertex attribute data. Arguments:
    // 0: the index of the vertex attribute to be modified.
    // 2: the number of components per generic vertex attribute. Since we're using 2D points, this is 2.
    // GL_FLOAT: the data type of each component in the array.
    // GL_FALSE: specifies whether fixed-point data values should be normalized or not.
    // 2 * sizeof(float): the byte offset between consecutive vertex attributes.
    // (void*)0: offset of the first component of the first generic vertex attribute in the array.
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Enable the vertex attribute array for the VAO at index 0
    glEnableVertexAttribArray(0);
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Unbind the VAO to prevent further modification.
    glBindVertexArray(0);
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    return status;
}

int CircleRenderer::CompileAndLinkCircleShaders(float __AspectRatioUniform) {
    int status = 0;

    // Set the aspect ratio
    _AspectRatioUniform = __AspectRatioUniform;

    // Compile vertex shader
    GLuint temp_vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(temp_vertex_shader, 1, &_circVertexShaderSource, nullptr);
    glCompileShader(temp_vertex_shader);
    if (_CheckShaderCompilation(temp_vertex_shader, "VERTEX") < 0) {
        glDeleteShader(temp_vertex_shader);
        ROS_ERROR("[CircleRenderer::CompileAndLinkCircleShaders] Vertex shader compilation failed.");
        return -1;
    }
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Compile fragment shader
    GLuint temp_fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(temp_fragment_shader, 1, &_circFragmentShaderSource, nullptr);
    glCompileShader(temp_fragment_shader);
    if (_CheckShaderCompilation(temp_fragment_shader, "FRAGMENT") < 0) {
        glDeleteShader(temp_vertex_shader);
        glDeleteShader(temp_fragment_shader);
        ROS_ERROR("[CircleRenderer::CompileAndLinkCircleShaders] Fragment shader compilation failed.");
        return -1;
    }
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Link shaders into a program
    _ShaderProgram = glCreateProgram();
    glAttachShader(_ShaderProgram, temp_vertex_shader);
    glAttachShader(_ShaderProgram, temp_fragment_shader);
    glLinkProgram(_ShaderProgram);
    if (_CheckProgramLinking(_ShaderProgram) < 0) {
        glDeleteShader(temp_vertex_shader);
        glDeleteShader(temp_fragment_shader);
        glDeleteProgram(_ShaderProgram);
        ROS_ERROR("[CircleRenderer::CompileAndLinkCircleShaders] Shader program linking failed.");
        return -1;
    }
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Cleanup: detach and delete shaders
    glDetachShader(_ShaderProgram, temp_vertex_shader);
    glDeleteShader(temp_vertex_shader);
    glDetachShader(_ShaderProgram, temp_fragment_shader);
    glDeleteShader(temp_fragment_shader);
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Get uniform locations
    _ColorLocation = glGetUniformLocation(_ShaderProgram, "color");
    _TransformLocation = glGetUniformLocation(_ShaderProgram, "transform");
    _AspectRatioLocation = glGetUniformLocation(_ShaderProgram, "aspectRatio");
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Check for errors in getting uniforms
    if (_ColorLocation == -1 || _TransformLocation == -1 || _AspectRatioLocation == -1) {
        ROS_ERROR("[CircleRenderer::CompileAndLinkCircleShaders] Error getting uniform locations.");
        return -1;
    }

    return status;
}

int CircleRenderer::SetupShader() {
    int status = 0;

    // Use the shader program
    glUseProgram(_ShaderProgram);
    // status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Set the aspect ratio uniform
    glUniform1f(_AspectRatioLocation, _AspectRatioUniform);
    // status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    return status;
}

int CircleRenderer::UnsetShader() {
    // Unset the shader program
    glUseProgram(0);
    return CheckErrorOpenGL(__LINE__, __FILE__);
}

void CircleRenderer::setPosition(cv::Point2f _circPosition) {
    // Modify the y position based on the aspect ratio
    _circPosition.y /= _AspectRatioUniform;
    circPosition = _circPosition;
}

void CircleRenderer::setRadius(float _cirRadius){
    cirRadius = _cirRadius;
}

void CircleRenderer::setColor(cv::Scalar col) {
    circColor = col;
}

int CircleRenderer::updateCircleObject(bool do_coord_warp) {
    int status = 0;

    // Generate the new vertices based on the current position, radius, and circSegments
    _computeVertices(circVertices);

    // Convert the circle vertices to NDC space
    if (do_coord_warp)
        _convertToNDC(circVertices);

    // Bind the VBO to the GL_ARRAY_BUFFER target
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Update the VBO's data with the new vertices. This call will reallocate the buffer if necessary
    // or simply update the data store's contents if the buffer is large enough.
    glBufferData(GL_ARRAY_BUFFER, circVertices.size() * sizeof(float), circVertices.data(), GL_DYNAMIC_DRAW);
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Unbind the buffer
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    return status;
}

int CircleRenderer::draw() {
    int status = 0;

    // Set color
    glUniform4f(_ColorLocation, circColor[0], circColor[1], circColor[2], 1.0f);
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Use the updated transformation matrix instead of an identity matrix
    auto transformArray = _cvMatToGlArray(_transformationMatrix);
    glUniformMatrix4fv(_TransformLocation, 1, GL_FALSE, transformArray.data());
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Bind the VAO and draw the circle using GL_TRIANGLE_FAN
    glBindVertexArray(_vao);
    glDrawArrays(GL_TRIANGLE_FAN, 0, static_cast<GLsizei>(circVertices.size() / 2));
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    // Unbind the VAO to leave a clean state
    glBindVertexArray(0);
    status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;

    return status;
}

int CircleRenderer::CleanupClassResources() {
    int status = 0;

    // Delete the shader program
    if (_ShaderProgram != 0) {
        glDeleteProgram(_ShaderProgram);
        _ShaderProgram = 0;
        status = CheckErrorOpenGL(__LINE__, __FILE__) < 0 ? -1 : status;
    }

    // Reset the static uniform locations to -1 indicating they are no longer valid
    _ColorLocation = -1;
    _TransformLocation = -1;
    _AspectRatioLocation = -1;

    // Reset the window aspect ratio uniform
    _AspectRatioUniform = 0.0f;

    return status;
}

int CircleRenderer::_CheckShaderCompilation(GLuint ___ShaderProgram, const std::string &shader_type) {
    GLint success;
    glGetShaderiv(___ShaderProgram, GL_COMPILE_STATUS, &success);
    if (!success) {
        // Get and log the error message
        char info_log[512];
        glGetShaderInfoLog(___ShaderProgram, 512, nullptr, info_log);
        ROS_ERROR("[CircleRenderer::CircleRenderer] %s shader compilation error: %s", shader_type.c_str(), info_log);
        return -1;
    }
    return 0;
}

int CircleRenderer::_CheckProgramLinking(GLuint ___ShaderProgram) {
    GLint success;
    glGetProgramiv(___ShaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        // Get and log the error message
        char info_log[512];
        glGetProgramInfoLog(___ShaderProgram, 512, nullptr, info_log);
        ROS_ERROR("[CircleRenderer::CircleRenderer] Program linking error: %s", info_log);
        return -1;
    }
    return 0;
}

std::array<float, 16> CircleRenderer::_cvMatToGlArray(const cv::Mat &out_transformationMatrix) {
    assert(out_transformationMatrix.cols == 4 && out_transformationMatrix.rows == 4 && out_transformationMatrix.type() == CV_32F);
    std::array<float, 16> gl_array;
    std::copy(out_transformationMatrix.begin<float>(), out_transformationMatrix.end<float>(), gl_array.begin());
    return gl_array;
}

void CircleRenderer::_computeVertices(std::vector<float> &out_circVertices) {
    // Clear the vertex vector to prepare for new vertices
    out_circVertices.clear();

    // Loop to generate vertices for the circle
    for (unsigned int i = 0; i <= circSegments; ++i) {
        // Calculate the angle for the current segment
        float angle = 2.0f * std::acos(-1.0) * i / circSegments;
        // Determine the x circPosition of the vertex on the circle
        float baseX = circPosition.x + (cirRadius * std::cos(angle));
        // Determine the y circPosition of the vertex on the circle
        float baseY = circPosition.y + (cirRadius * std::sin(angle));

        // Create a 4x1 matrix (homogeneous coordinates) for the vertex
        cv::Mat vertex = (cv::Mat_<float>(4, 1) << baseX, baseY, 0, 1);
        // Apply the transformation matrix to the vertex
        cv::Mat transformedVertex = _transformationMatrix * vertex;
        // Extract the transformed x coordinate and add to the vertices list
        out_circVertices.push_back(transformedVertex.at<float>(0, 0));
        // Extract the transformed y coordinate and add to the vertices list
        out_circVertices.push_back(transformedVertex.at<float>(1, 0));
    }
}

void CircleRenderer::_convertToNDC(std::vector<float> &out_circVertices) {
    // Convert to vector of cv::Point2f
    std::vector<cv::Point2f> circPoints;
    for (size_t i = 0; i < out_circVertices.size(); i += 2)
        circPoints.push_back(cv::Point2f(out_circVertices[i], out_circVertices[i + 1]));

    // Apply the homography transformation
    std::vector<cv::Point2f> transformedPoints;
    cv::perspectiveTransform(circPoints, transformedPoints, circHomMatNDC);

    // Store the transformed points back into out_circVertices
    out_circVertices.clear();
    for (const auto &pt : transformedPoints) {
        out_circVertices.push_back(pt.x);
        out_circVertices.push_back(pt.y);
    }
}

// ================================================== FUNCTIONS ==================================================

int64_t dbTraceCalls(bool do_reset, int line, const char *file_path) {
    if (!GLB_DO_VERBOSE_DEBUG)
        return 0;

    static ros::Time start_time = ros::Time::now();
    static int cnt_calls = 0;
    static int line_start = 0;
    static std::string file_name_start = "";

    // Function to extract file name from file path
    auto extractFileName = [](const char *path) -> std::string {
        if (path == nullptr) return std::string("NULL");

        const char *file_name = strrchr(path, '\\'); // Windows file path separator
        if (!file_name) file_name = strrchr(path, '/'); // In case of Unix-style paths
        if (file_name) return std::string(file_name + 1); // Skip past the last separator
        return std::string(path); // No separator found, return the whole string
    };

    // Print elapsed time
    cnt_calls++;
    ros::Duration elapsed_time = ros::Time::now() - start_time;
    std::string file_name_current = extractFileName(file_path);
    if (line_start == 0 || line == 0)
        ROS_INFO("[dbTraceCalls] Call[%d]: Elapsed Time: %f milliseconds", cnt_calls, elapsed_time.toNSec() / 1e6);
    else {
        ROS_INFO("[dbTraceCalls] Call[%d]: Elapsed Time from %s[%d] to %s[%d] is %0.2f milliseconds",
                 cnt_calls, file_name_start.c_str(), line_start, file_name_current.c_str(), line,
                 elapsed_time.toNSec() / 1e6);
    }

    // Reset start time
    if (do_reset) {
        cnt_calls = 0;
        line_start = line;
        file_name_start = extractFileName(file_path);
        start_time = ros::Time::now();
    }
    // Return elapsed time in ms
    return static_cast<int64_t>(elapsed_time.toNSec() / 1e6);
}

/**
 * @brief Manages timed delays within a loop.
 *
 * @param dt_wait Latency in milliseconds for the delay; used only on the initial call.
 *
 * @details
 * Maintains a static timestamp to enforce a delay relative to `ros::Time::now()`.
 * Returns false if the delay has not elapsed, true otherwise, resetting the timer.
 *
 * @return True if the delay has elapsed, otherwise false.
 *
 * @example
 * dbDelayRun(500); // Delay for 500 ms
 * while (!dbDelayRun()){}; // Loop till delay has elapsed
 */
bool dbDelayRun(int dt_wait){
    if (!GLB_DO_VERBOSE_DEBUG) return false;

    // Initialize static variable
    static ros::Time ts_wait = ros::Time(0);

    // Get the current time
    ros::Time now = ros::Time::now();

    // Set the next wait timestamp
    if (dt_wait > 0) ts_wait = now + ros::Duration(0, dt_wait * 1000000); // Convert ms to ns

    // Check if ts_wait is set to zero or the current time is past ts_wait
    if (now > ts_wait) return true;

    return false;
}

void dbWaitForInput() {
    ROS_INFO("Paused for Debugging: Press Enter to Continue...");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

void dbLogQuadVertices(const std::vector<cv::Point2f> &quad_vertices) {
    if (!GLB_DO_VERBOSE_DEBUG) return;

    if (quad_vertices.size() != 4) {
        ROS_ERROR("[dbLogQuadVertices] Invalid number of vertices. Expected 4.");
        return;
    }

    ROS_INFO("         Quad Vertices             ");
    ROS_INFO("===================================");
    ROS_INFO("    |     Left     |     Right    |");

    if (std::any_of(quad_vertices.begin(), quad_vertices.end(), [](const cv::Point2f &point)
                    { return point.x > 10.0f || point.y > 10.0f; })) {
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
    } else {
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

void dbLogCtrlPointCoordinates(const std::array<std::array<cv::Point2f, 4>, 4> &r_ctrl_pnt_coords) {
    if (!GLB_DO_VERBOSE_DEBUG) return;

    ROS_INFO("        Control Point Coordinates        ");
    ROS_INFO("=========================================");
    ROS_INFO("        |      Left     |     Right     |");

    // Loop through each control point
    for (int cp = 0; cp < 4; ++cp) {
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

void dbLogWallVerticesCoordinates(const std::array<std::array<std::array<cv::Point2f, 4>, GLB_MAZE_SIZE>, GLB_MAZE_SIZE> &_WALL_WARP_COORDS) {
    if (!GLB_DO_VERBOSE_DEBUG) return;

    ROS_INFO("                                       Warped Wall Coordinates                                               ");
    ROS_INFO("=============================================================================================================");
    ROS_INFO("-------------------------------------------------------------------------------------------------------------");

    // Loop through each row and column in the maze
    for (int row = 0; row < GLB_MAZE_SIZE; ++row) {
        ROS_INFO("        ||   X   ,   Y   |   X   ,   Y   ||   X   ,   Y   |   X   ,   Y   ||   X   ,   Y   |   X   ,   Y   ||");
        ROS_INFO("-------------------------------------------------------------------------------------------------------------");

        // Buffer to hold the formatted string for each row
        char buffer[256];

        // Format and print the Top row coordinates
        snprintf(buffer, sizeof(buffer), "(%d) Top ||", row);
        for (int col = 0; col < GLB_MAZE_SIZE; ++col) {
            // Fetch the quad vertices for the current [row][col]
            auto &quad_vertices = _WALL_WARP_COORDS[row][col];
            snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), " %+4.2f , %+4.2f | %+4.2f , %+4.2f ||",
                     quad_vertices[0].x, quad_vertices[0].y, quad_vertices[1].x, quad_vertices[1].y);
        }
        ROS_INFO("%s", buffer);

        // Format and print the Bottom row coordinates
        snprintf(buffer, sizeof(buffer), "(%d) Btm ||", row);
        for (int col = 0; col < GLB_MAZE_SIZE; ++col) {
            // Fetch the quad vertices for the current [row][col]
            auto &quad = _WALL_WARP_COORDS[row][col];
            snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), " %+4.2f , %+4.2f | %+4.2f , %+4.2f ||",
                     quad[3].x, quad[3].y, quad[2].x, quad[2].y);
        }
        ROS_INFO("%s", buffer);
        ROS_INFO("-------------------------------------------------------------------------------------------------------------");
    }
}

void dbLogHomMat(const cv::Mat &r_HMAT) {
    if (!GLB_DO_VERBOSE_DEBUG) return;

    // Check if the input matrix is 3x3
    if (r_HMAT.rows != 3 || r_HMAT.cols != 3) {
        ROS_WARN("The input matrix is not 3x3. Cannot print.");
        return;
    }

    ROS_INFO("         Homography Matrix        ");
    ROS_INFO("==================================");
    ROS_INFO("          |  C0   |  C1   |  C2   |");
    ROS_INFO("----------------------------------");

    for (int i = 0; i < 3; ++i) {
        ROS_INFO("R%d        | %+5.2f | %+5.2f | %+5.2f |", i,
                 r_HMAT.at<double>(i, 0),
                 r_HMAT.at<double>(i, 1),
                 r_HMAT.at<double>(i, 2));
    }
    ROS_INFO("==================================");
}

void dbLogProjWallImageCfg4D(const ProjWallConfigIndices4D &wallImageConfig) {
    if (!GLB_DO_VERBOSE_DEBUG) return;

    ROS_INFO("                                Projector Wall Image Configuration                                       ");
    ROS_INFO("=======================================================");
    ROS_INFO("       ||  Column [0]  |  Column [0]  |  Column [2]  ||");
    ROS_INFO("-------------------------------------------------------");

    // Loop through each projector, row, and column
    for (int proj = 0; proj < 4; ++proj) {
        ROS_INFO("Projector %d", proj);
        ROS_INFO("-------------------------------------------------------");

        for (int row = 0; row < 3; ++row) {
            // Buffer to hold the formatted string for each row
            char buffer[512]; // Adjust size as necessary for your specific logging needs
            snprintf(buffer, sizeof(buffer), "(%d) Row ||", row);

            for (int col = 0; col < 3; ++col) {
                auto &cell = wallImageConfig[proj][row][col];
                snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), " %3d,%3d,%3d |",
                         cell[0], cell[1], cell[2]);
            }

            // Print the formatted string for the current row
            ROS_INFO("%s", buffer);
        }
        ROS_INFO("-------------------------------------------------------");
    }
}

void dbDispImgMat(const cv::Mat &img_mat) {
    cv::namedWindow("Debug display image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Debug display image", img_mat);
    cv::waitKey(0);
    cv::destroyWindow("Debug display image");
}

int promptForProjectorNumber() {
    std::string input;
    int proj_ind = -1;

    // Loop until a valid input is received
    while (true) {
        std::cout << "[promptForProjectorNumber] Enter the actual number/index of the projector being calibrated [0,1,2,3], or press 'q' to quit: ";
        std::cin >> input;

        // Check for quit command
        if (input == "q" || input == "Q") {
            std::cout << "[promptForProjectorNumber] Operation cancelled by the user." << std::endl;
            return -1;
        }

        // Check if the input is a single digit and is a number
        if (input.length() == 1 && std::isdigit(input[0])) {
            proj_ind = std::stoi(input);

            // Check if the input is between 0 and 3
            if (proj_ind < N_PROJ) {
                std::cout << "[promptForProjectorNumber] Projector number/index set to " + std::to_string(proj_ind) + "." << std::endl;
                return std::stoi(input);
            }
            else std::cout << "[promptForProjectorNumber] Invalid input: Please enter a number between 0 and 3 or 'q' to quit." << std::endl;
        }
        else std::cout << "[promptForProjectorNumber] Invalid input: Please enter a single numeric digit or 'q' to quit." << std::endl;

        // Clear the input stream in case of invalid input (e.g., if more than one character was entered)
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}

void xmlFrmtFileStringsControlPoints(int proj_ind, std::string &out_path) {
    out_path = CONFIG_DIR_PATH + "/cp_p" + std::to_string(proj_ind) + ".xml"; // Format the output tag
}

void xmlFrmtFileStringsHmat(int proj_ind, std::string &out_path) {
    out_path = CONFIG_DIR_PATH + "/hmats_p" + std::to_string(proj_ind) + ".xml"; // Format the output tag
}

void xmlFrmtFileStringsVertices(std::string &out_path) {
    out_path = CONFIG_DIR_PATH + "/maze_vertices.xml"; // Format the output tag
}

int xmlSaveControlPoints(const std::array<cv::Point2f, 4> &CP_ARR, int proj_ind, CalibrationMode _CAL_MODE, int cp_ind) {
    // Define file path and calibration mode string
    std::string file_path;
    xmlFrmtFileStringsControlPoints(proj_ind, file_path); // Assume you have a similar function to set file path
    std::string cal_mode_str = CAL_MODE_STR_VEC[_CAL_MODE];

    // Attempt to load the XML file
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(file_path.c_str());

    // Check if the file exists and has a proper Root node
    pugi::xml_node root_node = doc.child("Root");
    if (!result || !root_node) root_node = doc.append_child("Root");

    // Look for the calibration node with the matching mode attribute
    pugi::xml_node calibration_node;
    for (pugi::xml_node node = root_node.child("calibration");
         node; node = node.next_sibling("calibration")) {
        if (node.attribute("mode").value() == cal_mode_str) {
            calibration_node = node;
            break;
        }
    }

    if (!calibration_node) {
        calibration_node = root_node.append_child("calibration");
        calibration_node.append_attribute("mode") = cal_mode_str.c_str();
    }

    // Search for an existing CP node with the same cp_ind
    pugi::xml_node cp_node;
    for (pugi::xml_node node = calibration_node.child("CP");
         node; node = node.next_sibling("CP")) {
        if (node.attribute("cp_index").as_int() == cp_ind) {
            cp_node = node;
            break;
        }
    }

    // If a CP node with the index is not found, add it
    if (!cp_node) {
        cp_node = calibration_node.append_child("CP");
        cp_node.append_attribute("cp_index") = cp_ind;
    } else {
        // Clear existing data to replace with new points
        calibration_node.remove_child(cp_node);
        cp_node = calibration_node.append_child("CP");
        cp_node.append_attribute("cp_index") = cp_ind;
    }

    // Add control points data to the CP node
    for (int i = 0; i < CP_ARR.size(); ++i) {
        const auto &point = CP_ARR[i];
        pugi::xml_node point_node = cp_node.append_child("Point");
        point_node.append_attribute("id") = i; // Unique ID for each point
        point_node.append_attribute("x") = point.x;
        point_node.append_attribute("y") = point.y;
    }

    // Save the document
    if (!doc.save_file(file_path.c_str())) {
        ROS_ERROR("[xmlSaveControlPoints] Failed to save control points to XML File[%s]", file_path.c_str());
        return -1;
    }
    return 0;
}

int xmlLoadControlPoints(int proj_ind, CalibrationMode _CAL_MODE, int cp_ind, std::array<cv::Point2f, 4> &out_CP_ARR) {
    // Define file path and calibration mode string
    std::string file_path;
    xmlFrmtFileStringsControlPoints(proj_ind, file_path); // Assume you have a similar function to set file path
    std::string cal_mode_str = CAL_MODE_STR_VEC[_CAL_MODE];

    // Attempt to load the XML file
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(file_path.c_str());

    // Check if the file was loaded successfully
    if (!result) {
        ROS_ERROR("[xmlLoadControlPoints] Failed to load control points from XML File[%s]", file_path.c_str());
        return -1;
    }

    // Look for the calibration node with the matching mode attribute
    pugi::xml_node calibration_node;
    for (pugi::xml_node node = doc.child("Root").child("calibration");
         node; node = node.next_sibling("calibration")) {
        if (node.attribute("mode").value() == cal_mode_str) {
            calibration_node = node;
            break;
        }
    }

    if (!calibration_node) {
        ROS_ERROR("[xmlLoadControlPoints] No calibration node[%s] found in XML File[%s]", cal_mode_str.c_str(), file_path.c_str());
        return -1;
    }

    // Search for a CP node with the same cp_ind
    pugi::xml_node cp_node;
    for (pugi::xml_node node = calibration_node.child("CP");
         node; node = node.next_sibling("CP")) {
        if (node.attribute("cp_index").as_int() == cp_ind) {
            cp_node = node;
            break;
        }
    }

    if (!cp_node) {
        ROS_ERROR("[xmlLoadControlPoints] No CP node with cp_index[%d] found in calibration mode[%s] for XML File[%s]",
                  cp_ind, cal_mode_str.c_str(), file_path.c_str());
        return -1;
    }

    // Load the control points data
    int i = 0;
    for (pugi::xml_node point_node = cp_node.child("Point"); point_node && i < 4; point_node = point_node.next_sibling("Point"), ++i) {
        out_CP_ARR[i].x = point_node.attribute("x").as_float();
        out_CP_ARR[i].y = point_node.attribute("y").as_float();
    }

    if (i != 4) { // Ensure that exactly 4 control points are loaded
        ROS_ERROR("[xmlLoadControlPoints] Incorrect number of control points in CP. Expected 4, got %d", i);
        return -1;
    }
    return 0;
}

int xmlSaveHMAT(const cv::Mat &_H, int proj_ind, CalibrationMode _CAL_MODE, int grid_row, int grid_col) {
    // Get the full file path and attribute string for the given calibration mode
    std::string file_path;
    xmlFrmtFileStringsHmat(proj_ind, file_path);
    std::string cal_mode_str = CAL_MODE_STR_VEC[_CAL_MODE];

    // Attempt to load the XML file
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(file_path.c_str());

    // Check if the file exists and has a proper Root node
    pugi::xml_node root_node = doc.child("Root");
    if (!result || !root_node) {
        // If not, create a new Root node
        root_node = doc.append_child("Root");
    }

    // Look for the calibration node with the matching mode attribute
    pugi::xml_node calibration_node;
    for (pugi::xml_node node = root_node.child("calibration");
         node; node = node.next_sibling("calibration")) {
        if (node.attribute("mode").value() == cal_mode_str) {
            calibration_node = node;
            break;
        }
    }

    // If a calibration node with the mode is not found, add it
    if (!calibration_node) {
        calibration_node = root_node.append_child("calibration");
        calibration_node.append_attribute("mode") = cal_mode_str.c_str();
    }

    // Search for an existing HMAT with the same grid_row and grid_col
    for (pugi::xml_node hmat_node = calibration_node.child("HMAT");
         hmat_node; hmat_node = hmat_node.next_sibling("HMAT")) {
        if (hmat_node.attribute("grid_row").as_int() == grid_row &&
            hmat_node.attribute("grid_col").as_int() == grid_col) {
            // If found, remove it to replace with updated matrix
            calibration_node.remove_child(hmat_node);
            break;
        }
    }

    // Create a new HMAT node with the updated matrix
    pugi::xml_node new_hmat_node = calibration_node.append_child("HMAT");
    new_hmat_node.append_attribute("grid_row") = grid_row;
    new_hmat_node.append_attribute("grid_col") = grid_col;

    // Add the matrix data to the HMAT node
    for (int r = 0; r < _H.rows; ++r) {
        pugi::xml_node row_node = new_hmat_node.append_child("row");
        for (int c = 0; c < _H.cols; ++c) {
            pugi::xml_node cell_node = row_node.append_child("cell");
            cell_node.append_child(pugi::node_pcdata).set_value(std::to_string(_H.at<double>(r, c)).c_str());
        }
    }

    // Save the document to the specified file path
    if (!doc.save_file(file_path.c_str())) {
        ROS_ERROR("[xmlSaveHMAT] Failed to save homography matrix to XML File[%s]", file_path.c_str());
        return -1;
    }
    return 0;
}

int xmlLoadHMAT(int proj_ind, CalibrationMode _CAL_MODE, int grid_row, int grid_col, cv::Mat &out_H) {
    // Get the full file path and attribute string for the given calibration mode
    std::string file_path;
    xmlFrmtFileStringsHmat(proj_ind, file_path);
    std::string cal_mode_str = CAL_MODE_STR_VEC[_CAL_MODE];

    // Attempt to load the XML file
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(file_path.c_str());

    // Check if the file was loaded successfully
    if (!result) {
        ROS_ERROR("[xmlLoadHMAT] Failed to load homography matrix from XML File[%s]", file_path.c_str());
        return -1;
    }

    // Look for the calibration node with the matching mode attribute
    pugi::xml_node calibration_node;
    for (pugi::xml_node node = doc.child("Root").child("calibration");
         node; node = node.next_sibling("calibration")) {
        if (node.attribute("mode").value() == cal_mode_str) {
            calibration_node = node;
            break;
        }
    }

    if (!calibration_node) {
        ROS_ERROR("[xmlLoadHMAT] No calibration node[%s] found in XML File[%s]", cal_mode_str.c_str(), file_path.c_str());
        return -1;
    }

    // Search for an HMAT node with the same grid_row and grid_col
    pugi::xml_node hmat_node;
    for (pugi::xml_node node = calibration_node.child("HMAT");
         node; node = node.next_sibling("HMAT")) {
        if (node.attribute("grid_row").as_int() == grid_row &&
            node.attribute("grid_col").as_int() == grid_col) {
            hmat_node = node;
            break;
        }
    }

    if (!hmat_node) {
        ROS_ERROR("[xmlLoadHMAT] No HMAT with grid_row[%d] and grid_col[%d] found in calibration mode[%s] for XML File[%s]",
                  grid_row, grid_col, cal_mode_str.c_str(), file_path.c_str());
        return -1;
    }

    // Load the matrix data
    std::vector<double> matrix_data;
    for (pugi::xml_node row_node = hmat_node.child("row"); row_node; row_node = row_node.next_sibling("row")) {
        for (pugi::xml_node cell_node = row_node.child("cell"); cell_node; cell_node = cell_node.next_sibling("cell")) {
            matrix_data.push_back(std::stod(cell_node.child_value()));
        }
    }

    // Ensure the matrix data is the right size for a homography matrix
    if (matrix_data.size() != 9) {
        ROS_ERROR("[xmlLoadHMAT] Incorrect number of elements in HMAT. Expected 9, got %zu", matrix_data.size());
        return -1;
    }

    // Assign the matrix data to the output matrix
    out_H = cv::Mat(3, 3, CV_64F, &matrix_data[0]).clone(); // Clone to ensure continuous memory

    return 0;
}

int xmlSaveVertices(const std::vector<cv::Point2f> &quad_vertices_ndc, int proj_ind) {
    // Define file path
    std::string file_path;
    xmlFrmtFileStringsVertices(file_path);

    // Load or create an XML document
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(file_path.c_str());
    pugi::xml_node root_node = doc.child("Root");

    if (!result || !root_node) root_node = doc.append_child("Root");

    // Search for an existing monitor node with the same index
    pugi::xml_node monitor_node;
    for (pugi::xml_node node = root_node.child("projector"); node; node = node.next_sibling("projector")) {
        if (node.attribute("index").as_int() == proj_ind) {
            monitor_node = node;
            break;
        }
    }

    // If a monitor node with the index is not found, add it
    if (!monitor_node) {
        monitor_node = root_node.append_child("projector");
        monitor_node.append_attribute("index") = proj_ind;
    } else {
        // Clear existing vertices if the monitor node is already present
        while (monitor_node.first_child()) {
            monitor_node.remove_child(monitor_node.first_child());
        }
    }

    // Add vertices to the monitor node
    for (const auto &vertex : quad_vertices_ndc) {
        pugi::xml_node vertex_node = monitor_node.append_child("vertex");
        vertex_node.append_child("coord").text().set(vertex.x);
        vertex_node.append_child("coord").text().set(vertex.y);
    }

    // Save the document to the specified file path
    if (!doc.save_file(file_path.c_str())) {
        ROS_ERROR("[xmlSaveVertices] Failed to save vertices to XML File[%s]", file_path.c_str());
        return -1;
    }
    return 0;
}

int xmlLoadVertices(int proj_ind, std::vector<cv::Point2f> &out_quad_vertices_ndc) {
    // Define file path
    std::string file_path;
    xmlFrmtFileStringsVertices(file_path);

    // Load the XML document
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(file_path.c_str());

    if (!result) {
        ROS_ERROR("[xmlLoadHMAT] Failed to load vertices from XML File[%s]", file_path.c_str());
        return -1;
    }

    // Search for the monitor node with the same index
    pugi::xml_node monitor_node;
    for (pugi::xml_node node = doc.child("Root").child("projector"); node; node = node.next_sibling("projector")) {
        if (node.attribute("index").as_int() == proj_ind) {
            monitor_node = node;
            break;
        }
    }

    if (!monitor_node) {
        ROS_ERROR("[xmlLoadHMAT] No monitor node[%d] found in XML File[%s]", proj_ind, file_path.c_str());
        return -1;
    }

    // Clear existing data in the output vector
    out_quad_vertices_ndc.clear();

    // Load the vertices from the monitor node
    for (pugi::xml_node vertex_node = monitor_node.child("vertex"); vertex_node; vertex_node = vertex_node.next_sibling("vertex")) {
        cv::Point2f vertex;
        pugi::xml_node coord_node = vertex_node.child("coord");
        if (coord_node) {
            vertex.x = coord_node.text().as_float();
            coord_node = coord_node.next_sibling("coord");
        }

        if (coord_node)
            vertex.y = coord_node.text().as_float();
        else {
            ROS_ERROR("[xmlLoadHMAT] Incomplete vertex data in XML File[%s]", file_path.c_str());
            return -1; // Handle error: incomplete vertex data
        }
        out_quad_vertices_ndc.push_back(vertex);
    }

    // Check if the vector has exactly four vertices
    if (out_quad_vertices_ndc.size() != 4) {
        ROS_ERROR("[xmlLoadHMAT] Loaded vector is wrong size for XML: Expected[4] Actual[%d] Projector[%d] File[%s]",
                  out_quad_vertices_ndc.size(), proj_ind, file_path.c_str());
        return -1;
    }
    return 0;
}

int checkHMAT(const cv::Mat &_H) {
    // Check if the input matrix is 3x3
    if (_H.empty() || _H.rows != 3 || _H.cols != 3) {
        ROS_ERROR("[updateTexture] Homography matrix size error: Size[%d][%d]", _H.rows, _H.cols);
        return -1;
    }

    // Check for a non-singular matrix
    double det = cv::determinant(_H);
    if (fabs(det) < std::numeric_limits<double>::epsilon()) {
        ROS_ERROR("Homography matrix is singular or near-singular with determinant %.6f", det);
        return -1;
    }
    return 0;
}

int checkQuadVertices(const std::vector<cv::Point2f> &quad_vertices) {
    // Check if the input vector has exactly 4 vertices
    if (quad_vertices.size() != 4)
        return -1;

    // Check if any three points are collinear; for a valid quadrilateral, no three points should be collinear
    for (int i = 0; i < 4; ++i) {
        cv::Point2f p1 = quad_vertices[i];
        cv::Point2f p2 = quad_vertices[(i + 1) % 4];
        cv::Point2f p3 = quad_vertices[(i + 2) % 4];
        float area = p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y);
        if (std::abs(area) < 1e-5) return -2; // The points are collinear
    }
    return 0;
}

std::vector<cv::Point2f> quadVertNdc2Pxl(const std::vector<cv::Point2f> &quad_vertices_ndc, int window_width_pxl, int window_height_pxl, bool do_y_invert) {
    std::vector<cv::Point2f> quad_vertices_pxl;
    quad_vertices_pxl.reserve(quad_vertices_ndc.size());

    for (const auto &vertex : quad_vertices_ndc) {
        // Convert x point values from NDC to pixel coordinates
        float x_pixel = (vertex.x + 1.0f) * (window_width_pxl / 2.0f);

        // Convet y points
        float y_pixel;
        if (!do_y_invert)
            y_pixel = (vertex.y + 1.0f) * (window_height_pxl / 2.0f); // Convert y points but keep y-axis direction the same
        else
            y_pixel = (1.0f - vertex.y) * ((float)window_height_pxl / 2.0f); // Invert the y-axis direction

        // Store values
        quad_vertices_pxl.emplace_back(x_pixel, y_pixel);
    }
    return quad_vertices_pxl;
}

std::vector<cv::Point2f> quadVertPxl2Ndc(const std::vector<cv::Point2f> &quad_vertices_pxl, int window_width_pxl, int window_height_pxl, bool do_y_invert) {
    std::vector<cv::Point2f> quad_vertices_ndc;
    quad_vertices_ndc.reserve(quad_vertices_pxl.size());

    for (const auto &vertex : quad_vertices_pxl) {
        // Convert x point values from pixel to NDC coordinates
        float x_ndc = (vertex.x / (window_width_pxl / 2.0f)) - 1.0f;

        // Convert y point values from pixel to NDC coordinates
        // Convet y points
        float y_ndc;
        if (!do_y_invert) y_ndc = (vertex.y / (window_height_pxl / 2.0f)) - 1.0f;
        else y_ndc = 1.0f - (vertex.y / (window_height_pxl / 2.0f));

        // Store values
        quad_vertices_ndc.emplace_back(x_ndc, y_ndc);
    }
    return quad_vertices_ndc;
}

int computeHomographyMatrix(const std::vector<cv::Point2f> &source_vertices,
                            const std::vector<cv::Point2f> &target_vertices,
                            cv::Mat &out_H) {
    int status;

    // Check that the source plane vertices are valid
    status = checkQuadVertices(source_vertices);
    if (status < 0) {
        ROS_ERROR("[computeHomographyMatrix] Source Plane Vertices[%d] Invalid: %s",
                  source_vertices.size(), status == -1 ? "Wrong Number of Vertices" : "Vertices are Collinear");
        return -1;
    }

    // Check that the target plane vertices are valid
    status = checkQuadVertices(target_vertices);
    if (status < 0) {
        ROS_ERROR("[computeHomographyMatrix] Target Plane Vertices[%d] Invalid: %s",
                  target_vertices.size(), status == -1 ? "Wrong Number of Vertices" : "Vertices are Collinear");
        return -1;
    }

    // Use OpenCV's findHomography function to compute the homography matrix
    cv::Mat H = cv::findHomography(source_vertices, target_vertices);

    // Check for valid homography matrix
    if (H.empty()) {
        ROS_ERROR("[computeHomographyMatrix] Failed to Compute Homography Matrix");
        return -1;
    }

    // Update the output matrix
    out_H = H.clone();

    // Return success
    return 0;
}

int loadImgMat(const std::vector<std::string> &img_paths_vec, std::vector<cv::Mat> &out_img_mat_vec) {
    out_img_mat_vec.clear(); // Ensure the output vector is empty before starting
    int img_cnt = 0;
    for (const std::string &img_path : img_paths_vec) {
        // Load image using OpenCV
        cv::Mat img = cv::imread(img_path, cv::IMREAD_UNCHANGED);

        // Check if image is loaded successfully
        if (img.empty()) {
            ROS_ERROR("[loadImgMat] Failed to load image: Path[%s]", img_path.c_str());
            return -1;
        }

        // If the image does not have an alpha channel (less than 4 channels), add one
        if (img.channels() != 4) {
            // Create a new image with an alpha channel (4 channels)
            cv::Mat img_with_alpha;
            cv::Mat alpha = cv::Mat(img.size(), CV_8UC1, cv::Scalar(255)); // Fully opaque alpha channel

            // Merge the original image with the alpha channel
            if (img.channels() == 3) {
                std::vector<cv::Mat> channels;
                cv::split(img, channels);            // Split the BGR channels
                channels.push_back(alpha);           // Add the alpha channel
                cv::merge(channels, img_with_alpha); // Merge channels into a 4-channel image
            }
            else {
                ROS_ERROR("[loadImgMat] Image must include at least 3 channels: Channels[%d] Path[%s]", img.channels(), img_path.c_str());
                return -1;
            }
            img = img_with_alpha; // Use the new image with the alpha channel
        }

        // Determine depth
        std::string depth_str;
        if (img.depth() == CV_8U) depth_str = "CV_8U";
        else if (img.depth() == CV_8S) depth_str = "CV_8S";
        else if (img.depth() == CV_16U) depth_str = "CV_16U";
        else if (img.depth() == CV_16S) depth_str = "CV_16S";
        else if (img.depth() == CV_32S) depth_str = "CV_32S";
        else if (img.depth() == CV_32F) depth_str = "CV_32F";
        else if (img.depth() == CV_64F) depth_str = "CV_64F";
        else depth_str = "Unknown";

        // Store the loaded image in the output vector
        out_img_mat_vec.push_back(img);

        // Log the image information
        if (GLB_DO_VERBOSE_DEBUG)
            ROS_INFO("[loadImgMat] Image[%d of %d] loaded sucesfully: Size[%d,%d] Channels[%d] Depth[%s] Path[%s]",
                     img_cnt, img_paths_vec.size() - 1, img.cols, img.rows, img.channels(), depth_str.c_str(), img_path.c_str());
        img_cnt++;
    }
    return 0;
}

int mergeImgMat(const cv::Mat &mask_img, cv::Mat &out_base_img) {
    cv::Mat mask_img_4ch;
    cv::extractChannel(mask_img, mask_img_4ch, 3); // Extract the alpha channel from the mask image
    cv::add(out_base_img, mask_img, out_base_img, mask_img_4ch); // Add the mask image to the base image using the alpha channel as a mask
    return 0;
}

bool fileExists(const std::string &_file_path) {
    // Check if the file exists at the given path
    std::ifstream file(_file_path);
    return file.good();
}

void blankMazeMap(MazeMap<int> &maze_map) {
    // Initialize the maze map with -1 for all chambers and surfaces
    for (auto &cham: CHAMBERS) {
        for (auto &surf: SURFACES) {
            maze_map[cham][surf] = 0; // 0 indicates blank image assigned
        }
    }
}

void blankProjectionMap(ProjectionMap<int> &projection_map) {
    // Initialize the projection map with -1 for all projectors, rows, columns, and calibration modes
    for (auto &proj: PROJECTORS) {
        for (auto &row: ROWS) {
            for (auto &col: COLS) {
                for (auto &mode: CAL_MODES) {
                    projection_map[proj][row][col][mode] = 0; // 0 indicates blank image assigned
                }
            }
        }
    }
}
