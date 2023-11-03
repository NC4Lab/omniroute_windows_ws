// ########################################################################################################

// ======================================== projection_display.cpp ========================================

// ########################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_display.h"

// ================================================== FUNCTIONS ==================================================

// Error callback for GLFW
void glfwErrorCallback(int error, const char *description)
{
    std::cerr << "GLFW Error: " << error << " - " << description << std::endl;
}

// Function to check and log GL errors
bool checkGLError(const std::string &message)
{
    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR)
    {
        std::cerr << "OpenGL Error: " << err << " - " << message << std::endl;
        return false;
    }
    return true;
}

// Vertex Shader
const char *vertexShaderSource = R"glsl(
#version 330 core
layout (location = 0) in vec2 position;
uniform vec2 offset;
uniform float size;
void main() {
    gl_Position = vec4(position + offset, 0.0, 1.0);
    gl_PointSize = size;
}
)glsl";

// Geometry Shader
const char *geometryShaderSource = R"glsl(
#version 330 core
layout (points) in;
layout (triangle_strip, max_vertices = 64) out;
uniform float size;
out vec3 fragColor;
void main() {
    const int num_vertices = 32; // Number of vertices to approximate the circle
    const float PI = 3.1415926;
    for (int i = 0; i <= num_vertices; ++i) {
        float angle = 2.0 * PI * float(i) / float(num_vertices);
        vec4 offset = vec4(cos(angle), sin(angle), 0.0, 0.0) * size / 2.0;
        gl_Position = gl_in[0].gl_Position + offset;
        EmitVertex();
    }
    EndPrimitive();
}
)glsl";

// Fragment Shader
const char *fragmentShaderSource = R"glsl(
#version 330 core
uniform vec3 color;
out vec4 FragColor;
void main() {
    FragColor = vec4(color, 1.0);
}
)glsl";

class Circle
{
public:
    cv::Point2f position;
    float size;
    cv::Vec3f color;
    GLuint VAO, VBO;

    Circle() : position(0.0f, 0.0f), size(10.0f), color(1.0f, 1.0f, 1.0f)
    {
        setupCircle();
    }

    void setupCircle()
    {
        // Initialize VAO and VBO
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(cv::Point2f), &position, GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
    }

    void updatePosition(const cv::Point2f &newPos)
    {
        position = newPos;
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(cv::Point2f), &position);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void updateSize(float newSize)
    {
        size = newSize;
    }

    void updateColor(const cv::Vec3f &newColor)
    {
        color = newColor;
    }

    void draw(GLuint shaderProgram)
    {
        glUseProgram(shaderProgram);
        glUniform2fv(glGetUniformLocation(shaderProgram, "offset"), 1, &position.x);
        glUniform1f(glGetUniformLocation(shaderProgram, "size"), size);
        glUniform3fv(glGetUniformLocation(shaderProgram, "color"), 1, &color[0]);
        glBindVertexArray(VAO);
        glDrawArrays(GL_POINTS, 0, 1);
    }

    ~Circle()
    {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
    }
};

GLuint compileAndLinkShaders(const GLchar *vertex_source, const GLchar *geometry_source, const GLchar *fragment_source)
{
    auto checkCompileErrors = [](GLuint shader, const std::string &type)
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
            }
        }
        else
        {
            glGetProgramiv(shader, GL_LINK_STATUS, &success);
            if (!success)
            {
                glGetProgramInfoLog(shader, 1024, NULL, infoLog);
                ROS_ERROR("[Program Linking Error] Type: %s\n%s", type.c_str(), infoLog);
            }
        }
    };

    GLuint shader_program = glCreateProgram();

    // Vertex Shader
    if (vertex_source != nullptr)
    {
        GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex_shader, 1, &vertex_source, NULL);
        glCompileShader(vertex_shader);
        checkCompileErrors(vertex_shader, "VERTEX");
        glAttachShader(shader_program, vertex_shader);
        glDeleteShader(vertex_shader); // Delete after attaching
    }

    // Geometry Shader
    if (geometry_source != nullptr)
    {
        GLuint geometry_shader = glCreateShader(GL_GEOMETRY_SHADER);
        glShaderSource(geometry_shader, 1, &geometry_source, NULL);
        glCompileShader(geometry_shader);
        checkCompileErrors(geometry_shader, "GEOMETRY");
        glAttachShader(shader_program, geometry_shader);
        glDeleteShader(geometry_shader); // Delete after attaching
    }

    // Fragment Shader
    if (fragment_source != nullptr)
    {
        GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment_shader, 1, &fragment_source, NULL);
        glCompileShader(fragment_shader);
        checkCompileErrors(fragment_shader, "FRAGMENT");
        glAttachShader(shader_program, fragment_shader);
        glDeleteShader(fragment_shader); // Delete after attaching
    }

    // Link the shader program
    glLinkProgram(shader_program);
    checkCompileErrors(shader_program, "PROGRAM");

    return shader_program;
}

int main(int argc, char **argv)
{
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit())
    {
        return -1;
    }

    // Set up OpenGL context for GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *window = glfwCreateWindow(640, 480, "Circle Renderer", nullptr, nullptr);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Initialize GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        glfwDestroyWindow(window);
        glfwTerminate();
        return -1;
    }

    // Compile and link shaders
    GLuint shaderProgram = compileAndLinkShaders(vertexShaderSource, geometryShaderSource, fragmentShaderSource);

    // Setup the circles with different positions, sizes, and colors
    std::vector<Circle> circles;
    for (int i = 0; i < 16; ++i)
    {
        Circle circle;
        circle.position = cv::Point2f(0.1f * (i % 4) - 0.15f, 0.1f * (i / 4) - 0.15f);
        circle.size = 10.0f + i; // Different size for each circle
        circle.color = cv::Vec3f(i % 3 == 0, i % 3 == 1, i % 3 == 2);
        circles.push_back(circle);
    }

    // Render loop
    while (!glfwWindowShouldClose(window))
    {
        // Clear the color buffer
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Render each circle
        for (auto &circle : circles)
        {
            circle.draw(shaderProgram);
        }

        // Swap buffers and poll IO events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup and exit
    glDeleteProgram(shaderProgram);
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}