// ########################################################################################################

// ======================================== projection_display.cpp ========================================

// ########################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_display.h"

// ================================================== FUNCTIONS ==================================================

class Circle
{
public:
    std::vector<float> vertices;
    GLuint VAO;
    GLuint VBO;
    cv::Scalar color;
    cv::Point2f position;
    float radius;
    unsigned int numSegments;
    float aspectRatio;
    cv::Point2f scalingFactor;
    float rotationAngle;

    cv::Mat transformationMatrix; // Transformation matrix for the circle

    Circle(cv::Point2f pos, float rad, cv::Scalar col, unsigned int segments, float aspect)
        : position(pos), radius(rad), color(col), numSegments(segments), aspectRatio(aspect)
    {
        // Initialize the transformation matrix as an identity matrix
        transformationMatrix = cv::Mat::eye(4, 4, CV_32F);

        // Run initial vertex computation and setup OpenGL
        computeVertices();
        setupOpenGL();
    }

    ~Circle()
    {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
    }

    void setPosition(cv::Point2f pos)
    {
        position = pos;
        computeVertices();
        updateOpenGLVertices();
    }

    void setRadius(float rad)
    {
        radius = rad;
        computeVertices();
        updateOpenGLVertices();
    }

    void setColor(cv::Scalar col)
    {
        color = col;
    }

    // Method to set the rotation angle and apply the rotation
    void setRotation(float angle)
    {
        rotationAngle = angle;
        applyTransformation();
        computeVertices();
        updateOpenGLVertices();
    }

    // Method to set the scaling factor of the circle along the X and Y axes
    void setScaling(cv::Point2f scaling_factors)
    {
        scalingFactor = scaling_factors;
        applyTransformation();
        computeVertices();
        updateOpenGLVertices();
    }

    void updateVertices()
    {
        // Generate the new vertices based on the current position, radius, and numSegments
        std::vector<float> newVertices = computeVertices(position, radius, numSegments);

        // Bind the VBO, update the vertex buffer with the new data
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, newVertices.size() * sizeof(float), newVertices.data(), GL_DYNAMIC_DRAW);

        // Unbind the buffer
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void draw(GLuint shaderProgram, GLint colorLocation, GLint transformLocation)
    {
        glUniform4f(colorLocation, color[0], color[1], color[2], 1.0f); // Set color
        cv::Mat transform = cv::Mat::eye(4, 4, CV_32F);                 // Identity matrix for no transformation
        auto transformArray = cvMatToGlArray(transform);
        glUniformMatrix4fv(transformLocation, 1, GL_FALSE, transformArray.data());

        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLE_FAN, 0, vertices.size() / 2);
    }

private:
    std::vector<float> computeVertices()
    {
        return computeVertices(position, radius, numSegments);
    }
    std::vector<float> computeVertices(cv::Point2f position, float radius, unsigned int numSegments)
    {
        vertices.clear();
        for (unsigned int i = 0; i <= numSegments; ++i)
        {
            float angle = 2.0f * std::acos(-1.0) * i / numSegments;
            float baseX = position.x + (radius * std::cos(angle));
            float baseY = position.y + (radius * std::sin(angle)) * aspectRatio;
            cv::Mat vertex = (cv::Mat_<float>(4, 1) << baseX, baseY, 0, 1);
            cv::Mat transformedVertex = transformationMatrix * vertex;
            vertices.push_back(transformedVertex.at<float>(0, 0));
            vertices.push_back(transformedVertex.at<float>(1, 0));
        }
        return vertices;
    }

    void setupOpenGL()
    {
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);
        glEnableVertexAttribArray(0);
    }

    void updateOpenGLVertices()
    {
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);
    }

    // Helper function to convert OpenCV Mat to an array for OpenGL
    std::array<float, 16> cvMatToGlArray(const cv::Mat &mat)
    {
        assert(mat.cols == 4 && mat.rows == 4 && mat.type() == CV_32F);
        std::array<float, 16> glArray;
        std::copy(mat.begin<float>(), mat.end<float>(), glArray.begin());
        return glArray;
    }

    void applyTransformation()
    {
        // Ensure transformationMatrix is initialized
        if (transformationMatrix.empty())
            transformationMatrix = cv::Mat::eye(4, 4, CV_32F);

        // Translate the center of the circle to the origin
        cv::Mat translationToOrigin = cv::Mat::eye(4, 4, CV_32F);
        translationToOrigin.at<float>(0, 3) = -position.x;
        translationToOrigin.at<float>(1, 3) = -position.y;

        // Rotate around the origin
        cv::Mat rotation = cv::getRotationMatrix2D(cv::Point2f(0, 0), rotationAngle, 1.0);
        cv::Mat rot4x4 = cv::Mat::eye(4, 4, CV_32F);
        rotation.copyTo(rot4x4.rowRange(0, 2).colRange(0, 3));

        // Translate back to the original position
        cv::Mat translationBack = cv::Mat::eye(4, 4, CV_32F);
        translationBack.at<float>(0, 3) = position.x;
        translationBack.at<float>(1, 3) = position.y;

        // Apply the transformations
        transformationMatrix = translationBack * rot4x4 * translationToOrigin * transformationMatrix;
    }
};

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

const char *vertexShaderSource = R"glsl(
    #version 330 core
    layout (location = 0) in vec2 aPos;
    uniform mat4 transform;
    void main() {
        gl_Position = transform * vec4(aPos, 0.0, 1.0);
    }
)glsl";

// Fragment shader with color uniform
const char *fragmentShaderSource = R"glsl(
    #version 330 core
    out vec4 FragColor;
    uniform vec4 color;
    void main() {
        FragColor = color; // Use uniform color
    }
)glsl";

int main(int argc, char **argv)
{
    // ROS Initialization
    ros::init(argc, argv, "projection_calibration", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    ROS_INFO("RUNNING MAIN");

    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit())
    {
        return -1;
    }

    int width = 640;
    int height = 480;
    float aspectRatio = width / (float)height;
    GLFWwindow *window = glfwCreateWindow(width, 480, "Diagnostic Point", nullptr, nullptr);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        glfwDestroyWindow(window);
        glfwTerminate();
        return -1;
    }

    // Compile shaders
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
    glCompileShader(vertexShader);
    checkGLError("Vertex Shader Compilation");

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
    glCompileShader(fragmentShader);
    checkGLError("Fragment Shader Compilation");

    // Link shaders
    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    checkGLError("Shader Program Linking");

    // Get uniform locations
    GLint colorLocation = glGetUniformLocation(shaderProgram, "color");
    GLint transformLocation = glGetUniformLocation(shaderProgram, "transform");

    // Create a circle
    Circle myCircle(cv::Point2f(0.0f, 0.0f), 0.5f, cv::Scalar(1.0f, 0.0f, 0.0f), 50, aspectRatio);

    // Stretch the circle by factors of 1.2 along the X-axis and
    // 0.8 along the Y-axis
    myCircle.setScaling (cv::Point2f(1.25f, 0.8f));

    // Render loop
    while (!glfwWindowShouldClose(window))
    {
        glClear(GL_COLOR_BUFFER_BIT);

        // Use the shader program
        glUseProgram(shaderProgram);

        // Change circle properties
        myCircle.setPosition(cv::Point2f(0.5f, -0.5f));  // Move the circle to position (0.5, -0.5)
        myCircle.setColor(cv::Scalar(0.0f, 1.0f, 0.0f)); // Change the circle color to green
        myCircle.setRadius(0.05f);                       // Change the circle radius to 0.25

        // if (dbRunDT(500))
        // {
        //     static cv::Point2f stretch = cv::Point2f(1.0f, 1.0f);
        //     myCircle.setScaling (stretch); // Rotate the circle by 45 degrees
        //     stretch.x += 0.001f;
        //     stretch.y += 0.001f;
        //     if (stretch.x > 0.5f)
        //         stretch = cv::Point2f(0.0f, 0.0f);
        // }

        if (dbRunDT(100))
        {
            myCircle.setRotation(5.0);
        }

        // Generate new vertices for the updated circle
        myCircle.updateVertices();

        // Draw the circle
        myCircle.draw(shaderProgram, colorLocation, transformLocation);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    glDeleteProgram(shaderProgram);
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
