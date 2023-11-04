// ########################################################################################################

// ======================================== projection_display.cpp ========================================

// ########################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_display.h"

// ================================================== FUNCTIONS ==================================================

class Circle
{
public:
    std::vector<float> circVertices;
    GLuint VAO;
    GLuint VBO;
    cv::Scalar circColor;
    cv::Point2f circPosition;
    float cirRadius;
    unsigned int numSegments;
    float aspectRatio;

    cv::Mat transformationMatrix; // Transformation matrix for the circle

    Circle(cv::Point2f pos, float rad, cv::Scalar col, unsigned int segments, float aspect)
        : circPosition(pos), cirRadius(rad), circColor(col), numSegments(segments), aspectRatio(aspect)
    {
        // Initialize the transformation matrix as an identity matrix
        transformationMatrix = cv::Mat::eye(4, 4, CV_32F);

        // Run initial vertex computation
        _computeVertices();
        setupOpenGL();
    }

    ~Circle()
    {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
    }

    void setPosition(cv::Point2f pos)
    {
        circPosition = pos;
        updateTransformationMatrix();

        // Recompute the vertices and update the OpenGL buffer
        _computeVertices();
        _updateOpenGLVertices();
    }

    void setRadius(float rad)
    {
        cirRadius = rad;

        // Recompute the vertices and update the OpenGL buffer
        _computeVertices();
        _updateOpenGLVertices();
    }
    void setColor(cv::Scalar col)
    {
        circColor = col;
    }

    // Method to set the rotation angle and apply the rotation
    void setRotation(float angle)
    {
        // Update transforamtion matrix with new angle
        updateTransformationMatrix(angle);

        // Recompute the vertices and update the OpenGL buffer
        _computeVertices();
        _updateOpenGLVertices();
    }

    // Method to set the scaling factor of the circle along the X and Y axes
    void setScaling(cv::Point2f scaling_factors)
    {
        updateTransformationMatrix(scaling_factors);

        // Recompute the vertices and update the OpenGL buffer
        _computeVertices();
        _updateOpenGLVertices();
    }

    void updateOpenGL()
    {
        // Generate the new vertices based on the current position, radius, and numSegments
        std::vector<float> newVertices = _computeVertices(circPosition, cirRadius, numSegments);

        // Bind the VBO, update the vertex buffer with the new data
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, newVertices.size() * sizeof(float), newVertices.data(), GL_DYNAMIC_DRAW);

        // Unbind the buffer
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void draw(GLuint shaderProgram, GLint colorLocation, GLint transformLocation)
    {
        glUniform4f(colorLocation, circColor[0], circColor[1], circColor[2], 1.0f); // Set color
        cv::Mat transform = cv::Mat::eye(4, 4, CV_32F);                             // Identity matrix for no transformation
        auto transformArray = cvMatToGlArray(transform);
        glUniformMatrix4fv(transformLocation, 1, GL_FALSE, transformArray.data());

        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLE_FAN, 0, circVertices.size() / 2);
    }

private:
    std::vector<float> _computeVertices()
    {
        return _computeVertices(circPosition, cirRadius, numSegments);
    }
    std::vector<float> _computeVertices(cv::Point2f position, float radius, unsigned int numSegments)
    {
        circVertices.clear();
        for (unsigned int i = 0; i <= numSegments; ++i)
        {
            float angle = 2.0f * std::acos(-1.0) * i / numSegments;
            float baseX = position.x + (radius * std::cos(angle));
            float baseY = position.y + (radius * std::sin(angle)) * aspectRatio;
            cv::Mat vertex = (cv::Mat_<float>(4, 1) << baseX, baseY, 0, 1);
            cv::Mat transformedVertex = transformationMatrix * vertex;
            circVertices.push_back(transformedVertex.at<float>(0, 0));
            circVertices.push_back(transformedVertex.at<float>(1, 0));
        }
        return circVertices;
    }

    void setupOpenGL()
    {
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, circVertices.size() * sizeof(float), circVertices.data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);
        glEnableVertexAttribArray(0);
    }

    void _updateOpenGLVertices()
    {
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, circVertices.size() * sizeof(float), circVertices.data(), GL_DYNAMIC_DRAW);
    }

    // Helper function to convert OpenCV Mat to an array for OpenGL
    std::array<float, 16> cvMatToGlArray(const cv::Mat &mat)
    {
        assert(mat.cols == 4 && mat.rows == 4 && mat.type() == CV_32F);
        std::array<float, 16> glArray;
        std::copy(mat.begin<float>(), mat.end<float>(), glArray.begin());
        return glArray;
    }

    void updateTransformationMatrix(float angle)
    {
        updateTransformationMatrix(angle, cv::Point2f(1.0f, 1.0f));
    }
    void updateTransformationMatrix(cv::Point2f scaling_factors)
    {
        updateTransformationMatrix(0, scaling_factors);
    }
    void updateTransformationMatrix()
    {
        updateTransformationMatrix(0, cv::Point2f(1.0f, 1.0f));
    }
    void updateTransformationMatrix(float angle, cv::Point2f scaling_factors)
    {
        // Initialize transformation matrix as identity if not done
        if (transformationMatrix.empty())
            transformationMatrix = cv::Mat::eye(4, 4, CV_32F);

        // Apply transformations in correct order: Translate to origin -> Rotate -> Scale -> Translate back
        cv::Mat translationToOrigin = cv::Mat::eye(4, 4, CV_32F);
        translationToOrigin.at<float>(0, 3) = -circPosition.x;
        translationToOrigin.at<float>(1, 3) = -circPosition.y;

        cv::Mat rotation = cv::Mat::eye(4, 4, CV_32F);
        // ... set rotation matrix using angle ...

        cv::Mat scaling = cv::Mat::eye(4, 4, CV_32F);
        scaling.at<float>(0, 0) = scaling_factors.x;
        scaling.at<float>(1, 1) = scaling_factors.y;

        cv::Mat translationBack = cv::Mat::eye(4, 4, CV_32F);
        translationBack.at<float>(0, 3) = circPosition.x;
        translationBack.at<float>(1, 3) = circPosition.y;

        // The multiplication order here is important
        transformationMatrix = translationBack * scaling * rotation * translationToOrigin;
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

    // Create an array of Circle class objects
    constexpr size_t nCircles = 3;
    std::vector<Circle> CVEC;
    CVEC.reserve(nCircles); // Optional: Reserve the memory upfront for efficiency
    for (size_t i = 0; i < nCircles; ++i)
    {
        // Assuming you want to vary the parameters for each circle
        float radius = 0.05 * (i + 1);
        float pos = 0.5 * (i + 1) - 1.0f;
        cv::Point2f position(pos, pos);
        cv::Scalar color(i * 50, 255 - i * 50, 0); // Example: Different colors for each circle
        unsigned int segments = 36;                // Example: Same number of segments for each circle

        // Construct the Circle object with the given parameters and add it to the vector
        CVEC.emplace_back(position, radius, color, segments, aspectRatio);
    }

    // Stretch the circle by factors of 1.2 along the X-axis and
    // 0.8 along the Y-axis
    // myCircle.setScaling(cv::Point2f(1.25f, 0.8f));

    // Render loop
    while (!glfwWindowShouldClose(window))
    {
        glClear(GL_COLOR_BUFFER_BIT);

        // Use the shader program
        glUseProgram(shaderProgram);

        if (dbRunDT(100))
        {
            CVEC[1].setRotation(5.0);

            static cv::Point2f pos = (-1.0f, -1.0f);
            pos.x += 0.01f;
            pos.y += 0.01f;
            if (pos.x > 1.0f)
                pos = cv::Point2f(-1.0f, -1.0f);
            // cv::Point2f pos = (CVEC[0].position.x += 0.01f, CVEC[0].position.y += 0.01f);
            CVEC[0].setPosition(pos);

            static bool do_switch = false;
            if (do_switch)
                CVEC[2].setColor(cv::Scalar(0.0f, 0.0f, 1.0f));
            else
                CVEC[2].setColor(cv::Scalar(0.0f, 1.0f, 0.0f));
            do_switch = !do_switch;
        }

        // if (dbRunDT(500))
        // {
        //     static cv::Point2f stretch = cv::Point2f(1.0f, 1.0f);
        //     myCircle.setScaling (stretch); // Rotate the circle by 45 degrees
        //     stretch.x += 0.001f;
        //     stretch.y += 0.001f;
        //     if (stretch.x > 0.5f)
        //         stretch = cv::Point2f(0.0f, 0.0f);
        // }

        // Generate new vertices for the updated circle
        for (size_t i = 0; i < nCircles; ++i)
        {
            CVEC[i].updateOpenGL();
            CVEC[i].draw(shaderProgram, colorLocation, transformLocation);
        }

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
