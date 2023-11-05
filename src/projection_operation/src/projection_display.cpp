// ########################################################################################################

// ======================================== projection_display.cpp ========================================

// ########################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_display.h"

// ================================================== FUNCTIONS ==================================================

Circle::Circle(int idx, cv::Point2f pos, float rad, cv::Scalar col, unsigned int segments, float aspect)
    : circIndex(idx), circPosition(pos), cirRadius(rad), circColor(col), numSegments(segments), aspectRatio(aspect)
{
    // Initialize the transformation matrix as an identity matrix
    transformationMatrix = cv::Mat::eye(4, 4, CV_32F);

    // Initialize variables
    circScalingFactors = cv::Point2f(1.0f, 1.0f);
    circRotationAngle = 0.0f;

    // Run initial vertex computation
    _computeVertices(circPosition, cirRadius, numSegments, circVertices);

    // Setup OpenGL buffers
    _setupOpenGL();
}

Circle::~Circle()
{
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
}

void Circle::setPosition(cv::Point2f pos)
{
    circPosition = pos;
}

void Circle::setRadius(float rad)
{
    cirRadius = rad;
}

void Circle::setRotationAngle(float angle)
{
    circRotationAngle = angle;
}

void Circle::setScaling(cv::Point2f scaling_factors)
{
    circScalingFactors = scaling_factors;
}

void Circle::setColor(cv::Scalar col)
{
    circColor = col;
}

void Circle::recomputeParameters()
{
    // Update the transformation matrix
    _computeTransformation();

    // Generate the new vertices based on the current position, radius, and numSegments
    _computeVertices(circPosition, cirRadius, numSegments, circVertices);

    // Update the OpenGL buffer
    _updateOpenGLVertices();

    // Bind the VBO, update the vertex buffer with the new data
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, circVertices.size() * sizeof(float), circVertices.data(), GL_DYNAMIC_DRAW);

    // Unbind the buffer
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Circle::draw(GLuint shaderProgram, GLint colorLocation, GLint transformLocation)
{
    // Set color
    glUniform4f(colorLocation, circColor[0], circColor[1], circColor[2], 1.0f);

    // Use the updated transformation matrix instead of an identity matrix
    auto transformArray = _cvMatToGlArray(transformationMatrix);
    glUniformMatrix4fv(transformLocation, 1, GL_FALSE, transformArray.data());

    // Bind the VAO and draw the circle using GL_TRIANGLE_FAN
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLE_FAN, 0, static_cast<GLsizei>(circVertices.size() / 2));
}

void Circle::_computeVertices(cv::Point2f position, float radius, unsigned int numSegments, std::vector<float> &circVertices)
{
    // Clear the vertex vector to prepare for new vertices
    circVertices.clear();

    // Loop to generate vertices for the circle
    for (unsigned int i = 0; i <= numSegments; ++i)
    {
        // Calculate the angle for the current segment
        float angle = 2.0f * std::acos(-1.0) * i / numSegments;
        // Determine the x position of the vertex on the circle
        float baseX = position.x + (radius * std::cos(angle));
        // Determine the y position of the vertex on the circle
        float baseY = position.y + (radius * std::sin(angle));

        // Create a 4x1 matrix (homogeneous coordinates) for the vertex
        cv::Mat vertex = (cv::Mat_<float>(4, 1) << baseX, baseY, 0, 1);
        // Apply the transformation matrix to the vertex
        cv::Mat transformedVertex = transformationMatrix * vertex;
        // Extract the transformed x coordinate and add to the vertices list
        circVertices.push_back(transformedVertex.at<float>(0, 0));
        // Extract the transformed y coordinate and add to the vertices list
        circVertices.push_back(transformedVertex.at<float>(1, 0));
    }
}

void Circle::_setupOpenGL()
{
    // Generate a new Vertex Array Object (VAO) and store the ID
    glGenVertexArrays(1, &VAO);
    // Generate a new Vertex Buffer Object (VBO) and store the ID
    glGenBuffers(1, &VBO);

    // Bind the VAO to set it up
    glBindVertexArray(VAO);

    // Bind the VBO to the GL_ARRAY_BUFFER target
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    // Copy vertex data into the buffer's memory (GL_DYNAMIC_DRAW hints that the data might change often)
    glBufferData(GL_ARRAY_BUFFER, circVertices.size() * sizeof(float), circVertices.data(), GL_DYNAMIC_DRAW);

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

void Circle::_updateOpenGLVertices()
{
    // Bind the VBO to the GL_ARRAY_BUFFER target
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    // Update the VBO's data with the new vertices. This call will reallocate the buffer if necessary
    // or simply update the data store's contents if the buffer is large enough.
    glBufferData(GL_ARRAY_BUFFER, circVertices.size() * sizeof(float), circVertices.data(), GL_DYNAMIC_DRAW);
    // No need to unbind here if this method is always followed by a draw call,
    // which binds its own buffers as needed.
}

void Circle::_computeTransformation()
{
    // Initialize transformation matrix as identity if not done
    if (transformationMatrix.empty())
    {
        transformationMatrix = cv::Mat::eye(4, 4, CV_32F);
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
    transformationMatrix = translationBack * scaling * rotation * translationToOrigin;
}

// Helper function to convert OpenCV Mat to an array for OpenGL
std::array<float, 16> Circle::_cvMatToGlArray(const cv::Mat &mat)
{
    assert(mat.cols == 4 && mat.rows == 4 && mat.type() == CV_32F);
    std::array<float, 16> glArray;
    std::copy(mat.begin<float>(), mat.end<float>(), glArray.begin());
    return glArray;
}

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
    glShaderSource(vertexShader, 1, &Circle::vertexShaderSource, nullptr);
    glCompileShader(vertexShader);
    checkGLError("Vertex Shader Compilation");

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &Circle::fragmentShaderSource, nullptr);
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
    GLint aspectRatioLocation = glGetUniformLocation(shaderProgram, "aspectRatio"); // Get the location of the aspectRatio uniform

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
        CVEC.emplace_back(static_cast<int>(i), position, radius, color, segments, aspectRatio);
    }
    CVEC[0].setColor(cv::Scalar(1.0f, 0.0f, 0.0f));
    CVEC[1].setColor(cv::Scalar(0.0f, 1.0f, 0.0f));
    CVEC[3].setColor(cv::Scalar(0.0f, 0.0f, 1.0f));

    // Stretch the circle by factors of 1.2 along the X-axis and
    // 0.8 along the Y-axis
    CVEC[1].setScaling(cv::Point2f(0.8f, 1.25f));

    // Render loop
    while (!glfwWindowShouldClose(window))
    {
        glClear(GL_COLOR_BUFFER_BIT);

        // Use the shader program
        glUseProgram(shaderProgram);

        // Set the aspect ratio uniform
        glUniform1f(aspectRatioLocation, aspectRatio);

        if (dbRunDT(100))
        {
            static cv::Point2f pos = (-1.0f, -1.0f);
            pos.x += 0.01f;
            pos.y += 0.01f;
            if (abs(pos.x) > 1.0f || abs(pos.y) > 1.0f)
                pos = cv::Point2f(-1.0f, -1.0f);
            // cv::Point2f pos = (CVEC[0].position.x += 0.01f, CVEC[0].position.y += 0.01f);
            CVEC[0].setPosition(pos);

            static float angle = CVEC[1].circRotationAngle;
            angle += 5.00f;
            if (angle >= 360.0f)
                angle = 0.0f;
            CVEC[1].setRotationAngle(angle);

            static float radius = CVEC[2].cirRadius;
            radius += 0.01f;
            if (radius > 0.1)
                radius = 0.05f;
            CVEC[2].setRadius(radius);

            // static bool do_switch = false;
            // if (do_switch)
            //     CVEC[2].setColor(cv::Scalar(0.0f, 0.0f, 1.0f));
            // else
            //     CVEC[2].setColor(cv::Scalar(0.0f, 1.0f, 0.0f));
            // do_switch = !do_switch;

            for (size_t i = 0; i < nCircles; ++i)
            {
                CVEC[i].recomputeParameters();
            }
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
