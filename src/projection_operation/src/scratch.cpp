// // ########################################################################################################

// // ======================================== projection_display.cpp ========================================

// // ########################################################################################################

// // ================================================== INCLUDE ==================================================

// #include "projection_display.h"

// // ================================================== FUNCTIONS ==================================================

// // Error callback for GLFW
// void glfwErrorCallback(int error, const char *description)
// {
//     std::cerr << "GLFW Error: " << error << " - " << description << std::endl;
// }

// // Function to check and log GL errors
// bool checkGLError(const std::string &message)
// {
//     GLenum err;
//     while ((err = glGetError()) != GL_NO_ERROR)
//     {
//         std::cerr << "OpenGL Error: " << err << " - " << message << std::endl;
//         return false;
//     }
//     return true;
// }

// const char *vertexShaderSource = R"glsl(
//     #version 330 core
//     layout (location = 0) in vec2 aPos;
//     uniform mat4 transform;
//     void main() {
//         gl_Position = transform * vec4(aPos, 0.0, 1.0);
//     }
// )glsl";

// // Fragment shader with color uniform
// const char *fragmentShaderSource = R"glsl(
//     #version 330 core
//     out vec4 FragColor;
//     uniform vec4 color;
//     void main() {
//         FragColor = color; // Use uniform color
//     }
// )glsl";

// // Function to generate circle vertices
// std::vector<float> generateCircleVertices(float centerX, float centerY, float radius, unsigned int numSegments)
// {
//     std::vector<float> vertices;
//     // Center vertex for triangle fan
//     vertices.push_back(centerX);
//     vertices.push_back(centerY);
//     for (unsigned int i = 0; i <= numSegments; ++i)
//     {
//         float angle = 2.0f * std::acos(-1.0) * static_cast<float>(i) / static_cast<float>(numSegments);
//         vertices.push_back(centerX + (radius * std::cos(angle))); // Use std::cos for consistency
//         vertices.push_back(centerY + (radius * std::sin(angle))); // Use std::sin for consistency
//     }
//     return vertices;
// }

// // Convert OpenCV Mat to an array that OpenGL can understand
// std::array<float, 16> cvMatToGlArray(const cv::Mat& mat) {
//     assert(mat.cols == 4 && mat.rows == 4 && mat.type() == CV_32F);
//     std::array<float, 16> glArray;
//     std::copy(mat.begin<float>(), mat.end<float>(), glArray.begin());
//     return glArray;
// }

// int main(int argc, char **argv)
// {
//     glfwSetErrorCallback(glfwErrorCallback);
//     if (!glfwInit())
//     {
//         return -1;
//     }

//     GLFWwindow *window = glfwCreateWindow(640, 480, "Diagnostic Point", nullptr, nullptr);
//     if (!window)
//     {
//         glfwTerminate();
//         return -1;
//     }
//     glfwMakeContextCurrent(window);
//     if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
//     {
//         glfwDestroyWindow(window);
//         glfwTerminate();
//         return -1;
//     }

//     // Generate circle vertices for two circles
//     std::vector<float> circleVertices1 = generateCircleVertices(0.0f, 0.0f, 0.5f, 50);
//     std::vector<float> circleVertices2 = generateCircleVertices(0.5f, -0.5f, 0.25f, 50); // Different position and size

//     // Compile shaders
//     GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
//     glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
//     glCompileShader(vertexShader);
//     checkGLError("Vertex Shader Compilation");

//     GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
//     glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
//     glCompileShader(fragmentShader);
//     checkGLError("Fragment Shader Compilation");

//     // Link shaders
//     GLuint shaderProgram = glCreateProgram();
//     glAttachShader(shaderProgram, vertexShader);
//     glAttachShader(shaderProgram, fragmentShader);
//     glLinkProgram(shaderProgram);
//     checkGLError("Shader Program Linking");

//     // Set up vertex data and buffers for two circles
//     GLuint VAOs[2], VBOs[2];
//     glGenVertexArrays(2, VAOs);
//     glGenBuffers(2, VBOs);

//     // Setup first circle
//     glBindVertexArray(VAOs[0]);
//     glBindBuffer(GL_ARRAY_BUFFER, VBOs[0]);
//     glBufferData(GL_ARRAY_BUFFER, circleVertices1.size() * sizeof(float), &circleVertices1[0], GL_STATIC_DRAW);
//     glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);
//     glEnableVertexAttribArray(0);

//     // Setup second circle
//     glBindVertexArray(VAOs[1]);
//     glBindBuffer(GL_ARRAY_BUFFER, VBOs[1]);
//     glBufferData(GL_ARRAY_BUFFER, circleVertices2.size() * sizeof(float), &circleVertices2[0], GL_STATIC_DRAW);
//     glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);
//     glEnableVertexAttribArray(0);

//     // Get uniform locations
//     GLint colorLocation = glGetUniformLocation(shaderProgram, "color");
//     GLint transformLocation = glGetUniformLocation(shaderProgram, "transform");

//     // Render loop
//     while (!glfwWindowShouldClose(window))
//     {
//         glClear(GL_COLOR_BUFFER_BIT);

//         // Use the shader program
//         glUseProgram(shaderProgram);

//         // Draw first circle with no transformation
//         cv::Mat transform = cv::Mat::eye(4, 4, CV_32F); // Identity matrix
//         auto transformArray = cvMatToGlArray(transform);
//         glUniformMatrix4fv(transformLocation, 1, GL_FALSE, transformArray.data());
//         glUniform4f(colorLocation, 1.0f, 0.0f, 0.0f, 1.0f); // Red color
//         glBindVertexArray(VAOs[0]);
//         glDrawArrays(GL_TRIANGLE_FAN, 0, circleVertices1.size() / 2);

//         // Draw second circle with different transformation and color
//         cv::Mat translation = cv::Mat::eye(4, 4, CV_32F);
//         translation.at<float>(0, 3) = 0.5f; // Translation in x
//         translation.at<float>(1, 3) = -0.5f; // Translation in y

//         cv::Mat scaling = cv::Mat::eye(4, 4, CV_32F);
//         scaling.at<float>(0, 0) = 0.25f; // Scale in x
//         scaling.at<float>(1, 1) = 0.25f; // Scale in y

//         transform = translation * scaling;
//         transformArray = cvMatToGlArray(transform);
//         glUniformMatrix4fv(transformLocation, 1, GL_FALSE, transformArray.data());
//         glUniform4f(colorLocation, 0.0f, 1.0f, 0.0f, 1.0f); // Green color
//         glBindVertexArray(VAOs[1]);
//         glDrawArrays(GL_TRIANGLE_FAN, 0, circleVertices2.size() / 2);
//         glfwSwapBuffers(window);
//         glfwPollEvents();
//     }

//     glDeleteVertexArrays(1, &VAOs[0]);
//     glDeleteVertexArrays(1, &VAOs[1]);
//     glDeleteShader(vertexShader);
//     glDeleteShader(fragmentShader);
//     glDeleteProgram(shaderProgram);
//     glfwDestroyWindow(window);
//     glfwTerminate();
//     return 0;
// }





class Circle
{
public:
    std::vector<float> vertices;
    GLuint VAO;
    GLuint VBO;
    cv::Scalar color;
    cv::Point2f position;
    float cirRadius;
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

    // // Method to set the scaling factor of the circle along the X and Y axes
    // void setScaling(cv::Point2f scaling_factors)
    // {
    //     // Create a scaling matrix with the X and Y scaling factors
    //     cv::Mat scaling = cv::Mat::eye(4, 4, CV_32F);
    //     scaling.at<float>(0, 0) = scaling_factors.x; // Apply scaling on X-axis
    //     scaling.at<float>(1, 1) = scaling_factors.y; // Apply scaling on Y-axis

    //     // Apply the scaling to the transformation matrix
    //     transformationMatrix = scaling * transformationMatrix;

    //     // Reset the transformation matrix to identity if it's empty
    //     if (transformationMatrix.empty())
    //         transformationMatrix = cv::Mat::eye(4, 4, CV_32F);

    //     // Apply the updated transformation
    //     applyTransformation();
    //     computeVertices();
    //     updateOpenGLVertices();
    // }

//     void updateVertices()
//     {
//         // Generate the new vertices based on the current position, radius, and numSegments
//         std::vector<float> newVertices = computeVertices(position, radius, numSegments);

//         // Bind the VBO, update the vertex buffer with the new data
//         glBindBuffer(GL_ARRAY_BUFFER, VBO);
//         glBufferData(GL_ARRAY_BUFFER, newVertices.size() * sizeof(float), newVertices.data(), GL_DYNAMIC_DRAW);

//         // Unbind the buffer
//         glBindBuffer(GL_ARRAY_BUFFER, 0);
//     }

//     void draw(GLuint shaderProgram, GLint colorLocation, GLint transformLocation)
//     {
//         glUniform4f(colorLocation, color[0], color[1], color[2], 1.0f); // Set color
//         cv::Mat transform = cv::Mat::eye(4, 4, CV_32F);                 // Identity matrix for no transformation
//         auto transformArray = cvMatToGlArray(transform);
//         glUniformMatrix4fv(transformLocation, 1, GL_FALSE, transformArray.data());

//         glBindVertexArray(VAO);
//         glDrawArrays(GL_TRIANGLE_FAN, 0, vertices.size() / 2);
//     }

// private:
//     std::vector<float> computeVertices()
//     {
//         return computeVertices(position, radius, numSegments);
//     }
//     std::vector<float> computeVertices(cv::Point2f position, float radius, unsigned int numSegments)
//     {
//         vertices.clear();
//         for (unsigned int i = 0; i <= numSegments; ++i)
//         {
//             float angle = 2.0f * std::acos(-1.0) * i / numSegments;
//             float baseX = position.x + (radius * std::cos(angle));
//             float baseY = position.y + (radius * std::sin(angle)) * aspectRatio;
//             cv::Mat vertex = (cv::Mat_<float>(4, 1) << baseX, baseY, 0, 1);
//             cv::Mat transformedVertex = transformationMatrix * vertex;
//             vertices.push_back(transformedVertex.at<float>(0, 0));
//             vertices.push_back(transformedVertex.at<float>(1, 0));
//         }
//         return vertices;
//     }

//     void setupOpenGL()
//     {
//         glGenVertexArrays(1, &VAO);
//         glGenBuffers(1, &VBO);
//         glBindVertexArray(VAO);
//         glBindBuffer(GL_ARRAY_BUFFER, VBO);
//         glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);
//         glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);
//         glEnableVertexAttribArray(0);
//     }

//     // void updateOpenGLVertices()
//     // {
//     //     glBindBuffer(GL_ARRAY_BUFFER, VBO);
//     //     glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(float), vertices.data());
//     // }

//     void updateOpenGLVertices()
//     {
//         glBindBuffer(GL_ARRAY_BUFFER, VBO);
//         glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);
//     }

//     // Helper function to convert OpenCV Mat to an array for OpenGL
//     std::array<float, 16> cvMatToGlArray(const cv::Mat &mat)
//     {
//         assert(mat.cols == 4 && mat.rows == 4 && mat.type() == CV_32F);
//         std::array<float, 16> glArray;
//         std::copy(mat.begin<float>(), mat.end<float>(), glArray.begin());
//         return glArray;
//     }

//     // void applyTransformation()
//     // {
//     //     // Ensure transformationMatrix is initialized
//     //     if (transformationMatrix.empty())
//     //         transformationMatrix = cv::Mat::eye(4, 4, CV_32F);

//     //     // Translate the center of the circle to the origin
//     //     cv::Mat translationToOrigin = cv::Mat::eye(4, 4, CV_32F);
//     //     translationToOrigin.at<float>(0, 3) = -position.x;
//     //     translationToOrigin.at<float>(1, 3) = -position.y;

//     //     // Rotate around the origin
//     //     cv::Mat rotation = cv::getRotationMatrix2D(cv::Point2f(0, 0), rotationAngle, 1.0);
//     //     cv::Mat rot4x4 = cv::Mat::eye(4, 4, CV_32F);
//     //     rotation.copyTo(rot4x4.rowRange(0, 2).colRange(0, 3));

//     //     // Translate back to the original position
//     //     cv::Mat translationBack = cv::Mat::eye(4, 4, CV_32F);
//     //     translationBack.at<float>(0, 3) = position.x;
//     //     translationBack.at<float>(1, 3) = position.y;

//     //     // Apply the transformations
//     //     transformationMatrix = translationBack * rot4x4 * translationToOrigin * transformationMatrix;
//     // }

//     void applyTransformation()
//     {
//         // Ensure transformationMatrix is initialized
//         if (transformationMatrix.empty())
//             transformationMatrix = cv::Mat::eye(4, 4, CV_32F);

//         // Apply aspect ratio correction by scaling the y-axis
//         cv::Mat aspectRatioCorrection = cv::Mat::eye(4, 4, CV_32F);
//         aspectRatioCorrection.at<float>(1, 1) = aspectRatio;

//         // Apply uniform scaling for stretch (if needed)
//         cv::Mat scaling = cv::Mat::eye(4, 4, CV_32F);
//         // Assuming uniformScale is the stretch factor
//         scaling.at<float>(0, 0) = scalingFactor.x;
//         scaling.at<float>(1, 1) = scalingFactor.y;

//         // Translate to origin, rotate, scale, correct aspect ratio, and translate back
//         cv::Mat translationToOrigin = cv::Mat::eye(4, 4, CV_32F);
//         translationToOrigin.at<float>(0, 3) = -position.x;
//         translationToOrigin.at<float>(1, 3) = -position.y;

//         cv::Mat rotation = cv::getRotationMatrix2D(cv::Point2f(0, 0), rotationAngle, 1.0);
//         cv::Mat rot4x4 = cv::Mat::eye(4, 4, CV_32F);
//         rotation.copyTo(rot4x4.rowRange(0, 2).colRange(0, 3));

//         cv::Mat translationBack = cv::Mat::eye(4, 4, CV_32F);
//         translationBack.at<float>(0, 3) = position.x;
//         translationBack.at<float>(1, 3) = position.y;

//         transformationMatrix = translationBack * aspectRatioCorrection * scaling * rot4x4 * translationToOrigin * transformationMatrix;
//     }
// };