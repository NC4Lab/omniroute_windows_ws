// ######################################################################################################

// ======================================== projection_display.h ========================================

// ######################################################################################################

#ifndef _PROJECTION_DISPLY_H
#define _PROJECTION_DISPLY_H

// ================================================== INCLUDE ==================================================

// Local custom libraries
#include "projection_utils.h"

// ================================================== VARIABLES ==================================================

/**
 * @class Circle
 * @brief Encapsulates a circle's properties and its rendering.
 *
 * This class is designed to represent a circle with various attributes such as
 * position, color, and scale. It also provides functionality to compute vertex
 * data for rendering the circle in OpenGL and update its transformation matrix.
 */
class Circle
{
public:
    /**
     * @brief Vertex shader source code with aspect ratio correction.
     *
     * @note The shader program will reverse the y axis to match image coordinate systems used by OpenCV.
     *
     * @details
     * This GLSL vertex shader code corrects for the aspect ratio of the display
     * when rendering circle vertices.
     *
     * - `#version 330 core`: Specifies that we're using GLSL version 3.30 with the core profile.
     * - `layout (location = 0) in vec2 aPos;`: Defines the input vertex attribute at location 0.
     * - `uniform mat4 transform;`: A uniform for the transformation matrix.
     * - `uniform float aspectRatio;`: A uniform for the aspect ratio correction. Also flips the y-coordinate.
     * - `void main() { ... }`: The main function that applies transformation and aspect ratio correction.
     *
     * Use this shader source by compiling it into a vertex shader object and linking it into a shader program.
     * Set the `aspectRatio` uniform before drawing to correct the y-coordinate of vertices based on the display aspect ratio.
     */
    static constexpr const char *vertexShaderSource = R"glsl(
    #version 330 core
    layout (location = 0) in vec2 aPos;
    uniform mat4 transform;
    uniform float aspectRatio; // Uniform for the aspect ratio

    void main() {
        vec2 correctedPos = aPos;
        correctedPos.y *= -aspectRatio; // Flip the y-coordinate and correct the aspect ratio
        gl_Position = transform * vec4(correctedPos, 0.0, 1.0);
    }
    )glsl";
    /**
     * @brief Fragment shader source code for circle coloring.
     *
     * @details
     * This is a GLSL fragment shader source code stored as a C++ raw string literal.
     * - `#version 330 core`: Specifies that the GLSL version is 3.30 and we're using the core profile.
     * - `out vec4 FragColor;`: Declares an output variable for the final fragment color.
     * - `uniform vec4 color;`: Declares a uniform variable for the color, which can be set dynamically via OpenGL calls.
     * - `void main() { ... }`: Main function of the fragment shader, sets the fragment color to the uniform color value.
     */
    static constexpr const char *fragmentShaderSource = R"glsl(
        #version 330 core
        out vec4 FragColor;
        uniform vec4 color;

        void main() {
            FragColor = color;
        }
    )glsl";

    std::vector<float> circVertices; // Vertex data for the circle's geometry.
    GLuint VAO;                      // Vertex Array Object for the circle.
    GLuint VBO;                      // Vertex Buffer Object for the circle's vertices.
    int circIndex;                   // Index of the circle, used for identification.
    cv::Scalar circColor;            // Color of the circle.
    cv::Point2f circPosition;        // Position of the circle in 2D space.
    float cirRadius;                 // Radius of the circle.
    float circRotationAngle;         // Rotation angle of the circle in degrees.
    cv::Point2f circScalingFactors;  // Scaling factors for the circle's x and y dimensions.
    unsigned int numSegments;        // Number of segments used to approximate the circle geometry.
    float aspectRatio;               // Aspect ratio of the rendering surface.
    cv::Mat transformationMatrix;    // Transformation matrix for the circle's vertices.
    bool doUpdateOpenGLVertices;     // Flag to indicate if OpenGL vertices need to be updated.

    /**
     * @brief Construct a new Circle object.
     *
     * @param idx Index of the circle.
     * @param pos Initial position of the circle.
     * @param rad Radius of the circle.
     * @param col Color of the circle.
     * @param segments Number of segments to approximate the circle.
     * @param aspect Aspect ratio of the rendering surface.
     */
    Circle(int idx, cv::Point2f pos, float rad, cv::Scalar col, unsigned int segments, float aspect);

    /**
     * @brief Destroy the Circle object.
     *
     * Cleans up the OpenGL objects associated with the circle.
     */
    ~Circle();

    void setPosition(cv::Point2f pos);
    void setRadius(float rad);
    void setRotationAngle(float angle);
    void setScaling(cv::Point2f scaling_factors);
    void setColor(cv::Scalar col);
    void recomputeParameters();
    void draw(GLuint shaderProgram, GLint colorLocation, GLint transformLocation);

private:
    void _computeVertices(cv::Point2f position, float radius, unsigned int numSegments, std::vector<float> &circVertices);
    void _setupOpenGL();
    void _updateOpenGLVertices();
    void _computeTransformation();
    std::array<float, 16> _cvMatToGlArray(const cv::Mat &mat);

    // Consider making constants such as pi as a private static constexpr member.
    static constexpr float PI = 3.14159265358979323846f;
};

/**
 * @brief  Entry point for the projection_display ROS node.
 *
 * This program initializes ROS, DevIL, and GLFW, and then enters a main loop
 * to handle image projection and calibration tasks.
 *
 * @param  argc  Number of command-line arguments.
 * @param  argv  Array of command-line arguments.
 *
 * @return 0 on successful execution, -1 on failure.
 */
int main(int, char **);

#endif
