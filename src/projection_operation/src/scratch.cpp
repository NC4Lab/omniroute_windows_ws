
/**
 * @brief Vertex shader for rendering points with pixel coordinates.
 *
 * @details
 * This vertex shader converts pixel coordinates to normalized device
 * coordinates (NDC) and flips the y-axis to accommodate OpenGL's
 * bottom-left origin. This shader is used for rendering control points
 * where the position is specified in window pixel coordinates.
 *
 * Uniforms:
 * - `screenWidth` : The width of the window in pixels.
 * - `screenHeight`: The height of the window in pixels.
 *
 * Attributes:
 * - `position`: The x and y pixel coordinates of the point.
 * - `color`   : The RGB color of the point.
 * - `size`    : The size of the point in pixels.
 *
 * Outputs:
 * - `fragColor`: The color passed to the fragment shader.
 */
const GLchar *ctrlPtVertexSource = R"glsl(
    #version 330 core
    uniform float screenWidth;
    uniform float screenHeight;
    layout (location = 0) in vec2 position; // Position in pixels
    layout (location = 1) in vec3 color;
    layout (location = 2) in float size;
    out vec3 fragColor;

    void main() {
        // Convert pixel coordinates to NDC
        vec2 ndcPosition = (position - vec2(screenWidth / 2.0, screenHeight / 2.0)) / vec2(screenWidth / 2.0, screenHeight / 2.0);
        ndcPosition.y = -ndcPosition.y; // Flip y-axis

        gl_Position = vec4(ndcPosition, 0.0, 1.0);
        gl_PointSize = size;
        fragColor = color;
    }
)glsl";

/**
 * @brief Fragment shader for rendering points with specified colors.
 *
 * @details
 * This fragment shader takes the color from the vertex shader and assigns
 * it to the fragment's final color output. It's used in conjunction with
 * the vertex shader for control point rendering.
 *
 * Inputs:
 * - `fragColor`: The RGB color passed from the vertex shader.
 *
 * Outputs:
 * - `color`: The final color of the fragment.
 */
const GLchar *ctrlPtFragmentSource = R"glsl(
    #version 330 core
    in vec3 fragColor;
    out vec4 color;

    void main() {
        color = vec4(fragColor, 1.0);
    }
)glsl";

void callbackFrameBufferSizeGLFW(GLFWwindow *window, int width, int height)
{
    // Update the viewport
    glViewport(0, 0, width, height);

    // Update the uniform values
    GLint screenWidthLoc = glGetUniformLocation(CP_SHADER, "screenWidth");
    GLint screenHeightLoc = glGetUniformLocation(CP_SHADER, "screenHeight");
    glUseProgram(CP_SHADER);
    glUniform1f(screenWidthLoc, (float)width);
    glUniform1f(screenHeightLoc, (float)height);

    checkErrorOpenGL(__LINE__, __FILE__);
}