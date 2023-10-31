//  srcPoints:
//           Quad Vertices
//  ===================================
//      |     Left     |     Right    |
//  -----------------------------------
//      |V0  X , Y     |V1  X , Y     |
//  -----------------------------------
//  Top |    +0,    +0 |  +300,    +0 |
//  -----------------------------------
//      |V3  X , Y     |V2  X , Y     |
//  -----------------------------------
//  Btm |    +0,  +540 |  +300,  +540 |
//  =====================================
//  dstPoints:
//           Quad Vertices
//  ===================================
//      |     Left     |     Right    |
//  -----------------------------------
//      |V0  X , Y     |V1  X , Y     |
//  -----------------------------------
//  Top |  +375,  +230 |  +675,  +205 |
//  -----------------------------------
//      |V3  X , Y     |V2  X , Y     |
//  -----------------------------------
//  Btm |  +350,  +770 |  +600,  +695 |
//  ===================================
//           Homography Matrix
//  ==================================
//            |  C0   |  C1   |  C2   |
//  ----------------------------------
//  R0        | +1.31 | +0.11 | +375.00 |
//  R1        | +0.01 | +1.34 | +230.00 |
//  R2        | +0.00 | +0.00 | +1.00 |
//  ==================================

/**
*               ________          ________          ________
*             /   (w2)   \      /          \      /          \
*           / (w1)     (w3)\  /              \  /              \
*          |                ||                ||                |
*          |(w0)  [c0]  (w4)||      [c1]      ||      [c2]      |
*          |                ||                ||                |
*           \ (w7)    (w5) /  \              /  \              /
*             \ __(w6)__ /      \ ________ /      \ ________ /
*             /          \      /          \      /          \
*           /              \  /              \  /              \
*          |                ||                ||                |
*          |      [c3]      ||      [c4]      ||      [c5]      |
*          |                ||                ||                |
*           \              /  \              /  \              /
*             \ ________ /      \ ________ /      \ ________ /
*             /          \      /          \      /          \
*           /              \  /              \  /              \
*          |                |                 ||                |
*          |      [c6]      ||      [c7]      ||      [c8]      |
*          |                ||                ||                |
*           \              /  \              /  \              /
*             \ ________ /      \ ________ /      \ ________ /
*/


/** 
* @var const GLchar* vertexSource
* @brief Vertex shader source code.

* @details
* This is a GLSL (OpenGL Shading Language) vertex shader source code stored as a C++ raw string literal.
* - `#version 330 core`: Specifies that the GLSL version is 3.30 and we're using the core profile.
* - `in vec2 position;`: Declares an input vertex attribute called `position`. Receives vertex coordinates from the application.
* - `in vec2 texcoord;`: Declares another input vertex attribute called `texcoord`. Receives texture coordinates from the application.
* - `out vec2 Texcoord;`: Declares an output variable that will be passed to the fragment shader.
* - `void main() { ... }`: Main function where the vertex shader performs its work.
*/
const GLchar *vertexSource = R"glsl(
const GLchar *vertexSource = R"glsl(
    #version 330 core
    in vec2 position;
    in vec2 texcoord;
    out vec2 Texcoord;
    void main() {
        Texcoord = texcoord;
        gl_Position = vec4(position, 0.0, 1.0);
    }
)glsl";

/**
* @var const GLchar* fragmentSource
* @brief Fragment shader source code.
*
* @details
* This is a GLSL (OpenGL Shading Language) fragment shader source code also stored as a C++ raw string literal.
* - `#version 330 core`: Specifies that the GLSL version is 3.30 and we're using the core profile.
* - `in vec2 Texcoord;`: Receives the texture coordinates from the vertex shader.
* - `out vec4 outColor;`: Declares an output variable for storing the color to be used for the fragment.
* - `uniform sampler2D tex;`: Declares a uniform variable representing a 2D texture.
* - `void main() { ... }`: Main function of the fragment shader, samples the texture at the given coordinates and sets the output color.
*/
const GLchar *fragmentSource = R"glsl(
    #version 330 core
    in vec2 Texcoord;
    out vec4 outColor;
    uniform sampler2D tex;
    void main() {
        outColor = texture(tex, Texcoord);
    }
)glsl";
