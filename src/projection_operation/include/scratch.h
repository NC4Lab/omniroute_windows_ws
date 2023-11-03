
// class Circle
// {
// public:
//     std::vector<float> vertices;
//     GLuint VAO;
//     GLuint VBO;
//     cv::Scalar color;
//     cv::Point2f position;
//     float radius;
//     unsigned int numSegments;
//     float aspectRatio;
//     float rotationAngle;
//     cv::Mat transformationMatrix; // Transformation matrix for the circle

//     Circle(cv::Point2f pos, float rad, cv::Scalar col, unsigned int segments, float aspect)
//         : position(pos), radius(rad), color(col), numSegments(segments), aspectRatio(aspect) // Initialize aspectRatio here
//     {
//         computeVertices();
//         setupOpenGL();
//         // Initialize the transformation matrix as an identity matrix
//         transformationMatrix = cv::Mat::eye(4, 4, CV_32F);
//     }

//     ~Circle()
//     {
//         glDeleteVertexArrays(1, &VAO);
//         glDeleteBuffers(1, &VBO);
//     }

//     void setPosition(cv::Point2f pos)
//     {
//         position = pos;
//         computeVertices();
//         updateOpenGLVertices();
//     }

//     void setRadius(float rad)
//     {
//         radius = rad;
//         computeVertices();
//         updateOpenGLVertices();
//     }

//     void setColor(cv::Scalar col)
//     {
//         color = col;
//     }

//     // Method to stretch the circle by factors along the X and Y axes
//     void stretch(cv::Point2f factors)
//     {
//         // Create a scaling matrix with the X and Y scaling factors
//         cv::Mat scaling = cv::Mat::eye(4, 4, CV_32F);
//         scaling.at<float>(0, 0) = factors.x; // Apply scaling on X-axis
//         scaling.at<float>(1, 1) = factors.y; // Apply scaling on Y-axis

//         // Apply the scaling to the transformation matrix
//         transformationMatrix = scaling * transformationMatrix;

//         // Apply the updated transformation
//         applyTransformation();

//         computeVertices();
//         updateOpenGLVertices();
//     }

//     // Method to set the rotation angle and apply the rotation
//     void setRotation(float angle)
//     {
//         rotationAngle = angle; // Set the new rotation angle
//         applyTransformation(); // Apply the transformation

//         computeVertices();
//         updateOpenGLVertices();
//     }

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
//         vertices.push_back(position.x); // Center x
//         vertices.push_back(position.y); // Center y
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

//     void applyTransformation()
//     {
//         // Update the transformation matrix with the new rotation
//         cv::Mat rotation = cv::getRotationMatrix2D(position, rotationAngle, 1.0);
//         // Convert the 2x3 rotation matrix to a 4x4 matrix
//         cv::Mat rot4x4 = cv::Mat::eye(4, 4, CV_32F);
//         rotation.copyTo(rot4x4.rowRange(0, 2).colRange(0, 3));
//         // Apply the rotation to the transformation matrix
//         transformationMatrix = rot4x4 * transformationMatrix;
//     }
// };