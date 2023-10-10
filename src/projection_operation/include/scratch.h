// // // Initialize an empty vector of cv::Point2f
// // std::vector<cv::Point2f> rect_vertices_vec;
// // rect_vertices_vec.push_back(cv::Point2f(-0.2f, 0.36f));
// // rect_vertices_vec.push_back(cv::Point2f(0.0f, 0.36f));
// // rect_vertices_vec.push_back(cv::Point2f(0.0f, 0.0f));
// // rect_vertices_vec.push_back(cv::Point2f(-0.2f, 0.0f));
// // drawRectImage(rect_vertices_vec);

// void renderTestImage_v1(GLFWwindow *p_window_id, GLuint img_id)
// {
//     // Make the window's context current
//     glfwMakeContextCurrent(p_window_id);

//     // Clear the back buffer
//     glClear(GL_COLOR_BUFFER_BIT);

//     // Enable 2D texturing
//     glEnable(GL_TEXTURE_2D);

//     // Bind the texture
//     glBindTexture(GL_TEXTURE_2D, img_id);

//     // Draw a textured quad to FBO
//     glBegin(GL_QUADS);
//     glTexCoord2f(0.0f, 0.0f);
//     glVertex2f(-0.5f, -0.5f);
//     glTexCoord2f(1.0f, 0.0f);
//     glVertex2f(0.5f, -0.5f);
//     glTexCoord2f(1.0f, 1.0f);
//     glVertex2f(0.5f, 0.5f);
//     glTexCoord2f(0.0f, 1.0f);
//     glVertex2f(-0.5f, 0.5f);
//     glEnd();

//     // Unbind the texture
//     glBindTexture(GL_TEXTURE_2D, 0);

//     // Disable 2D texturing
//     glDisable(GL_TEXTURE_2D);

//     // Swap front and back buffers
//     glfwSwapBuffers(p_window_id);
// }

// void renderTestImage_v2(GLFWwindow *p_window_id, GLuint fbo_id, GLuint fbo_texture_id, ILuint img_id)
// {
//     // // Make the window's context current
//     // glfwMakeContextCurrent(p_window_id);

//     // Clear the back buffer
//     glClear(GL_COLOR_BUFFER_BIT);

//     // Bind the FBO
//     glBindFramebuffer(GL_FRAMEBUFFER, fbo_id);

//     // Enable 2D texturing
//     glEnable(GL_TEXTURE_2D);

//     // Bind the FBO's texture
//     glBindTexture(GL_TEXTURE_2D, fbo_texture_id);

//     // Load image into FBO's texture
//     ilBindImage(img_id);
//     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ilGetInteger(IL_IMAGE_WIDTH),
//                  ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGBA,
//                  GL_UNSIGNED_BYTE, ilGetData());

//     // // Draw a textured quad to FBO
//     // glBegin(GL_QUADS);
//     // glTexCoord2f(0.0f, 0.0f);
//     // glVertex2f(-0.5f, -0.5f);
//     // glTexCoord2f(1.0f, 0.0f);
//     // glVertex2f(0.5f, -0.5f);
//     // glTexCoord2f(1.0f, 1.0f);
//     // glVertex2f(0.5f, 0.5f);
//     // glTexCoord2f(0.0f, 1.0f);
//     // glVertex2f(-0.5f, 0.5f);
//     // glEnd();

//     // Initialize an empty vector of cv::Point2f
//     std::vector<cv::Point2f> rect_vertices_vec;
//     rect_vertices_vec.push_back(cv::Point2f(-0.2f, 0.36f));
//     rect_vertices_vec.push_back(cv::Point2f(0.0f, 0.36f));
//     rect_vertices_vec.push_back(cv::Point2f(0.0f, 0.0f));
//     rect_vertices_vec.push_back(cv::Point2f(-0.2f, 0.0f));
//     drawRectImage(rect_vertices_vec);

//     // Unbind the FBO's texture
//     glBindTexture(GL_TEXTURE_2D, 0);

//     // Unbind the FBO and revert to the default framebuffer
//     glBindFramebuffer(GL_FRAMEBUFFER, 0);

//     // Disable 2D texturing
//     glDisable(GL_TEXTURE_2D);

//     // Swap front and back buffers
//     glfwSwapBuffers(p_window_id);

//     ROS_INFO("TESTING: Rendered Test Image using FBO[%d] and Texture [%d]",
//              fbo_id, fbo_texture_id);
// }

// void renderTestImage_v3(GLFWwindow *p_window_id, GLuint fbo_id, GLuint fbo_texture_id, ILuint img_id)
// {
//     // Make the window's context current
//     glfwMakeContextCurrent(p_window_id);

//     // Clear the back buffer
//     glClear(GL_COLOR_BUFFER_BIT);

//     // Bind the FBO
//     glBindFramebuffer(GL_FRAMEBUFFER, fbo_id);

//     // Check for FBO completeness
//     GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
//     if (status != GL_FRAMEBUFFER_COMPLETE)
//     {
//         ROS_ERROR("FBO is not complete");
//         return;
//     }

//     // Enable 2D texturing
//     glEnable(GL_TEXTURE_2D);

//     // Bind the FBO's texture
//     glBindTexture(GL_TEXTURE_2D, fbo_texture_id);

//     // Load image into FBO's texture
//     ilBindImage(img_id);
//     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ilGetInteger(IL_IMAGE_WIDTH),
//                  ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGBA,
//                  GL_UNSIGNED_BYTE, ilGetData());

//     // Draw a textured quad to FBO
//     glBegin(GL_QUADS);
//     glTexCoord2f(0.0f, 0.0f);
//     glVertex2f(-0.5f, -0.5f);
//     glTexCoord2f(1.0f, 0.0f);
//     glVertex2f(0.5f, -0.5f);
//     glTexCoord2f(1.0f, 1.0f);
//     glVertex2f(0.5f, 0.5f);
//     glTexCoord2f(0.0f, 1.0f);
//     glVertex2f(-0.5f, 0.5f);
//     glEnd();

//     // Add a delay
//     ros::Duration(1.0).sleep(); // Sleeps for 1 second

//     // Unbind the FBO and revert to the default framebuffer
//     glBindFramebuffer(GL_FRAMEBUFFER, 0);

//     // Clear the back buffer again
//     glClear(GL_COLOR_BUFFER_BIT);

//     // Bind the FBO's texture
//     glBindTexture(GL_TEXTURE_2D, fbo_texture_id);

//     // Draw a textured quad to default framebuffer
//     glBegin(GL_QUADS);
//     glTexCoord2f(0.0f, 0.0f);
//     glVertex2f(-0.5f, -0.5f);
//     glTexCoord2f(1.0f, 0.0f);
//     glVertex2f(0.5f, -0.5f);
//     glTexCoord2f(1.0f, 1.0f);
//     glVertex2f(0.5f, 0.5f);
//     glTexCoord2f(0.0f, 1.0f);
//     glVertex2f(-0.5f, 0.5f);
//     glEnd();

//     // Unbind the FBO's texture
//     glBindTexture(GL_TEXTURE_2D, 0);

//     // Disable 2D texturing
//     glDisable(GL_TEXTURE_2D);

//     // Swap front and back buffers
//     glfwSwapBuffers(p_window_id);

//     ROS_INFO("TESTING: Rendered Test Image using FBO[%d] and Texture [%d]",
//              fbo_id, fbo_texture_id);
// }