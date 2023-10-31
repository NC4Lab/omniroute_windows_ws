// // int callbackErrorOpenCV(int status, const char* func_name,
// //                    const char* err_msg, const char* file_name,
// //                    int line, void* userdata)
// // {
// //     ROS_ERROR("[OpenCV] Error Callback: Function[%s] File[%s] Line[%d] Description[%s]", func_name, file_name, line, err_msg);
// //     return 0;  // Return 0 to suppress the default OpenCV error handling
// // }

// // // Redirect OpenCV errors to custom error handler
// // cv::redirectError(callbackErrorOpenCV);

// // // Some OpenCV code that might trigger an error
// // cv::Mat img = cv::imread("nonexistent_file.jpg");
// // return 0;



// int main(int argc, char **argv)
// {
//     // ROS Initialization
//     ros::init(argc, argv, "projection_calibration");
//     ros::NodeHandle n;
//     ros::NodeHandle nh("~");

//     // --------------- VARIABLE SETUP ---------------

//     // Specify window resolution: 4K resolution (3840x2160)
//     const int win_height_pxl = 1000;
//     const int win_width_pxl = 1000;
//     const float win_aspect_ratio = (float)win_width_pxl / (float)win_height_pxl;

//     // Image size (pixels)
//     const int img_width_pxl = 300;
//     const int img_height_pxl = 540;

//     // Image size (NDC)
//     const float img_width_ndc = (static_cast<float>(img_width_pxl) / static_cast<float>(win_width_pxl)) * 2;
//     const float img_height_ndc = (static_cast<float>(img_height_pxl) / static_cast<float>(win_height_pxl)) * 2;

//     // --------------- Specify Transform ---------------

//     // Define origin plane vertices
//     std::vector<cv::Point2f> origin_vertices_ndc = {
//         cv::Point2f(0.0f, img_height_ndc),          // Top-left
//         cv::Point2f(img_width_ndc, img_height_ndc), // Top-right
//         cv::Point2f(0.0f, 0.0f),                    // Bottom-left
//         cv::Point2f(img_width_ndc, 0.0f)};          // Bottom-right

//     // Center the image
//     for (int i = 0; i < 4; ++i)
//     {
//         origin_vertices_ndc[i].x -= img_width_ndc / 2.0f;
//         origin_vertices_ndc[i].y -= img_height_ndc / 2.0f;
//     }

//     // Copy the origin plane vertices
//     std::vector<cv::Point2f> target_vertices_ndc;
//     for (const auto &point : origin_vertices_ndc)
//     {
//         target_vertices_ndc.push_back(cv::Point2f(point.x, point.y));
//     }

//     // Warp the target in some way
//     target_vertices_ndc[1].x += 0.1f;
//     target_vertices_ndc[3].y -= 0.1f;
//     target_vertices_ndc[3].x -= 0.1f;
//     target_vertices_ndc[2].x -= 0.1f;
//     target_vertices_ndc[0].y += 0.1f;

//     // --------------- OpenGL SETUP ---------------

//     // Initialize GLFW
//     if (!glfwInit())
//     {
//         ROS_ERROR("Failed to initialize GLFW");
//         return -1;
//     }

//     // Create a GLFW window
//     GLFWwindow *window = glfwCreateWindow(win_height_pxl, win_width_pxl, "Projection Calibration", NULL, NULL);
//     if (window == NULL)
//     {
//         ROS_ERROR("Failed to create GLFW window");
//         glfwTerminate();
//         return -1;
//     }
//     glfwMakeContextCurrent(window);

//     // Load OpenGL function pointers using GLAD
//     if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
//     {
//         ROS_ERROR("Failed to initialize GLAD");
//         return -1;
//     }

//     // OpenGL texture for the warped image
//     GLuint warped_texture_id;
//     glGenTextures(1, &warped_texture_id);

//     // OpenGL setup
//     glViewport(0, 0, win_width_pxl, win_height_pxl);
//     glMatrixMode(GL_PROJECTION);
//     glLoadIdentity();
//     glOrtho(-1, 1, -1, 1, -1, 1);
//     glMatrixMode(GL_MODELVIEW);
//     glLoadIdentity();

//     // --------------- Image Setup ---------------

//     // Load an image using OpenCV
//     std::string img_path = "C:/Users/lester/MeDocuments/Research/MadhavLab/CodeBase/omniroute_windows_ws/data/proj_img/calibration_images/1_test_pattern.bmp";
//     // std::string img_path = "C:/Users/lester/MeDocuments/Research/MadhavLab/CodeBase/omniroute_windows_ws/data/proj_img/calibration_images/2_manu_pirate.bmp";
//     cv::Mat img_raw = cv::imread(img_path.c_str());
//     if (img_raw.empty())
//     {
//         ROS_ERROR("Failed to load image");
//         return -1;
//     }
//     if (img_raw.type() == CV_8UC3)
//     {
//         ROS_INFO("Color format is RGB (BGR order)");
//     }
//     else if (img_raw.type() == CV_8UC1)
//     {
//         ROS_INFO("Color format is grayscale");
//     }
//     else
//     {
//         ROS_INFO("Color format is not RGB (BGR order) or grayscale");
//     }
//     ROS_INFO("Number of channels in the image: %d", img_raw.channels());

//     // --------------- Get pixel coordinates ---------------

//     // Function to convert NDC to Pixel Coordinates
//     auto convertToPixelCoords = [](std::vector<cv::Point2f> &quad_vertices, int width_pxl, int height_pxl, float width_ndc, float height_ndc)
//     {
//         float min_x = std::numeric_limits<float>::max();
//         float min_y = std::numeric_limits<float>::max();

//         // First pass: Convert from NDC [-1, 1] to pixel [0, width or height] and find minimum x and y
//         for (const auto &point : quad_vertices)
//         {
//             float x_pxl = ((point.x / width_ndc) + 0.5f) * width_pxl;
//             float y_pxl = ((-point.y / height_ndc) + 0.5f) * height_pxl; // Invert y to match OpenCV's top-left origin

//             min_x = std::min(min_x, x_pxl);
//             min_y = std::min(min_y, y_pxl);
//         }

//         // Calculate the shifts needed to make all coordinates non-negative
//         float x_shift = (min_x < 0) ? -min_x : 0;
//         float y_shift = (min_y < 0) ? -min_y : 0;

//         // Second pass: Shift all points by the calculated shifts
//         for (auto &point : quad_vertices)
//         {
//             point.x = ((point.x / width_ndc) + 0.5f) * width_pxl + x_shift;
//             point.y = ((-point.y / height_ndc) + 0.5f) * height_pxl + y_shift;
//         }
//     };

//     // Convert origin_vertices to pixel coordinates
//     std::vector<cv::Point2f> origin_vertices_pixel = origin_vertices_ndc;
//     convertToPixelCoords(origin_vertices_pixel, img_width_pxl, img_height_pxl, img_width_ndc, img_height_ndc);

//     // Convert target_vertices to pixel coordinates
//     std::vector<cv::Point2f> target_vertices_pixel = target_vertices_ndc;
//     convertToPixelCoords(target_vertices_pixel, img_width_pxl, img_height_pxl, img_width_ndc, img_height_ndc);

//     // --------------- Warp the Image ---------------

//     // Find the bounding rectangle for the target vertices
//     cv::Rect targ_bounding_rec = cv::boundingRect(target_vertices_pixel);
//     cv::Size targ_img_size(targ_bounding_rec.width, targ_bounding_rec.height);

//     // Find Homography and Warp Image
//     cv::Mat img_warped;
//     cv::Mat H = cv::findHomography(origin_vertices_pixel, target_vertices_pixel);
//     cv::warpPerspective(img_raw, img_warped, H, targ_img_size);

//     // --------------- Get texture coordinates ---------------

//     auto scaleQuadVert2UnitSquare = [](std::vector<cv::Point2f> &quad_vertices)
//     {
//         // Initialze max to type min and min to type max
//         cv::Point2f p_min(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
//         cv::Point2f p_max(std::numeric_limits<float>::min(), std::numeric_limits<float>::min());

//         // Find the min/max values
//         for (const auto &point : quad_vertices)
//         {
//             p_min.x = std::min(p_min.x, point.x);
//             p_max.x = std::max(p_max.x, point.x);
//             p_min.y = std::min(p_min.y, point.y);
//             p_max.y = std::max(p_max.y, point.y);
//         }

//         // Specify range
//         float s_min = 0.0f;
//         float s_max = 1.0f;

//         // Scale the values to the new range
//         for (auto &point : quad_vertices)
//         {
//             point.x = s_min + (point.x - p_min.x) * (s_max - s_min) / (p_max.x - p_min.x);
//             point.y = s_min + (point.y - p_min.y) * (s_max - s_min) / (p_max.y - p_min.y);
//         }
//     };

//     // Flip the y-coordinate to change from OpenGL to OpenCV
//     auto invertQuadVertYAxis = [](std::vector<cv::Point2f> &quad_vertices, float y_max)
//     {
//         for (cv::Point2f &point : quad_vertices)
//             point.y = y_max - point.y;
//     };

//     // Convert target vertices to texture coordinates
//     std::vector<cv::Point2f> target_vertices_uv = target_vertices_ndc;
//     scaleQuadVert2UnitSquare(target_vertices_uv);

//     // Flip the y-axis
//     // invertQuadVertYAxis(target_vertices_uv);

//     // --------------- Debugging ---------------

//     cv::namedWindow("Warped Image Display", cv::WINDOW_AUTOSIZE);
//     cv::imshow("Warped Image Display", img_warped);
//     // cv::waitKey(0);
//     // cv::destroyWindow("Warped Image Display");

//     // Log the homography matrix and vertices
//     ROS_INFO("[COMPUTE HOMOGRAPHY] target_vertices_pixel:");
//     dbLogQuadVertices(target_vertices_pixel);
//     ROS_INFO("[COMPUTE HOMOGRAPHY] target_vertices_ndc:");
//     dbLogQuadVertices(target_vertices_ndc);
//     ROS_INFO("[COMPUTE HOMOGRAPHY] target_vertices_uv:");
//     dbLogQuadVertices(target_vertices_uv);
//     // dbLogHomMat(H);

//     // Log targ_img_size
//     ROS_INFO("PIXEL BOUNDING BOX: Width[%d] Height[%d]", targ_img_size.width, targ_img_size.height);

//     // Define origin plane vertices
//     std::vector<cv::Point2f> temp_vertices = {
//         cv::Point2f(50.0f, 300.0f),  // Top-left
//         cv::Point2f(150.0f, 350.0f), // Top-right
//         cv::Point2f(0.0f, 0.0f),     // Bottom-left
//         cv::Point2f(100.0f, 50.0f)}; // Bottom-right
//     ROS_INFO("[COMPUTE HOMOGRAPHY] Example Vertices:");
//     dbLogQuadVertices(temp_vertices);

//     // --------------- Draw Maker setup ---------------

//     // Colors for markers
//     std::array<float, 3> rgb_green = {0.0f, 1.0f, 0.0f};
//     std::array<float, 3> rgb_red = {1.0f, 0.0f, 0.0f};
//     std::array<float, 3> rgb_blue = {0.0f, 0.0f, 1.0f};
//     std::array<float, 3> rgb_cyan = {0.0f, 1.0f, 1.0f};
//     float cp_rad = 0.05f;

//     auto drawMaker = [](float x, float y, float radius, std::array<float, 3> rgb_arr)
//     {
//         const int segments = 100; // Number of segments to approximate a circle

//         // Begin drawing a filled circle
//         glBegin(GL_TRIANGLE_FAN);

//         // Set the color to green
//         glColor3f(rgb_arr[0], rgb_arr[1], rgb_arr[2]);

//         // Center of the circle
//         glVertex2f(x, y);

//         // Calculate and draw the vertices of the circle
//         for (int i = 0; i <= segments; i++)
//         {
//             float theta = 2.0f * 3.1415926f * float(i) / float(segments);
//             float px = x + radius * cosf(theta);
//             float py = y + (radius * PROJ_WIN_ASPECT_RATIO) * sinf(theta);
//             glVertex2f(px, py);
//         }

//         // End drawing
//         glEnd();

//         // Return GL status
//         return checkErrorOpenGL(__LINE__, __FILE__);
//     };

//     // --------------- Draw the wall image ---------------

//     // Main loop
//     while (!glfwWindowShouldClose(window))
//     {
//         // Rendering commands here
//         glClear(GL_COLOR_BUFFER_BIT);

//         // Enable OpenGL texture mapping
//         glEnable(GL_TEXTURE_2D);

//         // Bind the texture and upload the warped image
//         glBindTexture(GL_TEXTURE_2D, warped_texture_id);

//         // Upload the warped RGB image
//         glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_warped.cols, img_warped.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, img_warped.data);

//         // Upload the RGB image
//         // glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_rgb.cols, img_rgb.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, img_rgb.data); // Modified Line

//         // Texture Parameters
//         glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//         glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

//         // Start drawing a quadrilateral
//         glBegin(GL_QUADS);

//         // Set the color to white (for texture mapping)
//         glColor3f(1.0f, 1.0f, 1.0f);

//         // Top-left corner of texture
//         glTexCoord2f(target_vertices_uv[0].x, target_vertices_uv[0].y);
//         glVertex2f(target_vertices_ndc[0].x, target_vertices_ndc[0].y);

//         // Top-right corner of texture
//         glTexCoord2f(target_vertices_uv[1].x, target_vertices_uv[1].y);
//         glVertex2f(target_vertices_ndc[1].x, target_vertices_ndc[1].y);

//         // Bottom-right corner of texture
//         glTexCoord2f(target_vertices_uv[3].x, target_vertices_uv[3].y);
//         glVertex2f(target_vertices_ndc[3].x, target_vertices_ndc[3].y);

//         // Bottom-left corner of texture
//         glTexCoord2f(target_vertices_uv[2].x, target_vertices_uv[2].y);
//         glVertex2f(target_vertices_ndc[2].x, target_vertices_ndc[2].y);

//         // End drawing
//         glEnd();

//         // // Draw markers
//         // drawMaker(-0.5f, 0.5f, cp_rad, rgb_green); // Top-left
//         // drawMaker(0.5f, 0.5f, cp_rad, rgb_red);    // Top-right
//         // drawMaker(-0.5f, -0.5f, cp_rad, rgb_blue); // Bottom-left
//         // drawMaker(0.5f, -0.5f, cp_rad, rgb_cyan);  // Bottom-right

//         // Swap buffers and poll events
//         glfwSwapBuffers(window);
//         glfwPollEvents();
//     }

//     // _______________ CLEANUP _______________

//     // Cleanup and exit
//     glDisable(GL_TEXTURE_2D);
//     glDeleteTextures(1, &warped_texture_id);
//     glfwDestroyWindow(window);
//     glfwTerminate();

//     return 0;
// }