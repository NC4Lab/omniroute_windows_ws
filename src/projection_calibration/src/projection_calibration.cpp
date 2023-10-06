/**
 * @file projection_calibration.cpp
 * @brief This file contains the implementation for the projection calibration.
 */

//============= INCLUDE ================
#include "projection_calibration.h"

/**
 * @brief Callback function for handling key bindings.
 *
 * @param window Pointer to the GLFW window.
 * @param key The key that was pressed or released.
 * @param scancode The system-specific scancode of the key.
 * @param action GLFW_PRESS, GLFW_RELEASE, or GLFW_REPEAT.
 * @param mods Bit field describing which modifier keys were held down.
 *
 *  @ref: GLFW/glfw3.h for keybindings enum
 *
 * Key Bindings:
 * - [1-4]: Select target control point (Top-left, Top-right, Bottom-right, Bottom-left)
 * - [F1-F12]: Set image to image 1 to 12
 * - [A, D, S]: Change control point mode(position/translation, dimension/height, shear)
 * - [ENTER]: Save coordinates to XML
 * - [L]: Load coordinates from XML
 * - [F]: Fullscreen on second monitor
 * - [M]: Move window to next monitor
 * - [Arrow Keys]: Move selected control point or adjust dimension/shear
 */
void callbackKeyBinding(GLFWwindow *window, int key, int scancode, int action, int mods)
{

    // Set the current OpenGL context to the window
    glfwMakeContextCurrent(window);

    // _______________ ANY KEY RELEASE ACTION _______________

    if (action == GLFW_RELEASE)
    {
        // ---------- Target selector keys [1-4] ----------

        // Top-left control point
        if (key == GLFW_KEY_1)
        {
            cpSelected = 0;
        }

        // Top-right control point
        else if (key == GLFW_KEY_2)
        {
            cpSelected = 1;
        }

        // Bottom-right control point
        else if (key == GLFW_KEY_3)
        {
            cpSelected = 2;
        }

        // Bottom-left control point
        else if (key == GLFW_KEY_4)
        {
            cpSelected = 3;
        }

        // ---------- Image selector keys [F1-F12] ----------

        else if (key == GLFW_KEY_F1)
        {
            imgTestInd = 0;
        }
        else if (key == GLFW_KEY_F2)
        {
            imgTestInd = 1;
        }
        else if (key == GLFW_KEY_F3)
        {
            imgTestInd = 2;
        }
        else if (key == GLFW_KEY_F4)
        {
            imgTestInd = 3;
        }

        // ---------- Change mode keys [A, D, S] ----------

        // Control point position [up, down, left, right]
        else if (key == GLFW_KEY_A)
        {
            cpModMode = "position";
            imgModeInd = 0;
        }

        // Control point height [up, down]
        else if (key == GLFW_KEY_D)
        {
            cpModMode = "dimension";
            imgModeInd = 1;
        }

        // Control point shear [up, down]
        else if (key == GLFW_KEY_S)
        {
            cpModMode = "shear";
            imgModeInd = 2;
        }

        // ---------- XML Handling [ENTER, L] ----------

        // Save coordinates to XML
        else if (key == GLFW_KEY_ENTER)
        {
            saveCoordinatesXML();
        }

        // Load coordinates from XML
        else if (key == GLFW_KEY_L)
        {
            loadCoordinatesXML();
        }

        // ---------- Monitor handling [F, M] ----------

        // Set/unset Fullscreen
        else if (key == GLFW_KEY_F)
        {
            isFullScreen = !isFullScreen;
            changeWindowMonMode();
        }

        // Move the window to the other monitor
        else if (key == GLFW_KEY_M)
        {
            imgMonNumInd = (imgMonNumInd < monitorCount - 1) ? imgMonNumInd + 1 : 0;
            changeWindowMonMode();
        }

        // ---------- Control Point Reset [R] ----------

        else if (key == GLFW_KEY_R)
        {
            resetParamCP();
        }
    }

    // _______________ ANY KEY PRESS OR REPEAT ACTION _______________
    else if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {

        // ---------- Control point position change [LEFT, RIGHT, UP, DOWN] ----------
        if (cpModMode == "position")
        {
            // Set the position increment based on whether the shift key is pressed
            float pos_inc = (mods & GLFW_MOD_SHIFT) ? 0.05f : 0.01f;

            // Listen for arrow key input to move selected control point
            if (key == GLFW_KEY_LEFT)
            {
                cpParam[cpSelected][0] -= pos_inc;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                cpParam[cpSelected][0] += pos_inc;
            }
            else if (key == GLFW_KEY_UP)
            {
                cpParam[cpSelected][1] += pos_inc;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                cpParam[cpSelected][1] -= pos_inc;
            }
        }

        // ---------- Control point dimension/hight change [UP, DOWN] ----------
        if (cpModMode == "dimension")
        {
            // Set the dimension increment based on whether the shift key is pressed
            float dim_inc = (mods & GLFW_MOD_SHIFT) ? 0.001f : 0.0005f;

            // Listen for arrow key input to adjust dimension/height
            if (key == GLFW_KEY_UP)
            {
                cpParam[cpSelected][3] += dim_inc;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                cpParam[cpSelected][3] -= dim_inc;
            }
        }

        // ---------- Control point shear change [UP, DOWN] ----------
        if (cpModMode == "shear")
        {
            // Set the shear increment based on whether the shift key is pressed
            float shr_inc = (mods & GLFW_MOD_SHIFT) ? 0.05f : 0.001f;

            // Listen for arrow key input to adjust shear
            if (key == GLFW_KEY_UP)
            {
                cpParam[cpSelected][4] += shr_inc;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                cpParam[cpSelected][4] -= shr_inc;
            }
        }
    }

    // ---------- Recompute homography matrix ----------
    computeHomography();
}

/**
 * @brief Callback function for handling framebuffer size changes.
 *
 * This function is called whenever the framebuffer size changes,
 * and it updates the OpenGL viewport to match the new dimensions.
 *
 * @param window Pointer to the GLFW window.
 * @param width The new width of the framebuffer.
 * @param height The new height of the framebuffer.
 */
void callbackFrameBufferSize(GLFWwindow *window, int width, int height)
{
    glViewport(0, 0, width, height);
}

/**
 * @brief Callback function for handling errors.
 *
 * This function is called whenever an error occurs in the GLFW context.
 * It logs the error message using ROS_ERROR.
 *
 * @param error The error code.
 * @param description The error description.
 */
static void callbackError(int error, const char *description)
{
    ROS_ERROR("Error: %s\n", description);
}

/**
 * @brief Draws a control point as a quadrilateral using OpenGL.
 *
 * This function uses OpenGL to draw a quadrilateral that represents a control point.
 * The control point is drawn in a clockwise direction, starting from the bottom-left corner.
 *
 * @param x The x-coordinate of the bottom-left corner of the control point.
 * @param y The y-coordinate of the bottom-left corner of the control point.
 * @param radius The radius of the control point.
 * @param rgb Vector of rgb values to color the marker.
 */
void drawControlPoint(float x, float y, float radius, std::vector<float> rgb)
{
    int segments = 100; // Number of segments to approximate a circle

    // Begin drawing a filled circle
    glBegin(GL_TRIANGLE_FAN);

    // Set the color to green
    glColor3f(rgb[0], rgb[1], rgb[2]);

    // Center of the circle
    glVertex2f(x, y);

    // Calculate and draw the vertices of the circle
    for (int i = 0; i <= segments; i++)
    {
        float theta = 2.0f * 3.1415926f * float(i) / float(segments);
        float px = x + radius * cosf(theta);
        float py = y + (radius * winAspectRatio) * sinf(theta);
        glVertex2f(px, py);
    }

    // End drawing
    glEnd();
}

/**
 * @brief Draws a textured wall using OpenGL.
 *
 * @param img_vertices Vector of vertex/corner points for the wall.
 */
void drawWall(std::vector<cv::Point2f> img_vertices)
{

    // Start drawing a quadrilateral
    glBegin(GL_QUADS);

    // Set the color to white
    glColor3f(1.0f, 1.0f, 1.0f);

    // Set texture and vertex coordinates for each corner
    // Bottom-left corner
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(img_vertices[0].x, img_vertices[0].y);

    // Bottom-right corner
    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(img_vertices[1].x, img_vertices[1].y);

    // Top-right corner
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(img_vertices[2].x, img_vertices[2].y);

    // Top-left corner
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(img_vertices[3].x, img_vertices[3].y);

    // End drawing
    glEnd();
}

/**
 * @brief Draws all the walls in the maze using OpenGL and OpenCV.
 *
 * This function iterates through the maze grid and draws each wall with
 * texture mapping and perspective warping. It uses control points to
 * determine the shear and height for each wall.
 */
void drawWallsAll()
{
    // Extract shear and height values from control points
    float height1 = cpParam[0][3];
    float height3 = cpParam[2][3];
    float height4 = cpParam[3][3];
    float shear1 = cpParam[0][4];
    float shear3 = cpParam[2][4];
    float shear4 = cpParam[3][4];

    // Enable OpenGL texture mapping
    glEnable(GL_TEXTURE_2D);

    // Iterate through the maze grid
    for (float i_wall = 0; i_wall < MAZE_SIZE; i_wall++)
    {

        // Iterate through each cell in the maze row
        for (float j_wall = 0; j_wall < MAZE_SIZE; j_wall++)
        {

            // Bind image
            if (i_wall == 1 && j_wall == 1)
            {
                // Merge images
                ILuint merge_images_1 = mergeImages(imgTestIDs[imgTestInd], imgMonNumIDs[imgMonNumInd]);
                ILuint merge_images_2 = mergeImages(merge_images_1, imgModeIDs[imgModeInd]);
                ilBindImage(merge_images_2);
            }
            else
            {
                ilBindImage(imgTestIDs[imgTestInd]); // show test pattern
            }

            // Set texture image
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
                         ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB,
                         GL_UNSIGNED_BYTE, ilGetData());

            // Bind texture to framebuffer object
            glBindTexture(GL_TEXTURE_2D, fboTexture);

            // Calculate shear and height for the current wall
            float shear_val = shear4 + (i_wall / (MAZE_SIZE - 1)) * (shear3 - shear4) +
                              (j_wall / (MAZE_SIZE - 1)) * (shear1 - shear4);
            float height_val = height4 + (i_wall / (MAZE_SIZE - 1)) * (height3 - height4) +
                               (j_wall / (MAZE_SIZE - 1)) * (height1 - height4);

            // Create wall vertices
            std::vector<cv::Point2f> img_vertices = computeWallVertices(0.0f, 0.0f, wallWidth, height_val, shear_val);

            // Apply perspective warping to vertices
            for (auto &p : img_vertices)
            {
                // Update vertex positions based on shear and height
                p.x += i_wall * wallSpace;
                p.y += j_wall * wallSpace;

                // Apply homography matrix to warp perspective
                float data[] = {p.x, p.y, 1};
                cv::Mat ptMat(3, 1, CV_32F, data);
                H.convertTo(H, ptMat.type());
                ptMat = H * ptMat;
                ptMat /= ptMat.at<float>(2);

                // Update vertex coordinates
                p.x = ptMat.at<float>(0, 0);
                p.y = ptMat.at<float>(0, 1);
            }

            // Draw the wall
            drawWall(img_vertices);
        }
    }

    // Disable OpenGL texture mapping
    glDisable(GL_TEXTURE_2D);
}

/**
 * @brief Merges two images by overlaying non-white pixels from the second image onto the first.
 *
 * This function takes two images, img1 and img2, represented as ILuint IDs. It overlays img2 onto img1,
 * replacing pixels in img1 with corresponding non-white pixels from img2. The resulting merged image is
 * returned as a new ILuint ID.
 *
 * @param img1 The ILuint ID of the baseline image.
 * @param img2 The ILuint ID of the mask image.
 * @return ILuint ID of the merged image. Returns 0 if an error occurs.
 *
 * @warning The dimensions of img1 and img2 must match.
 */
ILuint mergeImages(ILuint img1, ILuint img2)
{
    // Bind and get dimensions of img1 (baseline image)
    ilBindImage(img1);
    int width1 = ilGetInteger(IL_IMAGE_WIDTH);
    int height1 = ilGetInteger(IL_IMAGE_HEIGHT);
    ILubyte *data1 = ilGetData();
    ILenum error = ilGetError();
    if (error != IL_NO_ERROR)
    {
        ROS_ERROR("Error binding img1: %s", iluErrorString(error));
        return 0;
    }

    // Bind and get dimensions of img2 (mask image)
    ilBindImage(img2);
    int width2 = ilGetInteger(IL_IMAGE_WIDTH);
    int height2 = ilGetInteger(IL_IMAGE_HEIGHT);
    ILubyte *data2 = ilGetData();
    error = ilGetError();
    if (error != IL_NO_ERROR)
    {
        ROS_ERROR("Error binding img2: %s", iluErrorString(error));
        return 0;
    }

    // Check for dimension match
    if (width1 != width2 || height1 != height2)
    {
        ROS_ERROR("Dimensions do not match: img1(%d, %d), img2(%d, %d)",
                  width1, height1, width2, height2);
        return 0;
    }

    // Create merged image
    ILuint mergedImg;
    ilGenImages(1, &mergedImg);
    ilBindImage(mergedImg);
    ilTexImage(width1, height1, 1, 4, IL_RGBA, IL_UNSIGNED_BYTE, NULL);
    error = ilGetError();
    if (error != IL_NO_ERROR)
    {
        ROS_ERROR("Error creating merged image: %s", iluErrorString(error));
        return 0;
    }

    // Initialize mergedData array
    ILubyte *mergedData = new ILubyte[width1 * height1 * 4];

    // Loop to overlay non-white pixels from img2 onto img1
    for (int i = 0; i < width1 * height1 * 4; i += 4)
    {
        if (data2[i] != 255 || data2[i + 1] != 255 || data2[i + 2] != 255)
        {
            // If the pixel is not white in img2, use it in the merged image
            for (int j = 0; j < 4; ++j)
            {
                mergedData[i + j] = data2[i + j];
            }
        }
        else
        {
            // Otherwise, use the pixel from img1
            for (int j = 0; j < 4; ++j)
            {
                mergedData[i + j] = data1[i + j];
            }
        }
    }

    // Set mergedData to the new image
    ilBindImage(mergedImg);
    ilSetPixels(0, 0, 0, width1, height1, 1, IL_RGBA, IL_UNSIGNED_BYTE, mergedData);
    error = ilGetError();
    if (error != IL_NO_ERROR)
    {
        ROS_ERROR("Error setting pixels for merged image: %s", iluErrorString(error));
        delete[] mergedData;
        return 0;
    }

    // Clean up
    delete[] mergedData;

    return mergedImg;
}

/**
 * @brief Changes the display mode and monitor of the application window.
 *
 * This function switches the application window between full-screen and windowed modes
 * and moves it to the monitor specified by the global variable imgMonNumInd.
 *
 * In full-screen mode, the window is resized to match the dimensions of the selected monitor.
 * In windowed mode, the window is resized to a default size and positioned near the top-left
 * corner of the selected monitor.
 *
 * @note The global variables monitor, monitors, imgMonNumInd, window, and isFullScreen are
 *       used to control the behavior of this function.
 *
 * @warning This function relies on the GLFW library for window management and ROS for logging.
 */
void changeWindowMonMode()
{
    monitor = monitors[imgMonNumInd];

    if (monitor)
    {
        // Get the video mode of the selected monitor
        const GLFWvidmode *mode = glfwGetVideoMode(monitor);

        // Set the window to full-screen mode on the current monitor
        glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);

        if (!isFullScreen)
        {
            // Get the position of the current monitor
            int monitor_x, monitor_y;
            glfwGetMonitorPos(monitor, &monitor_x, &monitor_y);

            // Set the window to windowed mode and position it on the current monitor
            glfwSetWindowMonitor(window, NULL, monitor_x + 100, monitor_y + 100, (int)(500.0f * winAspectRatio), 500, 0);
        }
        ROS_INFO("RAN: Move window to monitor %d and set to %s", imgMonNumInd, isFullScreen ? "fullscreen" : "windowed");
    }
    else
    {
        ROS_WARN("FAILED: Move window to monitor %d and set to %s", imgMonNumInd, isFullScreen ? "fullscreen" : "windowed");
    }
}

/**
 * @brief Creates a vector of points representing a rectangle with shear for each wall.
 *
 * This function generates a rectangle's corner points starting from the top-left corner
 * and going clockwise. The rectangle is defined by its top-left corner (x0, y0),
 * width, height, and a shear amount.
 *
 * @param x0 The x-coordinate of the top-left corner of the rectangle.
 * @param y0 The y-coordinate of the top-left corner of the rectangle.
 * @param width The width of the rectangle.
 * @param height The height of the rectangle.
 * @param shear_amount The amount of shear to apply to the rectangle.
 *
 * @return std::vector<cv::Point2f> A vector of 4 points representing the corners of the rectangle.
 */
std::vector<cv::Point2f> computeWallVertices(float x0, float y0, float width, float height, float shear_amount)
{
    std::vector<cv::Point2f> rect_vertices;

    // Top-left corner after applying shear
    rect_vertices.push_back(cv::Point2f(x0 + height * shear_amount, y0 + height));

    // Top-right corner after applying shear
    rect_vertices.push_back(cv::Point2f(x0 + height * shear_amount + width, y0 + height));

    // Bottom-right corner
    rect_vertices.push_back(cv::Point2f(x0 + width, y0));

    // Bottom-left corner
    rect_vertices.push_back(cv::Point2f(x0, y0));

    return rect_vertices;
}

/**
 * @brief Computes the homography matrix based on control points and wall image vertices.
 *
 * This function calculates the homography matrix that maps points from the source image (wall images)
 * to the destination image (control points). The homography matrix is stored in the global variable H.
 *
 * Control points are specified in normalized coordinates and are fetched from the global variable cpParam.
 * Wall image vertices are calculated based on the dimensions and spacing of the maze walls.
 *
 * @note This function uses the OpenCV library to compute the homography matrix.
 * @note The global variables cpParam, MAZE_SIZE, and wallSpace are used to control the behavior of this function.
 *
 * @warning Make sure that the number of control points and wall image vertices are the same and that they are ordered correspondingly.
 */
void computeHomography()
{
    // Get the corner/vertex values for each of the control points
    std::vector<cv::Point2f> cp_vertices;
    cp_vertices.push_back(cv::Point2f(cpParam[0][0], cpParam[0][1]));
    cp_vertices.push_back(cv::Point2f(cpParam[1][0], cpParam[1][1]));
    cp_vertices.push_back(cv::Point2f(cpParam[2][0], cpParam[2][1]));
    cp_vertices.push_back(cv::Point2f(cpParam[3][0], cpParam[3][1]));

    // Get the corner/vertex values for each of the wall images
    std::vector<cv::Point2f> img_vertices;
    img_vertices = computeWallVertices(0.0f, 0.0f, (float(MAZE_SIZE) - 1) * wallSpace, (float(MAZE_SIZE) - 1) * wallSpace, 0);

    // Compute the homography matrix
    H = findHomography(img_vertices, cp_vertices);
}

/**
 * @brief Used to reset control point parameter list.
 */
void resetParamCP()
{
    // Copy the default array to the dynamic one
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 5; ++j)
        {
            cpParam[i][j] = cpParamDefault[i][j];
        }
    }
}

/**
 * @brief Loads control point parameters and homography matrix from an XML file.
 * 
 * This function reads an XML file specified by the global variable `configPath` to load 
 * configuration data for control points (stored in cpParam) and the homography matrix (stored in H).
 * 
 * The XML file is expected to have a specific structure with elements labeled "cpParam" and "H".
 * Each of these elements should contain nested "Row" and "Cell" elements to represent the matrix data.
 * 
 * @note This function uses the pugiXML library to parse the XML file.
 * @note The global variables `configPath`, `cpParam`, and `H` are used in this function.
 * 
 * @warning This function will log an error message and return if the XML file cannot be loaded.
 */
void loadCoordinatesXML()
{
    // Get file name from path
    std::string file_name = configPath.substr(configPath.find_last_of('/') + 1);

    // Create an XML document object
    pugi::xml_document doc;
    if (!doc.load_file(configPath.c_str()))
    {
        ROS_ERROR("LOAD XML: Could Not Load XML: File[%s]", file_name.c_str());
        return;
    }

    // Retrieve cpParam
    std::vector<std::vector<float>> cpParam2;
    pugi::xml_node cpParamNode = doc.child("config").child("cpParam");
    for (pugi::xml_node rowNode = cpParamNode.child("Row"); rowNode; rowNode = rowNode.next_sibling("Row"))
    {
        std::vector<float> row;
        for (pugi::xml_node cellNode = rowNode.child("Cell"); cellNode; cellNode = cellNode.next_sibling("Cell"))
        {
            float value = std::stof(cellNode.child_value());
            row.push_back(value);
        }
        cpParam2.push_back(row);
    }

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            cpParam[i][j] = cpParam2[i][j];
        }
    }

    // Retrieve H
    std::vector<std::vector<float>> H2;
    pugi::xml_node HNode = doc.child("config").child("H");
    for (pugi::xml_node rowNode = HNode.child("Row"); rowNode; rowNode = rowNode.next_sibling("Row"))
    {
        std::vector<float> row;
        for (pugi::xml_node cellNode = rowNode.child("Cell"); cellNode; cellNode = cellNode.next_sibling("Cell"))
        {
            float value = std::stof(cellNode.child_value());
            row.push_back(value);
        }
        H2.push_back(row);
    }
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            H.at<float>(i, j) = H2[i][j];
        }
    }
}

/**
 * @brief Loads images from specified file paths and stores their IDs in a reference vector.
 * 
 * This function takes a vector of file paths (`ref_img_paths`) and iteratively loads each image
 * using the DevIL library. The function then stores the ILuint IDs of successfully loaded images 
 * in a reference vector (`ref_image_ids`). 
 * 
 * @param ref_image_ids A reference to a vector of ILuint where the IDs of the loaded images will be stored.
 * @param ref_img_paths A reference to a vector of file paths to the images to be loaded.
 * 
 * @note Utilizes the DevIL image library for image loading operations.
 * 
 * @warning Logs an error message and continues if any image fails to load.
 */
void loadImgTextures(std::vector<ILuint> &ref_image_ids, std::vector<std::string> &ref_img_paths)
{
    // Iterate through img file paths
    for (const std::string &img_path : ref_img_paths)
    {
        ILuint img_id;
        ilGenImages(1, &img_id);
        ilBindImage(img_id);

        // Get width and height of image
        int width = ilGetInteger(IL_IMAGE_WIDTH);
        int height = ilGetInteger(IL_IMAGE_HEIGHT);

        // Get file name from path
        std::string file_name = img_path.substr(img_path.find_last_of('/') + 1);

        // Attempt to load image
        ILboolean success = ilLoadImage(img_path.c_str());
        if (success == IL_TRUE)
        {
            ref_image_ids.push_back(img_id);
            ROS_INFO("DevIL: Loaded image: File[%s]", file_name.c_str());
            ilConvertImage(IL_RGB, IL_UNSIGNED_BYTE);
        }
        else
        {
            ILenum error = ilGetError();
            ilDeleteImages(1, &img_id);
            ROS_ERROR("DevIL: Failed to load image: Error[%s] File[%s]", iluErrorString(error), file_name.c_str());
        }
    }
}

/**
 * @brief Saves the control point positions and homography matrix to an XML file.
 *
 * This function uses the pugixml library to create an XML document and populate it with
 * the control point positions and homography matrix. The control point positions are stored in a 2D array
 * and the homography matrix is stored in a cv::Mat object. Both are saved under their respective
 * XML nodes.
 *
 * @note The XML file is saved to the path specified by the global variable 'configPath'.
 *
 * Example XML structure:
 * @code
 * <config>
 *   <cpParam>
 *     <Row>
 *       <Cell>value</Cell>
 *       ...
 *     </Row>
 *     ...
 *   </cpParam>
 *   <H>
 *     <Row>
 *       <Cell>value</Cell>
 *       ...
 *     </Row>
 *     ...
 *   </H>
 * </config>
 * @endcode
 *
 * @return void
 */
void saveCoordinatesXML()
{
    // Create an XML document object
    pugi::xml_document doc;

    // Create the root element "config"
    pugi::xml_node root = doc.append_child("config");

    // Create a child node for storing control point positions
    pugi::xml_node arrayNode = root.append_child("cpParam");

    // Iterate over the rows of the 2D array 'cpParam'
    for (const auto &row : cpParam)
    {
        // Create a row element under "cpParam"
        pugi::xml_node rowNode = arrayNode.append_child("Row");

        // Iterate over the elements in the row
        for (const auto &value : row)
        {
            // Create a cell element under the row
            pugi::xml_node cellNode = rowNode.append_child("Cell");
            cellNode.append_child(pugi::node_pcdata).set_value(std::to_string(value).c_str());
        }
    }

    // Create a 2D array to store the homography matrix
    float array2[3][3];

    // Copy data from cv::Mat 'H' to the 2D array 'array2'
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            array2[i][j] = H.at<float>(i, j);
        }
    }

    // Create a child node for storing the homography matrix
    pugi::xml_node arrayNode2 = root.append_child("H");

    // Iterate over the rows of the 2D array 'array2'
    for (const auto &row : array2)
    {
        // Create a row element under "H"
        pugi::xml_node rowNode = arrayNode2.append_child("Row");

        // Iterate over the elements in the row
        for (const auto &value : row)
        {
            // Create a cell element under the row
            pugi::xml_node cellNode = rowNode.append_child("Cell");
            cellNode.append_child(pugi::node_pcdata).set_value(std::to_string(value).c_str());
        }
    }

    // Save the XML document to a file specified by 'configPath'
    if (doc.save_file(configPath.c_str()))
    {
        ROS_INFO("SAVE XML: File Saved Successfully: Path[%s]", configPath.c_str());
    }
    else
    {
        ROS_ERROR("SAVE XML: Failed to Save XML: Path[%s]", configPath.c_str());
    }
}

/**
 * @brief  Entry point for the projection_calibration_node ROS node.
 *
 * This program initializes ROS, DevIL, and GLFW, and then enters a main loop
 * to handle image projection and calibration tasks.
 *
 * @param  argc  Number of command-line arguments.
 * @param  argv  Array of command-line arguments.
 *
 * @return 0 on successful execution, -1 on failure.
 */
int main(int argc, char **argv)
{
    // _______________ SETUP _______________

    // ROS Initialization
    ros::init(argc, argv, "projection_calibration_node", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    ROS_INFO("RUNNING MAIN");

    // Log paths for debugging
    ROS_INFO("SETTINGS: Package Path: %s", packagePath.c_str());
    ROS_INFO("SETTINGS: Config XML Path: %s", configPath.c_str());
    ROS_INFO("SETTINGS: Display: XYLim=[%0.2f,%0.2f] Width=%d Height=%d AR=%0.2f", xy_lim, xy_lim, winWidth, winHeight, winAspectRatio);
    ROS_INFO("SETTINGS: Wall (Norm): Width=%0.2f Space=%0.2f", wallWidth, wallSpace);
    ROS_INFO("SETTINGS: Wall (Pxl): Width=%d Space=%d", (int)(wallWidth * (float)winWidth), (int)(wallSpace * (float)winWidth));

    // Initialize control point parameters
    resetParamCP();

    // Initialize DevIL library
    ilInit();

    // Load images
    loadImgTextures(imgTestIDs, imgTestPaths);
    loadImgTextures(imgMonNumIDs, imgMonNumPaths);
    loadImgTextures(imgModeIDs, imgModePaths);

    // TODO: Check necessity of these lines
    textureImgWidth = ilGetInteger(IL_IMAGE_WIDTH);
    textureImgHeight = ilGetInteger(IL_IMAGE_HEIGHT);

    // Initialize GLFW
    glfwSetErrorCallback(callbackError);
    if (!glfwInit())
    {
        ROS_ERROR("GLFW: Initialization Failed");
        return -1;
    }

    // Create GLFW window
    window = glfwCreateWindow(winWidth, winHeight, windowName.c_str(), NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        ROS_ERROR("GLFW: Create Window Failed");
        return -1;
    }

    // Set OpenGL context and callbacks
    glfwMakeContextCurrent(window);
    gladLoadGL();
    glfwSetKeyCallback(window, callbackKeyBinding);
    glfwSetFramebufferSizeCallback(window, callbackFrameBufferSize);

    // Initialize FBO and attach texture to it
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glGenTextures(1, &fboTexture);
    glBindTexture(GL_TEXTURE_2D, fboTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, winWidth, winHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fboTexture, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Get the list of available monitors and their count
    monitors = glfwGetMonitors(&monitorCount);
    // TEMP: hardcoding for now
    monitorCount = 2;

    // Set the window to the first monitor
    computeHomography();
    changeWindowMonMode();

    // _______________ MAIN LOOP _______________

    while (!glfwWindowShouldClose(window))
    {
        // Clear back buffer for new frame
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw/update wall images
        drawWallsAll();

        // Draw/update control points
        for (int i = 0; i < 4; i++)
        {
            drawControlPoint(cpParam[i][0], cpParam[i][1], cpParam[i][2], cpSelected == i ? cpActiveRGB : cpInactiveRGB);
        }

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();

        // Exit condition
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(window))
            break;
    }

    // _______________ CLEANUP _______________

    // Destroy GLFW window and DevIL images
    glfwDestroyWindow(window);
    for (ILuint imageID : imgTestIDs)
    {
        ilDeleteImages(1, &imageID);
    }

    // Shutdown DevIL
    ilShutDown();

    // Terminate GLFW
    glfwTerminate();

    return 0;
}
