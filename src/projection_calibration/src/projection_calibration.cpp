/**
 * @file projection_calibration.cpp
 * @brief This file contains the implementation for the projection calibration.
 */

//============= INCLUDE ================
#include "projection_calibration.h"

std::vector<cv::Point2f> createRectPoints(float x0, float y0, float width, float height, float shearAmount)
{
    std::vector<cv::Point2f> rectPoints;
    rectPoints.push_back(cv::Point2f(x0 + height * shearAmount, y0 + height));
    rectPoints.push_back(cv::Point2f(x0 + height * shearAmount + width, y0 + height));
    rectPoints.push_back(cv::Point2f(x0 + width, y0));
    rectPoints.push_back(cv::Point2f(x0, y0));

    return rectPoints;
}

void loadCoordinatesXML()
{
    pugi::xml_document doc;
    if (!doc.load_file(configPath.c_str()))
    {
        ROS_ERROR("Failed to load XML file.");
        return;
    }

    // Retrieve cpPositions
    std::vector<std::vector<float>> cpPositions2;
    pugi::xml_node cpPositionsNode = doc.child("config").child("cpPositions");
    for (pugi::xml_node rowNode = cpPositionsNode.child("Row"); rowNode; rowNode = rowNode.next_sibling("Row"))
    {
        std::vector<float> row;
        for (pugi::xml_node cellNode = rowNode.child("Cell"); cellNode; cellNode = cellNode.next_sibling("Cell"))
        {
            float value = std::stof(cellNode.child_value());
            row.push_back(value);
        }
        cpPositions2.push_back(row);
    }

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            cpPositions[i][j] = cpPositions2[i][j];
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
 *   <cpPositions>
 *     <Row>
 *       <Cell>value</Cell>
 *       ...
 *     </Row>
 *     ...
 *   </cpPositions>
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
    pugi::xml_node arrayNode = root.append_child("cpPositions");

    // Iterate over the rows of the 2D array 'cpPositions'
    for (const auto &row : cpPositions)
    {
        // Create a row element under "cpPositions"
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
        ROS_INFO("XML file saved successfully.");
    }
    else
    {
        ROS_ERROR("Failed to save XML file.");
    }
}


void computeHomography()
{
    std::vector<cv::Point2f> targetCorners;
    std::vector<cv::Point2f> imageCorners;
    // hard coding the specific corners for each of the control points.
    targetCorners.push_back(cv::Point2f(cpPositions[0][0], cpPositions[0][1]));
    targetCorners.push_back(cv::Point2f(cpPositions[1][0], cpPositions[1][1]));
    targetCorners.push_back(cv::Point2f(cpPositions[2][0], cpPositions[2][1]));
    targetCorners.push_back(cv::Point2f(cpPositions[3][0], cpPositions[3][1]));
    imageCorners = createRectPoints(0.0f, 0.0f, (float(MAZE_SIZE) - 1) * wallSep, (float(MAZE_SIZE) - 1) * wallSep, 0);

    H = findHomography(imageCorners, targetCorners);
    // H = findHomography(targetCorners, imageCorners);

    // std::cerr << H;
}

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
 * - [C, T]: Set image to image 1 or image 2
 * - [P, D, S]: Change mode (control point position, control point dimensions, control point shear)
 * - [ENTER]: Save coordinates to XML
 * - [L]: Load coordinates from XML
 * - [F]: Fullscreen on second monitor
 * - [M]: Move window to next monitor
 * - [Arrow Keys]: Move selected control point or adjust dimensions/shear
 */
void callbackKeyBinding(GLFWwindow *window, int key, int scancode, int action, int mods)
{

    glfwMakeContextCurrent(window);

    // Any key release action
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

        // ---------- Image selector keys [C, T] ----------

        // Set image to image 1
        else if (key == GLFW_KEY_C)
        {
            imageNumber = 1;
        }

        // Set image to image 2
        else if (key == GLFW_KEY_T)
        {
            imageNumber = 0;
        }

        // ---------- Change mode keys [P, D, S] ----------

        // Control point position [up, down, left, right]
        else if (key == GLFW_KEY_P)
        {
            cpModMode = "pos";
        }

        // Control point height [up, down]
        else if (key == GLFW_KEY_D)
        {
            cpModMode = "dimensions";
        }

        // Control point shear [up, down]
        else if (key == GLFW_KEY_S)
        {
            cpModMode = "shear";
        }

        // ---------- XML Handling [ENTER, L] ----------

        // Save coordinates to XML
        else if (key == GLFW_KEY_ENTER)
        {
            ROS_INFO("save hit");
            saveCoordinatesXML();
        }

        // Load coordinates from XML
        else if (key == GLFW_KEY_L)
        {
            loadCoordinatesXML();
        }

        // ---------- Monitor handling [F, M] ----------

        // Fullscreen on second monitor
        else if (key == GLFW_KEY_F)
        {
            monitors = glfwGetMonitors(&monitor_count);

            // Find the second monitor (index 1) by checking its position
            for (int i = 0; i < monitor_count; i++)
            {
                const GLFWvidmode *mode = glfwGetVideoMode(monitors[i]);
                int monitor_x, monitor_y;
                glfwGetMonitorPos(monitors[i], &monitor_x, &monitor_y);
                if (monitor_x != 0 || monitor_y != 0)
                {
                    monitorNumber = i;
                    monitor = monitors[i];
                    break;
                }
            }

            // Make the window full screen on the second monitor
            if (monitor)
            {
                const GLFWvidmode *mode = glfwGetVideoMode(monitor);
                glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            }
        }

        // Move window to next monitor
        else if (key == GLFW_KEY_M)
        {
            ROS_INFO(windowName.c_str()); // this should be showing something in the terminal, but isn't atm
            monitors = glfwGetMonitors(&monitor_count);
            monitorNumber++;
            monitor = monitors[monitorNumber % monitor_count];
            if (monitor)
            {
                const GLFWvidmode *mode = glfwGetVideoMode(monitor);
                glfwSetWindowAttrib(window, GLFW_FOCUS_ON_SHOW, GL_TRUE);
                glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            }
        }
    }

    // Any key press or repeat action
    else if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        if (key == GLFW_KEY_ENTER)
        {
            std::cerr << "stuffies";
            ROS_INFO("BRO DOES THIS WORK?");
        }

        if (cpModMode == "pos")
        {

            // Listen for arrow key input to move selected control point
            if (key == GLFW_KEY_LEFT)
            {
                cpPositions[cpSelected][0] -= 0.05f;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                cpPositions[cpSelected][0] += 0.05f;
            }
            else if (key == GLFW_KEY_UP)
            {
                cpPositions[cpSelected][1] += 0.05f;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                cpPositions[cpSelected][1] -= 0.05f;
            }
        }

        if (cpModMode == "dimensions")
        {
            if (key == GLFW_KEY_UP)
            {
                cpPositions[cpSelected][3] += 0.001f;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                cpPositions[cpSelected][3] -= 0.001f;
            }
        }

        if (cpModMode == "shear")
        {
            if (key == GLFW_KEY_UP)
            {
                cpPositions[cpSelected][4] += 0.05f;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                cpPositions[cpSelected][4] -= 0.05f;
            }
        }
    }

    computeHomography();
}

void callbackFrameBufferSize(GLFWwindow *window, int width, int height)
{
    glViewport(0, 0, width, height);
}

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
 * @param cp_width The width of the control point.
 * @param cp_height The height of the control point.
 */
void drawControlPoint(float x, float y, float cp_width, float cp_height)
{
    // Begin drawing a quadrilateral
    glBegin(GL_QUADS);

    // Define the vertices of the quadrilateral in a clockwise direction
    // starting from the bottom-left corner
    glVertex2f(x, y);                  // Bottom-left corner
    glVertex2f(x, y + cp_height);      // Top-left corner
    glVertex2f(x + cp_width, y + cp_height); // Top-right corner
    glVertex2f(x + cp_width, y);       // Bottom-right corner

    // End drawing
    glEnd();
}


/**
 * @brief Draws a textured wall using OpenGL.
 * 
 * @param corners Vector of corner points for the wall.
 * @param imageNumber Index of the texture image to use.
 */
void drawWall(std::vector<cv::Point2f> corners, int imageNumber)
{
    // Start drawing a quadrilateral
    glBegin(GL_QUADS);

    // Set texture and vertex coordinates for each corner
    // Bottom-left corner
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(corners[0].x, corners[0].y);

    // Bottom-right corner
    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(corners[1].x, corners[1].y);

    // Top-right corner
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(corners[2].x, corners[2].y);

    // Top-left corner
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(corners[3].x, corners[3].y);

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
    float shear4 = cpPositions[3][4];
    float shear3 = cpPositions[2][4];
    float shear1 = cpPositions[0][4];
    float height4 = cpPositions[3][3];
    float height3 = cpPositions[2][3];
    float height1 = cpPositions[0][3];

    // Enable OpenGL texture mapping
    glEnable(GL_TEXTURE_2D);

    // Iterate through the maze grid
    for (float i = 0; i < MAZE_SIZE; i++)
    {
        // Bind and set texture image
        ilBindImage(imageIDs[imageNumber]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
                     ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB,
                     GL_UNSIGNED_BYTE, ilGetData());

        // Iterate through each cell in the maze row
        for (float j = 0; j < MAZE_SIZE; j++)
        {
            // Bind texture to framebuffer object
            glBindTexture(GL_TEXTURE_2D, fboTexture);

            // Calculate shear and height for the current wall
            shearAmount = shear4 + (i / (MAZE_SIZE - 1)) * (shear3 - shear4) +
                          (j / (MAZE_SIZE - 1)) * (shear1 - shear4);
            float heightAmount = height4 + (i / (MAZE_SIZE - 1)) * (height3 - height4) +
                                 (j / (MAZE_SIZE - 1)) * (height1 - height4);

            // Create wall vertices
            std::vector<cv::Point2f> vertices = createRectPoints(0.0f, 0.0f, wallWidth, heightAmount, shearAmount);

            // Apply perspective warping to vertices
            for (auto& p : vertices)
            {
                // Update vertex positions based on shear and height
                p.x += i * wallSep;
                p.y += j * wallSep;

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
            drawWall(vertices, i);
        }
    }
}


int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "projection_calibration_node", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    ROS_INFO("Running: main()");

    // Print paths
    ROS_INFO("packagePath: %s", packagePath.c_str());
    ROS_INFO("workspacePath: %s", workspacePath.c_str());
    ROS_INFO("imgPath: %s", imgPath.c_str());
    ROS_INFO("configPath: %s", configPath.c_str());

    std::string tempPath, tempName;

    nh.param<std::string>("configPath", tempPath, "");
    nh.param<std::string>("windowName", tempName, "");

    configPath = tempPath.c_str();
    windowName = tempName.c_str();

    ROS_INFO("config path is: %s", configPath.c_str());

    // Initialize DevIL
    ilInit();

    for (const std::string &imagePath : imagePaths)
    {
        // Generate a new DevIL image ID
        ILuint imageID;
        ilGenImages(1, &imageID);
        ilBindImage(imageID);

        // Load the image file
        ILboolean success = ilLoadImage(imagePath.c_str());
        if (success == IL_TRUE)
        {
            // Image loaded successfully
            imageIDs.push_back(imageID);
            ROS_INFO(imagePath.c_str());

            ROS_INFO("Loading image: %s", iluErrorString(ilGetError()));

            ilConvertImage(IL_RGB, IL_UNSIGNED_BYTE);

            ROS_INFO("Converting image: %s", iluErrorString(ilGetError()));
        }
        else
        {
            // Failed to load the image
            ILenum error = ilGetError();

            // Handle the error as needed
            ilDeleteImages(1, &imageID); // Clean up the image ID

            ROS_ERROR("DevIL: Failed to load image: %s", iluErrorString(error));
        }
    }

    texWidth = ilGetInteger(IL_IMAGE_WIDTH);
    texHeight = ilGetInteger(IL_IMAGE_HEIGHT);

    // ROS_INFO("window name is: %s", windowName);

    ROS_INFO("%d", texWidth);
    ROS_INFO("%d", texHeight);

    glfwSetErrorCallback(callbackError);

    if (!glfwInit())
    {
        ROS_ERROR("glfw init issue");
        return -1;
    }
    // Create a window with a 4K resolution (3840x2160)
    window = glfwCreateWindow(winWidth, winHeight, windowName.c_str(), NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        ROS_ERROR("GLFW Create Window Failed");
        return -1;
    }

    // Set the window as the current OpenGL context
    glfwMakeContextCurrent(window);
    ROS_INFO("window ran");

    gladLoadGL();
    glfwSwapInterval(1);
    glfwSetKeyCallback(window, callbackKeyBinding);

    // Set the window resize callback
    glfwSetFramebufferSizeCallback(window, callbackFrameBufferSize);

    // Create an FBO and attach the texture to it
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    glGenTextures(1, &fboTexture);
    glBindTexture(GL_TEXTURE_2D, fboTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, winWidth, winHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fboTexture, 0);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    while (!glfwWindowShouldClose(window))
    {

        // glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        ////glViewport(0, 0, winWidth, winHeight);

        //// Clear the FBO

        glEnable(GL_TEXTURE_2D);

        // Draw control points with updated positions
        glClear(GL_COLOR_BUFFER_BIT);
        //// Load image data into texture

        drawWallsAll();

        for (int i = 0; i < 4; i++)
        {
            drawControlPoint(cpPositions[i][0], cpPositions[i][1], cpPositions[i][2], cpPositions[i][3]);
        }

        // Swap the buffers
        glfwSwapBuffers(window);
        glfwPollEvents();
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // Exit the loop when the window is closed or escape key is pressed
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(window))
            break;
    }

    glfwDestroyWindow(window);
    for (ILuint imageID : imageIDs)
    {
        ilDeleteImages(1, &imageID);
    }

    // Shutdown DevIL
    ilShutDown();

    // Terminate GLFW
    glfwTerminate();

    return 0;
}