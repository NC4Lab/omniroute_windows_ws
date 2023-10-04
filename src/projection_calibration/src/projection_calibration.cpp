// ######################################

// ===== projection_calibration.cpp =====

// ######################################

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

void loadCoordinates()
{
    pugi::xml_document doc;
    if (!doc.load_file(configPath.c_str()))
    {
        std::cout << "Failed to load XML file." << std::endl;
        return;
    }

    // Retrieve squarePositions
    std::vector<std::vector<float>> squarePositions2;
    pugi::xml_node squarePositionsNode = doc.child("config").child("squarePositions");
    for (pugi::xml_node rowNode = squarePositionsNode.child("Row"); rowNode; rowNode = rowNode.next_sibling("Row"))
    {
        std::vector<float> row;
        for (pugi::xml_node cellNode = rowNode.child("Cell"); cellNode; cellNode = cellNode.next_sibling("Cell"))
        {
            float value = std::stof(cellNode.child_value());
            row.push_back(value);
        }
        squarePositions2.push_back(row);
    }

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            squarePositions[i][j] = squarePositions2[i][j];
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

void saveCoordinates()
{

    pugi::xml_document doc;
    std::cerr << "doc created";
    // Create the root element
    pugi::xml_node root = doc.append_child("config");

    pugi::xml_node arrayNode = root.append_child("squarePositions");

    // Iterate over the rows of the 2D array
    for (const auto &row : squarePositions)
    {
        // Create a row element
        pugi::xml_node rowNode = arrayNode.append_child("Row");

        // Iterate over the elements in the row
        for (const auto &value : row)
        {
            // Create a cell element
            pugi::xml_node cellNode = rowNode.append_child("Cell");
            cellNode.append_child(pugi::node_pcdata).set_value(std::to_string(value).c_str());
        }
    }

    std::cerr << "created squaresP";

    float array2[3][3];

    // Copy data from cv::Mat to the 2D array
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            array2[i][j] = H.at<float>(i, j);
        }
    }

    pugi::xml_node arrayNode2 = root.append_child("H");

    // Iterate over the rows of the 2D array
    for (const auto &row : array2)
    {
        // Create a row element
        pugi::xml_node rowNode = arrayNode2.append_child("Row");

        // Iterate over the elements in the row
        for (const auto &value : row)
        {
            // Create a cell element
            pugi::xml_node cellNode = rowNode.append_child("Cell");
            cellNode.append_child(pugi::node_pcdata).set_value(std::to_string(value).c_str());
        }
    }

    std::cerr << "created H";
    // Save the XML document to a file
    if (doc.save_file(configPath.c_str()))
    {
        std::cout << "XML file saved successfully." << std::endl;
    }
    else
    {
        std::cout << "Failed to save XML file." << std::endl;
    }
}

void computeHomography()
{
    std::vector<cv::Point2f> targetCorners;
    std::vector<cv::Point2f> imageCorners;
    // hard coding the specific corners for each of the squares.
    targetCorners.push_back(cv::Point2f(squarePositions[0][0], squarePositions[0][1]));
    targetCorners.push_back(cv::Point2f(squarePositions[1][0], squarePositions[1][1]));
    targetCorners.push_back(cv::Point2f(squarePositions[2][0], squarePositions[2][1]));
    targetCorners.push_back(cv::Point2f(squarePositions[3][0], squarePositions[3][1]));
    imageCorners = createRectPoints(0.0f, 0.0f, (float(MAZE_SIZE) - 1) * wallSep, (float(MAZE_SIZE) - 1) * wallSep, 0);

    H = findHomography(imageCorners, targetCorners);
    // H = findHomography(targetCorners, imageCorners);

    // std::cerr << H;
}

/// @ref: GLFW/glfw3.h for keybindings enum
void callbackKeyBinding(GLFWwindow *window, int key, int scancode, int action, int mods)
{

    glfwMakeContextCurrent(window);

    // Any key release action
    if (action == GLFW_RELEASE)
    {
        // ---------- Target selector keys [1-4] ----------

        // Top-left square
        if (key == GLFW_KEY_1)
        {
            selectedSquare = 0;
        }

        // Top-right square
        else if (key == GLFW_KEY_2)
        {
            selectedSquare = 1;
        }

        // Bottom-right square
        else if (key == GLFW_KEY_3)
        {
            selectedSquare = 2;
        }

        // Bottom-left square
        else if (key == GLFW_KEY_4)
        {
            selectedSquare = 3;
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

        // Square position [up, down, left, right]
        else if (key == GLFW_KEY_P)
        {
            changeMode = "pos";
        }

        // Square height [up, down]
        else if (key == GLFW_KEY_D)
        {
            changeMode = "dimensions";
        }

        // Square shear [up, down]
        else if (key == GLFW_KEY_S)
        {
            changeMode = "shear";
        }

        // ---------- CSV Handling [ENTER, L] ----------

        // Save coordinates to CSV
        else if (key == GLFW_KEY_ENTER)
        {
            ROS_INFO("save hit");
            saveCoordinates();
        }

        // Load coordinates from CSV
        else if (key == GLFW_KEY_L)
        {
            loadCoordinates();
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

        if (changeMode == "pos")
        {

            // Listen for arrow key input to move selected square
            if (key == GLFW_KEY_LEFT)
            {
                squarePositions[selectedSquare][0] -= 0.05f;
            }
            else if (key == GLFW_KEY_RIGHT)
            {
                squarePositions[selectedSquare][0] += 0.05f;
            }
            else if (key == GLFW_KEY_UP)
            {
                squarePositions[selectedSquare][1] += 0.05f;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                squarePositions[selectedSquare][1] -= 0.05f;
            }
        }

        if (changeMode == "dimensions")
        {
            if (key == GLFW_KEY_UP)
            {
                squarePositions[selectedSquare][3] += 0.001f;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                squarePositions[selectedSquare][3] -= 0.001f;
            }
        }

        if (changeMode == "shear")
        {
            if (key == GLFW_KEY_UP)
            {
                squarePositions[selectedSquare][4] += 0.05f;
            }
            else if (key == GLFW_KEY_DOWN)
            {
                squarePositions[selectedSquare][4] -= 0.05f;
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

void drawTarget(float x, float y, float targetWidth, float targetHeight)
{
    glBegin(GL_QUADS);

    // in clockwise direction, starting from left bottom
    glVertex2f(x, y);
    glVertex2f(x, y + targetHeight);

    glVertex2f(x + targetWidth, y + targetHeight);
    glVertex2f(x + targetWidth, y);

    glEnd();
}

void drawRect(std::vector<cv::Point2f> corners, int imageNumber)
{
    glBegin(GL_QUADS);

    // glTexCoord2f(x, y);
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(corners[0].x, corners[0].y);

    // glTexCoord2f(x+texWidth, y);
    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(corners[1].x, corners[1].y);
    // glVertex2f(x+width, y);

    // glTexCoord2f(x+texWidth, y+texHeight);
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(corners[2].x, corners[2].y);
    // glVertex2f(x+width, y+height);

    // glTexCoord2f(x, y+texHeight);
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(corners[3].x, corners[3].y);
    // glVertex2f(x, y+height);
    glEnd();
}

void drawWalls()
{

    float shear4 = squarePositions[3][4];
    float shear3 = squarePositions[2][4];
    float shear1 = squarePositions[0][4];

    float height4 = squarePositions[3][3];
    float height3 = squarePositions[2][3];
    float height1 = squarePositions[0][3];

    // Enable texture mapping
    glEnable(GL_TEXTURE_2D);
    for (float i = 0; i < MAZE_SIZE; i++)
    {
        ilBindImage(imageIDs[imageNumber]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ilGetInteger(IL_IMAGE_WIDTH),
                     ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGB,
                     GL_UNSIGNED_BYTE, ilGetData());
        glEnable(GL_TEXTURE_2D);
        for (float j = 0; j < MAZE_SIZE; j++)
        {
            glBindTexture(GL_TEXTURE_2D, fboTexture);

            shearAmount = shear4 + (i / (float(MAZE_SIZE) - 1)) * (shear3 - shear4) + (j / (float(MAZE_SIZE) - 1)) * (shear1 - shear4);
            float heightAmount = height4 + (i / float(MAZE_SIZE) - 1) * (height3 - height4) + (j / (float(MAZE_SIZE) - 1)) * (height1 - height4);
            std::vector<cv::Point2f> c = createRectPoints(0.0f, 0.0f, wallWidth, heightAmount, shearAmount);

            for (auto it = c.begin(); it != c.end(); it++)
            {
                cv::Point2f p = *it;

                p.x += i * wallSep;
                p.y += j * wallSep;

                // p.x += shearAmount * p.y;
                float data[] = {p.x, p.y, 1}; // apparently that's how it is

                cv::Mat ptMat(3, 1, CV_32F, data);
                cv::Mat ptMat2(3, 1, CV_32F, data);

                H.convertTo(H, ptMat.type());

                // warpPerspective(ptMat2, ptMat, H, ptMat.size());

                ptMat = H * ptMat;

                ptMat /= ptMat.at<float>(2);

                // std::cerr << "\n" << ptMat;

                it->x = ptMat.at<float>(0, 0);
                it->y = ptMat.at<float>(0, 1);
            }

            drawRect(c, i);
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

        // Draw squares with updated positions
        glClear(GL_COLOR_BUFFER_BIT);
        //// Load image data into texture

        drawWalls();

        for (int i = 0; i < 4; i++)
        {
            drawTarget(squarePositions[i][0], squarePositions[i][1], squarePositions[i][2], squarePositions[i][3]);
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