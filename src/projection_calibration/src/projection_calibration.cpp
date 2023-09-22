// ######################################

// ===== projection_calibration.cpp =====

// ######################################

//============= INCLUDE ================
#include "projection_calibration.h"

// ============= VARIABLES =============

// Constants
const int MAZE_SIZE = 3;

// Variables related to square positions and transformation
int imageNumber = 0;
float squarePositions[4][5] = {
    {-0.8f, 0.8f, 0.02f, 0.02f, 0.0f}, // top-left square
    {0.8f, 0.8f, 0.02f, 0.02f, 0.0f},  // top-right square
    {0.8f, -0.8f, 0.02f, 0.02f, 0.0f}, // bottom-right square
    {-0.8f, -0.8f, 0.02f, 0.02f, 0.0f} // bottom-left square
};
float shearValues[MAZE_SIZE][MAZE_SIZE];
float sizeValues[MAZE_SIZE][MAZE_SIZE];
float configurationValues[3][3][3];
cv::Mat H = cv::Mat::eye(3, 3, CV_32F);
int selectedSquare = 0;

// Variables related to wall properties
float wallWidth = 0.02f;
float wallHeight = 0.02f;
float wallSep = 0.05f;
string changeMode = "pos";
float shearAmount = 0.0f;
vector<cv::Point2f> wallCorners = createRectPoints(0.0f, 0.0f, wallWidth, wallHeight, 0);

// Variables related to image and file paths
string packagePath = ros::package::getPath("projection_calibration");
string configPath;
string windowName;

// List of image file paths
std::vector<std::string> imagePaths = {
    packagePath + "/img/tj.bmp",
    packagePath + "/img/mmCarribean.png",
    // Add more image file paths as needed
};

// Container to hold the loaded images
std::vector<ILuint> imageIDs;

// Variables related to window and OpenGL
int winWidth = 3840;
int winHeight = 2160;
GLFWwindow *window;
GLuint fbo;
GLuint fboTexture;
GLFWmonitor *monitor = NULL;
int monitorNumber = 0;
GLFWmonitor **monitors;
int monitor_count;

ILint texWidth;
ILint texHeight;

// ============= METHODS =============

vector<cv::Point2f> createRectPoints(float x0, float y0, float width, float height, float shearAmount)
{
    vector<cv::Point2f> rectPoints;
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
    for (int i; i < 3; i++)
    {
        for (int j; j < 3; j++)
        {
            H.at<float>(i, j) = H2[i][j];
        }
    }
}


void saveCoordinates()
{

    pugi::xml_document doc;
    cerr << "doc created";
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

    cerr << "created squaresP";

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

    cerr << "created H";
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
    vector<cv::Point2f> targetCorners;
    vector<cv::Point2f> imageCorners;
    // hard coding the specific corners for each of the squares.
    targetCorners.push_back(cv::Point2f(squarePositions[0][0], squarePositions[0][1]));
    targetCorners.push_back(cv::Point2f(squarePositions[1][0], squarePositions[1][1]));
    targetCorners.push_back(cv::Point2f(squarePositions[2][0], squarePositions[2][1]));
    targetCorners.push_back(cv::Point2f(squarePositions[3][0], squarePositions[3][1]));
    imageCorners = createRectPoints(0.0f, 0.0f, (float(MAZE_SIZE) - 1) * wallSep, (float(MAZE_SIZE) - 1) * wallSep, 0);

    H = findHomography(imageCorners, targetCorners);
    // H = findHomography(targetCorners, imageCorners);

    // cerr << H;
}


/// @ref: GLFW/glfw3.h for keybindings enum
void callbackKeyBinding(GLFWwindow *window, int key, int scancode, int action, int mods)
{

    glfwMakeContextCurrent(window);

    // Any key release action
    // Any key release action
    if (action == GLFW_RELEASE)
    {
        // Image selector keys [1-4]

        // Top-left square
        // Image selector keys [1-4]

        // Top-left square
        if (key == GLFW_KEY_1)
        {
            selectedSquare = 0;
        }
        // Top-right square
        // Top-right square
        else if (key == GLFW_KEY_2)
        {
            selectedSquare = 1;
        }
        // Bottom-right square
        // Bottom-right square
        else if (key == GLFW_KEY_3)
        {
            selectedSquare = 2;
        }
        // Bottom-left square
        // Bottom-left square
        else if (key == GLFW_KEY_4)
        {
            selectedSquare = 3;
        }

        // Save coordinates to CSV

        // Save coordinates to CSV
        else if (key == GLFW_KEY_ENTER)
        {
            ROS_ERROR("save hit");
            saveCoordinates();
        }

        // Set image to image 1

        // Set image to image 1
        else if (key == GLFW_KEY_C){
            imageNumber = 1;
        }
        // Set image to image 2
        // Set image to image 2
        else if (key == GLFW_KEY_T){
            imageNumber = 0;
        }

        // Change mode keys [P, D, S]

        // Square position [up, down, left, right]
        else if (key == GLFW_KEY_P)
        {
            changeMode = "pos";
        }
        // Square height [up, down]
        /// @note: can only select squares 1-3

        // Change mode keys [P, D, S]

        // Square position [up, down, left, right]
        else if (key == GLFW_KEY_P)
        {
            changeMode = "pos";
        }
        // Square height [up, down]
        /// @note: can only select squares 1-3
        else if (key == GLFW_KEY_D)
        {
            changeMode = "dimensions";
        }
        // Square shear [up, down]
        /// @note: can only select squares 1-3
        // Square shear [up, down]
        /// @note: can only select squares 1-3
        else if (key == GLFW_KEY_S)
        {
            changeMode = "shear";
        }
        // Load coordinates from CSV
        // Load coordinates from CSV
        else if (key == GLFW_KEY_L){
            loadCoordinates();
        }

        else if (key == GLFW_KEY_F)
        {
            monitors = glfwGetMonitors(&monitor_count);
            // GLFWmonitor* monitor = glfwGetWindowMonitor(window);
            // const GLFWvidmode* mode = glfwGetVideoMode(monitor);

            //// Create a window
            ////GLFWwindow* window = glfwCreateWindow(mode->width, mode->height, "My Window", monitor, NULL);

            //// Make the window fullscreen
            ////int count;
            ////GLFWmonitor** monitors = glfwGetMonitors(&count);
            // glfwSetWindowMonitor(window, NULL, 0, 0, mode->width, mode->height, mode->refreshRate);

            // find the second monitor (index 1) by checking its position
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

            // make the window full screen on the second monitor
            if (monitor)
            {
                const GLFWvidmode *mode = glfwGetVideoMode(monitor);
                glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            }
        }
        else if (key == GLFW_KEY_M)
        {
            ROS_ERROR(windowName.c_str()); // this should be showing something in the terminal, but isn't atm
            monitors = glfwGetMonitors(&monitor_count);
            monitorNumber++;
            monitor = monitors[monitorNumber % monitor_count];
            if (monitor)
            {
                const GLFWvidmode *mode = glfwGetVideoMode(monitor);
                glfwSetWindowAttrib(window, GLFW_FOCUS_ON_SHOW, GL_TRUE);
                // glfwSetWindowAttrib(window, GLFW_RESIZABLE,	GLFW_TRUE);
                // glfwSetWindowAttrib(window, GLFW_CONTEXT_RELEASE_BEHAVIOR,	GLFW_CONTEXT_RELEASE_BEHAVIOR_NONE);
                glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            }
        }
    }
    else if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        if (key == GLFW_KEY_ENTER)
        {
            cerr << "stuffies";
            ROS_ERROR("BRO DOES THIS WORK?");
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


void drawRect(vector<cv::Point2f> corners, int imageNumber)
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
            vector<cv::Point2f> c = createRectPoints(0.0f, 0.0f, wallWidth, heightAmount, shearAmount);

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

                // cerr << "\n" << ptMat;

                it->x = ptMat.at<float>(0, 0);
                it->y = ptMat.at<float>(0, 1);
            }

            drawRect(c, i);
        }
    }
}


int main(int argc, char **argv)
{
    return 0;
}