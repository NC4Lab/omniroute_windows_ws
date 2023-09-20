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
    packagePath + "/src/tj.bmp",
    packagePath + "/src/mmCarribean.png",
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