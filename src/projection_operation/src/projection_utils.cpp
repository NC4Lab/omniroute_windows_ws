// ######################################################################################################

// ======================================== projection_utils.cpp ========================================

// ######################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_utils.h"

// ================================================== FUNCTIONS ==================================================

std::string formatCoordinatesFilePathXML(int mon_id_ind, int mode_cal_ind, std::string config_dir_path)
{
    std::string file_path =
        config_dir_path + "/" +
        "cfg_m" + std::to_string(mon_id_ind) + "_c" + std::to_string(mode_cal_ind) +
        ".xml";
    return file_path;
}

int checkQuadVertices(const std::vector<cv::Point2f> &quad_vertices)
{

    // Check if the input vector has exactly 4 vertices
    if (quad_vertices.size() != 4)
        return -1;

    // Check if any three points are collinear; for a valid quadrilateral, no three points should be collinear
    for (int i = 0; i < 4; ++i)
    {
        cv::Point2f p1 = quad_vertices[i];
        cv::Point2f p2 = quad_vertices[(i + 1) % 4];
        cv::Point2f p3 = quad_vertices[(i + 2) % 4];
        float area = p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y);
        if (std::abs(area) < 1e-5)
            return -2; // The points are collinear
    }
    return 0;
}

std::vector<cv::Point2f> quadVertNdc2Pxl(const std::vector<cv::Point2f> &quad_vertices_ndc, int window_width_pxl, int window_height_pxl)
{

    std::vector<cv::Point2f> quad_vertices_pxl;
    quad_vertices_pxl.reserve(quad_vertices_ndc.size());

    for (const auto &vertex : quad_vertices_ndc)
    {
        // Convert x point values from NDC to pixel coordinates
        float x_pixel = (vertex.x + 1.0f) * (window_width_pxl / 2.0f);

        // Convet y points but keep y-axis direction the same as NDC, with origin at the bottom
        float y_pixel = (vertex.y + 1.0f) * (window_height_pxl / 2.0f);

        // // Y is inverted because pixel coordinates increase downwards
        // float y_pixel = (1.0f - vertex.y) * ((float)window_height_pxl / 2.0f);

        // Store values
        quad_vertices_pxl.emplace_back(x_pixel, y_pixel);
    }

    return quad_vertices_pxl;
}

float bilinearInterpolation(float a, float b, float c, float d, int grid_row_i, int grid_col_i, int grid_size)
{
    // Calculate the relative position within the grid.
    float x = static_cast<float>(grid_col_i) / (grid_size - 1);
    float y = static_cast<float>(grid_row_i) / (grid_size - 1);

    // Perform bilinear interpolation using the formula.
    float interp_val = (1 - x) * (1 - y) * a +
                       x * (1 - y) * b +
                       (1 - x) * y * c +
                       x * y * d;

    // Return the final interpolated value.
    return interp_val;
}

int updateHomography(
    const std::array<std::array<cv::Point2f, 4>, 4> &_CP_COORDS,
    std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> &out_WALL_HMAT_DATA)
{
    // Define origin plane vertices
    std::vector<cv::Point2f> source_vertices_pxl = {
        cv::Point2f(0.0f, 0.0f),                                  // Top-left
        cv::Point2f(WALL_IMAGE_WIDTH_PXL, 0.0f),                  // Top-right
        cv::Point2f(WALL_IMAGE_WIDTH_PXL, WALL_IMAGE_HEIGHT_PXL), // Bottom-right
        cv::Point2f(0.0f, WALL_IMAGE_HEIGHT_PXL)};                // Bottom-left

    // // TEMP
    // ROS_INFO("Source Vertices:");
    // dbLogQuadVertices(source_vertices_pxl);

    // Iterate trough grid/wall rows
    for (float gr_i = 0; gr_i < MAZE_SIZE; gr_i++) // image bottom to top
    {
        // Iterate trough grid/wall columns
        for (float gc_i = 0; gc_i < MAZE_SIZE; gc_i++) // image left to right
        {
            std::vector<cv::Point2f> target_vertices_ndc(4);
            for (int p_i = 0; p_i < 4; p_i++)
            {
                // Get the wall vertex values for each maze corner for the interpolation function
                ///@note that y values must be flipped to account for the image origin being in the top-left corner
                cv::Point2f p_a = _CP_COORDS[0][p_i]; // bottom-left interp == top left NDC
                cv::Point2f p_b = _CP_COORDS[1][p_i]; // bottom-right interp == top right NDC
                cv::Point2f p_c = _CP_COORDS[3][p_i]; // top-left interp == bottom left NDC
                cv::Point2f p_d = _CP_COORDS[2][p_i]; // top-right interp == bottom right NDC

                // Get the interpolated vertex x-coordinate
                cv::Point2f p_interp(
                    bilinearInterpolation(p_a.x, p_b.x, p_c.x, p_d.x, gr_i, gc_i, MAZE_SIZE),  // x
                    bilinearInterpolation(p_a.y, p_b.y, p_c.y, p_d.y, gr_i, gc_i, MAZE_SIZE)); // y

                // Store the warped vertex coordinates
                target_vertices_ndc[p_i] = p_interp;
            }

            // Convert to pixel coordinates for OpenCV's findHomography function
            std::vector<cv::Point2f> target_vertices_pxl = quadVertNdc2Pxl(target_vertices_ndc, WINDOW_WIDTH_PXL, WINDOW_HEIGHT_PXL);

            // Check that the target plane vertices are valid
            int resp = checkQuadVertices(target_vertices_pxl);
            if (resp < 0)
            {
                ROS_WARN("[updateHomography] Target Plane Vertices Invalid: Reason[%s]",
                         resp == -1 ? "Wrong Number of Vertices" : "Vertices are Collinear");
                return -1;
            }

            // Use OpenCV's findHomography function to compute the homography matrix
            cv::Mat H = cv::findHomography(source_vertices_pxl, target_vertices_pxl);

            // Check for valid homography matrix
            if (H.empty())
            {
                ROS_ERROR("[updateHomography] Failed to Compute Homography Matrix");
                return -1;
            }

            // Store the homography matrix
            out_WALL_HMAT_DATA[gr_i][gc_i] = H;
        }
    }

    // TEMP
    // dbLogCtrlPointCoordinates(_CP_COORDS);
    // TEMP
    // return -1;

    // Return success
    return 0;
}

bool dbRunDT(int dt_wait)
{
    // Initialize with 0 to return true on the first call
    static ros::Time ts_wait = ros::Time(0);
    ros::Time now = ros::Time::now();

    // Check if ts_wait is set to zero or the current time is past ts_wait
    if (ts_wait == ros::Time(0) || now > ts_wait)
    {
        // Set the next wait timestamp
        ts_wait = now + ros::Duration(0, dt_wait * 1000000); // Convert ms to ns
        return true;
    }

    return false;
}

void dbLogDT(bool do_reset, int line, const char *file_path)
{
    static ros::Time start_time;
    static int line_start = 0;
    static std::string file_name_start = "";

    // Function to extract file name from file path
    auto extractFileName = [](const char *path) -> std::string
    {
        if (path == nullptr)
        {
            return std::string("NULL");
        }
        const char *file_name = strrchr(path, '\\'); // Windows file path separator
        if (!file_name)
            file_name = strrchr(path, '/'); // In case of Unix-style paths
        if (file_name)
        {
            return std::string(file_name + 1); // Skip past the last separator
        }
        return std::string(path); // No separator found, return the whole string
    };

    // Reset start time
    if (do_reset)
    {
        line_start = line;
        file_name_start = extractFileName(file_path);
        start_time = ros::Time::now();
    }
    // Print elapsed time
    else
    {
        ros::Duration elapsed_time = ros::Time::now() - start_time;
        std::string file_name_current = extractFileName(file_path);
        if (line_start == 0 || line == 0)
        {
            ROS_INFO("Elapsed Time: %f milliseconds", elapsed_time.toNSec() / 1e6);
        }
        else
        {
            ROS_INFO("Elapsed Time from %s[%d] to %s[%d]: %0.2f milliseconds",
                     file_name_start.c_str(), line_start, file_name_current.c_str(), line,
                     elapsed_time.toNSec() / 1e6);
        }
    }
}

void dbWaitForInput()
{
    ROS_INFO("Paused for Debugging: Press Enter to Continue...");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

void dbLogQuadVertices(const std::vector<cv::Point2f> &quad_vertices)
{
        if (quad_vertices.size() != 4)
    {
        ROS_ERROR("[dbLogQuadVertices] Invalid number of vertices. Expected 4.");
        return;
    }

    ROS_INFO("         Quad Vertices             ");
    ROS_INFO("===================================");
    ROS_INFO("    |     Left     |     Right    |");

    if (std::any_of(quad_vertices.begin(), quad_vertices.end(), [](const cv::Point2f &point)
                    { return point.x > 10.0f || point.y > 10.0f; }))
    {
        ROS_INFO("-----------------------------------");
        ROS_INFO("    |V0  X , Y     |V1  X , Y     |");
        ROS_INFO("-----------------------------------");
        ROS_INFO("Top | %+5.0f, %+5.0f | %+5.0f, %+5.0f |",
                 quad_vertices[0].x, quad_vertices[0].y, quad_vertices[1].x, quad_vertices[1].y);
        ROS_INFO("-----------------------------------");
        ROS_INFO("    |V3  X , Y     |V2  X , Y     |");
        ROS_INFO("-----------------------------------");
        ROS_INFO("Btm | %+5.0f, %+5.0f | %+5.0f, %+5.0f |",
                 quad_vertices[3].x, quad_vertices[3].y, quad_vertices[2].x, quad_vertices[2].y);
    }
    else
    {
        ROS_INFO("-----------------------------------");
        ROS_INFO("    |V0  X , Y     |V1  X , Y     |");
        ROS_INFO("-----------------------------------");
        ROS_INFO("Top | %+5.2f, %+5.2f | %+5.2f, %+5.2f |",
                 quad_vertices[0].x, quad_vertices[0].y, quad_vertices[1].x, quad_vertices[1].y);
        ROS_INFO("-----------------------------------");
        ROS_INFO("    |V3  X , Y     |V2  X , Y     |");
        ROS_INFO("-----------------------------------");
        ROS_INFO("Btm | %+5.2f, %+5.2f | %+5.2f, %+5.2f |",
                 quad_vertices[3].x, quad_vertices[3].y, quad_vertices[2].x, quad_vertices[2].y);
    }

    ROS_INFO("===================================");
}

void dbLogCtrlPointCoordinates(const std::array<std::array<cv::Point2f, 4>, 4> &r_ctrl_pnt_coords)
{
    ROS_INFO("        Control Point Coordinates        ");
    ROS_INFO("=========================================");
    ROS_INFO("        |      Left     |     Right     |");

    // Loop through each control point
    for (int cp = 0; cp < 4; ++cp)
    {
        // Fetch the vertices for the current control point
        auto &quad_vertices = r_ctrl_pnt_coords[cp];

        // Print the top row coordinates
        ROS_INFO("=========================================");
        ROS_INFO("        |V0   X , Y     |V1   X , Y     ||");
        ROS_INFO("-----------------------------------------");
        ROS_INFO("[%d] Top | %+5.2f , %+5.2f | %+5.2f , %+5.2f |",
                 cp,
                 quad_vertices[0].x, quad_vertices[0].y, quad_vertices[1].x, quad_vertices[1].y);

        // Print the bottom row coordinates
        ROS_INFO("---------------------------------------");
        ROS_INFO("        |V3   X , Y     |V2   X , Y     |");
        ROS_INFO("---------------------------------------");
        ROS_INFO("[%d] Btm | %+5.2f , %+5.2f | %+5.2f , %+5.2f |",
                 cp,
                 quad_vertices[3].x, quad_vertices[3].y, quad_vertices[2].x, quad_vertices[2].y);
    }
    ROS_INFO("=========================================");
}

void dbLogWallVerticesCoordinates(const std::array<std::array<std::array<cv::Point2f, 4>, MAZE_SIZE>, MAZE_SIZE> &_WALL_WARP_COORDS)
{
    ROS_INFO("                                       Warped Wall Coordinates                                               ");
    ROS_INFO("=============================================================================================================");
    ROS_INFO("        ||   (0) Left    |   (0) Right   ||   (1) Left    |   (1) Right   ||   (2) Left    |   (2) Right   ||");
    ROS_INFO("-------------------------------------------------------------------------------------------------------------");

    // Loop through each row and column in the maze
    for (int row = 0; row < MAZE_SIZE; ++row)
    {
        ROS_INFO("        ||   X   ,   Y   |   X   ,   Y   ||   X   ,   Y   |   X   ,   Y   ||   X   ,   Y   |   X   ,   Y   ||");
        ROS_INFO("-------------------------------------------------------------------------------------------------------------");

        // Buffer to hold the formatted string for each row
        char buffer[256];

        // Format and print the Top row coordinates
        snprintf(buffer, sizeof(buffer), "(%d) Top ||", row);
        for (int col = 0; col < MAZE_SIZE; ++col)
        {
            // Fetch the quad vertices for the current [row][col]
            auto &quad_vertices = _WALL_WARP_COORDS[row][col];
            snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), " %+4.2f , %+4.2f | %+4.2f , %+4.2f ||",
                     quad_vertices[0].x, quad_vertices[0].y, quad_vertices[1].x, quad_vertices[1].y);
        }
        ROS_INFO("%s", buffer);

        // Format and print the Bottom row coordinates
        snprintf(buffer, sizeof(buffer), "(%d) Btm ||", row);
        for (int col = 0; col < MAZE_SIZE; ++col)
        {
            // Fetch the quad vertices for the current [row][col]
            auto &quad = _WALL_WARP_COORDS[row][col];
            snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), " %+4.2f , %+4.2f | %+4.2f , %+4.2f ||",
                     quad[3].x, quad[3].y, quad[2].x, quad[2].y);
        }
        ROS_INFO("%s", buffer);

        ROS_INFO("-------------------------------------------------------------------------------------------------------------");
    }
}

void dbLogHomMat(const cv::Mat &r_HMAT)
{
    // Check if the input matrix is 3x3
    if (r_HMAT.rows != 3 || r_HMAT.cols != 3)
    {
        ROS_WARN("The input matrix is not 3x3. Cannot print.");
        return;
    }

    ROS_INFO("         Homography Matrix        ");
    ROS_INFO("==================================");
    ROS_INFO("          |  C0   |  C1   |  C2   |");
    ROS_INFO("----------------------------------");

    for (int i = 0; i < 3; ++i)
    {
        ROS_INFO("R%d        | %+5.2f | %+5.2f | %+5.2f |", i,
                 r_HMAT.at<double>(i, 0),
                 r_HMAT.at<double>(i, 1),
                 r_HMAT.at<double>(i, 2));
    }

    // Separator line
    ROS_INFO("==================================");
}
