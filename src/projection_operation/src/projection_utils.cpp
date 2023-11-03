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

std::array<cv::Point2f, 4> quadVec2Arr(const std::vector<cv::Point2f> &quad_vert_vec)
{
    std::array<cv::Point2f, 4> quad_vert_arr{0, 0, 0, 0};

    // Check if the input vector has exactly 4 vertices
    if (quad_vert_vec.size() != 4)
    {
        ROS_WARN("[BAD ARG] Vertices Must have 4 Points for Array Conversion");
        return quad_vert_arr;
    }

    // Convert to an array and return
    std::copy(quad_vert_vec.begin(), quad_vert_vec.end(), quad_vert_arr.begin());
    return quad_vert_arr;
}

std::vector<cv::Point2f> quadArr2Vec(const std::array<cv::Point2f, 4> &quad_vert_arr)
{
    // Convert array to vector and return
    std::vector<cv::Point2f> quad_vert_vec(quad_vert_arr.begin(), quad_vert_arr.end());
    return quad_vert_vec;
}

int checkQuadVertices(const std::array<cv::Point2f, 4> &quad_vertices)
{
    return checkQuadVertices(quadArr2Vec(quad_vertices));
}
int checkQuadVertices(const std::vector<cv::Point2f> &quad_vertices)
{

    // Check if the input vector has exactly 4 vertices
    if (quad_vertices.size() != 4)
        return 1;

    // Check if any three points are collinear; for a valid quadrilateral, no three points should be collinear
    for (int i = 0; i < 4; ++i)
    {
        cv::Point2f p1 = quad_vertices[i];
        cv::Point2f p2 = quad_vertices[(i + 1) % 4];
        cv::Point2f p3 = quad_vertices[(i + 2) % 4];
        float area = p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y);
        if (std::abs(area) < 1e-5)
            return 2; // The points are collinear
    }
    return 0;
}

cv::Size getBoundaryDims(std::array<cv::Point2f, 4> quad_vertices)
{
    float max_width = 0.0;
    float max_height = 0.0;

    for (int i = 0; i < 4; ++i)
    {
        for (int j = i + 1; j < 4; ++j)
        {
            float dx = quad_vertices[i].x - quad_vertices[j].x;
            float dy = quad_vertices[i].y - quad_vertices[j].y;
            max_width = std::max(max_width, std::abs(dx));
            max_height = std::max(max_height, std::abs(dy));
        }
    }

    int bound_width = static_cast<int>(max_width);
    int bound_height = static_cast<int>(max_height);

    return cv::Size(bound_width, bound_height);
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

void initControlPointCoordinates(std::array<std::array<cv::Point2f, 4>, 4> &out_CP_COORDS)
{

    // Specify the control point limits
    const float cp_x = MAZE_WIDTH_NDC / 2;  // starting X-coordinate in NDC coordinates
    const float cp_y = MAZE_HEIGHT_NDC / 2; // starting Y-coordinate in NDC coordinates

    // Iterate through control points
    for (float cp_i = 0; cp_i < 4; cp_i++) // image bottom to top
    {
        cv::Point2f p_org;

        // 0: image top-left
        if (cp_i == 0)
            p_org = cv::Point2f(-cp_x, -cp_y);

        // 1: image top-right
        else if (cp_i == 1)
            p_org = cv::Point2f(+cp_x, -cp_y);

        // 2: image bottom-right
        else if (cp_i == 2)
            p_org = cv::Point2f(+cp_x, +cp_y);

        // 3: image bottom-left
        else if (cp_i == 3)
            p_org = cv::Point2f(-cp_x, +cp_y);

        // Set x y values for each vertex
        out_CP_COORDS[cp_i] = {
            cv::Point2f(p_org.x, p_org.y),                                    // top left
            cv::Point2f(p_org.x + WALL_WIDTH_NDC, p_org.y),                   // top right
            cv::Point2f(p_org.x + WALL_WIDTH_NDC, p_org.y + WALL_HEIGHT_NDC), // bottom right
            cv::Point2f(p_org.x, p_org.y + WALL_HEIGHT_NDC),                  // bottom left
        };
    }
}

int updateHomographyMatrices(
    const std::array<std::array<cv::Point2f, 4>, 4> &_CP_COORDS,
    std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> &out_WALL_HMAT_DATA)
{
    // Define origin plane vertices
    std::vector<cv::Point2f> source_vertices_pxl = {
        cv::Point2f(0.0f, 0.0f),                      // Top-left
        cv::Point2f(WALL_WIDTH_PXL, 0.0f),            // Top-right
        cv::Point2f(WALL_WIDTH_PXL, WALL_HEIGHT_PXL), // Bottom-right
        cv::Point2f(0.0f, WALL_HEIGHT_PXL)};          // Bottom-left

    // Iterate trough grid/wall rows
    for (float grow_i = 0; grow_i < MAZE_SIZE; grow_i++) // image bottom to top
    {
        // Iterate trough grid/wall columns
        for (float gcol_i = 0; gcol_i < MAZE_SIZE; gcol_i++) // image left to right
        {
            std::vector<cv::Point2f> target_vertices_ndc(4);
            for (int p_i = 0; p_i < 4; p_i++)
            {
                // Get the corner values for the interpolation function
                ///@note that y values must be flipped to account for the image origin being in the top-left corner
                cv::Point2f p_a = _CP_COORDS[0][p_i]; // bottom-left interp == top left NDC
                cv::Point2f p_b = _CP_COORDS[1][p_i]; // bottom-right interp == top right NDC
                cv::Point2f p_c = _CP_COORDS[3][p_i]; // top-left interp == bottom left NDC
                cv::Point2f p_d = _CP_COORDS[2][p_i]; // top-right interp == bottom right NDC

                // Get the interpolated vertex x-coordinate
                cv::Point2f p_interp(
                    bilinearInterpolation(p_a.x, p_b.x, p_c.x, p_d.x, grow_i, gcol_i, MAZE_SIZE),  // x
                    bilinearInterpolation(p_a.y, p_b.y, p_c.y, p_d.y, grow_i, gcol_i, MAZE_SIZE)); // y

                // Store the warped vertex coordinates
                target_vertices_ndc[p_i] = p_interp;
            }

            // Convert to pixel coordinates for OpenCV's findHomography function
            std::vector<cv::Point2f> target_vertices_pxl = quadVertNdc2Pxl(target_vertices_ndc, WALL_WIDTH_PXL, WALL_HEIGHT_PXL, MAZE_WIDTH_NDC, MAZE_HEIGHT_NDC);

            // Check that the target plane vertices are valid
            if (checkQuadVertices(target_vertices_pxl) != 0)
            {
                ROS_WARN("[COMPUTE HOMOGRAPHY] Target Plane Vertices Invalid: Reason[%s]",
                         checkQuadVertices(target_vertices_pxl) == 1 ? "Wrong Number of Vertices" : "Vertices are Collinear");
                return -1;
            }

            // TEMP
            ROS_INFO("Source Vertices:");
            dbLogQuadVertices(source_vertices_pxl);
            ROS_INFO("Target Vertices:");
            dbLogQuadVertices(target_vertices_pxl);
            //return -1;

            // Use OpenCV's findHomography function to compute the homography matrix
            cv::Mat H = cv::findHomography(source_vertices_pxl, target_vertices_pxl);

            // Check for valid homography matrix
            if (H.empty())
            {
                ROS_ERROR("[COMPUTE HOMOGRAPHY] Failed to Compute Homography Matrix");
                return -1;
            }

            // Store the homography matrix
            out_WALL_HMAT_DATA[grow_i][gcol_i] = H;
        }
    }

    // TEMP
    // dbLogCtrlPointCoordinates(r_CP_COORDS);

    // Return success
    return 0;
}

void dbTrackDT(int line, const char *file_path, bool do_reset)
{
    static ros::Time start_time;
    static int line_start = 0;
    static std::string file_name_start = "";

    // Function to extract file name from file path
    auto extractFileName = [](const char *path) -> std::string
    {
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
        ROS_INFO("Elapsed Time from %s[%d] to %s[%d]: %f milliseconds",
                 file_name_start.c_str(), line_start, file_name_current.c_str(), line,
                 elapsed_time.toNSec() / 1e6);
    }
}

void dbLogQuadVertices(const std::vector<cv::Point2f> &quad_vertices)
{
    if (quad_vertices.size() != 4)
    {
        ROS_ERROR("Invalid number of vertices. Expected 4.");
        return;
    }
    dbLogQuadVertices(quadVec2Arr(quad_vertices));
}

void dbLogQuadVertices(const std::array<cv::Point2f, 4> &quad_vertices)
{
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
