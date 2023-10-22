// ######################################################################################################

// ======================================== projection_utils.cpp ========================================

// ######################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_utils.h"

// ================================================== CLASSES ==================================================

// ... (include the header file and other necessary includes) ...

/**
 * @brief Constructor that initializes GLFW.
 */
GLFWWrapper::GLFWWrapper()
    : initStatus(glfwInit()), p_windowID(nullptr), pp_monitorIDVec(nullptr)
{
    if (initStatus) {
        pp_monitorIDVec = glfwGetMonitors(&nMonitors);
    }
}

/**
 * @brief Destructor that releases GLFW resources.
 */
GLFWWrapper::~GLFWWrapper() {
    cleanUp();
}

/**
 * @brief Cleans up the GLFW resources.
 */
void GLFWWrapper::cleanUp() {
    if (p_windowID) {
        glfwDestroyWindow(p_windowID);
        p_windowID = nullptr;
    }
    // Note: GLFW monitors do not need to be destroyed manually
    // glfwTerminate() will handle it
    glfwTerminate();
}

/**
 * @brief Creates a GLFW window.
 * 
 * @param width Width of the window.
 * @param height Height of the window.
 * @param title Title of the window.
 * @param monitor Monitor to use.
 * @param share Window whose context to share.
 * @return True if window creation is successful, false otherwise.
 */
bool GLFWWrapper::createWindow(int width, int height, const char* title, GLFWmonitor* monitor, GLFWwindow* share) {
    // Clean up existing resources if any
    cleanUp();  
   
    // Create a window
    p_windowID = glfwCreateWindow(width, height, title, monitor, share);
    return p_windowID != nullptr;
}

/**
 * @brief Get the window ID.
 * 
 * @return Pointer to GLFW window.
 */
GLFWwindow* GLFWWrapper::getWindowID() const {
    return p_windowID;
}

/**
 * @brief Get the monitor ID vector.
 * 
 * @return Pointer to array of GLFW monitors.
 */
GLFWmonitor** GLFWWrapper::getMonitorIDVec() const {
    return pp_monitorIDVec;
}


// ================================================== FUNCTIONS ==================================================

int checkErrorDevIL(int line, const char *file_str, const char *msg_str)
{
    ILenum il_err = ilGetError();
    if (il_err != IL_NO_ERROR)
    {
        if (msg_str)
            ROS_ERROR("[DevIL] Error Flagged: Message[%s] Description[%s] File[%s] Line[%d]", msg_str, iluErrorString(il_err), file_str, line);
        else
            ROS_ERROR("[DevIL] Error Flagged: Description[%s] File[%s] Line[%d]", iluErrorString(il_err), file_str, line);
        return -1;
    }
    return 0;
}

std::string formatCoordinatesFilePathXML(int mon_ind, int cal_ind, std::string config_dir_path)
{
    std::string file_path =
        config_dir_path + "/" +
        "cfg_m" + std::to_string(mon_ind) + "_c" + std::to_string(cal_ind) +
        ".xml";
    return file_path;
}

int loadCoordinatesXML(cv::Mat &ref_H, float (&ref_cal_param_arr)[4][5], std::string full_path, int verbose_level)
{
    // Get file name from path
    std::string file_name = full_path.substr(full_path.find_last_of('/') + 1);

    // Create an XML document object
    pugi::xml_document doc;
    if (!doc.load_file(full_path.c_str()))
    {
        ROS_ERROR("[LOAD XML] Could Not Load XML: File[%s]", file_name.c_str());
        return -1;
    }

    // Retrieve control point parameters
    std::vector<std::vector<float>> cal_param_temp;
    pugi::xml_node cal_param_node = doc.child("config").child("cal_param");
    for (pugi::xml_node row_node = cal_param_node.child("Row"); row_node; row_node = row_node.next_sibling("Row"))
    {
        std::vector<float> row;
        for (pugi::xml_node cell_node = row_node.child("Cell"); cell_node; cell_node = cell_node.next_sibling("Cell"))
        {
            float value = std::stof(cell_node.child_value());
            row.push_back(value);
        }
        cal_param_temp.push_back(row);
    }

    // Check the dimensions of control pount array
    if (cal_param_temp.size() != 4)
    {
        ROS_ERROR("[LOAD XML] Control Point Array from XML has Wrong Number of Rows[%zu]", cal_param_temp.size());
        return -1;
    }
    for (const auto &row : cal_param_temp)
    {
        if (row.size() != 5)
        {
            ROS_ERROR("[LOAD XML] Control Point Array from XML has Wrong Number of Columns[%zu]", row.size());
            return -1;
        }
    }

    // Copy data from temporary array to reference array
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            ref_cal_param_arr[i][j] = cal_param_temp[i][j];
        }
    }

    // Retrieve homography matrix
    std::vector<std::vector<float>> H_temp;
    pugi::xml_node H_node = doc.child("config").child("H");
    for (pugi::xml_node row_node = H_node.child("Row"); row_node; row_node = row_node.next_sibling("Row"))
    {
        std::vector<float> row;
        for (pugi::xml_node cell_node = row_node.child("Cell"); cell_node; cell_node = cell_node.next_sibling("Cell"))
        {
            float value = std::stof(cell_node.child_value());
            row.push_back(value);
        }
        H_temp.push_back(row);
    }

    // Check the dimensions of homography matrix
    if (H_temp.size() != 3)
    {
        ROS_ERROR("[LOAD XML] Homography Matrix from XML has Wrong Number of Rows[%zu]", H_temp.size());
        return -1;
    }
    for (const auto &row : H_temp)
    {
        if (row.size() != 3)
        {
            ROS_ERROR("[LOAD XML] Homography Matrix from XML has Wrong Number of Columns[%zu]", row.size());
            return -1;
        }
    }

    // Copy data from temporary array to reference matrix
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            ref_H.at<float>(i, j) = H_temp[i][j];
        }
    }

    // Print the loaded data
    if (verbose_level > 0)
    {
        // Print the file name
        if (verbose_level == 1)
        {
            ROS_INFO("[LOAD XML] Loaded XML: File[%s]", file_name.c_str());
        }
        // Print the control point array
        if (verbose_level == 2)
        {
            std::ostringstream oss;
            oss << "[LOAD XML] Control Point Array:\n";
            for (const auto &row : cal_param_temp)
            {
                for (const auto &value : row)
                {
                    oss << value << "\t";
                }
                oss << "\n";
            }
            ROS_INFO("%s", oss.str().c_str());
        }
        // Print the homography matrix
        if (verbose_level == 3)
        {
            std::ostringstream oss;
            oss << "[LOAD XML] Homography Matrix:\n";
            for (const auto &row : H_temp)
            {
                for (const auto &value : row)
                {
                    oss << value << "\t";
                }
                oss << "\n";
            }
            ROS_INFO("%s", oss.str().c_str());
        }
    }

    return 0;
}

void saveCoordinatesXML(cv::Mat H, float cal_param_arr[4][5], std::string full_path)
{
    // Create an XML document object
    pugi::xml_document doc;

    // Create the root element "config"
    pugi::xml_node root = doc.append_child("config");

    // Create a child node for storing control point positions
    pugi::xml_node arrayNode = root.append_child("cal_param");

    // Iterate over the rows of the 2D array 'cal_param_arr'
    for (int i = 0; i < 4; ++i)
    {
        // Create a row element under "cal_param_arr"
        pugi::xml_node rowNode = arrayNode.append_child("Row");

        // Iterate over the elements in the row
        for (int j = 0; j < 5; ++j)
        {
            // Create a cell element under the row
            pugi::xml_node cellNode = rowNode.append_child("Cell");
            cellNode.append_child(pugi::node_pcdata).set_value(std::to_string(cal_param_arr[i][j]).c_str());
        }
    }

    // Create a 2D array to store the homography matrix
    float array_2d[3][3];

    // Copy data from cv::Mat 'H' to the 2D array 'array2'
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            array_2d[i][j] = H.at<float>(i, j);
        }
    }

    // Create a child node for storing the homography matrix
    pugi::xml_node arrayNode2 = root.append_child("H");

    // Iterate over the rows of the 2D array 'array2'
    for (const auto &row : array_2d)
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

    // Get file name from path
    std::string file_name = full_path.substr(full_path.find_last_of('/') + 1);

    // Save the XML document to a file specified by 'configPath'
    if (doc.save_file(full_path.c_str()))
    {
        ROS_INFO("[SAVE XML] File Saved Successfully: File[%s]", file_name.c_str());
    }
    else
    {
        ROS_ERROR("[SAVE XML] Failed to Save XML: File[%s]", file_name.c_str());
    }
}

int loadImgTextures(std::vector<ILuint> &ref_image_ids_vec, std::vector<std::string> img_paths_vec)
{
    int img_i = 0;
    int n_img = ref_image_ids_vec.size();

    // Iterate through img file paths
    for (const std::string &img_path : img_paths_vec)
    {
        ILuint img_id;
        char msg_str[128];

        // Get file name from path
        std::string file_name = img_path.substr(img_path.find_last_of('/') + 1);

        // Generate image ID
        ilGenImages(1, &img_id);
        snprintf(msg_str, sizeof(msg_str), "Failed to Generate Image: Ind[%d/%d] ID[%u] File[%s]", img_i, n_img - 1, img_id, file_name.c_str());
        if (checkErrorDevIL(__LINE__, __FILE__, msg_str) != 0)
        {
            return -1;
        }

        // Bind image ID
        ilBindImage(img_id);
        snprintf(msg_str, sizeof(msg_str), "Failed to Bind Image: Ind[%d/%d] ID[%u] File[%s]", img_i, n_img - 1, img_id, file_name.c_str());
        if (checkErrorDevIL(__LINE__, __FILE__, msg_str) != 0)
        {
            return -1;
        }

        // Attempt to load image
        ILboolean success = ilLoadImage(img_path.c_str());
        if (success == IL_TRUE)
        {
            // Get width and height of image
            int width = ilGetInteger(IL_IMAGE_WIDTH);
            int height = ilGetInteger(IL_IMAGE_HEIGHT);

            // Check if width and height are equal to WALL_WIDTH_PXL and WALL_HEIGHT_PXL
            if (width != WALL_WIDTH_PXL || height != WALL_HEIGHT_PXL)
            {
                ROS_ERROR("[DevIL] Image is Wrong Size: Ind[%d/%d] ID[%u] File[%s] Size Actual[%d,%d] Size Expected[%d,%d]",
                          img_i, n_img - 1, img_id, file_name.c_str(), width, height, WALL_WIDTH_PXL, WALL_HEIGHT_PXL);
                ilDeleteImages(1, &img_id);
                return -1;
            }

            // Check if image is IL_BGR(Blue, Green, Red)
            ILenum format = ilGetInteger(IL_IMAGE_FORMAT);
            if (format != IL_BGR && format != IL_RGB)
            {
                ROS_ERROR("[DevIL] Image is Not IL_BGR or IL_RGB: Ind[%d/%d] ID[%u] File[%s]", img_i, n_img - 1, img_id, file_name.c_str());
                ilDeleteImages(1, &img_id);
                return -1;
            }

            // Convert image to IL_RGB
            ilConvertImage(IL_RGB, IL_UNSIGNED_BYTE);
            snprintf(msg_str, sizeof(msg_str), "Failed to Convet Image to IL_RGB: Ind[%d/%d] ID[%u] File[%s]", img_i, n_img - 1, img_id, file_name.c_str());
            if (checkErrorDevIL(__LINE__, __FILE__, msg_str) != 0)
            {
                return -1;
            }

            // Add image ID to vector
            ref_image_ids_vec.push_back(img_id);
            ROS_INFO("[DevIL] Loaded Image: Ind[%d/%d] ID[%u] File[%s] Size[%d,%d]",
                     img_i, n_img - 1, img_id, file_name.c_str(), width, height);
        }
        else
        {
            snprintf(msg_str, sizeof(msg_str), "Failed to Load Image: Ind[%d/%d] ID[%u] File[%s]", img_i, n_img - 1, img_id, file_name.c_str());
            if (checkErrorDevIL(__LINE__, __FILE__, msg_str) != 0)
            {
                // Delete image
                ilDeleteImages(1, &img_id);
            }
        }
        img_i++;
    }

    // Return success
    return 0;
}

int deleteImgTextures(std::vector<ILuint> &ref_image_ids_vec)
{
    int status = 0;
    int img_i = 0;
    int n_img = ref_image_ids_vec.size();
    char msg_str[128];

    // Iterate through image IDs
    for (ILuint img_id : ref_image_ids_vec)
    {
        // Delete image
        ilDeleteImages(1, &img_id);
        snprintf(msg_str, sizeof(msg_str), "Failed to Delete Image: Ind[%d/%d] ID[%u]", img_i, n_img - 1, img_id);
        if (checkErrorDevIL(__LINE__, __FILE__, msg_str) != 0)
            status = -1;
        else
            ROS_INFO("[DevIL] Deleted Image: Ind[%d/%d] ID[%u]", img_i, n_img - 1, img_id);
        img_i++;
    }

    // Clear the vector after deleting the images.
    ref_image_ids_vec.clear();

    // Return DevIL status
    return status;
}

int mergeImages(ILuint img1_id, ILuint img2_id, ILuint &ref_img_merge_id)
{
    // Bind and get dimensions of img1 (baseline image)
    ilBindImage(img1_id);
    if (checkErrorDevIL(__LINE__, __FILE__, "Binding Image1") != 0)
    {
        ROS_ERROR("[MERGE IMAGE] Error Binding Image1: ID[%u]", img1_id);
        return -1;
    }
    int width1 = ilGetInteger(IL_IMAGE_WIDTH);
    int height1 = ilGetInteger(IL_IMAGE_HEIGHT);
    ILubyte *data1 = ilGetData();

    // Bind and get dimensions of img2 (mask image)
    ilBindImage(img2_id);
    if (checkErrorDevIL(__LINE__, __FILE__, "Binding Image2") != 0)
    {
        ROS_ERROR("[MERGE IMAGE] Error Binding Image2: ID[%u]", img2_id);
        return -1;
    }
    int width2 = ilGetInteger(IL_IMAGE_WIDTH);
    int height2 = ilGetInteger(IL_IMAGE_HEIGHT);
    ILubyte *data2 = ilGetData();

    // Check for dimension match
    if (width1 != width2 || height1 != height2)
    {
        ROS_ERROR("[MERGE IMAGE] Dimensions Do Not Match: Image1: ID[%u] W/H(%d, %d); Image2: ID[%u] W/H(%d, %d)",
                  img1_id, width1, height1, img2_id, width2, height2);
        return -1;
    }

    // Create merged image
    ilGenImages(1, &ref_img_merge_id);
    ilBindImage(ref_img_merge_id);
    ilTexImage(width1, height1, 1, 4, IL_RGBA, IL_UNSIGNED_BYTE, NULL);
    if (checkErrorDevIL(__LINE__, __FILE__, "Creating Merged Image") != 0)
    {
        ROS_ERROR("[MERGE IMAGE] Error Creating Merged Image: ID[%u]", ref_img_merge_id);
        return -1;
    }

    // Specify number of pixels x color channels in the image
    int n_pxl_rbga = width1 * height1 * 4;

    // Initialize merged_img_data vector
    std::vector<ILubyte> merged_img_data;
    merged_img_data.resize(n_pxl_rbga);

    // Check for null pointers or zero dimensions
    if (!data1 || !data2 || merged_img_data.empty())
    {
        ROS_ERROR("[MERGE IMAGE] Null data pointer detected.");
        if (!data1)
            ROS_ERROR("[MERGE IMAGE] data1 is null.");
        if (!data2)
            ROS_ERROR("[MERGE IMAGE] data2 is null.");
        if (n_pxl_rbga == 0)
            ROS_ERROR("[MERGE IMAGE] n_pxl_rbga is zero.");
        return -1;
    }

    // Loop to overlay non-white pixels from img2 onto img1
    for (int i = 0; i < n_pxl_rbga; i += 4)
    {
        if (data2[i] != 255 || data2[i + 1] != 255 || data2[i + 2] != 255)
        {
            for (int j = 0; j < 4; ++j)
                merged_img_data[i + j] = data2[i + j];
        }
        else
        {
            for (int j = 0; j < 4; ++j)
                merged_img_data[i + j] = data1[i + j];
        }
    }

    // Set merge image data to the new image
    ilBindImage(ref_img_merge_id);
    ilSetPixels(0, 0, 0, width1, height1, 1, IL_RGBA, IL_UNSIGNED_BYTE, merged_img_data.data());
    if (checkErrorDevIL(__LINE__, __FILE__, "Setting Pixels for Merged Image") != 0)
    {
        ROS_ERROR("[MERGE IMAGE] Error Setting Pixels for Merged Image: ID[%u]", ref_img_merge_id);
        return -1;
    }

    // No need for manual delete[] here, as we used std::vector
    return 0;
}

void calculateCornerSpacing(float cal_param_arr[4][5], float (&corner_spacings_x)[2][2], float (&corner_spacings_y)[2][2], int maze_size)
{
    // Iterate over each corner
    for (int i = 0; i < 2; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            // Top-left and Bottom-right use the same logic
            if ((i == 0 && j == 0) || (i == 1 && j == 1))
            {
                corner_spacings_x[i][j] = (fabs(cal_param_arr[0][0]) + cal_param_arr[1][0]) / (float(maze_size) - 1);
                corner_spacings_y[i][j] = (cal_param_arr[0][1] + fabs(cal_param_arr[3][1])) / (float(maze_size) - 1);
            }
            // Top-right and Bottom-left use the same logic
            else
            {
                corner_spacings_x[i][j] = (fabs(cal_param_arr[0][0]) + cal_param_arr[1][0]) / (float(maze_size) - 1);
                corner_spacings_y[i][j] = (cal_param_arr[0][1] + fabs(cal_param_arr[3][1])) / (float(maze_size) - 1);
            }
        }
    }
}

float calculateInterpolatedWallSpacing(float corner_spacings[2][2], float wall_i, float wall_j, int maze_size)
{
    // Normalize indices
    float normalized_i = wall_i / (float(maze_size) - 1);
    float normalized_j = wall_j / (float(maze_size) - 1);

    // Interpolate spacings
    float interp_val_i = normalized_i * (corner_spacings[1][0] - corner_spacings[0][0]);
    float interp_val_j = normalized_j * (corner_spacings[0][1] - corner_spacings[0][0]);

    // Return interpolated spacing
    return corner_spacings[0][0] + interp_val_i + interp_val_j;
}

float calculateInterpolatedValue(float cal_param_arr[4][5], int cal_param_ind, int grid_ind_i, int grid_ind_j, int grid_size)
{
    // Get the 3 corner values
    float corner1 = cal_param_arr[0][cal_param_ind];
    float corner3 = cal_param_arr[2][cal_param_ind];
    float corner4 = cal_param_arr[3][cal_param_ind];

    // Normalize the indices by dividing by (grid_size - 1)
    float normalized_i = static_cast<float>(grid_ind_i) / (grid_size - 1);
    float normalized_j = static_cast<float>(grid_ind_j) / (grid_size - 1);

    // Linearly interpolate values between corners 3 and 4 based on grid_i_ind
    float interp_val_i = normalized_i * (corner3 - corner4);

    // Linearly interpolate values between corners 1 and 4 based on grid_j_ind
    float interp_val_j = normalized_j * (corner1 - corner4);

    // Sum the interpolated values along grid_i_ind and grid_j_ind with the base corner values (corner4)
    return corner4 + interp_val_i + interp_val_j;
}

std::vector<cv::Point2f> computeRectVertices(float x0, float y0, float width, float height, float shear_amount)
{
    std::vector<cv::Point2f> rect_vertices_vec;

    // Top-left corner after applying shear
    rect_vertices_vec.push_back(cv::Point2f(x0 + height * shear_amount, y0 + height));

    // Top-right corner after applying shear
    rect_vertices_vec.push_back(cv::Point2f(x0 + height * shear_amount + width, y0 + height));

    // Bottom-right corner
    rect_vertices_vec.push_back(cv::Point2f(x0 + width, y0));

    // Bottom-left corner
    rect_vertices_vec.push_back(cv::Point2f(x0, y0));

    return rect_vertices_vec;
}

std::vector<cv::Point2f> computePerspectiveWarp(std::vector<cv::Point2f> rect_vertices_vec, cv::Mat &ref_H, float x_offset, float y_offset)
{
    // Iterate through each vertex in the rectangle to apply perspective warping.
    for (auto &p : rect_vertices_vec)
    {
        // Step 1: Apply Offsets to Vertex Positions
        // Add the x and y offsets to the current vertex coordinates.
        // This essentially translates the vertex to a new position.
        p.x += x_offset;
        p.y += y_offset;

        // Step 2: Prepare Homogeneous Coordinates
        // Create a column matrix with the vertex's homogeneous coordinates [x, y, 1].
        // Homogeneous coordinates are used in projective geometry and make the math
        // work out when applying transformations like translation, rotation, and shearing.
        float data[] = {p.x, p.y, 1};
        cv::Mat ptMat(3, 1, CV_32F, data); // 3x1 matrix of type CV_32F (32-bit float)

        // Step 3: Homography Matrix Type Conversion
        // Ensure the homography matrix (ref_H) and the point matrix (ptMat) are of the same type.
        // This is necessary for the matrix multiplication operation that follows.
        ref_H.convertTo(ref_H, ptMat.type());

        // Step 4: Apply Homography Matrix to Warp Perspective
        // Multiply the homography matrix with the point's homogeneous coordinates.
        // This results in a new column matrix representing the point's warped coordinates.
        ptMat = ref_H * ptMat;

        // Step 5: Convert to Cartesian Coordinates
        // Divide the first two elements by the third element to convert the point back to
        // Cartesian coordinates from homogeneous coordinates.
        ptMat /= ptMat.at<float>(2); // Divide by the third element (w)

        // Step 6: Update Vertex Coordinates
        // Update the vertex's x and y coordinates with the new warped values.
        p.x = ptMat.at<float>(0, 0);
        p.y = ptMat.at<float>(0, 1);
    }

    // Return the list of vertices with their perspectives warped.
    return rect_vertices_vec;
}

void computeHomography(cv::Mat &ref_H, float cal_param_arr[4][5])
{
    // Step 1: Extract Control Point Vertices
    // Create a vector to store the vertices of the control points.
    // Each control point's x and y coordinates are taken from the cal_param_arr.
    std::vector<cv::Point2f> cp_vertices;
    cp_vertices.push_back(cv::Point2f(cal_param_arr[0][0], cal_param_arr[0][1])); // top-left
    cp_vertices.push_back(cv::Point2f(cal_param_arr[1][0], cal_param_arr[1][1])); // top-right
    cp_vertices.push_back(cv::Point2f(cal_param_arr[2][0], cal_param_arr[2][1])); // bottom-right
    cp_vertices.push_back(cv::Point2f(cal_param_arr[3][0], cal_param_arr[3][1])); // bottom-left

    // Step 2: Calculate Control Point Boundary Dimensions
    // Compute the width and height of the rectangular region that contains all control points.
    // The width is the sum of the absolute x-values of the top-left and top-right control points.
    // The height is the sum of the absolute y-values of the top-left and bottom-left control points.
    float cp_bound_width = fabs(cal_param_arr[0][0]) + cal_param_arr[1][0];
    float cp_bound_height = cal_param_arr[0][1] + fabs(cal_param_arr[3][1]);

    // Step 3: Compute Image Vertices
    // Calculate the vertices for the wall images based on the control point boundary dimensions.
    // These vertices will be used as source points for computing the homography matrix.
    std::vector<cv::Point2f> img_vertices;
    img_vertices = computeRectVertices(0.0f, 0.0f, cp_bound_width, cp_bound_height, 0);

    // Step 4: Compute Homography Matrix
    // Use OpenCV's findHomography function to compute the homography matrix.
    // This matrix will map the coordinates from the image plane to the control point plane.
    ref_H = findHomography(img_vertices, cp_vertices);
}

void updateCalParams(float (&ref_cal_param)[4][5], int cal_ind)
{
    // Copy the default array to the dynamic one
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 5; ++j)
        {
            ref_cal_param[i][j] = CAL_PARAM_DEFAULT[i][j];
        }
    }

    // Add an offset when calibrating left or right wall images
    float horz_offset = 0.05f;
    if (cal_ind == 0) // left wall
    {
        ref_cal_param[0][0] -= horz_offset; // top-left
        ref_cal_param[1][0] -= horz_offset; // top-right
        ref_cal_param[2][0] -= horz_offset; // bottom-right
        ref_cal_param[3][0] -= horz_offset; // bottom-left
    }
    else if (cal_ind == 2) // right wall
    {
        ref_cal_param[0][0] += horz_offset; // top-left
        ref_cal_param[1][0] += horz_offset; // top-right
        ref_cal_param[2][0] += horz_offset; // bottom-right
        ref_cal_param[3][0] += horz_offset; // bottom-left
    }
}
