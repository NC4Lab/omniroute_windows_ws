// ######################################################################################################

// ======================================== projection_utils.cpp ========================================

// ######################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_utils.h"

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

std::string formatCoordinatesFilePathXML(int mon_id_ind, int mode_cal_ind, std::string config_dir_path)
{
    std::string file_path =
        config_dir_path + "/" +
        "cfg_m" + std::to_string(mon_id_ind) + "_c" + std::to_string(mode_cal_ind) +
        ".xml";
    return file_path;
}

int loadCoordinatesXML(cv::Mat &r_hom_mat, std::array<std::array<float, 6>, 4> &r_ctrl_point_params, std::string full_path, int verbose_level)
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
    std::vector<std::vector<float>> ctrl_point_params_vec_temp;
    pugi::xml_node ctrl_point_params_node = doc.child("config").child("ctrl_point_params");
    for (pugi::xml_node row_node = ctrl_point_params_node.child("row"); row_node; row_node = row_node.next_sibling("row"))
    {
        std::vector<float> row;
        for (pugi::xml_node cell_node = row_node.child("cell"); cell_node; cell_node = cell_node.next_sibling("cell"))
        {
            float value = std::stof(cell_node.child_value());
            row.push_back(value);
        }
        ctrl_point_params_vec_temp.push_back(row);
    }

    // Check the dimensions of control pount array
    if (ctrl_point_params_vec_temp.size() != 4)
    {
        ROS_ERROR("[LOAD XML] Control Point Data from XML has Wrong Number of Rows[%zu] File[%s]", ctrl_point_params_vec_temp.size(), file_name.c_str());
        return -1;
    }
    for (const auto &row : ctrl_point_params_vec_temp)
    {
        if (row.size() != 6)
        {
            ROS_ERROR("[LOAD XML] Control Point Data from XML has Wrong Number of Columns[%zu] File[%s]", row.size(), file_name.c_str());
            return -1;
        }
    }

    // Copy data from temporary array to reference array
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            r_ctrl_point_params[i][j] = ctrl_point_params_vec_temp[i][j];
        }
    }

    // Retrieve homography matrix
    std::vector<std::vector<float>> hom_mat_temp;
    pugi::xml_node hom_mat_node = doc.child("config").child("hom_mat");
    for (pugi::xml_node row_node = hom_mat_node.child("row"); row_node; row_node = row_node.next_sibling("row"))
    {
        std::vector<float> row;
        for (pugi::xml_node cell_node = row_node.child("cell"); cell_node; cell_node = cell_node.next_sibling("cell"))
        {
            float value = std::stof(cell_node.child_value());
            row.push_back(value);
        }
        hom_mat_temp.push_back(row);
    }

    // Check the dimensions of homography matrix
    if (hom_mat_temp.size() != 3)
    {
        ROS_ERROR("[LOAD XML] Homography Matrix from XML has Wrong Number of Rows[%zu] File[%s]", hom_mat_temp.size(), file_name.c_str());
        return -1;
    }
    for (const auto &row : hom_mat_temp)
    {
        if (row.size() != 3)
        {
            ROS_ERROR("[LOAD XML] Homography Matrix from XML has Wrong Number of Columns[%zu] File[%s]", row.size(), file_name.c_str());
            return -1;
        }
    }

    // Copy data from temporary array to reference matrix
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            r_hom_mat.at<float>(i, j) = hom_mat_temp[i][j];
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
        if (verbose_level == 1 || verbose_level == 2)
        {
            std::ostringstream oss;
            oss << "[LOAD XML] Control Point Array:\n";
            for (const auto &row : ctrl_point_params_vec_temp)
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
        if (verbose_level == 1 || verbose_level == 3)
        {
            std::ostringstream oss;
            oss << "[LOAD XML] Homography Matrix:\n";
            for (const auto &row : hom_mat_temp)
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

void saveCoordinatesXML(cv::Mat hom_mat, std::array<std::array<float, 6>, 4> ctrl_point_params, std::string full_path)
{
    // Create an XML document object
    pugi::xml_document doc;

    // Create the root element "config"
    pugi::xml_node root = doc.append_child("config");

    // Create a child node for storing control point positions
    pugi::xml_node arr_node = root.append_child("ctrl_point_params");

    // Iterate over the rows of the 2D array 'ctrl_point_params'
    for (int i = 0; i < 4; ++i)
    {
        // Create a row element under "ctrl_point_params"
        pugi::xml_node rowNode = arr_node.append_child("row");

        // Iterate over the elements in the row
        for (int j = 0; j < 6; ++j)
        {
            // Create a cell element under the row
            pugi::xml_node cellNode = rowNode.append_child("cell");
            cellNode.append_child(pugi::node_pcdata).set_value(std::to_string(ctrl_point_params[i][j]).c_str());
        }
    }

    // Create a 2D array to store the homography matrix
    float hom_arr_2d[3][3];

    // Copy data from cv::Mat homology matrix to the 2D array 'array2'
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            hom_arr_2d[i][j] = hom_mat.at<float>(i, j);
        }
    }

    // Create a child node for storing the homography matrix
    pugi::xml_node arrayNode2 = root.append_child("hom_mat");

    // Iterate over the rows of the 2D array 'array2'
    for (const auto &row : hom_arr_2d)
    {
        // Create a row element under "hom_mat"
        pugi::xml_node row_node = arrayNode2.append_child("row");

        // Iterate over the elements in the row
        for (const auto &value : row)
        {
            // Create a cell element under the row
            pugi::xml_node cell_node = row_node.append_child("cell");
            cell_node.append_child(pugi::node_pcdata).set_value(std::to_string(value).c_str());
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

int loadImgTextures(std::vector<std::string> img_paths_vec, std::vector<ILuint> &r_image_id_vec)
{
    int img_i = 0;
    int n_img = (int)r_image_id_vec.size();

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
            r_image_id_vec.push_back(img_id);
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

int deleteImgTextures(std::vector<ILuint> &r_image_id_vec)
{
    int status = 0;
    int img_i = 0;
    int n_img = (int)r_image_id_vec.size();
    char msg_str[128];

    // Iterate through image IDs
    for (ILuint img_id : r_image_id_vec)
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
    r_image_id_vec.clear();

    // Return DevIL status
    return status;
}

int mergeImages(ILuint img1_id, ILuint img2_id, ILuint &r_img_merge_id)
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
    ilGenImages(1, &r_img_merge_id);
    ilBindImage(r_img_merge_id);
    ilTexImage(width1, height1, 1, 4, IL_RGBA, IL_UNSIGNED_BYTE, NULL);
    if (checkErrorDevIL(__LINE__, __FILE__, "Creating Merged Image") != 0)
    {
        ROS_ERROR("[MERGE IMAGE] Error Creating Merged Image: ID[%u]", r_img_merge_id);
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
    ilBindImage(r_img_merge_id);
    ilSetPixels(0, 0, 0, width1, height1, 1, IL_RGBA, IL_UNSIGNED_BYTE, merged_img_data.data());
    if (checkErrorDevIL(__LINE__, __FILE__, "Setting Pixels for Merged Image") != 0)
    {
        ROS_ERROR("[MERGE IMAGE] Error Setting Pixels for Merged Image: ID[%u]", r_img_merge_id);
        return -1;
    }

    // No need for manual delete[] here, as we used std::vector
    return 0;
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

cv::Mat computeHomography(const std::array<std::array<cv::Point2f, 4>, 4> &r_ctrl_pnt_coords)
{
    // Calculate the vertices for based on the initial control point boundary dimensions.
    // These vertices will be used as points for the 'origin' or source' when computing the homography matrix.
    std::vector<cv::Point2f> origin_plane_vertices = {
        cv::Point2f(0.0f, originPlaneHeight),             // top-left
        cv::Point2f(originPlaneWidth, originPlaneHeight), // top-right
        cv::Point2f(0.0f, 0.0f),                          // bottom-left
        cv::Point2f(originPlaneWidth, 0.0f)};             // bottom-right

    // Create a vector containing teh x and y cordinates of the 4 control points, whoe's origin is the center of the image.
    // These vertices will be used as points for the 'target' or 'destination' plane when computing the homography matrix.
    std::vector<cv::Point2f> target_plane_vertices;
    target_plane_vertices.push_back(cv::Point2f(r_ctrl_pnt_coords[0][2].x, r_ctrl_pnt_coords[0][2].y)); // top-left
    target_plane_vertices.push_back(cv::Point2f(r_ctrl_pnt_coords[1][2].x, r_ctrl_pnt_coords[1][2].y)); // top-right
    target_plane_vertices.push_back(cv::Point2f(r_ctrl_pnt_coords[2][2].x, r_ctrl_pnt_coords[2][2].y)); // bottom-left
    target_plane_vertices.push_back(cv::Point2f(r_ctrl_pnt_coords[3][2].x, r_ctrl_pnt_coords[3][2].y)); // bottom-right

    // Use OpenCV's findHomography function to compute the homography matrix.
    // This matrix will map the coordinates of the image (origin/source) plane the control point (target/destination) plane.
    cv::Mat h_mat = findHomography(origin_plane_vertices, target_plane_vertices);

    // Return the homography matrix for use in other libraries
    return h_mat;
}

cv::Point2f perspectiveWarpPoint(cv::Point2f p_unwarped, cv::Mat h_mat)
{
    // Convert to 3x1 homogeneous coordinate matrix
    float data[] = {p_unwarped.x, p_unwarped.y, 1}; // Column matrix with the vertex's homogeneous coordinates [x, y, 1].
    cv::Mat ptMat(3, 1, CV_32F, data);              // Point's homogeneous coordinates stored as a 3x1 matrix of type CV_32F (32-bit float)

    // Homography Matrix Type Conversion (for later matrix multiplication)
    h_mat.convertTo(h_mat, ptMat.type());

    // Apply Homography Matrix to Warp Perspective
    // Multiply the homography matrix with the point's homogeneous coordinates.
    // This results in a new column matrix representing the point's warped coordinates.
    ptMat = h_mat * ptMat;

    // Convert back to Cartesian Coordinates
    ptMat /= ptMat.at<float>(2); // Divide first two elements by the third element (w)

    // Update/overwrite original vertex coordinates with the warped coordinates
    cv::Point2f v_warped(
        ptMat.at<float>(0, 0),  // x
        ptMat.at<float>(0, 1)); // y

    return v_warped;
}
std::array<std::array<cv::Point2f, 4>, 4> initControlPointCoordinates()
{
    // Create a 2D array to store the control point coordinates
    std::array<std::array<cv::Point2f, 4>, 4> ctrl_pnt_coords;

    // Specify the control point limits
    const float cp_x = originPlaneWidth / 2;  // starting X-coordinate in NDC coordinates
    const float cp_y = originPlaneHeight / 2; // starting Y-coordinate in NDC coordinates

    // Iterate through control points
    for (float cp_i = 0; cp_i < 4; cp_i++) // image bottom to top
    {
        cv::Point2f p_org;

        // 0: image top-left
        if (cp_i == 0)
            p_org = cv::Point2f(-cp_x, +cp_y);

        // 1: image top-right
        else if (cp_i == 1)
            p_org = cv::Point2f(+cp_x, +cp_y);

        // 2: image bottom-left
        else if (cp_i == 2)
            p_org = cv::Point2f(-cp_x, -cp_y);

        // 3: image bottom-right
        else if (cp_i == 3)
            p_org = cv::Point2f(+cp_x, -cp_y);

        // Set x y values for each vertex
        ctrl_pnt_coords[cp_i] = {
            cv::Point2f(p_org.x, p_org.y + W_HT_DEF),            // top left
            cv::Point2f(p_org.x + W_WD_DEF, p_org.y + W_HT_DEF), // top right
            cv::Point2f(p_org.x, p_org.y),                       // bottom left
            cv::Point2f(p_org.x + W_WD_DEF, p_org.y),            // bottom right
        };
    }

    // Return the data container for use in other libraries
    return ctrl_pnt_coords;
}


std::array<std::array<std::array<cv::Point2f, 4>, MAZE_SIZE>, MAZE_SIZE> updateWarpedWallVertices(
    const cv::Mat &r_h_mat,
    const std::array<std::array<cv::Point2f, 4>, 4> &r_ctrl_pnt_coords)
{
    // Initialize the warped wall coordinates array
    std::array<std::array<std::array<cv::Point2f, 4>, MAZE_SIZE>, MAZE_SIZE> warp_wall_coords;

    // Iterate trough grid/wall rows
    for (float grow_i = 0; grow_i < MAZE_SIZE; grow_i++) // image bottom to top
    {
        // Iterate trough grid/wall columns
        for (float gcol_i = 0; gcol_i < MAZE_SIZE; gcol_i++) // image left to right
        {
            // Itterate through verteces
            for (int p_i = 0; p_i < 4; p_i++)
            {
                // Get the corner values for the interpolation function
                ///@note that y values must be flipped to account for the image origin being in the top-left corner
                cv::Point2f p_a = r_ctrl_pnt_coords[0][p_i]; // bottom-left interp == top left NDC
                cv::Point2f p_b = r_ctrl_pnt_coords[1][p_i]; // bottom-right interp == top right NDC
                cv::Point2f p_c = r_ctrl_pnt_coords[2][p_i]; // top-left interp == bottom left NDC
                cv::Point2f p_d = r_ctrl_pnt_coords[3][p_i]; // top-right interp == bottom right NDC

                // Get the interpolated vertex x-coordinate
                cv::Point2f p_interp(
                    bilinearInterpolation(p_a.x, p_b.x, p_c.x, p_d.x, grow_i, gcol_i, MAZE_SIZE),  // x
                    bilinearInterpolation(p_a.y, p_b.y, p_c.y, p_d.y, grow_i, gcol_i, MAZE_SIZE)); // y

                // Get the warped vertex coordinates
                cv::Point2f p_warped = perspectiveWarpPoint(p_interp, r_h_mat);

                // Store the warped vertex coordinates
                warp_wall_coords[grow_i][gcol_i][p_i] = p_warped;
            }
        }
    }

    // TEMP
    // dbLogCtrlPointCoordinates(r_ctrl_pnt_coords);
    // dbLogWallVerticesCoordinates(warp_wall_coords);

    // Return the warped wall coordinates for use in other libraries
    return warp_wall_coords;
}

void dbLogCtrlPointParams(std::array<std::array<float, 6>, 4> ctrl_point_params)
{
    ROS_INFO("Control Point Parameters");
    ROS_INFO("CP  |  X-Org   |  Y-Org   |    Width  |    Height  |  ShearX |  ShearY  |");
    ROS_INFO("---------------------------------------------------------");
    for (int i = 0; i < 4; ++i)
    {
        ROS_INFO("[%d] |  %6.2f  |  %6.2f  |  %6.2f   |  %6.2f   |  %6.2f",
                 i,
                 ctrl_point_params[i][0],
                 ctrl_point_params[i][1],
                 ctrl_point_params[i][2],
                 ctrl_point_params[i][3],
                 ctrl_point_params[i][4],
                 ctrl_point_params[i][5]);
    }
    ROS_INFO("---------------------------------------------------------");
}

void dbLogQuadVertices(const std::vector<cv::Point2f> &quad_vertices)
{
    // Check if the input vector has exactly 4 vertices
    if (quad_vertices.size() != 4)
    {
        ROS_WARN("The input vector does not contain exactly 4 vertices. Cannot print.");
        return;
    }

    // Convert to an array
    std::array<cv::Point2f, 4> result;
    std::copy(quad_vertices.begin(), quad_vertices.end(), result.begin());

    // Run main version of function
    dbLogQuadVertices(result);
}

void dbLogQuadVertices(const std::array<cv::Point2f, 4> &vertices)
{
    ROS_INFO("         Quad Vertices               ");
    ROS_INFO("=====================================");
    ROS_INFO("    |      Left     |     Right     |");

    ROS_INFO("-------------------------------------");
    ROS_INFO("    |   X   ,   Y   |   X   ,   Y   |");
    ROS_INFO("-------------------------------------");

    // Print the top row coordinates
    ROS_INFO(" Top | %+5.2f , %+5.2f | %+5.2f , %+5.2f |",
             vertices[0].x, vertices[0].y, vertices[1].x, vertices[1].y);

    // Print the bottom row coordinates
    ROS_INFO(" Btm | %+5.2f , %+5.2f | %+5.2f , %+5.2f |",
             vertices[2].x, vertices[2].y, vertices[3].x, vertices[3].y);

    ROS_INFO("=====================================");
}

void dbLogWallVerticesCoordinates(const std::array<std::array<std::array<cv::Point2f, 4>, MAZE_SIZE>, MAZE_SIZE> &r_warp_wall_coords)
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
            auto &quad = r_warp_wall_coords[row][col];
            snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), " %+4.2f , %+4.2f | %+4.2f , %+4.2f ||",
                     quad[0].x, quad[0].y, quad[1].x, quad[1].y);
        }
        ROS_INFO("%s", buffer);

        // Format and print the Bottom row coordinates
        snprintf(buffer, sizeof(buffer), "(%d) Btm ||", row);
        for (int col = 0; col < MAZE_SIZE; ++col)
        {
            // Fetch the quad vertices for the current [row][col]
            auto &quad = r_warp_wall_coords[row][col];
            snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), " %+4.2f , %+4.2f | %+4.2f , %+4.2f ||",
                     quad[2].x, quad[2].y, quad[3].x, quad[3].y);
        }
        ROS_INFO("%s", buffer);

        ROS_INFO("-------------------------------------------------------------------------------------------------------------");
    }
}

void dbLogHomMat(const cv::Mat &r_h_mat)
{
    // Check if the input matrix is 3x3
    if (r_h_mat.rows != 3 || r_h_mat.cols != 3)
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
                 r_h_mat.at<float>(i, 0),
                 r_h_mat.at<float>(i, 1),
                 r_h_mat.at<float>(i, 2));
    }

    // Separator line
    ROS_INFO("==================================");
}
