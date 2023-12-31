// ######################################################################################################

// ======================================== projection_utils.cpp ========================================

// ######################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_utils.h"

// ================================================== VARIABLES ==================================================

// Initialze struct class for storing wall image parameters
DebugParams dbParams;

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

    static bool t1 = true;
    if (t1)
    {
        t1 = false;
        ROS_INFO("TEST1");
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

    static bool t2 = true;
    if (t2)
    {
        t2 = false;
        ROS_INFO("TEST2");
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

float bilinearInterpolation(std::array<std::array<float, 6>, 4> ctrl_point_params, int ctrl_point_params_ind, int grid_row_i, int grid_col_i, int grid_size, bool do_offset)
{
    // Get control point values that will be used as the reference corners for interpolation.
    // Note: Only 3 control points are used in this implimentation.
    float cp_val_0 = ctrl_point_params[0][ctrl_point_params_ind];
    // float cp_val_1 = ctrl_point_params[1][ctrl_point_params_ind];
    float cp_val_2 = ctrl_point_params[2][ctrl_point_params_ind];
    float cp_val_3 = ctrl_point_params[3][ctrl_point_params_ind];

    // Calculate the relative position within the grid by dividing the current index by the maximum index (grid_size - 1).
    float norm_grid_row_i = static_cast<float>(grid_row_i) / (grid_size - 1);
    float norm_grid_col_i = static_cast<float>(grid_col_i) / (grid_size - 1);

    // Perform 1D linear interpolation along the row.
    // The interpolation is between the third and fourth control points (cp_val_2 and cp_val_3).
    float interp_1d_row = norm_grid_row_i * (cp_val_2 - cp_val_3);

    // Perform 1D linear interpolation along the column.
    // The interpolation is between the first and fourth control points (cp_val_0 and cp_val_3).
    float interp_1d_col = norm_grid_col_i * (cp_val_0 - cp_val_3);

    // Combine the 1D interpolated values to compute the final 2D interpolated value.
    float interp_2d = interp_1d_row + interp_1d_col;

    // Optionally add an offset to the interpolated value. The offset is the parameter value of the control point at the origin (cp_val_3).
    interp_2d += do_offset ? cp_val_3 : 0.0;

    // Return the final interpolated value.
    return interp_2d;
}

float bilinearInterpolationFull(std::array<std::array<float, 6>, 4> ctrl_point_params, int ctrl_point_params_ind, int grid_row_i, int grid_col_i, int grid_size)
{
    // Adjust the control point values based on the new mapping.
    float A = ctrl_point_params[3][ctrl_point_params_ind]; // Corresponds to grid point row[0] col[0]
    float B = ctrl_point_params[2][ctrl_point_params_ind]; // Corresponds to grid point row[0] col[s-1]
    float C = ctrl_point_params[0][ctrl_point_params_ind]; // Corresponds to grid point row[s-1] col[0]
    float D = ctrl_point_params[1][ctrl_point_params_ind]; // Corresponds to grid point row[s-1] col[s-1]

    // Calculate the relative position within the grid.
    float x = static_cast<float>(grid_col_i) / (grid_size - 1);
    float y = static_cast<float>(grid_row_i) / (grid_size - 1);

    // Perform bilinear interpolation using the formula.
    float interp_val = (1 - x) * (1 - y) * A +
                       x * (1 - y) * B +
                       (1 - x) * y * C +
                       x * y * D;

    // Return the final interpolated value.
    return interp_val;
}

// std::vector<cv::Point2f> computeQuadVertices(float x0, float y0, float width, float height, float shear_x, float shear_y)
// {
//     std::vector<cv::Point2f> quad_vertices_vec;

//     // Top-left vertex after applying shear
//     quad_vertices_vec.push_back(cv::Point2f(x0 + height * shear_x, y0 + height));

//     // Top-right vertex after applying shear
//     quad_vertices_vec.push_back(cv::Point2f(x0 + height * shear_x + width, y0 + height));

//     // Bottom-right vertex
//     quad_vertices_vec.push_back(cv::Point2f(x0 + width, y0));

//     // Bottom-left vertex
//     quad_vertices_vec.push_back(cv::Point2f(x0, y0));

//     return quad_vertices_vec;
// }

std::vector<cv::Point2f> computeQuadVertices(float x0, float y0, float width, float height, float shear_x, float shear_y)
{
    std::vector<cv::Point2f> quad_vertices_vec;

    // Top-left vertex after applying shear
    quad_vertices_vec.push_back(cv::Point2f(x0 + height * shear_x, y0 + height));

    // Top-right vertex after applying shear
    quad_vertices_vec.push_back(cv::Point2f(x0 + height * shear_x + width, y0 + height + width * shear_y));

    // Bottom-right vertex
    quad_vertices_vec.push_back(cv::Point2f(x0 + width, y0 + width * shear_y));

    // Bottom-left vertex
    quad_vertices_vec.push_back(cv::Point2f(x0, y0));

    return quad_vertices_vec;
}

void computeHomography(cv::Mat &r_hom_mat, std::array<std::array<float, 6>, 4> ctrl_point_params)
{
    // Calculate the vertices for the control point boundary dimensions.
    // These vertices will be used as points for the 'origin' or source' when computing the homography matrix.
    std::vector<cv::Point2f> origin_plane_vertices;
    origin_plane_vertices = computeQuadVertices(0.0f, 0.0f, originPlaneWidth, originPlaneHeight, 0.0f, 0.0f);

    // Create a vector containing teh x and y cordinates of the 4 control points, whoe's origin is the center of the image.
    // These vertices will be used as points for the 'target' or 'destination' plane when computing the homography matrix.
    std::vector<cv::Point2f> target_plane_vertices;
    target_plane_vertices.push_back(cv::Point2f(ctrl_point_params[0][0], ctrl_point_params[0][1])); // top-left
    target_plane_vertices.push_back(cv::Point2f(ctrl_point_params[1][0], ctrl_point_params[1][1])); // top-right
    target_plane_vertices.push_back(cv::Point2f(ctrl_point_params[2][0], ctrl_point_params[2][1])); // bottom-right
    target_plane_vertices.push_back(cv::Point2f(ctrl_point_params[3][0], ctrl_point_params[3][1])); // bottom-left

    // Use OpenCV's findHomography function to compute the homography matrix.
    // This matrix will map the coordinates of the image (origin/source) plane the control point (target/destination) plane.
    r_hom_mat = findHomography(origin_plane_vertices, target_plane_vertices);
}

std::vector<cv::Point2f> computePerspectiveWarp(std::vector<cv::Point2f> quad_vertices_vec, cv::Mat &r_hom_mat)
{
    // Iterate through each vertex in the quadrilateral
    for (auto &vert : quad_vertices_vec)
    {
        // Convert to 3x1 homogeneous coordinate matrix
        float data[] = {vert.x, vert.y, 1}; // Column matrix with the vertex's homogeneous coordinates [x, y, 1].
        cv::Mat ptMat(3, 1, CV_32F, data);  // Point's homogeneous coordinates stored as a 3x1 matrix of type CV_32F (32-bit float)

        // Homography Matrix Type Conversion (for later matrix multiplication)
        r_hom_mat.convertTo(r_hom_mat, ptMat.type());

        // Apply Homography Matrix to Warp Perspective
        // Multiply the homography matrix with the point's homogeneous coordinates.
        // This results in a new column matrix representing the point's warped coordinates.
        ptMat = r_hom_mat * ptMat;

        // Convert back to Cartesian Coordinates
        ptMat /= ptMat.at<float>(2); // Divide first two elements by the third element (w)

        // Update/overwrite original certex coordinates with the warped coordinates
        vert.x = ptMat.at<float>(0, 0);
        vert.y = ptMat.at<float>(0, 1);
    }

    return quad_vertices_vec;
}

void updateCalParams(std::array<std::array<float, 6>, 4> &r_ctrl_point_params, int mode_cal_ind)
{
    // Copy the default array to the dynamic one
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            r_ctrl_point_params[i][j] = CTRL_POINT_PARAMS[i][j];
        }
    }

    // Add an offset when calibrating left or right wall images
    float horz_offset = 0.05f;
    if (mode_cal_ind == 0) // left wall
    {
        r_ctrl_point_params[0][0] -= horz_offset; // top-left
        r_ctrl_point_params[1][0] -= horz_offset; // top-right
        r_ctrl_point_params[2][0] -= horz_offset; // bottom-right
        r_ctrl_point_params[3][0] -= horz_offset; // bottom-left
    }
    else if (mode_cal_ind == 2) // right wall
    {
        r_ctrl_point_params[0][0] += horz_offset; // top-left
        r_ctrl_point_params[1][0] += horz_offset; // top-right
        r_ctrl_point_params[2][0] += horz_offset; // bottom-right
        r_ctrl_point_params[3][0] += horz_offset; // bottom-left
    }
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

void dbStoreQuadParams(float grid_row_i, float grid_col_i,
                       float quad_width, float quad_height,
                       float quad_shear_x, float quad_shear_y,
                       float quad_origin_x, float quad_origin_y,
                       const std::vector<cv::Point2f> &quad_vertices_raw,
                       const std::vector<cv::Point2f> &quad_vertices_warped)
{
    // Cast float indices to int
    int row = static_cast<int>(grid_row_i);
    int col = static_cast<int>(grid_col_i);

    // Store the parameters in the DebugParams struct
    dbParams.quad_width[row][col] = quad_width;
    dbParams.quad_height[row][col] = quad_height;
    dbParams.quad_shear_x[row][col] = quad_shear_x;
    dbParams.quad_shear_y[row][col] = quad_shear_y;
    dbParams.quad_origin_x[row][col] = quad_origin_x;
    dbParams.quad_origin_y[row][col] = quad_origin_y;
    dbParams.quad_vertices_raw[row][col] = quad_vertices_raw;
    dbParams.quad_vertices_warped[row][col] = quad_vertices_warped;
}

void dbLogQuadParams(std::string param_str)
{

    if (param_str == "wall_params")
    {

        ROS_INFO("Wall Parameters for Each Cell in Maze");
        ROS_INFO(" [Row,Col]| X-Org | Y-Org  | Width  | Height  | ShearX | ShearY |");
        ROS_INFO("-----------------------------------------------------------------------------");

        // Loop through each row and column in the maze
        for (int row = 0; row < MAZE_SIZE; ++row)
        {
            for (int col = 0; col < MAZE_SIZE; ++col)
            {
                ROS_INFO(" [%d, %d]  |  %4.2f  |  %4.2f  |  %4.2f  |  %4.2f  |  %4.2f  |  %4.2f  |",
                         row, col,
                         dbParams.quad_origin_x[row][col],
                         dbParams.quad_origin_y[row][col],
                         dbParams.quad_width[row][col],
                         dbParams.quad_height[row][col],
                         dbParams.quad_shear_x[row][col],
                         dbParams.quad_shear_y[row][col]);
            }
        }

        ROS_INFO("-----------------------------------------------------------------------------");
    }

    if (param_str == "quad_vec")
    {
        ROS_INFO("Wall Parameters for Each Cell in Maze");
        ROS_INFO(" [Row,Col] | Bottom Left     | Bottom Right    | Top Right       | Top Left        |");
        ROS_INFO("---------------------------------------------------------------------------------------");

        // Loop through each row and column in the maze
        for (int row = 0; row < MAZE_SIZE; ++row)
        {
            for (int col = 0; col < MAZE_SIZE; ++col)
            {
                // Fetch the quad vertices for the current [row][col]
                std::vector<cv::Point2f> quad = dbParams.quad_vertices_warped[row][col];

                // Ensure there are 4 vertices in the vector
                if (quad.size() == 4)
                {
                    ROS_INFO("  [%2d,%2d]  | X[%4.2f] Y[%4.2f] | X[%4.2f] Y[%4.2f] | X[%4.2f] Y[%4.2f] |  X[%4.2f] Y[%4.2f]",
                             row, col,
                             quad[0].x, quad[0].y,
                             quad[1].x, quad[1].y,
                             quad[2].x, quad[2].y,
                             quad[3].x, quad[3].y);
                }
                else
                {
                    // Print an error message if the size isn't 4
                    ROS_WARN("Skipped printing quad vertices for cell [%2d, %2d] due to incorrect size.", row, col);
                }
            }
        }

        ROS_INFO("---------------------------------------------------------------------------------------");
    }
}