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
            ROS_ERROR("[DevIL ERROR CHECK] Message[%s] Description[%s] File[%s] Line[%d]", msg_str, iluErrorString(il_err), file_str, line);
        else
            ROS_ERROR("[DevIL ERROR CHECK] Description[%s] File[%s] Line[%d]", iluErrorString(il_err), file_str, line);
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

int loadCoordinatesXML(std::string full_path, int verbose_level, cv::Mat &out_HMAT, std::array<std::array<float, 6>, 4> &out_ctrl_point_params)
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
            out_ctrl_point_params[i][j] = ctrl_point_params_vec_temp[i][j];
        }
    }

    // Retrieve homography matrix
    std::vector<std::vector<float>> h_mat_temp;
    pugi::xml_node h_mat_node = doc.child("config").child("h_mat");
    for (pugi::xml_node row_node = h_mat_node.child("row"); row_node; row_node = row_node.next_sibling("row"))
    {
        std::vector<float> row;
        for (pugi::xml_node cell_node = row_node.child("cell"); cell_node; cell_node = cell_node.next_sibling("cell"))
        {
            float value = std::stof(cell_node.child_value());
            row.push_back(value);
        }
        h_mat_temp.push_back(row);
    }

    // Check the dimensions of homography matrix
    if (h_mat_temp.size() != 3)
    {
        ROS_ERROR("[LOAD XML] Homography Matrix from XML has Wrong Number of Rows[%zu] File[%s]", h_mat_temp.size(), file_name.c_str());
        return -1;
    }
    for (const auto &row : h_mat_temp)
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
            out_HMAT.at<float>(i, j) = h_mat_temp[i][j];
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
            for (const auto &row : h_mat_temp)
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

void saveCoordinatesXML(cv::Mat h_mat, std::array<std::array<float, 6>, 4> ctrl_point_params, std::string full_path)
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
            hom_arr_2d[i][j] = h_mat.at<float>(i, j);
        }
    }

    // Create a child node for storing the homography matrix
    pugi::xml_node arrayNode2 = root.append_child("h_mat");

    // Iterate over the rows of the 2D array 'array2'
    for (const auto &row : hom_arr_2d)
    {
        // Create a row element under "h_mat"
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

int loadImgTextures(std::vector<std::string> img_paths_vec, std::vector<ILuint> &out_tex_id_vec)
{
    int img_i = 0;
    int n_img = (int)out_tex_id_vec.size();

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
            out_tex_id_vec.push_back(img_id);
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

int deleteImgTextures(std::vector<ILuint> &r_tex_id_vec)
{
    int status = 0;
    int img_i = 0;
    int n_img = (int)r_tex_id_vec.size();
    char msg_str[128];

    // Iterate through image IDs
    for (ILuint img_id : r_tex_id_vec)
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
    r_tex_id_vec.clear();

    // Return DevIL status
    return status;
}

int textureMerge(ILuint tex_mask_id, ILuint &out_tex_base_id)
{
    // Bind and get dimensions of the baseline image
    ilBindImage(out_tex_base_id);
    if (checkErrorDevIL(__LINE__, __FILE__) != 0)
    {
        ROS_ERROR("[MERGE TEX] Error Binding Base Image: ID[%u]", out_tex_base_id);
        return -1;
    }
    int width1 = ilGetInteger(IL_IMAGE_WIDTH);
    int height1 = ilGetInteger(IL_IMAGE_HEIGHT);
    ILubyte *data1 = ilGetData();

    // Bind and get dimensions of the mask image
    ilBindImage(tex_mask_id);
    if (checkErrorDevIL(__LINE__, __FILE__) != 0)
    {
        ROS_ERROR("[MERGE TEX] Error Binding Mask Image: ID[%u]", tex_mask_id);
        return -1;
    }
    int width2 = ilGetInteger(IL_IMAGE_WIDTH);
    int height2 = ilGetInteger(IL_IMAGE_HEIGHT);
    ILubyte *data2 = ilGetData();

    // Check for dimension match
    if (width1 != width2 || height1 != height2)
    {
        ROS_ERROR("[MERGE TEX] Dimensions Do Not Match: Base Image: ID[%u] W/H(%d, %d); Mask Image: ID[%u] W/H(%d, %d)",
                  out_tex_base_id, width1, height1, tex_mask_id, width2, height2);
        return -1;
    }

    // Check for null pointers or zero dimensions
    if (!data1 || !data2 || width1 == 0 || height1 == 0)
    {
        ROS_ERROR("[MERGE TEX] Null data pointer or zero dimension detected.");
        if (!data1)
            ROS_ERROR("[MERGE TEX] data1 is null.");
        if (!data2)
            ROS_ERROR("[MERGE TEX] data2 is null.");
        if (width1 == 0 || height1 == 0)
            ROS_ERROR("[MERGE TEX] Zero dimension detected.");
        return -1;
    }

    // Initialize merged_img_data vector
    std::vector<ILubyte> merged_img_data(width1 * height1 * 4);

    // Loop to overlay non-white pixels from mask onto base
    for (int i = 0; i < width1 * height1 * 4; i += 4)
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

    // Set merged image data to out_tex_base_id
    ilBindImage(out_tex_base_id);
    ilSetPixels(0, 0, 0, width1, height1, 1, IL_RGBA, IL_UNSIGNED_BYTE, merged_img_data.data());
    if (checkErrorDevIL(__LINE__, __FILE__) != 0)
    {
        ROS_ERROR("[MERGE TEX] Error Setting Pixels for Merged Texture: ID[%u]", out_tex_base_id);
        return -1;
    }

    // Unbind the image before exiting
    ilBindImage(0);

    // Return success
    return 0;
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

int textureWarp(cv::Mat _HMAT, std::array<cv::Point2f, 4> target_plane_vertices, ILuint &out_tex_source_id)
{
    // Compute output/target image size
    cv::Size warped_texture_size = getBoundaryDims(target_plane_vertices);

    // Bind the DevIL image
    ilBindImage(out_tex_source_id);

    // Get the dimensions and data of the texture image
    int source_width = ilGetInteger(IL_IMAGE_WIDTH);
    int source_height = ilGetInteger(IL_IMAGE_HEIGHT);
    ILubyte *data = ilGetData();

    // Create a cv::Mat from the ILuint image
    cv::Mat source_texture_mat(source_height, source_width, CV_8UC3, data);

    // Create a cv::Mat to hold the warped texture
    cv::Mat warped_texture_mat;

    // Use OpenCV's warpPerspective function to apply the homography matrix to the texture image
    cv::warpPerspective(
        source_texture_mat,
        warped_texture_mat,
        _HMAT,
        warped_texture_size);

    // Update the original image data with the new warped data
    ilTexImage(
        warped_texture_mat.cols,
        warped_texture_mat.rows,
        1, 3,
        IL_RGB, IL_UNSIGNED_BYTE,
        warped_texture_mat.data);
    if (checkErrorDevIL(__LINE__, __FILE__) != 0)
    {
        ROS_ERROR("[WARP TEX] Error Updating Warped Texture: ID[%u]", out_tex_source_id);
        return -1;
    }

    // Unbind the image before exiting
    ilBindImage(0);

    // Return success
    return 0;
}

void initControlPointCoordinates(std::array<std::array<cv::Point2f, 4>, 4> &out_CTRL_PNT_DATA)
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
        out_CTRL_PNT_DATA[cp_i] = {
            cv::Point2f(p_org.x, p_org.y + WALL_HEIGHT_NDC),                  // top left
            cv::Point2f(p_org.x + WALL_WIDTH_NDC, p_org.y + WALL_HEIGHT_NDC), // top right
            cv::Point2f(p_org.x, p_org.y),                                    // bottom left
            cv::Point2f(p_org.x + WALL_WIDTH_NDC, p_org.y),                   // bottom right
        };
    }
}

int updateWallVertices(
    const std::array<std::array<cv::Point2f, 4>, 4> &_CTRL_PNT_DATA,
    std::array<std::array<std::array<cv::Point2f, 4>, MAZE_SIZE>, MAZE_SIZE> &out_WALL_VERT_DATA)
{

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
                cv::Point2f p_a = _CTRL_PNT_DATA[0][p_i]; // bottom-left interp == top left NDC
                cv::Point2f p_b = _CTRL_PNT_DATA[1][p_i]; // bottom-right interp == top right NDC
                cv::Point2f p_c = _CTRL_PNT_DATA[2][p_i]; // top-left interp == bottom left NDC
                cv::Point2f p_d = _CTRL_PNT_DATA[3][p_i]; // top-right interp == bottom right NDC

                // Get the interpolated vertex x-coordinate
                cv::Point2f p_interp(
                    bilinearInterpolation(p_a.x, p_b.x, p_c.x, p_d.x, grow_i, gcol_i, MAZE_SIZE),  // x
                    bilinearInterpolation(p_a.y, p_b.y, p_c.y, p_d.y, grow_i, gcol_i, MAZE_SIZE)); // y

                // Store the warped vertex coordinates
                out_WALL_VERT_DATA[grow_i][gcol_i][p_i] = p_interp;
            }
        }
    }

    // TEMP
    // dbLogCtrlPointCoordinates(r_CTRL_PNT_DATA);
    // dbLogWallVerticesCoordinates(r_WALL_VERT_DATA);

    // Return success
    return 0;
}

int updateWallHomography(
    const std::array<std::array<cv::Point2f, 4>, 4> &_CTRL_PNT_DATA,
    const std::array<std::array<std::array<cv::Point2f, 4>, MAZE_SIZE>, MAZE_SIZE> &_WALL_VERT_DATA,
    std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> &out_WALL_HMAT_DATA)
{
    // Define origin plane vertices
    std::vector<cv::Point2f> origin_vertices = {
        cv::Point2f(0.0f, WALL_HEIGHT_NDC),           // Top-left
        cv::Point2f(WALL_WIDTH_NDC, WALL_HEIGHT_NDC), // Top-right
        cv::Point2f(0.0f, 0.0f),                      // Bottom-left
        cv::Point2f(WALL_WIDTH_NDC, 0.0f)};           // Bottom-right

    // Iterate through the maze grid rows
    for (float grow_i = 0; grow_i < MAZE_SIZE; grow_i++) // image bottom to top
    {
        // Iterate through each column in the maze row
        for (float gcol_i = 0; gcol_i < MAZE_SIZE; gcol_i++) // image left to right
        {
            // Get warped vertices for this wall to use as target
            std::vector<cv::Point2f> target_vertices = quadArr2Vec(_WALL_VERT_DATA[grow_i][gcol_i]);

            // Convert to pixel coordinates for OpenCV's findHomography function
            std::vector<cv::Point2f> target_vertices_pxl =  quadVertNdc2Pxl(target_vertices, WALL_WIDTH_PXL, WALL_HEIGHT_PXL, MAZE_WIDTH_NDC, MAZE_HEIGHT_NDC);
            std::vector<cv::Point2f> origin_vertices_pxl =  quadVertNdc2Pxl(origin_vertices, WALL_WIDTH_PXL, WALL_HEIGHT_PXL, MAZE_WIDTH_NDC, MAZE_HEIGHT_NDC);

            // Check that the target plane vertices are valid
            if (checkQuadVertices(target_vertices_pxl) != 0)
            {
                ROS_WARN("[COMPUTE HOMOGRAPHY] Target Plane Vertices Invalid: Reason[%s]",
                         checkQuadVertices(target_vertices_pxl) == 1 ? "Wrong Number of Vertices" : "Vertices are Collinear");
                return -1;
            }
            // Use OpenCV's findHomography function to compute the homography matrix
            cv::Mat _HMAT = cv::findHomography(origin_vertices_pxl, target_vertices_pxl);

            // Check for valid homography matrix
            if (_HMAT.empty())
            {
                ROS_ERROR("[COMPUTE HOMOGRAPHY] Failed to Compute Homography Matrix");
                return -1;
            }

            // Copy to the output homography matrix dataset
            out_WALL_HMAT_DATA[grow_i][gcol_i] = _HMAT;

            // TEMP
            if (gcol_i == 0 && grow_i == 0)
            {
                ROS_INFO("[COMPUTE HOMOGRAPHY] Origin Plane Vertices:");
                dbLogQuadVertices(origin_vertices_pxl);
                ROS_INFO("[COMPUTE HOMOGRAPHY] Target Plane Vertices:");
                dbLogQuadVertices(target_vertices_pxl);
                dbLogHomMat(_HMAT);
            }
        }
    }

    return 0;
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
    dbLogQuadVertices(quadVec2Arr(quad_vertices));
}

void dbLogQuadVertices(const std::array<cv::Point2f, 4> &quad_vertices)
{
    ROS_INFO("         Quad Vertices               ");
    ROS_INFO("=====================================");
    ROS_INFO("    |      Left     |     Right     |");

    ROS_INFO("-------------------------------------");
    ROS_INFO("    |   X   ,   Y   |   X   ,   Y   |");
    ROS_INFO("-------------------------------------");

    // Print the top row coordinates
    ROS_INFO(" Top | %+5.2f , %+5.2f | %+5.2f , %+5.2f |",
             quad_vertices[0].x, quad_vertices[0].y, quad_vertices[1].x, quad_vertices[1].y);

    // Print the bottom row coordinates
    ROS_INFO(" Btm | %+5.2f , %+5.2f | %+5.2f , %+5.2f |",
             quad_vertices[2].x, quad_vertices[2].y, quad_vertices[3].x, quad_vertices[3].y);

    ROS_INFO("=====================================");
}

void dbLogWallVerticesCoordinates(const std::array<std::array<std::array<cv::Point2f, 4>, MAZE_SIZE>, MAZE_SIZE> &_WALL_VERT_DATA)
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
            auto &quad = _WALL_VERT_DATA[row][col];
            snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), " %+4.2f , %+4.2f | %+4.2f , %+4.2f ||",
                     quad[0].x, quad[0].y, quad[1].x, quad[1].y);
        }
        ROS_INFO("%s", buffer);

        // Format and print the Bottom row coordinates
        snprintf(buffer, sizeof(buffer), "(%d) Btm ||", row);
        for (int col = 0; col < MAZE_SIZE; ++col)
        {
            // Fetch the quad vertices for the current [row][col]
            auto &quad = _WALL_VERT_DATA[row][col];
            snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), " %+4.2f , %+4.2f | %+4.2f , %+4.2f ||",
                     quad[2].x, quad[2].y, quad[3].x, quad[3].y);
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
