// ######################################################################################################

// ======================================== projection_utils.cpp ========================================

// ######################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_utils.h"

// ================================================== FUNCTIONS ==================================================

// void initializeGL(GLFWwindow*& window, int win_width_pxl, int win_height_pxl, std::string window_name, GLuint &fboTexture)
// {
//     // Initialize GLFW
//     //glfwSetErrorCallback(callbackErrorGLFW);
//     if (!glfwInit())
//     {
//         ROS_ERROR("GLFW: Initialization Failed");
//         exit(-1);
//     }

//     // Create GLFW window
//     window = glfwCreateWindow(win_width_pxl, win_height_pxl, window_name.c_str(), NULL, NULL);
//     if (!window)
//     {
//         glfwTerminate();
//         ROS_ERROR("GLFW: Create Window Failed");
//         exit(-1);
//     }

//     // Set OpenGL context and callbacks
//     glfwMakeContextCurrent(window);
//     gladLoadGL();
//     //glfwSetFramebufferSizeCallback(window, callbackFrameBufferSizeGLFW);

//     // Initialize FBO and attach texture to it
//     GLuint fbo;
//     glGenFramebuffers(1, &fbo);
//     glBindFramebuffer(GL_FRAMEBUFFER, fbo);
//     glGenTextures(1, &fboTexture);
//     glBindTexture(GL_TEXTURE_2D, fboTexture);
//     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, win_width_pxl, win_height_pxl, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
//     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//     glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fboTexture, 0);
//     glBindFramebuffer(GL_FRAMEBUFFER, 0);
// }

std::string formatCoordinatesFilePathXML(int mon_ind, int cal_ind, std::string config_dir_path)
{
    std::string file_path =
        config_dir_path + "/" +
        "cfg_m" + std::to_string(mon_ind) + "_c" + std::to_string(cal_ind) +
        ".xml";
    return file_path;
}

void loadCoordinatesXML(cv::Mat &ref_H, std::string full_path)
{
    // Create a dummy array to store control point paramameter 2D array
    float cp_param[4][5];

    // Pass to the actual function
    loadCoordinatesXML(ref_H, cp_param, full_path);
}

void loadCoordinatesXML(cv::Mat &ref_H, float (&ref_cp_param)[4][5], std::string full_path)
{
    // Get file name from path
    std::string file_name = full_path.substr(full_path.find_last_of('/') + 1);

    // Create an XML document object
    pugi::xml_document doc;
    if (!doc.load_file(full_path.c_str()))
    {
        ROS_ERROR("LOAD XML: Could Not Load XML: File[%s]", file_name.c_str());
        return;
    }

    // Retrieve control point parameters
    std::vector<std::vector<float>> cp_param_temp;
    pugi::xml_node cpParamNode = doc.child("config").child("cp_param");
    for (pugi::xml_node rowNode = cpParamNode.child("Row"); rowNode; rowNode = rowNode.next_sibling("Row"))
    {
        std::vector<float> row;
        for (pugi::xml_node cellNode = rowNode.child("Cell"); cellNode; cellNode = cellNode.next_sibling("Cell"))
        {
            float value = std::stof(cellNode.child_value());
            row.push_back(value);
        }
        cp_param_temp.push_back(row);
    }

    // Check the dimensions of control pount array
    if (cp_param_temp.size() != 4)
    {
        ROS_ERROR("Control Point Array from XML has Wrong Number of Rows[%zu]", cp_param_temp.size());
        return;
    }
    for (const auto &row : cp_param_temp)
    {
        if (row.size() != 5)
        {
            ROS_ERROR("Control Point Array from XML has Wrong Number of Columns[%zu]", row.size());
            return;
        }
    }

    // Copy data from temporary array to reference array
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            ref_cp_param[i][j] = cp_param_temp[i][j];
        }
    }

    // Retrieve homography matrix
    std::vector<std::vector<float>> H_temp;
    pugi::xml_node HNode = doc.child("config").child("H");
    for (pugi::xml_node rowNode = HNode.child("Row"); rowNode; rowNode = rowNode.next_sibling("Row"))
    {
        std::vector<float> row;
        for (pugi::xml_node cellNode = rowNode.child("Cell"); cellNode; cellNode = cellNode.next_sibling("Cell"))
        {
            float value = std::stof(cellNode.child_value());
            row.push_back(value);
        }
        H_temp.push_back(row);
    }

    // Check the dimensions of homography matrix
    if (H_temp.size() != 3)
    {
        ROS_ERROR("Homography Matrix from XML has Wrong Number of Rows[%zu]", H_temp.size());
        return;
    }
    for (const auto &row : H_temp)
    {
        if (row.size() != 3)
        {
            ROS_ERROR("Homography Matrix from XML has Wrong Number of Columns[%zu]", row.size());
            return;
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
}

void saveCoordinatesXML(cv::Mat H, float cp_param[4][5], std::string full_path)
{
    // Create an XML document object
    pugi::xml_document doc;

    // Create the root element "config"
    pugi::xml_node root = doc.append_child("config");

    // Create a child node for storing control point positions
    pugi::xml_node arrayNode = root.append_child("cp_param");

    // Iterate over the rows of the 2D array 'cp_param'
    for (int i = 0; i < 4; ++i)
    {
        // Create a row element under "cp_param"
        pugi::xml_node rowNode = arrayNode.append_child("Row");

        // Iterate over the elements in the row
        for (int j = 0; j < 5; ++j)
        {
            // Create a cell element under the row
            pugi::xml_node cellNode = rowNode.append_child("Cell");
            cellNode.append_child(pugi::node_pcdata).set_value(std::to_string(cp_param[i][j]).c_str());
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
        ROS_INFO("SAVE XML: File Saved Successfully: File[%s]", file_name.c_str());
    }
    else
    {
        ROS_ERROR("SAVE XML: Failed to Save XML: File[%s]", file_name.c_str());
    }
}

void loadImgTextures(std::vector<ILuint> &ref_image_ids_vec, std::vector<std::string> img_paths_vec)
{
    // Iterate through img file paths
    for (const std::string &img_path : img_paths_vec)
    {
        ILuint img_id;
        ilGenImages(1, &img_id);
        ilBindImage(img_id);

        // Get file name from path
        std::string file_name = img_path.substr(img_path.find_last_of('/') + 1);

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
                ROS_ERROR("DevIL: Image is Wrong Size: File[%s] Size Actual[%d,%d] Size Expected[%d,%d]",
                          file_name.c_str(), width, height, WALL_WIDTH_PXL, WALL_HEIGHT_PXL);
                ilDeleteImages(1, &img_id);
                continue;
            }

            // // // Check if image is IL_BGR (Blue, Green, Red)
            // ILenum format = ilGetInteger(IL_IMAGE_FORMAT);
            // if (format != IL_BGR)
            // {
            //     ROS_ERROR("DevIL: Image is Not IL_BGR: File[%s]", file_name.c_str());
            //     ilDeleteImages(1, &img_id);
            //     continue;
            // }

            // Add image ID to vector
            ref_image_ids_vec.push_back(img_id);
            ROS_INFO("DevIL: Loaded Image: File[%s] Size[%d,%d]",
                     file_name.c_str(), width, height);
            ilConvertImage(IL_RGB, IL_UNSIGNED_BYTE);
        }
        else
        {
            ILenum error = ilGetError();
            ilDeleteImages(1, &img_id);
            ROS_ERROR("DevIL: Failed to Load Image: Error[%s] PATH[%s]", iluErrorString(error), img_path.c_str());
        }
    }
}

ILuint mergeImages(ILuint img1, ILuint img2)
{
    // Bind and get dimensions of img1 (baseline image)
    ilBindImage(img1);
    int width1 = ilGetInteger(IL_IMAGE_WIDTH);
    int height1 = ilGetInteger(IL_IMAGE_HEIGHT);
    ILubyte *data1 = ilGetData();
    ILenum error = ilGetError();
    if (error != IL_NO_ERROR)
    {
        ROS_ERROR("Error binding img1: %s", iluErrorString(error));
        return 0;
    }

    // Bind and get dimensions of img2 (mask image)
    ilBindImage(img2);
    int width2 = ilGetInteger(IL_IMAGE_WIDTH);
    int height2 = ilGetInteger(IL_IMAGE_HEIGHT);
    ILubyte *data2 = ilGetData();
    error = ilGetError();
    if (error != IL_NO_ERROR)
    {
        ROS_ERROR("Error binding img2: %s", iluErrorString(error));
        return 0;
    }

    // Check for dimension match
    if (width1 != width2 || height1 != height2)
    {
        ROS_ERROR("Dimensions do not match: img1(%d, %d), img2(%d, %d)",
                  width1, height1, width2, height2);
        return 0;
    }

    // Create merged image
    ILuint mergedImg;
    ilGenImages(1, &mergedImg);
    ilBindImage(mergedImg);
    ilTexImage(width1, height1, 1, 4, IL_RGBA, IL_UNSIGNED_BYTE, NULL);
    error = ilGetError();
    if (error != IL_NO_ERROR)
    {
        ROS_ERROR("Error creating merged image: %s", iluErrorString(error));
        return 0;
    }

    // Initialize mergedData array
    ILubyte *mergedData = new ILubyte[width1 * height1 * 4];

    // Loop to overlay non-white pixels from img2 onto img1
    for (int i = 0; i < width1 * height1 * 4; i += 4)
    {
        if (data2[i] != 255 || data2[i + 1] != 255 || data2[i + 2] != 255)
        {
            // If the pixel is not white in img2, use it in the merged image
            for (int j = 0; j < 4; ++j)
            {
                mergedData[i + j] = data2[i + j];
            }
        }
        else
        {
            // Otherwise, use the pixel from img1
            for (int j = 0; j < 4; ++j)
            {
                mergedData[i + j] = data1[i + j];
            }
        }
    }

    // Set mergedData to the new image
    ilBindImage(mergedImg);
    ilSetPixels(0, 0, 0, width1, height1, 1, IL_RGBA, IL_UNSIGNED_BYTE, mergedData);
    error = ilGetError();
    if (error != IL_NO_ERROR)
    {
        ROS_ERROR("Error setting pixels for merged image: %s", iluErrorString(error));
        delete[] mergedData;
        return 0;
    }

    // Clean up
    delete[] mergedData;

    return mergedImg;
}

/**
 * @brief Calculates an interpolated value using bilinear interpolation on a 2D grid.
 *
 * This function performs bilinear interpolation based on the position of a point
 * within a 2D grid (grid_i_ind, grid_j_ind) and predefined values at the grid corners.
 *
 * @param cp_param The array of control point parameters.
 * @param cp_ind The index of the control point parameter (3:height, 4:sheer).
 * @param grid_i_ind The index of the point along the first axis within the grid.
 * @param grid_j_ind The index of the point along the second axis within the grid.
 * @param GRID_SIZE The size of the 2D grid.
 *
 * @return float The calculated interpolated value.
 */
float calculateInterpolatedValue(float cp_param[4][5], int cp_ind, int grid_i_ind, int grid_j_ind, int GRID_SIZE)
{
    // Get the 3 corner values
    float corner1 = cp_param[0][cp_ind];
    float corner3 = cp_param[2][cp_ind];
    float corner4 = cp_param[3][cp_ind];

    // Normalize the indices by dividing by (GRID_SIZE - 1)
    float normalized_i = static_cast<float>(grid_i_ind) / (GRID_SIZE - 1);
    float normalized_j = static_cast<float>(grid_j_ind) / (GRID_SIZE - 1);

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
    // Apply perspective warping to vertices
    for (auto &p : rect_vertices_vec)
    {
        // Apply offset for vertex positions
        p.x += x_offset;
        p.y += y_offset;

        // Apply homography matrix to warp perspective
        float data[] = {p.x, p.y, 1};
        cv::Mat ptMat(3, 1, CV_32F, data);
        ref_H.convertTo(ref_H, ptMat.type());
        ptMat = ref_H * ptMat;
        ptMat /= ptMat.at<float>(2);

        // Update vertex coordinates
        p.x = ptMat.at<float>(0, 0);
        p.y = ptMat.at<float>(0, 1);
    }

    // Return warped vertices
    return rect_vertices_vec;
}