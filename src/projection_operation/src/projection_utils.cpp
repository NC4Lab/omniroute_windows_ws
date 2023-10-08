// ######################################################################################################

// ======================================== projection_utils.cpp ========================================

// ######################################################################################################

// ================================================== INCLUDE ==================================================

#include "projection_utils.h"

// ================================================== FUNCTIONS ==================================================

std::string formatCoordinatesFilePathXML(int cal_ind, int mon_ind, std::string config_dir_path)
{
    std::string file_path =
        config_dir_path + "/" +
        "cfg_c" + std::to_string(cal_ind) + "_m" + std::to_string(mon_ind) +
        ".xml";
    return file_path;
}

void loadCoordinatesXML(cv::Mat& ref_H, std::string full_path)
{
    // Create a dummy array to store control point paramameter 2D array
    float ref_cp_param[4][5];

    // Pass to the actual function
    loadCoordinatesXML(ref_H, ref_cp_param, full_path);
}

void loadCoordinatesXML(cv::Mat& ref_H, float (&ref_cp_param)[4][5], std::string full_path)
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

    // Retrieve cpParam
    std::vector<std::vector<float>> cp_param_temp;
    pugi::xml_node cpParamNode = doc.child("config").child("cpParam");
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

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            ref_cp_param[i][j] = cp_param_temp[i][j];
        }
    }

    // Retrieve H
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

std::vector<cv::Point2f> computeWallVertices(float x0, float y0, float width, float height, float shear_amount)
{
    std::vector<cv::Point2f> rect_vertices;

    // Top-left corner after applying shear
    rect_vertices.push_back(cv::Point2f(x0 + height * shear_amount, y0 + height));

    // Top-right corner after applying shear
    rect_vertices.push_back(cv::Point2f(x0 + height * shear_amount + width, y0 + height));

    // Bottom-right corner
    rect_vertices.push_back(cv::Point2f(x0 + width, y0));

    // Bottom-left corner
    rect_vertices.push_back(cv::Point2f(x0, y0));

    return rect_vertices;
}

// void drawWall(std::vector<cv::Point2f> img_vertices)
// {

//     // Start drawing a quadrilateral
//     glBegin(GL_QUADS);

//     // Set the color to white
//     glColor3f(1.0f, 1.0f, 1.0f);

//     // Set texture and vertex coordinates for each corner
//     // Bottom-left corner
//     glTexCoord2f(0.0f, 1.0f);
//     glVertex2f(img_vertices[0].x, img_vertices[0].y);

//     // Bottom-right corner
//     glTexCoord2f(1.0f, 1.0f);
//     glVertex2f(img_vertices[1].x, img_vertices[1].y);

//     // Top-right corner
//     glTexCoord2f(1.0f, 0.0f);
//     glVertex2f(img_vertices[2].x, img_vertices[2].y);

//     // Top-left corner
//     glTexCoord2f(0.0f, 0.0f);
//     glVertex2f(img_vertices[3].x, img_vertices[3].y);

//     // End drawing
//     glEnd();
// }