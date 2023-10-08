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