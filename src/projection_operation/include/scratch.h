
// int loadCoordinatesXML(cv::Mat &r_hom_mat, std::array<std::array<float, 6>, 4> &r_ctrl_point_params, std::string full_path, int verbose_level)
// {
//     // Get file name from path
//     std::string file_name = full_path.substr(full_path.find_last_of('/') + 1);

//     // Create an XML document object
//     pugi::xml_document doc;
//     if (!doc.load_file(full_path.c_str()))
//     {
//         ROS_ERROR("[LOAD XML] Could Not Load XML: File[%s]", file_name.c_str());
//         return -1;
//     }

//     // Retrieve control point parameters
//     std::vector<std::vector<float>> ctrl_point_params_vec_temp;
//     pugi::xml_node ctrl_point_params_node = doc.child("config").child("ctrl_point_params");
//     for (pugi::xml_node row_node = ctrl_point_params_node.child("row"); row_node; row_node = row_node.next_sibling("row"))
//     {
//         std::vector<float> row;
//         for (pugi::xml_node cell_node = row_node.child("cell"); cell_node; cell_node = cell_node.next_sibling("cell"))
//         {
//             float value = std::stof(cell_node.child_value());
//             row.push_back(value);
//         }
//         ctrl_point_params_vec_temp.push_back(row);
//     }

//     // Check the dimensions of control pount array
//     if (ctrl_point_params_vec_temp.size() != 4)
//     {
//         ROS_ERROR("[LOAD XML] Control Point Data from XML has Wrong Number of Rows[%zu] File[%s]", ctrl_point_params_vec_temp.size(), file_name.c_str());
//         return -1;
//     }
//     for (const auto &row : ctrl_point_params_vec_temp)
//     {
//         if (row.size() != 6)
//         {
//             ROS_ERROR("[LOAD XML] Control Point Data from XML has Wrong Number of Columns[%zu] File[%s]", row.size(), file_name.c_str());
//             return -1;
//         }
//     }

//     // Copy data from temporary array to reference array
//     for (int i = 0; i < 4; i++)
//     {
//         for (int j = 0; j < 6; j++)
//         {
//             r_ctrl_point_params[i][j] = ctrl_point_params_vec_temp[i][j];
//         }
//     }

//     // Retrieve homography matrix
//     std::vector<std::vector<float>> hom_mat_temp;
//     pugi::xml_node hom_mat_node = doc.child("config").child("hom_mat");
//     for (pugi::xml_node row_node = hom_mat_node.child("row"); row_node; row_node = row_node.next_sibling("row"))
//     {
//         std::vector<float> row;
//         for (pugi::xml_node cell_node = row_node.child("cell"); cell_node; cell_node = cell_node.next_sibling("cell"))
//         {
//             float value = std::stof(cell_node.child_value());
//             row.push_back(value);
//         }
//         hom_mat_temp.push_back(row);
//     }

//     // Check the dimensions of homography matrix
//     if (hom_mat_temp.size() != 3)
//     {
//         ROS_ERROR("[LOAD XML] Homography Matrix from XML has Wrong Number of Rows[%zu] File[%s]", hom_mat_temp.size(), file_name.c_str());
//         return -1;
//     }
//     for (const auto &row : hom_mat_temp)
//     {
//         if (row.size() != 3)
//         {
//             ROS_ERROR("[LOAD XML] Homography Matrix from XML has Wrong Number of Columns[%zu] File[%s]", row.size(), file_name.c_str());
//             return -1;
//         }
//     }

//     // Copy data from temporary array to reference matrix
//     for (int i = 0; i < 3; i++)
//     {
//         for (int j = 0; j < 3; j++)
//         {
//             r_hom_mat.at<float>(i, j) = hom_mat_temp[i][j];
//         }
//     }

//     // Print the loaded data
//     if (verbose_level > 0)
//     {
//         // Print the file name
//         if (verbose_level == 1)
//         {
//             ROS_INFO("[LOAD XML] Loaded XML: File[%s]", file_name.c_str());
//         }
//         // Print the control point array
//         if (verbose_level == 1 || verbose_level == 2)
//         {
//             std::ostringstream oss;
//             oss << "[LOAD XML] Control Point Array:\n";
//             for (const auto &row : ctrl_point_params_vec_temp)
//             {
//                 for (const auto &value : row)
//                 {
//                     oss << value << "\t";
//                 }
//                 oss << "\n";
//             }
//             ROS_INFO("%s", oss.str().c_str());
//         }
//         // Print the homography matrix
//         if (verbose_level == 1 || verbose_level == 3)
//         {
//             std::ostringstream oss;
//             oss << "[LOAD XML] Homography Matrix:\n";
//             for (const auto &row : hom_mat_temp)
//             {
//                 for (const auto &value : row)
//                 {
//                     oss << value << "\t";
//                 }
//                 oss << "\n";
//             }
//             ROS_INFO("%s", oss.str().c_str());
//         }
//     }

//     return 0;
// }

// void saveCoordinatesXML(cv::Mat hom_mat, std::array<std::array<float, 6>, 4> ctrl_point_params, std::string full_path)
// {
//     // Create an XML document object
//     pugi::xml_document doc;

//     // Create the root element "config"
//     pugi::xml_node root = doc.append_child("config");

//     // Create a child node for storing control point positions
//     pugi::xml_node arr_node = root.append_child("ctrl_point_params");

//     // Iterate over the rows of the 2D array 'ctrl_point_params'
//     for (int i = 0; i < 4; ++i)
//     {
//         // Create a row element under "ctrl_point_params"
//         pugi::xml_node rowNode = arr_node.append_child("row");

//         // Iterate over the elements in the row
//         for (int j = 0; j < 6; ++j)
//         {
//             // Create a cell element under the row
//             pugi::xml_node cellNode = rowNode.append_child("cell");
//             cellNode.append_child(pugi::node_pcdata).set_value(std::to_string(ctrl_point_params[i][j]).c_str());
//         }
//     }

//     // Create a 2D array to store the homography matrix
//     float hom_arr_2d[3][3];

//     // Copy data from cv::Mat homology matrix to the 2D array 'array2'
//     for (int i = 0; i < 3; i++)
//     {
//         for (int j = 0; j < 3; j++)
//         {
//             hom_arr_2d[i][j] = hom_mat.at<float>(i, j);
//         }
//     }

//     // Create a child node for storing the homography matrix
//     pugi::xml_node arrayNode2 = root.append_child("hom_mat");

//     // Iterate over the rows of the 2D array 'array2'
//     for (const auto &row : hom_arr_2d)
//     {
//         // Create a row element under "hom_mat"
//         pugi::xml_node row_node = arrayNode2.append_child("row");

//         // Iterate over the elements in the row
//         for (const auto &value : row)
//         {
//             // Create a cell element under the row
//             pugi::xml_node cell_node = row_node.append_child("cell");
//             cell_node.append_child(pugi::node_pcdata).set_value(std::to_string(value).c_str());
//         }
//     }

//     // Get file name from path
//     std::string file_name = full_path.substr(full_path.find_last_of('/') + 1);

//     // Save the XML document to a file specified by 'configPath'
//     if (doc.save_file(full_path.c_str()))
//     {
//         ROS_INFO("[SAVE XML] File Saved Successfully: File[%s]", file_name.c_str());
//     }
//     else
//     {
//         ROS_ERROR("[SAVE XML] Failed to Save XML: File[%s]", file_name.c_str());
//     }
// }