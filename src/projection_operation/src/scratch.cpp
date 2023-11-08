// int saveHMATxml(const std::string full_path,
//                 const std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> &_HMAT_GRID_ARR)
// {
//     // Create an XML document
//     pugi::xml_document doc;

//     // Add root node for the homography matrix grid
//     pugi::xml_node hmat_grid_node = doc.append_child("HMATGrid");

//     // Iterate over each matrix in the grid
//     for (int grid_row = 0; grid_row < MAZE_SIZE; ++grid_row)
//     {
//         for (int grid_col = 0; grid_col < MAZE_SIZE; ++grid_col)
//         {
//             // Get the current homography matrix from the array
//             const cv::Mat &current_hmat = _HMAT_GRID_ARR[grid_row][grid_col];

//             // Create a child node for storing the current homography matrix
//             pugi::xml_node current_hmat_node = hmat_grid_node.append_child("HMAT");
//             current_hmat_node.append_attribute("row") = grid_row;
//             current_hmat_node.append_attribute("col") = grid_col;

//             // Iterate over the rows of the matrix
//             for (int i = 0; i < MAZE_SIZE; ++i)
//             {
//                 // Create a row node
//                 pugi::xml_node row_node = current_hmat_node.append_child("row");

//                 // Iterate over the elements in the row
//                 for (int j = 0; j < MAZE_SIZE; ++j)
//                 {
//                     // Get the value from the matrix
//                     float value = current_hmat.at<float>(i, j);

//                     // Create a cell node and set its value
//                     pugi::xml_node cell_node = row_node.append_child("cell");
//                     cell_node.append_child(pugi::node_pcdata).set_value(std::to_string(value).c_str());
//                 }
//             }
//         }
//     }

//     // Save the document to the provided file path and check if it was successful
//     bool save_succeeded = doc.save_file(full_path.c_str());
//     if (!save_succeeded)
//     {
//         ROS_ERROR("[saveHMATxml] Failed to Save Homography Matrix from XML File[%s]", full_path.c_str());
//         return -1;
//     }

//     ROS_INFO("[saveHMATxml] Saved homography matrix to XML File[%s]", full_path.c_str());
//     return 0;
// }

// int loadHMATxml(const std::string full_path,
//                 std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> &out_HMAT_GRID_ARR)
// {
//     // Create an XML document
//     pugi::xml_document doc;

//     // Parse the XML file
//     pugi::xml_parse_result result = doc.load_file(full_path.c_str());
//     if (!result)
//     {
//         ROS_ERROR("[loadHMATxml] Failed to Load Homography Matrix from XML File[%s]", full_path.c_str());
//         return -1;
//     }

//     // Get the root node for the homography matrix grid
//     pugi::xml_node hmat_grid_node = doc.child("HMATGrid");
//     if (hmat_grid_node.empty())
//     {
//         ROS_ERROR("[loadHMATxml] The 'HMATGrid' Node is Missing in the XML File[%s]", full_path.c_str());
//         return -1;
//     }

//     // Iterate over each matrix node in the grid
//     for (pugi::xml_node current_hmat_node = hmat_grid_node.child("HMAT");
//          current_hmat_node;
//          current_hmat_node = current_hmat_node.next_sibling("HMAT"))
//     {
//         // Get grid position attributes
//         int grid_row = current_hmat_node.attribute("row").as_int();
//         int grid_col = current_hmat_node.attribute("col").as_int();

//         // Check if grid positions are within bounds
//         if (grid_row < 0 || grid_row >= MAZE_SIZE ||
//             grid_col < 0 || grid_col >= MAZE_SIZE)
//         {
//             continue; // Skip to the next HMAT node
//         }

//         // Initialize a temporary matrix to store the values
//         cv::Mat current_hmat = cv::Mat::zeros(3, 3, CV_32F);

//         // Iterate over each row node within the current matrix node
//         int i = 0; // Row counter
//         for (pugi::xml_node row_node = current_hmat_node.child("row");
//              row_node && i < MAZE_SIZE;
//              row_node = row_node.next_sibling("row"), ++i)
//         {
//             int j = 0; // Column counter

//             // Iterate over each cell node within the row
//             for (pugi::xml_node cell_node = row_node.child("cell");
//                  cell_node && j < MAZE_SIZE;
//                  cell_node = cell_node.next_sibling("cell"), ++j)
//             {
//                 // Get the value from the cell and store it in the matrix
//                 float value = std::stof(cell_node.child_value());
//                 current_hmat.at<float>(i, j) = value;
//             }
//         }

//         // Store the matrix in the output array
//         out_HMAT_GRID_ARR[grid_row][grid_col] = current_hmat;
//     }

//     // Return success code
//      ROS_INFO("[loadHMATxml] Loaded saved homography matrix from XML File[%s]", full_path.c_str());
//     return 0;
// }