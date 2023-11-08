// /**
//  * @brief Saves the homography matrix array to an XML file.
//  *
//  * @param full_path Path to the XML file.
//  * @param _HMAT_GRID_ARR Reference to the 3x3 array of Homography matrices to be saved.
//  *
//  * @return Integer status code [0:successful, -1:error].
//  *
//  * @details
//  * This function uses the pugixml library to create an XML document and populate it with
//  * the homography matrix for each wall in a 3x3 grid.
//  */
// int saveHMATxml(std::string, const std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> &);

// /**
//  * @brief Loads the homography matrix array from an XML file.
//  *
//  * @param full_path Path to the XML file.
//  * @param out_HMAT_GRID_ARR Reference to 3x3 array of Homography matrices to be initialized with loaded values.
//  *
//  * @return Integer status code [0:successful, -1:error].
//  *
//  * @note Uses pugiXML for XML parsing.
//  */
// int loadHMATxml(std::string, std::array<std::array<cv::Mat, MAZE_SIZE>, MAZE_SIZE> &);