// void initControlPointCoordinates(std::array<std::array<cv::Point2f, 4>, 4> &out_CP_COORDS)
// {

//     // Specify the control point limits
//     const float cp_x = MAZE_WIDTH_NDC / 2;  // starting X-coordinate in NDC coordinates
//     const float cp_y = MAZE_HEIGHT_NDC / 2; // starting Y-coordinate in NDC coordinates

//     // Iterate through control point outer array (corners)
//     for (float cp_i = 0; cp_i < 4; cp_i++) // image bottom to top
//     {
//         cv::Point2f p_org;

//         // 0: image top-left
//         if (cp_i == 0)
//             p_org = cv::Point2f(-cp_x, -cp_y);

//         // 1: image top-right
//         else if (cp_i == 1)
//             p_org = cv::Point2f(+cp_x, -cp_y);

//         // 2: image bottom-right
//         else if (cp_i == 2)
//             p_org = cv::Point2f(+cp_x, +cp_y);

//         // 3: image bottom-left
//         else if (cp_i == 3)
//             p_org = cv::Point2f(-cp_x, +cp_y);

//         // Set x y values for each vertex
//         out_CP_COORDS[cp_i] = {
//             cv::Point2f(p_org.x, p_org.y),                                                // top left
//             cv::Point2f(p_org.x + WALL_IMAGE_WIDTH_NDC, p_org.y),                         // top right
//             cv::Point2f(p_org.x + WALL_IMAGE_WIDTH_NDC, p_org.y + WALL_IMAGE_HEIGHT_NDC), // bottom right
//             cv::Point2f(p_org.x, p_org.y + WALL_IMAGE_HEIGHT_NDC),                        // bottom left
//         };

//         // Assuming you want to vary the parameters for each circle
//         float radius = 0.05 * (i + 1);
//         float pos = 0.5 * (i + 1) - 1.0f;
//         cv::Point2f position(pos, pos);
//         cv::Scalar color(i * 50, 255 - i * 50, 0); // Example: Different colors for each circle
//         unsigned int segments = 36;                // Example: Same number of segments for each circle

//         // Iterate through control point inner array to initialize the CircleRenderer class objects array
//         for (int cv_i = 0; cv_i < 4; ++cv_i)
//         {
//             CP_RENDERERS[cv_i][cv_i].initializeCircleRenderer(
//                 out_CP_COORDS[cp_i][cv_i], // position
//                 cpDefualtMakerRadius,      // radius
//                 cpVertSelectedRGB,         // color
//                 cpRenderSegments           // segments
//             );
//         }
//     }
// }