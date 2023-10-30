// int callbackErrorOpenCV(int status, const char* func_name,
//                    const char* err_msg, const char* file_name,
//                    int line, void* userdata)
// {
//     ROS_ERROR("[OpenCV] Error Callback: Function[%s] File[%s] Line[%d] Description[%s]", func_name, file_name, line, err_msg);
//     return 0;  // Return 0 to suppress the default OpenCV error handling
// }

// // Redirect OpenCV errors to custom error handler
// cv::redirectError(callbackErrorOpenCV);

// // Some OpenCV code that might trigger an error
// cv::Mat img = cv::imread("nonexistent_file.jpg");
// return 0;