#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#pragma once
std::pair<cv::Mat, cv::Mat> rs2getNewFrame(rs2::pipeline &pipe);

cv::Ptr<cv::aruco::CharucoBoard>
aruco_board_generate(int width = 5, int height = 7,
                     double square_length = 0.0373,
                     double marker_length = 0.0280, int offset_id = 10,
                     std::string img_save_path = "");
