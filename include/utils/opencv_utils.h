#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

#pragma once
std::pair<cv::Mat,cv::Mat> rs2getNewFrame(rs2::pipeline& pipe);

