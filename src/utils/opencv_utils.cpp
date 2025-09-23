#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <Eigen/Dense>
#include <kdl/frames.hpp>
/**
 * @brief 获取一帧新的图像和深度
 * 
 * @param pipe rs2::pipeline
 * @return std::pair<cv::Mat,cv::Mat> copy of color and depth
 */
std::pair<cv::Mat,cv::Mat> rs2getNewFrame(rs2::pipeline& pipe)
{
    rs2::frameset frames;
    frames = pipe.wait_for_frames();
    rs2::frame color_frame = frames.get_color_frame();
    rs2::frame depth_frame = frames.get_depth_frame();
    const int w = color_frame.as<rs2::video_frame>().get_width();
    const int h = color_frame.as<rs2::video_frame>().get_height();
    cv::Mat color(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat depth(cv::Size(640,480), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(color, color, cv::COLOR_RGB2BGR);
    return {color.clone(), depth.clone()};
}
