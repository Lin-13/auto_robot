#include <Eigen/Dense>
#include <fmt/format.h>
#include <kdl/frames.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
/**
 * @brief 获取一帧新的图像和深度
 *
 * @param pipe rs2::pipeline
 * @return std::pair<cv::Mat,cv::Mat> copy of color and depth
 */
std::pair<cv::Mat, cv::Mat> rs2getNewFrame(rs2::pipeline &pipe) {
  rs2::frameset frames;
  frames = pipe.wait_for_frames();
  rs2::frame color_frame = frames.get_color_frame();
  rs2::frame depth_frame = frames.get_depth_frame();
  const int w = color_frame.as<rs2::video_frame>().get_width();
  const int h = color_frame.as<rs2::video_frame>().get_height();
  cv::Mat color(cv::Size(640, 480), CV_8UC3, (void *)color_frame.get_data(),
                cv::Mat::AUTO_STEP);
  cv::Mat depth(cv::Size(640, 480), CV_16UC1, (void *)depth_frame.get_data(),
                cv::Mat::AUTO_STEP);
  cv::cvtColor(color, color, cv::COLOR_RGB2BGR);
  return {color.clone(), depth.clone()};
}

/**
 * @brief 生成棋盘标定版
 *
 * @param width 棋盘格宽度，单位：个
 * @param height 棋盘格高度，单位：个
 * @param square_length 棋盘格边长，单位：米
 * @param marker_length 标记边长，单位：米
 * @param offset_id 起始ID号
 * @return cv::Ptr<cv::aruco::CharucoBoard>
 */
cv::Ptr<cv::aruco::CharucoBoard>
aruco_board_generate(int width, int height, double square_length,
                     double marker_length, int offset_id,
                     std::string img_save_path) {
  auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  // double square_length = 0.02;   // 棋盘格边长，单位：米
  // double marker_length = 0.015; // 标记边长，单位：米
  // double squre_length = 0.0373;  // 棋盘格边长，单位：米
  // double marker_length = 0.0280; // 标记边长，单位：米
  int id_cnt = 0;
  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      if (i % 2 != j % 2) {
        id_cnt++;
      }
    }
  }
  int id = offset_id;
  // cv::Mat ids = cv::Mat::zeros(cv::Size(id_cnt,1), CV_32S);
  // std::for_each(ids.begin<int>(), ids.end<int>(), [&id](int &n) { n += id++;
  // });
  id = offset_id;
  std::vector<int> ids_vec(id_cnt);
  std::for_each(ids_vec.begin(), ids_vec.end(), [&id](int &n) { n = id++; });
  auto board = std::make_shared<cv::aruco::CharucoBoard>(
      cv::Size(width, height), square_length, marker_length, dict, ids_vec);
  double ratio = marker_length / square_length;

  cv::Mat boardImage;
  cv::Size img_size((width - (1 - ratio)) * 400, (height - (1 - ratio)) * 400);
  // cv::Size img_size(1000, 1440);

  board->generateImage(img_size, boardImage, 100, 1);
  // std::string filename = fmt::format("charuco/charuco_board_{}x{}_id{}.png",
  //                                    width, height, offset_id);
  if (!img_save_path.empty()) {
    int ret = cv::imwrite(img_save_path, boardImage);
    fmt::print("Aruco image size: {}x{} save to {} {}\n", img_size.width,
               img_size.height, img_save_path, ret ? "success" : "failed");
  }
  return board;
}