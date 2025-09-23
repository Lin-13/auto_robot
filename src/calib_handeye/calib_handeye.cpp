#include "robot_interface/robot.h"
#include "utils/matrix_utils.h"
#include "utils/opencv_utils.h"
#include "utils/utils.h"
#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
// 假设的机器人控制函数
bool getRobotPose(cv::Mat &R_base2gripper, cv::Mat &t_base2gripper) {
  // 实现从机器人控制器获取位姿
  t_base2gripper = (cv::Mat_<double>(3, 1) << 0.1, 0.2, 0.3);
  R_base2gripper = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  return true;
}

/**
 * @brief 生成棋盘标定版
 *
 * @param width
 * @param height
 * @param offset_id
 * @return cv::Ptr<cv::aruco::CharucoBoard>
 */
cv::Ptr<cv::aruco::CharucoBoard>
aruco_board_generate(int width = 5, int height = 7, int offset_id = 10) {
  auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  double squre_length = 0.02;   // 棋盘格边长，单位：米
  double marker_length = 0.015; // 标记边长，单位：米
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
      cv::Size(width, height), squre_length, marker_length, dict, ids_vec);
  double ratio = marker_length / squre_length;

  cv::Mat boardImage;
  cv::Size img_size((width - (1 - ratio)) * 400, (height - (1 - ratio)) * 400);
  // cv::Size img_size(1000, 1440);

  board->generateImage(img_size, boardImage, 100, 1);
  std::string filename = fmt::format("charuco/charuco_board_{}x{}_id{}.png",
                                     width, height, offset_id);
  int ret = cv::imwrite(filename, boardImage);
  fmt::print("Aruco image size: {}x{} save to {} {}\n", img_size.width,
             img_size.height, filename, ret ? "success" : "failed");
  return board;
}
int main() {
  cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);

  auto dictionary = std::make_shared<cv::aruco::Dictionary>(
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250));
  // 1. 创建 ChArUco 板
  auto board = aruco_board_generate(3, 3, 60);

  cv::Mat cameraMatrix, distCoeffs;
  cameraMatrix = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
  distCoeffs = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
  std::vector<cv::Mat> R_base2gripper_list, t_base2gripper_list;
  std::vector<cv::Mat> R_target2cam_list, t_target2cam_list;
  rs2::pipeline pipe;
  pipe.start();
  rs2::frameset frames;
  int num_poses = 30;
  cv::namedWindow("Aruco", cv::WINDOW_NORMAL);
  for (int i = 0; i < num_poses; ++i) {
    fmt::print("采集第 {} 个位姿\n", i + 1);
    cv::Mat R_base2gripper, t_base2gripper;
    if (!getRobotPose(R_base2gripper, t_base2gripper))
      continue;
    // 检测 ChArUco 板
    cv::Mat image, depth;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    int auto_check = 0;
    while (true) {
      auto [new_image, new_depth] = rs2getNewFrame(pipe);
      image = new_image.clone();
      depth = new_depth.clone();
      cv::aruco::detectMarkers(new_image, dictionary, markerCorners, markerIds);
      cv::aruco::drawDetectedMarkers(new_image, markerCorners, markerIds);
      cv::imshow("Aruco", new_image);
      if (auto_check && (markerIds.size() == board->getIds().size())) {
        break;
      }
      int key = cv::waitKey(30);
      if (key != -1) {
        fmt::print("Key {} pressed\n", key);
        if (key == 27) {
          cv::destroyAllWindows();
          return 0; // 按下 'Esc' 键退出
        }
        if (key == 'q' || key == 'Q')
          break;
        if (key == 'c' || key == 'C')
          auto_check = 1; //
        fmt::print("Auto Check :{}\n", auto_check);

        if (key == 'i' || key == 'I') {
          fmt::print("info : markerIds.size() = {}\n", markerIds.size());
        }
      }

      // if (markerIds.empty()) continue;
    }

    // 插值角点
    std::vector<cv::Point2f> charucoCorners;
    std::vector<int> charucoIds;
    cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board,
                                         charucoCorners, charucoIds);

    if (charucoIds.empty())
      continue;

    // 估计姿态
    cv::Mat rvec, tvec;
    bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds,
                                                     board, cameraMatrix,
                                                     distCoeffs, rvec, tvec);

    // if (!valid) continue;
    fmt::print("valid: {}\n", valid);
    cv::waitKey(500);
    // 转换为旋转矩阵并存储
    cv::Mat R_target2cam;
    cv::Rodrigues(rvec, R_target2cam);
    Eigen::MatrixXd R = cvMatToEigenXd(R_target2cam);
    Eigen::MatrixXd t = cvMatToEigenXd(tvec);
    fmt::print("T_target2cam:\n{}\n", HomoMatrix(R, t));
    KDL::Frame T = eigenXdToKdlFrame(HomoMatrix(R, t));
    Eigen::Vector3d rpy;
    T.M.GetRPY(rpy(0), rpy(1), rpy(2));
    fmt::print("Aruco RPY: {},XYZ:{}\n", rpy, T.p);
    cv::imwrite(fmt::format("charuco/data/charuco_color_{}.png", i), image);
    cv::imwrite(fmt::format("charuco/data/charuco_depth_{}.png", i), depth);
    writeEigenXdToFile(fmt::format("charuco/data/target2cam_{}.txt", i),
                       HomoMatrix(R, t));

    // fmt::print("t_target2cam:\n{}\n", t);
    R_base2gripper_list.push_back(R_base2gripper.clone());
    t_base2gripper_list.push_back(t_base2gripper.clone());
    R_target2cam_list.push_back(R_target2cam.clone());
    t_target2cam_list.push_back(tvec.clone());
  }

  // // 4. 手眼标定
  // std::vector<cv::Mat> R_gripper2base_list, t_gripper2base_list;
  // for (size_t i = 0; i < R_base2gripper_list.size(); ++i) {
  //     cv::Mat R_gripper2base = R_base2gripper_list[i].t();
  //     cv::Mat t_gripper2base = -R_gripper2base * t_base2gripper_list[i];

  //     R_gripper2base_list.push_back(R_gripper2base);
  //     t_gripper2base_list.push_back(t_gripper2base);
  // }

  // cv::Mat R_cam2base, t_cam2base;
  // cv::calibrateHandEye(R_gripper2base_list, t_gripper2base_list,
  //                     R_target2cam_list, t_target2cam_list,
  //                     R_cam2base, t_cam2base,
  //                     cv::CALIB_HAND_EYE_TSAI);

  // std::cout << "手眼标定完成!" << std::endl;
  // std::cout << "R_cam2base:\n" << R_cam2base << std::endl;
  // std::cout << "t_cam2base:\n" << t_cam2base << std::endl;

  // cv::FileStorage fs("hand_eye_calibration.yaml", cv::FileStorage::WRITE);
  // fs << "R_cam2base" << R_cam2base;
  // fs << "t_cam2base" << t_cam2base;
  // fs.release();
  cv::destroyAllWindows();
  return 0;
}