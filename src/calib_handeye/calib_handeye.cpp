#include "aubo/aubo_robot.h"
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
int main() {
  fmt::print("Calib HandEye\nOpenCV Version: {}\n", cv::getVersionString());
  cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);

  // start up aubo robot
  auto robot = auboRobotLeft();
  // 尝试启动机器人
  try {
    if (robot->start(30ms) != 0) {
      fmt::print("Aubo robot start failed.\n");
    }
  } catch (const std::exception &e) {
    fmt::print("Aubo robot Error: {}\n", e.what());
  }

  auto dictionary = std::make_shared<cv::aruco::Dictionary>(
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250));
  // 1. 创建 ChArUco 板
  auto board = aruco_board_generate(3, 3, 0.0373, 0.0280, 60,
                                    "charuco/charuco_board_3x3_id60.png");

  cv::Mat cameraMatrix, distCoeffs;
  cameraMatrix = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
  distCoeffs = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
  // 从基座坐标系到末端坐标系、从相机坐标系到目标坐标系的转换
  std::vector<cv::Mat> R_base2gripper_list, t_base2gripper_list;
  std::vector<cv::Mat> R_cam2target_list, t_cam2target_list;
  std::vector<Eigen::Matrix4d> T_base2gripper_list, T_cam2target_list;
  rs2::pipeline pipe;
  pipe.start();
  rs2::frameset frames;
  int num_poses = 10;
  cv::namedWindow("Aruco", cv::WINDOW_AUTOSIZE);
  for (int i = 0; i < num_poses; ++i) {
    fmt::print("采集第 {} 个位姿\n", i + 1);
    cv::Mat R_base2gripper, t_base2gripper;
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
    // 捕获机器人位姿
    Eigen::Matrix4d T_base2gripper = Eigen::Matrix4d::Identity();
    try {
      T_base2gripper = robot->currentPose();
      T_base2gripper_list.push_back(T_base2gripper);
      R_base2gripper_list.push_back(
          eigenXdToCvMat(T_base2gripper.block<3, 3>(0, 0)));
      t_base2gripper_list.push_back(
          eigenXdToCvMat(T_base2gripper.block<3, 1>(0, 3)));
    } catch (const std::exception &e) {
      fmt::print("Aubo robot Error: {}, use Identity Matrix\n", e.what());
      Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
      T_base2gripper_list.push_back(I);
      R_base2gripper_list.push_back(eigenXdToCvMat(I.block<3, 3>(0, 0)));
      t_base2gripper_list.push_back(eigenXdToCvMat(I.block<3, 1>(0, 3)));
    }
    int ret = writeEigenXdToFile(
        fmt::format("charuco/data/base2gripper_{}.txt", i), T_base2gripper);
    fmt::print("base2gripper_{}.txt save {}\n", i,
               ret == 0 ? "success" : "failed");
    std::cout << "T_base2gripper:\n" << T_base2gripper << std::endl;
    std::cout << "  RPY: "
              << RotToRPY(T_base2gripper.block<3, 3>(0, 0)).transpose() * 180 /
                     M_PI
              << std::endl;
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

    if (!valid) {
      continue;
    }
    fmt::print("Charuco found: {}\n", valid);
    cv::waitKey(500);
    // 转换为旋转矩阵并存储
    cv::Mat R_cam2target;
    cv::Rodrigues(rvec, R_cam2target);
    Eigen::MatrixXd R = cvMatToEigenXd(R_cam2target);
    Eigen::MatrixXd t = cvMatToEigenXd(tvec);
    fmt::print("T_cam2target:\n{}\n", HomoMatrix(R, t));
    KDL::Frame T = eigenXdToKdlFrame(HomoMatrix(R, t));
    // Eigen::Vector3d rpy;
    // T.M.GetRPY(rpy(0), rpy(1), rpy(2));
    // fmt::print("Aruco RPY: {}\n", rpy.transpose() * 180 / M_PI);
    std::cout << " RPY: " << RotToRPY(R).transpose() * 180 / M_PI << std::endl;
    cv::imwrite(fmt::format("charuco/data/charuco_color_{}.png", i), image);
    cv::imwrite(fmt::format("charuco/data/charuco_depth_{}.png", i), depth);
    ret = writeEigenXdToFile(fmt::format("charuco/data/cam2target_{}.txt", i),
                             HomoMatrix(R, t));
    fmt::print("cam2target_{}.txt save {}\n", i,
               ret == 0 ? "success" : "failed");

    // 用于opencv自身的手眼标定实现
    T_cam2target_list.push_back(HomoMatrix(R, t));
    R_cam2target_list.push_back(R_cam2target.clone());
    t_cam2target_list.push_back(tvec.clone());
  }
  // 4. 标定手眼变换
  auto T_et_adjust = [](Eigen::Matrix3d &R) {
    adjustRotateInplace(R, -1, 0, -1);
  };
  // 棋盘z轴向内
  auto T_bc_adjust = [](Eigen::Matrix3d &R) {
    adjustRotateInplace(R, 1, 0, 1);
  };
  double res = 0;
  auto T = calibrationHandtoEye(T_cam2target_list, T_base2gripper_list,
                                T_bc_adjust, T_et_adjust, &res);
  std::cout << "res: " << res << std::endl;
  std::cout << "T_bc:\n" << T[0] << std::endl;
  std::cout << "  RPY: "
            << RotToRPY(T[0].block<3, 3>(0, 0)).transpose() * 180 / M_PI
            << std::endl;
  std::cout << "T_et:\n" << T[1] << std::endl;
  writeEigenXdToFile("charuco/data/T_bc.txt", T[0]);
  std::cout << " RPY: "
            << RotToRPY(T[1].block<3, 3>(0, 0)).transpose() * 180 / M_PI
            << std::endl;
  writeEigenXdToFile("charuco/data/T_et.txt", T[1]);
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