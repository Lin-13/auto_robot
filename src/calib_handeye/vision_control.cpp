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
/**
 * @brief 捕获rs2相机图片并得到charuco板的位姿
 *
 */
class CameraCap {
public:
  CameraCap(const std::vector<cv::Ptr<cv::aruco::CharucoBoard>> &boards,
            const cv::Ptr<cv::aruco::Dictionary> &dictionary) {
    pipe_ = std::make_shared<rs2::pipeline>();
    pipe_->start();
    cameraMatrix =
        (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
    distCoeffs = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
    dictionary_ = dictionary;
    boards_ = boards;
    T_cams2targets.resize(boards_.size(), Eigen::Matrix4d::Identity());
    T_is_valid.resize(boards_.size(), false);
    T_mutex.reserve(boards_.size());
    for (int i = 0; i < boards_.size(); ++i) {
      T_mutex.push_back(std::make_unique<std::mutex>());
    }
    thread_should_stop_ = 0;
    cap_thread_ = std::make_shared<std::thread>(&CameraCap::CapThread, this);
  }
  void CapThread() {
    rs2::frameset frames;
    cv::namedWindow("Aruco", cv::WINDOW_AUTOSIZE);
    cv::Mat R_base2gripper, t_base2gripper;
    // 检测 ChArUco 板
    cv::Mat image, depth;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    while (!thread_should_stop_) {
      int key = cv::waitKey(30);
      auto [new_image, new_depth] = rs2getNewFrame(*pipe_);
      image = new_image.clone();
      depth = new_depth.clone();
      cv::aruco::detectMarkers(new_image, dictionary_, markerCorners,
                               markerIds);
      cv::aruco::drawDetectedMarkers(new_image, markerCorners, markerIds);
      cv::imshow("Aruco", new_image);
      if (markerIds.size() == 0) {
        continue;
      }
      // markerIds筛选
      std::vector<std::vector<int>> markerIds_filtered;
      std::vector<std::vector<std::vector<cv::Point2f>>> markerCorners_filtered;
      markerIds_filtered.resize(boards_.size());
      markerCorners_filtered.resize(boards_.size());
      for (int i = 0; i < markerIds.size(); i++) {
        for (int board_idx = 0; board_idx < boards_.size(); board_idx++) {
          auto ids = boards_[board_idx]->getIds();
          if (std::ranges::find(ids, markerIds[i]) != ids.end()) {
            markerIds_filtered[board_idx].push_back(markerIds[i]);
            markerCorners_filtered[board_idx].push_back(markerCorners[i]);
          }
        }
      }
      // 插值角点
      for (int board_idx = 0; board_idx < boards_.size(); board_idx++) {
        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds;
        //强筛选
        // if (markerIds_filtered[board_idx].size() <
        //     boards_[board_idx]->getIds().size()) {
        //   continue;
        // }
        // or 弱筛选
        if (markerIds_filtered[board_idx].size() == 0) {
          {
            std::unique_lock<std::mutex> lock(*T_mutex[board_idx]);
            T_is_valid[board_idx] = false;
          }
          continue;
        }
        int inter_ret = cv::aruco::interpolateCornersCharuco(
            markerCorners_filtered[board_idx], markerIds_filtered[board_idx],
            image, boards_[board_idx], charucoCorners, charucoIds);

        // 估计姿态
        cv::Mat rvec, tvec;
        int valid = cv::aruco::estimatePoseCharucoBoard(
            charucoCorners, charucoIds, boards_[board_idx], cameraMatrix,
            distCoeffs, rvec, tvec);
        if (valid == 0) {
          {
            std::unique_lock<std::mutex> lock(*T_mutex[board_idx]);
            T_is_valid[board_idx] = false;
          }
          continue;
        }
        // 转换为旋转矩阵并存储
        cv::Mat R_cam2target;
        cv::Rodrigues(rvec, R_cam2target);
        Eigen::Matrix3d R = cvMatToEigenXd(R_cam2target);
        Eigen::Vector3d t = cvMatToEigenXd(tvec);
        auto T = HomoMatrix(R, t);
        {
          std::unique_lock<std::mutex> lock(*T_mutex[board_idx]);
          T_cams2targets[board_idx] = T;
          T_is_valid[board_idx] = valid;
        }
      }
    }
    return;
  }
  Eigen::MatrixXd GetTransformcam2target(int board_idx) {
    std::unique_lock<std::mutex> lock(*T_mutex[board_idx]);
    return T_cams2targets[board_idx];
  }
  bool IsTransformValid(int board_idx) {
    std::unique_lock<std::mutex> lock(*T_mutex[board_idx]);
    return T_is_valid[board_idx];
  }
  ~CameraCap() {
    thread_should_stop_ = 1;
    cap_thread_->join();
    pipe_->stop();
    cv::destroyWindow("Aruco");
  }

private:
  std::atomic<int> thread_should_stop_;
  std::shared_ptr<std::thread> cap_thread_;
  std::shared_ptr<rs2::pipeline> pipe_;
  std::shared_ptr<cv::aruco::Dictionary> dictionary_;
  std::vector<cv::Ptr<cv::aruco::CharucoBoard>> boards_;
  std::vector<Eigen::MatrixXd> T_cams2targets;
  std::vector<int> T_is_valid;
  std::vector<std::unique_ptr<std::mutex>> T_mutex;
  cv::Mat cameraMatrix, distCoeffs;
};
Eigen::Matrix4d MatrixMove(const Eigen::Matrix4d &old,
                           const Eigen::Vector3d &rpy,
                           const Eigen::Vector3d &xyz) {
  Eigen::Matrix4d move_matrix = Eigen::Matrix4d::Identity();
  move_matrix.block<3, 3>(0, 0) = RPYToRot(rpy);
  move_matrix.block<3, 1>(0, 3) = xyz;
  Eigen::Matrix4d new_matrix = move_matrix * old;
  return new_matrix;
}
int main(int argc, char *argv[]) {
  fmt::print("Calib HandEye\nOpenCV Version: {}\n", cv::getVersionString());
  cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);

  // start up aubo robot
  std::unique_ptr<Robot> robot = auboRobotLeft();
  // robot->controller();
  // 1. 尝试启动机器人
  try {
    if (robot->start(30ms) != 0) {
      fmt::print("Aubo robot start failed.\n");
    }
  } catch (const std::exception &e) {
    fmt::print("Aubo robot Error: {}\n", e.what());
  }

  auto dictionary = std::make_shared<cv::aruco::Dictionary>(
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250));
  // 2. 创建 ChArUco 板
  std::vector<cv::Ptr<cv::aruco::CharucoBoard>> boards;
  auto left_board = aruco_board_generate(3, 3, 0.0373, 0.0280, 60);
  auto right_board = aruco_board_generate(3, 3, 0.0373, 0.0280, 20);
  // fmt::format("charuco/charuco_board_{}x{}_id{}.png", 3, 3, 20);
  boards.push_back(left_board);
  boards.push_back(right_board);
  CameraCap camera_cap(boards, dictionary);
  for (int i = 0; i < 3; i++) {
    std::this_thread::sleep_for(1s);
    for (int board_idx = 0; board_idx < boards.size(); board_idx++) {
      auto T = camera_cap.GetTransformcam2target(board_idx);
      fmt::print("---------{}---------\n", i);
      fmt::print("T_cam2target[{}] {}:\n{}\n", board_idx,
                 camera_cap.IsTransformValid(board_idx) ? "valid" : "invalid",
                 T);
    }
  }
  // 3. 读取标定矩阵,左，右机器人分别标定T_bc和T_et
  // 下面仅展示左机器人的标定结果
  fmt::print("---------Left Robot---------\n");
  auto T_bc = readEigenXdFromFile("charuco/data/T_bc.txt");
  auto T_et = readEigenXdFromFile("charuco/data/T_et.txt");
  if (T_bc.rows() != 4 || T_et.rows() != 4 || T_bc.cols() != 4 ||
      T_et.cols() != 4) {
    fmt::print("T_bc or T_et is not 4x4 matrix.\n");
    return -1;
  }
  fmt::print("T_bc\n: {}\n", T_bc);
  fmt::print("T_et\n: {}\n", T_et);
  Eigen::Vector3d move_rpy;
  Eigen::Vector3d move_xyz;
  move_rpy << 0, 0, 30;
  move_rpy = move_rpy * M_PI / 180;
  move_xyz << -0.05, 0, 0.05;
  Eigen::MatrixXd T_move = HomoMatrix(RPYToRot(move_rpy), move_xyz);
  while (!camera_cap.IsTransformValid(0)) {
    std::this_thread::sleep_for(1s);
  }
  Eigen::MatrixXd T = camera_cap.GetTransformcam2target(0);
  Eigen::MatrixXd T_target = T; // new T_ct
  T_target.block<3, 3>(0, 0) = RPYToRot(move_rpy) * T.block<3, 3>(0, 0);
  T_target.block<3, 1>(0, 3) = move_xyz + T.block<3, 1>(0, 3);
  fmt::print("T_move:\n{}\n", T_move);
  // T_bc * T_ct = T_be * T_et
  Eigen::MatrixXd actual_pose = robot->currentPose();
  // 标定精度测试
  auto error = (T_bc * T - actual_pose * T_et).norm();
  fmt::print("Calibration error:\n{}\n", error);
  Eigen::PartialPivLU<Eigen::Matrix4d> lu_et_transpose(T_et.transpose());
  Eigen::MatrixXd T_be_from =
      lu_et_transpose.solve(T.transpose() * T_bc.transpose()).transpose();
  fmt::print("Robot pose:\n{}\n", actual_pose);
  fmt::print("In camera : T_be_from:\n {}\n", T_be_from);
  std::cout << "RPY: "
            << RotToRPY(T_be_from.block<3, 3>(0, 0)).transpose() * 180 / M_PI
            << std::endl;
  Eigen::MatrixXd T_be_target =
      lu_et_transpose.solve(T_target.transpose() * T_bc.transpose())
          .transpose();
  fmt::print("In camera : T_be_target:\n {}\n", T_be_target);
  fmt::print(
      "distance:{}\n",
      (T_be_target.block<3, 1>(0, 3) - T_be_from.block<3, 1>(0, 3)).norm());
  std::cout << "RPY: "
            << RotToRPY(T_be_target.block<3, 3>(0, 0)).transpose() * 180 / M_PI
            << std::endl;
  Robot::Trajectory traj;
  traj.push_back({0, T_be_target});
  int ret = robot->MovePose(traj, 20ms, 0);
  if (ret != 0) {
    fmt::print("MovePose failed.\n");
  }
  // 等到机器人移动到该位置
  std::this_thread::sleep_for(3s);

  auto T_ct_final = camera_cap.GetTransformcam2target(0);
  fmt::print("==========Test Result==========\n");
  fmt::print("T_ct_start:\n {}\n", T);
  fmt::print("T_ct_target:\n {}\n", T_target);
  fmt::print("T_ct_final:\n {}\n", T_ct_final);
  fmt::print("image distance: {}\n",
             (T_ct_final.block<3, 1>(0, 3) - T.block<3, 1>(0, 3)).norm());
  fmt::print(
      "image move error: {}\n",
      (T_ct_final.block<3, 1>(0, 3) - T_target.block<3, 1>(0, 3)).norm());
  Eigen::MatrixXd actual_final_pose = robot->currentPose();
  fmt::print("actual start pose :\n {}\n", actual_pose);
  fmt::print("actual target pose:\n {}\n", T_be_target);
  fmt::print("actual final pose:\n {}\n", actual_final_pose);
  fmt::print("actual distance: {}\n", (actual_final_pose.block<3, 1>(0, 3) -
                                       actual_pose.block<3, 1>(0, 3))
                                          .norm());
  fmt::print("actual move error: {}\n", (actual_final_pose.block<3, 1>(0, 3) -
                                         T_be_target.block<3, 1>(0, 3))
                                            .norm());
}