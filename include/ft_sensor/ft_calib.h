#pragma once
// #include "aubo/aubo_robot.h"
#include "ft_sensor/ft_sensor.h"
#include "robot_interface/robot.h"
#include "utils/matrix_utils.h"
#include "utils/utils.h"
#include <chrono>
#include <filesystem>
#include <fmt/ranges.h>
#include <string>
#include <thread>
#include <unordered_map>
using namespace std::chrono_literals;
using std::string;
constexpr char LEFT_ATI_IP[] = "192.168.1.111";
constexpr char RIGHT_ATI_IP[] = "192.168.1.112";
using FTParams = std::unordered_map<std::string, Eigen::Vector3d>;
/*****************************************==Utils==*****************************************/
FTParams read_gravity_calib_data(const std::string &filename);

int write_gravity_calib_data(const std::string &filename,
                             const FTParams &params);

Eigen::VectorXd decompose_force(const Eigen::Matrix3d &R,
                                const Eigen::VectorXd &wrench,
                                const FTParams &params);
/***************************==FTSensorGravityCompensation==***********************************/
// constructor : sensor_ip, calib_file
// bindPoseDetector : 绑定位姿检测函数，返回位姿矩阵
// getPose : 获取当前位姿
// getWrench : 获取当前力矩
// getCompensatedWrench : 获取补偿后的力矩
class FTSensorGravityCompensation {
public:
  FTSensorGravityCompensation(const string sensor_ip, const string calib_file) {
    sensor_ = std::make_shared<ati::FTSensor>();
    sensor_->init(sensor_ip);
    params_ = read_gravity_calib_data(calib_file);
    soft_bias_ = Eigen::VectorXd::Zero(6);
  }
  FTSensorGravityCompensation(std::shared_ptr<ati::FTSensor> sensor,
                              const string calib_file) {
    sensor_ = sensor;
    params_ = read_gravity_calib_data(calib_file);
    soft_bias_ = Eigen::VectorXd::Zero(6);
  }
  /**
   * @brief 获取FT传感器
   *
   * @return std::shared_ptr<ati::FTSensor>
   */
  std::shared_ptr<ati::FTSensor> getSensor() { return sensor_; }
  /**
   * @param pose_callback return R or T
   * @return int 0表示成功，-1表示失败
   */
  int bindPoseDetector(std::function<Eigen::MatrixXd(void)> pose_callback) {
    pose_callback_ = pose_callback;
    return 0;
  }
  /**
   * @brief 获取当前位姿
   *
   * @return Eigen::MatrixXd 4x4
   */
  Eigen::MatrixXd getPose() {
    if (!pose_callback_) {
      return Eigen::MatrixXd::Identity(4, 4);
    } else {
      return pose_callback_();
    }
  }
  Eigen::VectorXd getWrench() {
    Eigen::Vector<double, 6> wrench;
    sensor_->getMeasurements(wrench.data());
    return wrench;
  }
  void setSoftBias() { soft_bias_ += getCompensatedWrench(); }
  void setSoftBiasMean(std::chrono::microseconds duration) {
    // 计算在给定时间段内的平均值
    Eigen::Vector<double, 6> mean;
    mean.setZero();

    auto now = std::chrono::steady_clock::now();
    auto start = now;
    int cnt = 0;
    while (std::chrono::duration_cast<std::chrono::microseconds>(now - start) <
           duration) {
      mean += getCompensatedWrench();
      cnt++;
      std::this_thread::sleep_for(10ms);
      now = std::chrono::steady_clock::now();
    }
    mean /= cnt;
    soft_bias_ = mean;
  }

  Eigen::VectorXd getCompensatedWrench() {
    Eigen::Vector<double, 6> wrench = getWrench();
    Eigen::MatrixXd pose = getPose();
    Eigen::Vector<double, 6> comp_wrench =
        decompose_force(pose.block<3, 3>(0, 0), wrench, params_);
    return comp_wrench - soft_bias_;
  }

  ~FTSensorGravityCompensation() = default;

private:
  std::shared_ptr<ati::FTSensor> sensor_;
  FTParams params_;
  std::function<Eigen::MatrixXd(void)> pose_callback_;
  Eigen::VectorXd soft_bias_;
};

/*****************************************==Test==*****************************************/
/**
 * @brief 测试FT传感器,打印测量值
 *
 * @return int 0表示成功，-1表示失败
 */
int test_ft_sensor();
/**
 * @brief 测试重力补偿
 *
 * @param signal 信号量，用于触发校准
 * @param sensor 传感器指针
 * @param robot 机器人指针
 * @param R_sensor 传感器相对于末端执行器的旋转矩阵
 * @param count 校准数据数量，默认3
 * @return int 0表示成功，-1表示失败
 */
std::unordered_map<std::string, Eigen::Vector3d>
test_gravity_compensation(int &signal, std::shared_ptr<ati::FTSensor> sensor,
                          std::shared_ptr<Robot> robot,
                          Eigen::Matrix3d R_sensor, int count = 3);
/**
 * @brief 测试重力分解
 * @param sensor 传感器指针
 * @param robot 机器人指针
 * @param R_sensor 传感器相对于末端执行器的旋转矩阵
 * @param params 重力补偿参数
 * @return int 0表示成功，-1表示失败
 */
int test_gravity_decompensation(
    std::shared_ptr<ati::FTSensor> sensor, std::shared_ptr<Robot> robot,
    Eigen::Matrix3d R_sensor,
    const std::unordered_map<std::string, Eigen::Vector3d> &params);