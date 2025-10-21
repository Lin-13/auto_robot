#pragma once
// #include "aubo/aubo_robot.h"
#include "ft_sensor/ft_sensor.h"
#include "robot_interface/robot.h"
#include "utils/matrix_utils.h"
#include "utils/utils.h"
#include <chrono>
#include <filesystem>
#include <fmt/ranges.h>
#include <thread>
#include <unordered_map>
constexpr char LEFT_ATI_IP[] = "192.168.1.111";
constexpr char RIGHT_ATI_IP[] = "192.168.1.112";
using FTParams = std::unordered_map<std::string, Eigen::Vector3d>;
FTParams read_gravity_calib_data(const std::string &filename);

int write_gravity_calib_data(const std::string &filename,
                             const FTParams &params);

Eigen::VectorXd decompose_force(const Eigen::Matrix3d &R,
                                const Eigen::VectorXd &wrench,
                                const FTParams &params);

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
 * @param sensor_name 传感器名称，left或right
 * @param data_folder 标定数据文件夹，默认gravity_compensation
 * @return int 0表示成功，-1表示失败
 */
std::unordered_map<std::string, Eigen::Vector3d>
test_gravity_compensation(int &signal, std::shared_ptr<ati::FTSensor> sensor,
                          std::shared_ptr<Robot> robot, int count = 3);
/**
 * @brief 测试重力分解
 *
 * @param sensor_name 传感器名称，left或right
 * @param data_folder 标定数据文件夹，默认gravity_compensation
 * @return int 0表示成功，-1表示失败
 */
int test_gravity_decompensation(
    std::shared_ptr<ati::FTSensor> sensor, std::shared_ptr<Robot> robot,
    const std::unordered_map<std::string, Eigen::Vector3d> &params);