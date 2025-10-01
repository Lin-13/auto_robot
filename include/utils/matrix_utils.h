#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <opencv2/opencv.hpp>
#pragma once
Eigen::MatrixXd HomoMatrix(const Eigen::Matrix3d &R, const Eigen::Vector3d &t);
Eigen::MatrixXd cvMatToEigenXd(const cv::Mat &cv_mat);
cv::Mat eigenXdToCvMat(const Eigen::MatrixXd &eigen_mat);
KDL::Frame eigenXdToKdlFrame(const Eigen::MatrixXd &eigen_mat);
Eigen::MatrixXd kdlFrameToEigenXd(const KDL::Frame &frame);
Eigen::MatrixXd RPYToRot(const Eigen::Vector3d &rpy);
Eigen::Vector3d RotToRPY(const Eigen::Matrix3d &R);
void adjustRotateInplace(Eigen::Matrix3d &R, int x, int y, int z);
std::vector<Eigen::Matrix3d> generateRandomRotations(int n);
std::vector<Eigen::Vector3d>
generateRandomTranslation(int n, double limit = 1.0, double norm_min = 0.1);
std::vector<Eigen::Matrix4d>
generateRandomTransformations(int n, double limit = 1.0, double norm_min = 0.1);
int writeEigenXdToFile(const std::string &filename, const Eigen::MatrixXd &mat);
Eigen::MatrixXd readEigenXdFromFile(const std::string &filename);

Eigen::Matrix3d
solveAXXBShiu(const Eigen::Matrix3d &A, const Eigen::Matrix3d &B,
              std::function<void(Eigen::Matrix3d &)> func = nullptr,
              double *res = nullptr);
Eigen::Matrix3d
solveAXXBKron(const Eigen::Matrix3d &A, const Eigen::Matrix3d &B,
              std::function<void(Eigen::Matrix3d &)> func = nullptr,
              double *res = nullptr);
Eigen::Matrix3d
solveAXXBKron(const std::vector<Eigen::Matrix3d> &A,
              const std::vector<Eigen::Matrix3d> &B,
              std::function<void(Eigen::Matrix3d &)> func = nullptr,
              double *res = nullptr);

Eigen::Matrix4d solveATTBKron(
    const std::vector<Eigen::Matrix3d> &R_A,
    const std::vector<Eigen::Vector3d> &p_A,
    const std::vector<Eigen::Matrix3d> &R_B,
    const std::vector<Eigen::Vector3d> &p_B,
    std::function<void(Eigen::Matrix3d &)> adjustRotateInplace = nullptr,
    double *res = nullptr);
Eigen::Matrix4d
solveATTBKron(const std::vector<Eigen::Matrix4d> &T_A,
              const std::vector<Eigen::Matrix4d> &T_B,
              std::function<void(Eigen::Matrix3d &)> func = nullptr,
              double *res = nullptr);
std::vector<Eigen::Matrix4d>
calibrationHandtoEye(const std::vector<Eigen::Matrix4d> &T_c_t,
                     const std::vector<Eigen::Matrix4d> &T_b_e,
                     std::function<void(Eigen::Matrix3d &)> adjustTbc = nullptr,
                     std::function<void(Eigen::Matrix3d &)> adjustTet = nullptr,
                     double *res = nullptr);
// 李群李代数
Eigen::Matrix3d skew(const Eigen::Vector3d &v);
Eigen::Vector3d unskew(const Eigen::Matrix3d &skew_mat);

Eigen::Vector3d SO3Toso3(const Eigen::Matrix3d &R);
Eigen::Matrix3d so3ToSO3(const Eigen::Vector3d &so3);
Eigen::VectorXd SE3Tose3(const Eigen::Matrix4d &T);
Eigen::Matrix4d se3ToSE3(const Eigen::VectorXd &se3);

// 重力补偿
double gravity_compensation(const std::vector<Eigen::Vector3d> &F_measure,
                            const std::vector<Eigen::Vector3d> &M_measure,
                            const std::vector<Eigen::Matrix3d> &R_i,
                            Eigen::Vector3d &L, Eigen::Vector3d &G_w,
                            Eigen::Vector3d &f_0, Eigen::Vector3d &m_0);