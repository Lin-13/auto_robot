#include <Eigen/Core>
#include <Eigen/Dense>
#include <fmt/format.h>
#include <fstream>
#include <kdl/frames.hpp>
#include <opencv2/opencv.hpp>
#include <random>
#include <unsupported/Eigen/KroneckerProduct>
/**
 * @brief double cv::Mat -> EigenXd
 *
 * @param cv_mat
 * @return Eigen::Matrix3d
 */
Eigen::MatrixXd cvMatToEigenXd(const cv::Mat &cv_mat) {
  if (cv_mat.type() != CV_64F) {
    throw std::invalid_argument("cvMatToEigenXd: 输入矩阵必须是CV_64F类型");
  }
  Eigen::MatrixXd eigen_mat(cv_mat.rows, cv_mat.cols);
  for (int i = 0; i < cv_mat.rows; ++i) {
    for (int j = 0; j < cv_mat.cols; ++j) {
      eigen_mat(i, j) = cv_mat.at<double>(i, j);
    }
  }
  return eigen_mat;
}
/**
 * @brief  EigenXD -> cv::Mat
 *
 * @param eigen_mat
 * @return cv::Mat
 */
cv::Mat eigenXdToCvMat(const Eigen::MatrixXd &eigen_mat) {
  cv::Mat cv_mat(eigen_mat.rows(), eigen_mat.cols(), CV_64F);
  for (int i = 0; i < eigen_mat.rows(); ++i) {
    for (int j = 0; j < eigen_mat.cols(); ++j) {
      cv_mat.at<double>(i, j) = eigen_mat(i, j);
    }
  }
  return cv_mat;
}
Eigen::MatrixXd HomoMatrix(const Eigen::Matrix3d &R, const Eigen::Vector3d &t) {
  Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  return T;
}
/**
 * @brief EigenXd(4x4) -> KDL::Frame
 *
 * @param eigen_mat
 * @return KDL::Frame
 */
KDL::Frame eigenXdToKdlFrame(const Eigen::MatrixXd &eigen_mat) {
  if (eigen_mat.rows() != 4 || eigen_mat.cols() != 4) {
    throw std::invalid_argument("eigenXdToKdlFrame: 输入矩阵必须是4x4类型");
  }
  KDL::Rotation R(eigen_mat(0, 0), eigen_mat(0, 1), eigen_mat(0, 2),
                  eigen_mat(1, 0), eigen_mat(1, 1), eigen_mat(1, 2),
                  eigen_mat(2, 0), eigen_mat(2, 1), eigen_mat(2, 2));
  KDL::Vector p(eigen_mat(0, 3), eigen_mat(1, 3), eigen_mat(2, 3));
  return KDL::Frame(R, p);
}
/**
 * @brief KDL::Frame -> EigenXd(4x4)
 *
 * @param frame
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd kdlFrameToEigenXd(const KDL::Frame &frame) {
  Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
  KDL::Rotation M = frame.M;
  T.block<3, 3>(0, 0) << M(0, 0), M(0, 1), M(0, 2), M(1, 0), M(1, 1), M(1, 2),
      M(2, 0), M(2, 1), M(2, 2);
  KDL::Vector p = frame.p;
  T.block<3, 1>(0, 3) << p(0), p(1), p(2);
  return T;
}
/**
 * @brief RPY -> MatrixXd(3x3)
 *
 * @param rpy
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd RPYToRot(const Eigen::Vector3d &rpy) {
  double roll = rpy(0);
  double pitch = rpy(1);
  double yaw = rpy(2);
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  return R;
}

Eigen::Vector3d RotToRPY(const Eigen::Matrix3d &R) {
  double roll, pitch, yaw;
  // 计算pitch
  pitch = std::asin(-R(2, 0));
  // 计算roll和yaw
  if (std::cos(pitch) > 1e-6) { // 避免除以零
    roll = std::atan2(R(2, 1), R(2, 2));
    yaw = std::atan2(R(1, 0), R(0, 0));
  } else {
    // Gimbal lock情况
    roll = 0;
    yaw = std::atan2(-R(0, 1), R(1, 1));
  }
  Eigen::Vector3d rpy;
  rpy << roll, pitch, yaw;
  return rpy;
}

/**
 * @brief 调整旋转矩阵
 * @details 调整矩阵det=1
 *          调整轴的朝向，1代表正方向，-1代表负方向，
 *          0代表不做约束（会在调整det和其他轴时被动调整
 *
 * @param R
 * @param x X轴朝向
 * @param y Y轴朝向
 * @param z Z轴朝向
 */
void adjustRotateInplace(Eigen::Matrix3d &R, int x, int y, int z) {
  assert(x == 1 || x == -1 || x == 0);
  assert(y == 1 || y == -1 || y == 0);
  assert(z == 1 || z == -1 || z == 0);
  std::vector<int> free_axis = {};
  if (x == 0)
    free_axis.push_back(0);
  if (y == 0)
    free_axis.push_back(1);
  if (z == 0)
    free_axis.push_back(2);
  if (free_axis.size() == 3) {
    return;
  } else if (free_axis.size() == 0) {
    throw std::runtime_error("过约束");
  }
  if (R.determinant() < 0) {
    R.col(free_axis[0]) = -R.col(free_axis[0]);
  }
  // 纠正x轴
  if (x != 0 && R.col(0).dot(Eigen::Vector3d(x, 0, 0)) < 0) {
    R.col(0) = -R.col(0);
    R.col(free_axis[0]) = -R.col(free_axis[0]);
  }
  //
  if (y != 0 && R.col(1).dot(Eigen::Vector3d(0, y, 0)) < 0) {
    R.col(1) = -R.col(1);
    R.col(free_axis[0]) = -R.col(free_axis[0]);
  }
  // 纠正Z轴
  if (z != 0 && R.col(2).dot(Eigen::Vector3d(0, 0, z)) < 0) {
    R.col(2) = -R.col(2);
    R.col(free_axis[0]) = -R.col(free_axis[0]);
  }
}

/**
 * @brief 随机生成旋转矩阵 先随机生成RPY在RPRPYToRot
 * @details RPY角在 [-pi,pi]之间，并且 R*R+P*P+Y*Y > 0.01
 * @param n 旋转矩阵数量
 * @return 旋转矩阵 std::vector<Eigen::Matrix3d>
 */
std::vector<Eigen::Matrix3d> generateRandomRotations(int n) {
  std::vector<Eigen::Matrix3d> rotations;
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_real_distribution<double> distribution(-3.14, 3.14);
  for (int i = 0; i < n; ++i) {
    double roll, pitch, yaw;
    do {
      roll = distribution(generator);
      pitch = distribution(generator);
      yaw = distribution(generator);
    } while (roll * roll + pitch * pitch + yaw * yaw < 0.01);
    Eigen::Vector3d rpy(roll, pitch, yaw);
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    rotations.push_back(R);
  }
  return rotations;
}

/**
 * @brief 在[-limit ,limit]中生成均匀分布的n个向量(norm > norm_min),
 *
 * @param n 随机向量的个数
 * @param limit 随机向量的范围
 * @param norm_min 随机向量的最小模长
 * @return std::vector<Eigen::Vector3d>
 */
std::vector<Eigen::Vector3d> generateRandomTranslation(int n, double limit,
                                                       double norm_min) {
  std::vector<Eigen::Vector3d> translations;
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_real_distribution<double> distribution(-limit, limit);
  for (int i = 0; i < n; ++i) {
    double x = 0, y = 0, z = 0;
    do {
      x = distribution(generator);
      y = distribution(generator);
      z = distribution(generator);
    } while (x * x + y * y + z * z < norm_min);
    Eigen::Vector3d t(x, y, z);
    translations.push_back(t);
  }
  return translations;
}
std::vector<Eigen::Matrix4d> generateRandomTransformations(int n, double limit,
                                                           double norm_min) {
  std::vector<Eigen::Matrix4d> transformations;
  auto R = generateRandomRotations(n);
  auto p = generateRandomTranslation(n, limit, norm_min);
  for (int idx = 0; idx < n; ++idx) {
    transformations.push_back(HomoMatrix(R[idx], p[idx]));
  }
  return transformations;
}
/**
 * @brief 读取Eigen矩阵
 * @param filename 文件名
 * @param mat 矩阵
 * @return 读取成功返回0，否则返回-1
 */
Eigen::MatrixXd readEigenXdFromFile(const std::string &filename) {
  std::ifstream infile(filename);
  Eigen::MatrixXd mat;
  if (!infile.is_open()) {
    fmt::print("无法打开文件: {}\n", filename);
    return mat;
  }
  std::vector<std::vector<double>> data;
  std::string line;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    std::vector<double> row;
    double value;
    while (iss >> value) {
      row.push_back(value);
    }
    if (!row.empty()) {
      data.push_back(row);
    }
  }
  infile.close();
  if (data.empty()) {
    fmt::print("文件为空或格式错误: {}\n", filename);
    return mat;
  }
  size_t rows = data.size();
  size_t cols = data[0].size();
  mat.resize(rows, cols);
  for (size_t i = 0; i < rows; ++i) {
    for (size_t j = 0; j < cols; ++j) {
      mat(i, j) = data[i][j];
    }
  }
  return mat;
}
/**
 * @brief 将Eigen::MatrixXd写入文件
 *
 * @param filename 文件名
 * @param mat 矩阵
 * @return int 0:成功, -1:失败
 */
int writeEigenXdToFile(const std::string &filename,
                       const Eigen::MatrixXd &mat) {
  std::ofstream outfile(filename);
  if (!outfile.is_open()) {
    fmt::print("无法打开文件: {}\n", filename);
    return -1;
  }
  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      outfile << mat(i, j);
      if (j < mat.cols() - 1) {
        outfile << " ";
      }
    }
    outfile << "\n";
  }
  outfile.close();
  return 0;
}

/**
 * @brief 使用Shiu的算法求解AX=XB问题
 *  TODO: 算法存在问题 输出结果未能通过测试
    Given A and B, solve for X in the equation AX = XB using Shiu's method.
    This function assumes A and B are proper rotation matrices (3x3).
    The solution X is also a rotation matrix (3x3).
 *
 * @param A
 * @param B
 * @param res AX=XB 的残差(AX-XB)
 * @param adjustRotateInplace 调整旋转矩阵的函数
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d
solveAXXBShiu(const Eigen::Matrix3d &A, const Eigen::Matrix3d &B,
              std::function<void(Eigen::Matrix3d &)> adjustRotateInplace,
              double *res) {
  Eigen::Matrix3d A_rot = A;
  Eigen::Matrix3d B_rot = B;
  // 计算迹
  double trA = A_rot.trace();
  double trB = B_rot.trace();

  Eigen::Matrix3d M;

  // 填充M矩阵 (根据Shiu算法推导)
  M(0, 0) = trA + 1.0;
  M(0, 1) = A_rot(2, 1) - A_rot(1, 2);
  M(0, 2) = A_rot(0, 2) - A_rot(2, 0);

  M(1, 0) = A_rot(2, 1) - A_rot(1, 2);
  M(1, 1) = trA + 1.0;
  M(1, 2) = A_rot(1, 0) - A_rot(0, 1);

  M(2, 0) = A_rot(0, 2) - A_rot(2, 0);
  M(2, 1) = A_rot(1, 0) - A_rot(0, 1);
  M(2, 2) = trA + 1.0;

  // 构建右侧向量b
  Eigen::Vector3d b;
  b(0) = B_rot(0, 0) - 1.0;
  b(1) = B_rot(1, 0);
  b(2) = B_rot(2, 0);

  // 求解线性方程组M * x = b，得到旋转向量的分量
  Eigen::Vector3d x = M.colPivHouseholderQr().solve(b);

  // 从旋转向量构建旋转矩阵（使用罗德里格斯公式）
  double theta = x.norm();
  Eigen::Matrix3d X;

  if (theta < 1e-8) {
    // 角度接近0，返回单位矩阵
    X = Eigen::Matrix3d::Identity();
  } else {
    Eigen::Vector3d u = x.normalized();
    Eigen::Matrix3d u_hat;

    // 构建反对称矩阵
    u_hat << 0, -u(2), u(1), u(2), 0, -u(0), -u(1), u(0), 0;

    // 罗德里格斯公式
    X = Eigen::Matrix3d::Identity() + sin(theta) * u_hat +
        (1 - cos(theta)) * u_hat * u_hat;
  }
  if (adjustRotateInplace != nullptr) {
    adjustRotateInplace(X);
  }
  if (res != nullptr) {
    *res = (A_rot * X - X * B_rot).norm();
  }
  return X;
}
/**
 * @brief 通过Kronecker积求解AX=XB问题
 *        该问题存在多解，该方法进保证`det(X) = 1` 需要调用者进行额外调整
 * @param A N个3x3旋转矩阵
 * @param adjustRotateInplace 调整旋转矩阵
 * @param B N个3x3旋转矩阵
 * @param res AX=XB 的残差(AX-XB)
 * @return Eigen::Matrix3d X
 */
Eigen::Matrix3d
solveAXXBKron(const std::vector<Eigen::Matrix3d> &A,
              const std::vector<Eigen::Matrix3d> &B,
              std::function<void(Eigen::Matrix3d &)> adjustRotateInplace,
              double *res) {
  if (A.size() != B.size() || A.empty()) {
    throw std::invalid_argument("solveAXXBKron: A和B的数量必须相同且非空");
  }
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(9 * A.size(), 9);
  for (size_t i = 0; i < A.size(); ++i) {
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    M.block<9, 9>(9 * i, 0) = Eigen::kroneckerProduct(A[i], I) -
                              Eigen::kroneckerProduct(I, B[i].transpose());
  }
  // 使用SVD求解M * vec(X) = 0
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV);
  Eigen::VectorXd vecX = svd.matrixV().col(8); // 取最后一列对应最小奇异值
  // 将vec(X)重塑为3x3矩阵X
  Eigen::Matrix3d X;
  X << vecX(0), vecX(1), vecX(2), vecX(3), vecX(4), vecX(5), vecX(6), vecX(7),
      vecX(8);
  // 确保X是一个正交矩阵（旋转矩阵）
  Eigen::JacobiSVD<Eigen::Matrix3d> svd2(X, Eigen::ComputeFullU |
                                                Eigen::ComputeFullV);
  X = svd2.matrixU() * svd2.matrixV().transpose();
  // 确保行列式为1
  if (X.determinant() < 0) {
    X.col(2) *= -1;
  }
  // auto correctRotateInplace = [](Eigen::Matrix3d &R) {
  //   if (R.determinant() < 0) {
  //     R.col(1) = -R.col(1);
  //   }
  //   // 纠正x轴指向正方向
  //   if (R.col(0).dot(Eigen::Vector3d(1, 0, 0)) < 0) {
  //     R.col(0) = -R.col(0);
  //     R.col(2) = -R.col(2);
  //   }
  //   // 纠正Z轴指向正方向
  //   if (R.col(2).dot(Eigen::Vector3d(0, 0, -1)) < 0) {
  //     R.col(1) = -R.col(1);
  //     R.col(2) = -R.col(2);
  //   }
  // };
  if (adjustRotateInplace != nullptr) {
    adjustRotateInplace(X);
  }
  if (res != nullptr) {
    *res = 0;
    for (int i = 0; i < A.size(); i++) {
      *res += (A[i] * X - X * B[i]).norm();
    }
    *res = *res / A.size();
  }
  return X;
}
/**
 * @brief 通过Kronecker积求解AX=XB问题
 *        该问题存在多解，该方法进保证`det(X) = 1` 需要调用者进行额外调整
 *
 * @param A
 * @param B
 * @param correctRotateInplace 旋转矩阵的修正函数
 * @param res AX=XB 的残差(AX-XB) ,默认 nullptr
 * @return Eigen::Matrix3d
 */
Eigen::Matrix3d
solveAXXBKron(const Eigen::Matrix3d &A, const Eigen::Matrix3d &B,
              std::function<void(Eigen::Matrix3d &)> adjustRotateInplace,
              double *res) {
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(9, 9);
  // 构建矩阵M
  M = Eigen::kroneckerProduct(A, I) - Eigen::kroneckerProduct(I, B.transpose());
  // 使用SVD求解M * vec(X) = 0
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV);
  Eigen::VectorXd vecX = svd.matrixV().col(8); // 取最后一列对应最小奇异值
  // 将vec(X)重塑为3x3矩阵X
  Eigen::Matrix3d X;
  X << vecX(0), vecX(1), vecX(2), vecX(3), vecX(4), vecX(5), vecX(6), vecX(7),
      vecX(8);
  // 确保X是一个正交矩阵（旋转矩阵）
  Eigen::JacobiSVD<Eigen::Matrix3d> svd2(X, Eigen::ComputeFullU |
                                                Eigen::ComputeFullV);
  X = svd2.matrixU() * svd2.matrixV().transpose();
  // 确保行列式为1
  if (X.determinant() < 0) {
    X.col(2) *= -1;
  }
  if (adjustRotateInplace != nullptr) {
    adjustRotateInplace(X);
  }
  if (res != nullptr) {
    *res = 0;
    *res = (A * X - X * B).norm();
  }
  return X;
}

/**
 * @brief 通过Kronecker积求解AT=TB问题(T表示齐次矩阵以区分AX=XB问题)
 * @details 通过调用 solveAXXBKron 求解AX=XB问题，
 *          然后求解平衡方程R_A*p_X + p_A = R_X * p_B +p_X
 *
 * @param R_A
 * @param p_A
 * @param R_B
 * @param p_B
 * @param adjustRotateInplace
 * @param res
 * @return Eigen::Matrix4d
 */
Eigen::Matrix4d
solveATTBKron(const std::vector<Eigen::Matrix3d> &R_A,
              const std::vector<Eigen::Vector3d> &p_A,
              const std::vector<Eigen::Matrix3d> &R_B,
              const std::vector<Eigen::Vector3d> &p_B,
              std::function<void(Eigen::Matrix3d &)> adjustRotateInplace,
              double *res) {
  auto R_X1 = solveAXXBKron(R_A, R_B, adjustRotateInplace, res);

  // solve (R_Ai - I)*pX=p_A+R_X*p_B
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3 * R_A.size(), 3);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(3 * R_A.size());
  auto I = Eigen::Matrix3d::Identity();
  for (int i = 0; i < R_A.size(); i++) {
    A.block<3, 3>(3 * i, 0) = R_A[i] - I;
    b.block<3, 1>(3 * i, 0) = -p_A[i] + R_X1 * p_B[i];
  }
  // Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr_A(A);
  // auto p_X1 = qr_A.solve(b);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_A(A, Eigen::ComputeFullU |
                                                 Eigen::ComputeFullV);
  Eigen::VectorXd p_X1 = svd_A.solve(b);
  auto T_X1 = HomoMatrix(R_X1, p_X1);
  return HomoMatrix(R_X1, p_X1);
}

/**
 * @brief 通过Kronecker积求解AT=TB问题
 * @details 通过将齐次坐标分解在调用solveATTBKron 函数计算
 *
 * @param T_A
 * @param T_B
 * @param adjustRotateInplace
 * @param res
 * @return Eigen::Matrix4d
 */
Eigen::Matrix4d
solveATTBKron(const std::vector<Eigen::Matrix4d> &T_A,
              const std::vector<Eigen::Matrix4d> &T_B,
              std::function<void(Eigen::Matrix3d &)> adjustRotateInplace,
              double *res) {
  if (T_A.size() != T_B.size() || T_B.empty()) {
    throw std::invalid_argument("solveATTBKron: TA和TB的数量必须相同且非空");
  }
  std::vector<Eigen::Matrix3d> R_A, R_B;
  std::vector<Eigen::Vector3d> p_A, p_B;
  for (const auto &T : T_A) {
    R_A.push_back(T.block<3, 3>(0, 0));
    p_A.push_back(T.block<3, 1>(0, 3));
  }
  for (const auto &T : T_B) {
    R_B.push_back(T.block<3, 3>(0, 0));
    p_B.push_back(T.block<3, 1>(0, 3));
  }
  auto X = solveATTBKron(R_A, p_A, R_B, p_B, adjustRotateInplace, res);
  return X;
}

/**
 * @brief 眼在手外标定，通过Kronecker积求解
 *
 * @param T_c_t 相机到工具的变换矩阵
 * @param T_b_e 基座到末端的变换矩阵
 * @param adjustTbc 对T_bc(base->相机)的修正函数
 * @param adjustTet 对T_et(相机->工具)的修正函数correctTransformInplace
 * @param res 残差指针，默认 nullptr
 * @return std::vector<Eigen::Matrix4d> {T_bc, T_et}
 */
std::vector<Eigen::Matrix4d>
calibrationHandtoEye(const std::vector<Eigen::Matrix4d> &T_c_t,
                     const std::vector<Eigen::Matrix4d> &T_b_e,
                     std::function<void(Eigen::Matrix3d &)> adjustTbc,
                     std::function<void(Eigen::Matrix3d &)> adjustTet,
                     double *res) {
  if (T_c_t.size() != T_b_e.size() || T_b_e.empty()) {
    throw std::invalid_argument(
        "calibrationHandtoEye: T_c_t和T_b_e的数量必须相同且非空");
  }
  std::vector<Eigen::Matrix4d> T_ct_r, T_be_r, T_tc_r, T_eb_r;
  for (int i = 0; i < T_c_t.size() - 1; i++) {
    T_ct_r.push_back(T_c_t[i] * T_c_t[i + 1].inverse());
    T_be_r.push_back(T_b_e[i] * T_b_e[i + 1].inverse());
    T_tc_r.push_back(T_c_t[i].inverse() * T_c_t[i + 1]);
    T_eb_r.push_back(T_b_e[i].inverse() * T_b_e[i + 1]);
  }
  double res_tbc = 0, res_tet = 0;
  auto T_bc = solveATTBKron(T_be_r, T_ct_r, adjustTbc, &res_tbc);
  auto T_et = solveATTBKron(T_eb_r, T_tc_r, adjustTet, &res_tet);
  if (res != nullptr) {
    *res = res_tbc + res_tet;
  }
  return {T_bc, T_et};
}
Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d skew_mat = Eigen::Matrix3d::Zero();
  skew_mat << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return skew_mat;
}
Eigen::Vector3d unskew(const Eigen::Matrix3d &skew_mat) {
  Eigen::Vector3d v;
  v << skew_mat(2, 1), skew_mat(0, 2), skew_mat(1, 0);
  return v;
}
/**
 * @brief 计算旋转矩阵的李代数
 * @details 李代数是旋转矩阵的向量表示，用于描述旋转的角度和轴
 *
 * @param R 旋转矩阵
 * @return Eigen::Vector3d so3李代数
 */
Eigen::Vector3d SO3Toso3(const Eigen::Matrix3d &R) {
  Eigen::Vector3d lie_algebra;
  double trace = R.trace();
  if (trace > 1) {
    trace = 1;
  }
  if (trace < -1) {
    trace = -1;
  }
  double theta = acos((trace - 1) / 2);
  if (fabs(theta) < 1e-6) {
    lie_algebra = Eigen::Vector3d::Zero();
  } else {
    // Eigen::Vector3d axis =
    //     (R - R.transpose()).block<3, 1>(0, 2) / (2 * sin(theta));
    Eigen::Vector3d axis;
    axis[0] = R(2, 1) - R(1, 2); // (R-R^T)(2,1)
    axis[1] = R(0, 2) - R(2, 0); // (R-R^T)(0,2)
    axis[2] = R(1, 0) - R(0, 1); // (R-R^T)(1,0)
    axis /= 2 * sin(theta);
    lie_algebra = theta * axis;
  }
  return lie_algebra;
}
Eigen::Matrix3d so3ToSO3(const Eigen::Vector3d &so3) {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  double theta = so3.norm();
  if (fabs(theta) < 1e-6) {
    return R;
  }
  Eigen::Vector3d axis = so3 / theta;
  Eigen::Matrix3d skew_mat = skew(axis);
  R += sin(theta) * skew_mat + (1 - cos(theta)) * skew_mat * skew_mat;
  return R;
}
/**
 * @brief 计算旋转矩阵的李代数的雅可比矩阵
 * @details 雅可比矩阵是旋转矩阵的导数，用于描述旋转的角度和轴的变化
 *
 * @param omega so3李代数
 * @return Eigen::Matrix3d 雅可比矩阵
 */
Eigen::Matrix3d computeJ(const Eigen::Vector3d &omega) {
  double theta = omega.norm();
  if (theta < 1e-6) {
    return Eigen::Matrix3d::Identity() + 0.5 * skew(omega); // 小角度近似
  }
  Eigen::Vector3d axis = omega / theta;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d skew_axis = skew(axis);
  return (sin(theta) / theta) * I +
         (1 - sin(theta) / theta) * (axis * axis.transpose()) +
         ((1 - cos(theta)) / theta) * skew_axis;
}

/**
 * @brief 雅可比矩阵的逆J^{-1}（用于SE(3)对数映射）
 * @details 雅可比矩阵的逆用于将旋转矩阵的李代数转换为旋转矩阵
 *
 * @param omega so3李代数
 * @return Eigen::Matrix3d 雅可比矩阵的逆
 */
Eigen::Matrix3d computeJinv(const Eigen::Vector3d &omega) {
  double theta = omega.norm();
  if (theta < 1e-6) {
    return Eigen::Matrix3d::Identity() - 0.5 * skew(omega); // 小角度近似
  }
  Eigen::Vector3d axis = omega / theta;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d skew_axis = skew(axis);
  double cot_half = 1.0 / tan(theta / 2);
  return (theta / 2 * cot_half) * I +
         (1 - theta / 2 * cot_half) * (axis * axis.transpose()) -
         (theta / 2) * skew_axis;
}
/**
 * @brief se3李代数到SE3变换矩阵
 *
 * @param lie_algebra se3李代数[omega,v]T
 * @return Eigen::Matrix4d SE3变换矩阵
 */
Eigen::Matrix4d se3ToSE3(const Eigen::VectorXd &se3) {
  if (se3.size() != 6) {
    throw std::invalid_argument("se3ToSE3: se3的大小必须为6");
  }
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  Eigen::Vector3d omega = se3.head<3>();
  Eigen::Vector3d v = se3.tail<3>();
  T.block<3, 3>(0, 0) = so3ToSO3(omega);
  T.block<3, 1>(0, 3) = computeJ(omega) * v;
  return T;
}
/**
 * @brief SE3变换矩阵到se3李代数
 *
 * @param T SE3变换矩阵
 * @return Eigen::VectorXd se3李代数 [omega,v]T
 */
Eigen::VectorXd SE3Tose3(const Eigen::Matrix4d &T) {
  Eigen::VectorXd se3 = Eigen::VectorXd::Zero(6);
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Vector3d t = T.block<3, 1>(0, 3);
  Eigen::Vector3d omega = SO3Toso3(R);
  se3.head<3>() = omega;
  se3.tail<3>() = computeJinv(omega) * t;
  return se3;
}
// Eigen::Matrix4d se3ToSE3(const Eigen::VectorXd &lie_algebra) {
//   if (lie_algebra.size() != 6) {
//     throw std::invalid_argument("se3ToSE3: lie_algebra的大小必须为6");
//   }
//   Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
//   Eigen::Vector3d p = lie_algebra.head<3>();
//   Eigen::Vector3d so3 = lie_algebra.tail<3>();
//   T.block<3, 3>(0, 0) = so3ToSO3(so3);
//   T.block<3, 1>(0, 3) = p;
//   return T;
// }
// Eigen::VectorXd SE3Tose3(const Eigen::Matrix4d &T) {
//   Eigen::VectorXd lie_algebra = Eigen::VectorXd::Zero(6);
//   lie_algebra.head<3>() = T.block<3, 1>(0, 3);
//   lie_algebra.tail<3>() = SO3Toso3(T.block<3, 3>(0, 0));
//   return lie_algebra;
// }

/**
 * @brief 重力补偿
 * @details
        求解方程组F_measure[i] = R_{i}^T * G_w +f_0
        M_measure[i] = L cross (R_{i}^T * G_w) +m_0
        其中 G_w 为世界坐标系中的重力向量{0,0,-mg}，
        f_0 为力漂移向量，m_0为力矩漂移向量,
        R_{i} 为世界坐标系中的力传感器位姿 L 为传感器坐标系中的重心坐标
 * @param F_measure 力传感器测量值
 * @param M_measure 力矩传感器测量值
 * @param R_i 力传感器位姿 - 世界坐标系中测得
 * @param L 传感器坐标系中的重心坐标
 * @param G_w 世界坐标系中的重力向量
 * @param f_0 为力漂移向量
 * @param m_0 为力矩漂移向量
 * @return double 重力补偿值
 */
double gravity_compensation(const std::vector<Eigen::Vector3d> &F_measure,
                            const std::vector<Eigen::Vector3d> &M_measure,
                            const std::vector<Eigen::Matrix3d> &R_i,
                            Eigen::Vector3d &L, Eigen::Vector3d &G_w,
                            Eigen::Vector3d &f_0, Eigen::Vector3d &m_0) {
  if (F_measure.size() != M_measure.size()) {
    throw std::invalid_argument(
        "F_measure and M_measure must have the same size");
  }
  // 首先通过方程组F_measure[i] = R_{i}^T * G_w +f_0
  // 来求解f_0，G_W
  Eigen::VectorXd F = Eigen::VectorXd::Zero(F_measure.size() * 3);
  for (int i = 0; i < F_measure.size(); ++i) {
    F.block<3, 1>(i * 3, 0) = F_measure[i];
  }
  Eigen::MatrixXd R_0 = Eigen::MatrixXd::Zero(R_i.size() * 3, 6);
  for (int i = 0; i < R_i.size(); ++i) {
    R_0.block<3, 3>(i * 3, 0) = R_i[i].transpose();
    R_0.block<3, 3>(i * 3, 3) = Eigen::Matrix3d::Identity();
  }
  Eigen::VectorXd x =
      R_0.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(F);
  Eigen::Vector3d G_w_estimate = x.head<3>();
  Eigen::Vector3d f_0_estimate = x.tail<3>();

  // 然后通过方程组M_measure[i] = L cross (R_{i}^T * G_w) +m_0
  // 改写为 M_measure[i] = (F_measure[i] - f_0_estimate) * [L] + m_0
  // 同样求解L,m_0
  Eigen::VectorXd M = Eigen::VectorXd::Zero(M_measure.size() * 3);
  for (int i = 0; i < M_measure.size(); ++i) {
    M.block<3, 1>(i * 3, 0) = M_measure[i];
  }
  Eigen::VectorXd F_1 = Eigen::VectorXd::Zero(F_measure.size() * 3);
  for (int i = 0; i < F_measure.size(); ++i) {
    F_1.block<3, 1>(i * 3, 0) = F_measure[i] - f_0_estimate;
  }
  Eigen::MatrixXd R_1 = Eigen::MatrixXd::Zero(F_measure.size() * 3, 6);
  for (int i = 0; i < F_measure.size(); ++i) {
    R_1.block<3, 3>(i * 3, 0) = skew(F_1.block<3, 1>(i * 3, 0));
    R_1.block<3, 3>(i * 3, 3) = Eigen::Matrix3d::Identity();
  }
  Eigen::VectorXd x_1 =
      R_1.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(M);
  Eigen::Vector3d L_estimate = x_1.head<3>();
  Eigen::Vector3d m_0_estimate = x_1.tail<3>();
  // 计算残差
  double res = 0;
  for (int i = 0; i < F_measure.size(); ++i) {
    res += (F_measure[i] - R_i[i].transpose() * G_w_estimate - f_0_estimate)
               .squaredNorm();
  }
  for (int i = 0; i < M_measure.size(); ++i) {
    res += (M_measure[i] - L_estimate.cross(R_i[i].transpose() * G_w_estimate) -
            m_0_estimate)
               .squaredNorm();
  }
  // 更新结果
  L = L_estimate;
  G_w = G_w_estimate;
  f_0 = f_0_estimate;
  m_0 = m_0_estimate;
  return res;
}