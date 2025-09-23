#include <fmt/format.h>
#include <gtest/gtest.h>
#include <random>
#include <ranges>
#include <utils/matrix_utils.h>
#include <utils/utils.h>
// 简单的测试用例

void correctRotateInplace(Eigen::Matrix3d &R) {
  adjustRotateInplace(R, 1, 0, 1);
}
void correctTransformInplace(Eigen::Matrix4d &T) {
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  correctRotateInplace(R);
  T.block<3, 3>(0, 0) = R;
}
// std::vector<Eigen::MatrixXd>
TEST(MatrixTest, ShiuIdentity) {
  Eigen::Matrix3d X = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d B = Eigen::Matrix3d::Identity();
  X = solveAXXBShiu(A, B);
  EXPECT_EQ(X, Eigen::Matrix3d::Identity());
}

TEST(MatrixTest, KronIdentity) {
  Eigen::Matrix3d X = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d B = Eigen::Matrix3d::Identity();
  X = solveAXXBKron(A, B);
  EXPECT_EQ(X, Eigen::Matrix3d::Identity());
}
// TEST(MatrixTest, ShiuRotationRandom) {
//   auto A_X = generateRandomRotations(2);

//   Eigen::Matrix3d A = A_X[0];
//   Eigen::Matrix3d X = A_X[1];
//   Eigen::PartialPivLU<Eigen::Matrix3d> lu(X);
//   Eigen::Matrix3d B = lu.solve(A * X);
//   auto X1 = solveAXXBShiu(A, B);
//   //   std::cout << "|A|=" << A.determinant() << ", |B|=" << B.determinant()
//   //             << ", |X|=" << X.determinant() << ", |X1|=" <<
//   X1.determinant()
//   //             << std::endl;
//   //   std::cout << "X\n"
//   //             << X << std::endl
//   //             << "X1\n"
//   //             << X1 << std::endl
//   //             << "A*X - X*B\n"
//   //             << A * X - X * B << std::endl
//   //             << "A*X1 - X1*B\n"
//   //             << A * X1 - X1 * B << std::endl;
//   // fmt::print("A:\n{}\nB:\n{}\nX:\n{}\nX1:\n{}\n",A, B, X, X1);
//   EXPECT_TRUE(X.isApprox(X1, 0.01));
// }
TEST(MatrixTest, KronRotationRandom) {
  auto A = generateRandomRotations(10);
  auto X = generateRandomRotations(1);
  correctRotateInplace(X[0]);
  std::vector<Eigen::Matrix3d> B;
  std::for_each(A.begin(), A.end(), [&](const Eigen::Matrix3d &Ai) {
    Eigen::PartialPivLU<Eigen::Matrix3d> lu(X[0]);
    B.push_back(lu.solve(Ai * X[0]));
  });
  double res;
  auto X1 = solveAXXBKron(A, B, correctRotateInplace, &res);
  // correctRotateInplace(X1);
  std::stringstream ss;
  ss << "Test KronRotationRandom\n";

  for (int i = 0; i < A.size(); ++i) {
    Eigen::Matrix3d Ai = A[i];
    Eigen::Matrix3d Bi = B[i];
    // std::cout << "Pair " << i << ":\n";
    // std::cout << "|A|=" << Ai.determinant() << ", |B|=" << Bi.determinant()
    //           << ", |X|=" << X[0].determinant() << ", |X1|=" <<
    //           X1.determinant()
    //           << std::endl;
    // std::cout << "A*X - X*B\n"
    //           << (Ai * X[0] - X[0] * Bi).norm() << std::endl
    ss << "|A*X1 - X1*B|2: " << (Ai * X1 - X1 * Bi).norm() << std::endl;
  }
  ss << "X\n" << X[0] << std::endl << "X1\n" << X1 << std::endl;
  ss << "|X-X1|2 : " << (X[0] - X1).norm() << std::endl;
  // std::cout << ss.str();
  double test_res = 0;
  for (int i = 0; i < A.size(); i++) {
    test_res += (A[i] * X1 - X1 * B[i]).norm();
  }
  test_res = test_res / A.size();
  ss << "Average |R_A*R_X1 - R_X1*R_B|2 = " << test_res << std::endl;
  EXPECT_TRUE(X[0].isApprox(X1, 1e-6)) << ss.str();
  EXPECT_LE(res, 1e-6) << ss.str();
  // fmt::print("A:\n{}\nB:\n{}\nX:\n{}\nX1:\n{}\n",A, B, X, X1);
}
TEST(MatrixTest, KronHomoMatirx) {
  auto R_A = generateRandomRotations(10);
  auto R_X = generateRandomRotations(1);
  correctRotateInplace(R_X[0]);
  std::vector<Eigen::Matrix3d> R_B;
  std::for_each(R_A.begin(), R_A.end(), [&](const Eigen::Matrix3d &Ai) {
    // solve AX=XB
    // Eigen::PartialPivLU<Eigen::Matrix3d> lu(R_X[0]);
    // R_B.push_back(lu.solve(Ai * R_X[0]));
    R_B.push_back(R_X[0].transpose() * Ai * R_X[0]);
  });
  auto p_A = generateRandomTranslation(R_A.size(), 1);
  auto p_X = generateRandomTranslation(1, 1);
  std::vector<Eigen::Vector3d> p_B;
  // auto ziped = std::views::zip(R_A, p_A, R_B, p_B);
  Eigen::PartialPivLU<Eigen::Matrix3d> lu(R_X[0]);
  std::stringstream ss;
  for (int idx = 0; idx < R_A.size(); ++idx) {
    p_B.push_back(lu.solve(R_A[idx] * p_X[0] + p_A[idx] - p_X[0]));
    // p_B.push_back(R_X[0].transpose() * (R_A[idx] * p_X[0] + p_A[idx] -
    // p_X[0]));
    auto T_A = HomoMatrix(R_A[idx], p_A[idx]);
    auto T_X = HomoMatrix(R_X[0], p_X[0]);
    auto T_B = HomoMatrix(R_B[idx], p_B[idx]);
    // ss << "A:\n" << T_A << std::endl << "B:\n" << T_B << std::endl;
    // ss << "Index " << idx << " :"
    //    << "|T_A*T_X-T_X*T_B|2:" << (T_A * T_X - T_X * T_B).norm() <<
    //    std::endl;
  }
  // solve R_A*R_X = R_X*R_B
  double res = 0;
  auto R_X1 = solveAXXBKron(R_A, R_B, correctRotateInplace, &res);
  // correctRotateInplace(R_X1);
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
  auto T_X = HomoMatrix(R_X[0], p_X[0]);
  auto T_X1 = HomoMatrix(R_X1, p_X1);
  ss << "T_X:\n" << T_X << std::endl << "T_X1:\n" << T_X1 << std::endl;
  ss << "|T_X - T_X1|2 = " << (T_X - T_X1).norm() << std::endl;
  ss << "det R_X1 = " << R_X1.determinant() << std::endl
     << "det R_X = " << R_X[0].determinant() << std::endl;
  double test_res = 0;
  for (int i = 0; i < R_A.size(); i++) {
    test_res += (R_A[i] * R_X1 - R_X1 * R_B[i]).norm();
  }
  test_res = test_res / A.size();
  ss << "Average |R_A*R_X1 - R_X1*R_B|2 = " << test_res << std::endl;
  // std::cout << ss.str();
  EXPECT_TRUE(T_X.isApprox(T_X1, 1e-6)) << ss.str();
  EXPECT_LE(res, 1e-6) << ss.str();
}
TEST(MatrixTest, KronHomoMatirxATTBKron) {
  auto R_A = generateRandomRotations(10);
  auto R_X = generateRandomRotations(1);
  correctRotateInplace(R_X[0]);
  std::vector<Eigen::Matrix3d> R_B;
  std::for_each(R_A.begin(), R_A.end(), [&](const Eigen::Matrix3d &Ai) {
    R_B.push_back(R_X[0].transpose() * Ai * R_X[0]);
  });
  auto p_A = generateRandomTranslation(R_A.size(), 1);
  auto p_X = generateRandomTranslation(1, 1);
  std::vector<Eigen::Vector3d> p_B;
  std::stringstream ss;
  for (int idx = 0; idx < R_A.size(); ++idx) {
    p_B.push_back(R_X[0].transpose() * (R_A[idx] * p_X[0] + p_A[idx] - p_X[0]));
  }
  double res = 0;
  auto T_X = HomoMatrix(R_X[0], p_X[0]);
  std::vector<Eigen::Matrix4d> T_A, T_B;
  for (int idx = 0; idx < R_A.size(); ++idx) {
    T_A.push_back(HomoMatrix(R_A[idx], p_A[idx]));
    T_B.push_back(HomoMatrix(R_B[idx], p_B[idx]));
  }
  auto T_X1 = solveATTBKron(T_A, T_B, correctRotateInplace, &res);
  ss << "T_X:\n" << T_X << "\nT_X1:\n" << T_X1 << std::endl;
  ss << "|T_X - T_X1|2 = " << (T_X - T_X1).norm() << std::endl;
  EXPECT_TRUE(T_X.isApprox(T_X1, 1e-6)) << ss.str();
  EXPECT_LE(res, 1e-6) << ss.str();
}
TEST(MatrixTest, KronHomoMatirxATTBKronCalibration) {
  auto T_b_e = generateRandomTransformations(10, 1);
  auto T_b_c = generateRandomTransformations(1, 1);
  auto T_e_t = generateRandomTransformations(1, 1);
  correctTransformInplace(T_b_c[0]);
  correctTransformInplace(T_e_t[0]);

  std::vector<Eigen::Matrix4d> T_c_t;
  double res = 0;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_A(T_b_c[0], Eigen::ComputeFullU |
                                                        Eigen::ComputeFullV);
  for (int idx = 0; idx < T_b_e.size(); ++idx) {
    T_c_t.push_back(svd_A.solve(T_b_e[idx] * T_e_t[0]));
  }

  std::stringstream ss;
  auto T_calib = calibrationHandtoEye(T_c_t, T_b_e, correctRotateInplace,
                                      correctRotateInplace, &res);
  correctTransformInplace(T_calib[0]);
  correctTransformInplace(T_calib[1]);
  ss << "T_bc:\n" << T_b_c[0] << std::endl;
  ss << "T_bc1:\n" << T_calib[0] << std::endl;
  ss << "T_et:\n" << T_e_t[0] << std::endl;
  ss << "T_et1:\n" << T_calib[1] << std::endl;
  ss << "res: " << res << std::endl;
  EXPECT_TRUE(T_b_c[0].isApprox(T_calib[0], 1e-6) &&
              T_e_t[0].isApprox(T_calib[1], 1e-6))
      << ss.str();
  EXPECT_LE(res, 1e-6) << ss.str();
}