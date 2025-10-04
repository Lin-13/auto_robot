#include <Eigen/Core>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// 去除字符串首尾空白
inline std::string trim(const std::string &s) {
  auto start = s.begin();
  while (start != s.end() && std::isspace(*start)) {
    start++;
  }
  auto end = s.end();
  do {
    end--;
  } while (std::distance(start, end) > 0 && std::isspace(*end));
  return std::string(start, end + 1);
}

// 序列化：将矩阵转换为[1,2,3;4,5,6;7,8,9]格式
template <typename Derived>
std::ostream &operator<<(std::ostream &os,
                         const Eigen::MatrixBase<Derived> &mat) {
  os << "["; // 矩阵开始标记

  for (int i = 0; i < mat.rows(); ++i) {
    // 输出一行元素，用逗号分隔
    for (int j = 0; j < mat.cols(); ++j) {
      os << mat(i, j);
      if (j != mat.cols() - 1) {
        os << ",";
      }
    }
    // 行之间用分号分隔，最后一行不加分号
    if (i != mat.rows() - 1) {
      os << ";";
    }
  }

  os << "]"; // 矩阵结束标记
  return os;
}

// 反序列化：从[1,2,3;4,5,6;7,8,9]格式解析矩阵
template <typename Derived>
std::istream &operator>>(std::istream &is, Eigen::MatrixBase<Derived> &mat) {
  using Scalar = typename Derived::Scalar;
  std::string content;
  char c;

  // 读取矩阵开始标记'['
  if (!(is >> c) || c != '[') {
    is.setstate(std::ios::failbit);
    return is;
  }

  // 读取到']'为止的所有内容
  while (is >> c && c != ']') {
    content += c;
  }
  if (c != ']') { // 未找到结束标记
    is.setstate(std::ios::failbit);
    return is;
  }

  // 按分号分割行
  std::vector<std::string> row_strs;
  size_t pos = 0;
  while (pos < content.size()) {
    size_t next = content.find(';', pos);
    if (next == std::string::npos) {
      next = content.size();
    }
    row_strs.push_back(trim(content.substr(pos, next - pos)));
    pos = next + 1;
  }

  // 检查行数是否匹配（动态矩阵自动调整）
  int rows = row_strs.size();
  int cols = -1;
  std::vector<std::vector<Scalar>> data;

  // 解析每行的元素（按逗号分割）
  for (const auto &row_str : row_strs) {
    std::vector<Scalar> row_data;
    std::stringstream ss(row_str);
    std::string elem_str;

    while (std::getline(ss, elem_str, ',')) {
      elem_str = trim(elem_str);
      if (elem_str.empty())
        continue;

      std::stringstream elem_ss(elem_str);
      Scalar val;
      if (!(elem_ss >> val)) { // 解析单个元素失败
        is.setstate(std::ios::failbit);
        return is;
      }
      row_data.push_back(val);
    }

    // 检查所有行的列数是否一致
    if (cols == -1) {
      cols = row_data.size();
    } else if (row_data.size() != static_cast<size_t>(cols)) {
      is.setstate(std::ios::failbit);
      return is;
    }

    data.push_back(row_data);
  }

  // 处理动态矩阵的尺寸调整
  if constexpr (Derived::RowsAtCompileTime == Eigen::Dynamic) {
    const_cast<Eigen::MatrixBase<Derived> &>(mat).resize(rows, cols);
  } else if (rows != mat.rows()) { // 固定行数不匹配
    is.setstate(std::ios::failbit);
    return is;
  }

  if constexpr (Derived::ColsAtCompileTime == Eigen::Dynamic) {
    const_cast<Eigen::MatrixBase<Derived> &>(mat).resize(rows, cols);
  } else if (cols != mat.cols()) { // 固定列数不匹配
    is.setstate(std::ios::failbit);
    return is;
  }

  // 填充矩阵数据
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      const_cast<Eigen::MatrixBase<Derived> &>(mat)(i, j) = data[i][j];
    }
  }

  return is;
}
