#pragma once
#include <Eigen/Core>
#include <iostream>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <concepts>
#include <sstream>
#include <type_traits>
#include <string>
#include <concepts>
#include <vector>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <array>
#include <cstddef>

 //[] formatter
template <typename T>
concept KDLArrayLike = std::disjunction_v<
    std::is_same<T, KDL::JntArray>,
    std::is_same<T, KDL::Twist>
>;
template <KDLArrayLike T>
T& operator<<(T& array, const std::vector<double>& vec) {
    for (int i = 0; i < vec.size(); i++) {
        array(i) = vec[i]; //array 不一定提供size，不检查数组越界，需要自己保证
    }
    return array;
}


template <typename T>
concept KDLFormat = std::disjunction_v<
    std::is_same<T, KDL::Frame> ,
    //std::is_same<T, KDL::FrameVel>,
	std::is_same<T, KDL::Vector>,
	std::is_same<T, KDL::Rotation>,
	std::is_same<T, KDL::Twist>,
	std::is_same<T, KDL::Wrench>,
    std::is_same<T, KDL::RigidBodyInertia>,
    std::is_same<T, Eigen::MatrixXd>,
    std::is_same<T, Eigen::VectorXd>,
    std::is_same<T, Eigen::Map<Eigen::Matrix3d>>,
    std::is_same<T, Eigen::Map<Eigen::MatrixXd>>
    >;
template <KDLFormat T>
struct fmt::formatter<T> : fmt::formatter<std::string> {
    //constexpr auto parse(format_parse_context& ctx) {
    //    return formatter<std::string>::parse(ctx);
    //}
    auto format(const T& frame, format_context& ctx) const {
        std::ostringstream oss;
        oss << frame;
        return formatter<std::string>::format(oss.str(), ctx);
    }
};
