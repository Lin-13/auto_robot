project(aubo_robot)
cmake_minimum_required(VERSION 3.15)

set(CXX_STANDARD 20)

# set(CMAKE_C_COMPILER "/usr/bin/clang")
# set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

# set(CMAKE_C_COMPILER "/usr/bin/gcc-12")
# set(CMAKE_CXX_COMPILER "/usr/bin/g++-12")
set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install")
add_compile_options(-std=c++20)

set(OpenCV4_DIR "/home/lwx/local/lib/cmake/opencv4")
find_package(OpenCV REQUIRED PATHS ${OpenCV4_DIR} NO_DEFAULT_PATH)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(LibXml2 REQUIRED)

find_package(gRPC CONFIG REQUIRED)
include_directories(
    include
    dependents/robotSDK/inc
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${LIBXML2_INCLUDE_DIR}
)
link_directories(
    include
    dependents/robotSDK/lib/linux_x64
    dependents/log4cplus/linux_x64/lib
)
add_library(ft_sensor
    STATIC
    src/ft_sensor/ft_sensor.cpp
)
target_link_libraries(ft_sensor
    PRIVATE
    ${LIBXML2_LIBRARIES}
)
add_library(
    utils
    STATIC
    src/utils/opencv_utils.cpp
    src/utils/matrix_utils.cpp
)
add_library(robot_interface
    STATIC
    src/robot_interface/robot.cpp
    src/robot_interface/robot_topology.cpp
    src/robot_interface/robot_controller.cpp
    src/robot_interface/timer.cpp
)
target_link_libraries(robot_interface
    PRIVATE
    orocos-kdl
    Eigen3::Eigen
    utils
)
add_library(aubo
    STATIC
    src/aubo/aubo_controller.cpp
    src/aubo/aubo_robot.cpp
)
target_link_libraries(aubo
    PRIVATE
    robot_interface
    fmt::fmt
    auborobotcontroller

    # orocos-kdl
    # Eigen3::Eigen
)
target_link_libraries(
    utils
    PRIVATE
    fmt::fmt
    ${OpenCV_LIBS}
    Eigen3::Eigen
    orocos-kdl
    realsense2
)

add_library(optitrack
    STATIC
    src/optitrack/optitrack_vrpn_cap.cpp
)
target_link_libraries(optitrack
    PRIVATE
    vrpn
    quat # vrpn的依赖
)

add_library(grpc_proto
    STATIC
    include/context_monitor/proto/monitor.pb.cc
    include/context_monitor/proto/monitor.grpc.pb.cc
)
target_link_libraries(grpc_proto
    PRIVATE
    gRPC::grpc++
    gRPC::grpc++_reflection
    protobuf::libprotobuf
)
add_library(context_monitor
    STATIC
    src/context_monitor/monitor_server.cpp
    src/context_monitor/monitor_client.cpp
)
target_link_libraries(context_monitor
    PRIVATE
    grpc_proto
)
add_executable(aruco_detect src/calib_handeye/opencv_aruco_detect.cpp)
target_link_libraries(
    aruco_detect
    PRIVATE
    fmt::fmt
    ${OpenCV_LIBS}
    realsense2
)

add_executable(calib
    src/calib_chess/chess_board.cpp
    src/calib_chess/CalibChessBoard.cpp
)
target_link_libraries(
    calib
    PRIVATE
    fmt::fmt
    ${OpenCV_LIBS}
)

add_executable(calib_handeye src/calib_handeye/calib_handeye.cpp)
target_link_libraries(
    calib_handeye
    PRIVATE
    aubo
    robot_interface
    fmt::fmt
    ${OpenCV_LIBS}
    orocos-kdl
    realsense2
    utils
)
add_executable(calib_handeye_optitrack src/calib_handeye/calib_handeye_optitrack.cpp)
target_link_libraries(
    calib_handeye_optitrack
    PRIVATE
    aubo
    fmt::fmt
    utils
    optitrack
)

# Test
add_executable(AuboTestsFunction test/test_aubo_function.cpp)
target_link_libraries(
    AuboTestsFunction
    auborobotcontroller
    log4cplus
    utils
    orocos-kdl
)
add_executable(RobotInterface test/test_robot_interface.cpp)
target_link_libraries(
    RobotInterface
    PRIVATE
    robot_interface
    ${GTEST_LIBRARIES}
    GTest::gtest_main
)
add_executable(AuboTests test/test_aubo.cpp)
target_link_libraries(
    AuboTests
    aubo

    # auborobotcontroller
    ${GTEST_LIBRARIES}
    GTest::gtest_main
)
add_executable(OptitrackTests test/test_vrpn.cpp)
target_link_libraries(
    OptitrackTests
    PRIVATE
    vrpn
    quat # vrpn的依赖
)
add_executable(OptitrackControl src/calib_handeye/optitrack_control.cpp)
target_link_libraries(
    OptitrackControl
    PRIVATE
    aubo
    fmt::fmt
    utils
    optitrack
    context_monitor
)
install(
    TARGETS
    aruco_detect calib calib_handeye calib_handeye_optitrack OptitrackControl
    DESTINATION
    ${CMAKE_INSTALL_PREFIX}
)

add_executable(ContextMonitorTests test/test_context_monitor.cpp)
target_link_libraries(
    ContextMonitorTests
    PRIVATE
    context_monitor
    ${GTEST_LIBRARIES}
    GTest::gtest_main
)

# Google Test
enable_testing()
find_package(GTest REQUIRED)
include(GoogleTest)
include(CTest)
include_directories(${GTEST_INCLUDE_DIRS})
add_executable(MatrixTests test/test_matrix.cpp)
target_link_libraries(
    MatrixTests
    ${GTEST_LIBRARIES}
    GTest::gtest_main
    utils
)

add_test(NAME MatrixTests COMMAND MatrixTests)

# add_test(NAME RobotInterface COMMAND RobotInterface)
# add_test(NAME AuboTests COMMAND AuboTests)
# gtest_discover_tests(my_tests)
