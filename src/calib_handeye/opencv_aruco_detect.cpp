
// OpenCV 在自己编译的4.13.0-dev中使用,其api与系统默认的 opencv不兼容
#include <fmt/format.h>
#include <librealsense2/rs.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>
void onMouse(int event, int x, int y, int flags, void *img);
void depth2pointcloud(const cv::Mat &depth, cv::Mat &pointcloud,
                      const cv::Mat &cameraMatrix) {
  if (depth.empty() || depth.type() != CV_16UC1) {
    throw "Input depth is empty or not CV_16UC1";
  }
  if (cameraMatrix.empty() || cameraMatrix.type() != CV_64FC1 ||
      cameraMatrix.rows != 3 || cameraMatrix.cols != 3) {
    throw "Input cameraMatrix is empty or not CV_64FC1 3x3";
  }
  if (pointcloud.empty() || pointcloud.type() != CV_32FC3 ||
      pointcloud.size() != depth.size()) {
    pointcloud = cv::Mat::zeros(depth.size(), CV_32FC3);
  }
  float fx = cameraMatrix.at<double>(0, 0);
  float fy = cameraMatrix.at<double>(1, 1);
  float cx = cameraMatrix.at<double>(0, 2);
  float cy = cameraMatrix.at<double>(1, 2);
  for (int y = 0; y < depth.rows; y++) {
    for (int x = 0; x < depth.cols; x++) {
      uint16_t d = depth.at<uint16_t>(y, x);
      if (d == 0) {
        pointcloud.at<cv::Vec3f>(y, x) = cv::Vec3f(0, 0, 0);
        continue;
      }
      float z = d / 1000.0f; // mm to m
      float x3d = (x - cx) * z / fx;
      float y3d = (y - cy) * z / fy;
      pointcloud.at<cv::Vec3f>(y, x) = cv::Vec3f(x3d, y3d, z);
    }
  }
}
int read_from_rs() {
  rs2::pipeline pipe;
  fmt::print("Starting RealSense camera...\n");
  rs2::pipeline_profile profile = pipe.start();
  rs2::frameset frames = pipe.wait_for_frames();
  rs2::align align_to(RS2_STREAM_COLOR);
  frames = align_to.process(frames);
  rs2::depth_frame depth_frame = frames.get_depth_frame();
  rs2::video_frame color_frame = frames.get_color_frame();
  cv::Mat color(cv::Size(640, 480), CV_8UC3, (void *)color_frame.get_data(),
                cv::Mat::AUTO_STEP);
  cv::Mat depth(cv::Size(640, 480), CV_16UC1, (void *)depth_frame.get_data(),
                cv::Mat::AUTO_STEP);
  cv::Mat point_cloud(cv::Size(640, 480), CV_32FC3);
  rs2::pointcloud pc;
  pc.map_to(color_frame);
  auto points = pc.calculate(depth_frame);
  auto sp = points.get_profile().as<rs2::video_stream_profile>();
  auto R =
      frames.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

  fmt::print("Intrinsics: fx:{} ,fy:{}, cx:{}, cy:{}\n", R.fy, R.fy, R.ppx,
             R.ppy);
  cv::Mat cameraMatrix =
      (cv::Mat_<double>(3, 3) << R.fx, 0, R.ppx, 0, R.fy, R.ppy, 0, 0, 1);
  depth2pointcloud(depth, point_cloud, cameraMatrix);

  fmt::print("Test OK\n");
  cv::namedWindow("Color", cv::WINDOW_NORMAL);
  cv::namedWindow("Depth", cv::WINDOW_NORMAL);
  cv::setMouseCallback("Depth", onMouse, &point_cloud);
  fmt::print("Press ESC to exit\n");

  // keyward
  int enable_draw = 1;
  while (1) {
    frames = pipe.wait_for_frames();
    frames = align_to.process(frames);
    depth_frame = frames.get_depth_frame();
    color_frame = frames.get_color_frame();
    color = cv::Mat(cv::Size(640, 480), CV_8UC3, (void *)color_frame.get_data(),
                    cv::Mat::AUTO_STEP);
    depth = cv::Mat(cv::Size(640, 480), CV_16UC1,
                    (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(color, color, cv::COLOR_BGR2RGB);

    depth2pointcloud(depth, point_cloud, cameraMatrix);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams =
        std::make_shared<cv::aruco::DetectorParameters>();
    auto dictionary = std::make_shared<cv::aruco::Dictionary>(
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250));

    cv::aruco::detectMarkers(color, dictionary, markerCorners, markerIds,
                             detectorParams, rejectedCandidates);
    cv::Mat depth_normalized;
    double minVal = 0, maxVal = 2000; // 2m
    // cv::minMaxIdx(depth, &minVal, &maxVal);
    depth.convertTo(depth_normalized, CV_8UC1, 255.0 / (maxVal - minVal), 0);
    // 应用彩虹色图
    cv::Mat depth_colormap;
    cv::applyColorMap(depth_normalized, depth_colormap, cv::COLORMAP_RAINBOW);
    if (enable_draw) {
      cv::aruco::drawDetectedMarkers(color, markerCorners, markerIds);
    }

    cv::imshow("Color", color);
    cv::imshow("Depth", depth_colormap);
    int key = cv::waitKey(30);
    if (key != -1)
      fmt::print("Key pressed: {}\n", key);
    if (key == 27)
      break;
    if (key == 'a' || key == 'A') {
      fmt::print("markerIds size: {}\n", markerIds.size());
      for (size_t i = 0; i < markerIds.size(); i++) {
        fmt::print("\tmarker id: {}\n", markerIds[i]);
        for (size_t j = 0; j < markerCorners[i].size(); j++) {
          auto point = markerCorners[i][j];
          auto depth_value = depth.at<uint16_t>(static_cast<int>(point.y),
                                                static_cast<int>(point.x));
          auto depth_point = point_cloud.at<cv::Vec3f>(
              static_cast<int>(point.y), static_cast<int>(point.x));
          fmt::print("\t\tcorner {}: ({}, {}, {})\n", j, point.x, point.y,
                     depth_value);
          fmt::print("\t\txyz at corner {}: ({},{},{})\n", j, depth_point[0],
                     depth_point[1], depth_point[2]);
        }
      }
    }
    if (key == 'd' || key == 'D') {
      enable_draw = !enable_draw;
      fmt::print("enable_draw: {}\n", enable_draw);
    }
    if (key == 's' || key == 'S') {
      static int img_id = 0;
      std::string color_filename = fmt::format("imsg/color_{:03d}.png", img_id);
      std::string depth_filename = fmt::format("imgs/depth_{:03d}.png", img_id);
      cv::imwrite(color_filename, color);
      cv::imwrite(depth_filename, depth);
      fmt::print("Saved {} and {}\n", color_filename, depth_filename);
      img_id++;
    }
  }
  return 0;
}
void onMouse(int event, int x, int y, int flags, void *img) {
  static float threshold = 10000;
  int z_max = false;
  cv::Mat mouse_show;
  static bool left_mouse;
  static cv::Vec3f last_point;
  ((cv::Mat *)img)->copyTo(mouse_show);
  //(*((cv::Mat*)img), mouse_show);
  cv::Size size = mouse_show.size();
  while (x >= size.width) {
    x = x - size.width;
  }
  while (y >= size.height) {
    y = y - size.height;
  }

  if (event == cv::EVENT_LBUTTONDOWN) {
    std::cout << "Img x: " << x << " y: " << y << std::endl;
    cv::Vec3f point = mouse_show.at<cv::Vec3f>(y, x);
    cv::Vec3f delta = point - last_point;
    if (delta[2] >= threshold && z_max) {
      delta[2] = 0;
    }
    double d = cv::norm(delta);

    std::cout << "x: " << point[0] << " y: " << point[1] << " z: " << point[2]
              << " distance:" << d << std::endl;
    last_point = point;
    left_mouse = true;
  } else if (event == cv::EVENT_LBUTTONUP) {
    left_mouse = false;
  } else if ((event == cv::EVENT_MOUSEMOVE) && (left_mouse == true)) {
  } else if (event == cv::EVENT_RBUTTONDOWN) {
    int datatype = mouse_show.type();
    if (datatype == CV_8UC3) {
      std::cout << "Type:8FC3" << std::endl;
    }
    if (datatype == CV_32FC3) {
      std::cout << "Type:32FC3" << std::endl;
    }
    if (datatype == CV_16FC3) {
      std::cout << "Type:16FC3" << std::endl;
    }
    if (datatype == CV_32F) {
      std::cout << "Type:32F" << std::endl;
    }
    // std::cout << "Type:" << datatype << std::endl;
    std::cout << "Channel:" << mouse_show.channels() << std::endl;
  }
}
int main() {
  cv::Mat markerImage;
  fmt::print("OpenCV version: {}\n", CV_VERSION);
  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

  // cv::aruco::drawMarker(dictionary, 23, 200, markerImage, 1);// opencv 4.5
  cv::aruco::generateImageMarker(dictionary, 23, 200, markerImage, 1);
  // cv::aruco::detectMarkers()
  cv::imwrite("marker23.png", markerImage);

  read_from_rs();
  return 0;
}