#include "CalibChessBoard.h"
int main_fcn();
#include <thread>
int main(int argc, char **argv) {
  // long long checkpoint_loaded = 17039166378741100;
  long long checkpoint_loaded = 17039239807428598;
  CalibChessBoard demo;
  //-----Calibration------
  // demo.LoadImgsForCalib();
  demo.LoadImgsForCalib(checkpoint_loaded, 22);
  // demo.SaveCalibImages();
  // demo.CalibStart(9,6,7);
  // demo.SaveParamsCheckPoints();
  //----------------------
  demo.LoadParamsCheckPoints(checkpoint_loaded);
  // demo.match_img_loader_.Init(demo.left_calib_imgs_,
  // demo.right_calib_imgs_);//use calib imgs
  demo.match_img_loader_.Init(); // use device video
  std::thread match_thread(std::bind(&CalibChessBoard::Match, demo));
  match_thread.join();
  // main_fcn();
  return 0;
}
int main_fcn() try {
  // Initialize calibration data and load imgs for calibration
  std::cout << "OpenCV Version: " << cv::getVersionString() << std::endl;
  auto check_point_time = std::chrono::system_clock::now();
  std::chrono::system_clock::rep check_point_time_rep =
      check_point_time.time_since_epoch().count();
  // rep:long long for system_clock
  std::cout << "System time for checkpoints: " << check_point_time_rep
            << std::endl;
  std::string checkpoints_main_path =
      std::string("checkpoints/") + std::to_string(check_point_time_rep);
  int state = std::filesystem::create_directory(checkpoints_main_path);
  if (state == false) {
    throw "Create file path error";
  }

  std::vector<std::string> imgFileName1{
      "D:/calib/calib0.jpg", "D:/calib/calib1.jpg", "D:/calib/calib2.jpg",
      "D:/calib/calib3.jpg", "D:/calib/calib4.jpg", "D:/calib/calib5.jpg"};
  auto imgFileName2 = imgFileName1;
  int boardWidth = 9, boardHeight = 6;
  float squareSize = 7; // 7mm

  cv::Mat cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2;
  std::vector<std::vector<cv::Point3f>> objPointsList;
  std::vector<std::vector<cv::Point2f>> img1PointList, img2PointList;
  std::vector<cv::Mat> imgs1, imgs2, imgs1_filtered, imgs2_filtered;
  // readImageFile(imgFileName1, imgs1);
  // readImageFile(imgFileName2, imgs2);
  // imgsLoad(imgs1, "D:/calib/stereo/left",8);
  // imgsLoad(imgs2, "D:/calib/stereo/right",8);
  readVideoStereo(imgs1, imgs2);
  // checkpoint
  imgsSave(imgs1, checkpoints_main_path + "/stereo/left");
  imgsSave(imgs2, checkpoints_main_path + "/stereo/right");
  auto imageSize = imgs1[0].size();
  // Calibration start!
  std::vector<int> chess1, chess2;
  // Calibration method
  Calib(imgs1, boardWidth, boardHeight, squareSize, cameraMatrix1, distCoeffs1,
        objPointsList, img1PointList, chess1);
  objPointsList.clear();
  Calib(imgs2, boardWidth, boardHeight, squareSize, cameraMatrix2, distCoeffs2,
        objPointsList, img2PointList, chess2);
  PointListFilter(objPointsList, img1PointList, img2PointList, chess1, chess2);
  cv::Mat mapX1, mapY1, mapX2, mapY2, Q;
  stereoCalibrateAndRectify(
      objPointsList, img1PointList, img2PointList, cameraMatrix1, distCoeffs1,
      cameraMatrix2, distCoeffs2, imageSize, mapX1, mapY1, mapX2, mapY2, Q);
  // Output and save
  std::cout << "Q matrix:\n" << Q << std::endl;
  cv::imwrite(checkpoints_main_path + "/map/mapX1.jpg", mapX1);
  cv::imwrite(checkpoints_main_path + "/map/mapY1.jpg", mapY1);
  cv::imwrite(checkpoints_main_path + "/map/mapX2.jpg", mapX2);
  cv::imwrite(checkpoints_main_path + "/map/mapY2.jpg", mapY2);
  // cv::imwrite(checkpoints_main_path + "/Q.jpg", Q);
  cv::FileStorage file(checkpoints_main_path + "/param.xml",
                       cv::FileStorage::WRITE);
  file << "Q" << Q;
  file << "cameraMatrix1" << cameraMatrix1;
  file << "distCoeffs1" << distCoeffs1;
  file << "cameraMatrix2" << cameraMatrix2;
  file << "distCoeffs2" << distCoeffs2;
  file.release();
  std::vector<cv::Mat> imgs_match1, imgs_match2;
  // readVideoStereo(imgs_match1, imgs_match2);
  imgs_match1 = imgs1;
  imgs_match2 = imgs2;

  // Initialize for match
  cv::Mat src1, dst1, src2, dst2, src, dst;
  cv::Ptr<cv::StereoSGBM> matcher = cv::StereoSGBM::create();
  // cv::Ptr<cv::StereoBM> matcher = cv::StereoBM::create();
  // SGBMConfig(matcher);
  // Disparity scaling and bias
  float scale = (float)matcher->getNumDisparities();
  float bias = (float)matcher->getMinDisparity();
  // scale = 1;
  cv::Mat disparity, dis, disparity_norm, dis_color;
  cv::Mat pointcloud, pointcloudZ;
  std::vector<cv::Mat> pointxyz;
  cv::namedWindow("Before remap");
  cv::namedWindow("After remap");
  cv::namedWindow("3D Point");
  cv::setMouseCallback("3D Point", onMouse, &pointcloud);
  // Videocapture
  cv::VideoCapture cap1, cap2;
  cap1.open(0);
  cap2.open(1);
  while (1) {
    cap1.grab();
    cap2.grab();
    cap1.retrieve(src1);
    cap2.retrieve(src2);
    // src1 = imgs_match1[imgs_match1.size() - 1];
    // src2 = imgs_match2[imgs_match2.size() - 1];
    // Camera1 remap
    auto start = std::chrono::steady_clock::now();
    cv::remap(src1, dst1, mapX1, mapY1, cv::INTER_LINEAR);
    // Camera2 remap
    cv::remap(src2, dst2, mapX2, mapY2, cv::INTER_LINEAR);
    cv::hconcat(src1, src2, src);
    cv::hconcat(dst1, dst2, dst);
    cv::imshow("Before remap", src);
    cv::imshow("After remap", dst);
    auto end = std::chrono::steady_clock::now();
    // std::cout << "Time for remap: " << (end-start).count() / 1e9 << " s." <<
    // std::endl;

    // Match
    // std::cout << "Now match and compute" << std::endl;

    start = std::chrono::steady_clock::now();
    matcher->compute(dst1, dst2, disparity);
    end = std::chrono::steady_clock::now();
    // std::cout << "Time for match: " << (end - start).count() / 1e9 << " s."
    // << std::endl; dis.convertTo(disparity, CV_32F);

    // Disparity scaling
    disparity_norm = (disparity / 16.0f - bias) / scale;
    DisparityColoring(disparity_norm, dis_color);
    // Q = [1 0  0 -cx;
    //      0 1  0 -cy;
    //      0 0  0   f;
    //      0 0 -1/T 0 ]
    // X = [x;y;dis;1]
    // P = QX = [-T/dis*(x-cx);-T/dis*(y,cy);-T*f/dis;1]
    disparity = disparity / 16;
    cv::reprojectImageTo3D(disparity, pointcloud, Q, true);
    cv::split(pointcloud, pointxyz);
    pointcloudZ = pointxyz[2];
    cv::absdiff(pointcloudZ, cv::Mat::zeros(pointcloud.size(), CV_32FC1),
                pointcloudZ);
    // Output and show
    cv::imshow("3D Point", dis_color);
    int key = cv::waitKey(100);
    if (key == 27) {
      break;
    }
  }
  cv::imwrite(checkpoints_main_path + "/result/remap.jpg", dst);
  cv::imwrite(checkpoints_main_path + "/result/remap1.jpg", dst1);
  cv::imwrite(checkpoints_main_path + "/result/remap2.jpg", dst2);
  cv::imwrite(checkpoints_main_path + "/result/cloudpoint.jpg", pointcloud);
  cv::imwrite(checkpoints_main_path + "/result/dis_color.jpg", dis_color);
  cv::imwrite("pointcloud.jpg", pointcloudZ);
  std::cout << "Done!" << std::endl;

  cv::destroyAllWindows();
  return 0;
} catch (cv::Exception &e) {
  std::cout << "Exception in " << __FUNCTION__ << std::endl;
  std::cout << e.what() << std::endl;
  return -1;
} catch (std::exception &e) {
  std::cout << e.what() << std::endl;
  return -1;
}
