#include "calib_chess/CalibChessBoard.h"
void CameraParams::Load(std::string map_xml, std::string matrix_xml) {

  cv::FileStorage map(map_xml, cv::FileStorage::READ);
  if (!map.isOpened()) {
    throw std::string("Map can not load from xml file.\n") +
        "Better code use xml to save CV_32FC1 Matrix,instead of jpg,in the old "
        "version."
        "If file: " +
        map_xml + " does not exist,you can load stereo imgs in this point" +
        " and generate this file by CameraParams::Write after calib.";
  }
  map["mapX1"] >> mapX1;
  map["mapY1"] >> mapY1;
  map["mapX2"] >> mapX2;
  map["mapY2"] >> mapY2;
  cv::FileStorage fs(matrix_xml, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    throw "Params xml file can not load";
  }
  fs["Q"] >> Q;
  fs["cameraMatrix1"] >> cameraMatrix1;
  fs["distCoeffs1"] >> distCoeffs1;
  fs["cameraMatrix2"] >> cameraMatrix2;
  fs["distCoeffs2"] >> distCoeffs2;
  fs.release();
}
void CameraParams::Write(std::string map_xml, std::string matrix_xml) {
  cv::FileStorage map(map_xml, cv::FileStorage::WRITE);
  if (!map.isOpened()) {
    throw "Map can not save to file.";
  }
  map << "mapX1" << mapX1;
  map << "mapY1" << mapY1;
  map << "mapX2" << mapX2;
  map << "mapY2" << mapY2;

  cv::FileStorage fs(matrix_xml, cv::FileStorage::WRITE);
  if (!fs.isOpened()) {
    throw "Params can not save to file.";
  }
  fs << "Q" << Q;
  fs << "cameraMatrix1" << cameraMatrix1;
  fs << "distCoeffs1" << distCoeffs1;
  fs << "cameraMatrix2" << cameraMatrix2;
  fs << "distCoeffs2" << distCoeffs2;
  fs.release();
}
std::ostream &operator<<(std::ostream &os, CameraParams params) {
  os << "CameraParams:" << std::endl;
  os << "cameraMatrix1:\n" << params.cameraMatrix1 << std::endl;
  os << "distCoeffs1:\n" << params.distCoeffs1 << std::endl;
  os << "cameraMatrix2:\n" << params.cameraMatrix2 << std::endl;
  os << "distCoeffs2:\n" << params.distCoeffs2 << std::endl;
  os << "Q:\n" << params.Q << std::endl;
  return os;
}

StereoImageLoader::StereoImageLoader() {
  type_ = NOT_INIT;
  count_ = 0;
  container_idx_ = 0;
  container_max_ = -1;
}
int StereoImageLoader::Init() {
  cap1_.open(0);
  cap2_.open(1);
  if (!cap1_.isOpened() || !cap2_.isOpened()) {
    throw "StereoImageLoader::StereoImageLoader::Can not open device";
  }
  type_ = DEVICE;
  container_idx_ = -1;
  container_max_ = -1;
  return 1;
}
int StereoImageLoader::Init(std::vector<cv::Mat> &can1,
                            std::vector<cv::Mat> &can2) {
  if (can1.size() != can2.size()) {
    throw "StereoImageLoader::StereoImageLoader::Containers must have the same "
          "size";
  }
  type_ = MAT_CONTAINER;
  container_idx_ = 0;
  container_max_ = -1;
  container1_ = can1;
  container2_ = can2;
  container_max_ = container1_.size();
  return 1;
}
int StereoImageLoader::nextFrame(cv::Mat &img1, cv::Mat &img2) {
  count_++;
  if (type_ == DEVICE) {
    cap1_.grab();
    cap2_.grab();
    cap1_.retrieve(img1);
    cap2_.retrieve(img2);
  } else if (type_ == MAT_CONTAINER) {
    img1 = container1_[container_idx_].clone();
    img2 = container2_[container_idx_].clone();
    container_idx_++;
    // Loop
    if (container_idx_ >= container_max_) {
      container_idx_ = 0;
    }
  } else if (type_ == NOT_INIT) {
    // Do nothing
    return 0;
  }
  return 1;
}
int StereoImageLoader::release() {
  type_ = NOT_INIT;
  count_ = 0;
  container_idx_ = 0;
  container_max_ = -1;
  cap1_.release();
  cap2_.release();
  container1_.clear();
  container2_.clear();
  return 1;
}

CalibChessBoard::CalibChessBoard() {
  auto checkpoints = std::chrono::system_clock::now();
  checkpoints_time_rep_ = checkpoints.time_since_epoch().count();
  checkpoints_main_path_ =
      std::string("checkpoints/") + std::to_string(checkpoints_time_rep_);
  int state = std::filesystem::create_directory(checkpoints_main_path_);
  if (state == false) {
    throw "Create file path error";
  }
}

void CalibChessBoard::LoadImgsForCalib() {
  readVideoStereo(left_calib_imgs_, right_calib_imgs_);
}
/*|-main_path
    |-stereo
        |-left1.jpg
        |-left2.jpg
        |- ...
        |-right1.jpg
        |-right2.jpg
*/
void CalibChessBoard::LoadImgsForCalib(std::string main_path, int num) {
  imgsLoad(left_calib_imgs_, main_path + "/stereo/left", num);
  imgsLoad(right_calib_imgs_, main_path + "/stereo/right", num);
}
void CalibChessBoard::LoadImgsForCalib(long long checkpoints, int num) {
  std::string main_path = checkpoints == 0 ? checkpoints_main_path_
                                           : std::string("checkpoints/") +
                                                 std::to_string(checkpoints);
  imgsLoad(left_calib_imgs_, main_path + "/stereo/left", num);
  imgsLoad(right_calib_imgs_, main_path + "/stereo/right", num);
}
void CalibChessBoard::SaveCalibImages(std::string main_path) {
  std::filesystem::create_directory(main_path + "/stereo");
  imgsSave(left_calib_imgs_, main_path + "/stereo/left");
  imgsSave(right_calib_imgs_, main_path + "/stereo/right");
}
void CalibChessBoard::SaveCalibImages() {
  SaveCalibImages(checkpoints_main_path_);
}
void CalibChessBoard::LoadParamsCheckPoints(
    std::string check_points_main_path) {
  params_.Load(check_points_main_path + "/map.xml",
               check_points_main_path + "/param.xml");
}
void CalibChessBoard::LoadParamsCheckPoints(long long checkpoints) {
  LoadParamsCheckPoints(checkpoints == 0 ? checkpoints_main_path_
                                         : std::string("checkpoints/") +
                                               std::to_string(checkpoints));
}
void CalibChessBoard::SaveParamsCheckPoints(
    std::string check_points_main_path) {
  // boost::filesystem::create_directory(check_points_main_path + "/map");
  params_.Write(check_points_main_path + "/map.xml",
                check_points_main_path + "/param.xml");
}
void CalibChessBoard::SaveParamsCheckPoints(void) {
  SaveParamsCheckPoints(checkpoints_main_path_);
}
void CalibChessBoard::CalibStart(int boardWidth, int boardHeight,
                                 float squareSize) {
  // Calibration start!
  if (left_calib_imgs_.size() == 0 && right_calib_imgs_.size() == 0) {
    throw "Imgs for Calib not loaded successfullt!";
  }
  auto imageSize = right_calib_imgs_[0].size();
  std::vector<int> chess1, chess2;
  // Calibration method
  std::vector<std::vector<cv::Point3f>> objPointsList;
  std::vector<std::vector<cv::Point2f>> img1PointList, img2PointList;
  // Calib left camera
  Calib(left_calib_imgs_, boardWidth, boardHeight, squareSize,
        params_.cameraMatrix1, params_.distCoeffs1, objPointsList,
        img1PointList, chess1);
  objPointsList.clear();
  // Calib right camera
  Calib(right_calib_imgs_, boardWidth, boardHeight, squareSize,
        params_.cameraMatrix2, params_.distCoeffs2, objPointsList,
        img2PointList, chess2);
  PointListFilter(objPointsList, img1PointList, img2PointList, chess1, chess2);

  stereoCalibrateAndRectify(objPointsList, img1PointList, img2PointList,
                            params_.cameraMatrix1, params_.distCoeffs1,
                            params_.cameraMatrix2, params_.distCoeffs2,
                            imageSize, params_.mapX1, params_.mapY1,
                            params_.mapX2, params_.mapY2, params_.Q);
}
void CalibChessBoard::Match() {
  // StateMachine
  std::vector<int> StateMap = {0, 50};
  std::vector<int> NextState = {1, 0};
  int state = 0;
  static cv::Ptr<cv::StereoSGBM> matcher = cv::StereoSGBM::create();
  SGBMConfig(matcher);
  // cv::Ptr<cv::StereoBM> matcher = cv::StereoBM::create();
  // BMConfig(matcher);//��40mm����ʱ����
  // Disparity scaling and bias
  std::cout << "Matcher:" << matcher << std::endl;
  float scale = (float)matcher->getNumDisparities();
  float bias = (float)matcher->getMinDisparity();
  // scale = 1;
  // Image field
  cv::Mat src1, dst1, dst1_gray, src2, dst2, dst2_gray, src, dst;
  cv::Mat disparity, dis, disparity_norm, dis_color;
  cv::Mat pointcloud, pointcloudZ;
  std::vector<cv::Mat> pointxyz;
  cv::namedWindow("Before remap");
  cv::namedWindow("After remap");
  cv::namedWindow("3D Point");
  cv::setMouseCallback("3D Point", onMouse, &pointcloud);
  cv::setMouseCallback("After remap", onMouse, &pointcloud);
  // cv::VideoCapture cap1, cap2;
  // cap1.open(0);
  // cap2.open(1);

  while (1) {
    // cap1.grab();
    // cap2.grab();
    // cap1.retrieve(src1);
    // cap2.retrieve(src2);
    match_img_loader_.nextFrame(src1, src2);
    // Camera1 remap
    auto start = std::chrono::steady_clock::now();
    cv::remap(src1, dst1, params_.mapX1, params_.mapY1, cv::INTER_LINEAR);
    // Camera2 remap
    cv::remap(src2, dst2, params_.mapX2, params_.mapY2, cv::INTER_LINEAR);
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
    cv::cvtColor(dst1, dst1_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(dst2, dst2_gray, cv::COLOR_BGR2GRAY);
    matcher->compute(dst1_gray, dst2_gray, disparity);
    end = std::chrono::steady_clock::now();
    // std::cout << "Time for match: " << (end - start).count() / 1e9 << " s."
    // << std::endl; dis.convertTo(disparity, CV_32F);

    // Disparity scaling (disparity.type() == CV_16SC1)==true
    // std::cout << "\nDisparity type:" << (disparity.type() == CV_16SC1) <<
    // std::endl; bias=0
    disparity.convertTo(disparity_norm, CV_8UC1, 255.0 / (scale * 16.0),
                        bias * 255.0 / (scale * 16.0));
    // disparity_norm = (disparity / 16.0f - bias) / scale;
    DisparityColoring(disparity_norm, dis_color);
    // dis_color = disparity_norm;
    //  Q = [1 0  0 -cx;
    //       0 1  0 -cy;
    //       0 0  0   f;
    //       0 0 -1/T 0 ]
    //  X = [x;y;dis;1]
    //  P = QX = [-T/dis*(x-cx);-T/dis*(y,cy);-T*f/dis;1]
    // disparity = disparity / 16;
    cv::reprojectImageTo3D(disparity, pointcloud, params_.Q, true);
    pointcloud *= 16;
    // std::cout << "\nPointcloud type:"<< (pointcloud.type() == CV_32FC3) <<
    // std::endl;
    cv::split(pointcloud, pointxyz);
    pointcloudZ = pointxyz[2];
    // PointcloudProcess(pointcloud, 10000);
    cv::absdiff(pointcloudZ, cv::Mat::zeros(pointcloud.size(), CV_32FC1),
                pointcloudZ);
    // Output and show
    cv::imshow("3D Point", dis_color);
    int key = cv::waitKey(StateMap[state]);
    if (key == 27) { // Esc
      break;
    }
    if (key == 13) { // Enter
      state = NextState[state];
      std::cout << "State:" << state << std::endl;
    }
  }
  match_img_loader_.release();
}

// Utils OpenCV Calib method
int readVideoStereo(std::vector<cv::Mat> &imgs1, std::vector<cv::Mat> &imgs2) {
  cv::VideoCapture cap1, cap2;
  try {
    cap1.open(0);
    cap2.open(1);
  } catch (cv::Exception &e) {
    std::cout << e.what() << std::endl;
  }
  cv::Mat frame1, frame2, frames; // 640X480
  cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);
  cv::resizeWindow("Video", 640 * 2, 480);
  int key = 0;
  int count = 0;
  for (;;) {
    cap1.grab();
    cap2.grab();
    cap1.retrieve(frame1);
    cap2.retrieve(frame2);
    cv::hconcat(std::vector<cv::Mat>{frame1, frame2}, frames);
    cv::imshow("Video", frames);
    key = cv::waitKey(100);
    if (key == 27) {
      break;
    } else if (key == 13) { // Enter
      imgs1.push_back(frame1.clone());
      imgs2.push_back(frame2.clone());
      std::cout << "Capture a stereo frame." << std::endl;
      count++;
    }
  }
  cv::destroyWindow("Video");
  cv::namedWindow("Left Camera");
  std::cout << "Left Camera" << std::endl;
  for (auto &img : imgs1) {
    cv::imshow("Left Camera", img);
    std::cout << "Img size: " << img.size() << std::endl;
    cv::waitKey(250);
  }
  cv::destroyWindow("Left Camera");

  cv::namedWindow("Right Camera");
  std::cout << "Right Camera" << std::endl;
  for (auto &img : imgs2) {
    cv::imshow("Right Camera", img);
    std::cout << "Img size: " << img.size() << std::endl;
    cv::waitKey(250);
  }
  cv::destroyWindow("Right Camera");
  return 1;
}

int readImageFile(const std::vector<std::string> imgFile,
                  std::vector<cv::Mat> &imgs) {
  bool success = true;
  for (auto &file : imgFile) {
    try {
      auto img = cv::imread(file);
      imgs.push_back(img.clone());
    } catch (cv::Exception &e) {
      std::cout << e.what() << std::endl;
      success = false;
    }
  }
  return success;
}
int imgsSave(std::vector<cv::Mat> &imgs, std::string header) {
  int i = 0;
  for (auto &img : imgs) {
    cv::imwrite(header + std::to_string(i) + ".jpg", img);
    i++;
  }
  return i;
}
int imgsLoad(std::vector<cv::Mat> &imgs, std::string header, int num) {
  for (int i = 0; i < num; i++) {
    imgs.push_back(cv::imread(header + std::to_string(i) + ".jpg"));
  }
  return 1;
}
// Calib Mono
int Calib(std::vector<cv::Mat> imgs, int boardWidth, int boardHeight,
          float squareSize, cv::Mat &cameraMatrix, cv::Mat &distCoeffs,
          std::vector<std::vector<cv::Point3f>> &objPointsList,
          std::vector<std::vector<cv::Point2f>> &imgPointList,
          std::vector<int> &find_chessboard, bool output) try {
  cv::Size patternSize(boardWidth, boardHeight);

  std::vector<cv::Point3f> objPoints;
  for (int i = 0; i < boardHeight; ++i) {
    for (int j = 0; j < boardWidth; ++j) {
      objPoints.push_back(cv::Point3f(j * squareSize, i * squareSize, 0.0f));
    }
  }

  cv::Mat gray;
  if (output == true) {
    cv::namedWindow("chess board", cv::WindowFlags::WINDOW_NORMAL);
    cv::resizeWindow("chess board", 800, 600);
  }
  auto calib_obj = objPointsList;
  auto calib_img = imgPointList;
  for (auto &image : imgs) {
    cv::Mat img = image.clone();
    std::vector<cv::Point2f> corners;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    bool find = cv::findChessboardCorners(img, patternSize, corners);
    if (find) {
      cv::drawChessboardCorners(img, patternSize, corners, true);
      objPointsList.push_back(objPoints);
      imgPointList.push_back(corners);
      calib_obj.push_back(objPoints);
      calib_img.push_back(corners);
      find_chessboard.push_back(1);
      if (output == true) {
        std::cout << "Find chess board " << std::endl;
      }
    } else {
      if (output == true) {
        std::cout << "Can not find chess board " << std::endl;
      }
      // place hold
      objPointsList.push_back(objPoints);
      imgPointList.push_back(corners);
      find_chessboard.push_back(0);
    }
    if (output == true) {
      cv::imshow("chess board", img);
      cv::waitKey(250);
    }
  }
  if (output == true) {
    std::cout << "Calibrating Camera..." << std::endl;
  }
  // cv::Mat cameraMatrix, distCoeffs;
  std::vector<cv::Mat> rvecs, tvecs;
  cv::calibrateCamera(calib_obj, calib_img, gray.size(), cameraMatrix,
                      distCoeffs, rvecs, tvecs);
  std::cout << "Camera Matrix:\n" << cameraMatrix << "\n\n";
  std::cout << "Distortion Coefficients:\n" << distCoeffs << "\n\n";
  for (int i = 0; i < tvecs.size(); i++) {
    std::cout << "T vector " << i << ": " << tvecs[i] << std::endl;
    std::cout << "R vector " << i << ": " << rvecs[i] << std::endl;
  }
  cv::destroyWindow("chess board");
  return true;
} catch (cv::Exception &e) {
  std::cout << "Exception in " << __FUNCTION__ << std::endl;
  cv::destroyWindow("chess board");
  std::cout << e.what() << std::endl;
  return false;
}

int imgsFilter(std::vector<cv::Mat> imgs1, std::vector<int> find1,
               std::vector<cv::Mat> imgs2, std::vector<int> find2,
               std::vector<cv::Mat> &new_imgs1,
               std::vector<cv::Mat> &new_imgs2) {
  new_imgs1.clear();
  new_imgs2.clear();
  if (imgs1.size() != find1.size() || imgs2.size() != find2.size()) {
    throw "Imgs Size error";
  }
  if (&imgs1 == &new_imgs1 || &imgs2 == &new_imgs2) {
    throw "New image container must be new";
  }
  int count = 0;
  for (int i = 0; i < imgs1.size(); i++) {
    if (find1[i] == 1 && find2[i] == 1) {
      new_imgs1.push_back(imgs1[i]);
      new_imgs2.push_back(imgs2[i]);
      count++;
    }
  }
  return count;
}
int PointListFilter(std::vector<std::vector<cv::Point3f>> &objPointsList,
                    std::vector<std::vector<cv::Point2f>> &img1PointList,
                    std::vector<std::vector<cv::Point2f>> &img2PointList,
                    std::vector<int> flags1, std::vector<int> flags2) {
  if (img1PointList.size() != img1PointList.size() ||
      img1PointList.size() != img2PointList.size() ||
      img1PointList.size() != flags1.size() || flags1.size() != flags2.size()) {
    throw "Length error in PointListFilter";
  }
  std::vector<std::vector<cv::Point3f>> newObj;
  std::vector<std::vector<cv::Point2f>> newImg1, newImg2;
  int count = 0;
  for (int i = 0; i < flags1.size(); i++) {
    if (flags1[i] == 1 && flags2[i] == 1) {
      newObj.push_back(objPointsList[i]);
      newImg1.push_back(img1PointList[i]);
      newImg2.push_back(img2PointList[i]);
      std::cout << "Point" << i << " Pass" << std::endl;
      count++;
    } else {
      std::cout << "Point" << i << " Not Pass" << std::endl;
    }
  }
  std::cout << "Point list size: " << newObj.size() << std::endl;
  objPointsList = newObj;
  img1PointList = newImg1;
  img2PointList = newImg2;
  return count;
}
int stereoCalibrateAndRectify(
    std::vector<std::vector<cv::Point3f>> objPointsList,
    std::vector<std::vector<cv::Point2f>> img1PointList,
    std::vector<std::vector<cv::Point2f>> img2PointList, cv::Mat cameraMatrix1,
    cv::Mat distCoeffs1, cv::Mat cameraMatrix2, cv::Mat distCoeffs2,
    cv::Size imageSize, cv::Mat &mapX1, cv::Mat &mapY1, cv::Mat &mapX2,
    cv::Mat &mapY2, cv::Mat &Q) try {
  cv::Mat R, T, E, F;
  std::cout << "Camera Matrix1:\n" << cameraMatrix1 << "\n\n";
  std::cout << "Distortion Coefficients1:\n" << distCoeffs1 << "\n\n";
  std::cout << "Camera Matrix2:\n" << cameraMatrix2 << "\n\n";
  std::cout << "Distortion Coefficients2:\n" << distCoeffs2 << "\n\n";
  double reprojectionError = cv::stereoCalibrate(
      objPointsList, img1PointList, img2PointList, cameraMatrix1, distCoeffs1,
      cameraMatrix2, distCoeffs2, imageSize, R, T, E, F);
  std::cout << "Reprojection error: " << reprojectionError << std::endl;
  // For Debug
  std::cout << "Stereo calib:R:\n" << R << std::endl;
  std::cout << "Stereo calib:T:\n" << T << std::endl;
  cv::Mat R1, R2, P1, P2;
  cv::Size newSize = imageSize;
  cv::Rect roi1, roi2;
  cv::stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2,
                    imageSize, R, T, R1, R2, P1, P2, Q, 1024, 0, newSize, &roi1,
                    &roi2);
  // flags=-1:default;flags=0:fill vaild image
  std::cout << "StereoRectify: newsize " << newSize << " roi1:" << roi1
            << " roi2:" << roi2 << std::endl;
  // cv::Mat mapX1, mapY1, mapX2, mapY2;
  cv::Size newImageSize1, newImageSize2;
  cv::Rect ValidROI1, validROI2;
  cv::Mat newCameraMatrix1 =
      cv::getOptimalNewCameraMatrix(cameraMatrix1, distCoeffs1, imageSize, 1);
  cv::Mat newCameraMatrix2 =
      cv::getOptimalNewCameraMatrix(cameraMatrix2, distCoeffs2, imageSize, 1);
  cv::initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize,
                              CV_32FC1, mapX1, mapY1);
  cv::initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize,
                              CV_32FC1, mapX2, mapY2);
  std::cout << "R1:\n" << R1 << "\nR2:\n" << R2 << std::endl;
  std::cout << "P1:\n" << P1 << "\nP2:\n" << P2 << std::endl;
  std::cout << "Q:\n" << Q << std::endl;
  return 1;
} catch (cv::Exception &e) {
  std::cout << "Exception in " << __FUNCTION__ << std::endl;
  std::cout << e.what() << std::endl;
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
void BMConfig(cv::Ptr<cv::StereoBM> matcher) {
  matcher->setPreFilterType(cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE);
  matcher->setPreFilterCap(31);
  matcher->setBlockSize(21);
  matcher->setMinDisparity(0);
  matcher->setNumDisparities(96);

  matcher->setTextureThreshold(10);
  matcher->setUniquenessRatio(25);
  matcher->setSpeckleWindowSize(100);
  matcher->setSpeckleRange(32);
  matcher->setDisp12MaxDiff(-1);
}
void SGBMConfig(cv::Ptr<cv::StereoSGBM> matcher) {
  int block_size = 9, NumDisparities = 16 * 3;
  int img_channels = 1;
  matcher->setMinDisparity(0);
  matcher->setNumDisparities(NumDisparities);
  matcher->setBlockSize(block_size);
  matcher->setP1(8 * img_channels * block_size * block_size);
  matcher->setP1(32 * img_channels * block_size * block_size);
  matcher->setDisp12MaxDiff(1);
  matcher->setPreFilterCap(1);
  matcher->setUniquenessRatio(25);
  matcher->setSpeckleWindowSize(100);
  matcher->setSpeckleRange(1);
  matcher->setMode(cv::StereoSGBM::MODE_SGBM);
}
void DisparityColoring(cv::Mat &dis_norm, cv::Mat &colored) {
  cv::Mat dis8u;
  double minValue, maxValue;
  cv::Point minLoc, maxLoc;
  cv::minMaxLoc(dis_norm, &minValue, &maxValue, &minLoc, &maxLoc);
  std::cout << "Disparity Max: " << maxValue << " at " << maxLoc << ";"
            << "Disparity Min: " << minValue << " at " << minLoc << ".\r";
  // dis_norm.convertTo(dis8u, CV_8UC1, 255.0);
  dis8u = dis_norm;
  if (colored.empty() || colored.type() != CV_8UC3 ||
      colored.size() != dis_norm.size()) {
    colored = cv::Mat::zeros(dis_norm.size(), CV_8UC3);
  }
  for (int y = 0; y < dis8u.rows; y++) {
    for (int x = 0; x < dis8u.cols; x++) {
      uchar val = dis8u.at<uchar>(y, x);
      uchar r, g, b;
      if (val == 0)
        r = g = b = 0;
      else {
        r = 255 - val;
        g = val < 128 ? val * 2 : (uchar)((255 - val) * 2);
        b = val;
      }
      colored.at<cv::Vec3b>(y, x) = cv::Vec3b(r, g, b);
    }
  }
}
void PointcloudProcess(cv::Mat &pointcloud, float threshold) {
  cv::Mat input = pointcloud;
  // assert (input.type() == CV_32FC3);
  cv::Mat inpaint_(input.size(), CV_32FC1);
  std::vector<cv::Mat> pointcloudxyz, inpaintxyz;
  cv::split(input, pointcloudxyz);
  // process
  for (auto &point : pointcloudxyz) {
    // std::cerr << "\nPoint size: " << point.size() << " " <<
    //     "Point type: " << (point.type() == CV_32FC1) << std::endl;
    cv::Mat mask = (point >= threshold);
    mask.convertTo(mask, CV_8UC1);
    // std::cerr << "Mask size: " << mask.size() << " " <<
    //     "Mask type: " << (mask.type() == CV_8UC1) << std::endl;
    cv::imshow("threshold", mask);
    auto start = std::chrono::steady_clock::now();
    cv::inpaint(point, mask, inpaint_, 50, cv::INPAINT_TELEA);
    auto end = std::chrono::steady_clock::now();
    std::cout << "Time of inpaint: " << (end - start).count() / 1e9 << " s."
              << std::endl;
    inpaintxyz.push_back(inpaint_.clone());
  }
  cv::Mat output;
  cv::merge(pointcloudxyz, output);
  output.copyTo(pointcloud);
  return;
}