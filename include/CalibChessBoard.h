#pragma once
// Chess board calib
// Author:Lin Wenxiang
// E-mail:3065746398@qq.com
#include <chrono>
#include <filesystem>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <vector>

int readImageFile(const std::vector<std::string> imgFile,
                  std::vector<cv::Mat> &imgs);
int readVideoStereo(std::vector<cv::Mat> &imgs1, std::vector<cv::Mat> &imgs2);
int imgsSave(std::vector<cv::Mat> &imgs, std::string header = "img");
int imgsLoad(std::vector<cv::Mat> &imgs, std::string header, int num);
int Calib(std::vector<cv::Mat>, int, int, float, cv::Mat &, cv::Mat &,
          std::vector<std::vector<cv::Point3f>> &,
          std::vector<std::vector<cv::Point2f>> &, std::vector<int> &,
          bool output = true);
int imgsFilter(std::vector<cv::Mat> imgs1, std::vector<int> find1,
               std::vector<cv::Mat> imgs2, std::vector<int> find2,
               std::vector<cv::Mat> &new_imgs1,
               std::vector<cv::Mat> &new_imgs2);
int PointListFilter(std::vector<std::vector<cv::Point3f>> &objPointsList,
                    std::vector<std::vector<cv::Point2f>> &img1PointList,
                    std::vector<std::vector<cv::Point2f>> &img2PointList,
                    std::vector<int> flags1, std::vector<int> flags2);
int stereoCalibrateAndRectify(
    std::vector<std::vector<cv::Point3f>> objPointsList,
    std::vector<std::vector<cv::Point2f>> img1PointList,
    std::vector<std::vector<cv::Point2f>> img2PointList, cv::Mat cameraMatrix1,
    cv::Mat distCoeffs1, cv::Mat cameraMatrix2, cv::Mat distCoeffs2,
    cv::Size imageSize, cv::Mat &mapX1, cv::Mat &mapY1, cv::Mat &mapX2,
    cv::Mat &mapY2, cv::Mat &Q);
void DisparityColoring(cv::Mat &dis_norm, cv::Mat &colored);
void PointcloudProcess(cv::Mat &pointcloud, float threshold = 10000);
void onMouse(int event, int x, int y, int flags, void *img);
void BMConfig(cv::Ptr<cv::StereoBM> matcher);
void SGBMConfig(cv::Ptr<cv::StereoSGBM> matcher);

// Class
class CameraParams {
public:
  // mapX1,mapY1,mapX2,mapY2,matrix_xml
  void Load(std::string, std::string);
  void Write(std::string, std::string);
  friend std::ostream &operator<<(std::ostream &os, CameraParams params);
  cv::Mat mapX1;
  cv::Mat mapY1;
  cv::Mat mapX2;
  cv::Mat mapY2;
  cv::Mat Q, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2;
};
class StereoImageLoader {
public:
  enum LoadType { NOT_INIT, DEVICE, MAT_CONTAINER };
  StereoImageLoader();
  int Init(); // load image fron device videocapture
  int Init(std::vector<cv::Mat> &,
           std::vector<cv::Mat> &); // load image from container
  int nextFrame(cv::Mat &img1, cv::Mat &img2);
  int release();

private:
  int type_;
  int count_;
  cv::VideoCapture cap1_, cap2_;
  long container_idx_, container_max_;
  std::vector<cv::Mat> container1_, container2_;
};
class CalibChessBoard {
public:
  CalibChessBoard();
  void CalibStart(int boardWidth = 9, int boardHeight = 6,
                  float squareSize = 7);

  // Override
  // Load imgs from device
  void LoadImgsForCalib();
  // Override
  //  folder_path must have sub folder left and right
  void LoadImgsForCalib(std::string folder_path, int num);
  // Override
  // use checkpoint to generate folder_path,0 for checkpoints_main_path_
  void LoadImgsForCalib(long long checkpoints, int num);
  void SaveCalibImages(std::string);
  void SaveCalibImages(); // load from checkpoints_main_path_

  void LoadParams(CameraParams params) { params_ = params; }
  void LoadParamsCheckPoints(std::string check_point_main_path);
  void LoadParamsCheckPoints(long long checkpoints = 0); // override
  void SaveParamsCheckPoints(std::string check_point_path);
  void SaveParamsCheckPoints(void); // override
  void Match();

  CameraParams params_;
  std::vector<cv::Mat> left_calib_imgs_;
  std::vector<cv::Mat> right_calib_imgs_;
  StereoImageLoader match_img_loader_;

private:
  long long checkpoints_time_rep_;
  std::string checkpoints_main_path_;
};
