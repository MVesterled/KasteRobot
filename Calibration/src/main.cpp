#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "Camera.h"

int main() {

  Camera visionCam;
  visionCam.calibrateCamera();
  cv::Point2f pixelPoint(695, 413); //creates point to convert fra camera to table
  cv::Point2f TablePoint(0, 0);     //creates point to store real world coordinate
  TablePoint = visionCam.TransformPoint(pixelPoint);

  std::cout << "Homography matrix: " << visionCam.getHomoMat() << std::endl;

  std::cout << "The transformed real world coordinates are: ("
            << TablePoint.x << ", "
            << TablePoint.y << ")" << std::endl;

  return 0;
}
