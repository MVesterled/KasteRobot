#ifndef CAMERA_H
#define CAMERA_H

//Includes for camera
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

class Camera
{
public:
    Camera();

    //Function that calibrates the camera view
    void calibrateCamera();
    //Function to convert from camera/pixel coordinates to real world.
    cv::Point2f TransformPoint(cv::Point2f pixelPoint);

    //returns transformation matrix
    cv::Mat getHomoMat() const;

    //returns transformation matrix
    cv::Mat getMapX() const;

    //returns transformation matrix
    cv::Mat getMapY() const;

    //Vizulize calibration:
    void visualizeCalib();

private:
    cv::Mat mHomoMat;
    cv::Mat mMapX;
    cv::Mat mMapY;
    std::vector<cv::Point2f> mImagePoints;
    std::vector<cv::Point2f> mRealWorldPoints;
};

#endif // CAMERA_H
