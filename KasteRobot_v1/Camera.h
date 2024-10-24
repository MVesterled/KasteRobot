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
    void transformPicture();

    //function to return the center of balls
    void ballDetect();
    cv::Point2f nextPoint();

    //finds the balls with coulerr
    void detectGreen();
    void detectRed();
    void detectBlue();
    void colourDetection();

    //Function to test camera live feed
    void liveFeed();

private:
    cv::Mat mHomoMat;
    cv::Mat mMapX;
    cv::Mat mMapY;
    std::vector<cv::Point2f> mImagePoints;
    std::vector<cv::Point2f> mRealWorldPoints;
    std::vector<cv::Point2f> ballPoints;
    cv::Mat greenBallPicture;
    cv::Mat redBallPicture;
    cv::Mat blueBallPicture;
};

#endif // CAMERA_H
