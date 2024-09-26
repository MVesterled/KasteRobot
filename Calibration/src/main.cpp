#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char **argv) {

  (void)argc;
  (void)argv;

  std::vector<cv::String> fileNames;
  //glob gets all files of a folder (false means no sub directories)
  cv::glob("/home/matmat1000/Documents/Calibration_cpp/TestPic2/*.png", fileNames, false);
  cv::Size patternSize(9, 6); //internal corners
  // Declare a vector of vectors to store 2D points (found corners) for each image
  // The size of 'q' matches the number of files in 'fileNames', so each image will have a corresponding entry
  std::vector<std::vector<cv::Point2f>> q(fileNames.size());

  // Detect feature points
  std::size_t i = 0;
  for (auto const &f : fileNames) {
    std::cout << std::string(f) << std::endl;

    // 1. Read in the image an call cv::findChessboardCorners()
    cv::Mat img = cv::imread(f, cv::IMREAD_GRAYSCALE);
    //cv::findChessboardCorners(img, patternSize, q[i]);
    bool success = cv::findChessboardCorners(img, patternSize, q[i], cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    // 2. Use cv::cornerSubPix() to refine the found corner detections
    cv::Size winSize = cv::Size( 11, 11 );
    cv::Size zeroZone = cv::Size( -1, -1 );
    //stop critirias for corner detection (100 times max or max change by 0.001)
    cv::TermCriteria criteria = cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 0.001 );
    cv::cornerSubPix(img, q[i], winSize, zeroZone, criteria);

    // Display
    //Succes Parameter indicating whether the complete board was found or not
    cv::drawChessboardCorners(img, patternSize, q[i], success);
    //cv::imshow("chessboard detection", img);
    //cv::waitKey(0);
    i++;
  }

  std::vector<std::vector<cv::Point3f>> Q;
  /* 3. Generate checkerboard (world) coordinates Q. The board has 10*7
     fields with a size of 70x70mm.
     Note: You can freely choose where to put the coordinate frame. The standard
     is to define the points in the xy-plane and set the z-coordinate to 0 */
  //Size of board square
  float squareSize = 70.0f;

  // Generate the 3D coordinates for the checkerboard corners (Z = 0)
  std::vector<cv::Point3f> QView;
  for (int i = 0; i < patternSize.height; ++i) {
    for (int j = 0; j < patternSize.width; ++j) {
        QView.push_back(cv::Point3f(j * squareSize, i * squareSize, 0.0f));
    }
  }

  // Since the same checkerboard is used in all views, we push QView n times (for each image)
  for (std::size_t i = 0; i < fileNames.size(); i++) {
    Q.push_back(QView); // Each element in Q corresponds to a view/image
  }

  cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
  cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients

  std::vector<cv::Mat> rvecs, tvecs;
  std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
  cv::Size frameSize(1440, 1080);

  std::cout << "Calibrating..." << std::endl;
  // 4. Call "float error = cv::calibrateCamera()" with the input coordinates
  // and output parameters as declared above...

  float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, stdIntrinsics, stdExtrinsics, perViewErrors);

  std::cout << "Reprojection error = " << error << "\nK =\n"
            << K << "\nk=\n"
            << k << std::endl;

  // Precompute lens correction interpolation
  cv::Mat mapX, mapY;
  cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1,
                              mapX, mapY);

  // Show lens corrected images
  for (auto const &f : fileNames) {
    std::cout << std::string(f) << std::endl;

    cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);

    cv::Mat imgUndistorted;
    // 5. Remap the image using the precomputed interpolation maps.
    cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);


    // Display
    cv::imshow("Undistorted Image", imgUndistorted);
    cv::waitKey(0);
  }

  return 0;
}
