#include "Camera.h"

Camera::Camera(){}

//Function that calibrates the camera view
void Camera::calibrateCamera(){
    std::cout << "Calibration started..." << std::endl;
    std::vector<cv::String> fileNames;
    //glob gets all files of a folder (false means no sub directories)
    cv::glob("/home/matmat1000/Documents/Calibration_cpp/TestPic2/*.png", fileNames, false);
    //cv::glob("/home/matmat1000/Documents/calibrationImages/*.png", fileNames, false);
    cv::Size patternSize(9, 6); //internal corners of calibrationplate
    //cv::Size patternSize(24, 17); //internal corners of calibrationplate
    // Declare a vector of vectors to store 2D points (found corners) for each image
    // The size of 'q' matches the number of files in 'fileNames', so each image will have a corresponding entry
    std::vector<std::vector<cv::Point2f>> q(fileNames.size());

    // Detect feature points
    std::size_t i = 0;
    for (auto const &f : fileNames) {
        //std::cout << std::string(f) << std::endl; //can print out every file read

        // Read in the image as graycale
        cv::Mat img = cv::imread(f, cv::IMREAD_GRAYSCALE);
        //cv::findChessboardCorners finds inner corners automaticly (succes = true if all corners are found )
        bool success = cv::findChessboardCorners(img, patternSize, q[i], cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        // Refines the found corner detections
        cv::Size winSize = cv::Size( 11, 11 );
        cv::Size zeroZone = cv::Size( -1, -1 );
        //stop critirias for corner detection (100 times max or max change by 0.001)
        cv::TermCriteria criteria = cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 0.001 );
        cv::cornerSubPix(img, q[i], winSize, zeroZone, criteria);

        // Display
        //Succes Parameter indicating whether the complete board was found or not
/*
        cv::drawChessboardCorners(img, patternSize, q[i], success);
        cv::namedWindow("chessboard detection", cv::WINDOW_NORMAL); // Make window resizable
        cv::resizeWindow("chessboard detection", 1500, 1000); // Resize window to specific size
        cv::imshow("chessboard detection", img);
        cv::waitKey(0);
*/
        i++;
    }

    std::vector<std::vector<cv::Point3f>> Q;
    /* Generate checkerboard (world) coordinates Q. The board has 10*7
     fields with a size of 70x70mm. */
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

    std::vector<cv::Mat> rvecs, tvecs; //rotation and translation vector
    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors; //for standard deviations
    cv::Size frameSize(1440, 1080); //camera resolution


    std::cout << "Calibrating..." << std::endl;
    // Finds the error in the calibration
    float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, stdIntrinsics, stdExtrinsics, perViewErrors);

    std::cout << "Reprojection error = " << error << "\nK =\n"
              << K << "\nk=\n"
              << k << std::endl;


    // Precompute lens correction interpolation
    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1,
                                mMapX, mMapY);

    // Show lens corrected images
    for (auto const &f : fileNames) {
        //::cout << std::string(f) << std::endl; //can print out every file read

        cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);

        cv::Mat imgUndistorted;
        // Remap the image using the precomputed interpolation maps.
        cv::remap(img, imgUndistorted, mMapX, mMapY, cv::INTER_LINEAR);


        // Display
        //cv::namedWindow("Undistorted Image", cv::WINDOW_NORMAL); // Make window resizable
        //cv::resizeWindow("Undistorted Image", 1500, 1000); // Resize window to specific size
        //cv::imshow("Undistorted Image", imgUndistorted);
        //cv::waitKey(0);
    }

    //Real world transformation of coordiantes
    // Image points (corresponding to real-world points)
    mImagePoints.push_back(cv::Point2f(417, 385)); // (0, 0) real world top left
    mImagePoints.push_back(cv::Point2f(1043, 409)); // (800, 0) real world top right
    mImagePoints.push_back(cv::Point2f(421, 1041)); // (0, 750) real world bottom left
    mImagePoints.push_back(cv::Point2f(1047, 972)); // (800, 750) real world bottom right

    // Real world points
    mRealWorldPoints.push_back(cv::Point2f(0.0f, 0.0f));     // (0, 0) real world top left
    mRealWorldPoints.push_back(cv::Point2f(800.0f, 0.0f));    // (800, 0) real world top right
    mRealWorldPoints.push_back(cv::Point2f(0.0f, 750.0f));    // (0, 750) real world bottom left
    mRealWorldPoints.push_back(cv::Point2f(800.0f, 750.0f));   // (800, 750) real world bottom right

    // Compute the homography matrix
    mHomoMat = cv::getPerspectiveTransform(mImagePoints, mRealWorldPoints);

    std::cout << "Calibration done" << std::endl;
}

//Function to convert from camera/pixel coordinates to real world.
cv::Point2f Camera::TransformPoint(cv::Point2f pixelPoint){
    // We can use the homography matrix to transform a point

    // Puts the coordinate into a vector as cv::perspectiveTransform needs a vector
    std::vector<cv::Point2f> pixelPoints;
    pixelPoints.push_back(pixelPoint);

    // Creates a vector for outpur coordinate
    std::vector<cv::Point2f> realWorldTransformedPoints;

    // Apply the homography to the point. This also converts point to homogeneous coordinates (adds 1)
    cv::perspectiveTransform(pixelPoints, realWorldTransformedPoints, mHomoMat);

    return realWorldTransformedPoints[0];
}

cv::Mat Camera::getHomoMat() const{
    return mHomoMat;
}

//returns mMapX
cv::Mat Camera::getMapX() const{
    return mMapX;
}

//returns mMapY
cv::Mat Camera::getMapY() const{
    return mMapY;
}

//Vizulize calibration:
void Camera::visualizeCalib(){
    cv::Mat input = cv::imread("/home/matmat1000/Documents/ballTest.jpg");
    cv::Mat output;
    cv::warpPerspective(input, output, mHomoMat, cv::Size(800,750)); // Use input size or desired output size
    //cv::imwrite("/home/matmat1000/Documents/ballTestPerspective.jpg", output);
    // Display the transformed image
    cv::namedWindow("Undist before perspectiveTrans", cv::WINDOW_NORMAL); // Make window resizable
    cv::resizeWindow("Undist before perspectiveTrans", 800, 750);       // Resize window to specific size
    cv::imshow("Undist before perspectiveTrans", input);
    cv::waitKey(0); // Wait for a key press

    // Display the transformed image
    cv::namedWindow("Undist after perspectiveTrans", cv::WINDOW_NORMAL); // Make window resizable
    cv::resizeWindow("Undist after perspectiveTrans", 800, 750);       // Resize window to specific size
    cv::imshow("Undist after perspectiveTrans", output);
    cv::waitKey(0); // Wait for a key press
}


