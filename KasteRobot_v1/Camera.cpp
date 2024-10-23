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

void Camera::ballDetect(){
/*
int main(int argc, char** argv)
{
    //load the iimage, by adding the file path
    const char* filename = argc >=2 ? argv[1] : "/home/kally/Downloads/ballTestPerspective.jpg";
    // Loads the image in colour
    Mat src = imread( samples::findFile( filename ), IMREAD_COLOR );
    // Check if image is loaded fine
    if(src.empty()){
        printf(" Error opening image\n");
        printf(" Program Arguments: [image_name -- default %s] \n", filename);
        return EXIT_FAILURE;
    }
    //define a matrix to store the gray scale of the image
    Mat gray;
    //convert the imagee from BGR to grayscale
    cvtColor(src, gray, COLOR_BGR2GRAY);
    //Apply a median filter to reduce noise, with a medium sized kernel  5
    medianBlur(gray, gray, 5);
    //create a vec to store the circles, each circle is represented by 3 values: x,y,radius
    vector<Vec3f> circles;
    HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
                 gray.rows/16,  // change this value to detect circles with different distances to each other
                 100, 30, 10, 30 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );
    for( size_t i = 0; i < circles.size(); i++ ) //loop through the detected circles and draw them on the original image
    {
        Vec3i c = circles[i]; //get the ith  circle (x,y,r)
        Point center = Point(c[0], c[1]); // circle center
        //draw the cicrle center as a small dot
        circle( src, center, 1, Scalar(0,100,100), 3, LINE_AA); //draw a small circle in the center with colour (0,100,1)
        //draw the circle outline with the rr of the detected circle
        int radius = c[2];//get the radius of the ith circle
        circle( src, center, radius, Scalar(255,0,255), 3, LINE_AA); //draw the circles perimeter with colour (255,0,255)
        cout<< center.x << ", "<< center.y <<endl;
    }
    imshow("detected circles", src);
    waitKey();
    return EXIT_SUCCESS;
} */
}

void Camera::colourDetection()
{
    /*
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "iostream"

void DetectGreen(cv::Mat input, cv::Mat &output)
{
    //Initializes temporary images for computation:
    cv::Mat imgHSV;
    //HSV values for the green colour wanted:
    int hmin = 45, smin = 251, vmin = 180;
    int hmax = 179, smax = 255, vmax = 255;
    //Convertion to HSV-colourspace from RGB-colourspace:
    cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

    //Sets the HSV colour values:
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);

    //Applying colourfilter to the image:
    cv::inRange(imgHSV, lower, upper, output);
    //Shows the colour mask:
    cv::imshow("Image Green", output);
    cv::waitKey(0);
}

void DetectBlue(cv::Mat input, cv::Mat &output)
{
    //Initializes temporary images for computation:
    cv::Mat imgHSV;
    //HSV values for the blue colour wanted:
    int hmin = 87, smin = 134, vmin = 13;
    int hmax = 120, smax = 255, vmax = 84;
    //Convertion to HSV-colourspace from RGB-colourspace:
    cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

    //Sets the HSV colour values:
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);

    //Applying colourfilter to the image:
    cv::inRange(imgHSV, lower, upper, output);
    //Shows the colour mask:
    cv::imshow("Image Blue", output);
    cv::waitKey(0);
}

void DetectRed(cv::Mat input, cv::Mat &output)
{
    //Initializes temporary images for computation:
    cv::Mat imgHSV;
    //HSV values for the red colour wanted:
    int hmin = 0, smin = 0, vmin = 71;
    int hmax = 13, smax = 255, vmax = 215;
    //Convertion to HSV-colourspace from RGB-colourspace:
    cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

    //Sets the HSV colour values:
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);

    //Applying colourfilter to the image:
    cv::inRange(imgHSV, lower, upper, output);
    //Shows the colour mask:
    cv::imshow("Image Red", output);
    cv::waitKey(0);
}

void DetectOrange(cv::Mat input, cv::Mat &output)
{
    //Initializes temporary images for computation:
    cv::Mat imgHSV;
    //HSV values for the orange colour wanted:
    int hmin = 0, smin = 255, vmin = 66;
    int hmax = 20, smax = 255, vmax = 165;
    //Convertion to HSV-colourspace from RGB-colourspace:
    cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

    //Sets the HSV colour values:
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);

    //Applying colourfilter to the image:
    cv::inRange(imgHSV, lower, upper, output);
    //Shows the colour mask:
    cv::imshow("Image Orange", output);
    cv::waitKey(0);
}

void DetectYellow(cv::Mat input, cv::Mat &output)
{
    //Initializes temporary images for computation:
    cv::Mat imgHSV;
    //HSV values for the yellow colour wanted:
    int hmin = 25, smin = 255, vmin = 66;
    int hmax = 38, smax = 255, vmax = 165;
    //Convertere til HSV-colourspace fra RGB-colourspace
    cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

    //Sets the HSV colour values:
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);
    //Applying colourfilter to the image:
    cv::inRange(imgHSV, lower, upper, output);
    //Shows the colour mask:
    cv::imshow("Image Yellow", output);
    cv::waitKey(0);
}

void CalibrateColours(cv::Mat input)
{
    //Initializes temporary images for computation:
    cv::Mat imgHSV, mask;
    //initialize min og max HSV values for colourspace:
    int hmin = 0, smin = 0, vmin = 0;
    int hmax = 179, smax = 255, vmax = 255;
    //Converters to HSV Colourspace:
    cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);
    //Creates trackbars:
    cv::namedWindow("Trackbars", (640, 200));
    cv::createTrackbar("Hue min", "Trackbars", &hmin, 179);
    cv::createTrackbar("Hue max", "Trackbars", &hmax, 179);
    cv::createTrackbar("Sat min", "Trackbars", &smin, 255);
    cv::createTrackbar("Sat max", "Trackbars", &smax, 255);
    cv::createTrackbar("Val min", "Trackbars", &vmin, 255);
    cv::createTrackbar("Val max", "Trackbars", &vmax, 255);

    //Runs the program as a video while the values get updated from the trackbars:
    while (true){
        //Sets upper and lower limits:
        cv::Scalar lower(hmin, smin, vmin);
        cv::Scalar upper(hmax, smax, vmax);
        //Applying colourfilter to the image:
        cv::inRange(imgHSV, lower, upper, mask);
        //Show the colour mask:
        cv::resize(mask, mask, cv::Size(mask.cols * 0.8, mask.rows * 0.8));
        cv::imshow("Image mask", mask);
        cv::waitKey(1);
    }
}

int main()
{
    // Load an image from a file
    std::string imagePath = "/home/kally/Downloads/ballTest.jpg";  // Replace with the actual path to your image
    cv::Mat inputImage = cv::imread(imagePath);

    cv::imshow("Input", inputImage);
    cv::waitKey(0);
    CalibrateColours(inputImage);
    // Check if the image was loaded successfully
    if (inputImage.empty())
    {
        std::cout << "Could not load image. Check the file path." << std::endl;
        return -1;
    }

    // Create a Mat to store the output
    cv::Mat outputImage;

    // Call the DetectGreen function
    DetectGreen(inputImage, outputImage);



    return 0;
}#include "opencv2/imgcodecs.hpp"
        #include "opencv2/highgui.hpp"
        #include "opencv2/imgproc.hpp"
        #include "iostream"

        void DetectGreen(cv::Mat input, cv::Mat &output)
        {
            //Initializes temporary images for computation:
            cv::Mat imgHSV;
            //HSV values for the green colour wanted:
            int hmin = 45, smin = 251, vmin = 180;
            int hmax = 179, smax = 255, vmax = 255;
            //Convertion to HSV-colourspace from RGB-colourspace:
            cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

            //Sets the HSV colour values:
            cv::Scalar lower(hmin, smin, vmin);
            cv::Scalar upper(hmax, smax, vmax);

            //Applying colourfilter to the image:
            cv::inRange(imgHSV, lower, upper, output);
            //Shows the colour mask:
            cv::imshow("Image Green", output);
            cv::waitKey(0);
        }

        void DetectBlue(cv::Mat input, cv::Mat &output)
        {
            //Initializes temporary images for computation:
            cv::Mat imgHSV;
            //HSV values for the blue colour wanted:
            int hmin = 87, smin = 134, vmin = 13;
            int hmax = 120, smax = 255, vmax = 84;
            //Convertion to HSV-colourspace from RGB-colourspace:
            cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

            //Sets the HSV colour values:
            cv::Scalar lower(hmin, smin, vmin);
            cv::Scalar upper(hmax, smax, vmax);

            //Applying colourfilter to the image:
            cv::inRange(imgHSV, lower, upper, output);
            //Shows the colour mask:
            cv::imshow("Image Blue", output);
            cv::waitKey(0);
        }

        void DetectRed(cv::Mat input, cv::Mat &output)
        {
            //Initializes temporary images for computation:
            cv::Mat imgHSV;
            //HSV values for the red colour wanted:
            int hmin = 0, smin = 0, vmin = 71;
            int hmax = 13, smax = 255, vmax = 215;
            //Convertion to HSV-colourspace from RGB-colourspace:
            cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

            //Sets the HSV colour values:
            cv::Scalar lower(hmin, smin, vmin);
            cv::Scalar upper(hmax, smax, vmax);

            //Applying colourfilter to the image:
            cv::inRange(imgHSV, lower, upper, output);
            //Shows the colour mask:
            cv::imshow("Image Red", output);
            cv::waitKey(0);
        }

        void DetectOrange(cv::Mat input, cv::Mat &output)
        {
            //Initializes temporary images for computation:
            cv::Mat imgHSV;
            //HSV values for the orange colour wanted:
            int hmin = 0, smin = 255, vmin = 66;
            int hmax = 20, smax = 255, vmax = 165;
            //Convertion to HSV-colourspace from RGB-colourspace:
            cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

            //Sets the HSV colour values:
            cv::Scalar lower(hmin, smin, vmin);
            cv::Scalar upper(hmax, smax, vmax);

            //Applying colourfilter to the image:
            cv::inRange(imgHSV, lower, upper, output);
            //Shows the colour mask:
            cv::imshow("Image Orange", output);
            cv::waitKey(0);
        }

        void DetectYellow(cv::Mat input, cv::Mat &output)
        {
            //Initializes temporary images for computation:
            cv::Mat imgHSV;
            //HSV values for the yellow colour wanted:
            int hmin = 25, smin = 255, vmin = 66;
            int hmax = 38, smax = 255, vmax = 165;
            //Convertere til HSV-colourspace fra RGB-colourspace
            cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

            //Sets the HSV colour values:
            cv::Scalar lower(hmin, smin, vmin);
            cv::Scalar upper(hmax, smax, vmax);
            //Applying colourfilter to the image:
            cv::inRange(imgHSV, lower, upper, output);
            //Shows the colour mask:
            cv::imshow("Image Yellow", output);
            cv::waitKey(0);
        }

        void CalibrateColours(cv::Mat input)
        {
            //Initializes temporary images for computation:
            cv::Mat imgHSV, mask;
            //initialize min og max HSV values for colourspace:
            int hmin = 0, smin = 0, vmin = 0;
            int hmax = 179, smax = 255, vmax = 255;
            //Converters to HSV Colourspace:
            cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);
            //Creates trackbars:
            cv::namedWindow("Trackbars", (640, 200));
            cv::createTrackbar("Hue min", "Trackbars", &hmin, 179);
            cv::createTrackbar("Hue max", "Trackbars", &hmax, 179);
            cv::createTrackbar("Sat min", "Trackbars", &smin, 255);
            cv::createTrackbar("Sat max", "Trackbars", &smax, 255);
            cv::createTrackbar("Val min", "Trackbars", &vmin, 255);
            cv::createTrackbar("Val max", "Trackbars", &vmax, 255);

            //Runs the program as a video while the values get updated from the trackbars:
            while (true){
                //Sets upper and lower limits:
                cv::Scalar lower(hmin, smin, vmin);
                cv::Scalar upper(hmax, smax, vmax);
                //Applying colourfilter to the image:
                cv::inRange(imgHSV, lower, upper, mask);
                //Show the colour mask:
                cv::resize(mask, mask, cv::Size(mask.cols * 0.8, mask.rows * 0.8));
                cv::imshow("Image mask", mask);
                cv::waitKey(1);
            }
        }

        int main()
        {
            // Load an image from a file
            std::string imagePath = "/home/kally/Downloads/ballTest.jpg";  // Replace with the actual path to your image
            cv::Mat inputImage = cv::imread(imagePath);

            cv::imshow("Input", inputImage);
            cv::waitKey(0);
            CalibrateColours(inputImage);
            // Check if the image was loaded successfully
            if (inputImage.empty())
            {
                std::cout << "Could not load image. Check the file path." << std::endl;
                return -1;
            }

            // Create a Mat to store the output
            cv::Mat outputImage;

            // Call the DetectGreen function
            DetectGreen(inputImage, outputImage);



            return 0;
        }
        */
}


