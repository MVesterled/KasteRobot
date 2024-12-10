#include "Camera.h"

Camera::Camera(){
    //Change this file to your own path
    mPicture = cv::imread("/home/matmat1000/C++/SemProjekt/KasteRobot/KasteRobot_v1/Img/ballTest.jpg");
    //cv::imshow("Default image", mPicture);
    //cv::waitKey(0);
}

//Function that calibrates the camera view
void Camera::calibrateCamera(){
    std::cout << "Calibration started..." << std::endl;
    std::vector<cv::String> fileNames;
    //glob gets all files of a folder (false means no sub directories)
    //Change path to match your system
    cv::glob("/home/matmat1000/C++/SemProjekt/KasteRobot/KasteRobot_v1/Img/Calib/*.png", fileNames, false);
    cv::Size patternSize(9, 6); //internal corners of calibrationplate
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
/* Prints out the distortion parameters
    std::cout << "Reprojection error = " << error << "\nK =\n"
              << K << "\nk=\n"
              << k << std::endl;
*/

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
    mImagePoints.push_back(cv::Point2f(412, 380)); // (0, 0) real world top left
    mImagePoints.push_back(cv::Point2f(1042, 405)); // (800, 0) real world top right
    mImagePoints.push_back(cv::Point2f(415, 1041)); // (0, 750) real world bottom left
    mImagePoints.push_back(cv::Point2f(1046, 969)); // (800, 750) real world bottom right

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
void Camera::transformPicture(){
    //Warps picture by applying the homoMat
    cv::warpPerspective(mPicture, mPicture, mHomoMat, cv::Size(800,750)); // Use input size or desired output size
    std::cout << "Picture warpped succesfully!" << std::endl;
    //cv::imwrite("/home/matmat1000/Documents/ballTestPerspective.jpg", output);
    /*
    // Display the transformed image
    cv::namedWindow("Undist before perspectiveTrans", cv::WINDOW_NORMAL); // Make window resizable
    cv::resizeWindow("Undist before perspectiveTrans", 800, 750);       // Resize window to specific size
    cv::imshow("Undist before perspectiveTrans", input);
    cv::waitKey(0); // Wait for a key press
*/
    //Display the transformed image
    //cv::namedWindow("Undist after perspectiveTrans", cv::WINDOW_NORMAL); // Make window resizable
    //cv::resizeWindow("Undist after perspectiveTrans", 800, 750);       // Resize window to specific size
    //cv::imshow("Undist after perspectiveTrans", mPicture);
    //cv::waitKey(0); // Wait for a key press

}

void Camera::ballDetect(std::string color){
    cv::Mat src_grey;
    //loads in image as grayscale (Because of binary)
    if (color == "red"){
        src_grey = 255*redBallPicture;;
    }
    else if (color == "green"){
        src_grey = 255*greenBallPicture;;
    }
    //imshow("255*", src_grey);
    //cv::waitKey();

    //Apply a median filter to reduce noise, with a medium sized kernel  5
    cv::medianBlur(src_grey, src_grey, 5);

    cv::Mat edges;
    cv::Canny(src_grey, src_grey, 50, 150);

    //create a vec to store the circles, each circle is represented by 3 values: x,y,radius
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(src_grey, circles, cv::HOUGH_GRADIENT, 1,
                     src_grey.rows/8,  // change this value to detect circles with different distances to each other
                     50, 15, 10, 30); // change the last two parameters (min radius and max radius)

    //vizulice  circles
    for( size_t i = 0; i < circles.size(); i++ ) //loop through the detected circles and draw them on the original image
    {
        cv::Vec3i c = circles[i]; //get the ith  circle (x,y,r)
        cv::Point2f center = cv::Point2f(c[0], c[1]); // circle center
        //draw the cicrle center as a small dot
        cv::circle( mPicture, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA); //draw a small circle in the center with colour (0,100,1)
        //draw the circle outline with the rr of the detected circle
        int radius = c[2];//get the radius of the ith circle
        circle( mPicture, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA); //draw the circles perimeter with colour (255,0,255)
        //std::cout<< center.x << ", "<< center.y << std::endl;
        ballPoints.emplace_back(cv::Point2f(c[0], c[1]));
    }

    //imshow("detected circles", mPicture);
    //cv::waitKey();

    if (ballPoints.size() > 0){
        std::cout << "Balls detected" << std::endl; }
    else{
        std::cout << "No balls detected" << std::endl; }

}

cv::Point2f Camera::nextPoint()
{
    if (ballPoints.empty()) {
        std::cerr << "Error: No more points available in ballPoints" << std::endl;
        return cv::Point2f(200, 200);
    }

    cv::Point2f Temp = ballPoints[ballPoints.size()-1];
    ballPoints.pop_back();

    return Temp;

}

void Camera::detectGreen()
{
    //Initializes temporary images for computation:
    cv::Mat imgHSV;
    //HSV values for the green colour wanted:
    int hmin = 54, smin = 223, vmin = 90;
    int hmax = 83, smax = 255, vmax = 255;
    //Convertion to HSV-colourspace from RGB-colourspace:
    cv::cvtColor(mPicture, imgHSV, cv::COLOR_BGR2HSV);

    //Sets the HSV colour values:
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);

    //Applying colourfilter to the image:
    cv::inRange(imgHSV, lower, upper, greenBallPicture);
    //Shows the colour mask:
    //cv::imshow("Image Green", greenBallPicture);
    //cv::waitKey(0);
}

void Camera::detectRed()
{
    //Initializes temporary images for computation:
    cv::Mat imgHSV;
    //HSV values for the green colour wanted:
    int hmin = 0, smin = 190, vmin = 80;
    int hmax = 22, smax = 255, vmax = 255;
    //Convertion to HSV-colourspace from RGB-colourspace:
    cv::cvtColor(mPicture, imgHSV, cv::COLOR_BGR2HSV);

    //Sets the HSV colour values:
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);

    //Applying colourfilter to the image:
    cv::inRange(imgHSV, lower, upper, redBallPicture);
    //Shows the colour mask:
    //cv::imshow("Image Green", redBallPicture);
    //cv::waitKey(0);
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
    int hmin = 54, smin = 223, vmin = 90;
    int hmax = 83, smax = 255, vmax = 255;
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
    int hmin = 0, smin = 152, vmin = 141;
    int hmax = 20, smax = 255, vmax = 222;
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

void Camera::centerOfMass(){
    //loads in image as grayscale
    cv::Mat src_grey = 255*greenBallPicture;

    imshow("255*", src_grey);
    cv::waitKey();
    //change to won path (saves center of mass picture)
    cv::imwrite("/home/matmat1000/Documents/centerOfMass.jpg", src_grey);


    // Calculate moments directly on the binary image
    cv::Moments M = cv::moments(src_grey, true);

    // Calculate the center using the moments
    if (M.m00 != 0) {
        int cx = static_cast<int>(M.m10 / M.m00);
        int cy = static_cast<int>(M.m01 / M.m00);
        std::cout << "Center of the circle: (" << cx << ", " << cy << ")" << std::endl;
        // Draw the center point for visualization
        cv::circle(mPicture, cv::Point(cx, cy), 5, cv::Scalar(255,255,0), 2, cv::LINE_AA);
        cv::imshow("Center of Circle", mPicture);
        cv::waitKey(0);
    } else {
        std::cerr << "No mass found in the image, cannot determine center." << std::endl;
    }
    //change to won path (saves center of mass result-picture)
    cv::imwrite("/home/matmat1000/Documents/centerOfMassVSHough.jpg", mPicture);

}

void Camera::liveFeed()
{
    /*
    //Live camera feed test
    int myExposure = 30000;

    // The exit code of the sample application.
    int exitCode = 0;

    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;

    try
    {
        // Create an instant camera object with the camera device found first.
        Pylon::CInstantCamera camera( Pylon::CTlFactory::GetInstance().CreateFirstDevice());

        // Get a camera nodemap in order to access camera parameters.
        GenApi::INodeMap& nodemap= camera.GetNodeMap();

        // Open the camera before accessing any parameters.
        camera.Open();
        // Create pointers to access the camera Width and Height parameters.
        GenApi::CIntegerPtr width= nodemap.GetNode("Width");
        GenApi::CIntegerPtr height= nodemap.GetNode("Height");

        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        //camera.MaxNumBuffer = 5;

        // Create a pylon ImageFormatConverter object.
        Pylon::CImageFormatConverter formatConverter;
        // Specify the output pixel format.
        formatConverter.OutputPixelFormat= Pylon::PixelType_BGR8packed;
        // Create a PylonImage that will be used to create OpenCV images later.
        Pylon::CPylonImage pylonImage;

        // Create an OpenCV image.
        cv::Mat openCvImage;


        // Set exposure to manual
        GenApi::CEnumerationPtr exposureAuto( nodemap.GetNode( "ExposureAuto"));
        if ( GenApi::IsWritable( exposureAuto)){
            exposureAuto->FromString("Off");
            std::cout << "Exposure auto disabled." << std::endl;
        }

        // Set custom exposure
        GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
        std::cout << "Old exposure: " << exposureTime->GetValue() << std::endl;
        if(exposureTime.IsValid()) {
            if(myExposure >= exposureTime->GetMin() && myExposure <= exposureTime->GetMax()) {
                exposureTime->SetValue(myExposure);
            }else {
                exposureTime->SetValue(exposureTime->GetMin());
                std::cout << ">> Exposure has been set with the minimum available value." << std::endl;
                std::cout << ">> The available exposure range is [" << exposureTime->GetMin() << " - " << exposureTime->GetMax() << "] (us)" << std::endl;
            }
        }else {

            std::cout << ">> Failed to set exposure value." << std::endl;
            return false;
        }
        std::cout << "New exposure: " << exposureTime->GetValue() << std::endl;

        // Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

        // This smart pointer will receive the grab result data.
        Pylon::CGrabResultPtr ptrGrabResult;

        // image grabbing loop
        int frame = 1;
        while ( camera.IsGrabbing())
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult( 5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                // Access the image data.
                //cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
                //cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;

                // Convert the grabbed buffer to a pylon image.
                formatConverter.Convert(pylonImage, ptrGrabResult);

                // Create an OpenCV image from a pylon image.
                openCvImage= cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());



                //////////////////////////////////////////////////////
                //////////// Here your code begins ///////////////////
                //////////////////////////////////////////////////////

                // Create an OpenCV display window.
                cv::namedWindow( "myWindow", cv::WINDOW_NORMAL); // other options: CV_AUTOSIZE, CV_FREERATIO

                // Display the current image in the OpenCV display window.
                cv::imshow( "myWindow", openCvImage);

                // Detect key press and quit if 'q' is pressed
                int keyPressed = cv::waitKey(1);
                if(keyPressed == 'q'){ //quit
                    std::cout << "Shutting down camera..." << std::endl;
                    camera.Close();
                    std::cout << "Camera successfully closed." << std::endl;
                    break;
                }

                ////////////////////////////////////////////////////
                //////////// Here your code ends ///////////////////
                ////////////////////////////////////////////////////




                frame++;

            }
            else
            {
                std::cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
            }
        }

    }
    catch (GenICam::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
        exitCode = 1;
    }

    return exitCode;
*/
}

void Camera::capturePicture()
{

    //Camera capure 1 picture, can only be ran with camera connected

    // Initialize Pylon runtime
    Pylon::PylonAutoInitTerm autoInitTerm;

    try
    {
        // Create an instant camera object with the camera device found first.
        Pylon::CInstantCamera camera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());

        // Get a camera nodemap in order to access camera parameters.
        GenApi::INodeMap& nodemap = camera.GetNodeMap();

        // Open the camera before accessing any parameters.
        camera.Open();

        // Create a pylon ImageFormatConverter object.
        Pylon::CImageFormatConverter formatConverter;
        formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;

        // Create a PylonImage that will hold the captured image.
        Pylon::CPylonImage pylonImage;

        // Set custom exposure (optional, you can remove this if not needed)
        int myExposure = 30000;
        GenApi::CEnumerationPtr exposureAuto(nodemap.GetNode("ExposureAuto"));
        if (GenApi::IsWritable(exposureAuto))
        {
            exposureAuto->FromString("Off");
        }

        GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
        if (exposureTime.IsValid())
        {
            if (myExposure >= exposureTime->GetMin() && myExposure <= exposureTime->GetMax())
            {
                exposureTime->SetValue(myExposure);
            }
            else
            {
                exposureTime->SetValue(exposureTime->GetMin());
            }
        }

        // Start grabbing a single image.
        camera.StartGrabbing(1);

        // This smart pointer will receive the grab result data.
        Pylon::CGrabResultPtr ptrGrabResult;

        // Wait for an image and then retrieve it.
        camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

        // Check if the image was grabbed successfully.
        if (ptrGrabResult->GrabSucceeded())
        {
            // Convert the grabbed buffer to a pylon image.
            formatConverter.Convert(pylonImage, ptrGrabResult);

            // Create an OpenCV image (cv::Mat) from the pylon image.
            cv::Mat img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());

            // At this point, img contains the captured image as a cv::Mat object.
            // You can now use this cv::Mat object for further processing.

            // Undistort the image using the precomputed maps (optional if needed)
            cv::Mat undistortedImage;
            cv::remap(img, undistortedImage, getMapX(), getMapY(), cv::INTER_LINEAR);

            std::cout << "Image captured successfully!" << std::endl;
            //change to own path (saves picture)
            //cv::imwrite("/home/matmat1000/Documents/ballTestNewX2.jpg", undistortedImage);
            mPicture = undistortedImage.clone();
            // Create an OpenCV display window

            // cv::namedWindow( "Captured Image unDist", cv::WINDOW_NORMAL); // other options: CV_AUTOSIZE, CV_FREERATIO
            // cv::resizeWindow("Captured Image unDist", 900, 600); // Resize window to specific size
            // cv::imshow("Captured Image unDist", undistortedImage);
            // cv::waitKey(0);  // Wait for a key press

        }
        else
        {
            std::cerr << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
        }

        // Close the camera
        camera.Close();
    }
    catch (GenICam::GenericException &e)
    {
        // Error handling
        std::cerr << "An exception occurred: " << e.GetDescription() << std::endl;
    }

}


