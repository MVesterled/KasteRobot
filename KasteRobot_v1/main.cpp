#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pylon/PylonIncludes.h>

// standard libraries
#include <iostream>
#include <vector>
#include <math.h>
#include <QtCore/QCoreApplication>
#include <thread>
#include <chrono>

// project libraries
#include <opencv2/opencv.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <pylon/PylonIncludes.h>


//Own classes
#include "Camera.h"
#include "Gripper.h"

using namespace ur_rtde;

//Function for multiplying matrices - temp location
void mulMat(double mat1[4][4], double mat2[4][1], double result[4][1], int R1, int C1, int R2, int C2) {
    // Check if matrix dimensions are compatible for multiplication
    if (C1 != R2) {
        std::cout << "Matrix multiplication not possible. Incompatible dimensions." << std::endl;
        return;
    }

    // Matrix multiplication
    for (int i = 0; i < R1; i++) {
        for (int j = 0; j < C2; j++) {
            result[i][j] = 0;
            for (int k = 0; k < C1; k++) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}

int main(int argc, char* argv[])
{
 /*

    //Calibration of camera test that also prints transformation matrix
    Camera visionCam;
    visionCam.calibrateCamera();
    cv::Point2f pixelPoint(593, 519); //creates point to convert fra camera to table (Red ball)
    cv::Point2f TablePoint(0, 0);     //creates point to store real world coordinate
    TablePoint = visionCam.TransformPoint(pixelPoint);

    std::cout << "Homography matrix: " << visionCam.getHomoMat() << std::endl;

    std::cout << "The transformed real world coordinates for the red ball are: ("
              << TablePoint.x << ", "
              << TablePoint.y << ")" << std::endl;

    visionCam.visualizeCalib();
    std::cout << "Program started" << std::endl;

*/
  
    std::cout << "test" << std::endl;
    //multiplication test
    double invTransformation[4][4] = {
        {0.0057, -1.0, 0.0013, 436.6030},
        {-0.9999, -0.0057, 0.0102, 496.1043},
        {-0.0102, -0.0014, -0.9999, -603.7265},
        {0, 0, 0, 1}
    };
    double point[4][1] = {{299}, {737}, {-151-186}, {1}};
    double result[4][1] = {0};
    // Call the multiplication function
    mulMat(invTransformation, point, result, 4, 4, 4, 1);
    for (int i = 0; i < 4; i++) {
        std::cout << result[i][0] << std::endl;
    }
//////////////////////////////////////////////////////////////////////////////////////////////////

    QCoreApplication app(argc, argv); // Initialize Qt application
    Gripper gripper; // Create an instance of Gripper
    gripper.connectToServer("192.168.1.20", 1000); // Attempt to connect to the server

    // Check if the connection was successful
    if (!gripper.isConnected()) {
        std::cout << "Failed to connect to the server." << std::endl;
        return 1; // Exit if the connection was not successful
    }

    if(gripper.Home()){
        qDebug() << "Step 1 OK";
        if(gripper.Command("MOVE(20)"))
            qDebug() << "Step 2 OK";
    }
    else{
        qDebug() << "FAIL";
    }
    // Test, sending back and forward

    /*
    QTextStream in(stdin); // Create a QTextStream to read from standard input
    std::cout << "Connected to the server. Type your messages (type 'exit' to quit):" << std::endl;
    while (true) {
        QString input; // String to hold user input
        in >> input; // Read input from the user
        // Exit condition
        if (input == "exit") {
            std::cout << "Exiting..." << std::endl;
            break; // Exit the loop if the user types 'exit'
        }
        // Send the data to the server
        QByteArray dataToSend = input.toUtf8(); // Convert QString to QByteArray
        gripper.sendData(dataToSend); // Send data
    }
    gripper.disconnectFromServer(); // Disconnect before exiting
    return 0;
    */


    // Robot connection test that prints out live shoulder location

    // Simple example using the RTDE Receive Interface to get the joint positions of the robot
    // OBS TJEK IP


    // Simple example using the RTDE IO Interface to set a standard digital output.
    //RTDEControlInterface ControlInter("192.168.1.54");
    //ControlInter.moveL({-0.18,0.26,-0.277,2.2,2.52,0.0},);

    RTDEControlInterface rtde_control("192.168.1.54", 500.0, RTDEControlInterface::FLAG_USE_EXT_UR_CAP);
    rtde_control.moveL({-0.18,0.26,-0.277,2.2,2.52,0.0}, 0.5, 0.2);

    /*
    RTDEReceiveInterface rtde_receive("192.168.1.54");
    std::cout << "is connected: " << rtde_receive.isConnected() << std::endl;
    while(true) {
        std::vector<double> joint_positions = rtde_receive.getActualQ();
        //std::cout << "Base: " << joint_positions[0] * (180.0/3.141592653589793238463) << std::endl;
        std::cout << "Shoulder: " << joint_positions[1] * (180.0/3.141592653589793238463) << std::endl;
        //std::cout << "Elbow: " << joint_positions[2] * (180.0/3.141592653589793238463) << std::endl;
        //std::cout << "Wrist 1: " << joint_positions[3] * (180.0/3.141592653589793238463) << std::endl;
        //std::cout << "Wrist 2: " << joint_positions[4] * (180.0/3.141592653589793238463) << std::endl;
        //std::cout << "Wrist 3: " << joint_positions[5] * (180.0/3.141592653589793238463) << std::endl;
    }
    */



// ////////////////////////////////////////////////////////////////////////////////////////////////
    /*
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
            cv::remap(img, undistortedImage, visionCam.getMapX(), visionCam.getMapY(), cv::INTER_LINEAR);

            // Example: Transform a pixel point to real-world coordinates
            cv::Point2f pixelPointRed(592, 514);  // Red Ball
            cv::Point2f pixelPointGreen(796, 639);  // Green Ball

            cv::Point2f realWorldPointRed = visionCam.TransformPoint(pixelPointRed);
            cv::Point2f realWorldPointGreen = visionCam.TransformPoint(pixelPointGreen);

            std::cout << "Real-world coordinates red: (" << realWorldPointRed.x << ", " << realWorldPointRed.y << ")" << std::endl;
            std::cout << "Real-world coordinates green: (" << realWorldPointGreen.x << ", " << realWorldPointGreen.y << ")" << std::endl;

            std::cout << "Image captured successfully!" << std::endl;
            // Optional: Save the image to disk for verification
            cv::imwrite("/home/matmat1000/Documents/ballTest.jpg", undistortedImage);
            // Create an OpenCV display window
            cv::namedWindow( "Captured Image unDist", cv::WINDOW_NORMAL); // other options: CV_AUTOSIZE, CV_FREERATIO
            cv::resizeWindow("Captured Image unDist", 900, 600); // Resize window to specific size
            cv::imshow("Captured Image unDist", undistortedImage);
            cv::waitKey(0);  // Wait for a key press

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
        return 1;
    }
    */

// ////////////////////////////////////////////////////////////////////////////////////////////////

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
