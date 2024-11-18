#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pylon/PylonIncludes.h>
#include <cmath>

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
#include "Trajectory.h"

using namespace ur_rtde;
using namespace std::chrono;

//Function for multiplying matrices - temp location
void mulMat(float mat1[4][4], float mat2[4][1], float result[4][1], int R1, int C1, int R2, int C2) {
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

std::vector<double> cornerToFrame(double x, double y, double z){
    std::vector<double> output = {0, 0, 0, 0, 0, 0};
    output[0] = -0.923 * x - 0.3849 * y + 0.7031903;
    output[1] = -0.3849 * x + 0.923 * y - 0.7098926;
    output[2] = -z - 0.01068;
    output[3] = 0.432;
    output[4] = -3.065;
    output[5] = 0.029;
    return output;
}

int main(int argc, char* argv[])
{

    //Calibration of camera test that also prints transformation matrix
    //When running on a new PC - change path in camera class.cpp (line: 5, 16, 418, 437)
    Camera visionCam;
    //Calibrates camera
    visionCam.calibrateCamera();
    //Captures a picture and saves it
    //visionCam.capturePicture();
    //Warps the taken image before locating object for more precision
    visionCam.transformPicture();
    //Finds center of balls in picture
    visionCam.detectGreen();
    visionCam.ballDetect();
    //visionCam.centerOfMass();
    //Gets next point if any. If no balls left then -500, -500 is returned
    cv::Point2f ballPoint = visionCam.nextPoint();
    std::cout << "Ball 1 is located at: " << ballPoint.x << ", " << ballPoint.y << std::endl;
    cv::Point2f ballPoint2 = visionCam.nextPoint();
    std::cout << "Ball 2 is located at: " << ballPoint2.x << ", " << ballPoint2.y << std::endl;
    cv::Point2f ballPoint3 = visionCam.nextPoint();
    std::cout << "Ball 3 is located at: " << ballPoint3.x << ", " << ballPoint3.y << std::endl << std::endl;
    /*
    //Udregning til kørselsmønster / kast for robot
    Trajectory linaryThrow;
    //Point where ball is
    std::vector<float> target =  {600.0/1000.0, 200.0/1000.0, -0.1};
    //Findes koordinates in base fram from robot
    std::vector<float> koordinates = linaryThrow.getTrajectory(target);
    //robot points
    std::vector<double> from = {koordinates[0], koordinates[1], koordinates[2], 0.622, -3.065, 0.029};
    std::vector<double> to = {koordinates[3], koordinates[4], koordinates[5], 0.622, -3.065, 0.029};

    //Udregning af bold pos
    std::vector<double> pickUp = cornerToFrame(ballPoint.x/1000, ballPoint.y/1000, -0.20);
    std::vector<double> pickUpDown = cornerToFrame(ballPoint.x/1000, ballPoint.y/1000, -0.155);


    // Robot connection and Setup parameters
    std::string robot_ip = "192.168.1.54";
    double rtde_frequency = 500.0; // Hz
    double dt = 1.0 / rtde_frequency; // 2ms
    uint16_t flags = RTDEControlInterface::FLAG_VERBOSE | RTDEControlInterface::FLAG_UPLOAD_SCRIPT;
    int ur_cap_port = 50002;

    // ur_rtde realtime priorities
    int rt_receive_priority = 90;
    int rt_control_priority = 85;

    RTDEControlInterface rtde_control(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority);
    RTDEReceiveInterface rtde_receive(robot_ip, rtde_frequency, {}, true, false, rt_receive_priority);

    // Set application realtime priority
    RTDEUtility::setRealtimePriority(80);

    std::cout << "is connected: " << rtde_receive.isConnected() << std::endl;

    //rtde_control.moveJ({0, -M_PI/2, -M_PI/4, -M_PI/2, 0, 0}, 0.1, 0.1);
    //rtde_control.moveL(init_pose, 0.2, 0.2);

    //std::vector<double> target = rtde_receive.getActualTCPPose();
    std::cout << std::endl;
    for (int i = 0; i < target.size();  ++i){
        std::cout << target[i] <<  " ";
    }
    //target[0] += 0.10;
    //rtde_control.moveL(target, 0.1, 0.1, true);
    //Gripper connect
    QCoreApplication app(argc, argv); // Initialize Qt application
    Gripper gripper; // Create an instance of Gripper
    gripper.connectToServer("192.168.1.20", 1000); // Attempt to connect to the server

    // Check if the connection was successful
    if (!gripper.isConnected()) {
        std::cout << "Failed to connect to the server." << std::endl;
        return 1; // Exit if the connection was not successful
    }

    int z = 0;
    while (z < 1){

        gripper.Home();
        rtde_control.moveL(pickUp, 0.2, 0.2);
        rtde_control.moveL(pickUpDown, 0.2, 0.2);
        gripper.Grip(5, 36);
        rtde_control.moveL(pickUp, 0.2, 0.2);
        rtde_control.moveL(from, 0.3, 0.3);

        // Start moving to the "to" position
        std::thread releaseThread([&]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(185)); // Adjust delay to release mid-way
            gripper.Command("RELEASE(2, 420)");  // Trigger gripper release
        });

        // Begin motion to "to" position, allowing release to occur mid-motion
        rtde_control.moveL(to, 3, 40);

        // Wait for the release thread to complete
        releaseThread.join();
        z++;

        }
*/





}
