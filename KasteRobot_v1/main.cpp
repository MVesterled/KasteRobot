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

std::vector<double> getCircleTarget(const std::vector<double> &pose, double timestep, double radius=0.075, double freq=1.0)
{
    std::vector<double> circ_target = pose;
    circ_target[0] = pose[0] + radius * cos((2 * M_PI * freq * timestep));
    circ_target[1] = pose[1] + radius * sin((2 * M_PI * freq * timestep));
    return circ_target;
}

int main(int argc, char* argv[])
{
    Trajectory test;
    std::vector<float> punkt = test.corner2BaseTransformation({0.125, 0.075, -0.1});
    punkt = test.rotateZ(M_PI/8, punkt);
    std::cout << punkt[1] << " ," << -punkt[0] << " , " << punkt[2] << std::endl;

    std::vector<float> trac = test.getTrajectory({0.1, 0.1, 0.1});
    float from[4][1] = {{trac[0]*1000}, {trac[1]*1000}, {trac[2]*1000}, {1}};
    float to[4][1] = {{trac[3]*1000}, {trac[4]*1000}, {trac[5]*1000}, {1}};

    //Transformation (Fætter og Spade)
    //multiplication test
    float invTransformation[4][4] = {
        {0.0057, -1.0, 0.0013, 436.6030},
        {-0.9999, -0.0057, 0.0102, 496.1043},
        {-0.0102, -0.0014, -0.9999, -603.7265},
        {0, 0, 0, 1}
    };
    float fromBase[4][1] = {0};
    float toBase[4][1] = {0};
    // Call the multiplication function
    std::cout << std::endl;
    mulMat(invTransformation, from, fromBase, 4, 4, 4, 1);
    mulMat(invTransformation, to, toBase, 4, 4, 4, 1);
    for (int i = 0; i < 4; i++) {
        std::cout << fromBase[i][0] << ", ";
    }
    std::cout << std::endl;
    for (int i = 0; i < 4; i++) {
        std::cout << toBase[i][0] << ", ";
    }


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

 /*
    std::cout << "Program started" << std::endl << std::endl << std::endl;
    visionCam.ballDetect();

    cv::Point2f ballPoint = visionCam.nextPoint();
    std::cout << "Ball 1 is located at: " << ballPoint.x << ", " << ballPoint.y << std::endl;
    cv::Point2f ballPoint2 = visionCam.nextPoint();
    std::cout << "Ball 2 is located at: " << ballPoint2.x << ", " << ballPoint2.y << std::endl;
    cv::Point2f ballPoint3 = visionCam.nextPoint();
    std::cout << "Ball 3 is located at: " << ballPoint3.x << ", " << ballPoint3.y << std::endl;
 */
    //visionCam.capturePicture()

/*
    QCoreApplication app(argc, argv); // Initialize Qt application
    Gripper gripper; // Create an instance of Gripper
    gripper.connectToServer("192.168.1.20", 1000); // Attempt to connect to the server

    // Check if the connection was successful
    if (!gripper.isConnected()) {
        std::cout << "Failed to connect to the server." << std::endl;
        return 1; // Exit if the connection was not successful
    }

*/

    // Setup parameters
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

    double time_counter = 0.0;

    // Move to init position using moveL
    std::vector<double> actual_tcp_pose = rtde_receive.getActualTCPPose();
    std::vector<double> init_pose = getCircleTarget(actual_tcp_pose, time_counter);
    //rtde_control.moveJ({0, -M_PI/2, -M_PI/4, -M_PI/2, 0, 0}, 0.1, 0.1);
    //rtde_control.moveL(init_pose, 0.2, 0.2);

    std::vector<double> target = rtde_receive.getActualTCPPose();
    std::cout << std::endl;
    for (int i = 0; i < target.size();  ++i){
        std::cout << target[i] <<  " ";
    }
    target[0] += 0.10;
    //rtde_control.moveL(target, 0.1, 0.1, true);
/*
    while (true){
            rtde_control.moveL({0.076, -0.5,  0.423, 0.495, -3.09, 0.043},  0.1, 0.1);
            rtde_control.moveL({0.076, -0.5,  0.323, 0.495, -3.09, 0.043},  0.1, 0.1);
        }
*/
    /*
    while (true)
    rtde_control.moveL({0.076, -0.5,  0.423, 0.495, -3.09, 0.043},  0.1, 0.1);
    std::vector<double> joint_positions = rtde_receive.getActualQ();
    std::cout << "Shoulder: " << joint_positions[1] * (180.0/3.141592653589793238463) << std::endl;
    rtde_control.moveL({0.13, -0.525, 0.011, 0.37, 3.11, 0.123}, 0.1, 0.1);
    }
*/

}
