// standard libraries
#include <iostream>
#include <vector>
#include <math.h>
#include <QtCore/QCoreApplication>
#include <QThread>
#include <thread>
#include <chrono>
#include <cmath>
#include <fstream>

// project libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pylon/PylonIncludes.h>
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

std::vector<double> cornerToFrame(double x, double y, double z){
    std::vector<double> output = {0, 0, 0, 0, 0, 0};
    output[0] = -0.923 * x - 0.3849 * y + 0.7031903;
    output[1] = -0.3849 * x + 0.923 * y - 0.7098926;
    output[2] = -z - 0.01068;
    output[3] = 0.634;
    output[4] = -3.067;
    output[5] = 0.026;
    return output;
}

int main(int argc, char* argv[])
{

    // ----------------------------------------- Robot connection and Setup parameters -----------------------------------------
    std::string robot_ip = "192.168.1.54";
    double rtde_frequency = 125.0; // Hz
    double dt = 1.0 / rtde_frequency; // 8ms
    uint16_t flags = RTDEControlInterface::FLAG_VERBOSE | RTDEControlInterface::FLAG_UPLOAD_SCRIPT;
    int ur_cap_port = 50002;

    // ur_rtde realtime priorities
    int rt_receive_priority = 90;
    int rt_control_priority = 85;

    RTDEControlInterface rtde_control(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority);
    RTDEReceiveInterface rtde_receive(robot_ip, rtde_frequency, {}, true, false, rt_receive_priority);
    RTDEIOInterface  rtde_io(robot_ip);

    // Set application realtime priority
    RTDEUtility::setRealtimePriority(80);

    std::cout << "Robot Connection: " << rtde_receive.isConnected() << std::endl;
    // ----------------------------------------- End -----------------------------------------

    // ----------------------------------------- Calibration of camera test that also prints transformation matrix -----------------------------------------
    //When running on a new PC - change path in camera class.cpp (line: 5, 16, 418, 437)
    Camera visionCam;
    //Calibrates camera
    visionCam.calibrateCamera();
    //Captures a picture and saves it
    visionCam.capturePicture();
    //Warps the taken image before locating object for more precision
    visionCam.transformPicture();
    //Finds colors in pictures and extracts to binary pictures
    visionCam.detectRed();
    visionCam.detectGreen();
    //Finds center of balls and cups in picture
    visionCam.ballDetect("red");
    visionCam.cupDetect("red");
    visionCam.ballDetect("green");
    visionCam.cupDetect("green");
    /*
    cv::Point2f ballPoint = visionCam.nextPoint();
    std::cout << "Ball green located at: " << ballPoint.x << ", " << ballPoint.y << std::endl;
    ballPoint = visionCam.nextPoint();
    std::cout << "Ball red located at: " << ballPoint.x << ", " << ballPoint.y << std::endl;
    cv::Point2f cupPoint = visionCam.nextCup();
    std::cout << "Cup green located at: " << cupPoint.x << ", " << cupPoint.y << std::endl;
    cupPoint = visionCam.nextCup();
    std::cout << "Cup red located at: " << cupPoint.x << ", " << cupPoint.y << std::endl;
*/
    //visionCam.centerOfMass();


    // ----------------------------------------- End -----------------------------------------

    // ----------------------------------------- Gripper connection and object handling  -----------------------------------------

    //Gripper connect
    QCoreApplication app(argc, argv); // Initialize Qt application
    Gripper gripper; // Create an instance of Gripper
    gripper.connectToServer("192.168.1.20", 1000); // Attempt to connect to the server

    // Check if the connection was successful
    if (!gripper.isConnected()) {
        std::cout << "Failed to connect to the server." << std::endl;
        return 1; // Exit if the connection was not successful
    }

    // ----------------------------------------- End  -----------------------------------------

    // ----------------------------------------- Parameters  -----------------------------------------

    // Set parameters for the Throw
    double acceleration = 4;
    double throwTol = 0.04;
    double rampUpTime = 0.4;
    double deltaD = 0.5;
    double factor = 1.13;
    double accSpdMoveJ = 0.5;
    double accSpdMoveL = 0.2;

    // Define cup height
    double cupHeight = -0.115;

    // Define static poses
    std::vector<double> throwPose = {-1.19394 , -1.0508627 , -0.7332036732 , -0.51487 , M_PI/2, 0};
    std::vector<double> middlePose = {-1.152 , -1.972 , -1.251 , -1.476 , M_PI/2, 0};

    // Define empty current pose
    std::vector<double> currentPose = {0,0,0,0,0,0};

    // Make object of Trajectory
    Trajectory speedJThrow;

    // ----------------------------------------- End  -----------------------------------------


    // ----------------------------------------- For loop for executing throwing -----------------------------------------

for (int j = 0; j < 2; ++j) {

    // Gets next point if any. If no balls left then -500, -500 is returned
    cv::Point2f ballPoint = visionCam.nextPoint();
    std::cout << "Ball is located at: " << ballPoint.x << ", " << ballPoint.y << std::endl;

    // Get the next cup
    cv::Point2f cupPoint = visionCam.nextCup();
    std::cout << "Cup is located at: " << cupPoint.x << ", " << cupPoint.y << std::endl;

    // Calc target from cup coordinates
    std::vector<double> tabelTarget = {(cupPoint.x-10)/1000, cupPoint.y/1000, cupHeight};

    // Calc points for pick up of next ball
    std::vector<double> pickUp = cornerToFrame(ballPoint.x/1000, ballPoint.y/1000, -0.40);
    std::vector<double> pickUpDown = cornerToFrame(ballPoint.x/1000, ballPoint.y/1000, -0.155);

    // Build velocity profiles (Up / Down)
    speedJThrow.buildLinearVelocityProfiles(tabelTarget, rampUpTime, factor);

    // Define startpose from target and deltaD
    std::vector<double> startPose = speedJThrow.getStartPose(tabelTarget, deltaD);

    // Home gripper
    gripper.Home();
    // Move to middle pose
    rtde_control.moveJ(middlePose, accSpdMoveJ, accSpdMoveJ);
    // Pickup ball
    rtde_control.moveL(pickUp, accSpdMoveL, accSpdMoveL);
    rtde_control.moveL(pickUpDown, accSpdMoveL, accSpdMoveL);
    gripper.Grip(5, 36);
    rtde_control.moveL(pickUp, accSpdMoveL, accSpdMoveL);

    // Move to middle pose
    rtde_control.moveJ(middlePose, accSpdMoveJ, accSpdMoveJ);
    // Move to lineup pose
    rtde_control.moveJ(throwPose, accSpdMoveJ, accSpdMoveJ);
    // Move to pull back
    rtde_control.moveJ({// First get direction to pull from target lineup
                        startPose[0], startPose[1], startPose[2],
                        // Use same pose for rest of axis'
                        throwPose[3], throwPose[4], throwPose[5]}, accSpdMoveJ, accSpdMoveJ);


    // Reset values for next throw
    float realTime = 0;
    std::vector<double> joint_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> jointSpeedF = {0,0};

    // Get the unitvector and jointspeeds for the target
    std::vector<double> unitVector = speedJThrow.getUnitVector(tabelTarget);
    std::vector<double> targetJointSpeeds = speedJThrow.getTargetJointSpeeds(tabelTarget);


    // Rampup loop, first loop in series
    // Execute 125Hz control loop for 2 seconds, each cycle is ~8ms
    for (unsigned int i=0; i<140; i++)
    {
        steady_clock::time_point t_start = rtde_control.initPeriod();

        //Get the current volcities for the current realtime
        jointSpeedF = speedJThrow.getLinearRampUpVelocity(realTime, unitVector, targetJointSpeeds);

        // Safety check, if speed are positive, robot is moving in wrong direction!
        if(jointSpeedF[0] > 0 || jointSpeedF[1] > 0)
            break;

        // Try to force speed slider to 1
        rtde_io.setSpeedSlider(1);

        // Send speedJ command
        rtde_control.speedJ(joint_speed, acceleration, dt);

        // Set joint speeds
        joint_speed[1] = jointSpeedF[0];
        joint_speed[2] = jointSpeedF[1];

        std::cout << "---------------------" << std::endl;
        std::cout << "Actual Shoulder: " << rtde_receive.getActualQd()[1] << " Aiming for speed: " <<joint_speed[1] << std::endl;
        std::cout << "Actual Elbow: " << rtde_receive.getActualQd()[2] << " Aiming for speed: " <<joint_speed[2] << std::endl;
        std::cout << "Time: " << realTime << std::endl;
        std::cout << "Speed scaling: " << rtde_receive.getSpeedScaling() << std::endl;
        std::cout << "---------------------" << std::endl;

        // When rampUpTime finished, exit loop
        if(rampUpTime < realTime){
            std::cout << "Rampup time reached, Coasting to pos!" << std::endl;
            break;
        }

        //  Add 8ms to realtime after loop
        realTime += 0.008;
        rtde_control.waitPeriod(t_start);
    }

    // Coasting loop, keep robot at speed until throwing position is inside tolerance
    for (unsigned int i=0; i<140; i++)
    {
        steady_clock::time_point t_start = rtde_control.initPeriod();

        //Get the current volcities for the current realtime (coasting, max speed is reached!)
        jointSpeedF = speedJThrow.getLinearRampUpVelocity(realTime, unitVector, targetJointSpeeds);

        // Safety check, if speed are positive, robot is moving in wrong direction!
        if(jointSpeedF[0] > 0 || jointSpeedF[1] > 0)
            break;

        // Try to force speed slider to 1
        rtde_io.setSpeedSlider(1);

        // Send speedJ command
        rtde_control.speedJ(joint_speed, acceleration, dt);

        // Set joint speeds
        joint_speed[1] = jointSpeedF[0];
        joint_speed[2] = jointSpeedF[1];

        currentPose = rtde_receive.getActualQ();
        std::cout << "---------------------" << std::endl;
        std::cout << "Shoulder Current Pos: " << currentPose[1] <<" Releasing at: " << throwPose[1] << std::endl;
        std::cout << "Elbow Current Pos: " << currentPose[2] <<" Releasing at: " << throwPose[2] << std::endl;
        std::cout << "Time: " << realTime << std::endl;
        std::cout << "---------------------" << std::endl;

        if((abs(currentPose[1]) + throwTol > abs(throwPose[1])) && (abs(currentPose[2]) + throwTol > abs(throwPose[2]))){
            std::cout << "Throwing pose reached, releasing ball!" << std::endl;
            break;
        }

        realTime += 0.008;
        rtde_control.waitPeriod(t_start);
    }

    std::thread releaseThread([&]() {
        //std::this_thread::sleep_for(std::chrono::milliseconds(185)); // Adjust delay to release mid-way
        gripper.Command("RELEASE(2, 420)");  // Trigger gripper release
    });

    // Reset realtime after coasting! Very important!
    realTime = 0;

    // Rampdown loop, slowly stop robot!
    for (unsigned int i=0; i<140; i++)
    {
        steady_clock::time_point t_start = rtde_control.initPeriod();

        // Robot should now rampdown speed, get down
        jointSpeedF = speedJThrow.getLinearRampDownVelocity(realTime, unitVector);

        // Safety check, if speed are positive, robot is moving in wrong direction!
        if(jointSpeedF[0] > 0 || jointSpeedF[1] > 0)
            break;

        rtde_control.speedJ(joint_speed, acceleration, dt);

        joint_speed[1] = jointSpeedF[0];
        joint_speed[2] = jointSpeedF[1];

        std::cout << "---------------------" << std::endl;
        std::cout << "Actual Shoulder: " << rtde_receive.getActualQd()[1] << " Aiming for speed: " <<joint_speed[1] << std::endl;
        std::cout << "Actual Elbow: " << rtde_receive.getActualQd()[2] << " Aiming for speed: " <<joint_speed[2] << std::endl;
        std::cout << "Time: " << realTime << std::endl;
        std::cout << "Speed scaling: " << rtde_receive.getSpeedScaling() << std::endl;
        std::cout << "---------------------" << std::endl;

        if(rampUpTime < realTime){
            std::cout << "Rampdown finished!" << std::endl;
            break;
        }
        rtde_control.waitPeriod(t_start);
        realTime += 0.008;
    }

    std::cout << "Stopping!" << std::endl;
    rtde_control.speedStop();
    releaseThread.join();


    }
    rtde_control.stopScript();

    // ----------------------------------------- End  -----------------------------------------



    // ----------------------------------------- Code for logging -----------------------------------------
    /*
    // Open CSV log file in append mode
    std::ofstream logFile;
    //logFile.open("logAcc" + std::to_string(acceleration) + ".csv", std::ios::app);
    //logFile.open("logAccShoulder" + std::to_string(acceleration) + ".csv", std::ios::app);
    //logFile.open("logAccElbow" + std::to_string(acceleration) + ".csv", std::ios::app);


    // Log both axis
    if (logFile.tellp() == 0) {
        logFile << "ShoulderSpeed,ShoulderAim,ElbowSpeed,ElbowAim,Time,SpeedScaling" << std::endl;
    }
    // Log Shoulder axis
    if (logFile.tellp() == 0) {
        logFile << "ShoulderSpeed,ShoulderAim,Time,SpeedScaling" << std::endl;
    }
    // Log Elbow axis
    if (logFile.tellp() == 0) {
        logFile << "ElbowSpeed,ElbowAim,Time,SpeedScaling" << std::endl;
    }
    */

    /*
        // Log data to CSV file
        double shoulder = rtde_receive.getActualQd()[1];
        double shoulderAim = joint_speed[1];
        double elbow = rtde_receive.getActualQd()[2];
        double elbowAim = joint_speed[2];
        double time = realTime;
        double speedScaling = rtde_receive.getSpeedScalingCombined();
        //logFile << shoulder << "," << shoulderAim << "," << elbow << "," << elbowAim << "," << time << "," << speedScaling << std::endl;
        //logFile << shoulder << "," << shoulderAim << "," << time << "," << speedScaling << std::endl;
        //logFile << elbow << "," << elbowAim << "," << time << "," << speedScaling << std::endl;
        */        /*
        // Log data to CSV file
        double shoulder = rtde_receive.getActualQd()[1];
        double shoulderAim = joint_speed[1];
        double elbow = rtde_receive.getActualQd()[2];
        double elbowAim = joint_speed[2];
        double time = realTime;
        double speedScaling = rtde_receive.getSpeedScalingCombined();
        //logFile << shoulder << "," << shoulderAim << "," << elbow << "," << elbowAim << "," << time << "," << speedScaling << std::endl;
        //logFile << shoulder << "," << shoulderAim << "," << time << "," << speedScaling << std::endl;
        //logFile << elbow << "," << elbowAim << "," << time << "," << speedScaling << std::endl;
        */

    // -------------------- End
    return 0;
}
