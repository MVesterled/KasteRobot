// standard libraries
#include <iostream>
#include <vector>
#include <math.h>
#include <QtCore/QCoreApplication>
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
    output[3] = 0.634;
    output[4] = -3.067;
    output[5] = 0.026;
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
    visionCam.capturePicture();
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

    //Udregning til kørselsmønster / kast for robot
    Trajectory linaryThrow;
    //Point where ball is
    std::vector<double> target =  {600.0/1000.0, 200.0/1000.0, -0.1};
    //Findes koordinates in base fram from robot
    //std::vector<float> koordinates = linaryThrow.(target);
    //robot points
    //std::vector<double> from = {koordinates[0], koordinates[1], koordinates[2], 0.622, -3.065, 0.029};
    //std::vector<double> to = {koordinates[3], koordinates[4], koordinates[5], 0.622, -3.065, 0.029};

    //Udregning af bold pos
    std::vector<double> pickUp = cornerToFrame(ballPoint.x/1000, ballPoint.y/1000, -0.40);
    std::vector<double> pickUpDown = cornerToFrame(ballPoint.x/1000, ballPoint.y/1000, -0.155);


    // Robot connection and Setup parameters
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

    std::cout << "is connected: " << rtde_receive.isConnected() << std::endl;

    //rtde_control.moveJ({0, -M_PI/2, -M_PI/4, -M_PI/2, 0, 0}, 0.1, 0.1);
    //rtde_control.moveL(init_pose, 0.2, 0.2);

    //std::vector<double> target = rtde_receive.getActualTCPPose();
    std::cout << std::endl;
    for (int i = 0; i < target.size();  ++i){
        std::cout << target[i] <<  " ";
    }


    //Gripper connect
    QCoreApplication app(argc, argv); // Initialize Qt application
    Gripper gripper; // Create an instance of Gripper
    gripper.connectToServer("192.168.1.20", 1000); // Attempt to connect to the server

    // Check if the connection was successful
    if (!gripper.isConnected()) {
        std::cout << "Failed to connect to the server." << std::endl;
        return 1; // Exit if the connection was not successful
    }

    // Real time movement
    // Parameters
    // Declared futher up
    // double rtde_frequency = 125.0; // Hz
    // double dt = 1.0 / rtde_frequency; // 8ms
    double acceleration = 4;
    double throwTol = 0.04;
    double rampUpTime = 0.4;
    double deltaD = 0.5;
    double factor = 1.13;
    std::vector<double> throwPose= {-1.19394 , -1.0508627 , -0.7332036732 , -0.51487 , M_PI/2, 0};

    Trajectory speedJThrow;
                                       //X   Y      Z
    std::vector<double> tabelTarget = {0.50,0.30,-0.125};
    std::vector<double> currentPose = {0,0,0,0,0,0};

    speedJThrow.buildLinearVelocityProfiles(tabelTarget, rampUpTime, factor);

    std::vector<double> startPose = speedJThrow.getStartPose(tabelTarget, deltaD);

    // Home gripper
    gripper.Home();
    // Move to middle pose
    rtde_control.moveJ({-1.152 , -1.972 , -1.251 , -1.476 , M_PI/2, 0}, 0.6, 0.6);
    // Pickup ball
    rtde_control.moveL(pickUp, 0.2, 0.2);
    rtde_control.moveL(pickUpDown, 0.2, 0.2);
    gripper.Grip(5, 36);
    rtde_control.moveL(pickUp, 0.2, 0.2);

    // Move to middle pose
    rtde_control.moveJ({-1.152 , -1.972 , -1.251 , -1.476 , M_PI/2, 0}, 0.6, 0.6);
    // Move to lineup pose
    rtde_control.moveJ(throwPose,  0.3, 0.3);
    // Move to pull back
    rtde_control.moveJ({startPose[0], startPose[1], startPose[2], -0.51487, M_PI/2, 0}, 0.6, 0.6);

    rtde_control.speedStop();

    float realTime = 0;
    std::vector<double> joint_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> jointSpeedF = {0,0};
    std::vector<double> unitVector = speedJThrow.getUnitVector(tabelTarget);
    std::vector<double> targetJointSpeeds = speedJThrow.getTargetJointSpeeds(tabelTarget);


    // Open CSV log file in append mode
    std::ofstream logFile;
    //logFile.open("logAcc" + std::to_string(acceleration) + ".csv", std::ios::app);
    //logFile.open("logAccShoulder" + std::to_string(acceleration) + ".csv", std::ios::app);
    //logFile.open("logAccElbow" + std::to_string(acceleration) + ".csv", std::ios::app);

    /*
    // Log both axis
    if (logFile.tellp() == 0) {
        logFile << "ShoulderSpeed,ShoulderAim,ElbowSpeed,ElbowAim,Time,SpeedScaling" << std::endl;
    }
    // Log Shoulder axis
    if (logFile.tellp() == 0) {
        logFile << "ShoulderSpeed,ShoulderAim,Time,SpeedScaling" << std::endl;
    }
    // Log EElbow axis
    if (logFile.tellp() == 0) {
        logFile << "ElbowSpeed,ElbowAim,Time,SpeedScaling" << std::endl;
    }
    */

    // Rampup loop, first loop in series
    // Execute 125Hz control loop for 2 seconds, each cycle is ~8ms
    for (unsigned int i=0; i<140; i++)
    {
        steady_clock::time_point t_start = rtde_control.initPeriod();
        jointSpeedF = speedJThrow.getLinearRampUpVelocity(realTime, unitVector, targetJointSpeeds);

        if(jointSpeedF[0] > 0 || jointSpeedF[1] > 0)
            break;

        rtde_io.setSpeedSlider(1);
        rtde_control.speedJ(joint_speed, acceleration, dt);

        joint_speed[1] = jointSpeedF[0];
        joint_speed[2] = jointSpeedF[1];

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
        */

        std::cout << "---------------------" << std::endl;
        std::cout << "Actual Shoulder: " << rtde_receive.getActualQd()[1] << " Aiming for speed: " <<joint_speed[1] << std::endl;
        std::cout << "Actual Elbow: " << rtde_receive.getActualQd()[2] << " Aiming for speed: " <<joint_speed[2] << std::endl;
        std::cout << "Time: " << realTime << std::endl;
        std::cout << "Speed scaling: " << rtde_receive.getSpeedScaling() << std::endl;
        std::cout << "Speed fraction: " << rtde_receive.getTargetSpeedFraction() << std::endl;
        std::cout << "Speed scaling combined: " << rtde_receive.getSpeedScalingCombined() << std::endl;
        std::cout << "---------------------" << std::endl;


        if(rampUpTime < realTime){
            std::cout << "Rampup time reached, Coasting to pos!" << std::endl;
            break;
        }

        /*if(rtde_receive.getActualQd()[1] < -1.26 && rtde_receive.getActualQd()[2] < -0.59){
            //rtde_receive.getActualQd()[1] < -1.26 && rtde_receive.getActualQd()[2] < -0.59
            std::cout << "Speed reached, releaseing ball!" << std::endl;
            break;
        }*/


        realTime += 0.008;
        rtde_control.waitPeriod(t_start);
    }

    // Coasting loop, keep robot at speed until throwing position is inside tolerance
    for (unsigned int i=0; i<140; i++)
    {
        steady_clock::time_point t_start = rtde_control.initPeriod();

        jointSpeedF = speedJThrow.getLinearRampUpVelocity(realTime, unitVector, targetJointSpeeds);

        if(jointSpeedF[0] > 0 || jointSpeedF[1] > 0)
            break;

        rtde_io.setSpeedSlider(1);
        rtde_control.speedJ(joint_speed, acceleration, dt);

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
        jointSpeedF = speedJThrow.getLinearRampDownVelocity(realTime, unitVector);

        if(jointSpeedF[0] > 0 || jointSpeedF[1] > 0)
            break;

        rtde_control.speedJ(joint_speed, acceleration, dt);

        joint_speed[1] = jointSpeedF[0];
        joint_speed[2] = jointSpeedF[1];

        std::cout << "---------------------" << std::endl;
        std::cout << "Actual Shoulder: " << rtde_receive.getActualQd()[1] << " Aiming for speed: " <<joint_speed[1] << std::endl;
        std::cout << "Actual Elbow: " << rtde_receive.getActualQd()[2] << " Aiming for speed: " <<joint_speed[2] << std::endl;
        std::cout << "Time: " << realTime << std::endl;
        std::cout << "Speed scaling: " << rtde_receive.getSpeedScalingCombined() << std::endl;
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
    rtde_control.stopScript();

    /*
    for (unsigned int i=0; i<250; i++)
    {
        steady_clock::time_point t_start = rtde_control.initPeriod();
        auto t1 = steady_clock::now();
        rtde_control.speedJ(joint_speed, acceleration, dt);

        std::cout << "Actual: " << rtde_receive.getActualQd()[2] << " Aiming for speed: " <<joint_speed[2] << " Current Time: " << realTime <<std::endl;
        //jointSpeedF = speedJThrow.getRampUpVelocity(realTime, unitVector, targetJointSpeeds);

        if(realTime > rampUpTime){
            break;
        }
        else{
            joint_speed[1] = static_cast<double>(jointSpeedF[0]);
            joint_speed[2] = static_cast<double>(jointSpeedF[1]);
        }

        rtde_control.waitPeriod(t_start);

        auto t2 = steady_clock::now();
        auto duration = duration_cast<milliseconds>(t2 - t1).count();
        std::cout << "Tid brugt: " << duration << " ms\n";

        realTime += 0.008;
    }

    while((abs(currentPose[2]) + throwTol < abs(throwPose[2]))){
        currentPose = rtde_receive.getActualQ();
        //std::cout << "neek: " << currentPose[2] << std::endl;
    }

    std::thread releaseThread([&]() {
        //std::this_thread::sleep_for(std::chrono::milliseconds(185)); // Adjust delay to release mid-way
        gripper.Command("RELEASE(2, 420)");  // Trigger gripper release
    });

    realTime = 0;

    for (unsigned int i=0; i<130; i++)
    {
        steady_clock::time_point t_start = rtde_control.initPeriod();
        rtde_control.speedJ(joint_speed, acceleration, dt);

        jointSpeedF = speedJThrow.getRampDownVelocity(realTime, unitVector);

        if(realTime > rampUpTime)
            break;

        std::cout << "Actual: " << abs(rtde_receive.getActualQd()[2]) << " Aiming for speed: " <<joint_speed[2] << " Current Time: " << realTime <<std::endl;
        joint_speed[1] = static_cast<double>(jointSpeedF[0]);
        joint_speed[2] = static_cast<double>(jointSpeedF[1]);

        rtde_control.waitPeriod(t_start);
        realTime += 0.008;
    }
    rtde_control.speedStop();
    rtde_control.stopScript();
*/
    /* --------------- Forste speedJ OUTDATED!
    // Real time movement
    // Parameters
    double acceleration = 40;
    double throwTol = 0.04;
    std::vector<double> throwPose= {-1.19394 , -1.0508627 , -0.7332036732 , -0.51487 , M_PI/2, 0};
    std::vector<double> joint_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    Trajectory speedJThrow;
    std::vector<float> tabelTarget = {0.5,0.1,0};
    std::vector<double> currentPose = {0,0,0,0,0,0};

    float buildUpTime = speedJThrow.buildQubicVelocityProfiles(tabelTarget);

    std::vector<float> startPose = speedJThrow.getStartPose(tabelTarget);


    gripper.Home();
    rtde_control.moveL(pickUp, 0.2, 0.2);
    rtde_control.moveL(pickUpDown, 0.2, 0.2);
    gripper.Grip(5, 36);
    rtde_control.moveL(pickUp, 0.2, 0.2);
    rtde_control.moveJ(throwPose, 0.3, 0.3);
    rtde_control.moveJ({startPose[0], startPose[1], startPose[2], -0.51487, M_PI/2, 0}, 0.3);

    float realTime = 0;
    std::vector<float> jointSpeedF = {0,0};
    std::vector<float> unitVector = speedJThrow.getUnitJointVelocity(tabelTarget);

    // Execute 125Hz control loop for 2 seconds, each cycle is ~8ms
    for (unsigned int i=0; i<130; i++)
    {
        steady_clock::time_point t_start = rtde_control.initPeriod();
        rtde_control.speedJ(joint_speed, acceleration, dt);

        std::cout << "Actual: " << abs(rtde_receive.getActualQd()[2]) << " Aiming for speed: " <<joint_speed[2] << " Current Time: " << realTime <<std::endl;
        jointSpeedF = speedJThrow.getRampUpVelocity(realTime, unitVector);

        if(realTime > buildUpTime){
            break;
        }
        else{
            joint_speed[1] = static_cast<double>(jointSpeedF[0]);
            joint_speed[2] = static_cast<double>(jointSpeedF[1]);
        }

        rtde_control.waitPeriod(t_start);
        realTime += 0.008;
    }

    while((abs(currentPose[2]) + throwTol < abs(throwPose[2]))){
        currentPose = rtde_receive.getActualQ();
        std::cout << "neek: " << currentPose[2] << std::endl;
    }

    std::thread releaseThread([&]() {
        //std::this_thread::sleep_for(std::chrono::milliseconds(185)); // Adjust delay to release mid-way
        gripper.Command("RELEASE(2, 420)");  // Trigger gripper release
    });

    realTime = 0;

    for (unsigned int i=0; i<130; i++)
    {
        steady_clock::time_point t_start = rtde_control.initPeriod();
        rtde_control.speedJ(joint_speed, acceleration, dt);

        jointSpeedF = speedJThrow.getRampDownVelocity(realTime, unitVector);

        if(realTime > buildUpTime)
            break;

        std::cout << "Actual: " << abs(rtde_receive.getActualQd()[2]) << " Aiming for speed: " <<joint_speed[2] << " Current Time: " << realTime <<std::endl;
        joint_speed[1] = static_cast<double>(jointSpeedF[0]);
        joint_speed[2] = static_cast<double>(jointSpeedF[1]);

        rtde_control.waitPeriod(t_start);
        realTime += 0.008;
    }

    rtde_control.speedStop();
    rtde_control.stopScript();


/*
    // moveJ example, OUTDATED! ----------------------
    int z = 0;
    while (z < 1){

        gripper.Home();
        rtde_control.moveL(pickUp, 0.2, 0.2);
        rtde_control.moveL(pickUpDown, 0.2, 0.2);
        gripper.Grip(5, 36);
        rtde_control.moveL(pickUp, 0.2, 0.2);
        //rtde_control.moveJ({0 , -M_PI/2 , 0 , -0.51487 , M_PI/2, 0}, 0.1);
        rtde_control.moveJ({-1.19394 , -M_PI/2 , 0 , -0.51487 , M_PI/2, 0}, 0.1);
        rtde_control.moveJ({-1.19394 , -M_PI/2 , 0.7363 , -0.51487 , M_PI/2, 0}, 0.1);

        // Start moving to the "to" position
        std::thread releaseThread([&]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(195)); // Adjust delay to release mid-way
            gripper.Command("RELEASE(2, 420)");  // Trigger gripper release
        });

        // Begin motion to "to" position, allowing release to occur mid-motion
        rtde_control.moveJ({-1.19394 , -M_PI/2 , -0.7363 , -0.51487 , M_PI/2, 0}, 2.575+0.5, 40);

        // Wait for the release thread to complete
        releaseThread.join();
        z++;
    }

/*
    //Gripper connect MOVEL funktion! OUTDATED! -------------------------------------------
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
        //rtde_control.moveL(pickUp, 0.2, 0.2);
        //rtde_control.moveL(from, 0.3, 0.3);

        rtde_control.moveJ({0 , -M_PI/4 , 0 , 0.51487 , M_PI/4, 0});

        // Start moving to the "to" position
        std::thread releaseThread([&]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(185)); // Adjust delay to release mid-way
            gripper.Command("RELEASE(2, 420)");  // Trigger gripper release
        });

        // Begin motion to "to" position, allowing release to occur mid-motion
        //rtde_control.moveL(to, 3, 40);

        // Wait for the release thread to complete
        releaseThread.join();
        z++;
        }
        */
    return 0;
}
