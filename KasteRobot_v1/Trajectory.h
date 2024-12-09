#pragma once
#include <vector>
#include <iostream>
#include <cmath>

class Trajectory
{
public:
    //constructors
    Trajectory();

    //speedJ
    void buildQubicVelocityProfiles(std::vector<double> target, double rampUpTime, double velocityFactor = 1.0);
    void buildLinearVelocityProfiles(std::vector<double> target, double rampUpTime, double velocityFactor = 1.0);
    std::vector<double> getStartPose(std::vector<double> target, double deltaD);
    std::vector<double> getUnitVector(std::vector<double> target);
    std::vector<double> getTargetJointSpeeds(std::vector<double> target);
    double getCartesianVelocity(std::vector<double> target);
    double getTimeUntilRelease(std::vector<double> jointPositions, std::vector<double> jointVelocities);

    std::vector<double> getQubicRampUpVelocity(double time, std::vector<double> unitVector, std::vector<double> targetJointVelocities);
    std::vector<double> getQubicRampDownVelocity(double time, std::vector<double> unitVector);
    std::vector<double> getLinearRampUpVelocity(double time, std::vector<double> unitVector, std::vector<double> targetJointVelocities);
    std::vector<double> getLinearRampDownVelocity(double time, std::vector<double> unitVector);

    //jacobians
    std::vector<double> jacobian2D(std::vector<double> angles, std::vector<double> jointVelocities);
    std::vector<double> jacobianInverse2D(std::vector<double> angles, std::vector<double> cartesianVelocity);

    //rotations and transformations
    double findAngle(std::vector<double> punkt);
    std::vector<double> rotateZ(double angle, std::vector<double> punkt);
    std::vector<double> corner2BaseTransformation(std::vector<double> punkt);
    std::vector<double> base2CornerTransformation(std::vector<double> punkt);

    //grafer
    std::vector<double> parabel3punkter(std::pair<double, double> punkt1, std::pair<double, double> punkt2, std::pair<double, double> punkt3);
    std::vector<double> linear2punkter(std::pair<double, double> punkt1, std::pair<double, double> punkt2);

private:

    std::vector<double> mThrowPose;
    double mTCPoffsetY, mStartHeight, mDeltaD, mRampUpTime, mVelocityFactor;
    double mQubicRampUpProfileA, mQubicRampUpProfileB, mQubicRampUpProfileC, mQubicRampDownProfileA, mQubicRampDownProfileB, mQubicRampDownProfileC;
    double mLinearRampUpProfileA, mLinearRampUpProfileB, mLinearRampDownProfileA, mLinearRampDownProfileB;
};
