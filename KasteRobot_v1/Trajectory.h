#pragma once
#include <vector>
#include <iostream>
#include <cmath>

class Trajectory
{
public:
    //Constructor
    Trajectory();

    //SpeedJ
    void buildLinearVelocityProfiles(std::vector<double> target, double rampUpTime, double velocityFactor = 1.0);
    std::vector<double> getStartPose(std::vector<double> target, double deltaD);
    std::vector<double> getUnitVector(std::vector<double> target);
    std::vector<double> getTargetJointSpeeds(std::vector<double> target);
    double getCartesianVelocity(std::vector<double> target);

    std::vector<double> getLinearRampUpVelocity(double time, std::vector<double> unitVector, std::vector<double> targetJointVelocities);
    std::vector<double> getLinearRampDownVelocity(double time, std::vector<double> unitVector);

    //Jacobians
    std::vector<double> jacobian2D(std::vector<double> angles, std::vector<double> jointVelocities);
    std::vector<double> jacobianInverse2D(std::vector<double> angles, std::vector<double> cartesianVelocity);

    //Transformations
    std::vector<double> corner2BaseTransformation(std::vector<double> punkt);
    std::vector<double> base2CornerTransformation(std::vector<double> punkt);

    //Other
    std::vector<double> linear2punkter(std::pair<double, double> punkt1, std::pair<double, double> punkt2);

private:

    std::vector<double> mThrowPose;
    double mTCPoffsetY, mStartHeight, mDeltaD, mRampUpTime, mVelocityFactor;
    double mLinearRampUpProfileA, mLinearRampUpProfileB, mLinearRampDownProfileA, mLinearRampDownProfileB;
};