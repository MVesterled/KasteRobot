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
    void buildQubicVelocityProfiles(std::vector<float> target, float buildUpTime, float velocityFactor = 1.0);
    std::vector<float> getStartPose(std::vector<float> target, float deltaD);
    std::vector<float> getUnitVector(std::vector<float> target);
    std::vector<float> getTargetJointSpeeds(std::vector<float> target);
    float getCartesianVelocity(std::vector<float> target);

    std::vector<float> getRampUpVelocity(float time, std::vector<float> unitVector, std::vector<float> targetJointVelocities);
    std::vector<float> getRampDownVelocity(float time, std::vector<float> unitVector);
       
    //rotations and transformations
    float findAngle(std::vector<float> punkt);
    std::vector<float> rotateZ(float angle, std::vector<float> punkt);
    std::vector<float> corner2BaseTransformation(std::vector<float> punkt);
    std::vector<float> base2CornerTransformation(std::vector<float> punkt);

    //parabel
    std::vector<float> parabel3punkter(std::pair<float, float> punkt1, std::pair<float, float> punkt2, std::pair<float, float> punkt3);

    //jacobians
    std::vector<float> jacobian2D(std::vector<float> angles, std::vector<float> jointVelocities);
    std::vector<float> jacobianInverse2D(std::vector<float> angles, std::vector<float> cartesianVelocity);
    
private:

    std::vector<float> mThrowPose;
    float mThrowBuildUp, mTCPoffsetY, mStartHeight, mDeltaD, mRampUpTime;
    float mRampUpProfileA, mRampUpProfileB, mRampUpProfileC, mRampDownProfileA, mRampDownProfileB, mRampDownProfileC;
};

