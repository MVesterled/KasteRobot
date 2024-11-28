#pragma once
#include <vector>
#include <iostream>
#include <cmath>

class Trajectory
{
public:
    //constructors
    Trajectory();
    Trajectory(std::pair<float, float> startPunkt, std::pair<float, float> topPunkt, float offset, float throwBuiltUp);

    //rotations and transformations
    float findAngle(std::vector<float> punkt);
    std::vector<float> rotateZ(float angle, std::vector<float> punkt);
    std::vector<float> corner2BaseTransformation(std::vector<float> punkt);
    std::vector<float> base2CornerTransformation(std::vector<float> punkt);

    //moveL
    std::vector<float> parabel3punkter(std::pair<float, float> punkt1, std::pair<float, float> punkt2, std::pair<float, float> punkt3);
    std::vector<float> getMoveLTrajectory(std::vector<float> target);

    //speedJ
    float getOverhandVelocity(float targetHeight, float targetDistance);
    std::vector<float> getSpeedJOverhand(std::vector<float> target);
    std::vector<float> jacobian2D(std::vector<float> angles, std::vector<float> jointVelocities);
    std::vector<float> jacobianInverse2D(std::vector<float> angles, std::vector<float> cartesianVelocity);
    void buildQubicVelocityProfiles(std::vector<float> target, float buildUpTime);
    std::vector<float> getRampUpVelocity(float time, std::vector<float> unitVector);
    std::vector<float> getRampDownVelocity(float time, std::vector<float> unitVector);
    std::vector<float> getUnitJointVelocity(std::vector<float> target);
    std::vector<float> getStartPose(std::vector<float> target, float deltaD);

private:
    std::pair<float, float> mStartPunkt;
    std::pair<float, float> mTopPunkt;
    std::vector<float> mThrowPose;
    float mOffset, mThrowBuildUp, mTCPoffsetY, mStartHeight;
    float mRampUpProfileA, mRampUpProfileB, mRampUpProfileC, mRampDownProfileA, mRampDownProfileB, mRampDownProfileC;
};