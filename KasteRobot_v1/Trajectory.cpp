#include "Trajectory.h"

Trajectory::Trajectory() {
    mThrowBuildUp = 0.2;
    mTCPoffsetY = -0.10915;
    mStartHeight = 1.1;
    mRampUpProfileA = 0;
    mRampUpProfileB = 0;
    mThrowPose = { -1.0509, -0.7332 };
}

float Trajectory::getCartesianVelocity(std::vector<float> target) {

    float g = 9.8;

    std::vector<float> targetBase = corner2BaseTransformation(target);
    float x = targetBase[0];
    float y = targetBase[1];
    float z = targetBase[2];

    //udregner velocity
    float hyp = std::sqrt(x * x + y * y);
    float throwDistance = std::sqrt(x * x + y * y - mTCPoffsetY * mTCPoffsetY);
    float velocity = std::sqrt((-g * throwDistance * throwDistance) / (2 * z - 2 * mStartHeight));

    return velocity;
}

void Trajectory::buildQubicVelocityProfiles(std::vector<float> target, float rampUpTime, float velocityFactor) {

    mRampUpTime = rampUpTime;

    std::vector<float> jointVelocities = getTargetJointSpeeds(target);

    float shoulderJointVelocity = jointVelocities[0];
    float elbowJointVelocity = jointVelocities[1];

    float throwVelocityJoints = velocityFactor * std::sqrt(shoulderJointVelocity * shoulderJointVelocity + elbowJointVelocity * elbowJointVelocity);

    std::pair<float, float> punkt1 = std::make_pair(0.0, 0.0);
    std::pair<float, float> punkt2 = std::make_pair(rampUpTime, throwVelocityJoints);
    std::pair<float, float> punkt3 = std::make_pair(rampUpTime * 2, 0.0);

    std::vector<float> profile = parabel3punkter(punkt1, punkt2, punkt3);

    mRampUpProfileA = profile[0];
    mRampUpProfileB = profile[1];
    mRampUpProfileC = 0;
    mRampDownProfileA = profile[0];
    mRampDownProfileB = 0;
    mRampDownProfileC = throwVelocityJoints;
}

std::vector<float> Trajectory::getRampUpVelocity(float time, std::vector<float> unitVector, std::vector<float> targetJointVelocities) {

    if (time > mRampUpTime) {
        return targetJointVelocities;
    }

    float velocityMagnitude = mRampUpProfileA * time * time + mRampUpProfileB * time + mRampUpProfileC;

    return { velocityMagnitude * unitVector[0], velocityMagnitude * unitVector[1] };
}

std::vector<float> Trajectory::getRampDownVelocity(float time, std::vector<float> unitVector) {

    if (time > mRampUpTime) {
        return { 0,0 };
    }

    float velocityMagnitude = mRampDownProfileA * time * time + mRampDownProfileB * time + mRampDownProfileC;

    return { velocityMagnitude * unitVector[0], velocityMagnitude * unitVector[1] };
}

std::vector<float> Trajectory::getUnitVector(std::vector<float> target) {

    std::vector<float> jointVelocities = getTargetJointSpeeds(target);

    float shoulderJointVelocity = jointVelocities[0];
    float elbowJointVelocity = jointVelocities[1];

    float shoulderUnitVelocity = shoulderJointVelocity / std::sqrt(shoulderJointVelocity * shoulderJointVelocity + elbowJointVelocity * elbowJointVelocity);
    float elbowUnitVelocity = elbowJointVelocity / std::sqrt(shoulderJointVelocity * shoulderJointVelocity + elbowJointVelocity * elbowJointVelocity);

    return { shoulderUnitVelocity, elbowUnitVelocity };
}

std::vector<float> Trajectory::getStartPose(std::vector<float> target, float deltaD) {

    mDeltaD = deltaD;

    float pi = 3.14159;

    //transformere fra cornerframe til robotbaseframe
    std::vector<float> targetBase = corner2BaseTransformation(target);
    float x = targetBase[0];
    float y = targetBase[1];
    float z = targetBase[2];

    //udregner base startpose
    float hyp = std::sqrt(x * x + y * y);
    float B_angle = std::acos(mTCPoffsetY / hyp);
    float v_angle = std::atan2(y, x) + pi / 2;
    float baseStartPose = -(pi - B_angle - v_angle);

    //udregner shoulder og elbow startpose
    std::vector<float> jointVelocities = getTargetJointSpeeds(target);

    float shoulderJointVelocity = jointVelocities[0];
    float elbowJointVelocity = jointVelocities[1];

    float shoulderUnitVelocity = shoulderJointVelocity / std::sqrt(shoulderJointVelocity * shoulderJointVelocity + elbowJointVelocity * elbowJointVelocity);
    float elbowUnitVelocity = elbowJointVelocity / std::sqrt(shoulderJointVelocity * shoulderJointVelocity + elbowJointVelocity * elbowJointVelocity);

    float shoulderStartPose = mThrowPose[0] - shoulderUnitVelocity * deltaD;
    float elbowStartPose = mThrowPose[1] - elbowUnitVelocity * deltaD;

    return { baseStartPose, shoulderStartPose, elbowStartPose };
}

std::vector<float> Trajectory::getTargetJointSpeeds(std::vector<float> target) {
    float cartesianVelocity = getCartesianVelocity(target);
    std::vector<float> jointVelocities = jacobianInverse2D(mThrowPose, { -cartesianVelocity,0 });
    return jointVelocities;
}





std::vector<float> Trajectory::jacobian2D(std::vector<float> angles, std::vector<float> jointVelocities) {
    float q1 = angles[0];
    float q2 = angles[1];
    float qdot1 = jointVelocities[0];
    float qdot2 = jointVelocities[1];

    float l1 = 0.425;
    float l2 = 0.675811;

    float j11 = -l1 * std::sin(q1) - l2 * std::sin(q1 + q2);
    float j12 = -l2 * std::sin(q1 + q2);
    float j21 = l1 * std::cos(q1) + l2 * std::cos(q1 + q2);
    float j22 = l2 * std::cos(q1 + q2);

    float velocityX = j11 * qdot1 + j12 * qdot2;
    float velocityY = j21 * qdot1 + j22 * qdot2;

    std::cout << "x: " << velocityX << " y: " << velocityY << std::endl;

    return { velocityX, velocityY };
}

std::vector<float> Trajectory::jacobianInverse2D(std::vector<float> angles, std::vector<float> cartesianVelocity) {
    float q1 = angles[0];
    float q2 = angles[1];
    float vX = cartesianVelocity[0];
    float vY = cartesianVelocity[1];

    float l1 = 0.425;
    float l2 = 0.675811;

    float j11 = -l1 * std::sin(q1) - l2 * std::sin(q1 + q2);
    float j12 = -l2 * std::sin(q1 + q2);
    float j21 = l1 * std::cos(q1) + l2 * std::cos(q1 + q2);
    float j22 = l2 * std::cos(q1 + q2);

    float determinant = j11 * j22 - j12 * j21;

    float invj11 = j22 / determinant;
    float invj12 = -j12 / determinant;
    float invj21 = -j21 / determinant;
    float invj22 = j11 / determinant;

    float joint1Velocity = invj11 * vX + invj12 * vY;
    float joint2Velocity = invj21 * vX + invj22 * vY;

    std::cout << "joint 1: " << joint1Velocity << " joint 2: " << joint2Velocity << " determinant: " << determinant << std::endl;

    if (std::abs(determinant) < 0.0001) {
        std::cout << "error: at or close to singularity" << std::endl;
        return { 0,0 };
    }

    return { joint1Velocity, joint2Velocity };
}

float Trajectory::findAngle(std::vector<float> punkt) {
    float x = punkt[0];
    float y = punkt[1];

    float angle = std::atan(y / x);

    return angle;
}

std::vector<float> Trajectory::rotateZ(float angle, std::vector<float> punkt) {
    float x = punkt[0];
    float y = punkt[1];
    float z = punkt[2];

    std::vector<float> output;
    output.resize(3);

    output[0] = std::cos(angle) * x - std::sin(angle) * y;
    output[1] = std::sin(angle) * x + std::cos(angle) * y;
    output[2] = z;

    return output;
}

std::vector<float> Trajectory::corner2BaseTransformation(std::vector<float> punkt) {
    float x = punkt[0];
    float y = punkt[1];
    float z = punkt[2];

    std::vector<float> output;
    output.resize(3);

    output[0] = -0.923 * x - 0.3849 * y + 0.7031903;
    output[1] = -0.3849 * x + 0.923 * y - 0.7098926;
    output[2] = -z - 0.01068;

    return output;
}

std::vector<float> Trajectory::base2CornerTransformation(std::vector<float> punkt) {
    float x = punkt[0];
    float y = punkt[1];
    float z = punkt[2];

    std::vector<float> output;
    output.resize(3);

    output[0] = -0.923 * x - 0.3849 * y + 0.3757709;
    output[1] = -0.3849 * x + 0.923 * y + 0.9258619;
    output[2] = -z - 0.01068;

    return output;
}

std::vector<float> Trajectory::parabel3punkter(std::pair<float, float> punkt1, std::pair<float, float> punkt2, std::pair<float, float> punkt3) {
    float x1 = punkt1.first;
    float x2 = punkt2.first;
    float x3 = punkt3.first;
    float z1 = punkt1.second;
    float z2 = punkt2.second;
    float z3 = punkt3.second;

    float naevner = (x1 - x2) * (x1 - x3) * (x2 - x3);

    float a = (x3 * (z2 - z1) + x2 * (z1 - z3) + x1 * (z3 - z2)) / naevner;
    float b = (x3 * x3 * (z1 - z2) + x2 * x2 * (z3 - z1) + x1 * x1 * (z2 - z3)) / naevner;
    float c = (x2 * x3 * (x2 - x3) * z1 + x3 * x1 * (x3 - x1) * z2 + x1 * x2 * (x1 - x2) * z3) / naevner;

    return{ a,b,c };
}
