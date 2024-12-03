#include "Trajectory.h"

//Constructor
Trajectory::Trajectory() {
    mTCPoffsetY = -0.10915;
    mStartHeight = 1.1;
    mThrowPose = { -1.0509, -0.7332 };
}

//SpeedJ funktioner
double Trajectory::getCartesianVelocity(std::vector<double> target) {

    double g = 9.8;

    std::vector<double> targetBase = corner2BaseTransformation(target);
    double x = targetBase[0];
    double y = targetBase[1];
    double z = targetBase[2];

    //udregner velocity
    double hyp = std::sqrt(x * x + y * y);
    double throwDistance = std::sqrt(x * x + y * y - mTCPoffsetY * mTCPoffsetY);
    double velocity = std::sqrt((-g * throwDistance * throwDistance) / (2 * z - 2 * mStartHeight));

    return velocity;
}

void Trajectory::buildQubicVelocityProfiles(std::vector<double> target, double rampUpTime, double velocityFactor) {

    mRampUpTime = rampUpTime;
    mVelocityFactor = velocityFactor;

    std::vector<double> jointVelocities = getTargetJointSpeeds(target);

    double shoulderJointVelocity = jointVelocities[0];
    double elbowJointVelocity = jointVelocities[1];

    double throwVelocityJoints = velocityFactor * std::sqrt(shoulderJointVelocity * shoulderJointVelocity + elbowJointVelocity * elbowJointVelocity);

    std::pair<double, double> punkt1 = std::make_pair(0.0, 0.0);
    std::pair<double, double> punkt2 = std::make_pair(rampUpTime, throwVelocityJoints);
    std::pair<double, double> punkt3 = std::make_pair(rampUpTime * 2, 0.0);

    std::vector<double> profile = parabel3punkter(punkt1, punkt2, punkt3);

    mQubicRampUpProfileA = profile[0];
    mQubicRampUpProfileB = profile[1];
    mQubicRampUpProfileC = 0;
    mQubicRampDownProfileA = profile[0];
    mQubicRampDownProfileB = 0;
    mQubicRampDownProfileC = throwVelocityJoints;
}

void Trajectory::buildLinearVelocityProfiles(std::vector<double> target, double rampUpTime, double velocityFactor) {

    mRampUpTime = rampUpTime;
    mVelocityFactor = velocityFactor;

    std::vector<double> jointVelocities = getTargetJointSpeeds(target);

    double shoulderJointVelocity = jointVelocities[0];
    double elbowJointVelocity = jointVelocities[1];

    double throwVelocityJoints = velocityFactor * std::sqrt(shoulderJointVelocity * shoulderJointVelocity + elbowJointVelocity * elbowJointVelocity);

    std::pair<double, double> punkt1 = std::make_pair(0.0, 0.0);
    std::pair<double, double> punkt2 = std::make_pair(rampUpTime, throwVelocityJoints);

    std::vector<double> profile = linear2punkter(punkt1, punkt2);

    mLinearRampUpProfileA = profile[0];
    mLinearRampUpProfileB = 0;
    mQubicRampDownProfileA = -profile[0];
    mQubicRampDownProfileB = rampUpTime;
}

std::vector<double> Trajectory::getQubicRampUpVelocity(double time, std::vector<double> unitVector, std::vector<double> targetJointVelocities) {

    if (time > mRampUpTime) {
        return { targetJointVelocities[0] * mVelocityFactor, targetJointVelocities[1] * mVelocityFactor };
    }

    double velocityMagnitude = mQubicRampUpProfileA * time * time + mQubicRampUpProfileB * time + mQubicRampUpProfileC;

    return { velocityMagnitude * unitVector[0], velocityMagnitude * unitVector[1] };
}

std::vector<double> Trajectory::getQubicRampDownVelocity(double time, std::vector<double> unitVector) {

    if (time > mRampUpTime) {
        return { 0,0 };
    }

    double velocityMagnitude = mQubicRampDownProfileA * time * time + mQubicRampDownProfileB * time + mQubicRampDownProfileC;

    return { velocityMagnitude * unitVector[0], velocityMagnitude * unitVector[1] };
}

std::vector<double> Trajectory::getLinearRampUpVelocity(double time, std::vector<double> unitVector, std::vector<double> targetJointVelocities) {
    
    if (time > mRampUpTime) {
        return { targetJointVelocities[0] * mVelocityFactor, targetJointVelocities[1] * mVelocityFactor };
    }

    double velocityMagnitude = mLinearRampUpProfileA * time + mLinearRampUpProfileB;

    return { velocityMagnitude * unitVector[0], velocityMagnitude * unitVector[1] };
}

std::vector<double> Trajectory::getLinearRampDownVelocity(double time, std::vector<double> unitVector) {
    if (time > mRampUpTime) {
        return { 0,0 };
    }

    double velocityMagnitude = mLinearRampDownProfileA * time + mLinearRampDownProfileB;

    return { velocityMagnitude * unitVector[0], velocityMagnitude * unitVector[1] };
}

std::vector<double> Trajectory::getUnitVector(std::vector<double> target) {

    std::vector<double> jointVelocities = getTargetJointSpeeds(target);

    double shoulderJointVelocity = jointVelocities[0];
    double elbowJointVelocity = jointVelocities[1];

    double shoulderUnitVelocity = shoulderJointVelocity / std::sqrt(shoulderJointVelocity * shoulderJointVelocity + elbowJointVelocity * elbowJointVelocity);
    double elbowUnitVelocity = elbowJointVelocity / std::sqrt(shoulderJointVelocity * shoulderJointVelocity + elbowJointVelocity * elbowJointVelocity);

    return { shoulderUnitVelocity, elbowUnitVelocity };
}

std::vector<double> Trajectory::getStartPose(std::vector<double> target, double deltaD) {

    mDeltaD = deltaD;

    double pi = 3.14159;

    //transformere fra cornerframe til robotbaseframe
    std::vector<double> targetBase = corner2BaseTransformation(target);
    double x = targetBase[0];
    double y = targetBase[1];
    double z = targetBase[2];

    //udregner base startpose
    double hyp = std::sqrt(x * x + y * y);
    double B_angle = std::acos(mTCPoffsetY / hyp);
    double v_angle = std::atan2(y, x) + pi / 2;
    double baseStartPose = -(pi - B_angle - v_angle);

    //udregner shoulder og elbow startpose
    std::vector<double> jointVelocities = getTargetJointSpeeds(target);

    double shoulderJointVelocity = jointVelocities[0];
    double elbowJointVelocity = jointVelocities[1];

    double shoulderUnitVelocity = shoulderJointVelocity / std::sqrt(shoulderJointVelocity * shoulderJointVelocity + elbowJointVelocity * elbowJointVelocity);
    double elbowUnitVelocity = elbowJointVelocity / std::sqrt(shoulderJointVelocity * shoulderJointVelocity + elbowJointVelocity * elbowJointVelocity);

    double shoulderStartPose = mThrowPose[0] - shoulderUnitVelocity * deltaD;
    double elbowStartPose = mThrowPose[1] - elbowUnitVelocity * deltaD;

    return { baseStartPose, shoulderStartPose, elbowStartPose };
}

std::vector<double> Trajectory::getTargetJointSpeeds(std::vector<double> target) {
    double cartesianVelocity = getCartesianVelocity(target);
    std::vector<double> jointVelocities = jacobianInverse2D(mThrowPose, { -cartesianVelocity,0 });
    return jointVelocities;
}

double Trajectory::getTimeUntilRelease(std::vector<double> jointPositions, std::vector<double> jointVelocities) {
    double targetShoulderPosition = mThrowPose[0];
    double targetElbowPosition = mThrowPose[1];
    double actualShoulderPosition = jointPositions[0];
    double actualElbowPosition = jointPositions[1];
    double shoulderVelocity = jointVelocities[0];
    double elbowVelocity = jointVelocities[1];

    double shoulderDistance = std::abs(targetShoulderPosition - actualShoulderPosition);
    double elbowDistance = std::abs(targetElbowPosition - actualElbowPosition);

    double timeUntilReleaseShoulder = shoulderDistance / shoulderVelocity;
    double timeUntilReleaseElbow = elbowDistance / elbowVelocity;

    double timeUntilReleaseAverage = (timeUntilReleaseShoulder + timeUntilReleaseElbow) / 2;

    return timeUntilReleaseAverage;
}

//Jacobians
std::vector<double> Trajectory::jacobian2D(std::vector<double> angles, std::vector<double> jointVelocities) {
    double q1 = angles[0];
    double q2 = angles[1];
    double qdot1 = jointVelocities[0];
    double qdot2 = jointVelocities[1];

    double l1 = 0.425;
    double l2 = 0.675811;

    double j11 = -l1 * std::sin(q1) - l2 * std::sin(q1 + q2);
    double j12 = -l2 * std::sin(q1 + q2);
    double j21 = l1 * std::cos(q1) + l2 * std::cos(q1 + q2);
    double j22 = l2 * std::cos(q1 + q2);

    double velocityX = j11 * qdot1 + j12 * qdot2;
    double velocityY = j21 * qdot1 + j22 * qdot2;

    std::cout << "x: " << velocityX << " y: " << velocityY << std::endl;

    return { velocityX, velocityY };
}

std::vector<double> Trajectory::jacobianInverse2D(std::vector<double> angles, std::vector<double> cartesianVelocity) {
    double q1 = angles[0];
    double q2 = angles[1];
    double vX = cartesianVelocity[0];
    double vY = cartesianVelocity[1];

    double l1 = 0.425;
    double l2 = 0.675811;

    double j11 = -l1 * std::sin(q1) - l2 * std::sin(q1 + q2);
    double j12 = -l2 * std::sin(q1 + q2);
    double j21 = l1 * std::cos(q1) + l2 * std::cos(q1 + q2);
    double j22 = l2 * std::cos(q1 + q2);

    double determinant = j11 * j22 - j12 * j21;

    double invj11 = j22 / determinant;
    double invj12 = -j12 / determinant;
    double invj21 = -j21 / determinant;
    double invj22 = j11 / determinant;

    double joint1Velocity = invj11 * vX + invj12 * vY;
    double joint2Velocity = invj21 * vX + invj22 * vY;

    std::cout << "joint 1: " << joint1Velocity << " joint 2: " << joint2Velocity << " determinant: " << determinant << std::endl;

    if (std::abs(determinant) < 0.0001) {
        std::cout << "error: at or close to singularity" << std::endl;
        return { 0,0 };
    }

    return { joint1Velocity, joint2Velocity };
}

//Rotations and transformations
double Trajectory::findAngle(std::vector<double> punkt) {
    double x = punkt[0];
    double y = punkt[1];

    double angle = std::atan(y / x);

    return angle;
}

std::vector<double> Trajectory::rotateZ(double angle, std::vector<double> punkt) {
    double x = punkt[0];
    double y = punkt[1];
    double z = punkt[2];

    std::vector<double> output;
    output.resize(3);

    output[0] = std::cos(angle) * x - std::sin(angle) * y;
    output[1] = std::sin(angle) * x + std::cos(angle) * y;
    output[2] = z;

    return output;
}

std::vector<double> Trajectory::corner2BaseTransformation(std::vector<double> punkt) {
    double x = punkt[0];
    double y = punkt[1];
    double z = punkt[2];

    std::vector<double> output;
    output.resize(3);

    output[0] = -0.923 * x - 0.3849 * y + 0.7031903;
    output[1] = -0.3849 * x + 0.923 * y - 0.7098926;
    output[2] = -z - 0.01068;

    return output;
}

std::vector<double> Trajectory::base2CornerTransformation(std::vector<double> punkt) {
    double x = punkt[0];
    double y = punkt[1];
    double z = punkt[2];

    std::vector<double> output;
    output.resize(3);

    output[0] = -0.923 * x - 0.3849 * y + 0.3757709;
    output[1] = -0.3849 * x + 0.923 * y + 0.9258619;
    output[2] = -z - 0.01068;

    return output;
}

//grafer
std::vector<double> Trajectory::parabel3punkter(std::pair<double, double> punkt1, std::pair<double, double> punkt2, std::pair<double, double> punkt3) {
    double x1 = punkt1.first;
    double x2 = punkt2.first;
    double x3 = punkt3.first;
    double z1 = punkt1.second;
    double z2 = punkt2.second;
    double z3 = punkt3.second;

    double naevner = (x1 - x2) * (x1 - x3) * (x2 - x3);

    double a = (x3 * (z2 - z1) + x2 * (z1 - z3) + x1 * (z3 - z2)) / naevner;
    double b = (x3 * x3 * (z1 - z2) + x2 * x2 * (z3 - z1) + x1 * x1 * (z2 - z3)) / naevner;
    double c = (x2 * x3 * (x2 - x3) * z1 + x3 * x1 * (x3 - x1) * z2 + x1 * x2 * (x1 - x2) * z3) / naevner;

    return{ a,b,c };
}

std::vector<double> Trajectory::linear2punkter(std::pair<double, double> punkt1, std::pair<double, double> punkt2) {
    double x1 = punkt1.first;
    double y1 = punkt1.second;
    double x2 = punkt2.first;
    double y2 = punkt2.second;

    double a = (y2 - y2) / (x2 - x1);
    double b = y1 - A * x1;

    return { a,b };
}