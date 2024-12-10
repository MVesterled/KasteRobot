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

    //transformere target til baseframe
    std::vector<double> targetBase = corner2BaseTransformation(target);
    double x = targetBase[0];
    double y = targetBase[1];
    double z = targetBase[2];

    //udregner throwdistance og derefter velocity
    double hyp = std::sqrt(x * x + y * y);
    double throwDistance = std::sqrt(x * x + y * y - mTCPoffsetY * mTCPoffsetY);
    double velocity = std::sqrt((-g * throwDistance * throwDistance) / (2 * z - 2 * mStartHeight));

    return velocity;
}

void Trajectory::buildLinearVelocityProfiles(std::vector<double> target, double rampUpTime, double velocityFactor) {

    //sætter membervariabler
    mRampUpTime = rampUpTime;
    mVelocityFactor = velocityFactor;

    //sætter jointvelocities
    std::vector<double> jointVelocities = getTargetJointSpeeds(target);

    double shoulderJointVelocity = jointVelocities[0];
    double elbowJointVelocity = jointVelocities[1];

    //får magnituden på vores hastighed (i joint space)(her ganges faktoren på)
    double throwVelocityJoints = velocityFactor * std::sqrt(shoulderJointVelocity * shoulderJointVelocity + elbowJointVelocity * elbowJointVelocity);

    //laver punkter og får linjer
    std::pair<double, double> punkt1 = std::make_pair(0.0, 0.0);
    std::pair<double, double> punkt2 = std::make_pair(rampUpTime, throwVelocityJoints);

    std::vector<double> profile = linear2punkter(punkt1, punkt2);

    //gemmer linjer i membervariabler
    mLinearRampUpProfileA = profile[0];
    mLinearRampUpProfileB = 0;
    mLinearRampDownProfileA = -profile[0];
    mLinearRampDownProfileB = throwVelocityJoints;
}

std::vector<double> Trajectory::getLinearRampUpVelocity(double time, std::vector<double> unitVector, std::vector<double> targetJointVelocities) {

    //hvis vi er over rampuptime retuneres vores target velocity
    if (time > mRampUpTime) {
        return { targetJointVelocities[0] * mVelocityFactor, targetJointVelocities[1] * mVelocityFactor };
    }

    //ellers udregnes hastighedsmagnituden
    double velocityMagnitude = mLinearRampUpProfileA * time + mLinearRampUpProfileB;

    //de individuelle jointspeeds retuneres, ved at gange magnituden med en unitvektor
    return { velocityMagnitude * unitVector[0], velocityMagnitude * unitVector[1] };
}

std::vector<double> Trajectory::getLinearRampDownVelocity(double time, std::vector<double> unitVector) {
    
    //hvis vi er over rampuptime retuneres hastigheden 0
    if (time > mRampUpTime) {
        return { 0,0 };
    }

    //ellers udregnes hastighedsmagnituden
    double velocityMagnitude = mLinearRampDownProfileA * time + mLinearRampDownProfileB;

    //de individuelle jointspeeds retuneres, ved at gange magnituden med en unitvektor
    return { velocityMagnitude * unitVector[0], velocityMagnitude * unitVector[1] };
}

std::vector<double> Trajectory::getUnitVector(std::vector<double> target) {

    //sætter jointvelocities 
    std::vector<double> jointVelocities = getTargetJointSpeeds(target);

    double shoulderJointVelocity = jointVelocities[0];
    double elbowJointVelocity = jointVelocities[1];

    //vektoren af joint velocities normaliseres og retuneres
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

    //udregner base startpose med noget trigonometri
    double hyp = std::sqrt(x * x + y * y);
    double B_angle = std::acos(mTCPoffsetY / hyp);
    double v_angle = std::atan2(y, x) + pi / 2;
    double baseStartPose = -(pi - B_angle - v_angle);

    //sætter joint velocities
    std::vector<double> jointVelocities = getTargetJointSpeeds(target);

    double shoulderJointVelocity = jointVelocities[0];
    double elbowJointVelocity = jointVelocities[1];

    //vektoren af joint velocities normaliseres
    double shoulderUnitVelocity = shoulderJointVelocity / std::sqrt(shoulderJointVelocity * shoulderJointVelocity + elbowJointVelocity * elbowJointVelocity);
    double elbowUnitVelocity = elbowJointVelocity / std::sqrt(shoulderJointVelocity * shoulderJointVelocity + elbowJointVelocity * elbowJointVelocity);

    //udregner shoulder og elbow startpose ud fra unitvektoren og deltaD
    double shoulderStartPose = mThrowPose[0] - shoulderUnitVelocity * deltaD;
    double elbowStartPose = mThrowPose[1] - elbowUnitVelocity * deltaD;

    return { baseStartPose, shoulderStartPose, elbowStartPose };
}

std::vector<double> Trajectory::getTargetJointSpeeds(std::vector<double> target) {
    double cartesianVelocity = getCartesianVelocity(target);
    std::vector<double> jointVelocities = jacobianInverse2D(mThrowPose, { -cartesianVelocity,0 });
    return jointVelocities;
}

//Jacobians
std::vector<double> Trajectory::jacobian2D(std::vector<double> angles, std::vector<double> jointVelocities) {

    //variabler sættes
    double q1 = angles[0];
    double q2 = angles[1];
    double qdot1 = jointVelocities[0];
    double qdot2 = jointVelocities[1];

    double l1 = 0.425;
    double l2 = 0.675811;

    //de 4 elementer udregnes
    double j11 = -l1 * std::sin(q1) - l2 * std::sin(q1 + q2);
    double j12 = -l2 * std::sin(q1 + q2);
    double j21 = l1 * std::cos(q1) + l2 * std::cos(q1 + q2);
    double j22 = l2 * std::cos(q1 + q2);

    //joint hastigheder ganges på og cartesian hastigheder udregnes
    double velocityX = j11 * qdot1 + j12 * qdot2;
    double velocityY = j21 * qdot1 + j22 * qdot2;

    return { velocityX, velocityY };
}

std::vector<double> Trajectory::jacobianInverse2D(std::vector<double> angles, std::vector<double> cartesianVelocity) {

    //variabler sættes
    double q1 = angles[0];
    double q2 = angles[1];
    double vX = cartesianVelocity[0];
    double vY = cartesianVelocity[1];

    double l1 = 0.425;
    double l2 = 0.675811;

    //de 4 elementer udregnes
    double j11 = -l1 * std::sin(q1) - l2 * std::sin(q1 + q2);
    double j12 = -l2 * std::sin(q1 + q2);
    double j21 = l1 * std::cos(q1) + l2 * std::cos(q1 + q2);
    double j22 = l2 * std::cos(q1 + q2);

    //determinanten udregnes
    double determinant = j11 * j22 - j12 * j21;

    //jacobianten inverteres
    double invj11 = j22 / determinant;
    double invj12 = -j12 / determinant;
    double invj21 = -j21 / determinant;
    double invj22 = j11 / determinant;

    //cartesian hastigheder ganges på og joint hastigheder udregnes
    double joint1Velocity = invj11 * vX + invj12 * vY;
    double joint2Velocity = invj21 * vX + invj22 * vY;

    //printer hastigheder
    std::cout << "joint 1: " << joint1Velocity << " joint 2: " << joint2Velocity << " determinant: " << determinant << std::endl;

    //printer en besked og retunerer hastighed 0, hvis vi er tæt på en singularity
    if (std::abs(determinant) < 0.0001) {
        std::cout << "error: at or close to singularity" << std::endl;
        return { 0,0 };
    }

    return { joint1Velocity, joint2Velocity };
}

//Transformations
std::vector<double> Trajectory::corner2BaseTransformation(std::vector<double> punkt) {

    //sætter variabler
    double x = punkt[0];
    double y = punkt[1];
    double z = punkt[2];

    //laver output
    std::vector<double> output;
    output.resize(3);

    //transformere punktet
    output[0] = -0.923 * x - 0.3849 * y + 0.7031903;
    output[1] = -0.3849 * x + 0.923 * y - 0.7098926;
    output[2] = -z - 0.01068;

    return output;
}

std::vector<double> Trajectory::base2CornerTransformation(std::vector<double> punkt) {
    
    //sætter variabler
    double x = punkt[0];
    double y = punkt[1];
    double z = punkt[2];

    //laver output
    std::vector<double> output;
    output.resize(3);

    //transformere punktet
    output[0] = -0.923 * x - 0.3849 * y + 0.3757709;
    output[1] = -0.3849 * x + 0.923 * y + 0.9258619;
    output[2] = -z - 0.01068;

    return output;
}

//Other
std::vector<double> Trajectory::linear2punkter(std::pair<double, double> punkt1, std::pair<double, double> punkt2) {
    
    //sætter variabler
    double x1 = punkt1.first;
    double y1 = punkt1.second;
    double x2 = punkt2.first;
    double y2 = punkt2.second;

    //udregner a og b
    double a = (y2 - y1) / (x2 - x1);
    double b = y1 - a * x1;

    return { a,b };
}
