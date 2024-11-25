#include "Trajectory.h"

Trajectory::Trajectory() {
    mOffset = 0.35;
    mStartPunkt = std::make_pair(0.0, 0.3);
    mTopPunkt = std::make_pair(0.3 - mOffset / 2, 0.3);
    mThrowBuildUp = 0.2;
    mL = 0.1;
}

Trajectory::Trajectory(std::pair<float, float> startPunkt, std::pair<float, float> topPunkt, float offset, float throwBuildUp) {
    mStartPunkt = startPunkt;
    mTopPunkt = topPunkt;
    mOffset = offset;
    mThrowBuildUp = throwBuildUp;
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

std::vector<float> Trajectory::getMoveLTrajectory(std::vector<float> target) {

    //transformere fra cornerframe til robotbaseframe
    target = corner2BaseTransformation(target);
    float x = target[0];
    float y = target[1];
    float z = target[2];

    //roterer s? vores target er p? XZ planen og der kan arbejdes i 2D
    float targetAngle = findAngle(target);
    std::vector<float> target2D = rotateZ(-targetAngle, target);

    //tilf?jer et offset, s? robotten starter kastet 30 cm vaek fra sig og ikke lige over sig selv
    target2D[0] -= mOffset;

    //laver en parabel der beskriver kastet
    std::vector<float> parabel = parabel3punkter(mStartPunkt, mTopPunkt, std::make_pair(target2D[0], target2D[2]));
    float a = parabel[0];
    float b = parabel[1];
    float c = parabel[2];

    //opretter variable
    float timeToRelease;
    float g = 9.8;
    float throwAngle;
    float velocity;
    std::vector<float> throwStart;
    std::vector<float> throwRelease;
    std::vector<float> throwEnd;
    throwStart.resize(3);
    throwRelease.resize(3);
    throwEnd.resize(3);

    //udregner outputs (husker at tage offset med igen)
    throwAngle = std::atan(b);
    velocity = std::sqrt(g / (-2 * a * std::cos(throwAngle) * std::cos(throwAngle)));

    timeToRelease = mThrowBuildUp / velocity;

    throwRelease[0] = mOffset;
    throwRelease[1] = 0;
    throwRelease[2] = c;

    throwEnd[0] = throwRelease[0] + (std::cos(throwAngle) * mThrowBuildUp);
    throwEnd[1] = 0;
    throwEnd[2] = throwRelease[2] + (std::sin(throwAngle) * mThrowBuildUp) + 0.175;

    throwStart[0] = throwRelease[0] - (std::cos(throwAngle) * mThrowBuildUp);
    throwStart[1] = 0;
    throwStart[2] = throwRelease[2] - (std::sin(throwAngle) * mThrowBuildUp) + 0.175;

    //rotere tilbage s? vi igen har Y dimensionen med
    throwRelease = rotateZ(targetAngle, throwRelease);
    throwEnd = rotateZ(targetAngle, throwEnd);
    throwStart = rotateZ(targetAngle, throwStart);
    /*
    //transformere fra robotbaseframe til cornerframe
    throwRelease = base2CornerTransformation(throwRelease);
    throwEnd = base2CornerTransformation(throwEnd);
    throwStart = base2CornerTransformation(throwStart);
*/
//output
    std::cout << "All coordinates are in base frame. The throw should start in ( " << throwStart[0] << " , " << throwStart[1] << " , " << throwStart[2] << " ) and end in ( " << throwEnd[0] << " , " << throwEnd[1] << " , " << throwEnd[2] << " ), with a velocity of " << velocity << " m / s. the ball should be released in ( " << throwRelease[0] << " , " << throwRelease[1] << " , " << throwRelease[2] << " ) or " << timeToRelease << " s after the throw has started." << std::endl;
    std::cout << "the angle the gripper should be is" << targetAngle << std::endl;
    return { throwStart[0], throwStart[1], throwStart[2], throwEnd[0], throwEnd[1], throwEnd[2], throwRelease[0], throwRelease[1], throwRelease[2], velocity, timeToRelease, targetAngle };
}

float Trajectory::getOverhandVelocity(float targetHeight, float targetDistance) {
    float g = 9.8;
    float x = targetDistance;
    float z = targetHeight;
    float velocity = std::sqrt((-g * x * x) / (2 * z - 2 * 1.18997));
    return velocity;
}

std::vector<float> Trajectory::getSpeedJOverhand(std::vector<float> target) {
    float pi = 3.14159;
    //transformere fra cornerframe til robotbaseframe
    target = corner2BaseTransformation(target);
    float x = target[0];
    float y = target[1];
    float z = target[2];
    //udregner base vinkel og velocity
    float hyp = std::sqrt(x * x + y * y);
    float B_angle = std::acos(mL / hyp);
    float throwDistance = std::sqrt(x * x + y * y - mL * mL);
    float velocity = getOverhandVelocity(z, throwDistance);
    float v_angle = std::atan2(y, x) + pi / 2;
    float baseAngle = pi - B_angle - v_angle;
    //output
    std::cout << "the base should be rotated " << baseAngle << " radians and the throw should have a start velocity of " << velocity << " m/s" << std::endl;
    return {};
}

std::vector<float> Trajectory::jacobian2D(std::vector<float> angles, std::vector<float> jointVelocities) {
    float q1 = angles[0];
    float q2 = angles[1];
    float qdot1 = jointVelocities[0];
    float qdot2 = jointVelocities[1];

    float l1 = 0.425;
    float l2 = 1.18997 - l1 - 0.089159;

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
    float l2 = 1.18997 - l1 - 0.089159;

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