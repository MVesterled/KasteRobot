#pragma once
#include <vector>
#include <iostream>

class Trajectory
{
public:
	Trajectory();
	Trajectory(std::pair<float, float> startPunkt, std::pair<float, float> topPunkt, float offset, float throwBuiltUp);

	float findAngle(std::vector<float> punkt);
	std::vector<float> rotateZ(float angle, std::vector<float> punkt);
	std::vector<float> corner2BaseTransformation(std::vector<float> punkt);
	std::vector<float> base2CornerTransformation(std::vector<float> punkt);

	std::vector<float> parabel3punkter(std::pair<float, float> punkt1, std::pair<float, float> punkt2, std::pair<float, float> punkt3);
	std::vector<float> getTrajectory(std::vector<float> target);

private:
	std::pair<float, float> mStartPunkt;
	std::pair<float, float> mTopPunkt;
	float mOffset, mThrowBuildUp;
};

