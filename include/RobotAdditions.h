#pragma once

#include "MathAdditions.h"

#define PI 3.141592653589793

using DirectPoint = Eigen::Matrix4d; //definition of point as matrix 4x4

template <typename X>
using forwardFunc = X (*)(Eigen::VectorXd x);

enum Axes //Enumeration for Axis direction (Z - UP)
{
	NoneAxis,
	X_Axis,
	Y_Axis,
	Z_Axis
};

enum MoveType //Enumeration for joint movement
{
	NoneJoint,
	RotJoint,
	DispJoint
};

struct Offset //Structure for point without orientation
{
	double x, y, z;
};

struct DHParams //D-H params structure for robot arm calculation
{
	double alpha;
	double a;
	double theta;
	double d;
};

struct Bounds
{
	double min;
	double max;	
};

namespace RobotAdditions
{
	Eigen::Matrix4d CalcTransposeMatrix(DHParams params, double jointValue, MoveType jointType);

	DHParams CalcDHParams(DirectPoint point_o, DirectPoint point_e);

	Eigen::VectorXd errorRobotPoses(DirectPoint needPoint, DirectPoint currentPoint);

	Eigen::MatrixXd calcRobotJacobian(DirectPoint (*forwFunc)(Eigen::VectorXd q), Eigen::VectorXd q_init, double eps = 1e-6);

	double costRobotFunction(DirectPoint (*forwFunc)(Eigen::VectorXd q), Eigen::VectorXd q, DirectPoint target);

	Eigen::VectorXd gradientCostRobotFunction(DirectPoint (*forwFunc)(Eigen::VectorXd q), Eigen::VectorXd q_init, DirectPoint target);
}