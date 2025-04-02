#pragma once
#include <vector>
#include "RobotLink.h"
#include "RobotJoint.h"
#include "RobotAdditions.h"

class RobotArm
{
private:
	bool m_isInitialized = false;

	RobotLink m_LinkZero;
	std::vector<RobotLink> m_links;
	std::vector<RobotJoint> m_joints;
	std::vector<DirectPoint> m_DHPoints;
	std::vector<DHParams> m_LinkJointParams;

	Offset m_originPosition;

	void CalcDHPoints();
	void PointsToParams();
	Offset CalcLinkFullOffset(int p_index);

public:
	RobotArm() = default;
	RobotArm(Offset p_originPoint, Offset p_endLinkZero, Axes p_AxisLinkZero);

	void AddLink(Offset p_endLink, Axes p_AxisLink, MoveType p_jointType, double p_boundUpper, double p_boundLower);

	void initialize();

	Offset getOriginPosition();

	Eigen::VectorXd getJointAngles();

	void setJointAngles(Eigen::VectorXd q);

	DirectPoint ForwardKinematics(Eigen::VectorXd q);
	
	static DirectPoint ForwardKinematics_static(RobotArm* arm, Eigen::VectorXd q);

	Eigen::VectorXd solveIK_POS(DirectPoint needPoint, std::string method);

	Eigen::VectorXd solveIK_VEL(Eigen::VectorXd needVelocity);
};