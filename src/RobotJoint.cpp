#include <RobotJoint.h>

RobotJoint::RobotJoint(MoveType p_j, double p_maxPos, double p_minPos)
{
	// Initialize joint
	// Set joint type
	this->m_jointType = p_j;
	// Set joint bounds
	Bounds lv_jointBound;
	lv_jointBound.max = p_maxPos;
	lv_jointBound.min = p_minPos;
	this->m_jointBounds = lv_jointBound;
	// Initialize joint coordinates
	this->m_jointPos = 0;
	this->m_jointVel = 0;
	this->m_jointAcc = 0;
	// Set targets to zero
	this->m_jointTargetType = NoneTarget;
	this->m_jointTargetPos = 0;
	this->m_jointTargetVel = 0;
	this->m_jointTargetForce = 0;

	// Kinematic mode is default, so set motor parameters to zero
	this->m_gearRatio = 0;
	this->m_motorPower = 0;
	this->m_motorSpeed = 0;
	this->m_motorSpeed_max = 0;
	this->m_motorTorque = 0;
	this->m_motorTorque_max = 0;
}

void RobotJoint::enableDynamicMode(double p_gearRatio, double p_motorPower, double p_motorSpeed, double p_motorSpeed_max, double p_motorTorque, double p_motorTorque_max)
{
	this->m_gearRatio = p_gearRatio;
	this->m_motorPower = p_motorPower;
	this->m_motorSpeed = p_motorSpeed;
	this->m_motorSpeed_max = p_motorSpeed_max;
	this->m_motorTorque = p_motorTorque;
	this->m_motorTorque_max = p_motorTorque_max;
}

double RobotJoint::getJointPos() const
{
	return this->m_jointPos;
}

double RobotJoint::getJointVel() const
{
	return this->m_jointVel;
}

double RobotJoint::getJointAcc() const
{
	return this->m_jointAcc;
}

double RobotJoint::getJointTarget() const
{
	switch (this->m_jointTargetType)
	{
	case PosTarget:
		return this->m_jointTargetPos;
	case VelTarget:
		return this->m_jointTargetVel;
	case ForceTarget:
		return this->m_jointTargetForce;
	default:
		return 0;
	}
}

DriveTarget RobotJoint::getJointTargetType() const
{
	return this->m_jointTargetType;
}

MoveType RobotJoint::getJointType() const
{
	return this->m_jointType;
}

void RobotJoint::setJointTarget(DriveTarget p_targetType, double p_targetValue)
{
	this->m_jointTargetType = p_targetType;
	switch (p_targetType)
	{
	case PosTarget:
		this->m_jointTargetPos = p_targetValue;
		break;
	case VelTarget:
		this->m_jointTargetVel = p_targetValue;
		break;
	case ForceTarget:
		this->m_jointTargetForce = p_targetValue;
		break;	
	default:
		break;
	}
}

void RobotJoint::setJointBounds(double p_max, double p_min)
{
	this->m_jointBounds.max = p_max;
	this->m_jointBounds.min = p_min;
}