#pragma once

#include "RobotAdditions.h"

enum DriveTarget
{
	NoneTarget,
	PosTarget,
	VelTarget,
	ForceTarget
};

class RobotJoint //Class of joints and motors of Robot 
{
private:
	// Joint coordinates
	double m_jointPos;
	double m_jointVel;
	double m_jointAcc;

	// Target coordinates
	DriveTarget m_jointTargetType;
	double m_jointTargetPos;
	double m_jointTargetVel;
	double m_jointTargetForce;

	// Joint parameters
	MoveType m_jointType;
	Bounds m_jointBounds;

	// Motor parameters
	double m_gearRatio;
	double m_motorPower;
	double m_motorSpeed;
	double m_motorSpeed_max;
	double m_motorTorque;
	double m_motorTorque_max;	
public:
	// Class constructor
	RobotJoint() = default;
	RobotJoint(MoveType p_j, double p_maxPos = PI, double p_minPos = -PI);
	// Dynamic mode switch
	void enableDynamicMode(double p_gearRatio, double p_motorPower, double p_motorSpeed, double p_motorSpeed_max, double p_motorTorque, double p_motorTorque_max);

	// Getters of joint parameters
	double getJointPos() const;
	double getJointVel() const;
	double getJointAcc() const;
	double getJointTarget() const;
	DriveTarget getJointTargetType() const;
	MoveType getJointType() const;

	// Setter of joint target
	void setJointTarget(DriveTarget _targetType, double _targetValue);
	// Setter of joint bounds
	void setJointBounds(double p_minPos, double p_maxPos);
};