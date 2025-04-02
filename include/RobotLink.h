#pragma once

#include "RobotAdditions.h"

class RobotLink //Class of links of Robot include math and visualization
{
private: //Private members
	// Dynamic parameters
	double m_mass;
	Offset m_massPoint;
	double m_inertia;

	// Kinematic parameters
	Axes m_PreviousJointOrientation;
	Axes m_ExitJointOrientation;
	Offset m_ExitPoint;
public:
    //Dynamic parameters
	double mass;
	Offset massPoint;
	double inertia;

    //Kinematic parameters
	Axes PreviousJointOrientation;
	Axes ExitJointOrientation;
	Offset ExitPoint;

	//Class constructor
	RobotLink() = default;
	RobotLink(const RobotLink &t);
	RobotLink(Offset p_endLink, Axes p_AxisLink, Axes p_AxisLinkPrev);
	// Dynamic mode switch
	void enableDynamicMode(double p_mass, Offset p_massPoint, double p_inertia);

	// Getters of link parameters
	double getMass() const;
	Offset getMassPoint() const;
	double getInertia() const;
	Axes getPreviousJointOrientation() const;
	Axes getExitJointOrientation() const;
	Offset getExitPoint() const;

	// Setters of link parameters
	void setMass(double p_mass);
	void setMassPoint(Offset p_massPoint);
	void setInertia(double p_inertia);
	void setPreviousJointOrientation(Axes p_previousJointOrientation);
	void setExitJointOrientation(Axes p_exitJointOrientation);
	void setExitPoint(Offset p_exitPoint);
};