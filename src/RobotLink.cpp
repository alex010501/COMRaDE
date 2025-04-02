#include <RobotLink.h>

RobotLink::RobotLink(Offset p_endLink, Axes p_AxisLink, Axes p_AxisLinkPrev)
{
	this->m_ExitPoint = p_endLink;
	this->m_ExitJointOrientation = p_AxisLink;
	this->m_PreviousJointOrientation = p_AxisLinkPrev;

	// Kinematic mode
	this->m_mass = 0;
	Offset lv_temp; //remake with dynamics adding
	lv_temp.x = 0;
	lv_temp.y = 0;
	lv_temp.z = 0;
	this->m_massPoint = lv_temp;
	this->m_inertia = 0;
}

RobotLink::RobotLink(const RobotLink &t)
{
	this->m_ExitPoint = t.getExitPoint();
	this->m_ExitJointOrientation = t.getExitJointOrientation();
	this->m_PreviousJointOrientation = t.getPreviousJointOrientation();
	this->m_mass = t.getMass();
	this->m_massPoint = t.getMassPoint();
	this->m_inertia = t.getInertia();
}

void RobotLink::enableDynamicMode(double p_mass, Offset p_massPoint, double p_inertia)
{
	this->m_mass = p_mass;
	this->m_massPoint = p_massPoint;
	this->m_inertia = p_inertia;
}

double RobotLink::getMass() const
{
	return this->m_mass;
}

Offset RobotLink::getMassPoint() const
{
	return this->m_massPoint;
}

double RobotLink::getInertia() const
{
	return this->m_inertia;
}

Axes RobotLink::getPreviousJointOrientation() const
{
	return this->m_PreviousJointOrientation;
}

Axes RobotLink::getExitJointOrientation() const
{
	return this->m_ExitJointOrientation;
}

Offset RobotLink::getExitPoint() const
{
	return this->m_ExitPoint;
}

void RobotLink::setMass(double p_mass)
{
	this->m_mass = p_mass;
}

void RobotLink::setMassPoint(Offset p_massPoint)
{
	this->m_massPoint = p_massPoint;
}

void RobotLink::setInertia(double p_inertia)
{
	this->m_inertia = p_inertia;
}

void RobotLink::setPreviousJointOrientation(Axes p_previousJointOrientation)
{
	this->m_PreviousJointOrientation = p_previousJointOrientation;
}

void RobotLink::setExitJointOrientation(Axes p_exitJointOrientation)
{
	this->m_ExitJointOrientation = p_exitJointOrientation;
}

void RobotLink::setExitPoint(Offset p_exitPoint)
{
	this->m_ExitPoint = p_exitPoint;
}