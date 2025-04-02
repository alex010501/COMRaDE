#include <iostream>
#include <RobotArmLib.h>

RobotArm::RobotArm(Offset p_originPoint, Offset p_endLinkZero, Axes p_AxisLinkZero)
{
	this->m_originPosition = p_originPoint;
	RobotLink lv_NewLink(p_endLinkZero, p_AxisLinkZero, NoneAxis);
	this->m_LinkZero = lv_NewLink;	
}

/**
 * Adds a new link to the RobotArm.
 *
 * @param endLink the offset of the end of the new link
 * @param AxisLink the axes of the new link
 * @param jointType the type of joint for the new link
 *
 * @throws std::invalid_argument if the offset is zero
 */
void RobotArm::AddLink(Offset p_endLink, Axes p_AxisLink, MoveType p_jointType, double p_boundUpper, double p_boundLower)
{
	if (!((p_endLink.x)||(p_endLink.y)||(p_endLink.z)))
	{
		throw std::invalid_argument("Offset must not be zero");
	}

	// Add new joint
	RobotJoint lv_NewJoint(p_jointType, p_boundUpper, p_boundLower);
	this->m_joints.push_back(lv_NewJoint);

	// Add new link
	Axes lv_prevAxis;
	if (this->m_links.size() > 0)
		lv_prevAxis = this->m_links.back().ExitJointOrientation;
	else
		lv_prevAxis = this->m_LinkZero.ExitJointOrientation;
	RobotLink lv_NewLink(p_endLink, p_AxisLink, lv_prevAxis);
	this->m_links.push_back(lv_NewLink);
	this->m_isInitialized = false;
}

/**
 * Calculates the full offset of a robot arm link based on the given index
 *
 * @param index The index of the link
 *
 * @return The calculated offset of the link
 */
Offset RobotArm::CalcLinkFullOffset(int p_index)
{
	Offset offset;
	if (p_index == 0)
	{
		offset.x = this->m_LinkZero.ExitPoint.x;
		offset.y = this->m_LinkZero.ExitPoint.y;
		offset.z = this->m_LinkZero.ExitPoint.z;
		return offset;
	}
	else
	{
		offset.x = this->m_links[p_index].ExitPoint.x + this->CalcLinkFullOffset(p_index - 1).x;
		offset.y = this->m_links[p_index].ExitPoint.y + this->CalcLinkFullOffset(p_index - 1).y;
		offset.z = this->m_links[p_index].ExitPoint.z + this->CalcLinkFullOffset(p_index - 1).z;
		return offset;
	}
}

/**
 * Calculate the DH points for the robot arm using links info
 */
void RobotArm::CalcDHPoints()
{
	DirectPoint NewPoint;
	switch (this->m_LinkZero.ExitJointOrientation)
	{
	case X_Axis:
		NewPoint << 0, 0, 1, 0,
					1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 0, 1;
		break;
	case Y_Axis:
		NewPoint << 0, 1, 0, 0,
					0, 0, 1, 0,
					1, 0, 0, 0,
					0, 0, 0, 1;
		break;
	case Z_Axis:
		NewPoint << 1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1;
		break;
	default:
		break;
	}
	this->m_DHPoints.push_back(NewPoint);

	int count = this->m_links.size();
	for (int i = 0; i < count; i++)
	{
		switch (this->m_links[i].ExitJointOrientation)
		{
		case X_Axis:
			NewPoint << 0, 0, 1, 0,
						1, 0, 0, 0,
						0, 1, 0, 0,
						0, 0, 0, 1;
			break;
		case Y_Axis:
			NewPoint << 0, 1, 0, 0,
						0, 0, 1, 0,
						1, 0, 0, 0,
						0, 0, 0, 1;
			break;
		case Z_Axis:
			NewPoint << 1, 0, 0, 0,
						0, 1, 0, 0,
						0, 0, 1, 0,
						0, 0, 0, 1;
			break;		
		default:
			break;
		}		
		this->m_DHPoints.push_back(NewPoint);	
	}

	for(int i = count; i > 0; i--)
	{
		RobotLink link = this->m_links[i];
		RobotLink prevLink = this->m_links[i - 1];
		double x = link.ExitPoint.x;
		double y = link.ExitPoint.y;
		double z = link.ExitPoint.z;
		Eigen::Vector2d v;
		DirectPoint NewPoint;
		if (link.ExitJointOrientation == prevLink.ExitJointOrientation) // case of parallel axis
		{
			switch (link.ExitJointOrientation)
			{
			case X_Axis:
				v << y, z;
				v.normalize();
				NewPoint <<    0,     0, 1, this->CalcLinkFullOffset(i).x,
					   		v[0], -v[1], 0,	this->CalcLinkFullOffset(i).y,
					   		v[1],  v[0], 0,	this->CalcLinkFullOffset(i).z,
					   		   0,     0, 0,	1;
				this->m_DHPoints[i] = NewPoint;
				break;
			case Y_Axis:
				v << x, z;
				v.normalize();				
				NewPoint << v[0], -v[1], 0, this->CalcLinkFullOffset(i).x,
					   		   0,     0, 1,	this->CalcLinkFullOffset(i).y,
					   		v[1],  v[0], 0,	this->CalcLinkFullOffset(i).z,
					   		   0,     0, 0,	1;
				this->m_DHPoints[i] = NewPoint;
				break;
			case Z_Axis:
				v << x, y;
				v.normalize();
				NewPoint << v[0], -v[1], 0, this->CalcLinkFullOffset(i).x,
					   		v[1],  v[0], 0,	this->CalcLinkFullOffset(i).y,
					   		   0,     0, 1,	this->CalcLinkFullOffset(i).z,
					   		   0,     0, 0,	1;
				this->m_DHPoints[i] = NewPoint;
				break;
			default:
				break;
			}
		}
		// cases of nonparallel axis
		else if ((link.ExitJointOrientation == X_Axis) && (prevLink.ExitJointOrientation == Y_Axis))
		{
			DirectPoint NewPoint;
			NewPoint <<  0, 0, 1, this->CalcLinkFullOffset(i-1).x,
					   	 0, 1, 0, this->CalcLinkFullOffset(i).y,
					   	-1, 0, 0, this->CalcLinkFullOffset(i).z,
					   	 0, 0, 0, 1;		
			this->m_DHPoints[i] = NewPoint;
		}
		else if ((link.ExitJointOrientation == Y_Axis) && (prevLink.ExitJointOrientation == X_Axis))
		{
			DirectPoint NewPoint;
			NewPoint << 0, 1, 0, this->CalcLinkFullOffset(i).x,
					   	0, 0, 1, this->CalcLinkFullOffset(i-1).y,
					   	1, 0, 0, this->CalcLinkFullOffset(i).z,
					   	0, 0, 0, 1;
			this->m_DHPoints[i] = NewPoint;
		}
		else if ((link.ExitJointOrientation == Y_Axis) && (prevLink.ExitJointOrientation == Z_Axis))
		{
			DirectPoint NewPoint;
			NewPoint << -1, 0, 0, this->CalcLinkFullOffset(i).x,
						 0, 0, 1, this->CalcLinkFullOffset(i-1).y,
						 0, 1, 1, this->CalcLinkFullOffset(i).z,
						 0, 0, 0, 1;
			this->m_DHPoints[i] = NewPoint;
		}
		else if ((link.ExitJointOrientation == Z_Axis) && (prevLink.ExitJointOrientation == Y_Axis))
		{
			DirectPoint NewPoint;
			NewPoint << 1, 0, 0, this->CalcLinkFullOffset(i).x,
						0, 1, 0, this->CalcLinkFullOffset(i).y,
						0, 0, 1, this->CalcLinkFullOffset(i-1).z,
						0, 0, 0, 1;
			this->m_DHPoints[i] = NewPoint;
		}
		else if ((link.ExitJointOrientation == Z_Axis) && (prevLink.ExitJointOrientation == X_Axis))
		{
			DirectPoint NewPoint;
			NewPoint <<  0, 1, 0, this->CalcLinkFullOffset(i).x,
						-1, 0, 0, this->CalcLinkFullOffset(i).y,
						 0, 0, 1, this->CalcLinkFullOffset(i-1).z,
						 0, 0, 0, 1;
			this->m_DHPoints[i] = NewPoint;

		}
		else if ((link.ExitJointOrientation == X_Axis) && (prevLink.ExitJointOrientation == Z_Axis))
		{
			DirectPoint NewPoint;
			NewPoint << 0, 0, 1, this->CalcLinkFullOffset(i-1).x,
					   	1, 0, 0, this->CalcLinkFullOffset(i).y,
					   	0, 1, 0, this->CalcLinkFullOffset(i).z,
					   	0, 0, 0, 1;
			this->m_DHPoints[i] = NewPoint;
		}	
	}
}

/**
 * Converts a set of DH points to DH parameters.
 *
 * @throws std::invalid_argument if any axis of the DH points are invalid.
 */
void RobotArm::PointsToParams()
{
	int count = this->m_DHPoints.size();
	for (int i = 1; i < count; i++)
		try {
			DHParams params = RobotAdditions::CalcDHParams(this->m_DHPoints[i-1], this->m_DHPoints[i]);
			this->m_LinkJointParams.push_back(params);
		}
		catch (const std::invalid_argument& e) {
			throw(e);			
		}
}

/**
 * Calculates the forward kinematics of the robot arm
 *
 * @return The transformation matrix representing the end-effector position
 */
DirectPoint RobotArm::ForwardKinematics(Eigen::VectorXd q)
{
	if (!this->m_isInitialized)
	{
		Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
		int count = this->m_LinkJointParams.size();
		for (int i = 0; i < count; i++)
		{
			DHParams params = this->m_LinkJointParams[i];
			T = T * RobotAdditions::CalcTransposeMatrix(params, q[i], this->m_joints[i].getJointType());
			// T = T * CalcTransposeMatrix(params, this->joints[i].curCoord, this->joints[i].jointType);
		}
		return T;
	}
	else
	{
		std::cout << "RobotArm is not initialized" << std::endl;
		std::cout << "Do you want to initialize it now? (y/n)" << std::endl;
		char answer;
		std::cin >> answer;
		if (answer == 'y')
		{
			this->initialize();
			return this->ForwardKinematics(q);
		}
		else
			return DirectPoint::Identity();
	}
}

/**
 * Initializes the RobotArm.
 *
 * @throws std::exception if one of the DH points is invalid
 */
void RobotArm::initialize()
{
	if (!this->m_isInitialized)
		try	{
			this->CalcDHPoints();
			this->PointsToParams();
			this->m_isInitialized = true;
		}
		catch(const std::exception& e)
		{
			std::cerr << "One of the DH points is invalid: "<< e.what() << '\n';
			throw(e);
		}
	else
		std::cout << "RobotArm is already initialized" << std::endl;
}

/**
 * Retrieves the origin position of the robot arm.
 *
 * @return The origin position of the robot arm.
 */
Offset RobotArm::getOriginPosition()
{
	return this->m_originPosition;
}

/**
 * Retrieves the joint angles of the robot arm.
 *
 * @return An Eigen::VectorXd object representing the joint angles
 */
Eigen::VectorXd RobotArm::getJointAngles()
{
	int lv_numDOF = this->m_joints.size();
	Eigen::VectorXd lv_q(lv_numDOF);
	lv_q = Eigen::VectorXd::Zero(lv_numDOF);
	for (int i = 0; i < lv_numDOF; i++)
		lv_q[i] = this->m_joints[i].getJointPos();
	return lv_q;
}

/**
 * Sets the joint angles of the robot arm.
 *
 * @param q An Eigen::VectorXd representing the joint angles.
 */
void RobotArm::setJointAngles(Eigen::VectorXd q)
{
	for (int i = 0; i < this->m_joints.size(); i++)
		this->m_joints[i].setJointTarget(PosTarget, q[i]);
}

Eigen::VectorXd RobotArm::solveIK_POS(DirectPoint needPoint, std::string method = "1 - BFGS")
{
	Eigen::VectorXd q = this->getJointAngles();

	std::function<DirectPoint(Eigen::VectorXd)> forwFunct = std::bind(&RobotArm::ForwardKinematics, this, std::placeholders::_1);
    auto forwFunc = forwFunct.target<DirectPoint(*)(Eigen::VectorXd)>();

    // Call the optimization functions
    if (forwFunc)
    {
        switch (method.c_str()[0])
        {
        case '1':
            return MathAdditions::BFGS<DirectPoint>(needPoint, q.size(), *forwFunc,
													RobotAdditions::costRobotFunction,
													RobotAdditions::gradientCostRobotFunction,
													q);
            break;
        default:
            return Eigen::VectorXd::Zero(q.size());
            break;
        }
    }
    else return Eigen::VectorXd::Zero(6);
}

Eigen::VectorXd RobotArm::solveIK_VEL(Eigen::VectorXd needVelocity)
{
	Eigen::VectorXd q = this->getJointAngles();

	std::function<DirectPoint(Eigen::VectorXd)> forwFunct = std::bind(&RobotArm::ForwardKinematics, this, std::placeholders::_1);
    auto forwFunc = forwFunct.target<DirectPoint(*)(Eigen::VectorXd)>();

    if (forwFunc)
    {
		Eigen::MatrixXd J = RobotAdditions::calcRobotJacobian(*forwFunc, q);
		return J.inverse() * needVelocity;
	}
	else return Eigen::VectorXd::Zero(q.size());
}