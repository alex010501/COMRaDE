#include <iostream>
#include <RobotAdditions.h>

/**
 * Calculates the transpose matrix of a given set of DH parameters, joint value, and joint type
 *
 * @param params the DH parameters (Denavit-Hartenberg parameters)
 * @param jointValue the value of the joint (either the rotation angle or the displacement)
 * @param jointType the type of the joint (rotation or displacement)
 *
 * @return the transpose matrix of the DH parameters
 */
Eigen::Matrix4d RobotAdditions::CalcTransposeMatrix(DHParams params, double jointValue, MoveType jointType)
{
    Eigen::Matrix4d T;

    double theta = params.theta;
    double d = params.d;
    double a = params.a;
    double alpha = params.alpha;

    if (jointValue == RotJoint)
    {
        theta = params.theta + jointValue;
    }
    else
    {
        d = params.d + jointValue;
    }
    T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
        sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
        0, sin(alpha), cos(alpha), d,
        0, 0, 0, 1;

    return T;
}

/**
 * Calculates the DH parameters between i-1 and i XYZ axes
 *
 * @param point_o DirectPoint that represents the i-1 XYZ axes.
 * @param point_e DirectPoint that represents the i XYZ axes
 *
 * @return The calculated DHParams object.
 *
 * @throws std::invalid_argument if one of the vectors that represent the coordinate axis is null.
 */
DHParams RobotAdditions::CalcDHParams(DirectPoint point_o, DirectPoint point_e)
{
    Eigen::Vector3d offset;
    offset << point_e(0, 3) - point_o(0, 3), point_e(1, 3) - point_o(1, 3), point_e(2, 3) - point_o(2, 3);

    Eigen::Vector3d axisX1, axisZ1, axisX2, axisZ2;
    axisX1 << point_o(0, 0), point_o(1, 0), point_o(2, 0);
    axisZ1 << point_o(0, 2), point_o(1, 2), point_o(2, 2);
    axisX2 << point_e(0, 0), point_e(1, 0), point_e(2, 0);
    axisZ2 << point_e(0, 2), point_e(1, 2), point_e(2, 2);

    DHParams params;
    try
    {
        params.a = MathAdditions::projVector(offset, axisX2);
    }
    catch (const std::invalid_argument &e)
    {
        throw(e);
    }
    try
    {
        params.alpha = MathAdditions::getAngleAroundAxis(axisZ1, axisZ2, axisX2);
    }
    catch (const std::invalid_argument &e)
    {
        throw(e);
    }
    try
    {
        params.d = MathAdditions::projVector(offset, axisZ1);
    }
    catch (const std::invalid_argument &e)
    {
        throw(e);
    }
    try
    {
        params.theta = MathAdditions::getAngleAroundAxis(axisX1, axisX2, axisZ1);
    }
    catch (const std::invalid_argument &e)
    {
        throw(e);
    }
    return params;
}

/**
 * Calculate the error between the desired point and the current point of a robot.
 *
 * @param needPoint The desired point represented as a DirectPoint object.
 * @param currentPoint The current point represented as a DirectPoint object.
 *
 * @return An Eigen::VectorXd object representing the error between the desired point and the current point.
 * The error vector has 6 elements, where the first 3 elements represent the position error
 * and the last 3 elements represent the rotation error.
 */
Eigen::VectorXd RobotAdditions::errorRobotPoses(DirectPoint needPoint, DirectPoint currentPoint)
{
    // Extract position and rotation of current point
    Eigen::Vector3d current_position;
    current_position << currentPoint(0, 3), currentPoint(1, 3), currentPoint(2, 3);
    Eigen::Matrix3d current_rotation = currentPoint.block<3, 3>(0, 0);

    // Extract position and rotation of needed point
    Eigen::Vector3d need_position;
    need_position << needPoint(0, 3), needPoint(1, 3), needPoint(2, 3);
    Eigen::Matrix3d need_rotation = needPoint.block<3, 3>(0, 0);

    // Calculate position error
    Eigen::Vector3d position_error = need_position - current_position;

    // Calculate rotation error
    Eigen::Matrix3d rotation_error = need_rotation.inverse() * current_rotation;
    Eigen::Vector3d angle_error;
    angle_error << rotation_error(0, 0), rotation_error(1, 1), rotation_error(2, 2);
    // Eigen::Vector3d angle_error = rotation_error.eulerAngles(0, 1, 2); - deprecated

    // Create error vector
    Eigen::VectorXd error(6);
    error << position_error, angle_error;
    return error;
}

/**
 * Calculates the robot Jacobian matrix.
 *
 * @param forwFunc the forward function object for the direct point
 * @param q_init the initial vector value
 * @param eps the epsilon value (default: 1e-6)
 *
 * @return the calculated Jacobian matrix
 */
Eigen::MatrixXd RobotAdditions::calcRobotJacobian(forwardFunc<DirectPoint> forwFunc, Eigen::VectorXd q_init, double eps)
{
    return MathAdditions::calcJacobian<DirectPoint>(forwFunc, RobotAdditions::errorRobotPoses, q_init, 6, eps);
}

/**
 * Calculates the cost of a robot function.
 *
 * @param forwFunc a pointer to a function that calculates the forward transformation of the robot
 * @param q the joint configuration of the robot
 * @param target the desired end-effector transformation
 *
 * @return the cost of the robot function
 */
double RobotAdditions::costRobotFunction(DirectPoint (*forwFunc)(Eigen::VectorXd q), Eigen::VectorXd q, DirectPoint target)
{
    // Calculate end-effector transformation matrix
    DirectPoint T = forwFunc(q);

    // Calculate position error
    Eigen::Vector3d position_error = T.block<3, 1>(0, 3) - target.block<3, 1>(0, 3);

    // Calculate rotation error
    Eigen::Matrix3d rotation_error = T.block<3, 3>(0, 0) - target.block<3, 3>(0, 0);

    // Calculate cost
    return position_error.squaredNorm() + rotation_error.norm();
}

/**
 * Calculates the gradient of the cost function for a robot.
 *
 * @param forwFunc a pointer to a function that calculates the forward kinematics of the robot
 * @param q_init the initial configuration of the robot
 * @param target the target position and orientation for the end-effector
 *
 * @return the gradient of the cost function
 */
Eigen::VectorXd RobotAdditions::gradientCostRobotFunction(DirectPoint (*forwFunc)(Eigen::VectorXd q), Eigen::VectorXd q_init, DirectPoint target)
{
    // Calculate Jacobian
    Eigen::MatrixXd J = calcRobotJacobian(forwFunc, q_init);

    // Calculate end-effector transformation matrix
    DirectPoint T = forwFunc(q_init);

    // Calculate position error
    Eigen::Vector3d position_error = T.block<3, 1>(0, 3) - target.block<3, 1>(0, 3);

    // Calculate rotation error
    Eigen::Matrix3d rotation_error = T.block<3, 3>(0, 0) - target.block<3, 3>(0, 0);

    // Calculate gradient
    return 2 * J.transpose() * position_error + 2 * rotation_error.norm() * J.transpose();
}