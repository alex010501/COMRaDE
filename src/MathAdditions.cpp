#include <MathAdditions.h>
#include <iostream>

/**
 * Converts degrees to radians.
 *
 * @param p_deg the value in degrees to be converted
 *
 * @return the value in radians
 */
double MathAdditions::DegToRad(double p_deg)
{
    return p_deg * M_PI / 180.0;
}

/**
 * Converts radians to degrees.
 *
 * @param p_rad the value in radians to be converted
 *
 * @return the value in degrees
 */
double MathAdditions::RadToDeg(double p_rad)
{
    return p_rad * 180.0 / M_PI;
}

/**
 * Calculates the projection of vector `a` onto vector `b`.
 *
 * @param p_a The first vector.
 * @param p_b The second vector.
 *
 * @return The projection of vector `a` onto vector `b`.
 * 
 * @throws std::invalid_argument if the second vector is null.
 */
double MathAdditions::projVector(Eigen::Vector3d p_a, Eigen::Vector3d p_b)
{
    if (p_b.norm() == 0)
    {
        throw std::invalid_argument("Vector b must not be null");
    }
    return p_a.dot(p_b) / p_b.norm();
}

/**
 * Calculates the angle between two vectors around a given axis.
 *
 * @param p_a The first vector.
 * @param p_b The second vector.
 * @param p_Axis The axis around which the angle is calculated.
 *
 * @return The angle between the vectors in radians with direction of rotation.
 *
 * @throws std::invalid_argument if either of the vectors is null or one of the vectors is parallel to the axis.
 */
double MathAdditions::getAngleAroundAxis(Eigen::Vector3d p_a, Eigen::Vector3d p_b, Eigen::Vector3d p_Axis)
{
    if (p_Axis.norm() == 0)
        throw std::invalid_argument("Axis must not be null");
    
    p_Axis.normalize();
    p_a = p_a - MathAdditions::projVector(p_a, p_Axis) * p_Axis;
    p_b = p_b - MathAdditions::projVector(p_b, p_Axis) * p_Axis;
    Eigen::Vector3d lv_c = p_a.cross(p_b);
    if (p_a.norm() == 0 || p_b.norm() == 0)
        throw std::invalid_argument("Vectors must not be parallel to axis");
    else if (p_a.norm()*p_b.norm() == p_a.dot(p_b))
        return 0;
    else if (p_a.norm()*p_b.norm() == -p_a.dot(p_b))
        return M_PI;
    else if (MathAdditions::projVector(p_a, p_b) == 0)
        if (MathAdditions::projVector(lv_c, p_Axis) > 0)
            return M_PI/2;
        else
            return -M_PI/2;
    else
        if (MathAdditions::projVector(lv_c, p_Axis) > 0)
            return acos(p_a.dot(p_b) / (p_a.norm() * p_b.norm()));
        else
            return -acos(p_a.dot(p_b) / (p_a.norm() * p_b.norm()));
}

/**
 * Calculates the angle between two vectors around a given axis.
 *
 * @param p_a The first vector.
 * @param p_b The second vector.
 *
 * @return The angle between the vectors in radians.
 *
 * @throws std::invalid_argument if either of the vectors is null.
 */
double MathAdditions::getAngle(Eigen::Vector3d p_a, Eigen::Vector3d p_b)
{
    if (p_a.norm() == 0 || p_b.norm() == 0)
        throw std::invalid_argument("Vectors must not be null"); 
    else if (p_a.norm()*p_b.norm() == p_a.dot(p_b))
        return 0;
    else if (p_a.norm()*p_b.norm() == -p_a.dot(p_b))
        return M_PI;
    else if (MathAdditions::projVector(p_a, p_b) == 0)
        return M_PI/2;
    else
        return acos(p_a.dot(p_b) / (p_a.norm() * p_b.norm()));
}

/**
 * Generates a 3x3 rotation matrix around the x-axis
 *
 * @param p_angle The angle of rotation in radians
 *
 * @return The rotation matrix
 */
Eigen::Matrix3d MathAdditions::Rx(double p_angle)
{
    Eigen::Matrix3d lv_Rx;
    lv_Rx << 1,            0,             0,
             0, cos(p_angle), -sin(p_angle),
             0, sin(p_angle),  cos(p_angle);
    return lv_Rx;
}

/**
 * Generates a 3x3 rotation matrix around the y-axis
 *
 * @param p_angle The angle of rotation in radians
 *
 * @return The rotation matrix.
 */
Eigen::Matrix3d MathAdditions::Ry(double p_angle)
{
    Eigen::Matrix3d lv_Ry;
    lv_Ry << cos(p_angle), 0, sin(p_angle),
                        0, 1,            0,
            -sin(p_angle), 0, cos(p_angle);
    return lv_Ry;
}

/**
 * Generates a 3x3 rotation matrix around the z-axis
 *
 * @param p_angle The angle of rotation in radians
 *
 * @return The rotation matrix
 */
Eigen::Matrix3d MathAdditions::Rz(double p_angle)
{
    Eigen::Matrix3d lv_Rz;
    lv_Rz << cos(p_angle), -sin(p_angle), 0,
             sin(p_angle),  cos(p_angle), 0,
                        0,             0, 1;
    return lv_Rz;
}

/**
 * Calculates the rotation matrix R(x, y, z) based on the given Euler angles.
 *
 * @param p_x The angle in radians around the x-axis.
 * @param p_y The angle in radians around the y-axis.
 * @param p_z The angle in radians around the z-axis.
 *
 * @return The 3x3 rotation matrix R(x, y, z)
 */
Eigen::Matrix3d MathAdditions::R(double p_x, double p_y, double p_z)
{
    return Rx(p_x) * Ry(p_y) * Rz(p_z);
}

/**
 * Calculates the Jacobian matrix for a given forward function and error function.
 *
 * @tparam T the type of the output of the forward function and the input of the error function
 * 
 * @param forwFunc a pointer to the forward function
 * @param errFunc a pointer to the error function
 * @param x_init the initial vector
 * @param num_DOF the number of degrees of freedom
 * @param eps the epsilon value (default: 1e-6)
 *
 * @return the Jacobian matrix
 *
 */
template <typename T>
Eigen::MatrixXd MathAdditions::calcJacobian(forwardFunc<T> forwFunc, // Forward function
                                            errorFunc<T> errFunc, // Error function
                                            Eigen::VectorXd x_init, int num_DOF, double eps)
// Eigen::MatrixXd calcJacobian(forwardFunc<T> forwFunc, Eigen::VectorXd (*errFunc)(T a, T b), Eigen::VectorXd x_init, int num_DOF, double eps = 1e-6)
{
	int vectorSize = x_init.size();	

	Eigen::MatrixXd J(num_DOF, vectorSize);

	for (int i = 0; i < vectorSize; i++)
	{
		// Calculate forward kinematics for q + delta_q
        Eigen::VectorXd x_plus = x_init;
        x_plus(i) += eps;
        T T_plus = forwFunc(x_plus);

		// Calculate forward kinematics for q - delta_q
		Eigen::VectorXd x_minus = x_init;
        x_minus(i) -= eps;
        T T_minus = forwFunc(x_minus);

		// Calculate partial derivative
		Eigen::VectorXd derivative = errFunc(T_plus, T_minus) / (2 * eps);

        // Add to Jacobian matrix
		// J.block<num_DOF,1>(0,i) = derivative;
        J.col(i) = derivative;       
	}
	
	return J;
}

/**
 * This function implements the Broyden-Fletcher-Goldfarb-Shanno (BFGS) optimization algorithm
 * to find the minimum of a given cost function. It iteratively updates the Hessian approximation
 * to approximate the inverse of the true Hessian matrix, and calculates the search direction
 * using the updated Hessian approximation and the gradient of the cost function.
 *
 * @tparam T The type of the target value and forward function.
 * @param target The target value.
 * @param num_DOF The number of degrees of freedom.
 * @param forwFunc The forward function.
 * @param f The cost function.
 * @param df The gradient of the cost function.
 * @param x_init The initial guess. Default is a zero vector of size num_DOF.
 * @param eps The tolerance. Default is 1e-6.
 * @param alpha The step size. Default is 0.01.
 * @param max_iterations The maximum number of iterations. Default is 100.
 * @return The optimized vector x that minimizes the cost function.
 * 
 * @throws std::invalid_argument if the initial guess is not of size num_DOF.
 */
template <typename T>
Eigen::VectorXd MathAdditions::BFGS(T target, int num_DOF, // Target value, number of degrees of freedom
                                    forwardFunc<T> forwFunc, // Forward function
                                    double (*f)(forwardFunc<T>, Eigen::VectorXd q, T target), // Cost function
                                    Eigen::VectorXd (*df)(forwardFunc<T>, Eigen::VectorXd q, T target), // Gradient of cost function
                                    Eigen::VectorXd x_init, // Initial guess
                                    double eps, double alpha, int max_iterations) // Tolerance, step size, max iterations
{

    if (x_init.size() != num_DOF) {
        throw std::invalid_argument("x_init must be of size num_DOF");
    }

    // Initial guess
    Eigen::VectorXd x = x_init;

    // Initial Hessian approximation
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(x.size(), x.size());


    // BFGS iterations
    for (int i = 0; i < max_iterations; ++i) {
        // Calculate search direction
        Eigen::VectorXd p = -H * df(forwFunc, x, target);

        // Update x for comparison
        Eigen::VectorXd x_new = x + alpha * p;

        // Check for convergence
        if ((x_new - x).norm() < eps) {
            break;
        }

        // Update Hessian approximation
        Eigen::VectorXd s = x_new - x;
        Eigen::VectorXd y = df(forwFunc, x_new, target) - df(forwFunc, x, target);
        double rho = 1 / y.dot(s);
        H = (Eigen::MatrixXd::Identity(x.size(), x.size()) - rho * s * y.transpose()) * H
            * (Eigen::MatrixXd::Identity(x.size(), x.size()) - rho * y * s.transpose())
            + rho * s * s.transpose();

        // Update x
        x = x_new;
    }

    return x;
}

std::vector<double> MathAdditions::make_vector(double p_begin, double p_end, double p_step)
{
    std::vector<double> lv_vector;
    lv_vector.reserve((p_end - p_begin) / p_step + 1);
    while (p_begin <= p_end)
    {
        lv_vector.push_back(p_begin);
        p_begin += p_step;
    }
    return lv_vector;
}

/*std::vector<double> MathAdditions::zero_vector(int p_size)
{
    std::vector<double> lv_vector;
    lv_vector.reserve(p_size);
    for (int i = 0; i < p_size; i++)
    {
        lv_vector.push_back(0.0);
    }
    return lv_vector;
}*/

void MathAdditions::Integrator::init(double p_initValue)
{
    this->IntegratorValue = p_initValue;    
    this->PrevValue = 0;
}

double MathAdditions::Integrator::calculate(double p_funcValue, double p_dt)
{
    this->IntegratorValue += (p_funcValue + this->PrevValue) / 2 * p_dt;
    this->PrevValue = p_funcValue;
    return this->IntegratorValue;
}

void MathAdditions::Derivator::init()
{
    this->PrevValue = 0;
}

double MathAdditions::Derivator::calculate(double p_funcValue, double p_dt)
{
    double derivative = (p_funcValue - this->PrevValue) / p_dt;
    this->PrevValue = p_funcValue;
    return derivative;
}

MathAdditions::randSignal::randSignal()
{
    // Initialize random coefficients
    this->a = this->getRand(-1.0, 1.0);
    this->b = this->getRand(-0.5, 0.5);
    this->c = this->getRand(-0.1, 0.1);
    // Initialize random frequencies
    int k1 = this->getRand(1, 5);
    int k2 = this->getRand(5, 10);
    int k3 = this->getRand(10, 20);
    this->f1 = k1 * M_PI / 2;
    this->f2 = k2 * M_PI / 2;
    this->f3 = k3 * M_PI / 2;
}

template <typename T>
T MathAdditions::randSignal::getRand(T lower_bound, T upper_bound)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());

    if constexpr (std::is_integral_v<T>)
    {
        // For integral types (int, long, etc.)
        std::uniform_int_distribution<T> dist(lower_bound, upper_bound);
        return dist(gen);
    }
    else if constexpr (std::is_floating_point_v<T>)
    {
        // For floating-point types (float, double, etc.)
        std::uniform_real_distribution<T> dist(lower_bound, upper_bound);
        return dist(gen);
    }
    else
    {
        // Unsupported type
        throw std::invalid_argument("Unsupported type for getRand");
    }
}

double MathAdditions::randSignal::get_signal(double t)
{
    double firstHarmonic = this->a * sin(this->f1 * t);
    double secondHarmonic = this->b * sin(this->f2 * t);
    double thirdHarmonic = this->c * sin(this->f3 * t);
    // std::cout << "Time " << t << std::endl;
    // std::cout << firstHarmonic << " " << secondHarmonic << " " << thirdHarmonic << std::endl;
    return firstHarmonic + secondHarmonic + thirdHarmonic;
}