#pragma once

#include <Eigen/Dense>
#include <random>

#include <DLL_API.h>

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

namespace MathAdditions
{
    template <typename X>
    using forwardFunc = X (*)(Eigen::VectorXd x);

    template <typename X>
    using errorFunc = Eigen::VectorXd (*)(X a, X b);

    DLL_API double DegToRad(double p_deg);

    DLL_API double RadToDeg(double p_rad);

    DLL_API double projVector(Eigen::Vector3d p_a, Eigen::Vector3d p_b);

    DLL_API double getAngleAroundAxis(Eigen::Vector3d p_a, Eigen::Vector3d p_b, Eigen::Vector3d p_Axis);

    DLL_API double getAngle(Eigen::Vector3d p_a, Eigen::Vector3d p_b);

    DLL_API Eigen::Matrix3d Rx(double p_angle);

    DLL_API Eigen::Matrix3d Ry(double p_angle);

    DLL_API Eigen::Matrix3d Rz(double p_angle);

    DLL_API Eigen::Matrix3d R(double p_x, double p_y, double p_z);

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
    Eigen::MatrixXd calcJacobian(forwardFunc<T> forwFunc, // Forward function
                                 errorFunc<T> errFunc, // Error function
                                 Eigen::VectorXd x_init, int num_DOF, double eps = 1e-6)
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
    Eigen::VectorXd BFGS(T target, int num_DOF, // Target value, number of degrees of freedom
                         forwardFunc<T> forwFunc, // Forward function
                         double (*f)(forwardFunc<T>, Eigen::VectorXd q, T target), // Cost function
                         Eigen::VectorXd (*df)(forwardFunc<T>, Eigen::VectorXd q, T target), // Gradient of cost function
                         Eigen::VectorXd x_init, // Initial guess
                         double eps = 1e-6, double alpha = 0.01, int max_iterations = 100) // Tolerance, step size, max iterations
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

    DLL_API std::vector<double> make_vector(double p_begin, double p_end, double p_step = 0.1);

    // std::vector<double> zero_vector(int p_size);

    class Integrator
    {
    private:
        double IntegratorValue;
        double PrevValue;
    public:
        void init(double p_initValue);
        double calculate(double p_funcValue, double p_dt);
    };

    class Derivator
    {
    private:
        double PrevValue;
    public:
        void init();
        double calculate(double p_funcValue, double p_dt);
    };

    class randSignal
    {
    private:
        double a, b, c;
        double f1, f2, f3;

        template <typename T>
        T getRand(T lower_bound, T upper_bound);

    public:
        randSignal();
        double get_signal(double t);
    };
}