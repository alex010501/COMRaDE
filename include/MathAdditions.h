#pragma once

#include <Eigen/Dense>
#include <random>

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

namespace MathAdditions
{
    template <typename X>
    using forwardFunc = X (*)(Eigen::VectorXd x);

    template <typename X>
    using errorFunc = Eigen::VectorXd (*)(X a, X b);

    double DegToRad(double p_deg);

    double RadToDeg(double p_rad);

    double projVector(Eigen::Vector3d p_a, Eigen::Vector3d p_b);

    double getAngleAroundAxis(Eigen::Vector3d p_a, Eigen::Vector3d p_b, Eigen::Vector3d p_Axis);

    double getAngle(Eigen::Vector3d p_a, Eigen::Vector3d p_b);

    Eigen::Matrix3d Rx(double p_angle);

    Eigen::Matrix3d Ry(double p_angle);

    Eigen::Matrix3d Rz(double p_angle);

    Eigen::Matrix3d R(double p_x, double p_y, double p_z);

    template <typename T>
    Eigen::MatrixXd calcJacobian(forwardFunc<T> forwFunc, errorFunc<T> errFunc, Eigen::VectorXd x_init, int num_DOF, double eps = 1e-6);

    template <typename T>
    Eigen::VectorXd BFGS(T target, int num_DOF,
                         forwardFunc<T> forwFunc,
                         double (*f)(forwardFunc<T>, Eigen::VectorXd q, T target),
                         Eigen::VectorXd (*df)(forwardFunc<T>, Eigen::VectorXd q, T target),
                         Eigen::VectorXd x_init,
                         double eps = 1e-6, double alpha = 0.01, int max_iterations = 100);

    std::vector<double> make_vector(double p_begin, double p_end, double p_step = 0.1);

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