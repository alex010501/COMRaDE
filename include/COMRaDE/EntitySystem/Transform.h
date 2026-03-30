#pragma once

#include <DLL_API.h>

#include <Eigen/Geometry>
#include <btBulletDynamicsCommon.h>

class DLL_API Transform
{
public:
    Transform();
    explicit Transform(const Eigen::Isometry3d& value);
    explicit Transform(const Eigen::Matrix4d& matrix);
    Transform(const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation);
    Transform(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation);

    static Transform Identity();

    const Eigen::Isometry3d& isometry() const noexcept;
    Eigen::Matrix4d matrix() const;
    void setMatrix(const Eigen::Matrix4d& matrix);

    void setPosition(const Eigen::Vector3d& position);
    Eigen::Vector3d getPosition() const;

    void setRotation(const Eigen::Quaterniond& rotation);
    Eigen::Quaterniond getRotation() const;

    void setRotationMatrix(const Eigen::Matrix3d& rotation);
    Eigen::Matrix3d getRotationMatrix() const;

    Transform inverse() const;

    btTransform toBtTransform() const;
    static Transform fromBtTransform(const btTransform& bulletTransform);

private:
    Eigen::Isometry3d m_value;
};

DLL_API Transform operator*(const Transform& lhs, const Transform& rhs);
