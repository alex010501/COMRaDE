#include <COMRaDE/EntitySystem/Transform.h>

Transform::Transform() :
    m_value(Eigen::Isometry3d::Identity())
{
}

Transform::Transform(const Eigen::Isometry3d& value) :
    m_value(value)
{
}

Transform::Transform(const Eigen::Matrix4d& matrix) :
    m_value(Eigen::Isometry3d(matrix))
{
}

Transform::Transform(const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation) :
    m_value(Eigen::Isometry3d::Identity())
{
    setPosition(position);
    setRotation(rotation);
}

Transform::Transform(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation) :
    m_value(Eigen::Isometry3d::Identity())
{
    setPosition(position);
    setRotationMatrix(rotation);
}

Transform Transform::Identity()
{
    return Transform(Eigen::Isometry3d::Identity());
}

const Eigen::Isometry3d& Transform::isometry() const noexcept
{
    return m_value;
}

Eigen::Matrix4d Transform::matrix() const
{
    return m_value.matrix();
}

void Transform::setMatrix(const Eigen::Matrix4d& matrix)
{
    m_value = Eigen::Isometry3d(matrix);
}

void Transform::setPosition(const Eigen::Vector3d& position)
{
    m_value.translation() = position;
}

Eigen::Vector3d Transform::getPosition() const
{
    return m_value.translation();
}

void Transform::setRotation(const Eigen::Quaterniond& rotation)
{
    m_value.linear() = rotation.normalized().toRotationMatrix();
}

Eigen::Quaterniond Transform::getRotation() const
{
    return Eigen::Quaterniond(m_value.linear());
}

void Transform::setRotationMatrix(const Eigen::Matrix3d& rotation)
{
    m_value.linear() = rotation;
}

Eigen::Matrix3d Transform::getRotationMatrix() const
{
    return m_value.linear();
}

Transform Transform::inverse() const
{
    return Transform(m_value.inverse());
}

btTransform Transform::toBtTransform() const
{
    const Eigen::Quaterniond q = getRotation();
    const Eigen::Vector3d p = getPosition();

    return btTransform(
        btQuaternion(q.x(), q.y(), q.z(), q.w()),
        btVector3(p.x(), p.y(), p.z()));
}

Transform Transform::fromBtTransform(const btTransform& bulletTransform)
{
    const btVector3 position = bulletTransform.getOrigin();
    const btQuaternion rotation = bulletTransform.getRotation();

    return Transform(
        Eigen::Vector3d(position.x(), position.y(), position.z()),
        Eigen::Quaterniond(rotation.w(), rotation.x(), rotation.y(), rotation.z()));
}

Transform operator*(const Transform& lhs, const Transform& rhs)
{
    return Transform(lhs.isometry() * rhs.isometry());
}

