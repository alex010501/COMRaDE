#pragma once

#include <DLL_API.h>

#include <string>

#include <COMRaDE/EntitySystem/Entity.h>

class btCollisionShape;
class btDefaultMotionState;
class btRigidBody;

enum class ObjectMotionType
{
    VisualOnly,
    Static,
    Dynamic
};

enum class ObjectShapeType
{
    Mesh,
    Box,
    Sphere,
    Cylinder
};

class DLL_API Object : public Entity
{
public:
    Object(const char* name, const Transform& worldTransform = Transform::Identity(), Entity* parent = nullptr, const char* modelPath = nullptr);
    ~Object() override;

    EntityType getEntityType() const noexcept override;

    void update() override;
    void initPhysics(btDiscreteDynamicsWorld* dynamicsWorld) override;

    const std::string& getModelPath() const noexcept;
    void setModelPath(const char* modelPath);

    const Eigen::Vector4d& getColor() const noexcept;
    void setColor(const Eigen::Vector4d& color);

    double getMass() const noexcept;
    void setMass(double mass);

    ObjectMotionType getMotionType() const noexcept;
    void setMotionType(ObjectMotionType motionType);

    double getFriction() const noexcept;
    void setFriction(double friction);

    double getRestitution() const noexcept;
    void setRestitution(double restitution);

    double getLinearDamping() const noexcept;
    double getAngularDamping() const noexcept;
    void setDamping(double linearDamping, double angularDamping);

    ObjectShapeType getShapeType() const noexcept;
    Eigen::Vector3d getShapeDimensions() const noexcept;
    void setBoxGeometry(double x, double y, double z);
    void setSphereGeometry(double radius);
    void setCylinderGeometry(double radius, double height);

    btRigidBody* getRigidBody() const noexcept;
    void syncPhysicsTransform();

protected:
    virtual btCollisionShape* createCollisionShape() const;
    virtual void rebuildRigidBody();
    void destroyRigidBody();

    std::string m_modelPath;
    Eigen::Vector4d m_color = Eigen::Vector4d(1.0, 1.0, 1.0, 1.0);
    ObjectMotionType m_motionType = ObjectMotionType::VisualOnly;
    ObjectShapeType m_shapeType = ObjectShapeType::Mesh;
    Eigen::Vector3d m_shapeDimensions = Eigen::Vector3d(1.0, 1.0, 1.0);

    btCollisionShape* m_shape = nullptr;
    btDefaultMotionState* m_motionState = nullptr;
    btRigidBody* m_rigidBody = nullptr;
    btDiscreteDynamicsWorld* m_dynamicsWorld = nullptr;
    double m_mass = 0.0;
    double m_friction = 0.5;
    double m_restitution = 0.0;
    double m_linearDamping = 0.0;
    double m_angularDamping = 0.0;
};

class DLL_API CubeObject : public Object
{
public:
    CubeObject(const char* name, double a, double b, double c, const Transform& worldTransform = Transform::Identity(), Entity* parent = nullptr);

    EntityType getEntityType() const noexcept override;
};

class DLL_API SphereObject : public Object
{
public:
    SphereObject(const char* name, double radius, const Transform& worldTransform = Transform::Identity(), Entity* parent = nullptr);

    EntityType getEntityType() const noexcept override;
};

class DLL_API CylinderObject : public Object
{
public:
    CylinderObject(const char* name, double radius, double height, const Transform& worldTransform = Transform::Identity(), Entity* parent = nullptr);

    EntityType getEntityType() const noexcept override;
};
