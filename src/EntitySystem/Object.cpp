#include <COMRaDE/EntitySystem/Object.h>

#include <btBulletDynamicsCommon.h>

namespace
{
double resolveMass(ObjectMotionType motionType, double requestedMass)
{
    switch (motionType)
    {
    case ObjectMotionType::VisualOnly:
    case ObjectMotionType::Static:
        return 0.0;

    case ObjectMotionType::Dynamic:
        return requestedMass > 0.0 ? requestedMass : 1.0;
    }

    return 0.0;
}
}

Object::Object(const char* name, const Transform& worldTransform, Entity* parent, const char* modelPath) :
    Entity(name != nullptr ? name : "Object", worldTransform, parent)
{
    if (modelPath != nullptr)
    {
        m_modelPath = modelPath;
    }
}

Object::~Object()
{
    destroyRigidBody();
}

EntityType Object::getEntityType() const noexcept
{
    return EntityType::Object;
}

void Object::update()
{
    if (m_rigidBody == nullptr)
    {
        return;
    }

    btTransform worldTransform = m_rigidBody->getWorldTransform();
    if (m_motionState != nullptr)
    {
        m_motionState->getWorldTransform(worldTransform);
    }

    setWorldTransform(Transform::fromBtTransform(worldTransform));
}

void Object::initPhysics(btDiscreteDynamicsWorld* dynamicsWorld)
{
    m_dynamicsWorld = dynamicsWorld;
    rebuildRigidBody();
}

const std::string& Object::getModelPath() const noexcept
{
    return m_modelPath;
}

void Object::setModelPath(const char* modelPath)
{
    m_modelPath = modelPath != nullptr ? modelPath : "";
    rebuildRigidBody();
}

const Eigen::Vector4d& Object::getColor() const noexcept
{
    return m_color;
}

void Object::setColor(const Eigen::Vector4d& color)
{
    m_color = color;
}

double Object::getMass() const noexcept
{
    return m_mass;
}

void Object::setMass(double mass)
{
    m_mass = mass;
    rebuildRigidBody();
}

ObjectMotionType Object::getMotionType() const noexcept
{
    return m_motionType;
}

void Object::setMotionType(ObjectMotionType motionType)
{
    m_motionType = motionType;
    rebuildRigidBody();
}

double Object::getFriction() const noexcept
{
    return m_friction;
}

void Object::setFriction(double friction)
{
    m_friction = friction;
    if (m_rigidBody != nullptr)
    {
        m_rigidBody->setFriction(m_friction);
    }
}

double Object::getRestitution() const noexcept
{
    return m_restitution;
}

void Object::setRestitution(double restitution)
{
    m_restitution = restitution;
    if (m_rigidBody != nullptr)
    {
        m_rigidBody->setRestitution(m_restitution);
    }
}

double Object::getLinearDamping() const noexcept
{
    return m_linearDamping;
}

double Object::getAngularDamping() const noexcept
{
    return m_angularDamping;
}

void Object::setDamping(double linearDamping, double angularDamping)
{
    m_linearDamping = linearDamping;
    m_angularDamping = angularDamping;
    if (m_rigidBody != nullptr)
    {
        m_rigidBody->setDamping(m_linearDamping, m_angularDamping);
    }
}

ObjectShapeType Object::getShapeType() const noexcept
{
    return m_shapeType;
}

Eigen::Vector3d Object::getShapeDimensions() const noexcept
{
    return m_shapeDimensions;
}

void Object::setBoxGeometry(double x, double y, double z)
{
    m_shapeType = ObjectShapeType::Box;
    m_shapeDimensions = Eigen::Vector3d(x, y, z);
    rebuildRigidBody();
}

void Object::setSphereGeometry(double radius)
{
    m_shapeType = ObjectShapeType::Sphere;
    m_shapeDimensions = Eigen::Vector3d(radius, radius, radius);
    rebuildRigidBody();
}

void Object::setCylinderGeometry(double radius, double height)
{
    m_shapeType = ObjectShapeType::Cylinder;
    m_shapeDimensions = Eigen::Vector3d(radius, radius, height);
    rebuildRigidBody();
}

btRigidBody* Object::getRigidBody() const noexcept
{
    return m_rigidBody;
}

void Object::syncPhysicsTransform()
{
    if (m_rigidBody == nullptr || m_motionType == ObjectMotionType::Dynamic)
    {
        return;
    }

    const btTransform worldTransform = getWorldTransform().toBtTransform();
    if (m_motionState != nullptr)
    {
        m_motionState->setWorldTransform(worldTransform);
    }

    m_rigidBody->setWorldTransform(worldTransform);
    m_rigidBody->setInterpolationWorldTransform(worldTransform);
    m_rigidBody->clearForces();
    m_rigidBody->setLinearVelocity(btVector3(0.0, 0.0, 0.0));
    m_rigidBody->setAngularVelocity(btVector3(0.0, 0.0, 0.0));
    m_rigidBody->activate(true);

    if (m_dynamicsWorld != nullptr)
    {
        m_dynamicsWorld->updateSingleAabb(m_rigidBody);
    }
}

btCollisionShape* Object::createCollisionShape() const
{
    switch (m_shapeType)
    {
    case ObjectShapeType::Box:
        return new btBoxShape(btVector3(
            0.5 * m_shapeDimensions.x(),
            0.5 * m_shapeDimensions.y(),
            0.5 * m_shapeDimensions.z()));

    case ObjectShapeType::Sphere:
        return new btSphereShape(m_shapeDimensions.x());

    case ObjectShapeType::Cylinder:
        return new btCylinderShapeZ(btVector3(
            m_shapeDimensions.x(),
            m_shapeDimensions.y(),
            0.5 * m_shapeDimensions.z()));

    case ObjectShapeType::Mesh:
        return nullptr;
    }

    return nullptr;
}

void Object::rebuildRigidBody()
{
    destroyRigidBody();

    if (m_dynamicsWorld == nullptr || m_motionType == ObjectMotionType::VisualOnly)
    {
        return;
    }

    m_shape = createCollisionShape();
    if (m_shape == nullptr)
    {
        return;
    }

    const double resolvedMass = resolveMass(m_motionType, m_mass);
    btVector3 inertia(0.0, 0.0, 0.0);
    if (resolvedMass > 0.0)
    {
        m_shape->calculateLocalInertia(resolvedMass, inertia);
    }

    m_motionState = new btDefaultMotionState(getWorldTransform().toBtTransform());
    btRigidBody::btRigidBodyConstructionInfo info(resolvedMass, m_motionState, m_shape, inertia);
    m_rigidBody = new btRigidBody(info);
    m_rigidBody->setUserPointer(this);
    m_rigidBody->setFriction(m_friction);
    m_rigidBody->setRestitution(m_restitution);
    m_rigidBody->setDamping(m_linearDamping, m_angularDamping);

    if (m_motionType == ObjectMotionType::Static)
    {
        m_rigidBody->setCollisionFlags(m_rigidBody->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
    }

    m_dynamicsWorld->addRigidBody(m_rigidBody);
    syncPhysicsTransform();
}

void Object::destroyRigidBody()
{
    if (m_dynamicsWorld != nullptr && m_rigidBody != nullptr)
    {
        m_dynamicsWorld->removeRigidBody(m_rigidBody);
    }

    delete m_rigidBody;
    delete m_motionState;
    delete m_shape;

    m_rigidBody = nullptr;
    m_motionState = nullptr;
    m_shape = nullptr;
}

CubeObject::CubeObject(const char* name, double a, double b, double c, const Transform& worldTransform, Entity* parent) :
    Object(name, worldTransform, parent)
{
    setBoxGeometry(a, b, c);
}

EntityType CubeObject::getEntityType() const noexcept
{
    return EntityType::Cube;
}

SphereObject::SphereObject(const char* name, double radius, const Transform& worldTransform, Entity* parent) :
    Object(name, worldTransform, parent)
{
    setSphereGeometry(radius);
}

EntityType SphereObject::getEntityType() const noexcept
{
    return EntityType::Sphere;
}

CylinderObject::CylinderObject(const char* name, double radius, double height, const Transform& worldTransform, Entity* parent) :
    Object(name, worldTransform, parent)
{
    setCylinderGeometry(radius, height);
}

EntityType CylinderObject::getEntityType() const noexcept
{
    return EntityType::Cylinder;
}
