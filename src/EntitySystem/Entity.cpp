#include <COMRaDE/EntitySystem/Entity.h>

#include <utility>

Entity::Entity(std::string name, const Transform& worldTransform, Entity* parent) :
    m_name(std::move(name)),
    m_parent(parent)
{
    setWorldTransform(worldTransform);
}

void Entity::update()
{
}

void Entity::initPhysics(btDiscreteDynamicsWorld* dynamicsWorld)
{
    (void)dynamicsWorld;
}

const std::string& Entity::getName() const noexcept
{
    return m_name;
}

void Entity::rename(std::string name)
{
    m_name = std::move(name);
}

EntityType Entity::getEntityType() const noexcept
{
    return EntityType::Entity;
}

Entity* Entity::getParent() const noexcept
{
    return m_parent;
}

void Entity::setParent(Entity* parent)
{
    const Transform worldTransform = getWorldTransform();
    m_parent = parent;
    setWorldTransform(worldTransform);
}

std::vector<Entity*> Entity::getChildren() const
{
    return {};
}

Transform Entity::getWorldTransform() const
{
    if (m_parent == nullptr)
    {
        return m_localTransform;
    }

    return m_parent->getWorldTransform() * m_localTransform;
}

const Transform& Entity::getLocalTransform() const noexcept
{
    return m_localTransform;
}

void Entity::setLocalTransform(const Transform& localTransform)
{
    m_localTransform = localTransform;
}

void Entity::setWorldTransform(const Transform& worldTransform)
{
    if (m_parent == nullptr)
    {
        m_localTransform = worldTransform;
        return;
    }

    m_localTransform = m_parent->getWorldTransform().inverse() * worldTransform;
}

void Entity::move(MoveOption moveOption, const Transform& transform)
{
    switch (moveOption)
    {
    case MoveOption::LocalRelative:
        m_localTransform = m_localTransform * transform;
        break;

    case MoveOption::LocalAbsolute:
        m_localTransform = transform;
        break;

    case MoveOption::WorldRelative:
        setWorldTransform(getWorldTransform() * transform);
        break;

    case MoveOption::WorldAbsolute:
        setWorldTransform(transform);
        break;
    }
}

