#include <COMRaDE/EntitySystem/Folder.h>

#include <algorithm>

Folder::Folder(const char* name, const Transform& worldTransform, Entity* parent) :
    Entity(name != nullptr ? name : "Folder", worldTransform, parent)
{
}

Folder::~Folder()
{
    for (Entity* child : m_children)
    {
        delete child;
    }
}

EntityType Folder::getEntityType() const noexcept
{
    return EntityType::Folder;
}

void Folder::update()
{
    for (Entity* child : m_children)
    {
        child->update();
    }
}

void Folder::initPhysics(btDiscreteDynamicsWorld* dynamicsWorld)
{
    for (Entity* child : m_children)
    {
        child->initPhysics(dynamicsWorld);
    }
}

void Folder::addChild(Entity* entity)
{
    if (entity == nullptr)
    {
        return;
    }

    entity->setParent(this);
    m_children.push_back(entity);
}

void Folder::removeChild(Entity* entity)
{
    if (entity == nullptr)
    {
        return;
    }

    m_children.erase(
        std::remove(m_children.begin(), m_children.end(), entity),
        m_children.end());
    entity->setParent(nullptr);
}

void Folder::clearChildren()
{
    for (Entity* child : m_children)
    {
        delete child;
    }

    m_children.clear();
}

std::vector<Entity*> Folder::getChildren() const
{
    return m_children;
}

SceneRoot::SceneRoot() :
    Folder("Scene root")
{
}

SceneRoot::SceneRoot(const char* name) :
    Folder(name, Transform::Identity(), nullptr)
{
}

EntityType SceneRoot::getEntityType() const noexcept
{
    return EntityType::SceneRoot;
}

