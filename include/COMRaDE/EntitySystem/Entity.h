#pragma once

#include <DLL_API.h>

#include <string>
#include <vector>

#include <COMRaDE/EntitySystem/Transform.h>

class btDiscreteDynamicsWorld;

enum class EntityType
{
    Entity,
    Dummy,
    Folder,
    Object,
    Cube,
    Sphere,
    Cylinder,
    MultiBody,
    SceneRoot
};

enum class MoveOption
{
    LocalRelative,
    LocalAbsolute,
    WorldRelative,
    WorldAbsolute
};

class DLL_API Entity
{
public:
    Entity(std::string name, const Transform& worldTransform = Transform::Identity(), Entity* parent = nullptr);
    virtual ~Entity() = default;

    virtual void update();
    virtual void initPhysics(btDiscreteDynamicsWorld* dynamicsWorld);

    const std::string& getName() const noexcept;
    void rename(std::string name);

    virtual EntityType getEntityType() const noexcept;

    Entity* getParent() const noexcept;
    void setParent(Entity* parent);

    virtual std::vector<Entity*> getChildren() const;

    Transform getWorldTransform() const;
    const Transform& getLocalTransform() const noexcept;
    void setLocalTransform(const Transform& localTransform);
    void setWorldTransform(const Transform& worldTransform);

    void move(MoveOption moveOption, const Transform& transform);

protected:
    std::string m_name;
    Entity* m_parent = nullptr;
    Transform m_localTransform = Transform::Identity();
};

struct EntityInfo
{
    EntityType type = EntityType::Entity;
    const char* name = nullptr;
    Transform worldTransform = Transform::Identity();
    Entity* parent = nullptr;

    const char* modelPath = nullptr;

    double a = 1.0;
    double b = 1.0;
    double c = 1.0;
};
