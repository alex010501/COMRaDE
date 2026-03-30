#pragma once

#include <DLL_API.h>

#include <COMRaDE/EntitySystem/Entity.h>

class DLL_API Folder : public Entity
{
public:
    Folder(const char* name, const Transform& worldTransform = Transform::Identity(), Entity* parent = nullptr);
    ~Folder() override;

    EntityType getEntityType() const noexcept override;

    void update() override;
    void initPhysics(btDiscreteDynamicsWorld* dynamicsWorld) override;

    void addChild(Entity* entity);
    void removeChild(Entity* entity);
    void clearChildren();
    std::vector<Entity*> getChildren() const override;

protected:
    std::vector<Entity*> m_children;
};

class DLL_API SceneRoot : public Folder
{
public:
    SceneRoot();
    explicit SceneRoot(const char* name);

    EntityType getEntityType() const noexcept override;
};
