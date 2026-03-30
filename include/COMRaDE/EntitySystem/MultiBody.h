#pragma once

#include <DLL_API.h>

#include <COMRaDE/EntitySystem/Entity.h>

class DLL_API MultiBody : public Entity
{
public:
    MultiBody(const char* name, const Transform& worldTransform = Transform::Identity(), Entity* parent = nullptr) :
        Entity(name != nullptr ? name : "MultiBody", worldTransform, parent)
    {
    }

    EntityType getEntityType() const noexcept override
    {
        return EntityType::MultiBody;
    }
};
