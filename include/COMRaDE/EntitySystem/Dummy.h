#pragma once

#include <DLL_API.h>

#include <COMRaDE/EntitySystem/Entity.h>

class DLL_API Dummy : public Entity
{
public:
    Dummy(const char* name, const Transform& worldTransform = Transform::Identity(), Entity* parent = nullptr) :
        Entity(name != nullptr ? name : "Dummy", worldTransform, parent)
    {
    }

    EntityType getEntityType() const noexcept override
    {
        return EntityType::Dummy;
    }
};
