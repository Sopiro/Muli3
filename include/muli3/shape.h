#pragma once

#include "aabb.h"

namespace muli3
{

enum class ShapeType
{
    sphere = 0,
};

class Shape
{
public:
    explicit Shape(ShapeType type)
        : type{ type }
    {
    }

    virtual ~Shape() = default;

    ShapeType GetType() const
    {
        return type;
    }

    virtual const Vec3& GetCenterOfMass() const = 0;
    virtual Mat3 ComputeLocalInertiaTensor(float mass) const = 0;
    virtual AABB ComputeAABB(const Transform& transform) const = 0;

private:
    ShapeType type;
};

} // namespace muli3
