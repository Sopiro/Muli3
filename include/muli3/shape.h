#pragma once

#include "aabb.h"

namespace muli3
{

struct MassData
{
    float mass = 0.0f;
    Mat3 inertia = Mat3::Diagonal(0.0f, 0.0f, 0.0f);
    Vec3 centerOfMass{ 0.0f, 0.0f, 0.0f };
};

class Shape
{
public:
    enum Type
    {
        // Order matters!
        sphere = 0,
        shape_count,
    };

    Shape(Type type, float radius)
        : type{ type }
        , radius{ radius }
    {
    }

    virtual ~Shape() = default;

    Type GetType() const
    {
        return type;
    }

    float GetRadius() const
    {
        return radius;
    }

    float GetVolume() const
    {
        return volume;
    }

    const Vec3& GetCenter() const
    {
        return center;
    }

    const Vec3& GetCenterOfMass() const
    {
        return center;
    }

    AABB ComputeAABB(const Transform& transform) const
    {
        AABB aabb;
        ComputeAABB(transform, &aabb);
        return aabb;
    }

    virtual void ComputeMass(float density, MassData* outMassData) const = 0;
    virtual void ComputeAABB(const Transform& transform, AABB* outAABB) const = 0;
    virtual Mat3 ComputeLocalInertiaTensor(float mass) const = 0;

    virtual int32 GetVertexCount() const = 0;
    virtual Vec3 GetVertex(int32 id) const = 0;
    virtual int32 GetSupport(const Vec3& localDir) const = 0;

    virtual bool TestPoint(const Transform& transform, const Vec3& q) const = 0;
    virtual Vec3 GetClosestPoint(const Transform& transform, const Vec3& q) const = 0;

private:
    Type type;

protected:
    Vec3 center{ 0.0f, 0.0f, 0.0f };
    float radius = 0.0f;
    float volume = 0.0f;
};

} // namespace muli3
