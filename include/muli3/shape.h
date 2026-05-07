#pragma once

#include "bounding_box.h"
#include "primitives.h"
#include "raycast.h"

namespace muli3
{

struct MassData
{
    float mass;
    Mat3 inertia;
    Vec3 centerOfMass;
};

class Shape
{
public:
    enum Type
    {
        // Order matters!
        sphere = 0,
        capsule,
        box,
        convex,
        shape_count,
    };

    Shape(Type type, float radius);
    virtual ~Shape() = default;

    Shape::Type GetType() const;

    float GetRadius() const;
    float GetVolume() const;
    const Vec3& GetCenter() const;

    virtual void ComputeMass(float density, MassData* outMassData) const = 0;
    virtual void ComputeAABB(const Transform& transform, AABB* outAABB) const = 0;

    virtual int32 GetVertexCount() const = 0;
    virtual Vec3 GetVertex(int32 id) const = 0;
    virtual int32 GetSupport(const Vec3& localDir) const = 0;
    virtual Face GetFeaturedFace(const Transform& transform, const Vec3& dir) const = 0;

    virtual bool TestPoint(const Transform& transform, const Vec3& q) const = 0;
    virtual Vec3 GetClosestPoint(const Transform& transform, const Vec3& q) const = 0;
    virtual bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const = 0;

protected:
    friend class ContactGraph;
    friend class RigidBody;

    Type type;

    Vec3 center;
    float radius;
    float volume;
};

inline Shape::Shape(Shape::Type type, float radius)
    : type{ type }
    , center{ 0.0f }
    , radius{ radius }
{
}

inline Shape::Type Shape::GetType() const
{
    return type;
}

inline float Shape::GetRadius() const
{
    return radius;
}

inline float Shape::GetVolume() const
{
    return volume;
}

inline const Vec3& Shape::GetCenter() const
{
    return center;
}

} // namespace muli3
