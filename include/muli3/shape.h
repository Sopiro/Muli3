#pragma once

#include "bounding_box.h"
#include "dynamic_dispatcher.h"
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

using Shapes = TypePack<class SphereShape, class CapsuleShape, class BoxShape, class ConvexShape, class HeightFieldShape>;

class Shape : public DynamicDispatcher<Shapes>
{
public:
    using Types = Shapes;

    enum Type
    {
        // Order matters!
        sphere = 0,
        capsule,
        box,
        convex,
        height_field,
        shape_count,
    };

    ~Shape() = default;

    Type GetType() const;

    float GetRadius() const;
    float GetVolume() const;
    const Vec3& GetCenter() const;

    void ComputeMass(float density, MassData* outMassData) const;
    void ComputeAABB(const Transform& transform, AABB* outAABB) const;

    int32 GetVertexCount() const;
    Vec3 GetVertex(int32 id) const;
    int32 GetSupport(const Vec3& localDir) const;
    Face GetFeaturedFace(const Transform& transform, const Vec3& dir) const;

    bool TestPoint(const Transform& transform, const Vec3& q) const;
    Vec3 GetClosestPoint(const Transform& transform, const Vec3& q) const;
    bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const;

protected:
    friend class Body;
    friend class ConstraintGraph;

    Shape(Type type, float radius);

    Vec3 center;
    float radius;
    float volume;
};

inline Shape::Shape(Type type, float radius)
    : DynamicDispatcher(int32(type))
    , center{ 0.0f }
    , radius{ radius }
{
}

inline Shape::Type Shape::GetType() const
{
    return Shape::Type(type_index);
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
