#pragma once

#include "bounding_box.h"
#include "dynamic_dispatcher.h"
#include "raycast.h"
#include "settings.h" // IWYU pragma: export

namespace muli3
{

struct Face
{
    // Shapes whose face index ranges do not fit in uint16 have undefined behavior.
    uint16 vertexStart;
    uint16 vertexCount;
    Vec3 normal;
};

struct MassData
{
    float mass;
    Mat3 inertia;
    Vec3 centerOfMass;
};

using Shapes = TypePack<
    class SphereShape,
    class CapsuleShape,
    class BoxShape,
    class ConvexShape,
    class PolygonShape,
    class TriangleShape,
    class HeightFieldShape,
    class MeshShape>;

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
        polygon,
        triangle,
        height_field,
        mesh,
        shape_count,
    };

    ~Shape() = default;

    Type GetType() const;
    bool IsSimpleShape() const;

    float GetRadius() const;
    float GetVolume() const;
    const Vec3& GetCenter() const;

    void ComputeMass(float density, MassData* outMassData) const;
    void ComputeAABB(const Transform& transform, AABB* outAABB) const;

    int32 GetVertexCount() const;
    Vec3 GetVertex(int32 index) const;
    int32 GetVertexIndex(int32 index) const;
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
    MuliAssert(radius >= 0.0f);
}

inline Shape::Type Shape::GetType() const
{
    return Shape::Type(type_index);
}

inline bool Shape::IsSimpleShape() const
{
    return GetType() < Shape::height_field;
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
