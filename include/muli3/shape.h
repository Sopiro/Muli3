#pragma once

#include "bounding_box.h"

namespace muli3
{

struct MassData
{
    float mass;
    Mat3 inertia;
    Vec3 centerOfMass;
};

struct Point
{
    Vec3 p;
    int32 id;
};

struct Face
{
    Point points[4];
    Vec3 normal;
    int32 count;
    int32 id;
};

enum ShapeType
{
    // Order matters!
    sphere = 0,
    box,
    shape_count,
};

class Shape
{
public:
    Shape(ShapeType type, float radius);
    virtual ~Shape() = default;

    ShapeType GetType() const;

    float GetRadius() const;
    float GetVolume() const;
    const Vec3& GetCenter() const;

    virtual void ComputeMass(float density, MassData* outMassData) const = 0;
    virtual void ComputeAABB(const Transform& transform, AABB* outAABB) const = 0;

    virtual int32 GetVertexCount() const = 0;
    virtual Vec3 GetVertex(int32 id) const = 0;
    virtual int32 GetSupport(const Vec3& localDir) const = 0;

    virtual bool GetFace(int32 id, const Transform& transform, Face* outFace) const = 0;
    virtual bool GetFeaturedFace(const Transform& transform, const Vec3& dir, Face* outFace) const = 0;

    virtual bool TestPoint(const Transform& transform, const Vec3& q) const = 0;
    virtual Vec3 GetClosestPoint(const Transform& transform, const Vec3& q) const = 0;

protected:
    friend class ContactGraph;
    friend class RigidBody;

    ShapeType type;

    Vec3 center{ 0.0f, 0.0f, 0.0f };
    float radius = 0.0f;
    float volume = 0.0f;
};

inline Shape::Shape(ShapeType type, float radius)
    : type{ type }
    , radius{ radius }
{
}

inline ShapeType Shape::GetType() const
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
